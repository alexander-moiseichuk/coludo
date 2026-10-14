/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "hosted_no_reboot.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

/* esp_hosted's own port config first: it defines the transport macros that decide the table's layout */
#include "port_esp_hosted_host_config.h"
#include "esp_hosted_os_abstraction.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"

static const char *TAG = "hosted_no_reboot";
/* esp_hosted's return codes (port_esp_hosted_host_os.h RET_OK, RET_INVALID; that header pulls in esp_hosted's
 * private ones) */
#define HOSTED_RET_OK 0
#define HOSTED_RET_INVALID -2
static volatile bool link_down;
static bool installed;
static bool pinned;
static bool buffers_in_psram;
/* esp_hosted buffers PSRAM had no room for, so they came from internal DMA memory after all */
static volatile uint32_t internal_fallbacks;

/* The core esp_hosted's tasks run on: core 0, with the rest of the radio. Core 1 is the flight core
 * (doc/specs/avionics-port.md §4.2), and an unpinned task at priority 23 preempts anything there. */
#define HOSTED_CORE 0

/* esp_hosted's own hooks (port_esp_hosted_host_os.c; non-static, no public prototypes) */
extern int hosted_restart_host(void);
extern void *hosted_thread_create(const char *name, uint32_t priority, uint32_t stack_size,
                                  void (*start_routine)(void const *), void *argument);
extern int hosted_thread_cancel(void *thread_handle);
extern void *hosted_malloc_align(size_t size, size_t align);

/* Called where esp_hosted would restart the host. It must NOT return: at sdio_drv.c:450 the caller holds the
 * SDIO driver lock and would retry a failing command forever -- and every failed command holds the SDMMC
 * controller (shared with the microSD on slot 0) for up to a second. Parking the calling task ends that. */
static int park(void)
{
    link_down = true;
    ESP_LOGE(TAG, "the C6 link failed; parking %s instead of restarting the host (Wi-Fi down until reboot)",
             pcTaskGetName(NULL));
    vTaskSuspend(NULL);
    return 0;
}

/* esp_hosted's thread handle, ours: the TaskHandle_t first, where esp_hosted reads it (`*(TaskHandle_t *)handle`),
 * then how the task was made, so cancel_thread deletes it the same way. */
typedef struct {
    TaskHandle_t task;
    bool psram_stack;
} hosted_thread_t;

/* hosted_thread_create, pinned to HOSTED_CORE and on a PSRAM stack (2026-10-13, doc/specs/avionics-port.md §4.7:
 * internal RAM is the camera's and the flight's). esp_hosted's tasks hand the SDIO DMA only buffers from its aligned
 * malloc (malloc_align_psram below), never stack memory, and none of them maps flash, the one thing a PSRAM-stack task
 * must not do. PSRAM full: an internal stack, as before (`heap stacks` shows each task's region). */
static void *create_pinned_thread(const char *name, uint32_t priority, uint32_t stack_size,
                                  void (*start_routine)(void const *), void *argument)
{
    if (!start_routine) {
        return NULL;
    }
    hosted_thread_t *handle = malloc(sizeof(hosted_thread_t));
    if (!handle) {
        return NULL;
    }
    handle->task = NULL;
    handle->psram_stack = true;
    if (xTaskCreatePinnedToCoreWithCaps((void (*)(void *))start_routine, name, stack_size, argument, priority,
                                        &handle->task, HOSTED_CORE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdTRUE &&
        handle->task) {
        return handle;
    }
    handle->psram_stack = false;
    if (xTaskCreatePinnedToCore((void (*)(void *))start_routine, name, stack_size, argument, priority, &handle->task,
                                HOSTED_CORE) != pdTRUE || !handle->task) {
        ESP_LOGE(TAG, "esp_hosted task %s not created", name);
        free(handle);
        return NULL;
    }
    return handle;
}

/* hosted_thread_cancel for create_pinned_thread's tasks: a task made with caps is deleted with caps (plain vTaskDelete
 * would keep its PSRAM stack), then the handle is freed as esp_hosted's own cancel frees it. */
static int cancel_thread(void *thread_handle)
{
    hosted_thread_t *handle = thread_handle;
    if (!handle) {
        return HOSTED_RET_INVALID;
    }
    if (handle->psram_stack) {
        vTaskDeleteWithCaps(handle->task);
    } else {
        vTaskDelete(handle->task);
    }
    free(handle);
    return HOSTED_RET_OK;
}

/* hosted_malloc_align, in PSRAM: every buffer esp_hosted hands to the SDIO DMA comes from it -- the frame pool
 * behind the RX assert (sdio_drv.c:862 pkt_rxbuff) and the TX copy (transport_drv.c:267 copy_buff), the RX stream's
 * double buffers (sdio_drv.c:830), each one an assert when internal DMA memory runs out (doc/specs/avionics-port.md
 * §4.7). The same caps as upstream esp_hosted 2.12.8: with MALLOC_CAP_DMA the heap aligns the address and the size
 * to the cache line (64 B, pinned in sdkconfig.defaults) and then takes PSRAM (heap_align_hw.c); the SDMMC host
 * syncs the cache around every transfer, and the P4's SDMMC reaches PSRAM (SOC_SDMMC_PSRAM_DMA_CAPABLE). The free
 * stays esp_hosted's own free(), which takes either heap. PSRAM full: internal DMA memory, as before, counted. */
static void *malloc_align_psram(size_t size, size_t align)
{
    void *block = heap_caps_aligned_alloc(align, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (block) {
        return block;
    }
    internal_fallbacks++;
    return hosted_malloc_align(size, align);
}

/* Install the hooks before esp_hosted starts. esp_hosted initialises from a plain C constructor
 * (port_esp_hosted_host_init.c) and creates its first tasks there, so main() is too late: a prioritised
 * constructor runs first (IDF calls the init_priority array ascending, before the plain init_array).
 * Each hook is guarded by value: it writes only where esp_hosted's own function sits, so a layout mismatch (a
 * different esp_hosted version, other transport macros) refuses loudly instead of overwriting another pointer. */
static void __attribute__((constructor(101))) install(void)
{
    if (g_hosted_osi_funcs._h_restart_host == hosted_restart_host) {
        g_hosted_osi_funcs._h_restart_host = park;
        installed = true;
    } else {
        ESP_LOGE(TAG, "esp_hosted's OS table does not hold its restart where expected: guard NOT installed");
    }
    /* create and cancel together: a task this create makes is deleted by this cancel */
    if (g_hosted_osi_funcs._h_thread_create == hosted_thread_create &&
        g_hosted_osi_funcs._h_thread_cancel == hosted_thread_cancel) {
        g_hosted_osi_funcs._h_thread_create = create_pinned_thread;
        g_hosted_osi_funcs._h_thread_cancel = cancel_thread;
        pinned = true;
    } else {
        ESP_LOGE(TAG, "esp_hosted's OS table does not hold its thread create and cancel where expected: tasks NOT "
                      "pinned, stacks internal");
    }
    if (g_hosted_osi_funcs._h_malloc_align == hosted_malloc_align) {
        g_hosted_osi_funcs._h_malloc_align = malloc_align_psram;
        buffers_in_psram = true;
    } else {
        ESP_LOGE(TAG, "esp_hosted's OS table does not hold its aligned malloc where expected: buffers stay internal");
    }
}

bool hosted_no_reboot_installed(void)
{
    return installed;
}

bool hosted_tasks_pinned(void)
{
    return pinned;
}

bool hosted_link_down(void)
{
    return link_down;
}

bool hosted_buffers_in_psram(void)
{
    return buffers_in_psram;
}

uint32_t hosted_internal_fallbacks(void)
{
    return internal_fallbacks;
}
