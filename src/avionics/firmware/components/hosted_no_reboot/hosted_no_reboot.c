#include "hosted_no_reboot.h"

#include <stddef.h>
#include <stdint.h>

/* esp_hosted's own port config first: it defines the transport macros that decide the table's layout */
#include "port_esp_hosted_host_config.h"
#include "esp_hosted_os_abstraction.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "hosted_no_reboot";
static volatile bool link_down;
static bool installed;

/* esp_hosted's restart (port_esp_hosted_host_os.c; non-static, no public prototype) */
extern int hosted_restart_host(void);

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

void hosted_no_reboot_install(void)
{
    /* Guarded by value: write only where the original restart actually sits. A layout mismatch (a different
     * esp_hosted version, other transport macros) then refuses loudly instead of overwriting another pointer. */
    if (g_hosted_osi_funcs._h_restart_host != hosted_restart_host) {
        ESP_LOGE(TAG, "esp_hosted's OS table does not hold its restart where expected: guard NOT installed");
        return;
    }
    g_hosted_osi_funcs._h_restart_host = park;
    installed = true;
}

bool hosted_no_reboot_installed(void)
{
    return installed;
}

bool hosted_link_down(void)
{
    return link_down;
}
