/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* The flash chip's busy wait, slept instead of spun, for the tasks that ask (see CMakeLists.txt). */
#include "flash_wait.h"

#include <stdatomic.h>

#include "esp_flash.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/spi_flash_hal.h"
#include "hal/spi_flash_ll.h"

/* The tasks that sleep through the chip's busy waits: the ring's threads. */
#define FLASH_WAIT_TASKS 4

/* How long a registered task spins before it sleeps: every command but an erase or a page program is done in a few
 * microseconds (a status read, a write enable, a 64-byte read), and those never sleep. */
#define SPIN_US 100

/* The main chip's host driver with the command poll swapped; the table the host points at once installed. */
static spi_flash_host_driver_t s_driver;

/* IDF's own command poll: the spin every other caller keeps. */
static void (*s_spin)(spi_flash_host_inst_t *host);

/* The registered tasks; a free slot is NULL. */
static _Atomic(TaskHandle_t) s_sleepers[FLASH_WAIT_TASKS];

/* Whether the calling task is registered. */
static bool sleeper(void)
{
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (int index = 0; index < FLASH_WAIT_TASKS; index++) {
        if (atomic_load_explicit(&s_sleepers[index], memory_order_acquire) == self) {
            return true;
        }
    }
    return false;
}

/* The host's wait for its command: IDF's spin, except in a registered task that may block, which spins SPIN_US and
 * then sleeps a tick at a time until the controller reports the command done (its auto wait idle holds an erase or a
 * page program open until the chip's busy bit clears). */
static void poll_cmd_done(spi_flash_host_inst_t *host)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING || !xPortCanYield() || !sleeper()) {
        s_spin(host);
        return;
    }
    spi_dev_t *dev = ((spi_flash_hal_context_t *)host)->spi;
    int64_t until_us = esp_timer_get_time() + SPIN_US;
    while (!spi_flash_ll_cmd_is_done(dev)) {
        if (esp_timer_get_time() >= until_us) {
            vTaskDelay(1);
        }
    }
}

esp_err_t flash_wait_install(void)
{
    esp_flash_t *chip = esp_flash_default_chip;
    if (chip == NULL || chip->host == NULL || chip->host->driver == NULL || chip->host->driver->poll_cmd_done == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (chip->host->driver == &s_driver) {
        return ESP_OK;
    }
    s_driver = *chip->host->driver;
    s_spin = s_driver.poll_cmd_done;
    s_driver.poll_cmd_done = poll_cmd_done;
    /* the copy is complete before a flash call on the other core can see the pointer */
    atomic_thread_fence(memory_order_release);
    chip->host->driver = &s_driver;
    return ESP_OK;
}

esp_err_t flash_wait_sleep(void)
{
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    for (int index = 0; index < FLASH_WAIT_TASKS; index++) {
        TaskHandle_t free = NULL;
        if (atomic_compare_exchange_strong(&s_sleepers[index], &free, self) || free == self) {
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}
