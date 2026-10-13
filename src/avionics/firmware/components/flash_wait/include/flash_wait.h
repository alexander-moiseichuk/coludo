/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* The flash chip's busy wait, slept instead of spun, for the tasks that ask (see CMakeLists.txt). */
#pragma once

#include "esp_err.h"

/* Swaps the main flash chip's host driver for a copy whose command poll sleeps in a task that called
 * flash_wait_sleep(); every other caller spins as IDF does. Call once, before such a task starts. Returns
 * ESP_ERR_INVALID_STATE when the main chip has no host driver yet. */
esp_err_t flash_wait_install(void);

/* Makes the calling task sleep through the flash chip's busy waits from now on: a command still open after 100 us
 * (an erase, a page program) is polled once a tick, so it ends up to 1 ms later, with its CPU free meanwhile. Returns
 * ESP_ERR_NO_MEM when FLASH_WAIT_TASKS tasks are registered already. */
esp_err_t flash_wait_sleep(void);
