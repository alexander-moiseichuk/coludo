/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "safe_restart.h"

#include "esp_ipc_isr.h"
#include "esp_newlib.h"
#include "esp_private/system_internal.h"
#include "esp_rom_sys.h"
#include "soc/cache_reg.h"
#include "soc/soc.h"

/* How long the restart waits for the cache's sync engine, us: one sync takes microseconds. Bounded: the done bit
 * reads 0 until the first sync since the chip's reset, and the restart must not hang on it. */
#define SYNC_WAIT_US 1000

void safe_restart(void)
{
    esp_sync_timekeeping_timers();
    /* the other core into a high-priority interrupt, spinning in IRAM; this core's interrupts masked. A sync it had
     * started finishes in hardware meanwhile. */
    esp_ipc_isr_stall_other_cpu();
    for (int waited = 0; waited < SYNC_WAIT_US && !REG_GET_BIT(CACHE_SYNC_CTRL_REG, CACHE_SYNC_DONE); waited++) {
        esp_rom_delay_us(1);
    }
    esp_restart_noos();
}
