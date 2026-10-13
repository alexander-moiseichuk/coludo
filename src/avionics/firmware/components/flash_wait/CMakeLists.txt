# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# The flash chip's busy wait, slept instead of spun, for the tasks that ask. On the P4 the flash controller waits for
# the chip itself (auto wait idle, SOC_SPI_MEM_SUPPORT_AUTO_WAIT_IDLE): an erase or a page program stays one open
# command until the chip's busy bit clears, and IDF's host driver waits for that command in a bare register loop
# (v5.5.5 hal/spi_flash_hal_common.inc, spi_flash_hal_poll_cmd_done; called from spi_flash_hal_erase_sector). A 4 KiB
# sector erase spins there ~22 ms (measured on the taster, 2026-10-13), and the ring_eraser's ~8 erases a second on
# the pad held 18-28 % of core 0. Neither of IDF's own outs reaches it: SPI_FLASH_YIELD_DURING_ERASE yields only
# between the sectors of one call (the ring erases one per call), and the OS delay hook serves only the status poll
# after the command, by then idle. On this build a flash operation holds only a mutex, with the cache on and the
# scheduler running (spi_flash_os_func_app.c:29: XIP from PSRAM), so a task may sleep inside one: this component swaps
# the main chip's host driver for a copy whose command poll sleeps a tick at a time in a registered task. In C because
# the host driver table, its context and the controller's done bit are IDF's C (hal/spi_flash_hal.h, spi_flash_ll.h).
idf_component_register(SRCS "flash_wait.c"
                       INCLUDE_DIRS "include"
                       REQUIRES spi_flash freertos esp_timer hal)
