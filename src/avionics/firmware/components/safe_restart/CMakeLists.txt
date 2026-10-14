# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# A software restart that cannot fault in its first cache write. IDF's esp_restart (v5.5.5, esp32p4
# system_internal.c) resets the other core without stalling it first, then writes the L1 data cache back through the
# cache's sync engine: a core reset in the middle of a cache sync leaves the engine busy, and the write-back's first
# register store faults (esp-idf#19171). That fault skips the restart's module resets, and the boards then came up
# without the microSD and the C6 link until a chip reset (7F, 2026-10-12). This restart stalls the other core in an
# interrupt first, so it is in no sync when it is reset, and waits for the engine to go idle. Its reset sequence is
# IDF's esp_restart_noos, line for line, with the RTC watchdog ENABLED through it: IDF's configures the watchdog after
# disabling it and never enables it (and the app runs with it off), so a stall in the cache write-back or the resets
# hung the taster silently (2026-10-13) instead of resetting it. In C because the cache registers, the watchdog HAL
# and the reset sequence are IDF's C (soc/cache_reg.h, hal/wdt_hal.h, esp_private/system_internal.h).
idf_component_register(SRCS "safe_restart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_system esp_rom esp_hw_support hal newlib soc)
