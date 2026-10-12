# Coludo project, copyright under MIT license, Alexander Moiseichuk
#
# A software restart that cannot fault in its first cache write. IDF's esp_restart (v5.5.5, esp32p4
# system_internal.c) resets the other core without stalling it first, then writes the L1 data cache back through the
# cache's sync engine: a core reset in the middle of a cache sync leaves the engine busy, and the write-back's first
# register store faults (esp-idf#19171). That fault skips the restart's module resets, and the boards then came up
# without the microSD and the C6 link until a chip reset (7F, 2026-10-12). This restart stalls the other core in an
# interrupt first, so it is in no sync when it is reset, and waits for the engine to go idle. In C because the cache
# registers and esp_restart_noos are IDF's C (soc/cache_reg.h, esp_private/system_internal.h).
idf_component_register(SRCS "safe_restart.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_system esp_rom newlib soc)
