/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* A software restart that cannot fault in its first cache write (see CMakeLists.txt). */
#pragma once

/* Restarts the board as esp_restart does -- a software CPU reset, the peripherals reset and the data cache written
 * back on the way -- with the other core stalled first and the cache's sync engine idle. Of esp_restart's shutdown
 * handlers it runs only the time of day's (esp_sync_timekeeping_timers): esp_wifi_stop is a remote call through the C6
 * that a parked link makes wait seconds, and the C6 is reset at the next boot anyway. Call it from a task. */
void safe_restart(void) __attribute__((noreturn));
