/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* A software restart that cannot fault in its first cache write (see CMakeLists.txt). */
#pragma once

/* Restarts the board as esp_restart does -- a software CPU reset, the peripherals reset and the data cache written
 * back on the way -- with the other core stalled first and the cache's sync engine idle. Of esp_restart's shutdown
 * handlers it runs only the time of day's (esp_sync_timekeeping_timers): esp_wifi_stop is a remote call through the C6
 * that a parked link makes wait seconds, and the C6 is reset at the next boot anyway. Its reset sequence is IDF's with
 * the RTC watchdog enabled through it: a stall resets the chip within 1 s instead of hanging it. Call it from a task. */
void safe_restart(void) __attribute__((noreturn));

/* A test of the guard (`panic kind=hang`): arms the RTC watchdog as the restart does and stops this core with its
 * interrupts off; the watchdog must reset the chip within 1 s, and the next boot report a watchdog reset. */
void safe_restart_hang(void) __attribute__((noreturn));
