/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "log_hook.h"

#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_log.h"
#include "panic_capture.h"

/* The longest line the recorder keeps; a longer one is cut. IDF's lines are tag + one sentence, well under it. */
#define LINE_BYTES 256

/* The Rust callback; NULL until installed. */
static log_hook_sink_t sink;

/* What printed the lines before the hook: the console. */
static vprintf_like_t console;

/* One line buffer for all tasks, in .bss rather than on the logging task's stack: IDF logs from tasks with tight
 * stacks (the system event task has 2.3 KB), and 256 more bytes there could overflow one. A task that finds it
 * busy skips the copy and counts it, never waits: logs are best effort. */
static char line[LINE_BYTES];

/* Taken while a task formats into `line` and the sink reads it. */
static atomic_flag busy = ATOMIC_FLAG_INIT;

/* Lines the sink missed because `line` was busy. */
static atomic_uint_least32_t dropped;

/* The vprintf IDF calls once per log line: a copy for the sink, then the console as before. */
static int hook(const char *format, va_list arguments)
{
    if (sink != NULL) {
        if (!atomic_flag_test_and_set_explicit(&busy, memory_order_acquire)) {
            va_list copy;
            va_copy(copy, arguments);
            int length = vsnprintf(line, sizeof line, format, copy);
            va_end(copy);
            if (length > 0) {
                size_t kept = length < (int)sizeof line ? (size_t)length : sizeof line - 1;
                sink(line, kept);
                /* the panic log keeps the newest lines before a panic */
                panic_capture_log(line, kept);
            }
            atomic_flag_clear_explicit(&busy, memory_order_release);
        } else {
            atomic_fetch_add_explicit(&dropped, 1, memory_order_relaxed);
        }
    }
    /* `console` is stored just after esp_log_set_vprintf returns: a line logged in between finds it unset, and the
     * console IDF starts with is plain vprintf */
    return console != NULL ? console(format, arguments) : vprintf(format, arguments);
}

void log_hook_install(log_hook_sink_t callback)
{
    sink = callback;
    console = esp_log_set_vprintf(hook);
}

uint32_t log_hook_dropped(void)
{
    return atomic_load_explicit(&dropped, memory_order_relaxed);
}
