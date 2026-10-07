/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* IDF's log lines into a callback, next to the console (see CMakeLists.txt). */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Receives one formatted IDF log line: not NUL-terminated, cut to the hook's line buffer, and it may end in '\n'
 * and carry ANSI colour codes. It runs on the task that logged, with that task's stack, so it must be quick and
 * shallow, must not log, must not block and must not allocate. */
typedef bool (*log_hook_sink_t)(const char *text, size_t length);

/* Route every IDF log line to `sink` as well as to the console. Install once. */
void log_hook_install(log_hook_sink_t sink);

/* The lines the sink never saw because another task was in the hook at the same moment (best effort: the console
 * still printed them). */
uint32_t log_hook_dropped(void);
