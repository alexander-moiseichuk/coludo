/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* The fatal panic's capture (see CMakeLists.txt): a record in RTC RAM that survives the panic's reset -- what the
 * panic handler printed, and the log lines before it. The boot reads it (panic_capture_pending), writes it to the panic
 * log, clears it, then names itself (panic_capture_context) for the record its own panic would leave. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The most panic text a capture keeps: one panic log slot's text room (crates/panics TEXT_BYTES). */
#define PANIC_CAPTURE_TEXT_BYTES 7680

/* The log tail a capture keeps: the newest log lines before the panic, as many as fit. */
#define PANIC_CAPTURE_TAIL_BYTES 2048

/* A capture the previous boot left. The strings are NUL-terminated (empty when not known); text and tail are not. */
typedef struct {
    uint32_t boot_id;       /* the boot that panicked (0 when not known) */
    uint32_t uptime_ms;     /* its uptime when the capture began */
    char session[48];
    char firmware[32];
    char config[16];
    const char *text;       /* what the panic handler printed (a Rust panic's message first), in RTC RAM until
                               panic_capture_clear */
    uint32_t length;
    const char *tail;       /* the log lines before the panic, oldest first, in RTC RAM until panic_capture_clear */
    uint32_t tail_length;
    bool intact;            /* the text is whole: its checksum matched, and nothing was cut for want of room */
} panic_capture_t;

/* Names this boot for the record a panic of it would leave: its boot id, session, firmware and config id. */
void panic_capture_context(uint32_t boot_id, const char *session, const char *firmware, const char *config);

/* Adds text to this boot's capture, starting it: a Rust panic's message, before abort() reaches the panic handler. */
void panic_capture_note(const char *text, size_t length);

/* Adds one log line (its end of line included or not) to the log tail a panic would keep. Never blocks for long and
 * never logs: the log paths call it for every line. */
void panic_capture_log(const char *line, size_t length);

/* True, and *capture filled, when the previous boot left a capture. */
bool panic_capture_pending(panic_capture_t *capture);

/* Forgets the previous boot's capture (once it is in the panic log, or cannot ever be). */
void panic_capture_clear(void);
