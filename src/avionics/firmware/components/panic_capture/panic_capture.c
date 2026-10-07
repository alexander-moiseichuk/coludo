/* Coludo project, copyright under MIT license, Alexander Moiseichuk */

#include "panic_capture.h"

#include <string.h>

#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "hal/uart_hal.h"
#include "sdkconfig.h"

#if !CONFIG_ESP_CONSOLE_UART
#error "panic_capture tees the panic handler's output to a UART console (uart_hal_write_txfifo): the console must be a UART"
#endif

/* "PANC": a capture began */
#define MAGIC 0x50414E43u

/* The capture as it survives the reset. */
typedef struct {
    uint32_t magic;          /* MAGIC once a capture began, 0 once the next boot cleared it */
    uint32_t magic_inverse;  /* ~MAGIC with it: 64 bits that a power-on's random RAM does not hold by chance */
    uint32_t length;         /* bytes of text */
    uint32_t crc;            /* CRC-32 of text[0..length) */
    uint32_t cut;            /* bytes the text had no room for */
    uint32_t uptime_ms;
    uint32_t boot_id;
    uint32_t tail_length;    /* bytes of tail */
    char session[48];
    char firmware[32];
    char config[16];
    char text[PANIC_CAPTURE_TEXT_BYTES];
    char tail[PANIC_CAPTURE_TAIL_BYTES];  /* the log tail when the capture began, oldest first */
} record_t;

/* The log tail of this boot: the newest PANIC_CAPTURE_TAIL_BYTES of its log lines, a byte ring. */
typedef struct {
    uint32_t written;        /* bytes ever written this boot; the ring holds the last PANIC_CAPTURE_TAIL_BYTES */
    char bytes[PANIC_CAPTURE_TAIL_BYTES];
} tail_t;

/* This boot's names, copied into the record when a capture begins. */
typedef struct {
    uint32_t boot_id;
    char session[48];
    char firmware[32];
    char config[16];
} context_t;

/* In RTC (LP) RAM, never initialised at boot: what a panic leaves there, the next boot finds. The live tail is there
 * too only to keep 2 KB off the internal heap; it starts over each boot. */
static RTC_NOINIT_ATTR record_t s_record;
static RTC_NOINIT_ATTR tail_t s_tail;
/* In internal RAM, zeroed every boot. */
static context_t s_context;
static bool s_begun;              /* this boot began a capture */
static bool s_tail_begun;         /* this boot's tail was started over */
static volatile bool s_panicking; /* the panic handler runs: tee what it prints */
static portMUX_TYPE s_tail_lock = portMUX_INITIALIZER_UNLOCKED;

/* CRC-32 (IEEE, reflected), bitwise: no table, a few KB of text once in a panic. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
    crc = ~crc;
    while (length--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

/* A NUL-terminated copy of `source` into `target` of `size` bytes, cut short if it must. */
static void copy_name(char *target, const char *source, size_t size)
{
    size_t index = 0;
    for (; source && source[index] && index + 1 < size; index++) {
        target[index] = source[index];
    }
    target[index] = '\0';
}

/* Begins this boot's capture once: the context, and the log tail as it stands, oldest first. A previous boot's record
 * not yet cleared is overwritten (the newest wins). */
static void capture_begin(void)
{
    if (s_begun) {
        return;
    }
    s_begun = true;
    s_record.magic = 0;
    s_record.length = 0;
    s_record.crc = 0;
    s_record.cut = 0;
    s_record.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_record.boot_id = s_context.boot_id;
    copy_name(s_record.session, s_context.session, sizeof s_record.session);
    copy_name(s_record.firmware, s_context.firmware, sizeof s_record.firmware);
    copy_name(s_record.config, s_context.config, sizeof s_record.config);
    /* the tail, unrolled from its ring; no lock: in a panic the other core is stalled, and a line half written by
     * this core's interrupted task shows as half a line */
    uint32_t held = 0;
    if (s_tail_begun) {
        uint32_t written = s_tail.written;
        uint32_t first = written > PANIC_CAPTURE_TAIL_BYTES ? written - PANIC_CAPTURE_TAIL_BYTES : 0;
        /* a ring that wrapped starts inside a line: begin at the next whole one */
        if (first > 0) {
            while (first < written && s_tail.bytes[first % PANIC_CAPTURE_TAIL_BYTES] != '\n') {
                first++;
            }
            first++;
        }
        for (uint32_t index = first; index < written; index++) {
            s_record.tail[held++] = s_tail.bytes[index % PANIC_CAPTURE_TAIL_BYTES];
        }
    }
    s_record.tail_length = held;
    s_record.magic_inverse = ~MAGIC;
    s_record.magic = MAGIC;
}

/* Appends to the capture what fits, counting what does not: the text, then its checksum, then its length, so a reset
 * between them leaves the text readable and the checksum telling. Both cores may tee at once (IDF lets both enter the
 * handler), so the room is checked before any arithmetic on it. */
static void capture_put(const uint8_t *data, size_t length)
{
    uint32_t used = s_record.length;
    if (used >= PANIC_CAPTURE_TEXT_BYTES) {
        s_record.cut += length;
        return;
    }
    size_t room = PANIC_CAPTURE_TEXT_BYTES - used;
    size_t kept = length > room ? room : length;
    memcpy(&s_record.text[used], data, kept);
    s_record.crc = crc32_update(s_record.crc, data, kept);
    s_record.length = used + kept;
    s_record.cut += length - kept;
}

/* esp_panic_handler (esp_system/panic.c), wrapped: a panic began, so begin the capture (a Rust panic's note may have
 * begun it already) and tee everything the handler prints from here to the reset. Not in IRAM: the handler itself runs
 * from PSRAM (CONFIG_ESP_PANIC_HANDLER_IRAM off), so IRAM here would save nothing it does not. */
extern void __real_esp_panic_handler(void *info);
void __wrap_esp_panic_handler(void *info)
{
    capture_begin();
    s_panicking = true;
    __real_esp_panic_handler(info);
}

/* uart_hal_write_txfifo (hal/uart_hal_iram.c), wrapped: the panic handler prints every character to the UART console
 * through it. Outside a panic it only passes through (the console driver uses it too). */
extern void __real_uart_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf, uint32_t data_size,
                                         uint32_t *write_size);
void __wrap_uart_hal_write_txfifo(uart_hal_context_t *hal, const uint8_t *buf, uint32_t data_size,
                                  uint32_t *write_size)
{
    __real_uart_hal_write_txfifo(hal, buf, data_size, write_size);
    if (s_panicking) {
        capture_put(buf, *write_size);
    }
}

void panic_capture_context(uint32_t boot_id, const char *session, const char *firmware, const char *config)
{
    s_context.boot_id = boot_id;
    copy_name(s_context.session, session, sizeof s_context.session);
    copy_name(s_context.firmware, firmware, sizeof s_context.firmware);
    copy_name(s_context.config, config, sizeof s_context.config);
}

void panic_capture_note(const char *text, size_t length)
{
    capture_begin();
    capture_put((const uint8_t *)text, length);
}

void panic_capture_log(const char *line, size_t length)
{
    if (s_panicking || length == 0) {
        return;
    }
    if (length > PANIC_CAPTURE_TAIL_BYTES) {
        line += length - PANIC_CAPTURE_TAIL_BYTES;
        length = PANIC_CAPTURE_TAIL_BYTES;
    }
    bool newline = line[length - 1] != '\n';
    portENTER_CRITICAL_SAFE(&s_tail_lock);
    if (!s_tail_begun) {
        s_tail_begun = true;
        s_tail.written = 0;
    }
    uint32_t written = s_tail.written;
    for (size_t index = 0; index < length; index++) {
        s_tail.bytes[(written + index) % PANIC_CAPTURE_TAIL_BYTES] = line[index];
    }
    written += length;
    if (newline) {
        s_tail.bytes[written % PANIC_CAPTURE_TAIL_BYTES] = '\n';
        written++;
    }
    s_tail.written = written;
    portEXIT_CRITICAL_SAFE(&s_tail_lock);
}

bool panic_capture_pending(panic_capture_t *capture)
{
    if (s_record.magic != MAGIC || s_record.magic_inverse != ~MAGIC || s_record.length > PANIC_CAPTURE_TEXT_BYTES ||
        s_record.tail_length > PANIC_CAPTURE_TAIL_BYTES) {
        return false;
    }
    capture->boot_id = s_record.boot_id;
    capture->uptime_ms = s_record.uptime_ms;
    copy_name(capture->session, s_record.session, sizeof capture->session);
    copy_name(capture->firmware, s_record.firmware, sizeof capture->firmware);
    copy_name(capture->config, s_record.config, sizeof capture->config);
    capture->text = s_record.text;
    capture->length = s_record.length;
    capture->tail = s_record.tail;
    capture->tail_length = s_record.tail_length;
    capture->intact = s_record.cut == 0 &&
                      crc32_update(0, (const uint8_t *)s_record.text, s_record.length) == s_record.crc;
    return true;
}

void panic_capture_clear(void)
{
    s_record.magic = 0;
    s_record.magic_inverse = 0;
}
