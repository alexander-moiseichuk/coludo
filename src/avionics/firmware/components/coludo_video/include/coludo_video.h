/*
 * Coludo project, copyright under MIT license, Alexander Moiseichuk
 *
 * coludo_video -- the flight camera recorder: OV5647 (MIPI-CSI, 2 lanes, RAW10) -> ISP (YUV420)
 * -> H.264 hardware encoder (baseline, CAVLC) -> raw Annex-B file on a mounted FAT volume.
 *
 * Every access unit carries one SEI user_data_unregistered NAL (UUID below) just before its slice:
 *   uuid[16] | boot_id u32 | frame u32 | uptime_microseconds u64 | dropped u32      (little-endian, 36 bytes)
 * `frame` counts sensor frames (dropped ones included), `uptime_microseconds` is esp_timer at the end of the
 * frame's readout. Every IDR is preceded by SPS+PPS, so the file is decodable from any IDR.
 *
 * Callable from Rust (plain C types, no unions or bitfields).
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The SEI user_data_unregistered UUID: "coludo-video-v1\0" */
#define COLUDO_VIDEO_SEI_UUID "coludo-video-v1"

/** Recorder configuration; every field is required (no hidden defaults). */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;  /*!< the caller's I2C master bus the camera's SCCB is on (port 0, SDA 7 / SCL 8) */
    uint16_t width;                   /*!< sensor mode width: 1920 (1080p RAW10) or 1280 (960p binned RAW10) */
    uint16_t height;                  /*!< sensor mode height: 1080 or 960 */
    uint8_t fps;                      /*!< frame rate, at most the mode's native rate (30 / 45); lowered via VTS */
    uint8_t gop;                      /*!< IDR period in frames (1..255); fps -> one IDR per second */
    uint8_t qp_min;                   /*!< rate-control QP floor (0..51) */
    uint8_t qp_max;                   /*!< rate-control QP ceiling (qp_min..51) */
    uint32_t bitrate;                 /*!< target bit/s */
    uint32_t frame_bytes;             /*!< largest encoded frame accepted (larger ones are dropped, next frame IDR) */
    uint32_t ring_bytes;              /*!< PSRAM bitstream ring between encoder and SD writer (absorbs SD stalls) */
    uint32_t boot_id;                 /*!< stamped into every SEI to tie the video to the telemetry */
    uint8_t encode_core;              /*!< core of the capture+encode task */
    uint8_t encode_priority;          /*!< FreeRTOS priority of the capture+encode task */
    uint8_t write_priority;           /*!< FreeRTOS priority of the SD writer task (same core) */
    const char *path;                 /*!< the first file, <directory>/<start uptime ms>.h264, truncated on start; its
                                           directory is copied, for the segments beside it (read only during the call) */
    uint8_t test_pattern;             /*!< 1: the sensor's colour bars instead of the image (pipeline check) */
    int8_t bayer;                     /*!< Bayer order for the ISP (0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR), -1: the driver's */
    uint32_t sensor_clock_hz;         /*!< the module's oscillator (0 = 25 MHz); sets the frame-rate stretch */
    uint8_t exposure_target;          /*!< the sensor AEC's mean linear luma, 8-bit (0: left as the registers hold it) */
    uint16_t segment_seconds;         /*!< on the pad, a new file at the first IDR this long into a file (0: one file) */
    uint8_t preview;                  /*!< 1: on the pad, a JPEG of the latest frame once a second (720x384, the 1080p
                                           mode's centre 1920x1024), for coludo_video_preview; 0: none */
    const uint32_t *flight_word;      /*!< the board's flight word, read atomically (crates/sequencer Flight::pack: the
                                           stage in bits 0-2, SETTING 1; launched bit 5): no cut once it leaves SETTING.
                                           Lives as long as the board; NULL: never cut */
} coludo_video_config_t;

/** Counters since the last start; all monotonic, kept after stop. */
typedef struct {
    uint32_t frames_captured;         /*!< frames dequeued from the CSI device */
    uint32_t frames_missed;           /*!< sensor frames the CSI dropped (from timestamp gaps) */
    uint32_t frames_encoded;          /*!< frames that reached the ring */
    uint32_t frames_idr;              /*!< of which IDR */
    uint32_t frames_dropped;          /*!< encoded frames dropped: ring full or frame larger than frame_bytes */
    uint32_t encode_errors;           /*!< esp_h264 failures (timeouts, overflows) */
    uint32_t capture_timeouts;        /*!< DQBUF waits that saw no frame for one second */
    uint32_t write_errors;            /*!< failed write()/fsync() calls */
    uint64_t bytes_encoded;           /*!< bytes put into the ring (SEI included) */
    uint64_t bytes_written;           /*!< bytes the writer handed to the file */
    uint32_t encode_microseconds_last;          /*!< duration of the last esp_h264_enc_process() */
    uint32_t encode_microseconds_max;           /*!< longest esp_h264_enc_process() */
    uint64_t encode_microseconds_total;         /*!< sum of esp_h264_enc_process() durations */
    uint32_t write_microseconds_max;            /*!< longest single chunk write() */
    uint64_t write_microseconds_total;          /*!< time in write() and fsync(): the card's busy time */
    int64_t started_microseconds;               /*!< esp_timer when the stream came up (0: never) */
    int64_t stopped_microseconds;               /*!< esp_timer when the file was closed (0: still recording) */
    uint32_t ring_used_max;           /*!< ring high-water mark in bytes */
    uint16_t red_gain_thousandths;    /*!< the white-balance trim's red gain x 1000 (1000 = none) */
    uint16_t blue_gain_thousandths;   /*!< the white-balance trim's blue gain x 1000 */
    uint32_t balance_errors;          /*!< trims the ISP refused */
    uint32_t segments;                /*!< files the recording moved on to at a cut (0: still its first file) */
    uint32_t preview_frames;          /*!< preview JPEGs made */
    uint32_t preview_errors;          /*!< preview scales or encodes that failed (a failed setup counts once) */
    uint32_t preview_microseconds_max;          /*!< longest preview (scale + JPEG), the capture buffer held */
} coludo_video_stats_t;

/**
 * Opens the file, starts the writer and the encode task; the encode task starts the stream on config->encode_core,
 * so its interrupts land there. The first start also opens the pipeline, which later starts reuse: the encoder (a
 * start with another size or QP range reopens it) and the camera -- esp_video (CSI + ISP, OV5647 detect over
 * config->i2c_bus), the capture buffers and the ISP tuning (another mode or Bayer order reopens it). Returns once
 * streaming, or with the first error: the file and tasks undone, the camera closed, the encoder kept.
 * ESP_ERR_INVALID_STATE after a failed STREAMOFF (nothing records again until a reboot).
 */
esp_err_t coludo_video_start(const coludo_video_config_t *config);

/**
 * Opens the encoder half for config (its size, QP range, and the rate control a start then resets) and keeps it,
 * so the first coludo_video_start finds it. Call it at boot, before anything fragments the internal heap: the
 * encoder's reference buffer is one ~138 KB block of INTERNAL RAM at 1080p, and the M3 ring threads' stacks and
 * buffers left the largest block at 128 KB. Its interrupt lands on the calling task's core (that of the encode task).
 * Only the encoder fields of config are read. ESP_ERR_INVALID_STATE when the encoder is open already.
 */
esp_err_t coludo_video_reserve(const coludo_video_config_t *config);

/**
 * Copies the latest preview JPEG (config.preview) into out: its size, 0 when there is none yet or it does not fit
 * capacity. sequence (may be NULL) counts the previews made, uptime_microseconds (may be NULL) is its frame's. Any
 * task; the copy is made under a lock the encode task takes only to publish the next one.
 */
size_t coludo_video_preview(uint8_t *out, size_t capacity, uint32_t *sequence, int64_t *uptime_microseconds);

/** Makes the next encoded frame an IDR (non-blocking, any task). */
esp_err_t coludo_video_force_idr(void);

/** Copies the counters of the running (or the last) recording. */
void coludo_video_stats(coludo_video_stats_t *stats);

/** Stops the stream, flushes the ring to the file, fsyncs and closes it, and frees the recording's buffers. The
 * pipeline stays open (its encoder holds a contiguous internal-RAM block a later start could not find again). */
esp_err_t coludo_video_stop(void);

#ifdef __cplusplus
}
#endif
