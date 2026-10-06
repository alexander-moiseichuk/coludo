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
    const char *path;                 /*!< output file, e.g. "/sdcard/v0001.h264"; truncated on start */
    uint8_t test_pattern;             /*!< 1: the sensor's colour bars instead of the image (pipeline check) */
    int8_t bayer;                     /*!< Bayer order for the ISP (0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR), -1: the driver's */
    uint32_t sensor_clock_hz;         /*!< the module's oscillator (0 = 25 MHz); sets the frame-rate stretch */
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
    uint32_t ring_used_max;           /*!< ring high-water mark in bytes */
} coludo_video_stats_t;

/**
 * Opens the file, starts the writer and the encode task; the encode task brings up esp_video (CSI + ISP,
 * OV5647 detect over config->i2c_bus), the encoder and the stream on config->encode_core, so their
 * interrupts land there. Returns once streaming (or with the first error, everything undone).
 */
esp_err_t coludo_video_start(const coludo_video_config_t *config);

/** Makes the next encoded frame an IDR (non-blocking, any task). */
esp_err_t coludo_video_force_idr(void);

/** Copies the counters of the running (or the last) recording. */
void coludo_video_stats(coludo_video_stats_t *stats);

/** Stops capture, flushes the ring to the file, fsyncs and closes it, and releases every resource. */
esp_err_t coludo_video_stop(void);

#ifdef __cplusplus
}
#endif
