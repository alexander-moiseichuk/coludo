/*
 * Coludo project, copyright under MIT license, Alexander Moiseichuk
 *
 * coludo_video -- capture (V4L2 /dev/video0, MMAP) -> H.264 HW encoder (esp_h264, called directly) ->
 * PSRAM byte ring -> SD writer (POSIX write of aligned 64 KB chunks, fsync once a second).
 *
 * Why esp_h264 directly and not the esp_video H.264 M2M device (/dev/video11): the M2M device has no
 * force-IDR control, does not report the frame type, and fixes the QP range at stream-on. Both paths
 * run the encode in the caller's task, blocked on the encoder's frame-done interrupt (no CPU spin),
 * and both are zero-copy on the input side.
 *
 * Tasks (both on config->encode_core):
 *   encode: DQBUF -> timestamp -> esp_h264_enc_process (blocks ~25-33 ms at 1080p) -> QBUF -> SEI + copy to ring
 *   write:  whole 64 KB chunks from the ring -> write(); fsync() once a second; tail + close on stop
 * The CPU touches the YUV frame only through a cache write-back call; it copies the bitstream (~40 KB
 * per frame at 8 Mbit/s) once, into the ring.
 */
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdatomic.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_h264_enc_single_hw.h"
#include "coludo_video.h"
#include "coludo_ov5647.h"

/** Capture buffers: one held by the CSI receiver, one being encoded, one spare for scheduling jitter. */
#define _CAPTURE_BUFFERS        3
/** SD write unit: ring offsets and file offsets stay multiples of this (direct PSRAM -> SDMMC DMA). */
#define _CHUNK_BYTES            (64 * 1024)
/** fsync period: a crash loses at most this much video plus one chunk. */
#define _SYNC_PERIOD_MICROSECONDS         1000000
/** DQBUF timeout, so a dead sensor shows up in the counters and stop() is never blocked. */
#define _CAPTURE_TIMEOUT_MICROSECONDS     1000000
/** SCCB clock for the OV5647 (it is on the flight sensors' bus). */
#define _SCCB_HZ                400000
/** Room for one SEI NAL with emulation-prevention bytes. */
#define _SEI_BYTES_MAX          96
#define _SEI_PAYLOAD_BYTES      36
#define _TASK_STACK_BYTES       4096

static const char *TAG = "coludo_video";

typedef struct {
    coludo_video_config_t config;
    int capture;                                    /*!< /dev/video0 file descriptor */
    uint8_t *capture_buffers[_CAPTURE_BUFFERS];     /*!< MMAP'ed CSI/ISP output buffers (PSRAM, YUV420) */
    esp_h264_enc_handle_t encoder;
    esp_h264_enc_param_hw_handle_t encoder_parameters;
    uint8_t *frame;                                 /*!< encoder output, config.frame_bytes (PSRAM) */
    uint8_t *ring;                                  /*!< bitstream ring, ring_bytes (PSRAM) */
    uint32_t ring_bytes;                            /*!< config.ring_bytes rounded up to a whole chunk */
    atomic_uint_least32_t ring_head;                /*!< bytes produced, modulo 2^32 */
    atomic_uint_least32_t ring_tail;                /*!< bytes consumed, modulo 2^32 */
    int file;
    bool video_initialised;                         /*!< esp_video_init succeeded */
    bool streaming;                                 /*!< STREAMON succeeded */
    TaskHandle_t write_task;
    SemaphoreHandle_t finished;                     /*!< given once by each task when it exits */
    SemaphoreHandle_t started;                      /*!< given by the encode task once the pipeline is up (or failed) */
    esp_err_t start_result;
    atomic_bool stopping;
    atomic_bool encode_finished;
} _recorder_t;

static _recorder_t *_recorder;
/** Counters live outside the recorder so coludo_video_stats() never touches freed memory. */
static coludo_video_stats_t _stats;
static portMUX_TYPE _stats_lock = portMUX_INITIALIZER_UNLOCKED;

/** Appends the H.264 emulation-prevention-escaped bytes of `rbsp` to `out`; returns the new length. */
static size_t _escape(uint8_t *out, size_t length, const uint8_t *rbsp, size_t rbsp_bytes)
{
    int zeros = 0;
    for (size_t index = 0; index < rbsp_bytes; index++) {
        if (zeros >= 2 && rbsp[index] <= 3) {
            out[length++] = 0x03;
            zeros = 0;
        }
        out[length++] = rbsp[index];
        zeros = rbsp[index] == 0 ? zeros + 1 : 0;
    }
    return length;
}

static void _put_le(uint8_t *out, uint64_t value, int bytes)
{
    for (int index = 0; index < bytes; index++) {
        out[index] = (uint8_t)(value >> (8 * index));
    }
}

/** Builds the per-frame SEI NAL (start code included); returns its length. */
static size_t _build_sei(uint8_t *out, uint32_t boot_id, uint32_t frame, int64_t uptime_microseconds, uint32_t dropped)
{
    uint8_t rbsp[2 + _SEI_PAYLOAD_BYTES + 1];
    rbsp[0] = 5;                                    /* payloadType: user_data_unregistered */
    rbsp[1] = _SEI_PAYLOAD_BYTES;                   /* payloadSize */
    memset(&rbsp[2], 0, 16);
    memcpy(&rbsp[2], COLUDO_VIDEO_SEI_UUID, sizeof(COLUDO_VIDEO_SEI_UUID) - 1);
    _put_le(&rbsp[18], boot_id, 4);
    _put_le(&rbsp[22], frame, 4);
    _put_le(&rbsp[26], (uint64_t)uptime_microseconds, 8);
    _put_le(&rbsp[34], dropped, 4);
    rbsp[38] = 0x80;                                /* rbsp_trailing_bits */
    static const uint8_t header[] = {0x00, 0x00, 0x00, 0x01, 0x06};
    memcpy(out, header, sizeof(header));
    return _escape(out, sizeof(header), rbsp, sizeof(rbsp));
}

/** Offset of the start code of the first slice NAL (type 1 or 5): the SEI goes there, after AUD/SPS/PPS. */
static uint32_t _first_slice_offset(const uint8_t *bitstream, uint32_t length)
{
    for (uint32_t index = 0; index + 3 < length; index++) {
        if (bitstream[index] == 0 && bitstream[index + 1] == 0 && bitstream[index + 2] == 1) {
            uint8_t type = bitstream[index + 3] & 0x1f;
            if (type == 1 || type == 5) {
                return (index > 0 && bitstream[index - 1] == 0) ? index - 1 : index;
            }
        }
    }
    return 0;
}

/** Copies `bytes` to the ring at absolute position `position` (wraps). */
static void _ring_copy(_recorder_t *recorder, uint32_t position, const uint8_t *source, uint32_t bytes)
{
    uint32_t offset = position % recorder->ring_bytes;
    uint32_t first = MIN(bytes, recorder->ring_bytes - offset);
    memcpy(recorder->ring + offset, source, first);
    memcpy(recorder->ring, source + first, bytes - first);
}

/** Puts [AUD/SPS/PPS][SEI][slice] into the ring; drops the frame (and forces an IDR) if it does not fit. */
static void _put_access_unit(_recorder_t *recorder, const esp_h264_enc_out_frame_t *out, uint32_t frame, int64_t uptime_microseconds)
{
    uint8_t sei[_SEI_BYTES_MAX];
    size_t sei_bytes = _build_sei(sei, recorder->config.boot_id, frame, uptime_microseconds, _stats.frames_missed);
    uint32_t split = _first_slice_offset(out->raw_data.buffer, out->length);
    uint32_t total = out->length + sei_bytes;
    uint32_t head = atomic_load(&recorder->ring_head);
    uint32_t used = head - atomic_load(&recorder->ring_tail);
    if (total > recorder->ring_bytes - used) {
        esp_h264_enc_force_idr(&recorder->encoder_parameters->base);
        portENTER_CRITICAL(&_stats_lock);
        _stats.frames_dropped++;
        portEXIT_CRITICAL(&_stats_lock);
        return;
    }
    _ring_copy(recorder, head, out->raw_data.buffer, split);
    _ring_copy(recorder, head + split, sei, sei_bytes);
    _ring_copy(recorder, head + split + sei_bytes, out->raw_data.buffer + split, out->length - split);
    atomic_store(&recorder->ring_head, head + total);
    xTaskNotifyGive(recorder->write_task);
    portENTER_CRITICAL(&_stats_lock);
    _stats.frames_encoded++;
    _stats.frames_idr += out->frame_type == ESP_H264_FRAME_TYPE_IDR;
    _stats.bytes_encoded += total;
    _stats.ring_used_max = MAX(_stats.ring_used_max, used + total);
    portEXIT_CRITICAL(&_stats_lock);
}

static esp_err_t _open_pipeline(_recorder_t *recorder);
static void _close_pipeline(_recorder_t *recorder);

/** Owns the pipeline: opens it on this core (so the CSI, ISP and H.264 interrupts land here), loops, closes it. */
static void _encode_loop(void *argument)
{
    _recorder_t *recorder = argument;
    const int64_t period_microseconds = 1000000 / recorder->config.fps;
    int64_t previous_microseconds = 0;
    uint32_t frame = 0;

    recorder->start_result = _open_pipeline(recorder);
    xSemaphoreGive(recorder->started);
    if (recorder->start_result != ESP_OK) {
        atomic_store(&recorder->stopping, true);
    }

    while (!atomic_load(&recorder->stopping)) {
        struct v4l2_buffer buffer = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(recorder->capture, VIDIOC_DQBUF, &buffer) != 0) {
            portENTER_CRITICAL(&_stats_lock);
            _stats.capture_timeouts++;
            portEXIT_CRITICAL(&_stats_lock);
            continue;
        }
        int64_t uptime_microseconds = esp_timer_get_time();
        uint32_t missed = 0;
        if (previous_microseconds != 0) {
            uint32_t periods = (uint32_t)((uptime_microseconds - previous_microseconds + period_microseconds / 2) / period_microseconds);
            missed = periods > 1 ? periods - 1 : 0;
            frame += missed + 1;
        }
        previous_microseconds = uptime_microseconds;

        esp_h264_err_t result = ESP_H264_ERR_FAIL;
        esp_h264_enc_out_frame_t out = {
            .raw_data = {.buffer = recorder->frame, .len = recorder->config.frame_bytes},
        };
        int64_t encode_start_microseconds = esp_timer_get_time();
        if (!(buffer.flags & V4L2_BUF_FLAG_ERROR)) {
            esp_h264_enc_in_frame_t in = {
                .raw_data = {.buffer = recorder->capture_buffers[buffer.index], .len = buffer.bytesused},
                .pts = (uint32_t)(uptime_microseconds / 1000),
            };
            result = esp_h264_enc_process(recorder->encoder, &in, &out);
        }
        uint32_t encode_microseconds = (uint32_t)(esp_timer_get_time() - encode_start_microseconds);
        ioctl(recorder->capture, VIDIOC_QBUF, &buffer);  /* back to the receiver before the copy */

        portENTER_CRITICAL(&_stats_lock);
        _stats.frames_captured++;
        _stats.frames_missed += missed;
        _stats.encode_errors += result != ESP_H264_ERR_OK;
        _stats.encode_microseconds_last = encode_microseconds;
        _stats.encode_microseconds_max = MAX(_stats.encode_microseconds_max, encode_microseconds);
        _stats.encode_microseconds_total += encode_microseconds;
        portEXIT_CRITICAL(&_stats_lock);
        if (result == ESP_H264_ERR_OK) {
            _put_access_unit(recorder, &out, frame, uptime_microseconds);
        }
        /* any encoder error already made the next frame an IDR (esp_h264 >= 1.3.8) */
    }
    _close_pipeline(recorder);
    atomic_store(&recorder->encode_finished, true);
    xTaskNotifyGive(recorder->write_task);
    xSemaphoreGive(recorder->finished);
    vTaskDelete(NULL);
}

/** Writes ring[tail .. tail+bytes) (never wraps: callers pass whole chunks or the in-order tail pieces). */
static void _write_out(_recorder_t *recorder, uint32_t tail, uint32_t bytes)
{
    int64_t start_microseconds = esp_timer_get_time();
    ssize_t written = write(recorder->file, recorder->ring + tail % recorder->ring_bytes, bytes);
    uint32_t took_microseconds = (uint32_t)(esp_timer_get_time() - start_microseconds);
    portENTER_CRITICAL(&_stats_lock);
    _stats.write_errors += written != (ssize_t)bytes;
    _stats.bytes_written += written > 0 ? (uint64_t)written : 0;
    _stats.write_microseconds_max = MAX(_stats.write_microseconds_max, took_microseconds);
    portEXIT_CRITICAL(&_stats_lock);
}

static void _sync(_recorder_t *recorder)
{
    if (fsync(recorder->file) != 0) {
        portENTER_CRITICAL(&_stats_lock);
        _stats.write_errors++;
        portEXIT_CRITICAL(&_stats_lock);
    }
}

static void _write_loop(void *argument)
{
    _recorder_t *recorder = argument;
    int64_t synced_microseconds = esp_timer_get_time();
    bool unsynced = false;

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        bool last = atomic_load(&recorder->encode_finished);
        uint32_t tail = atomic_load(&recorder->ring_tail);
        uint32_t head = atomic_load(&recorder->ring_head);
        while (head - tail >= _CHUNK_BYTES) {
            _write_out(recorder, tail, _CHUNK_BYTES);
            tail += _CHUNK_BYTES;
            atomic_store(&recorder->ring_tail, tail);
            unsynced = true;
        }
        if (last) {
            uint32_t remaining = head - tail;
            uint32_t first = MIN(remaining, recorder->ring_bytes - tail % recorder->ring_bytes);
            _write_out(recorder, tail, first);
            if (remaining > first) {
                _write_out(recorder, tail + first, remaining - first);
            }
            atomic_store(&recorder->ring_tail, head);
            _sync(recorder);
            close(recorder->file);
            break;
        }
        if (unsynced && esp_timer_get_time() - synced_microseconds >= _SYNC_PERIOD_MICROSECONDS) {
            _sync(recorder);
            synced_microseconds = esp_timer_get_time();
            unsynced = false;
        }
    }
    xSemaphoreGive(recorder->finished);
    vTaskDelete(NULL);
}

/** Selects the OV5647 mode of the requested size (both 1080p and 960p are compiled in). */
static esp_err_t _select_sensor_mode(int capture, uint16_t width, uint16_t height)
{
    struct v4l2_sensor_format_enum mode = {0};
    for (mode.index = 0; ioctl(capture, VIDIOC_ENUM_SENSOR_FMT, &mode) == 0; mode.index++) {
        if (mode.format.width == width && mode.format.height == height) {
            ESP_LOGI(TAG, "sensor mode %s", mode.format.name);
            return ioctl(capture, VIDIOC_S_SENSOR_FMT, &mode.format) == 0 ? ESP_OK : ESP_FAIL;
        }
    }
    ESP_LOGE(TAG, "no %ux%u sensor mode", width, height);
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t _open_capture(_recorder_t *recorder)
{
    const coludo_video_config_t *config = &recorder->config;
    recorder->capture = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    ESP_RETURN_ON_FALSE(recorder->capture >= 0, ESP_FAIL, TAG, "open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
    ESP_RETURN_ON_ERROR(_select_sensor_mode(recorder->capture, config->width, config->height), TAG, "sensor mode");

    struct v4l2_format format = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {.width = config->width, .height = config->height, .pixelformat = V4L2_PIX_FMT_YUV420},
    };
    ESP_RETURN_ON_FALSE(ioctl(recorder->capture, VIDIOC_S_FMT, &format) == 0, ESP_FAIL, TAG, "S_FMT YUV420");

    struct timeval timeout = {.tv_sec = _CAPTURE_TIMEOUT_MICROSECONDS / 1000000, .tv_usec = _CAPTURE_TIMEOUT_MICROSECONDS % 1000000};
    ESP_RETURN_ON_FALSE(ioctl(recorder->capture, VIDIOC_S_DQBUF_TIMEOUT, &timeout) == 0, ESP_FAIL, TAG, "DQBUF timeout");

    struct v4l2_requestbuffers request = {
        .count = _CAPTURE_BUFFERS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ESP_RETURN_ON_FALSE(ioctl(recorder->capture, VIDIOC_REQBUFS, &request) == 0, ESP_ERR_NO_MEM, TAG, "REQBUFS");
    for (uint32_t index = 0; index < _CAPTURE_BUFFERS; index++) {
        struct v4l2_buffer buffer = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = index,
        };
        ESP_RETURN_ON_FALSE(ioctl(recorder->capture, VIDIOC_QUERYBUF, &buffer) == 0, ESP_FAIL, TAG, "QUERYBUF");
        recorder->capture_buffers[index] = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                                recorder->capture, buffer.m.offset);
        ESP_RETURN_ON_FALSE(recorder->capture_buffers[index] != MAP_FAILED, ESP_FAIL, TAG, "mmap");
        ESP_RETURN_ON_FALSE(ioctl(recorder->capture, VIDIOC_QBUF, &buffer) == 0, ESP_FAIL, TAG, "QBUF");
    }
    return ESP_OK;
}

static esp_err_t _open_encoder(_recorder_t *recorder)
{
    const coludo_video_config_t *config = &recorder->config;
    esp_h264_enc_cfg_hw_t encoder_config = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,   /* = the ISP's V4L2_PIX_FMT_YUV420 layout */
        .gop = config->gop,
        .fps = config->fps,
        .res = {.width = config->width, .height = config->height},
        .rc = {.bitrate = config->bitrate, .qp_min = config->qp_min, .qp_max = config->qp_max},
    };
    ESP_RETURN_ON_FALSE(esp_h264_enc_hw_new(&encoder_config, &recorder->encoder) == ESP_H264_ERR_OK,
                        ESP_ERR_NO_MEM, TAG, "esp_h264_enc_hw_new");
    ESP_RETURN_ON_FALSE(esp_h264_enc_hw_get_param_hd(recorder->encoder, &recorder->encoder_parameters) == ESP_H264_ERR_OK,
                        ESP_FAIL, TAG, "encoder parameters");
    ESP_RETURN_ON_FALSE(esp_h264_enc_open(recorder->encoder) == ESP_H264_ERR_OK, ESP_FAIL, TAG, "esp_h264_enc_open");
    return ESP_OK;
}

static esp_err_t _open_pipeline(_recorder_t *recorder)
{
    esp_err_t ret = ESP_OK;  /* the name ESP_GOTO_ON_* assigns */
    const esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = recorder->config.i2c_bus,
            .freq = _SCCB_HZ,
        },
        .reset_pin = GPIO_NUM_NC,                   /* CSI_IO0 is a 10K pull-up to 3V3 on this board, IO1 unconnected */
        .pwdn_pin = GPIO_NUM_NC,
        .dont_init_ldo = false,                     /* esp_video takes LDO channel 3 at 2.5 V = VDD_MIPI_DPHY here */
    };
    const esp_video_init_config_t video = {.csi = &csi};
    coludo_ov5647_set_fps(recorder->config.fps);
    coludo_ov5647_set_test_pattern(recorder->config.test_pattern != 0);
    coludo_ov5647_set_bayer(recorder->config.bayer);
    coludo_ov5647_set_clock(recorder->config.sensor_clock_hz);
    ESP_GOTO_ON_ERROR(esp_video_init_with_flags(&video, ESP_VIDEO_INIT_FLAGS_MIPI_CSI | ESP_VIDEO_INIT_FLAGS_ISP),
                      done, TAG, "esp_video_init");
    recorder->video_initialised = true;
    ESP_GOTO_ON_ERROR(_open_capture(recorder), done, TAG, "capture");
    ESP_GOTO_ON_ERROR(_open_encoder(recorder), done, TAG, "encoder");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_FALSE(ioctl(recorder->capture, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, done, TAG, "STREAMON");
    recorder->streaming = true;
done:
    return ret;
}

/** Undoes whatever _open_pipeline got through, in reverse. */
static void _close_pipeline(_recorder_t *recorder)
{
    if (recorder->streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(recorder->capture, VIDIOC_STREAMOFF, &type);
    }
    if (recorder->encoder != NULL) {
        esp_h264_enc_close(recorder->encoder);
        esp_h264_enc_del(recorder->encoder);
    }
    if (recorder->capture >= 0) {
        struct v4l2_requestbuffers request = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
        ioctl(recorder->capture, VIDIOC_REQBUFS, &request);
        close(recorder->capture);
    }
    if (recorder->video_initialised) {
        esp_video_deinit();
    }
}

static void _release(_recorder_t *recorder)
{
    if (recorder->finished != NULL) {
        vSemaphoreDelete(recorder->finished);
    }
    if (recorder->started != NULL) {
        vSemaphoreDelete(recorder->started);
    }
    heap_caps_free(recorder->frame);
    heap_caps_free(recorder->ring);
    heap_caps_free(recorder);
}

esp_err_t coludo_video_start(const coludo_video_config_t *config)
{
    ESP_RETURN_ON_FALSE(_recorder == NULL, ESP_ERR_INVALID_STATE, TAG, "already started");
    ESP_RETURN_ON_FALSE(config->i2c_bus != NULL && config->path != NULL && config->fps > 0 && config->gop > 0 &&
                        config->qp_min <= config->qp_max && config->qp_max <= 51 &&
                        config->ring_bytes >= 2 * _CHUNK_BYTES && config->frame_bytes > 0,
                        ESP_ERR_INVALID_ARG, TAG, "config");

    esp_err_t ret = ESP_OK;  /* the name ESP_GOTO_ON_* assigns */
    _recorder_t *recorder = heap_caps_calloc(1, sizeof(_recorder_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(recorder != NULL, ESP_ERR_NO_MEM, TAG, "recorder");
    recorder->config = *config;
    recorder->capture = -1;
    recorder->ring_bytes = (config->ring_bytes + _CHUNK_BYTES - 1) / _CHUNK_BYTES * _CHUNK_BYTES;
    recorder->ring = heap_caps_aligned_alloc(_CHUNK_BYTES, recorder->ring_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    recorder->frame = heap_caps_calloc(1, config->frame_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED | MALLOC_CAP_8BIT);
    recorder->finished = xSemaphoreCreateCounting(2, 0);
    recorder->started = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(recorder->ring != NULL && recorder->frame != NULL && recorder->finished != NULL &&
                      recorder->started != NULL, ESP_ERR_NO_MEM, fail, TAG, "buffers");

    recorder->file = open(config->path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    ESP_GOTO_ON_FALSE(recorder->file >= 0, ESP_FAIL, fail, TAG, "open %s", config->path);
    ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(_write_loop, "video_write", _TASK_STACK_BYTES, recorder,
                                              config->write_priority, &recorder->write_task, config->encode_core) == pdPASS,
                      ESP_ERR_NO_MEM, fail_file, TAG, "write task");

    portENTER_CRITICAL(&_stats_lock);
    memset(&_stats, 0, sizeof(_stats));
    portEXIT_CRITICAL(&_stats_lock);
    if (xTaskCreatePinnedToCore(_encode_loop, "video_encode", _TASK_STACK_BYTES, recorder,
                                config->encode_priority, NULL, config->encode_core) != pdPASS) {
        atomic_store(&recorder->encode_finished, true);  /* the writer closes the empty file */
        xTaskNotifyGive(recorder->write_task);
        xSemaphoreTake(recorder->finished, portMAX_DELAY);
        ret = ESP_ERR_NO_MEM;
        goto fail;
    }
    xSemaphoreTake(recorder->started, portMAX_DELAY);
    if (recorder->start_result != ESP_OK) {
        /* the encode task closed what it opened and exits; the writer closes the empty file */
        xSemaphoreTake(recorder->finished, portMAX_DELAY);
        xSemaphoreTake(recorder->finished, portMAX_DELAY);
        ret = recorder->start_result;
        goto fail;
    }
    _recorder = recorder;
    return ESP_OK;

fail_file:
    close(recorder->file);
fail:
    _release(recorder);
    return ret;
}

esp_err_t coludo_video_force_idr(void)
{
    ESP_RETURN_ON_FALSE(_recorder != NULL, ESP_ERR_INVALID_STATE, TAG, "not started");
    return esp_h264_enc_force_idr(&_recorder->encoder_parameters->base) == ESP_H264_ERR_OK ? ESP_OK : ESP_FAIL;
}

void coludo_video_stats(coludo_video_stats_t *stats)
{
    portENTER_CRITICAL(&_stats_lock);
    *stats = _stats;
    portEXIT_CRITICAL(&_stats_lock);
}

esp_err_t coludo_video_stop(void)
{
    _recorder_t *recorder = _recorder;
    ESP_RETURN_ON_FALSE(recorder != NULL, ESP_ERR_INVALID_STATE, TAG, "not started");
    atomic_store(&recorder->stopping, true);
    xSemaphoreTake(recorder->finished, portMAX_DELAY);  /* encode task (within one DQBUF timeout) */
    xSemaphoreTake(recorder->finished, portMAX_DELAY);  /* writer: tail written, fsync'ed, closed */
    _recorder = NULL;
    _release(recorder);
    return ESP_OK;
}
