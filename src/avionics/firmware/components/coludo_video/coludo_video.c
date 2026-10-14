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
 *
 * Segments (M7b): on the pad a recording moves on to a new file every config->segment_seconds, bounding what one
 * damaged file can cost; once the board leaves SETTING the file runs on to the end, so a switch never happens in the
 * air. The cut is in band: at the first IDR at or past the segment's length, the encode task pads the ring with zeros
 * up to the next chunk boundary and puts the IDR there, so the old file ends padded to a whole chunk (tools/video_sei.py
 * strips the padding) and the new one begins chunk-aligned, its writes still whole aligned chunks. The writer moves on
 * when its tail reaches that boundary, after reading the flight word again: a board that left SETTING meanwhile keeps
 * the old file, the padding then zero bytes between two access units, which an Annex-B stream allows.
 *
 * The pipeline is opened by the first recording and then kept: the encoder's reference buffer must be one contiguous
 * block of INTERNAL RAM, and after a recording (or ~40 s of Wi-Fi traffic) the internal heap no longer has one (the
 * taster's second start failed "No memory for reference frame" with 271 KB free, largest block 75 KB). A stop is
 * STREAMOFF; a start re-queues the buffers, sets the rate control, forces an IDR and STREAMONs. It has two halves:
 *   - the encoder: opened first (while the heap is whole), closed only for another size or QP range;
 *   - the camera: esp_video (CSI + ISP + sensor), the capture buffers, the ISP tuning; closed for another mode or
 *     Bayer order, and after a failed start (a clean slate) -- never taking the encoder with it.
 * A failed STREAMOFF wedges the recorder: CSI/ISP may still be writing the buffers, so nothing is freed or started
 * again until a reboot.
 *
 * Image tuning: no IPA runs on rev < 3 (esp_ipa ships no library for it), so the ISP gets a fixed tuning when the
 * pipeline opens -- Bayer denoise, the OV5647's colour matrix (both from esp_cam_sensor's ov5647_default.json) and
 * a BT.709 gamma curve. Exposure and white balance are the sensor's own (coludo_ov5647.c).
 */
#include <math.h>
#include <stdio.h>
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
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_video_isp_ioctl.h"
#include "esp_h264_enc_single_hw.h"
#include "driver/ppa.h"
#include "driver/jpeg_encode.h"
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
/** Bayer-filter denoise strength (the ISP's range is 2..20; ov5647_default.json uses 5). */
#define _DENOISE_LEVEL          5
/* Grey-world white-balance trim (see _trim_balance): once a second, from 2 x 8 rows of every 4th pixel pair */
#define _BALANCE_PERIOD_FRAMES          25
#define _BALANCE_ROW_PAIRS              8
#define _BALANCE_PIXEL_PAIR_STEP        4
/** Only mid-tones are evidence: clipped highlights and black noise carry no colour of the light. */
#define _BALANCE_LUMA_LOW               24
#define _BALANCE_LUMA_HIGH              230
/** Fewer qualifying samples per chroma than this (a dark or blown-out frame): no trim. */
#define _BALANCE_EVIDENCE_MIN           256
/** Only near-neutral samples are evidence (|U| or |V| up to this): a blue sky or a green field is the scene's colour,
 * not the light's, and would drag a plain grey-world mean. */
#define _BALANCE_CHROMA_MAX             32
/** The trim's range around the sensor's own white balance: a residual, not a white balance of its own (the taster
 * settled at R 0.94 / B 0.91), so a scene that fools it cannot push the colours far. */
#define _BALANCE_GAIN_MIN               0.7f
#define _BALANCE_GAIN_MAX               1.4f
/* The flight word (crates/sequencer Flight::pack): the stage id in bits 0-2, SETTING 1, and `launched` in bit 5 */
#define _FLIGHT_STAGE_BITS              0x7u
#define _FLIGHT_STAGE_SETTING           1u
#define _FLIGHT_LAUNCHED_BIT            (1u << 5)
/** A segment's file name: its start uptime in ms, up to 20 digits, ".h264" and the terminator, after the directory. */
#define _SEGMENT_NAME_BYTES             32
/* The preview (M7b): the 1080p frame's centre 1920x1024 scaled by 6/16 (PPA SRM takes steps of 1/16) to 720x384 RGB565,
 * then the hardware JPEG (dimensions a multiple of 16 for 4:2:0; RGB565, since rev < 3 takes no YUV420 in) */
#define _PREVIEW_SOURCE_WIDTH           1920
#define _PREVIEW_SOURCE_HEIGHT          1080
#define _PREVIEW_SOURCE_ROWS            1024
#define _PREVIEW_WIDTH                  720
#define _PREVIEW_HEIGHT                 384
#define _PREVIEW_SCALE                  0.375f
#define _PREVIEW_JPEG_BYTES             (160 * 1024)
#define _PREVIEW_QUALITY                60
#define _PREVIEW_TIMEOUT_MILLISECONDS   100

static const char *TAG = "coludo_video";

/** The camera pipeline: opened by the first recording, kept open after it (see the top of this file). Only the
 * encode task of the running recording touches it (starts and stops never overlap). */
typedef struct {
    /* the camera half */
    bool camera_open;                               /*!< everything in this half is up */
    bool video_initialised;                         /*!< esp_video_init succeeded */
    int capture;                                    /*!< /dev/video0 file descriptor */
    int isp;                                        /*!< /dev/video20 file descriptor: the ISP's controls */
    uint8_t *capture_buffers[_CAPTURE_BUFFERS];     /*!< MMAP'ed CSI/ISP output buffers (PSRAM, YUV420) */
    uint16_t width;                                 /*!< the mode it was opened for */
    uint16_t height;
    int8_t bayer;
    float red_gain;                                 /*!< white-balance trim, kept from one recording to the next */
    float blue_gain;
    /* the encoder half */
    esp_h264_enc_handle_t encoder;
    esp_h264_enc_param_hw_handle_t encoder_parameters;
    uint16_t encoder_width;                         /*!< what it was allocated for */
    uint16_t encoder_height;
    uint8_t qp_min;
    uint8_t qp_max;
    bool wedged;                                    /*!< a STREAMOFF failed: no start until a reboot */
} _pipeline_t;

static _pipeline_t _pipeline = {.capture = -1, .isp = -1};

/** One recording: its file, ring and tasks. */
typedef struct {
    coludo_video_config_t config;
    uint8_t *frame;                                 /*!< encoder output, config.frame_bytes (PSRAM) */
    uint8_t *ring;                                  /*!< bitstream ring, ring_bytes (PSRAM) */
    uint32_t ring_bytes;                            /*!< config.ring_bytes rounded up to a whole chunk */
    atomic_uint_least32_t ring_head;                /*!< bytes produced, modulo 2^32 */
    atomic_uint_least32_t ring_tail;                /*!< bytes consumed, modulo 2^32 */
    int file;
    char *directory;                                /*!< the files' directory, copied from config.path (PSRAM) */
    int64_t segment_started_microseconds;           /*!< the running file's first frame (encode task only) */
    uint32_t cut_position;                          /*!< the ring position the next file starts at (chunk-aligned) */
    uint32_t cut_name_milliseconds;                 /*!< the next file's name: its first frame's uptime */
    atomic_bool cut_pending;                        /*!< a cut placed, the writer not yet past it (set after the two) */
    TaskHandle_t write_task;
    SemaphoreHandle_t finished;                     /*!< given once by each task when it exits */
    SemaphoreHandle_t started;                      /*!< given by the encode task once the pipeline is up (or failed) */
    esp_err_t start_result;
    atomic_bool stopping;
    atomic_bool encode_finished;
} _recorder_t;

static _recorder_t *_recorder;

/** The preview's engines and buffers: set up by the first preview (the encode task), kept for the boot. */
typedef struct {
    bool tried;                                     /*!< the setup ran (once a boot) */
    bool ready;                                     /*!< and succeeded */
    ppa_client_handle_t ppa;
    jpeg_encoder_handle_t jpeg;
    uint8_t *rgb;                                   /*!< the scaled frame, RGB565 (PSRAM, DMA-aligned) */
    size_t rgb_bytes;
    uint8_t *jpegs[2];                              /*!< the published one and the next (PSRAM, DMA-aligned) */
    size_t jpeg_capacity;
    uint32_t sizes[2];
    int64_t uptimes[2];
    int published;                                  /*!< the JPEG readers copy, -1 before the first */
    uint32_t sequence;
    SemaphoreHandle_t lock;                         /*!< held to publish, and to copy the published one out */
} _preview_t;

static _preview_t _preview = {.published = -1};
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

/** Whether the board is on the pad: the flight word in SETTING and never launched; false with no word. */
static bool _on_pad(const _recorder_t *recorder)
{
    if (recorder->config.flight_word == NULL) {
        return false;
    }
    uint32_t word = __atomic_load_n(recorder->config.flight_word, __ATOMIC_ACQUIRE);
    return (word & _FLIGHT_STAGE_BITS) == _FLIGHT_STAGE_SETTING && (word & _FLIGHT_LAUNCHED_BIT) == 0;
}

/**
 * Puts [AUD/SPS/PPS][SEI][slice] into the ring; drops the frame (and forces an IDR) if it does not fit. With `cut` (an
 * IDR, see the top of this file), zeros first up to the next chunk boundary, and the access unit there: the next file's
 * start, for the writer. Returns whether the cut was placed (a dropped frame places none: the next IDR tries again).
 */
static bool _put_access_unit(_recorder_t *recorder, const esp_h264_enc_out_frame_t *out, uint32_t frame,
                             int64_t uptime_microseconds, bool cut)
{
    uint8_t sei[_SEI_BYTES_MAX];
    size_t sei_bytes = _build_sei(sei, recorder->config.boot_id, frame, uptime_microseconds, _stats.frames_missed);
    uint32_t split = _first_slice_offset(out->raw_data.buffer, out->length);
    uint32_t head = atomic_load(&recorder->ring_head);
    uint32_t pad = cut ? (_CHUNK_BYTES - head % _CHUNK_BYTES) % _CHUNK_BYTES : 0;
    uint32_t total = pad + out->length + sei_bytes;
    uint32_t used = head - atomic_load(&recorder->ring_tail);
    if (total > recorder->ring_bytes - used) {
        esp_h264_enc_force_idr(&_pipeline.encoder_parameters->base);
        portENTER_CRITICAL(&_stats_lock);
        _stats.frames_dropped++;
        portEXIT_CRITICAL(&_stats_lock);
        return false;
    }
    if (cut) {
        /* the ring is whole chunks, so the padding up to a chunk boundary never wraps */
        memset(recorder->ring + head % recorder->ring_bytes, 0, pad);
        head += pad;
        recorder->cut_position = head;
        recorder->cut_name_milliseconds = (uint32_t)(uptime_microseconds / 1000);
        atomic_store(&recorder->cut_pending, true);  /* before the head moves past it: the writer sees both */
    }
    _ring_copy(recorder, head, out->raw_data.buffer, split);
    _ring_copy(recorder, head + split, sei, sei_bytes);
    _ring_copy(recorder, head + split + sei_bytes, out->raw_data.buffer + split, out->length - split);
    atomic_store(&recorder->ring_head, head + total - pad);
    xTaskNotifyGive(recorder->write_task);
    portENTER_CRITICAL(&_stats_lock);
    _stats.frames_encoded++;
    _stats.frames_idr += out->frame_type == ESP_H264_FRAME_TYPE_IDR;
    _stats.bytes_encoded += total;
    _stats.ring_used_max = MAX(_stats.ring_used_max, used + total);
    portEXIT_CRITICAL(&_stats_lock);
    return cut;
}

/** Grey-world evidence from one captured frame: sums over its mid-tone samples (U and V offset from 128). */
typedef struct {
    int32_t luma;
    int32_t u;
    int32_t v;
    uint32_t u_count;
    uint32_t v_count;
} _chroma_t;

/**
 * Samples _BALANCE_ROW_PAIRS row pairs spread over the frame, every _BALANCE_PIXEL_PAIR_STEP-th pixel pair. The
 * layout is O_UYY_E_VYY: even rows [U Y Y] and odd rows [V Y Y] per pixel pair (esp_h264's test_apps/h264_io.c).
 * The ISP wrote the frame by DMA, so each sampled row's cache lines are dropped first -- whole lines: a 1080p row
 * is 2880 bytes = 45 x 64, the frame base is line-aligned. Called before the buffer goes back to the receiver.
 */
static void _measure_chroma(const uint8_t *frame, _chroma_t *chroma)
{
    const uint32_t row_bytes = (uint32_t)_pipeline.width * 3 / 2;
    *chroma = (_chroma_t){0};
    for (uint32_t pair = 0; pair < _BALANCE_ROW_PAIRS; pair++) {
        uint32_t row = (uint32_t)(_pipeline.height / 2) * (2 * pair + 1) / (2 * _BALANCE_ROW_PAIRS) * 2;
        for (uint32_t line = 0; line < 2; line++) {
            const uint8_t *bytes = frame + (row + line) * row_bytes;
            if (esp_cache_msync((void *)bytes, row_bytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C | ESP_CACHE_MSYNC_FLAG_TYPE_DATA) != ESP_OK) {
                continue;  /* no fresh view of this row: leave it out */
            }
            for (uint32_t index = 0; index + 2 < row_bytes; index += 3 * _BALANCE_PIXEL_PAIR_STEP) {
                uint8_t luma = bytes[index + 1];
                int32_t offset = (int32_t)bytes[index] - 128;
                if (luma < _BALANCE_LUMA_LOW || luma > _BALANCE_LUMA_HIGH || offset > _BALANCE_CHROMA_MAX ||
                    offset < -_BALANCE_CHROMA_MAX) {
                    continue;
                }
                chroma->luma += luma;
                if (line == 0) {
                    chroma->u += offset;
                    chroma->u_count++;
                } else {
                    chroma->v += offset;
                    chroma->v_count++;
                }
            }
        }
    }
}

static float _clamp_gain(float gain)
{
    return gain < _BALANCE_GAIN_MIN ? _BALANCE_GAIN_MIN : gain > _BALANCE_GAIN_MAX ? _BALANCE_GAIN_MAX : gain;
}

/**
 * Grey-world white-balance trim: moves the ISP's red and blue gains a damped step toward a grey mean of the
 * near-neutral mid-tones.
 *
 * The sensor runs its own AWB, but leaves a few percent of magenta that the colour matrix (diagonal ~2) amplifies:
 * the taster's white walls came out pink (mean R 112, G 104, B 118). This loop measures the FINAL image and trims
 * what is left, so it converges whatever the matrix and gamma do. On rev < 3 (no white-balance block) esp_video
 * folds the gains into the matrix's input columns. The step is the square root of the measured G/R and G/B ratios:
 * through the matrix and gamma the loop gain is ~1, so the square root settles in a few seconds without overshoot.
 */
static void _trim_balance(const _chroma_t *chroma)
{
    if (chroma->u_count < _BALANCE_EVIDENCE_MIN || chroma->v_count < _BALANCE_EVIDENCE_MIN) {
        return;
    }
    float luma = (float)chroma->luma / (float)(chroma->u_count + chroma->v_count);
    float u = (float)chroma->u / (float)chroma->u_count;
    float v = (float)chroma->v / (float)chroma->v_count;
    /* the mean colour of the mid-tones (BT.601) */
    float red = luma + 1.402f * v;
    float green = luma - 0.344f * u - 0.714f * v;
    float blue = luma + 1.772f * u;
    if (red <= 0.0f || green <= 0.0f || blue <= 0.0f) {
        return;
    }
    _pipeline.red_gain = _clamp_gain(_pipeline.red_gain * sqrtf(green / red));
    _pipeline.blue_gain = _clamp_gain(_pipeline.blue_gain * sqrtf(green / blue));
    struct v4l2_ext_control controls[] = {
        {.id = V4L2_CID_RED_BALANCE, .value = (int32_t)(_pipeline.red_gain * V4L2_CID_RED_BALANCE_DEN + 0.5f)},
        {.id = V4L2_CID_BLUE_BALANCE, .value = (int32_t)(_pipeline.blue_gain * V4L2_CID_BLUE_BALANCE_DEN + 0.5f)},
    };
    struct v4l2_ext_controls request = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = sizeof(controls) / sizeof(controls[0]),
        .controls = controls,
    };
    bool failed = ioctl(_pipeline.isp, VIDIOC_S_EXT_CTRLS, &request) != 0;
    portENTER_CRITICAL(&_stats_lock);
    _stats.balance_errors += failed;
    _stats.red_gain_thousandths = (uint16_t)(_pipeline.red_gain * 1000.0f + 0.5f);
    _stats.blue_gain_thousandths = (uint16_t)(_pipeline.blue_gain * 1000.0f + 0.5f);
    portEXIT_CRITICAL(&_stats_lock);
}

/** The preview's engines and buffers, once a boot; false when they are not there (the failure counted once). */
static bool _preview_setup(void)
{
    if (_preview.tried) {
        return _preview.ready;
    }
    _preview.tried = true;
    ppa_client_config_t ppa = {.oper_type = PPA_OPERATION_SRM, .max_pending_trans_num = 1};
    jpeg_encode_engine_cfg_t jpeg = {.timeout_ms = _PREVIEW_TIMEOUT_MILLISECONDS};
    jpeg_encode_memory_alloc_cfg_t output = {.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER};
    size_t allocated = 0;
    size_t internal_before = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    _preview.lock = xSemaphoreCreateMutex();
    /* PPA's output must sit on PSRAM's L1 and L2 cache lines (128 bytes), address and size alike: the JPEG driver's
     * input allocation aligns to less, and PPA refused it */
    _preview.rgb_bytes = _PREVIEW_WIDTH * _PREVIEW_HEIGHT * 2;
    _preview.rgb = heap_caps_aligned_calloc(128, 1, _preview.rgb_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    _preview.jpegs[0] = jpeg_alloc_encoder_mem(_PREVIEW_JPEG_BYTES, &output, &_preview.jpeg_capacity);
    _preview.jpegs[1] = jpeg_alloc_encoder_mem(_PREVIEW_JPEG_BYTES, &output, &allocated);
    _preview.ready = _preview.lock != NULL && _preview.rgb != NULL && _preview.jpegs[0] != NULL &&
                     _preview.jpegs[1] != NULL && ppa_register_client(&ppa, &_preview.ppa) == ESP_OK &&
                     jpeg_new_encoder_engine(&jpeg, &_preview.jpeg) == ESP_OK;
    if (!_preview.ready) {
        ESP_LOGE(TAG, "preview: its engines or buffers would not come up: no preview this boot");
        portENTER_CRITICAL(&_stats_lock);
        _stats.preview_errors++;
        portEXIT_CRITICAL(&_stats_lock);
    } else {
        ESP_LOGI(TAG, "preview: %ux%u JPEG once a second (its engines took %d B of internal RAM: %u free, largest "
                 "block %u)", _PREVIEW_WIDTH, _PREVIEW_HEIGHT,
                 (int)(internal_before - heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                 heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }
    return _preview.ready;
}

/** One preview from a captured 1080p frame (YUV420, before it goes back to the receiver): scale, JPEG, publish. */
static void _preview_frame(const uint8_t *frame, int64_t uptime_microseconds)
{
    if (!_preview_setup()) {
        return;
    }
    int64_t start_microseconds = esp_timer_get_time();
    ppa_srm_oper_config_t scale = {
        .in = {.buffer = frame, .pic_w = _PREVIEW_SOURCE_WIDTH, .pic_h = _PREVIEW_SOURCE_HEIGHT,
               .block_w = _PREVIEW_SOURCE_WIDTH, .block_h = _PREVIEW_SOURCE_ROWS, .block_offset_x = 0,
               .block_offset_y = (_PREVIEW_SOURCE_HEIGHT - _PREVIEW_SOURCE_ROWS) / 2,
               .srm_cm = PPA_SRM_COLOR_MODE_YUV420, .yuv_range = PPA_COLOR_RANGE_FULL,
               .yuv_std = PPA_COLOR_CONV_STD_RGB_YUV_BT601},
        .out = {.buffer = _preview.rgb, .buffer_size = _preview.rgb_bytes, .pic_w = _PREVIEW_WIDTH,
                .pic_h = _PREVIEW_HEIGHT, .srm_cm = PPA_SRM_COLOR_MODE_RGB565},
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = _PREVIEW_SCALE,
        .scale_y = _PREVIEW_SCALE,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    jpeg_encode_cfg_t encode = {
        .height = _PREVIEW_HEIGHT,
        .width = _PREVIEW_WIDTH,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = _PREVIEW_QUALITY,
    };
    int next = _preview.published == 0 ? 1 : 0;  /* never the one a reader may be copying */
    uint32_t size = 0;
    bool made = ppa_do_scale_rotate_mirror(_preview.ppa, &scale) == ESP_OK;
    int64_t scaled_microseconds = esp_timer_get_time();
    made = made && jpeg_encoder_process(_preview.jpeg, &encode, _preview.rgb, _preview.rgb_bytes,
                                        _preview.jpegs[next], _preview.jpeg_capacity, &size) == ESP_OK;
    if (_preview.sequence < 3) {
        ESP_LOGI(TAG, "preview: scale %lld us, JPEG %lld us, %lu bytes", scaled_microseconds - start_microseconds,
                 esp_timer_get_time() - scaled_microseconds, (unsigned long)size);
    }
    if (made) {
        xSemaphoreTake(_preview.lock, portMAX_DELAY);
        _preview.sizes[next] = size;
        _preview.uptimes[next] = uptime_microseconds;
        _preview.published = next;
        _preview.sequence++;
        xSemaphoreGive(_preview.lock);
    }
    uint32_t took_microseconds = (uint32_t)(esp_timer_get_time() - start_microseconds);
    portENTER_CRITICAL(&_stats_lock);
    _stats.preview_frames += made;
    _stats.preview_errors += !made;
    _stats.preview_microseconds_max = MAX(_stats.preview_microseconds_max, took_microseconds);
    portEXIT_CRITICAL(&_stats_lock);
}

static esp_err_t _stream_on(const coludo_video_config_t *config);
static void _stream_off(void);

/** Runs one recording: streams on from this core (so the CSI and ISP interrupts land here; the H.264 one landed on the
 * core of the recording that opened the pipeline), loops, streams off. */
static void _encode_loop(void *argument)
{
    _recorder_t *recorder = argument;
    const int64_t period_microseconds = 1000000 / recorder->config.fps;
    int64_t previous_microseconds = 0;
    uint32_t frame = 0;
    uint32_t frames_since_balance = 0;
    uint32_t frames_since_preview = 0;

    recorder->start_result = _stream_on(&recorder->config);
    xSemaphoreGive(recorder->started);
    if (recorder->start_result != ESP_OK) {
        atomic_store(&recorder->stopping, true);
    }

    while (!atomic_load(&recorder->stopping)) {
        struct v4l2_buffer buffer = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(_pipeline.capture, VIDIOC_DQBUF, &buffer) != 0) {
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
        if (recorder->segment_started_microseconds == 0) {
            recorder->segment_started_microseconds = uptime_microseconds;
        }

        esp_h264_err_t result = ESP_H264_ERR_FAIL;
        esp_h264_enc_out_frame_t out = {
            .raw_data = {.buffer = recorder->frame, .len = recorder->config.frame_bytes},
        };
        int64_t encode_start_microseconds = esp_timer_get_time();
        if (recorder->config.record && !(buffer.flags & V4L2_BUF_FLAG_ERROR)) {
            esp_h264_enc_in_frame_t in = {
                .raw_data = {.buffer = _pipeline.capture_buffers[buffer.index], .len = buffer.bytesused},
                .pts = (uint32_t)(uptime_microseconds / 1000),
            };
            result = esp_h264_enc_process(_pipeline.encoder, &in, &out);
        }
        uint32_t encode_microseconds = (uint32_t)(esp_timer_get_time() - encode_start_microseconds);
        /* the white-balance trim reads the frame before it goes back; the colour bars are left as they are */
        _chroma_t chroma;
        bool balance = ++frames_since_balance >= _BALANCE_PERIOD_FRAMES && !(buffer.flags & V4L2_BUF_FLAG_ERROR) &&
                       recorder->config.test_pattern == 0;
        if (balance) {
            _measure_chroma(_pipeline.capture_buffers[buffer.index], &chroma);
            frames_since_balance = 0;
        }
        /* the preview, once a second on the pad, reads the frame by DMA before it goes back */
        if (recorder->config.preview && ++frames_since_preview >= recorder->config.fps &&
            !(buffer.flags & V4L2_BUF_FLAG_ERROR) && _pipeline.width == _PREVIEW_SOURCE_WIDTH &&
            _pipeline.height == _PREVIEW_SOURCE_HEIGHT && _on_pad(recorder)) {
            frames_since_preview = 0;
            _preview_frame(_pipeline.capture_buffers[buffer.index], uptime_microseconds);
        }
        ioctl(_pipeline.capture, VIDIOC_QBUF, &buffer);  /* back to the receiver before the copy */
        if (balance) {
            _trim_balance(&chroma);
        }

        portENTER_CRITICAL(&_stats_lock);
        _stats.frames_captured++;
        _stats.frames_missed += missed;
        _stats.encode_errors += recorder->config.record && result != ESP_H264_ERR_OK;
        _stats.encode_microseconds_last = encode_microseconds;
        _stats.encode_microseconds_max = MAX(_stats.encode_microseconds_max, encode_microseconds);
        _stats.encode_microseconds_total += encode_microseconds;
        portEXIT_CRITICAL(&_stats_lock);
        if (result == ESP_H264_ERR_OK) {
            /* the first IDR at or past the segment's length, on the pad, one cut at a time */
            bool cut = out.frame_type == ESP_H264_FRAME_TYPE_IDR && recorder->config.segment_seconds > 0 &&
                       uptime_microseconds - recorder->segment_started_microseconds >=
                           (int64_t)recorder->config.segment_seconds * 1000000 &&
                       !atomic_load(&recorder->cut_pending) && _on_pad(recorder);
            if (_put_access_unit(recorder, &out, frame, uptime_microseconds, cut)) {
                recorder->segment_started_microseconds = uptime_microseconds;
            }
        }
        /* any encoder error already made the next frame an IDR (esp_h264 >= 1.3.8) */
    }
    if (recorder->start_result == ESP_OK) {
        _stream_off();
    }
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
    _stats.write_microseconds_total += took_microseconds;
    portEXIT_CRITICAL(&_stats_lock);
}

static void _sync(_recorder_t *recorder)
{
    int64_t start_microseconds = esp_timer_get_time();
    bool failed = fsync(recorder->file) != 0;
    uint32_t took_microseconds = (uint32_t)(esp_timer_get_time() - start_microseconds);
    portENTER_CRITICAL(&_stats_lock);
    _stats.write_errors += failed;
    _stats.write_microseconds_total += took_microseconds;  /* the card is as busy flushing as writing */
    portEXIT_CRITICAL(&_stats_lock);
}

/**
 * The writer at a cut (see the top of this file): every byte before it is in the old file. On the pad still, the next
 * file opens, then the old one is fsync'ed and closed; a board that left SETTING meanwhile, or a file that would not
 * open, keeps the old file running (the open failure counted and logged).
 */
static void _cut(_recorder_t *recorder)
{
    atomic_store(&recorder->cut_pending, false);
    if (!_on_pad(recorder)) {
        ESP_LOGI(TAG, "segment: the board left the pad, the file runs on");
        return;
    }
    size_t bytes = strlen(recorder->directory) + 1 + _SEGMENT_NAME_BYTES;
    char *path = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    int file = -1;
    if (path != NULL) {
        snprintf(path, bytes, "%s/%lu.h264", recorder->directory, (unsigned long)recorder->cut_name_milliseconds);
        file = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    }
    if (file < 0) {
        ESP_LOGE(TAG, "segment: %s would not open, the file runs on", path != NULL ? path : "(no memory for its name)");
        portENTER_CRITICAL(&_stats_lock);
        _stats.write_errors++;
        portEXIT_CRITICAL(&_stats_lock);
    } else {
        _sync(recorder);
        close(recorder->file);
        recorder->file = file;
        ESP_LOGI(TAG, "segment: %s", path);
        portENTER_CRITICAL(&_stats_lock);
        _stats.segments++;
        portEXIT_CRITICAL(&_stats_lock);
    }
    heap_caps_free(path);
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
            /* a cut sits on a chunk boundary and the tail moves whole chunks from 0, so it meets it exactly */
            if (atomic_load(&recorder->cut_pending) && tail == recorder->cut_position) {
                _cut(recorder);
                synced_microseconds = esp_timer_get_time();
                unsynced = false;
            }
            _write_out(recorder, tail, _CHUNK_BYTES);
            tail += _CHUNK_BYTES;
            atomic_store(&recorder->ring_tail, tail);
            unsynced = true;
        }
        if (last && recorder->file < 0) {
            break;  /* the preview alone: no file */
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

static esp_err_t _open_capture(const coludo_video_config_t *config)
{
    _pipeline.capture = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    ESP_RETURN_ON_FALSE(_pipeline.capture >= 0, ESP_FAIL, TAG, "open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
    ESP_RETURN_ON_ERROR(_select_sensor_mode(_pipeline.capture, config->width, config->height), TAG, "sensor mode");

    struct v4l2_format format = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {.width = config->width, .height = config->height, .pixelformat = V4L2_PIX_FMT_YUV420},
    };
    ESP_RETURN_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_S_FMT, &format) == 0, ESP_FAIL, TAG, "S_FMT YUV420");

    struct timeval timeout = {.tv_sec = _CAPTURE_TIMEOUT_MICROSECONDS / 1000000, .tv_usec = _CAPTURE_TIMEOUT_MICROSECONDS % 1000000};
    ESP_RETURN_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_S_DQBUF_TIMEOUT, &timeout) == 0, ESP_FAIL, TAG, "DQBUF timeout");

    struct v4l2_requestbuffers request = {
        .count = _CAPTURE_BUFFERS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    ESP_RETURN_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_REQBUFS, &request) == 0, ESP_ERR_NO_MEM, TAG, "REQBUFS");
    for (uint32_t index = 0; index < _CAPTURE_BUFFERS; index++) {
        struct v4l2_buffer buffer = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = index,
        };
        ESP_RETURN_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_QUERYBUF, &buffer) == 0, ESP_FAIL, TAG, "QUERYBUF");
        _pipeline.capture_buffers[index] = mmap(NULL, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                                                _pipeline.capture, buffer.m.offset);
        ESP_RETURN_ON_FALSE(_pipeline.capture_buffers[index] != MAP_FAILED, ESP_FAIL, TAG, "mmap");
    }
    return ESP_OK;
}

/** Hands every capture buffer to the receiver: once per recording, because STREAMOFF takes them all back. */
static esp_err_t _queue_buffers(void)
{
    for (uint32_t index = 0; index < _CAPTURE_BUFFERS; index++) {
        struct v4l2_buffer buffer = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = index,
        };
        ESP_RETURN_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_QBUF, &buffer) == 0, ESP_FAIL, TAG, "QBUF %" PRIu32, index);
    }
    return ESP_OK;
}

/** BT.709's transfer curve (linear toe below 0.018, so the darkest noise is not amplified) at x of [0, 256]. */
static uint8_t _gamma_709(uint32_t x)
{
    float linear = (float)x / 256.0f;
    float encoded = linear < 0.018f ? 4.5f * linear : 1.099f * powf(linear, 0.45f) - 0.099f;
    float y = encoded * 255.0f + 0.5f;
    return y >= 255.0f ? 255 : (uint8_t)y;
}

/**
 * The ISP's fixed tuning, set on /dev/video20 and applied by esp_video at every stream-on:
 *   - Bayer-filter denoise (the stock pipeline runs none: the taster's indoor clip was grainy);
 *   - the OV5647's colour matrix from esp_cam_sensor's ov5647_default.json -- every row sums to 1, so the sensor's
 *     white balance stays neutral, and it restores the saturation the raw image lacks;
 *   - a BT.709 gamma curve (the stock pipeline output linear light: dark midtones, so the AEC aimed high and clipped).
 * The curve's 16 points crowd the dark end (the ISP wants power-of-two steps between them, ending at 255).
 */
static esp_err_t _tune_image(void)
{
    esp_video_isp_bf_t denoise = {
        .enable = true,
        .level = _DENOISE_LEVEL,
        .matrix = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}},
    };
    esp_video_isp_ccm_t colour = {
        .enable = true,
        .matrix = {
            {2.0000f, -0.5459f, -0.4541f},
            {-0.4751f, 1.7696f, -0.2945f},
            {-0.2002f, -0.7998f, 2.0000f},
        },
    };
    static const uint8_t _GAMMA_X[ISP_GAMMA_CURVE_POINTS_NUM] = {
        4, 8, 16, 24, 32, 48, 64, 80, 96, 112, 128, 160, 192, 224, 240, 255,
    };
    esp_video_isp_gamma_t gamma = {.enable = true};
    for (int index = 0; index < ISP_GAMMA_CURVE_POINTS_NUM; index++) {
        gamma.points[index].x = _GAMMA_X[index];
        gamma.points[index].y = _gamma_709(_GAMMA_X[index] == 255 ? 256 : _GAMMA_X[index]);
    }
    struct v4l2_ext_control controls[] = {
        {.id = V4L2_CID_USER_ESP_ISP_BF, .size = sizeof(denoise), .p_u8 = (uint8_t *)&denoise},
        {.id = V4L2_CID_USER_ESP_ISP_CCM, .size = sizeof(colour), .p_u8 = (uint8_t *)&colour},
        {.id = V4L2_CID_USER_ESP_ISP_GAMMA, .size = sizeof(gamma), .p_u8 = (uint8_t *)&gamma},
    };
    struct v4l2_ext_controls request = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = sizeof(controls) / sizeof(controls[0]),
        .controls = controls,
    };
    /* kept open: the white-balance trim sets the ISP's gains through it once a second */
    _pipeline.isp = open(ESP_VIDEO_ISP1_DEVICE_NAME, O_RDWR);
    ESP_RETURN_ON_FALSE(_pipeline.isp >= 0, ESP_FAIL, TAG, "open %s", ESP_VIDEO_ISP1_DEVICE_NAME);
    ESP_RETURN_ON_FALSE(ioctl(_pipeline.isp, VIDIOC_S_EXT_CTRLS, &request) == 0, ESP_FAIL, TAG, "ISP tuning");
    _pipeline.red_gain = 1.0f;   /* the ISP's own: no balance gains until the first trim */
    _pipeline.blue_gain = 1.0f;
    return ESP_OK;
}

/** The encoder half: esp_h264's hardware encoder for this size and QP range (fps, GOP and bit rate are set per start). */
static esp_err_t _open_encoder(const coludo_video_config_t *config)
{
    esp_h264_enc_cfg_hw_t encoder_config = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,   /* = the ISP's V4L2_PIX_FMT_YUV420 layout */
        .gop = config->gop,
        .fps = config->fps,
        .res = {.width = config->width, .height = config->height},
        .rc = {.bitrate = config->bitrate, .qp_min = config->qp_min, .qp_max = config->qp_max},
    };
    ESP_RETURN_ON_FALSE(esp_h264_enc_hw_new(&encoder_config, &_pipeline.encoder) == ESP_H264_ERR_OK,
                        ESP_ERR_NO_MEM, TAG, "esp_h264_enc_hw_new (internal RAM: %u free, largest block %u)",
                        heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    ESP_RETURN_ON_FALSE(esp_h264_enc_hw_get_param_hd(_pipeline.encoder, &_pipeline.encoder_parameters) == ESP_H264_ERR_OK,
                        ESP_FAIL, TAG, "encoder parameters");
    ESP_RETURN_ON_FALSE(esp_h264_enc_open(_pipeline.encoder) == ESP_H264_ERR_OK, ESP_FAIL, TAG, "esp_h264_enc_open");
    _pipeline.encoder_width = config->width;
    _pipeline.encoder_height = config->height;
    _pipeline.qp_min = config->qp_min;
    _pipeline.qp_max = config->qp_max;
    return ESP_OK;
}

/** Gives the encoder half back (whatever _open_encoder got through). */
static void _close_encoder(void)
{
    if (_pipeline.encoder != NULL) {
        esp_h264_enc_close(_pipeline.encoder);
        esp_h264_enc_del(_pipeline.encoder);
    }
    _pipeline.encoder = NULL;
    _pipeline.encoder_parameters = NULL;
}

/** Gives the camera half back (whatever _open_camera got through), in reverse; never called while streaming. */
static void _close_camera(void)
{
    if (_pipeline.capture >= 0) {
        struct v4l2_requestbuffers request = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
        ioctl(_pipeline.capture, VIDIOC_REQBUFS, &request);
        close(_pipeline.capture);
    }
    if (_pipeline.isp >= 0) {
        close(_pipeline.isp);
    }
    if (_pipeline.video_initialised) {
        esp_video_deinit();
    }
    _pipeline.camera_open = false;
    _pipeline.video_initialised = false;
    _pipeline.capture = -1;
    _pipeline.isp = -1;
    memset(_pipeline.capture_buffers, 0, sizeof(_pipeline.capture_buffers));
}

/** The camera half: esp_video (CSI + ISP, OV5647 detect), the capture buffers and the ISP tuning, for this mode. */
static esp_err_t _open_camera(const coludo_video_config_t *config)
{
    const esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = config->i2c_bus,
            .freq = _SCCB_HZ,
        },
        .reset_pin = GPIO_NUM_NC,                   /* CSI_IO0 is a 10K pull-up to 3V3 on this board, IO1 unconnected */
        .pwdn_pin = GPIO_NUM_NC,
        .dont_init_ldo = false,                     /* esp_video takes LDO channel 3 at 2.5 V = VDD_MIPI_DPHY here */
    };
    const esp_video_init_config_t video = {.csi = &csi};
    coludo_ov5647_set_bayer(config->bayer);         /* read by the format query during detect */
    ESP_RETURN_ON_ERROR(esp_video_init_with_flags(&video, ESP_VIDEO_INIT_FLAGS_MIPI_CSI | ESP_VIDEO_INIT_FLAGS_ISP),
                        TAG, "esp_video_init");
    _pipeline.video_initialised = true;
    ESP_RETURN_ON_ERROR(_open_capture(config), TAG, "capture");
    ESP_RETURN_ON_ERROR(_tune_image(), TAG, "image tuning");
    _pipeline.width = config->width;
    _pipeline.height = config->height;
    _pipeline.bayer = config->bayer;
    _pipeline.camera_open = true;
    return ESP_OK;
}

/**
 * Starts one recording's stream:
 *   1. drops a half allocated for something else (the encoder for another size or QP range, the camera for another
 *      mode or Bayer order);
 *   2. opens what is missing, the encoder first, while the internal heap is least fragmented;
 *   3. sets the encoder's rate control and an IDR first (the new file must decode from its first frame), queues the
 *      buffers and STREAMONs -- which runs the sensor's stream-on (frame rate, exposure: coludo_ov5647.c) and starts
 *      CSI + ISP.
 * A failure closes the camera half (STREAMOFF resets nothing on a stream that never started, so the queues need the
 * reopen) and keeps the encoder, whose internal-RAM block a reopen could not find again.
 */
static esp_err_t _stream_on(const coludo_video_config_t *config)
{
    esp_err_t ret = ESP_OK;  /* the name ESP_GOTO_ON_* assigns */
    ESP_RETURN_ON_FALSE(!_pipeline.wedged, ESP_ERR_INVALID_STATE, TAG, "an earlier STREAMOFF failed: reboot to record");
    if (_pipeline.encoder != NULL &&
        (_pipeline.encoder_width != config->width || _pipeline.encoder_height != config->height ||
         _pipeline.qp_min != config->qp_min || _pipeline.qp_max != config->qp_max)) {
        ESP_LOGI(TAG, "new size or QP range: reopening the encoder");
        _close_encoder();
    }
    if (_pipeline.camera_open &&
        (_pipeline.width != config->width || _pipeline.height != config->height || _pipeline.bayer != config->bayer)) {
        ESP_LOGI(TAG, "new mode or Bayer order: reopening the camera");
        _close_camera();
    }
    coludo_ov5647_set_fps(config->fps);
    coludo_ov5647_set_test_pattern(config->test_pattern != 0);
    coludo_ov5647_set_clock(config->sensor_clock_hz);
    coludo_ov5647_set_exposure_target(config->exposure_target);
    if (_pipeline.encoder == NULL) {
        ret = _open_encoder(config);
        if (ret != ESP_OK) {
            _close_encoder();
            return ret;
        }
    }
    if (!_pipeline.camera_open) {
        ESP_GOTO_ON_ERROR(_open_camera(config), fail, TAG, "camera");
    }
    esp_h264_enc_param_handle_t parameters = &_pipeline.encoder_parameters->base;
    ESP_GOTO_ON_FALSE(esp_h264_enc_set_fps(parameters, config->fps) == ESP_H264_ERR_OK &&
                      esp_h264_enc_set_gop(parameters, config->gop) == ESP_H264_ERR_OK &&
                      esp_h264_enc_set_bitrate(parameters, config->bitrate) == ESP_H264_ERR_OK &&
                      esp_h264_enc_force_idr(parameters) == ESP_H264_ERR_OK,
                      ESP_FAIL, fail, TAG, "rate control");
    ESP_GOTO_ON_ERROR(_queue_buffers(), fail, TAG, "buffers");
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_FALSE(ioctl(_pipeline.capture, VIDIOC_STREAMON, &type) == 0, ESP_FAIL, fail, TAG, "STREAMON");
    return ESP_OK;
fail:
    _close_camera();
    return ret;
}

/** Ends one recording's stream (sensor to standby, CSI + ISP stopped); both halves stay open for the next. */
static void _stream_off(void)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(_pipeline.capture, VIDIOC_STREAMOFF, &type) != 0) {
        /* CSI/ISP may still be writing the capture buffers: freeing them would let DMA write freed memory */
        ESP_LOGE(TAG, "STREAMOFF failed: the recorder is wedged until a reboot");
        _pipeline.wedged = true;
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
    heap_caps_free(recorder->directory);
    heap_caps_free(recorder);
}

esp_err_t coludo_video_start(const coludo_video_config_t *config)
{
    ESP_RETURN_ON_FALSE(_recorder == NULL, ESP_ERR_INVALID_STATE, TAG, "already started");
    ESP_RETURN_ON_FALSE(config->i2c_bus != NULL && (config->path != NULL || !config->record) && config->fps > 0 &&
                        config->gop > 0 &&
                        config->qp_min <= config->qp_max && config->qp_max <= 51 &&
                        config->ring_bytes >= 2 * _CHUNK_BYTES && config->frame_bytes > 0,
                        ESP_ERR_INVALID_ARG, TAG, "config");

    esp_err_t ret = ESP_OK;  /* the name ESP_GOTO_ON_* assigns */
    _recorder_t *recorder = heap_caps_calloc(1, sizeof(_recorder_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_RETURN_ON_FALSE(recorder != NULL, ESP_ERR_NO_MEM, TAG, "recorder");
    recorder->config = *config;
    recorder->config.path = NULL;  /* the caller's: read during this call only, its directory copied below */
    recorder->ring_bytes = (config->ring_bytes + _CHUNK_BYTES - 1) / _CHUNK_BYTES * _CHUNK_BYTES;
    recorder->ring = heap_caps_aligned_alloc(_CHUNK_BYTES, recorder->ring_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    recorder->frame = heap_caps_calloc(1, config->frame_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED | MALLOC_CAP_8BIT);
    recorder->finished = xSemaphoreCreateCounting(2, 0);
    recorder->started = xSemaphoreCreateBinary();
    const char *slash = config->path != NULL ? strrchr(config->path, '/') : NULL;
    size_t directory_bytes = slash != NULL ? (size_t)(slash - config->path) : 1;
    recorder->directory = heap_caps_malloc(directory_bytes + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_GOTO_ON_FALSE(recorder->ring != NULL && recorder->frame != NULL && recorder->finished != NULL &&
                      recorder->started != NULL && recorder->directory != NULL, ESP_ERR_NO_MEM, fail, TAG, "buffers");
    if (slash != NULL) {
        memcpy(recorder->directory, config->path, directory_bytes);
        recorder->directory[directory_bytes] = '\0';
    } else {
        strcpy(recorder->directory, ".");
    }

    recorder->file = config->record ? open(config->path, O_WRONLY | O_CREAT | O_TRUNC, 0644) : -1;
    ESP_GOTO_ON_FALSE(recorder->file >= 0 || !config->record, ESP_FAIL, fail, TAG, "open %s", config->path);
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
        /* _stream_on closed the pipeline and the encode task exits; the writer closes the empty file */
        xSemaphoreTake(recorder->finished, portMAX_DELAY);
        xSemaphoreTake(recorder->finished, portMAX_DELAY);
        ret = recorder->start_result;
        goto fail;
    }
    portENTER_CRITICAL(&_stats_lock);
    _stats.started_microseconds = esp_timer_get_time();
    _stats.red_gain_thousandths = (uint16_t)(_pipeline.red_gain * 1000.0f + 0.5f);  /* the trim carried over */
    _stats.blue_gain_thousandths = (uint16_t)(_pipeline.blue_gain * 1000.0f + 0.5f);
    portEXIT_CRITICAL(&_stats_lock);
    _recorder = recorder;
    return ESP_OK;

fail_file:
    if (recorder->file >= 0) {
        close(recorder->file);
    }
fail:
    _release(recorder);
    return ret;
}

esp_err_t coludo_video_reserve(const coludo_video_config_t *config)
{
    ESP_RETURN_ON_FALSE(_recorder == NULL && _pipeline.encoder == NULL, ESP_ERR_INVALID_STATE, TAG, "already open");
    ESP_RETURN_ON_FALSE(config->fps > 0 && config->gop > 0 && config->qp_min <= config->qp_max && config->qp_max <= 51,
                        ESP_ERR_INVALID_ARG, TAG, "config");
    esp_err_t result = _open_encoder(config);
    if (result != ESP_OK) {
        _close_encoder();
        return result;
    }
    ESP_LOGI(TAG, "encoder reserved for %ux%u (internal RAM left: %u free, largest block %u)", config->width,
             config->height, heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    return ESP_OK;
}

size_t coludo_video_preview(uint8_t *out, size_t capacity, uint32_t *sequence, int64_t *uptime_microseconds)
{
    if (_preview.lock == NULL) {
        return 0;
    }
    size_t size = 0;
    xSemaphoreTake(_preview.lock, portMAX_DELAY);
    if (_preview.published >= 0 && _preview.sizes[_preview.published] <= capacity) {
        size = _preview.sizes[_preview.published];
        memcpy(out, _preview.jpegs[_preview.published], size);
        if (sequence != NULL) {
            *sequence = _preview.sequence;
        }
        if (uptime_microseconds != NULL) {
            *uptime_microseconds = _preview.uptimes[_preview.published];
        }
    }
    xSemaphoreGive(_preview.lock);
    return size;
}

esp_err_t coludo_video_force_idr(void)
{
    ESP_RETURN_ON_FALSE(_recorder != NULL, ESP_ERR_INVALID_STATE, TAG, "not started");
    return esp_h264_enc_force_idr(&_pipeline.encoder_parameters->base) == ESP_H264_ERR_OK ? ESP_OK : ESP_FAIL;
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
    portENTER_CRITICAL(&_stats_lock);
    _stats.stopped_microseconds = esp_timer_get_time();
    portEXIT_CRITICAL(&_stats_lock);
    /* the last preview is no picture of now: kept, it was served as live after the stop (the dashboard showed one frozen
     * frame and never turned the camera on again); none until the camera runs again */
    if (_preview.lock != NULL) {
        xSemaphoreTake(_preview.lock, portMAX_DELAY);
        _preview.published = -1;
        xSemaphoreGive(_preview.lock);
    }
    _recorder = NULL;
    _release(recorder);
    return ESP_OK;
}
