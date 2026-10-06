/*
 * Coludo project, copyright under MIT license, Alexander Moiseichuk
 *
 * The OV5647 as Coludo uses it: the stock esp_cam_sensor driver behind one wrapped ioctl.
 *
 * esp_cam_sensor 2.6.0 ov5647_set_stream() computes MIPI_CTRL00 (0x4800) and then writes the literal
 * 0x14 (line sync, free-running clock lane) instead -- esp-video-components issue #97: the P4 receiver
 * then never sees a frame. Stream-on is done here instead: VTS for the requested frame rate, 0x4800 =
 * clock-lane gate | bus idle | line sync, then 0x0100 = 1. Everything else is the stock driver.
 *
 * Needs CONFIG_CAMERA_OV5647_AUTO_DETECT_MIPI_INTERFACE_SENSOR=n, so this is the only OV5647 detect entry.
 * Assumes the module's own 25 MHz oscillator (Raspberry-Pi OV5647 modules): the stock tables' PLL math is
 * 25 MHz (1080p 30.0 fps, 960p 45.0 fps; a 24 MHz part runs 4 % slower).
 */
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_sccb_intf.h"
#include "ov5647.h"
#include "coludo_ov5647.h"

#define _MIPI_CTRL00_CLOCK_LANE_GATE  0x20
#define _MIPI_CTRL00_LINE_SYNC        0x10
#define _MIPI_CTRL00_BUS_IDLE         0x04
#if CONFIG_CAMERA_OV5647_CSI_LINESYNC_ENABLE
#define _MIPI_CTRL00_STREAMING        (_MIPI_CTRL00_CLOCK_LANE_GATE | _MIPI_CTRL00_BUS_IDLE | _MIPI_CTRL00_LINE_SYNC)
#else
#define _MIPI_CTRL00_STREAMING        (_MIPI_CTRL00_CLOCK_LANE_GATE | _MIPI_CTRL00_BUS_IDLE)
#endif
#define _REGISTER_MIPI_CTRL00         0x4800
#define _REGISTER_MODE_SELECT         0x0100
#define _REGISTER_VTS_HIGH            0x380e
#define _REGISTER_VTS_LOW             0x380f
#define _REGISTER_TEST_PATTERN        0x503d
#define _TEST_PATTERN_COLOUR_BARS     0x80

static const char *TAG = "coludo_ov5647";

/** The stock driver's operations, with priv_ioctl replaced by _priv_ioctl. */
static esp_cam_sensor_ops_t _operations;
/** The stock priv_ioctl, called for everything but stream-on. */
static int (*_stock_priv_ioctl)(esp_cam_sensor_device_t *device, uint32_t command, void *argument);
/** Frame rate the next stream-on programs; 0 keeps the mode's native rate. */
static uint8_t _target_fps;
/** The module's oscillator. The stock tables' native rates assume 25 MHz; a 24 MHz module (the taster's, found
 * from the SEI frame stamps: every rate came out 24/25 of the request) runs them 4 % slow. */
static uint32_t _clock_hz = 25000000;

void coludo_ov5647_set_clock(uint32_t hz)
{
    _clock_hz = hz != 0 ? hz : 25000000;
}
/** Colour bars instead of the image on the next stream-on. */
static bool _test_pattern;
/** The stock driver's format query, wrapped by _query_formats. */
static int (*_stock_query_formats)(esp_cam_sensor_device_t *device, esp_cam_sensor_format_array_t *formats);
/** Bayer order the ISP is told for every mode (esp_cam_sensor_bayer_pattern_t); -1 keeps the stock tables'. */
static int _bayer = -1;
/** The stock format table with the Bayer order replaced (esp_video resolves every format to the sensor's own
 * table entry by its register list, so a corrected copy only takes effect if it IS the table). */
#define _FORMATS_MAX 8
static esp_cam_sensor_format_t _formats[_FORMATS_MAX];
static esp_cam_sensor_isp_info_t _isp_infos[_FORMATS_MAX];

void coludo_ov5647_set_bayer(int order)
{
    _bayer = order;
}

static int _query_formats(esp_cam_sensor_device_t *device, esp_cam_sensor_format_array_t *formats)
{
    int result = _stock_query_formats(device, formats);
    if (result != ESP_OK || _bayer < 0) {
        return result;
    }
    uint32_t count = formats->count < _FORMATS_MAX ? formats->count : _FORMATS_MAX;
    for (uint32_t index = 0; index < count; index++) {
        memcpy(&_formats[index], &formats->format_array[index], sizeof(_formats[index]));
        if (_formats[index].isp_info != NULL) {
            /* memcpy: both structs carry const members */
            memcpy(&_isp_infos[index], _formats[index].isp_info, sizeof(_isp_infos[index]));
            _isp_infos[index].isp_v1_info.bayer_type = (esp_cam_sensor_bayer_pattern_t)_bayer;
            _formats[index].isp_info = &_isp_infos[index];
        }
    }
    formats->format_array = _formats;
    formats->count = count;
    return ESP_OK;
}

void coludo_ov5647_set_test_pattern(bool on)
{
    _test_pattern = on;
}

void coludo_ov5647_set_fps(uint8_t fps)
{
    _target_fps = fps;
}

/** Stretches VTS (frame length in lines) so the mode runs at _target_fps on this module's oscillator. */
static esp_err_t _apply_fps(esp_cam_sensor_device_t *device)
{
    /* the mode's native rate on this oscillator, in milli-fps (integer: the rate scales with the clock) */
    uint32_t native_millifps = (uint32_t)((uint64_t)device->cur_format->fps * 1000 * _clock_hz / 25000000);
    if (_target_fps == 0 || (uint32_t)_target_fps * 1000 >= native_millifps) {
        return ESP_OK; /* the native frame length is the shortest: cannot go faster */
    }
    uint8_t high;
    uint8_t low;
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_receive_reg_a16v8(device->sccb_handle, _REGISTER_VTS_HIGH, &high), TAG, "read VTS");
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_receive_reg_a16v8(device->sccb_handle, _REGISTER_VTS_LOW, &low), TAG, "read VTS");
    uint32_t vts = ((uint32_t)high << 8) | low;
    uint32_t stretched = (vts * native_millifps + _target_fps * 500) / (_target_fps * 1000);
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_reg_a16v8(device->sccb_handle, _REGISTER_VTS_HIGH, stretched >> 8), TAG, "write VTS");
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_reg_a16v8(device->sccb_handle, _REGISTER_VTS_LOW, stretched & 0xff), TAG, "write VTS");
    ESP_LOGI(TAG, "VTS %" PRIu32 " -> %" PRIu32 " (%" PRIu32 " -> %u fps at %" PRIu32 " Hz)", vts, stretched,
             native_millifps / 1000, _target_fps, _clock_hz);
    return ESP_OK;
}

static int _priv_ioctl(esp_cam_sensor_device_t *device, uint32_t command, void *argument)
{
    if (command != ESP_CAM_SENSOR_IOC_S_STREAM || *(int *)argument == 0) {
        return _stock_priv_ioctl(device, command, argument);
    }
    ESP_RETURN_ON_ERROR(_apply_fps(device), TAG, "frame rate");
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_reg_a16v8(device->sccb_handle, _REGISTER_TEST_PATTERN,
                                                    _test_pattern ? _TEST_PATTERN_COLOUR_BARS : 0x00), TAG, "test pattern");
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_reg_a16v8(device->sccb_handle, _REGISTER_MIPI_CTRL00, _MIPI_CTRL00_STREAMING), TAG, "MIPI_CTRL00");
    ESP_RETURN_ON_ERROR(esp_sccb_transmit_reg_a16v8(device->sccb_handle, _REGISTER_MODE_SELECT, 0x01), TAG, "stream on");
    device->stream_status = 1;
    return ESP_OK;
}

ESP_CAM_SENSOR_DETECT_FN(coludo_ov5647_detect, ESP_CAM_SENSOR_MIPI_CSI, OV5647_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    esp_cam_sensor_device_t *device = ov5647_detect(config);
    if (device != NULL) {
        _operations = *device->ops;
        _stock_priv_ioctl = _operations.priv_ioctl;
        _operations.priv_ioctl = _priv_ioctl;
        _stock_query_formats = _operations.query_support_formats;
        _operations.query_support_formats = _query_formats;
        device->ops = &_operations;
    }
    return device;
}
