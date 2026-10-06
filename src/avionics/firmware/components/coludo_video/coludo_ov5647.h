/* Coludo project, copyright under MIT license, Alexander Moiseichuk */
/* Private to coludo_video: the OV5647 stream-on override (see coludo_ov5647.c). */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Frame rate the next stream-on programs (0 = the mode's native rate). Referencing it also links the detect entry. */
void coludo_ov5647_set_fps(uint8_t fps);

/** The sensor's colour-bar test pattern on the next stream-on (proves the pipeline without light). */
void coludo_ov5647_set_test_pattern(bool on);

/** Bayer order the ISP is told for every mode (esp_cam_sensor_bayer_pattern_t: 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR);
 * -1 keeps the stock tables' (GBRG). Set before the sensor is detected. */
void coludo_ov5647_set_bayer(int order);

/** The module's oscillator in Hz (0 = 25 MHz, what the stock tables assume): the frame-rate stretch uses it. */
void coludo_ov5647_set_clock(uint32_t hz);
