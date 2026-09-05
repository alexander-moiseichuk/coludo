> **GENERATED from `src/glider/config_default.py` + `src/glider/layout.py` by `tools/gen_pinmap.py` -- do not hand-edit.** Regenerate after any bus/pin change (`python3 tools/gen_pinmap.py`); `--check` fails CI if it is stale. `config_default` is the pin source of truth, validated on hardware by `test/test_pins.py`.

# ESP32-P4 pin map -- v1.0 (target layout)

The board being ordered. Derived by applying the v1.0 revision to the firmware defaults, so it matches what `layout.apply()` installs when detection picks v1.0.

## Buses

| Bus | id | Pins |
|---|---|---|
| `i2c` | 0 | sda 7, scl 8 @ 400000 |
| `i2c` | 1 | sda 31, scl 30 @ 100000 |
| `spi` | 1 | sck 48, mosi 47, miso 46, mode 3 @ 5000000 |
| `uart` | 1 | tx 20 @ 921600 |
| `uart` | 2 | tx 22, rx 23 @ 9600 |

## GPIO assignments

| GPIO | Claimed by |
|---|---|
| 7 | i2c:0 sda |
| 8 | i2c:0 scl |
| 20 | uart:1 tx |
| 22 | uart:2 tx |
| 23 | uart:2 rx |
| 26 | servo_yaw.pin |
| 27 | servo_eleron_left.pin |
| 28 | imu_lsm6dso32.int_pin |
| 29 | power_ina226.alert_pin |
| 30 | i2c:1 scl |
| 31 | i2c:1 sda |
| 32 | servo_eleron_right.pin |
| 33 | separation.pin |
| 46 | spi:1 miso |
| 47 | spi:1 mosi |
| 48 | spi:1 sck |
| 50 | imu_lsm6dso32.cs_pin |

## Device -> pins

| Device | Bus | Pin fields |
|---|---|---|
| `accel_adxl375` | spi:1 @ 0x53 | cs_pin=adxl375_cs (GPIO49), int_pin=adxl375_int (GPIO4) |
| `imu_lsm6dso32` | spi:1 @ 0x6A | cs_pin=lsm6dso32_cs (GPIO50), int_pin=lsm6dso32_int1 (GPIO28) |
| `imu_bno055` | i2c:0 @ 0x28 | - |
| `attitude` | - | - |
| `baro_icp10111` | i2c:1 @ 0x63 | - |
| `baro_bmp280` | i2c:0 @ 0x76 | - |
| `airspeed_sdp810` | i2c:1 @ 0x25 | - |
| `laser_agl` | i2c:1 @ 0x29 | - |
| `power_ina226` | i2c:0 @ 0x40 | alert_pin=ina226_alert (GPIO29) |
| `gnss` | uart:2 | - |
| `recorder` | uart:1 | - |
| `separation` | - | pin=separation_switch (GPIO33) |
| `servo_yaw` | - | pin=servo_yaw (GPIO26) |
| `servo_eleron_left` | - | pin=servo_eleron_left (GPIO27) |
| `servo_eleron_right` | - | pin=servo_eleron_right (GPIO32) |
| `sequencer` | - | - |
| `checkpoint` | - | - |
| `gnss_calib` | - | - |
| `flight` | - | - |
| `watchdog` | - | - |
| `health` | - | - |
| `field` | - | - |
| `bluetooth` | - | - |
| `wifi` | - | - |
| `cc` | - | - |

## Reserved (never assign)

GPIO6, GPIO14, GPIO15, GPIO16, GPIO17, GPIO18, GPIO19, GPIO24, GPIO25, GPIO37, GPIO38, GPIO54 -- ESP32-P4 flash/PSRAM/USB/console straps.

# Transition v0.1 -> v1.0

What to change on a v0.1 board. Derived by diffing the two configs, so it always matches what `layout.apply()` actually does.

## Devices that change bus (same physical pins)

| Device | v0.1 | v1.0 |
|---|---|---|
| `baro_icp10111` | i2c:0 | **i2c:1** |
| `airspeed_sdp810` | i2c:0 | **i2c:1** |
| `laser_agl` | i2c:0 | **i2c:1** |
| `power_ina226` | i2c:1 | **i2c:0** |

## Bus rates

| Bus | v0.1 | v1.0 |
|---|---|---|
| `i2c:1` freq | 400000 | **100000** |

## GPIOs freed

| GPIO | was |
|---|---|
| 3 | laser_agl.int_pin |
| 4 | accel_adxl375.int_pin |
| 5 | laser_agl.xshut_pin |
| 49 | accel_adxl375.cs_pin |

**Not fitted on v1.0:** `accel_adxl375`.

**Unchanged, no re-check needed:** GPIO7, GPIO8, GPIO20, GPIO22, GPIO23, GPIO26, GPIO27, GPIO28, GPIO29, GPIO30, GPIO31, GPIO32, GPIO33, GPIO46, GPIO47, GPIO48, GPIO50.

# ESP32-P4 pin map -- v0.1 (as built)

TMS-7C, TMS-7D and the breadboard until the transition above is done. A config with no `layout` key resolves here.

## Buses

| Bus | id | Pins |
|---|---|---|
| `i2c` | 0 | sda 7, scl 8 @ 400000 |
| `i2c` | 1 | sda 31, scl 30 @ 400000 |
| `spi` | 1 | sck 48, mosi 47, miso 46, mode 3 @ 5000000 |
| `uart` | 1 | tx 20 @ 921600 |
| `uart` | 2 | tx 22, rx 23 @ 9600 |

## GPIO assignments

| GPIO | Claimed by |
|---|---|
| 3 | laser_agl.int_pin |
| 4 | accel_adxl375.int_pin |
| 5 | laser_agl.xshut_pin |
| 7 | i2c:0 sda |
| 8 | i2c:0 scl |
| 20 | uart:1 tx |
| 22 | uart:2 tx |
| 23 | uart:2 rx |
| 26 | servo_yaw.pin |
| 27 | servo_eleron_left.pin |
| 28 | imu_lsm6dso32.int_pin |
| 29 | power_ina226.alert_pin |
| 30 | i2c:1 scl |
| 31 | i2c:1 sda |
| 32 | servo_eleron_right.pin |
| 33 | separation.pin |
| 46 | spi:1 miso |
| 47 | spi:1 mosi |
| 48 | spi:1 sck |
| 49 | accel_adxl375.cs_pin |
| 50 | imu_lsm6dso32.cs_pin |

## Device -> pins

| Device | Bus | Pin fields |
|---|---|---|
| `accel_adxl375` | spi:1 @ 0x53 | cs_pin=adxl375_cs (GPIO49), int_pin=adxl375_int (GPIO4) |
| `imu_lsm6dso32` | spi:1 @ 0x6A | cs_pin=lsm6dso32_cs (GPIO50), int_pin=lsm6dso32_int1 (GPIO28) |
| `imu_bno055` | i2c:0 @ 0x28 | - |
| `attitude` | - | - |
| `baro_icp10111` | i2c:0 @ 0x63 | - |
| `baro_bmp280` | i2c:0 @ 0x76 | - |
| `airspeed_sdp810` | i2c:0 @ 0x25 | - |
| `laser_agl` | i2c:0 @ 0x29 | int_pin=laser_int (GPIO3), xshut_pin=laser_xshut (GPIO5) |
| `power_ina226` | i2c:1 @ 0x40 | alert_pin=ina226_alert (GPIO29) |
| `gnss` | uart:2 | - |
| `recorder` | uart:1 | - |
| `separation` | - | pin=separation_switch (GPIO33) |
| `servo_yaw` | - | pin=servo_yaw (GPIO26) |
| `servo_eleron_left` | - | pin=servo_eleron_left (GPIO27) |
| `servo_eleron_right` | - | pin=servo_eleron_right (GPIO32) |
| `sequencer` | - | - |
| `checkpoint` | - | - |
| `gnss_calib` | - | - |
| `flight` | - | - |
| `watchdog` | - | - |
| `health` | - | - |
| `field` | - | - |
| `bluetooth` | - | - |
| `wifi` | - | - |
| `cc` | - | - |

## Reserved (never assign)

GPIO6, GPIO14, GPIO15, GPIO16, GPIO17, GPIO18, GPIO19, GPIO24, GPIO25, GPIO37, GPIO38, GPIO54 -- ESP32-P4 flash/PSRAM/USB/console straps.
