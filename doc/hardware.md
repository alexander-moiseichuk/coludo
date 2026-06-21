# Main hardware

Preliminary list before checking weight, as it should fit to 100 gramms limitation for F6 engine.
Another important limitation is power consumption which nice to keep under 5V 0.7A as per LM7805 stabilizer.

## Platform

To have good enough performance (as micropython is running on single core) the esp32-p4 selected as potential working
solution: [FireBeetle 2 ESP32-P4 AI Development Kit MIPI CSI DSI Wi-Fi 6 and IO Expansion Board](https://www.dfrobot.com/product-2950.html).
WiFi is a nice option to have remote (telnet) console before and for some period of time during launch.

If this unit will be not enough [low-end ARM](https://www.cnx-software.com/2025/10/31/sakura-pi-rk3308b-sbc-offers-rgb-lcd-interface-supports-mainline-linux/) 
could be an option but may lead too much battery pack weigh.

**Required, weight 30.5g**

## Sensor & actuator inventory (on hand)

Parts physically available now (2026-06) — these drive the Phase-2 sensor drivers in
`src/glider/drivers/`. Most sensors sit on `i2c:0`; the ADXL375 is on its own `spi:1` bus and GNSS on
`uart:2`. The I²C addresses are distinct.

| Quantity | Part | Bus / addr | Notes |
| --- | --- | --- | --- |
| high-G accel | **ADXL375** ([Adafruit 5374](https://www.adafruit.com/product/5374)) | SPI `1` (cs 49) | ±200 g; moved off I²C for clean high-rate reads (see wiring below) |
| attitude (9-DOF) + baro | **sen0253** = BNO055 + BMP280 | I²C `0x28` / `0x76` | one board, two devices |
| pressure | **sen0517** = ICP-10111 | I²C `0x63` | primary altimeter |
| AGL laser | **VL53L4CX** ([Adafruit 5425](https://www.adafruit.com/product/5425)) | I²C `0x29` | ToF, low-altitude (<~6–10 m) |
| GNSS | **ATGM336H** | UART 9600, 10 Hz | position; may lose lock under high-g |

**Interchangeable / additional options:**
[AHT20+BMP280](https://www.aliexpress.us/item/3256806546750874.html) (temp + baro),
[MPU6050](https://www.amazon.com/dp/B0BMY15TC4) (cheap IMU, noisy under high-g),
[VL53L0X / VL53L1X](https://www.aliexpress.us/item/3256807793059841.html) (alternate ToF for the `agl`
quantity). These provide the same quantities, so the fusion layer can use them as drop-in fallbacks.

## Accelerometer and Gyro

Something like [MPU6050](https://www.amazon.com/dp/B0BMY15TC4?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_13) highly not recommended due to noise for high moving cases. [HPR Rocket Flight Computer](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer) points the best `LSM6DSOX` keeping wide range as acceptable including `MPU6050`. 

[bno055](https://www.dfrobot.com/product-1793.html)'s own accelerometer is only 16 g (30–100 g is
recommended for boost), so the high-G channel is the dedicated [ADXL375 (±200 g)](https://www.adafruit.com/product/5374).
The **BNO055 is still on board** (as part of the **sen0253** combo, with BMP280) and provides the
**9-DOF attitude/orientation** for stabilisation — the two complement each other (ADXL375 = high-G
accel, BNO055 = attitude). MPU6050 stays a cheap fallback IMU only.

### ADXL375 → SPI wiring (Adafruit 5374 → ESP32-P4)

The ADXL375 runs on its own **SPI(1)** bus (mode 3, 5 MHz), off the shared I²C so its ~100 Hz reads
never queue behind the baros. **Watch the breakout labels** — on the Adafruit board the SPI data pins
are silk-printed by their I²C names, so the mapping is *not* one-to-one:

- **SDA = MOSI** (controller→sensor, "SDI") → wire to the controller's **MOSI**.
- **SDO = MISO** (sensor→controller) → wire to the controller's **MISO**.

| ADXL375 breakout pin | meaning | ESP32-P4 GPIO | net |
| --- | --- | --- | --- |
| VIN | power | 3V3 | 3.3 V |
| GND | ground | GND | ground |
| SCL | SPI clock (SCK) | **48** | SPI1 SCK |
| **SDA** | SPI **MOSI** (SDI) | **47** | SPI1 MOSI |
| **SDO** | SPI **MISO** | **46** | SPI1 MISO |
| CS | chip-select (active low) | **49** | `adxl375_cs` |
| INT1 | DATA_READY | **4** | `adxl375_int` |

To revert to I²C: tie CS high, wire SDA/SCL to GPIO7/8, and set the component `bus: 'i2c', id: 0`
(the driver keeps the I²C path; `addr 0x53`).

**Required, weight 1.0 g**

## Altimeter (pressure)
Primary is the [Gravity: ICP-10111 Pressure Sensor](https://www.dfrobot.com/product-2525.html) (on hand as
**sen0517**) — accuracy (8.5 cm), sampling rate and power (2 mA) all look good. The **BMP280** on the sen0253
combo is the backup baro (lower priority in fusion); an AHT20+BMP280 board is an alternate.

**Required, weight 7.1g**

## Altimeter (laser)
Barometer works very badly at very low altitudes, so the laser module becomes essential to cover the 10 meters and below range.
**Chosen: [VL53L4CX](https://www.adafruit.com/product/5425)** ToF ranger (I²C `0x29`) — covers the close range well enough.
VL53L0X / VL53L1X are drop-in alternates; the [50m TOF Laser Ranging Sensor, 100Hz](https://www.dfrobot.com/product-2923.html)
([sen0648 spec](https://wiki.dfrobot.com/SKU_SEN0648_TOF_laser_ranging_sensor_50m)) is the long-range fallback if needed.

**as power consumption needs another battery, weight ~20g**

## Battery
Not many options for [5V USB-C low-weight](https://www.amazon.com/dp/B07SZKNST4) power delivery are avaialbe.
Alternative is to connect e.g. from [6F22 9V using plug](https://www.amazon.com/dp/B083QFFH66) and a lightweight power-down module or a LiPo 3.7 V single cell battery and boost up circuit for controller and a separate circuit for the servos.

**Required, weight 42.1g for 6F22, and power-down board**

## Separation switch
Detects stage separation — when the engine has burned out and the booster throws the glider away (parachute opens).
**Chosen design (light + reliable): two adhesive copper pads**, one on the glider and one on the booster. While nested
the pads touch and route **3V3 to the pin → HIGH (connected/nested)**; after separation the pad opens and the pin reads
**LOW (separated)**. A 4.7 kΩ pull-down between pin and ground can be added if needed, though usually not required on ESP32.
(Note: this **flips the polarity** of the earlier Gravity Crash Sensor button idea, which read LOW=nested.)

**Required, weight ~negligible (copper tape) — was 6.2g for the button option**

## Camera
Some simple camera under 4K nice to have, ideally with autofocus. Used [Camera for Raspberry Pi](https://www.dfrobot.com/product-1179.html)
just because it was in shop to fit free delivery. Due to performance restrictions and software global lock it is more optimal to make it its own isolated module.

**Optional, weight 4.5g**

## Navigation
As accelerometer might be not enough for landing into proper zone and glissade, the auxillary
[Teyleten Robot ATGM336H GPS+BDS Dual-Mode Module Flight Control Satellite Positioning Navigator](https://www.amazon.com/dp/B09LQDG1HY)
will be helpful as has [low weight, sufficient accuracy, <30 mA power consumption and up to 10 Hz update rate](https://docs.cirkitdesigner.com/component/ab5c0c19-2fd9-4121-964e-1009970a950a/gps-atgm336h). Through fast movement the GPS may lose lock with satelites, so its software implementation and its enablment will not be a priority.

**Required, weight 7.4g**

## Servos for fins
Candidates (SG90 expected primary — cheap, compact, light):
- **SG90 Micro Servo 9g, 180°** ([temu](https://share.temu.com/XLKTfrLq6oC)) — primary.
- **MG90S Metal Gear, 360°** ([temu](https://share.temu.com/2K2ks3JEWZC)).
- **MG996R, 180°** ([temu](https://share.temu.com/CWtCeOW2kVC)) — heavier (~55 g), only if torque demands it.

**Gearing/transmission**: a reduction can trade angle for force and lower sustained current — 180°→90°/60°/45°.
**60° looks interesting** (gives **±30°** of fin throw at ~4× torque); 45° (±22.5°) is too little angle.

**Power**: servos run from their **own boost rail** (expected 5V, can be 7/9/12V if needed), separate from the
controller; **per-pin diode protection** is required. In case of high current peaks, drive the servos **sequentially**
(not all at once).

**Required, weight 10.6g per each engine and wires, at least 2 are required**

## SD card
Any suitable by size and throughput as code, videos and logs will be written here. 

**Optional (only for camera), weight 1g**

# Auxillary hardware

## Logic analyser
Just a popular and sufficient unit to check what is happening [HiLetgo USB Logic Analyzer Device with EMI Ferrite Ring USB Cable 24MHz 8CH 24MHz 8 Channel UART IIC SPI Debug](https://www.amazon.com/dp/B077LSG5P2)
Some set of [Goupchn SMD IC Test Hook Clips 10PCS 10 Colors for Logic Analyzer](https://www.amazon.com/dp/B0D3ZWTCW4) will speedup process.

## Power meter 
For periodic checks during development how much power consumed something like [USB C Tester Power Meter](https://www.amazon.com/dp/B0DFBSFL38).
If device allow to pass commands over USB it will be much better as will allow to control situation when wifi console is off.


# Potential configurations

There are a number of composition options possible

| Engines | Components configuration         | Weight [g] | Notes                                                                            |
| ------- | -------------------------------- | ---------- | -------------------------------------------------------------------------------- |
|    2    | required only components         |   117.7    | very minimalistic version without video recording                                |
|    2    | required and optional components |   123.2    | this one is plan B if glider will go out of weight or power consumption targets  |
|    3    | required only components         |   128.3    | more control but no video, not much difference to plan A                         |
|    3    | required and optional components |   133.8    | this one working target configuration as plan A                                  |
|    4    | required only components         |   138.9    | no idea how it will be useful                                                    |
|    4    | required and optional components |   144.4    | rich case, controllable on top level, but could be issues with power consumption |

**Note:** sticky pads for attaching boards and engines furniture not added
