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
`src/glider/drivers/`. Most sensors sit on `i2c:0`; the two raw IMUs (LSM6DSO32 + ADXL375) share
`spi:1` for clean high-rate reads, GNSS is on `uart:2`. The I²C addresses are distinct. Full per-pin
wiring is in [`waveshare_esp32p4_pins.md`](waveshare_esp32p4_pins.md).

| Role | Part | Bus / addr | Notes |
| --- | --- | --- | --- |
| **6-DoF (accel + gyro)** | **LSM6DSO32** | SPI `1` (cs 50) | ±32 g + ±2000 dps; **primary** accel (airspeed/boost) and the only gyro `rate` |
| high-G accel | **ADXL375** ([Adafruit 5374](https://www.adafruit.com/product/5374)) | SPI `1` (cs 49) | ±200 g; **>32 g backstop only** — LSM6DSO32 already covers the 8–12 g boost |
| **attitude (9-DOF)** + baro | **sen0253** = BNO055 + BMP280 | I²C `0x28` / `0x76` | one board, two devices; **BNO055 is flight-critical** (sole heading) |
| pressure | **sen0517** = ICP-10111 | I²C `0x63` | primary altimeter |
| AGL laser | **VL53L4CX** ([Adafruit 5425](https://www.adafruit.com/product/5425)) | I²C `0x29` | ToF, low-altitude (<~6–10 m) |
| GNSS | **ATGM336H** | UART 9600, 10 Hz | position; may lose lock under high-g |

**Interchangeable / additional options:**
[AHT20+BMP280](https://www.aliexpress.us/item/3256806546750874.html) (temp + baro),
[MPU6050](https://www.amazon.com/dp/B0BMY15TC4) (cheap IMU, noisy under high-g),
[VL53L0X / VL53L1X](https://www.aliexpress.us/item/3256807793059841.html) (alternate ToF for the `agl`
quantity). These provide the same quantities, so the fusion layer can use them as drop-in fallbacks.

## Flight criticality — what we cannot fly without

What actually gates a launch, sorted by how badly its loss hurts. "Critical" = **no-fly without it**;
"important" = fly degraded; "optional" = nice-to-have / data only.

| Device | Class | Why | On hand |
| --- | --- | --- | --- |
| ESP32-P4 controller | **Critical** | the flight computer | ✔ |
| **BNO055** (attitude + heading) | **Critical — NO-FLY WITHOUT** | sole source of fused 9-DoF attitude *and* magnetometer heading; both the stabilisation PID and the bank-to-turn navigation depend on it. LSM6DSO32 is raw 6-DoF (no mag, no fusion) and **cannot** replace it. | **only 2 — must order more** |
| LSM6DSO32 (6-DoF) | **Critical** (lean-bundle primary) | primary accel for the airspeed integrator + boost detect, and the only gyro `rate` | ✔ (best-bundle) |
| Power (5 V controller rail + servo rail) | **Critical** | — | ✔ |
| Servos ×≥2 (SG90) | **Critical** | the fin actuators | ✔ |
| Separation switch (copper pads) | **Critical** | the BOOSTING→GLIDING trigger | ✔ |
| ICP-10111 baro | Important | primary altimeter (apogee / glide profile) | ✔ |
| VL53L4CX laser | Important | low-altitude AGL (<~10 m) for the landing — baro is poor there | ✔ |
| ADXL375 (±200 g) | Optional | >32 g high-g backstop; LSM6DSO32 ±32 g already covers the 8–12 g boost. Keep for telemetry / data-quality launches (run both, compare traces) | ✔ |
| BMP280 baro | Optional | backup baro (rides on the sen0253 board with BNO055 anyway) | ✔ |
| ATGM336H GNSS | Optional | aux position; loses lock under high-g, non-priority in fusion | ✔ |
| Camera + SD | Optional | nice-to-have, isolated module | ✔ |

> **⚠ BNO055 is the one hard blocker.** We have **2 units**; that is enough for a single board (and
> the maketboard wants two BNO055 anyway, since with one IMU attitude is a single point of failure).
> **Order more BNO055 (sen0253) before fielding multiple units** — every flight unit needs one, and
> none of the other sensors can stand in for the magnetometer-referenced heading.

The leanest flyable sensor set is therefore **LSM6DSO32 (accel + gyro) + BNO055 (attitude/heading)**
+ a baro + the laser; ADXL375 / BMP280 / GNSS / camera are add-ons.

## Accelerometer and Gyro

Something like [MPU6050](https://www.amazon.com/dp/B0BMY15TC4?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_13) highly not recommended due to noise for high moving cases. [HPR Rocket Flight Computer](https://github.com/SparkyVT/HPR-Rocket-Flight-Computer) points the best `LSM6DSOX` keeping wide range as acceptable including `MPU6050`. 

Three parts split the IMU job; the roles do **not** overlap, so dropping one is not free:

- **LSM6DSO32 — primary 6-DoF (accel + gyro).** ±32 g covers the 8–12 g boost without clipping and
  still gives ~1 mg resolution at 1 g, so it is the lead `accel` for the g12 airspeed integrator and
  boost detect, and the sole `rate` (gyro) source. The LSM6DSOX-family part the HPR computer favours.
- **BNO055 — attitude/heading (flight-critical).** Its own accelerometer is only ±16 g, but that is
  not why it is on board: it is the **only** device that outputs **fused 9-DoF orientation +
  magnetometer-referenced heading**, which the stabilisation PID and bank-to-turn navigation both
  need. A raw 6-DoF cannot stand in (gyro-only yaw drifts within seconds without a magnetometer).
  See *Flight criticality* above — **we have only 2; order more.**
- **ADXL375 — ±200 g backstop (optional).** Its sole edge over LSM6DSO32 is surviving a **>32 g**
  shock (hard chute snap, tumble, off-nominal motor) without clipping. Not needed for a nominal
  flight; keep it for telemetry / data-quality launches where both accels log side by side.

MPU6050 stays a cheap fallback IMU only.

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
(the driver keeps the I²C path; `addr 0x53`). LSM6DSO32 now shares this same SPI1 bus on its own
chip-select (cs 50) — see [`waveshare_esp32p4_pins.md`](waveshare_esp32p4_pins.md) for both.

**Optional (>32 g high-g backstop), weight 1.0 g**

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

**Optional (aux navigation, non-priority — see Flight criticality), weight 7.4g**

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

# 3D model data

These numbers are coming from the printed TMS-7 models. 

|Component|Count|Weight [g]|
| ------------------- | --- | ---- |
|Booster Body Tube    |  1  |  55  |
|Engine Holder and Cap|  1  |  35  |
|Parachute            |  1  |   3  |
|F15-4 Engine         |  1  | 98.3 |
|Wing                 |  2  |  18  |
|Upper Body           |  1  |  10  |
|Fin                  |  3  |   7  |
|Front Lower Body     |  1  | 16.1 |
|Back Lower Body      |  1  | 29.5 |

So assembled booster will weigh 191.3 grams, the glider itself without electronics will weigh around 112.6 grams, the electronics are excpected to weigh around 100-150 grams.
Thus the assembled Coludo will be between 405-455 grams.

The F15-4 mass above (98.3 g) is measured. The alternative **E16** motor is lighter — ~57 g loaded
(~36 g propellant), published — so an E16 build is ~40 g lighter (≈365–415 g assembled). Both motors'
estimated flight envelope (peak accel / speed / apogee / glide range) is in
[`specs/coludo.md` → Flight envelope](../specs/coludo.md).

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
