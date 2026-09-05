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
| airspeed | **SDP810-500Pa** ([Sensirion](https://sensirion.com/products/catalog/SDP810-500Pa)) | I²C `0x25` | pitot/static ±500 Pa; the **direct** airspeed → fin governor (see *Airspeed* below) |
| GNSS | **ATGM336H** | UART 9600, 10 Hz | position; may lose lock under high-g |

**Interchangeable / additional options:**
[AHT20+BMP280](https://www.aliexpress.us/item/3256806546750874.html) (temp + baro),
[MPU6050](https://www.amazon.com/dp/B0BMY15TC4) (cheap IMU, noisy under high-g),
[VL53L0X / VL53L1X](https://www.aliexpress.us/item/3256807793059841.html) (alternate ToF for the `agl`
quantity), [Gravity 10DOF BMI323+BMM350+BMP581](https://www.dfrobot.com/product-3126.html) (better RAW
IMU/mag/baro data than the BNO055 but **no on-chip fusion** — see *The BNO055 successor path* below).
These provide the same quantities, so the fusion layer can use them as drop-in fallbacks.

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
| SDP810 airspeed | Important | **direct** pitot airspeed → the fin-authority cap (the estimate was the weakest signal). Degrades gracefully to the accel+GNSS estimate — the pre-pitot baseline flown in all HITL to date — if absent | ✔ (5) |
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
  still gives ~1 mg resolution at 1 g, so it is the lead `accel` for the airspeed integrator and
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

### The BNO055 successor path — BMI323 + BMM350 + BMP581 (10-DOF)

[Gravity 10DOF IMU](https://www.dfrobot.com/product-3126.html) ($19.90, I²C/UART) or the smaller
[Fermion SEN0697](https://wiki.dfrobot.com/sen0697/) carry **BMI323** (6-axis) + **BMM350** (mag) +
**BMP581** (baro). On raw data they beat the BNO055 on every axis that matters:

| | BNO055 (current) | BMI323 / BMM350 / BMP581 |
| --- | --- | --- |
| Accelerometer | **14-bit**, ±16 g | **16-bit**, ±16 g — **4× finer resolution** |
| Gyroscope | 16-bit, ±2000 °/s | 16-bit, ±2000 °/s, ±1 °/s zero-rate offset |
| Sample rate | **100 Hz** (fusion-capped) | **up to 6400 Hz** — no fusion cap |
| Magnetometer | ~0.3 µT resolution | **~0.1 µT**, 190 nT rms noise (X/Y) |
| Barometer | (BMP280 alongside) | **BMP581: 1/64 Pa resolution**, ±6 Pa relative, 240 Hz |
| Current | Cortex-M0+ fusion core, substantially more | **1.1–3.6 mA** |
| Orientation output | **fused on-chip** | **raw only** |

**"No on-chip fusion" sounds fatal, but for THIS architecture it mostly is not — we already fuse on the
MCU.** `tasks/attitude.py` is a complementary-filter AHRS running today: gyro integration plus an accel
gravity-vector re-anchor through the integer CORDIC `fixed.atan2_cd`, with a coordinated-turn gate so a
banked turn cannot roll the estimate flat. It is integer/fixnum throughout (the only boxed float is the
heading the channel format requires), runs at 50 Hz beside the 100 Hz flight loop, and is **flight-proven**:
in [TMS-7-attitude](sims/TMS-7-attitude/) the BNO055 is killed mid-glide and the backup flies to a
controlled landing (E16 **15 m in-zone**), tracking truth to **~1° roll / ~0.5° pitch** (`attitude_soak`).
So the question is not "can the P4 fuse?" — it demonstrably can — but "what would a magnetometer add,
and what does owning the fusion cost?".

**What the magnetometer actually adds.** The backup's yaw is *not* free-running: it already has an
absolute reference, a weak pull toward the **GNSS ground track**. Its two real weaknesses are narrow but
real — the course reference needs **motion** (`course_gate` ~5 m/s, so it is useless on the pad and at
low speed), and in a **crosswind crab the ground track is not the heading**, which the weak blend can
only average out. A magnetometer answers both: true heading, at rest, crab-free.

**The real cost is calibration, not the filter.** Rolling our own 9-DoF means owning **hard- and
soft-iron magnetometer calibration** — near a carbon airframe, servo currents and a booster — and that is
exactly the messy part the BNO055's black box hides. Budget that, not the AHRS math.

**So the incremental path is the attractive one:** add **just the BMM350** as an extra input to the
filter that already exists. One driver, no new fusion architecture, no change to the BNO055 primary,
and it directly attacks the BNO055 single-point-of-failure. Only if that proves out does replacing the
BNO055 outright become a real question.

Practical notes: the ±16 g accelerometer is the same ceiling as the BNO055's, so this **cannot** be the
primary boost accel either — **LSM6DSO32 (±32 g) stays** the lead `accel`. Prefer the **UART** variant if
adopted: `i2c:0` already carries five devices (BNO055, BMP280, ICP-10111, VL53L4CX, SDP810).

### RETIRED: two ICP-10111 on different buses. One ICP, on the front bus.

Briefly considered for v2.0 -- same-quality altitude either side of the bus split, and the parts are on
hand. **It does not work on a two-bus board, and the reason is the ICP's own recovery.**

The part **stalls**: an unclean reboot latches its digital core so the address still acks while every
command NAKs, and the only thing measured to clear it is an I2C **general-call reset** on its bus. That
recovery holds the bus for up to `_RESET_TRIES` x `_RESET_RETRY_MS` = **~120 ms**, spanning one
max-length conversion window, and resets every peer that honours the call.

Now put a second one on `i2c:0`. That bus carries the **attitude primary at 100 Hz**, so a latch-up
recovery would stall roughly a dozen attitude samples and reset the BMP280 and INA226 alongside --
importing a disruptive, bus-wide recovery onto the one bus that must stay quiet, in exchange for an
altitude backup that is already adequate.

And it cannot go anywhere else: `_ADDR` is `const(0x63)`, **fixed**, so two of them cannot share a bus,
and the ESP32-P4 offers exactly two usable I2C controllers (`I2C(2)` hard-crashes). Two ICPs therefore
*require* both buses, one of which must not have it. So this is retired rather than deferred -- a third
bus, or a part with a selectable address, would be needed to revisit it.

**What stays:** one ICP-10111 on `i2c:1` as the altitude primary, with the BMP280 backup on `i2c:0`.
The BMP280 rides on the sen0253 board with the BNO055 ("one board, two devices") so it costs nothing,
and being a *different* part it also guards the common-mode case that two identical ICPs would share.

`layout.detect()` keeps its tolerance for a second `0x63` anyway -- `0x63` counts as evidence only for
v1.0, and for nothing on `i2c:0` where it would be true either way. That is now defensive rather than
planned: if one is ever fitted, the detector loses no vote instead of silently cancelling one out.

**CORRECTION (2026-09-05): the parts actually bought are four Adafruit 4754 (BNO085), NOT SEN0697.**
An earlier note here recorded SEN0697; that was wrong and the analysis attached to it was answering a
question about hardware nobody owns. The SEN0697 comparison below stays as the write-up of an
alternative, and the DECISION that follows still stands for v1.0 — but the live option is now the
BNO085, which is a *different trade*, not a cheaper version of the same one.

**What changes, and it is the load-bearing part.** The reason this section rejected the SEN0697 was
that it is **raw only**, so adopting it means owning hard- and soft-iron magnetometer calibration next
to a carbon airframe, servo currents and a booster. **That objection does not apply to the BNO085**: it
fuses on-chip, so the calibration stays inside the vendor's black box exactly as the BNO055's does. The
cost moves instead to a new driver speaking **SHTP** -- packets, sequence numbers, channel
multiplexing, feature reports -- plus re-earning the calibration latch, NVS profile restore, stall
detector and peer re-arm the current driver carries.

So the two candidates are opposite trades rather than better and worse:

| | BNO085 (owned) | SEN0697 / 3126 |
|---|---|---|
| fusion | **on-chip (SH-2)** | raw only |
| magnetometer calibration | vendor's problem | **yours** |
| driver cost | **SHTP protocol + re-earned behaviours** | three simple register drivers |
| attitude path | fused output, as today | `attitude.py` promoted to primary |

**The recommendation for the BNO085 is unchanged and is stated in its own section below** ("Recommendation
on the BNO085 swap"): not yet, and what decides it is the FIFO-drained shock capture, because +/-8 g
against the BNO055's +/-16 g cannot be judged from simulation numbers that contain no ignition
transient, ejection shock or landing impact. Owning four of them removes the availability question and
nothing else.

**DECISION (2026-08-06): not adopted — we stay on BNO055.** With **5+ BNO055 on hand** the unit-count
blocker is gone, and that was the only pressing reason to move. Keeping the fused part also keeps the
magnetometer calibration problem inside Bosch's black box. Recorded here so the comparison does not have
to be redone; revisit only if a *new* need appears (a board with no BNO055, or a measured attitude
problem the backup cannot fix).

## Post-flight consolidation — measure first, then remove

The goal after the passive flights is a **minimal device count and a simpler PCB**. Every part below is
carried because of an *assumption*; the flights turn each into a measurement. Nothing is removed on
opinion — the point of flying both is to earn the right to delete one.

| Candidate | What decides it | Drop when | What removal buys |
| --- | --- | --- | --- |
| **ADXL375** (±200 g) | `flight_kpi` prints **peak \|a\|** for both accels + a KEEP/DROP verdict | LSM6DSO32 never approaches its ±32 g rail **and** the ADXL never exceeds 32 g | a SPI chip-select + an INT pin, ~1 g, board area — **and possibly the whole SPI bus (see below)** |
| **VL53L4CX** (AGL laser) | does it return valid AGL **outdoors in Florida sun**? (Phase 2/9 of `field_test.md`) | it is blind or noise-dominated in daylight | an I²C device, INT + XSHUT pins — but a **replacement landing trigger** is then required |
| **SDP810** (pitot) | does the pitot actually beat the accel+GNSS estimate? (`flight.csv` records both — the airspeed panel overlays them) | the fused estimate is no better than the pre-pitot baseline | an I²C device + the pitot plumbing |
| **BMP280** | is a second baro worth its I²C address? | ICP-10111 alone detects apogee cleanly | little physically — it **rides on the sen0253 board with the BNO055**, so it is nearly free to keep |
| **ATGM336H** (GNSS) | — | never: navigation and the zone logic depend on it | — |

**The big PCB simplification hides behind the ADXL375.** It shares `spi:1` with the LSM6DSO32. If the
measured boost never clips ±32 g and the ADXL goes, the LSM6DSO32 is the **only** device left on SPI —
at which point the question becomes whether it can move to `i2c:0` and let the **entire SPI bus
disappear** (SCK/MOSI/MISO + 2 chip-selects + an INT = up to **6 GPIOs** and a bus's worth of routing).
It sits on SPI today for clean high-rate reads, so that is a bench question — can `i2c:0` carry a 104 Hz
6-DoF alongside its existing devices without hurting the gyro `rate` the PID D-term depends on? Worth
answering deliberately, because it is the single largest layout win available.

Measure it with: `python3 tools/flight_kpi.py <label>:<capture>` — the first lines are the G envelope and
the high-g verdict. (In *sim* captures both accel streams carry the same synthetic value, so the verdict
is only meaningful on a **real** flight, where the ADXL is the only sensor that can read past 32 g.)

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

### LSM6DSO32 → SPI wiring — it has TWO clock pins and TWO data-out pins

The same label trap as the ADXL375 above, and worse: this breakout carries a **second, AUXILIARY**
interface alongside the primary one, so `SCL`/`SCX` and `DO`/`DO` both appear on the board.

```
  bottom row (PRIMARY -- use this one):   VIN  3Vo  GND  SCL  SDA  DO  CS  I1  I2
  top row    (AUXILIARY -- do NOT use):   SCX  SDX  CS   DO   GND
```

The auxiliary port is the sensor-hub / OIS interface for an external magnetometer. It is a **separate
peripheral**: clocking it does nothing for the primary bus.

| LSM6DSO32 pin | meaning | ESP32-P4 GPIO |
| --- | --- | --- |
| VIN *(bottom 1)* | power | 3V3 |
| GND *(top 5)* | ground | GND |
| **SCL** *(bottom 4)* | SPI clock (SCK) — **NOT `SCX`** | **48** |
| **SDA** *(bottom 5)* | SPI MOSI (SDI) | **47** |
| **DO** *(bottom 6)* | SPI MISO (SDO) — **NOT the top-row `DO`** | **46** |
| CS *(bottom 7)* | chip-select | **50** |
| I1 *(bottom 8)* | INT1 data-ready | **28** |

> ⚠️ **v0.1 GOT THIS WRONG, ON BOTH BUILT BOARDS — fix the netlist for v1.0.** The layout took the
> clock from `SCX` and the data-out from the top-row `DO`, i.e. both from the auxiliary side. MOSI,
> CS, INT1, VIN and GND were correct. Repaired on TMS-7C and TMS-7D with two jumpers from the MCU
> header to the primary pads; both then read `WHO_AM_I 0x6c` on the shipped mode-3 / 5 MHz bus.
>
> **Why it was so hard to see (2026-08-24/25).** With the clock on the auxiliary pin the part is
> silent on **SPI *and* I²C** — both need that same primary `SCL` — so it reads exactly like an
> absent or dead device, and no chip-select experiment helps. What broke it open was proving `CS` and
> `SDA` good *independently*: driving CS low removes the part from an I²C scan (so CS is wired), and
> I²C data flowing at all proves SDA. That left the clock as the only untested input.
>
> **Two diagnostics that DO NOT work here, both tried:** the I²C address strap is **latched at
> power-up**, not sampled live, so touching a wire to `DO`/SA0 cannot flip `0x6a`↔`0x6b` and cannot be
> used to find the pad — the ADXL375 proved it by holding `0x53` while its SDO demonstrably worked.
> And do not interleave `SoftI2C` and `SPI` on these pins in one script: it leaves them misconfigured
> and produces convincing nonsense (a false "only SPI mode 0 works", with the ADXL375 degrading to
> `0xff` in the same run as the tell).

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

## Airspeed (pitot/static)
The airspeed estimate was the weakest signal in the stack (an accel integrator nudged by GNSS **ground**
speed). A direct pitot/static reading fixes it. **Chosen: [Sensirion SDP810-500Pa](https://sensirion.com/products/catalog/SDP810-500Pa)**
differential-pressure sensor (thermal flow-through, ±500 Pa, I²C `0x25`, 3.3 V, tube barbs) — its range
suits the glide (q ≈ 135 Pa at 15 m/s, well inside ±500 Pa, ~29 m/s full scale) and its 0.1 Pa zero /
`<0.05 Pa/yr` drift beat the cheaper ADP2100/MS4525DO for a flight sensor. Bench-verified on hand (5 units);
scale factor 60 (Pa = raw/60), zero ~0.02 Pa. Driver `drivers/sdp810.py` (@viper CRC-8, fixnum dynamic
pressure, one airspeed float per read) fuses into the fin governor as the **direct** airspeed source ahead
of the accel+GNSS backbone; it rails past ±500 Pa (boost / a steep dive), where the governor drops back to
the accel backbone.

### Pins and tube polarity

Pins number from the one marked **"1"** on the package; the datasheet defines pin 1 = SCL.

| Pin | Name | Connect to |
| --- | --- | --- |
| **1** (marked ●) | **SCL** | I²C clock — GPIO **8** (`i2c:0`) |
| **2** | **VDD** | **3.3 V** |
| **3** | **GND** | ground |
| **4** | **SDA** | I²C data — GPIO **7** (`i2c:0`) |

```
  looking AT THE PIN SIDE (pins toward you):

     [4]    [3]    [2]    [1●]        ● = the "1" mark
     SDA    GND    VDD    SCL
      |                      |
     P+                     P−        measured -- see below
   (pitot)               (static)
```

**Anchor each tube to a PIN, never to "left" or "right".** Left/right swaps with which face you look
at, and that ambiguity produced a *wrong recorded result* here — see below. Stated pin-wise there is
nothing to get backwards:

- **P+** (total → the nose pitot) is the barb on the **SDA / pin-4 side**, i.e. **opposite the "1" mark**.
- **P−** (static → the interior bay) is the barb on the **SCL / pin-1 side**.

Measured 2026-08-14 on a unit that had already had **one barb cut off**, which is what makes it
conclusive: with a single tube left there is no left/right to confuse. Blowing it produced **eight
sustained plateaus pinned at the +546 Pa rail (116 railed samples, longest 4.2 s) and not one railed
negative sample**. Blowing raises the blown port, so that port is P+. The short (≤0.8 s, never railing)
negative dips in the trace each land immediately *before* a rail — the inhale before the blow, not the
port.

> ⚠️ **This reverses the 2026-07-26 bench note**, which recorded "right tube = P+, left tube = P−" and
> was repeated in `test/live_pitot.py` and `field_test.md`. That run read its *signs* correctly — the
> sensor was **upside-down**, so "left" then and "left" later were opposite barbs. Nothing was wrong
> with the instrument or the method; the label frame moved. Which is the whole reason this section
> names a pin and not a side. It also means the design intent holds as written: the **P− barb is the
> one to cut back to ~2 cm** for the interior bay, and P+ is the one that must reach the nose.

Plumbing it backwards is **not damaging** and not a calibration problem — the cell is differential and
its ports are symmetric, so it simply reads negative. Fix by swapping the tubes or negating in the
driver; no recalibration.

**Build convention for the remaining units — let the LENGTH carry the label.** Keep **P+ at full
length** and cut **P− to half**. After that the part is self-documenting: the long barb is the pitot,
and no viewpoint, silkscreen or memory is involved. Use the pin anchor above once, to decide which one
to cut; the length encodes it permanently for the rest of the build and for whoever opens the airframe
next.

Cut P− to *half*, not flush, deliberately — it stays long enough to push a tube onto, so a wrong call
is still recoverable by swapping the two lines instead of by scrapping the sensor. (Half also stays
within the "≥ 1 cm, ~2 cm clear of the camera board" the bay wants.)

The asymmetry is pneumatically harmless: unequal tube volume shifts the *response time* of each side by
milliseconds at these lengths, not the steady reading, and the airspeed the governor consumes is a
steady-flight quantity.

**Plumbing (integrated into the printed body, sensor fully inside — do NOT strip the calibrated
flow-through cap):**
- **P+ (total) → a forward-facing pitot** integrated into the body, parallel to the body bottom, on the
  frontal face **under the camera** → direct frontal air = total pressure. A **3.8 mm** printed channel
  mates the SDP810 P+ stub (dead-ended, so the bore is a mechanical fit only). Keep it **airtight** (resin
  print or a seal coat; silicone sleeve on the barb) — at 135 Pa a pinhole biases the reading directly —
  with a **weep / no low point** so rain or dirt can't block the line, and the mouth clear of the camera
  bump's wake.
- **P− (static) → the vented interior bay**, ≥ 1 cm (cut back to ~2 cm) clear of the camera board. The
  camera is angled ~25° up/down (not a forward ram-inlet), so it cannot pressurize the bay → interior ≈
  ambient static.

**Calibration (two knobs, `airspeed_sdp810` config):** `zero_offset_pa` — a **pad tare** (CC
`update {"zero": true}` with the glider still) cancels the interior-static pressure bias; `air_density` —
the single q→v knob (it absorbs the position-span error), default **1.18 kg/m³** (~25 °C sea level for
Florida, not ISA 15 °C's 1.225) and **trimmed on a calm-day pass** so the fused airspeed matches GNSS
ground speed in still air.

**Required (airspeed), weight ~5g (sensor + tube); needs the calm-pass air_density trim before trusting.**

## Battery
Not many options for [5V USB-C low-weight](https://www.amazon.com/dp/B07SZKNST4) power delivery are avaialbe.
Alternative is to connect e.g. from [6F22 9V using plug](https://www.amazon.com/dp/B083QFFH66) and a lightweight power-down module or a LiPo 3.7 V single cell battery and boost up circuit for controller and a separate circuit for the servos.

**Required, weight 42.1g for 6F22, and power-down board**

## Converter
The servo rail is driven by a **ND3A05SD DC-DC module (5 V / 3 A, isolated)**, separate from the
controller rail. **MEASURED (2026-07-25, INA226 then on the servo rail @ ~100 Hz, MG90S yaw, 10 × full
0↔180° at max slew):** **peak 3.9 W = 0.79 A**, mean during travel ~1.4 W, ~625 mJ per 180° sweep,
41 mW holding. So three moving together is **~2.4 A peak — inside the 3 A module**, not the ~3.5 A the
earlier ~1.2 A/servo estimate suggested. (A USB power meter reads only ~2 W here: it updates at a few Hz
and smooths, so it sees roughly the mean plus board baseline, never the sub-100 ms spike. Size the
**capacitor from the peak** and the **converter from the mean**.) The rail still carries a **reservoir
capacitor** to source the transient — the decided, primary protection — but it is headroom, not a rescue.

The module maker also suggests a series **diode** (an isolated buck cannot sink current, so a
back-driven motor's regenerative kick pushes the floating output up until something clamps it). **That
advice targets a single larger motor; three small MG90S regenerate far less, and the bulk cap already
doubles as the regen sink** (ΔV = Q/C is tiny for a small kick into 1000 µF). So the series diode is
**omitted** — it would cost ~0.4 V drop / ~1.4 W / reduced fin torque for protection the cap already
provides. A cheap **TVS clamp** across the rail is the optional belt-and-suspenders (≈ 0 loss when idle)
if any overvoltage is still a worry.

```
  battery  ──▶  ND3A05SD  ──▶ (D1 — omitted) ──┬──────────▶  3× MG90S servos
  6F22/LiPo     5 V / 3 A                       │            0.79 A each (measured)
                (isolated)                       │            ~2.4 A if all three
                                      ┌──────────┴─────────┐
                                      │ Cbulk 1000 µF      │  reservoir (decided):
                                      │  ‖ 100 nF          │  sources the spike AND
                                      │  ‖ TVS 5 V (opt.)  │  absorbs the small regen
                                      └──────────┬─────────┘
                                                GND
  module 0 V ──────────── bond to system GND ───────────────▶ (shared PWM reference)
```

**Reservoir capacitor — the decided protection.** **TWO capacitors (simplified 2026-08-14 from three):
a 1000 µF bulk + a 100 nF ceramic**, placed **right at the servo header** so the spike path
(cap → servos) carries no series impedance. Keep the module's own output cap as well. 470 µF is the
minimum; 2200 µF is fine but watch the power-on inrush tripping the module soft-start.

The **1000 µF is the one to keep, not 470 µF**, on two grounds. Droop while the converter's loop catches
up is ΔV = I·Δt/C, so at the measured 2.4 A the bulk sets the margin: over a 100 µs window 1000 µF droops
0.24 V against 470 µF's 0.51 V, and the MG90S sit near their limit around 4.2 V — the same headroom
argument that rejects two series diodes. More importantly the **series diode is omitted *because* the
bulk cap is the regen sink** ("ΔV = Q/C is tiny for a small kick into 1000 µF"): halving the reservoir
doubles that kick and quietly erodes the reasoning that justified leaving the diode out. The two
decisions are coupled — do not shrink the bulk without revisiting the diode.

**The dropped part is the 10 µF X7R**, and it is the right one to lose: the 1000 µF covers the servo
transient and the 100 nF the high-frequency bypass, which are the two jobs that matter here. It is not
free, though — the 10 µF bridged the band between them, roughly 100 kHz–1 MHz, which is exactly where
the ND3A05SD switches. The module keeps its own output cap, local to its own ripple, which covers most
of it; the residual mid-band notch is real but not flight-critical.

**What the bulk part must be: LOW-ESR. That matters far more than aluminium-vs-polymer.** Two separate
things happen when the servos snatch 2.4 A — the droop over time is I·Δt/C (capacitance), but the
INSTANTANEOUS step is I × ESR and lands the moment current flows:

| 1000 µF type | typical ESR @100 kHz | instant drop at 2.4 A |
| --- | --- | --- |
| general-purpose aluminium | ~0.15 Ω | **0.36 V** |
| **low-ESR aluminium (the baseline)** | ~0.04 Ω | 0.10 V |
| polymer (optional upgrade) | ~0.015 Ω | 0.04 V |

Against a ~0.6–0.8 V budget a general-purpose part spends half of it before the capacitance does any
work, so the gap general-purpose→low-ESR dwarfs the gap low-ESR→polymer. A **ripple-current rating
≥ ~1 A at 100 kHz** is the reliable way to tell a low-ESR part from a generic one. Polymer is worth it
for a flight article for its *mechanical* properties — no electrolyte to dry out, ESR nearly flat when
cold, better under vibration — not because the electrical margin demands it. **If the only 1000 µF to
hand is general-purpose, two 470 µF in parallel beat it** (~940 µF at half the ESR), at the cost of the
part count this simplification just bought back.

Bond the module output − to system ground so the servo PWM shares the logic reference.

**If a diode is ever wanted** (a bigger single motor, or reverse-polarity protection): use a **Schottky**,
V_F as low as possible, **I_F ≥ 8 A**, **V_RRM ≥ 20 V**, on the **+ rail only** — never one in each line
(a diode in the 0 V line lifts the servo ground and shifts the PWM reference, and two series diodes drop
the servos to ~4.2 V, near the MG90S limit). For reverse-polarity specifically an **ideal-diode P-FET**
(≈ milliohm drop) beats a Schottky.

**Firmware helps too.** The fin `concurrency` gate staggers servo motion so they do not all slam at
once — that caps the *simultaneity* of the draw; the reservoir cap caps the *transient*. A sustained
all-three-stall > 3 A is an average-power limit the cap cannot fix (the module current-limits), so it is
handled by not commanding three hardovers at once, not by more capacitance.

**Required, weight ~5–7 g (module + reservoir cap; optional TVS).**

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

**Power**: servos run from their **own converter rail** (5 V — see [Converter](#converter) above),
separate from the controller. The measured ~2.4 A peak of 3× MG90S is handled by that rail's **reservoir
capacitor**; a series diode is not used (small servos → low back-EMF, the cap absorbs it). The firmware
fin `concurrency` gate staggers servo motion so they do not all draw at once.

**Required, weight 10.6g per each engine and wires, at least 2 are required**

**Torque / hinge moment.** The SG90 is **direct-driven** (1:1, no gearing) and at 5 V (1.3–1.5 kg·cm)
runs the *governed* fin at ~3–4 % of stall — **~25–30× torque margin, no gearing needed**. The only
stress case is an un-governed ±45° hardover near burnout (servo back-drives at ~60–90 m/s), where the
output-shaft **bending** load is the limit, not torque — the sole reason to prefer the metal-gear
**MG90S** (~+3 g). Derivation: [`../doc/specs/coludo.md`](../doc/specs/coludo.md) → "Fin authority → Servo torque".

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

## Printed parts longevity

The **engine holder is a consumable** — it takes the motor's heat plus the clamp/thrust stress on
every burn (the 7/03 static burn confirmed it as the wear item), so print spares per flight campaign.
All other printed parts (body tubes, wings, fins, cap) are **long-living**: they see only aerodynamic
and handling loads and survive many flights unless crashed.

# 3D model data

The detailed numbers are avaliable in the [TMS-7 readme.md file](../models/TMS-7/readme.md).

The F15-4 mass above (98.8 g) is measured. The alternative **E16** motor is lighter — ~82.5 g loaded. Both motors'
estimated flight envelope (peak accel / speed / apogee / glide range) is in
[`doc/specs/coludo.md` → Flight envelope](../doc/specs/coludo.md).


# Main board v1.0 — the target layout

The board to be ordered. Two I2C clusters split **on-board vs front-panel**, the LSM6DSO32 alone on SPI,
GNSS and power merged onto the main board, no ADXL375.

## Why v1.0 is worth building: it is a MASS change, not an accuracy change

The board's value does not arrive through control behaviour, and measuring it that way asks the wrong
question. It arrives through **grams**, and [`TMS-7-preflight`](sims/TMS-7-preflight/) already
quantified exactly what grams are worth on this airframe:

| combo | glide mass | median miss | in zone |
|---|---|---|---|
| `f15_full` | 285 g | 88-91 m | **0 / 10** |
| `f15_half` | 235 g | **37-46 m** | **7 / 10** |

**50 g bought 45 m of median miss and seven in-zone landings** -- about 0.9 m per gram, over three runs
each, with the in-zone set reproducing exactly. Nothing else measured on this project moves accuracy
that far.

TMS-7D's glider is **287.5 g**, which is `f15_full` almost exactly. The v1.0 savings -- a shorter nose
(7D runs 10 cm longer than 7C), the deleted power and GNSS harnesses, and the connectors that go with
them -- are expected at **30-50 g**, putting the airframe at **237-257 g**. The optimistic end lands on
`f15_half`'s mass, the configuration that put 7 of 10 in the zone.

**So the flight-validation result should be read accordingly.** `TMS-7-board_v1.0_validation` found the
rewire behaviour-neutral to within the endgame's own phase variance, and **that is the desired
outcome, not a disappointment**: the board is meant to fly the SAME while weighing less. A layout
change that altered control behaviour would be a reason to investigate, not to celebrate. The accuracy
comes from the scale, and the scale is measured.

## v1.0 allocation — DECIDED

Two clusters split **on-board vs front-panel**, which is simultaneously the fast/slow split, the
kept/expendable split, and the short-trace/long-harness split. One line does all four jobs.

**i2c:0 — ON-BOARD, 400 kHz**, SDA **7** / SCL **8**

| device | addr | rate | note |
|---|---|---|---|
| BNO055 | `0x28` | **100 Hz** | attitude primary; the only device needing the fast bus |
| BMP280 | `0x76` | — | altitude BACKUP — now out of the ICP's general-call blast radius |
| INA226 | `0x40` | 10 Hz | moved from i2c:1 so that bus is purely front-panel |

**i2c:1 — FRONT PANEL, 100 kHz**, SDA **31** / SCL **30**

| device | addr | rate | note |
|---|---|---|---|
| ICP-10111 | `0x63` | 50 Hz | altitude PRIMARY; physically nearest the front |
| SDP810 | `0x25` | 50 Hz | nose, pitot tubing |
| VL53L4CX | `0x29` | 20 Hz | downward-facing, AGL |

**The 100 kHz is the point, not a compromise.** Nothing on this bus exceeds 50 Hz, so a quarter of the
clock costs nothing — three devices at 50/50/20 Hz sit under 20 % utilisation — while slower edges buy
real margin on a long, unterminated harness. The bus that leaves the board runs at the speed the
harness wants, not the speed the silicon allows.

**spi:1 — SCK 48 / MOSI 47 / MISO 46** — LSM6DSO32 alone (cs **50**, int1 **28**). Kept as a separate
bus family so the attitude backup does not share a failure domain with the BNO055.

**UART** — `uart:1` TX **20** @921600 (recorder); `uart:2` TX **22** / RX **23** (GNSS, on-board).

**Discretes** — `ina226_alert` **29**, `separation_switch` **33**, servos yaw **26** / eleron_left **27**
/ eleron_right **32**.

### What this arrangement buys

* **Altitude redundancy is finally split across failure domains.** Primary (`icp10111`, i2c:1) and
  backup (`bmp280`, i2c:0) can no longer be lost together — the one redundancy gap this file has
  flagged since the Gerber review. Whichever bus dies, an altitude source survives.
* **The BMP280 leaves the general-call blast radius.** The ICP's latch-up recovery resets peers that
  honour it; with the backup on the other bus, recovering the primary can no longer disturb it.
* **The harness is one lane to one connector** — no local stub, one pull-up pair, sized for one length.
* **The front bus can be slowed** without costing any device its rate.

### What it does NOT fix

`icp10111` and `airspeed_sdp810` still share a bus, so the ICP's general call still reaches the pitot —
which today has no `rearm()` and is started once in `setup()`. The layout moves that exposure; it does
not remove it. Tracked separately as a firmware fix, not a board one.

## Running one firmware on both boards

The two layouts are distinguishable **by scan alone** — no strapping resistor, no stored flag, nothing
an operator has to set correctly:

| bus | v0.1 answers | v1.0 answers |
|---|---|---|
| i2c:0 | `0x28` `0x63` `0x76` `0x25` `0x29` | `0x28` `0x76` `0x40` |
| i2c:1 | `0x40` | `0x63` `0x25` `0x29` |

Every marker address moves except `0x28` (BNO055) and `0x76` (BMP280), which stay on i2c:0 in both — so
the discriminators are `0x63`, `0x25`, `0x29` (i2c:0 → i2c:1) and `0x40` (i2c:1 → i2c:0). Four
independent votes, which is what makes the detection robust to a single dead device rather than hinging
on one probe.

Detection runs before device setup, scans both buses at the lower (100 kHz) rate that every part
tolerates, scores each layout by how many of its expected addresses appear on the expected bus, and
applies the winner's bus assignments and speeds. An explicit `board.layout` of `v0.1` or `v1.0` in the
config always wins over the scan; `auto` (the default) detects. An ambiguous or failed scan changes
nothing and says so loudly — the config as written is the fallback, never a guess.

## REQUIRED on v1.0: a Schottky on the USB 5 V feed

**The problem the merge creates.** With the power island on the main board, plugging USB into the MCU
while the battery or bench simulator is connected puts **two sources on one 5 V net**. That is not
merely untidy: the ND3A05SD is an **isolated buck and cannot sink current**, so whichever source is
higher drives backwards into the other -- into the converter's output, or out of the board into the
host's USB port. Both are out of spec, and the converter's own maker warns about the first.

**The fix: one Schottky in series with the USB VBUS feed.** One is enough, and it gives automatic
changeover with the island preferred: the net sits at `max(island, USB - Vf)`, so with the island at
5.0 V and USB at ~4.68 V no current leaves the USB side at all, and if the battery sags below that USB
takes over by itself. A second diode on the island side buys nothing.

**Part: 1N5817** (of the 1N5817/18/19 family on hand), or **SS12/SS14 (SMA)** / **PMEG2020** in surface
mount for the PCB.

| part | reverse | Vf @ 1 A (max) | Vf @ ~0.7 A (typ) |
|---|---|---|---|
| **1N5817** | **20 V** | **0.45 V** | **~0.32 V** |
| 1N5818 | 30 V | 0.55 V | ~0.40 V |
| 1N5819 | 40 V | 0.60 V | ~0.45 V |

Take the LOWEST reverse rating available. Within a Schottky family a higher reverse rating costs
forward drop, and the reverse stress here never exceeds ~5 V -- so 20 V is already 4x margin while 30 V
and 40 V each cost ~0.1 V of headroom for nothing. That drop is the entire design constraint: it comes
straight off what the 3V3 regulator sees. Current is undemanding (the stated budget is **0.7 A**, and
the servos are on the island's separate rail, not through USB), so a 1 A part is adequate and the
choice is about Vf alone.

**Orientation: STRIPE (cathode) toward the board's 5 V pin.**

```
   USB VBUS --|>|-- board 5 V
            anode  cathode = stripe
```

Dissipation is ~0.7 A x 0.32 V = **0.23 W**, comfortable in a DO-41 -- but do not crop the leads flush,
they are the heatsink.

### Check these two before soldering

* **Does the board already diode VBUS?** Many dev boards do. Probe VBUS at the USB connector against
  the board's 5 V / `VSYS` pin with USB plugged in: **~0.3 V of difference means a Schottky is already
  there**, and adding a second leaves only ~4.35 V into the regulator. **`VSYS` is exactly the pin
  where this is decided** -- on most dev boards it is the system rail, VBUS-after-diode OR'd with an
  external input, and if the WaveShare follows that convention then feeding the island into `VSYS` IS
  the designed dual-supply path and no extra part is needed. That is **not documented anywhere in this
  repo**, so measure it rather than assume it.
* **What is the 3V3 regulator's dropout?** With one diode, USB-only running gives the board ~4.68 V
  instead of 5.0. Most parts are happy down to 4.5 V, but this is the number that would fail in the
  field, on USB, with no battery -- confirm it rather than inherit it.

**Lower-drop alternative if that headroom comes out tight:** a P-FET ideal diode using the **AO3401A**
already stocked for the deferred switched rail -- source to the supply, drain to the load, gate pulled
toward the load. It conducts through the body diode and then enhances, giving tens of millivolts
instead of 300. More subtle at crossover, so only worth it if the dropout check demands it.

## GNSS: chip on the main board, antenna outboard with the recorder

That split is the right one, and it weakens the noise caveat above rather than triggering it. An active
antenna carries its own LNA, so the low-noise amplification — the stage that actually sets sensitivity —
happens outboard, away from the switching stage and the servo currents. What remains on the main board
is the receiver's correlator and ADC, far less sensitive to a noisy neighbour than a front-end would be.

Two things still worth doing:

* give the GNSS chip's supply its own ferrite/LC-filtered branch rather than a bare tap off the main
  rail — conducted noise reaches the chip regardless of where the antenna sits;
* keep the RF trace from chip to antenna connector short and away from the switching node, since that
  run is now the one unshielded RF path on a board that also carries ~4 A servo transients.

Placing the antenna board with the recorder is also convenient for the power topology: those two are
already the pair that sit on their own rail today.

## Why it is arranged this way

The reasoning behind each decision, kept because the alternatives are not obviously wrong and will be
proposed again otherwise.

### 1. Merge main + power onto one board — APPROVE

Already the v1.0 direction. It deletes the inter-board harness and, with it, the two hand-jumpers per
board that repair the v0.1 netlist error. Fewer connectors at the extremities is the single most
reliable simplification available on this airframe.

The one thing to design for is that a switching power stage now shares a substrate with the I2C bus and
the IMU: keep the energy island partitioned, with its own ground pour region and a single-point tie, so
servo transients (~4 A when three fins slew) do not appear as ground bounce under the sensors.

### 2. Merge GNSS onto the main board — APPROVE, with one caveat

Cuts a power run and a UART run, both worth having. Two notes:

* The receiver front-end is RF-sensitive and will now sit beside the switching stage and the servo
  currents. Separating the antenna (as planned) handles radiated coupling but not **conducted** noise on
  the receiver supply — give the GNSS its own ferrite/LC-filtered branch off the main rail rather than a
  bare tap.
* This retires a standing diagnostic trap. Today GNSS sits on the recorder power cluster, so it probes
  DEAD over USB and that has already cost one real investigation. After the merge that asymmetry
  disappears, which is a genuine debugging improvement, not just tidiness.

### 3. Separate I2C cluster for SDP810 + VL53L4CX — STRONGLY APPROVE

The best of the proposals, and it should be stated as a reliability change rather than a layout one.

Those two are precisely the devices at the airframe **extremities** — the pitot in the nose with its
tubing, the laser pointing down — so they carry the long harness runs, the added capacitance and the
connector count. Every other I2C device is on-board with short traces. Splitting on that boundary means
a fault, a stretched harness or a marginal connector on the long runs cannot take down the BNO055 and
the baros, which are flight-critical. That is the concrete answer to the standing "single I2C bus is a
single point of failure for every sensor" finding.

It also uses the silicon budget exactly: the ESP32-P4 exposes **two** usable HW I2C controllers, and
`I2C(2)` hard-crashes the board. Two clusters is the maximum, so the only question was where to cut, and
internal-vs-external is the right seam.

### 4. Drop the ADXL375 — APPROVE

Supported by measurement, not preference. Peak acceleration across the board HITL matrix is **3.3 / 3.7 /
3.8 / 4.3 g**, and `flight_kpi` prints a KEEP/DROP verdict per capture that reads DROP. The part's only
edge over the LSM6DSO32 is surviving **>32 g** without clipping, and nothing in the measured envelope
approaches a quarter of that. It costs a chip-select, an interrupt line and board area for a case that
has not occurred.

### 5. Drop the LSM6DSO32 — KEEP IT, AND KEEP IT ON SPI

Answering the direct question "do we absolutely need it?": **not for function — for independence.**

#### What the LSM6DSO32 is the primary of, and what backs each channel up

| channel | providers **on v1.0** (by priority) | if the LSM6DSO32 goes |
|---|---|---|
| `accel` | **lsm6dso32 p0**, **bno055 p2** | falls to the BNO055 automatically -- already configured, 40 ms window. **Backed up**, but by a single fallback: the ADXL375 that held p1 on v0.1 is not fitted here, so this row is one deep rather than two. |
| `rate` (gyro) | **lsm6dso32 p0 — sole provider** | **disappears.** No other device publishes it. |
| `attitude` | bno055 p0 (chip fusion), `attitude` task p1 (computed) | drops to **one** provider -- the p1 backup is computed FROM `accel` + `rate`, so it dies with the gyro |

Who actually consumes them: `rate` feeds the PID D term (`flight.py`) and the attitude backup filter
(`attitude.py`). `accel` feeds launch detect (`sequencer`), the airspeed backbone (`governor`),
auto-arm (`field`) and that same backup filter.

So removing it costs two things, and only one is serious:

* **PID D term** -- degrades gracefully. `pid.step()` takes `rate=None` and falls back to
  derivative-on-error, a documented mode. Worse damping, not a failure.
* **The attitude backup disappears entirely** -- and that backup exists for one specific, MEASURED
  failure: the BNO055's fusion core stalls while its channel stays FRESH, returning a bit-identical
  Euler triple indefinitely while raw accel and gyro keep streaming, across a power cycle, in both
  fusion modes, on either clock. The driver detects it (using the part's own gyro, only while
  rotating, since a still part legitimately repeats). Detection without a fallback means the glider
  would know its attitude is dead and have nothing to fly on.

**A gyro published from the BNO055 does not fix this.** The part already reads its gyro in the same
24-byte block and could publish `rate` cheaply -- that would restore the D term -- but an attitude
backup computed from the same chip that just froze is not redundancy. The same applies to a BNO085
successor: a better part is still one part.

#### And keep it on SPI -- correcting an earlier recommendation here

An earlier draft of this section proposed moving the LSM6DSO32 to the internal I2C cluster to free six
GPIOs and retire the v0.1 SPI wiring fault. **That was wrong, and the reason is two sections below:**
attitude is deliberately isolated ACROSS BUS FAMILIES -- primary on I2C, backup fed by the SPI gyro --
so a bus-level I2C fault cannot take both attitude paths at once. Moving the gyro to I2C collapses that
onto one family, and this board measures **0.50 % of ICP-10111 frames arriving corrupted on I2C**
(single-bit flips, bench, idle). Trading a measured-noisy single bus for six pins is the wrong side of
that trade.

The v1.0 two-cluster split cannot restore the isolation either: the only other I2C controller is the
EXTERNAL cluster, and putting the primary accel/gyro on the long harness is worse than either option.
Two usable controllers is the hard ceiling (`I2C(2)` hard-crashes the P4).

So the SPI bus stays, carrying one device instead of two. The v0.1 wiring fault it enabled is a
netlist error to fix on the new board, not a reason to delete the bus.

#### Deferred: a power-managed i2c:1 cluster

Not in v1.0 — it adds a power stage to a board revision that is otherwise a netlist change, and the
hardware delta is already large enough. Recorded because it is the right answer once the board settles,
and because it is strictly better than the XSHUT line it would replace.

**The idea.** Gate the 3V3 feeding the i2c:1 segment with a high-side P-FET (an **AO3401A** is on hand
and is over-specified for the ~30 mA load — tens of milliohms, so single-digit millivolts of drop).
Source to always-on 3V3, drain to the segment, gate to a GPIO. No level shifter, because the switched
rail is the same voltage as the GPIO: 3.3 V high gives Vgs = 0 (off), low gives Vgs = -3.3 V (hard on).
That equivalence stops holding if the segment is ever fed from another rail.

**Why it beats a per-sensor reset line.** It recovers BOTH external devices, where XSHUT reset only the
laser and a wedged SDP810 was never recoverable at all. And it can ISOLATE: a device holding SDA low
through a wiring fault keeps holding it after a reset, whereas removing power removes it from the bus, so
a failed harness can be cut and the flight continued. Losing the segment is survivable by design already
in the code — `agl` and `airspeed` are single-provider, but both fall back onto i2c:0, which is never
cut: the landing detector drops to `elevation` when `agl` has no source, and the governor's backbone is
the accel+GNSS estimator with the pitot as a corrector.

**What it would need, none of which exists yet.**

* **Gate pull-up to SOURCE (10k-100k), not optional.** GPIOs are high-impedance through reset and early
  boot; a floating gate leaves the rail undefined — it can come up, half come up, or oscillate. The
  pull-up gives one defined power-up state, off, until firmware deliberately pulls the gate low.
* **~100 R gate series** to bound the gate-charge spike, and **~100 nF gate-to-source** as the soft
  start: with the pull-up that is an RC of about a millisecond, slewing turn-on so two sensor boards'
  decoupling inrush does not land hard on the MCU's own rail. A recovery that brown-outs the MCU is not
  a recovery. Linear-region dissipation during the slew stays under 100 mW at 3.3 V / 30 mA.
* **i2c:1 pull-ups on the DRAIN (switched) side**, so bus and devices lose power together. Left on the
  always-on rail, the unpowered devices' ESD diodes clamp SDA/SCL and the segment stays stuck with the
  rail down.
* **INA226 would have to leave i2c:1**, because everything on a switched segment is switchable and it is
  the instrument you most want alive while cutting a suspect harness. With constant 3V3 that constraint
  does not apply, which is why it stays put in the v1.0 allocation above.
* **Firmware: a re-entrant `setup()`, not a reset.** After a power cycle the VL53L4CX needs its
  configuration and VHV calibration rewritten and the SDP810 its measurement command re-issued.
  `setup()` runs once at bring-up today. Plus a policy that cuts permanently instead of oscillating, and
  a budget of roughly **0.3-0.5 s** to detect and re-init — about a metre of altitude at trim sink.

> ⚠️ If this is ever built on the pin `laser_xshut` used to occupy, note the sense **inverts**: that pin
> was active-low SHUTDOWN, while a P-FET enable is active-low POWER-ON. Same wire, same idle level,
> opposite meaning — name it `i2c1_power`, never reuse the old name.

# Transition v0.1 → v1.0 — the bench worklist

What to change on an existing v0.1 board to make it a v1.0 board. Nothing here needs the new PCB: the
breadboard can be rewired to this and the firmware will detect it.

## Pin delta v0.1 → v1.0 — what actually changes

**No pin is renumbered, and no bus pin moves.** Only the rows below.

**REMOVED — four GPIOs freed**

| net | GPIO | why |
|---|---|---|
| `adxl375_cs` | **49** | ADXL375 not fitted |
| `adxl375_int` | **4** | ditto |
| `laser_int` | **3** | poll fallback is the same code path at 20 Hz |
| `laser_xshut` | **5** | reset at `setup()` only; a board power cycle clears a wedged laser between flights |

**CHANGED — bus membership, at the same physical pins**

| device | v0.1 | v1.0 |
|---|---|---|
| ICP-10111 | i2c:0 | **i2c:1** |
| SDP810 | i2c:0 | **i2c:1** |
| VL53L4CX | i2c:0 | **i2c:1** |
| INA226 | i2c:1 | **i2c:0** |

**CHANGED — bus speed**

| bus | v0.1 | v1.0 |
|---|---|---|
| i2c:1 | 400 kHz | **100 kHz** (front harness) |

**UNCHANGED — no re-check needed**

`i2c:0` sda **7** / scl **8** · `i2c:1` sda **31** / scl **30** · `spi:1` sck **48** / mosi **47** /
miso **46** · `lsm6dso32_cs` **50** · `lsm6dso32_int1` **28** · `uart:1` tx **20** · `uart:2` tx **22** /
rx **23** · servos **26** / **27** / **32** · `separation_switch` **33** · `ina226_alert` **29** ·
BNO055 and BMP280 stay on i2c:0.

#### The front harness: 8 wires to 5

The real constraint is the CONNECTOR, not the wire count. The front lane carries **8** today:

| today (8) | v1.0 (5) |
|---|---|
| recorder UART x1 | recorder UART x1 |
| GNSS UART x2 | — *(GNSS chip moves to the main board)* |
| pitot + laser: INT, XSHUT, SDA, SCL, 3V3, GND | SDA, SCL, 3V3, GND x4 |

**8 -> 5 frees enough edge for a USB connector on the same face**, so the recorder can be serviced or
swapped without opening the airframe. That is worth more than any single signal on the lane.

Both laser control lines go, and for different reasons.

**`laser_int` (GPIO 3) — dropped, no functional loss.** `_setup_interrupt()` returns early when no
`int_pin` is declared, and the run loop waits on `_ready.wait(period_ms)`, which covers interrupt and
timeout in ONE path with no branch. At `period_ms` **50** (20 Hz) against a **100 ms** `agl` window,
polling carries 2x margin. The only casualty is the `irq_runs` column, whose job is detecting a dead
interrupt wire — a diagnostic that exists because the wire does.

**`laser_xshut` (GPIO 5) — dropped, and it costs less than it appears.** `_reset()` pulses it at
`setup()` **and nowhere else**: there is no in-flight recovery path for this sensor, no strike counter
and no escalation, unlike the ICP-10111. So the wire buys a forced reset at bring-up only. Two things
survive without it:

* **false-present detection still works.** The XSHUT pulse is conditional on the pin existing, but the
  FIRMWARE__SYSTEM_STATUS boot poll inside `_reset()` runs regardless — so `setup()` still refuses a
  dead-firmware sensor that ACKs its hard-silicon model id.
* **a wedged laser is still recoverable between flights**, by the board power cycle the operator does
  anyway. What is lost is only the ability to do it without one, in a situation where no code would
  attempt it.

Declare the laser with no `int_pin` and no `xshut_pin`; the driver already treats both as optional.

## ADXL375 — the one software delta between v0.1 and v1.0

With the LSM6DSO32 staying on SPI, the ADXL375 is the only device difference between the boards, and
`accel` simply loses its priority-1 provider: `lsm6dso32` p0 stays primary, `bno055` p2 stays the
fallback. Priorities only order the providers, so a gap in the numbering costs nothing.

Declare it per-airframe with the existing `absent` mechanism rather than letting setup fail and calling
it detection. A device that is not fitted and a device that is broken should not look the same in
`verify`: `absent` is silent and intentional, while a missing part left enabled burns `setup_retries`
attempts and then reports a failure that an operator has to learn to ignore — which is how a real fault
gets ignored too.

So: keep the firmware default `enabled` (the default config describes a fully-populated board), and add
`accel_adxl375` to the `absent` tuple of the v1.0 profiles only. TMS-7C and TMS-7D keep it active with
no config change at all.

# Main board v0.1 — as built

The two boards flying today (TMS-7C, TMS-7D) and the breadboard, until the rewire above is done.
Generated from `launches/20261003/TMS-7D/tms7d.config`, which is the authority — this table is a
convenience, not a second source of truth.

**i2c:0 — 400 kHz**, SDA **7** / SCL **8**

| device | addr |
|---|---|
| airspeed_sdp810 | `0x25` |
| baro_bmp280 | `0x76` |
| baro_icp10111 | `0x63` |
| imu_bno055 | `0x28` |
| laser_agl | `0x29` |

**i2c:1 — 400 kHz**, SDA **31** / SCL **30**

| device | addr |
|---|---|
| power_ina226 | `0x40` |

**spi:1 — 5 MHz**, SCK **48** / MOSI **47** / MISO **46**

| device | chip select |
|---|---|
| accel_adxl375 | `49` |
| imu_lsm6dso32 | `50` |

**pins**

| net | GPIO |
|---|---|
| `adxl375_cs` | **49** |
| `adxl375_int` | **4** |
| `ina226_alert` | **29** |
| `laser_int` | **3** |
| `laser_xshut` | **5** |
| `lsm6dso32_cs` | **50** |
| `lsm6dso32_int1` | **28** |
| `separation_switch` | **33** |
| `servo_eleron_left` | **27** |
| `servo_eleron_right` | **32** |
| `servo_yaw` | **26** |

Two things about this layout that the v1.0 arrangement exists to fix:

* **Five devices share `i2c:0`**, including both baros and the two long harness runs. That bus measures
  **0.50 % corrupted ICP-10111 frames**, and the altitude primary and backup are on it together, so one
  stuck slave takes both.
* **Both IMU-adjacent SPI parts need the two-jumper rework.** The v0.1 netlist takes the LSM6DSO32's
  clock from the module's AUX row, which silences the part on SPI *and* I2C and reads exactly like a
  dead chip. Every built board carries hand-jumpers for it.

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

## v1.0 board — hardware TODO

- [ ] **LSM6DSO32: move SCK to the primary `SCL` and MISO to the primary `DO`.** v0.1 routes both to
      the module's AUXILIARY row (`SCX`, top-row `DO`), so the part is unreachable on SPI and I²C
      alike — confirmed on both built boards and jumper-repaired on each. Full detail and the
      diagnosis in *LSM6DSO32 → SPI wiring* above. This is the single highest-value netlist fix:
      without it every board needs the same two-wire rework.

Found on the v0.1 hand-wired board (bench, 2026-07-26). **v0.1 PCB is already ordered**, so these are
carried to v1.0. Each has a software mitigation on the branch, so none blocks flying v0.1 — but the
mitigations are degradations, not equivalents.

| # | Issue | Evidence | v1.0 action | Software mitigation today |
|---|---|---|---|---|
| 1 | **LSM6DSO32 INT1 not connected** | `INT1_CTRL` 0x01 written and read back, accel + gyro both at 104 Hz, `STATUS` continuously data-ready — yet **GPIO28 stuck low, never toggles** | Route INT1 to its GPIO and verify the net | Driver detects the silent line after 3 timeouts and polls at 10 ms instead (`rate` 2.0 → 72 Hz). Costs the interrupt's timing precision and some CPU |
| 2 | **BNO055 attitude frozen — cause UNDETERMINED** | Bit-identical Euler triple, `sys`/`mag` calibration stuck at 0. Originally called a faulty fusion core; **that verdict does not hold** (see below) | Re-test with VERIFIED motion before condemning any part. Self-test does **not** exercise fusion (`ST_RESULT` 0x0F on a part that was not updating) | Driver withholds a frozen attitude *while rotating* so the priority-1 gyro backup takes over |
| 3 | **Split the I²C buses** — but isolate the **icp10111**, not the BNO055 | Five devices share `i2c:0`; a wedge, or the icp10111 latch-up recovery's general-call reset, takes them down together | **`i2c:0` = icp10111 alone; `i2c:1` = everything else.** Superseded the original "move the BNO055" plan — see *Which device to isolate* below | None possible in software — the buses are physical |
| 4 | **BNO055 breakout has no 32.768 kHz crystal** | Selecting `CLK_SEL` external kills fusion outright (EUL all zeros) | Prefer a crystal-equipped module: Bosch specifies the external crystal for fusion modes | Driver leaves `CLK_SEL` internal |

### The BNO055 "faulty part" call — retracted

A v0.1 module was condemned as having a dead fusion core. **That conclusion was not supported.** Every
test behind it was run with the board STATIONARY, and a stationary BNO055 legitimately repeats its
fused output bit for bit — the replacement part does exactly the same at rest. The bench rig is fixed
to a breadboard, so sustained motion is awkward, and three later runs that were *assumed* to be moving
turned out to read 0.1–0.4 °/s. The diagnostics now print the gyro on every line and annotate a still
sample as **"STILL — proves nothing"**, so the mistake cannot be repeated silently.

What survives: the two parts behave identically at rest, and neither calibrates without motion. What
does not: any claim that either is broken. Re-test with the gyro column showing >5 °/s before
condemning hardware.

**The operational consequence is the real finding.** NDOF fusion needs motion to calibrate, and a
glider sits still on the pad — so the BNO055 can be uncalibrated and its attitude frozen *at launch*,
on a perfectly good part. That is a pre-flight procedure item, not a hardware defect (see
`doc/field_test.md` Phase 3).

### v0.2 — BNO055 control lines to break out

The v0.1 module is wired with a DFRobot Gravity 4-pin (VCC/GND/SDA/SCL), so every control pin the part
offers is unreachable. Ranked by what this branch actually needed and could not do:

| Pin | v0.2 | Why |
|---|---|---|
| **`RST`** | **MCU GPIO** | The escalation tier we are missing. Recovery today goes device → I2C general call → bus clear, and **all three stop short of "reset this specific part"**. The BNO055 pulled from v0.1 held its latched state through a full power cycle, and the icp10111 latch-up notes say only a rail cycle clears that one. A reset line turns "sensor lost for the rest of the flight" into "lost for 200 ms" |
| **`INT`** | **MCU GPIO** | Data-ready sampling instead of polling. Worth less here than on the LSM6DSO32 (fusion is 100 Hz, we poll at 50), but the pin is cheap and it removes a poll loop |
| `PS1` (+`PS0`) | **jumper** | Selects UART instead of I2C. The BNO055 is a documented I2C **clock-stretcher** — exactly the failure the bus-wedge recovery in `i2cbus` exists for, where one slave holding the line takes down four other devices. UART moves the worst offender off the shared bus entirely, which addresses #3 more thoroughly than relocating it to `i2c:1`. **Confirmed available:** UART3/4/5 all instantiate on this MicroPython build (uart1 = recorder, uart2 = GNSS), and 21 GPIOs are free — so a third UART costs pins, not peripherals |
| `ADR` | solder pad | Selects 0x29 → a **second BNO055** (true attitude redundancy rather than gyro integration), or clash avoidance. No GPIO needed |
| `BL_IND`, `NBOOT` | test pads | Reflashing the sensor's own firmware only |

Free GPIOs after v0.1 (reserved straps excluded): **0, 1, 2, 9–13, 34–36, 39–45, 51–53**.

**Every routed line must be continuity-checked at assembly.** `lsm6dso32_int1` is declared on GPIO28 in
`config_default.py` and is not connected in copper: the driver silently sampled at 2 Hz, `rate` was
stale 96 % of the time, and the PID's D term had quietly degraded to derivative-on-error. Nothing
noticed until an unrelated investigation went looking. A routed-but-dead line is **worse** than an
absent one, because the config claims it works. `diag_lsm_int.py` is the pattern: drive the pin, read
it back, fail loudly.

**Why #3 is the one worth spending layout on.** The attitude PRIMARY (BNO055, `i2c:0`) and its BACKUP
(`tasks/attitude.py`, integrating the LSM6DSO32 on **SPI1**) are meant to be independent. Today they
are — but only because the backup happens to be on SPI. Everything *else* the attitude path depends
on shares `i2c:0`, so a single stuck slave on that bus takes out the primary plus the baro plus the
pitot plus the laser at once. Moving the BNO055 to `i2c:1` leaves `i2c:0` as the "everything else"
bus and gives the attitude chain no common failure point at all: primary on `i2c:1`, backup on SPI1.

### Which device to isolate — it is the icp10111, not the BNO055

The original v0.2 plan was "move the BNO055 to `i2c:1`". Walking the redundancy pairs shows that buys
little, because **attitude is already isolated across bus families**:

| quantity | primary | backup | already isolated? |
|---|---|---|---|
| attitude | **`bno055` (I²C)** | complementary filter (`attitude`), fed by the **SPI** gyro + GNSS | ✅ different bus families |
| airspeed | `sdp810` (I²C) | accel+GNSS estimator (**not on a bus**) | ✅ fallback is not I²C |
| power | `ina226` (I²C) | — | n/a, not flight-critical |
| **altitude** | **`icp10111` (I²C)** | **`bmp280` (I²C)** + laser at rank 2 | ❌ **all on one bus** |

> ⚠️ **Measured: 0.50 % of ICP-10111 frames arrive corrupted on this I²C bus** (2 of 400, bench, board
> idle). Both failures were a SINGLE BIT flipped in the frame's own CRC byte — `0xe9` received where
> `0xf9` is correct — with plausible data bytes either side, and good frames validate exactly. So this
> is bus signal integrity, not a bad sensor and not a bad validator.
>
> The sensor appends a CRC to each of its three words and the driver used to discard all three. At the
> 10 Hz read rate that is a corrupt frame every ~20 s, and since 6 of the 9 bytes are data rather than
> CRC, roughly **one wrong altitude every 30 s was being accepted silently** — into `elevation`, which
> drives the endgame band, the landing trigger and the launch baro backup. The frames are now checked
> and refused; a refused frame costs one sample and takes the same path as a sensor that stopped
> answering (`_recover()` after repeated strikes), with the BMP280 holding the channel up meanwhile.
>
> **This is a v0.2 layout input.** The altitude row above is already the one redundancy gap — primary
> and backup share a bus — and that bus is now measured to be dropping bits. Shortening the run,
> revisiting the pull-ups and isolating the ICP-10111 all get more valuable, not less.

Altitude is the only redundancy pair living entirely inside I²C, and it is the expensive one to lose:
the host fault matrix priced a dead barometer at **100.2 m of miss**, because the endgame band is
elevation-driven. So:

```
i2c:0  ->  icp10111 alone            (on-board, short, primary altitude)
i2c:1  ->  bmp280 + bno055 (SEN0253), laser_agl, sdp810, ina226
```

A wedge on `i2c:1` then costs the backups and the pitot — and the pitot already degrades to an
estimator built for exactly that — while **primary altitude survives**.

**Bandwidth is not a factor and should not be argued about.** Measured from the configured poll rates
and read sizes, `i2c:1` runs at **4.2 %** of a 400 kHz bus and `i2c:0` at **1.5 %**. Even at the
cable-friendly 100 kHz they are 16.7 % and 6.1 %. This is purely a fault-isolation decision.

**A code-level bonus for isolating this particular part.** `icp10111` clears digital latch-up with an
I²C **general-call reset** (`0x00 0x06`), and its driver notes this "also resets peers that honour it
(bmp280, ina226)" — collateral accepted because the alternative is losing the primary baro. Alone on
`i2c:0`, the most aggressive recovery action in the codebase can no longer disturb anything else.

## v1.0 PCB — design review of the v0.1 Gerbers

Measured from the copper (`models/PCBs/*.zip`), not from the schematic. The netlist itself
cross-checks clean against `config_default.py` — this is the separate question of whether it is a
*good* board.

| | main board | power board |
|---|---|---|
| outline | 39.88 × 97.16 mm | 40.00 × 43.94 mm |
| track widths | **0.30 mm — one width for everything** | 0.50 mm signal + 2.75 mm power |
| copper pours | **none** | **none** |
| vias | 12 | 2 |

IPC-2221, external trace, 1 oz copper, ΔT = 10 °C: 0.30 mm ≈ **1.0 A**, 0.50 mm ≈ 1.4 A,
2.75 mm ≈ 5.0 A.

**1 — No ground pour on either board, either layer.** Every return path is a point-to-point trace, on
a board carrying 5 MHz SPI, two I²C buses, PWM to three servos with ~0.8 A transients, **and an
INA226 resolving 2.5 µV per LSB across a shunt**. Servo return currents down a thin shared trace put
a millivolt-scale IR drop across the very ground that measurement references — the power figures the
energy budget rests on are the ones most exposed. *Fix: pour both layers, stitch every 5–10 mm along
SPI/I²C and around the servo connectors. Free in EasyEDA, costs no board area.*

**2 — The main board uses one trace width for signal and power alike.** 0.30 mm ≈ 1.0 A against a
**measured 0.79 A per MG90S** and an INA226 over-current alert set at **3000 mA** — a threshold the
trace feeding it cannot carry. Three servos moving together is ~2.4 A, so the shared servo feed is
roughly **2.4× undersized**. Telling detail: the **power board already does this correctly**; the main
board simply did not inherit the practice. *Fix: ≥1.5 mm for the servo rail and its return, 2.5 mm to
match the power board's margin. 0.30 mm is fine for signals.*

Smaller: only 12 vias on the main board (a pour will raise this naturally), and the two boards use
different Gerber units (main mm, power inch) — cosmetic, but it will bite anyone cross-checking
dimensions by hand.

### Merging main + power onto one board — the energy island

The v0.2 intent is a single board carrying an **energy island**: the main board fed 5 V from that
island rather than from USB, a boost module supplying Recorder + main board, and the servos on an
ND3A05SD with two-capacitor protection (1000 µF + 100 nF). Electrically that is what exists today,
minus the wires and the inter-board grid.

**The island is what makes the merge safe.** The objection to merging is that it puts ~2.4 A of servo
return onto the INA226's copper; an energy island *is* the star ground, made explicit at layout time
— provided it is a genuinely separate pour joined to signal ground at **exactly one point, at the
shunt**, with no signal trace crossing the boundary anywhere else.

**Weight, which is the real driver on a 215–270 g airframe:**

| removed | mass |
|---|---|
| ~10 mm of length × 2 boards | 2.37 g |
| 2 × 8-pin 2.54 mm headers | 2.20 g |
| 8 jumper conductors, ~3 cm | 1.20 g |
| **total** | **~5.8 g** |

That is **2.7 % of the 215 g light glide mass** (2.1 % of 270 g full), and sink scales as √m, so
~1.35 % less sink — before counting the mechanical failure point removed under a measured 3.3–4.3 g
boost. The two bare boards together are only ~16.7 g, so this is a third of the connector-and-edge
overhead. Area is not the obstacle: at the same 40 mm width the combined board is ~141 mm long.

> ⚠️ **Corrected 2026-08-24: LENGTH is the binding constraint, not width.** This paragraph used to
> end "and width is what the body tube constrains", which is wrong for the built airframe -- the
> GNSS + main + power boards in series do not fit the TMS-7 body nose-to-tail. Any layout reasoning
> that trades length for width is therefore backwards. Merging still helps, but because it removes a
> BOARD FROM THE CHAIN, not because it saves area.

**Two things to get right while merging:**

1. **The switcher versus the shunt.** The ND3A05SD and the boost module switch at hundreds of kHz to
   MHz, and the INA226 resolves 2.5 µV/LSB on the rail they feed. Keep the Kelvin sense pair off the
   inductor field, route it as a pair, and place the shunt **upstream of the servo bulk caps** so the
   part measures load current rather than capacitor ripple.
2. **USB stops being a power path — which changes the BENCH workflow.** Board recovery currently
   depends on USB power: `uhubctl -l 1-3 -p 1 -a cycle` clears a wedged USB CDC by cutting the port's
   5 V. Fed only from the island, that recovery ceases to exist and a wedged board needs the battery
   unplugged. Keep a bench path — a diode-OR from USB 5 V, or a jumper selecting USB or island — or
   accept the DTR/RTS reset as the only route. Decide it at layout, not at the bench.

### Decided direction (2026-08-24): simplify the TMS-7 board, defer the flying wing

Two routes were weighed. **Chosen: shorten the existing board** — move the GNSS onto the main board,
drop the separate power board via the energy island above, and remove the ADXL375 / LSM6DSO32 *if the
flight data licenses it* (see below). That takes two boards out of the nose-to-tail chain, which is
what the length constraint actually needs.

**Deferred ~3 months: "TMS-8", a flying wing** whose wider body would take a shorter, wider board with
the island built in. It is a sound idea and is not rejected — it is sequenced. The reason is that it
resets the AERODYNAMICS, not the electronics: `sim_model.AIR_QUALITY` 5.5, the stall bracket, the
catapult energy calibration and every sim conclusion resting on that polar were all measured on the
TMS-7 tube body. A flying wing returns the polar to a guess, which is exactly what those measurements
just eliminated. Build it later on electronics already proven, so one variable changes at a time.

Removal order matters, and is not the obvious one. `hardware.md` classes the **ADXL375 as Optional**
("LSM6DSO32 ±32 g already covers the 8-12 g boost") and the **LSM6DSO32 as Critical** ("the only gyro
`rate`"). So dropping the ADXL375 alone is nearly free — ~25x18 mm at no functional cost — while
dropping both costs the gyro and puts `accel` on the BNO055's ±16 g against an 8-12 g boost. The gyro
loss is separately recoverable: `drivers/bno055.py` already reads its own gyro every sample (bytes
12..17 of the block it fetches anyway) and discards it; publishing it as `rate` restores the PID D
term and the attitude backup with no extra part.

## v1.0 idea — if boost really stays under 16 g, the IMU stack collapses

Measured peak acceleration across the board HITL matrix: **3.3 g** (F15 full), 3.7, 3.8, **4.3 g**
(E16 light) — all far under 16 g, and consistent with the physics (F15 is 15 N average against a
~467 g stack ≈ 3.3 g of thrust, plus 1 g static).

If that holds on real hardware, a **BNO085** becomes interesting: it supersedes the BNO055 with SH-2
fusion and, importantly here, without the calibration-state behaviour that cost a whole bench session
(a part was declared faulty when it was merely uncalibrated). One part could then cover attitude +
accel + gyro, retiring the BNO055 and possibly collapsing the LSM6DSO32/ADXL375 split.

**What must be measured before buying anything:** those figures are *simulation* numbers from a thrust
model containing **no ignition transient, no separation/ejection shock and no landing impact** — and
shock, not thrust, is why a ±200 g ADXL375 is on the board at all. Sustained boost at ~4 g says
nothing about a millisecond ejection spike. So: fly one real capture with the ADXL375 logging at full
rate through boost, separation and landing; read the actual peak; only then decide what the BNO085
replaces. Attitude alone is already a win — retiring the ADXL375 needs the shock number specifically.
The same capture should re-check `launch_g` (2.5 g today, ~1 g of margin against a 3.3 g boost).

> ⚠️ **"Full rate" is 100 Hz today, and that does NOT resolve an ejection spike.** The driver reads one
> sample per poll, the poll floor is the ~10 ms asyncio floor, and the ODR is set to match at 100 Hz —
> so anti-alias bandwidth is ~50 Hz and a millisecond event is attenuated in the analogue path before
> it is ever sampled. Decimation is not the limiter either: the flight profiles set
> `telemetry_ms` 0 (no global decimation), so every ADXL sample the 10 ms poll produces is already
> recorded and there is nothing left to turn off.
>
> **A comfortable ~4 g peak from such a capture is therefore evidence about SUSTAINED BOOST ONLY and
> says nothing about shock** — do not retire the ADXL375 on it. What the capture *does* answer:
> sustained boost g, L/D and sink, wind, landing impact (tens of ms, so 100 Hz catches it), and the
> whole pipeline end to end.
>
### Recommendation on the BNO085 swap (asked 2026-09-05): NOT YET, and here is what decides it

**Where a +/-8 g accelerometer would and would not hurt.** The BNO055's accelerometer is `accel`
**priority 2** -- a fallback behind the LSM6DSO32's +/-32 g at p0 -- so its range does not gate the
`accel` channel at all. Where it matters is INSIDE the part's own fusion: an accel that saturates
during boost degrades the attitude it outputs, at the moment the pending +/-10 deg boost-phase control
would need it. Today control engages after separation, so a boost-time fusion wobble that recovers is
tolerable; with boost control it would not be. (Confirm the part's actual full-scale from the datasheet
before buying -- the BNO08x family's figure is not something to take from memory.)

**If it is adopted, put it on I2C, not SPI.** The breakout offers I2C, SPI and UART-RVC. Choosing SPI
would land it beside the LSM6DSO32 and collapse the deliberate bus-family isolation described above --
attitude primary on I2C, backup fed by the SPI gyro -- which is the property that survives a bus-level
I2C fault. Saving a bus is not worth re-creating a common failure point for both attitude paths.

**The problem it solves is already contained.** The strongest argument for the swap is escaping the
BNO055's measured fusion stall -- a bit-identical Euler returned indefinitely while the channel stays
FRESH. But the driver now detects that (off the part's own gyro, only while rotating) and the
LSM6DSO32 provides an independent backup to fall to. The failure is mitigated, not open, so the swap
buys robustness rather than rescuing a live hazard.

**The cost is a driver, not a part.** The BNO055 is register reads; the BNO08x speaks SHTP -- a packet
protocol with sequence numbers, channel multiplexing and feature reports. The existing driver also
carries hard-won behaviour that would have to be re-earned: the calibration latch, the NVS profile
restore, the stall detector, the peer re-arm. That is spine work landing next to a launch date.

**Sequence:** fly October on what exists; take the FIFO-drained shock capture described above, which
answers the +/-8 g question AND whether the ADXL375 can retire; only then decide. Buying the part now
is cheap and harmless -- committing the firmware to it before the shock number exists is not.

> Measuring shock needs the sensor's 32-sample FIFO drained per poll — 800 Hz gives 8 samples per
> 10 ms poll, ~40 KB/s of the 92 KB/s recorder link, and ~400 Hz of anti-alias bandwidth. Deliberately
> NOT done before the first flights: it rewrites a tested driver's read path, and the flight is worth
> more than the extra number.
