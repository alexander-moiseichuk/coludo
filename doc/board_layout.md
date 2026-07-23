# Board layout & wiring — Phase-5 board setup

Physical partitioning of the electronics into small carrier boards, the harness to the ESP32-P4
main board, and the placement/assembly path to the proto (maket) glider. Pin numbers are the live
values from [`../src/glider/config_default.py`](../src/glider/config_default.py) (`buses` + `pins`,
hardware-validated by `test/test_pins.py`) — **not** the stale `waveshare_esp32p4_pins.md`.

Design goal: **minimise the connection count on the main board.** Cable count = carrier count, not
device count — a shared bus daisy-chains N devices onto one cable. The two levers applied here are
(1) cluster devices by bus onto carriers, (2) split the current-sense onto the free second I2C bus
so the forward sensor bus stays short.

## 1. Main-board pin map (ESP32-P4)

Status legend: **req** = firmware needs it · **dbg** = optional, firmware polls by default but the
pin is reserved so you can solder the INT for debugging · **tie** = not wired, strapped on the carrier.

| Signal | GPIO | Bus / role | Status |
|---|---|---|---|
| I2C0 SDA / SCL | 7 / 8 | forward sensor cluster (BNO055, BMP280, ICP-10111, VL53L4CX, SDP810 airspeed) | **req** |
| I2C1 SDA / SCL | 31 / 30 | aft power bus (INA226) — real header pins (codec 9–13 not broken out) | **req** |
| SPI1 SCK / MOSI / MISO | 48 / 47 / 46 | IMU bus (LSM6DSO32, ADXL375) | **req** |
| LSM6DSO32 CS | 50 | SPI chip-select | **req** |
| ADXL375 CS | 49 | SPI chip-select | **req** |
| LSM6DSO32 INT1 | 28 | gyro data-ready → PID **D-term** (jitter-sensitive) | **req** |
| INA226 ALERT | 29 | hardware over-current trip (rides the I2C1 cable) | **req** |
| ADXL375 INT | 4 | >32 g backstop data-ready | **dbg** (poll via `fallback_ms`) |
| VL53L4CX INT | 3 | laser data-ready | **dbg** (poll via `fallback_ms`) |
| VL53L4CX XSHUT | 5 | laser enable/reset | **tie** high on the I2C carrier (single laser, no addr conflict) |
| UART2 TX / RX | 22 / 23 | GNSS (ATGM336H) | **req** |
| UART1 TX / RX | 20 / 21 | Recorder (Luckfox) | **req** |
| Servo PWM ×3 | 26 / 27 / 32 | yaw / eleron-L / eleron-R | **req** |
| Separation switch | 33 | copper pads: HIGH=nested, LOW=separated | **req** |

**Lane count on the main board:** **19 required** signal lanes → **21** if you solder
the two debug INTs (ADXL 4, VL53 3) → XSHUT (5) freed by strapping. This is the "everything except
XSHUT" harness you prefer, with only LSM-INT + INA-ALERT actually load-bearing in firmware.

## 2. Carrier-board split

Each carrier presents **one bus plug** to the main board (+ optional debug pins). Nose→tail order
follows the glider layout below.

### A · Forward I2C sensor carrier  *(movement-sensitive cluster, forward, laser down-facing)*
- **Devices:** BNO055 + BMP280 (sen0253 combo, 0x28/0x76) · ICP-10111 (sen0517, 0x63) ·
  VL53L4CX (laser, 0x29, pointing **down**; can be a short daisy stub below the stack) ·
  **SDP810** (airspeed, 0x25) — mount it forward enough that the **P+ tube** reaches the nose pitot
  in a short run (P− is open to the interior bay); no extra main-board pin, it just daisy-chains the bus.
- **Plug → main board:** `[SDA0, SCL0, 3V3, GND]` — **4-pin (req)** (5 devices on the one daisy).
- **Optional debug:** `[VL53 INT → GPIO 3]` — 1-pin. XSHUT strapped to 3V3 on the carrier (no wire).
- Bus is now **forward-only** (INA226 moved off) → short run, keeps the calibrated high I2C freq.

### B · IMU (SPI) carrier  *(mid, mounted under the main board, reversed)*
- **Devices:** LSM6DSO32 (primary accel + gyro, CS 50) · ADXL375 (>32 g backstop, CS 49).
- **Plug → main board:** `[SCK 48, MOSI 47, MISO 46, 3V3, GND, LSM-CS 50, ADXL-CS 49, LSM-INT 28]`
  — **8-pin (req)** (LSM-INT is the one required interrupt).
- **Optional debug:** `[ADXL INT → GPIO 4]` — 1-pin.
- Mount directly under the main board so the high-rate SPI run is as short as possible (least
  capacitance/jitter — this is why the gyro-INT stays clean).

### C · Aft power / INA carrier  *(the "power board", aft, ahead of engines)*
- **Devices:** INA226 (current sense, **I2C1**, 0x40) · 4–6 power-ups (servo-rail boost/buck) ·
  output caps on the shunt · battery input.
- **Plug → main board:** `[SDA1=31, SCL1=30, 3V3, GND, INA-ALERT=29]` — **5-pin (req)**.
- **Power distribution (not main-board signal):** battery in → 5 V controller rail + separate servo
  rail out; USB out to the recorder and the main board.
- INA226 on its own local I2C1 keeps the current-sense bus 2 cm long instead of a nose-to-tail run.

### D · GNSS  *(forward + high, away from the engine antenna — EMI)*
- **Plug → main board:** `[TX 22, RX 23, 3V3, GND]` — **4-pin (req)**.

### E · Recorder (Luckfox)  *(forward-mid, next to/under GNSS; drives the camera over MIPI CSI)*
- **Plug → main board:** `[TX 20, RX 21, GND]` — **3-pin (req)**. Self-powered from the power board.
- Camera (3336b) connects to the Luckfox, **not** the main board.

### F · Servos  *(3× SG90 at the fins, tail)*
- **Plug → main board:** `[PWM yaw 26, PWM el-L 27, PWM el-R 32, GND]` — **4-pin (req)** (signal only).
- Power from the C-board servo rail, not the main board.

### G · Separation switch  *(glider/booster interface, aft)*
- `[GPIO 33, GND]` — 2-pin, or copper-pad direct (it is already pad-based).

**Main-board plug census: 7 plugs** (A 4-pin, B 8-pin, C 5-pin, D 4-pin, E 3-pin, F 4-pin, G 2-pin)
— all within your 2–8-pin connector inventory. Optional debug adds 1 pin each to A and B.

## 3. Placement & lane optimisation

Nose → tail (your vision, annotated):

```
 nose ─ camera → Luckfox(recorder) → GNSS(up) ─┐ forward I2C carrier A (BNO/baro/ICP, laser↓, SDP810)
   └ pitot P+ (forward, under camera) → SDP810 │ tube; P− open to the interior bay
                                               │
 ───────────────────────── middle ────────────┤
                                               │
        IMU carrier B (under main, reversed) ──┤
        MAIN BOARD (ESP32-P4)                  │
        power carrier C (battery, INA, powerups)
        separation pads ─ engines(booster, ejects) ─ tail   fins+servos F
```

- **Keep SPI short:** carrier B goes on the underside of the main board (reversed) → millimetres of
  SPI, not a cable. This is what lets LSM-INT stay the one clean interrupt.
- **Keep I2C0 forward-only:** with INA226 on I2C1, the forward bus threads just the 5 sensors in a
  line — one short daisy chain, one 4-pin cable back.
- **Pitot at the nose, short P+ tube:** the SDP810 sits on carrier A but its P+ tube must reach the
  forward-facing pitot under the camera — keep that run short + sealed; P− vents to the interior.
- **Don't run the GNSS UART parallel to the servo PWM** (PWM edges couple into the 9600-baud line).
  GNSS forward + up also maximises antenna separation from the engine.
- **Copper-adhesive + conductive-pen** is fine for the short static runs — separation pads and
  servo/power grounds. Use **soldered wire + plugs for the buses** (I2C/SPI/UART): signal integrity
  and re-pluggability for the breadboard→proto move.
- **Both faces of a 32×78 mm carrier:** e.g. carrier A hosts the attitude/baro stack on top and the
  laser looking down through the board; carrier B is simply the underside of the main board. One
  connector per carrier regardless of which face a device sits on.

## 4. Proto (maket) assembly & test path

The proto is a **wingless glider shell** with all carrier boards + battery + engines + fins, flown
like the real thing for the Phase-5 walk-test (plan.md → Phase 5 · 1):

1. **Breadboard first** — assemble and verify each carrier (A–G) on the current breadboard via its
   plug: `verify` (device up + probe) + the readiness gate, then bench HITL.
2. **Move to proto** — same plugs, re-seated into the maket. No re-wiring, just re-plugging.
3. **Walk-test** — flight loop in GLIDING (manual hold), armed: carry it at ~5 m/s and watch the
   fins track live attitude + landing-zone heading; the Luckfox captures a `flight_report`.
4. **Separation-by-hand** — trip the switch, confirm `BOOSTING → GLIDING` and the loop engages.
5. **Drop tests (2–3 m)** — the low-altitude landing path (laser AGL → LANDING → stationary), the
   final stage of the ladder.

## Disabling an optional pin

To turn an optional feature off on a board that does not wire it, set its `pins` entry to **`null`**
(preferred — the row stays as a documented placeholder) or any **negative** number. It resolves to
"no pin" exactly like an absent entry: the driver skips the feature (poll instead of INT, no XSHUT
toggle, no hardware ALERT). E.g. `"laser_xshut": null` runs the laser always-on; `"adxl375_int":
null` polls the ADXL on its `fallback_ms` timer. A non-negative GPIO is the only "wired" value, so a
disabled pin never collides in `verify`.

## Open items

- ~~I2C1 SDA/SCL GPIOs~~ — **DONE (7/07)**: i2c:1 = sda 31 / scl 30, ALERT 29; INA226 on `id:1`,
  scan-confirmed (mfr TI, Vbus ~5 V).
- ~~Disabled-pin convention + generated pin doc~~ — **DONE (7/07)**: `null`/negative → feature off
  (`task._pin_gpio` + validator); `doc/waveshare_esp32p4_pins.md` is now **generated** from
  `config_default` by `tools/gen_pinmap.py` (`--check` gates staleness); GPIO 2 / LED removed.
- **When you wire it:** the debug INTs (ADXL 4, VL53 3) are on by default and self-poll if silent;
  set them (and `laser_xshut`) to `null` per board once you finalise which are physically connected.
