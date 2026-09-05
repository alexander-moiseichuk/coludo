# Board layout & wiring — v1.0 (target)

**Designed, not built.** The physical consequence of the v1.0 decisions in
[`hardware.md`](hardware.md): main and power merged, the GNSS chip on the main board with only its
antenna outboard, the front-panel devices alone on `i2c:1`, and no ADXL375. Pin numbers come from the
same `config_default` + `layout.apply()` pair that generates
[`waveshare_esp32p4_pins.md`](waveshare_esp32p4_pins.md).

The design goal is unchanged — **minimise the connection count on the main board** — and v1.0 makes
the biggest available cut: three of the seven carriers stop being carriers at all.

## What moves onto the main board

* **Power stage + INA226.** Carrier C disappears; the battery input, the servo-rail converters, the
  shunt and the current sense become an energy island on the main board. INA226 joins `i2c:0` with
  the on-board sensors — it must not sit on the front bus, which is the one that can be lost.
  **This merge is what makes the USB 5 V Schottky mandatory** (`hardware.md` → *REQUIRED on v1.0*):
  once the island is on the board, plugging USB in with the battery connected puts two sources on one
  net, and the isolated buck cannot sink the current that results.
* **BNO055 + BMP280.** The attitude primary and the altitude backup are on-board devices on `i2c:0`,
  short traces, no cable.
* **GNSS chip.** Only the antenna stays outboard (an active antenna carries its own LNA, so the stage
  that sets sensitivity is away from the switching noise). The 4-pin UART plug is gone; what remains
  is one RF run to the antenna connector.

## Carriers

### A · Front-panel cluster  *(nose, `i2c:1` at 100 kHz)*
- **Devices:** ICP-10111 (`0x63`, altitude primary) · SDP810 (`0x25`, pitot) · VL53L4CX (`0x29`,
  down-facing AGL).
- **Lane → main board:** `[UART1 TX 20, SDA1 31, SCL1 30, 3V3, GND]` — **5-pin**, shared with the
  recorder because both run forward down the same side. This is the 8→5 cut that frees enough edge
  for a USB connector on the same face.
- **No INT, no XSHUT.** `laser_int` is dropped (the poll fallback is the same code path at 20 Hz) and
  `laser_xshut` with it (`_reset()` runs only at `setup()`, so it was never in-flight recovery).
- Bus runs at **100 kHz**: nothing here exceeds 50 Hz, so a quarter of the clock costs no sample rate
  and buys edge margin on the longest run in the airframe.

### B · IMU (SPI) carrier  *(mid, under the main board, reversed)*
- **Devices:** LSM6DSO32 alone (primary accel + the only gyro, CS 50). ADXL375 not fitted.
- **Plug → main board:** `[SCK 48, MOSI 47, MISO 46, 3V3, GND, CS 50, INT 28]` — **7-pin** (was 8).
- Still its own bus family on purpose: the attitude backup is computed from this part, so it must not
  share a failure domain with the BNO055 on `i2c:0`.

### C · Recorder + GNSS antenna  *(forward-mid)*
- Recorder (Luckfox) rides the shared front lane above; the camera connects to it, not to the main
  board. The GNSS antenna sits beside it — the two are already the pair that share a rail.

### D · Servos  *(3× SG90 at the fins, tail)* — `[PWM 26, 27, 32, GND]`, **4-pin**, unchanged.

### E · Separation switch — `[GPIO 33, GND]`, 2-pin or copper-pad, unchanged.

**Plug census: 4** (front lane 5-pin, IMU 7-pin, servos 4-pin, separation 2-pin) — down from **7**.
Two of the three removed plugs were not simplifications of the harness but deletions of it: the power
and GNSS cables stop existing rather than getting shorter.

**That is the point, and it is measured in grams rather than tidiness.** The deleted harnesses, the
connectors that went with them and the shorter nose (7D runs 10 cm longer than 7C) are expected to save
**30–50 g**, taking the glider from 287.5 g to **237–257 g**. On this airframe 50 g is worth **45 m of
median miss and seven in-zone landings** (`TMS-7-preflight`: 285 g → 88–91 m and 0/10, against 235 g →
37–46 m and 7/10). No control change available to us moves accuracy nearly that far — so the carrier
count above is an accuracy argument, not a housekeeping one.

# Transition v0.1 → v1.0

Physical work, in the order it is least annoying to do. The per-GPIO delta is generated in
[`waveshare_esp32p4_pins.md`](waveshare_esp32p4_pins.md); this is the carrier-level view.

| # | Change | Effect |
|---|---|---|
| 1 | Move ICP-10111 off carrier A's `i2c:0` daisy onto `i2c:1` | front cluster becomes one bus, one lane |
| 2 | Move SDP810 and VL53L4CX to `i2c:1` with it | `i2c:0` keeps only on-board devices |
| 3 | Move INA226 to `i2c:0` | `i2c:1` is purely front-panel and expendable |
| 4 | Drop `laser_int` (GPIO 3) and `laser_xshut` (GPIO 5) | front lane 6 wires → 4, plus the recorder UART = 5 |
| 5 | Remove ADXL375 from carrier B; drop CS 49 and INT 4 | IMU plug 8-pin → 7-pin |
| 6 | Set `i2c:1` to 100 kHz | edge margin on the long run |
| 7 | *(new PCB only)* merge power + GNSS onto the main board | carriers C and D stop existing |

**Steps 1–6 need no new PCB** — they are a rewire of the existing breadboard, and the firmware detects
the result: `layout.detect()` votes on which bus the four moving addresses answer on, so a correctly
rewired board comes up as v1.0 with no config edit. Step 7 is the board order.

**No pin is renumbered by any of this.** Four GPIOs are freed (3, 4, 5, 49) and four devices change
bus at unchanged pins; everything else is untouched, which is why the rewire is a re-plug rather than
a re-route.

# Board layout & wiring — v0.1 (as built)

**The boards that exist today**: TMS-7C, TMS-7D and the breadboard, until the transition above is
done. Physical partitioning of the electronics into small carrier boards, the harness to the ESP32-P4
main board, and the placement/assembly path to the proto (maket) glider. Pin numbers are the live
values from [`../src/glider/config_default.py`](../src/glider/config_default.py) (`buses` + `pins`,
hardware-validated by `test/test_pins.py`); `waveshare_esp32p4_pins.md` is generated from the same
source and carries both revisions.

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
| UART1 TX / RX | 20 / 21 | Recorder (Luckfox) | **req** — TX only on the v0.1 PCB; see below |
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

The step-by-step **no-ignition ground checklist** for steps 2–4 (CC + glider: power-up → sensors →
attitude/IMU → GNSS/guidance → airspeed → boost-detect → separation → data, with pass criteria) is
[`field_test.md`](field_test.md).

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
  **v1.0 settles this by deletion** — all three pins are dropped, and `layout.apply()` nulls the laser
  pair automatically when the revision resolves, so nothing has to be remembered per board.

## GPIO21 (UART1 RX) is not wired on the v0.1 PCB

> Carried into the v1.0 review as a board question: the recorder link is write-only today, so the
> missing return path costs nothing that is currently used. Route it or drop it deliberately on the
> new board rather than inheriting the omission.

The netlist confirms it: **20 of 22 GPIOs route to their connector pads, and GPIO21 has no copper.**
UART1 TX (GPIO20) reaches the Recorder's RX through front pin 8, but the return path — Recorder TX
back to GPIO21 — was never routed.

That is **fine today and intentional in effect**: the Recorder link is telemetry-write-only, the board
never reads UART1, and nothing in the firmware expects a backchannel. It is recorded here because it
is invisible from the schematic side and would otherwise be discovered the hard way by whoever first
wants bidirectional Recorder comms — at which point it needs either a patch wire to a free Recorder
GPIO, or a route in the next board revision.

(The other absent GPIO, 5 / `laser_xshut`, is deliberate: XSHUT is strapped high on the VL53L4CX
carrier, since a single laser has no address conflict to resolve.)
