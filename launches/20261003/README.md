# Launch 2026-10-03 — HPRC

Rescheduled from **2026-09-05, cancelled for weather**.

Seven slots: one booster-only structural test, two glide-comparison airframes, one telemetry airframe,
one servo-equipped airframe, and two airframes on the new main boards: TMS-7E (v1.0) and TMS-7F
(v1.1), both built.

## Manifest

Airframe masses are **as assembled, without the motor**. Motor masses are the measured values from
`doc/hardware.md` — **F15-4 98.8 g**, **E16 ~82.5 g** loaded.

| airframe | glider | booster | motor | + motor | **liftoff** | glide mass |
|---|---|---|---|---|---|---|
| [TMS-7](TMS-7/README.md) | — | **129.5 g** | F15 | 98.8 g | **228.3 g** | n/a (no glider) |
| [TMS-7A](TMS-7A/README.md) | 183.0 g | 99.0 g | E16 | 82.5 g | **364.5 g** | 183.0 g |
| [TMS-7B](TMS-7B/README.md) | 184.3 g | 98.2 g | E16 | 82.5 g | **365.0 g** | 184.3 g |
| [TMS-7C](TMS-7C/README.md) | 216.5 g | 98.9 g | F15 | 98.8 g | **414.2 g** | 216.5 g |
| [TMS-7D](TMS-7D/README.md) | 287.5 g | 97.9 g | F15 | 98.8 g | **484.2 g** | 287.5 g |
| [TMS-7E](TMS-7E/README.md) | 250.0 g | 94.8 g | F15 | 98.8 g | **443.6 g** | 250.0 g |
| [TMS-7F](TMS-7F/README.md) | *not assembled in time — not flying 10-03* | — | — | — | — | — |

## What each slot answers

* **TMS-7** — does the new booster construction survive the strongest motor? Structure only, no glider,
  but carrying the 10 g nose logger so it also returns peak boost acceleration and apogee altitude.
  The F15 is deliberate: test the worst case before trusting it under an instrumented airframe.
  **Result: yes.** It flew to ~470 m (14.9 g peak, 125 m/s) and came back intact, and the logger returned
  the whole flight. These are the boundaries for every F15 airframe; see [TMS-7](TMS-7/README.md).
* **TMS-7A vs TMS-7B** — the wing comparison, flown as a matched pair on the same motor: **ASE wings
  with fins at 0°** against **carbon wings with fins at −5°**. Dual cameras, 1.3 g apart in mass, so the
  difference that shows up is the wing and the fin setting rather than the build.
* **TMS-7C** — telemetry. Fins **fixed at whatever 7A/7B selects**, so the instrumented flight uses the
  setting the comparison just validated rather than a guess.
  **Result: crashed.** Fins at 0°. The stack lost boost stability just off the rod (roll torque ∝ q,
  up to 9.3 g sideways), looped under thrust and hit the ground; the recorder kept 1.31 s. Hardware to
  be validated for December; see [TMS-7C](TMS-7C/README.md).
* **TMS-7D** — the servo airframe: three SG90s and the full power module.
  **Result: no separation, dropped.** It bent over in a ~4.5 m/s crosswind to a 32° path but stayed
  bounded. Apogee was 91–98 m at 5.4 s; the glider never separated, and the stack came down ~250 m out.
  The recorder kept 6.4 s. See [TMS-7D](TMS-7D/README.md).
* **TMS-7E** — the first airframe on main board v1.0, where the payoff is mass; see its folder.
  **Result: no separation, dropped.** The PCB broke in two, and the recorder's SD card is electrically dead
  (it heats to 60 °C+, never initialises), so there is no data.
* **TMS-7F** — the first v1.1 airframe (SEN0697: the board fuses its own attitude and carries a
  magnetometer); see its folder.

## What 10-03 measured — summary

Recorded flights: **TMS-7** (nose logger, the whole flight), **TMS-7C** (recorder, the first 1.31 s),
**TMS-7D** (recorder, the first 6.42 s). 7E's card died; 7F did not fly. Each flight was cut from its
recorder dump by `src/logger/flight.py` or `tools/recorder_flight.py` and measured with a gyro-plus-
accelerometer strapdown. Each was then checked by independent analyses and an adversarial judge,
recomputing from the data; the details are in each airframe's README. Not much data in total, but
enough to bound the dynamics and to show which sensors work in flight.

### What happened

| | outcome | why, as far as the data says |
|---|---|---|
| TMS-7 | **success**: 464–479 m, chute recovery | motor normal; the chute partly collapsed below ~150 m (descent 6.5 → 11 m/s) |
| TMS-7C | **looped and crashed** within ~2.5 s | a boost-stability departure at rod exit (0.31 s): near-neutral margin plus a built-in roll asymmetry; 9.3 g aerodynamic side load |
| TMS-7D | **no separation, dropped** ~250 m out | bent over in a crosswind but stayed bounded; low apogee (91–98 m); separation never happened before 6.42 s |
| TMS-7E | **no separation, dropped**; PCB broken | no data (SD card dead) |

### The corner parameters

The extremes these flights reached, against each sensor's range:

| quantity | measured | sensor range / note |
|---|---|---|
| boost peak (axial) | **14.9 g** (TMS-7, 228 g) · 8.25 g (7C, 414 g) · 6.2–6.7 g (7D, 484 g) | BMI323 ±16 g: **93 %**; peak g ≈ 32 N ÷ mass |
| shocks | 19 g (TMS-7 landing sequence), ~9 g (ejection) | **lower bounds**: 50 Hz filtering flattens them |
| lateral load | **9.3 g** (7C) · 3.6 g (7D) | aerodynamic normal force at an angle of attack of 20–30°+ |
| rotation | **1595 °/s** roll (7D), 775 (7C), 2000 = rail on 0.9 % of TMS-7's chute samples | LSM6DSO32 / BMI323 ±2000 °/s |
| speed | **125 m/s, Mach 0.36** (TMS-7) · 53.5 m/s (7D) · ~33 m/s (7C) | — |
| dynamic pressure | ~8.8 kPa (TMS-7) · ~1.6 kPa (7D) | pitot ceiling **0.55 kPa** |
| apogee | 464–479 m (TMS-7) · 91–98 m (7D) | — |
| F15 motor | **~32 N spike at 0.3 s**, burnout 2.53–2.68 s, **42 N·s**, ejection 5.8 s after burnout | the simulator flies 14.4 N × 3.45 s = 49.7 N·s, with a 4 s delay |
| descent | 6.1–6.9 m/s, then 9–12 m/s (TMS-7, chute degrading) | — |
| battery (7D) | 3.89–3.91 V, 0.47–0.70 A in flight; 1.55 A servo sweep on the pad; 198 mAh over 23.5 min of pad wait | INA226 |
| temperatures | MCU 40–42 °C, bay baros ~38 °C, logger die 35 °C (pad in the sun) | — |
| wind | ~4.5 m/s crosswind at 7D's launch (fitted); 9 m/s off the rod gave a 26° angle of attack | — |

### Sensor by sensor: what worked

- **Gyros worked:** LSM6DSO32, BMI323. Strapdown from the pad's gravity vector reproduced the baro apogee to
  within 3 %. Rates stayed in range (7D peaked at 80 % of ±2000 °/s). TMS-7's nose rail hits came only
  under the chute. Possible x-gyro scale error of ~2 % on 7D: check it on a rate jig.
- **Accelerometers were mixed.** The LSM6DSO32 (±32 g) and ADXL375 (±200 g) are fine. The ADXL375 carries a
  pre-flight +0.78 g offset on x: calibrate it out. The **BMI323 (±16 g) is marginal** for a light airframe
  on an F15, and anything under ~215 g at liftoff rails it.
- **The BNO055 is not a boost sensor.** Its fusion accelerometer clips at 4 g: 41 of 45 rows on 7C, 17 of 231
  on 7D. Its Euler roll sat near the singularity on the pad. Only its nose elevation held up, checked
  against the gyro.
- **The pitot reads low, then tops out.** It read **0.52–0.8 of the inertial dynamic pressure**, so its span
  is low. Then it **topped out at 546 Pa ≈ 30 m/s** for half of 7D's flight, and it dropped out at large
  angles of attack (7C, 0.69–0.75 s). For boost and coast, airspeed has to come from the inertial
  solution, or from a higher-range sensor.
- **The baros are trustworthy only when slow.**
  - The nose baro read **+83 m high at burnout**: suction of ~0.1 × the dynamic pressure.
  - The bay baros over-read at large angles of attack (45 m against a 34 m ceiling on 7C).
  - The temperature scale adds ±4 % (die 35–38 °C against air).
  - They are trustworthy for apogee, descent and ground. The filtered baro never read falling before
    ejection.
- **Launch detection worked.** 7D detected `|a|=5.1g dwell=100ms` at +0.18 s. The apogee detector
  (5 m drop, 100 ms dwell) would fire ~1.4 s late on a shallow arc.
- **GNSS** had no fix on any board. **The laser** was sporadic and only valid near the ground.
- **The recorder** is the weak link:
  - **14–30 % of rows were lost** on the UART, and thousands of corrupted file names were spun off.
  - The **tail goes at power loss** (page cache): 7C kept 1.31 s, and 7D ends ~4 s before impact.
  - `recorder.log` is buffered.
  - The operator's reading: video plus logging overloads it.
- **The nose logger worked:** lossless at 50 Hz through the whole TMS-7 flight. It nearly overwrote the
  flight on the walk back, and that is fixed now: launch latching, flight protection, a self-stop once
  landed.
- **Board time:**
  - The RTC is unset, so every session is named 2000-01-01; flights are found by their boost.
  - **`ticks_us` wraps every 17.9 min**, so a long pad wait fakes a reset in the data.
  - The OOM forecast is wrong in both builds (`leak_kbps` 1 against a measured ~340 KB/s).

### What to improve before December

1. **Stack stability**, the cause of 7C and of 7D's bent path.
   - Measure the stack CG with the motor, and swing-test it in both planes.
   - Fly only at ≥ +1 caliber of effective margin at 20° angle of attack (~2–2.5 cal on Barrowman's linear
     formula; 7C had 0.25, 7D 0.71).
   - Enlarge the booster fins, fair the folded wings, or add nose ballast.
   - Jig the fins to 0 ± 0.25° and key the joint against roll.
   - Use a 2–3 m rail, or a wind limit of about a quarter of the rod-exit speed.
2. **Separation**, which failed on 7D and 7E; the operator is redesigning it.
   - Release from the board on a vertical speed ≤ 0, with the motor charge only as a backstop: no fixed
     delay covers 2.9–5.1 s after burnout.
   - Prove ≥ 1.3 J at full glider mass after a real burn.
   - Rate the motor retention above the ejection force.
   - Log the pin level in every checkpoint.
   - **Never engage control on an unseparated stack.**
3. **The recorder**, per the operator's plan: no video, a ~2000 µF hold-up capacitor, a new file organization.
   Accept it with a power-pull test while logging.
4. **Firmware.**
   - Emit a non-wrapping timestamp, and set the clock so sessions carry the date.
   - Fix the OOM forecast.
   - Arm the apogee detector on measured burnout.
   - Log an "ejection kick" event separately from "separated".
5. **Sensors.**
   - Calibrate the pitot's span, and give it a higher range or treat it as invalid above 30 m/s.
   - Give the bay a static port.
   - Keep the BNO055 out of the boost.
   - Calibrate the ADXL375 offset.
   - Check the gyro scale on a jig.
   - Re-validate every re-used part (checklists in the 7C and 7D READMEs).
6. **The simulator.**
   - Fly the measured F15: a 32 N spike, 42 N·s, burnout 2.6 s, ejection 5.8 s after.
   - Model a 6-degree-of-freedom boost with wind, a q-proportional roll asymmetry, margin per plane, and
     the rod.
   - Add replay regressions: 7C must depart, 7D must reproduce its 32° path minimum and ~95 m apogee, and
     TMS-7 must reach ~470 m.
7. **Recovery:** inspect TMS-7's chute and lines, which degraded below 150 m. Keep flying the nose logger
   as a black box.

## One thing the flight data already says about 7D's mass

[`TMS-7-preflight`](../../doc/sims/TMS-7-preflight/) flew 90 HITL flights across three load/thrust
combos last week. Two of them bracket the airframes here:

| study combo | glide mass | median miss | in zone |
|---|---|---|---|
| `f15_half` | 235 g | 37–46 m | **7/10** |
| `f15_full` | 285 g | 88–91 m | **0/10** |

**TMS-7D at 287.5 g is `f15_full` almost exactly**, and TMS-7C at 216.5 g is lighter than `f15_half`.

This does **not** affect 2026-10-03: 7D flies `tms7d.config`, which has servos fitted but **control
OFF**, so no airframe here attempts a guided landing. It matters for the flight after: when 7D does fly
under a PID, it will do so at the mass that landed 0 of 10 in the zone, and reaching the 235 g that
landed 7 of 10 means removing ~52 g. Better to know that before the airframe is finished than after.

## Firmware and calibration per airframe (2026-09-24)

Two firmware generations fly on purpose: the hardened build on the two new airframes, the long-tested
builds on the two that already proved themselves. Every capture stamps its firmware version, so the
groups stay separable in the data.

| airframe | firmware | config | still owed before flight |
|---|---|---|---|
| **TMS-7E** (v1.0) | `2026.09.23.55c494a0c659` | [`tms7e.config`](TMS-7E/tms7e.config) = the board's `board.config` | BNO055 figure-8, then **`calibrate imu_bno055`** (its profile has never reached NVS); pad pitot tare |
| **TMS-7F** (v1.1) | `2026.09.23.55c494a0c659` | [`tms7f.config`](TMS-7F/tms7f.config) = the board's `board.config` | BMM350 LEVEL full circle, then **`calibrate mag_bmm350`**; pad pitot tare. **Sealed: updates over WiFi only** (`tools/ota_push.sh TMS-7F ...`), no more USB |
| TMS-7C, TMS-7D | their long-tested builds (read `whoami`) | unchanged | as before |

**Calibrate over WiFi, assembled and on battery** -- a tethered airframe can do neither a figure-8 nor a
full circle. The commands go from the dashboard's send-command box (`calibrate`, params `imu_bno055` /
`mag_bmm350` / `airspeed_sdp810`); each is saved to NVS and restored on every boot. On 7F the
magnetometer refuses anything short of a real turn ("N of 8 sectors covered" must reach the count).

**On 7E/7F (the hardened build)** active commands answer `err unsafe` once airborne -- probe, verify's
sweep, arm, `calibrate <device>`, detect, set-config, reboot -- while `stage`, `disarm` and every read
stay open. A LANDING that never settles ends in DONE after 10 s (`land_timeout_ms`).

**On 7C/7D (the older builds) the operator is the guard:**
- they execute active commands in flight: send nothing but reads to them once launched, and no `all`
  commands while anything is airborne;
- never leave a stage forced: `stage setting`, then power-cycle before flight;
- tare the pitot with `calibrate airspeed_sdp810` (it persists) -- not `update {"zero": true}`, which
  is RAM-only on these builds; re-zero the baro with `calibrate baro_icp10111` (`update {"rezero": true}`
  errors there);
- a green `verify` is weaker for the ICP-10111 and the pitot on these builds (their probes pass on
  stale data): glance at the live values too.

## Plan — 2026-09-04

- [ ] check **TMS-7D**, upload the latest firmware; after final assembly the board is worked over **OTA**
- [ ] prepare firmware changes if any are needed
- [ ] close the `lmoiseichuk/launch_0905` branch

## Plan — next branch

- [ ] move the breadboard to **MicroPython 1.29** (1.30-pre carries nothing interesting and no fixes we need)
- [ ] possibly update `mpy-cross` alongside it
- [ ] settle **main board v0.2**:
  - merge **main + power** onto one board
  - merge **GNSS** in, cutting its power lines and UART run
  - a **separate I²C cluster for SDP810 + VL53L4CX**, everything else on the internal cluster
- [ ] rework the board layout for that
- [ ] rework the pin maps
- [ ] design the PCB and order it
- [ ] start building **TMS-7E** on the new main board
- [ ] disable the devices the new layout drops, in config

## After the October launch

- analysis session on the flight data
- possibly a **Rust migration**, staying on the same ESP32-P4 so the hardware is reused
- further out, a **C6 board** — only if its memory and FPU-emulation speed prove sufficient
