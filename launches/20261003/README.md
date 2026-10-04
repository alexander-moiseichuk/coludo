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
| dynamic pressure | ~8.8 kPa (TMS-7) · ~1.6 kPa (7D) | pitot ceiling **0.546 kPa** (the sensor's own) |
| apogee | 464–479 m (TMS-7) · 91–98 m (7D) | — |
| F15 motor | **~32 N spike at 0.3 s**, burnout 2.53–2.68 s, **42 N·s**, ejection 5.8 s after burnout | the simulator flies 14.4 N × 3.45 s = 49.7 N·s, with a 4 s delay |
| descent | 6.1–6.9 m/s, then 9–12 m/s (TMS-7, chute degrading) | — |
| battery (7D) | 3.89–3.91 V, 0.47–0.70 A in flight; 1.55 A servo sweep on the pad; 198 mAh over 23.5 min of pad wait | INA226 |
| temperatures | MCU 40–42 °C, bay baros ~38 °C, logger die 35 °C (pad in the sun) | — |
| wind | ~4.5 m/s crosswind at 7D's launch (fitted); 9 m/s off the rod gave a 26° angle of attack | — |

### Sensor evaluation

Every sensor was evaluated from all recorded data: both recorder dumps (bench and field sessions), the
cut flights, and TMS-7's nose logger. Four independent analyses and a judge recomputed the numbers from
the data.

| device | airframe | status | decisive evidence | fix |
|---|---|---|---|---|
| **GNSS fix** (ATGM336H) | 7C, 7D | **not working** | **0 fixes in 11,334 GGA rows**, and no board in the fleet has ever had one. In the field it was powered and talking for 14.7 min (7C) and 23.6 min (7D) | see the GNSS section below |
| OOM forecast (`health`) | 7D | **not working** | leak 1 KB/s and 29,361 s to OOM, while memory actually fell at **360 KB/s**: OOM ~85 s away. The window was never reset after the 23 min pad wait | reset the window at GC-off / stage change, then soak-test |
| recorder SD card | 7E | **not working** | electrically dead; no data | replace it |
| **SDP810 pitot** | 7C, 7D | **degraded** | the part is fine (probes 25/25), but it reads **~0.51–0.59 of the inertial dynamic pressure** at low angle of attack. It **tops out at the sensor's own 546 Pa** (115 of 229 flight rows on 7D) and drops out at high angle of attack | calibrate the span with the flight tubing; flag ≥ 545 Pa invalid; add a static port |
| **BNO055** | 7C, 7D | **degraded** | the mount tilt is uncorrected (11–14° error); the accelerometer clips at 4 g (41 of 45 boost rows on 7C). It **output all zeros on a 7D bench session while its probe passed**, and its pitch sign flipped in 3 of 12 boots | apply the mount rotation; fail the probe on zeros; log CALIB_STAT; use it only in slow phases |
| ADXL375 | 7C, 7D | degraded (uncalibrated) | x offset **+0.78 / +0.75 g**; the scale is fine, and with the offset removed it matches the LSM6DSO32 to 0.5 % | six-position calibration; subtract the offset in firmware |
| recorder link | 7C, 7D | degraded | **~9–10 % of rows lost on battery** (97–100 % arrive on USB). The first row only arrives 28–31 s after boot, losing the headers. The tails are lost at power-off | per the recorder plan; add a per-row sequence counter |
| VL53L4CX laser | 7C, 7D | **untested** for its job | the part works, but the 2 cm it read on the pad was **the launch rod**: it vanished at rod exit with the stack intact. 47 % of rows are stale repeats | a 0–4 m drop test in glide configuration; set period = timing budget |
| INA226 ALERT; separation switch; servos in flight | 7D (7C) | untested | ALERT never exercised. The pin reads nested/separated correctly, but bench opens of 20–292 ms got through a 20 ms debounce. Servos are fine on bench and pad (117 of 117 steps) but never moved in flight | > 3 A pulse test; debounce ≥ 50 ms and log the level; servo jig after the crash |
| GNSS on the main board | 7E, 7F | untested | only the link is proven, and on v1.0 the module sits on 3V3 with no filter | the sky test below |
| LSM6DSO32 | 7C, 7D | **working** | pad \|a\| 0.99 / 1.01 g; gyro bias < 1 °/s; peaks 8.25 g and 1595 °/s, no rails | six-position calibration; a rate jig for 7D's x axis (~2 % scale suspicion) |
| ICP-10111, BMP280 | 7C, 7D | working (relative height) | σ 1.2 / 1.8 Pa; they agree within ±1–2 m in flight. The absolute BMP280−ICP split is 317 Pa (7C) and stepped to 113 Pa (7D) | re-zero ≤ 1 min before launch (weather moves it 0.1–0.2 m/min) |
| INA226, health | 7D (7C) | working | 3.9–4.1 V, 0.5–0.7 A; MCU 34 → 42 °C on the pad. The 10 Hz poll misses servo peaks | — |
| GNSS NMEA link | 7C, 7D | working | 25 of 25 probes ok; GGA cadence 1.0001 s; the "GGA lost" events are board stalls, not the receiver | — |
| BMI323, BMP581 (nose logger) | TMS-7 | working | 4891 of 4892 frames at exactly 20 ms; peak 14.8 g (93 % of range); the BMP581 is valid only when slow | — |

### GNSS: is a bigger antenna needed?

**Probably not as the main cause, and the logs can't settle it.**
- **The receivers were alive and configured** (10 Hz RMC, 1 Hz GGA) on two airframes, outdoors, for 14.7
  and 23.6 min, yet reported **zero satellites used** throughout.
- **That's too large a gap for a small antenna.** A healthy ATGM336H even on a small patch fixes cold in
  0.5–2 min. A bigger patch buys 2–6 dB; failing for 20+ minutes needs ~10–15 dB missing, or no signal at
  all.
- **The firmware turns off the evidence.** It disables GSV and the antenna-status message, so "no
  signal", "weak signal" and "jammed" look identical in these logs.

The likely causes, ranked:
1. **The installation:** interference shared by both airframes. The patch sits on the same carrier and rail as
   the Luckfox and camera, with WiFi on in both field sessions.
2. **No signal reaching the receiver:** an antenna mismatch, a connector, or a dead antenna.
3. **Orientation:** the patch faces up in glide, so on the rod it faced the horizon. It costs a few dB, not
   enough alone.
4. **A small antenna.** It may contribute, but not alone.

**The decisive test (~30 min, no firmware):**
1. Take a module, with its own antenna, off the airframe and onto a USB-UART at 9600 baud. After a power
   cycle it outputs GSV and `$GPTXT ANTENNA …` again.
2. Leave it 10 min under open sky with nothing else powered.
3. Read the result:
   - **Fix in < 2 min, strongest four satellites ≥ 38 dB-Hz:** the module and antenna are fine, and the
     installation is the problem. Re-add video, WiFi, the airframe and the vertical attitude one at a time.
   - **Nothing tracked, or ANTENNA OPEN/SHORT:** replace the antenna or the module.
   - **Satellites tracked but weak (< 30 dB-Hz bare):** only then a bigger, active antenna.

**Next flight:** log GSV every ~5 s (satellites in view and tracked, the best four C/N0) and the antenna
status. Record events for the first UTC time and the first fix. Make the pre-rod check require a fix, or at
least 4 satellites at ≥ 30 dB-Hz.

### Which sensor to trust for what

- **Attitude:** the LSM6DSO32 gyro, integrated from the pad's gravity vector. The BNO055 only in slow phases,
  once its mount is corrected.
- **Speed in boost and coast:** inertial (LSM6DSO32, with the ADXL375 as the high-g backup once its offset
  is removed). The two agree to 0.5 %.
- **Airspeed:** the pitot in the glide only (q < ~500 Pa, angle of attack < 15°), after its span
  calibration. Ignore it in boost and coast.
- **Height:** the ICP-10111 first, the BMP280 as backup, both re-zeroed just before launch. Trust any baro
  only when slow. There is no GNSS altitude yet, and the laser is unvalidated.
- **Data hygiene:** ~10 % of rows are missing on battery, and spliced rows remain (one ADXL row parses as
  2050 g). Always gate on range.

### Proposals for the post-launch cycle

Input to sit beside the operator's own ideas, most valuable first:

1. **Stack stability**, the root of 7C's loop and 7D's bent path.
   - Measure the CG with the motor, and swing-test it in both planes. Fly only at ≥ +1 caliber of effective
     margin at 20° angle of attack (7C was ~0.25, 7D ~0.71 nominal).
   - Bigger booster fins, a fairing over the folded wings, or nose ballast.
   - Jig the fins to 0 ± 0.25° and key the joint against roll.
   - A 2–3 m rail, or a wind limit.
2. **Separation**, which failed on 7D and 7E.
   - A board-fired release on a vertical speed ≤ 0, with the motor charge only as a backstop: no fixed
     delay fits apogee 2.9–5.1 s after burnout.
   - Proven energy at full glider mass after a real burn; motor retention rated above the ejection force.
   - Log the pin level; never engage control on an unseparated stack.
3. **GNSS:** the sky test above before buying anything, then GSV logging. If no fix can be had reliably,
   decide deliberately what guidance does without it: dead reckoning from a surveyed pad, which then puts
   the pitot and magnetometer on the critical path.
4. **The recorder:** the operator's plan (no video, ~2000 µF hold-up capacitor, a new file organization),
   accepted with a power-pull test. Add a per-row sequence counter so losses are countable.
5. **Firmware.**
   - A non-wrapping timestamp; set the clock so sessions carry the date.
   - Fix the OOM forecast.
   - Arm the apogee detector on measured burnout.
   - An "ejection kick" event.
   - Make the BNO055 probe fail on all-zero data.
   - Debounce the separation pin at ≥ 50 ms.
6. **Calibration:**
   - six-position tumbles (LSM6DSO32, ADXL375);
   - the BNO055 mount rotation and CALIB_STAT;
   - the pitot span with the flight tubing, plus a static port;
   - a gyro rate jig;
   - a 10-minute baro side-by-side against an airport QNH;
   - a laser drop test.
7. **The simulator:**
   - the measured F15 (32 N spike, 42 N·s, burnout 2.6 s, ejection 5.8 s after);
   - a 6-degree-of-freedom boost with wind, a q-proportional roll asymmetry and margin per plane;
   - replay regressions of 7C, 7D and TMS-7.
8. **Re-validating the reused hardware:** the checklists in the 7C and 7D READMEs, against the pad medians recorded there.
9. **Recovery:** inspect TMS-7's chute (it degraded below 150 m). Keep flying the nose logger as a black box.

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
