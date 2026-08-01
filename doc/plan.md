# Plan

Required hardware and the phased development roadmap. Architecture lives in
[`../doc/specs/`](../doc/specs/); working conventions in [`skills.md`](skills.md).

## Status

- **Phase 0 — done.** `config`, `task`+`controller`, `cc_protocol`, `recorder` (SPSC rings +
  `Telemetry`) in `src/glider/`, all MicroPython, on-board tested (`make test`).
- **Phase 1 board side — done.** `wifi` (STA) and `cc_client` (`Dispatcher`/`Client` +
  `create_dispatcher` answering whoami/ping/health/state/report/get-config/set-config/
  reset-config/reboot). 8/8 on-board tests.
- **Phase 2 — done (bench-verified).** All sensors (ADXL375/BNO055/BMP280/ICP-10111/ATGM336H/
  VL53L4CX) → `databoard` read-time fusion, per-sensor telemetry, board health, separation switch,
  `probe` self-tests, servos, and CC log streaming with auto hub discovery. 26/26 board + 3/3
  control. The **LSM6DSO32 6-DoF IMU is now integrated** (±32 g accel as the primary channel + the
  sole gyro `rate`, on SPI1, on-board verified). Deferred (not blocking): outdoor GNSS fix.
- **Control hub — done.** `src/control/` split into `board.py` (Board, 10 s exchange timeout),
  `server.py` (the hub: board listener + ~2 s heartbeat + telnet operator console, drop-in
  `commands/`, `all`-only broadcast) and `main.py` (CLI: `--host 0.0.0.0` / `--port` / `--help`),
  plus the HTTP/SSE dashboard (`web.py`). Per-file host tests (`test_board`, `test_server`).
  Remaining polish: draft config + save/reboot from the UI. **Then** the Controller
  **bring-up wiring** (connect → time-sync → start tasks), two-sided so it wants Control live first.
  The Recorder is now wired into the task graph by a thin adapter (`@task.activity('recorder')` in
  `tasks/recorder.py`); the `recorder` component (bus uart:1) makes the Controller create + supervise
  its drain loop. ✅
- **Phase 3 — done.** Active control: surface `mixer`, PID stabilization `flight` loop, stage
  `sequencer`, per-phase behaviour, `watchdog` + heartbeat, arming/ground-test safety. 36/36 board.
- **Phase 4 — in progress.** *Done:* landing-zone `navigation` (heading-to-home, 3 GPS-degrading tiers)
  + **bank-to-turn energy management** (the airframe was over-*ranging* the 100 m zone ~456 m downrange;
  it now orbits the target to bleed altitude, contained to ≤ ~224 m from the pad); the **LSM6DSO32
  6-DoF IMU** integrated + verified; **boot hardening** (Wi-Fi never blocks boot — radio comes up lazily,
  retries until ignition; HPRC launch default so a fresh board is field-ready); and a **bring-up /
  diagnostics layer** — per-device `diagnose()` (wire-level fault when setup fails: chip-select dead /
  MISO floating / wrong device, surfaced by `verify`/`probe`) and per-board **bus-frequency `calibrate`**
  (CC sweeps each i2c/spi bus to its stable ceiling and names the limiting device — bench: i2c 400k→1 MHz,
  spi 5→16 MHz, ADXL375-bound); the real-airframe **launch-g fix** (config shipped 3.0 g but the v2 F15
  stack boosts at ~2.84 g → sat in SETTING) + **apogee-timed deploy** (baro peak-detect, mass/motor-
  independent, burnout timeout as fallback); a **fixed-point (`fixnum`) control path** (centidegree
  integer PID + attitude + driver internals — measured 0 B/step in flight); and the **gyro-rate PID D
  term** — the LSM6DSO32 `rate` now damps every axis (derivative-on-measurement), so the paid-for gyro is
  finally consumed end-to-end (driver → databoard → PID, and rendered in flight reports); the
  **structural refactoring** (7/02–7/03, roadmap below: `guidance.py` + `governor.py` extracted as
  host-runnable unit-tested leaves, the mixer FUSED into the servo write via `bind()`/`actuate()`,
  `virtual_flight` driving the REAL law — validated by the
  [TMS-7-guiding_refactoring](sims/TMS-7-guiding_refactoring/) HITL set: exact trajectory parity, real
  control-path leak 15.0 KB/s settled → ~36 min OOM, ~10–16 % less servo work); the **attitude-gated
  GNSS corrector** (`gnss_steep_pitch` — near-vertical flight must not blend 2D ground speed into the
  airspeed estimate; HITL-neutral, unit-proven); `flight_kpi.py` (guidance-effectiveness KPIs, the
  per-capture-set regression practice); and `config_default` documenting **every runtime knob** where
  it is consumed. *Open:* flight-log review, optional **viperization** of the now-integer hot path
  (DEFERRED on `bench_flight` evidence: a control step is ~4.5% of the 100 Hz budget and no single
  slice dominates — 3× `pid.step` ~20%, the fused `mixer.actuate` ~15%, `navigation.steer` throttled
  to ~23 µs amortized/step; `pid.step` can't be `@viper` wholesale — object attrs + `None` sentinels),
  and Phase 5 prep.
- **Simulation & performance — done.** On-board **HITL simulator** (closed-loop, no production-code
  changes) + host **virtual-flight** tool with interactive HTML/SVG reports (`doc/sims/TMS-7/`:
  noise/wind/spike sweeps + corner cases); **perf cluster** (nav-heading cache, zero-alloc
  mixer, GC-disabled-in-flight, sleeping CPU-load probe → **7.2 W → 3.6 W**); spike injection;
  the TMS-7 flight envelope + airframe notes in `coludo.md`. Bench re-run + numbers refreshed.
- **Phase 5 — field testing (next).** Staged ladder: maket walk-test → telemetry launch → powered
  launch, likely on the wider **TMS-8** airframe. See below.
- **Firmware / toolchain — PINNED at `v1.29.0-preview.414.g533a154c8a`** for Phase 5 (repo `mpy-cross`
  gate at `preview.417`, same mpy v6.3; `deploy.sh` ships `.py`, the board compiles on-device). This
  build already carries the **PSRAM speed-up** — heap memcpy 11.8 → **33.5 MB/s (2.85×)**
  (`doc/benches/WaveShare_esp32p4-micropython-findings.md`) — which cascades into every PSRAM-bound path
  (slice-assign, the Recorder rings). **Do not chase newer previews.** Reviewed 414→434 (20 commits): all
  but one are irrelevant to us (Seeed boards / Wi-Fi CSI / `machine.SDCard` — no SD here / other ports /
  formatting); the `int`/`struct` overflow change is a no-op in V1 (checks off outside unittest); the one
  delta that touches us flips the **esp32 `i2c_master` driver default at IDF ≥ 5.5.2** — a regression risk
  to the four `i2c:0` sensors (bmp280/bno055/icp10111/vl53l4cx) with **no benefit**. Only bump for a
  concrete upstream fix, and re-qualify every I2C sensor on hardware (`verify`/`probe`) before trusting it.

## Phase 5 — field testing

The flight stack is shipped, simulated and tuned. Phase 5 is a staged ladder from handheld to powered
flight on real hardware (extends the flight-test-plan):

1. **Maket walk-test** — the assembled *maket* (prototype) board on **battery + the Luckfox recorder**,
   carried by hand outdoors:
   - **power-up + register to CC** (Wi-Fi join / hub discovery), health + telemetry flowing;
   - **`launchpad` config outdoors** — real GNSS fix at the pad, mission zone loaded;
   - **walk-around "flight simulation"** — flight loop in GLIDING (manual stage hold), armed: carry it
     like a glider and watch the **fins react** to the live attitude + landing-zone heading, Luckfox
     capturing telemetry for a `flight_report`. No 58 m/s sprint required ) — the loop flies at
     walking pace;
   - **trip "separation" by hand** — confirm the switch drives `BOOSTING → GLIDING` and the loop engages.
2. **Telemetry launch** — passive electronics-only flight (no SG90 actuation): fly E16/F15, record the
   full sensor + stage timeline to the Luckfox, review with `flight_report` — proves the data pipeline
   and stage detection under a real boost → coast → descent.
3. **Powered launch with active control** — enable the flight loop; first controlled glides.

**Airframe:** the real flights likely move to **TMS-8** — a wider, more triangular glider (~2 cm wider
span) for more wing area / stability than the TMS-7 modelled here. When it is printed, the sim's flight
envelope + landing-zone numbers re-derive from the TMS-8 masses/geometry (`coludo.md`, `doc/sims/`).

Supporting precision + tooling (continue alongside / from Phase 4):

- **Nail the landing strip** — bank-to-turn lands ~85 m from the 40 m-wide strip centre (good sensors).
  **The cause recorded here was WRONG and is corrected (7/31).** It said the glider "drifts off on short
  final because LANDING goes wings-level". Measured from a board capture, it does the opposite — it is
  pinned at the FULL endgame bank right down to touchdown and only levels after it is on the ground:

  | t (s) | elevation | roll setpoint | heading error |
  |---|---|---|---|
  | 44.5 | 71.8 m | −30° | −25° |
  | 48.5 | 41.5 m | **−60°** | **−162°** |
  | 51.5 | 17.4 m | **−60°** | −110° |
  | 53.5 | 1.4 m | **−60°** | −64° |

  So the endgame spiral **tightens to maximum bank and never converges**: the glider is still mid-turn
  when the altitude runs out, and touchdown lands at whatever phase of the spiral it reached. That is
  phase luck — the same failure the old racetrack law had, at a smaller radius.

  The fix is therefore NOT more authority on final (there is already maximum authority). Two candidates
  to evaluate: **roll out to wings-level and hold the final heading** once inside a commit point, or
  **floor the spiral radius** at what the remaining altitude can complete (`R_min` is already computed
  for the airspeed-gated bank, so the input exists). Either needs a sim sweep plus a `flight_kpi`
  regression against the existing capture set before it goes near the airframe.
- **High-noise robustness** — **RE-SCOPED, THEN DONE (7/31, `doc/sims/TMS-7-noise_tolerance`).** The
  hypothesis here was that "the bank loop reads attitude, so heavy noise degrades the orbit", wanting a
  filter or rate-limit on the steering input. Measured per channel at 100 % noise, that is wrong:
  attitude, gyro rate and heading at twenty times nominal move the landing by **at most 5 m**. The
  steering input needed no protection at all. The fragility was in the **sequencer**, and both halves
  are now fixed and shared with the host sim through `commons`:
  - **Launch** — a leaky dwell (`commons.dwell_step`): the condition must hold `launch_ms` of NET time,
    and a dip drains the credit instead of resetting it. 100 % accel noise went from *never launching*
    to flying **identically to the clean baseline** (268 m apogee, 48.9 s). Keeping the TIME unit was
    the whole trick — a leaky *sample* count is equally noise-tolerant but silently turns `launch_ms`
    into a sample count, so the trigger drifts with tick rate; `test_sequencer` caught that.
  - **Apogee** — the peak is a running maximum, so one high sample was latched forever and every later
    reading judged against it. `commons.apogee_step` now IIR-smooths elevation (1/4) before comparing.
    Apogee improves at every noise level (15 %: 180→215 m, 50 %: 168→178 m, true 268 m). Heavier
    smoothing was measured and rejected: no gain noisy, ~20 m of lag clean.
  - The **100 % altitude case stays at 177 m and that is correct** — `noisy` scales by `abs(value)+1`,
    so 100 % makes the baro uniform-random over `[0, 2×altitude]`, carrying zero information. What flies
    it then is the burnout timeout, which is the fallback working. That case tests redundancy, not
    filtering.
  - Also fixed the study's own blind spot: `virtual_flight` had a private copy of launch detect, so the
    original sweep graded the host, not the board. **A sim that reimplements the flight code grades
    itself.**
- **GNSS miss/absence tolerance** — **DONE (7/31).** The receiver is *expected* to lose lock through
  the boost (high-g, vibration, antenna shadow) and may never reacquire inside a <60 s flight, so this
  is a design case rather than an error path. The no-fix branch used to steer the pad→target bearing
  forever, ignoring that the glider had moved — the 720 m the fault matrix priced. It now **dead-reckons**
  (`guidance._reckon`): advance the last fix by airspeed × heading plus the last wind estimate, every
  input of which survives a dead GNSS. Tiers are now 1 live fix → 2 dead reckoning → 3 launch point
  (never locked at all) → 4 blind heading hold. `flight.csv` carries a `reckoning` column so the
  operator sees it. Steps over 0.5 s are skipped rather than integrated in one jump. Frozen wind is the
  dominant residual error and is bounded: a 2 m/s estimate error costs ~80 m over 40 s.
  **MEASURED on the board** (`captures/gnss`, F15 270 g): a 60 s blackout covering essentially the whole
  glide costs **30 m** — 122 m clean vs 152 m dead, with 368 of 522 flight rows flying on dead
  reckoning — against the **720 m** the earlier fault matrix priced for the same failure.
  Also: **board floats are single-precision**, so advancing a latitude near 48° by a 0.2 m step changes
  it by exactly nothing (~0.4 m ULP). The reckoner accumulates metres and converts once; anything that
  integrates small increments into a large absolute coordinate on this hardware is silently wrong, and
  worst precisely when the increments are smallest.
  Per-sentence **RMC/GGA outage events** now go to the durable `gnss_events.csv`, and GGA's quality /
  satellites / HDOP / altitude — parsed but never recorded — get their own `gnss_gga.csv` stream. They
  fail separately (a receiver with almanac but no fix keeps emitting GGA at quality 0 while RMC goes
  void), and since dead reckoning makes an outage nearly invisible in the trajectory, these events are
  the only record that the receiver was dead.
  **Still to measure:** GNSS position is noised in neither sim, so a *noisy* (as opposed to absent) fix
  remains untested.
- **Float allocation is the leak, and the 100 Hz channels are where it lives** — **PARTLY DONE
  (2026-08-01).** First target closed: `bno055` was still building two `to_str` strings per sample,
  MEASURED at 170 B — the single largest piece of that driver's sample, and formatting rather than
  measurement. It is the same fix `lsm6dso32` already took for its gyro columns and this driver simply
  never got. Removing it: **246.0 → 240.3 KB/s, OOM 130 → 133 s**. Both sims and `flight_report` were
  moved to raw centidegrees with it, so board and host still agree — that divergence is exactly how the
  noise study came to grade the wrong code. The two remaining `to_str` callers (`sdp810.probe`,
  `attitude.inspect`) are cold and correct as they are.
  **Ground truth, measured per driver against a 240.3 KB/s baseline:** `imu_lsm6dso32` **50.9 KB/s**,
  `imu_bno055` **44.4 KB/s** — together **40 % of the whole leak**. `accel_adxl375` not yet measured
  (the CDC wedged); it runs at the same 100 Hz and should be assumed comparable until measured.

  **CORRECTION — the telemetry row is NOT the cost.** An earlier revision of this entry claimed it was,
  by multiplying the measured row cost by 100 Hz. That was wrong: the recorder already applies a GLOBAL
  25 Hz decimation (`telemetry_us: 40000`, itself already halved from 50 Hz as "the cheapest leak
  reduction available"), which every stream inherits unless it sets its own. At 25 Hz a 160 B row is
  **4 KB/s** — under 10 % of `imu_lsm6dso32`'s 50.9 KB/s. Global decimation is therefore SPENT as a
  lever; it cannot be applied twice.

  **The cost is the 100 Hz SAMPLE LOOP**, which runs at full rate regardless of telemetry decimation.
  For `lsm6dso32` the accountable per-sample pieces are two 3-element slices (96 B), the sample tuple
  (~96 B with floats) and two databoard pushes (~64 B) = ~256 B x 100 Hz = **~26 KB/s**, leaving ~25 KB/s
  unaccounted — the prime suspect being the I2C `readfrom` allocating a fresh bytes buffer per read, at
  100 Hz. **Measure that next** (`readinto` into a preallocated buffer is the standard fix), and measure
  `accel_adxl375`, before touching the row encoding at all.

  The row encodings below stay priced and correct as measurements, but they are a SMALL lever at 25 Hz
  and should not be the next move:

  | row encoding | bytes | us | note |
  |---|---|---|---|
  | `%`-format decimal (today) | 144 | 36.0 | |
  | raw sensor COUNTS as `%d` | 128 | 35.7 | free — the driver already holds them |
  | `struct.pack` + `hexlify` | 112 | 23.7 | better on BOTH axes; needs a header marker |
  | raw binary `struct.pack` | 48 | 87.6 | **DO NOT** — see below |
  | repack float→int, then `%d` | 208 | 67 | **net LOSS**, the conversion costs more than it saves |

  Three conclusions worth keeping. **Converting floats to ints for telemetry loses** (`from_float` is
  80 B for three values against 16 B saved) — the win only exists when the ints come FREE, i.e. the
  driver logs the raw counts it already read and the host applies the scale, which is the `to_str`
  lesson one level up. **Raw binary is not viable**: the recorder's wire format is line-framed
  (`@session_file@...\n`), so packed bytes containing 0x0A or `@` would silently corrupt row
  boundaries — hex keeps it ASCII-safe, which is why it is the right choice and not a compromise.
  And a per-column encoding marker in the header (operator's suggestion, e.g. `name+` = hex-encoded)
  makes the streams self-describing so the host decoder needs no per-file knowledge.

  Next step: `accel_adxl375`'s number, then raw-counts + hex on the three 100 Hz streams. Every analysis
  tool reads these columns, so board and host must move in the SAME commit — a column meaning different
  things on the two sides is how the noise study came to grade the wrong code. Measured on the board: a boxed float is exactly **16 B**, an int
  operation allocates **0**, floats run ~22 % slower, and with GC off in flight none of it is transient.
  Crucially `push` costs **32 B for a float and 0 for an int**, while `value()` costs ~0 for either — so
  the cost is at the PRODUCER, scaling with its rate.
  That kills the altitude/elevation migration (the baro runs at **10 Hz**: ~2.6 KB/s, OOM 130 → ~132 s,
  for a refactor across drivers/databoard/guidance/sequencer/both sims/telemetry — not worth it) and
  points at the **100 Hz channels instead: attitude, rate, accel**, where a 3-float tuple costs 64 B a
  push ≈ **6.4 KB/s per channel**, roughly 10x the payoff for the same work. `imu_bno055` alone was
  profiled at **47.9 KB/s** and has never been examined. Do that evaluation before any more of the leak
  work — same method as the baro one, which is written up in the icp10111 commit.
- **Do NOT migrate altitude/elevation to fixnum** — **CLOSED, evaluated and rejected (2026-08-01).**
  The float32 ULP problem that motivated it does not exist in this domain: it was a lat/lon DYNAMIC
  RANGE failure (48° encodes 5.34 Mm of arc), while altitude's ULP is **30 µm at 288 m**, 61 µm at
  1000 m, `elevation = altitude - ground` differences to 0.06 mm and the ground average to 0.6 mm — four
  to five orders inside the 10 cm border, against a part that resolves 8.9 cm. Speed and OOM gains are
  ~15-22 % on the affected ops and ~2 s respectively. The driver ALSO cannot go integer even in
  principle: the TDK compensation's `b` coefficient is **-4.23e12**, past MicroPython's unboxed 2^30 by
  ~4000x and past viper's 2^31 (which wraps SILENTLY) by ~2000x, so any integer form needs 64-bit
  intermediates = bignums = more allocation than the floats replaced.
- **Catapult `board.config` profiles** — **DONE (2026-08-01), `tools/make_catapult_config.py` ->
  `configs/tms7c.config`, `configs/tms7d.config`.** 7C is telemetry-only (all three servos AND the
  flight activity disabled -- the airframe flies as instrumented ballast); 7D is full active control.
  Both validate and load as `active`.
  **The firmware defaults would have FAILED on a catapult, not merely degraded.** Worked from the
  catapult's geometry (~1 m of boost, 45 deg, 15-20 m/s at release -> apogee **5.7-10.2 m** at
  **1.08-1.44 s**, boost ~11 g for ~130 ms), four separate thresholds break:
  `apogee_arm_ms` 4000 leaves the detector blind until well past apogee, so apogee is NEVER seen;
  `boost_timeout_ms` 12000 then fires the fallback long after touchdown, so the glider would spend its
  whole flight in BOOSTING with fins at the boost attitude; `launch_alt_m` 10.0 sits at or above the
  entire arc so the baro backup can never trip; and `apogee_drop_m` 5.0 is most of the arc height.
  Applied instead: arm 300 ms, timeout 1500 ms, launch_alt 3.0 m, drop 1.0 m, `launch_ms` 40 (the
  ~130 ms pulse must CONTAIN the dwell, not race it), `flight_timeout_ms` 60 s. `launch_g` stays 2.5 --
  the catapult pulls ~11 g, so that is the one threshold with margin to spare.
  Generated rather than hand-written so the derivation stays attached to the numbers, and validated
  through the firmware's own `config.validate`/`config.load` rather than trusted -- which caught a real
  bug: 7D had `flight` disabled, because the generator only ever CLEARED the flag while the default
  ships it off. A 7D built that way would have flown with no control loop.
- **`launch.config` autogen** + GPX export of telemetry. Deferred hardware: outdoor GNSS fix.

### Near-term work from `findings.txt` (quality pass)

- **Conventions** adopted (`CLAUDE.md` + `skills.md`) and **protocol reorder** done
  (`board command`, Inspectable commands). ✅
- **Tooling**: `ruff` config + `deploy.sh` (lint + `mpy-cross` + push to `/pyboard/`) +
  `ssid.creds` (gitignored Wi-Fi password). ✅
- **`recorder.py` refactor** as the golden reference module (rename to full names, async
  `StreamWriter` drain, drop `stop`/`sink`/`also`/`limit`, log-drop vs tlm-raise, `const`, type
  annotations) + finish the `_Msg` rename in `cc_protocol`; sync tests. ✅
- **Config schema reorg** (board-config.md): nested `buses: {uart:{1,2}, i2c:{0,1}, spi:{}}`, a
  `sensors:` section, no abbreviations, configs in a `configs/` subfolder. ✅ (doc + code)
- **`Inspectable` mixin** (`inspect`/`update`/`stats` + `type`/`name`) — design then adopt. ✅
- **`BoardHealth` task** — `esp32.mcu_temperature()`, `idf_heap_info()` (PSRAM-aware), idle/load,
  periodic Telemetry every ~1 s. ✅
- **`test/probe_pins.py`** — sweep pins 0..60 and UART/I2C/SPI 0..5 to auto-derive a board map;
  inspect `machine.Pin.board`. ✅ (low priority — we have a map)

### Structural refactoring roadmap (findings.md §20)

Top-down proposals S01–S07 live in `findings.md` §20; this is the agreed execution order
(value / effort), reconciled with the fixnum + gyro-D-term + adaptive-airspeed work of the last
week. Architecture is largely sound — this is targeted slimming, not a rewrite. The declined
non-goals stand (no DI container, no message bus replacing the databoard, no config micro-ORM).

1. ✅ **Slim the `Flight` loop** (7/02) — extracted **S03 `guidance.py`** (per-stage law table (**S04**,
   the sequencer pattern): boost hold, glide/landing steering with the 3 GPS tiers + nav cache,
   final-approach centreline; `GuidanceConfig` typed knobs) **and `governor.py`** (airspeed estimator +
   adaptive throttle + dive/overspeed full-rate overrides + the 1/v² mixer cap; `GovernorConfig`).
   Both are **host-runnable leaf modules** (injected databoard-style handles, `now_us` passed in,
   `commons.ticks_diff` shim) with isolated unit tests (`test_guidance.py`, `test_governor.py`).
   `Flight._step` is pure orchestration: dt → Governor → gate → Guidance → PID → actuate (374 → ~200
   lines). The scoped **S02** (typed configs) landed with it for these two. Validated on-board:
   46/46 tests + the [TMS-7-guiding_refactoring](sims/TMS-7-guiding_refactoring/) HITL set — exact
   trajectory parity vs fixnums, real control-path leak down to ~15 KB/s settled (~36 min OOM @ 100 Hz).
2. ✅ **`mixer.mix` → actuate fusion** (7/02) — `Mixer.bind(fins)` + `actuate(roll, pitch, yaw)`
   drive `set_angle` inside the mixing loop; `Flight._apply` and the per-step `.items()`/`.get()`
   pair are gone (`mix()` stays for tests/host tools).
3. ✅ **`virtual_flight.py` de-duplication** (7/02, promoted from #5) — the host tool now drives the
   REAL `guidance`/`governor`/`pid`/`mixer.actuate` through stub handles; only the sequencer stage
   machine + physics remain mirrored (they are genuinely board-bound). Touchdown parity with the
   pre-refactor tool: ~1 m on the clean F15 scenario. The governor now flies the ESTIMATED airspeed
   on the host too (board parity, was the sim's true airspeed).
4. ❌ **S06 databoard `FusionStrategy`** — DECLINED on re-scan (7/02): its premise went stale — the
   `isinstance` guard sits only in `_extrapolate`, the *degraded* no-fresh-channel path, not per-read;
   strategy classes would ADD a per-read indirection + RAM on a MicroPython hot path for pluggability
   nobody needs yet (YAGNI). Revisit only when a new fusion behaviour (median, confidence blend) is
   actually wanted.
5. **Clarity, low-effort, whenever:** **S05** split `drivers/` vs `services/` (wifi/bluetooth) vs
   `bus/` (i2cbus/spibus); **S07** cluster `commons.py` along the now-natural fixnum-int vs float seam;
   **S02** typed config objects for the remaining tasks. Deliberately parked before the flight tests —
   pure file churn.
6. **S01 instance-ize** Databoard + Recorder, merge Inspector into Controller — highest impact + effort;
   defer until parallel testability or a hot-spare / HITL-in-HIL need forces it. **Not pre-flight.**

### Feasible next work — control / stability / hardening (assessed 7/03)

The structural phase is closed (roadmap above); remaining risk is physical calibration + field
robustness. Ranked by value ÷ effort, glider and Control side together. None started — pick per
appetite; the KPI regression practice (`flight_kpi.py` vs the previous capture set) gates every
control change.

**Pre-flight hardening (small, do before the first field day):**

1. ✅ **Flight-readiness gate in `verify`** (7/06) — `cc_client._readiness()`: the field-dangerous
   CONFIG states no device probe can see, reported by `verify` as a separate `ready`/`readiness`
   verdict (hardware `pass` unchanged — a bench session is healthy but not flight-ready): watchdog
   disabled, flight task disabled or zero/unset gains per axis, `fin_limit_multiplier` ≠ 1, no
   landing zone AND no CC-less sites to select one from. Zone-beyond-range and launch-position
   checks were already `mission.probe` (runs inside the same `verify`). The radios-enabled hazard
   was retired structurally by #2. Pairs with the `launch.config` autogen below.
2. ✅ **Stage-aware radio quiescence** (7/04) — wifi holds reconnects from BOOSTING through LANDING
   (resumes at DONE), so the airborne phase stays allocation-free even with the radios configured on.
3. ✅ **Launch-point auto-capture on `arm`** (7/06) — `mission.freeze_launch()` from
   `Controller.arm()` (both the CC `arm` and field auto-arm paths): the live GNSS fix is pinned as
   the persistent launch point at the last on-the-pad moment, so the tier-2 open-loop heading (and
   the warm-start crumb's `launch` field) survives a mid-flight fix loss. Operator-set positions
   are never overwritten; no fix → the live-fallthrough behaviour is unchanged.
3a. ✅ **In-flight OOM soak** (7/06, `tools/oom_soak.py` — wishes 7/06 #2) — a real mid-glide OOM
   forced by heap ballast (566 KB free at GLIDING entry, ~140 KB/s GC-off burn with the HITL sim's
   churn on top of the control path). Finding: at exhaustion the asyncio runtime dies wholesale —
   watchdog task AND crash→neutral included — the fins freeze at the last commanded deflection for
   ~1.4 s, then the starved hardware `machine.WDT` panics the chip (`reset_cause 3`); main.py ran
   the five-signal gate against the genuine WDT cause, refused on the bench (separation nested),
   cleared the crumb and rejoined wifi + CC. Full chain spec'd in coludo.md "In-flight reboot".
3a'. ✅ **Memory rescue + OOM/landing forecast** (7/06, `board_health` — user proposal, physics
   trigger per user redesign) — collect when memory dies before the flight is safely over:
   predicted `oom_s` < 2 × `land_s` (memory-decay vs elevation-decay slopes, all-integer EMAs;
   no descent trend yet → no rescue), with PROVEN safe altitude
   (elevation > `rescue_agl_m` 10 m ≈ 2× the landing gate), BOOSTING/GLIDING only. The collect
   is bracketed by watchdog `kick()`s. Both forecasts ride health.csv + `inspect health`.
   Measured: collects 65–260 ms on a mostly-free heap (the real anomaly case) but SECONDS on a
   ballast-full one — so `wdt_timeout_ms` stays 1000 (500 killed the rescue in HITL; the fast
   stall detect is `stall_ms` 500, independent). VALIDATED on-board: the soak that hard-panicked
   the board now lands (8 rescues, sawtooth in mem_free, stand-down at LANDING, DONE).
   Defence-in-depth stack: rescue → WDT reset → warm start → the 3b hardware supervisor below.
3b. ❌ **HARDWARE power-cycle supervisor — DECLINED (7/09, user).** The soak proved a WDT chip reset
   does NOT clear the ICP-10111 latch-up (only a rail power cycle does), which motivated an external
   heartbeat-watchdog + P-FET rail-cut supervisor. Not necessary: the existing defense-in-depth already
   covers it — the **memory rescue** (`board_health`) catches the majority (pre-OOM), the **HW WDT +
   warm start** recover a hang/OOM, and critically **bmp280 is a hardware backup for the icp10111** (the
   databoard fuses baro altitude by priority), so a single-baro latch-up loses nothing — the glide
   continues on bmp280 + the laser AGL. A rail-cut supervisor only clears that one latch-up, which the
   sensor redundancy already tolerates, so the added part + its two design tensions (boot-gap re-arm,
   the RTC dying on a power cut) are not worth carrying. Escalation stays: memory rescue → WDT reset +
   warm start → sensor redundancy.

**Control & stability (medium, sim-validated before the board):**

4. ✅ **Attitude redundancy** (7/07) — `tasks/attitude.py`: a complementary-filter backup deriving
   (heading, roll, pitch) from the LSM6DSO32 gyro `rate` + accel gravity vector, published at
   **priority 1** so the databoard's timeout-handoff swaps to it the moment the BNO055 (priority 0)
   stops — no `flight.py` change. Mirrors the BNO055 while it is the fused source (warm, zero math),
   free-runs only once lost. All-integer: roll/pitch from an **integer CORDIC `atan2` + `isqrt` in
   `fixed.py`** (viper, ~0.17° over 12 iterations, zero float boxed); the only float is the heading
   the channel format requires. The accel gravity correction is gated OFF in turns (`turn_gate` on
   yaw rate) — in a coordinated turn the accel points down the body axis (looks level at any bank),
   so the gyro carries the bank and the accel only re-anchors in straight-ish flight. Heading (yaw)
   gyro-integrates (no magnetometer) but is pulled toward the **GNSS ground-track** (`course`) when
   moving — an absolute reference that bounds the drift (7/08 #3); roll/pitch stay solid. **Validated
   closed-loop on the board** (`tools/attitude_soak.py`: drop the sim attitude mid-glide): handover is
   seamless, the backup tracks truth to ~1° roll (×1000) / ~1.8° roll (×100 fixnum) / ≤0.7° pitch
   through the loiter, and the flight reaches DONE flown entirely on the backup. The single biggest
   stability win on the table — done.
5. ✅ **Steering noise filter** (7/06) — `guidance._filter_error()`: an all-integer EMA on the
   heading error (state ×16 fixed, `steer_filter_shift` 3 = alpha 1/8, τ ≈ 80 ms @ 100 Hz; a
   > 90° jump — an overfly flip, a law handover — resets the state so steering follows at once;
   0 disables). Zero-alloc, GC-off safe. Validated on the calm host sweep: **fin travel −56 %**
   at 25 % noise (66 217 → 29 419 deg over the F15 flight) and E16 in-zone at both 5 % and 25 %
   noise (16/17 m); calm median 31 m ≈ the ≤ 30 m target (residual is run variance — field
   calibration territory).
6. **Glide energy management — the fly-long behaviour (tuned 7/06; program of 7/05).** Objectives
   in strict priority order (doc/specs/coludo.md "Gliding"): **① fly as long as possible, ② land
   in-zone, ③ land near the midpoint**. Steps 1–2 DONE: six tuning iterations on the 24-case host
   sweep replaced the racetrack (steer-at-point + bank cap: 184 m leg swings, 129 m median,
   phase-luck landings) with the **loiter orbit + endgame spiral** (capture the tangent circle at
   R 30 m within 120 m of the centre; below 50 m elevation the radius scales with remaining
   altitude and `land_bank_gain` 3.0 unlocks the full 45° bank → R_min ~20 m). Results: **calm/5 %
   in-zone at 17–18 m for BOTH motors** (E16 included); ① never regressed (121–148 % of the polar
   ceiling in every case, every iteration). Acceptance scorecard: calm/5 % ✅; calm/25 % at
   35–37 m vs the ≤ 30 m target — closed by item 5's steering filter (7/06: calm median 31 m,
   E16 in-zone at 25 % noise, fin travel −56 %); wind ≥ 6 m/s physics-bounded (launch wind limit
   rule, coludo.md). ✅ **Step 3 on-board confirmation DONE (7/07, `doc/sims/TMS-7-attitude`)** —
   the tuned loiter law flown on the board, in-zone all four (E16/F15 × full/light, miss 52–61 m);
   provenance caveat: board misses run 52–61 m vs the host's 17–18 m. **Root cause found (7/08):**
   the miss is almost entirely ALONG the strip's long axis (N–S tight ±2–20 m), because the endgame
   spiral commands a radius below the physical **minimum turn radius** (~20 m at the 45° bank limit,
   `v²/(g·tanφ)`) so it cannot collapse tighter than ~20 m, and the final-approach centreline tracker
   then lands the glider along the strip. NOT a nav-recompute-rate issue (a faster endgame
   `nav_period` was measured — no change, reverted). Objective #2 (in-zone) is met; tightening #3
   (midpoint) needs a final-approach redesign (aim at the midpoint) — a scoped future control task.
   **For the CC-less case this is moot (7/08):** the fallback zone is now a GENEROUS 100 × 90 m box
   the spiral just lands INSIDE (validated: 31.8 m from centre, in-box) — objective #2 over #3, no
   tight midpoint needed. Remaining: **step 5 the field trim procedure (FIELD-GATED**).
7. ✅ **Reachability telemetry** (7/08) — `guidance.reachability(glide_ratio, wind_e, wind_n, airspeed)`:
   reach = `glide_ratio` × elevation, WIND-ADJUSTED by the component along the bearing to target
   (`reach × (1 + w_along/airspeed)`; tailwind extends, headwind shrinks), vs the distance from the live
   fix to the zone target → {reachable, margin_m, distance_m, headwind_mps}, on the flight-panel
   heartbeat via `flight.vitals()`. The dashboard shows **zone ✓ +Nm** (green) / **zone ✗ −Nm** (red) so
   the operator sees an unreachable zone early — groundwork for a deliberate land-short decision.
   `glide_ratio` config (nominal L/D, default 3.0, field-tuned from real glide telemetry). Plus
   **degraded-mode annunciation**: `health.degraded` gathers the non-nominal states (attitude-backup
   active, memory-rescued, warm-started, CC-less fallback) into one ⚠ signal in the panel, so a glance
   shows the board is not flying clean. Board + control suites.
7b. ✅ **Wind estimation** (7/09, wishes #6 — `wind.py`) — the MINIMAL wind triangle (`wind =
   ground_velocity − airspeed × heading`, EMA-smoothed), fed once per NEW GNSS fix off
   `databoard.Parameter.stamp()` (self-tunes to the receiver rate 1/5/25 Hz, wrap-safe — no magic
   period; replaces a `ticks_diff(start, 0)` throttle that silently died once board uptime passed 2^29 µs
   ~9 min). On the flight panel + feeding the reachability headwind. HITL-validated: tracks a 6 m/s
   crosswind to ~0.5 m/s in slow/wide/straight flight (collapses in the tight loiter — EMA smear — but
   that is off both use points). The airspeed-free min/max-ground-speed method was built then STRIPPED
   as premature. Also fixed sim fidelity: the HITL GNSS `speed` was 3D airspeed, now 2D ground speed WITH
   wind (`sim_model.ground_speed()`). **Final-approach CRAB DECLINED on data**: wind_soak touchdown miss
   is IDENTICAL closest-approach with/without wind (~15 m — the position feedback already rejects steady
   wind); the +15 m touchdown growth is free-drift in the last low-authority descent (≈ wind × descent
   time), which a controlled-approach crab cannot fix. Board suite green (49/49).

**Spec'd and IMPLEMENTED 7/04 (doc/specs/coludo.md; 47/47 on-board):**

8. ✅ **Warm-restart recovery** — `warmstart.py`: NVS breadcrumb (flag i32 + one JSON blob) dropped
   at BOOSTING entry for armed flights, cleared at DONE; `main._restore_flight()` restores
   GLIDING + arm + baro-ground rebase + GC-off behind the FIVE-signal gate (crumb, separation
   latch, baro ≥ pad+15 m, reset-cause WDT/SOFT/HARD, crumb age 0–10 min by the surviving RTC).
   **Still owed (FIELD-GATED):** the mid-glide restore ran on-board in the TMS-7-warm_reboot HITL
   set (both reboot flights recovered through the five-signal gate); what remains needs the built
   airframe — the PHYSICAL separation-switch latch held low through a real reset, on the bench rig
   before the first field day.
9. ✅ **CC-less field operation** — `mission.select_site`/`fallback_zone` (nearest launch.config
   site within 200 m; else the spiral-landing zone +50 m at the clear-sector bearing),
   `tasks/field.py` (site-once + dwell-gated `auto_arm`, disabled by default), wifi
   `policy auto|disabled` + round-robin `networks`, quiescent BOOSTING→LANDING and RESUMING at
   DONE. Absorbed items ② and ③ above.

**Control (CC) side:**

9. ✅ **Flight-ready config autogen** (7/07, `tools/launch_config.py`) — emits a board.config that
   passes the `verify` readiness gate: watchdog ON, flight ON with gains, radios quiesced (policy
   `auto` — silent airborne, live pre-launch/post-land), fin cap 1.0. Gains are SIM-derived proposals
   (sourced from `config_hitl` so they stay in sync), so the board arms out of the box but a loud
   banner says VERIFY on the first glide. Self-verifies (validate + `_readiness`) before emitting;
   `set-config board @flight.config` pushes it. Pair with a launch.config (site/zone) for the zone
   half of the gate. Control suite covers it (6/6).
10. ✅ **Live flight panel** (7/07) — the SSE dashboard's `flight` column: armed dot + the governor's
    airspeed estimate + fin-authority cap + AGL, engaged/idle. `flight.vitals()` (airspeed + `governor.cap()`
    + active) rides the health heartbeat alongside stage/armed; `board_rows` passes it through; the
    dashboard shows the operator's go/no-go glance during the ladder tests. Board + control suites cover it.
11. ✅ **One-shot field capture pull** (7/07, `tools/flight_pull.sh`) — pulls ONE recorder session off
    the Luckfox (default: the latest), assembles the capture, and renders the SVG + HTML report + KPIs
    in one command (was hand-run per stream). Tested end-to-end against a live session.

## Memory budget — resolved (2026-07-30)

The GC-off leak had four disagreeing numbers on record. Measured by varying the sim's publish rate
across three board flights and fitting the line:

    leak = 187 + 3.32 * inject_hz   KB/s

- The **HITL sim itself costs 166 KB/s at 50 Hz — 48 %** of the board-HITL total. This independently
  reproduces the earlier "~46 % of a HITL leak" A/B *on the current build*, simulated pitot included,
  so that method is validated rather than folklore.
- **The intercept is 187 KB/s**, but that is *HITL with the sim's publishing removed*, not a real
  flight — the sim still steps float physics at 50 Hz underneath it, while HITL masks seven real
  sensors whose drivers a real flight runs. The two are not nested, so neither bounds the other.
- **Profiled per component** (`test/diag_real_leak.py`, disable-one-and-diff against a 331 KB/s
  baseline with `flight` enabled):

  | component | KB/s | share |
  |---|---|---|
  | `accel_adxl375` | **111.3** | 34 % |
  | `imu_lsm6dso32` | **63.7** | 19 % |
  | `imu_bno055` | 34.4 | 10 % |
  | `gnss` | 12.8 | 4 % |
  | `airspeed_sdp810` | 12.7 | 4 % |
  | `recorder` (drain task) | 11.7 | 4 % |
  | `attitude` | 10.9 | 3 % |
  | `baro_icp10111` | 10.5 | 3 % |

  Three interrupt/high-rate IMU paths are **63 %** of the leak. Disabling `recorder` saves only
  11.7 KB/s, so the cost is in the driver read/push paths, not the telemetry drain. `flight` measured
  ~0 because on the bench it never leaves SETTING — a real flight adds its 100 Hz control work on top,
  so treat 331 KB/s as a floor.

  **Where it goes** — measured per call, not inferred (`test/diag_alloc_hotloop.py`):

  | piece of the ADXL375 sample | B/call | rate | KB/s |
  |---|---|---|---|
  | `asyncio.wait_for_ms(flag.wait(), fallback)` | **560** | 100 Hz | **56.0** |
  | `Telemetry.push` + its rounded tuple | 272 | 50 Hz | 13.6 |
  | the `(x*s, y*s, z*s)` float tuple | 80 | 100 Hz | 8.0 |
  | `struct.unpack('<hhh', buf)` | 32 | 100 Hz | 3.2 |
  | `channel.push((x, y, z))` | **0** | 100 Hz | 0 |

  That sums to ~81 of the measured 111 KB/s, the rest being loop overhead. Two conclusions:
  `channel.push` is genuinely zero-alloc, so the databoard hot path is doing its job — and **the
  timeout wrapper is over half the cost of the worst component**. A bare `ThreadSafeFlag.wait()` is
  only **96 B**, so `wait_for_ms` adds **464 B of pure overhead per sample**.

  **The route to 150 s** (needs ~212 KB/s, i.e. -120 from 331):

  1. **Drop `wait_for_ms` from the INT path — about -46 KB/s.** Wait on the flag directly and let a
     slow (~2 Hz) kicker set it when the interrupt goes quiet, so the fallback costs one timer instead
     of a timeout object at 100 Hz. `lsm6dso32` has the same shape and does not pay it *only* because
     its INT1 is unwired and it polls; wiring INT1 in v0.2 would make it start paying unless fixed
     first.
  2. **Route LSM6DSO32 INT1 in v0.2 — recovers part of its 63.7 KB/s**, since it currently polls at
     100 Hz rather than sampling on data-ready.
  3. **Decimate the accel/gyro telemetry streams** — 272 B per row is the second cost; halving those
     stream rates is worth ~7 KB/s each and costs only plot resolution on channels sampled far above
     what the reports render.

  **Done, and re-measured (7/31).** `commons.Waiter` replaced `asyncio.wait_for_ms` in all three
  interrupt drivers (560 B/call → 48 B, doubling backoff), `to_str` came out of the LSM6DSO32
  telemetry (224 B/sample of string formatting), and `period_ms`/`fallback_ms` merged into one
  `period_us` with the interrupt branch deleted. The operator also fixed two wiring faults `irq_runs`
  exposed — a misplaced INT1 wire and an ADXL375 CS that had come out.

  **331 → 252 KB/s, OOM 96 → 127 s (−24 % leak, +32 % survival)**, with both interrupts live so the
  figure is honest. Re-attributed:

  | component | before | now | note |
  |---|---|---|---|
  | `accel_adxl375` | 111.3 | **40.2** | −64 % |
  | `imu_lsm6dso32` | 63.7 | **50.6** | −21 % *while sampling 29× more often* |
  | `imu_bno055` | 34.4 | **47.9** | untouched — now the largest single consumer |
  | `airspeed_sdp810` | 12.7 | 22.0 | |
  | `gnss` | 12.8 | 12.1 | |

  The profile has FLATTENED: there is no dominant consumer left, the top three sit within 10 KB/s of
  each other and together are 55 % of the total, and the remaining ~80 KB/s is spread thin across a
  dozen tasks. That changes the strategy — further wins come from the shared paths every driver uses
  (`Telemetry.push` at 192 B, the per-sample tuples at 80 B), not from another single hot spot.

  **Validated against read rate, and the hunt can stop here.** Dividing every sensor's rate by N
  (`diag_real_leak.probe(slow=N)`):

  | rate | leak | time-to-OOM |
  |---|---|---|
  | ×1 (100 Hz) | 252.0 KB/s | 127 s |
  | **×2 (50 Hz)** | **175.2** | **182 s** |
  | ×5 | 144.7 | 221 s |
  | ×10 | 135.4 | 236 s |

  The relationship holds — and the curve **asymptotes at ~130 KB/s rather than falling toward zero**.
  Fitting `leak = fixed + variable/N` gives **~122 KB/s that is rate-INDEPENDENT** and ~130 KB/s that
  scales with reads (the fit predicts 187 at ×2 and 148 at ×5 against 175.2 and 144.7 measured). So
  the model matches the hardware, and there is a structural floor no sampling change can remove: the
  recorder, the control loop, asyncio itself.

  **Decision: stop optimising, mitigate instead.** 127 s is already ~2× the longest planned flight
  (65 s), and the two levers are cheap:

  1. **Halve the sensor rate to 50 Hz if 127 s ever looks tight** → ~175 KB/s, **182 s**, roughly 3×
     the flight. The cost is resolution: ~30 cm of linear travel per sample at 15 m/s instead of
     ~15 cm, which nothing in the control path needs.
  2. **The OOM predictor is the backstop** if the estimate is ever wrong — and it is now trustworthy:
     the forecast reads within ~1.5 s sample-to-sample, and the rescue it arms costs a measured
     34–45 ms and fires 0–2 times per flight.

  Remaining, if it is ever wanted: `imu_bno055` is the largest single consumer at 47.9 KB/s and has
  never been examined (a 24-byte block read at 50 Hz with float conversion). Not needed for flight.

- **Measured directly instead** (`test/diag_real_leak.py`: default config, real drivers, no sim, GC
  forced off): **~324 KB/s → OOM in ~99 s** from a 31.9 MB heap. That is well BELOW the ">150 s is
  acceptable" bar and below what the HITL intercept suggested. Caveats both ways: the bench run
  excludes the `flight` control task (a real flight leaks more) and includes an LSM6DSO32 polling at
  100 Hz because its INT1 line is unwired on v0.1 (fixing that in v0.2 reduces it). The in-flight
  memory rescue remains the backstop, and it is now measured at 34-45 ms per collect.
- `oom_soak.py`'s header figure of "15-18 KB/s" measures the **control path alone**, without telemetry
  and recorder. Not wrong, but narrower than it reads.

Two supporting fixes landed with it. `board_health`'s OOM forecast was unusable — successive readings
of 271, 155, 119, 109, None, 362, None, 206 s, on a number that **arms a ~200 ms mid-glide
`gc.collect()`** — because one 1 Hz difference *was* the slope and any sample where the heap grew
zeroed the estimate. It is now a cumulative average since the last collect, targeting a 512 KB
reserve, and reads to within ~1.5 s between samples. And the rescue trigger moved from
`oom < 2 x land` to `oom <= land`: three stacked safety margins were one too many, and the change
halved the unscheduled pauses across the matrix (10 → 5) with the heap never falling below 16 MB of
~30. `rescues`, `leak_kbps` and `rescue_ms` are now in `health.csv`, and the flight report draws the rescue
count as a staircase beside free memory.

**Durable diagnostics.** `Recorder.log()` is best-effort — buffered, flushed roughly every 1000
telemetry messages, so a short flight ends without them. Measured: a 57 s capture carried 5 log lines
(the controller's stage transitions) while the sequencer's own stage lines, both gc-collect durations
and every rescue line were absent. `Task.event()` now sends those down a per-component
`<name>_events.csv` Telemetry stream — which commits per line — as well as through `log()` for the
operator console. The first flight with it measured what had never been readable: **rescue collects
take 34–45 ms** (not the ~67 ms bench figure the threshold was justified against), the pre-flight
collect 26 ms, and the post-flight collect — the whole airborne phase's deferred cost — **159 ms**.

**Still open on `oom_soak.py`:** its default `target_kb=600` is degenerate against the new 512 KB
reserve (the rescue thrashes and the flight cannot progress), and `machine.reset_cause()` cannot be
checked the way its header describes because `mpremote` soft-resets on connect and overwrites it.

The leak is **steady across flight phases**: measured over three runs, the pre-glide (setting +
boosting) slope is 281–298 KB/s against 329–343 KB/s in the glide — a consistent 0.86 ratio, not a
separate boost anomaly. The glide is slightly higher because the guidance and endgame paths are only
fully active there.

**Run-to-run repeatability of a board HITL flight is ~1 m** (three identical F15 flights: 121/122/121 m,
no drift), so a difference above ~1 m in the matrix is a real change rather than scatter.

## Hardware roadmap — v0.2 and v1.0

v0.1 PCBs are ordered and the Gerbers cross-check clean against `config_default.py`. The design
review, the merge-to-one-board analysis and the bus-split decision live in
[`hardware.md`](hardware.md); in short:

- **v0.2, electrical:** ground pour on both layers (there is none today), and widen the servo rail —
  the main board uses one 0.30 mm width (~1.0 A) for signal and power alike, against 0.79 A measured
  per servo. The power board already gets this right.
- **v0.2, layout:** merge main + power into one board with an **energy island** (~5.8 g saved, 2.7 %
  of the light glide mass), keeping a bench power path so USB port-cycling still recovers a wedged
  board.
- **v0.2, buses:** `i2c:0` = `icp10111` alone, `i2c:1` = everything else. Altitude is the only
  redundancy pair living entirely on I²C, and a dead barometer costs ~100 m of miss. Bandwidth is not
  a factor — both buses sit under 5 % load.
- **v1.0:** a BNO085 could collapse the IMU stack *if* real boost shock stays under 16 g. Measured
  peak in sim is 3.3–4.3 g, but that model has no ignition transient, separation shock or landing
  impact — the decision needs one real capture logging through separation first.

## Required hardware

1. ESP32-P4 with Wi-Fi (or ESP32-C6 low-end variant) — Main Controller
2. accelerometer + gyro (ADXL375 high-G; BNO055 may be present too for redundancy)
3. altimeter (barometric: ICP-10111 primary, BMP280 backup)
4. laser altimeter for low-altitude AGL (ultrasonic also possible)
5. battery (separate rails for controller and servos)
6. 3 servos for fins
7. button / breakaway for stage-separation detection
8. wires
9. nice to have: camera + recorder (Luckfox Pico) — **separate board, own power, own SD**

The controller itself carries **no SD card** (logs/telemetry/video go to the Recorder over
UART). Everything must stay under 100 g to fit F6-class lift. See
[`hardware.md`](hardware.md) for what is purchased.

## Development roadmap

The project rule drives the order: **telemetry-only first, active control later.** Host-
testable foundations and connectivity come before the flight loop.

### Phase 0 — Foundations (MicroPython, on-board tested) — DONE
- ✅ `config.py` + `config_default.py`: loader, validator (pin uniqueness, bus refs, reserved
  pins, board.id, `recorder` section), `config_id` hashing, layered load + fallback, atomic save.
- ✅ `task.py` + `controller.py`: Task base + `ACTIVITIES` registry; Controller
  (`directory/create/active/close`, supervised run loops, flight stage machine).
- ✅ `cc_protocol.py`: line parser — bare tokens or `base64:<data>` (no quoting/tokenizing),
  `parse`/`build`/`encode`/`decode`.
- ✅ `recorder.py`: lock-free SPSC `Ring`s (write via `struct.pack_into`), `Recorder` singleton
  (`log`/`tlm`, async drain, session prefix, stats), `Telemetry(file, fields)`.
- **Milestone met:** config + tasks + protocol + recorder green on the board.

### Phase 1 — Connectivity & ground ops (no flight control)
- ✅ **Board side:** `wifi.py` (STA join `panda`, tx-power), `cc_client.py` (`Client` dial-out +
  `serve` loop, `create_dispatcher` answering whoami/ping/health/state/report/get-config/
  set-config/reset-config/reboot). System status (temp/mem/uptime) is in the health handler.
- ✅ **CC hub** (`src/control/`): `board.py` (Board, 10 s exchange timeout) + `server.py` (board
  listener 1234 + ~2 s heartbeat + operator console 1235, routing `<board>`/`all`, sticky `select`,
  drop-in `commands/`) + `main.py` (CLI `--host`/`--port`/`--help`) + **HTTP/SSE dashboard** (8080,
  `web.py`+`static/index.html`) with a **config editor** (load `get-config` → edit the draft → `set-config`
  → `reboot`/`reset-config`, all via `/api/cmd`; rows click-to-target). Per-file host tests.
- ✅ **Task packages** — `drivers/` (HAL: `led`, `wifi`, the future sensors/servo, via `@task.driver`)
  and `tasks/` (subsystems: Recorder adapter, `board_health`, `cc_link`, via the `@task.activity`
  alias), each with a `load()` that imports its modules so the registrations run (one shared
  `ACTIVITIES` registry for now). A config component names its impl with `driver` (drivers/) or
  `activity` (tasks/). `task.py` stays the base; top-level `recorder` / `mission` stay (data path /
  identity). `deploy.sh` pushes packages in one batched
  mpremote session (a deployed main.py auto-runs on each soft-reset).
- ✅ **Controller bring-up wiring** — `main.py` (boot): `drivers.load()` + `tasks.load()` → create
  Mission → hand the config to the Controller, which builds + starts the *enabled* tasks (Recorder,
  LED, BoardHealth; driverless sensors skipped). **No driver named by hand** — drop a file in
  `drivers/`/`tasks/` and enable it in the config. Then Wi-Fi join → dial + serve Control.
  Telemetry-first; time sync arrives from Control (`update mission {epoch}`). Tested (`test_main`).
- ✅ **Status LED** — `drivers/led.py` `@task.driver('led')`: one GPIO (`led_status`) blinks the
  board state (fast=error, slow=standby, solid=flying) from controller state + health. `test_led`.
- **Milestone:** power a board → it appears on CC → view health → toggle a component →
  save + reboot → it returns from the saved config.

### Phase 2 — Sensors & telemetry (enables telemetry-only flights) — DONE (bench-verified)
- ✅ **`databoard.py`** — latest-value store **+ read-time fusion** in one structure: a registry of
  `Parameter` objects, each owning a channel per source (rank=priority, expiry, 2 slots). `value()`
  returns the lowest-rank fresh source, else linearly extrapolates the newest source to now — so
  "rank 0 until it expires, then rank 1" is emergent (no queue, no fusion task). Inspectable.
  `test_databoard`.
- ✅ **Shared (locked) I²C bus** (`i2cbus.py`) — one `machine.I2C` + `asyncio.Lock` per id, shared
  via `get()`; ADXL375 + BNO055 + BMP280 coexist on `i2c:0`. `test_i2cbus`.
- ✅ Sensor drivers + tests (all → databoard, write `altitude`/`temperature`/`accel`/`attitude`/
  `agl`/`position`):
  ✅ **ADXL375** (`accel` g, on SPI(1) — I²C code retained; interrupt-driven + timeout fallback);
  ✅ **BNO055** (NDOF fusion `attitude` deg, polled 50 Hz);
  ✅ **BMP280** (Bosch-compensated `altitude` m + `temperature` °C, 10 Hz, backup baro);
  ✅ **ICP-10111** (TDK command/OTP protocol, `altitude`/`pressure`/`temperature`/`elevation`,
  primary baro, prio 0);
  ✅ **ATGM336H GNSS** (UART, CASIC/PCAS RMC-only @10 Hz → `position`);
  ✅ **VL53L4CX** (laser ToF, I²C, VL53L4CD ULD init → `agl` m). All live-verified.
- ✅ **Sensor fusion** — folded into the databoard's read-time `value()` (the separate fusion task
  is gone). rank=`priority`, expiry=`timeout_ms` (realistic windows: a few sample periods). Live:
  altitude/temperature → ICP-10111 (rank 0) over BMP280, accel/attitude pass through.
- ✅ **Telemetry wiring** — `Telemetry(decimate_us)` rate-limits a stream (a fast sensor pushes every
  sample, decimated to a sane rate). Each sensor emits its own `SENSOR.csv` (accel 50 Hz, imu 25 Hz,
  baros 10 Hz); separation emits a durable `separation.csv`. Live-verified: rows/sec match the
  configured `telemetry_us`. (No `fused.csv` — the fused value is read-time via the databoard.)
- ✅ **Separation switch** (`drivers/separation.py` `@task.driver('separation')`) — copper pads on
  `separation_switch` (GPIO33): HIGH=nested, LOW=separated, internal pull-down (no external resistor
  needed). IRQ on either edge → debounce → on a confirmed separation during Boosting, drive
  Boosting→Gliding (guarded). Live-verified: unplugging 3V3 fired the transition. `test_separation`.
- ✅ Recorder UART link to the physical Luckfox recorder (rings drain to uart:1).
- ✅ **`probe` self-tests** — on-demand pre-flight check over every Inspectable (`probe [name|all]`
  via CC), per-step inline logging; active checks (servo sweep) run on demand, never at boot.
- ✅ **Servos** (`drivers/sg90.py`) — 3× SG90 on PWM, integer-degree open-loop, N-slew concurrency
  gate (`servo_concurrency`) to bound the boost-rail transient. Live-verified (groundwork for Phase 3).
- ✅ **CC log streaming** — board tees `Recorder.log()` into a deadline-gated ring on demand; the
  `<board> log <ms>` poll (operator console or dashboard `POST /api/log`) returns each window's lines
  to the console + `/logs` SSE. Auto hub discovery: a board with no `cc_host` dials its subnet `.1`
  (explicit address overrides; `''` = standalone).
- **Deferred (not blocking close):** LSM6DSO32 IMU driver (hardware not arrived — fusion already
  accepts a new ranked accel source); outdoor GNSS fix (driver works indoors, needs an open-sky test).
- **Milestone:** a real telemetry-only flight — collect data, no actuation.

### Phase 3 — Active control
Builds on what Phase 2 already shipped: the `Stage` machine (`SETTING→BOOSTING→GLIDING→LANDING`,
`controller.py`), 3× SG90 fins (`servo_yaw` + `servo_eleron_left`/`_right`, integer-degree `move()`
+ N-slew gate), the separation switch (drives `BOOSTING→GLIDING`), and databoard signals
(`attitude` deg from BNO055, `accel` g from ADXL375, `altitude`/`elevation` m, `agl` m, `position`).
Order: **1 → 2 → 3 → 4** (mixer, then the loop it drives, then automate the stages that gate it,
then per-phase behaviour); 5–6 harden it. All tasks positive + negative tests, on-board.

- ✅ **1. Surface mixer** (`mixer.py`, sibling of servo.py) — `Mixer.mix(roll, pitch, yaw)` →
  `{fin: angle}`: **elevon mixing** (pitch = both elerons together, roll = differential) + **yaw =
  rudder**, per-fin trim (neutral) + direction-sign gains + a hard ±`limit_deg` clamp on control
  deflection (config `mixer`). Pure integer-degree math, `neutralise()` for the safe output;
  `test_mixer` covers the matrix/trim/clamp. No servo motion until the control task drives it.
- ✅ **2. Stabilization loop** (`tasks/flight.py`, `@task.activity('flight')` + `pid.py`) — reads
  `attitude` (heading/roll/pitch), **PID per axis** (`pid.Pid`, integral + output clamp) to the
  setpoint + heading-hold → `mixer.mix()` → `sg90.update()`. **Active only in `GLIDING`** (every other
  stage holds fins neutral); stale/absent attitude → neutral (degraded). **Timer-driven**:
  `schedule_hz > 0` → `machine.Timer` ticks a ThreadSafeFlag (deterministic slice independent of other
  tasks, ~1 m/step at 100 Hz/100 m/s); `schedule_hz 0` → asyncio at `period_ms`. Self-times (steps +
  max_step_us via inspect) for a load sweep vs board_health. Gains 0 + task disabled by default. Tests
  `test_pid` (P/I/D + clamps) + `test_flight` (gating, degraded, PID→mix→fins, both schedule modes).
  **Load sweep** (ESP32-P4, otherwise-idle board, gains 0 so each step does the full read→PID→mix→
  apply; CPU from an idle-counter vs a no-flight baseline):

  | schedule | achieved | worst step | CPU load |
  |---|---|---|---|
  | asyncio (10 ms) | 100 Hz | ~1.15 ms | ~5 % |
  | timer 50 Hz | 50.0 Hz | ~1.08 ms | ~12 % |
  | timer 100 Hz | 100.0 Hz | ~1.07 ms | ~15 % |
  | timer 200 Hz | 200.0 Hz | ~1.07 ms | ~20 % |

  Takeaways: the **timer hits the configured rate exactly** (deterministic), CPU scales ~linearly with
  rate with comfortable headroom even at 200 Hz; the **worst-case step is ~1.1 ms** (an occasional GC
  pause — typical steps are far shorter — and it still fits the 5 ms period at 200 Hz). asyncio also
  reached 100 Hz here at lower CPU, but it has no rate guarantee under contention (the timer's whole
  point). **Chosen: 100 Hz timer** — 1 m/step at 100 m/s, deterministic, ~15 % load, matched to the
  BNO055's ~100 Hz attitude ceiling.
- ✅ **3. Stage automation** (`tasks/sequencer.py`, `@task.activity('sequencer')`) — a guarded,
  forward-only `_tick` driving `set_stage` from databoard signals: `SETTING→BOOSTING` sustained
  `|accel| > launch_g` for `launch_ms`; `BOOSTING→GLIDING` the separation switch (primary, exists) with
  a `boost_timeout_ms` burnout fallback; `GLIDING→LANDING` `agl < land_agl_m` (elevation fallback);
  `LANDING→done` `|accel|≈1g` stationary for `ground_ms`. Each fires once (stage check + reset-on-change
  guard, incl. separation-driven hops), logged (`controller :: stage ->`, picked up by the analysis
  tool) + a `sequencer.csv` marker. Thresholds in config; **enabled** (safe on passive flights — logs
  the stage timeline, no actuation since the flight task is disabled). `test_sequencer` drives every
  transition + the transient guard with synthetic accel/agl. The control loop (task 2) already gates to
  `GLIDING`, so this closes that loop.
- ✅ **4. Per-phase behaviour** (folded into `tasks/flight.py`) — the loop's gating generalised from
  GLIDING-only to a `phases` config map: each entry names a CONTROL stage and its attitude setpoint;
  stages absent from it (`SETTING`/`BOOSTING`/`DONE`) hold the fins neutral (no actuation under thrust /
  on the ground). `GLIDING` = wings-level + heading hold; `LANDING` carries its own setpoint (the flare
  knob, 0 until tuned). Heading is captured on entering control and held across a glide→landing hand-off
  (continuous control, no neutral between); PIDs reset only on (re)entering control from a non-control
  stage. `inspect` exposes `active`/`phase`. `test_flight` covers the per-stage engage/switch/disengage.
  (Spiral-to-launch-site maneuver waits on GNSS nav — Phase 4.)
- ✅ **5. Watchdog + heartbeat** (`tasks/watchdog.py`, `@task.activity('watchdog')`) — two layers:
  a hardware `machine.WDT` fed every period (a TOTAL event-loop wedge stops the feed → hard reset — the
  backstop), plus a **control-loop heartbeat** (while the flight task is in a control phase its step
  counter must advance; a stall → **full `machine.reset()`**). Recovery is a full reset, not a soft
  event-loop restart — MicroPython can't preempt a wedged native call and the HW (PWM / I²C / sensors)
  needs a clean reset; boot re-centres the fins to bound the window. Stale attitude is NOT a trigger
  (the flight loop already fail-safes to neutral, task 2). Disabled by default (a live WDT also resets
  the board during REPL bench work). `test_watchdog` (injected WDT/reset) covers the stall decision +
  feed-vs-reset. **±90°/tumble is deliberately not a trigger** — control should keep fighting there.
- ✅ **6. Arming + ground-test safety** — `Controller.armed` gates actuation: the flight loop holds
  the fins neutral unless **armed** *and* in a control phase. CC **`arm`** enables it only when board
  verify is clean (every device up + probe healthy — and `mission.probe` requires the launch position,
  so arming is mission-gated for free); a refused arm returns the problems. **`disarm`** clears it.
  Ground test: **`stage <name>`** holds a stage (`Controller.manual` → the sequencer pauses) so the
  live loop can be exercised on the bench (`arm` + `stage gliding` → tilt the board, watch the fins);
  `stage auto` resumes. Tests across controller/flight/sequencer/cc_client (disarmed→neutral,
  manual→no auto-advance, arm refused-on-probe-fail / clean→armed).
- **Landing-zone navigation** (heading-to-home) is now implemented — see Phase 4. GPX export still
  deferred.
- **Bench-complete (all 6 tasks ✅):** the full chain — sensors → stage machine → armed + per-phase
  gated PID → mixer → fins, with a watchdog backstop — is built and on-board tested (34/34). The only
  thing left before flight is **gain tuning on the real airframe** (gains default 0; the flight +
  watchdog tasks ship disabled), informed by the E16/F15 passive-flight data.
- **Milestone:** a controlled glide — launch detect → separation → stabilized glide → flare, with
  the loop gated by stage + arming + a fed watchdog.

### Phase 4 — Polish (in progress)
- ✅ **Landing-zone navigation** (`navigation.py` + `tasks/flight.py` + `mission.py`) — the mission's zone is a
  lat/lon rectangle (TL/BR); `navigation.zone()` → target (centre) + gates (short-side midpoints, the safe
  approach corridors — operator orients the zone, `coludo.md`). In GLIDING the yaw heading setpoint is
  `navigation.steer()` toward the nearer gate then the centre (overshoot → ~180° re-approach, emergent). Three
  GPS-degrading tiers: live fix → steer from the current position; no fix + CC-set launch point → hold
  the launch→gate bearing (open-loop, GPS-denied fallback); neither → captured heading. Mission resolves
  the zone vs the launch point (CC-set or GNSS), gates it on `max_range_m` (board config — airframe
  glide range), exposes points + distances + `in_range` in `inspect()`, and `probe` fails a too-far zone.
- ✅ **GC/perf strategy** — done (see Simulation & performance below): GC disabled in flight (collect on the ground), nav-heading
  cache + zero-alloc mixer keep the 100 Hz loop near-zero-allocation (`coludo.md` "Garbage collection
  in flight"). Second-core / native control loop not needed at current latency.
- Still open: flight-log review, **code-audit follow-ups**, **Phase 5 prep** (maket-board bring-up),
  enabling the **6-DoF IMU** (LSM6DSO32 — frees the BNO055 / adds redundant attitude+accel), per-launch
  mission config (`launch.config`), GPX export of telemetry, richer browser UI, spiral-to-launch-site
  maneuver.

### Simulation & performance — DONE
- ✅ **HITL simulator** (`tasks/hitl.py` + `sim_model.py` + `config_hitl.py`) — closed-loop on the board
  with no production-code changes: reads the commanded fins, steps a flight model, and provides the
  simulated sensors on the databoard at priority 0 so `sequencer`/`flight`/`pid`/`mixer`/`navigation`
  can't tell it is not real. Controlled noise N (5/10/25/50/100 %) + spikes.
- ✅ **Virtual flight** (`tools/virtual_flight.py`) — the same model + real control code on the host,
  emitting recorder captures; `tools/flight_report.py` (interactive plotly: 3D track + accel/altitude/
  speed/attitude/**fins**/board-health/agl, unified hover) and `tools/flight_svg.py` (dependency-free).
  Experiments + corner cases stored in [`sims/TMS-7/`](sims/TMS-7/).
- ✅ **Energy management** (bank-to-turn, Phase 4) and the **perf cluster** — measured on
  the board (`steer()` 174 µs cached off the hot loop; mixer zero-alloc; GC-off-flight 20 MB headroom;
  load probe 7.2 W → 3.6 W).
- ✅ spike injection + two stored corner-case traces (glitch-rejection; everything-degraded).

### `launch.config` (mission config)
A separate config document, same layered/validated/save+reactivate form as `board.config`
([`../doc/specs/board-config.md`](../doc/specs/board-config.md)), describing a *specific launch* rather
than the *board*:
- landing zone (TL / BR corners, target point), entrance threshold, allowed-zone radius,
- AGL landing-trigger altitude and vertical-speed/roll gates,
- minimum controllable airspeed, and any per-launch tuning.

Needed by Phase 3/4 navigation. **Interim:** until it exists, and because all launches are from
the same site, these parameters may live in `board.config` (noted in `board-config.md`).

**Known launch pads** (several may be defined; the on-board GNSS and the host GPS each select the
nearest one). First site — **HPRC** (Homestead Public Rocketry Club):
- launch pad: `25.514379, -80.391795`
- landing zone (Google Maps, north up): top-left `25.514944, -80.392972`,
  bottom-right `25.514583, -80.391111` — a **~40 m (N–S) × ~187 m (E–W)** strip (~7500 m²),
  centre `25.514764, -80.392042`, ~49 m from the pad. Long axis E–W → the nav gates the short
  (E/W) ends (`navigation.zone()`), so the glider runs in along the strip.
- constraint: landing-zone centre must be < 100 m from the pad (49 m here ✓).

If the host laptop has a GPS (GPSD), the board could take **assisted GPS** from it during setup —
a sync loop that waits until GPS is ready.

## Task data-flow model

Adopted and documented in [`../doc/specs/coludo.md`](../doc/specs/coludo.md) ("Task Data-Flow and
Message Propagation"), grounded by on-board measurements (see
[benchmark findings](benches/WaveShare_esp32p4-micropython-findings.md)). Not one paradigm — chosen per
data class to respect the GC-pause and <10 ms control-loop budgets on MicroPython:

- **Hot sensor data** (IMU/baro at 100–200 Hz) → a shared **latest-value databoard**:
  preallocated per-quantity slots (value + timestamp + source), latest-wins, no per-sample
  allocation. The control loop and fusion read it directly. Avoids GC churn and starvation.
- **Discrete events / commands** (separation, phase change, CC commands, enable/disable) →
  a small **topic event-bus / `notify()` callbacks**. Low rate, allocation acceptable.
- **Bulk telemetry / logs** → **ring buffers** (producer/consumer), drained to the Recorder
  over UART and served to CC on poll.

The control loop stays self-contained (reads databoard, writes servos) so it runs even if
other tasks stall, and is paced by a hardware timer (not `asyncio.sleep`, which floors at
~10 ms on this port). The Recorder's PSRAM queues are written with `struct.pack_into`, never
slice-assignment (which is O(buffer length) here).
