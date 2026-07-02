# Plan

Required hardware and the phased development roadmap. Architecture lives in
[`../specs/`](../specs/); working conventions in [`skills.md`](skills.md).

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
  finally consumed end-to-end (driver → databoard → PID, and rendered in flight reports). *Open:* flight-log
  review, the **code-audit follow-ups** (findings.md, re-iterated — `config.validate` +
  `cc_client.create_dispatcher` decomposed so far), optional **viperization** of the now-integer hot path
  (DEFERRED on `bench_flight` evidence: a control step is only ~4% of the 100 Hz budget, and no single
  component dominates — 3× `pid.step` is ~22% of a step, the biggest single slice is `_apply`/fin-writes
  ~25% which is not viperizable arithmetic, and `navigation.steer` is throttled to ~24 µs amortized/step;
  `pid.step` also can't be `@viper` wholesale — object attrs + `None` sentinels — only a leaf extraction
  would help, for a fraction of that 22%), and Phase 5 prep.
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

- **Nail the landing strip** — bank-to-turn lands ~85 m from the 40 m-wide strip centre (good sensors);
  it drifts off on short final because LANDING goes wings-level. A tighter final-approach / flare (or a
  smaller orbit radius), re-measured in the sim.
- **High-noise robustness** (≥50 %): the bank loop reads attitude, so heavy noise degrades the orbit —
  a filter / rate-limit on the steering input.
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
([`../specs/board-config.md`](../specs/board-config.md)), describing a *specific launch* rather
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

Adopted and documented in [`../specs/coludo.md`](../specs/coludo.md) ("Task Data-Flow and
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
