# Plan

Required hardware and the phased development roadmap. Architecture lives in
[`../specs/`](../specs/); working conventions in [`skills.md`](skills.md).

## Status

- **Phase 0 — done.** `config`, `task`+`controller`, `cc_protocol`, `recorder` (SPSC rings +
  `Telemetry`) in `src/glider/`, all MicroPython, on-board tested (`make test`).
- **Phase 1 board side — done.** `wifi` (STA) and `cc_client` (`Dispatcher`/`Client` +
  `create_dispatcher` answering whoami/ping/health/state/report/get-config/save-config/
  reset-config/reboot). 8/8 on-board tests.
- **Control hub — done.** `src/control/` split into `board.py` (Board, 10 s exchange timeout),
  `server.py` (the hub: board listener + ~2 s heartbeat + telnet operator console, drop-in
  `commands/`, `all`-only broadcast) and `main.py` (CLI: `--host 0.0.0.0` / `--port` / `--help`),
  plus the HTTP/SSE dashboard (`web.py`). Per-file host tests (`test_board`, `test_server`).
  Remaining polish: draft config + save/reboot from the UI. **Then** the Controller
  **bring-up wiring** (connect → time-sync → start tasks), two-sided so it wants Control live first.
  The Recorder is now wired into the task graph by a thin adapter (`@task.activity('recorder')` in
  `tasks/recorder.py`); the `recorder` component (bus uart:1) makes the Controller create + supervise
  its drain loop. ✅

### Near-term work from `findings.txt` (quality pass)

- **Conventions** adopted (`CLAUDE.md` + `skills.md`) and **protocol reorder** done
  (`board command`, Inspectable commands). ✅
- **Tooling**: `ruff` config + `deploy.sh` (lint + `mpy-cross` + push to `/pyboard/`) +
  `ssid.creds` (gitignored Wi-Fi password). ◻
- **`recorder.py` refactor** as the golden reference module (rename to full names, async
  `StreamWriter` drain, drop `stop`/`sink`/`also`/`limit`, log-drop vs tlm-raise, `const`, type
  annotations) + finish the `_Msg` rename in `cc_protocol`; sync tests. ◻
- **Config schema reorg** (board-config.md): nested `buses: {uart:{1,2}, i2c:{0,1}, spi:{}}`, a
  `sensors:` section, no abbreviations, configs in a `configs/` subfolder. ◻ (doc + code)
- **`Inspectable` mixin** (`inspect`/`update`/`stats` + `type`/`name`) — design then adopt. ◻
- **`BoardHealth` task** — `esp32.mcu_temperature()`, `idf_heap_info()` (PSRAM-aware), idle/load,
  periodic Telemetry every ~1 s. ◻
- **`test/probe_pins.py`** — sweep pins 0..60 and UART/I2C/SPI 0..5 to auto-derive a board map;
  inspect `machine.Pin.board`. ◻ (low priority — we have a map)

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
  save-config/reset-config/reboot). System status (temp/mem/uptime) is in the health handler.
- ✅ **CC hub** (`src/control/`): `board.py` (Board, 10 s exchange timeout) + `server.py` (board
  listener 1234 + ~2 s heartbeat + operator console 1235, routing `<board>`/`all`, sticky `select`,
  drop-in `commands/`) + `main.py` (CLI `--host`/`--port`/`--help`) + **HTTP/SSE dashboard** (8080,
  `web.py`+`static/index.html`) with a **config editor** (load `get-config` → edit the draft → `save-config`
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

### Phase 2 — Sensors & telemetry (enables telemetry-only flights)
- ✅ **`blackboard.py`** — latest-value store **+ read-time fusion** in one structure: a registry of
  `Parameter` objects, each owning a channel per source (rank=priority, expiry, 2 slots). `value()`
  returns the lowest-rank fresh source, else linearly extrapolates the newest source to now — so
  "rank 0 until it expires, then rank 1" is emergent (no queue, no fusion task). Inspectable.
  `test_blackboard`.
- ✅ **Shared (locked) I²C bus** (`i2cbus.py`) — one `machine.I2C` + `asyncio.Lock` per id, shared
  via `get()`; ADXL375 + BNO055 + BMP280 coexist on `i2c:0`. `test_i2cbus`.
- ◧ Sensor drivers + tests (all → blackboard, write `altitude`/`temperature`/`accel`/`attitude`):
  ✅ **ADXL375** (`accel` g, interrupt-driven on DATA_READY/INT1→GPIO4 + timeout fallback);
  ✅ **BNO055** (NDOF fusion `attitude` deg, polled 50 Hz);
  ✅ **BMP280** (Bosch-compensated `altitude` m + `temperature` °C, 10 Hz, backup baro);
  ✅ **ICP-10111** (`drivers/icp10111.py` — TDK command/OTP protocol, `altitude` + `temperature`,
  primary baro, prio 0); ◻ GNSS (ATGM336H), VL53L4CX. All live-verified on `i2c:0`.
- ✅ **Sensor fusion** — folded into the blackboard's read-time `value()` (the separate fusion task
  is gone). rank=`priority`, expiry=`timeout_ms` (realistic windows: a few sample periods). Live:
  altitude/temperature → ICP-10111 (rank 0) over BMP280, accel/attitude pass through.
- ✅ **Telemetry wiring** — `Telemetry(decimate_us)` rate-limits a stream (a fast sensor pushes every
  sample, decimated to a sane rate). Each sensor emits its own `SENSOR.csv` (accel 50 Hz, imu 25 Hz,
  baros 10 Hz); separation emits a durable `separation.csv`. Live-verified: rows/sec match the
  configured `telemetry_us`. (No `fused.csv` — the fused value is read-time via the blackboard.)
- ✅ **Separation switch** (`drivers/separation.py` `@task.driver('separation')`) — copper pads on
  `separation_switch` (GPIO33): HIGH=nested, LOW=separated, internal pull-down (no external resistor
  needed). IRQ on either edge → debounce → on a confirmed separation during Boosting, drive
  Boosting→Gliding (guarded). Live-verified: unplugging 3V3 fired the transition. `test_separation`.
- ◻ Recorder UART link to the physical Luckfox recorder (rings drain to uart:1 already).
- **Milestone:** a real telemetry-only flight — collect data, no actuation.

### Phase 3 — Active control
- Servo task (sequential updates, calibration maps, ±45° limits).
- Flight stage machine (ignition → separation → apogee → landing thresholds) + PID
  stabilization + per-phase maneuvers + degraded modes + watchdog / health monitor.
- **Milestone:** a controlled glide.

### Phase 4 — Polish
- Landing-zone navigation + per-launch mission config, GPX export of telemetry, richer
  browser UI, GC/perf strategy (second core or native control loop if latency demands).

### `launch.config` (mission config)
A separate config document, same layered/validated/save+reactivate form as `board.json`
([`../specs/board-config.md`](../specs/board-config.md)), describing a *specific launch* rather
than the *board*:
- landing zone (TL / BR corners, target point), entrance threshold, allowed-zone radius,
- AGL landing-trigger altitude and vertical-speed/roll gates,
- minimum controllable airspeed, and any per-launch tuning.

Needed by Phase 3/4 navigation. **Interim:** until it exists, and because all launches are from
the same site, these parameters may live in `board.json` (noted in `board-config.md`).

**Known launch pads** (several may be defined; the on-board GNSS and the host GPS each select the
nearest one). First site — **HPRC** (Homestead Public Rocketry Club):
- launch pad: `25.514379, -80.391795`
- landing zone: top-left `25.514630, -80.392880`, bottom-right `25.514656, -80.391155`
- constraint: landing-zone centre must be < 100 m from the pad.

If the host laptop has a GPS (GPSD), the board could take **assisted GPS** from it during setup —
a sync loop that waits until GPS is ready.

## Task data-flow model

Adopted and documented in [`../specs/coludo.md`](../specs/coludo.md) ("Task Data-Flow and
Message Propagation"), grounded by on-board measurements (see
[benchmark findings](benches/WaveShare_esp32p4-micropython-findings.md)). Not one paradigm — chosen per
data class to respect the GC-pause and <10 ms control-loop budgets on MicroPython:

- **Hot sensor data** (IMU/baro at 100–200 Hz) → a shared **latest-value blackboard**:
  preallocated per-quantity slots (value + timestamp + source), latest-wins, no per-sample
  allocation. The control loop and fusion read it directly. Avoids GC churn and starvation.
- **Discrete events / commands** (separation, phase change, CC commands, enable/disable) →
  a small **topic event-bus / `notify()` callbacks**. Low rate, allocation acceptable.
- **Bulk telemetry / logs** → **ring buffers** (producer/consumer), drained to the Recorder
  over UART and served to CC on poll.

The control loop stays self-contained (reads blackboard, writes servos) so it runs even if
other tasks stall, and is paced by a hardware timer (not `asyncio.sleep`, which floors at
~10 ms on this port). The Recorder's PSRAM queues are written with `struct.pack_into`, never
slice-assignment (which is O(buffer length) here).
