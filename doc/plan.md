# Plan

Required hardware and the phased development roadmap. Architecture lives in
[`../specs/`](../specs/); working conventions in [`skills.md`](skills.md).

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

### Phase 0 — Foundations (host-testable, no board required)
- `src/glider/` config: `config_default.py`, loader, validator (pin uniqueness, bus refs,
  types), `config_id` hashing, defaults → active overlay + fallback to defaults.
- Task base (`setup/run/notify/report/finish/validate/testing`) + Controller skeleton
  (`directory/create/active/close`, hardcoded creation order).
- Line-protocol tokenizer (positional / `key=value` / quoted) + message framing.
- Log/telemetry ring buffers + record formats.
- Tests for each under `src/glider/test/`, runnable on the host.
- **Milestone:** config + tasks + protocol parsing green on host.

### Phase 1 — Connectivity & ground ops (no flight control)
- Board: Wi-Fi STA join (`panda`), CC client (dial out, `whoami`/`iam`, command loop),
  LED status, system status (temp/mem/load).
- `src/control/` CC hub: board listener (1234), telnet (1235), HTTP + SSE (8080), registry,
  2 s poll loop, `help`/`list`/`select`, draft config.
- Minimal browser dashboard: health, enable/disable, `save-config` → `reboot`.
- **Milestone:** power a board → it appears on CC → view health → toggle a component →
  save + reboot → it returns from the saved config.

### Phase 2 — Sensors & telemetry (enables telemetry-only flights)
- Sensor drivers + tests: ADXL375, BNO055, ICP-10111, BMP280, GNSS (ATGM336H), laser AGL.
- Sensor fusion (priority/timeout from each component's `provides`), separation switch (IRQ).
- Recorder UART link from the board; telemetry/log flush to the Recorder.
- **Milestone:** a real telemetry-only flight — collect data, no actuation.

### Phase 3 — Active control
- Servo task (sequential updates, calibration maps, ±45° limits).
- Flight state machine (ignition → separation → apogee → landing thresholds) + PID
  stabilization + per-phase maneuvers + degraded modes + watchdog / health monitor.
- **Milestone:** a controlled glide.

### Phase 4 — Polish
- Landing-zone navigation + per-launch mission config, GPX export of telemetry, richer
  browser UI, GC/perf strategy (second core or native control loop if latency demands).

## Task data-flow model

Adopted and documented in [`../specs/coludo.md`](../specs/coludo.md) ("Task Data-Flow and
Message Propagation"), grounded by on-board measurements (see
[benchmark findings](benches/esp32p4-micropython-findings.md)). Not one paradigm — chosen per
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
~10 ms on this port). The logger's PSRAM queues are written with `struct.pack_into`, never
slice-assignment (which is O(buffer length) here).
