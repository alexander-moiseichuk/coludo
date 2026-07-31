# Telemetry schema

> **GENERATED from the sources by `tools/gen_schema.py` — do not hand-edit.** Regenerate after changing any `recorder.Telemetry(...)` declaration (`python3 tools/gen_schema.py`); `--check` fails the local gate if it is stale.

> A stream listed for BOTH `board` and `host sim` must carry the same fields in both — that is what makes a host capture and a board capture interchangeable to every renderer. The host declarations live in `tools/virtual_flight.py`; the board ones in the driver or task that owns the stream.

Every stream a capture can contain, and the fields in each. A recorder capture interleaves `@<session>_<file>@<row>` telemetry rows with plain log lines; `tools/flight_telemetry.py` demuxes them, and every renderer resolves streams **by role** (the fields they carry) rather than by file name — a capture's file names track the fitted hardware, so a fallback flight names them differently.

## Streams

| stream | origin | declared in | fields |
|---|---|---|---|
| `accel_adxl375.csv` | board | `hitl.py` | `ax`, `ay`, `az` |
| `accel_adxl375.csv` | host sim | `virtual_flight.py` | `ax`, `ay`, `az` |
| `airspeed_sdp810.csv` | board | `hitl.py` | `dynamic_pressure`, `airspeed_cms`, `temperature` |
| `airspeed_sdp810.csv` | host sim | `virtual_flight.py` | `dynamic_pressure`, `airspeed_cms`, `temperature` |
| `baro_icp10111.csv` | board | `hitl.py` | `altitude`, `temperature`, `pressure`, `elevation` |
| `baro_icp10111.csv` | host sim | `virtual_flight.py` | `altitude`, `temperature`, `pressure`, `elevation` |
| `checkpoint.csv` | board | `warmstart.py` | `stage`, `altitude`, `speed`, `airspeed`, `ticks_ms` |
| `fins.csv` | board | `hitl.py` | `eleron_left`, `eleron_right`, `yaw` |
| `fins.csv` | host sim | `virtual_flight.py` | `eleron_left`, `eleron_right`, `yaw` |
| `flight.csv` | board | `flight.py` | `stage`, `active`, `airspeed_cms`, `fin_cap`, `roll_sp`, `pitch_sp`, `heading_err`, `roll_cmd`, `pitch_cmd`, `yaw_cmd`, `wind_cms`, `wind_from` |
| `flight.csv` | host sim | `virtual_flight.py` | `stage`, `active`, `airspeed_cms`, `fin_cap`, `roll_sp`, `pitch_sp`, `heading_err`, `roll_cmd`, `pitch_cmd`, `yaw_cmd`, `wind_cms`, `wind_from` |
| `gnss.csv` | board | `hitl.py` | `lat`, `lon`, `speed_kn`, `course` |
| `gnss.csv` | host sim | `virtual_flight.py` | `lat`, `lon`, `speed_kn`, `course` |
| `health.csv` | board | `board_health.py` | `temp`, `mem_free`, `load`, `oom_s`, `land_s`, `leak_kbps`, `rescues`, `rescue_ms` |
| `health.csv` | host sim | `virtual_flight.py` | `temp`, `mem_free`, `load`, `oom_s`, `land_s`, `leak_kbps`, `rescues`, `rescue_ms` |
| `hitl_clock.csv` | board | `hitl.py` | `sim_s`, `wall_s`, `lag_s` |
| `imu_bno055.csv` | board | `hitl.py` | `heading`, `roll`, `pitch` |
| `imu_bno055.csv` | host sim | `virtual_flight.py` | `heading`, `roll`, `pitch` |
| `imu_lsm6dso32.csv` | board | `hitl.py` | `ax`, `ay`, `az`, `gx`, `gy`, `gz` |
| `imu_lsm6dso32.csv` | host sim | `virtual_flight.py` | `ax`, `ay`, `az`, `gx`, `gy`, `gz` |
| `laser_agl.csv` | board | `hitl.py` | `agl` |
| `laser_agl.csv` | host sim | `virtual_flight.py` | `agl` |
| `power_ina226.csv` | host sim | `virtual_flight.py` | `voltage_mv`, `current_ma`, `power_mw`, `alerts` |
| `separation.csv` | board | `separation.py` | `event`, `stage` |
| _per-device_ (`<name>.csv`) | board | `adxl375.py` | `ax`, `ay`, `az` |
| _per-device_ (`<name>.csv`) | board | `bmp280.py` | `altitude`, `temperature`, `pressure`, `elevation` |
| _per-device_ (`<name>.csv`) | board | `bno055.py` | `heading`, `roll`, `pitch`, `ax`, `ay`, `az` |
| _per-device_ (`<name>.csv`) | board | `gnss.py` | `lat`, `lon`, `speed_kn`, `course` |
| _per-device_ (`<name>.csv`) | board | `icp10111.py` | `altitude`, `temperature`, `pressure`, `elevation` |
| _per-device_ (`<name>.csv`) | board | `ina226.py` | `voltage_mv`, `current_ma`, `power_mw`, `alerts` |
| _per-device_ (`<name>.csv`) | board | `lsm6dso32.py` | `ax`, `ay`, `az`, `gx`, `gy`, `gz` |
| _per-device_ (`<name>.csv`) | board | `sdp810.py` | `dynamic_pressure`, `airspeed_cms`, `temperature` |
| _per-device_ (`<name>.csv`) | board | `sequencer.py` | `stage`, `reason` |
| _per-device_ (`<name>.csv`) | board | `sg90.py` | `angle`, `pulse_us`, `done` |
| _per-device_ (`<name>.csv`) | board | `task.py` | `event` |
| _per-device_ (`<name>.csv`) | board | `vl53l4cx.py` | `agl` |

## Shapes that differ between the sim and the board

These are the traps — a renderer written against one shape silently finds nothing in the other:

- **Fins.** The HITL sim records ONE fused `fins.csv` with a column per surface; a real board records one stream PER SERVO (`servo_<surface>.csv`, column `angle`, from `drivers/sg90.py`). `flight_telemetry` rebuilds the fused shape from the per-servo streams when a capture has none, forward-filling each surface (a servo holds its last commanded angle), so both render identically — including on captures recorded before that existed.
- **Per-device streams.** Any `Telemetry('%s.csv' % self.name, ...)` takes its file name from the CONFIG, so the same driver appears under whatever the board named it.

## Contract

`tools/preflight.py` gates on this: it parses the sim's own capture header and requires every stream a renderer depends on to still resolve. A rename that would break the tools fails the gate first — that is the standing guard for findings §27.1.
