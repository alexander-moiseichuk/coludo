# Telemetry schema

> **GENERATED from the sources by `tools/gen_schema.py` — do not hand-edit.** Regenerate after changing any `recorder.Telemetry(...)` declaration (`python3 tools/gen_schema.py`); `--check` fails the local gate if it is stale.

> **The `host sim` origin is currently EMPTY, and that is a known gap, not agreement.** `tools/virtual_flight.py` builds its streams by hand rather than through `recorder.Telemetry(...)`, so this generator cannot see them — host/board schema drift would go undetected on the host side.

Every stream a capture can contain, and the fields in each. A recorder capture interleaves `@<session>_<file>@<row>` telemetry rows with plain log lines; `tools/flight_telemetry.py` demuxes them, and every renderer resolves streams **by role** (the fields they carry) rather than by file name — a capture's file names track the fitted hardware, so a fallback flight names them differently.

## Streams

| stream | origin | declared in | fields |
|---|---|---|---|
| `accel_adxl375.csv` | board | `hitl.py` | `ax`, `ay`, `az` |
| `airspeed_sdp810.csv` | board | `hitl.py` | `dynamic_pressure`, `airspeed_cms`, `temperature` |
| `baro_icp10111.csv` | board | `hitl.py` | `altitude`, `temperature`, `pressure`, `elevation` |
| `checkpoint.csv` | board | `warmstart.py` | `stage`, `altitude`, `speed`, `airspeed`, `ticks_ms` |
| `fins.csv` | board | `hitl.py` | `eleron_left`, `eleron_right`, `yaw` |
| `flight.csv` | board | `flight.py` | `stage`, `active`, `airspeed_cms`, `fin_cap`, `roll_sp`, `pitch_sp`, `heading_err`, `roll_cmd`, `pitch_cmd`, `yaw_cmd`, `wind_cms`, `wind_from` |
| `gnss.csv` | board | `hitl.py` | `lat`, `lon`, `speed_kn`, `course` |
| `health.csv` | board | `board_health.py` | `temp`, `mem_free`, `load`, `oom_s`, `land_s`, `leak_kbps`, `rescues`, `rescue_ms` |
| `imu_bno055.csv` | board | `hitl.py` | `heading`, `roll`, `pitch` |
| `imu_lsm6dso32.csv` | board | `hitl.py` | `ax`, `ay`, `az`, `gx`, `gy`, `gz` |
| `laser_agl.csv` | board | `hitl.py` | `agl` |
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
