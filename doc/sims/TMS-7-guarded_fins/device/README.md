# Device-collected vitals — on-board HITL (real load / temp / mem)

Unlike the host sweep in the parent directory (where `board_health` is **synthetic / phase-modeled**,
because the host has no MCU), these two files are **real telemetry from the ESP32-P4**, recorded while
the board flew a guarded HITL flight and streamed to the Luckfox recorder.

- **`health.csv`** — the real `board_health` task: MCU temperature, `gc.mem_free()`, and CPU load
  (estimated from probe-task wake-up lateness), once per second through the flight.
- **`sequencer.csv`** — the real on-board stage machine, with the actual transition *reasons*.

## How it was collected

The workflow (board control split across the three tools):

1. **rshell** — copy a small HITL runner to the board and fly it:
   `rshell -p /dev/ttyACM0 cp hitl_run.py /pyboard/` then
   `boardrun.py /dev/ttyACM0 runfile hitl_run.py 90`. The runner brings up `config_hitl` (real sensors
   off, `hitl` + `flight` + `sequencer` + `health` + `recorder` on) and runs the loop to `DONE`.
2. **Recorder → Luckfox** — the board streams telemetry over UART:1 (GPIO20, 921600) to the Luckfox,
   which demuxes it into per-stream CSVs under `/userdata/recordings/<session>_<file>.csv`.
3. **adb** — pull the session: `adb pull /userdata/recordings/<session>_health.csv`.

The board flew the real stage machine end to end:

```
boosting   launch |a|=3.5g
gliding    burnout timeout (no separation)
landing    agl -12.0m
done       stationary 1.1g
```

## Real vs the synthetic model

The host sweep's `health.csv` was a guess. The board says otherwise:

| metric | **real (this board)** | synthetic (host model) |
|---|---|---|
| temperature | **31–32 °C**, steady | 45–63 °C, drifting up |
| mem_free | **~31–32 MB**, shallow GC sawtooth | ~4 MB |
| CPU load | **0–6 % cruising, one 47 % peak at the landing transition** | 30–60 % per stage |

So the P4 runs **far cooler, emptier and more idle** than the model assumed — the control loop barely
loads it (single core, but mostly asleep between 50 Hz steps), and the 32 MB PSRAM is nearly untouched.
The model did get the *shape* right: load peaks at landing (the laser hammering I²C). These numbers are
the ones to fold back into the synthetic generator, or just read here for the truth.

## Caveats

- **Sim sensors are not recorded on-board.** In HITL the real sensor drivers are disabled (the `hitl`
  task provides their quantities on the databoard instead), so this session has `health` + `sequencer`
  + fin commands but no accel/imu/baro/gnss/laser CSVs. The trajectory reference stays the host sweep;
  this is the *vitals* reference.
- **The glide was timing-degenerate.** With separation off, `boosting → gliding` waits on the burnout
  timeout, which here fired after the model had already fallen back (`gliding` then immediately
  `landing` at agl −12 m). Fine for measuring board vitals under the running control stack; not a
  representative glide. A faithful on-board glide needs the apogee/separation timing tightened in
  `config_hitl` — a separate fidelity fix.
