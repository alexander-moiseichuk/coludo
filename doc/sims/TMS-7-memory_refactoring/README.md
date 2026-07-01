# TMS-7 memory refactoring — GC-off leak, on-board HITL (F15-4)

The airborne phase runs with **GC disabled** (`tasks/sequencer.py` calls `gc.disable()` at BOOSTING, and
`gc.enable()` + `gc.collect()` only at DONE — see `specs/coludo.md` "Garbage collection in flight"), so
every heap byte allocated in flight accumulates until the board runs out of PSRAM and the watchdog reboots
it **mid-air**. The PSRAM budget is only good for ~60 s of flight; a lighter build, a head-wind, or a wider
orbit all make the glide *longer*, so the leak — not the airframe — becomes the thing that kills the board.

This capture measures the leak **after** the Phase-3 memory refactoring, on the real ESP32-P4 via
`config_hitl` (real sensor drivers off; the `hitl` task feeds the *real* `sequencer` + `flight` + `pid` +
`mixer` + `navigation` a simulated 6-DoF body). The metric is the slope of `mem_free` in `board_health.csv`
over the GC-off window (BOOSTING → DONE); **time-to-OOM = free-at-boost / leak-rate**.

**Gate:** time-to-OOM ≥ **180 s** (3 min) → green. Otherwise the refactoring continues.

## What changed going in

The hot-path allocation work landed before this capture (all `src/glider`, full suite 43/43):

| item | change | measured |
|---|---|---|
| F02 | SG90 telemetry decimation | ~48 KB/s |
| F03 | flat IMU sample tuples (no accel+rate concat) | ~9 KB/s |
| U02 | INA226 telemetry decimation | — |
| `note()` | deferred error format — no eager `'%r' % e` on the hot path | leak-on-fault |
| Telemetry | precomputed row-format string — one `%` pass, no per-row generator/join | per-row |
| **F01** | **fixed-point PID** — integer millidegrees, **176 → 0 B/step** | **~47 KB/s** |

## Method

Two rounds, same scenario (F15-4, 5 % sensor noise, calm), differing only in glider mass:

- **normal** — the measured TMS-7 v2 stack (468 g).
- **half** — `mass_scale=0.5` (234 g): a 50 %-lighter build climbs higher (6.4 g vs 3.2 g boost) and glides
  **longer** — the worst case for a time-based leak. Added a `mass_scale` knob to `config_hitl.default()` /
  `hitl_run.fly()` for exactly this.

## Results

| round | flight (boost→done) | leak rate | free @ boost | low-water free | **time-to-OOM** |
|---|---|---|---|---|---|
| normal (468 g) | 40.0 s | **447 KB/s** | 30.4 MB | 14.8 MB | **~71 s** |
| half (234 g)   | 58.4 s | **435 KB/s** | 30.4 MB | **7.4 MB** | **~73 s** |

Raw traces: `f15_normal_health.csv` / `f15_half_health.csv` (+ `_sequencer.csv` for the stage timestamps).

## Verdict — not green (yet)

**Time-to-OOM ≈ 71–73 s**, well short of the 180 s gate. The leak is **time-based (~440 KB/s) and
weight-independent**: the half-weight glider is not safer for being lighter — it is *more* dangerous because
it stays airborne 58 s and bottoms out at **7.4 MB free**, ~17 s from OOM. A stronger head-wind or a wider
landing orbit would push a real flight past OOM and reboot the board in the flare.

**Caveat — HITL over-churns.** These numbers include the `sim_model` floating-point physics stepped at
50 Hz, which does **not** exist in a real flight (real sensors read into preallocated buffers instead). So
~440 KB/s is a **pessimistic** proxy; the real-flight leak is lower. But even generously discounting the sim
overhead, the trend is nowhere near 180 s — the refactoring must continue.

## Continue — remaining levers

Biggest expected wins, largest first:

1. **Binary telemetry** — the per-row string formatting + `.encode()` across every stream is still an
   allocator on each emitted row; a packed binary record (host-side decoders in Control) removes it wholesale.
2. **Sensor-read float boxing** — every driver's `sample()` boxes a float per axis in the scaling multiply
   (the same pattern F01 fixed in the PID), ×N sensors ×100+ Hz. The broad version of F01.
3. **F05** — databoard read-tuple reuse (~24 KB/s), medium risk.

Each lands with a full on-board suite + a re-run of this HITL pair; green when both rounds clear 180 s.

## Regenerate

```sh
# normal weight
tools/hitl_collect.sh F15 mem_normal 0.05 0.0 210.0 False /tmp/hitl_mem
# 50 % weight (mass_scale=0.5 via a one-line launcher)
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py tools/hitl_run.py :
printf 'import hitl_run\nhitl_run.fly("F15", 0.05, 0.0, 210.0, False, 0.5)\n' > /tmp/launch_half.py
tools/board_reboot.py /dev/ttyACM0 && mpremote connect /dev/ttyACM0 run /tmp/launch_half.py
adb pull /userdata/recordings/<session>_health.csv .
```
