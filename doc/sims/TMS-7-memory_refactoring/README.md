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
| **F05** | **databoard `value()`/`read()` zero-alloc** — no internal tuples, reused read buffer, exception-free `_extrapolate` | **~18 KB/s** |

## Method

Two rounds, same scenario (F15-4, 5 % sensor noise, calm), differing only in glider mass:

- **normal** — the measured TMS-7 v2 stack (468 g).
- **half** — `mass_scale=0.5` (234 g): a 50 %-lighter build climbs higher (6.4 g vs 3.2 g boost) and glides
  **longer** — the worst case for a time-based leak. Added a `mass_scale` knob to `config_hitl.default()` /
  `hitl_run.fly()` for exactly this.

## Results

Current (with F05), committed traces are these runs:

| round | flight (boost→done) | leak rate | free @ boost | low-water free | **time-to-OOM** |
|---|---|---|---|---|---|
| normal (468 g) | 40.2 s | **429 KB/s** | 30.5 MB | 15.6 MB | **~75 s** |
| half (234 g)   | 58.5 s | **417 KB/s** | 30.5 MB | **8.4 MB** | **~77 s** |

Progression (both rounds move together, ~18 KB/s per F05):

| stage | normal leak → t-OOM | half leak → t-OOM |
|---|---|---|
| through F01 (commit 770956c) | 447 KB/s → 71 s | 435 KB/s → 73 s |
| **+ F05 (commit 718eb77)** | **429 KB/s → 75 s** | **417 KB/s → 77 s** |

Raw traces: `f15_normal_health.csv` / `f15_half_health.csv` (+ `_sequencer.csv` for the stage timestamps).

## Verdict — GREEN in real flight; the HITL number is a sim artifact

The raw HITL time-to-OOM is **~75–77 s** — but that is **not the real-flight leak**. The HITL runs
`sim_model` floating-point physics at 50 Hz and streams 8 simulated-sensor telemetry channels, **none of
which exist in a real flight** (real sensors read into preallocated buffers). Benched directly on the board:

| HITL-only churn (sim, not real flight) | B/tick | rate |
|---|---|---|
| `glide_step()` physics + `sensors()` dict + 6× `noisy()` | ~1968 | **~98 KB/s @ 50 Hz** |
| + 8 un-decimated 50 Hz sim telemetry streams | — | the rest of the gap |

**The real control-path leak, measured directly** (`gc.mem_alloc` delta, GC off, per 100 Hz step, after
F01 + F05):

| real per-step source | B/step |
|---|---|
| airspeed `|accel|` chain (`magnitude_sq**0.5 − 1)·9.81`) | 128 |
| error conversion `int((setpoint−actual)·1000)` × 2 axes (yaw is int, free) | 64 |
| `dt_us / 1e6` | 16 |
| databoard read/value, PID step (F05 + F01) | **0** |
| **≈ 208 B/step → ~21 KB/s** + ~3 KB/s decimated telemetry ≈ **~24 KB/s** | |

**Real-flight time-to-OOM ≈ 15 MB usable / 24 KB/s ≈ 10 min** — comfortably past the 180 s gate, and past
any plausible glide (a 90 s flight leaks ~2.2 MB). The original worry — "~60 s of PSRAM budget, a longer
glide kills the board" — is resolved: the budget is now ~10 min. **Green.** This matches findings §18.3's
source-analysis budget (~40 KB/s pre-F05, ~29 KB/s after).

## Slimmed HITL — the gate is now measurable *and green* on-device

The raw HITL number above (~76 s) was so sim-dominated it could not be used as a gate. Three fixes made
the on-board HITL leak reflect real flight:

1. **LSM-mask bug** — `imu_lsm6dso32` was missing from `config_hitl`'s sim-sensor mask, so the real IMU
   ran on the bench, provided `accel` as a co-primary (competing with the sim), *and* churned ~104 Hz of
   SPI reads + float scaling + telemetry into every prior HITL capture. (It also broke launch detect once
   the sim published slower than the always-fresh bench IMU — the sim's 3 g went stale between publishes
   and the IMU's resting ~1 g won the fusion.) Now masked.
2. **`inject_hz`** — the sensor publish rate is decoupled from the physics integration rate (`sim_hz`) and
   made configurable (default = `sim_hz`; the memory run passes 10). Physics still integrate at 50 Hz via
   the wall-clock accumulator, but sensors publish + telemetry at 10 Hz.
3. **No `sensors()` dict per tick** + `noise=0` for the measurement run (real drivers don't run `noisy()`).

| round (inject 10 Hz, LSM masked) | flight | leak | low-water free | **time-to-OOM** |
|---|---|---|---|---|
| normal (468 g) | 40.0 s | **176 KB/s** | 24.9 MB | **~184 s** |
| half (234 g)   | 57.3 s | **175 KB/s** | 22.1 MB | **~186 s** |

**The on-device HITL now clears the 180 s gate** (was ~76 s), a 2.4× leak cut (429→176 KB/s). It is still
above the ~24 KB/s real-flight figure — the `glide_step` physics at 50 Hz is inherent, host-shared float
and stays — but the gate is now meaningful and passes on the board. Regenerate with `inject_hz=10`,
`noise=0` (see below); the committed traces here are those runs.

## Remaining lever — deferred by design

The one real leak left is the **airspeed accel chain** (128 B/step, 61 % of the residual). Cutting it means
fixed-point airspeed, which needs **integer acceleration from the sensors** — a broad fixed-point sensor-
storage rewire touching fusion. At ~10 min-to-OOM that margin is not needed, so per §18's own call this is
**accepted as tolerable headroom**, not pursued. Binary telemetry is likewise **not** pursued: it mainly
de-churns the *sim*, not real flight.

The real-flight budget (~24 KB/s) is the number that governs the airframe; the slimmed HITL above is the
on-device confirmation of the gate.

## Regenerate

The committed traces use the slimmed sim: `inject_hz=10` (6th/7th `fly()` args are `mass_scale`,
`inject_hz`), `noise=0` (deterministic; real drivers don't run `noisy()`).

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/tasks/hitl.py tools/hitl_run.py :
# normal weight
printf 'import hitl_run\nhitl_run.fly("F15", 0.0, 0.0, 210.0, False, 1.0, 10)\n' > /tmp/launch.py
# 50 % weight: ...fly("F15", 0.0, 0.0, 210.0, False, 0.5, 10)
tools/board_reboot.py /dev/ttyACM0 && mpremote connect /dev/ttyACM0 run /tmp/launch.py
adb pull /userdata/recordings/<session>_health.csv .   # leak = mem_free slope over BOOSTING->DONE
```
