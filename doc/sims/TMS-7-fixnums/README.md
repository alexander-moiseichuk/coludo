# TMS-7 fixnums — on-board HITL re-capture on the fixed-point + gyro-D-term firmware

A re-run of **[TMS-7-memory_refactoring](../TMS-7-memory_refactoring/)** on the current firmware — the
**fixed-point (`fixnum`) control path** (centidegree integer PID + attitude + driver internals), the
**gyro-rate PID D term** (the LSM6DSO32 `rate` now damps every axis), and integer INA226 milli-units — over
the same **TMS-7 v2 weight matrix** (E16/F15 × full/half glider), same knobs (calm, 5 % noise,
`inject_hz=25`, 300 g full / 150 g half). Every trace is device telemetry from `config_hitl` (real sensors
off; the `hitl` task feeds the *real* `sequencer` + `flight` + `pid` + `mixer` + `navigation` a simulated
6-DoF body). New this set: an **`imu_lsm6dso32` gyro-rate panel** in each report (the D-term input).

## The four flights (calm, 5 % noise, 25 Hz) — current firmware

| config | whole | apogee | deploy @ | glide (boost→done) | GC-off leak | time-to-OOM | peak servo P | CPU load avg/max |
|---|---|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 500 g | 105 m | 6.7 s | 24.2 s | 264 KB/s | ~123 s | 7.1 W | 25 / 46 % |
| [E16 half](e16_half.html) | 350 g | 198 m | 8.1 s | 33.5 s | 260 KB/s | ~125 s | 6.8 W | 27 / 76 % |
| [F15 full](f15_full.html) | 517 g | 246 m | 9.8 s | 39.0 s | 272 KB/s | ~120 s | 7.0 W | 11 / 32 % |
| [F15 half](f15_half.html) | 370 g | 402 m | 11.2 s | 52.3 s | 256 KB/s | ~127 s | 7.2 W | 27 / 53 % |

Stage machine, apogee-detect deploy, glide durations and peak servo draw are all **within noise of the
baseline** — the control law flies the same trajectory. 0 over-current alerts across all four.

## Change vs TMS-7-memory_refactoring

| config | leak KB/s (mem → fixnum) | OOM s (mem → fixnum) | peak W (mem → fixnum) |
|---|---|---|---|
| E16 full | 250 → **264** | ~130 → ~123 | 7.2 → 7.1 |
| E16 half | 250 → **260** | ~130 → ~125 | 7.1 → 6.8 |
| F15 full | 246 → **272** | ~132 → ~120 | 7.7 → 7.0 |
| F15 half | 244 → **256** | ~133 → ~127 | 7.4 → 7.2 |

**The HITL leak is flat-to-slightly-up (~+5 %), and that rise is a SIM artifact, not the flight.** The HITL
GC-off leak is dominated by the `sim_model` float physics that run every tick and don't exist in a real
flight; this set *adds* per-tick sim allocation the baseline didn't have — the gyro-rate noise (3× `noisy`)
and the new `imu_lsm6dso32` telemetry row — so the sim-side number ticks up. Peak servo power is flat-to-
slightly-down (real INA226; the servos physically slew).

## The number that actually moved: the real control-path leak

The meaningful figure is the **masked-sensor control path** (what runs airborne), measured directly by
`test/bench_flight.py` (GC disabled, fresh databoard, 2000 steps) — not the sim-inflated HITL number:

```
_step allocation         :  416 B/step
  _update_airspeed       :  224 B   |accel| sqrt + 1/v² fin governor -- FLOAT BY DESIGN (airspeed left float)
  _compute_setpoints     :  128 B   setpoint from_float + bank_demand (nav/bank kept float)
  _run_pid (PID+mix+apply):  112 B   pid.step itself = 0 B; residual is _apply dict-iter + mixer
@ 100 Hz -> 41.6 KB/s -> time-to-OOM ~783 s (~13 min) from 32.5 MB free  (>> the 180 s gate)
```

`bench_pid_alloc.py` confirms `pid.Pid.step` is **0 B/step** — at err=5°, the ±180° swing, *and* with the
gyro-rate D term (was 176 B/step on the old float PID). So the fixed-point control arithmetic is alloc-free;
the residual leak is the **deliberately-float** airspeed/nav math (the user's call: "HITL and complex math
lets safely leave in floats") plus `_apply`'s dict iteration.

## Verdict

- **Memory:** the big win (fixed-point PID + zero-alloc databoard) was **already banked** in the
  memory_refactoring baseline. This session's fixnum work (attitude → centidegrees, gyro D term, INA226
  integers) removes the per-axis attitude *error* float conversion and keeps the control path integer, but
  the dominant residual allocator (`_update_airspeed`, 224 B) is float by design — so the real-flight leak
  is **~flat**, with a comfortable ~13 min OOM at 100 Hz vs the 180 s gate.
- **CPU:** a control step is ~400 µs (~4 % of the 100 Hz budget); on-board HITL load runs ~11–27 % mean,
  spiking to ~76 % when the laser hammers I2C on landing — unchanged headroom.
- **What this campaign really validates:** the fixed-point + gyro-D-term firmware **flies the same
  trajectory, at the same power and CPU, with the same (real-path) memory profile** — i.e. the migration
  is behaviour-neutral, while the control path is now integer, the gyro is consumed, and telemetry is
  float-free. See `../TMS-7-memory_refactoring/` for the pre-fixnum baseline and `doc/plan.md` for the
  viperization (#6) decision (deferred on the `bench_flight` step-time breakdown).

## Regenerate

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/sim_model.py \
  src/glider/tasks/hitl.py tools/hitl_run.py :
# hitl_collect.sh now takes trailing glider_g + inject_hz and pulls power_ina226 + imu_lsm6dso32:
for f in "E16 e16_full 300" "E16 e16_half 150" "F15 f15_full 300" "F15 f15_half 150"; do
  set -- $f
  PORT=/dev/ttyACM0 tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False /tmp/hitl/mem "$3" 25
done
PLY=~/.local/share/pipx/venvs/plotly/bin/python; PAD=25.514379,-80.391795
ZONE=25.514944,-80.392972,25.514583,-80.391111
for f in e16_full e16_half f15_full f15_half; do
  "$PLY" tools/flight_report.py /tmp/hitl/mem/$f.txt -o doc/sims/TMS-7-fixnums/$f.html --cdn
  python3 tools/flight_svg.py /tmp/hitl/mem/$f.txt -o doc/sims/TMS-7-fixnums/$f.svg --pad $PAD --zone $ZONE
done
# real control-path leak + step-time breakdown:
mpremote connect /dev/ttyACM0 run src/glider/test/bench_flight.py
```
