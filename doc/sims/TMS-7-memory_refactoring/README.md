# TMS-7 memory-refactoring — on-board HITL capture set (E16 / F15 × full / half glider)

A complete on-board HITL capture of the **post-memory-refactoring firmware** (fixed-point PID, zero-alloc
databoard, deferred logging, global telemetry rate), flown over the corrected **TMS-7 v2 weight matrix**.
Every trace here — trajectory, attitude, fins, **real INA226 servo-rail power**, and **board vitals with the
GC-off leak** — is device telemetry from `config_hitl` (real sensors off; the `hitl` task feeds the *real*
`sequencer` + `flight` + `pid` + `mixer` + `navigation` a simulated 6-DoF body). Calm wind, 5 % sensor
noise, sim sensor rate 25 Hz (`inject_hz=25` — mid-way between the original 50 Hz and the slim 10 Hz).

## Weight model — booster + glider, separation drop

The **booster (motor + casing) ejects at separation**, so the boost phase carries the whole stack and the
glide carries the glider alone (`sim_model.Body` drops to the glide mass at `begin_glide()`). The glider is
airframe + electronics: **300 g today, ~150 g the weight-optimisation target**. Heavier v2 stacks read a
lower boost |a| (specific force = thrust/mass): F15 at 517 g ≈ 2.84 g, so the HITL launch threshold is
dropped to **`launch_g` 2.0** (config_default's real launch_g wants the same review for the v2 stack).

| motor | booster | glider (full / half) | whole stack (full / half) |
|---|---|---|---|
| E16 (16.1 N, 1.77 s) | 200 g | 300 / 150 g | **500 / 350 g** |
| F15 (14.4 N, 3.45 s) | 217 g | 300 / 150 g | **517 / 370 g** |

## The four flights (calm, 5 % noise, 25 Hz)

| config | whole | apogee | deploy @ | glide (boost→done) | GC-off leak | time-to-OOM | peak servo P |
|---|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 500 g | 107 m | 6.6 s | 24.2 s | 250 KB/s | ~130 s | 7.2 W |
| [E16 half](e16_half.html) | 350 g | 200 m | 8.0 s | 33.6 s | 250 KB/s | ~130 s | 7.1 W |
| [F15 full](f15_full.html) | 517 g | 247 m | 9.8 s | 39.0 s | 246 KB/s | ~132 s | 7.7 W |
| [F15 half](f15_half.html) | 370 g | 404 m | 11.1 s | 52.5 s | 244 KB/s | ~133 s | 7.4 W |

Deploy now fires via the sequencer's **baro apogee-detect** (peak − 5 m) — at the top of the arc for every
mass, e.g. F15-full apogee 247 m → deploy 9.8 s (was a fixed burn-timed ~7.4 s, 1.3 s *before* apogee).
0 over-current alerts across all four (servo rail held). The sim's baro noise was also cut to a realistic
sub-metre so the peak-detect is clean.

A lighter glider **climbs higher and glides longer** — the worst case for a time-based leak (F15 half glides
53 s). Each flight is a full interactive report (`<config>.html`, plotly 3D trajectory + linked series) and a
dependency-free SVG (`<config>.svg`, plan view + altitude/roll). 🎬 **[`tms7_memory.mp4`](tms7_memory.mp4)**
is a narrated follow-cam animation of all four.

**Each report's series panel adds two things over the standard capture:**
- **engine (INA226)** — real voltage / current / power / cumulative over-current alerts on the servo rail
  (the SG90s physically slew during HITL, so this is real draw: ~7 W peaks, ~4.9 V, 0 alerts here).
- a **GC-off leak headline** — the `mem_free` slope over BOOSTING→DONE and the extrapolated time-to-OOM,
  computed by `flight_report.leak_estimate()` and printed in the panel title.

## Memory verdict

These HITL leaks (~250 KB/s → ~130 s) are **still sim-inflated** — the `sim_model` float physics run every
tick and don't exist in a real flight. The **directly-measured real control-path leak is ~24 KB/s** (after
F01 fixed-point PID + F05 zero-alloc databoard; airspeed `|accel|` chain 128 B/step + error conv 64 B + dt
16 B, PID/databoard now 0), giving **real-flight time-to-OOM ≈ 10 min** — comfortably past the 180 s gate,
for any config in the matrix. The `inject_hz` knob trades sim fidelity for a tighter proxy: 10 Hz gave a
176 KB/s / ~185 s on-device reading; 25 Hz here keeps smoother control + denser data at ~250 KB/s. The
biggest correctness win along the way was masking `imu_lsm6dso32` in `config_hitl` (it had been running on
the bench and competing with the sim's accel in every prior capture).

## Weight optimisation context

The full/half columns bracket **current (300 g glider) to target (~150 g)**: lighter carbon wings (shrinkable)
plus electronics consolidation (2× 1-cell batteries → one C25-rated 1-cell, etc.). The matrix will be
re-pinned once **TMS-7 v3** is measured with the final electronics — which needs 5 PCBs built per device.

## Regenerate

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/sim_model.py \
  src/glider/tasks/hitl.py tools/hitl_run.py :
# fly(motor, noise, wind, wind_dir, spike, glider_g, inject_hz); 300 full / 150 half glider
printf 'import hitl_run\nhitl_run.fly("F15", 0.05, 0.0, 210.0, False, 300, 25)\n' > /tmp/cap.py
tools/board_reboot.py /dev/ttyACM0 && mpremote connect /dev/ttyACM0 run /tmp/cap.py
# pull every stream (incl. power_ina226 + health) then assemble + render:
adb pull /userdata/recordings/<session>_<stream>.csv <dir>/   # accel_adxl375 baro_icp10111 imu_bno055
                                          # gnss laser_agl fins health sequencer power_ina226
python3 tools/assemble_capture.py <session> <dir> f15_full.txt
PLOTLY_PY tools/flight_report.py f15_full.txt -o f15_full.html --cdn   # 3D + engine + leak
python3 tools/flight_svg.py f15_full.txt -o f15_full.svg --pad <pad> --zone <zone>
python3 tools/flight_video.py tms7_memory.mp4 F15-full f15_full.txt ...   # movie
```
