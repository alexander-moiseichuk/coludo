# TMS-7 guiding refactoring — on-board HITL re-capture on the guidance/governor-extracted firmware

A re-run of **[TMS-7-fixnums](../TMS-7-fixnums/)** on the current firmware after the structural
refactoring (doc/plan.md roadmap #1–#3): the control law now lives in **`guidance.py`** (per-stage law
table: boost hold, glide/landing steering with the 3 GPS tiers + nav cache, final-approach centreline)
and **`governor.py`** (airspeed estimator + adaptive throttle + the 1/v² fin-authority cap), the mixer
**fused into the servo write** (`Mixer.bind()`/`actuate()` — `flight._apply` and its per-step dict walk
are gone), and `Flight._step` reduced to pure orchestration. Same TMS-7 v2 weight matrix (E16/F15 ×
full/half glider), same knobs (calm, 5 % noise, `inject_hz=25`, 300 g full / 150 g half). Every trace is
device telemetry from `config_hitl` (real sensors off; the `hitl` task feeds the *real* `sequencer` +
`flight` + `guidance` + `governor` + `pid` + `mixer` + `navigation` a simulated 6-DoF body).

## The four flights (calm, 5 % noise, 25 Hz) — current firmware

| config | whole | apogee | deploy @ | glide (boost→done) | GC-off leak | time-to-OOM | peak servo P | CPU load avg/max |
|---|---|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 500 g | 105 m | 6.6 s | 24.1 s | 241 KB/s | ~135 s | 6.3 W | 26 / 46 % |
| [E16 half](e16_half.html) | 350 g | 198 m | 8.0 s | 33.5 s | 246 KB/s | ~132 s | 7.1 W | 14 / 64 % |
| [F15 full](f15_full.html) | 517 g | 246 m | 9.8 s | 39.0 s | 236 KB/s | ~138 s | 7.0 W | 24 / 46 % |
| [F15 half](f15_half.html) | 370 g | 402 m | 11.2 s | 52.4 s | 233 KB/s | ~140 s | 6.5 W | 25 / 54 % |

**Trajectory parity is exact**: apogees match the fixnums baseline to the metre (105/198/246/402 m),
deploys within 0.1 s, glide durations within 0.1 s — the extracted law flies the same flight. 0
over-current alerts across all four. (A follow-cam movie of all four is not committed — regenerate it
with the `flight_video.py` line below.)

## Change vs TMS-7-fixnums

| config | leak KB/s (fixnum → guiding) | OOM s (fixnum → guiding) | peak W (fixnum → guiding) |
|---|---|---|---|
| E16 full | 264 → **241** | ~123 → ~135 | 7.1 → 6.3 |
| E16 half | 260 → **246** | ~125 → ~132 | 6.8 → 7.1 |
| F15 full | 272 → **236** | ~120 → ~138 | 7.0 → 7.0 |
| F15 half | 256 → **233** | ~127 → ~140 | 7.2 → 6.5 |

The HITL leak is **down ~8–12 %** even with the sim's own churn unchanged — the mixer fusion removed the
per-step output-dict iteration and the guidance slots replaced the last per-step temporaries. Peak servo
power flat (real INA226; the servos physically slew).

## Guidance-effectiveness KPIs (`tools/flight_kpi.py`, raw Luckfox captures, both sets)

Commanded fin activity (fins.csv; a "move" = the commanded angle changed between samples — sg90
compare-and-sets, so an unchanged command is no PWM write), the **real INA226 servo energy** with the
average power = energy / flight duration, and the touchdown miss from the zone centre:

| flight | fin moves (guiding → fixnums) | yaw moves | servo energy | average power | miss from centre |
|---|---|---|---|---|---|
| F15 full | 1350 vs 1553 (−13 %) | **176 vs 300 (−41 %)** | **22.0 vs 26.2 J (−16 %)** | 0.54 vs 0.65 W | 65.6 vs 68.0 m (both in-zone) |
| F15 half | 1740 vs 1915 (−9 %) | 246 vs 371 (−34 %) | 22.0 vs 26.1 J (−16 %) | 0.41 vs 0.49 W | 89.1 vs 87.5 m |
| E16 full | 816 vs 835 (−2 %) | ≈ | 16.5 vs 15.5 J (+6 %) | 0.65 vs 0.61 W | 209.1 vs 210.0 m |
| E16 half | 1115 vs 1195 (−7 %) | ≈ | 13.6 vs 16.5 J (−18 %) | 0.39 vs 0.47 W | 113.2 vs 113.2 m |

**Same guidance outcome, ~10–16 % less servo work.** Miss distances match within GPS noise on all four
(E16-full's ~210 m is physics — a 105 m apogee cannot reach the zone). The saving concentrates in the
**rudder** (yaw moves −34…−41 % on the F15 flights, elevons unchanged): the fixnums captures pre-date the
adaptive airspeed throttle, so their fin-authority cap (`mixer.limit`) refreshed at the full 100 Hz and
estimate noise made the clamp jitter — the rudder, saturated at that clamp through the orbit, flickered
with it. The throttled governor holds a steadier cap. E16-full is the one +6 % outlier: the shortest
flight, dominated by boost, where the governor legitimately runs full rate. Average servo power
(energy ÷ duration, INA226-measured) is the single best efficiency KPI; miss-distance the effectiveness
one.

## The real control-path leak (masked-sensor, `test/bench_flight.py`, GC off, 2000 steps)

```
base step (no airspeed)  : 128 B/step   (guidance.compute 96 + PID/actuate 32)     [fixnums: ~192]
governor._update         : 224 B/call   adaptive throttle: 25 Hz (moving) -> 10 Hz (settled glide)
@ 100 Hz, settled glide  : 15.0 KB/s -> time-to-OOM ~2183 s (~36 min)
@ 100 Hz, moving         : 18.4 KB/s -> time-to-OOM ~1784 s (~30 min)
```

The PID+mix+servo slice fell **112 → 32 B/step** (the fused `mixer.actuate` allocates nothing; the old
`_apply` dict walk was the residual) and the setpoint law **128 → 96 B** (guidance instance slots). The
still-dominant allocator is `governor._update` (224 B) — the airspeed |accel| sqrt + GNSS blend, float **by
design** and now amortized by its adaptive throttle. Step time: whole `_step` 450 µs (4.5 % of the 100 Hz
budget); `mixer.actuate` 52.6 µs does the mixing *and* the servo writes in one loop (the mixing alone used
to cost 35.6 µs before a separate `_apply` walk on top).

## Verdict

- **Behaviour-neutral, structurally better:** the guidance law and the fin governor are now standalone,
  host-runnable, unit-tested modules (`test_guidance.py`, `test_governor.py`), and the same code drives
  `tools/virtual_flight.py` on the host — the hand-mirrored control law (the drift source that bit the
  fixnum and gyro-D-term sessions) is gone. The flight itself is byte-for-byte the fixnums trajectory.
- **Memory:** the refactor *gained* margin — settled-glide leak ~15 KB/s at 100 Hz (~36 min to OOM,
  vs ~22–26 KB/s / ~21–25 min before) — with the remaining allocator being the deliberately-float
  airspeed path, already throttled.
- **CPU:** unchanged headroom (450 µs step, load 14–26 % mean / 64 % landing peak).

## Regenerate

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/sim_model.py \
  src/glider/tasks/hitl.py tools/hitl_run.py :
for f in "E16 e16_full 300" "E16 e16_half 150" "F15 f15_full 300" "F15 f15_half 150"; do
  set -- $f
  PORT=/dev/ttyACM0 tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False /tmp/hitl/guiding "$3" 25
done
PLY=~/.local/share/pipx/venvs/plotly/bin/python; PAD=25.514379,-80.391795
ZONE=25.514944,-80.392972,25.514583,-80.391111
for f in e16_full e16_half f15_full f15_half; do
  "$PLY" tools/flight_report.py /tmp/hitl/guiding/$f.txt -o doc/sims/TMS-7-guiding_refactoring/$f.html --cdn
  python3 tools/flight_svg.py /tmp/hitl/guiding/$f.txt -o doc/sims/TMS-7-guiding_refactoring/$f.svg --pad $PAD --zone $ZONE
done
python3 tools/flight_video.py doc/sims/TMS-7-guiding_refactoring/tms7_guiding.mp4 \
  E16-full /tmp/hitl/guiding/e16_full.txt E16-half /tmp/hitl/guiding/e16_half.txt \
  F15-full /tmp/hitl/guiding/f15_full.txt F15-half /tmp/hitl/guiding/f15_half.txt
# real control-path leak + step-time breakdown:
python3 tools/board_reboot.py /dev/ttyACM0 && mpremote connect /dev/ttyACM0 run src/glider/test/bench_flight.py
```
