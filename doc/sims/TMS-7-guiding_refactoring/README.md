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
