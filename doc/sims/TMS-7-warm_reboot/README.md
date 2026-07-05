# TMS-7 warm_reboot — v3 weights + mid-flight reboot recovery + CC-less fallback landing

The first capture set on the **TMS-7 v3 measured weights** (models/TMS-7 README: booster with
engine E16 165 g / F15 182 g; glider 285 g full = construction 134.8 g + 150 g electronics, 235 g
light) and the first to exercise the two 7/04 field-autonomy features **on the board**: the
**warm start** after a simulated mid-glide reboot, and the **CC-less spiral-landing fallback**
(no zone, no sites — the field agent synthesizes the landing zone from the GNSS fix). Same knobs
as every set: calm, 5 % noise, `inject_hz=25`; every trace is device telemetry from `config_hitl`
through the real `sequencer`+`field`+`flight`+`guidance`+`governor`+`pid`+`mixer`.

## The eight cases

| case | stack | apogee | deploy | glide (boost→done) | leak → OOM | peak servo | touchdown |
|---|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 450 g | 130 m | 7.1 s | 27.0 s | 249 KB/s → ~131 s | 6.8 W | 223 m (zone unreachable) |
| [E16 light](e16_half.html) | 400 g | 160 m | 7.6 s | 30.0 s | 240 KB/s → ~135 s | 5.0 W | 149 m (zone unreachable) |
| [F15 full](f15_full.html) | 467 g | 292 m | 10.2 s | 43.0 s | 235 KB/s → ~139 s | 6.9 W | **24 m, IN-ZONE** |
| [F15 light](f15_half.html) | 417 g | 345 m | 10.8 s | 47.5 s | 234 KB/s → ~139 s | 7.3 W | **33 m, IN-ZONE** |
| [E16 full + reboot](e16_full_reboot.html) | 450 g | 130 m | 7.1 s | 19.7 s | 241 KB/s → ~135 s | 7.2 W | 135 m (see below) |
| [F15 full + reboot](f15_full_reboot.html) | 467 g | 292 m | 10.3 s | 31.0 s | 232 KB/s → ~141 s | 6.3 W | 219 m (see below) |
| [E16 full + noCC](e16_full_nocc.html) | 450 g | 130 m | 7.1 s | 27.2 s | 240 KB/s → ~135 s | 6.0 W | 194 m off the FALLBACK centre |
| [F15 full + noCC](f15_full_nocc.html) | 467 g | 292 m | 10.3 s | 43.4 s | 237 KB/s → ~137 s | 7.0 W | **10.7 m off the FALLBACK centre, IN-ZONE** |

0 over-current alerts across all eight. 🎬 **[`tms7_warm.mp4`](tms7_warm.mp4)** is the follow-cam
movie of all eight (30 FPS — dropped from 50 to save space).

## Warm reboot — the mechanics work; read the trajectories with the sim caveat

Each reboot case injects a **3 s boot outage at a random early-glide moment** (never LANDING —
too little altitude to prove anything but luck; a BOOST-phase reset is non-restorable **by
design**, the separation latch still reads nested): disarm + neutral fins + stage SETTING under a
manual hold, then the REAL breadcrumb is loaded and the REAL five-signal gate decides, restoring
GLIDING + armed exactly as `main._restore_flight` does.

| case | outage | elevation across it | after restore |
|---|---|---|---|
| E16 full | 12.0–15.1 s (3.1 s) | 107 → 44 m (**−63 m**) | gate PASS, control re-engaged, landed |
| F15 full | 15.4–18.4 s (3.0 s) | 268 → 207 m (**−61 m**) | gate PASS, control re-engaged, landed |

**Caveat that dominates the touchdown numbers:** during the simulated outage the HITL body flies
**ballistic** (the sim applies no lift in SETTING), sinking ~20 m/s — so the model enters the
restore in a dive it barely recovers from, and the post-reboot trajectory (especially F15's
219 m) reflects that artifact, not the control law. A real airframe holds its **trim glide**
through a reboot (2–4 m/s sink ≈ ~10 m real cost), so real recovery margins are far better than
shown. What the captures DO prove on-board: the breadcrumb → gate → restore chain fires, the fins
resume (energy 0.33/0.45 W post-restore), and the flight completes to a detected landing.

## CC-less fallback landing (noCC cases)

Mission zone, sites and launch point wiped; the **field agent** synthesized the spiral-landing
zone from the first sim GNSS fix (+50 m north of the pad at the configured bearing) and the
glider flew the normal orbit-and-land against it. **F15 landed 10.7 m from the fallback centre,
in-zone** — the emergency zone is as landable as an operator-set one. E16 landed 194 m out for
the same reason it never reaches the HPRC zone either: with a 130 m apogee its recovery + glide
carries it ~200 m downrange regardless of where the zone sits — an airframe/energy property, not
a fallback defect.

## Control quality vs TMS-7-guiding_refactoring (the 4 standard cases)

**Read with care: same law, different physics.** The guiding set flew v2 masses (booster 200/217,
glider 300/150); this set flies v3 (165/182, 285/235), so the deltas mix airframe change with
anything else. The only near-iso-mass pair is **F15-full** (300 → 285 g):

| KPI (F15-full) | guiding (v2) | warm_reboot (v3) |
|---|---|---|
| touchdown miss | 65.6 m, in-zone | **24.0 m, in-zone** |
| avg servo power | 0.54 W | **0.43 W** |
| fin travel /s | 190 °/s | 129 °/s |
| glide | 39.0 s | 43.0 s (higher apogee 292 vs 246 m) |

The law flies the lighter v3 stack **tighter and cheaper** — more altitude margin turns into a
cleaner orbit + final. F15-light also converts: 33 m in-zone where the (much lighter, 150 g) v2
half landed 89 m outside. The E16 cases stay physics-limited (130–160 m apogee cannot reach the
~200 m HPRC zone) exactly as in every previous set.

## Regenerate

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/sim_model.py \
  src/glider/tasks/hitl.py tools/hitl_run.py :
# hitl_collect.sh gained [reboot_s] [no_cc] tails; v3 masses are the config_hitl defaults
for case in "E16 e16_full 285 0 False" "E16 e16_half 235 0 False" \
            "F15 f15_full 285 0 False" "F15 f15_half 235 0 False" \
            "E16 e16_full_reboot 285 3.0 False" "F15 f15_full_reboot 285 3.0 False" \
            "E16 e16_full_nocc 285 0 True" "F15 f15_full_nocc 285 0 True"; do
  set -- $case
  PORT=/dev/ttyACM0 tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False /tmp/hitl/warm "$3" 25 "$4" "$5"
done
PLY=~/.local/share/pipx/venvs/plotly/bin/python; PAD=25.514379,-80.391795
ZONE=25.514944,-80.392972,25.514583,-80.391111
FBZONE=25.5150078,-80.3922927,25.5146485,-80.3912973   # fallback: +50 m north of the pad
for f in e16_full e16_half f15_full f15_half e16_full_reboot f15_full_reboot; do
  "$PLY" tools/flight_report.py /tmp/hitl/warm/$f.txt -o doc/sims/TMS-7-warm_reboot/$f.html --cdn
  python3 tools/flight_svg.py /tmp/hitl/warm/$f.txt -o doc/sims/TMS-7-warm_reboot/$f.svg --pad $PAD --zone $ZONE
done
for f in e16_full_nocc f15_full_nocc; do  # the noCC cases score against the FALLBACK zone
  "$PLY" tools/flight_report.py /tmp/hitl/warm/$f.txt -o doc/sims/TMS-7-warm_reboot/$f.html --cdn
  python3 tools/flight_svg.py /tmp/hitl/warm/$f.txt -o doc/sims/TMS-7-warm_reboot/$f.svg --pad $PAD --zone $FBZONE
done
python3 tools/flight_video.py doc/sims/TMS-7-warm_reboot/tms7_warm.mp4 \
  E16-full ... F15-noCC ...   # 8 LABEL capture pairs, 30 FPS default
python3 tools/flight_kpi.py "F15-full:/tmp/hitl/warm/f15_full.txt" ...          # KPIs (HPRC zone)
python3 tools/flight_kpi.py "F15-noCC:/tmp/hitl/warm/f15_full_nocc.txt" --zone $FBZONE
```
