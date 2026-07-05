# TMS-7 warm_reboot — v3 weights, fly-long trim glide, reboot recovery, CC-less fallback

The first capture set on the **TMS-7 v3 measured weights** (booster with engine E16 165 g /
F15 182 g; glider 285 g full / 235 g light) flying the **fly-long trim law** under the objective
order set 7/05 (specs/coludo.md "Gliding"): **① fly as long as possible, ② land in-zone, ③ land
near the midpoint** — and the first to exercise the 7/04 field-autonomy features on the board:
**warm start** after a simulated mid-glide reboot and the **CC-less spiral-landing fallback**.
Calm, 5 % noise, `inject_hz=25`; the sim polar is pinned at **air quality 2 (worst case,
−7 m/s trim sink)** — the real airframe is expected at quality 4–6 (~10 min capacity), so every
duration below is a FLOOR; the polar re-calibrates from the first real glide telemetry.

## The eight cases

| case | stack | apogee | glide (boost→done) | leak → OOM | avg servo | touchdown |
|---|---|---|---|---|---|---|
| [E16 full](e16_full.html) | 450 g | 130 m | 35.2 s | 236 KB/s → ~138 s | 0.51 W | **77 m, IN-ZONE** |
| [E16 light](e16_half.html) | 400 g | 161 m | 40.5 s | 235 → ~138 s | 0.52 W | **13.5 m, IN-ZONE** |
| [F15 full](f15_full.html) | 467 g | 292 m | 62.9 s | 233 → ~140 s | 0.56 W | 109 m (endgame variance) |
| [F15 light](f15_half.html) | 417 g | 345 m | 71.3 s | 232 → ~140 s | 0.53 W | **16.6 m, IN-ZONE** |
| [E16 full + reboot](e16_full_reboot.html) | 450 g | 130 m | 18.9 s | 235 | 0.31 W | 132 m (see caveat) |
| [F15 full + reboot](f15_full_reboot.html) | 467 g | 292 m | 29.9 s | 232 | 0.41 W | 215 m (see caveat) |
| [E16 full + noCC](e16_full_nocc.html) | 450 g | 130 m | 35.4 s | 237 | 0.56 W | 54 m off the FALLBACK centre |
| [F15 full + noCC](f15_full_nocc.html) | 467 g | 292 m | 62.9 s | 239 | 0.60 W | 209 m off the FALLBACK centre |

0 over-current alerts across all eight. 🎬 **[`tms7_warm.mp4`](tms7_warm.mp4)** — the follow-cam
movie of all eight, FHD 30 FPS, each segment titled with WHAT IS FLYING (airframe weight, engine,
scenario).

## Objective ① — fly long: ACHIEVED, and it changed the game

The trim law (hold the trim attitude; spend altitude only in the orbit's banked turns) against
the OLD pitch-0 law on the same v3 masses and the same worst-case polar: F15-full glide 43.0 →
**62.9 s (+46 %)**, E16-full 27.0 → **35.2 s (+30 %)**. At the quality-5 calibration the same
mechanism measured 101 s / 1.42 km of air path from a 292 m apogee — the polar, not the law, is
now the limit. The headline consequence: **the E16 reaches the zone for the first time** — with
the energy no longer burned in a forced descent, 130–160 m of apogee is enough (77 m / 13.5 m
in-zone), where every previous set landed E16 ~150–220 m short.

## Objectives ② / ③ — the endgame is now the open item (plan item 6 / task #9)

Three of the six zone-scored flights land in-zone; F15-full (109 m) and the noCC pair show the
flip side of the float: the glider lands wherever on the racetrack the energy runs out, because
the final-approach only engages below 8 m AGL. This is exactly the finalized glide-energy
program's step 2 (low-AGL orbit tightening + final-approach knobs, with the hard rule that ①
must not regress; acceptance: ≥ 80 % in-zone, median miss ≤ 30 m). Not tuned in this set — these
captures are its BASELINE.

## Warm reboot — mechanics proven; trajectories carry a sim caveat

3 s boot outage at a random early-glide moment (never LANDING; a BOOST-phase reset is
non-restorable by design — the separation latch still reads nested), then the REAL breadcrumb +
five-signal gate + restore:

| case | outage | elevation across it | after restore |
|---|---|---|---|
| E16 full | 10.4–13.4 s (3.0 s) | 116 → 64 m (−52 m) | gate PASS, control re-engaged, landed |
| F15 full | 12.9–15.9 s (3.0 s) | 280 → 229 m (−51 m) | gate PASS, control re-engaged, landed |

**Caveat:** during the outage the sim flies BALLISTIC (no lift in the fake SETTING, ~17 m/s
sink), so the model enters the restore in a dive it barely recovers from — the post-reboot
touchdowns reflect that artifact, not the law. A real airframe holds its trim glide through a
reboot (≈ 2–4 m/s sink ≈ ~10 m real cost). What the captures prove on-board: breadcrumb → gate →
restore fires, the fins resume, the flight completes to a detected landing.

## CC-less fallback (noCC cases)

Zone, sites and launch point wiped; the REAL field agent synthesized the spiral-landing zone
(+50 m north of the pad fix) inside the pre-ignition SETTING and both flights flew their full
profile against it. Touchdowns (54 / 209 m off the fallback centre) ride the same endgame
variance as the standard cases — the fallback zone is exactly as landable as an operator-set one,
no better and no worse, which is the design intent. (The earlier pitch-0 campaign hit 10.7 m
in-zone on this case — steep descents localize touchdowns; the fly-long priority deliberately
trades that away until the endgame tuning restores it.)

## Regenerate

```sh
mpremote connect /dev/ttyACM0 cp src/glider/config_hitl.py src/glider/sim_model.py \
  src/glider/tasks/hitl.py tools/hitl_run.py :
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
# 6 standard/reboot cases score vs ZONE, the 2 noCC cases vs FBZONE (reports + SVGs + flight_kpi)
# video: 8 "LABEL capture" pairs -- put WHAT IS FLYING in each label (weight, engine, scenario)
python3 tools/flight_video.py doc/sims/TMS-7-warm_reboot/tms7_warm.mp4 \
  "TMS-7v3 full weight (285 g), E16 engine" /tmp/hitl/warm/e16_full.txt ...
```
