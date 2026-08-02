# TMS-7 pre-catapult evaluation — the first in-zone landings

Seven board HITL flights on the tree that is about to fly the catapult ladder: the standard four-case
matrix (E16/F15 × full 270 g / light 215 g, 5 % noise, calm) plus a zero-noise reference and two fault
cases. Flown after `bank_limit` 30 → 45, which is the change this study exists to evaluate.

[f15_full](f15_full.html) · [f15_light](f15_light.html) · [e16_full](e16_full.html) ·
[e16_light](e16_light.html) · [calm](calm.html) · [gnss_dead](gnss_dead.html) ·
[attitude_dead](attitude_dead.html) · [**overlay**](plan_matrix.svg) · [**video**](matrix.mp4)

## Results — 3 of 7 in-zone, and every case improved

| case | before | after | in-zone | apogee | duration |
|---|---|---|---|---|---|
| `f15_light` | 85.5 m | **14 m** | ✅ | 346 m | 62.9 s |
| `e16_full` | 88.7 m | **24 m** | ✅ | 127 m | 31.4 s |
| `e16_light` | 46.8 m | **38 m** | ✅ | 161 m | 36.4 s |
| `calm` (0 % noise) | 113 m | **52 m** | no | 288 m | 57.8 s |
| `gnss_dead` (60 s) | 152 m | **60 m** | no | 288 m | 54.8 s |
| `attitude_dead` (30 s) | 138 m | **79 m** | no | 288 m | 55.3 s |
| `f15_full` | 121.7 m | **82 m** | no | 288 m | 55.4 s |

**These are the first in-zone landings this project has recorded.** Before this change every case in
every matrix read `in-zone: no`, including flights with no noise and no faults at all.

## What changed, and why it was not obvious

The loiter orbit is flown under `bank_limit`, which was **30°**. At the measured 15.6 m/s the turn floor
is `R_min = v²/(g·tan φ)` ≈ **40 m**, while `loiter_radius_m` commands **30 m** — the law was asking for
a circle physically tighter than the airframe could fly. The heading controller saturated (`roll_sp`
pinned at ±30 with `heading_err` sustained at 112–157°) and settled into a limit cycle: a ~65 m orbit
centred ~95 m off target, swinging 28 m ↔ 163 m on a ~26 s period. The period is simply how long coming
around at 30° takes. At 45° the floor drops to ~25 m and fits inside the command.

Two wrong diagnoses preceded it, both worth remembering. The first read the 121 m miss as a navigation
failure — but the zone centre is only **49.5 m from the pad** against ~860 m of available glide range,
so it was never a range problem. The second concluded bank authority was irrelevant, having checked
`land_bank_limit` (45, the *final approach*) rather than `bank_limit` (30, the *loiter*). `fin_cap` sat
at 45 the whole flight, so fin authority genuinely never clipped — the demand did.

## Fault tolerance, re-measured at the new bank

`gnss_dead` (60 s blackout, covering essentially the whole glide) now lands **60 m** out and
`attitude_dead` **79 m**, against a 52 m calm reference — so a dead receiver costs ~8 m and a dead
BNO055 ~27 m. The earlier fault matrix priced a dead GNSS at **720 m** before dead reckoning existed,
and at 152 m before this bank change.

Do not rank the two fault cases against each other from one run each: both inject at a *random* glide
moment, so single-seed scatter is large.

## Still open

- **`f15_full` at 82 m is the worst case** and the only full-mass F15 miss above 60 m. It has the most
  energy to dissipate (288 m apogee, 270 g) — the endgame likely still runs out of orbit before it runs
  out of altitude.
- **`R_min` ≈ 25 m now leaves only 5 m under `loiter_radius_m` 30 m.** The radius may be the next binding
  limit rather than the bank; check before reaching for more bank angle.
- Pattern selection (o / oo / oval) was never implicated and is unchanged.

## Chart changes in this round

Two readability fixes to `flight_report.py`, applied to every HTML here:

- **Stage markers wrap onto two lines** (`stage` / `setting`) instead of `stage -> setting`, and are
  shifted clear of their vline and right-aligned against it. They sit on a vline in an eleven-row stack,
  and the arrow form ran wide enough to overlap its neighbours whenever two transitions landed close
  together; wrapping fixed that but left the two short lines crowding the line itself, so the offset and
  alignment finish the job — both lines end at the edge nearest the marker they describe.
- **Stage markers carry the stage's DURATION** as a third line, and neighbouring markers alternate
  vertically. `setting` and `boosting` are ~0.1 s apart, so no horizontal offset can separate them —
  one has to move down. Alternating by index holds whatever the spacing, where a proximity test would
  fail on a run of three close transitions.
- **The motor BURN is shaded inside BOOSTING** (`--motor E16|F15`). BOOSTING spans burn *plus* coast to
  separation, and only the coast is under meaningful aerodynamic control — the fins have little
  authority while the motor dominates. The band makes that split visible rather than inferred, and a
  separation falling *inside* the burn is a fault the eye catches at once.
- **The 3D track is coloured BY STAGE**, one trace each, replacing the Viridis time gradient. A time
  ramp answers "when", which the hover already gives to a tenth of a second; the question actually asked
  of that chart is where it boosted, where it glided, where it turned for the landing. Stage boundaries
  are invisible on a smooth gradient. Each stage is now a legend entry, so it can be toggled off to see
  what lies underneath.

## Reproduce

```bash
tools/deploy.sh
for spec in "F15 f15_full 270" "E16 e16_full 270" "E16 e16_light 215" "F15 f15_light 215"; do
    set -- $spec
    bash tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False captures/precatapult "$3"
done
# faults (last two args are attitude_drop_s and gnss_drop_s)
bash tools/hitl_collect.sh F15 calm          0.0  0.0 210.0 False captures/precatapult 270 0 0 False 0  0
bash tools/hitl_collect.sh F15 gnss_dead     0.05 0.0 210.0 False captures/precatapult 270 0 0 False 0  60
bash tools/hitl_collect.sh F15 attitude_dead 0.05 0.0 210.0 False captures/precatapult 270 0 0 False 30 0
python3 tools/flight_metrics.py captures/precatapult --sort name
# reports: pass the motor so the burn band is drawn inside BOOSTING
python3 tools/flight_report.py captures/precatapult/f15_full.txt --motor F15 -o f15_full.html
```
