# TMS-7 — board v1.1 (SEN0697): the new attitude module, and the magnetometer it brings

**54 board flights** — r5, the standard matrix (3 combos × 10 scenarios), and r6, 12 matched pairs flown
twice — after the SEN0253 came off the breadboard and the **SEN0697** went on in its place, one module
carrying three parts where the old one carried two:

| gone (v1.0) | arrived (v1.1) | channels |
|---|---|---|
| BNO055 `0x28` — fused attitude | BMI323 `0x69` — accel + gyro, INT1-driven | `attitude` → `accel`/`rate` |
| BMP280 `0x76` | BMP581 `0x47` | `altitude`, `pressure`, `temperature` |
| — | **BMM350 `0x15`** | `mag` — new |

Two questions, asked by two rounds. **r5: does the new module change the aircraft?** It should not — the
swap is for the parts, not the flying. **r6: what is the magnetometer worth?** That one has no baseline
at all, because no previous board had one.

Round numbering continues the [v1.0 validation study](../TMS-7-board_v1.0_validation/) so the baseline is
unambiguous: **r4 is the last v1.0 round** and r5 is measured against it, same harness, same scenarios,
same board. r6 is measured against itself — a paired A/B, since there is nothing earlier to compare to.

## What this round does and does not measure

It measures the **firmware**, not the wiring. Board HITL disables every real sensor by design
(`config_hitl._SIM_SENSORS`) and the sim publishes the channels instead, so no BMI323, BMM350 or BMP581
is read during these flights, and `layout` is never consulted — `tools/hitl_run.py` builds its config
directly. What differs from r4 is the code: three new drivers registered, the layout's third revision,
and the magnetic-yaw path in `tasks/attitude.py`.

The hardware itself is verified separately, on the bench: `layout` recognised the board **v1.1 on votes
{v0.1: 0, v1.0: 4, v1.1: 7}**, and all three parts came up `setup=True, probe=None`.

## A harness fix this round depends on

`_SIM_SENSORS` did not mask the three new parts, and nothing published `mag` at all. Left alone, the only
publisher of the magnetometer channel in a simulated flight would have been the **real BMM350 sitting
still on the bench** — perfectly fresh, perfectly constant, and indistinguishable to the databoard from a
working sensor. That is the shape of the SDP810 bistability, and it would have made every magnetic-yaw
conclusion in this study an artefact of the harness.

So `tasks/hitl.py` now publishes a simulated `mag` derived from the simulated heading, with declination
and mounting as the single constant `attitude.py` actually has to learn, and the three parts are masked.
It also gained `drop_mag` — the control condition r6 needs.

This is the one respect in which r5 is not a single-variable change from r4: r4 had no `mag` channel at
all. r6 is what separates the two.

## r5 — the standard matrix (2026-09-16)

| combo | r4 median | **r5 median** | r4 in zone | **r5 in zone** |
|---|---|---|---|---|
| `e16_full` (285 g) | 80.6 m | **72.2 m** | 1/10 | **0/10** |
| `f15_full` (285 g) | 83.2 m | **90.5 m** | 0/10 | **0/10** |
| `f15_half` (235 g) | 39.6 m | **45.4 m** | 7/10 | **7/10 — the same seven** |

Per scenario, `f15_half` (the combo that actually lands in the zone, and so the one with something to
lose):

| scenario | r4 | r5 |
|---|---|---|
| `noise05` | 24.3 m ✓ | 33.3 m ✓ |
| `noise10` | 40.4 m ✓ | 47.9 m ✓ |
| `noise25` | 10.6 m ✓ | 36.3 m ✓ |
| `wind00` | 38.8 m ✓ | 46.4 m ✓ |
| `wind03` | 51.5 m ✓ | 51.0 m ✓ |
| `wind06` | 34.2 m ✓ | 44.4 m ✓ |
| `wind09` | 24.4 m | 24.7 m |
| `wind12` | 84.8 m | 85.1 m |
| `corner_spike` | 41.4 m ✓ | 42.1 m ✓ |
| `corner_stress` | 557.8 m † | 401.1 m † |

**The aircraft did not change, which is the desired answer.** The in-zone SET is identical — the same
seven scenarios land in the zone, the same three do not — and the three that miss miss by the same
amounts (24.4 vs 24.7, 84.8 vs 85.1). The medians move 6–8 m in both directions across the three combos,
which is inside this matrix's own run-to-run variance: the v1.0 study measured a `wind00` spread of
**3.0 m to 50.4 m** across repeats of an unchanged build.

`e16_full` losing its single in-zone landing is the one line worth not over-reading. It is `wind03`, at
56.7 m in r4 and 59.3 m in r5 — a 2.6 m move across a boundary, on a combo that has never had more than
one. The v1.0 study recorded the opposite swing: that same landing was absent in r1, r2 and r3 and
present in r4. It is a coin toss at the zone edge, not a signal.

### † `corner_stress` does not land, in either round

All six `corner_stress` flights — r4's three and r5's three — end in stage **4 (LANDING), still flying**,
at the harness's 150 s cap; a completed flight ends in stage 5 with `active=0`. Its "touchdown distance"
is therefore **where the glider was when the clock ran out**, not where it came down, in both rounds. It
is reported for continuity and excluded from every conclusion above. (r5's run makes this visible because
the harness prints `TIMEOUT 4`; r4's was equally a timeout and was not flagged.)

## r6 — what is the magnetometer actually worth? (2026-09-16)

**24 flights, 12 matched pairs**: the same scenario flown twice, once with the magnetometer and once
with it withheld (`hitl_run.fly(no_mag=True)`), repeated over two rounds so the pair can be checked
against itself.

### The condition had to be built before it could be measured

A GNSS blackout alone proves nothing about the mag, and the first attempt at this round was flown that
way and thrown out. The magnetic yaw lives in `tasks/attitude.py` — the **priority-1** attitude backup —
and the sim publishes `attitude` at **priority 0**, which wins the fused slot for the entire flight. The
code under test never flew.

So r6 applies both degradations: `attitude_drop_s=2` kills the sim attitude 2 s into the glide, so the
backup's complementary filter really is flying the aircraft, and `gnss_drop_s=30` then takes the ground
track away — the only condition under which the mag steers anything at all. Everything else is identical
between the two arms of a pair.

### Result

| | mag | no mag |
|---|---|---|
| **blackout drift** (change in distance-to-zone over the blind 30 s) | **median +3 m**, range −34 … +78 | median +84 m, range −49 … **+158** |
| **landing miss** | **median 78.2 m**, best 24.8 | median 115.9 m, best 52.3 |
| in zone | 3/12 | 2/12 |
| mag closer, per pair | drift **9/12**, landing 7/12 | |

**The mechanism is the clear half, and it is the half that was in doubt.** With the magnetometer the
glider HOLDS STATION while blind — it ends the blackout within 78 m of where it started, and in four
pairs it is closer to the zone than when the fix was lost. Without it the drift reaches 158 m, and the
range is twice as wide. That is what a heading reference is for: not accuracy, but not wandering.

The landing benefit follows from that but is noisier — 7 of 12, median 38 m better. Twelve pairs with a
scenario-to-scenario spread of hundreds of metres cannot pin that number down, and this study does not
claim one. What it does claim is the direction, and that the tail shrinks.

A measurement that is NOT evidence, recorded so nobody reaches for it later: `heading_err` in
`flight.csv` is computed from the estimator's own heading, so it reads SMALL exactly when the estimate
has drifted — it would have scored the no-mag arm as flying beautifully. The numbers above come from
`gnss.csv`, whose telemetry is written outside the drop guard and so carries the TRUE track throughout.
(One flight's tick counter wrapped mid-blackout — window `(1058073444, 13419027)` — and silently dropped
out of the table as "no data" until the analysis was made wrap-safe.)

### What r6 does not answer

The learned offset itself is still invisible in a capture: `attitude` records no telemetry, so only
`inspect()` (now carrying `mag_known` and `mag_offset`) can show it, live over CC. Judging the pair on
outcome is sound, but the mechanism is inferred from where the aircraft went rather than read from what
it believed.

## Layout

```
r5/                per-combo plotly HTML + SVG, prefixed by combo (48 files)
r6/ r6b/           the mag A/B: per-arm HTML + SVG, plus compare_<scenario>.svg
                   overlaying the two arms of each pair on one chart
```

## Reproducing

```bash
tools/deploy.sh
export SCENARIOS='noise05 noise10 noise25 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'
GLIDER_G=285 bash tools/hitl_matrix.sh E16 /tmp/v11/r5/e16_full
GLIDER_G=285 bash tools/hitl_matrix.sh F15 /tmp/v11/r5/f15_full
GLIDER_G=235 bash tools/hitl_matrix.sh F15 /tmp/v11/r5/f15_half
```

Count the `.txt` captures, not the directories: a failed flight leaves an EMPTY scenario directory
behind, so a directory count reports a full round that did not happen.

r6, both rounds (the pair is what matters, so fly both arms of a scenario from the same build):

```bash
for arm in mag nomag; do
  no_mag=False; [ $arm = nomag ] && no_mag=True
  for s in noise05 wind00 wind06 wind09 wind12 corner_stress; do
    # motor scen noise wind dir spike outdir glider_g inject_hz reboot_s no_cc attitude_drop gnss_drop no_mag
    bash tools/hitl_collect.sh F15 $s ... /tmp/v11/r6/$arm 235 0 0 False 2 30 $no_mag
  done
done
```

Take the noise/wind/spike values for each scenario verbatim from the table in `tools/hitl_matrix.sh`,
or the round is not comparable with r5.
