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

**48 flights, 24 matched pairs**: the same scenario flown twice, once with the magnetometer and once
with it withheld (`hitl_run.fly(no_mag=True)`), over FOUR rounds — r6/r6b on the v1.1 board, r6c/r6d
after the board was rewired back to v1.0. The hardware makes no difference to this round by
construction (HITL masks every real sensor and the sim publishes `mag` either way, verified live on the
v1.0 board: `mag channel source=hitl value=(683, -729, 700)`), so r6c/r6d are simply two more repeats —
which is exactly what the first two rounds turned out to need.

### The condition had to be built before it could be measured

A GNSS blackout alone proves nothing about the mag, and the first attempt at this round was flown that
way and thrown out. The magnetic yaw lives in `tasks/attitude.py` — the **priority-1** attitude backup —
and the sim publishes `attitude` at **priority 0**, which wins the fused slot for the entire flight. The
code under test never flew.

So r6 applies both degradations: `attitude_drop_s=2` kills the sim attitude 2 s into the glide, so the
backup's complementary filter really is flying the aircraft, and `gnss_drop_s=30` then takes the ground
track away — the only condition under which the mag steers anything at all. Everything else is identical
between the two arms of a pair.

### Result — and the first two rounds did not survive the second two

| over 24 pairs | mag | no mag |
|---|---|---|
| **blackout drift** (change in distance-to-zone over the blind 30 s) | **median +20 m**, range −34 … +230 | median +64 m, range −49 … +203 |
| **landing miss** | **median 112 m**, best 24.8 | median 129 m, best 52.3 |
| in zone | 3/24 | 3/24 |
| mag closer, per pair | drift **14/24**, landing **13/24** | |

**On the first two rounds this looked like a clear win — drift median +3 m against +84 m, the mag closer
in 9 of 12 pairs — and it did not hold up.** Pooled over 24 pairs the mag is closer in 13–14, which is
a coin toss. The medians still favour it, and by a wide margin on drift (+20 m against +64 m), but a
median advantage that comes with a 50 % per-flight win rate is not a reliable benefit: it means the mag
usually helps a little and occasionally hurts a lot, in a scenario whose own spread runs to hundreds of
metres.

Written down plainly because the two-round version of this table was already committed and read like a
result. Twelve pairs were not enough, and the honest reading of twenty-four is **"probably helps, not
yet demonstrated"** — keep the magnetometer (it costs nothing on a module that is fitted anyway), do
not plan around it, and do not treat the first two rounds' numbers as the finding.

One asymmetry is worth recording for whoever picks this up. Across the rewire the **no-mag arm barely
moved** (median landing 116 m on the v1.1 board, 117 m on the v1.0 board) while the **mag arm got
worse** (78 m → 120 m). The mag path was verified live as identical on both boards, so this is most
likely the variance of a high-spread scenario showing its teeth — but it is the kind of pattern that
deserves a third look rather than an explanation, and n=12 per condition cannot tell the two apart.

A measurement that is NOT evidence, recorded so nobody reaches for it later: `heading_err` in
`flight.csv` is computed from the estimator's own heading, so it reads SMALL exactly when the estimate
has drifted — it would have scored the no-mag arm as flying beautifully. The numbers above come from
`gnss.csv`, whose telemetry is written outside the drop guard and so carries the TRUE track throughout.
(One flight's tick counter wrapped mid-blackout — window `(1058073444, 13419027)` — and silently dropped
out of the table as "no data" until the analysis was made wrap-safe.)

### What r6 does not answer, and what it cost to find out

**No servo-energy numbers exist for any round in this study.** `config_hitl` does not resolve the board
layout, so `power_ina226` — the one real sensor HITL does NOT mask — is configured on its v0.1 bus while
a v1.0/v1.1 board has it on the other one. It answers ENODEV, no `power_ina226.csv` is written, and
`flight_kpi` simply omits the servo-energy line rather than reporting its absence. This has been true of
every HITL round since the v1.0 rewire, r4 included.

Fixed after r6d: `tools/hitl_run.py` now calls `layout.resolve()` as the real boot path does, and a
verification flight records the stream again — **23.4 J over 100.5 s, 0.23 W average**. The rounds in
this study were flown before that fix and have no energy data; rounds after it will, and they carry one
extra real device on the bus, so treat energy as a new series rather than a continuation.

The same flaw was found in four diagnostics (`diag_channels.py`, `diag_icp.py`,
`diag_icp_concurrent.py`, `live_pitot.py`): each built a config without resolving the layout, so on a
v1.0 board they hunted the ICP-10111, pitot, laser and INA226 on their v0.1 buses and reported healthy
parts as ENODEV. `diag_devices.py` was always correct — it runs `main.bringup()`. If a diagnostic and
the boot path disagree about whether a device is alive, suspect the diagnostic's config first.

The learned offset itself is still invisible in a capture: `attitude` records no telemetry, so only
`inspect()` (now carrying `mag_known` and `mag_offset`) can show it, live over CC. Judging the pair on
outcome is sound, but the mechanism is inferred from where the aircraft went rather than read from what
it believed.

## v1.0 vs v1.1 overall — control, fins, health, battery

r4 and r5 are the same harness over the same 30 scenarios, so the four things an operator actually
weighs can be put side by side. Medians per group; `free_min` is the worst single sample in the group
and `rescues` the total emergency collects.

| | | landing miss | in zone | fin moves | fin travel | deg/s | free_min B | leak KB/s | rescues |
|---|---|---|---|---|---|---|---|---|---|
| **all 30** | v1.0 | 70.2 m | 8 | 3256 | 11612° | 121 | 40 832 | 367 | 122 |
| | **v1.1** | **67.7 m** | **7** | **3232** | **11584°** | **118** | **46 336** | **377** | **129** |
| `e16_full` | v1.0 | 80.6 | 1 | 1509 | 6197 | 129 | 40 832 | 360 | 1 |
| | v1.1 | 72.2 | 0 | 1415 | 5438 | 114 | 46 336 | 353 | 2 |
| `f15_full` | v1.0 | 83.2 | 0 | 3190 | 11612 | 117 | 202 464 | 370 | 50 |
| | v1.1 | 90.5 | 0 | 3220 | 11868 | 121 | 116 528 | 377 | 50 |
| `f15_half` | v1.0 | 39.6 | 7 | 3835 | 13726 | 121 | 44 736 | 365 | 71 |
| | v1.1 | 45.4 | 7 | 4090 | 14785 | 130 | 49 904 | 394 | 77 |

**Control and landing — indistinguishable.** Aggregate median 70.2 m against 67.7 m, in zone 8 against
7, and the per-combo differences (−8 m, +7 m, +6 m) point in BOTH directions inside a matrix whose own
repeat spread was measured at 3.0–50.4 m.

**Fins — indistinguishable.** 3256 against 3232 moves and 11612 against 11584 degrees of travel, under
1 % apart over 30 flights. Per combo it swings ±10 % both ways (v1.1 does 12 % less work on `e16_full`,
8 % more on `f15_half`), which is noise at this sample size. There is no servo-wear argument either way.

**In-flight health — marginally worse on v1.1, trivially so.** Leak 377 against 367 KB/s (+3 %),
rescues 129 against 122 (+6 %), and the global memory floor is actually BETTER (46 336 B against
40 832 B). Nothing approaches a threshold.

**Battery — no data, and a flight round cannot supply it.** No round since the v1.0 rewire recorded a
power stream (see above). Even with that fixed, HITL would not answer the question: on USB the INA226
sees the **servo rail only**, and servo work is a function of fin activity, which the table shows is
identical. What differs between the revisions is *sensor quiescent draw*, and that is a bench
measurement on battery, not a simulation.

### Conclusion

**In simulation the two revisions are the same aircraft** — which is both the result a module swap
should give and the result HITL is STRUCTURALLY OBLIGED to give, since it masks every real sensor and
publishes the channels itself. Neither round ever read a BNO055 or a BMI323. Any claim that one
revision *flies* better than the other cannot come from this study, and none is made here.

What actually separates them is redundancy, and it is visible in
[`datasources.md`](../../datasources.md) rather than in any flight:

| channel | v1.0 | v1.1 |
|---|---|---|
| `attitude` | `imu_bno055` p0 **+** the board's filter p1 | **the board's filter alone — no p0** |
| `rate` | 1 source | 2 |
| `mag` | 0 | 1 |

**v1.1 trades its second attitude source for a magnetometer and a second gyro.** That is the decision,
stated plainly: not better or worse flying, but a different failure surface. Two sensor sets with no
demonstrated performance gap is a good position to fly trials from — the fleet learns more from two
than from two of the same — provided the asymmetry is carried knowingly and not filed under "no
regressions".

**The gap this study leaves open.** Every round here flew with the sim publishing `attitude` at
priority 0, so on BOTH revisions the board's complementary filter never actually carried a flight. The
only rounds where it did (r6) also blacked out the GNSS, so filter-only attitude and dead-reckoning are
confounded. On a real v1.1 board, filter-only attitude is not a degraded mode — it is the NORMAL one.
The round that closes this is a paired A/B with `attitude_drop_s` set and the GNSS left healthy; it has
not been flown.

## Layout

```
r5/                per-combo plotly HTML + SVG, prefixed by combo (48 files)
r6/ r6b/           the mag A/B on the v1.1 board: per-arm HTML + SVG, plus
                   compare_<scenario>.svg overlaying the two arms of each pair
r6c/ r6d/          the same A/B after the rewire to v1.0 -- pair overlays only.
                   Same firmware and the same simulated mag, so these are repeats,
                   and they are what stopped r6/r6b being read as a result
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
