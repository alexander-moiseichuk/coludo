# TMS-7 — board v1.0, polishing: the launch-hardening firmware, re-flown

**76 board flights** on the v1.0 taster after the 2026-09-22 launch-hardening pass (33 commits: the
airborne ground-gate on CC, the separation hold, working baro re-zero, probes that need live data, the
attitude backup that no longer mirrors a stale primary, the warm-start identity, `attitude.csv`, and a
long tail of smaller fixes). Three rounds, continuing the numbering of the
[v1.1 SEN0697 study](../TMS-7-board_v1.1_sen0697/) (r5, r6) and the
[v1.0 validation](../TMS-7-board_v1.0_validation/) (r1–r4):

| round | question | flights |
|---|---|---|
| **r7** | did the hardening change the aircraft? (the standard matrix, against r4 and r5) | 30 |
| **r8** | the gap the SEN0697 study left open: the board's attitude filter flying ALONE, GNSS healthy | 10 |
| **r9** | a mid-glide reboot through the REAL warm-start restore | 3 (+3 that exposed the rig) |
| **r7b** | r7 again with the checkpoint task RUNNING — what every earlier HITL round left out | 30 |

Like every HITL round, this measures the **firmware**, not the parts: `config_hitl` masks every real
sensor the sim publishes over (now re-applied after the layout resolves — see r7), so no BNO055 or
BMI323 is read in flight.

## r7 — the standard matrix (2026-09-22)

Same harness, same 10 scenarios, same three combos as r4 (v1.0) and r5 (v1.1). All three rounds were
re-scored with the SAME `flight_kpi.py` (whose touchdown is now the last fix at or before DONE); r4 and r5
reproduce their published numbers exactly, so the columns compare like for like.

| | round | landing miss (median) | in zone | fin moves | fin travel | deg/s | free_min B | leak KB/s | rescues | servo energy |
|---|---|---|---|---|---|---|---|---|---|---|
| **all 30** | r4 v1.0 | 70.2 m | 8 | 3256 | 11612° | 121 | 40 832 | 367 | 122 | — |
| | r5 v1.1 | 67.7 m | 7 | 3232 | 11584° | 118 | 46 336 | 377 | 129 | — |
| | **r7 v1.0** | **71.8 m** | **7** | **3236** | **11384°** | **118** | **38 464** | **390** | **133** | **57.6 J** |
| `e16_full` | r4 / r5 | 80.6 / 72.2 | 1 / 0 | 1509 / 1415 | 6197 / 5438 | 129 / 114 | 40 832 / 46 336 | 361 / 353 | 1 / 2 | — |
| | **r7** | **77.9** | **0** | **1511** | **6122** | **128** | 83 632 | 391 | 3 | **26.4 J** |
| `f15_full` | r4 / r5 | 83.2 / 90.5 | 0 / 0 | 3190 / 3220 | 11612 / 11868 | 117 / 121 | 202 464 / 116 528 | 370 / 377 | 50 / 50 | — |
| | **r7** | **86.3** | **0** | **3154** | **11384** | **116** | 73 008 | 390 | 52 | **57.6 J** |
| `f15_half` | r4 / r5 | 39.6 / 45.4 | 7 / 7 | 3835 / 4090 | 13726 / 14785 | 121 / 130 | 44 736 / 49 904 | 365 / 395 | 71 / 77 | — |
| | **r7** | **42.4** | **7 — the same seven** | **3802** | **14089** | **124** | 38 464 | 387 | 78 | **70.8 J** |

`f15_half` per scenario (✓ = in zone):

| scenario | r4 | r5 | **r7** |
|---|---|---|---|
| `noise05` | 24.3 ✓ | 33.3 ✓ | **24.3 ✓** |
| `noise10` | 40.4 ✓ | 47.9 ✓ | **45.5 ✓** |
| `noise25` ‡ | 10.6 ✓ | 36.3 ✓ | **7.3 ✓** |
| `wind00` | 38.8 ✓ | 46.4 ✓ | **50.1 ✓** |
| `wind03` | 51.5 ✓ | 51.0 ✓ | **44.4 ✓** |
| `wind06` | 34.2 ✓ | 44.4 ✓ | **37.4 ✓** |
| `wind09` | 24.4 | 24.7 | **25.4** |
| `wind12` | 84.8 | 85.1 | **83.6** |
| `corner_spike` | 41.4 ✓ | 42.1 ✓ | **40.4 ✓** |
| `corner_stress` † | 557.8 | 401.1 | **486.9** |

**The aircraft did not change — the desired answer, for the third round running.** The in-zone set is
IDENTICAL across r4, r5 and r7: the same seven `f15_half` scenarios land in the zone and the same three
miss, `wind09` and `wind12` by the same metres (24.4 / 24.7 / 25.4, 84.8 / 85.1 / 83.6). Every median
moves inside the matrix's own repeat spread (the v1.0 study measured `wind00` at 3.0–50.4 m across
repeats of one build). Fin work is within 1 % over 30 flights. None of the hardening touched the control
law, and none of it moved the flight.

**Health — +3 % leak, and it is accounted for.** 390 KB/s against 377, rescues 133 against 129, the
memory floor a little lower. r7 is the first round that carries two new streams — `attitude.csv` (the
fused backup attitude, 10 Hz) and `power_ina226.csv` — and a REAL INA226 read on its correct bus (the
layout-resolve fix in `hitl_run`). That is more work per second than r5 did, by design. (The memory
floor is not a general squeeze — the median per-flight minimum is ~7.2 MB; see r7b for the two flights
that set it.)

**Servo energy — the first numbers since the v1.0 rewire.** Median **57.6 J** per flight: 26.4 J on the
short E16 flights, 57.6 J and 70.8 J on the F15 ones (the half-mass glider flies longer and turns
more). No earlier round in this series has this column — the INA226 sat on the wrong bus in every HITL
flight since the rewire — so it starts a new series rather than continuing one. On USB the INA226 sees
the servo rail only.

### ‡ `noise25` and † `corner_stress` do not reach DONE — for different reasons

Both end in stage 4 at the 150 s cap in every round (`TIMEOUT 4` in the harness log), but they are not
the same failure:

- **`noise25` LANDED.** The laser saw 4.4 m and the glider came down; at 25 % sensor noise the
  "stationary" detector never settles, so LANDING → DONE does not fire before the cap. Its miss is a real
  touchdown and is counted.
- **`corner_stress` is still FLYING** at the cap, so its "miss" is wherever the clock ran out. Reported
  for continuity, excluded from every conclusion above (as in r4 and r5).

### The attitude backup, seen for the first time

`attitude.csv` is new in r7, so for the first time a capture shows what the board's filter believed.
Over 30 flights the sim's priority-0 attitude never went stale mid-flight — the backup **never had to
free-run in flight**. Its only free-running rows (0.4 % of 24 352) are the last ~0.5 s after DONE, when
the sim stops publishing: exactly where the corrected code should free-run on its gyros. The code it
replaced mirrored the primary's EXTRAPOLATED last value there instead, and would have done the same in
any >40 ms gap in flight.

## r8 — the attitude filter flying alone, GNSS healthy (2026-09-22)

The SEN0697 study named this as its open gap: on a v1.1 board the complementary filter in
`tasks/attitude.py` is the ONLY attitude source, yet no round had flown it with the GNSS left healthy.
r8 is the `f15_half` set with the sim's priority-0 attitude withheld 2 s into the glide
(`attitude_drop_s=2`), everything else as in r7. The sim still RECORDS its true attitude
(`imu_bno055.csv`), and r7 made the filter's output visible (`attitude.csv`), so for the first time the
filter can be scored against truth rather than inferred from where the glider went.

| scenario | r7 (sim attitude) | **r8 (filter alone)** | roll err RMS | pitch err RMS | heading err median / max |
|---|---|---|---|---|---|
| `noise05` | 24.3 ✓ | 98.4 | 7.9° | 6.5° | 0.6° / 3.7° |
| `noise10` | 45.5 ✓ | 112.6 | 9.2° | 6.3° | 1.1° / 6.1° |
| `noise25` | 7.3 ✓ | 59.9 | 5.5° | 7.2° | 2.4° / 6.0° |
| `wind00` | 50.1 ✓ | 121.2 | 7.3° | 6.5° | 1.0° / 3.7° |
| `wind03` | 44.4 ✓ | 193.6 | 1.8° | 6.7° | 9.3° / 14.6° |
| `wind06` | 37.4 ✓ | 73.1 | 4.3° | 7.8° | 20.6° / 136° |
| `wind09` | 25.4 | 107.0 | 3.5° | 8.8° | 25.4° / 132° |
| `wind12` | 83.6 | 70.9 | 1.4° | 9.8° | 4.3° / 179° |
| `corner_spike` | 40.4 ✓ | 122.5 | 6.1° | 6.5° | 1.0° / 3.0° |
| `corner_stress` † | 486.9 | 862.1 | 9.1° | 6.2° | 61° / 180° |

**0/10 in the zone against 7/10, median miss ~107 m against 42 m — and this round CANNOT be read as the
filter's verdict, because the harness feeds it an accelerometer no real board has.** The sim publishes
`accel` as a MAGNITUDE on one axis (`accel[axis] = |a|`, the load factor 1/cos(bank) in the glide), with
no gravity direction at all. A real BMI323 in a straight glide reads the glide pitch in its along-body
component; this one reads exactly level. So whenever the filter's gravity correction is open, it pulls
pitch toward 0°:

- **Pitch: 6–10° RMS error in EVERY flight, and always the same sign** — mean error +4° to +10°: the
  filter's median pitch sits at −0.3° to −2.9° while the true glide pitch is −7.4° to −10°. The pitch loop then flies the nose ~8° lower than it believes, and the glider DIVES:
  r8 glides last ~44 s against ~98 s in r7 for the same scenario. This alone explains most of the
  landing numbers, and it is an artifact.
- **Roll: the bank is under-read in turns** (true 24–27°, filter 10–15° in `noise05`). This part is
  plausibly REAL: in a coordinated turn the specific force has no lateral component, so an accelerometer
  cannot see the bank on any board; the filter relies on its turn gate (yaw rate > 4°/s closes the
  gravity correction) and the gyro, and at turn entry, before the yaw rate builds, the correction pulls
  the bank back toward level. Worth a hardware check on 7F.
- **Heading: excellent in calm air** (median ≤ 2.4°), and in wind it converges on the GNSS **track**, not
  the nose — off by the crab angle (20–25° median at 6–9 m/s). That is the design of the course pull
  (`course_shift` pulls yaw toward the ground track, and the mag offset is learned against the track),
  and for steering to a zone the track is arguably the quantity that matters. The 130–180° maxima are
  single wrap samples in strong wind and in `corner_stress`.

**What it takes to answer the question.** The sim must publish a body-frame specific force consistent
with its own attitude — in coordinated flight roughly `(−sin θ, 0, n·cos θ)` in the filter's axis
convention, so pitch is observable and bank (correctly) is not. That same channel also feeds the
airspeed estimator's accel backbone and the governor, so it is a harness change that moves EVERY
flight, not only this one: it needs its own study against r7, not a quiet edit. Until then:

- the r6 result in the [SEN0697 study](../TMS-7-board_v1.1_sen0697/) (the magnetometer A/B) flew the
  filter on this same channel, so part of its very wide spread is likely this artifact, not the mag;
- the filter-only behaviour of a v1.1 board — 7F's NORMAL mode — is still unmeasured in simulation. The
  honest evidence will be 7F's own `attitude.csv` from 10-03, where `flight` is off and the filter only
  records.

## r9 — a mid-glide reboot through the real restore (2026-09-22)

Three `f15_half` scenarios with a 3 s outage early in the glide, restored by the REAL
`warmstart._apply_restore` (the rig used to hand-roll the stage/arm restore, and had crashed with a
TypeError since the gate's signature changed on 2026-07-14).

**The first attempt found the rig had never been able to work.** All three flights "passed" and ended in
DONE — and all three were COLD boots: `WARM GATE: False no crumb`. `hitl_run.py` imported `warmstart`
only inside the reboot rig, after the controller had set up, so the `checkpoint` activity was never
registered and **no HITL flight in any round has ever checkpointed** (no `checkpoint.csv` in any
capture, r4–r8 included). With no crumb the rig flew on as a cold boot: the sequencer, back in SETTING
at 253 m, fired the baro launch backup and re-launched in mid-air. And `hitl_collect.sh` printed OK,
because it only checked the post-restore crumb. Both are fixed: the import now happens at the top, as
`main.py` does, and a refused gate fails a reboot scenario.

Re-flown:

| scenario | gate | restore | post-restore crumb | checkpoints (by stage) | miss |
|---|---|---|---|---|---|
| `noise05` | **PASS** — recover gliding, 3 s after checkpoint | GLIDING, armed | **pad_altitude kept** | boost 10 · glide 2 · *outage* 1 · glide 16 · land 3 · done 1 | 122.5 m |
| `wind06` | **PASS** | GLIDING, armed | kept | 10 · 3 · *1* · 14 · 3 · 1 | 96.3 m |
| `corner_spike` | **PASS** | GLIDING, armed | kept | 10 · 3 · *1* · 16 · 3 · 1 | 116.4 m |

**The mechanism works end to end**: crumb written at 1 Hz while armed, gate passes, the flight comes
back GLIDING and armed with no re-launch, and — the fix this round exists to prove — the first
checkpoint after the restore keeps the recovery identity (before the fix it went out without
launch/zone/pad/pitot, so a SECOND reset would have skipped the baro rebase).

**The misses are not a verdict on the restore.** They carry the sim caveat the
[warm_reboot study](../TMS-7-warm_reboot/) recorded: during the fake outage the sim flies BALLISTIC (no
lift in SETTING, ~17 m/s sink), so the glider enters the restore in a dive and glides ~22 s after it
against ~98 s undisturbed — the same shape that study measured (132 m and 215 m on its reboot cases).

## r7b — the same matrix with the checkpoint task running (2026-09-22)

r9 found that no HITL flight had ever run the `checkpoint` task (see there), so every round in this
series — r1 to r8 — flew without the 1 Hz recovery crumb a real armed flight commits to NVS. r7b re-flies
r7 exactly, with it running: the first HITL numbers for the configuration that will actually fly.

| | round | landing miss | in zone | fin moves | fin travel | free_min B | leak KB/s | rescues | servo energy |
|---|---|---|---|---|---|---|---|---|---|
| **all 30** | r7 | 71.8 m | 7 | 3236 | 11384° | 38 464 | 390 | 133 | 57.6 J |
| | **r7b** | **67.2 m** | **8** | **3520** | **12007°** | **21 728** | **408** | **145** | **60.8 J** |
| `e16_full` | r7 → r7b | 77.9 → 74.2 | 0 → 0 | 1511 → 1501 | 6122 → 5724 | 83 632 → 21 728 | 391 → 386 | 3 → 3 | 26.4 → 29.1 |
| `f15_full` | r7 → r7b | 86.3 → 87.4 | 0 → 1 | 3154 → 3428 | 11384 → 12007 | 73 008 → 272 128 | 390 → 412 | 52 → 58 | 57.6 → 60.8 |
| `f15_half` | r7 → r7b | 42.4 → 43.1 | 7 → 7 (the same seven) | 3802 → 4098 | 14089 → 14790 | 38 464 → 34 256 | 387 → 409 | 78 → 84 | 70.8 → 72.3 |

**Flight: unchanged** — the same seven `f15_half` scenarios in the zone for the fourth round, medians
inside the repeat spread, fin work within the ±10 % per-combo swing earlier rounds already showed.
**Memory: the crumb costs ~5 %** of the GC-off allocation rate (390 → 408 KB/s) and ~9 % more rescues.
That is the honest price of the recovery path, now measured rather than assumed zero.

### A post-landing memory hazard this made visible

The two lowest memory floors, in BOTH rounds, are the flights that never reach DONE: `noise25` (landed,
but 25 % accel noise never lets the 3 s stationary detector settle) and `corner_stress`. Once in
LANDING nothing frees memory: GC is re-enabled only at DONE, and the memory rescue deliberately skips
LANDING (a collect during the flare is the thing it exists to avoid). In `e16_full/noise25` the heap ran
from ~18 MB at touchdown to 0.54 MB about 54 s later (~330 KB/s), and the harness's 150 s cap is all
that stopped it. On a board the flight-timeout backstop (300 s after BOOSTING) would force DONE — well AFTER that
horizon — and an OOM in LANDING resets through the watchdog into a warm start that restores LANDING
with GC off again.

In practice a landed glider reaches DONE after 3 s of stillness, so the exposure is narrow: an
airframe that keeps moving after touchdown (carried away before it settles, rocking in wind). Recorded
as an open item with two candidate fixes, neither made here because both touch the GC policy: a
LANDING timeout (the flare is over seconds after `land_agl_m`), or letting the rescue run in LANDING
once the laser has read ground for a few seconds.

## Layout

```
r7/    per-combo plotly HTML + SVG, prefixed by combo (48 files) -- same layout as the SEN0697 study's r5
r7b/   the same matrix with the checkpoint running, SVG only (36 files)
r8/    per-flight SVG for the filter-only arm
r9/    per-flight SVG for the three reboot flights
```

## Reproducing

```bash
tools/deploy.sh
export SCENARIOS='noise05 noise10 noise25 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'
GLIDER_G=285 bash tools/hitl_matrix.sh E16 /tmp/v10p/r7/e16_full
GLIDER_G=285 bash tools/hitl_matrix.sh F15 /tmp/v10p/r7/f15_full
GLIDER_G=235 bash tools/hitl_matrix.sh F15 /tmp/v10p/r7/f15_half
```

r8 is the `f15_half` set with the sim attitude dropped 2 s into the glide and the GNSS left alone; r9
reboots 3 s into the glide (take noise/wind/spike per scenario from the table in `tools/hitl_matrix.sh`):

```bash
# motor scen noise wind dir spike outdir glider_g inject_hz reboot_s no_cc attitude_drop_s gnss_drop_s no_mag
bash tools/hitl_collect.sh F15 wind06 0.10 6.0 210.0 False /tmp/v10p/r8 235 0 0 False 2 0 False   # r8
bash tools/hitl_collect.sh F15 wind06 0.10 6.0 210.0 False /tmp/v10p/r9 235 0 3 False 0 0 False   # r9
```

Count the `.txt` captures, not the directories: a failed flight leaves an empty scenario directory.
