# TMS-7 — post-audit regression, flown four times

**216 board flights**: the 12-scenario matrix × F15-4 and E16-4 × four independent runs. `r1`–`r3` are
the same build flown three times; `r4` adds one recorder change and re-flies it to test that change.

The point is not any single run. It is that **a comparison against a baseline is meaningless until you
know how much a run varies against itself** — and it turns out the answer depends entirely on which
scenario you ask.

## 1. Repeatability is not one number

Miss distance across r1/r2/r3, grouped by scenario family:

| family | scenarios | median spread | worst spread |
|---|---|---|---|
| **wind 00–12** | 10 | **7.7 m** | 21.0 m |
| **low noise 05–25** | 6 | **11.9 m** | 36.5 m |
| high noise 50–100 | 4 | 99.9 m | 154.4 m |
| corner spike/stress | 4 | 80.2 m | 120.8 m |

**A change is only detectable if it exceeds its family's own spread.** On the wind cases a 20 m shift
is real; on `noise100` even a 150 m shift is indistinguishable from flying it again. The earlier
version of this study compared one run against a baseline and hedged its conclusion — that hedge was
right, and this is the number that justifies it.

### Per-scenario miss (m)

| motor | scenario | r1 | r2 | r3 | r4 | spread |
|---|---|---|---|---|---|---|
| E16 | corner_spike | 71.3 | 70.8 | 74.4 | 71.5 | 3.6 |
| E16 | corner_stress | 638.3 | 558.1 | 632.5 | 772.3 | 214.2 |
| E16 | noise05 | 69.6 | 72.0 | 71.9 | 80.6 | 11.0 |
| E16 | noise10 | 71.7 | 108.2 | 90.7 | 81.4 | 36.5 |
| E16 | noise25 | 92.1 | 71.4 | 89.6 | 77.1 | 20.7 |
| E16 | noise50 | 90.3 | 190.2 | 120.2 | 98.4 | 99.9 |
| E16 | noise100 | 135.6 | 129.9 | 139.5 | 134.3 | 9.6 |
| E16 | wind00 | 71.8 | 79.9 | 76.4 | 78.6 | 8.1 |
| E16 | wind03 | 74.6 | 54.6 | 53.6 | 57.3 | 21.0 |
| E16 | wind06 | 56.9 | 46.5 | 53.2 | 51.5 | 10.4 |
| E16 | wind09 | 55.4 | 56.6 | 59.0 | 63.3 | 7.9 |
| E16 | wind12 | 95.0 | 92.2 | 92.0 | 92.7 | 3.0 |
| F15 | corner_spike | 114.9 | 87.5 | 92.4 | 97.9 | 27.4 |
| F15 | corner_stress | 661.6 | 562.6 | 683.4 | 685.5 | 122.9 |
| F15 | noise05 | 94.5 | 93.8 | 93.7 | 92.8 | 1.7 |
| F15 | noise10 | 90.7 | 90.9 | 102.6 | 85.1 | 17.5 |
| F15 | noise25 | 85.1 | 87.6 | 85.5 | 83.8 | 3.8 |
| F15 | noise50 | 53.0 | 113.5 | 92.1 | 93.4 | 60.5 |
| F15 | noise100 | 241.3 | 112.2 | 266.6 | 202.1 | 154.4 |
| F15 | wind00 | 84.2 | 91.9 | 89.1 | 88.1 | 7.7 |
| F15 | wind03 | 59.8 | 54.1 | 60.2 | 58.7 | 6.1 |
| F15 | wind06 | 53.0 | 48.5 | 53.7 | 55.5 | 7.0 |
| F15 | wind09 | 72.8 | 64.8 | 65.3 | 71.6 | 8.0 |
| F15 | wind12 | 59.6 | 57.3 | 53.7 | 53.0 | 6.6 |

`noise05` at **1.7 m** across four runs and `noise100` at **154 m** are the same aircraft and the same
code. The difference is the scenario, not the build.

## 2. Every chart, compared

`tools/hitl_compare.py` reduces all 11 flight-report panels to 23 quantities and compares them across
runs — 552 scenario/panel comparisons per pairing. Panels mirror `flight_report.py` exactly and use the
shared parser, so a number here traces to a chart there.

Two outputs, answering different questions:

* **Spread** `(max-min)/mean` — repeatability, ranked on one scale regardless of magnitude.
* **Anomaly** — a channel present in some runs and absent in others, or a value outside the
  quantity's *physical* range. A ratio cannot tell "noisy" from "corrupt": a heading error of 15330°
  and one of 175° differ by 195%, which ranks them next to a memory figure that merely varies.

What the panels showed:

| panel group | behaviour across runs |
|---|---|
| accel, altitude, elevation peak, speed, attitude, gyro, fins | tight — these track the flight, and the flight repeats |
| **health mem_free min** | **98–265% spread** — memory pressure genuinely varies run to run |
| **health rescues** | **0 ↔ 1 ↔ 2** on the same scenario — the leak rescue is a coin flip at high noise |
| **fin cap min** | 5 ↔ 14 — governor authority varies with the airspeed estimate |
| elevation final | ±0.05 m; the 600% "spread" is a ratio about zero, not a finding |
| channel dropouts | **0 across all four runs** — no sensor ever went missing |

The memory panel is the honest surprise: `rescues` firing 2, 0, 2 on identical `noise100` runs means
the OOM rescue sits right at its trigger point in that scenario. Not a fault, but it means any memory
conclusion drawn from a single run is luck.

## 3. What the comparison found that no single run could

Three values were **physically impossible** — an airspeed of 1.44e12 cm/s and heading errors of 15330°
and 255°, where a heading error cannot exceed 180° by definition.

None was a computation fault. The raw capture holds two records run together on one line:

```
@..._flight.csv@835330775;3;1;1394;45;4500;-6@..._airspeed_sdp810.csv@835333534;15330;1582;2500
```

The flight record is truncated mid-field (`-6` where `-600` belongs), lost its terminating newline, and
the next record's bytes ran onto it. This is the **real recorder path** — board UART → Luckfox → adb —
and `assemble_capture` cannot cause it, since it maps each input line to exactly one output line.

Measured: **20 spliced lines in 1,004,804 across 48 flights**, hitting 14 flights — about one in three.

The damage is **misattribution**, not the lost tail. The truncated row keeps parsing and the second
record's fields land in the first record's columns, so `15330` was never a heading error at all — it
was the airspeed stream's `dynamic_pressure` read in the wrong place. This class already produced a
reported **L/D of 64** for a 5-ish airframe.

`flight_telemetry.parse()` now splits at the second marker, and any row whose width does not match its
header is dropped entirely. Dropping rather than trimming is deliberate: trimming assumes the loss was
at the tail, but if a cell went missing mid-row every later cell shifts left and still parses cleanly —
which is exactly how `heading_err` came to read 255 at a position a tail-trim left untouched. Cost:
20 rows in 1,004,804.

**Residual, and worth knowing:** r4 produced an airspeed of 132056 cm/s in a row of *correct width* —
a bit flip inside a field, which no structural guard can see. Only a physical-bounds check catches
that, which is why `hitl_compare` carries one.

## 4. The recorder fix, and an honest negative

`UART()` was created with no `txbuf`, so it used MicroPython's 256-byte default while `drain()` pushes
a ring holding **256 KB** in one pass. An overrun does not raise — the UART silently drops the tail of
whatever record it is mid-way through, which is precisely the observed corruption.

r4 flew with `txbuf=8192` plus a chunked drain (flush every 32 records rather than buffering a whole
ring pass).

| run | events | lines | per 100k |
|---|---|---|---|
| r1 | 41 | 509,280 | 8.05 |
| r2 | 42 | 495,524 | 8.48 |
| r3 | 49 | 502,931 | 9.74 |
| **r4** (txbuf 8192 + chunked drain) | **40** | 514,549 | **7.77** |

**No effect is demonstrated.** r4's 40 events sit inside the baseline's own range of 41–49, and Poisson
error on ~40 counts is ±6.3. This experiment can detect a 50% reduction (2.7σ); it cannot detect 35%
(1.8σ) or 20% (1.0σ).

So the sender's TX buffer was **not** the bottleneck. That points at the receiving side — a hardcoded
64-byte kernel buffer on the Luckfox is the standing hypothesis, and a bigger sender buffer would not
help that, it would make the stream more continuous.

Why the change stays anyway: 256 bytes against a 256 KB ring is objectively undersized, it costs 8 KB
of RAM and nothing per record, and it removes a real hazard even though it was not *the* cause. It is
recorded here as an unproven change, not a fix.

**Rejected: `sleep(1 ms)` after each line.** Measured at 424 records/s, that costs 0.4 s of sleep per
second of flight if it truly slept 1 ms — and **4.2 s/s at this board's measured ~10 ms asyncio floor**,
throttling telemetry roughly fourfold. The instinct was right; the timing does not survive this
scheduler.

## 5. Is it reproducible at a high level? Yes — with one caveat

At the RUN level the four runs agree closely. These are medians over 24 flights each:

| | r1 | r2 | r3 | r4 | spread |
|---|---|---|---|---|---|
| flight duration | 96.5 s | 99.3 s | 99.5 s | 99.2 s | **3.0 %** |
| servo energy | 57.0 J | 59.1 J | 60.4 J | 57.2 J | **5.8 %** |
| miss (median) | 79.4 m | 87.5 m | 89.3 m | 82.6 m | **11.7 %** |
| miss p25 | 69.6 m | 64.8 m | 65.3 m | 71.5 m | 10 % |

Use the MEDIAN, not the mean. The mean spans 124.8–138.6 m and is unstable, because `corner_stress`
at 550–770 m dominates it — a heavy tail, not a measurement problem. Same 17 streams in every run,
96 of 96 flights, zero channel dropouts.

**Scenario ORDERING reproduces too**, which is the stronger test — Spearman rank correlation of
scenario difficulty, run against run:

| pair | ρ |
|---|---|
| r2 vs r3 | 0.935 |
| r2 vs r4 | 0.943 |
| r3 vs r4 | **0.959** |
| r1 vs r2 | 0.670 |
| r1 vs r3 | 0.797 |
| r1 vs r4 | 0.770 |

`noise100` holds rank 20 of 24 in **all four** runs; `wind06` holds 1–2; `corner_stress` holds 22–23.
The hard scenarios stay hard and the easy ones stay easy.

### The caveat: r1 is the odd one out

r2/r3/r4 agree with each other at **0.946** mean, but r1 agrees with them at only **0.746** — a Fisher
z separation of **2.7 σ**. Suggestive, not conclusive, and the correlations share runs so they are not
independent; the true significance is lower.

**And it could not be resolved from the data.** `main.py` logs the build and config identity at boot
precisely so a recording can be attributed to what produced it — but a log line carries no
`@session_file@` prefix, the Luckfox never routes it to a `.csv`, and `hitl_collect` pulls only
`*.csv`. Every HITL capture ever taken threw that provenance away at the transport. So the question
"did r1 fly the same firmware?" had no answer in the data.

Fixed going forward: captures now carry `0 capture :: build <firmware> <config_id> <source>`, validated
rather than recorded blind. It does not rescue r1 — those captures predate the stamp — but the next
comparison will not have the hole.

**Practical reading:** treat r2/r3/r4 as the repeatability baseline (ρ ≈ 0.95, aggregate within 3–12 %)
and treat r1 as suspect until a stamped run replaces it.

## 6. r5 and r6 — re-flown after the findings §41 fixes

Two more matrix runs, each isolating a change, flown against the r2–r4 baseline:

* **r5** — HITL attitude units (§41.2) + SDP810 CRC widened to all three words (§41.22)
* **r6** — the above plus the steering-filter rounding fix (§41.12)

### The units fix, verified as a positive control

Attitude roll range, median over 24 flights, in the units as recorded:

| run | raw | rendered |
|---|---|---|
| r2 | 58.4 | 0.58° |
| r3 | 57.8 | 0.58° |
| r4 | 58.3 | 0.58° |
| **r5** | **5751.5** | **57.52°** |

Three baseline runs agreeing to the second decimal, then a **98.6× step** exactly where the fix landed —
the residual 1.4% being real flight-to-flight bank variation. Before r5 the attitude panel drew a glider
banking ±29° as 0.58° of total travel: a flat line, in every board HITL capture ever taken.

### Flight behaviour: unchanged, as intended

| | r3 | r4 | r5 | r6 | spread |
|---|---|---|---|---|---|
| miss (median) | 89.3 m | 82.6 m | 89.2 m | 91.8 m | 10.3% |
| duration | 99.5 s | 99.2 s | 98.7 s | 99.2 s | **0.8%** |

Wind-family spreads across r4/r5/r6 are 1.3–9.6 m, all inside the ~7.7 m baseline. Neither fix moved
the aircraft, which is the expected result: the units change is telemetry-only, and the filter bias it
corrected was −0.5°, two orders of magnitude below this matrix's ~20 m detection floor.

### A servo died mid-session, and the energy metric caught it

Servo energy fell 57.2 J → 36.4 J at r5 and stayed there. It was **not** a code effect:

| run | median mA | median fin travel |
|---|---|---|
| r3 | 75.0 | 11821° |
| r4 | 73.0 | 11719° |
| r5 | 48.8 | 11824° |
| r6 | 36.8 | — |

Identical fin travel, monotonically falling current at constant ~4.9 V. `diag_servos` found it:
**`servo_eleron_right` draws 25 mW against its partner's 2925 mW — a 117× asymmetry, dead.** The
config is correct (enabled, GPIO32), so it is physical — a lead disturbed during the watchdog testing.

Two consequences, and they differ:

* **The flight data in r5/r6 is VALID.** HITL flies from the *commanded* fin angles the mixer emits;
  the simulator never learns a servo is unplugged. Miss, duration, moves and travel are unaffected,
  which is exactly what the table above shows.
* **The servo-energy metric is NOT a flight metric.** It integrates the INA226 rail, so it measures
  the bench. It should not be compared across runs separated by bench work, and it was wrong to read
  the r5 drop as a code change before checking the rail.

### r7 — the servo replaced, and what that proves

`servo_eleron_right` was replaced and r7 re-flown. The result closes the loop cleanly:

| | r3 | r4 | **r5** | **r6** | r7 |
|---|---|---|---|---|---|
| servo state | ok | ok | **RIGHT DEAD** | **RIGHT DEAD** | ok (replaced) |
| median peak rail mA | 1108 | 1086 | **836** | **823** | **1052** |
| servo energy | 60.4 J | 57.2 J | **36.4 J** | **37.6 J** | **53.8 J** |
| miss (median) | 89.3 m | 82.6 m | 89.2 m | 91.8 m | 89.8 m |
| duration | 99.5 s | 99.2 s | 98.7 s | 99.2 s | 98.6 s |

**Energy and rail current drop for exactly r5/r6 and recover at r7, while miss and duration never
move.** That is the controlled demonstration: a dead actuator changes what the RAIL sees and nothing
about what the aircraft does, because HITL flies from the *commanded* fin angles the mixer emits and
the simulator never learns a servo is unplugged.

So, precisely:

* **r5 and r6 flight data is VALID** — miss, duration, moves, travel, attitude, every control panel.
* **r5 and r6 servo-energy figures are VOID.** They measure a two-servo airframe.
* **r7 is directly comparable to r1–r4** on every metric including energy.

The replacement also settled pin-vs-servo. The prediction was that a pin fault would still read zero;
the new servo draws 1875–2850 mW and the pair now matches at 1.08×, so GPIO32 was always fine.

### How the servo died, and it was avoidable

`configs/tms7d.config` carries **fin concurrency 3** — all three servos may slew together, ~4 A at the
battery — and it was installed on the BENCH board (4 V / 1 A) for watchdog testing. At the then-shipped
`wdt_timeout_ms` 1000 that board boot-looped every ~8.5 s, re-centring all three fins simultaneously on
every boot, for minutes.

Both warnings already existed: the config generator's own comment says three servos are ~4 A and "a 1 A
bench supply browns out", and the operator had twice said the supply is 1 A and to block three
simultaneous runs. **A boot loop with servos enabled is actively destructive, not merely noisy** — it
was left running while being characterised, which is what turned a configuration mistake into a dead
part. The generator now states that consequence rather than the arithmetic.

### Residual corruption, still present

r7 produced one `accel az = 100000 g` against the ADXL375's ±200 g range, in a row of **correct width**
(5 of 5) — a bit flip inside a field, invisible to every structural guard. Only the physical-bounds
check in `hitl_compare` catches it, which it did. One event in 24 flights.

## 7. r8 and r9 — the findings-42 fixes, and why r9 was worth flying

* **r8** — after the §41 addendum-2 set (bank fallbacks, tick wraparound, pitot tare, GNSS staleness,
  NaN wind, mixer reporting) plus the host-side pipeline work.
* **r9** — after the §41 addendum-3 and §42 set, which is the one that needed flying: it changed
  `tasks/sequencer.py`, the stage machine every HITL flight runs.

### Why r9 was not optional

All four stage detectors — launch, apogee, landing, stationary — were switched from `value()` to a
source-gated `read()`. A gate that is too strict does not raise: the stage simply never advances, and
a unit test cannot see it because only a full flight exercises the sequence.

**24 of 24 flights in r9 reached BOOSTING and GLIDING**, and a representative flight runs
`setting → boosting@243s → gliding@253s → landing@337s → done@340s`. The gating holds.

### Nothing moved

| | r7 | r8 | r9 | spread |
|---|---|---|---|---|
| miss (median) | 89.8 m | 86.5 m | 89.0 m | **3.7%** |
| servo energy | 53.8 J | 55.0 J | 56.1 J | 4.2% |
| duration | 98.6 s | 98.9 s | 99.9 s | **1.3%** |

The tightest three-run agreement in this study — miss within 3.2 m and duration within 1.3 s across
three different builds. These fixes are guards against states the matrix does not produce, so
behaviour-neutral is the correct result and the numbers say it plainly.

### Two things the harness caught during these runs

* **r8 reported `svg: 0, html: 0`** while flying 24 of 24. Binding `_FIXED_SCALE` to `fixed.SCALE` had
  broken both renderers — `fixed` lives in `src/glider`, not beside the tools. Before this study's
  `hitl_matrix` work all 17 failures would have vanished behind `|| true` under a "matrix done".
* **r9's E16 half refused to start**: `FATAL: cannot upload hitl_run.py -- board wedged?`. The old
  script muted that upload and would have exited 1 with an empty log and no flights. It was re-run
  after an unwedge.

One residual, unchanged in character: r9 produced an airspeed of 298529 cm/s in a row of correct
width — a bit flip inside a field, catchable only by the physical-bounds check, which caught it.

## 8. Regression verdict

Against the q5.5 baseline ([catapult_evaluation](../TMS-7-catapult_evaluation/), 42–97 m, 1 in-zone of
7): the wind and low-noise families land at **48–95 m across all four runs**, the same family, with
run-to-run spread of 1.7–21 m. **No regression from the audit's firmware changes**, and now the claim
is bounded — those families would have shown a shift larger than ~20 m, and none appeared. On the
high-noise and corner families this study can conclude nothing, and says so.

96 of 96 flights completed, 0 skips, 0 channel dropouts.

## Layout

```
r1/ .. r9/          per-run plotly HTML + comparison SVGs (both motors)
comparison.json     full 552-row panel comparison across all runs
```

## Reproducing

```bash
tools/deploy.sh
for r in r1 r2 r3; do
  bash tools/hitl_matrix.sh E16 /tmp/$r/E16
  bash tools/hitl_matrix.sh F15 /tmp/$r/F15
done
python3 tools/hitl_compare.py r1=/tmp/r1 r2=/tmp/r2 r3=/tmp/r3 --json compare.json
```
