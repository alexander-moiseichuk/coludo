# TMS-7 — post-audit regression, flown four times

**96 board flights**: the 12-scenario matrix × F15-4 and E16-4 × four independent runs. `r1`–`r3` are
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

## 5. Regression verdict

Against the q5.5 baseline ([catapult_evaluation](../TMS-7-catapult_evaluation/), 42–97 m, 1 in-zone of
7): the wind and low-noise families land at **48–95 m across all four runs**, the same family, with
run-to-run spread of 1.7–21 m. **No regression from the audit's firmware changes**, and now the claim
is bounded — those families would have shown a shift larger than ~20 m, and none appeared. On the
high-noise and corner families this study can conclude nothing, and says so.

96 of 96 flights completed, 0 skips, 0 channel dropouts.

## Layout

```
r1/ r2/ r3/ r4/     per-run plotly HTML + comparison SVGs (both motors)
comparison.json     full 552-row panel comparison across all four runs
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
