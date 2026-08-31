# TMS-7 — preflight regression: three load/thrust combos, flown three times

**90 board flights.** Three load/thrust combos × 10 scenarios × 3 independent runs, flown on the bench
board after the findings-43 and findings-44 fixes, to answer two questions before a field day:

1. did the audit's firmware changes move the aircraft (they should not have), and
2. which motor/mass combination should actually fly.

The second question turned out to have a much sharper answer than the first.

## The combos

| combo | motor | glide mass | total impulse | intent |
|---|---|---|---|---|
| `e16_full` | E16 (16.1 N × 1.77 s) | 285 g | **28.5 N·s** | worst load/thrust |
| `f15_full` | F15 (14.4 N × 3.45 s) | 285 g | **49.7 N·s** | typical |
| `f15_half` | F15 | **235 g** | 49.7 N·s | best load/thrust |

Mass is the documented pair (`hitl_run.py`: "285 full, 235 light") — 285 g is airframe + **full**
payload, 235 g is airframe + **half** payload. Not half the airframe: the payload budget is ~100 g
(`coludo.md`), so "half" removes ~50 g.

`noise50` and `noise100` were dropped from the postaudit's twelve. They are the two scenarios whose own
run-to-run spread (99.9 m and 154.4 m) exceeds anything a build change could produce, and this study
reproduced that immediately: on the same firmware an hour apart, `noise100` read **113.2 m** on
`e16_full` and **49.3 m** on `f15_full`, and `noise50` read **205.0 m** and **27.1 m**. Reporting
either as an effect would have been fabrication.

## 1. The headline: mass, not code

Median miss over the ten scenarios, and how many landed in the zone:

| combo | r1 | r2 | r3 | median spread | in zone |
|---|---|---|---|---|---|
| `e16_full` | 74.0 m | 71.0 m | 76.6 m | **5.6 m** | 1/10 every run |
| `f15_full` | 90.9 m | 90.8 m | 88.0 m | **2.9 m** | 0/10 every run |
| **`f15_half`** | **39.3 m** | **46.3 m** | **37.1 m** | 9.2 m | **7/10 every run** |

`f15_half` lands roughly **half as far out** as either full-payload combo, and the result is not a
median artefact: it put **seven of ten flights in the zone in all three runs, and the same seven every
time** — `noise05`, `noise10`, `noise25`, `wind00`, `wind03`, `wind06`, `corner_spike`. `wind09`,
`wind12` and `corner_stress` stayed out in all three. Reproducing an in-zone *verdict* per scenario
across independent runs is a much stronger result than a matching median.

The mechanism is time aloft, measured rather than assumed:

| combo | typical flight | why |
|---|---|---|
| `e16_full` | ~49 s | least impulse carrying most mass |
| `f15_full` | ~100 s | 74 % more impulse than E16 |
| `f15_half` | ~116 s | same impulse, 50 g less to hold up |

Lower wing loading means slower sink, so the endgame gets more laps to converge. Note this **runs
against** [catapult_evaluation](../TMS-7-catapult_evaluation/)'s finding that excess altitude hurts —
there the glider arrived over the zone with 27–58 m in hand and flew away. The difference is that
lighter mass buys *loiter time*, which the pattern can spend, rather than *height*, which it cannot.

## 2. What is trustworthy, and what is not

The postaudit's central lesson applied again, and this study can now state the bar precisely:

* **Medians are solid.** Three runs agree to **2.9–9.2 m** across all three combos.
* **Single scenarios are not**, and the failure is not confined to noisy cases. Each run produced
  roughly **one 30–60 m excursion, in a different scenario every time**:

| run | combo | scenario | the other two runs | excursion |
|---|---|---|---|---|
| r1 | `e16_full` | `noise25` | 68.0 / 74.9 | +19 m |
| r2 | `f15_full` | `wind06` | 55.9 / 58.6 | **+57 m** |
| r3 | `e16_full` | `wind00` | 68.1 / 74.0 | +26 m |

`wind06` is the clean case: r1 and r3 agree to **2.7 m** and r2 sits 57 m away, so r2 is a lone
excursion rather than a state change — a prediction made before r3 flew and confirmed by it. And
`wind00` is **calm air**, so a 26 m swing cannot be wind.

This is the documented endgame lottery: on the HPRC strip (aspect k ≈ 4.7) the `auto` selector picks
the **`ov`** pattern, whose miss is orbit *phase* at altitude-zero — the highest-variance of the three,
spanning 111 m across a polar sweep. **A future regression claim on this matrix needs to clear ~30 m
per scenario, or use the median.**

## 3. `corner_stress` never lands — its number is censored

`corner_stress` (50 % noise + 12 m/s wind + spike, all at once) hit the board's 150 s flight cap in
**every one of its nine occurrences**, across both motors and both masses. No other scenario timed out
even once.

It reaches stage 4 (`LANDING`) and is cut mid-descent, so its 460–680 m "miss" is **where the glider
was when the clock stopped**, not a touchdown. Two consequences:

* its run-to-run "spread" (up to 92 m) is not an accuracy measurement and must not be read as one;
* the medians above are unaffected — it is always the largest of the ten, so the median of the 5th and
  6th values never touches it. Means would have been badly distorted, which is why medians are used.

The cap looks slightly tight for this one case: `hitl_collect.sh` sizes its *wall-clock* budget at 300 s
specifically so long glides are not falsely truncated, but the board's own 150 s flight cap was not
raised alongside it. Left unchanged here — moving it mid-study would make r3 incomparable to r1/r2.

## 4. Every panel, compared

`tools/hitl_compare.py` reduced all report panels to 23 quantities across the three runs
(`comparison.json`).

* **Channel dropouts: 0.** No sensor went missing in 90 flights.
* **Physically impossible values: 2**, both in rows of *correct width* — an airspeed of 149 198 cm/s
  against a [0, 20 000] range, and an engine power of 1.23e6 mW against [0, 50 000]. These are bit flips
  inside a field, which no structural guard can see; only the physical-bounds check catches them. Same
  class and roughly the same rate as the postaudit's residual (1 in 24 flights there, 2 in 90 here).
* **Wide but expected:** `health mem_free min` (100–230 %) and `health load max` (60–80 %) vary
  genuinely run to run, and `health rescues` went 1/1/3 on `e16_full corner_stress`. Memory pressure is
  not reproducible at this granularity — a conclusion drawn from one run's memory panel is luck.
* `elevation final` shows 300 % spread. That is a ratio about zero (0.01 m vs −0.00 m), not a finding.

## 5. Regression verdict

Against the [postaudit baseline](../TMS-7-postaudit_regression/), the two full-payload combos land in
family scenario by scenario — `e16_full` wind cases at 53.6–100.0 m against a 46.5–95.0 m baseline,
`f15_full` at 55.3–115.0 m against 48.5–114.9 m. **No regression from the findings-43/44 firmware
changes**, which is the expected result: those fixes are guards against states this matrix does not
produce (a silent IMU, an operator re-zero on a dead baro, a stale gyro feeding the D term).

The board suite passed **57/57** on the deployed build before the first flight.

## Caveats

* r1's `e16_full` and `f15_full` were flown before `noise50`/`noise100` were dropped, so they ran a
  12-flight sequence where every later run ran 10. Per-flight miss and duration are unaffected; the
  **memory** panel is not directly comparable for those two cells, since board state accumulates across
  a sequence.
* One `f15_half` duration reads 32.5 s where its siblings read ~116 s. The duration metric is the
  power-stream span, so a sparse INA226 stream shortens it; the flight itself landed in zone at 30.3 m.

## Layout

```
r1/ r2/ r3/        per-run plotly HTML + SVG, prefixed by combo
comparison.json    the full panel comparison across all three runs
```

## Reproducing

```bash
tools/deploy.sh
export SCENARIOS='noise05 noise10 noise25 wind00 wind03 wind06 wind09 wind12 corner_spike corner_stress'
for r in r1 r2 r3; do
  GLIDER_G=285 bash tools/hitl_matrix.sh E16 /tmp/preflight/$r/e16_full
  GLIDER_G=285 bash tools/hitl_matrix.sh F15 /tmp/preflight/$r/f15_full
  GLIDER_G=235 bash tools/hitl_matrix.sh F15 /tmp/preflight/$r/f15_half
done
python3 tools/hitl_compare.py r1=/tmp/preflight/r1 r2=/tmp/preflight/r2 r3=/tmp/preflight/r3 \
  --motors e16_full,f15_full,f15_half --json comparison.json
```

Count the artefacts. A short run prints "matrix done" like any other, and the board's CDC wedged
between halves once during this study — `e16_full` flew 12/12 and `f15_full` then flew **0 of 12**.
