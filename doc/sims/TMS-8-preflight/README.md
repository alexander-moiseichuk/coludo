# TMS-8 — pre-flight study over the SITL: the flat flying-wing frame against TMS-7, F15 at 235 and 285 g

**What it asks.** TMS-8 (`doc/plan.md`, "TMS-8, as the operator set it out", 2026-10-11) is a flat triangle frame
with fixed tips and a tail, no folding wings, the same booster, servos and electronics as TMS-7. The planning
figures: **L/D 8 at a 12 m/s trim** (TMS-7: 5.5 at 14 m/s), the stall unchanged (9 m/s), and an estimated boost CdA
of **0.0020 m²** (TMS-7's stack: 0.0010) for the open frame ~5 cm deep and 20 cm wide with 2 cm tips. Does it land
better or worse than TMS-7, at which mass, and how much does the drag estimate matter?

**How it was flown.** `tools/sitl_matrix.py`: the Rust flight code (the SITL, M14) flown lockstep against
`sim_model` with `--airframe tms7|tms8`, the board's HITL copy of the taster's own config, F15. The pre-flight
report's ten scenarios (calm, noise, wind 0-12 m/s, a spike, a stress corner) x 5 seeded runs x 2 masses x 2
airframes = **200 flights**, scored by the physics itself: the body's landing point against the HPRC zone (a strip
~40 m N-S x ~187 m E-W), the time from ignition to DONE, the apogee. Then a drag sweep for TMS-8: CdA 0.0015,
0.0020, 0.0025, 0.0030 m² x 2 masses x 10 scenarios x 2 runs = 80 flights. Data: `runs.csv`, `summary.csv`,
`drag_runs.csv`, `drag_summary.csv` (';').

**The SITL reproduces the board.** TMS-7 here against the 90 board HITL flights of
[TMS-7-preflight](../TMS-7-preflight/): 235 g lands at a **42.3 m** median (board: 37-46 m) with 35/50 in the zone
(board: 7/10 scenarios); 285 g at **88.8 m** (board: 88-91 m), 0/50 (board: 0/10). So the TMS-8 numbers below sit on
the same footing as the pre-flight report's.

## The headline

| | median miss | in the zone | median time aloft | median apogee |
|---|---|---|---|---|
| TMS-7, 235 g | 42.3 m | 35 / 50 | 112 s | 324 m |
| TMS-7, 285 g | 88.8 m | 0 / 50 | 97 s | 273 m |
| **TMS-8, 235 g** | **51.8 m** | **10 / 50** | **132 s** | **263 m** |
| **TMS-8, 285 g** | **48.0 m** | **26 / 50** | **116 s** | **231 m** |

- **TMS-8 flies at 285 g as TMS-7 flies at 235 g:** a 48 m median against 42 m, half its flights in the zone. The open
  frame costs ~15 % of the height (231 against 273 m at 285 g) and the better glide gives it back in time: 116 s
  aloft, where TMS-7 at the same mass manages 97 s and lands twice as far out.
- **At 235 g TMS-8 stays up longest (132 s) and lands worse.** It comes down 30-45 m from the centre, just outside the
  40 m-wide strip, in the calm and noise scenarios where 285 g lands inside. The endgame's orbit phase decides it,
  not the energy: the endgame lottery of [TMS-7-catapult_evaluation](../TMS-7-catapult_evaluation/) again.
- **Wind is TMS-8's weak side.** At 12 m/s, its own trim speed, it cannot make headway: 140 m at either mass, where
  TMS-7 manages 55-83 m; the stress corner (12 m/s with noise and spikes) costs it 220 m. Up to 9 m/s it holds its
  own.

![median miss per scenario](summary.svg)

| scenario | TMS-7 235 g | TMS-7 285 g | TMS-8 235 g | TMS-8 285 g |
|---|---|---|---|---|
| noise05 | 29 m, 5/5 | 92 m, 0/5 | 43 m, 0/5 | 34 m, 5/5 |
| noise10 | 46 m, 5/5 | 91 m, 0/5 | 32 m, 0/5 | 50 m, 5/5 |
| noise25 | 24 m, 5/5 | 94 m, 0/5 | 125 m, 0/5 | 36 m, 1/5 |
| wind00 | 46 m, 5/5 | 91 m, 0/5 | 32 m, 0/5 | 50 m, 5/5 |
| wind03 | 50 m, 5/5 | 58 m, 0/5 | 51 m, 5/5 | 56 m, 0/5 |
| wind06 | 39 m, 5/5 | 56 m, 0/5 | 68 m, 5/5 | 33 m, 5/5 |
| wind09 | 25 m, 0/5 | 71 m, 0/5 | 33 m, 0/5 | 46 m, 0/5 |
| wind12 | 83 m, 0/5 | 55 m, 0/5 | 140 m, 0/5 | 140 m, 0/5 |
| corner_spike | 43 m, 5/5 | 89 m, 0/5 | 28 m, 0/5 | 46 m, 5/5 |
| corner_stress | 164 m, 0/5 | 133 m, 0/5 | 222 m, 0/5 | 220 m, 0/5 |

(median miss over the five runs, then the runs in the zone; `wind00` and `noise10` are the same conditions)

## The boost drag

| boost CdA (m²) | 235 g: apogee, median miss, in zone | 285 g: apogee, median miss, in zone |
|---|---|---|
| 0.0015 | 290 m, 49 m, 9/20 (4 still flying at the 150 s cap) | 250 m, 78 m, 4/20 |
| **0.0020 (the estimate)** | **263 m, 52 m, 4/20** | **231 m, 51 m, 11/20** |
| 0.0025 | 243 m, 83 m, 3/20 | 215 m, 19 m, 12/20 |
| 0.0030 | 227 m, 37 m, 13/20 | 203 m, 54 m, 6/20 |

- **The apogee follows the drag smoothly:** ±~30 m around the estimate over the range a printed frame could land in.
- **The landing does not.** The medians jump 19-83 m with no trend, because a few metres of height move the endgame
  orbit's phase at touchdown. That is the pre-flight report's trust bar again: a single condition needs ~30 m to mean
  anything; the matrix median is the number to quote.

## What it means for the build

- **Fly TMS-8 at 285 g** (the full build): it matches TMS-7's best combination, and needs no lightening.
- **Keep it out of strong wind**, or give it a faster trim on windy days (a forward CG or a pitch trim): at 12 m/s
  trim speed, a 12 m/s wind stalls its progress.
- **Measure the real TMS-8 before trusting any of this.** The polar (L/D 8 at 12 m/s), the boost CdA and the control
  response are planning figures, and the control law flies TMS-7's gains; a measured glide (a catapult or a hand
  launch, as for TMS-7's polar) replaces `AIRFRAMES['tms8']` and this study re-flies in three minutes.

Re-run: `tools/sitl_matrix.py --config <a board's saved config> -o <dir>` (the study) and
`... --airframes tms8 --runs 2 --boost-cdas 0.0015 0.0020 0.0025 0.0030` (the sweep).
