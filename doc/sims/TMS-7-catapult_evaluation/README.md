# TMS-7 catapult evaluation — flying the **measured** polar

The same seven board HITL flights as [TMS-7-precatapult_evaluation](../TMS-7-precatapult_evaluation/),
re-flown with one thing changed: `sim_model.AIR_QUALITY` **2.0 → 5.5**. Nothing else moved — same
masses, same 5 % noise, same calm air, same guidance — so every delta below is attributable to the
polar alone.

5.5 is **measured**, not guessed. On 2026-08-12 a hand toss of TMS-7B (179.5 g, 2 cameras, no
avionics) glided **8.0 m from a 1.40 m release**, against a **2.70 m** ballistic reference thrown at
the same speed with an inert 173.1 g dummy of the same mass — every figure the mean of 3 attempts. The
dummy is what makes it a measurement rather than a guess: it carries the launch energy without lift,
so the glider's extra 5.3 m is bought by the wing and nothing else. Geometric and energy methods agree
to a decimal, giving **L/D 5.7** at the stated height and 5.3 if the release was really 1.5 m; **5.5 is
the midpoint** of that residual band.

The old 2.0 was an explicit placeholder — "the deliberately pessimistic worst-case floor… RE-CALIBRATES
from the first real glide telemetry". This is that re-calibration.

Treat 5.5 as a **floor**, not a best estimate: it was measured at ~5 m/s, below the airframe's trim
speed and at Re ≈ 44 000, where a thin wing is well short of its best L/D. The figure at the 7–10 m/s a
catapult or boost delivers should be better, and the sim erring pessimistic is the safe direction.

[f15_full](f15_full.html) · [f15_light](f15_light.html) · [e16_full](e16_full.html) ·
[e16_light](e16_light.html) · [calm](calm.html) · [gnss_dead](gnss_dead.html) ·
[attitude_dead](attitude_dead.html) · [**overlay**](plan_matrix.svg) · [**video**](matrix.mp4)

## Headline — accuracy got *worse*, and that is the real result

| case | q2.0 (before) | **q5.5 (now)** | in-zone | duration q2 → q5.5 |
|---|---|---|---|---|
| `e16_light` | 38 m ✅ | **42 m** | ✅ | 36.4 → 60.8 s |
| `e16_full` | 24 m ✅ | **55 m** | no | 31.4 → 49.9 s |
| `f15_full` | 82 m | **64 m** | no | 55.4 → 101.8 s |
| `calm` (0 % noise) | 52 m | **84 m** | no | 57.8 → 118.2 s |
| `f15_light` | 14 m ✅ | **84 m** | no | 62.9 → 117.8 s |
| `gnss_dead` (60 s) | 60 m | **97 m** | no | 54.8 → 99.7 s |
| `attitude_dead` (30 s) | 79 m | **149 m** | no | 55.3 → 72.0 s |

**3 in-zone → 1.** Flights roughly doubled in length, as the polar predicts, and the landings got
worse almost across the board.

That is not a regression introduced by this change. It is the change **removing a coincidence**: the
precatapult study's "first in-zone landings" were the energy budget running out at the right moment,
not the guidance holding position.

## Why — the glider now arrives with altitude to spare, and cannot spend it

Closest approach to the zone centre, and how much altitude was left at that instant (host runs, seed 1):

| case | closest approach | altitude there | flying time left | touchdown |
|---|---|---|---|---|
| `f15_215` **q2** | 3 m | **8 m** | 1 s | 13 m ✅ |
| `f15_215` **q5.5** | 2 m | **27 m** | 11 s | 101 m |
| `e16_270` **q2** | 26 m | **1 m** | 0 s | 26 m ✅ |
| `e16_270` **q5.5** | 4 m | **58 m** | 23 s | 57 m |

At the pessimistic polar the glider reached the zone with **1–8 m of altitude** — it arrived and
touched down in the same breath, so where it arrived *was* where it landed. At the measured polar it
reaches the same 2–4 m of the centre with **27–58 m in hand**, then spends the next 11–23 s flying the
holding pattern, which carries it away.

This is not a host artefact — the board flights do the same thing, and if anything more sharply:

| board flight, q5.5 | closest approach | touchdown |
|---|---|---|
| `e16_light` | **1 m** | 42 m ✅ |
| `e16_full` | **5 m** | 55 m |
| `f15_light` | **1 m** | 84 m |
| `f15_full` | **3 m** | 64 m |

**Every flight, host and board, flies within 1–5 m of the target and then lands 42–101 m away.** The
navigation is not the problem; it puts the glider over the zone every time. What is missing is any way
to *stop being* over the zone at altitude and *start being* on the ground there.

**The guidance has no mechanism to convert excess altitude into a landing at the zone.** The endgame
patterns are *positioning* patterns; none of them is an energy-dissipation pattern.

## The endgame patterns are a phase lottery, and `auto` is picking the worst one

If a pattern does not converge before touchdown, the landing point is just orbit **phase** when
altitude hits zero — so the miss is set by orbit size, and re-rolls with any change to the energy
budget. Sweeping the polar directly (F15 270 g, HPRC strip k ≈ 4.7, median of 3 seeds):

| pattern | q2.0 | q3.0 | q4.0 | q4.5 | q5.0 | q5.5 | q6.0 | q6.5 | **span** |
|---|---|---|---|---|---|---|---|---|---|
| **`o`** circle | 33 | 34 ✅ | 33 | 44 | 40 | 39 | 55 | 42 | **21 m** |
| `ov` oval | 75 | 117 | 16 ✅ | 71 | 98 | 56 | 6 ✅ | 68 ✅ | **111 m** |
| `oo` two-lobe | 64 | 53 | 59 | 68 | 17 ✅ | 82 | 64 | 15 ✅ | **67 m** |

*(✅ = in-zone on all three seeds.)* Seeds inside a column agree tightly — `ov` at q5.0 gives 98/102/98
— so the swing is **not noise**. It is the polar. `ov` moves 111 m across the range; `o` moves 21 m,
because a 30 m circle can only lose you 30 m.

The measured L/D is 5.5 with roughly ±0.2 of measurement slack, and it is a **floor** — taken below
trim speed at Re ≈ 44 000, so the real operating point is somewhere in 5–7 and will not be known
precisely until a capture with a few seconds of steady glide exists. Over that band `ov` ranges 6–98 m
and `oo` 15–82 m. **`o` is the only pattern whose accuracy can be predicted before the flight.**

`auto` currently selects on aspect alone (`guidance.Heading`: k < 2 → `o`, 2 ≤ k < 6 → `ov`, k ≥ 6 →
`oo`). The HPRC strip is k ≈ 4.7, so **`auto` flies `ov`** — the highest-variance pattern of the three.

That `OO_ASPECT = 6.0` boundary was set by [TMS-7_endgame_x3](../TMS-7_endgame_x3/) (2026-07-15), which
at quality 5 measured `ov` 27 m and `oo` 74 m and concluded "`ov` now beats `oo` at the realistic
polar". Re-measured at quality 5 on the current tree, that comes out the other way round: **`oo`
12–18 m, `ov` 98–102 m**. `guidance.py` has changed twice since (#37, #38), so these are not the same
experiment and the older number is not being called wrong — but the boundary it justifies no longer
reproduces, and the sweep above says no single-polar measurement could have justified it anyway. The
old study also reported `ov` scattering 27/219/22 m across seeds 1/2/3, where the current tree gives a
tight 98/102/98 — so what moved is not only the value.

> **Open decision, not taken here.** Pinning `endgame_pattern` to `'o'` trades the chance of a 6 m
> landing for a dependable ~40 m one. That is a design call about what the mission wants, so it is
> flagged rather than changed. The alternative — an endgame that genuinely converges (a descending
> spiral that shrinks with altitude, or an explicit dissipation phase) — is a bigger piece of work and
> the only route to landing accuracy that does not depend on knowing L/D in advance.

## Board and host now describe the same airframe

`AIR_QUALITY` used to exist twice — `sim_model.trim_sink = 7.0` on the board and
`14.0 / VF_QUALITY` defaulting to 2.0 on the host. Same physical claim, two copies, free to drift.
It is now one constant that both read. The agreement is the evidence:

| variant | host, median of 5 seeds | board HITL | host duration | board duration |
|---|---|---|---|---|
| `f15_full` (270 g) | 56 m | 64 m | 100 s | 101.8 s |
| `f15_light` (215 g) | 94 m | 84 m | 117 s | 117.8 s |
| `e16_full` (270 g) | 55 m | 55 m | 48 s | 49.9 s |
| `e16_light` (215 g) | 50 m | 42 m | 59 s | 60.8 s |

Durations match within ~2 s across a 100 s flight, and misses within ~10 m against a ±5 m seed spread.

## Also changed

- **`reachability.glide_ratio` 3.0 → 5.5.** The flight code carried its own L/D for the CC panel's
  "zone reachable y/n". It is **advisory** — `tasks/flight._flight_panel()` reads it and nothing
  steers on it — so the stale value never cost a flight, but it did tell the operator a reachable
  zone was out of range. `CONFIG_VERSION` bumped accordingly.
- **`hitl_collect.sh` wall-clock timeout 190 s → 300 s.** The board flight runs in real time and its
  own cap is 150 s; at the measured polar an `f15_light` glide lasts ~118 s where the old floor gave
  ~58 s. 190 s would have truncated the longest cases into a false `TIMEOUT`.
- **`test_sim_model.test_stall`** seeded its probe at a hardcoded `-7.0`, which *was* the old trim
  sink. Once the sink moved, the negative case passed for the wrong reason — any value above a fixed
  `-7.5` satisfies it, stalled or not. It now seeds at the trim sink itself.

## Reproduce

```bash
tools/deploy.sh
mpremote connect /dev/ttyACM0 cp tools/hitl_run.py :
for spec in "F15 f15_full 270" "E16 e16_full 270" "E16 e16_light 215" "F15 f15_light 215"; do
    set -- $spec
    bash tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False captures/q55 "$3"
done
bash tools/hitl_collect.sh F15 calm          0.0  0.0 210.0 False captures/q55 270 0 0 False 0  0
bash tools/hitl_collect.sh F15 gnss_dead     0.05 0.0 210.0 False captures/q55 270 0 0 False 0  60
bash tools/hitl_collect.sh F15 attitude_dead 0.05 0.0 210.0 False captures/q55 270 0 0 False 30 0
python3 tools/flight_metrics.py captures/q55 --sort name

# the polar sweep behind the phase-lottery table (host, ~0.6 s per flight)
for q in 2.0 3.0 4.0 4.5 5.0 5.5 6.0 6.5; do for p in o ov oo; do for s in 1 2 3; do
  VF_ENDGAME=$p VF_QUALITY=$q VF_SEED=$s python3 tools/virtual_flight.py --motor F15 --noise 0.05 \
    -o /tmp/qs/${p}_q${q}_s${s}.txt --no-preflight
done; done; done
python3 tools/flight_metrics.py /tmp/qs --sort name
```
