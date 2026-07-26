# TMS-7 physics refresh — drag polar, gusts, fault injection (host)

The sim gained three things it was missing (findings §27.20–22), and this is the canonical matrix
re-flown on the result. Host virtual-flight provenance (`tools/virtual_flight.py` — the real
`guidance`/`governor`/`pid`/`mixer` over `sim_model.py`, 5 % noise, calm, seed 1), E16/F15 × full/light
glider, at the worst-case **quality 2** polar and the realistic **quality 5**.

## What changed in the model

- **Drag polar (§27.22).** Sink was a function of bank only; now it follows the AIRSPEED polar too —
  profile drag ∝ v³, induced ∝ 1/v, normalised to exactly 1.0 at the 14 m/s trim. Minimum sink sits at
  **~0.76 × trim (~10.6 m/s)**, below best-glide speed, as on any real glider. Flying well off trim now
  costs energy in *both* directions, which is what turns an airspeed-gated bank into a real trade-off
  (it unblocks the turn-radius #5.2 work). **Calm runs at trim are unchanged** — the shape was the gap,
  not the magnitude.
- **Gusts / turbulence (§27.21).** `gust` (1-σ, m/s) adds an Ornstein-Uhlenbeck disturbance to the steady
  wind, correlated over `gust_tau` — a glider integrates gusts, so the slow wander matters, not per-step
  jitter. **Default 0 (calm), so every earlier study reproduces exactly.**
- **Fault injection (§27.20).** `sim_model.Faults('gnss@30,pitot@45')` degrades a channel at a chosen
  time. Dead channels publish `None`; a faulted **pitot RAILS** instead, because a saturated
  differential-pressure cell *under-reads* — the more dangerous failure, since a low airspeed *loosens*
  the fin cap.

Knobs: `VF_GUST`, `VF_GUST_TAU`, `VF_FAULT`, `VF_GLIDER_G` (+ the existing `VF_QUALITY`, `VF_SEED`).

## Canonical matrix — touchdown miss from zone centre

| case | motor | glider | quality 2 (floor) | quality 5 (realistic) | report |
|---|---|---|---|---|---|
| `e16_full` | E16 | 270 g | 85.8 m ✗ | 88.3 m ✗ | [report](e16_full.html) |
| `e16_light` | E16 | 215 g | 38.9 m ✗ | **19.9 m ✓ in-zone** | [report](e16_light.html) |
| `f15_full` | F15 | 270 g | 109.7 m ✗ | **27.0 m ✓ in-zone** | [report](f15_full.html) |
| `f15_light` | F15 | 215 g | 114.4 m ✗ | 56.0 m ✗ | [report](f15_light.html) |

[**Top-down plan**, all four at q5](plan_canonical.svg).

Consistent with the endgame study: **the polar dominates the landing**. Nothing reaches the zone at the
quality-2 floor; at the realistic quality 5 half the matrix lands in-zone. The lighter E16 and the full
F15 are the two that convert their glide into accuracy.

## Gust robustness (new capability)

F15, quality 5, calm baseline = 27.0 m:

| gust (1-σ) | miss | in-zone |
|---|---|---|
| 0 m/s (calm) | 27.0 m | ✓ |
| 1.5 m/s | 31.0 m | ✓ |
| 3.0 m/s | 39.0 m | ✓ |

**The converging endgame degrades gracefully** — a 3 m/s gust field costs ~12 m of accuracy and still
lands in-zone. That was the open question §27.21 raised: the `ov` result was previously proven only in
steady air, so its robustness margin was unknown. It holds.

## Fault matrix (new capability)

F15, quality 5, fault injected at t = 30 s:

| fault | miss | in-zone | reading |
|---|---|---|---|
| `pitot@30` (rails) | **27.0 m** | ✓ | **identical to baseline** — the saturation guard drops the governor back to the accel backbone at zero cost |
| `baro@30` (dead) | 100.2 m | ✗ | the endgame band is elevation-driven; losing it costs the descent plan |
| `gnss@30` (dead) | 720.2 m | ✗ | guidance falls to the blind tier — position is the one irreplaceable input |

The pitot row is the useful confirmation: the saturation fallback built for the SDP810 fusion behaves
exactly as designed under a real injected failure. The GNSS row quantifies what "GNSS is
flight-critical for accuracy" actually costs.

## On real hardware (board HITL)

The host runs above are the matrix; this is the same flight flown **on the board** — real drivers, real
servos physically moving, real INA226 on the servo rail, real MCU memory.
[**Report**](hitl_f15_full.html) · [**video (FHD, 64 s)**](hitl_f15_full.mp4).

| measured on the board | value |
|---|---|
| servo-rail power | **peak 5850 mW**, mean 101 mW, **6.1 J** over the 60 s flight |
| fin activity | 707 moves, 1594° of travel (27 °/s of flight) |
| `fin_cap` (governor authority) | swept **14 → 45°** — the unconfident floor, then the live 1/v² schedule |
| airspeed estimate | 0 → 14.4 m/s |
| pitot `dynamic_pressure` | ~0 Pa — correct: the SDP810 is on the bench with no airflow |

This run is what validated the capture-pipeline work end to end: `flight.csv` (542 rows), the three
per-servo streams and `airspeed_sdp810.csv` all came off a real board, and the fin/authority/airspeed
panels render from them.

### Memory: where the GC-off bytes actually go

Three fixes landed, each measured on the board rather than guessed:

- **`Telemetry.push` 240 → 176 B.** The wire line is now formatted and encoded **once** (`tlm()` used to
  re-copy the whole row to prepend the session prefix), and telemetry rows carry **centi-unit integers
  instead of floats** — a float in a row is heap-boxed on MicroPython *and* then `str()`-formatted.
- **Bus read 320 → 80 B (−75 %).** `i2cbus`/`spibus` took an `asyncio.Lock` per operation; the lock alone
  cost **288 of those 320 B**. Every locked section is ONE synchronous, non-yielding `machine.I2C` call,
  and MicroPython's loop is cooperative, so the lock protected nothing — and being released *between*
  calls, it never gave multi-step atomicity either. `Bus.transaction()` is now the explicit escape hatch
  for a sequence that genuinely must not interleave.
- **A per-tick raised exception.** `flight._record()` pushed telemetry every tick; with the Recorder not
  running that RAISED and was caught every tick. `Telemetry.due()` now reports not-due when there is no
  ring to write to. In `bench_flight` this alone was **720 of the 816 B** it attributed to a control step
  — the real control path is **96 B/step (22 KB/s at 100 Hz, OOM ~1450 s)** and was never the problem.

#### How much of a HITL leak is the simulator?

The sim is float physics, and every intermediate is a boxed float — so it is the standing suspect. It is
now measured two independent ways (`make bench-hitl`, and a flown A/B where each arm changes exactly one
rate). F15, noise 0.05, calm, 285 g, ~56 s airborne:

| arm | leak | isolates |
|---|---|---|
| `inject_hz` 50 (default) | 224, 255 KB/s | baseline — two runs, the spread is real (~±16) |
| `inject_hz` 10 | 193, 194 KB/s | −40 Hz of `_publish` → **46 KB/s** (~1150 B each) |
| `inject_hz` 10 + `sim_hz` 25 | 168 KB/s | −25 Hz of `glide_step` → **26 KB/s** (~1040 B each) |

At the default rates that is publish ~58 + physics ~52 = **~110 KB/s, 46 % of the ~240 KB/s baseline,
none of which a real flight runs**. The per-call bench agrees in shape but reads ~40 % high (145 KB/s) —
a tight bench loop takes branches a real flight skips, so it is an upper bound and the flown A/B is the
number to quote.

**A real flight's leak is therefore ~130 KB/s → OOM ~250 s**, not the ~240 KB/s / ~135 s a HITL capture
shows. `board_health` carries the OOM prediction and the safe-height elimination logic as the standing
mitigation, and a dedicated soak still belongs before any powered flight — but the headline HITL figure
should not be read as a flight number.

One trap worth recording: the first `sim_hz` arm measured *nothing*, because `hitl_run` imports
`config_hitl` **from the board**. Editing the host copy without deploying changes nothing.

## Regenerate

```bash
SP=/tmp/canon; mkdir -p $SP
for q in 2 5; do for case in "E16 270 e16_full" "E16 215 e16_light" "F15 270 f15_full" "F15 215 f15_light"; do
  set -- $case
  VF_QUALITY=$q VF_GLIDER_G=$2 VF_SEED=1 python3 tools/virtual_flight.py --motor $1 --noise 0.05 \
      -o $SP/${3}_q$q.txt
done; done
PV=~/.local/share/pipx/venvs/plotly/bin/python
for n in e16_full e16_light f15_full f15_light; do
  $PV tools/flight_report.py --cdn $SP/${n}_q5.txt -o doc/sims/TMS-7-physics_refresh/$n.html; done
# gust sweep / fault matrix
VF_GUST=3.0 VF_QUALITY=5 VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/gust3.txt
VF_FAULT=gnss@30 VF_QUALITY=5 VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/f_gnss.txt
```
