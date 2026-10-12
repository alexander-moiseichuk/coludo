# TMS-7 — the Rust board against plan B, the whole matrix (M14)

**The Rust flight firmware flies the 12-scenario HITL matrix (noise, wind, corners) × {E16, F15} like plan B.** The
comparison runs both stacks on the same frames: the Rust board on the taster (`tools/hitl_matrix_host.sh`), the Rust
SITL (`src/avionics/sitl`) and plan B's own flight code (`tools/planb_sitl.py`). All three sit behind one harness,
`tools/hitl_host.py` on `src/glider/sim_model.py`, and `tools/hitl_matrix_compare.py` judges them.

All flights: 285 g glider, the HPRC pad and zone, plan B's HITL test gains, the scenario table of `hitl_matrix.sh`.

## 1. The reference had to be plan B's own code

The first reference was `tools/virtual_flight.py`, and against it the Rust stack looked 10–20 m worse in the wind.
That was the harness, not the port. virtual_flight's loop:

- feeds **no wind estimator** ("this loop feeds no wind estimator");
- hands its law the GNSS **total speed, unnoised**, where plan B's own on-board HITL (`tasks/hitl.py`) publishes the
  noised horizontal ground speed and track;
- noises the barometer at the full `--noise`, where `tasks/hitl.py` scales it ×0.05. With that model it detected
  apogee early under noise, 100 m below the true one.

So it was never plan B's full stack. `tools/planb_sitl.py` is: plan B's Controller, `tasks/sequencer.py` and
`tasks/flight.py` under the golden shims, behind the same sim-link, lockstep. Fed the same frames, the two stacks
fly the same flight:

| F15, 285 g | miss | GLIDING | LANDING | fin travel |
|---|---|---|---|---|
| clean, Rust SITL | 41.6 m | 10.16 s | 108.9 s | 29.2 deg/s |
| clean, plan B SITL | 41.6 m | 10.16 s | 108.9 s | 28.9 deg/s |
| wind 6 m/s, the Rust board (HITL, M13) | 37.3 m | 10.10 s | 108.6 s | 27.8 deg/s |
| wind 6 m/s, Rust SITL | 37.4 m | 10.16 s | 108.5 s | 27.7 deg/s |
| wind 6 m/s, plan B SITL | 37.4 m | 10.16 s | 108.5 s | 27.9 deg/s |

## 2. The SITL matrix: 72 Rust flights, all inside plan B's

Three seeds per scenario (`hitl_host --seed`) on both SITLs. A seed's frames are the same for both stacks while
their flights agree. Gates: the apogee (3 %), GLIDING (0.5 s) and DONE on every scenario; the miss on the wind and
low-noise families, widened by the post-audit board matrix's run-to-run spread (7.7 m, 11.9 m).

**72 of 72 pass** ([table](sitl_vs_planb.md)). Seed for seed:

| family | pairs | miss difference, median / max | GLIDING difference, max |
|---|---|---|---|
| wind 00–12 | 30 | 1.0 / 7.6 m | 0.00 s |
| low noise 05–25 | 18 | 0.3 / 8.6 m | 0.00 s |
| corner spike/stress | 12 | 1.1 / 8.0 m | 0.00 s |
| high noise 50–100 | 12 | 9.9 / 118.6 m | 0.00 s |

The stage machine never differs. The misses fork only at high noise, where a one-degree rounding difference in a fin
sends the flights onto different tracks, which then meet different noise. The post-audit study measured that
family's run-to-run spread at 80–150 m on plan B alone.

All 144 lockstep flights (both stacks) took about 30 s on the host: a regression matrix for any law change, no board.

## 3. The board matrix: 24 flights on the taster, realtime

`tools/hitl_matrix_host.sh`, run r1. **23 of 24 pass** against plan B's seeds ([table](board_r1_vs_planb.md)).
Per scenario, the miss from the zone centre (m):

| motor | scenario | board r1 | Rust SITL s1/s2/s3 | plan B SITL s1/s2/s3 |
|---|---|---|---|---|
| E16 | noise05 | 76.7 | 78 / 79 / 75 | 78 / 79 / 75 |
| E16 | noise10 | 73.7 | 76 / 82 / 82 | 75 / 82 / 80 |
| E16 | noise25 | 78.6 | 113 / 116 / 76 | 122 / 116 / 74 |
| E16 | wind03 | 69.7 | 57 / 62 / 65 | 57 / 65 / 65 |
| E16 | wind06 | 53.6 | 57 / 52 / 55 | 53 / 49 / 54 |
| E16 | wind12 | 91.7 | 92 / 91 / 90 | 91 / 90 / 90 |
| F15 | noise05 | 92.5 | 92 / 93 / 91 | 92 / 93 / 91 |
| F15 | wind06 | 54.1 | 57 / 59 / 52 | 58 / 51 / 53 |
| F15 | wind09 | 65.2 | 66 / 73 / 62 | 72 / 74 / 62 |
| F15 | wind12 | 57.0 | 56 / 55 / 55 | 57 / 56 / 54 |
| F15 | corner_stress | 130.1 | 127 / 137 / 140 | 126 / 129 / 134 |

(the full 24 rows in the table file; chart: [`f15_wind06_board_vs_planb.html`](f15_wind06_board_vs_planb.html),
[SVG](f15_wind06_board_vs_planb.svg))

- **The one failure:** E16 corner_stress (noise 50 %, wind 12 m/s, spikes) detected apogee 0.9 s early on the board
  (GLIDING 5.6 s; plan B's seeds 6.5–6.9 s). Flown three more times, it gave 6.70, 6.75 and 6.50 s, all inside.
  Neither SITL did it in 10 seeds (both 6.2–6.8 s, the same per seed). It was one realtime flight's noise, in the
  family the spec does not gate.
- **No noisy flight lands in the zone, on either stack.** With noise at 5 % or more every miss is 50–100 m; clean,
  both land at ~40 m inside. This is plan B's behaviour on today's sim, reproduced, not something the port changed.

## 3b. The second board run (r2, 2026-10-12): the board's own run-to-run spread

The same firmware build and harness as r1, a day later. One flight (F15 noise10) was refused at the arm: the taster's
INA226 did not answer at that boot. It flew on the next boot. All 24 reached DONE.

Against plan B's three seeds, r2 failed two gates. Both outliers come from plan B's own spread, which three seeds do
not cover. Flown with 20 seeds, plan B gives a wider range, and **r2 passes all 24**. Table:
[`board_r1_r2_vs_planb20.md`](board_r1_r2_vs_planb20.md).

| scenario | r2 | plan B, 3 seeds | plan B, 20 seeds |
|---|---|---|---|
| E16 noise25, the miss | 156.3 m | 74.1..121.5 m | 72.4..188.6 m |
| F15 noise100, GLIDING | 7.8 s | 8.8..10.5 s | 7.5..10.5 s |

r1's one failure (E16 corner_stress, GLIDING 5.6 s) stays just outside the 20 seeds too (6.2..6.9 s, gate 0.5 s). Its
three re-flights and r2 (6.5 s) were inside.

r1 against r2, the same scenario flown twice on the board:

| family | pairs | miss difference, median / max | GLIDING difference, max |
|---|---|---|---|
| wind 00-12 | 10 | 2.7 / 9.8 m | 0.10 s |
| low noise 05-25 | 6 | 2.5 / 77.7 m (E16 noise25) | 0.15 s |
| corner spike/stress | 4 | 0.6 / 3.2 m | 0.80 s |
| high noise 50-100 | 4 | 11.0 / 143.3 m (F15 noise100) | 1.45 s |

The wind family repeats within ~10 m. This sits next to the 7.7 m median the miss gate takes from the post-audit
board study. noise25 forks the way plan B's seeds fork. The stage machine repeats within 0.15 s outside the
high-noise and corner families.

## 3c. The drops on the SITLs, and the sim's IMU frames (2026-10-12)

The lockstep SITLs now fly the realtime matrix's drops: the attitude gone 3 s into the glide, or the GNSS out for
5 s from 2-6 s in. First the sim's IMU frames had to match a board's. The sim published the rate as (roll, pitch,
yaw) and the glide's accel on one axis. The IMU drivers publish both in the marks frame (x forward, y left, z up),
and the law and the filter read them so (`rate_from_marks`). Now the sim publishes the rate as (roll, -pitch, -yaw)
and the glide's specific force as (sin pitch, 0, n cos pitch). `tools/planb_sitl.py` maps the rate back to plan B's
channel. The airspeed estimator integrates |a| - g, so the accel's change moves it only in a bank.

- **Both stacks on the new frames, 72 + 72 flights:** 72 of 72 pass. Seed for seed the misses agree within 0.7 m
  (wind) and 0.8 m (low noise) median, and GLIDING to the hundredth. The frames moved plan B's own misses by 2.7-3.2 m
  median (the bank's load), so later board runs compare against a reference flown on these frames.
- **The attitude drop** (noise 10 %, no wind, seeds 1-3): the Rust filter carries the glide. F15 lands 28 / 95 / 87 m
  from the centre against 92 / 90 / 89 m without the drop; E16 lands 71 / 75 / 99 m against 84 / 74 / 77 m. Before the
  frames it landed ~500 m off. Plan B's SITL has no reference here: it runs no attitude filter, so after the drop its
  law flies a frozen attitude (219-405 m off).
- **The GNSS drop:** the two stacks agree within 13 m (F15 70-87 m Rust, 77-86 m plan B; E16 95-110 m and 94-107 m).

- **On the board** (the taster, realtime, the firmware whose law reads the marks-frame rate): F15 noise10 lands
  101.2 m from the centre, inside plan B's 20 seeds on the new frames (87.2-103.6 m). The attitude drop lands 92.4 m:
  the board's filter carries the glide as the SITL's does (`HOST_FLAGS="--attitude-drop-s 3"
  tools/hitl_collect_host.sh ...`).

The reboot stays realtime-only: it needs a warm start, which the SITL does not have.

## 4. What it took

- `tools/hitl_matrix_host.sh`: the matrix for a Rust board, the scenario table of `hitl_matrix.sh` (a test keeps the
  three tables equal), the artefacts counted.
- `tools/hitl_matrix_compare.py`: `reference` (plan B's SITL), `sitl` (the Rust SITL), `compare` (any runs against
  the reference's seeds, the family gates).
- `src/avionics/sitl` and the firmware share their plumbing (crates/controller: the topology, `HitlWriters`,
  `StageSources`, `FlightQuantities`). The board refactored onto it flies as before: F15 clean 38.7 m in the zone.
- `tools/hitl_host.py --sitl`: lockstep, seeded, `--record`ed; `tools/sim_link_codec.py` holds both ends of the
  frames (a process under the shims cannot import the hub's transports).
- `tools/recorder_flight.py` anchors a cut on the stage machine's BOOSTING row when no |a| holds 3 g (a HITL boost is
  the sim's ~3 g; under noise six flights of r1 found no boost to cut at).
