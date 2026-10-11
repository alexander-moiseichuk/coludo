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
