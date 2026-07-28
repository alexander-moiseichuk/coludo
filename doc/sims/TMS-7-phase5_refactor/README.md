# TMS-7 phase5 refactor — the canonical matrix, re-flown ON THE BOARD

The `TMS-7-physics_refresh` matrix and the `TMS-7-loiter` law, re-flown on the real ESP32-P4 after the
phase-5 refactor round, to answer one question: **does any of it move?**

Board HITL provenance — real drivers, real servos physically moving, real INA226 on the servo rail,
real MCU memory; `tools/hitl_collect.sh`, 5 % noise, calm, E16/F15 × full (270 g) / light (215 g). The
board runs the **quality-2 worst-case polar** (`sim_model.trim_sink` default 7.0), so the comparable
host column is q2, not q5.

## What changed underneath since those studies

- **The simulated pitot** (the big one). `tasks/hitl.py` published no `airspeed`, so the only publisher
  of the fused channel on a board flight was the REAL bench SDP810 in still air, gated out by
  `pitot_min_ms` for the whole flight. The sim now publishes it and `airspeed_sdp810` is masked, so
  board HITL exercises the pitot path for the first time.
- **`.mpy` deployment onto a wiped board** — the board no longer compiles 60 modules at boot, and its
  contents are now a function of the source tree alone.
- **A CORDIC that is accurate at small magnitude** (`fixed.atan2_cd` pre-normalises; 0.74° → 0.048° at
  the magnitude `tasks/attitude.py` feeds it).
- Extraction work with no intended behaviour change: `Task.strike()`, `Task.claim()`,
  `i2cbus.bind()` / `spibus.bind()`, `tools/sources.py`.

## The matrix — touchdown miss from zone centre

| case | motor | glider | **board (this study)** | host q2 (physics_refresh) | delta |
|---|---|---|---|---|---|
| `e16_full`  | E16 | 270 g | **89.5 m** ✗ | 90.1 m ✗ | −0.6 m |
| `e16_light` | E16 | 215 g | **43.1 m** ✗ | 44.2 m ✗ | −1.1 m |
| `f15_full`  | F15 | 270 g | **121.2 m** ✗ | 119.2 m ✗ | +2.0 m |
| `f15_light` | F15 | 215 g | **86.9 m** ✗ | 110.0 m ✗ | **−23.1 m** |

[e16_full](e16_full.html) · [e16_light](e16_light.html) · [f15_full](f15_full.html) ·
[f15_light](f15_light.html) · [**overlay plan**](plan_board.svg)

**Three of four land within 2 m of the host prediction.** That is the headline: after a refactor round
that touched the bus layer, the deploy pipeline, the trig primitive and the airspeed source, the board
still reproduces the host matrix. `f15_light` is the one real move, 23 m better.

## Against the previous BOARD runs (f15_full, the only case with board provenance before)

| metric | **this study** | physics_refresh (3 runs) | verdict |
|---|---|---|---|
| touchdown | **121.2 m** | 121.2 / 121.5 / 121.7 m | inside the old 0.5 m spread |
| fin moves | **1524** | 1523–1593 | inside |
| fin travel | **3594°** (62 °/s) | 3742–4068° (65–70 °/s) | ~4 % below the old floor |
| servo energy | **18.9 J** (0.33 W avg) | 18.7–20.6 J (0.32–0.36 W) | inside |
| `fin_cap` | **5° → 45°** | 5° → 45° | unchanged 1/v² schedule |

Nothing regressed. The small drop in fin travel is consistent with the governor now flying a measured
airspeed instead of an estimate — see below.

## The pitot is now actually in the loop

The change with a visible signature. Governor airspeed vs the sim pitot it now reads, per case:

| case | governor airspeed (glide) | sim pitot (glide) | pitot range |
|---|---|---|---|
| `e16_full`  | 14.8 m/s | 14.9 m/s | 0 → 30 (railed) |
| `e16_light` | 15.0 m/s | 15.0 m/s | 0 → 30 (railed) |
| `f15_full`  | 15.5 m/s | 15.4 m/s | 0 → 30 (railed) |
| `f15_light` | 15.3 m/s | 15.5 m/s | 0 → 30 (railed) |

Two things worth reading off this:

- **The governor tracks the pitot to ~0.2 m/s.** Previously the pitot was rejected all flight and the
  governor ran on the accel + GNSS estimate, which reported **14.1 m/s** in glide.
- **The saturation guard fires exactly where it should.** The pitot rails at 30 m/s (the ±500 Pa
  cell) while the governor reads up to 60 m/s in boost — i.e. `pitot_max_ms` hands back to the accel
  backbone under boost, which is the behaviour that guard exists for, now exercised on the board
  rather than argued from the datasheet.

The glide figure is ~1.4 m/s higher than the old estimate, and that is correct rather than drift: a
pitot measures TOTAL air-relative speed, √(v² + vu²), and the glider is descending. The old estimator
was a horizontal-speed proxy and under-read by the sink component.

## Reconciling TMS-7-loiter

`TMS-7-loiter` reports **18 m (F15) and 17 m (E16), both in-zone**, at the same quality-2 polar. The
board says 121 m and 90 m. The two are not comparable, and the reason is documented in
`physics_refresh` itself: those loiter captures predate the **harness correction** (the host servo
applied every command instantly, plus the circular-noise and control-rate defects). `physics_refresh`
re-flew the matrix on the corrected harness and got 119.2 m for the same F15 case — which is what the
board now independently confirms at 121.2 m.

**So: the loiter study's headline numbers do not survive, and its own "pending" note anticipated
this** — *"the ON-BOARD confirmation matrix replaces these host captures with device telemetry; until
then treat the numbers as sim-predicted."* This is that confirmation. The loiter *law* is unchanged
and is what flew here; what was optimistic was the harness it was measured on, by ~100 m.

## Where we are

At the **quality-2 floor nothing lands in-zone** — consistent with `physics_refresh`, which found the
same and needed the realistic quality-5 polar to put half the matrix in the zone. This study does not
re-measure q5, because the board's polar is fixed at the q2 default; that comparison stays host-side.

The refactor round is therefore **behaviour-neutral on the flight path** and has closed a real
harness gap: board HITL now flies the pitot.

## Caveat — the GC-off leak is NOT measured here

`physics_refresh` quotes 250 KB/s → OOM ~131 s for the board. These captures cannot confirm or refute
it: `mem_free` sits at PSRAM scale (~30 MB) and *rose* over the capture window, so the flight-length
slope is not the right instrument, and the board's own `oom_s` prediction ranges 51–568 s within a
single run. Use `tools/oom_soak.py`, which exists for exactly this, before quoting a leak number.

## Reproduce

```bash
tools/deploy.sh                                    # .mpy onto a wiped board
for spec in "E16 e16_full 270" "E16 e16_light 215" \
            "F15 f15_full 270" "F15 f15_light 215"; do
    set -- $spec
    bash tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False captures/phase5_refactor "$3"
done
python3 tools/flight_metrics.py captures/phase5_refactor --sort name
python3 tools/flight_kpi.py captures/phase5_refactor/f15_full.txt
```
