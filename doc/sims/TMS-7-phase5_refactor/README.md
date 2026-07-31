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
| `e16_full`  | E16 | 270 g | **88.8 m** ✗ | 90.1 m ✗ | −1.3 m |
| `e16_light` | E16 | 215 g | **45.7 m** ✗ | 44.2 m ✗ | +1.5 m |
| `f15_full`  | F15 | 270 g | **121.7 m** ✗ | 119.2 m ✗ | +2.5 m |
| `f15_light` | F15 | 215 g | **83.6 m** ✗ | 110.0 m ✗ | **−26.4 m** |

Run-to-run spread across three flights of this matrix is ±3 m (the sim's 5 % sensor noise), so treat
these as ±3, not as exact.

🎬 **[`phase5_refactor.mp4`](phase5_refactor.mp4)** — follow-cam of all four, FHD 30 FPS.

[e16_full](e16_full.html) · [e16_light](e16_light.html) · [f15_full](f15_full.html) ·
[f15_light](f15_light.html) · [**overlay plan**](plan_board.svg)

The board-health panel of each report now carries a **`rescues` staircase right after `mem MB`** —
every step is a ~200 ms emergency collect, so the teeth in the memory sawtooth have a labelled cause.

**Three of four land within 3 m of the host prediction.** That is the headline: after a refactor round
that touched the bus layer, the deploy pipeline, the trig primitive and the airspeed source, the board
still reproduces the host matrix. `f15_light` is the one real move, 23 m better.

## Against the previous BOARD runs (f15_full, the only case with board provenance before)

| metric | **this study** | physics_refresh (3 runs) | verdict |
|---|---|---|---|
| touchdown | **121.7 m** | 121.2 / 121.5 / 121.7 m | inside the old spread |
| fin moves | **1508** | 1523–1593 | ~1 % below the old floor |
| fin travel | **3696°** (64 °/s) | 3742–4068° (65–70 °/s) | ~1 % below the old floor |
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

## Memory: the leak IS measurable, and it was hiding two defects

The first cut of this study said the leak could not be measured from a capture. That was wrong — the
window I used straddled a collect. Measured properly, on the longest monotonic decline:

**331 KB/s**, and the board's own estimator converges to **~350 KB/s** across all four cases. Not the
250 KB/s `physics_refresh` recorded. On a ~30 MB heap that is **~85 s to exhaustion**, against flights
of 33–66 s.

Investigating it surfaced two real defects, both now fixed.

### 1. The OOM forecast was unusable — and it arms a control-loop pause

`oom_s` feeds `board_health._rescue()`, which spends a **~200 ms `gc.collect()` mid-glide**. Successive
readings on the first run were **271, 155, 119, 109, None, 362, None, 206 s**. Two causes: a single
sample where the heap grew zeroed the estimate outright (`if slope > 0 else 0`), and one 1 Hz
difference *was* the slope, so sampler jitter went straight to the output.

Replaced with the cumulative average since the last collect — with GC off the heap only shrinks, so
accumulate the KB lost and the interval count and divide. It converges instead of oscillating, one
noisy sample moves it by 1/n, and a collect is the natural reset. The forecast now targets a **512 KB
reserve** rather than zero, since the last few hundred KB are too fragmented to serve a real
allocation. Measured on the re-flown matrix:

| case | leak KB/s | oom_s | max successive change |
|---|---|---|---|
| `e16_full`  | 299–370 | 60–100 s | 16 s |
| `e16_light` | 293–359 | 63–103 s | 14 s |
| `f15_full`  | 286–355 | 55–105 s | 13 s |
| `f15_light` | 290–357 | 52–104 s | 13 s |

A smooth monotonic countdown — **mean successive change 1.5 s** on `f15_full`, and the remaining
larger steps are all at collect boundaries, where the countdown *should* jump because memory was just
reclaimed.

### 2. A capture could not show that the rescue had fired — and it was firing too often

`rescues` lived in `inspect()` only, so a 200 ms control-loop pause left no trace in the record. It is
in `health.csv` now, together with `leak_kbps` — and flight_report draws it as a **staircase right
after `mem MB`**, so the teeth in the memory sawtooth have a labelled cause.

The first thing it showed is that the rescue was firing on a **third redundant safety margin**. The
trigger was `oom_s < 2 × land_s` — collect when memory dies within *twice* the remaining flight —
stacked on two margins that already point the same way:

- `oom_s` counts down to a **512 KB reserve**, not to zero, so it already reports exhaustion early;
- the safe-altitude floor prices the pause at `_RESCUE_PAUSE_MS` = 200 ms against a collect
  **measured in flight at 34–45 ms** — about 5×. (That figure only became readable once the rescue
  line moved from `log()` to durable telemetry; the earlier ~67 ms came from a bench benchmark.)

A third 2× on top did not buy safety, it spent control slices. Now `oom_s <= land_s`: rescue when
memory dies **before** the glider lands. Measured, same matrix, before and after:

| case | flight | rescues @ 2× | **rescues @ 1×** | min free heap |
|---|---|---|---|---|
| `e16_full`  | 33 s | 1 | **0** | 22.5 MB |
| `e16_light` | 38 s | 2 | **1** | 22.0 MB |
| `f15_full`  | 57 s | 3 | **2** | 18.6 MB |
| `f15_light` | 66 s | 4 | **2** | 16.0 MB |

**Ten pauses across the matrix became five**, and the margin is plainly intact — the heap never fell
below 16 MB of ~30 MB, nowhere near the reserve. `e16_full` now flies its whole 33 s without a single
unscheduled pause. The asymmetry still favours firing (an OOM mid-glide is the crash→neutral →
watchdog → reset chain), which is why the comparison is `<=` rather than a fraction of `land_s`.

**Caveat on the leak number.** This is HITL, and the sim allocates too. A previous flown A/B put the
sim at ~46 % of a HITL leak, which would put a real flight near ~190 KB/s and OOM ~160 s. That ratio
has not been re-measured on this build — `tools/oom_soak.py` is the instrument for the production
number.

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
