# TMS-7 guarded fins — on-board HITL matrix (F15-4 + E16-4)

The full TMS-7-basic experiment matrix, but flown **on the real ESP32-P4** (not the host) and for **both
motors we fly** — F15-4 and E16-4. Each run is a complete on-board HITL flight (`config_hitl`: the real
sensor drivers off, the `hitl` task feeding the *real* `sequencer` + `flight` + `pid` + `mixer` +
`navigation` a simulated 6-DoF body), streamed to the Luckfox recorder and pulled with `adb`. So unlike
the host sweep this replaced, every trace here — trajectory, fins, **and board vitals** — is real device
telemetry. `../TMS-7-basic` remains the host-sim reference (bank-to-turn glide, F15 only).

**Site (HPRC):** pad `25.514379, -80.391795`; zone TL `25.514944,-80.392972` / BR `25.514583,-80.391111`
— a ~40 m (N–S) × ~187 m (E–W) strip, centre ~49 m from the pad.

🎬 **[`TMS-7-guarded_fins_simulation.mp4`](TMS-7-guarded_fins_simulation.mp4)** — a narrated FHD animation of
the 5 %-noise / calm runs for **E16-4 then F15-4**: top-down field with the landing zone + corner trees,
the glider tracking its real trajectory, a live telemetry panel, and prompter captions
(ignition → climb → apogee/eject → glide → touchdown). Rendered by `tools/flight_video.py`.

> **Refreshed for TMS-7 v2:** the whole matrix below — both engines, all 12 scenarios — was **re-flown on
> the board** with the measured TMS-7 v2 stack masses, and the video re-rendered with the improved renderer
> (follow-cam, wings stowing under/aft then sweeping out, taller fin, wider field).

## TMS-7 v1 → v2 — flight capabilities

The printed v2 airframe was measured (`models/TMS-7/readme.md`); it differs from the v1 estimate the
original `coludo.md` envelope assumed:

| | TMS-7 v1 (estimate) | TMS-7 v2 (measured) | effect |
|---|---|---|---|
| Glider structure | ~113 g | **150.4 g** | heavier |
| Liftoff stack (100 g elec) | ~430 g | **E16 451 g / F15 468 g** | heavier |
| Wing area (total) | 124 cm² | **296 cm²** (AR 12.1) | 2.4× wing |
| Wing loading | ~19 kg/m² | **8.3 kg/m²** | halved |
| Stall speed | ~19 m/s | **~14 m/s** | slower, safer |
| Best-glide speed | — | **~17 m/s** | gentle cruise |
| Glide ratio L/D | — | **5–10 (mid ~6)** | a real glide |
| Apogee (E16 / F15) | ~180 / ~360 m | **~135 / ~291 m** | lower |

**Net:** v2 **climbs less high** (heavier stack, more drag) but **glides far better** — the 2.4× wing halves
the wing loading, drops the stall from ~19 to ~14 m/s, and gives a controllable ~17 m/s glide instead of
always riding the stall edge. That is the right trade for proving active control: less peak altitude, much
more flyability. In the HITL matrix this shows as **lower apogees** (shorter E16 flights) with the **same
control behaviour** — F15 still lands on the strip, E16's short burn keeps it hit-or-miss. Only the boost
masses feed the sim (`config_hitl` liftoff_g per motor); the glide model is unchanged, so the matrix is a
clean before/after on the boost energy the controller is handed.

## How it was collected

- A clean board reboot (`tools/board_reboot.py`) then **`mpremote run`** flies a one-line launcher
  (`tools/hitl_collect.sh`; **boardrun is retired**): the runner brings up `config_hitl`, **arms** the
  controller, and flies to `DONE`. The Recorder streams over UART:1 to the Luckfox
  (`/userdata/recordings/<session>_<file>.csv`); **adb** pulls the session, which is re-assembled into the
  recorder wire-format capture the report tools read.
- The `hitl` task now **records the simulated sensors as telemetry** (accel/imu/baro/gnss/laser + a
  combined fins, same CSV names/fields as the real drivers), so an on-board run yields a *complete*,
  renderable capture — `health` is the real `board_health`, the fins are the real commanded servo angles.
- Three HITL fidelity fixes made the on-board glide faithful (real-flight sequencer untouched): a
  **fixed-timestep wall-clock accumulator** (sim-time tracks the wall clock the sequencer's timeouts
  use), a **burn + ejection-delay** boost→glide timeout (the `-4`: F15-4 7.45 s, E16-4 5.77 s, ≈ apogee),
  and **arming** the controller (else the fins hold neutral → no bank → no descent).

## F15-4

*Miss* = touchdown distance to the zone centre; *max from pad* = furthest downrange; *dur* = flight time.

| Experiment 1 — noise (no wind) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|--------:|---------:|:-------:|-----------------:|--------:|
|     5 % |       34 | **yes** |              181 |    41.4 |
|    10 % |       39 | **yes** |              182 |    41.2 |
|    25 % |    n/a\* |    —    |               —  |     —   |
|    50 % |      119 |   no    |              186 |    54.7 |
|   100 % |      647 |   no    |              624 |    55.7 |

| Experiment 2 — wind (10 % noise) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|-----------:|---------:|:-------:|-----------------:|--------:|
|       calm |       34 | **yes** |              179 |    41.3 |
|        3   |       65 |   no    |              160 |    42.5 |
|        6   |      115 |   no    |              131 |    43.6 |
|        9   |       78 | **yes** |              156 |    43.6 |
|       12   |       80 | **yes** |              105 |    46.9 |

**Finding:** with clean-ish sensors the real on-board loop lands **on the strip** — in zone at 5–10 % noise
and in calm wind (miss ~34 m to the centre of the ~187 m strip), far tighter than the host bank-to-turn
baseline (~85 m). Wind drifts it ~65–115 m across the zone but it still settles in zone at 9–12 m/s. It
degrades at ≥50 % noise (the noisy attitude corrupts the bank loop → over-range to ~120–650 m), and those
high-noise runs also fail to *settle*: the "stationary ~1 g" landing detect can't sustain through 50–100 %
accel noise, so they run the 95 s cap (dur ~55 s) still in LANDING. \*The 25 % capture was anomalous this
re-fly (a frozen-GNSS round in the chaotic regime) and is omitted; the breakdown is bracketed by 10 % and 50 %.

## E16-4

| Experiment 1 — noise (no wind) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|--------:|---------:|:-------:|-----------------:|--------:|
|     5 % |      227 |   no    |              206 |    27.5 |
|    10 % |      220 |   no    |              200 |    27.4 |
|    25 % |    n/a\* |    —    |               —  |     —   |
|    50 % |      546 |   no    |              523 |    53.7 |
|   100 % |      643 |   no    |              620 |    54.9 |

| Experiment 2 — wind (10 % noise) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|-----------:|---------:|:-------:|-----------------:|--------:|
|       calm |      226 |   no    |              205 |    27.4 |
|        3   |      189 |   no    |              170 |    27.6 |
|        6   |      119 |   no    |              128 |    28.4 |
|        9   |       67 | **yes** |               92 |    29.2 |
|       12   |       56 |   no    |               14 |  10.8† |

**Finding:** E16-4's shorter burn → lower apogee (~135 m, ~27 s flights vs F15's ~291 m / ~41 s) → **less
glide time to null the miss**, so the spread is wider and noisier than F15-4 — clean is **hit-or-miss**
(~220 m out, just shy of the strip), and a fair tailwind actually *helps* (wind 9 m/s lands in zone). Same
≥50 % noise breakdown (over-range ~550–650 m) and landing-detect timeout as F15-4. \*25 % omitted (the
re-fly's capture was incomplete in the chaotic regime). †wind12 was an early-DONE: a noise coincidence
trips the stationary-1 g detect mid-glide (dur 10.8 s, never ranged out).

## Corner cases (spike injection)

`--spike` injects a transient 2× attitude/accel glitch every ~3 s (`report_corner_*` per engine):

- **corner_spike** (10 % noise + spikes) — the loop rejects glitches; F15-4 still lands in zone
  (miss 40 m), E16-4 holds ~223 m (its usual hit-or-miss).
- **corner_stress** (50 % noise + 12 m/s wind + spikes) — everything-degraded; both run the cap in
  LANDING (F15-4 202 m, E16-4 211 m from the centre, contained ≤ ~205 m).

Open `report_corner_*.html` (plotly) and watch the **fins** + **attitude** panels through the boost +
glide; the boost-phase elevon activity is the guarded-fins behaviour.

## Real board vitals

Every `report_*.html` health panel is **real** (`board_health`, not the host's synthetic model): the P4
runs **31–33 °C** and the control loop loads it **0–~50 %** cruising with a peak at the landing work — and
`mem_free` swings (down through the flight, recovered at touchdown) show the **GC-off-in-flight**
policy in real data, where the synthetic model assumed a flat ~4 MB.

## Files

Per engine (`f15-4/`, `e16-4/`):
- `report_*.svg` — dependency-free per-flight look (plan view + altitude/roll), all 12 rounds
- `report_{corner_spike,corner_stress,noise05,noise50,wind00}.html` — interactive plotly reports
  (3D trajectory + linked accel/altitude/speed/attitude/**fins**/**health**/agl panels)
- `compare_noise.svg`, `compare_wind.svg` — sweep overlays (all tracks on one plan view)

The raw per-flight captures are not kept (the reports carry them); regenerate any round on the board with
`tools/hitl_matrix.sh <F15|E16>` (board_reboot → `mpremote run` → adb-take → assemble → render; boardrun retired).
