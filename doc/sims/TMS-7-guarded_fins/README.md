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

> **Refreshed (TMS-7 v2):** the video above is re-rendered from fresh on-board 5 %/calm re-flies using the
> measured TMS-7 v2 stack masses (E16 451 g / F15 468 g → apogee ~135 m / ~291 m, see `models/TMS-7/readme.md`)
> and the improved renderer (follow-cam, wings stowing under/aft then sweeping out, taller fin, wider field).
> The matrix tables below are the prior sweep (qualitatively unchanged — the mass shift only lowers apogee
> a little); a full-matrix re-fly with the v2 masses is the next step.

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
|     5 % |        1 | **yes** |              181 |    43.4 |
|    10 % |        5 | **yes** |              181 |    43.5 |
|    25 % |      548 |   no    |              574 |    83.7 |
|    50 % |      335 |   no    |              358 |    80.4 |
|   100 % |      662 |   no    |              613 |    78.7 |

| Experiment 2 — wind (10 % noise) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|-----------:|---------:|:-------:|-----------------:|--------:|
|       calm |        4 | **yes** |              181 |    43.5 |
|        3   |       42 |   no    |              160 |    43.5 |
|        6   |      121 |   no    |              131 |    44.6 |
|        9   |       55 |   no    |               26 |    12.5 |
|       12   |      107 |   no    |              135 |    49.6 |

**Finding:** with clean-ish sensors the real on-board loop lands **on the strip** — miss 1–5 m at 5–10 %
noise and in calm wind, far tighter than the host bank-to-turn baseline (~85 m). Wind drifts it ~40–120 m
across the zone. It degrades at ≥25 % noise (the noisy attitude corrupts the bank loop → over-range to
~550–660 m) — and those high-noise runs also fail to *settle*: the "stationary ~1 g" landing detect can't
sustain through 50–100 % accel noise, so they run the 95 s cap (dur ~80 s) still in LANDING rather than
reaching DONE.

## E16-4

| Experiment 1 — noise (no wind) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|--------:|---------:|:-------:|-----------------:|--------:|
|     5 % |      228 |   no    |              203 |    28.5 |
|    10 % |       70 | **yes** |               66 |    10.8 |
|    25 % |      817 |   no    |              793 |    75.3 |
|    50 % |     1016 |   no    |              992 |    79.4 |
|   100 % |      673 |   no    |              625 |    78.7 |

| Experiment 2 — wind (10 % noise) | Miss (m) | In zone | Max from pad (m) | Dur (s) |
|-----------:|---------:|:-------:|-----------------:|--------:|
|       calm |      220 |   no    |              194 |    28.6 |
|        3   |      198 |   no    |              177 |    28.7 |
|        6   |      125 |   no    |              130 |    29.4 |
|        9   |       80 | **yes** |              106 |    30.0 |
|       12   |       94 |   no    |               66 |    31.1 |

**Finding:** E16-4's shorter burn → lower apogee (~28–31 s flights vs F15's ~43 s) → **less glide time to
null the miss**, so the spread is wider and noisier than F15-4 (clean still reaches the strip, but it is
hit-or-miss). Same ≥25 % noise breakdown and landing-detect timeout as F15-4. The short anomalous runs
(e.g. noise10 10.8 s) are early-DONE: a noise coincidence trips the stationary-1 g detect mid-glide.

## Corner cases (spike injection)

`--spike` injects a transient 2× attitude/accel glitch every ~3 s (`report_corner_*` per engine):

- **corner_spike** (10 % noise + spikes) — the loop rejects glitches; F15-4 still lands in zone
  (miss 3 m), E16-4 holds ~227 m.
- **corner_stress** (50 % noise + 12 m/s wind + spikes) — everything-degraded; both run the cap in
  LANDING (F15-4 287 m, E16-4 285 m from the centre, contained ≤ ~255 m).

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
`tools/hitl_matrix.sh <F15|E16>` (rshell-run → adb-take → assemble → render).
