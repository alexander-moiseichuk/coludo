# TMS-7 virtual-flight experiments

Closed-loop flight simulations of the TMS-7 airframe on an **F15** motor from the **HPRC** pad, produced
by `tools/virtual_flight.py`. Each run flies the *real* control code (`navigation` + `pid` + `mixer`)
under the *real* config (`config_hitl`) over the shared flight model (`src/glider/sim_model.py`) — the
same model and the same loop the board runs in HITL (`tasks/hitl.py`), just in CPython so it produces a
recorder capture without a flight.

**Site (HPRC, Google Maps, north up):** pad `25.514379, -80.391795`; landing zone TL `25.514944,
-80.392972` / BR `25.514583, -80.391111` — a **~40 m (N–S) × ~187 m (E–W)** strip (~7500 m²), centre
`25.514764, -80.392042`, ~49 m from the pad. Long axis E–W, so `navigation.zone()` gates the short
(E/W) ends and the glider runs in along the strip.

## Render / regenerate

```sh
# a capture (stdout or -o file); --noise degrades the sensors, --wind adds a steady cross-wind
python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o vf_noise05.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.10 --wind 12 --wind-dir 210 -o vf_wind12.txt

# interactive HTML report (3D trajectory + linked time-series); needs plotly.
# the committed report_*.html use --cdn (tiny files, load plotly.js from the CDN -> need internet);
# drop --cdn for a self-contained ~4.5 MB file that opens offline.
python3 tools/flight_report.py vf_noise05.txt -o report_noise05.html --cdn

# dependency-free SVG (plan view + altitude/roll), or an overlay of a whole sweep
python3 tools/flight_svg.py vf_noise05.txt -o report_noise05.svg \
    --pad 25.514379,-80.391795 --zone 25.514944,-80.392972,25.514583,-80.391111
```

## What each interactive `report_*.html` shows

A **3D trajectory** (GNSS ground-track × baro height, coloured by time) — hover/click any point to read
`t / height / speed / heading` — plus six **linked time-series** panels with the stage transitions
(boosting → gliding → landing) drawn as dashed markers. Hover (or click) any instant and **every panel
reads out together** (`hovermode: x unified`):

1. **|accel| (g)** — the boost spike and the glide load factor
2. **altitude / elevation (m)**
3. **speed (m/s)** — GPS ground speed
4. **attitude (deg)** — heading, roll, pitch
5. **fins — commanded (deg)** — `eleron_left`, `eleron_right`, `yaw`: *how the glider actually works the
   surfaces* to bank, turn and reject the disturbances (differential elerons = roll/bank, common = pitch)
6. **board health** — `load %`, `temp °C`, `mem MB` (`board_health.csv`): track CPU load, free memory and
   MCU temperature through the flight. *In these sims the health row is synthetic and phase-modeled* (the
   host has no MCU) — shaped like the board would read (load highest under boost sampling and on the
   landing laser-hammer, temperature drifting up, memory on a GC sawtooth). On a real board capture this
   panel shows the measured vitals.
7. **agl (m)** — the laser, only within range near the ground

(The `report_*.svg` files are a quick dependency-free look: plan-view ground track + altitude/roll.)

## Experiment 1 — sensor noise (no wind)

`--noise N` perturbs every accel/attitude/altitude/agl reading by ±N·|value| (the same `sim_model.noisy`
the board uses); GNSS position is left clean (board parity). *Miss* = touchdown distance to the zone
centre; *max from pad* = how far downrange it ever gets.

| Noise N | Miss (m) | In zone | Max from pad (m) | Duration (s) |
|--------:|---------:|:-------:|-----------------:|-------------:|
|     5 % |       88 |   no    |              178 |         34.6 |
|    10 % |       86 |   no    |              180 |         35.5 |
|    25 % |       83 |   no    |              194 |         41.8 |
|    50 % |      190 |   no    |              224 |         44.4 |
|   100 % |      447 |   no    |              416 |         47.8 |

**Finding:** with bank-to-turn the glider **orbits the zone and stays contained** (≤ ~190 m even at
50 %), and the miss now **tracks sensor quality** — ~85 m at 5–25 %, then it degrades as the noisy
attitude corrupts the bank loop, and at 100 % the orbit breaks and it over-ranges again (447 m). So
sensor quality *does* matter here, but only once it is bad enough to destabilise the turn (≳50 %).

## Experiment 2 — cross-wind (10 % noise)

A steady wind toward 210° (across the glide), 0 → 12 m/s vs the ~14 m/s trim airspeed:

| Wind (m/s) | Miss (m) | In zone | Max from pad (m) | Duration (s) |
|-----------:|---------:|:-------:|-----------------:|-------------:|
|       calm |       88 |   no    |              180 |         35.4 |
|        3   |      115 |   no    |              157 |         35.7 |
|        6   |      138 |   no    |              133 |         37.3 |
|        9   |      144 |   no    |              179 |         39.0 |
|       12   |       19 |  **yes**|               63 |         42.4 |

**Finding:** the orbit keeps it contained in wind too (never past ~180 m), and the touchdown drifts only
~50 m across 0 → 9 m/s. At 12 m/s the wind happens to carry it onto the strip (lands **in**); an equal
wind the other way would carry it off by as much — wind sets *where* on/around the orbit it comes down.

## The fix — bank-to-turn (energy management)

Earlier runs steered on the **rudder while holding wings level**: a flat, weak turn that let the airframe
**over-range** the zone, sailing ~456 m downrange before it could come back. The control now **banks to
turn** (`navigation.bank_demand`: GLIDING roll setpoint = `nav_bank_gain · heading_error`, capped at
`bank_limit`), so the turn is tight (~v²/(g·tan φ) radius) and the per-tick re-steer becomes an **orbit
that bleeds altitude over the target** instead of past it. Net effect above: max-from-pad dropped from
~456 m to ≤ ~224 m and the calm-air miss from ~456 m to ~85 m. The fin panel in the HTML shows it
working — the elerons swinging differentially to hold the bank around each orbit.

**What's left:** at good sensor quality it lands ~85 m from the centre — just *off* the 40 m-wide strip,
because LANDING rolls wings-level and lets it drift off on short final. Nailing the strip wants a tighter
final-approach / flare (or a smaller orbit radius); and the high-noise (≥50 %) robustness of the bank
loop is the other open thread.

## Corner cases — spike injection (g16)

`--spike` injects a **transient 2× glitch** on the attitude + accel for *one tick* every ~3 s
(deterministic, so the stored traces reproduce exactly) — a sudden bad sensor sample. The same `spike`
flag exists in the on-board HITL config (`config_hitl`), so these are repeatable inputs for either the
host sim or an on-board replay. Two corner cases are stored in full and committed as references:

- **`vf_corner_spike.txt`** / `report_corner_spike.{html,svg}` — F15, 10 % noise, spikes. Shows the loop
  **rejecting** glitches: at each spike tick the elevons kick to the limit for one frame (e.g. roll
  −13°→−26°, elevons (72,120)→(135,56)) then recover the next frame — the trajectory barely moves.
- **`vf_corner_stress.txt`** / `report_corner_stress.{html,svg}` — F15, **50 % noise + 12 m/s wind +
  spikes** together, the everything-degraded case: how the controller copes when noise, wind and
  glitches all stack up.

Open the HTML reports and watch the **fins** + **attitude** panels (unified hover) to see each glitch and
the loop's response.

```sh
python3 tools/virtual_flight.py --motor F15 --noise 0.10 --spike -o vf_corner_spike.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.50 --wind 12 --wind-dir 210 --spike -o vf_corner_stress.txt
```

## Files

- `vf_noise{05,10,25,50,100}.txt`, `vf_wind{00,03,06,09,12}.txt` — recorder captures (all rounds)
- `vf_corner_{spike,stress}.txt` + `report_corner_{spike,stress}.{html,svg}` — the two g16 corner cases (committed in full)
- `report_{noise05,noise50,wind00}.html` — **three committed interactive reports** kept as reference
  points (clean sensors / degraded sensors / calm baseline); the full set regenerates for any round with
  the `flight_report.py` command above — the other rounds are summarised in the tables and `compare_*.svg`
- `report_*.svg` — dependency-free per-flight look (plan view + altitude/roll), all rounds
- `compare_noise.svg`, `compare_wind.svg` — sweep overlays (all tracks on one plan view)
