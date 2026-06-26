# TMS-7 virtual-flight — guarded fins (g12)

Closed-loop flight simulations of the TMS-7 airframe on an **F15** motor from the **HPRC** pad, the same
set as [`../TMS-7-basic`](../TMS-7-basic/) but flying the **g12 dynamic-pressure fin governor + boost
control stage**. Each run flies the *real* control code (`navigation` + `pid` + `mixer`) under the *real*
config (`config_hitl`) over the shared flight model (`src/glider/sim_model.py`) — the same model and loop
the board runs in HITL (`tasks/hitl.py`), just in CPython, so it emits a recorder capture **without a
flight or a board**. `TMS-7-basic` is the prior behaviour (bank-to-turn glide only, boost flown
open-loop) kept for side-by-side comparison; this directory mirrors it file-for-file.

**Site (HPRC, Google Maps, north up):** pad `25.514379, -80.391795`; landing zone TL `25.514944,
-80.392972` / BR `25.514583, -80.391111` — a **~40 m (N–S) × ~187 m (E–W)** strip (~7500 m²), centre
`25.514764, -80.392042`, ~49 m from the pad. Long axis E–W, so `navigation.zone()` gates the short
(E/W) ends and the glider runs in along the strip.

## What's new vs TMS-7-basic

1. **Boost-phase attitude hold.** `BOOSTING` is now a control stage. Leaving the rod (airspeed >
   `boost_engage_speed` = 15 m/s) the loop captures the rod-vertical attitude and holds it — the fins
   **fight the crosswind weathercock** that would otherwise tilt the stack during the climb. On the rod
   the 3-point mount holds it and there is no `q` to bite, so the fins stay neutral until then. In
   `TMS-7-basic` the boost was a 1-DoF vertical climb with the fins parked at neutral.
2. **Dynamic-pressure governor.** Aero torque scales with `q ∝ v²`, so the max fin deflection is
   scheduled `∝ 1/v²`, clamped to `[5°, 45°]` (× the `fin_limit_multiplier` safety dial) — full authority
   slow, ±5° near burnout. The cap rides the **whole** flight (boost, glide, landing). See
   `specs/coludo.md` → "Fin authority".

The headline is in the **`fins.csv` boost rows**: here the elevons are non-neutral through the entire
post-rod climb (differential for roll, common for pitch) holding the stack vertical against the wind,
where the basic set's boost rows are flat `90;90;90`. The *glide* outcomes below track the basic set
closely — both glide on the same bank-to-turn loop, so the governor/boost changes are visible in the
climb and the fin-authority cap, not in where it lands.

## Render / regenerate

The guarded governor + boost stage are now the **default** control path in `virtual_flight.py`, so the
commands are plain — no special flag:

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
4. **attitude (deg)** — heading, roll, pitch (watch it stay **nearer vertical through boost** here)
5. **fins — commanded (deg)** — `eleron_left`, `eleron_right`, `yaw`: the guarded headline — the elevons
   are **working through boost** (differential = roll/bank, common = pitch), not just in the glide
6. **board health** — `load %`, `temp °C`, `mem MB` (`board_health.csv`). *In these host sims the health row is
   synthetic and phase-modeled* (the host has no MCU) — shaped like the board would read. **For the real
   thing see [`device/`](device/)**: load/temp/mem pulled off the ESP32-P4 over a real on-board HITL flight
   (the board runs much cooler / emptier / more idle than this model assumed).
7. **agl (m)** — the laser, only within range near the ground

(The `report_*.svg` files are a quick dependency-free look: plan-view ground track + altitude/roll.)

## Experiment 1 — sensor noise (no wind)

`--noise N` perturbs every accel/attitude/altitude/agl reading by ±N·|value| (the same `sim_model.noisy`
the board uses); GNSS position is left clean (board parity). *Miss* = touchdown distance to the zone
centre; *max from pad* = how far downrange it ever gets.

| Noise N | Miss (m) | In zone | Max from pad (m) | Duration (s) |
|--------:|---------:|:-------:|-----------------:|-------------:|
|     5 % |       87 |   no    |              178 |         34.9 |
|    10 % |       81 |   no    |              180 |         36.2 |
|    25 % |      108 |   no    |              192 |         38.8 |
|    50 % |      196 |   no    |              219 |         43.0 |
|   100 % |      427 |   no    |              396 |         46.5 |

**Finding:** the same as basic — the bank-to-turn glide **orbits the zone and stays contained** (≤ ~220 m
even at 50 %), the miss tracks sensor quality (~80–110 m at 5–25 %), and only at 100 % does the orbit
break and over-range (427 m). The governor caps fin authority but does not destabilise the turn; the
guarded numbers sit within a few metres of the basic set, confirming the fin-limit schedule does not cost
glide control.

## Experiment 2 — cross-wind (10 % noise)

A steady wind toward 210° (across the glide), 0 → 12 m/s vs the ~14 m/s trim airspeed:

| Wind (m/s) | Miss (m) | In zone | Max from pad (m) | Duration (s) |
|-----------:|---------:|:-------:|-----------------:|-------------:|
|       calm |       80 |   no    |              180 |         36.1 |
|        3   |      119 |   no    |              159 |         36.3 |
|        6   |      146 |   no    |              132 |         37.6 |
|        9   |      153 |   no    |              187 |         39.8 |
|       12   |       17 |  **yes**|               62 |         42.3 |

**Finding:** the orbit keeps it contained in wind (never past ~190 m), the touchdown drifts ~70 m across
0 → 9 m/s, and at 12 m/s the wind carries it onto the strip (lands **in**). What the guarded build adds
here is **upstream of the glide**: the boost-phase fins hold the stack vertical against the same wind
during the climb (see the `fins.csv` boost rows and the attitude panel), so it tips over at apogee from a
straighter, more repeatable attitude rather than already leaning downwind.

## The boost stage + governor — what changed

`TMS-7-basic` flew boost open-loop (vertical climb, fins neutral) and the glide on bank-to-turn. Guarded
keeps the same glide loop but:

- **Holds attitude through boost.** Past the rod the loop PIDs the captured rod-vertical attitude, so the
  elevons reject the crosswind weathercock during the climb. In the windy runs (`vf_wind*`,
  `vf_corner_stress`) the boost elevons are non-neutral for the whole post-rod climb.
- **Caps fin deflection by airspeed** (`commons.fin_deflection_limit`, ∝ 1/v², × `fin_limit_multiplier`):
  full ±45° when slow, down to ±5° near burnout, so a fin can't bite hard into high-`q` air. The fin
  panel in the HTML shows the commanded angles riding under that envelope.

**What's left** (unchanged from basic): at good sensor quality it lands ~80 m from the centre — just
*off* the 40 m-wide strip, because LANDING rolls wings-level and drifts off on short final. Nailing the
strip wants a tighter final-approach / flare; high-noise (≥50 %) robustness of the bank loop is the other
open thread.

## Corner cases — spike injection (g16)

`--spike` injects a **transient 2× glitch** on the attitude + accel for *one tick* every ~3 s
(deterministic timing, so the traces reproduce the spike instants exactly) — a sudden bad sensor sample.
The same `spike` flag exists in the on-board HITL config (`config_hitl`). Two corner cases are stored in
full:

- **`vf_corner_spike.txt`** / `report_corner_spike.{html,svg}` — F15, 10 % noise, spikes. The loop
  **rejects** glitches: each spike tick kicks the elevons for one frame then recovers; the trajectory
  barely moves (miss 81 m, contained ≤ 181 m).
- **`vf_corner_stress.txt`** / `report_corner_stress.{html,svg}` — F15, **50 % noise + 12 m/s wind +
  spikes**, everything-degraded: still contained (miss 95 m, ≤ 134 m from pad) with the boost fins
  fighting the wind through the climb.

```sh
python3 tools/virtual_flight.py --motor F15 --noise 0.10 --spike -o vf_corner_spike.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.50 --wind 12 --wind-dir 210 --spike -o vf_corner_stress.txt
```

Open the HTML reports and watch the **fins** + **attitude** panels (unified hover) through
`boosting → gliding → landing` — the boost-phase elevon activity is the guarded-fins behaviour the basic
set lacks.

## Files

- `vf_noise{05,10,25,50,100}.txt`, `vf_wind{00,03,06,09,12}.txt` — recorder captures (all rounds)
- `vf_corner_{spike,stress}.txt` + `report_corner_{spike,stress}.{html,svg}` — the two g16 corner cases (committed in full)
- `report_{corner_spike,corner_stress,noise05,noise50,wind00}.html` — **five committed interactive reports**
  (the same rounds basic keeps): the two corner cases + clean / degraded sensors / calm baseline; the full
  set regenerates for any round with the `flight_report.py` command above
- `report_*.svg` — dependency-free per-flight look (plan view + altitude/roll), all rounds
- `compare_noise.svg`, `compare_wind.svg` — sweep overlays (all tracks on one plan view)
