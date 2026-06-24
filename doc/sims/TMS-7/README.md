# TMS-7 virtual-flight experiments

Closed-loop flight simulations of the TMS-7 airframe on an **F15** motor from the **HPRC** pad
(`25.514379, -80.391795`), produced by `tools/virtual_flight.py`. Each run flies the *real* control
code (`navigation` + `pid` + `mixer`) under the *real* config (`config_hitl`) over the shared flight
model (`src/glider/sim_model.py`) — the same model and the same loop the board runs in HITL
(`tasks/hitl.py`), just in CPython so it produces a recorder capture without a flight.

The captures are in the exact wire format `flight_telemetry.parse()` reads. Each `report_*.svg` is a
per-flight report (plan-view ground track + altitude/roll time series); each `compare_*.svg` overlays a
whole sweep on one plan view. They are rendered by `tools/flight_svg.py` (dependency-free: metres from
the pad, north up, filled box = landing zone, dot = touchdown). For the interactive 3D + linked
time-series version, render any capture with `tools/flight_report.py` (needs plotly).

## Regenerate / render

```sh
# a capture (stdout or -o file); --noise is the sensor-degradation knob, --wind a steady cross-wind
python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o vf_noise05.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.10 --wind 12 --wind-dir 210 -o vf_wind12.txt

# per-flight SVG report (no deps); --pad/--zone draw the HPRC site
python3 tools/flight_svg.py vf_noise05.txt -o report_noise05.svg \
    --pad 25.514379,-80.391795 --zone 25.514630,-80.392880,25.514656,-80.391155

# a sweep overlay
python3 tools/flight_svg.py vf_noise*.txt --overlay -o compare_noise.svg --labels "5%,10%,25%,50%,100%" \
    --pad 25.514379,-80.391795 --zone 25.514630,-80.392880,25.514656,-80.391155

# interactive 3D trajectory + linked time-series (needs plotly: pip install plotly)
python3 tools/flight_report.py vf_noise05.txt -o noise05.html
```

## Experiment 1 — sensor noise (no wind) → `compare_noise.svg`, `report_noise{05,10,25,50,100}.svg`

`--noise N` perturbs every accel/attitude/altitude/agl reading by ±N·|value| (the same `sim_model.noisy`
the board uses); GNSS position is left clean (board parity). Sweep 5 → 100 %:

| Noise N | Landing miss (m) | Duration (s) | Peak baro reading (m) |
|--------:|-----------------:|-------------:|----------------------:|
|     5 % |              456 |         40.5 |                   285 |
|    10 % |              456 |         40.5 |                   299 |
|    25 % |              470 |         42.0 |                   354 |
|    50 % |              472 |         41.0 |                   407 |
|   100 % |              505 |         43.7 |                   594 |

**Finding:** the landing point barely moves (456 → 505 m) even at 100 % noise. The attitude loop rejects
zero-mean sensor noise and the steering uses the clean GNSS fix, so the *trajectory* is robust — while
the *telemetry* visibly degrades (the "peak baro reading" column is pure noise inflating over a true
apogee of ~290 m). Sensor noise alone is not what threatens the mission.

## Experiment 2 — cross-wind (10 % noise) → `compare_wind.svg`, `report_wind{00,03,06,09,12}.svg`

A steady wind blowing toward 210° (across the glide), 0 → 12 m/s vs the ~14 m/s trim airspeed:

| Wind | Landing miss (m) | Duration (s) |
|-----:|-----------------:|-------------:|
| calm |              456 |         40.5 |
|  3   |              385 |         40.4 |
|  6   |              333 |         40.9 |
|  9   |              165 |         40.5 |
| 12   |               77 |         40.5 |

**Finding:** wind dominates the trajectory — a 12 m/s wind shifts the landing point ~380 m. (Here it
blows back toward the zone and *reduces* the miss; a wind the other way would push it equally far out.)
At 12 m/s the glider can barely make headway against the air mass — it is the disturbance the nav must
actually fight, not sensor quality.

## The standing problem (both experiments)

In calm air the glider **overshoots the 100 m zone by ~456 m**: it reaches apogee ~290 m above the pad
and the ~8:1 glide carries it far downrange, while the straight-to-the-gate nav cannot bleed that energy
inside a 100 m radius. The airframe over-*ranges* this zone — landing in it needs energy management
(a spiral / loiter descent over the target), not just pointing at the gate. This is the next nav design
question; the noise/wind sweeps above are robust precisely because they ride on top of this dominant
geometry.

## Files

- `vf_noise{05,10,25,50,100}.txt` — sensor-noise sweep captures
- `vf_wind{00,03,06,09,12}.txt` — cross-wind sweep captures
- `report_noise*.svg`, `report_wind*.svg` — per-flight reports (plan view + altitude/roll vs time)
- `compare_noise.svg`, `compare_wind.svg` — sweep overlays (all tracks on one plan view)
