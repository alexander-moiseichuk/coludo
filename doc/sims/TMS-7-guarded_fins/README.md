# TMS-7 virtual-flight — guarded fins (g12)

The **guarded-fins** build of the TMS-7 virtual flight: the same closed-loop sim as
[`../TMS-7-basic`](../TMS-7-basic/) (real `navigation`+`pid`+`mixer` over `sim_model`, `config_hitl`,
F15 from the HPRC pad), but now flying the **g12 dynamic-pressure fin governor + boost control stage**.
`TMS-7-basic` is the prior behaviour (bank-to-turn glide only, boost flown open-loop) kept for comparison.

## What's new vs TMS-7-basic

1. **Boost-phase attitude hold.** `BOOSTING` is now a control stage. On leaving the rod the loop captures
   the rod-vertical attitude and holds it — the fins **fight the crosswind weathercock** that would
   otherwise tilt the stack over during the climb. In `TMS-7-basic` the boost was a 1-DoF vertical climb
   with the fins parked at neutral; here they work. Engages only **past the rod** (airspeed >
   `boost_engage_speed` = 15 m/s) — on the rod the 3-point mount holds it and there is no `q` to bite.
2. **Dynamic-pressure governor.** Aero torque scales with `q ∝ v²`, so the max fin deflection is
   scheduled `∝ 1/v²`, clamped to `[5°, 45°]` (× the `fin_limit_multiplier` safety dial) — full authority
   slow, ±5° near burnout. The cap rides the **whole** flight (boost, glide, landing), not just boost.
   See `specs/coludo.md` → "Fin authority".

## What the captures show

The `vf_guarded_*.txt` recorder captures are the wire format `flight_report.py` / `flight_svg.py` read
(same as the basic set). The headline is the **`fins.csv` stream during boost** — in the windy run the
elevons are non-neutral for the entire post-rod climb (e.g. `eleron_left;eleron_right` swinging
`84;91 → 84;96 …` — differential for roll, common for pitch), holding the stack vertical against the
12 m/s crosswind. In `TMS-7-basic` those same boost rows are flat `90;90;90`.

| run | scenario |
|---|---|
| `vf_guarded_noise05` | clean sensors (5 %), no wind — the baseline |
| `vf_guarded_wind12`  | 10 % noise, 12 m/s crosswind @210° — **the headline: boost fins fight the wind** |
| `vf_guarded_stress`  | 50 % noise + 12 m/s wind + spikes — everything-degraded |

The committed `report_guarded_*.svg` are the dependency-free look (plan-view ground track + altitude/roll);
the boost-phase **roll/attitude** stays nearer vertical here than the basic run would lean.

## Regenerate

```sh
# captures (the guarded governor + boost stage are now the default control path)
python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o vf_guarded_noise05.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.10 --wind 12 --wind-dir 210 -o vf_guarded_wind12.txt
python3 tools/virtual_flight.py --motor F15 --noise 0.50 --wind 12 --wind-dir 210 --spike -o vf_guarded_stress.txt

# dependency-free SVG (plan view + altitude/roll)
python3 tools/flight_svg.py vf_guarded_wind12.txt -o report_guarded_wind12.svg \
    --pad 25.514379,-80.391795 --zone 25.514944,-80.392972,25.514583,-80.391111

# interactive HTML (3D trajectory + the FINS time-series panel that shows the boost fins working) -- needs
# plotly: `python3 -m pip install plotly`, then:
python3 tools/flight_report.py vf_guarded_wind12.txt -o report_guarded_wind12.html --cdn
```

Open the HTML and watch the **fins** + **attitude** panels through `boosting → gliding → landing`
(unified hover): the boost-phase elevon activity is the g12 guarded-fins behaviour the basic set lacks.
