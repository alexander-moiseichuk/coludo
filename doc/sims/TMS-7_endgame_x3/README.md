# TMS-7 endgame ×3 — `o` / `ov` / `oo` + the air-quality effect (host)

Host virtual-flight provenance (`tools/virtual_flight.py`, the real `guidance`/`governor`/`pid`/`mixer`
over `sim_model.py`, **F15**, 5 % noise, no wind, seed 1) — the three endgame holding patterns flown over
the **HPRC 187 × 40 m strip** (aspect k ≈ 4.7), each at the worst-case **air-quality 2** polar (L/D 2,
~7 m/s trim sink) and the realistic **quality 5** (L/D 5, ~2.8 m/s sink, via the `sim_model.trim_sink`
/ `VF_QUALITY` knob).

This documents the **converging `ov` centreline-oval pattern** (`guidance.Heading.FIG_OVAL`) — a racetrack
whose leg-reversal point shrinks with the endgame altitude fraction so the oval collapses onto the zone
centre — and what changes as the polar improves from the sim floor toward the real airframe.

| pattern | `auto` k-range | 3D report | touchdown q2 (floor) | touchdown q5 (realistic) |
|---|---|---|---|---|
| **`o`** single circle | k < 2 | [report](o.html) | 31 m ✗ | 60 m ✓ in-zone |
| **`ov`** converging oval | 2 ≤ k < 6 | [report](ov.html) | 110 m ✗ | **27 m ✓ in-zone — closest** |
| **`oo`** two lobes | k ≥ 6 | [report](oo.html) | 77 m ✗ | 74 m ✗ |

Combined views: [**3D overlay** (all 6, q2 solid / q5 dashed)](endgame_3d.html) · [**top-down plan**](plan_x3.svg).

## Finding — the converging rework works, but *at the realistic polar, not the floor*

The rework gave `ov` a converging law (leg-reversal reach `∝ endgame altitude` + a damped `approach_to`
cross-track intercept in place of pure pursuit). At the airframe's expected **quality 5** it now
**collapses onto the centre and lands in-zone at 22–27 m — closer than `o` (60 m) and far better than the
limit-cycling `oo` (74 m).** Across seeds 1/2/3 it is `27✓/219/22✓` (two in-zone; seed 2 is a hard flight
that overruns for *every* pattern, incl. `o`=183 m).

The catch is **budget**. At the **quality-2 floor** the glider sinks too fast to complete the collapse, so
`ov` reverses only once or twice and touches down mid-swing 110 m out — *worse* than both `o` (31 m) and
`oo` (77 m). So the converging oval needs the realistic sink budget to pay off; at the pessimistic floor the
single circle `o` is still the closest and the only pattern that ever reaches the strip.

Two consequences for `auto`:

- **`ov` now beats `oo` at the realistic polar** (27 vs 74 m in-zone on the *same* k = 4.7 strip), so the
  `oo` auto-tier was pushed out to **`k ≥ 6`** — the whole moderate-to-high band `2 ≤ k < 6` (HPRC included)
  flies the converging `ov`, and `oo` is reserved for genuinely long strips. `'oo'` stays the documented
  negative result ([TMS-7-oo_landing](../TMS-7-oo_landing/)).
- **Which default is "safe" depends on the polar you design for.** Floor-first (q2) → `'o'` wins everywhere.
  Realistic (q4-6) → `'ov'` wins on any elongated strip. `'o'` remains the floor-safe fallback; `'ov'` is the
  realistic-quality optimum.

## Regenerate

```bash
SP=/tmp/caps; mkdir -p $SP
for q in 2 5; do for p in o ov oo; do
  VF_ENDGAME=$p VF_QUALITY=$q VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/q_${p}_${q}_1.txt
done; done
PV=~/.local/share/pipx/venvs/plotly/bin/python
for p in o ov oo; do $PV tools/flight_report.py --cdn $SP/q_${p}_5_1.txt -o doc/sims/TMS-7_endgame_x3/$p.html; done
$PV tools/flight_svg.py $SP/q_{o,ov,oo}_5_1.txt --overlay --labels o,ov,oo \
    --zone 25.514944,-80.392972,25.514583,-80.391111 --pad 25.514379,-80.391795 \
    -o doc/sims/TMS-7_endgame_x3/plan_x3.svg
# 3D overlay of all 6: doc/sims/TMS-7_endgame_x3/endgame_3d.py (VF_QUALITY 2 & 5)
```
