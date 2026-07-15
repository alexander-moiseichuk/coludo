# TMS-7 endgame ×3 — `o` / `ov` / `oo` + the air-quality effect (host)

Host virtual-flight provenance (`tools/virtual_flight.py`, the real `guidance`/`governor`/`pid`/`mixer`
over `sim_model.py`, **F15**, 5 % noise, no wind, seed 1) — the three endgame holding patterns flown over
the **HPRC 187 × 40 m strip** (aspect k ≈ 4.7), each at the worst-case **air-quality 2** polar (L/D 2,
~7 m/s trim sink) and the realistic **quality 5** (L/D 5, ~2.8 m/s sink, via the new `sim_model.trim_sink`
/ `VF_QUALITY` knob).

This documents the new **`ov` centreline-oval pattern** (`guidance.Heading.FIG_OVAL`) and the 3-tier
`auto` selector, plus what changes when the polar improves from the sim floor toward the real airframe.

| pattern | `auto` k-range | 3D report | touchdown q2 | touchdown q5 |
|---|---|---|---|---|
| **`o`** single circle | k < 2 | [report](o.html) | 31 m ✗ | **60 m ✓ in-zone** |
| **`ov`** centreline oval | 2 ≤ k < 4 | [report](ov.html) | 55 m ✗ | 120 m ✗ |
| **`oo`** two lobes | k ≥ 4 | [report](oo.html) | 77 m ✗ | 74 m ✗ |

Combined views: [**3D overlay** (all 6, q2 solid / q5 dashed)](endgame_3d.html) · [**top-down plan**](plan_x3.svg).

## Finding — `o` wins because it *converges*; more budget doesn't rescue a limit-cycle

Ranking is `o < ov < oo` at **both** qualities. `o` spirals inward (radius ∝ altitude) and collapses onto
the centre; `ov`'s cross-track track-hold and `oo`'s two lobes both **limit-cycle / orbit** — they never
settle, and `ov`'s round-trip legs + end U-turns still sweep the *narrow* dimension.

The counter-intuitive part is the **air-quality** result. Quality 5 gives ~2.5× more time aloft **and** the
glider arrives at the zone with far more energy to dissipate, so the endgame spends more laps — which
**amplifies whatever the pattern does**: the converging `o` uses the extra laps to land *in-zone* (31 → 60 m
but inside the strip), while the non-converging `ov` uses them to drift **further out** (55 → 120 m). So at
the airframe's true polar (4–6) the endgame's real job is to *dissipate arrival energy onto the target*, and
**convergence — not budget — is the bottleneck.**

Practical: the 3-tier `auto` selector (`o` / `ov` / `oo` by aspect, thresholds 2 and 4) is in place, but
`'ov'` is **not production-ready** — it needs a *converging* rework (a damped track-hold with a heading-rate
term + fitting the leg length to the sink budget so touchdown lands mid-leg on the centreline). Ship
**`endgame_pattern: 'o'`** for HPRC and any strip today. `'oo'` remains the documented negative result
([TMS-7-oo_landing](../TMS-7-oo_landing/)).

## Regenerate

```bash
SP=/tmp/caps; mkdir -p $SP
for q in 2 5; do for p in o ov oo; do
  VF_ENDGAME=$p VF_QUALITY=$q VF_SEED=1 python3 tools/virtual_flight.py --motor F15 --noise 0.05 -o $SP/q_${p}_${q}_1.txt
done; done
PV=~/.local/share/pipx/venvs/plotly/bin/python
for p in o ov oo; do $PV tools/flight_report.py --cdn $SP/q_${p}_2_1.txt -o doc/sims/TMS-7_endgame_x3/$p.html; done
$PV tools/flight_svg.py $SP/q_{o,ov,oo}_2_1.txt --overlay --labels o,ov,oo \
    --zone 25.514944,-80.392972,25.514583,-80.391111 --pad 25.514379,-80.391795 \
    -o doc/sims/TMS-7_endgame_x3/plan_x3.svg
# 3D overlay of all 6: doc/sims/TMS-7_endgame_x3/endgame_3d.py (VF_QUALITY 2 & 5)
```
