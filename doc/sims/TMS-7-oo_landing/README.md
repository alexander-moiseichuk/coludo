# TMS-7 "oo" endgame — the two-lobe landing pattern (on-board HITL)

On-board HITL provenance (`tasks/hitl.py`, the real `guidance`/`governor`/`pid`/`mixer` over the
quality-2 worst-case polar, 10 % noise, **3 m/s cross-wind toward 210°**); each variant has an
interactive 3D report and a top-down SVG plan.

This round documents the **`oo` endgame holding pattern** — two lobes along the zone's long axis
(`guidance.Heading.FIG_OO`, selected by `auto` because the HPRC strip's aspect k≈4.65 > 2). The
`oo` law is stable (two SAME-turn-sense lobes, no figure-8 divergence) and collapses toward the centre,
but the lobes are wider than the 40 m strip and drift on the wind, so it lands **out of zone in 3 of 4**.

| variant | glider | 3D report | plan | touchdown from centre | in-zone |
|---|---|---|---|---|---|
| **E16, full** | 285 g | [report](e16_full.html) | [svg](e16_full.svg) | 54.9 m | ✗ |
| **E16, half** | 235 g | [report](e16_half.html) | [svg](e16_half.svg) | 87.4 m | ✗ |
| **F15, full** | 285 g | [report](f15_full.html) | [svg](f15_full.svg) | 59.2 m | ✗ |
| **F15, half** | 235 g | [report](f15_half.html) | [svg](f15_half.svg) | 62.3 m | ✓ |

## Finding — `oo` is a documented negative result; ship `'o'`

The two-lobe pattern gets *closer* to centre than a circle in still air (no-wind min ~22 m) but cannot
both fit a narrow strip and survive wind in the ~7 s endgame: the lobes overrun the short edges, and a
cross-wind blows them further out. Capping the lobe radius to the strip width forces the steep bank
whose sink then drops the glider out anyway. The reliable in-zone endgame remains the **single circle**
(`endgame_pattern: 'o'`, ~37 m from centre). The pattern selector (`guidance.Heading` + `auto` /
`o` / `oo` / `o-o`, the Mission deciding from the zone aspect) is in place for when a working
elongated-strip geometry exists.

## Regenerate

```bash
# fly one variant (records to the Luckfox), pull the 3D report + svg
printf 'import hitl_run\nhitl_run.fly("F15", 0.10, 3.0, 210.0, glider_g=235)\n' > /tmp/fly.py
tools/board_reboot.py $PORT && mpremote connect $PORT run /tmp/fly.py
tools/flight_pull.sh <session> <dir>                       # -> <session>.txt + .html + .svg
```
