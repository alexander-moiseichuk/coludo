# TMS-7 — post-audit regression matrix

**24 board flights** (12 scenarios × F15-4 and E16-4) flown on the real ESP32-P4 HITL after a
repo-wide audit landed **twelve firmware changes**. The question this set answers is narrow and
deliberately unglamorous: *did any of them move the aircraft?*

## Why this run exists

The audit fixes were mostly latent or on error paths — a PID integral that only matters once `ki` is
non-zero, a CRC that only fires on a corrupted frame, an SPI resync that only runs when a peripheral is
replaced. Latent is not the same as harmless, and "it should be behaviour-neutral" is a claim, not a
result. So the matrix was flown and compared against the last board study at the same polar.

## Result — no regression

| | q5.5 baseline ([catapult_evaluation](../TMS-7-catapult_evaluation/)) | **this run** |
|---|---|---|
| normal-scenario miss | 42–97 m | **E16 55–95 m, F15 53–115 m** |
| in-zone | 1 of 7 | 0 of 24 |
| flights completed | — | **24 of 24, 0 skips** |

The distributions are the same family. In-zone stays the exception for the reason the previous study
established and this one does not revisit: at q5.5 the glider reaches the target with altitude in hand
and then flies the holding pattern away from it. That is a guidance limitation, not a regression.

### Per-scenario miss (m)

| scenario | E16 | F15 |
|---|---|---|
| `noise05` | 69.6 | 94.5 |
| `noise10` | 71.7 | 90.7 |
| `noise25` | 92.1 | 85.1 |
| `noise50` | 90.3 | 53.0 |
| `noise100` | 135.6 | 241.3 |
| `wind00` | 71.8 | 84.2 |
| `wind03` | 74.6 | 59.8 |
| `wind06` | 56.9 | 53.0 |
| `wind09` | 55.4 | 72.8 |
| `wind12` | 95.0 | 59.6 |
| `corner_spike` | 71.3 | 114.9 |
| `corner_stress` | 638.3 | 661.6 |

`corner_stress` (50 % noise + 12 m/s wind + spike injection) is the designed worst case and is the only
scenario either motor loses outright. `noise100` degrades as sensor quality collapses, which is the
sweep behaving as intended rather than a fault.

## What this run found that the flights did not

Two bugs in the harness itself, both of which had been silently corrupting results:

* **`hitl_matrix.sh` flew ONE scenario of twelve and printed "matrix done".** `mpremote run`, inside
  `hitl_collect.sh`, reads stdin — so the first flight consumed the rest of the scenario heredoc, the
  loop saw EOF and exited 0. Caught by counting `OK` lines after a run that looked clean: one, where
  there should have been twelve. **Any matrix result produced before this fix is one flight, not a
  matrix, whatever its log says.**
* **Zero HTML reports, reported as success.** `plotly` lives in a pipx venv here, not the system
  python, so every `flight_report.py` call failed and `|| true` swallowed all five.

Both are fixed, and the summary line now states what was actually produced (`flights: 12, svg: 14,
html: 5`) precisely because "matrix done" is the message that hid them.

## Caveat on the comparison

This is not a strict A/B. The baseline is the seven-case fault matrix; this is the twelve-case
noise/wind sweep, so the scenario sets differ and only the *distributions* are comparable. A true A/B
would mean re-flying 24 sorties against the pre-audit build. The weaker claim is the honest one: the
misses sit in the same range, and no scenario moved in a way that suggests the control path changed.

## Reproducing

```bash
tools/deploy.sh
bash tools/hitl_matrix.sh E16 /tmp/hitl/E16
bash tools/hitl_matrix.sh F15 /tmp/hitl/F15
```

`PLOTLY_PY` is auto-detected now; set it explicitly if plotly lives somewhere unusual.
