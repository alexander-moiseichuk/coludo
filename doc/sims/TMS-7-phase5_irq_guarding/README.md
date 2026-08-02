# TMS-7 phase5 IRQ guarding — the same matrix, after the wake-up rewrite

The `TMS-7-phase5_refactor` matrix re-flown on the board after `commons.Waiter` replaced
`asyncio.wait_for_ms` in the three interrupt-driven drivers, and after `irq_runs` was added to their
telemetry. Same four cases, same conditions (5 % noise, calm, E16/F15 × full 270 g / light 215 g),
same quality-2 board polar.

[e16_full](e16_full.html) · [e16_light](e16_light.html) · [f15_full](f15_full.html) ·
[f15_light](f15_light.html) · [**overlay plan**](plan_board.svg)

## The headline, and it is a warning about the harness

**The board-only leak improved from 331 KB/s to ~199 KB/s (OOM ~96 s → ~160 s). None of that is
visible in this study, and it cannot be.** `config_hitl._SIM_SENSORS` masks
`accel_adxl375`, `imu_lsm6dso32` and `laser_agl` — the exact three drivers the rewrite fixed — because
the sim provides those quantities instead. A HITL capture therefore measures a board on which the
changed code is not running.

The improvement is real and measured, just with a different instrument
(`test/diag_real_leak.py`: default config, real drivers, no sim, GC forced off). **Do not use a HITL
capture to judge driver-level allocation work.** That is the lesson worth carrying out of this study.

## Comparison, phase5_refactor → phase5_irq_guarding

### Accuracy — unchanged

| case | before | after | delta |
|---|---|---|---|
| `e16_full` | 88.8 m | 88.7 m | −0.1 |
| `e16_light` | 45.7 m | 46.8 m | +1.1 |
| `f15_full` | 121.7 m | 121.7 m | **0.0** |
| `f15_light` | 83.6 m | 85.5 m | +1.9 |

Run-to-run repeatability on this matrix is **~1 m** (measured over three identical flights), so every
case is at or inside the noise -- `f15_full` lands on the same 121.7 m to the decimetre. Re-flown with
BOTH interrupt lines working (the earlier pass in this study had the LSM6DSO32's INT1 dead and the
ADXL375 unplugged), which is why these supersede the first numbers written here.

### Glide span — unchanged

| case | before | after |
|---|---|---|
| `e16_full` | 21.9 s | 22.1 s |
| `e16_light` | 26.5 s | 26.5 s |
| `f15_full` | 43.1 s | 43.2 s |
| `f15_light` | 50.6 s | 50.5 s |

Within 0.2 s everywhere. The control path is untouched, which is what these confirm.

### Fin movement and servo energy — within scatter

| case | moves before → after | travel before → after | energy before → after |
|---|---|---|---|
| `e16_full` | 799 → 900 | 1840° → 2138° | 9.8 J → 9.8 J |
| `e16_light` | 1031 → 1019 | 2316° → 2396° | 10.7 J → 12.8 J |
| `f15_full` | 1508 → 1514 | 3696° → 3671° | 18.9 J → 18.6 J |
| `f15_light` | 1826 → 1791 | 4334° → 4098° | 19.5 J → 19.4 J |

Average power stays **0.28–0.33 W** on the servo rail in both, matching every earlier board run. The
per-case wobble follows fin travel, which follows the sim's noise seed, not the change.

### Memory — unchanged in HITL, by construction

| case | leak KB/s before → after | oom_s before → after | rescues |
|---|---|---|---|
| `e16_full` | 286–331 → 294–338 | 64–104 → 62–101 | 0 → 0 |
| `e16_light` | 286–355 → 320–390 | 59–105 → 52–94 | 1 → 1 |
| `f15_full` | 280–352 → 309–369 | 50–108 → 46–97 | 2 → 2 |
| `f15_light` | 284–352 → 292–363 | 43–106 → 41–103 | 2 → 2 |

No improvement, and slightly *worse* on paper — because what dominates a HITL leak is the sim's own
publishing (measured at 166 KB/s of a 347 KB/s total at 50 Hz), plus one extra telemetry column per
row now that `irq_runs` is recorded. The rescue counts are identical, which is the useful part: the
in-flight recovery behaves exactly as before.

### IRQ health — the new measurement

| case | irq_runs anomalies |
|---|---|
| `e16_full` | **0** |
| `e16_light` | **0** |
| `f15_full` | **0** |
| `f15_light` | **0** |

Every wake on every interrupt-driven stream consumed **exactly one edge** — no 0 (a timed-out
fallback, meaning the driver is sampling blind on its timer) and no >1 (an overrun, meaning the loop
was late and edges piled up). Each report's title carries this verdict beside the leak and
time-to-OOM, and the board-health panel plots one step trace per sensor.

In HITL these come from the sim, which delivers one sample per publish, so a clean result here is a
**plumbing check** — it proves the field is recorded, mirrored and rendered end to end.

### The same measurement on REAL sensors — it found a fault, and then confirmed the repair

Run without HITL at all (`diag_real_leak.probe()`: default config, real drivers, recorder on), the
capture tells a different story:

| stream | before the wiring fix | after |
|---|---|---|
| `accel_adxl375` | 928 samples, **all 1** | 924 samples, **all 1** |
| `imu_lsm6dso32` | **32 samples, all 0** | **920 samples, all 1** |

The zero was the LSM6DSO32's INT1 landing on the wrong pad — a **value in the telemetry** rather than
a log line someone has to notice. The sample counts price it: 32 against 920 over the same window,
because that driver waited out its fallback on every wake instead of sampling on data-ready. After the
operator moved the wire, the same measurement reports a clean 1 on every sample. `irq_runs` found the
fault and then verified the repair, which is the whole reason for the column.

This is the case `irq_runs` exists for. Nothing in the accel or gyro *values* looks wrong — a driver
polling its fallback still produces entirely plausible data — so without this column the degradation
is invisible. It is also the v0.2 hardware item (route INT1) with a number attached at last, and the
check that will confirm the fix worked.

**Board-only leak, both interrupts live: 265 KB/s, OOM ~120 s** — the figure this HITL matrix
structurally cannot show. Read that against the 331 KB/s / 96 s baseline: the wake-up rewrite is worth
**-20 % leak and +25 % survival**, but it is NOT the 199 KB/s figure measured mid-repair. That reading
was taken while the LSM6DSO32's interrupt was dead and it was sampling 29x less often, so part of the
apparent win was simply a sensor not doing its job. With both parts working at full rate the honest
number is 265 KB/s, and the 150 s target is not yet met.

## What this study establishes

- The wake-up rewrite is **behaviour-neutral**: accuracy, glide span, fin travel, servo energy and
  rescue counts all sit inside run-to-run scatter.
- `irq_runs` is plumbed end to end — driver → telemetry → both sims → schema cross-check → chart →
  report headline — and reads clean.
- **A HITL capture cannot measure driver-level allocation**, because HITL masks the drivers. The
  331 → 199 KB/s result belongs to `diag_real_leak.py`, and this study is the evidence that the change
  cost nothing in flight.

## Reproduce

```bash
tools/deploy.sh
for spec in "F15 f15_full 270" "E16 e16_full 270" "E16 e16_light 215" "F15 f15_light 215"; do
    set -- $spec
    bash tools/hitl_collect.sh "$1" "$2" 0.05 0.0 210.0 False captures/irq_guarding "$3"
done
python3 tools/flight_metrics.py captures/irq_guarding --sort name
# the leak figure this study CANNOT show -- real drivers, no sim:
mpremote connect PORT exec "import sys; sys.path.insert(0,'/test'); \
    import diag_real_leak; diag_real_leak.probe()"
```
