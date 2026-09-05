# TMS-7D — servo airframe

The long glider: three SG90 fin servos and the full power module.

| | |
|---|---|
| glider | **287.5 g** |
| booster | **97.9 g** |
| motor | **F15** (98.8 g loaded) |
| liftoff | **484.2 g** |
| wings | **ASE** |
| fins | three **SG90** servos (yaw, left eleron, right eleron) |
| electronics | telemetry board + **full power module** |
| config | [`tms7d.config`](tms7d.config) — servos fitted, **control OFF**, `concurrency: 3` |
| also here | [`tms7d_control.config`](tms7d_control.config) — full active control; **nothing flies it yet** |

Flies with servos fitted and exercisable but **no active control**: fins move for `probe()` and operator
commands, never under a PID. So this launch tests the servo install and the power module in flight, not
guidance.

**`concurrency: 3` is a flight-power-board setting.** Three SG90s slewing together draw ~4 A; a 4 V / 1 A
bench supply browns out. Installing this profile on a bench board once killed `servo_eleron_right`
outright. On the bench, use 1 or clear `board.config` so the firmware default applies.

## The mass to be aware of

At **287.5 g** this is almost exactly the `f15_full` combo (285 g) from
[`TMS-7-preflight`](../../../doc/sims/TMS-7-preflight/), which over three runs landed a **88–91 m median
and 0 of 10 in the zone** — against 37–46 m and **7 of 10** for the same motor at 235 g.

Irrelevant to 2026-10-03, since control is off. It matters for the first guided flight: at this mass the
study says the endgame does not converge, and closing the gap means finding ~52 g.
