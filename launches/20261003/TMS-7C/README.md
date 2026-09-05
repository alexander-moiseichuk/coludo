# TMS-7C — telemetry airframe

The instrumented flight. Built, calibrated and packed; see the fleet notes for its bring-up history.

| | |
|---|---|
| glider | **216.5 g** |
| booster | **98.9 g** |
| motor | **F15** (98.8 g loaded) |
| liftoff | **414.2 g** |
| wings | **ASE** |
| fins | **fixed — angle selected by the [7A](../TMS-7A/) / [7B](../TMS-7B/) result** |
| electronics | telemetry board, **no fin servos**, light power module |
| config | [`tms7c.config`](tms7c.config) — `concurrency: 1`, servos and control **disabled** |

Servos are absent and control is off, so the airframe is ballast under a fixed fin setting: this flight
buys sensor data, not guidance. That is why the fin angle is taken from the 7A/7B comparison rather than
chosen here — the instrumented flight should use a setting that has already flown.

At **216.5 g** it is lighter than the 235 g `f15_half` combo that landed 7 of 10 in the zone in
[`TMS-7-preflight`](../../../doc/sims/TMS-7-preflight/). That is encouraging for a later guided build on
this airframe, but it says nothing about this flight, which has no active control.
