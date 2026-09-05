# TMS-7E — simplified electronics (not yet built)

To be assembled after the **main board v0.2**, and the reason that board exists. Same airframe idea as
[TMS-7D](../TMS-7D/), with the electronics consolidated.

| | |
|---|---|
| status | **not built** — masses, booster and motor to be decided |
| config | none yet |

## What changes against 7D

* **GNSS onto the single main board**, antenna separated — removes a power run and a UART run
* **power module onto the main board** — main + power become one board
* **no ADXL375** — the LSM6DSO32's ±32 g already covers the 8–12 g boost, and the measured HITL peak is
  4.3 g. The backstop has never recorded anything the primary could not.
* **no LSM6DSO32** — dropped with the simplified layout

The v0.2 board also splits the buses: a **separate I²C cluster for SDP810 + VL53L4CX**, everything else
on the internal cluster.

Removing both SPI IMUs takes the primary `accel` and the only gyro `rate` with them, so the attitude and
airspeed paths must be re-sourced before this flies — the BNO055 backup becomes the primary rather than
the fallback. The devices the new layout drops need disabling in config, and `provides` priorities need
re-checking, not just the pin map.
