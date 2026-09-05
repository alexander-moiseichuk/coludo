# Launch 2026-10-03 — HPRC

Rescheduled from **2026-09-05, cancelled for weather**.

Six slots: one booster-only structural test, two glide-comparison airframes, one telemetry airframe,
one servo-equipped airframe, and one not yet built.

## Manifest

Airframe masses are **as assembled, without the motor**. Motor masses are the measured values from
`doc/hardware.md` — **F15-4 98.8 g**, **E16 ~82.5 g** loaded.

| airframe | glider | booster | motor | + motor | **liftoff** | glide mass |
|---|---|---|---|---|---|---|
| [TMS-7](TMS-7/) | — | **119.5 g** | F15 | 98.8 g | **218.3 g** | n/a (no glider) |
| [TMS-7A](TMS-7A/) | 183.0 g | 99.0 g | E16 | 82.5 g | **364.5 g** | 183.0 g |
| [TMS-7B](TMS-7B/) | 184.3 g | 98.2 g | E16 | 82.5 g | **365.0 g** | 184.3 g |
| [TMS-7C](TMS-7C/) | 216.5 g | 98.9 g | F15 | 98.8 g | **414.2 g** | 216.5 g |
| [TMS-7D](TMS-7D/) | 287.5 g | 97.9 g | F15 | 98.8 g | **484.2 g** | 287.5 g |
| [TMS-7E](TMS-7E/) | *to be built* | — | — | — | — | — |

## What each slot answers

* **TMS-7** — does the new booster construction survive the strongest motor? Structure only, no glider.
  The F15 is deliberate: test the worst case before trusting it under an instrumented airframe.
* **TMS-7A vs TMS-7B** — the wing comparison, flown as a matched pair on the same motor: **ASE wings
  with fins at 0°** against **carbon wings with fins at −5°**. Dual cameras, 1.3 g apart in mass, so the
  difference that shows up is the wing and the fin setting rather than the build.
* **TMS-7C** — telemetry. Fins **fixed at whatever 7A/7B selects**, so the instrumented flight uses the
  setting the comparison just validated rather than a guess.
* **TMS-7D** — the servo airframe: three SG90s and the full power module.
* **TMS-7E** — simplified electronics, see its folder.

## One thing the flight data already says about 7D's mass

[`TMS-7-preflight`](../../doc/sims/TMS-7-preflight/) flew 90 HITL flights across three load/thrust
combos last week. Two of them bracket the airframes here:

| study combo | glide mass | median miss | in zone |
|---|---|---|---|
| `f15_half` | 235 g | 37–46 m | **7/10** |
| `f15_full` | 285 g | 88–91 m | **0/10** |

**TMS-7D at 287.5 g is `f15_full` almost exactly**, and TMS-7C at 216.5 g is lighter than `f15_half`.

This does **not** affect 2026-10-03: 7D flies `tms7d.config`, which has servos fitted but **control
OFF**, so no airframe here attempts a guided landing. It matters for the flight after: when 7D does fly
under a PID, it will do so at the mass that landed 0 of 10 in the zone, and reaching the 235 g that
landed 7 of 10 means removing ~52 g. Better to know that before the airframe is finished than after.

## Plan — 2026-09-04

- [ ] check **TMS-7D**, upload the latest firmware; after final assembly the board is worked over **OTA**
- [ ] prepare firmware changes if any are needed
- [ ] close the `lmoiseichuk/launch_0905` branch

## Plan — next branch

- [ ] move the breadboard to **MicroPython 1.29** (1.30-pre carries nothing interesting and no fixes we need)
- [ ] possibly update `mpy-cross` alongside it
- [ ] settle **main board v0.2**:
  - merge **main + power** onto one board
  - merge **GNSS** in, cutting its power lines and UART run
  - a **separate I²C cluster for SDP810 + VL53L4CX**, everything else on the internal cluster
- [ ] rework the board layout for that
- [ ] rework the pin maps
- [ ] design the PCB and order it
- [ ] start building **TMS-7E** on the new main board
- [ ] disable the devices the new layout drops, in config

## After the October launch

- analysis session on the flight data
- possibly a **Rust migration**, staying on the same ESP32-P4 so the hardware is reused
- further out, a **C6 board** — only if its memory and FPU-emulation speed prove sufficient
