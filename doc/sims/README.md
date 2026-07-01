# Flight simulations

Closed-loop flight simulations of the Coludo glider — the *real* control code (`navigation` + `pid` +
`mixer` + `sequencer`) flown over the shared flight-dynamics model (`src/glider/sim_model.py`), in two
worlds that share the same physics: the host **virtual-flight** tool (CPython) and the on-board **HITL**
simulator (`tasks/hitl.py`, MicroPython). Each run emits a recorder capture rendered to interactive
plotly HTML + dependency-free SVG.

| Set | What was tested | Where it ran |
|---|---|---|
| [**TMS-7-basic**](TMS-7-basic/) | The original bank-to-turn glide on the TMS-7 airframe (F15): sensor-noise sweep (5–100 %), cross-wind sweep (0–12 m/s), and g16 spike-injection corner cases. The reference that showed the over-range → orbit fix (miss ~85 m). | **host** (`tools/virtual_flight.py`) |
| [**TMS-7-guarded_fins**](TMS-7-guarded_fins/) | The **g12 dynamic-pressure fin governor + boost control stage**, flown as a full matrix (noise 05–100, wind 00–12, corner spike/stress) **on the real ESP32-P4** for **both motors — F15-4 and E16-4**. Every trace (trajectory, fins, **real board vitals**) is device telemetry, collected via rshell-run / adb-take. | **board** (on-board HITL) |
| [**TMS-7-memory_refactoring**](TMS-7-memory_refactoring/) | The **GC-off memory leak** after the Phase-3 hot-path allocation work (fixed-point PID + telemetry/`note` trims), measured from the `mem_free` slope over BOOSTING→DONE. Two rounds (normal + 50 %-weight, the longer-glide worst case) → **time-to-OOM ≈ 72 s**, short of the 180 s gate — the refactoring continues. | **board** (on-board HITL) |

Each subfolder's `README.md` has the per-scenario metric tables (miss / in-zone / max-from-pad /
duration), the findings, and the regenerate commands.
