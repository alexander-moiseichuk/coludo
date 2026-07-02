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
| [**TMS-7-memory_refactoring**](TMS-7-memory_refactoring/) | The **GC-off memory leak** after the Phase-3 hot-path allocation work (fixed-point PID + telemetry/`note` trims), measured from the `mem_free` slope over BOOSTING→DONE. A full on-board HITL capture set of the post-refactoring firmware over the corrected TMS-7 v2 weight matrix (E16/F15 × full 300 g / half 150 g glider; booster ejects at separation). Calm, 5 % noise, 25 Hz. Each flight has a 3D report (trajectory + attitude + fins + **real INA226 servo power** + **GC-off leak/OOM**), an SVG, and a shared movie. Real-flight control-path leak measured ~24 KB/s → **time-to-OOM ≈ 10 min** (F01 + F05), well past the 180 s gate. | **board** (on-board HITL) |
| [**TMS-7-fixnums**](TMS-7-fixnums/) | The same weight matrix re-flown on the **fixed-point (`fixnum`) + gyro-D-term firmware** (centidegree integer PID/attitude/drivers, LSM6DSO32 rate as the PID D term, integer INA226). Behaviour is baseline-neutral (same trajectory, power, CPU); the HITL leak is flat (sim-dominated). The measured **real control-path leak is ~42 KB/s → time-to-OOM ~13 min** at 100 Hz (`pid.step` now 0 B/step; the residual is the deliberately-float airspeed `|accel|` chain), and each report adds a **gyro-rate panel**. | **board** (on-board HITL) |

Each subfolder's `README.md` has the per-scenario metric tables (miss / in-zone / max-from-pad /
duration), the findings, and the regenerate commands.
