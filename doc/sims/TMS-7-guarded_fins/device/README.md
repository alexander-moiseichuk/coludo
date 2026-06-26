# Device-collected HITL flights — real load / temp / mem / fins (F15-4 + E16-4)

Unlike the host sweep in the parent directory (where `board_health` is **synthetic / phase-modeled**,
because the host has no MCU), these are **real telemetry from the ESP32-P4**, recorded while the board
flew a *complete* guarded HITL flight to touchdown and streamed to the Luckfox recorder — one run per
motor we actually fly: **F15-4** and **E16-4** (the "-4" = the motor ejection delay, ~4 s after burnout).

Each `<motor>/` directory has:
- **`health.csv`** — real vitals: MCU temperature, `gc.mem_free()`, CPU load (probe-task wake lateness).
- **`sequencer.csv`** — the real on-board stage machine, with actual transition reasons.
- **`servo_eleron_left/right.csv`, `servo_yaw.csv`** — the **real commanded fin angles** (the control
  loop banking to turn): the elevons sweep the full ±45° about neutral 90° (~3000 / ~2100 samples).

## The flights

Both flew the full stage machine to a real glide and touchdown (board uptime µs):

| | F15-4 | E16-4 |
|---|---|---|
| boosting (launch) | 5.23 s, \|a\|=3.7 g | 5.18 s, \|a\|=3.7 g |
| gliding (ejection ≈ burnout+4 s) | 12.81 s | 11.06 s |
| landing (agl ≈ 0 m) | 44.09 s | 29.07 s |
| done (stationary ~1 g) | 47.18 s | 32.12 s |
| glide duration | ~31 s | ~18 s |
| fin samples (eleron) | 3121 | 2132 |

E16-4's shorter burn (1.77 s vs 3.45 s) → lower apogee → shorter glide, exactly as expected.

## Real vs the synthetic model

| metric | **F15-4 (real)** | **E16-4 (real)** | synthetic (host model) |
|---|---|---|---|
| temperature | 31–33 °C | 32–33 °C | 45–63 °C |
| mem_free | 17–32.6 MB | 22.6–32.6 MB | ~4 MB |
| CPU load | 0–47 % | 0–94 % | 30–60 % |

The P4 runs **far cooler** than the model assumed and has **a lot more headroom**. Note the `mem_free`
swing: it *falls* through the flight (down to ~17 MB) and snaps back at touchdown — that is the **g14 GC
policy** visible in real data: `gc.disable()` at launch, garbage accumulates on the ~32 MB PSRAM through
the airborne phase, then `gc.collect()` + re-enable once stationary. The model's flat ~4 MB missed this
entirely. Load peaks at the landing transition (laser/landing work) — the model had that shape right but
the magnitudes off (and E16's busier short flight peaks higher, to 94 %).

## How it was collected (CC-config / rshell-run / adb-take)

1. **rshell** — deploy a runner + per-motor launchers: `rshell -p /dev/ttyACM0 cp hitl_run.py /pyboard/`.
2. **run** — `boardrun.py /dev/ttyACM0 runfile hitl_f15.py` (runfile soft-resets first, clearing the
   module cache, then `exec`s the file; rshell's `repl` needs a TTY so it can't run non-interactively).
   The runner brings up `config_hitl`, **arms** the controller, and flies to `DONE`.
3. **Recorder → Luckfox** — the board streams over UART:1 (GPIO20, 921600); the Luckfox demuxes into
   `/userdata/recordings/<session>_<file>.csv`.
4. **adb** — `adb pull /userdata/recordings/<session>_*.csv`.

## Fixes that made the on-board glide faithful

The first board run glided for 0.3 s then "landed" underground — three bugs, now fixed (all HITL-only,
the real-flight sequencer untouched):

1. **Sim-time vs wall-time.** `hitl.run` advanced the model a *fixed* dt per loop iteration, but the
   sequencer's timeouts are wall-clock; the P4's loop runs ~3× sim_hz, so the model flew past apogee and
   underground before the wall-clock boost timeout fired. Fixed with a **fixed-timestep accumulator**:
   advance the model in stable 0.02 s sub-steps to cover the *real* elapsed time, so 1 sim-s == 1 wall-s.
2. **Ejection timing.** With separation off in HITL, boost→glide used a generic 6 s fallback. Now
   `config_hitl` sets it to **burn + ejection-delay** (F15-4 → 7.45 s, E16-4 → 5.77 s), so the glider
   deploys near apogee like the real `-4` charge.
3. **Arming.** The runner never armed the controller, so the flight loop held the fins **neutral** (no
   bank → no descent → float). Arming is now part of bring-up — hence the real ±45° fin sweep above.

## Caveat — sim sensors are not recorded on-board

In HITL the real sensor drivers are disabled (the `hitl` task provides accel/attitude/agl/altitude/
position on the databoard only), so these sessions have `health` + `sequencer` + the fin streams, but no
`accel/imu/baro/gnss/laser` CSVs. The trajectory/3D reference therefore stays the host sweep (which has
them); this directory is the **device vitals + real control-output** reference. (To also capture the sim
sensors on-board, the `hitl` task would need to push them as telemetry — a separate change.)
