# Introduction

This document is intended to provide a top-level architectural overview of the `Coludo` project.

> **Note:** hardware composition, pin mapping, Wi-Fi role, storage, and the configuration
> lifecycle are governed by [`board-config.md`](board-config.md), which is authoritative
> wherever it conflicts with statements below. In particular: the board joins the Control
> Center's Wi-Fi network as a **station** (it does not host an access point), the controller
> has **no SD card** (logs/telemetry/video go to the Recorder over UART), and the **camera
> lives on the Recorder**, not the controller.

The core concept of `Coludo` stems from the idea that traditional active-control rocket launches can be made significantly more engaging by introducing a secondary phase: at apogee, a glider deploys from the booster stage to achieve a controlled, gentle recovery back to earth.

The lower booster stage has already been successfully test-flown using E16 and F15 model rocket engines. The current phase of development focuses on replacing the standard nosecone with an autonomous glider capable of piggyback deployment, drawing inspiration from vehicles like the [Spiral space plane](https://russianspaceweb.com/spiral_development.html) and the [X-37B glider](https://en.wikipedia.org/wiki/Boeing_X-37).

![4th glider prototype composition](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-4%20with%20electronics.jpg)

## Limitations

To optimize weight distribution in future iterations, `Coludo` may eventually transition into a single-stage glider. However, the current airframe is strictly limited to a payload capacity of 100–150 grams, as documented in the [hardware components specifications](../doc/hardware.md) where everything fits under 100 grams. The video recording and telemetry recording module was implemented separately with its own power supply.

This rigid weight constraint severely limits the onboard power supply. The current physical envelope can accommodate a 800 mAh single cell LiPo battery with a power booster or a LiPo 6F22 9V battery paired with a power down regulator. To avoid any power shocks controller and engine regulators must be separated. Consequently, the maximum target power consumption for the electronics suite must remain under 3.5W (approximately 5V @ 700mA). To achieve this safely, a high-efficiency 5V switching regulator (such as a UBEC or buck converter) must be used; a traditional linear [LM7805 voltage stabilizer](https://www.amazon.com/dp/B00LTQTZYQ) cannot reliably handle the transient current spikes drawn by the control servos without inducing thermal shutdown.

The software architecture relies on MicroPython to manage this hardware stack, prioritizing cooperative multitasking via `asyncio`:
* **Hardware Constraints:** The weight and power limitations restrict the primary flight controller to the [eSBC esp32-p4](https://wiki.dfrobot.com/FireBeetle_2_ESP32_P4_Development_Board_IO_Expansion_Kit) development platform.
* **Language Choice:** While a compiled C/C++ codebase offers raw execution efficiency, MicroPython is preferred due to rapid prototyping familiarity.
* **Concurrency:** To circumvent MicroPython's single-thread affinity, [asyncio](https://github.com/peterhinch/micropython-async) is heavily utilized. This ensures non-blocking cooperative multitasking, supplemented by hardware interrupts and selective multi-threading to leverage the ESP32's second core where necessary.
* **Phased Rollout:** Phase one focuses entirely on validating sensor fusion and telemetry acquisition to guarantee data integrity. Active control surfaces and motorized actuation loops will only be enabled after these telemetry baselines are proven in flight trials.

There is some problems with micropython in general (measured on the target board — see the
[benchmark findings](../doc/benches/WaveShare_esp32p4-micropython-findings.md)):
- GC pauses scale with live-object count on the PSRAM heap: ~0.3 ms on a clean heap but
  ~67 ms with only 10k small live objects — far beyond the control-loop budget.
- `asyncio.sleep_ms()` quantises to the ~10 ms FreeRTOS tick, so cooperative scheduling tops
  out near 100 Hz; a 5 ms / 200 Hz loop is **not** achievable via `asyncio.sleep`.
- The whole GC heap is in PSRAM (~12 MB/s memcpy) and `bytearray` slice-assignment is
  O(buffer length), so large preallocated buffers must be written with `struct.pack_into`.
- Asyncio is cooperative — if one task yields late, the whole system lags.
- Servo updates, GNSS parsing, and IMU callbacks all compete for time.

There are several improvements planned to mitigate those problems:
1. The 32 MB PSRAM lets the heap grow so GC runs less often — but each collection is slower
   (PSRAM is slow), so allocations on hot paths are minimised regardless.
2. gc.collect() is called before critical moments and GC is disabled during the flight.
3. The sub-10 ms control loop is paced by a hardware timer (+ `ThreadSafeFlag`) or a busy-wait
   on `ticks_us()`, not `asyncio.sleep`; if needed it moves to its own core thread.
4. If that is not enough, native code is used for the flight controller and servo control.

### Garbage collection in flight — implemented (perf cluster)

The GC policy above is implemented in `tasks/sequencer.py`, gated behind the stage machine:

- **At BOOSTING** (launch detected): `gc.collect()` compacts and frees the heap, then `gc.disable()`
  — the whole **airborne phase runs with GC off**, so no collection pause (≈0.3 ms clean, *tens of ms*
  on a full heap — far past the 10 ms control budget) can stall a 100 Hz control slice.
- **At DONE** (stationary, on the ground): `gc.enable()` + `gc.collect()`. The re-enable is deliberately
  **not** at LANDING: that transition fires at `< land_agl_m` (≈5 m) and possibly mid-flare, the worst
  place to pay a tens-of-ms stall — it would be wrong to fly the whole descent and then crash on a GC
  pause at the end. GC is held off through the flare and the collect is paid only once stopped.

Disabling GC for the entire flight is only safe because the hot paths are near-zero-alloc: the mixer
pre-resolves its surfaces and rewrites a shared output dict (, ~0 bytes/call), and the flight loop
caches the landing-zone steering heading at GPS cadence instead of running `navigation.steer()` trig
(~174 µs) every 100 Hz step. The **PID is fixed-point** (`pid.py`): every MicroPython float `*`/`+`/`/`
boxes a heap float, so the old float PID leaked a **measured 176 bytes/step** — ×3 axes ×100 Hz ≈
**56 KB/s** with GC off. Rewritten in integer millidegrees (error) / integer-ms (dt) / millidegree
output, a step measures **0 bytes** even at a ±180° heading swing (products stay under the RV32 2³⁰
small-int ceiling, so nothing promotes to a 16-byte mpz); the only residual is the isolated
`int((setpoint−actual)·1000)` float conversion at the sensor boundary (~16–32 B/axis, and 0 for yaw
whose heading error is already integer). **Net ≈ 47 KB/s off the leak.** Telemetry rows likewise
precompute a single `%`-format string (no per-row generator/join).

Measured on the ESP32-P4 with GC disabled, so the allocation delta *is* the in-flight leak
(`test/bench_pid_alloc.py`):

| per PID step            | float `Pid.step` | fixed millidegree | fixed centidegree |
| ----------------------- | ---------------: | ----------------: | ----------------: |
| error = 5°              |          176 B   |             0 B   |             0 B   |
| error = 180°            |          176 B   |             0 B   |             0 B   |
| ±180° swing (worst D)   |          176 B   |             0 B   |             0 B   |

| per-axis error conversion (runtime floats) | bytes |
| ------------------------------------------ | ----: |
| float today: `setpoint − actual`           |  16 B |
| fixed: `int((setpoint − actual) × 1000)`   |  32 B |
| yaw: `heading_error × 1000` (already int°) |   0 B |

Millidegrees are alloc-free even worst-case, so we keep full 0.001° resolution (no need to drop to
centidegrees); the float PID boxes 176 B regardless of magnitude. On-board HITL flights — current
firmware, which also streams the
simulated sensors, so it churns *more* than a bare run — measured **~15 MB consumed with GC off, low-water
~17 MB free of ~32 MB** on a ~47 s F15-4 flight (the shorter ~32 s E16-4 bottoms out ~23 MB), with
`mem_free` snapping back to ~32 MB at touchdown when GC re-enables. The full sawtooth is visible in the
committed device `board_health.csv` (`doc/sims/TMS-7-guarded_fins/`); the pre- and post-flight collect
durations are also **logged** (`gc pre-/post-flight collect <us>`).

Side effect: the CPU-load probe in `tasks/board_health.py` was changed from a `sleep_ms(0)`
busy-spin to a sleeping probe that measures wake-up lateness — the core now idles between samples,
cutting draw markedly (measured **7.2 W → 3.6 W** with all servos active) at no loss of the load signal.

Measured board vitals from those on-board HITL flights: MCU temperature **31–33 °C** (steady, far below
the synthetic ~45–63 °C the host sim had assumed), and CPU **load 0–~50 %** cruising with the peak at the
landing stage (the laser hammering I²C) — the single P4 core is mostly idle between 50 Hz steps, so there
is ample headroom. The asyncio loop runs **~3× faster than the 50 Hz sim rate** under this load, which is
why `tasks/hitl.py` drives the model from a wall-clock accumulator rather than a fixed dt (see Phase-5).

## Flight envelope (E16 / F15 — measured masses)

Numbers to seed modelling and the HITL simulation (Phase-5), and to sanity-check sensor ranges and
the launch-detect threshold. Derived from the **measured TMS-7 v4 masses** (`models/TMS-7` README):
the **booster with the motor** (ejects at separation) — E16 **185 g**, F15 **201 g** — and the
**glider** (structure 115 g + electronics 155 g) **270 g** as built, **215 g** the light build. So
the full stack is **455 g (E16) / 471 g (F15)** at liftoff (400/416 g light), and the *glide* runs on
the glider alone. See [`hardware.md`](../doc/hardware.md) + [`models/TMS-7`](../models/TMS-7).

The v4 remeasure (glider −15 g, booster +20 g — all within the ±10 g build tolerance; wing geometry
unchanged) shifts the derived envelope below (a v3-mass HITL + analytic baseline) only marginally;
`config_hitl` already carries the v4 masses, and a v4 re-sim will refresh the device numbers.

**Boost assumptions (deliberately crude):** vertical launch, no wind; constant *average* motor
thrust over the burn with propellant mass burning off linearly; drag `F = ½·ρ·v²·Cd·A` with
sea-level `ρ = 1.225 kg/m³`, `Cd ≈ 0.6`. Frontal area from a **~46 mm effective diameter**
(`A ≈ 17 cm²`): the booster tube is only ~40 mm (motor 29 mm + holder + a ~3 mm ultra-light-filament
wall), and the glider rides on top at ~46 mm body width — the 143 mm in the model bbox is the
*deployed wing/fin span*, which is thin and edge-on to the airflow so it adds little frontal drag.
The **glide** rows are no longer an L/D guess: they are the fly-long trim law measured on-board
(HITL, `doc/sims/TMS-7-warm_reboot`) over the polar pinned at **air quality 2** — the worst case
(−7 m/s trim sink), so every glide duration is a FLOOR; the real airframe is expected at
quality 4–6 and the polar re-calibrates from the first real glide telemetry.

Numbers are the **full 270 g glider** (stack 455/471 g); the **215 g light build** climbs higher —
E16 **161 m**, F15 **345 m** apogee measured (159/357 m analytic) — and flies longer (40.5/71.3 s
at the quality-2 floor). The `sim_model` integration of thrust − drag − gravity matches the analytic
model to a few percent at the v3 masses too (E16-full 128 m analytic vs 130 m device, F15-full
304 vs 292 m).

| Parameter | **E16** | **F15** |
|---|---|---|
| Liftoff mass (booster + 270 g glider) | ~455 g | ~471 g |
| Total impulse / burn | 28 N·s / 1.8 s | 50 N·s / 3.5 s |
| Peak accel — accelerometer reads (specific force) | **~7.8 g** (peak thrust 33 N) | **~5.8 g** (peak thrust 25 N) |
| Early-boost — accelerometer reads | ~3.5 g | ~3.1 g |
| Peak speed (at burnout) | ~44 m/s (~160 km/h) | ~68 m/s (~245 km/h) |
| Apogee (vertical) | ~128 m @ ~6 s (130 m device) | ~304 m @ ~9.3 s (292 m device) |
| Flight time BOOSTING→DONE, quality-2 FLOOR (device) | 35.2 s | 62.9 s |
| At the quality-5 calibration (device) | — | 101 s / 1.42 km air path |

**What this means for the design:**
- *Accelerometer:* it reads **specific force** = kinematic acceleration **+ 1 g**. Peak is only ~6–9 g,
  so even the BNO055 (±16 g) would not clip — but the ADXL375 (±200 g) stays the boost source for
  headroom/noise margin (clones and the airframe vary; a spike can exceed the published peak).
- *Launch detect:* the accelerometer reads **specific force = thrust/mass** — at the v4 masses
  early boost is E16 (455 g) ≈ 3.5 g, F15 (471 g) ≈ **3.1 g**. `launch_g` stays **2.5**: sized for
  the heavier v2 stack (F15 read 2.84 g), it keeps ~20 % margin below the lightest real boost, plus
  an independent **`launch_alt_m = 10 m`** backup — the baro climbing 10 m off the pad trips
  BOOSTING regardless of the accel threshold, so a heavy/marginal boost or a dropped accel window
  still detects. The passive E16/F15 flights tune both from real data.
- *Mission profile:* the F15 more than **doubles apogee** and gives **~2×** the glide window — the
  better motor for exercising active control once the passive flights validate the data pipeline.
  Under the fly-long law even the E16's 130–160 m apogee is enough to reach the 200 m
  landing-zone gate (`max_range_m`): the E16 landed IN-ZONE on the quality-2 HITL floor
  (77 m full / 13.5 m light from the centre), where the old dive-at-the-midpoint law fell
  150–220 m short.

**Cross-check — on-board HITL (Phase-5).** `sim_model` integrates the same thrust/mass/drag, and
the on-board HITL flights land where this table predicts at the boost end — v3 device apogees
130/161 m (E16 full/light) and 292/345 m (F15) vs 128/159 and 304/357 m analytic — the sim is
calibrated to the envelope's drag model (Cd 0.6, A ~17 cm²), not tuned separately. The glide rows
are the same captures (fly-long trim law over the quality-2 worst-case polar); the high wing
loading in the airframe notes below is why the polar is modelled fast and steep rather than L/D-5.
Traces + reports: [`../doc/sims/TMS-7-warm_reboot`](../doc/sims/TMS-7-warm_reboot).

### Airframe notes from the printed models (`models/TMS-7`)

Measured from the meshes (bbox / volume): glider 394 (span) × 121 (folded height) × 388 mm (length);
booster tube ~40 mm (motor 29 + holder 37 + ~3 mm ultra-light wall) with the wing/fin span reaching
143 mm when deployed. Wing (each): 222 span × **2.0 mm thick** × 67.7 mm chord, ~62 cm² planform.
Solid-PLA volumes are 2–3× the measured part masses → ~30–42 % effective infill, consistent with
[`hardware.md`](../doc/hardware.md). Geometry is **symmetric** (L/R wings and fins identical) — good for
roll/trim balance. Structural / aerodynamic items to weigh before the active-control flights:

1. **Wing loading is high** — ~124 cm² total wing for the measured 270 g glider ≈ **~22 kg/m²**
   (light build 215 g ≈ 17 kg/m²), giving a stall of **~18–19 m/s** (CL_max ~0.9 for a flat plate).
   The glider therefore has to glide *fast* (~20–25 m/s) and lands hot. **Bigger wings (1.5–2× area)** are
   the single most impactful change — they drop the stall speed, make the glide controllable and the
   landing flare survivable, and improve the realistic L/D.
2. **Wings/fins are 2.0 mm flat plates** — over a 222 mm span this flexes/flutters, is fragile, and
   gives poor L/D with early stall. Thicken to 3–4 mm or use a thin cambered airfoil with a spar.
3. **Boost stability is unverified** — the booster has no dedicated fins; with the wings folded,
   confirm the glider's tail fins protrude enough (or add transient boost fins) so the boost-phase
   centre-of-pressure sits ≥1 caliber behind the CG, otherwise it weathercocks/tumbles off the rod.
4. **CG forward for pitch stability** — the glide CG should be ~25–35 % MAC (ahead of the aero centre);
   place the battery / LuckFox / heaviest electronics in the Front Lower Body (nose) and verify on a
   balance (the model geometry alone cannot fix the CG — it depends on the electronics layout).

# Lifecycle

The operational lifecycle of the glider is brief and divided into four distinct phases:
* **Setting:** Ground initialization, sensor calibration, and vertical pre-launch staging.
* **Boosting (Active Stage):** Powered ascent of the combined booster-glider stack along a near-vertical trajectory.
* **Gliding (Passive Stage):** Triggered at apogee when the rocket motor's built-in black powder ejection charge fires (following a designated 4–6 second delay after burnout). The resulting internal pressure forces the glider clear of the booster, initiating autonomous wing deployment and navigation back to the designated landing zone.
* **Landing:** The final approach matrix where the glider flares, stabilizes horizontal velocity, and touches down.

## Setting

The Setting phase begins at system power-on and terminates immediately upon engine ignition (transition to Boosting). The expected ground pad duration is approximately 15 minutes.

Upon electronic initialization, the following sequential operations are executed:
* **Physical Orientation:** The airframe must be kept horizontal and oriented toward true North for baseline indexing.
* **Status Indication:** The main power LED is set to flash at a slow 2 Hz cycle (250ms ON / 250ms OFF).
* **Object Instantiation:** The MicroPython environment initializes all core software components and drivers.
* **Calibration:** The system zeroes out the altimeter, digital compass, accelerometer, and gyroscope while performing a full deflection check of the fin servos.
* **Network Connectivity:** The board joins the Control Center's Wi-Fi network as a **station** (see [`board-config.md`](board-config.md)) and establishes a connection with the ground control station (PC) to facilitate remote diagnostics and real-time monitoring.
* **Recorder Link:** If the Recorder module is present, the UART telemetry/log sink is opened (the controller has no local SD card; the Recorder owns video and storage).
* **GNSS Lock:** The GPS module begins polling at 1 Hz to acquire a multi-satellite 3D fix. The coordinates of the target landing zone must fall within a 200-meter threshold vector relative to the launch point. System time is automatically synchronized to the GPS atomic clock.
* **Validation:** The Flight Controller polls all subsystems. If all validation gates pass, the LED status changes to a "Ready" heartbeat pattern (100ms ON / 900ms OFF).
* **Staging:** The vehicle is cleared to be mounted vertically on the launch rail.

Potential problems:
- GNSS accuracy during dynamic flight is often ±5–15 m.
- At low altitude, multipath error increases.
- A 200 m zone is fine, but your boundary‑tracking logic assumes much better accuracy.

What will happen - The glider will:
- “hunt” for the boundary
- overshoot repeatedly
- oscillate between entry points
- possibly exit the zone unintentionally

To fix these issues there are some possible workarounds:
1. deadband logic (don’t correct unless error > X meters)
2. heading‑only navigation (ignore lateral error when close)
3. wind compensation (IMU + GNSS drift vector)

## Flight Controller

Controller is the main component which:

- creates all required Components
- keeps track of the rocket's current Stage { Setting, Boosting, Gliding, Landing }
- get Landing Zone coordinates, understand TargetPoint and landing parameters e.g. distance from start to TargetPoint must be nearby to LaunchPoint e.g. 200m
- controls Components' states and gets async feedback for course correction
- if the Flight Controller crashes the async loop should be restarted

Main maneuvers of the Controller depend on the current stage. The main goal will be to prevent overcorrection which will be achieved with the use of a Proportional-Integral-Derivative gains control algorithm (PIDgca). The idea previous about the Feedback Multiplier (Proportional Control) won't work since the glider will wildly overcorrect causing severe rocking or even the risk of stall. Thus it would be nice to have an IMU which gives position, speed, and acceleration.

Examples:
- yaw shows direction 30 degrees right, in this case vertical fin needs to be turned 15 degrees left or however much the PIDgca states
- pitch shows nose down 10 pitch (or -10) degrees off, in this case horizontal fins must be turned down by 5 degrees (or -5) to push for resulting zero
- roll shows 25 degrees right, so fins should be twisted for 13 degrees but left down and right up.
- this is done to prevent overcorrection caused by potential communicational latency between components and sensors.
- This feedback() function could be smarter depending on air speed, sensor data quality and delivery delays, if software works fast then factor over 1 will allow to stabilize faster but with some extra G-factor.

## Controller Setting maneuvers

After pre-start internal things and testing fins rotations Controller should fix all fins into a "zeroed position" (angle=0) and check sensors for engine ignition.

## Controller Boosting maneuvers

The main point of the software is to keep glider and booster strictly vertical:

detect inclination and turn all fins to some direction to make vertical fin orthogonal to incline surface
as the vertical fin points to incline on angle A (glider collapses on top) turns horizontal fins down to opposite direction
OR when vertical fin points to opposite inclining on angle -A (glider falling on bottom) turn horizontal fins up to opposite direction
During Boosting phase as speed is not high probably it is probably best to keep the feedback factor > 1

## Controller Gliding maneuvers

Not many different evolutions are required:

if glider after separation happened to be upside-down the left or right half-roll is required using all 3 fins pointing into opposite direction to roll
direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
left turn when vertical fin turned right to target direction and left/right fins assists (or just keep horizon level)
right turn when vertical fin turned left for up to target direction and left/right fins assists (or just keep horizontal)
Aggressive turns will be permitted only over minimal controllable speed.

## Controller Landing manoeuvres

Final step when coming to LandingZone or nearby on low altitude, no time for manoeuvres except keep going with minimal corrections:

direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
small corrections to left turn when vertical fin turned right to target direction and left/right fins keep horizont
right turn when vertical fin turned left for up to target direction and left/right fins keep horizont

## Fin authority — dynamic-pressure limiting

Each of the three fins is driven **individually and directly by its own SG90 servo — there is no transmission, linkage, or reduction gearing** (no screw-gear or rack: those add mechanical complexity, backlash, and weight the airframe cannot spare). The fin angle *is* the servo angle, mapped 1:1. The upside is mechanical simplicity and one fewer failure mode; the cost is that nothing mechanical limits the throw, so the deflection limit must be enforced in **software**.

This matters because aerodynamic torque on a fin scales with **dynamic pressure** `q = ½·ρ·v²`, i.e. with the **square of airspeed**. The *same* fin angle produces ~`v²` more torque at high speed: a deflection that is a gentle nudge at 14 m/s is a violent, stack-flipping moment near burnout. A fixed angle limit is therefore wrong at one end or the other — too weak to control at low speed (where the fins are also aerodynamically soft and most authority is needed), too violent at high speed (risking loss of control, servo stall, or shearing a fin off).

The controller instead **schedules the maximum fin deflection by airspeed** to hold roughly *constant angular authority* (deflection ∝ 1/q ∝ 1/v²):

```
deflection_limit(v) = clamp(K / v², 5°, 45°) × fins.limit_multiplier     (K ≈ 12500, anchored at 50 m/s → 5°)
```

| airspeed (m/s) | ≤16 | 20 | 25 | 30 | 35 | 40 | 45 | ≥50 |
|---|---|---|---|---|---|---|---|---|
| max fin deflection | 45° | 31° | 20° | 14° | 10° | 8° | 6° | 5° |

* **Flight-wide, not boost-only.** Ignition can end off-vertical and fast, and a glide can build speed in a dive, so the governor caps the fins through **every** stage (boosting, gliding, landing). It is the final actuator clamp, applied after the per-stage control law and the mixer.
* **45° floor of authority at low speed** (≤16 m/s): low `q` means weak fins *and* low kinetic energy = low risk, so full mechanical throw is allowed — the "trade speed for altitude with gentle uplift" regime.
* **5° ceiling of restraint near burnout** (≥50 m/s): high `q`, highest consequence — but never 0°, so some authority always remains.
* **`fins.limit_multiplier` (board.config, default 1.0)** scales the whole schedule. It is the safety dial: if a flight starts **losing fins or control in the air** (flutter, servo stall, structural failure), drop it (e.g. 0.5) to halve authority everywhere without re-deriving the table.
* Stored as a **precomputed lookup table** indexed by integer m/s, so the 100 Hz path does a table read, not a `1/v²` division.
* **Airspeed** comes from a **pitot tube**: the SDP810-500Pa differential-pressure cell is the rank-0 primary source (`airspeed_sdp810`), band-gated at both ends — below `pitot_min_ms` (3 m/s ≈ 5.5 Pa) the reading is tare noise, above `pitot_max_ms` (28 m/s) the ±500 Pa cell rails and *under*-reads, and an under-read loosens the cap. Outside that band, and whenever the cell is absent or blocked, the fusion falls back to the **rank-1 estimator**: integrated vertical acceleration during boost, GNSS ground speed once gliding. That estimator is biased to *over*-read when uncertain — over-estimating airspeed tightens the cap, the safe direction. Measured on the board, the pitot and the governor agree to ~0.2 m/s in glide.

During boosting the **wings are folded inside the booster body tube** (rubber-band deployed to the flight position only after separation), so the fins here steer the slim *booster + folded-glider stack* — the CG and ~146 mm fin moment arm in the `models/TMS-7` analysis are the stack's, which is why the boost-torque numbers come out as they do. After separation the same fins control the deployed glider.

### Servo torque — why direct-drive SG90 is enough (and why the cap is not about torque)

Each fin is an **all-moving surface, 37 cm², 58 mm chord** (span ~64 mm, aspect ratio ~1.1), hinged near its aerodynamic centre (~25% chord), driven **directly (1:1) by an SG90** — no reduction gearing, so the servo carries the aerodynamic hinge moment one-for-one. A real SG90 at 5 V gives **1.3–1.5 kg·cm = 128–147 mN·m** of stall torque.

Because the surface is near-aero-balanced (hinge ≈ CoP), the hinge moment is small and grows ~linearly with deflection and with `q`:

```
H ≈ 0.37 · δ[°] · (v / 30 m/s)²   mN·m        (anchored on ±45° / 30 m/s = 16.6 mN·m)
```

**In governed flight the servo is barely loaded — and flat across the envelope.** The deflection cap holds `δ·v² ≈ K`, and the hinge moment is itself ~`δ·v²`, so it stays near-constant at **~5 mN·m everywhere = 3–4 % of SG90 stall (~25–30× margin):**

| airspeed | governed δ | hinge moment | % of SG90 stall (1.3–1.5 kg·cm) |
|---|---|---|---|
| ≤16 m/s | 45° | 4.7 mN·m | 3.2–3.7 % |
| 30 m/s  | 14° | 5.2 mN·m | 3.5–4.0 % |
| 50 m/s  |  5° | 5.1 mN·m | 3.5–4.0 % |

So **torque is never the binding constraint** — the plastic-gear SG90, direct-driven, holds the governed fin cool, and the `1/v²` cap is precisely what keeps the moment tiny despite there being no *mechanical* limit on the throw. This is the key point: **the cap exists for control authority and structure, not for servo torque.** The schedule holds `δ·v² ≈ K ≈ 12500`, i.e. *constant angular control moment* — 45°/16 m/s and 5°/50 m/s deliver the same authority. Relaxing the high-speed end does not "recover lost authority" (there is none lost); it multiplies the control moment by the relaxation factor → over-control / stack-flip, the very failure the governor prevents.

**The one stress case is an un-governed hardover to ±45°** (a control fault, or relaxing the cap): there `H = 0.0185·v²` reaches the 128–147 mN·m stall at **~83–89 m/s** (attached flow). At large deflection the flow separates and the centre of pressure moves aft, **~doubling** the moment and pulling the stall speed down to **~59–63 m/s**. Burnout is ~70 m/s, so a 45° hardover *near burnout* sits right in the back-drive/stall band — a concrete, SG90-specific reason the high-speed cap must not be relaxed. At that hardover it is the output-shaft **bending** load, not torque, that bounds things, which is the only reason to prefer the metal-gear **MG90S** (~+3 g) — a shock/robustness choice, not a torque one.

## Boosting

The Boosting phase spans engine ignition through booster separation. While a zero-delay motor (like an F15-0) would trigger instantly, the operational profile utilizes motors featuring a built-in 4–6 second delay tracking element to coast cleanly to apogee:
* **Attitude Maintenance:** The airframe occupies a vertical stance on the launching rail. The Flight Controller dynamically monitors the pitch and roll axes to maintain a trajectory perpendicular to the local horizon.
* **GNSS Acceleration:** Upon detecting launch rail departure, the GPS module is programmatically escalated to a high-speed update mode (5 Hz or 10 Hz) to maximize spatial resolution during high-velocity ascent.
* **Dynamic Stabilization:** The Flight Controller actively manipulates the control surfaces to counteract wind shear and aerodynamic instability.
* **Separation Matrix:** At peak altitude, the motor's integrated black powder ejection charge fires, pressurizing the interior of the booster body tube. This pressure forces the glider upward and out of the booster. During the boosting phase, the glider’s wingtips are nested inside the booster's main body tube to hold them securely folded against aerodynamic drag. As the glider is pushed clear of the airframe, tension from rubber bands anchored at the front of the airplane automatically pulls the wings outward into their locked, deployed flight configuration. Concurrently, a dedicated separation loop—monitored via a physical pressure switch or a breakaway wire pulled from a flight computer socket—flags the physical separation event, outputting a digital logic change to instantly transition the software into Gliding mode.

### Boost disturbance rejection — simulated (7/06)

Host sweep of the guarded-fin boost hold (real `guidance`/`pid`/`mixer` over the sim, F15 v3
masses, 5 % noise): constant **weight-imbalance / thrust-misalignment torque** vs **steady
crosswind**, on both lean axes.

| disturbance | worst lean off vertical | verdict |
|---|---|---|
| imbalance torque 10 °/s², pitch (top-to-bottom) | 10.5° | **controlled** — the hold absorbs a strong constant bias |
| imbalance torque 10 °/s², roll (side-to-side) | 6.8° | **controlled** |
| crosswind 6 m/s (either axis) | ~30° | partially resisted |
| crosswind 12 m/s (either axis) | 61–67° | the hold loses — physics, not firmware |
| combined 9 m/s @ 45° + both imbalances | 43.6° | between the pure cases |

Readings:
* **Imbalance is a solved problem** — the controllable disturbance class stays within ~10° of
  vertical on both axes; pitch- and roll-axis behaviour is symmetric.
* **Strong wind is weathercocking**: tail fins are weathervanes — the same passive stability that
  keeps the stack straight aligns it into the relative wind, and disturbance and fin authority
  both scale with q, so the ratio never improves with speed. The response is OPERATIONAL, not
  control gains: **launch wind limit ≤ 6 m/s** (lean ≤ ~30°, the classic model-rocketry
  threshold) for the flight campaigns.
* **Model caveat**: the boost sim integrates altitude 1-DoF vertically, so its apogee is
  LEAN-BLIND (every case reads the same apogee). A real 60° lean at burnout costs roughly half
  the vertical impulse plus a long downrange arc — the 12 m/s rows UNDERSTATE the consequence,
  which argues the wind limit even more strongly. A lean-aware boost model is a known
  improvement for later.
* The stage machine deployed at apogee and landed in every case — robust to all disturbances
  tried.

### Separation dynamics — measured (TMS-7 v3 static burn, 7/03)

The first static burn fired the ejection charge with the **194.4 g glider** (v3 construction 134.8 g +
partial electronics) seated horizontally; the flight computer failed to power (battery → IO-shield
incompatibility), so these numbers are reconstructed **ballistically from the video**, not telemetry:
the ejected glider landed **1.5 m** downrange from a **0.8 m** height, and the wings snapped open
**0.5–0.6 m** after clearing the tube.

* **Separation velocity ≈ 3.7 m/s** (fall time √(2·0.8/g) = 0.40 s over 1.5 m; drag negligible at
  this speed). Ejection energy ≈ **1.3 J**.
* **Ejection acceleration 3.5–7 g** (stroke-length bound: v²/2L for a 0.10–0.20 m nested overlap),
  i.e. **7–13 N** on the glider over a **55–110 ms** stroke — a sharp but modest transient, well
  inside the ±32 g accel range and far from any structural limit.
* **Wings locked open ≈ 135–160 ms after exit** (0.5–0.6 m at 3.7 m/s). Control relevance: the fins
  are aerodynamically useless until the wings deploy, which is comfortably inside the boost-engage /
  attitude-recovery window — no software change needed, but the flight loop must not expect roll
  authority in the first ~0.2 s after the separation switch fires.
* **Flight-weight scaling:** the full Coludo glider flies heavier (~235–285 g with complete
  electronics). The same charge then separates at **≈ 2.5–3.4 m/s** (impulse- vs energy-conserving
  bounds) and the wings-open delay stretches to **≈ 150–240 ms** — still within the same control
  window, but the margin shrinks with mass: re-measure on the first full-weight burn.

## Gliding

**Glide objectives, in strict priority order (7/05):**
1. **FLY AS LONG AS POSSIBLE** — hold the trim attitude (field-measured; the pitch PID damps, never
   forces an off-trim descent) and spend altitude only through the orbit's banked turns.
2. **Land INSIDE the landing zone.**
3. **Land as close to the zone midpoint as possible.**

A lower-priority objective must never be bought with a higher one — the glider does NOT dive at the
midpoint to improve #3 at the cost of #1; the endgame steering (#2/#3) tightens only as the
altitude runs out.

**The glide law that delivers this (tuned 7/06, `guidance.py`):**
* **Travel** (far from the zone): steer to the nearer short-side gate, as always.
* **Loiter** (within `loiter_capture_m` = 120 m of the centre): the heading command becomes the
  CIRCLE TANGENT plus an inward correction (`bearing_to_centre + 90° − gain·(distance − R)`,
  R = 30 m, gain 3), so the glider CAPTURES a constant-radius orbit around the centre instead of
  bang-banging between overfly and U-turn (the old point-steer law swung 184 m racetrack legs and
  landed on phase luck). The ~26° orbit bank sits inside the cruise `bank_limit`; altitude bleeds
  through the turn at the induced-drag rate — this IS objective #1's energy management. R must not
  be set below the cruise-bank minimum radius (~34 m at 30°) or the orbit destabilizes.
* **Endgame spiral** (below `endgame_alt_m` = 50 m): the loiter radius scales with the remaining
  altitude fraction, collapsing the orbit onto the centre exactly as the energy runs out, with the
  full `land_bank_limit` 45° available (`land_bank_gain` 3.0 — at 1.5 the rotating-target P-loop
  saturated near 25° and the spiral froze at that bank's 44 m radius; 45° gives the ~20 m minimum
  the miss target needs).
* **Turn-radius limit — a physical restriction, not a tuning knob.** A coordinated turn holds
  `R = v²/(g·tan φ)`; at ~14 m/s trim the endgame's 45° land-bank gives **R_min ≈ 20 m**, so the spiral
  CANNOT collapse tighter and the touchdown is bounded to that circle (the measured 15–20 m floor; the
  residual miss is otherwise along the strip's long axis plus ~15 m of unactuated free-drift in the
  final descent). Guidance therefore CLAMPS the commanded loiter/endgame radius to `min_turn_radius(bank)`
  at the live airspeed — it never asks for a turn the airframe cannot fly (cruise clamps at `bank_limit`,
  the endgame at `land_bank_limit`) — and reports the landing R_min in `flight.vitals` (`r_min_m`) for a
  land-short-vs-stretch decision. Tightening it further needs a steeper AIRSPEED-GATED bank bounded by the
  turn-stall speed (`V_stall·√n`, load `n = 1/cos φ`): a separate program (5.2) that first needs a stall
  model added to `sim_model` — the current dynamics has no stall break, so a naive steeper bank would look
  free in HITL and lie about the real airframe. These aero limits (the 45° cap, the launch-wind limit, the
  sink polar) were intuitive first guesses; they are worth re-deriving from the measured TMS-7 mass / wing
  area / CL_max.
* **Final approach** (below `final_approach_agl`): the strip-centreline tracker, unchanged.
* **Steering noise filter** (all laws): the heading error runs through an all-integer EMA
  (`steer_filter_shift` 3 = alpha 1/8, τ ≈ 80 ms at 100 Hz; zero-alloc under GC-off) so per-step
  sensor jitter never reaches the bank command; a genuine target change (> 90° — an overfly flip, a
  law handover) resets the filter so steering follows at once. At 25 % noise this halves the fin
  travel (−56 %) and pulls the E16 touchdown in-zone.

Measured on the worst-case quality-2 polar (host sweep, 24 cases): time aloft 121–148 % of the
straight-trim ceiling in every case (objective #1 never regressed through the whole tuning);
calm/5 %-noise touchdowns **17–18 m from the centre, in-zone, both motors** (the untuned racetrack
baseline: 129 m median); with the steering filter the calm/25 %-noise miss is 17 m (E16, in-zone) /
37 m (F15) — median 31 m against the ≤ 30 m acceptance line, the residual being run variance for
field calibration; wind ≥ 6 m/s remains physics-bounded for a 14 m/s glider.

Following booster separation, the Gliding phase executes, maneuvering the aircraft toward the target coordinates:
* **Attitude Recovery:** The glider must immediately execute an pitch/roll correction to transition from a vertical posture to a stable, horizontal gliding envelope, maintaining a "top-fin-up" orientation using real-time gyroscope vectors.
* **Navigation Architecture:** A streamlined gliding approach minimizes processing overhead:
  * The system computes a vector pointing to the shortest boundary entrance of the rectangular landing zone.
  * A heading adjustment is initiated via the vertical stabilizer (yaw axis), while the horizontal stabilizers actively damp out unwanted roll and pitch variations to maintain a flat slip-angle.
  * Upon crossing into the designated airspace, the system constantly samples its track over the landing zone.
  * If the glider overshoots or exits the boundaries without ground contact, it recalculates a vector to the nearest alternative entry point and loops the logic pattern.
* **Glissade Descent:** This iterative correction loop continues until the barometric altimeter registers a low-altitude threshold, shifting execution into Pre-Landing mode.

## Landing

The Pre-Landing sequence triggers when the glider drops to 4-12 meters AGL (Above Ground Level) relative to the launch pad elevation and speed is vertical speed < −1.5 m/s and roll < 10°. The priority shifts from destination tracking to structural preservation:
* **Attitude Lock:** The flight surfaces lock into a straight-and-level attitude glide. All aggressive rolling, pitching, or yawing maneuvers are suppressed to ensure clean underbelly contact with the ground.
* **Data Logging Surge:** To capture maximum high-resolution structural and aerodynamic impact data, the telemetry and multimedia flush rates are boosted from 1 Hz to 10 Hz.
* **Touchdown Detection:** Ground impact is verified when horizontal/vertical velocities decay to near-zero margins and barometric altitude output stabilizes completely.
* **De-initialization:** Following a 5-second confirmation window of absolute silence, the flight is officially flagged as completed. All open data streams are flushed to the Recorder over UART (the controller has no local filesystem to unmount), and the controller puts the hardware into a low-power state via the ESP32 `machine.deepsleep()` API (the earlier `pyb.stop()`/`pyb.standby()` calls are pyboard-only and do not apply to the ESP32 port).

Horizontally (longitude) stretched landing zone
```
            |TL------------------------------------------------------------------------|
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
       ---> Entrance                    targetPoint                            Entrance <----
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |------------------------------------------------------------------------BR|
```

Vertically (latitutude) stretched landing zone
```
                       Entrance
                           |
            |TL------------_--------------|
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |         targetPoint         |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |--------------^------------BR|
                          |
                      Entrance
```

### Zone orientation — an operator safety decision

The two **gate entrances are fixed to the midpoints of the SHORT sides** — the navigator resolves them
purely from geometry (`navigation.steer()` always vectors to the nearer short-side entrance, then to the
centre; on an overshoot it turns ~180° back through the gate it just crossed and re-approaches). Two
reasons reinforce that placement, but only if the zone is laid out deliberately:

- **Aerodynamic:** entering along the long (major) axis gives the longest run-in to the target — lower
  effective crosswind and more room to settle before touchdown.
- **Safety:** the short sides are the glider's only approach corridors. **The operator must orient the
  zone — the TL/BR corners in `launch.config` — so the two short-side entrances point at hazard-free
  airspace.** The long sides should border the hazards (trees, the launch pad, people); the short
  sides the clear approaches.

This is a placement decision at mission-setup time, **not just a rectangle**: `navigation.py` will steer to a
short-side entrance on every approach and re-approach, with **no knowledge of what lies beyond any
side**. Choosing the corners so the gates align with clear corridors is therefore safety-critical and
entirely the operator's responsibility — the firmware cannot verify it.

Task creation orders and internal dependencies are explicitly hardcoded within the controller to keep execution logic simple and predictable.

## Degraded Mode

Incase of complete sensor faliure or other critical errors the degraded mode will be enabled:

- When IMU degraded/produced invalid data the glider must fly straight or minimize turns
- If GNSS is lost - glide in current heading, prioritize gentle descent

## Field operation without CC (design, 7/04)

The board must be able to power up, configure itself and fly with **no Control hub present** —
a phone hotspot or laptop is a convenience, never a dependency.

* **Site selection by on-board GPS only.** `launch.config` carries a list of known sites
  (`sites: [{name, pad: [lat, lon], zone: [[TL], [BR]]}, …]` — the zone list is small and fixed,
  "like Cape Canaveral"). At boot, the first GNSS fix selects the site whose pad is nearest,
  gated by `max_range_m` (200 m). On a match: that site's zone becomes the mission zone and the
  **live fix** (not the stored pad) becomes the launch point — kept live until **arm**, which
  FREEZES the fix as the persistent launch point (so the open-loop heading tier and the
  warm-start crumb's launch field survive a mid-flight fix loss; a CC-set position always wins).
* **Navigating without a fix — four tiers.** GNSS is EXPECTED to drop through the boost (high-g,
  vibration, antenna shadow) and may never reacquire inside a <60 s flight, so losing it is a design
  case, not an error path. `guidance` degrades in order: **(1)** a fresh fix inside
  `position_age_max_ms` steers normally; **(2)** no fresh fix but one seen earlier →
  **dead reckoning**, advancing the last fix by airspeed × heading plus the last wind estimate (every
  input survives a dead GNSS — pitot and fused compass), so the position keeps MOVING; **(3)** no fix
  ever seen → the open-loop launch-point bearing, since the pad is itself a known position;
  **(4)** no launch point either → hold the heading captured on entering control.
  Tier 2 exists because tier 3 does not degrade — it steers the pad→target bearing forever and flies
  over the target. Dead reckoning degrades gradually instead; the frozen wind estimate is the dominant
  residual error (it needs GNSS to update), bounded at roughly 80 m per 2 m/s of error over 40 s.
  `flight.csv`'s `reckoning` column records when the glider is flying on tier 2.
* **No site within 200 m → the spiral-landing fallback.** The mission SYNTHESIZES a GENEROUS box
  the spiral just has to land INSIDE — we always know the pad, so this trades objective #3
  (near-centre) for #2 (in-zone) and needs no tight midpoint: a `fallback_width_m` (100 m, the
  WIDE side facing the pad — a broad left-to-right entrance) × `fallback_depth_m` (90 m) box, its
  near edge `fallback_near_m` (50 m) out from the pad (people stand there), so the CENTRE sits
  ~95 m off at `fallback_bearing_deg` (the operator points it at the clear sector). The existing
  bank-to-turn law orbits that centre bleeding altitude — the spiral landing emerges from the
  normal overshoot-orbit behaviour with zero new control code, and the ~55 m turn-radius-limited
  endgame miss fits the ±50 m box. Axis-aligned (an emergency orbit cares about the centre, not
  orientation). The field brief must keep the fallback sector clear.
* **Arming without CC** — `auto_arm` in launch.config (default OFF): arm once GNSS has a fix AND
  the board has been stationary (|a| ≈ 1 g sustained) for `auto_arm_dwell_s` (60 s) after boot.
  The long dwell makes a bench arm unlikely; the flight loop's control-stage gating still holds
  the fins neutral on the ground either way. CC `arm` keeps working when a hub is present.
* **Wi-Fi policy** — the `wifi` config gains `policy: auto | disabled` (distinct from the radio
  `mode: sta` key) and a `networks:` list (several SSIDs). `auto` retries every `retry_ms` (10 s
  default), alternating through the list one candidate per attempt; scanning STOPS at BOOSTING
  (no reconnect churn under GC-off) and RESUMES at DONE (the recovery-crew hotspot). `disabled`
  keeps the radio off for the whole session.

## In-flight reboot & warm start (design, 7/04)

Today a watchdog (or any) reset mid-air boots into SETTING with neutral fins — ballistic. The
warm start restores GLIDING within one boot (~2–4 s ≈ 20–40 m of lost altitude — expensive, but
against a guaranteed lawn-dart).

* **Breadcrumb in NVS, never a file.** A VFS write mid-flight locks the scheduler and wears the
  data flash; `esp32.NVS('coludo')` commits a few small key/values in milliseconds to the
  dedicated NVS partition. Written ONCE at BOOSTING entry (on the rod, next to the pre-flight
  `gc.collect()` we already pay): `flight=1`, launch fix (2× i32, deg×1e7), the active zone
  (4× i32), pad baro altitude, boost RTC stamp. Cleared (`flight=0`) at DONE and on orderly
  finish.
* **Warm-start gate at boot — ALL of, defense in depth:**
  1. NVS `flight == 1` (we were airborne when the reset hit);
  2. the **separation switch reads SEPARATED** — the physical latch no software state can fake
     (post-separation it stays LOW for the whole glide);
  3. baro ABSOLUTE altitude reads ≥ ~15 m above the NVS pad altitude;
  4. `machine.reset_cause()` is **WDT/SOFT/HARD** — a battery insertion or power switch reads
     PWRON, which is exactly what a RECOVERY CREW's hands do to a glider that crash-landed on a
     rise above the pad (where gate 3 alone would pass). A mid-air brownout also reads PWRON and
     stays cold — a browning-out battery cannot be trusted to finish the glide anyway;
  5. the **crumb age** (RTC now − boost stamp) is positive and < ~10 min. The RTC survives
     soft/WDT resets, so the arithmetic holds exactly when a warm start is legitimate (even an
     unsynced RTC — continuity matters, not absolute truth); a power cycle restarts the RTC and
     breaks it → cold.
  The breadcrumb is CLEARED at DONE (the stationary |a|≈1 g detect / the RSO timeout — not zero
  speed or zero elevation, which are unreliable on the ground) and by any rejected warm start, so
  the next boot is unambiguously cold.
* **Warm-start actions:** restore mission zone + launch point from NVS → stage := GLIDING →
  arm → `gc.collect()` + `gc.disable()` (the sequencer's BOOSTING hook was skipped) → the flight
  loop engages and re-captures the heading hold from the live attitude. The RSO
  `flight_timeout_ms` keeps bounding the restored flight (its clock restarts at the warm start —
  acceptable: the backstop stays bounded, just re-based).
* **Any gate missing → normal cold boot** in SETTING, breadcrumb cleared, event logged.
* **Validation:** HITL flight with a forced `machine.reset()` mid-glide (and a pulled USB on the
  bench): the board must come back armed, in GLIDING, steering to the same zone.
* **Measured — the in-flight OOM soak (7/06, `tools/oom_soak.py`):** a HITL glide ballasted to
  566 KB free hit a REAL mid-glide OOM (GC-off burn ~140 KB/s with the sim's own churn on top of
  the ~15–18 KB/s control-path leak). What actually happens at exhaustion: the asyncio runtime
  dies wholesale (every task supervisor aborts) — the watchdog TASK dies with it, so the graceful
  stall-detect path never runs, and the crash→neutral `finally` cannot execute either: **the fins
  freeze at the last commanded deflection** (~1.4 s from the last servo write to the reset), then
  the STARVED hardware `machine.WDT` panics the chip (`rst SW_CPU_RESET`, `reset_cause 3` = WDT).
  main.py then ran the five-signal gate against the genuine WDT cause and correctly REFUSED on
  the bench (`separation switch reads nested`), cleared the crumb, came up cold, rejoined the
  wifi and the CC hub. So the recovery chain is proven with one amendment to the outage model:
  the ~1.4 s pre-reset segment flies at the last banked deflection, not neutral — the backstop
  behind the backstop (hardware WDT outliving the watchdog task) is what carries the reset.
* **The memory-rescue layer (7/06, `board_health`):** the in-flight GC disable buys
  *predictability* (no pause the control loop did not schedule), not abstinence — an explicit
  collect at a known-safe moment is legitimate. Because the GC-off leak is *garbage*, the
  vitals task defuses the OOM before it lands — re-firing every health period for as long as the
  trigger holds (a persistent leak gets a collect per second, altitude allowing). The decision is
  physics, not a byte threshold — collect when memory dies before the flight is safely over: predicted **`oom_s` < 2 ×
  `land_s`** (time-to-exhaustion from the memory-decay slope vs time to sink to the rescue floor
  from the elevation-decay slope; no descent trend yet → no rescue — the glide always
  descends, so `land_s` exists exactly where a rescue is meaningful), with **proven safe
  altitude** (known elevation above `rescue_agl_m` = 10 m ≈
  2× the 5 m landing gate — a 0.2 s pause costs ~2 m), in BOOSTING/GLIDING only, never LANDING.
  The collect is bracketed by watchdog `kick()`s (it is atomic and unfeedable, so it starts on a
  full WDT budget). Both predictions ride `health.csv` + `inspect health` — the operator's OOM
  countdown and landing countdown. All-integer bookkeeping (cm, bytes/s, whole seconds).
  **Measured pause costs:** ~65–260 ms on a mostly-free heap (the real anomaly-rescue case — the
  trigger fires early, while collects are still cheap) but **3.4 s on a ballast-full 32 MB
  heap** — which is why `wdt_timeout_ms` stays 1000 (500 killed the rescue in HITL) and why a
  rescue near true exhaustion may still lose to the watchdog: the reset + warm-start chain below
  remains the layer behind it. **Validated on-board (the OOM soak re-flown, watchdog off):** the
  same ballasted scenario that hard-panicked the board now lands — 8 rescues, each logged with
  its decision pair (`oom 58s, land 58s` narrowing to `22s/12s`), the sawtooth visible in
  `mem_free`, rescues standing down at LANDING per the gates, flight to DONE.

## Sensors and Interrupts

If a hardware component can be linked to an interrupt, that interrupt must be utilized to reduce reaction time. Of course, interrupts must be properly connected to asyncio as specified in guides

## Sensors Fusion/Backup

To have reliable control over parameters the sensor data fusion must be implemented e.g. based on priorities and timeouts: if several sensors are available and they produce the same kind of data the best should be used until not the data isn't outdated, otherwise backup sensor(s) should be selected. If more advanced technique is available it could be applied as well.

Example: for altitude the queue of selection could be the following

- main sensor ICP-10111 timeout 100 ms (e.g. for drop speed 10m/s and granularity 8.5cm => ~10 samples per meter)
- backup sensor 1 could be BMP280 timeout 200 ms
- backup sensor 2 could be accelerometer
- backup sensor 3 could be navigation - it has rate 10 Hz/100ms but real elevation data update ~10m which leads to 1 second to timeout

Proper cross-analysis for initial fusion (backing) should be performed by documentation and can be tweaked later after trials. Sensor disagreements will be handled during timeouts and limits per each individually and switching to backup sensor. For example, the controller expects GPS data every 100 ms and if there is no data or repetative data for at least 200 ms then it will switch to the IMU.

**Attitude backup (implemented 7/07, `tasks/attitude.py`).** `attitude` is the one quantity with a
single hardware source — the BNO055 (fused 9-DoF, priority 0). Losing it mid-flight would hand the
control loop stale/absent attitude → neutral fins → ballistic, so a **complementary-filter backup**
derives (heading, roll, pitch) from the LSM6DSO32 gyro `rate` + accel gravity vector and provides it
at **priority 1**; the databoard's timeout handoff (40 ms) then swaps to it automatically the moment
the BNO055 stops — the same priority/timeout mechanism as every other fused quantity. It **mirrors**
the BNO055 while that is fresh (warm, so the handoff has no transient), and **free-runs** only once
the BNO055 is lost: gyro integration for the short-term motion, with the accel gravity vector
re-anchoring roll/pitch (drift-free) — but **gated off in turns** (`turn_gate` on the yaw rate),
because in a coordinated turn the accelerometer reads gravity+centripetal down the body axis and
would look wings-level at any bank; there the gyro carries the true bank. Heading (yaw) has no
magnetometer, so it gyro-integrates — but when moving it is pulled toward the **GNSS ground-track
bearing** (`course`, a weak `course_shift` blend gated on ground speed), an absolute reference that
bounds the drift; and the track is what the nav steers by anyway, so a crosswind crab averages out.
So nav heading degrades gracefully while roll/pitch stay
solid and the glider holds bank + pitch. All-integer (`fixed.atan2_cd` + `isqrt` + `blend_cd`, viper,
no float boxed but the heading the channel requires); the accel gravity vector feeds the CORDIC at the
control's centi-fixnum scale (via `from_float`, one scale everywhere — no separate milli type), which
costs **~0.5° typical / ~1.8° worst** attitude vs a finer ×1000 scaling's 0.16° — a deliberate trade:
the error is a *bounded bias* (it settles, does not accumulate — each `atan2` is independent and the
filter re-anchors), it is re-synced by the accel anchor + BNO055 recovery, and 1.8° is well inside both
the BNO055's own ~1–2° and the ≤300 m / ≤1500 m flight envelope. Validated closed-loop on the board
(`tools/attitude_soak.py` drops the sim attitude mid-glide): the backup tracks truth to ~1° roll (×1000)
/ ~1.8° roll (×100) and ≤0.7° pitch through the loiter, flying the descent to a controlled landing.

## Tasks

One task must be created explicitly - the Controller, it creates the rest of the tasks which are located in the tasks folder and support some common API e.g.:

- setup() - async call to make initial task activation (or reset) and returns True or False
- run() - async and other methods to proceed usual activities
- notify() - to subscribe owner object for specific tasks for some callback to send async notification about change/update (what is owner object?)
- report() - produce report about task status
- finish() - to shutdown task
- validate() - to evaluate current task status and return True if everything is fine or false otherwise

The testing part per each task can be implemented in test/ subfolder separately from the main code:
- testing() - async call performs basic functionality testing e.g. as a part of setup()

For the Task's common scope, more calls might be required, for example:

- directory() - build list of names for all tasks by scanning tasks/ subfolder. Controller will use it for activation auxiliary tasks e.g.
- create() - to create specified tasks by name if Settings allows for task class name and returns task reference or None
- close() - deactivate some task and cleanup resources, might be require after landing
- active() - query another active task or tasks (if None passed) by name e.g. camera may query Storage (SD card)
 here is the activation command of important tasks
.....
 here is the activation command of non-important tasks

for name in Task.directory():
    if not Task.active(name):
        task = Task.create()
        if task.setup():
            logger.info(f'task {name} is up and running')
        else:
            logger.warning(f'task {name} failed to setup')
            task.close()
As set of tasks is not changed over flight the creation order and dependencies can be hardcoded in Controller e.g. enabling Wifi enables

Console to control and check parameters
if Storage available - backing Logging and Telemetry to storage
if Camera available - translation of video stream begins
But to make code less complex one task can ask object of another task by class i.e. if Console created it will auto connect during setup() to Wifi or UART.

## Task Data-Flow and Message Propagation

Data does not flow through a single generic message bus. A bus that allocates a message
object per sample at IMU rates (100–200 Hz) would fragment the heap and trigger GC pauses,
violating the `<10 ms` control-loop budget; and under cooperative `asyncio` a single slow
subscriber would stall the publisher inline. Instead the mechanism is chosen per data class:

* **Hot sensor data (direct, latest-value "databoard").** High-rate readings (IMU, baro) are
  written in place into preallocated per-quantity slots — each holding `value + timestamp +
  source` — with latest-wins semantics and no per-sample allocation. The control loop and the
  sensor-fusion layer read the freshest *valid* slot directly (staleness is the fusion
  priority/timeout logic). The control loop is therefore self-contained: it reads the
  databoard and writes servos without pulling per-cycle data through any queue, so it keeps
  running even if other tasks stall. It is paced by a hardware timer, not `asyncio.sleep`,
  which floors at ~10 ms on this port (see the
  [benchmark findings](../doc/benches/WaveShare_esp32p4-micropython-findings.md)).

* **Everything else goes through one Recorder.** For simplicity there is a single non-hot path.
  Every task reports logs and telemetry **directly to the Recorder** (`Recorder.log()`,
  `Recorder.tlm()` — a global singleton), and each record is stamped with `time.time_ns()//1000`
  (microseconds, monotonic, no wrap). The Recorder enqueues complete UART-ready text lines into
  two PSRAM ring buffers by priority:
  * **Telemetry — 1st priority queue.**
  * **Logs — 2nd priority queue.**
  An async drain loop empties these to the Recorder module (Luckfox) over UART, telemetry before
  logs. The UART push happens **first** (it is the authoritative flight-data sink); any other
  subscribers — notably the Control Center live view — receive the same records **only after**
  they have been pushed to UART. This guarantees recorder durability first and treats CC as a
  best-effort secondary consumer. Records are written into the rings with `struct.pack_into`
  rather than slice-assignment, which is O(buffer length) on this port (see the
  [benchmark findings](../doc/benches/WaveShare_esp32p4-micropython-findings.md)). Telemetry streams are
  created via a `Telemetry(file, fields)` helper that emits a CSV header first and then
  timestamped rows; all streams in a boot share one session prefix (`YYYYMMDD_HHMMSS`, produced
  from the RTC the first time telemetry is emitted) so each flight's files are distinct.

This collapses what would otherwise be a separate event-bus plus ring buffers into the Recorder:
discrete events are just log records, and the priority queues are the decoupling buffers
between fast producers and the slow UART/CC drains.

## Logging

Log strings append system uptime values in milliseconds alongside a standard descriptor layout:

111 Controller :: setup started
 2222 Controller :: boosting detected
 5555 Controller :: landing completed

 The centralized logging manager multiplexes data across these potential sinks depending on system state:
- Hardwired UART serial interface (console).
- Raw network sockets to the Control Center over TCP (active only when the Wi-Fi connection is maintained, i.e. prestart).
- The Recorder module over the dedicated `uart_recorder` link, which persists logs to its own SD card (the controller has no local SD). See [recorder module](../src/camera).

## Telemetry

Telemetry mirrors the logging architecture but outputs structured, semicolon-separated CSV profiles streamed to the Recorder, which the Luckfox demuxes into one file per stream (`<session>_<file>.csv`). For example the board-vitals stream `board_health.csv` — real rows from an on-board flight (`uptime` µs; `temp` °C; `mem_free` bytes, showing the GC-off sawtooth; `load` %, peaking at the landing work):
uptime;temp;mem_free;load
4940864;32;32612240;0
11591868;31;31532800;47
14650552;31;32537888;6

Post-flight parsing arrays can extract these files to compile automated 3D spatial flight path models in standard GPX formatting.

## Storage Write Constraints

Raw write evaluations show that direct synchronous block-writing to local flash or SD arrays introduces SPI bus-locking delays lasting up to 80ms. This latency is unacceptable for a tight flight control loop. The controller therefore carries **no SD card at all**; data is offloaded over UART. Two defensive measures decouple data offload from the control loop:

- A high-rate circular FIFO buffer inside the ESP32's PSRAM absorbs bursts so producing tasks never block on I/O.

- The buffer drains over the dedicated `uart_recorder` line to the Recorder module, which owns the SD card and persists logs, telemetry, and video.

See [recorder module in sources](../src/camera) and [`board-config.md`](board-config.md).

## Console

The interactive system console provides terminal access to:
- Remote status monitoring.
- Active task auditing and dynamic run-state toggling.
- Global software reset commands.
- Profiling dumps using the localized task.report() method.
A local UART line is always available for direct on-bench debugging. Over Wi-Fi the board does **not** host a console; instead it connects to the Control Center as a client and answers the line protocol, which CC exposes to operators (telnet on TCP 1235) and to the browser. See [`cc-protocol.md`](cc-protocol.md).

## Pins Distribution

To ensure hardware modularity, physical microcontroller pins are **not** hardcoded; they are defined by the board configuration (`buses` and `pins` sections of `board.config`, with firmware defaults in `config_default.py`). The controller reads this config at boot to build the pin map and instantiate the declared components. See [`board-config.md`](board-config.md) for the schema and activation lifecycle.

## System Status

The telemetry loop tracks and records standard internal diagnostic variables exposed by the MicroPython ESP32 port architecture to simplify hardware debugging:
- Core CPU operational frequency, execution load factors, core temperatures, and thread tracking.
- Heap memory allocation pools, free block footprints, and fragment boundaries.
- Persistent storage availability benchmarks listed in kilobytes.

## Accelerometer

An onboard bno055 accelerometer registers linear forces, passing high-G vectors to the master process via asynchronous exception handling. The sensor abstracts data collection using standard bno055 MicroPython drivers. To eliminate the need for field calibration on the launch pad, static calibration vectors are loaded directly from NVS memory blocks during ground initialization.

This sensor serves as the primary trigger source for transitioning from the Setting phase to the Boosting phase.

## Gyroscope

The rotational tracking loops rely on the integrated bno055 gyroscope to sample yaw, pitch, and roll rates. The driver issues asynchronous callbacks to the flight controller whenever angular rates cross a defined deadband threshold. To eliminate sensor drift errors, the module must be physically aligned as closely as possible to the physical center of gravity of the airframe. Due to the low G tolerance of the BNO055, the ADXL375 is better to use.

## Geomagnetic

The bno055 geomagnetic sensor extracts absolute magnetic heading vectors. It serves as a direct drift-correction mechanism to validate and backup the primary GNSS tracking coordinates.

## Navigation

Horizontal position tracking uses an ATGM336H-5N-31 high-sensitivity GNSS array. The module operates in a low-power 1 Hz mode during ground staging. Upon detecting vertical launch acceleration, the controller forces a command down the serial line to escalate the update frequency to a high-speed 10 Hz rate.

The standard serial driver structure must be modified to use Interrupt Service Routines (ISR) to handle the higher data rates supported by the core AT6558 chip architecture, replacing standard polling examples found in open-source references:

- PermatechCA ATGM336H Library
- Liuyufanlyf MaixPy GNSS Driver
- Albresky ATGM336H Driver Repository

To scale the data processing up to the 10 Hz threshold without overflowing the serial buffers, the system follows standard NMEA high-rate command structures:

- The serial interface speed (Baud Rate) escalates from 9600 to 115200 bits per second via a $PCAS01,5*19\r\n control string.

- Unnecessary NMEA sentences (such as GSV or GSA) are suppressed using the $PCAS03 mask to minimize data packet sizes, leaving only GNGGA and GNRMC strings active.

$PCAS10,3*1F<cr><lf>    # Enforces factory cold restart
$PCAS01,5*19<cr><lf>    # Escalates interface speed to 115200 baud
$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02<cr><lf>  # Filters out all sentences except GNGGA and GNRMC

- The update rate is shifted to 100ms intervals using the tracking string $PCAS02,100*1E\r\n.

A verified MicroPython initialization snippet handles this handshake sequence.

The Flight Controller continually correlates accelerometer vectors alongside GNSS strings to maintain dead-reckoning positioning if the satellite signal drops out mid-flight.

## Altimeter

High-resolution altitude tracking uses a Gravity: ICP-10111 Pressure Sensor, selected for its 8.5cm operational accuracy and low 2mA current consumption. Barometric calculations are cross-checked against a secondary onboard BMP280 Digital Pressure Sensor and incoming GNSS elevation metrics.A verified vertical delta $\le 3\text{ meters}$ AGL acts as the absolute trigger to drop the master stage machine from Gliding to Landing mode. Due to low altitude mode not working very well on the barometer the laser range finder is mandatory for safety.

## Separation Sensor (Switch or Breakaway Wire)

The physical booster separation event is handled via an explicit electrical disconnect or micro-switch configured as a hardware interrupt. While nested on the booster airframe, the glider holds the circuit closed. Two implementation pathways are supported:

- Pressure Micro-Switch: A Gravity Digital Crash Sensor mounted to the airframe that springs open immediately as the glider leaves the booster body tube.
- Breakaway Pin/Socket: A physical wire loop plugged into a dedicated port on the flight computer. When the motor's black powder ejection charge pops the glider out of the body tube, the tethered wire pulls free from the socket.

The resulting state transition instantly alters the input pin logic to HIGH, invoking an unblock event via a hardware interrupt. This forces the master Flight Controller to transition immediately from Boosting to Gliding state. For separation detection, sensor or termination wire and IMU can be used simultaneously to ensure proper separation detection:
1. Separation sensor triggered
2. IMU detects sudden pitch/roll change
3. Altimeter shows positive vertical deceleration
4. Wire is disconnected and pin gets 0

## Servos

Three independent micro-servos drive the vertical stabilizer and dual elevon surfaces: **two SG90 on the elevons and one metal-gear MG90S on the yaw fin** (`config_default.py` drivers `sg90` / `mg90s`). They are electrically interchangeable — same PWM interface and rail — so the figures below apply to both. These servos provide a nominal stall torque of 1.2–1.4 kg·cm and an actuation speed of 0.11 seconds per 60 degrees. Due to significant manufacturer variability among component clones, custom hardware pulse-width modulation (PWM) calibration maps must be verified during system setup.To mitigate severe voltage drops on the primary 5V power line (as individual micro-servos can draw up to 1A under stall loads), the flight software enforces strict electrical safety protocols:
- Position update commands are suppressed if the target angle matches the current surface deflection state.
- Target positioning parameters are checked against baseline calibration maps loaded during system setup.
- The Flight Controller triggers servo updates sequentially rather than simultaneously to prevent additive current spikes.
- Angular deflections are structurally limited to an operational envelope of -45° to +45°.
- This small throw keeps surface travel times well under 1ms, utilizing range correction tracking profiles where applicable.
- The integrated diagnostic task handles sequential verification by sweeping the surfaces through steps and measuring return latencies.

## Storage

High-capacity storage does **not** live on the controller. The Recorder module (Luckfox Pico) owns the SD card and persists logs, telemetry, and video received over the `uart_recorder` link. See [recorder module](../src/camera) and [`board-config.md`](board-config.md). This keeps the SPI bus off the controller's critical path entirely.

## Wi-Fi

The integrated 2.4GHz Wi-Fi subsystem is optimized for extended range. During ground staging the board joins the **Control Center's** network as a **station** (SSID, credentials, CC host/port and tunable TX power come from the `wifi` section of `board.config`; Bluetooth is disabled to improve the link). Once a network socket connection to the Control Center is established, the flight controller unlocks remote parameter tuning, health monitoring, and live telemetry streaming. The link exists only in prestart; it is expected to be lost from ignition onward. See [`board-config.md`](board-config.md).

## Camera

Video capture is **not** a controller responsibility. It is handled by the independent Recorder module (Luckfox Pico + sc3336b), which records 2304×1296 30 FPS video to its own SD card on its own power supply. Isolating it on a separate board prevents video encoding overhead and storage I/O from impacting the primary flight control tasks. See [recorder module](../src/camera).

## Audio

Audio is captured (if at all) by the Recorder module alongside its video, not by the controller. The controller has no microphone or local storage. This subsystem is optional and out of scope for the controller firmware.

# Overall Design Risks and Mitigations

- First flights will be with telemetry ONLY collection without active control to understand potential locking and sensor problems which will be mitigated later by adding more functional components like watchdog, heartbeat, or runtime health monitor service.
- Assume by aerodynamics that the minimum effective airspeed to control the glider is about 10 meters per second.
- Having GPS accuracy target as 10 meters, I will asign the landing zone's sides at atleast 50 meters.
- To not overload the system with current and keep the battery safe, at least 800 mAh battary will be used with seperate voltage boosters for the controller and engines. Additionally, the servos' positioning / adjusting will be done sequentially.
- Definition of "no control" will be clarified through trials with a telemtry only glider, preliminary

| Subsystem / Loop            | Target Frequency | Max Allowed Latency | Notes / Rationale |
|-----------------------------|------------------|----------------------|-------------------|
| **Primary PID Control Loop** | 50–100 Hz        | < 10 ms     | Core stabilization loop; must run even if other tasks stall. Should ideally run on its own core or native module. |
| **IMU Sampling (BNO055)**   | 100–200 Hz       | < 5 ms       | Gyro/accel data must be fresh for stable control. Interrupt-driven preferred. |
| **Servo Output Update**     | 40–60 Hz         | < 20 ms      | Standard RC servo frame rate; sequential updates to avoid current spikes. |
| **GNSS Parsing**            | 10 Hz            | < 150 ms     | Only needed for navigation; not safety‑critical for attitude control. |
| **Altimeter (ICP‑10111)**   | 20–50 Hz         | < 50 ms      | Needed for landing detection and vertical rate estimation. |
| **Telemetry Logging**       | 1 Hz (normal)    | < 500 ms     | Low priority; should never block control loops. |
| **Telemetry Burst (Landing)** | 10 Hz          | < 100 ms     | Only after control authority is no longer critical. |
| **Task Scheduler / Asyncio** | 20–50 Hz        | < 20 ms      | Supervisory logic; must not interfere with PID loop timing. |
| **Watchdog Reset Window**   | —                | 100–200 ms   | If PID loop or IMU updates stall beyond this, system must reset or enter degraded mode. |

**Measured reality check** (see [benchmark findings](../doc/benches/WaveShare_esp32p4-micropython-findings.md)):
`asyncio.sleep_ms()` floors at ~10 ms (FreeRTOS 100 Hz tick), and a fragmented-heap `gc.collect()`
was measured at ~67 ms (hence GC is controlled and disabled in flight).

**Open risk — sub-10 ms control loop.** It is not yet settled that we need one. The glider flies
below ~100 m/s, so a ~20–50 Hz loop (well within `asyncio.sleep_ms`) may be plenty — if so this
risk disappears and the loop stays plain cooperative asyncio. If a faster loop *is* needed, ranked
mitigations (we have headroom — no multimedia, no native code yet): (1) split work across more
asyncio queues/tasks; (2) drive scheduling from **IRQs** (pin/UART) kicking a `ThreadSafeFlag`
rather than polling; (3) a **hardware-timer** tick that releases the control task; (4) move the
control loop to its **own core thread** (`_thread`); last resort, a **native** control/servo
module. This is decided in Phase 3 against real flight data, not now.
