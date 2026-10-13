# Boost control: the stack's attitude under thrust

The operator's next big item (2026-10-12): control in BOOSTING, so the stack leaves the rod and climbs instead of
bending over or looping. This plan says what exists, why it cannot be judged yet, and the stages that get it
judged and flown.

## 1. Where it stands

**The law exists.** Plan B (`src/glider/guidance.py` `_boost`) and the Rust port (`crates/control/src/guidance.rs`
`boost`) both hold the attitude captured at BOOSTING entry, once past the rod (airspeed ≥ `boost_engage_speed`,
15 m/s), under the speed-scaled governor (5° at 50 m/s and up). BOOSTING is not a control stage by default, and
no flight has used it: the flight-test ladder flew passive, and 7C had no servos.

**The 10-03 flights say what it must handle** (`launches/20261003/`, each recomputed by four analysts and a judge):
- **7C** (414 g, fins fixed at 0°, no servos): clean on the rod to 0.285 s (1.0 m, 10.6 m/s), departure at
  0.31 s, a roll torque proportional to dynamic pressure (−11.7 °/s² per Pa: a built-in asymmetry of about 1° of
  fin cant), side load up to 9.3 g, nose through horizontal at ≈1.06 s **under thrust**, a loop, impact. Static
  margin estimated +0.24 calibers.
- **7D** (484 g, servos fitted, control off): 9.0 m/s at 1 m in a ~4.5 m/s crosswind (26° angle of attack); the
  angle of attack held at 17–19° while q rose fourfold (margin near neutral early), the path bottomed at 32° at
  1.6 s, the margin turned restoring as the propellant burned, the roll spun up to 1595 °/s under thrust, burnout
  2.53 s at 53.5 m/s, apogee 91–98 m at 5.3–5.5 s, ~145 m out.
- **Both:** the side force locked into the plane normal to the folded wings (margin per plane), and both carried
  the same ~1° roll asymmetry. 7D's lower thrust-to-weight bought the time for its CG to move forward; 7C ran out.

**The simulator cannot judge any of it.** `src/glider/sim_model.py` `Body.boost_step` climbs 1-DoF vertically
(lean-blind: every case reaches the same apogee) and models "pitch" and "roll" as two lean axes driven by the wind
angle and the fins. The body's own lean never enters its angle of attack, so there is no static stability and no
margin; there is no rotation about the body axis; the motor is a flat 14.4 N for 3.45 s where the real F15 is a
32-33 N spike, 42 N·s, burnout 2.5-2.7 s. The 7/06 sweep that called imbalance "a solved problem"
(`doc/specs/coludo.md` "Boost disturbance rejection") ran on this model.

**The law has a representation problem near vertical.** It holds the Euler roll and pitch captured at entry. On
the rod the pitch is ~90°, where Euler roll and heading are both rotations about the vertical: the captured "roll"
is ill-defined, and a spin about the body axis (7D's 1595 °/s) is not something it can see as roll. The
launch-day verdict on sensors agrees: attitude in boost comes from the LSM6DSO32 gyro integrated from the pad's
gravity vector, not from a fused Euler estimate.

## 2. Stages

Each stage is committed on its own proof. The firmware changes only from BC3 on.

| stage | what | proof |
|---|---|---|
| BC1 | a 6-DoF boost in the simulator, the measured F15, per-airframe stack parameters | host tests: 7C, 7D and TMS-7 replays (below) |
| BC2 | the passive envelope: where an uncontrolled stack departs (margin x asymmetry x wind x rod exit speed) | a SITL sweep, published in `doc/sims` |
| BC3 | the boost law for vertical flight: the lean of the body axis + roll damping, the gyro-integrated attitude, the rod gate | host tests; the SITL sweep with control |
| BC4 | HITL on the taster, servos moving: the full matrix with metrics and plotly pictures | `doc/sims/TMS-7-boost_control/` |
| BC5 | turn it on: BOOSTING a control stage in the flight configs | the operator's decision, on BC2-BC4 |

### BC1: the 6-DoF boost

A rigid body from ignition to separation, beside the existing glide model (which is untouched):
- **State:** position and velocity (ENU), attitude (a quaternion, body x along the nose), body rates (p, q, r).
- **Rod:** constrained along the rod axis until the travel reaches its length (1 m on 10-03), then free.
- **Forces:** thrust along the body axis (optionally misaligned), gravity, axial drag from CdA (7D's coast:
  ~25 cm²), the normal force CNα·q·S·α in each lateral plane at the centre of pressure.
- **Moments:** static stability from the margin (CP behind CG, per plane: the folded wings make the two planes
  differ), pitch/yaw damping, roll damping, the roll forcing of an equivalent fin cant (∝ q), the control fins
  (∝ q·δ, the governor's cap applied by the board), and the gyroscopic coupling of a spinning body.
- **Mass properties:** the stack mass and inertia; the propellant burns off along the thrust curve, moving the CG
  forward (7D's margin turned restoring at 1.4-2.2 s).
- **Motor:** the measured F15: a 32-33 N spike at ~0.1-0.25 s, 42-43 N·s, burnout 2.5-2.7 s (TMS-7 and 7D).
- **Wind:** steady plus the existing gusts, with a surface profile as an option.
- **Sensors:** body rates and specific force in the board's marks frame (the gyro, both accelerometers), the Euler
  attitude the sim publishes, baro altitude, and the pitot's flight span (0.51-0.59 of q at low angle of attack,
  topping out at 546 Pa) so the governor sees what the board sees.
- **Integration:** fixed sub-steps of ≤ 1 ms inside the host's step (1600 °/s spins).

Parameters live in airframe profiles (`AIRFRAMES`, as TMS-8's do), one per stack: 7C, 7D, TMS-7. The old
1-DoF boost stays the default until BC1's proof, so every published study reproduces; then the 6-DoF becomes the
default and the plan B bands are re-measured once.

**Replay regressions (the acceptance, from the launch README):**
- 7C (+0.24 cal, ~1° cant, calm): clean on the rod, departs within 1.3 s, the nose past horizontal under thrust;
- 7D (+0.71 cal nominal, near neutral early, ~1° cant, 4.5 m/s crosswind): angle of attack 17-19° early, the
  path minimum near 32° at ~1.6 s, apogee ~95 m;
- TMS-7 (its clean flight): its apogee and burnout speed.

The fits are coarse by design: the data is 1.3 s (7C) and 6.4 s (7D) of one flight each. A regression passes on
the event and the order of magnitude, not on a curve fit.

### BC3: the law, for vertical flight

- **Attitude:** the gyro integrated as a quaternion from the pad's gravity vector, captured at the rod trigger
  (the BNO055 and the fused Euler angles near pitch 90° are not used in boost). 2.5 s of integration at < 1 °/s
  bias drifts < 2.5°.
- **What it holds:** the tilt of the body axis from the launch direction, as two components in the body frame
  (each fin pair's plane), and the roll RATE about the body axis toward zero. No roll angle, no heading.
- **When:** past the rod by measured travel (integrated acceleration ≥ the rod's length) and speed, not by the
  airspeed estimate alone.
- **How hard:** the governor's cap as everywhere; the operator's ±10° direction (2026-06-25) as the boost's own
  ceiling.

The law's design is open until BC2's envelope shows what it must cover.

## 3. Not in this plan

The stack's own design (margin, booster fins, the joint, a longer rail: the launch README's December list) and
separation. The simulator makes the first measurable; it does not replace a swing test.
