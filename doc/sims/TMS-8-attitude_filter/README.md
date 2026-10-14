# TMS-8 — the attitude filter flies alone: the boost and the glide on the board's own attitude

**What it asks.** TMS-8 has no BNO055: its attitude is the board's complementary filter alone, and the lean boost law
(BC5, law A) steers on it. Until 2026-10-14 every HITL and SITL flight published the sim's own attitude at priority 0,
so the filter only mirrored it: the filter had never flown. Does it hold the boost and fly the glide, and what did the
plan B -> Rust audit's fixes change?

**How it was flown.** `tools/hitl_host.py --no-attitude`: the sim publishes no attitude, and its sensors read as a real
airframe's (the 1 g on the nose axis on the pad, the gyro's BODY rates, the separation's jump handed over as a perfect
gyro would see it, a 60 s pad on which the filter settles). TMS-8's board config, F15, 285 g glider, the vertical boost
and the 6-DoF boost (`--boost 6dof`, the stack `tms7d`), crosswind toward 90 degrees, 3 seeds a cell. The IMU noise
either the sim's `--noise 0.1` fraction (+-10 % of every sample, no bias: hostile) or `--imu-noise real` (the
LSM6DSO32 as measured at rest: white noise and a seeded constant bias per axis, gyro up to 0.6 deg/s). `--score`
scores the filter against the truth: the tilt error (degrees between the true and estimated down axis, free of the
Euler angles' degeneracy at a vertical nose) in the boost and the glide, and the glide's heading error.

## What the audit found in the filter, and what was done

| | found | fixed |
|---|---|---|
| the GNSS course | never reached the filter on a board: the heading was the gyro's from power-up | wired (`f40154c`) |
| the kinematics | plan B added the body rates to the Euler angles: a banked turn drifted the pitch, a spinning boost scrambled all three (the lean law looped at 56 m) | a quaternion (`c64fe1a`) |
| BOOSTING | the accelerometer pull read thrust and drag as gravity, snapping the estimate to nose-up | no pull in BOOSTING (`c64fe1a`) |
| the gyro bias | a real gyro's constant bias drifts the attitude through every gated turn | learned at rest in SETTING, each IMU its own (`85683e4`, `c593ca4`) |
| the turn | the pulls that fire in banked flight flatten the bank | omega x V off the accelerometer, at the pitot's airspeed (`85683e4`, `c593ca4`) |
| the HITL hand-over | the sim-link holds a sample 1 s, so the filter copied the sim's last attitude frozen for a second after it stopped | mirror only a current sample (`3d9968f`) |
| a warm start into BOOSTING | the filter published an un-anchored LEVEL guess and the lean law saturated on it | publish nothing until anchored (`c593ca4`) |

## SITL: before and after the audit's first fixes (`sitl_runs.csv`, noise 0.1)

"After" here is `c64fe1a`: the course, the quaternion, no pull in BOOSTING (not yet the bias, the centripetal or the
hand-over fix).

| boost | attitude | firmware | 0 m/s | 3 m/s | 6 m/s |
|---|---|---|---|---|---|
| vertical | the sim's | before = after | 81-92 m, 97-100 s | 50-64 m, 98-100 s | 9-20 m (3 in), 99-100 s |
| vertical | the filter | before | crashed: 215 m, 34 s | 215-264 m, 35 s | 199-213 m, 35 s |
| vertical | the filter | after | 25-80 m (1 in), 77-83 s | 36-58 m (3 in), 85-88 s | 57-168 m (2 in), 89-93 s |
| 6-DoF | the sim's | before = after | path 89-90, 89-91 m, 69 s | path 85-86, 75-87 m | 1 loop of 3 |
| 6-DoF | the filter | before | path 79-88, 84-146 m, 22-24 s | path 82-84, 24 s | 2 loops of 3, 8-18 s |
| 6-DoF | the filter | after | **path 89-90**, 8-76 m (2 in) | **path 86**, 47-72 m (1 in) | **no loop** (path 56-57), 25 s (the hand-over freeze) |

- On the sim's attitude the fixes change nothing (identical flights): the law was right; its attitude was not.
- Before them the filter alone crashed every flight. After, every one lands, and the lean boost holds the 6-DoF stack
  on the filter as on a perfect attitude, 6 m/s included.

## SITL: the bias and the centripetal (`sitl_filter_variants.csv`, the measured IMU)

| variant | glide tilt, RMS (median) / worst | heading RMS | vertical: miss, in the zone, aloft | 6-DoF: in the zone, loops |
|---|---|---|---|---|
| neither (`c64fe1a`'s filter) | 3.2 / 8.8 deg | 19.8 | 80 m, 2 of 9, 72 s | 2 of 9, 1 |
| the bias | 1.5 / 3.8 | 7.1 | 66 m, 2 of 9, 83 s | 4 of 9, 0 |
| **the bias + the centripetal** | **0.6 / 3.0** | 8.6 | **47 m, 5 of 9, 88 s** | 3 of 9, 0 |
| the bias + the centripetal, turn gate opened | 3.1 / 10.9 | 7.1 | 80 m, 1 of 9, 82 s | 2 of 9, 0 |
| the centripetal alone, gate opened | 3.3 / 10.6 | 8.5 | 81 m, 2 of 9, 78 s | 3 of 9, 1 |

The bias and the centripetal went in on, the turn gate kept: opened, the airspeed estimate's error costs more than the
gate does. (These ran before the review moved the correction to the pitot's measured airspeed and fixed the hand-over.)

## SITL: the firmware as shipped (`sitl_shipped.csv`, `c593ca4`)

| boost | IMU | boost tilt RMS / worst | glide tilt RMS / worst | heading RMS | miss (median), in the zone | aloft | loops |
|---|---|---|---|---|---|---|---|
| vertical | +-10 % | 0.0 / 0.1 | 0.8 / 2.3 | 11.8 | 53 m, 5 of 9 | 90 s | 0 |
| vertical | measured | 1.0 / 1.1 | 0.5 / 1.6 | 9.9 | 43 m, 6 of 9 | 89 s | 0 |
| 6-DoF | +-10 % | 1.1 / 4.0 | 0.8 / 2.1 | 20.0 | 70 m, 3 of 9 | 62 s | 0 (path >= 56) |
| 6-DoF | measured | 1.1 / 6.0 | 0.6 / 1.5 | 15.4 | 66 m, 4 of 9 | 63 s | 0 (path >= 51) |

On the filter alone TMS-8 now lands at least as well as on the sim's perfect attitude (the vertical boost 3 of 9 in
the zone there, the 6-DoF 0 of 9). The heading error is mostly the crab: the filter pulls its heading onto the GNSS
track (the navigation steers by the track), so a crosswind's crab reads as heading error against the true nose.

## The boards

`hitl_runs.csv` (`d2fad7d`, before the bias and the centripetal; the +-10 % IMU) and `hitl_runs_final.csv`
(`c593ca4`, the measured IMU, a 40 s pad).

| board | firmware | run | miss | aloft | burn: path min / most lean |
|---|---|---|---|---|---|
| taster | `d2fad7d` | the sim's attitude, vertical | 88.6 m | 99.2 s | |
| taster | `d2fad7d` | the filter, vertical | 96.9 m | 75.4 s | |
| taster | `d2fad7d` | the sim's attitude, 6-DoF | 88.1 m | 68.7 s | 86.4 / 4.8 deg |
| taster | `d2fad7d` | the filter, 6-DoF | 48.5 m | 55.2 s | **89.9 / 0.2 deg** |
| TMS-7F | `d2fad7d` | the sim's attitude, 6-DoF | 90.8 m | 69.1 s | 88.5 / 3.6 deg |
| TMS-7F | `d2fad7d` | the filter, 6-DoF | 76.6 m | 51.2 s | **89.7 / 0.4 deg** |

| taster | `c593ca4` | the sim's attitude, vertical (+-10 %) | 83.6 m | 99.8 s | |
| taster | `c593ca4` | the filter, vertical | 71.1 m | 77.7 s | |
| taster | `c593ca4` | the filter, 6-DoF | 115.0 m | 47.9 s | **89.6 / 0.6 deg** |
| TMS-7F | `c593ca4` | the sim's attitude, vertical (+-10 %) | 86.3 m | 99.4 s | |
| TMS-7F | `c593ca4` | the filter, vertical | 56.7 m | 89.9 s | |
| TMS-7F | `c593ca4` | the filter, 6-DoF | 71.3 m | 64.3 s | **89.8 / 0.3 deg** |

- **The board's filter holds the boost closer to vertical than the sim's published attitude**: the lean law chasing a
  noisy published attitude leaned 3.6-4.8 degrees; on the filter, which integrates the gyro, 0.2-0.4.
- The filter carried each filter-alone flight (`free_rows`: its free-running attitude.csv rows, all but the pad's
  settling and the separation's half second); the sim's-attitude flights mirrored it.
- The realtime glides on the filter run shorter than the SITL's (48-90 s against 63-89): one seed each, the landings
  a lottery as in every study, and realtime adds the Wi-Fi's bunched frames. The boost is the same on both boards.

## Caveats

- The sim's separation jumps the attitude (sim_model begin_glide); `--no-attitude` hands the filter the truth for
  0.5 s there, a perfect gyro through the separation. A real separation's error is the gyro's over its pitch-over.
- The measured IMU model is the parts at rest: the boost's vibration and the gyro's scale error are not in it.
- A warm start flies without the learned gyro bias (the crumb does not carry it).
- 3 seeds a SITL cell, 1 a board run; the aerodynamics are BC1's fit and the sim's glide model (sink charged off trim).

## Files

- `sitl_runs.csv`: the before/after matrix (build, attitude, boost, wind, seed, path minimum, loop, miss, zone, aloft).
- `sitl_filter_variants.csv`: the bias and centripetal variants, with the attitude scores.
- `sitl_shipped.csv`: the shipped firmware on both IMU models, with the attitude scores.
- `hitl_runs.csv`, `hitl_runs_final.csv`: the board flights, with the filter's free-running rows.
