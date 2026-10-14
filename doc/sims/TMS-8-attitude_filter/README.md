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
  noisy published attitude leaned 3.6-4.8 degrees; on the filter, which integrates the gyro, 0.2-0.6.
- The filter carried each filter-alone flight (`free_rows`: its free-running attitude.csv rows, all but the pad's
  settling and the separation's half second); the sim's-attitude flights mirrored it.
- The realtime glides on the filter run shorter than the SITL's (48-90 s against 63-89): one seed each, the landings
  a lottery as in every study, and realtime adds the Wi-Fi's bunched frames. The boost is the same on both boards.

## Closing: `course_shift` 7.5 (`sitl_closing.csv`, `7452e2c`)

The filter pulls its heading onto the GNSS track 2^-shift per tick. Plan B's 5 (tau 0.64 s) drives a crosswind's crab
out, so the wind triangle lost the crosswind; 10 kept it but landed worse. The operator chose 7.5 (tau 3.6 s), and the
closing matrix flew it against 5: TMS-8's config, F15, the measured IMU, 0/3/6 m/s crosswind, the vertical and the
6-DoF boost. New in it: `--pad-heading unknown`, the separation no longer hands the filter north (a real board's
heading is its power-up pose until the track pulls it), 5 seeds a cell, and the old known heading at 3 seeds.

| pad heading | boost | `course_shift` | miss (median) | in the zone | heading RMS | wind error |
|---|---|---|---|---|---|---|
| unknown | vertical | 5 / 7.5 | 55 / 58 m | 9 / 10 of 15 | 10.6 / 15.2 deg | 1.9 / 1.4 m/s |
| unknown | 6-DoF | 5 / 7.5 | 65 / 72 m | 4 / 3 of 15 | 14.2 / 17.2 | 0.8 / 0.8 |
| known | vertical | 5 / 7.5 | 42 / 45 m | 6 / 8 of 9 | 9.9 / 7.6 | 2.0 / 1.4 |
| known | 6-DoF | 5 / 7.5 | 66 / 70 m | 4 / 4 of 9 | 15.4 / 11.1 | 1.8 / 1.0 |

- Paired seed for seed and widened to 108 pairs (the review flew seeds 6-15 too), 7.5 is neutral: the median miss
  +0.6 m, in the zone 48 -> 51, and the tail shrinks (misses over 120 m 9 -> 2). At 6 m/s on the vertical boost it
  lands 11 of 15 in the zone against 5, and reads the wind to 1.2 m/s against 3.2.
- The cost is finding north: with an unknown pad heading the heading takes ~10 s after the separation (or a warm
  start) to come within 10 degrees, against ~2.3 s at 5. It costs calm air and the short 6-DoF glides at 6 m/s.
  A fast first capture (the fast pull until the heading has met the track, then 7.5) is the phase 7 fix.
- The boost is identical at both shifts (the lean law reads roll and pitch only). The 6-DoF loops at 6 m/s in 4 of
  30 seeds on the filter and 17 of 30 on the sim's perfect attitude, on the shipped `c593ca4` as well: the
  placeholder stack's edge (the lean law needs 1.0 calibre at 6 m/s, tms7d has 0.84), 0 loops at 4-5 m/s or on a
  1.5 m rail. A phase 7 envelope requirement, not a port defect.
- The course-5, known-heading rows reproduce `sitl_shipped.csv` exactly: the harness and the cold start are unchanged.

The boards, `7452e2c`, the measured IMU, a 40 s pad, an unknown pad heading:

| board | run | miss | aloft | burn: path min / most lean | control |
|---|---|---|---|---|---|
| taster | the filter, vertical | 33.5 m, in | 88.6 s | | 0 overruns, late at most 39 us |
| taster | the filter, 6-DoF | 84.4 m | 64.7 s | 89.6 / 0.6 deg | 0 overruns, 50 us |
| taster | the filter, a panic mid-glide | 14.5 m, in | 55.9 s | | warm start: the gyro bias kept |
| TMS-7F | the filter, vertical | 95.1 m | 94.7 s | | 0 overruns, 41 us |
| TMS-7F | the filter, 6-DoF | 83.7 m | 65.5 s | 89.7 / 0.4 deg | 0 overruns, 33 us |

Every board run reached DONE. The warm start recovered GLIDING with the pad's bias as the filter's seed (`gyro 0 bias
(-0.505, 0.174, 0.212) deg/s kept as the filter's seed`) and landed in the zone.

## Caveats

- The sim's separation jumps the attitude (sim_model begin_glide); `--no-attitude` hands the filter the truth for
  0.5 s there, a perfect gyro through the separation, turned since the closing matrix by an unknown pad azimuth
  (`--pad-heading`). A real separation's error is the gyro's over its pitch-over. The closing matrix drew that
  azimuth from the seed's first number, the one the sim's x gyro bias draws too (seeds 3 and 4 got nearly the same
  heading); since then it has a seed of its own.
- Two sim issues found by the closing review, both outside the firmware: the sim's magnetometer is mirrored against
  the Rust filter's frame (the magnetic fallback is not exercised in simulation), and the vertical sim clamps the
  coast's specific force at 0 g, so with a positive accelerometer bias the thrust gate holds the apogee to the 12 s
  burnout timeout (about a third of the seeds: their vertical misses are pessimistic).
- The measured IMU model is the parts at rest: the boost's vibration and the gyro's scale error are not in it.
- A warm start carries the learned gyro biases since 2026-10-14 (the crumb's `gyro_bias`, frozen at BOOSTING entry);
  before, the rebuilt filter flew the raw gyro for the rest of the flight. No run here has a warm start.
- 3 seeds a SITL cell, 1 a board run; the aerodynamics are BC1's fit and the sim's glide model (sink charged off trim).

## Files

- `sitl_runs.csv`: the before/after matrix (build, attitude, boost, wind, seed, path minimum, loop, miss, zone, aloft).
- `sitl_filter_variants.csv`: the bias and centripetal variants, with the attitude scores.
- `sitl_shipped.csv`: the shipped firmware on both IMU models, with the attitude scores.
- `hitl_runs.csv`, `hitl_runs_final.csv`: the board flights, with the filter's free-running rows.
- `sitl_closing.csv`: the closing matrix, `course_shift` 5 against 7.5, the pad heading unknown and known.
- `hitl_runs_closing.csv`: the closing board runs on `7452e2c` beside the earlier ones, with the control timing.
