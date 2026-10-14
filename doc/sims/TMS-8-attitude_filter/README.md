# TMS-8 — the attitude filter flies alone: the boost and the glide on the board's own attitude

**What it asks.** TMS-8 has no BNO055: its attitude is the board's complementary filter alone, and the lean boost law
(BC5, law A) steers on it. Until 2026-10-14 every HITL and SITL flight published the sim's own attitude at priority 0,
so the filter only mirrored it: the filter had never flown. Does it hold the boost and fly the glide, and what did the
plan B -> Rust audit's fixes change?

**How it was flown.** `tools/hitl_host.py --no-attitude`: the sim publishes no attitude, and its sensors read as a real
airframe's (the 1 g on the nose axis on the pad, the gyro's BODY rates, the separation's jump handed over as a perfect
gyro would see it, a pad long enough for the filter to settle). TMS-8's board config, F15, 285 g glider, the vertical
boost and the 6-DoF boost (`--boost 6dof`, the stack `tms7d`), crosswind toward 90 degrees.
- the SITL on cheeta: the firmware before the audit (`1c8aea6`) against after (`d2fad7d`), the sim's attitude against
  the filter alone, 0 / 3 / 6 m/s, 3 seeds, noise 0.1: 72 flights (`sitl_runs.csv`);
- the boards (realtime HITL, `d2fad7d`): the taster (servos moving) and TMS-7F (rail off), each the sim's attitude
  against the filter alone, vertical and 6-DoF, calm, noise 0.1, and the filter alone at noise 0: 10 flights
  (`hitl_runs.csv`; `free_rows` counts the filter's free-running attitude.csv rows).

## What the audit found in the filter

- **The GNSS course never reached it** on a board: the heading was the gyro's from power-up.
- **Plan B's kinematics**: the gyro's body rates added to the Euler angles, right only about a level body. In a banked
  turn the pitch drifted by the turn rate times the sine of the bank while the turn gate held the accelerometer off;
  a spinning boost scrambled all three angles within a second. The sim published Euler rates for a gyro, which hid it.
- **The accelerometer pull in BOOSTING**: thrust, then drag, read as gravity whenever the magnitude crossed the
  0.7-1.3 g band, snapping the estimate to nose-up while the stack pitched over.

Fixed (`f40154c`, `c64fe1a`): the course and speed wired in, a quaternion turned by the body rates, no accelerometer
pull in BOOSTING (doc/specs/avionics-port.md M9).

## SITL (noise 0.1, 3 seeds a cell)

| boost | attitude | firmware | 0 m/s | 3 m/s | 6 m/s |
|---|---|---|---|---|---|
| vertical | the sim's | before = after | miss 81-92 m, 95-98 s | 50-64 m, 96-98 s | 9-20 m (3 in), 97-98 s |
| vertical | the filter | before | crashed at 19 s, 213 m | 20 s, 216-224 m | 20 s, 204-210 m |
| vertical | the filter | after | 35-99 m (1 in), 58-68 s | 36-69 m (2 in), 67-73 s | 49-82 m (1 in), 68-79 s |
| 6-DoF | the sim's | before = after | path 89-90, 90 m, 67 s | path 85-86, 75-87 m | 1 loop of 3 (path -36, 58, 57) |
| 6-DoF | the filter | before | path 79-80, crashed at 7-8 s | path 82-83, crashed at 9-11 s | 3 loops of 3 (path -6 to -34) |
| 6-DoF | the filter | after | **path 90**, 40-98 m, 30-40 s | **path 86**, 24-99 m (1 in) | 3 loops of 3 (path 46-52), late |

- On the sim's attitude the audit's fixes change nothing (identical flights): the law was right; its attitude was not.
- Before the audit the filter alone crashed every flight. After it every flight lands, and the lean boost holds the
  6-DoF stack on the filter as on a perfect attitude at 0 and 3 m/s. At 6 m/s (7D's stack, beyond its own margin as
  BC4 found) it loops later in the burn than before, and more often than on the sim's attitude.
- The glide on the filter is shorter than on a perfect attitude (58-79 s against 95-98): at noise 0.1 the sim adds
  +-10 % to every accelerometer and gyro sample, which costs the filter a few degrees of pitch in the turns, and this sim
  charges every degree off trim in sink. At noise 0 the filter tracks the truth to 1-2 degrees (a 6-DoF flight landed in
  the zone, 11.6 m). Real IMUs are far quieter than noise 0.1.

## The boards (calm)

| board | run | miss | aloft | burn: path min / most lean |
|---|---|---|---|---|
| taster | the sim's attitude, vertical | 88.6 m | 99.2 s | |
| taster | the filter, vertical | 96.9 m | 75.4 s | |
| taster | the filter, vertical, noise 0 | **26.6 m, in the zone** | 84.1 s | |
| taster | the sim's attitude, 6-DoF | 88.1 m | 68.7 s | 86.4 / 4.8 deg |
| taster | the filter, 6-DoF | 48.5 m | 55.2 s | **89.9 / 0.2 deg** |
| TMS-7F | the sim's attitude, vertical | 92.2 m | 97.7 s | |
| TMS-7F | the filter, vertical | 99.5 m | 73.4 s | |
| TMS-7F | the filter, vertical, noise 0 | 55.0 m | 80.5 s | |
| TMS-7F | the sim's attitude, 6-DoF | 90.8 m | 69.1 s | 88.5 / 3.6 deg |
| TMS-7F | the filter, 6-DoF | 76.6 m | 51.2 s | **89.7 / 0.4 deg** |

- **The board's filter holds the boost better than the sim's attitude did**: the lean law steering on a noisy published
  attitude chased its noise (3.6-4.8 degrees of lean); on the filter, which integrates the gyro, 0.2-0.4.
- The filter carried each filter-alone flight: 3236-4924 free-running rows of 3311-4998 (the rest the pad's settling
  and the separation's half second); the sim's-attitude flights mirrored it (20-23 free rows).
- The glide matches the SITL: shorter on the filter at noise 0.1, in the zone at noise 0 on the taster.

## Caveats

- The sim's separation jumps the attitude (sim_model begin_glide); `--no-attitude` hands the filter the truth for
  0.5 s there, a perfect gyro through the separation. A real separation's error is the gyro's over its pitch-over.
- One seed a board run; the SITL's 3 seeds a cell.
- The aerodynamics are BC1's fit and the sim's glide model (sink charged off trim) as in the boost studies.

## Files

- `sitl_runs.csv`: the 72 SITL flights (build, attitude, boost, wind, seed, path minimum, loop, miss, zone, aloft).
- `hitl_runs.csv`: the 10 board flights, with the filter's free-running rows.
