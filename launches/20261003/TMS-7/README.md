# TMS-7 — booster structural test

**No glider.** Booster only, flown to see how the new booster construction handles the strongest motor
available.

| | |
|---|---|
| assembled mass | **129.5 g** (119.5 g airframe + 10 g nose logger, without motor) |
| motor | **F15** (98.8 g loaded) |
| liftoff | **228.3 g** |
| electronics | **nose logger** — ESP32-C6 + SEN0697 + LiPo in the nose cone (`src/logger/`) |

The F15 is the point: it is the highest-impulse motor in the set (49.7 N·s against the E16's 28.5), so
this flies the worst structural case before an instrumented airframe is trusted to it. The booster's
condition afterwards is still the result that decides the test.

**The nose logger makes the flight yield numbers as well as a verdict.** It is standalone — no radio, no
link to anything — recording accel, gyro, pressure and temperature at 50 Hz to its own flash, so this
otherwise data-free flight returns **peak boost acceleration** and **apogee altitude**. It costs 10 g,
which on a chute-recovered booster buys accuracy nothing: unlike a glider, there is no landing zone to
miss.

Expected peak specific force is `25 N / (0.2283 kg × 9.81) ≈ **11.2 g**` against the BMI323's ±16 g
ceiling — 1.4× of margin, with no measured ignition transient anywhere in this project to bound the
spike. Hand punches on the bench already railed that ceiling on seven samples, so an impulsive event can
clip; a sustained boost of this size should not. If the peak must be guaranteed rather than probable, an
ADXL375 (±200 g, owned, driver written) goes in beside it.
