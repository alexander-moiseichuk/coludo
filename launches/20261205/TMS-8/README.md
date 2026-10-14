# TMS-8 — the 2026-12-05 launch's board config (draft)

`tms8.config` is TMS-8's board config as the operator described the airframe on 2026-10-14, for the field day on
2026-12-05.

**Set:**
- **Board:** id `TMS-8`, `layout: auto` (the v1.1 detection), the Rust avionics.
- **Attitude:** the SEN0697 as mounted on TMS-7F (BMI323 `-y+x+z`, BMM350 `+x-y-z`, proven by hand 2026-10-08), the
  BMP581 beside it.
- **GNSS:** the NEO-M8N (`neom8n`, 5 Hz). The NEO-6M is the backup: change the `gnss` entry's `driver` to `neo6mv2`.
- **Fins:** three MG90S (`mg90s`), `fins.concurrency` **3**: the flight battery gives 20 A and the electronics are
  designed for 15 W peaks (three MG90S reversing draw ~0.8 A each, doc/sims/TMS-7-mg90s_yaw). The 3-degree fin grid
  (`step_deg` default).
- **Flight:** the glide law with TMS-8's planning L/D (`glide_ratio` 8.0, doc/sims/TMS-8-preflight).
- **The boost (BC5, the operator 2026-10-14: law A):** BOOSTING is a control stage with `boost_law: "lean"` and no
  boost ceiling (the governor's 1/v^2 cap alone): off the 1.5 m rod, from the 15 m/s gate (a metre or two above the
  rod's top on an F15), the glider holds its body axis vertical until the separation ends BOOSTING (the baro apogee
  as the fallback). On a 1.5 m rail with 7D's stack the loop's edge is 0.2 / 0.5 / 1.0 cal at 3 / 4.5 / 6 m/s
  against passive's 0.2 / 0.8 / 1.5 (doc/sims/TMS-7-boost_envelope, doc/sims/TMS-7-boost_control).

**Pending:**
- **Trims:** the operator zeroes each fin visually (the CC fin-zero card), and the trims go into this file.
- **The launch manifest** (the date is set: 2026-12-05).
- `final_approach_agl` is 0 (TMS-7F's), the taster's bench config 8: to settle with the field.

**Never install this config on a bench board on the 1 A supply:** concurrency 3 lets all three fins slew at once, and
a 1 A supply browns out (it likely cost a servo on 2026-08-29).

Checked: the Rust SITL flies it (TMS-8 airframe, F15, 285 g, noise 0.1): DONE, in the zone at 38.2 m, apogee 231 m;
with the boost law on the 6-DoF stack (7D's, the only profile near it): no loop, the burn's path 89 deg in calm air
and 85 deg in 3 m/s, the elevons working through the burn.
