# TMS-8 — the next launch's board config (draft, no date yet)

`tms8.config` is TMS-8's board config as the operator described the airframe on 2026-10-14; this folder becomes
`launches/<YYYYMMDD>/TMS-8/` once the field day is set.

**Set:**
- **Board:** id `TMS-8`, `layout: auto` (the v1.1 detection), the Rust avionics.
- **Attitude:** the SEN0697 as mounted on TMS-7F (BMI323 `-y+x+z`, BMM350 `+x-y-z`, proven by hand 2026-10-08), the
  BMP581 beside it.
- **GNSS:** the NEO-M8N (`neom8n`, 5 Hz). The NEO-6M is the backup: change the `gnss` entry's `driver` to `neo6mv2`.
- **Fins:** three MG90S (`mg90s`), `fins.concurrency` **3**: the flight battery gives 20 A and the electronics are
  designed for 15 W peaks (three MG90S reversing draw ~0.8 A each, doc/sims/TMS-7-mg90s_yaw). The 3-degree fin grid
  (`step_deg` default).
- **Flight:** the glide law with TMS-8's planning L/D (`glide_ratio` 8.0, doc/sims/TMS-8-preflight); BOOSTING is not
  a control stage yet (below).

**Pending:**
- **Trims:** the operator zeroes each fin visually (the CC fin-zero card), and the trims go into this file.
- **BC5:** the boost law (doc/specs/boost-control.md BC4): with it, `stages` gains `boosting: {}` and the flight
  entry `boost_law: "lean"` (and `boost_limit_deg` if the operator keeps a ceiling).
- **The date** and the launch manifest.
- `final_approach_agl` is 0 (TMS-7F's), the taster's bench config 8: to settle with the field.

**Never install this config on a bench board on the 1 A supply:** concurrency 3 lets all three fins slew at once, and
a 1 A supply browns out (it likely cost a servo on 2026-08-29).

Checked: the Rust SITL flies it (TMS-8 airframe, F15, 285 g, noise 0.1): DONE, in the zone at 38.2 m, apogee 231 m.
