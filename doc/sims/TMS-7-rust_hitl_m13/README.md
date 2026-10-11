# TMS-7 — the Rust board flies the glide (M13, active HITL)

**The Rust flight firmware (`src/avionics`) now controls the glide.** The flight law (`crates/control`
`FlightLaw`, plan B's `tasks/flight.py` tick for tick) runs on the board's 100 Hz control thread and drives the
three SG90 fins on LEDC. It flies hardware-in-the-loop against plan B's physics (`src/glider/sim_model.py` on the
host, `tools/hitl_host.py`), with plan B's own HITL test gains (`config_hitl`: roll kp 2.0 kd 0.2, pitch kp 1.5,
yaw kp 1.5 kd 0.1). The board answers every sensor frame with its real fin angles, and the host flies the body
with them. M12 flew the same harness passive.

All flights: the taster (v1.1 board, real SG90s moving on the bench), F15, 285 g glider, the HPRC pad and zone,
collected with `tools/hitl_collect_host.sh`. Plan B is `tools/virtual_flight.py` on the same physics at equal mass
(`VF_GLIDER_G=285`, noise 0).

## 1. The two control stacks fly the same glide

| flight | GLIDING | LANDING | DONE | apogee | touchdown from the zone centre | in zone |
|---|---|---|---|---|---|---|
| Rust taster, clean — session 000573 | 10.2 s | 109.0 s | 112.1 s | 273.3 m | 39.5 m | yes |
| Rust taster, clean — session 000575 | 10.2 s | 109.0 s | 112.0 s | 273.3 m | 41.5 m | yes |
| plan B `virtual_flight`, clean | 10.0 s | 110.1 s | — | 273.3 m | 49.2 m | yes |
| Rust taster, wind 6 m/s toward 210° — session 000577 | 10.1 s | 108.6 s | 111.6 s | 273.3 m | 37.3 m | yes |
| plan B `virtual_flight`, wind 6 m/s toward 210° | 10.0 s | 109.8 s | — | 273.3 m | 55.8 m | yes |

(Times from each flight's BOOSTING entry. The touchdown is the last fix at or before DONE, as `flight_kpi` measures
it; a virtual flight stops at touchdown, so it has no DONE.)

- **The traces lie on top of each other:** the roll setpoint, the roll command, the left eleron's angle and the
  figure-8 ground track of the loiter all overlay plan B's, turn for turn
  ([`f15_clean_vs_planb.html`](f15_clean_vs_planb.html), [SVG](f15_clean_vs_planb.svg)).
- **Wind:** the board's wind estimate averages **6.11 m/s from 30°** over the glide; plan B's averages 6.00 m/s from 30°
  (true: 6 m/s toward 210°). The track follows plan B's until the final approach, where the last orbit's phase
  decides the touchdown (the endgame sensitivity known from plan B's own runs)
  ([`f15_wind6_vs_planb.html`](f15_wind6_vs_planb.html), [SVG](f15_wind6_vs_planb.svg)).
- **Repeatable:** the two clean Rust runs land 2 m apart, at the same stage times to 0.1 s.
- Every FINS answer carried the fins (5660 of 5660 on 000573); they read 90° on the pad until `arm`'s fin probe
  sweep (45..135°, bounded by `fins.mixer.limit_deg`), then follow the law from the first glide tick.

## 2. A real reboot mid-glide, under control

The console panics the board 1–6 s into the glide (session 000579). The warm start restores GLIDING and the arming
(`recover gliding, 3s after checkpoint (reset_cause panic)`) and **seeds the law's airspeed from the crumb**
(`flight :: airspeed 14 m/s kept as the law's seed (warm start)`): the restored law's first flight.csv row reads 1400
cm/s, not a zero to climb from. The flight then lands to DONE (session 000580).

The touchdown was 56.7 m from the centre, outside the zone. This is the HITL outage, not the law: the restored
board gets no sensor frame until its Wi-Fi joins again (~26 s here, 4446 of 5829 frames answered). It holds the fins
neutral with no fresh attitude and the glider flies straight, then the law guides again for the last 72 s. A flying
board's sensors are local, so its outage is the boot alone.

## 3. What it took

- **The control stack overflowed.** With the flight loop's state in the control thread's frame, an IMU probe's
  float formatting overflowed the 6 KB stack (a `verify` crashed the board). The loop moved to the heap, its
  `new`/`tick` stay out of the loop's frame, and the stack grew to 8 KB: 3.8 KB is never used through a whole glide.
- **The harness flies plan B's gains:** `hitl_config` turns the board's flight loop on with `config_hitl`'s
  test gains (read from plan B's config, one source), and refuses a config without exactly one flight entry.
- **The overlay** (`tools/hitl_overlay.py`) gained the law's roll setpoint and command, the left eleron, the landing
  zone on the track, and the touchdown miss in its table.

Not here yet: the 12-scenario matrix against plan B's spread (M14).
