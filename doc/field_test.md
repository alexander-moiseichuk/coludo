# Field ground test — no-ignition walk test (Phase 5)

The first rung of the flight-test ladder: the fully-assembled boards + firmware + CC, exercised **on the
ground with no motor and an inert booster**. Goal — confirm the whole live stack (sensors → attitude →
guidance → governor → fins → sequencer → recorder → CC link) behaves as it does in HITL, by walking the
glider around and hand-tripping the events a launch would produce (attitude, GNSS motion, a jerk for
boost-detect, a manual separation). No telemetry-launch numbers yet; this is the go/no-go before the
first powered flight.

> **Safety.** No motor is fitted; the booster is inert. **The fins move on their own once armed
> (GLIDING)** — keep fingers and clothing clear of the servos and horns. Keep the CC operator port open
> the whole time so you can disarm / force a stage. A wedged loop reboots itself (hardware watchdog);
> give it ~10 s before intervening.

## Kit
- [ ] Assembled glider (all carriers A–G plugged), inert booster, battery charged
- [ ] **CC host = the panda laptop** (this machine, WiFi AP `192.168.102.1`; the board joins as a TCP
      client) running the CC hub — board port, **operator :1235**, **web :8080**. *Fallback:* a spare PC
      runs CC in the field, then the recordings come back to the **panda** for `flight_report` / `flight_kpi` analysis
- [ ] Recorder (Luckfox) SD has free space; USB/adb cable for the post-test pull
- [ ] Open sky for GNSS; a flat patch to walk (10–30 m) with a marker for the "landing zone"
- [ ] Phone/laptop on the CC web dashboard; a second person helps (one carries, one watches CC)

---

## Phase 0 — Bench pre-checks (before leaving)
- [ ] Latest firmware deployed (`./deploy.sh`), correct board config; `make test` green
- [ ] Board boots to **main.py** running (boot log, not a bare REPL) and connects to CC
- [ ] All components **verify/probe green** on CC — no sensor absent/garbage
- [ ] Fins mechanically free, correct throw; each servo horn on the right spline (re-check after)
- [ ] Confirm **no motor** and the booster is inert

## Phase 1 — Power-up & CC link (at the field)
- [ ] Power on → board joins the AP → appears on the CC dashboard, **stage = SETTING (1)**
- [ ] Every device online: **BNO055** (0x28), **BMP280** (0x76), **ICP-10111** (0x63), **VL53L4CX**
      (0x29), **SDP810** (0x25), **LSM6DSO32** + **ADXL375** (SPI), **GNSS** (uart), **INA226** (i2c1 0x40)
- [ ] Telemetry streaming to CC at the expected rate; `inspect` on each component looks sane
- [ ] `mem_free` stable while idle (no leak on the ground — it scans WiFi in SETTING)

## Phase 2 — Static baseline (glider level and still on the ground)
- [ ] **Attitude:** roll ≈ 0, pitch ≈ 0; rotate to a known heading (e.g. north) → **heading tracks** (magnetometer)
- [ ] **Baro:** altitude/elevation steady; note the pad elevation (ground zero ≈ 0 m)
- [ ] **GNSS:** fix acquired — satellites up, HDOP low; position matches the spot
- [ ] **Airspeed:** dynamic pressure ≈ 0; do the **pad tare** (CC `update {"zero": true}` on
      `airspeed_sdp810`, glider still) → airspeed reads ~0
- [ ] **Fins:** disarmed → all at neutral; verify each fin's **zero/trim** via the CC fin-zero UI
      > ⚠️ **Drive servo checks through the firmware (CC / the driver), never raw PWM on one pin.** A
      > servo whose signal line is left FLOATING hunts on its own, so a bench script that drives one fin
      > and leaves the others unconfigured looks exactly like "all three fins move together" — a false
      > wiring fault that cost real time on 7/25. `sg90.setup()` gives every fin a valid PWM at bring-up,
      > which is why the fault never appears in normal operation. If you must poke a pin directly, hold
      > the other fin pins as **OUTPUT LOW** first.
- [ ] **Power:** INA226 servo-rail voltage/current sane at idle (no stall/short)
- [ ] **Separation switch:** pads nested → pin reads **HIGH = nested**

## Phase 3 — Attitude / IMU (pick it up, rotate, tilt)
- [ ] **CALIBRATE THE BNO055 BEFORE ARMING.** NDOF fusion does not converge without motion, and a
      glider sits still on the pad — so a perfectly healthy part can reach launch with `sys`/`mag`
      calibration at 0 and a FROZEN attitude. Move the airframe in a slow figure-8 until
      `diag_bno_calib.py` shows **mag 3** and the euler bytes changing (took ~1 s on a good part).
      Check the gyro column reads **> 5 °/s** while you do it — a still sample proves nothing, which
      is how a working module was once wrongly condemned
- [ ] Pitch nose up / down → **pitch tracks** the right sense; roll L/R → **roll tracks**
- [ ] Yaw / spin → **heading tracks**; no glitches or freezes on quick moves (gyro rate feeds the PID D-term)
- [ ] Return to level → attitude returns to ~0/0 and the heading settles
- [ ] *(optional, redundancy)* cover/disable the BNO055 mid-test → the complementary-filter **backup**
      takes over (attitude still tracks, degraded) → re-enable

## Phase 4 — Fins track attitude (armed, GLIDING, hand-held)
> Reach GLIDING the realistic way (Phases 7–8) **or** force it from CC for a quick fin check. Armed = fins live.
- [ ] In GLIDING, tilt the glider → **fins deflect to counter** the attitude (stabilisation PID) — confirm the **sense is correct** (a nose-up disturbance drives the fins to push it back)
- [ ] Rotate the glider relative to the landing zone → fins **bias for the bank-to-turn heading** toward the zone
- [ ] At ~0 airspeed the **fin-authority cap is wide** (low q, safe); confirm the governor isn't clamping hard on the ground
- [ ] No servo buzz/overheat; INA226 current stays within the servo-rail budget during active tracking

## Phase 5 — GNSS / position + zone guidance (walk the patch)
- [ ] Set/confirm the **landing zone** in CC (e.g. `assist` hands a launch/zone position → the mission updates)
- [ ] Walk around → **position + ground track** update on CC; the guidance **heading-to-zone** and reachability recompute
- [ ] Walk toward / away / across the zone → the commanded heading (and fins, if armed) update the right way
- [ ] *(if it runs on the ground)* the wind estimate populates from the GNSS triangle — sanity only

## Phase 6 — Airspeed in motion (the new SDP810)

`src/glider/test/live_pitot.py` prints q, airspeed and the governor's own verdict live, so this whole
phase is one run of it: `mpremote connect $PORT run live_pitot.py` (30 s window). Bench-validated
2026-07-26 with the values below, so treat a deviation as a real finding.

- [ ] **At rest** → q sits at the tare floor (**~-0.02 Pa**, ~0.2 m/s equivalent) → verdict **IGNORED
      (below floor)**. A blocked or disconnected tube looks EXACTLY like this, which is why the floor
      exists — a near-zero reading must never reach the estimate
- [ ] Jog with the **pitot exposed to airflow** (or gently blow the **P+** = RIGHT tube) → **TRUSTED (in
      band)**; measured 12–103 Pa → 4.5–13.2 m/s
- [ ] Confirm the governor's airspeed now tracks the **pitot** (the fin-authority cap tightens as airspeed rises)
- [ ] Blow **hard** → the cell **rails at ~546 Pa → 30.4 m/s** (a pinned, repeating value) → verdict
      **IGNORED (railed)** → the governor drops back to the accel+GNSS backbone — the safety path.
      `pitot_max_ms` 28 sits just below the rail, which is what makes the guard fire before the pin
- [ ] At rest again → airspeed returns to ~0 (tare holds)

## Phase 7 — Minimal acceleration / boost-detect (no ignition)
- [ ] Normal handling / walking does **NOT** false-trigger BOOSTING (stays SETTING)
- [ ] A deliberate **hard jerk / toss-and-catch** produces an accel spike ≥ **launch_g (2.5 g)** for `launch_ms` → **SETTING → BOOSTING** (the launch detector). If a hand jerk can't reach it, note it and force BOOSTING from CC for the sequence below
- [ ] *(alt trigger)* the **baro +10 m** backup: lifting the glider ~10 m above the pad also trips BOOSTING regardless of accel — usually impractical on flat ground, note only

## Phase 8 — Separation (the key ground test)
- [ ] Glider nested (pads closed) → pin **HIGH = nested**; get to **BOOSTING** first (Phase 7 jerk, or CC)
- [ ] **Manually separate the pads** → pin **LOW = separated** → **BOOSTING → GLIDING** fires, the control loop **engages, fins go live**
- [ ] Confirm the transition is clean and **latched** (re-nesting does not bounce it back to BOOSTING)
- [ ] Now repeat Phase 4 in this real GLIDING state — fins track attitude + zone heading
- [ ] Let it sit / lower it → the AGL/landing path (VL53L4CX < ~5 m for `land_ms`) → **GLIDING → LANDING → DONE**, fins return to neutral

## Phase 9 — Data capture & review
- [ ] Recorder captured the session (telemetry CSVs growing during the run)
- [ ] Pull the recording (adb from the Luckfox) → `flight_report` / `flight_kpi`
- [ ] `airspeed_sdp810.csv` shows the pitot pressure/airspeed during Phase 6
- [ ] `mem_free` over the whole session shows leak/OOM headroom well past a flight's duration
- [ ] Fin telemetry shows the tracking activity; no wedge/reboot in the logs (unless deliberately tested)

## Cross-cutting — safety, abort, recovery
- [ ] Fingers clear of the fins whenever armed (GLIDING/LANDING)
- [ ] CC operator port stays open → can **disarm / force a stage** at any time
- [ ] If the CDC/board wedges: `pkill mpremote` + reset / power-cycle; it re-enumerates (board recovery)
- [ ] Watchdog: a wedged loop **reboots**; confirm it comes back and re-links to CC (this is a feature to verify, not just a failure mode)

## Pass criteria (go for the next rung)
- All sensors green, telemetry stable, **no memory leak** on the ground
- Attitude **and** fins track in the **correct sense**; the loop is calm at rest (no oscillation)
- GNSS position + landing-zone guidance sane as you walk
- Airspeed reads with motion, **tares to 0** at rest, **rails → accel fallback** when over-driven
- **Separation reliably drives BOOSTING → GLIDING** and arms the fins; the stage machine latches
- The whole run **recorded and downloadable**; board recovers from any wedge

---

Next rung (see `plan.md` / [`board_layout.md`](board_layout.md) §4): the passive-electronics flights (E16/F15,
no active fin control) to build the telemetry pipeline, then active-control powered flights. The one field
calibration this test sets up: the SDP810 **`air_density` trim** — fly a calm steady pass later and run
`tools/airspeed_calibrate.py` on the recording.
