"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Hardware-In-The-Loop flight simulator (Phase-5). @task.activity('hitl').

Closes the control loop ON THE BOARD without changing any production code: it reads the commanded fin
angles from the cached servo tasks, steps a flight-dynamics model (sim_model.Body), and PROVIDES the
resulting sensor quantities on the databoard at priority 0 -- so sequencer.py / flight.py / pid / mixer /
navigation read it and cannot tell it is simulated. The full chain runs closed-loop: sim sensors ->
sequencer (stage machine) -> flight (PID -> mixer -> fins) -> back into the model. Use with config_hitl
(real sensors off, this on, flight + sequencer enabled, watchdog off). The physics live in sim_model.py
(pure, shared with the host-side tools/virtual_flight.py -- same model, both worlds).

Fidelity: BOOST adds attitude under thrust -- a crosswind weathercocks the stack and the boost stage's
guarded fins fight to hold it vertical, on top of the vertical 1-DoF that drives launch detect + apogee;
the GLIDE is a rigid body with roll/pitch/yaw state driven by the elevon/rudder deflections the flight
loop commands (that is where the rest of control happens). Aero is simplified and the coefficients are
deliberately tunable -- the point is a stable, closed loop that exercises the control code, not
aerodynamic truth. Outputs are perturbed by a noise level N and optional 2x spikes to study
sensor-quality degradation (e.g. the laser dropping out beyond its range).

The simulated sensors are ALSO recorded as telemetry under the SAME csv names/fields as the real drivers
(accel_adxl375 / imu_bno055 / baro_icp10111 / gnss / laser_agl + a combined fins), so an on-board HITL
run produces a COMPLETE, renderable capture on the Luckfox (flight_report/flight_svg), not just
health/sequencer/servo. The records are decimated so the recorder link keeps up.
"""

import asyncio
import math
import random
import time

import commons
import controller as controller_mod
import databoard
import inspector
import recorder
import sim_model
import task
from fixed import SCALE, from_float  # sim->control boundary: centidegree/Pa fixnums

_STAGE = controller_mod.Stage
_HPRC = sim_model.HPRC      # default scenario (HPRC launch site + landing zone)
_MOTORS = sim_model.MOTORS  # thrust/burn per motor
Body = sim_model.Body       # the pure flight-dynamics model
_noisy = sim_model.noisy    # the sensor-noise helper
_KNOTS = 1.94384            # m/s -> knots (GNSS speed convention)
_BARO_NOISE_SCALE = 0.05    # baro is ~20x more precise than the IMU/GNSS -> its noise is this x the nominal
_PITOT_RAIL_MS = (2.0 * 546.0 / sim_model.RHO) ** 0.5  # the +/-500 Pa SDP810 rails ~29.9 m/s (matches the host sim)


@task.activity('hitl')
class Hitl(task.Task):
    """The HITL simulator task: drive the model from the commanded fins and publish simulated sensors."""

    async def setup(self) -> bool:
        cfg = self.config
        scenario = dict(_HPRC)
        scenario.update(cfg.get('scenario', {}))
        self._sim_hz: int = cfg.get('sim_hz', 50)
        """
        INJECT rate = how often sensors are published to the databoard + telemetry (the sim's per-tick
        allocation). Decoupled from sim_hz (the physics INTEGRATION rate, kept for boost/apogee fidelity):
        the accumulator covers wall time with sim_hz sub-steps regardless of the loop rate. Default =
        sim_hz (publish every step, as before); lower it (e.g. 10) to slim the sim's own heap churn so an
        on-board HITL leak reflects real flight -- see doc/sims/TMS-7-memory_refactoring.
        """
        self._inject_hz: int = cfg.get('inject_hz', self._sim_hz)
        self._noise: float = cfg.get('noise', 0.0)             # N: 0.0 / 0.05 / 0.10 / 0.25 / 0.50
        self._laser_range_m: float = cfg.get('laser_range_m', 4.0)  # agl drops out beyond this
        self._spike: bool = cfg.get('spike', False)            # occasional 2x spikes
        # PAD DWELL (s): hold the stack armed + stationary on the pad before ignition (0 = ignite at once,
        # the default -> unchanged). A realistic pad wait, and it lets the GNSS drift calibration sample.
        self._pad_dwell_s: float = cfg.get('pad_dwell_s', 0.0)
        """
        which accelerometer axis carries the boost |a|: on the rod the IMU long-axis (often X or Y, since
        the board's Z is normal to the PCB) reads the thrust. Launch-detect is magnitude-based
        (axis-agnostic), so this is for matching the real mounting / exercising per-axis code.
        """
        self._axis_index: int = {'x': 0, 'y': 1, 'z': 2}.get(cfg.get('boost_axis', 'z'), 2)
        motor = _MOTORS.get(cfg.get('motor', 'F15'), _MOTORS['F15'])
        self._thrust, self._burn_s = motor
        mass = cfg.get('liftoff_g', 517) / 1000.0            # boost: whole stack (booster + glider)
        glide_mass = cfg.get('glider_g', 300) / 1000.0       # glide: glider alone (booster ejected)
        self._body = Body(mass, tuple(scenario['launch']), scenario['elevation_m'], scenario['heading_deg'],
                          glide_mass=glide_mass)
        # steady wind the glide must crab against (m/s, toward wind_dir degrees) -- a glide disturbance
        wind = cfg.get('wind', 0.0)
        wind_dir = cfg.get('wind_dir', 0.0)
        self._body.wind_e = wind * math.sin(math.radians(wind_dir))
        self._body.wind_n = wind * math.cos(math.radians(wind_dir))
        # GNSS consistent drift (m/s, toward gnss_drift_dir): a stationary receiver's slow apparent motion,
        # present in the reported ground velocity even on the pad -> the SETTING calibration measures it.
        drift = cfg.get('gnss_drift', 0.0)
        drift_dir = cfg.get('gnss_drift_dir', 0.0)
        self._body.gnss_drift_e = drift * math.sin(math.radians(drift_dir))
        self._body.gnss_drift_n = drift * math.cos(math.radians(drift_dir))
        self._fins = None
        # seed the mission with the scenario (launch point + landing zone) so the nav has a target
        mission = inspector.Inspector.get('mission')
        if mission is not None:
            mission.update({'latitude': scenario['launch'][0], 'longitude': scenario['launch'][1],
                            'altitude': scenario['elevation_m'], 'zone': scenario['zone']})
        # provide the sim's sensor quantities to the databoard (priority 0 -> the control code reads these)
        """
        `airspeed`/`dynamic_pressure` are here for a reason worth remembering: without them the sim
        published no airspeed at all, so the ONLY publisher of the fused `airspeed` channel on a board
        HITL flight was the REAL bench SDP810 sitting in still air. That is what caused the bistability
        (findings §28) -- fixed then with the `pitot_min_ms` consumer guard, which was right for a
        blocked tube in flight but left the harness itself lying: board HITL never exercised the pitot
        path, and the host sim (which does simulate it) flew the governor off a different source.
        config_hitl._SIM_SENSORS now masks `airspeed_sdp810` too, so the bench part cannot get in.
        """
        provided = {name: {'priority': 0, 'timeout_ms': 1000} for name in
                    ('accel', 'attitude', 'rate', 'agl', 'altitude', 'elevation', 'position', 'speed',
                     'course', 'airspeed', 'dynamic_pressure')}
        self._ch = databoard.Databoard.provide(self.name, provided)
        """
        attitude-redundancy validation: a runner flips this True mid-glide to simulate a BNO055 death
        (stop publishing the sim `attitude`); accel + rate keep flowing, so the priority-1 attitude backup
        (tasks/attitude.py) must take over the fused slot and keep the glider controllable.
        """
        self.drop_attitude: bool = False
        # simulated GNSS dropout (tunnel / antenna knock): stop publishing position/speed/course so the
        # guidance falls to its open-loop heading tiers and the wind feed stalls -- baro (altitude) stays.
        self.drop_gnss: bool = False
        # record the simulated sensors as telemetry (same names/fields as the real drivers + the host
        # tool) -> a complete renderable capture on the Luckfox. Decimated to keep the link sane.
        sensor_us = int(1_000_000 / cfg.get('record_hz', 25))   # sensor telemetry cadence
        self._tlm_accel = recorder.Telemetry('accel_adxl375.csv', ('ax', 'ay', 'az'), sensor_us)
        self._tlm_imu = recorder.Telemetry('imu_bno055.csv', ('heading', 'roll', 'pitch'), sensor_us)
        self._tlm_gyro = recorder.Telemetry('imu_lsm6dso32.csv', ('ax', 'ay', 'az', 'gx', 'gy', 'gz'), sensor_us)
        self._tlm_baro = recorder.Telemetry('baro_icp10111.csv',
                                            ('altitude', 'temperature', 'pressure', 'elevation'), sensor_us)
        self._tlm_gnss = recorder.Telemetry('gnss.csv', ('lat', 'lon', 'speed_kn', 'course'), 100_000)  # 10 Hz
        self._tlm_laser = recorder.Telemetry('laser_agl.csv', ('agl',), sensor_us)
        self._tlm_fins = recorder.Telemetry('fins.csv', ('eleron_left', 'eleron_right', 'yaw'), sensor_us)
        """
        SIM CLOCK vs WALL CLOCK. The accumulator caps each iteration at `max_catchup`, so wall time the
        loop could not cover is DROPPED and never made up -- simulated time falls permanently behind.
        That is invisible in a capture whose every row is wall-stamped, and it is why board flights
        read 12-21 % LONGER than the same case on the host: the trajectory is identical, the clock
        reporting it is not. Recording both makes the lag measurable instead of inferred, so a
        cross-world duration comparison can be done in SIM time where it is meaningful.
        """
        self._tlm_clock = recorder.Telemetry('hitl_clock.csv', ('sim_s', 'wall_s', 'lag_s'), 100_000)
        self._tlm_pitot = recorder.Telemetry(  # same name/fields/units as drivers/sdp810.py
            'airspeed_sdp810.csv', ('dynamic_pressure', 'airspeed_cms', 'temperature'), sensor_us)
        self._ok = True
        return True

    def _fin_angles(self) -> tuple:
        """
        Raw commanded servo angles read back from the cached servo tasks the flight loop writes.

        Args:
            (none)

        Returns:
            (eleron_left, eleron_right, yaw) in degrees (90 = neutral); (90, 90, 90) before the servos
            are found.
        """
        if self._fins is None:
            self._fins = self.controller.find(['servo_eleron_left', 'servo_eleron_right', 'servo_yaw'])
        neutral = commons.SERVO_NEUTRAL_DEG
        return tuple(neutral if fin is None else (fin.angle or neutral) for fin in self._fins)

    def _read_fins(self) -> tuple:
        """
        The commanded (roll, pitch, yaw) deflections in degrees from neutral (90).

        Recovered from the cached servo angles the flight loop wrote (mixer: elevons common = pitch,
        differential = roll).

        Args:
            (none)

        Returns:
            (roll, pitch, yaw) deflections in degrees from neutral.
        """
        left, right, yaw = self._fin_angles()
        return ((left - right) / 2.0, (left + right) / 2.0 - commons.SERVO_NEUTRAL_DEG,
                yaw - commons.SERVO_NEUTRAL_DEG)

    def _publish(self) -> None:
        """
        Push the (noised) simulated sensors onto the databoard and record them as telemetry.

        Runs every inject step: the control loop reads the databoard channels, and each sensor is also
        recorded as decimated telemetry (the recorder rate-limits each stream). Reads the Body state
        directly (not the sensors() dict) so no dict is allocated per tick.

        Args:
            (none)

        Returns:
            None; publishes the databoard channels and pushes the telemetry rows as a side effect.
        """
        body = self._body
        noise = self._noise
        accel = [0.0, 0.0, 0.0]
        accel[self._axis_index] = _noisy(body.accel_g, noise, -200.0, 200.0)  # |a| on the boost axis (g)
        # CIRCULAR: an absolute error band, not a fraction of a 0..360 magnitude (sim_model.noisy)
        heading = _noisy(body.heading % 360.0, noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)
        roll = _noisy(body.roll, noise, -180.0, 180.0)
        pitch = _noisy(body.pitch, noise, -180.0, 180.0)
        """
        the baro is far more precise than the IMU/GNSS -- its noise is ~sub-metre absolute, not the nominal
        % of the reading. Scale it down (5 % nominal -> ~0.25 %, ~0.6 m at 250 m) so the reading is
        realistic AND the sequencer's baro apogee-detect is not swamped by fake ±10 m jitter.
        """
        baro_noise = noise * _BARO_NOISE_SCALE
        altitude = _noisy(body.elev0 + body.alt, baro_noise, -100.0, 10000.0)
        elevation = _noisy(body.alt, baro_noise, -100.0, 10000.0)  # altitude above the pad (= altitude - elev0)
        speed = _noisy(body.ground_speed(), noise, 0.0, 200.0)  # GNSS ground speed (2D, WITH wind) -> governor + wind
        agl_clean = max(0.0, body.alt)
        position = body.position()
        course = _noisy(body.track(), noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)  # ground track -> yaw ref
        # databoard -> the control loop. roll/pitch are centidegree fixnum for the fixed-point PID (heading
        # stays float for the nav trig); the sim's float physics wraps to fixnum once, here at the boundary.
        self._ch['accel'].push((accel[0], accel[1], accel[2]))
        if not self.drop_attitude:  # simulated BNO055 death -> the priority-1 backup must carry attitude
            self._ch['attitude'].push((heading, from_float(roll), from_float(pitch)))
        """
        gyro rate -> the PID D term (rate damping). Noised deg/s (like the other IMU channels so the D term
        sees real jitter), pushed as centideg/s fixnum -- same unit + mapping as the LSM6DSO32 (roll,
        pitch, yaw). The same noised values feed the imu_lsm6dso32 telemetry below (board parity).
        """
        roll_rate = _noisy(body.roll_rate, noise, -2000.0, 2000.0)
        pitch_rate = _noisy(body.pitch_rate, noise, -2000.0, 2000.0)
        yaw_rate = _noisy(body.yaw_rate, noise, -2000.0, 2000.0)
        self._ch['rate'].push((from_float(roll_rate), from_float(pitch_rate), from_float(yaw_rate)))
        in_range = agl_clean <= self._laser_range_m  # laser only sees the ground within its range
        if in_range:
            self._ch['agl'].push(_noisy(agl_clean, noise, 0.0, 1000.0))
        self._ch['altitude'].push(altitude)
        self._ch['elevation'].push(elevation)
        if not self.drop_gnss:  # simulated GNSS dropout -> position/speed/course go stale (baro stays)
            self._ch['position'].push(position)
            self._ch['speed'].push(speed)
            self._ch['course'].push(course)
        """
        SDP810 pitot: the AIR-relative speed (body.speed is horizontal airspeed, vu vertical), NOT the
        ground speed above -- a pitot cannot see wind. Railed like the real ±500 Pa part, which tops out
        near 30 m/s: past that it under-reads, and an under-read LOOSENS the fin cap, so the governor's
        `pitot_max_ms` guard must have something to fire on. Noised like every other channel here, which
        the host sim does not do -- the board harness models the sensor, the host models the physics.
        """
        pitot = min((body.speed * body.speed + body.vu * body.vu) ** 0.5, _PITOT_RAIL_MS)
        pitot = _noisy(pitot, noise, 0.0, _PITOT_RAIL_MS)
        pressure_fx = int(0.5 * sim_model.RHO * pitot * pitot * SCALE)  # Pa fixnum, as the driver pushes
        self._ch['airspeed'].push(pitot)              # m/s float -> the governor's estimator
        self._ch['dynamic_pressure'].push(pressure_fx)  # Pa fixnum -> flight_report / airspeed_calibrate
        self._tlm_pitot.push((pressure_fx, int(pitot * SCALE), 2500))  # temperature 25.00 C, as the host writes
        # telemetry -> the Luckfox (decimate_us rate-limits each stream so this can run every step)
        self._tlm_accel.push((round(accel[0], 3), round(accel[1], 3), round(accel[2], 3)))
        self._tlm_imu.push((round(heading, 1), round(roll, 1), round(pitch, 1)))
        # the LSM6DSO32 stream a real board flight produces: low-g accel (reuse the boost-axis accel) +
        # the gyro rate (deg/s) the PID reads. Same fields as drivers/lsm6dso32 so flight_report keys on it.
        self._tlm_gyro.push((round(accel[0], 3), round(accel[1], 3), round(accel[2], 3),
                             round(roll_rate, 1), round(pitch_rate, 1), round(yaw_rate, 1)))
        self._tlm_baro.push((round(altitude, 2), 21.0, 100000, round(elevation, 2)))
        if in_range:
            self._tlm_laser.push((round(agl_clean, 3),))
        left, right, yaw = self._fin_angles()
        self._tlm_fins.push((int(left), int(right), int(yaw)))
        lat, lon = position
        self._tlm_gnss.push(('%.6f' % lat, '%.6f' % lon, round(speed * _KNOTS, 1), round(heading, 1)))

    async def run(self) -> None:
        """
        FIXED-TIMESTEP ACCUMULATOR. The sim must track the WALL clock, because the sequencer's stage
        timeouts (launch dwell, the boost->glide burnout/ejection timeout, ground dwell) are wall-clock
        (ticks_ms) -- if sim-time and wall-time drift, the stages fire at the wrong altitude (a fixed dt
        per iteration flew the model ~3x realtime, past apogee and underground before the 6 s timeout;
        naively clamping the measured dt does the reverse, throttling the sim below wall-time so the glide
        never reaches the ground). So each iteration measures the real elapsed time and advances the model
        in stable `fixed`-size sub-steps to COVER it: integration stays at sim_hz (accurate), while the
        number of sub-steps floats with the loop rate so 1 sim-second == 1 wall-second. A big one-off stall
        is capped so it cannot inject a burst of catch-up steps.
        """
        period = max(1, 1000 // self._inject_hz)  # loop + publish cadence (inject rate)
        fixed = 1.0 / self._sim_hz                 # physics sub-step (integration rate, wall-time covered)
        max_catchup = 0.5            # s: cap a scheduling stall's catch-up (<= 0.5 s of sub-steps)
        sim_time = 0.0
        accumulator = 0.0
        last = time.ticks_ms()
        wall_start = last  # for the sim-vs-wall clock record; see the hitl_clock.csv declaration
        while True:
            await asyncio.sleep_ms(period)
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, last) / 1000.0
            last = now
            """
            PAD DWELL: armed + stationary on the pad -> the sequencer stays in SETTING (accel 1 g < the
            launch threshold) while the GNSS drift calibration samples. Hold the body (no boost integration,
            no accumulator backlog) until the dwell elapses, then ignition proceeds.
            """
            if self._pad_dwell_s > 0.0 and self.controller.stage == _STAGE.SETTING and not self._body.gliding:
                self._pad_dwell_s -= elapsed
                self._body.accel_g = 1.0
                accumulator = 0.0
                self._publish()
                continue
            if self.controller.stage >= _STAGE.DONE:
                """
                The flight is over: stop FLYING it. The runner still holds the loop open for ~1.2 s to
                let the recorder drain its tail to the Luckfox, and the sim used to keep stepping the
                body and publishing sensors through that -- so every capture ended with ~1.2 s of
                samples of a glider sitting on the ground, visible on the reports as traces carrying
                on past DONE. It also inflated the capture's duration, which is one of the numbers
                the host and board were being compared on.
                """
                accumulator = 0.0
                continue
            accumulator += elapsed if elapsed < max_catchup else max_catchup
            # follow the REAL stage machine: SETTING/BOOSTING -> 1-DoF boost/coast (provides the launch
            # accel + altitude that drive the sequencer); GLIDING/LANDING -> fin-controlled 6-DoF glide.
            boosting = self.controller.stage < _STAGE.GLIDING
            if boosting:
                roll, pitch, _yaw = self._read_fins()  # the boost stage holds vertical via the fins
            else:
                if not self._body.gliding:
                    self._body.begin_glide()                 # BOOSTING -> GLIDING: deploy + glide
                roll, pitch, yaw = self._read_fins()
            while accumulator >= fixed:                      # advance enough sub-steps to cover real time
                if boosting:
                    self._body.boost_step(fixed, self._thrust if sim_time < self._burn_s else 0.0, pitch, roll)
                else:
                    spiked = roll * 2.0 if (self._spike and random.random() < fixed / 3.0) else roll
                    self._body.glide_step(fixed, spiked, pitch, yaw)  # occasional 2x roll spike
                accumulator -= fixed
                sim_time += fixed
            self._publish()
            # sim vs wall, decimated to 10 Hz: lag > 0 means the accumulator dropped uncoverable
            # wall time, so a wall-stamped duration OVER-reads the flight by exactly this much
            wall = time.ticks_diff(now, wall_start) / 1000.0
            self._tlm_clock.push((round(sim_time, 2), round(wall, 2), round(wall - sim_time, 2)))

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        sensors = self._body.sensors()
        status['gliding'] = self._body.gliding
        status['alt'] = round(sensors['altitude'], 1)
        status['heading'] = round(sensors['heading'], 1)
        status['noise'] = self._noise
        return status
