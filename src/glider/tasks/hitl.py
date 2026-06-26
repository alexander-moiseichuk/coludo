# tasks/hitl.py — Hardware-In-The-Loop flight simulator (Phase-5, 6/23 g15). @task.activity('hitl').
#
# Closes the control loop ON THE BOARD without changing any production code: it reads the commanded fin
# angles from the cached servo tasks, steps a flight-dynamics model (sim_model.Body), and PROVIDES the
# resulting sensor quantities on the databoard at priority 0 -- so sequencer.py / flight.py / pid /
# mixer / navigation read it and cannot tell it is simulated. The full chain runs closed-loop: sim
# sensors -> sequencer (stage machine) -> flight (PID -> mixer -> fins) -> back into the model. Use with
# config_hitl (real sensors off, this on, flight + sequencer enabled, watchdog off). The physics live in
# sim_model.py (pure, shared with the host-side tools/virtual_flight.py -- same model, both worlds).
#
# Fidelity: BOOST adds attitude under thrust (g12) -- a crosswind weathercocks the stack and the boost
# stage's guarded fins fight to hold it vertical, on top of the vertical 1-DoF that drives launch detect +
# apogee; the GLIDE is a rigid body with roll/pitch/yaw state driven by the elevon/rudder deflections the
# flight loop commands (that is where the rest of control happens). Aero is simplified and the
# coefficients are deliberately tunable -- the point is a stable, closed loop that exercises the control
# code, not aerodynamic truth. Outputs are perturbed by a noise level N (g15) and optional 2x spikes
# (g16) to study sensor-quality degradation (e.g. the laser dropping out beyond its range).
#
# The simulated sensors are ALSO recorded as telemetry under the SAME csv names/fields as the real
# drivers (accel_adxl375 / imu_bno055 / baro_icp10111 / gnss / laser_agl + a combined fins), so an
# on-board HITL run produces a COMPLETE, renderable capture on the Luckfox (flight_report/flight_svg),
# not just health/sequencer/servo. The records are decimated so the recorder link keeps up.

import asyncio
import math
import random
import time

import controller as controller_mod
import databoard
import inspector
import recorder
import sim_model
import task

_STAGE = controller_mod.Stage
_HPRC = sim_model.HPRC      # default scenario (HPRC launch site + landing zone)
_MOTORS = sim_model.MOTORS  # thrust/burn per motor
Body = sim_model.Body       # the pure flight-dynamics model
_noisy = sim_model.noisy    # the sensor-noise helper
_KNOTS = 1.94384            # m/s -> knots (GNSS speed convention)


@task.activity('hitl')
class Hitl(task.Task):
    """The HITL simulator task: drive the model from the commanded fins and publish simulated sensors."""

    async def setup(self) -> bool:
        cfg = self.config
        scenario = dict(_HPRC)
        scenario.update(cfg.get('scenario', {}))
        self._sim_hz: int = cfg.get('sim_hz', 50)
        self._noise: float = cfg.get('noise', 0.0)             # N: 0.0 / 0.05 / 0.10 / 0.25 / 0.50
        self._laser_range_m: float = cfg.get('laser_range_m', 4.0)  # agl drops out beyond this (g15)
        self._spike: bool = cfg.get('spike', False)            # g16: occasional 2x spikes
        # which accelerometer axis carries the boost |a|: on the rod the IMU long-axis (often X or Y,
        # since the board's Z is normal to the PCB) reads the thrust. Launch-detect is magnitude-based
        # (axis-agnostic), so this is for matching the real mounting / exercising per-axis code.
        self._axis_index: int = {'x': 0, 'y': 1, 'z': 2}.get(cfg.get('boost_axis', 'z'), 2)
        motor = _MOTORS.get(cfg.get('motor', 'F15'), _MOTORS['F15'])
        self._thrust, self._burn_s = motor
        mass = cfg.get('liftoff_g', 430) / 1000.0
        self._body = Body(mass, tuple(scenario['launch']), scenario['elevation_m'], scenario['heading_deg'])
        # steady wind the glide must crab against (m/s, toward wind_dir degrees) -- a glide disturbance
        wind = cfg.get('wind', 0.0)
        wind_dir = cfg.get('wind_dir', 0.0)
        self._body.wind_e = wind * math.sin(math.radians(wind_dir))
        self._body.wind_n = wind * math.cos(math.radians(wind_dir))
        self._fins = None
        # seed the mission with the scenario (launch point + landing zone) so the nav has a target
        mission = inspector.Inspector.get('mission')
        if mission is not None:
            mission.update({'latitude': scenario['launch'][0], 'longitude': scenario['launch'][1],
                            'altitude': scenario['elevation_m'], 'zone': scenario['zone']})
        # provide the sim's sensor quantities to the databoard (priority 0 -> the control code reads these)
        provided = {q: {'priority': 0, 'timeout_ms': 1000} for q in
                    ('accel', 'attitude', 'agl', 'altitude', 'elevation', 'position', 'speed')}
        self._ch = databoard.Databoard.provide(self.name, provided)
        # record the simulated sensors as telemetry (same names/fields as the real drivers + the host
        # tool) -> a complete renderable capture on the Luckfox. Decimated to keep the link sane.
        sensor_us = int(1_000_000 / cfg.get('record_hz', 25))   # sensor telemetry cadence
        self._tlm_accel = recorder.Telemetry('accel_adxl375.csv', ('ax', 'ay', 'az'), sensor_us)
        self._tlm_imu = recorder.Telemetry('imu_bno055.csv', ('heading', 'roll', 'pitch'), sensor_us)
        self._tlm_baro = recorder.Telemetry('baro_icp10111.csv',
                                            ('altitude', 'temperature', 'pressure', 'elevation'), sensor_us)
        self._tlm_gnss = recorder.Telemetry('gnss.csv', ('lat', 'lon', 'speed_kn', 'course'), 100_000)  # 10 Hz
        self._tlm_laser = recorder.Telemetry('laser_agl.csv', ('agl',), sensor_us)
        self._tlm_fins = recorder.Telemetry('fins.csv', ('eleron_left', 'eleron_right', 'yaw'), sensor_us)
        self._ok = True
        return True

    def _fin_angles(self) -> tuple:
        """Raw commanded servo angles (eleron_left, eleron_right, yaw) in degrees, from the cached servo
        tasks the flight loop writes (90 = neutral). (90, 90, 90) before the servos are found."""
        if self._fins is None:
            self._fins = self.controller.find(['servo_eleron_left', 'servo_eleron_right', 'servo_yaw'])
        return tuple(getattr(f, 'angle', 90) or 90 for f in self._fins)

    def _read_fins(self) -> tuple:
        """The commanded (roll, pitch, yaw) deflections in degrees from neutral (90), recovered from the
        cached servo angles the flight loop wrote (mixer: elevons common=pitch, differential=roll)."""
        left, right, yaw = self._fin_angles()
        return ((left - right) / 2.0, (left + right) / 2.0 - 90.0, yaw - 90.0)

    def _publish(self) -> None:
        """Push the (noised) simulated sensors onto the databoard every step (the control loop reads
        them), and record them as decimated telemetry (the recorder rate-limits each stream)."""
        s = self._body.sensors()
        n = self._noise
        accel = [0.0, 0.0, 0.0]
        accel[self._axis_index] = _noisy(s['accel'], n, -200.0, 200.0)  # |a| on the configured boost axis (g)
        heading = _noisy(s['heading'], n, 0.0, 360.0)
        roll = _noisy(s['roll'], n, -180.0, 180.0)
        pitch = _noisy(s['pitch'], n, -180.0, 180.0)
        altitude = _noisy(s['altitude'], n, -100.0, 10000.0)
        elevation = _noisy(s['altitude'] - self._body.elev0, n, -100.0, 10000.0)
        speed = _noisy(s['speed'], n, 0.0, 200.0)
        agl_clean = s['agl']
        # databoard -> the control loop
        self._ch['accel'].push((accel[0], accel[1], accel[2]))
        self._ch['attitude'].push((heading, roll, pitch))
        in_range = agl_clean <= self._laser_range_m  # laser only sees the ground within its range (g15)
        if in_range:
            self._ch['agl'].push(_noisy(agl_clean, n, 0.0, 1000.0))
        self._ch['altitude'].push(altitude)
        self._ch['elevation'].push(elevation)
        self._ch['position'].push(s['position'])
        self._ch['speed'].push(speed)
        # telemetry -> the Luckfox (decimate_us rate-limits each stream so this can run every step)
        self._tlm_accel.push((round(accel[0], 3), round(accel[1], 3), round(accel[2], 3)))
        self._tlm_imu.push((round(heading, 1), round(roll, 1), round(pitch, 1)))
        self._tlm_baro.push((round(altitude, 2), 21.0, 100000, round(elevation, 2)))
        if in_range:
            self._tlm_laser.push((round(agl_clean, 3),))
        left, right, yaw = self._fin_angles()
        self._tlm_fins.push((int(left), int(right), int(yaw)))
        lat, lon = s['position']
        self._tlm_gnss.push(('%.6f' % lat, '%.6f' % lon, round(speed * _KNOTS, 1), round(heading, 1)))

    async def run(self) -> None:
        # FIXED-TIMESTEP ACCUMULATOR. The sim must track the WALL clock, because the sequencer's stage
        # timeouts (launch dwell, the boost->glide burnout/ejection timeout, ground dwell) are wall-clock
        # (ticks_ms) -- if sim-time and wall-time drift, the stages fire at the wrong altitude (a fixed dt
        # per iteration flew the model ~3x realtime, past apogee and underground before the 6 s timeout;
        # naively clamping the measured dt does the reverse, throttling the sim below wall-time so the
        # glide never reaches the ground). So each iteration measures the real elapsed time and advances
        # the model in stable `fixed`-size sub-steps to COVER it: integration stays at sim_hz (accurate),
        # while the number of sub-steps floats with the loop rate so 1 sim-second == 1 wall-second. A big
        # one-off stall is capped so it cannot inject a burst of catch-up steps.
        period = max(1, 1000 // self._sim_hz)
        fixed = 1.0 / self._sim_hz
        max_catchup = 0.5            # s: cap a scheduling stall's catch-up (<= 0.5 s of sub-steps)
        t = 0.0
        accumulator = 0.0
        last = time.ticks_ms()
        while True:
            await asyncio.sleep_ms(period)
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, last) / 1000.0
            last = now
            accumulator += elapsed if elapsed < max_catchup else max_catchup
            # follow the REAL stage machine: SETTING/BOOSTING -> 1-DoF boost/coast (provides the launch
            # accel + altitude that drive the sequencer); GLIDING/LANDING -> fin-controlled 6-DoF glide.
            boosting = self.controller.stage < _STAGE.GLIDING
            if boosting:
                roll, pitch, _yaw = self._read_fins()  # g12: the boost stage holds vertical via the fins
            else:
                if not self._body.gliding:
                    self._body.begin_glide()                 # BOOSTING -> GLIDING: deploy + glide
                roll, pitch, yaw = self._read_fins()
            while accumulator >= fixed:                      # advance enough sub-steps to cover real time
                if boosting:
                    self._body.boost_step(fixed, self._thrust if t < self._burn_s else 0.0, pitch, roll)
                else:
                    spiked = roll * 2.0 if (self._spike and random.random() < fixed / 3.0) else roll
                    self._body.glide_step(fixed, spiked, pitch, yaw)  # g16: occasional 2x roll spike
                accumulator -= fixed
                t += fixed
            self._publish()

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        s = self._body.sensors()
        status['gliding'] = self._body.gliding
        status['alt'] = round(s['altitude'], 1)
        status['heading'] = round(s['heading'], 1)
        status['noise'] = self._noise
        return status
