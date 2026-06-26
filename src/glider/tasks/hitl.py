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

import asyncio
import random

import controller as controller_mod
import databoard
import inspector
import sim_model
import task

_STAGE = controller_mod.Stage
_HPRC = sim_model.HPRC      # default scenario (HPRC launch site + landing zone)
_MOTORS = sim_model.MOTORS  # thrust/burn per motor
Body = sim_model.Body       # the pure flight-dynamics model
_noisy = sim_model.noisy    # the sensor-noise helper


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
        motor = _MOTORS.get(cfg.get('motor', 'F15'), _MOTORS['F15'])
        self._thrust, self._burn_s = motor
        mass = cfg.get('liftoff_g', 430) / 1000.0
        self._body = Body(mass, tuple(scenario['launch']), scenario['elevation_m'], scenario['heading_deg'])
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
        self._ok = True
        return True

    def _read_fins(self) -> tuple:
        """The commanded (roll, pitch, yaw) deflections in degrees from neutral (90), recovered from the
        cached servo angles the flight loop wrote (mixer: elevons common=pitch, differential=roll)."""
        if self._fins is None:
            self._fins = self.controller.find(['servo_eleron_left', 'servo_eleron_right', 'servo_yaw'])
        left, right, yaw = (getattr(f, 'angle', 90) or 90 for f in self._fins)
        return ((left - right) / 2.0, (left + right) / 2.0 - 90.0, yaw - 90.0)

    def _publish(self) -> None:
        """Push the (noised) simulated sensors onto the databoard."""
        s = self._body.sensors()
        n = self._noise
        self._ch['accel'].push((0.0, 0.0, _noisy(s['accel'], n, -200.0, 200.0)))  # |a| on z (g)
        self._ch['attitude'].push((_noisy(s['heading'], n, 0.0, 360.0),
                                   _noisy(s['roll'], n, -180.0, 180.0), _noisy(s['pitch'], n, -180.0, 180.0)))
        agl = s['agl']
        if agl <= self._laser_range_m:  # laser only sees the ground within its range (g15: 3 m yes, 6 m no)
            self._ch['agl'].push(_noisy(agl, n, 0.0, 1000.0))
        self._ch['altitude'].push(_noisy(s['altitude'], n, -100.0, 10000.0))
        self._ch['elevation'].push(_noisy(s['altitude'] - self._body.elev0, n, -100.0, 10000.0))
        self._ch['position'].push(s['position'])
        self._ch['speed'].push(_noisy(s['speed'], n, 0.0, 200.0))  # true airspeed (m/s) -> fin governor (g12)

    async def run(self) -> None:
        dt = 1.0 / self._sim_hz
        t = 0.0
        period = max(1, 1000 // self._sim_hz)
        while True:
            # follow the REAL stage machine: SETTING/BOOSTING -> 1-DoF boost/coast (provides the launch
            # accel + altitude that drive the sequencer); GLIDING/LANDING -> fin-controlled 6-DoF glide.
            if self.controller.stage < _STAGE.GLIDING:
                roll, pitch, _yaw = self._read_fins()  # g12: the boost stage holds vertical via the fins
                self._body.boost_step(dt, self._thrust if t < self._burn_s else 0.0, pitch, roll)
            else:
                if not self._body.gliding:
                    self._body.begin_glide()                 # BOOSTING -> GLIDING: deploy + glide
                roll, pitch, yaw = self._read_fins()
                if self._spike and random.random() < dt / 3.0:   # g16: a 2x roll spike every ~3 s
                    roll *= 2.0
                self._body.glide_step(dt, roll, pitch, yaw)
            self._publish()
            t += dt
            await asyncio.sleep_ms(period)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        s = self._body.sensors()
        status['gliding'] = self._body.gliding
        status['alt'] = round(s['altitude'], 1)
        status['heading'] = round(s['heading'], 1)
        status['noise'] = self._noise
        return status
