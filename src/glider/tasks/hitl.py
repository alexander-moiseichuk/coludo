# tasks/hitl.py — Hardware-In-The-Loop flight simulator (Phase-5, 6/23 g15). @task.activity('hitl').
#
# Closes the control loop ON THE BOARD without changing any production code: it reads the commanded fin
# angles from the cached servo tasks, steps a flight-dynamics model, and PROVIDES the resulting sensor
# quantities on the databoard at priority 0 -- so sequencer.py / flight.py / pid / mixer / navigation
# read it and cannot tell it is simulated. The full chain runs closed-loop: sim sensors -> sequencer
# (stage machine) -> flight (PID -> mixer -> fins) -> back into the model. Use with config_hitl (real
# sensors off, this on, flight + sequencer enabled, watchdog off).
#
# Fidelity: boost + coast are 1-DoF vertical (only |accel| and altitude matter there -- launch detect
# and apogee); the GLIDE is a rigid body with roll/pitch/yaw state driven by the elevon/rudder
# deflections the flight loop commands (that is where control happens). Aero is simplified and the
# coefficients are deliberately tunable -- the point is a stable, closed loop that exercises the control
# code, not aerodynamic truth. Outputs are perturbed by a noise level N (g15) and optional 2x spikes
# (g16) to study sensor-quality degradation (e.g. the laser dropping out beyond its range).

import asyncio
import math
import random

import controller as controller_mod
import databoard
import inspector
import task

_STAGE = controller_mod.Stage

_G = 9.81
_RHO = 1.225            # sea-level air density (kg/m^3)
_M_PER_DEG = 111320.0   # metres per degree latitude (matches navigation.py)
_CDA = 0.6 * 0.0017     # Cd * frontal area (m^2) from the coludo.md envelope (~46 mm, ~17 cm^2)

# Default scenario (HPRC, Homestead Public Rocketry Club) -- overridable via the hitl config block.
_HPRC = {
    'launch': (25.514379, -80.391795),    # pad (lat, lon)
    'elevation_m': 2.0,                    # pad MSL
    'zone': [[25.514630, -80.392880], [25.514656, -80.391155]],  # TL, BR (a long E-W strip)
    'heading_deg': 30.0,                   # initial glide heading (deg) at separation
}
_MOTORS = {'E16': (16.1, 1.77), 'F15': (14.4, 3.45)}  # average thrust (N), burn (s) -- coludo.md envelope


class Body:
    """Flight-dynamics state + integrator (PURE -- no hardware, host-testable). World frame is ENU
    metres from the launch pad; attitude is Euler degrees (roll, pitch, yaw=heading). `boost_step()`
    climbs vertically; at apogee `begin_glide()` hands over to `glide_step()` (fin-controlled);
    `sensors()` returns what the on-board sensors would read."""

    def __init__(self, mass: float, launch: tuple, elevation_m: float, glide_heading: float):
        self.mass = mass
        self.lat0, self.lon0 = launch
        self.elev0 = elevation_m
        self.glide_heading = glide_heading
        self.pe = 0.0          # position east (m from pad)
        self.pn = 0.0          # position north (m from pad)
        self.alt = 0.0         # altitude above the pad (m)
        self.vu = 0.0          # vertical speed (m/s)
        self.speed = 0.0       # horizontal airspeed (m/s)
        self.heading = glide_heading  # deg (0 = north)
        self.roll = 0.0        # deg
        self.pitch = 90.0      # deg; start nose-up (on the rod, vertical)
        self.accel_g = 1.0     # |specific force| the accelerometer reads (g)
        self.gliding = False

    def boost_step(self, dt: float, thrust: float) -> None:
        """1-DoF vertical: thrust + gravity + drag. The accelerometer reads specific force =
        (thrust - drag)/mass (= kinematic a + g); ~0 g in ballistic coast (free fall)."""
        drag = 0.5 * _RHO * self.vu * abs(self.vu) * _CDA
        specific = (thrust - drag) / self.mass         # what the accelerometer measures (up +)
        self.accel_g = specific / _G if thrust else max(0.0, -drag / self.mass / _G + 0.0)
        self.vu += (specific - _G) * dt                # kinematic accel = specific - g
        self.alt += self.vu * dt

    def begin_glide(self) -> None:
        """Apogee hand-over: nose down to a shallow glide on the configured heading at ~trim speed."""
        self.gliding = True
        self.pitch = -6.0
        self.roll = 0.0
        self.heading = self.glide_heading
        self.speed = 14.0                              # trim airspeed (m/s)
        self.vu = self.speed * math.sin(math.radians(self.pitch))

    def glide_step(self, dt: float, roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> None:
        """Rigid-body glide. Fin deflections (deg from neutral) command roll/pitch; bank turns the
        heading (coordinated turn); a shallow nose-down trim holds the descent. First-order responses
        keep it stable. Eases the airspeed back toward trim."""
        self.roll += (1.2 * roll_cmd - 2.0 * self.roll) * dt        # ailerons -> bank, leveling
        self.pitch += (0.8 * pitch_cmd - 1.5 * (self.pitch + 6.0)) * dt  # elevator -> pitch about -6 trim
        self.roll = max(-60.0, min(60.0, self.roll))
        turn = _G * math.tan(math.radians(self.roll)) / max(self.speed, 5.0)  # rad/s heading rate from bank
        self.heading = (self.heading + math.degrees(turn) * dt + 0.05 * yaw_cmd * dt) % 360.0
        self.speed += (14.0 - self.speed) * 0.5 * dt
        self.vu += ((-_G + _G * math.cos(math.radians(self.roll))) - 0.1 * self.vu) * dt  # sink, more in a bank
        self.vu = self.vu - 0.4 * (self.pitch + 6.0) * dt            # pitch trims the sink rate
        self.alt += self.vu * dt
        self.pe += self.speed * math.sin(math.radians(self.heading)) * dt
        self.pn += self.speed * math.cos(math.radians(self.heading)) * dt
        self.accel_g = 1.0 / max(0.3, math.cos(math.radians(self.roll)))  # load factor rises in a bank

    def position(self) -> tuple:
        lat = self.lat0 + self.pn / _M_PER_DEG
        lon = self.lon0 + self.pe / (_M_PER_DEG * math.cos(math.radians(self.lat0)))
        return (lat, lon)

    def sensors(self) -> dict:
        """Clean (pre-noise) sensor readings from the current state."""
        return {
            'accel': self.accel_g, 'heading': self.heading % 360.0, 'roll': self.roll, 'pitch': self.pitch,
            'agl': max(0.0, self.alt), 'altitude': self.elev0 + self.alt, 'position': self.position(),
        }


def _noisy(value, frac: float, lo: float, hi: float):
    """Perturb a scalar by +/- frac of its magnitude (uniform), clamped to [lo, hi]. frac 0 -> clean."""
    if frac:
        value = value + (random.random() * 2 - 1) * frac * (abs(value) + 1.0)
    return lo if value < lo else (hi if value > hi else value)


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
                    ('accel', 'attitude', 'agl', 'altitude', 'elevation', 'position')}
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

    async def run(self) -> None:
        dt = 1.0 / self._sim_hz
        t = 0.0
        period = max(1, 1000 // self._sim_hz)
        while True:
            # follow the REAL stage machine: SETTING/BOOSTING -> 1-DoF boost/coast (provides the launch
            # accel + altitude that drive the sequencer); GLIDING/LANDING -> fin-controlled 6-DoF glide.
            if self.controller.stage < _STAGE.GLIDING:
                self._body.boost_step(dt, self._thrust if t < self._burn_s else 0.0)
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
