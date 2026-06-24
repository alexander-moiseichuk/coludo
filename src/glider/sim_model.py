# sim_model.py — pure flight-dynamics model shared by the on-board HITL task (tasks/hitl.py) and the
# host-side virtual-flight tool (tools/virtual_flight.py). PURE: math + random only, no hardware, so it
# runs identically on the board (MicroPython) and on the host (CPython) -- the virtual flight and the
# HITL sim are then the SAME physics, only the harness around them differs. World frame is ENU metres
# from the launch pad; attitude is Euler degrees (roll, pitch, yaw=heading).

import math
import random

_G = 9.81
_RHO = 1.225            # sea-level air density (kg/m^3)
_M_PER_DEG = 111320.0   # metres per degree latitude (matches navigation.py)
_CDA = 0.6 * 0.0017     # Cd * frontal area (m^2) from the coludo.md envelope (~46 mm, ~17 cm^2)

# average thrust (N), burn (s) per motor -- from the coludo.md flight envelope
MOTORS: dict = {'E16': (16.1, 1.77), 'F15': (14.4, 3.45)}

# Default scenario (HPRC, Homestead Public Rocketry Club) -- overridable via the hitl config block.
HPRC: dict = {
    'launch': (25.514379, -80.391795),    # pad (lat, lon)
    'elevation_m': 2.0,                    # pad MSL
    'zone': [[25.514944, -80.392972], [25.514583, -80.391111]],  # TL, BR (~40 m N-S x ~187 m E-W strip)
    'heading_deg': 30.0,                   # initial glide heading (deg) at separation
}


class Body:
    """Flight-dynamics state + integrator (PURE -- host-testable). `boost_step()` climbs vertically; at
    apogee `begin_glide()` hands over to `glide_step()` (fin-controlled); `sensors()` returns what the
    on-board sensors would read."""

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
        self.wind_e = 0.0      # steady wind advecting the body (m/s, east +) -- a glide disturbance
        self.wind_n = 0.0      # steady wind advecting the body (m/s, north +)

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
        # ground track = airspeed along the heading + the wind (the glider is blown with the air mass)
        self.pe += (self.speed * math.sin(math.radians(self.heading)) + self.wind_e) * dt
        self.pn += (self.speed * math.cos(math.radians(self.heading)) + self.wind_n) * dt
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


def noisy(value, frac: float, lo: float, hi: float):
    """Perturb a scalar by +/- frac of its magnitude (uniform), clamped to [lo, hi]. frac 0 -> clean."""
    if frac:
        value = value + (random.random() * 2 - 1) * frac * (abs(value) + 1.0)
    return lo if value < lo else (hi if value > hi else value)
