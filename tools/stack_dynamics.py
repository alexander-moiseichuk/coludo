"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The stack's boost in six degrees of freedom (doc/specs/boost-control.md, BC1): the booster and the folded glider
as one rigid body, from the rod to the separation. sim_model's glide takes over at the separation
(`Stack.hand_over`); its 1-DoF vertical boost stays the default, so every published study reproduces.

Why: sim_model's boost cannot fail the way 10-03's stacks did (launches/20261003). Its lean axes integrate the wind
and the fins only, so the body has no static stability, no margin and no spin about its own axis. Here:
* the rod guides the stack for its length, then the stack flies free;
* the measured F15 pushes along the body axis, the propellant burns off along its impulse and moves the CG forward
  (the margin grows: 7D's turned restoring at 1.4-2.2 s);
* each lateral plane has its own normal force: a linear part at the centre of pressure (the margin, per plane: the
  folded wings make the planes differ) and a crossflow part, the side force at large angles of attack, where the
  stack's planform pushes (ahead of a motor-heavy CG it destabilises: 7C's 9.3 g side load);
* aerodynamic damping in pitch, yaw and roll, a built-in roll asymmetry proportional to dynamic pressure (7C's
  -11.7 deg/s^2 per Pa), the control fins, and the gyroscopic coupling of a spinning body.

Frames: the world is ENU metres from the pad (x east, y north, z up), as sim_model's; the body is the board's marks
frame (x along the nose, y to the left wing, z dorsal), the frame the HITL host publishes accel and rate in. A
positive fin command raises the nose (pitch), rolls the right wing down (roll), turns the nose right (yaw): the
host's (roll, pitch, yaw) from the board's fins, as sim_model reads them.

Host-only (CPython): sim_model is imported on the board by plan B's HITL task, and this model is not.
"""

import math

G: float = 9.80665
RHO: float = 1.225  # sea-level air density, sim_model's

# The F15 TMS-7 measured on 2026-10-03 (launches/20261003/TMS-7: the axial force x the falling mass, plus the drag
# fitted from the coast): (seconds from ignition, newtons), linear between points; 41.6 N s, burnout 2.68 s.
F15: tuple = ((0.00, 0.0), (0.05, 6.0), (0.10, 12.4), (0.20, 22.3), (0.30, 31.5), (0.40, 22.4), (0.50, 19.2),
              (0.60, 17.5), (0.70, 16.9), (0.80, 16.6), (1.00, 15.5), (1.20, 14.9), (1.40, 14.5), (1.60, 14.5),
              (1.80, 13.9), (2.00, 13.3), (2.20, 13.3), (2.40, 12.7), (2.60, 13.0), (2.68, 0.0))
PROPELLANT_KG: float = 0.060  # the F15's propellant (the TMS-7 estimate assumed 60 g; 45-70 g moves it +-2 N s)

SUBSTEP_S: float = 0.0005  # the integration step inside a host step: a 1600 deg/s spin turns 0.8 deg per step

"""
STACKS: the stacks the boost flies, by name (`Stack(profile)`; the HITL host's --stack). Masses are 10-03's weighings;
the aerodynamics are one set shared by 7C and 7D, fitted to both flights' events (tools/stack_replay.py: the 4.5 m/s
crosswind of 10-03 against the folded wings' plane), not measured. With it:
 * 7C at its nominal margin (+0.24 cal) departs off the rod and passes horizontal under thrust at ~1.2 s with a ~10 g
   side load, as it did (1.06 s, 9.3 g);
 * 7D holds 14-16 deg of angle of attack through the early burn (17-19 measured), recovers as its CG moves forward,
   burns out at 55 m/s (53.5) and peaks at ~108 m, 150 m out (91-98 m, ~145 m). Its margin is the fitted 0.84 cal: at
   its nominal 0.71 this model loops it. The edge lies between, which is the launch analysis's "near neutral early".
TMS-7, a stable rocket, keeps its own: margin 2 cal, its coast's CdA.
Each entry:
 * mass_kg: at liftoff, the motor loaded;
 * diameter_m: the reference diameter (the booster tube) -- calibers and the reference area are its;
 * inertia_roll, inertia_pitch: kg m^2 at liftoff (pitch and yaw alike);
 * cda_m2: axial Cd x frontal area (7D's coast: ~25 cm^2; TMS-7's: 9.4);
 * cn_alpha: the normal-force slope per radian, (pitch plane: along z, yaw plane: along y);
 * crossflow: the crossflow coefficient, (z, y): the side force grows with sin^2 of the angle of attack, strongest
   against the folded wings (z);
 * margin_cal: the centre of pressure behind the liftoff CG, calibers, (z, y);
 * crossflow_arm_cal: where the crossflow pushes, calibers ahead (+) of the liftoff CG;
 * motor_arm_m: the propellant's centroid behind the liftoff CG: the CG moves forward as it burns;
 * damping_m4: the aerodynamic damping, (z, y): a moment of 0.5 rho V K omega against the rotation;
 * roll_forcing: the built-in roll asymmetry, rad/s^2 per pascal of dynamic pressure (7C's spin-up: -0.34);
 * roll_damping: per metre: a roll acceleration of roll_damping x V x the roll rate against it (with the forcing,
   ~24 deg/s of roll per m/s; 7D coasted at ~20);
 * fin_moment: the control fins, newton-metres per pascal per radian of command, (roll, pitch, yaw);
 * rod_m: the rod's length.
"""
_FOLDED: dict = {  # the folded-wing stacks' aerodynamics, fitted to 7C and 7D together
    'diameter_m': 0.042, 'cda_m2': 0.0025, 'cn_alpha': (12.0, 12.0), 'crossflow': (80.0, 40.0),
    'crossflow_arm_cal': 2.0, 'motor_arm_m': 0.25, 'damping_m4': (3.0e-3, 3.0e-3), 'roll_forcing': -0.34,
    'roll_damping': 0.5, 'fin_moment': (1.2e-4, 1.1e-3, 5.5e-4), 'rod_m': 1.0,
}
STACKS: dict = {
    'tms7c': dict(_FOLDED, mass_kg=0.4142, inertia_roll=2.0e-4, inertia_pitch=0.020, margin_cal=(0.24, 0.39)),
    'tms7d': dict(_FOLDED, mass_kg=0.4842, inertia_roll=2.4e-4, inertia_pitch=0.024, margin_cal=(0.84, 0.99)),
    'tms7': {
        'mass_kg': 0.2283, 'diameter_m': 0.042, 'inertia_roll': 1.0e-4, 'inertia_pitch': 0.010,
        'cda_m2': 0.00094, 'cn_alpha': (10.0, 10.0), 'crossflow': (15.0, 15.0), 'margin_cal': (2.0, 2.0),
        'crossflow_arm_cal': 1.0, 'motor_arm_m': 0.20, 'damping_m4': (9.0e-4, 9.0e-4),
        'roll_forcing': 0.0, 'roll_damping': 0.35, 'fin_moment': (0.0, 0.0, 0.0), 'rod_m': 1.0,
    },
}


def thrust_at(seconds: float, curve: tuple = F15) -> float:
    """The motor's thrust in newtons `seconds` after ignition: linear between the curve's points, 0 outside."""
    if seconds <= curve[0][0] or seconds >= curve[-1][0]:
        return 0.0
    for (start, low), (end, high) in zip(curve, curve[1:]):
        if seconds < end:
            return low + (high - low) * (seconds - start) / (end - start)
    return 0.0


def impulse_of(curve: tuple = F15) -> float:
    """The curve's total impulse, newton-seconds (trapezoids: exact for the linear curve)."""
    return sum((end - start) * (low + high) / 2.0 for (start, low), (end, high) in zip(curve, curve[1:]))


def _multiply(first: tuple, second: tuple) -> tuple:
    """The Hamilton product of two quaternions (w, x, y, z)."""
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _matrix(quaternion: tuple) -> tuple:
    """The rotation matrix (rows) of a unit quaternion: a body vector times it gives the world vector."""
    w, x, y, z = quaternion
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def _to_world(matrix: tuple, vector: tuple) -> tuple:
    """A body vector in the world."""
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))


def _to_body(matrix: tuple, vector: tuple) -> tuple:
    """A world vector in the body (the transpose)."""
    return tuple(sum(matrix[row][column] * vector[row] for row in range(3)) for column in range(3))


def _cross(first: tuple, second: tuple) -> tuple:
    """The cross product."""
    return (first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0])


def _unit(vector: tuple) -> tuple:
    """The vector scaled to length 1."""
    length = math.sqrt(sum(component * component for component in vector))
    return tuple(component / length for component in vector)


def _quaternion_of(matrix: tuple) -> tuple:
    """The unit quaternion of a rotation matrix (rows), Shepperd's branch for stability."""
    (m00, m01, m02), (m10, m11, m12), (m20, m21, m22) = matrix
    trace = m00 + m11 + m22
    if trace > 0:
        scale = 2.0 * math.sqrt(1.0 + trace)
        return (0.25 * scale, (m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale)
    if m00 > m11 and m00 > m22:
        scale = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        return ((m21 - m12) / scale, 0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale)
    if m11 > m22:
        scale = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        return ((m02 - m20) / scale, (m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale)
    scale = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
    return ((m10 - m01) / scale, (m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale)


class Stack:
    """
    The stack from ignition to separation: on the rod, then free, under thrust, then coasting.

    Args:
        profile - a STACKS entry, or a dict of the same keys (a sweep's variant).
        wind_e, wind_n - the steady wind, m/s (blowing toward east, north: sim_model's convention).
        azimuth_deg - the compass direction the stack's dorsal side (+z) faces on the rod: where the wind meets
            its folded wings.
        rod_tilt_deg, rod_azimuth_deg - the rod off vertical, and toward which compass direction it leans.
    """

    def __init__(self, profile: dict, wind_e: float = 0.0, wind_n: float = 0.0, azimuth_deg: float = 0.0,
                 rod_tilt_deg: float = 0.0, rod_azimuth_deg: float = 0.0):
        self.profile = profile
        self.wind = (wind_e, wind_n, 0.0)
        self.diameter = profile['diameter_m']
        self.area = math.pi * self.diameter * self.diameter / 4.0
        self.impulse = impulse_of()
        tilt, toward = math.radians(rod_tilt_deg), math.radians(rod_azimuth_deg)
        self.rod_axis = _unit((math.sin(tilt) * math.sin(toward), math.sin(tilt) * math.cos(toward), math.cos(tilt)))
        facing = math.radians(azimuth_deg)
        dorsal = (math.sin(facing), math.cos(facing), 0.0)
        # the dorsal side made square to the rod; the left wing completes the right-handed frame (y = z x x)
        along = sum(dorsal[index] * self.rod_axis[index] for index in range(3))
        dorsal = _unit(tuple(dorsal[index] - along * self.rod_axis[index] for index in range(3)))
        left = _cross(dorsal, self.rod_axis)
        columns = (self.rod_axis, left, dorsal)
        self.attitude = _quaternion_of(tuple(tuple(columns[column][row] for column in range(3)) for row in range(3)))
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.rates = (0.0, 0.0, 0.0)  # body p, q, r in rad/s
        self.seconds = 0.0  # since ignition
        self.burned = 0.0  # impulse delivered so far, N s
        self.on_rod = True
        self.specific = (G, 0.0, 0.0)  # the accelerometer's specific force, body, m/s^2 (on the rod: 1 g along x)
        self.angle_of_attack = 0.0  # degrees, total, the last step's
        self.dynamic_pressure = 0.0  # pascals, the last step's

    def mass(self) -> float:
        """The stack's mass now: the propellant burns off along the delivered impulse."""
        return self.profile['mass_kg'] - PROPELLANT_KG * min(1.0, self.burned / self.impulse)

    def _shift(self) -> float:
        """How far the CG has moved forward of its liftoff place, metres."""
        return PROPELLANT_KG * min(1.0, self.burned / self.impulse) * self.profile['motor_arm_m'] / self.mass()

    def step(self, seconds: float, fins: tuple = (0.0, 0.0, 0.0)) -> None:
        """
        Advance `seconds` of flight under the fin commands `fins` (roll, pitch, yaw, degrees from neutral), in
        sub-steps of SUBSTEP_S.
        """
        remaining = seconds
        while remaining > 1e-12:
            dt = min(SUBSTEP_S, remaining)
            self._substep(dt, fins)
            remaining -= dt

    def _substep(self, dt: float, fins: tuple) -> None:
        """One integration step: the forces and moments, then the state (semi-implicit Euler)."""
        profile = self.profile
        thrust = thrust_at(self.seconds)
        mass = self.mass()
        shift = self._shift()
        matrix = _matrix(self.attitude)
        air = tuple(self.velocity[index] - self.wind[index] for index in range(3))
        u, v, w = _to_body(matrix, air)
        speed = math.sqrt(u * u + v * v + w * w)
        pressure = 0.5 * RHO * speed * speed
        half_rho_area = 0.5 * RHO * self.area
        # axial drag, then each plane's normal force: linear at the centre of pressure, crossflow where the planform is
        drag = -0.5 * RHO * profile['cda_m2'] * u * abs(u)
        linear_z = -half_rho_area * profile['cn_alpha'][0] * u * w
        linear_y = -half_rho_area * profile['cn_alpha'][1] * u * v
        cross_z = -half_rho_area * profile['crossflow'][0] * w * abs(w)
        cross_y = -half_rho_area * profile['crossflow'][1] * v * abs(v)
        force = (thrust + drag, linear_y + cross_y, linear_z + cross_z)
        pressure_arm = (-(profile['margin_cal'][0] * self.diameter + shift),
                        -(profile['margin_cal'][1] * self.diameter + shift))
        cross_arm = profile['crossflow_arm_cal'] * self.diameter - shift
        # r x F for a force (0, F_y, F_z) at (x, 0, 0): (0, -x F_z, x F_y)
        moment = [0.0, -pressure_arm[0] * linear_z - cross_arm * cross_z,
                  pressure_arm[1] * linear_y + cross_arm * cross_y]
        roll_command, pitch_command, yaw_command = (math.radians(command) for command in fins)
        roll_fin, pitch_fin, yaw_fin = profile['fin_moment']
        inertia = (profile['inertia_roll'], profile['inertia_pitch'], profile['inertia_pitch'])
        moment[0] += inertia[0] * (profile['roll_forcing'] * pressure - profile['roll_damping'] * speed * self.rates[0])
        moment[0] += roll_fin * pressure * roll_command
        damping = 0.5 * RHO * speed
        moment[1] += -damping * profile['damping_m4'][0] * self.rates[1] - pitch_fin * pressure * pitch_command
        moment[2] += -damping * profile['damping_m4'][1] * self.rates[2] - yaw_fin * pressure * yaw_command
        self.dynamic_pressure = pressure
        self.angle_of_attack = math.degrees(math.atan2(math.sqrt(v * v + w * w), u)) if speed > 0.1 else 0.0
        self.specific = tuple(component / mass for component in force)
        acceleration = _to_world(matrix, self.specific)
        acceleration = (acceleration[0], acceleration[1], acceleration[2] - G)
        self.seconds += dt
        self.burned += thrust * dt
        if self.on_rod:
            along = max(0.0, sum(acceleration[index] * self.rod_axis[index] for index in range(3)))
            speed_along = sum(self.velocity[index] * self.rod_axis[index] for index in range(3)) + along * dt
            self.velocity = tuple(speed_along * axis for axis in self.rod_axis)
            self.position = tuple(self.position[index] + self.velocity[index] * dt for index in range(3))
            # the accelerometer on the rod: what moves it along the rod, plus what holds it up against gravity
            self.specific = _to_body(matrix, tuple(along * self.rod_axis[index] + (G if index == 2 else 0.0)
                                                   for index in range(3)))
            if sum(self.position[index] * self.rod_axis[index] for index in range(3)) >= profile['rod_m']:
                self.on_rod = False
            return
        self.velocity = tuple(self.velocity[index] + acceleration[index] * dt for index in range(3))
        self.position = tuple(self.position[index] + self.velocity[index] * dt for index in range(3))
        rates = self.rates
        spin = _cross(rates, tuple(inertia[index] * rates[index] for index in range(3)))
        self.rates = tuple(rates[index] + (moment[index] - spin[index]) / inertia[index] * dt for index in range(3))
        turn = _multiply(self.attitude, (0.0, self.rates[0] * dt / 2.0, self.rates[1] * dt / 2.0,
                                         self.rates[2] * dt / 2.0))
        self.attitude = _unit(tuple(self.attitude[index] + turn[index] for index in range(4)))

    def euler(self) -> tuple:
        """
        (heading, roll, pitch) in degrees, aerospace Z-Y-X from the body's attitude: what the sim publishes as
        `attitude`. Near a vertical nose heading and roll are both turns about the vertical, as on the board.
        """
        matrix = _matrix(self.attitude)
        # body forward/left/up to forward/right/down, world east/north/up to north/east/down
        nose = (matrix[1][0], matrix[0][0], -matrix[2][0])
        right = (-matrix[1][1], -matrix[0][1], matrix[2][1])
        down = (-matrix[1][2], -matrix[0][2], matrix[2][2])
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, -nose[2]))))
        roll = math.degrees(math.atan2(right[2], down[2]))
        heading = math.degrees(math.atan2(nose[1], nose[0])) % 360.0
        return heading, roll, pitch

    def path_angle(self) -> float:
        """The flight path's angle above the horizon, degrees (90 straight up)."""
        horizontal = math.hypot(self.velocity[0], self.velocity[1])
        return math.degrees(math.atan2(self.velocity[2], horizontal)) if self.velocity != (0.0, 0.0, 0.0) else 90.0

    def nose_elevation(self) -> float:
        """The nose's angle above the horizon, degrees."""
        return math.degrees(math.asin(max(-1.0, min(1.0, _matrix(self.attitude)[2][0]))))

    def hand_over(self, body) -> None:
        """
        The separation: sim_model's glide `body` takes over where the stack is (its position and climb); its own
        begin_glide() then sets the glide's attitude and speed as it always has.
        """
        body.pe, body.pn, body.alt = self.position
        body.vu = self.velocity[2]
