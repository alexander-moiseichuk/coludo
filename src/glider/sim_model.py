"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Pure flight-dynamics model shared by the on-board HITL task (tasks/hitl.py) and the host-side
virtual-flight tool (tools/virtual_flight.py). PURE: math + random only, no hardware, so it runs
identically on the board (MicroPython) and on the host (CPython) -- the virtual flight and the HITL
sim are then the SAME physics, only the harness around them differs. World frame is ENU metres from
the launch pad; attitude is Euler degrees (roll, pitch, yaw=heading).
"""

import math
import random

import commons
import navigation


def _unit_noise() -> float:
    """
    A ~standard-normal sample from uniforms (Irwin-Hall, 3 draws).

    `random.gauss` is absent on some MicroPython builds and sim_model must run identically on the board
    and the host, so the normal deviate is built from `random.random()`: three uniforms sum to variance
    1/4, hence the x2 to reach unit sigma.
    """
    return (random.random() + random.random() + random.random() - 1.5) * 2.0


def _polar(speed: float) -> float:
    """
    The sink multiplier at `speed`, relative to sink at trim (1.0 at _V_TRIM).

    Profile drag grows as v^3 and induced drag as 1/v; their half-sum is normalised so trim reads 1.0.

    Args:
        speed - the airspeed (m/s).

    Returns:
        The multiplier to apply to trim_sink.
    """
    v = speed if speed > _V_POLAR_FLOOR else _V_POLAR_FLOOR
    ratio = v / _V_TRIM
    return 0.5 * (ratio * ratio * ratio + 1.0 / ratio)


_G = 9.81
RHO = 1.225             # sea-level air density (kg/m^3) -- public: the HITL pitot model needs it too
_RHO = RHO              # legacy in-module alias (the physics below reads _RHO throughout)
_CDA = 0.6 * 0.0017     # Cd * frontal area (m^2) from the coludo.md envelope (~46 mm, ~17 cm^2)
"""
STALL floor: the 1-g stall speed. A coordinated turn pulls load n = 1/cos(bank), and the wing needs
airspeed >= _V_STALL_1G*sqrt(n) to hold it; below that it stalls (lift collapses -> a hard sink break,
controls go mushy). Trim is ~14 m/s so a straight glide sits ~1.5x above stall; it bites only at steep
bank (>~66 deg at trim) or when a degraded speed sags. This is the physical limit the airspeed-gated
endgame bank (in guidance) is gated to respect -- the sim PENALISES a gate that over-commands.
"""
_V_STALL_1G = 9.0       # m/s -- 1-g stall speed (below the 14 m/s trim)
_V_TRIM = 14.0          # m/s -- the speed the airframe trims to; the polar's minimum-sink point
"""
DRAG POLAR: sink varies with AIRSPEED, not just bank (findings §27.22). `trim_sink` is the sink AT trim;
away from it the classic decomposition applies -- profile/parasite drag grows as v^3, induced drag as
1/v -- so `_polar()` is normalised to exactly 1.0 at _V_TRIM. Note its MINIMUM sits at ~0.76*_V_TRIM
(~10.6 m/s), not at trim: minimum-sink speed is below best-glide speed on any real glider, so trimming
at 14 m/s buys range at a small cost in sink. Flying well off trim now costs energy either way, which
makes an airspeed-gated bank a real trade-off instead of a free one (it unblocks turn-radius #5.2).
Near the speeds the sim actually holds, the correction is small -- it is the SHAPE that was missing.
"""
_V_POLAR_FLOOR = 3.0    # m/s -- guard the 1/v term at a near-zero speed (post-touchdown, pre-launch)
_STALL_SINK = 1.0       # 1/s -- extra sink (m/s per m/s of speed deficit) once stalled, on top of drag
_ROLL_MAX = 70.0        # deg -- structural bank clamp (raised from 60 to give the gated bank room)
# boost-attitude model: crosswind WEATHERCOCK vs CONTROL-fin restore, both scaled by dynamic
# pressure (kPa), with aero damping -- so the guarded fins are seen fighting the wind during the climb.
_BOOST_COCK = 2.0       # deg/s^2 per (kPa * deg AoA) -- passive tilt of the nose toward the relative wind
_BOOST_FIN = 2.0        # deg/s^2 per (kPa * deg deflection) -- corrective fin authority
_BOOST_DAMP = 3.0       # 1/s -- aero angular-rate damping

# average thrust (N), burn (s) per motor -- from the coludo.md flight envelope
MOTORS: dict = {'E16': (16.1, 1.77), 'F15': (14.4, 3.45)}

# Default scenario (HPRC, Homestead Public Rocketry Club) -- overridable via the hitl config block.
HPRC: dict = {
    'launch': (25.514379, -80.391795),    # pad (lat, lon)
    'elevation_m': 2.0,                    # pad MSL
    'zone': [[25.514944, -80.392972], [25.514583, -80.391111]],  # TL, BR (~40 m N-S x ~187 m E-W strip)
    'heading_deg': 30.0,                   # initial glide heading (deg) at separation
}


class Faults:
    """
    Sensor-fault injection for robustness runs (findings §27.20).

    The firmware is full of degradation paths -- databoard priority fallback, the unconfident airspeed
    cap floor, the GNSS jump/steep gates, pitot saturation, warm start -- and those are exactly the paths
    that run when a flight is already going badly. Only a roll spike and a manual BNO055 drop were ever
    exercised, so the rest were untested. This injects the rest at the PUBLISH boundary (where a harness
    hands a reading to the control stack), which is where a real sensor failure appears.

    Spec is a comma-separated list of `channel` or `channel@seconds` (when it starts, default 0):

        'baro'              baro dead from t=0
        'gnss@30,pitot@45'  GNSS drops at 30 s, the pitot rails at 45 s

    Modes per channel: a DEAD channel returns None (the driver stopped / the databoard aged it out); the
    pitot instead RAILS to its full-scale reading, because a saturated differential-pressure sensor
    under-reads airspeed rather than going silent -- the more dangerous failure, since a low airspeed
    LOOSENS the fin cap.
    """

    RAIL_PITOT_MS: float = 29.85  # a +/-500 Pa cell pegs here; the governor must treat it as saturated

    def __init__(self, spec: str = ''):
        self.starts = {}  # channel -> the time (s) its fault begins
        for item in (spec or '').replace(' ', '').split(','):
            if not item:
                continue
            channel, _, when = item.partition('@')
            try:
                self.starts[channel] = float(when) if when else 0.0
            except ValueError:
                self.starts[channel] = 0.0

    def active(self, channel: str, t: float) -> bool:
        """Whether `channel` is faulted at time `t` (seconds since launch)."""
        start = self.starts.get(channel)
        return start is not None and t >= start

    def apply(self, channel: str, value, t: float):
        """
        The reading a faulted channel should publish; `value` unchanged when healthy.

        Args:
            channel - the quantity name ('baro', 'gnss', 'pitot', 'laser', 'attitude', 'accel', ...).
            value - the true reading the sim produced.
            t - seconds since launch.

        Returns:
            None for a dead channel, the rail value for a saturated pitot, else `value`.
        """
        if not self.active(channel, t):
            return value
        return self.RAIL_PITOT_MS if channel == 'pitot' else None

    def __bool__(self) -> bool:
        return bool(self.starts)

    def __repr__(self) -> str:
        return 'Faults(%s)' % ', '.join('%s@%gs' % item for item in sorted(self.starts.items()))


class Body:
    """
    Flight-dynamics state + integrator (PURE -- host-testable).

    `boost_step()` climbs vertically; at apogee `begin_glide()` hands over to `glide_step()`
    (fin-controlled); `sensors()` returns what the on-board sensors would read.
    """

    def __init__(self, mass: float, launch: tuple, elevation_m: float, glide_heading: float,
                 glide_mass: float = None):
        self.mass = mass  # boost mass (whole stack: booster + glider); drops to glide_mass at separation
        self.glide_mass = glide_mass if glide_mass else mass  # glider-only mass after the booster ejects
        self.lat0, self.lon0 = launch
        """
        metres per degree of LONGITUDE at the (fixed) pad latitude -- precomputed once, not per
        position() sample: cos(radians(lat0)) is constant for the whole flight (saves a per-call
        radians()+cos() box on the sim's publish-rate ground-track math).
        """
        self._m_per_deg_lon = commons.M_PER_DEG * math.cos(math.radians(self.lat0))
        self.elev0 = elevation_m
        self.glide_heading = glide_heading
        self.pe = 0.0          # position east (m from pad)
        self.pn = 0.0          # position north (m from pad)
        self.alt = 0.0         # altitude above the pad (m)
        self.vu = 0.0          # vertical speed (m/s)
        self.trim_sink = 7.0   # trim sink (m/s) at load 1 = 14/(L/D); 7.0 = "air quality 2" worst-case polar
        self.speed = 0.0       # horizontal airspeed (m/s)
        self.heading = glide_heading  # deg (0 = north)
        self.roll = 0.0        # deg
        self.pitch = 90.0      # deg; start nose-up (on the rod, vertical)
        self.pitch_rate = 0.0  # deg/s -- body angular rate (boost pitch lean + glide response) -> gyro
        self.roll_rate = 0.0   # deg/s -- body angular rate (boost roll lean + glide response) -> gyro
        self.yaw_rate = 0.0    # deg/s -- heading rate in the glide (coordinated turn) -> gyro
        self.accel_g = 1.0     # |specific force| the accelerometer reads (g)
        self.gliding = False
        self.wind_e = 0.0      # steady wind advecting the body (m/s, east +) -- a glide disturbance
        self.wind_n = 0.0      # steady wind advecting the body (m/s, north +)
        """
        GUSTS / turbulence (findings §27.21): real air is not the steady block `wind_e/wind_n` models, and
        an endgame proven only in steady wind has an unknown robustness margin. `gust` is the 1-sigma gust
        amplitude (m/s) added to the steady wind as an Ornstein-Uhlenbeck process -- correlated over
        `gust_tau` seconds rather than white noise, because a glider integrates gusts and what matters is
        the slow wander, not per-step jitter.

        DEFAULT 0 = CALM, so every existing study reproduces exactly; a robustness run turns it on.
        """
        self.gust = 0.0        # 1-sigma gust amplitude (m/s); 0 = calm
        self.gust_tau = 3.0    # s -- gust correlation time (how long a gust persists)
        self._gust_e = 0.0     # current gust component (m/s, east) -- state of the OU process
        self._gust_n = 0.0
        """
        GNSS consistent-drift velocity (m/s ENU): a stationary receiver's slow apparent motion (geometry
        / ionosphere / multipath). Present in the REPORTED ground velocity even on the pad (a fixed
        antenna, no wind) -> the calibration reads it. MEASURED (Adafruit DGPS, 8 sats, HDOP 1.01, 60 s
        stationary, 2026-07-09): ~0.25 m/s apparent, of which ~0.13 m/s is a CONSISTENT bias (the part
        the calib removes) and ~half is random walk (residual). So a realistic scenario value is ~0.15.
        """
        self.gnss_drift_e = 0.0
        self.gnss_drift_n = 0.0
        """
        weight-imbalance / thrust-misalignment disturbance (deg/s^2), applied while the motor
        burns: a CG offset or a canted nozzle torques the stack CONSTANTLY, unlike the
        weathercock which needs wind. Pitch = the top-to-bottom lean axis, roll = side-to-side.
        """
        self.imbalance_pitch = 0.0
        self.imbalance_roll = 0.0

    def boost_step(self, dt: float, thrust: float, pitch_cmd: float = 0.0, roll_cmd: float = 0.0) -> None:
        """
        Vertical climb (1-DoF: thrust + gravity + drag) PLUS attitude under thrust.

        A crosswind WEATHERCOCKS the stack off vertical (passive fin stability cocks the nose toward the
        relative wind, ∝ q·AoA), the CONTROL fins push it back toward vertical (∝ q·deflection), aero
        damps the rate. Two lean axes -- pitch off the east-wind component, roll off the north -- so the
        guarded fins are seen fighting the wind during the climb. The accelerometer reads specific force
        = (thrust - drag)/mass (= kinematic a + g); ~0 g in ballistic coast (free fall).

        Args:
            dt - the integration interval (seconds).
            thrust - the motor thrust this step (N); 0 in coast.
            pitch_cmd - the pitch fin deflection command (deg); 0 = neutral.
            roll_cmd - the roll fin deflection command (deg); 0 = neutral.

        Returns:
            None. Advances the vertical + attitude state in place.
        """
        drag = 0.5 * _RHO * self.vu * abs(self.vu) * _CDA
        specific = (thrust - drag) / self.mass         # what the accelerometer measures (up +)
        self.accel_g = specific / _G if thrust else max(0.0, -drag / self.mass / _G + 0.0)
        self.vu += (specific - _G) * dt                # kinematic accel = specific - g
        self.alt += self.vu * dt
        # attitude: dynamic pressure (kPa) scales BOTH the wind disturbance and the fin authority, so near
        # lift-off (q ~ 0) nothing moves (the rod holds it vertical) and both grow as it accelerates.
        q = 0.5 * _RHO * self.vu * self.vu / 1000.0
        climb = max(self.vu, 1.0)
        aoa_pitch = math.degrees(math.atan2(self.wind_e, climb))  # crosswind angle of attack (deg)
        aoa_roll = math.degrees(math.atan2(self.wind_n, climb))
        burn = 1.0 if thrust else 0.0  # imbalance torque acts only while the motor pushes
        self.pitch_rate += (-_BOOST_COCK * q * aoa_pitch + _BOOST_FIN * q * pitch_cmd
                            - _BOOST_DAMP * self.pitch_rate
                            + burn * self.imbalance_pitch) * dt   # cock leans off 90, fins restore
        self.roll_rate += (-_BOOST_COCK * q * aoa_roll + _BOOST_FIN * q * roll_cmd
                           - _BOOST_DAMP * self.roll_rate
                           + burn * self.imbalance_roll) * dt
        self.pitch += self.pitch_rate * dt             # 90 = vertical
        self.roll += self.roll_rate * dt

    def begin_glide(self) -> None:
        """
        Apogee hand-over: the booster ejects and the glider noses down into the trim glide.

        Mass drops to the glider-only glide_mass, then nose down to a shallow glide on the configured
        heading at ~trim speed.

        Returns:
            None. Switches the body into the gliding state.
        """
        self.mass = self.glide_mass  # separation: shed the booster -> the glider glides on its own mass
        self.gliding = True
        self.pitch = -6.0
        self.roll = 0.0
        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.yaw_rate = 0.0
        self.heading = self.glide_heading
        self.speed = 14.0                              # trim airspeed (m/s)
        self.vu = self.speed * math.sin(math.radians(self.pitch))

    def _advance_gust(self, dt: float) -> tuple:
        """
        Advance the gust process one step and return its current (east, north) components.

        Ornstein-Uhlenbeck: each component decays toward zero over `gust_tau` while being kicked by
        noise scaled so the stationary spread is `gust`. Correlated rather than white, because a glider
        responds to the gust it is IN, not to per-step jitter that averages out within one control tick.

        Args:
            dt - the integration interval (seconds).

        Returns:
            (gust_e, gust_n) in m/s; (0.0, 0.0) when gusts are off (the default, calm).
        """
        if not self.gust:
            return (0.0, 0.0)
        decay = dt / self.gust_tau if dt < self.gust_tau else 1.0
        kick = self.gust * (2.0 * decay) ** 0.5
        self._gust_e += -decay * self._gust_e + kick * _unit_noise()
        self._gust_n += -decay * self._gust_n + kick * _unit_noise()
        return (self._gust_e, self._gust_n)

    def glide_step(self, dt: float, roll_cmd: float, pitch_cmd: float, yaw_cmd: float) -> None:
        """
        Rigid-body glide step under fin control.

        Fin deflections (deg from neutral) command roll/pitch; bank turns the heading (coordinated
        turn); a shallow nose-down trim holds the descent. First-order responses keep it stable. Eases
        the airspeed back toward trim.

        Args:
            dt - the integration interval (seconds).
            roll_cmd - the roll (aileron) fin deflection command (deg from neutral).
            pitch_cmd - the pitch (elevator) fin deflection command (deg from neutral).
            yaw_cmd - the yaw fin deflection command (deg from neutral).

        Returns:
            None. Advances the glide state (attitude, heading, speed, position) in place.
        """
        roll0, pitch0 = self.roll, self.pitch  # for the gyro angular rates (finite difference, honours clamp)
        # STALL check for THIS step, from the bank we are flying: load n = 1/cos(roll), stall speed rises
        # as sqrt(n). A stalled wing loses lift (a sink break, below) and bites at half control authority.
        load0 = 1.0 / max(0.3, math.cos(math.radians(roll0)))
        stall_speed = _V_STALL_1G * load0 ** 0.5
        stalled = self.speed < stall_speed
        authority = 0.5 if stalled else 1.0                        # mushy controls in the stall
        self.roll += (1.2 * authority * roll_cmd - 2.0 * self.roll) * dt   # ailerons -> bank, leveling
        self.pitch += (0.8 * authority * pitch_cmd - 1.5 * (self.pitch + 6.0)) * dt  # elevator -> pitch -6 trim
        self.roll = max(-_ROLL_MAX, min(_ROLL_MAX, self.roll))
        roll_rad = math.radians(self.roll)  # cached: turn + the load factor reuse it (was 3 radians() calls)
        turn = _G * math.tan(roll_rad) / max(self.speed, 5.0)  # rad/s heading rate from bank
        self.yaw_rate = math.degrees(turn) + 0.05 * yaw_cmd  # deg/s (pre-wrap, so no 360->0 discontinuity)
        self.roll_rate = (self.roll - roll0) / dt   # deg/s the gyro would read
        self.pitch_rate = (self.pitch - pitch0) / dt
        self.heading = (self.heading + self.yaw_rate * dt) % 360.0
        heading_rad = math.radians(self.heading)  # cached: pe + pn reuse it (was 2 radians() calls)
        self.speed += (14.0 - self.speed) * 0.5 * dt
        """
        sink: the straight-TRIM glide settles at ~-7 m/s = "air quality 2", the WORST-CASE polar
        (14 m/s / L/D 2: 200 m of altitude buys ~400 m of air path). Deliberately pessimistic:
        the real airframe is expected at quality 4-6 (capacity ~10 min aloft), but simulating
        that makes every HITL campaign crazy long -- the sim stays conservative and the polar
        RE-CALIBRATES from the first real glide telemetry. A bank raises sink by the induced-drag
        law (load factor n^1.5: x1.24 at 30 deg, x1.68 at 45) -- the physical cost, replacing the
        old raw G*(1-cos) term (~10x too harsh, every turn hemorrhaged what trim saved). An
        off-trim pitch still adds sink, so holding trim flies longest and altitude bleeds through
        the TURNS -- the designed energy management (fly-long is the primary objective; see coludo.md).
        """
        load = 1.0 / max(0.3, math.cos(roll_rad))  # load factor n (clamped); the accel_g below reuses it
        # sink = trim sink x the POLAR at this airspeed x the bank's induced-drag cost (n^1.5)
        self.vu += -0.1 * (self.vu + self.trim_sink * _polar(self.speed) * load ** 1.5) * dt
        if stalled:  # lift lost -> a hard sink break on TOP of induced drag; deeper deficit, harder drop
            self.vu -= _STALL_SINK * (stall_speed - self.speed) * dt
        """
        ANY off-trim pitch adds sink (ABS -- both a nose-down dive and a nose-up mush cost energy), so
        holding trim flies longest. The old signed term let a sustained nose-DOWN pitch (< -6) REDUCE
        sink and even CLIMB -- unphysical for an unpowered glider (found by the combined-degradation
        HITL, where degraded control drove the pitch off-trim and the glider climbed instead of landing).
        """
        self.vu = self.vu - 0.4 * abs(self.pitch + 6.0) * dt
        self.alt += self.vu * dt
        # ground track = airspeed along the heading + the wind (the glider is blown with the air mass)
        gust_e, gust_n = self._advance_gust(dt)
        self.pe += (self.speed * math.sin(heading_rad) + self.wind_e + gust_e) * dt
        self.pn += (self.speed * math.cos(heading_rad) + self.wind_n + gust_n) * dt
        self.accel_g = load  # the load factor rises in a bank -- identical to `load` above (n = 1/cos roll)
        if self.alt <= 0.0:  # GROUND CONTACT: the glider is down -> stops (was gliding underground forever,
            self.alt = 0.0   # so a degraded flight that never levelled never went 'stationary' -> no DONE)
            self.vu = 0.0
            self.speed *= 0.5          # bleed the ground roll-out toward rest
            self.accel_g = 1.0         # stationary -> the sequencer's still-detect fires -> LANDING -> DONE

    def position(self) -> tuple:
        lat = self.lat0 + self.pn / commons.M_PER_DEG
        lon = self.lon0 + self.pe / self._m_per_deg_lon  # precomputed constant divisor (see __init__)
        return (lat, lon)

    def _ground_velocity(self) -> tuple:
        """
        The GNSS-REPORTED horizontal ground velocity (east, north m/s).

        AIRBORNE it is the air velocity along the heading + the wind (the glider drifts with the air
        mass) plus the receiver's consistent DRIFT; on the pad / boost climb the receiver is stationary
        (a fixed antenna is NOT carried by the wind), so it reports ONLY its drift -- exactly what the
        pad calibration measures over SETTING.

        Returns:
            (east, north) ground velocity in m/s.
        """
        if not self.gliding:
            return (self.gnss_drift_e, self.gnss_drift_n)
        return (self.speed * math.sin(math.radians(self.heading)) + self.wind_e + self._gust_e
                + self.gnss_drift_e,
                self.speed * math.cos(math.radians(self.heading)) + self.wind_n + self._gust_n
                + self.gnss_drift_n)

    def track(self) -> float:
        """
        Ground-track bearing (deg) -- the direction the glider MOVES over the ground.

        Air velocity along the heading PLUS the wind. In no wind it equals the heading; a crosswind adds
        a crab angle. This is what a GNSS receiver reports as course (the attitude backup's absolute yaw
        ref).

        Returns:
            The ground-track bearing in degrees [0, 360); the heading when the glider is not moving.
        """
        east, north = self._ground_velocity()
        return math.degrees(math.atan2(east, north)) % 360.0 if (east or north) else self.heading % 360.0

    def ground_speed(self) -> float:
        """
        Horizontal GNSS GROUND speed (m/s) -- the magnitude of the ground velocity, WITH the wind.

        A real GNSS reports ground speed, not airspeed. In a turn it swings +/- the wind around the
        airspeed, which is what the wind estimator's min/max reads.

        Returns:
            The ground speed in m/s.
        """
        east, north = self._ground_velocity()
        return math.sqrt(east * east + north * north)

    def sensors(self) -> dict:
        """Clean (pre-noise) sensor readings from the current state."""
        return {
            'accel': self.accel_g, 'heading': self.heading % 360.0, 'roll': self.roll, 'pitch': self.pitch,
            'agl': max(0.0, self.alt), 'altitude': self.elev0 + self.alt, 'position': self.position(),
            'speed': self.ground_speed(),  # GNSS ground speed (2D horizontal, WITH wind) -> governor + wind
            'roll_rate': self.roll_rate, 'pitch_rate': self.pitch_rate, 'yaw_rate': self.yaw_rate,  # gyro (deg/s)
        }


HEADING_NOISE_REF: float = 20.0
"""
Noise reference for CIRCULAR channels (heading, GNSS course): frac 0.05 -> +-1 deg, a realistic BNO055
fused-heading band, instead of the +-18 deg that scaling by a 0..360 magnitude produced. See noisy().
"""


POSITION_NOISE_REF: float = 10.0
"""
Noise reference for GNSS POSITION, in METRES: frac 0.05 -> +-0.5 m, frac 1.0 -> +-10 m (a bad urban
multipath fix, ~3x a typical consumer CEP). See noisy_position() for why this is not a fraction.
"""


def noisy_position(position, frac: float):
    """
    Perturb a (latitude, longitude) by a uniform +/- frac * POSITION_NOISE_REF METRES, per axis.

    Position cannot go through noisy(): that scales by `abs(value) + 1`, so on a latitude of 48 a
    "100 % noise" would mean +-49 DEGREES -- thousands of kilometres. It is the same mistake the
    heading channel already made and HEADING_NOISE_REF already fixed, one step worse. A GNSS error is
    an absolute distance on the ground and has nothing to do with where on the globe it is measured, so
    the perturbation is applied in metres and converted by the one geographic primitive.

    The conversion quantizes to the board's float32 grid (~0.42 m at latitude 48, see
    guidance._reckon), which at a +-10 m band is ~4 % granularity -- and is exactly the resolution the
    board would have for a real fix, so the sim inherits the hardware's own limit rather than
    pretending to a precision the flight code does not have.

    Args:
        position - the clean (latitude, longitude), or None when there is no fix.
        frac - the noise fraction; 0 (or no position) returns the input untouched.

    Returns:
        The perturbed (latitude, longitude).
    """
    if position is None:
        return None
    """
    Draw UNCONDITIONALLY, even at frac 0, and scale afterwards. An early return at zero would consume
    no randomness, so the clean run would sit on a different RNG stream from every noised one and the
    control in a noise sweep would not be comparable to the cases it controls -- measured: the first
    sweep put the frac-0 run 20 m apart in apogee purely from stream drift, which reads as a noise
    effect and is not one. Costs two uniforms per sample on a sim-only path.
    """
    east = (random.random() * 2 - 1) * frac * POSITION_NOISE_REF
    north = (random.random() * 2 - 1) * frac * POSITION_NOISE_REF
    return navigation.advance(position, east, north)


def noisy(value, frac: float, lo: float, hi: float, reference: float = None):
    """
    Perturb a scalar by +/- frac of a REFERENCE magnitude (uniform), clamped to [lo, hi].

    The reference defaults to `abs(value) + 1` -- noise proportional to the reading, which is right for
    a quantity whose error genuinely scales with it (speed, dynamic pressure, acceleration).

    It is WRONG for a CIRCULAR quantity, and that mattered. A compass heading lives on 0..360, so
    magnitude-proportional noise made "5 %" mean +-18 deg near 350 and +-0.5 deg near 10 -- as if north
    were more certain than south. A real BNO055 fused heading is ~+-1-2 deg wherever it points. Every
    noise>0 study was therefore exercising an absurdly hostile heading signal: measured on the host,
    switching heading to an absolute +-1 deg cut fin travel 22791 -> 16687 deg (-27 %) while touchdown
    moved 118.3 -> 118.4 m. So it never distorted the ACCURACY results, only the fin-activity and
    servo-power ones -- which is exactly what the numbers were being used for.

    Pass `reference` for such channels (see HEADING_NOISE_REF) to get an absolute error band instead.

    Args:
        value - the clean scalar to perturb.
        frac - the noise fraction (0 -> returned clean).
        lo - the lower clamp bound.
        hi - the upper clamp bound.
        reference - noise scale to use instead of abs(value) + 1 (for circular / absolute-error channels).

    Returns:
        The perturbed, clamped value; the clean value clamped when frac is 0.
    """
    if frac:
        scale = reference if reference is not None else abs(value) + 1.0
        value = value + (random.random() * 2 - 1) * frac * scale
    return lo if value < lo else (hi if value > hi else value)
