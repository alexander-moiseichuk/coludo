"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture attitude.json: plan B's attitude filter (src/glider/tasks/attitude.py, Attitude._tick) as the Rust
port (crates/control, module attitude, Attitude::tick) must answer it, tick by tick: the complementary filter, its
blind window, the mirror of a source that outranks it, the withhold that hands over to a backup, the
coordinated-turn accel gate, the GNSS track pull and the magnetic yaw.

Plan B runs the filter in FIXED POINT: centidegree states, an integer CORDIC atan2, an integer square root, gyro
deltas floored to the centidegree, pulls that are right shifts. The Rust port computes the real-number filter those
integers stand for, in f32 degrees and seconds. So every input here is written in plan B's own integer units,
which both sides read exactly:
  * `rate_cd_s` -- the gyro (roll, pitch, yaw) rate in centidegrees per second (the Rust rate is rate / 100
    degrees per second);
  * `accel_cg` -- the accelerometer in centi-g: plan B is handed accel_cg / 100 g, a float its from_float()
    truncates back to exactly accel_cg (only values int(x * 100) keeps are drawn), and the Rust reads the same
    accel_cg / 100 g;
  * `course_cd` -- the GNSS ground track in centidegrees, handed to plan B the same way; `speed` -- the ground
    speed in m/s, a plain float on both sides;
  * `mag` -- the magnetometer counts (integers on both sides: only their ratio sets the heading);
  * `fused` -- the fused attitude parameter's FRESH winner: its source, the rank that source registered at and
    its value [heading_cd, roll_cd, pitch_cd] (plan B's channel carries the heading as a float in degrees and
    roll/pitch as centidegrees), or null when nothing is fresh;
  * `backup` -- whether a source ranked BELOW the filter is fresh (plan B's Parameter.fresh_below);
  * `now_ms` -- the time of the tick, milliseconds after the filter's setup.
A null reading is a channel with no fresh source. Where a seeded run drops a channel, plan B is handed the
channel's last value with no source -- what its read() answers for a stale channel, an extrapolation -- and must
ignore it, as the Rust databoard (which has no extrapolation) hands the port nothing at all.

Each step's `out` is grouped by how it compares (`compare`):
  * exact -- the flags, EXACTLY: `free` (free-running, not mirroring), `blind` (no fresh gyro for five cycles),
    `seeded` (a source that outranks the filter has been mirrored), `mag_known` (the magnetic offset has been
    learned), and `log`, the line the tick logged (the blind transitions) or null;
  * degrees -- angle_deg 0.1, the plan's trajectory tolerance, through every step of every scenario: `state`, the
    filter's [heading, roll, pitch] after the tick, `mag_offset`, and `published`, the [heading, roll, pitch]
    pushed on the attitude channel, or null when the tick withheld it.
Each scenario closes with plan B's inspect() (its attitude keys), whose angles are text to the hundredth.

The 0.1 degree tolerance is the port's, so the fixture keeps plan B's own integer error inside it. The builder runs a
real-number filter beside plan B (_Law, written out below the way the Rust port computes it, in f64) and measures
plan B against it at every step. Plan B's integer error comes from four places, and each is held down here:
  * the gyro delta, rate x dt_ms // 1000, floors up to a centidegree per tick and never recovers it. Rates are
    whole degrees per second and every integrated step a multiple of 10 ms (a longer gap integrates the 20 ms
    period), so it is exact;
  * a pull is `error >> shift`, which floors, so a pull approaching its target from BELOW stalls up to 2^shift - 1
    centidegrees short of it for good (from above it converges exactly): at the defaults that is 0.15 degree
    (corr_shift 4), 0.31 (course_shift 5, mag_offset_shift 5) and 0.63 (mag_shift 6). Most scenarios here use
    shifts of 1 and 2, as test_attitude.py and test_magnetic_yaw.py do; the config_default scenarios leave the
    track and the magnetometer out and approach their gravity targets from above;
  * the integer CORDIC is good to ~0.05 degree, and the pitch's integer square root floors the denominator: up to
    ~0.3 degree of pitch at centi-g scale. Every accelerometer and magnetometer vector used here is one whose
    plan B angle is within angle_vetting.TARGET_ERROR of the true angle of the same integers (seeded runs draw again);
  * from_float() truncates the accelerometer to the centi-g, up to ~0.6 degree: the inputs are centi-g already.
Then the builder refuses a fixture in which plan B strays past BUDGET (angle_vetting.py) from the real-number filter
anywhere, or answers a flag differently, which leaves the port's f32 (~1e-4 degree over a run) its margin.

Decisions stay off their thresholds, where plan B's integers and the port's f32 could part: no fresh speed within
_SPEED_MARGIN of the course gate, no accelerometer magnitude within _BAND_MARGIN centi-g squared of the gravity band,
no roll or pitch within GATE_MARGIN of the magnetometer's level gate when a magnetometer reading is fresh, no pull
error within WRAP_MARGIN of the +/-180 wrap, and no published roll or pitch within GATE_MARGIN of +/-180. The turn
gate and the timers compare integers (whole-degree gates, whole-millisecond clocks) and are exact on both sides. A
scripted scenario that breaks the rule fails the build; a seeded run drops the offending reading for that tick.

Scenarios: test_attitude.py's blocks as tick sequences (its direct pokes at the state become a mirror of a
priority-0 source, the way a state is set in flight), the magnetic yaw through the tick, the corners found while
porting (the gap clamp, the accel throttle, the band edges, the flag that is never cleared, the gyro dropped on an
unseeded board), the orders a review found unpinned (the level gate reads the roll before the gyro step, the track
needs the course AND the speed, the accelerometer pull goes the long way round across +/-180), a 20 s free run long
enough for the tolerance to see a gyro step wrong by 0.02 %, config_default's own settings, then seeded flights.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import copy
import math
import random

import angle_vetting
import config_default
import recorder
import shims
from tasks import attitude

_SEED: int = 20261006  # the seeded flights' random.Random seed: the fixture is a function of the code alone
_SWEEPS: int = 6  # seeded flights
_SWEEP_STEPS: int = 160  # ticks per seeded flight
_SPEED_MARGIN: float = 1e-03  # m/s from the course gate
_BAND_MARGIN: int = 2  # centi-g squared from the gravity band's edges (plan B compares integers)
_MAXIMUM_GAP_MS: int = 500  # plan B's _MAX_DT_MS
_BLIND_CYCLES: int = 5  # plan B's _BLIND_CYCLES (a const: not readable from the module)
_SCALE: int = angle_vetting.SCALE  # centi-units per unit (fixed.SCALE)
_OWN_SOURCE: str = 'attitude'  # the filter's own source name
_LOG: list = []  # the lines plan B logged (Recorder.log is captured: no Recorder runs here)

"""test_attitude.py's component: a fast blend for the test (the defaults are weaker)."""
_TEST_CONFIG: dict = {'name': 'attitude', 'activity': 'attitude', 'period_ms': 20, 'accel_period_ms': 0,
                      'corr_shift': 2, 'grav_low_g': 0.7, 'grav_high_g': 1.3, 'turn_gate_deg_s': 4,
                      'course_gate_mps': 5.0, 'course_shift': 2,
                      'provides': {'attitude': {'priority': 1, 'timeout_ms': 40}}}

"""The same with test_magnetic_yaw.py's fast magnetometer knobs, so a few ticks converge."""
_MAGNETIC_CONFIG: dict = dict(_TEST_CONFIG, mag_shift=2, mag_offset_shift=1, mag_level_deg=15)

"""The fused attitude's sources by rank: the HITL sim at p0, a peer at the filter's own p1, the BNO055 backup at p2."""
_RANKS: dict = {'primary': 0, _OWN_SOURCE: 1, 'peer': 1, 'backup': 2}

"""Shorthands: a level airframe's gravity, and no field at all (present but outside the band)."""
_LEVEL: tuple = (0, 0, 100)
_NO_GRAVITY: tuple = (0, 0, 0)


class _Parameter:
    """A databoard parameter stand-in: read() answers [value, source, age] as plan B's read() does."""

    def __init__(self) -> None:
        """Nothing written yet."""
        self.value: object = None
        self.fresh: bool = False

    def read(self) -> list:
        """The scripted reading; a stale one keeps its value (plan B's extrapolation) with no source."""
        return [self.value, 'stub' if self.fresh else None, 0 if self.fresh else None]


class _Fused:
    """The fused `attitude` parameter: read() (the winner), rank(source) and fresh_below(rank)."""

    def __init__(self) -> None:
        """Nothing fresh yet."""
        self.value: object = None
        self.source: str | None = None
        self.backup: bool = False

    def read(self) -> list:
        """[value, source, age]: the winner, or (an extrapolation, None, None) when nothing is fresh."""
        return [self.value, self.source, 0 if self.source is not None else None]

    def rank(self, source: str) -> int | None:
        """The rank `source` registered at (Parameter.rank)."""
        return _RANKS.get(source)

    def fresh_below(self, rank: int) -> bool:
        """Whether a source ranked below the filter is fresh (Parameter.fresh_below): scripted."""
        return self.backup


class _Own:
    """The filter's own write channel: its rank, and the pushes it made."""

    def __init__(self, rank: int) -> None:
        """A channel at `rank`, nothing pushed."""
        self.rank: int = rank
        self.pushed: list = []

    def push(self, value: tuple) -> None:
        """Keep one push."""
        self.pushed.append(value)


class _Rows:
    """The attitude.csv stream: always due, keeps nothing (the state is compared directly)."""

    def due(self, now: int) -> bool:
        """Every tick is due."""
        return True

    def push(self, row: tuple) -> None:
        """Drop the row."""


def _capture_log(descriptor: str, message: str) -> bool:
    """Recorder.log, captured: plan B logs the blind transitions through it."""
    _LOG.append(message)
    return True


class _Law:
    """
    The real-number filter plan B's integers stand for: what the Rust port computes, here in f64, used only to vet.

    It also checks every decision it takes against its threshold (see the module notes) and every integer angle
    plan B takes from a vector, raising angle_vetting.NearThreshold for one that is too close to call.
    """

    def __init__(self, unit: attitude.Attitude, rank: int) -> None:
        """
        The law with plan B's resolved settings (so a config key both read is read the same).

        Args:
            unit - plan B's filter, after setup().
            rank - the filter's own rank.
        """
        self.period_ms: int = unit._period_ms
        self.accel_period_ms: int = unit._accel_period_us // 1000
        self.correction: float = 2.0 ** -unit._corr_shift
        self.turn_gate_cd_s: int = unit._turn_gate
        self.course_gate: float = unit._course_gate
        self.course_pull: float = 2.0 ** -unit._course_shift
        self.mag_pull: float = 2.0 ** -unit._mag_shift
        self.offset_pull: float = 2.0 ** -unit._mag_offset_shift
        self.level: float = unit._mag_level_cd / _SCALE
        self.low_sq: int = unit._grav_lo_sq
        self.high_sq: int = unit._grav_hi_sq
        self.rank: int = rank
        self.roll: float = 0.0
        self.pitch: float = 0.0
        self.yaw: float = 0.0
        self.offset: float = 0.0
        self.known: bool = False
        self.seeded: bool = False
        self.free: bool = False
        self.blind: bool = False
        self.strikes: int = 0
        self.last_ms: int = 0
        self.accel_ms: int = 0

    def tick(self, now_ms: int, reading: dict) -> tuple:
        """
        One tick (Attitude._tick).

        Args:
            now_ms - the tick's time, ms after setup.
            reading - the step's readings, in the fixture's units.

        Returns:
            (published [heading, roll, pitch] or None, the logged line or None).

        Raises:
            angle_vetting.NearThreshold - a decision or an integer angle is too close to call.
        """
        elapsed = now_ms - self.last_ms
        self.last_ms = now_ms
        dt_ms = self.period_ms if (elapsed > _MAXIMUM_GAP_MS or elapsed < 0) else elapsed
        log = None
        if reading['rate_cd_s'] is None:
            self.strikes += 1
            if self.strikes == _BLIND_CYCLES:
                self.blind = True
                log = 'gyro lost -- attitude filter blind'
        else:
            self.strikes = 0
            if self.blind:
                self.blind = False
                log = 'gyro back -- attitude filter sighted'
        fused = reading['fused']
        if fused is not None and fused['rank'] < self.rank:
            heading_cd, roll_cd, pitch_cd = fused['value']
            self.yaw, self.roll, self.pitch = heading_cd / _SCALE, roll_cd / _SCALE, pitch_cd / _SCALE
            self.seeded = True
            self.free = False
        elif self.seeded or reading['accel_cg'] is not None:
            self._integrate(dt_ms / 1000.0, now_ms, reading)
            self.free = True
            if self.blind and reading['backup']:
                return None, log
        else:
            return None, log
        for name in ('roll', 'pitch'):
            value = getattr(self, name)
            if angle_vetting.around(value, 180.0) < angle_vetting.GATE_MARGIN:
                raise angle_vetting.NearThreshold('fused', 'a published %s %r sits on the +/-180 wrap' % (name, value))
            setattr(self, name, angle_vetting.wrap180(value))
        self.yaw = angle_vetting.wrap360(self.yaw)
        return [self.yaw, self.roll, self.pitch], log

    def _integrate(self, dt: float, now_ms: int, reading: dict) -> None:
        """Free-run one tick (Attitude._integrate), `dt` in seconds."""
        rate = reading['rate_cd_s']
        roll_delta = pitch_delta = yaw_delta = 0.0
        turning = True
        if rate is not None:
            roll_delta, pitch_delta, yaw_delta = (axis / _SCALE * dt for axis in rate)
            turning = abs(rate[2]) > self.turn_gate_cd_s  # integers on both sides: whole-degree gates
        self.yaw = angle_vetting.wrap360(self.yaw + yaw_delta)
        course = None
        if reading['course_cd'] is not None and reading['speed'] is not None:
            if abs(reading['speed'] - self.course_gate) < _SPEED_MARGIN:
                raise angle_vetting.NearThreshold('speed', 'speed %r on the course gate' % reading['speed'])
            if reading['speed'] > self.course_gate:
                course = reading['course_cd'] / _SCALE
        if course is not None:
            self.yaw = angle_vetting.wrap360(self.yaw + self._error('course', course - self.yaw) * self.course_pull)
        self._magnetic_yaw(course, reading['mag'])
        target = None
        if not turning and now_ms - self.accel_ms >= self.accel_period_ms:
            self.accel_ms = now_ms
            accel = reading['accel_cg']
            if accel is not None:
                magnitude = sum(axis * axis for axis in accel)
                if min(abs(magnitude - self.low_sq), abs(magnitude - self.high_sq)) < _BAND_MARGIN:
                    raise angle_vetting.NearThreshold('accel_cg', '|a|^2 %d cg^2 on the gravity band' % magnitude)
                if self.low_sq <= magnitude <= self.high_sq:
                    if not angle_vetting.accel_usable(tuple(accel)):
                        raise angle_vetting.NearThreshold('accel_cg', 'plan B misreads %r by more than %g degree' % (
                            accel, angle_vetting.TARGET_ERROR))
                    target = angle_vetting.true_accel_angles(tuple(accel))
        self.roll += roll_delta
        self.pitch += pitch_delta
        if target is not None:
            self.roll += (target[0] - self.roll) * self.correction
            self.pitch += (target[1] - self.pitch) * self.correction

    def _error(self, channel: str, difference: float) -> float:
        """A pull's error wrapped to [-180, 180), refused within angle_vetting.WRAP_MARGIN of the wrap."""
        if angle_vetting.around(difference, 180.0) < angle_vetting.WRAP_MARGIN:
            raise angle_vetting.NearThreshold(channel, 'a pull error of %r degrees sits on the wrap' % difference)
        return angle_vetting.wrap180(difference)

    def _magnetic_yaw(self, course: float | None, mag: list | None) -> None:
        """The magnetic yaw (Attitude._magnetic_yaw): learn the offset while tracking, use it when not."""
        if mag is None:
            return
        for name in ('roll', 'pitch'):
            if abs(abs(angle_vetting.wrap180(getattr(self, name))) - self.level) < angle_vetting.GATE_MARGIN:
                raise angle_vetting.NearThreshold('mag', '%s %r on the level gate' % (name, getattr(self, name)))
        if abs(angle_vetting.wrap180(self.roll)) > self.level or abs(angle_vetting.wrap180(self.pitch)) > self.level:
            return
        if not angle_vetting.magnetic_usable(tuple(mag)):
            raise angle_vetting.NearThreshold('mag', 'plan B misreads %r by more than %g degree' % (
                mag, angle_vetting.TARGET_ERROR))
        magnetic = angle_vetting.true_magnetic(tuple(mag))
        if course is not None:
            target = angle_vetting.wrap180(course - magnetic)
            error = self._error('mag', target - self.offset)
            self.offset = angle_vetting.wrap360(self.offset + error * self.offset_pull)
            self.known = True
            return
        if not self.known:
            return
        error = self._error('mag', magnetic + self.offset - self.yaw)
        self.yaw = angle_vetting.wrap360(self.yaw + error * self.mag_pull)


class _Run:
    """One scenario: plan B's filter with its channels stood in for, the vetting law, and the steps so far."""

    def __init__(self, name: str, note: str, config: dict) -> None:
        """
        Set up plan B's filter on `config` and the law beside it.

        Args:
            name - the scenario name.
            note - what it shows.
            config - the attitude component's config.

        Raises:
            SystemExit - plan B's setup() failed.
        """
        self.name: str = name
        self.note: str = note
        self.config: dict = config
        self.unit = attitude.Attitude(_OWN_SOURCE, copy.deepcopy(config), None)
        if not asyncio.run(self.unit.setup()):
            raise SystemExit('attitude: plan B setup() failed for %r' % config)
        self.rank: int = self.unit._attitude.rank
        self.decimate_us: int = self.unit._telemetry.decimate_us  # attitude.csv's, as setup() resolved it
        self.unit._telemetry = _Rows()
        self.unit._attitude = _Own(self.rank)
        self.fused = _Fused()
        self.unit._attitude_param = self.fused
        self.parameters: dict = {'rate_cd_s': _Parameter(), 'accel_cg': _Parameter(), 'course_cd': _Parameter(),
                                 'speed': _Parameter(), 'mag': _Parameter()}
        self.unit._rate = self.parameters['rate_cd_s']
        self.unit._accel = self.parameters['accel_cg']
        self.unit._course = self.parameters['course_cd']
        self.unit._speed = self.parameters['speed']
        self.unit._mag = self.parameters['mag']
        self.settings: dict = self._settings()
        self.law = _Law(self.unit, self.rank)
        self.reading: dict = {'rate_cd_s': None, 'accel_cg': None, 'course_cd': None, 'speed': None, 'mag': None,
                              'fused': None, 'backup': False}
        self.now_ms: int = 0
        self.steps: list = []
        self.worst: float = 0.0

    def _settings(self) -> dict:
        """The settings plan B's setup() resolved, in its own units: what init.settings reports."""
        unit = self.unit
        return {'period_ms': unit._period_ms, 'accel_period_us': unit._accel_period_us, 'corr_shift': unit._corr_shift,
                'turn_gate_cd_s': unit._turn_gate, 'course_gate_mps': unit._course_gate,
                'mag_shift': unit._mag_shift, 'mag_offset_shift': unit._mag_offset_shift,
                'mag_level_cd': unit._mag_level_cd, 'course_shift': unit._course_shift,
                'grav_low_sq_cg2': unit._grav_lo_sq, 'grav_high_sq_cg2': unit._grav_hi_sq, 'rank': self.rank,
                'telemetry_decimate_us': self.decimate_us}

    def feed(self, reading: dict, stale: dict) -> None:
        """
        Hand plan B's stand-ins one tick's readings.

        Args:
            reading - the fresh readings, in the fixture's units (null: no fresh source).
            stale - the extrapolated values plan B's read() would answer for the channels that are not fresh.
        """
        for channel, parameter in self.parameters.items():
            value = reading[channel]
            parameter.fresh = value is not None
            if value is None:
                parameter.value = stale.get(channel)
            elif channel in ('accel_cg', 'course_cd'):
                parameter.value = tuple(axis / _SCALE for axis in value) if channel == 'accel_cg' else value / _SCALE
            else:
                parameter.value = tuple(value) if isinstance(value, list) else value
        fused = reading['fused']
        if fused is None:
            self.fused.value, self.fused.source = stale.get('fused'), None
        else:
            heading_cd, roll_cd, pitch_cd = fused['value']
            self.fused.value, self.fused.source = (heading_cd / _SCALE, roll_cd, pitch_cd), fused['source']
        self.fused.backup = reading['backup']

    def check(self, reading: dict) -> None:
        """
        Refuse readings plan B could not hand over exactly: a centi-g or centidegree from_float() does not keep.

        Raises:
            angle_vetting.NearThreshold - a reading is not kept.
        """
        accel = reading['accel_cg']
        if accel is not None and not all(angle_vetting.kept(axis) for axis in accel):
            raise angle_vetting.NearThreshold('accel_cg', '%r is not kept by from_float' % (accel,))
        if reading['course_cd'] is not None and not angle_vetting.kept(reading['course_cd']):
            raise angle_vetting.NearThreshold('course_cd', '%r is not kept by from_float' % reading['course_cd'])
        fused = reading['fused']
        if fused is not None and not angle_vetting.kept(fused['value'][0]):
            raise angle_vetting.NearThreshold('fused', 'heading %r is not kept by from_float' % fused['value'][0])

    def step(self, dt_ms: int, stale: dict | None = None, **changes: object) -> None:
        """
        One tick on plan B and the law, recorded.

        Args:
            dt_ms - how long after the last tick (0 or more).
            stale - the extrapolated values plan B's read() would answer for the stale channels.
            changes - the readings that change from the last tick (the rest carry over).

        Raises:
            angle_vetting.NearThreshold - a reading or a decision is too close to call.
            ValueError - plan B and the law disagree on a flag, or plan B strays past angle_vetting.BUDGET.
        """
        reading = dict(self.reading, **changes)
        self.check(reading)
        law = copy.deepcopy(self.law)  # a dry run first: a refused decision raises before plan B moves
        published, logged = law.tick(self.now_ms + dt_ms, reading)
        self.law, self.reading = law, reading
        self.now_ms += dt_ms
        shims.advance(dt_ms)
        self.feed(reading, stale or {})
        pushes, lines = len(self.unit._attitude.pushed), len(_LOG)
        self.unit._tick()
        pushed = self.unit._attitude.pushed[pushes:]
        lines = _LOG[lines:]
        unit = self.unit
        flags = {'free': unit._free, 'blind': unit.blind, 'seeded': unit._seeded, 'mag_known': unit._mag_known,
                 'log': lines[0] if lines else None}
        mine = {'free': law.free, 'blind': law.blind, 'seeded': law.seeded, 'mag_known': law.known, 'log': logged}
        if flags != mine or len(lines) > 1 or len(pushed) > 1 or bool(pushed) != (published is not None):
            raise ValueError('%s at %d ms: plan B %r (published %r) but the law %r (published %r)' % (
                self.name, self.now_ms, flags, pushed, mine, published))
        state = [unit._yaw_cd / _SCALE, unit._roll_cd / _SCALE, unit._pitch_cd / _SCALE]
        offset = unit._mag_offset_cd / _SCALE
        out = None
        if pushed:
            heading, roll_cd, pitch_cd = pushed[0]
            out = [heading, roll_cd / _SCALE, pitch_cd / _SCALE]
        pairs = list(zip(state, [law.yaw, law.roll, law.pitch])) + [(offset, law.offset)]
        if out is not None:
            pairs += list(zip(out, published))
        self.worst = max([self.worst] + [angle_vetting.around(mine, theirs) for mine, theirs in pairs])
        if self.worst > angle_vetting.BUDGET:
            raise ValueError('%s at %d ms: plan B strays %.4f degree from the real-number filter (budget %g)' % (
                self.name, self.now_ms, self.worst, angle_vetting.BUDGET))
        self.steps.append({'in': dict(reading, now_ms=self.now_ms),
                           'out': {'exact': flags, 'degrees': {'state': state, 'mag_offset': offset,
                                                               'published': out}}})

    def document(self) -> dict:
        """The scenario as the fixture writes it, closed by plan B's inspect() (its attitude keys)."""
        inspected = self.unit.inspect()
        for key in ('name', 'ok', 'healthy'):  # the Task's own keys: the firmware's Inspectable adds those
            inspected.pop(key)
        return {'name': self.name, 'note': self.note, 'init': {'config': self.config, 'settings': self.settings},
                'steps': self.steps, 'inspect': inspected}


def _from(source: str, heading_cd: int, roll_cd: int, pitch_cd: int) -> dict:
    """A fresh fused winner from `source`, its value in plan B's units (the heading in centidegrees here)."""
    return {'source': source, 'rank': _RANKS[source], 'value': [heading_cd, roll_cd, pitch_cd]}


def _primary(heading_cd: int, roll_cd: int, pitch_cd: int) -> dict:
    """A fresh fused winner from the priority-0 source (the HITL sim): the one the filter mirrors."""
    return _from('primary', heading_cd, roll_cd, pitch_cd)


def _default_component() -> dict:
    """config_default's own attitude component."""
    document = config_default.default()
    for section in document.values():
        if isinstance(section, list):
            for component in section:
                if isinstance(component, dict) and component.get('name') == 'attitude':
                    return copy.deepcopy(component)
    raise SystemExit('attitude: config_default has no attitude component')


def _explicit() -> list:
    """
    The scripted scenarios: test_attitude.py block by block, the magnetic yaw through the tick, the porting corners and
    config_default.

    Returns:
        The finished _Run objects.
    """
    runs = []

    def scenario(name: str, note: str, config: dict = _TEST_CONFIG) -> _Run:
        run = _Run(name, note, config)
        runs.append(run)
        return run

    run = scenario('mirror_an_outranking_source',
                   'test_attitude: while a priority-0 source (the HITL sim) is fresh the filter COPIES it -- roll and '
                   'pitch already centidegrees, the heading float degrees -- and does not free-run; once it goes stale '
                   'the seeded filter free-runs from the copy.')
    run.step(20, rate_cd_s=[0, 0, 0], fused=_primary(12300, 4500, -600))
    run.step(20, fused=_primary(12500, 4000, -500))
    run.step(20, fused=None)
    run.step(20)

    run = scenario('free_run_integrates_the_gyro',
                   'test_attitude: the accelerometer present but outside the 1 g band (no correction): roll, pitch and '
                   'yaw integrate the rate, +10, -5 and +20 degrees over a second of 20 ms ticks.')
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=list(_NO_GRAVITY), rate_cd_s=[1000, -500, 2000])
    for _ in range(49):
        run.step(20)

    run = scenario('the_accel_pulls_toward_gravity',
                   'test_attitude: at rest at roll 30 the gravity vector pulls roll toward it over 40 ticks; then a '
                   'high-g reading (boost, ~4.2 g) is rejected and the estimate holds.')
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=[0, 50, 86], rate_cd_s=[0, 0, 0])
    for _ in range(39):
        run.step(20)
    for _ in range(3):
        run.step(20, accel_cg=[0, 300, 300])

    run = scenario('the_turn_gate_holds_the_bank',
                   'test_attitude: banked 40 degrees with the accelerometer reading level (a coordinated turn) and a '
                   '30 degree/s yaw rate, the correction is gated off and the bank holds; at 1 degree/s (straight-ish) '
                   'gravity re-anchors roll to level.')
    run.step(20, fused=_primary(0, 4000, 0))
    run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 3000])
    run.step(20)
    for _ in range(40):
        run.step(20, rate_cd_s=[0, 0, 100])

    run = scenario('yaw_follows_the_ground_track',
                   'test_attitude: moving, yaw is pulled to the GNSS track (90 degrees); below the course gate the '
                   'track is ignored and the gyro alone holds yaw.')
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0], course_cd=9000, speed=14.0)
    for _ in range(39):
        run.step(20)
    for _ in range(5):
        run.step(20, speed=1.0)

    run = scenario('the_track_pull_wraps_through_north',
                   'test_attitude: a track just past north pulls a yaw of 358 degrees UP through 360, not back down.')
    run.step(20, fused=_primary(35800, 0, 0))
    run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0], course_cd=200, speed=14.0)
    for _ in range(11):
        run.step(20)

    run = scenario('a_silent_primary_hands_over_to_the_gyro',
                   'test_attitude: the priority-0 source pushes once and goes silent. Plan B\'s read() then answers '
                   'its extrapolated value with no source, which the filter must not mirror (it once did, pinning a '
                   'dead heading after every >40 ms gap): it free-runs the 30 degree/s yaw instead.')
    run.step(20, accel_cg=list(_NO_GRAVITY), rate_cd_s=[0, 0, 3000], fused=_primary(1000, 500, 0))
    for _ in range(19):
        run.step(20, fused=None, stale={'fused': (10.0, 500, 0)})

    run = scenario('blind_without_a_backup_keeps_publishing',
                   'test_attitude: 1..4 gyro-less ticks are a GC pause or a stall; the fifth raises blind (logged '
                   'once). With nothing fresh below the filter it keeps publishing its held estimate. One fresh sample '
                   'ends blind. A stale channel -- a value with no source -- is no gyro either.')
    run.step(20, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0])
    for _ in range(7):
        run.step(20, rate_cd_s=None)
    run.step(20, rate_cd_s=[0, 0, 0])
    for _ in range(5):
        run.step(20, rate_cd_s=None, stale={'rate_cd_s': (0, 0, 0)})
    run.step(20, rate_cd_s=[0, 0, 0])

    run = scenario('blind_while_mirroring',
                   'test_attitude: the blind window is counted in either regime, so the flag is true before a '
                   'handover.')
    for _ in range(5):
        run.step(20, fused=_primary(1000, 500, 0))
    run.step(20, rate_cd_s=[0, 0, 0])
    run.step(20, fused=None)

    run = scenario('never_mirror_a_backup_or_a_peer',
                   'test_attitude: a source ranked BELOW the filter (the BNO055 backup) winning a tick is never copied '
                   '-- its heading is in another frame; neither is a peer at the filter\'s own rank, nor the filter\'s '
                   'own last push winning.')
    run.step(20, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0], fused=_from('backup', 7700, 2500, 1500))
    for _ in range(9):
        run.step(20)
    for _ in range(5):
        run.step(20, fused=_from('peer', 7700, 2500, 1500))
    for _ in range(5):
        run.step(20, fused=_from(_OWN_SOURCE, 0, 0, 0))

    run = scenario('blind_with_a_fresh_backup_withholds',
                   'test_attitude: no false withhold on 1..4 gyro-less ticks; the fifth withholds so the channel goes '
                   'stale and the databoard hands the attitude to the fresh backup. A backup that went quiet too gets '
                   'the publish back. The gyro back: the filter resumes.')
    run.step(20, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0])
    for _ in range(4):
        run.step(20, rate_cd_s=None, backup=True, fused=_from(_OWN_SOURCE, 0, 0, 0))
    for _ in range(3):
        run.step(20, fused=_from('backup', 7700, 2500, 1500))
    run.step(20, fused=None, backup=False)
    run.step(20)
    run.step(20, rate_cd_s=[0, 0, 0], backup=True)

    run = scenario('never_seeded_and_no_accel_publishes_nothing',
                   'Corner: a filter no source ever seeded runs only while the accelerometer is fresh. Without it the '
                   'tick publishes nothing AND integrates nothing -- the gyro of that tick is lost, not deferred -- '
                   'and `free` keeps its last value: nothing clears it.')
    run.step(20, rate_cd_s=[1000, 0, 0])
    run.step(20)
    run.step(20, accel_cg=list(_NO_GRAVITY))
    run.step(20, accel_cg=None)
    run.step(20)
    run.step(20, accel_cg=list(_NO_GRAVITY))

    run = scenario('a_long_gap_integrates_one_nominal_period',
                   'Corner: a gap over 500 ms (an asyncio stall) integrates one nominal period instead of one huge '
                   'step the yaw would hold for good; exactly 500 ms is integrated whole; a repeated time integrates '
                   'nothing.')
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=list(_NO_GRAVITY), rate_cd_s=[1000, 0, 0])
    run.step(600)
    run.step(0)
    run.step(500)
    run.step(501)
    run.step(20)

    run = scenario('the_accel_correction_is_throttled',
                   'Corner: accel_period_ms 50 at 20 ms ticks pulls every third tick (60 ms apart). A turn neither '
                   'checks nor restarts the throttle clock, so the first straight tick after it corrects at once.',
                   dict(_TEST_CONFIG, accel_period_ms=50))
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=[0, 50, 86], rate_cd_s=[0, 0, 0])
    for _ in range(10):
        run.step(20)
    for _ in range(5):
        run.step(20, rate_cd_s=[0, 0, 500])
    for _ in range(6):
        run.step(20, rate_cd_s=[0, 0, 0])

    run = scenario('the_gravity_band_edges',
                   'Corner: the gravity vector is trusted only inside 0.7..1.3 g (squared, so no square root): 0.72 g '
                   'and 1.29 g pull, 0.68 g and 1.32 g do not.')
    run.step(20, fused=_primary(0, 1000, 600))
    run.step(20, fused=None, accel_cg=[0, 0, 72], rate_cd_s=[0, 0, 0])
    for _ in range(3):
        run.step(20)
    for _ in range(3):
        run.step(20, accel_cg=[0, 0, 68])
    for _ in range(3):
        run.step(20, accel_cg=[0, 0, 129])
    for _ in range(3):
        run.step(20, accel_cg=[0, 0, 132])

    run = scenario('the_magnetometer_carries_yaw_through_a_gnss_dropout',
                   'test_magnetic_yaw through the tick: while the track is good the magnetometer is MEASURED against '
                   'it (the offset learned, the mag steering nothing); when the track drops, the learned offset makes '
                   'the magnetometer a standalone heading and yaw is pulled to it.', _MAGNETIC_CONFIG)
    run.step(20, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0], mag=[1000, 0, 0], course_cd=9000, speed=14.0)
    for _ in range(40):
        run.step(20)
    run.step(20, fused=_primary(0, 0, 0))
    for _ in range(40):
        run.step(20, fused=None, course_cd=None, speed=None, stale={'course_cd': 90.0, 'speed': 14.0})

    run = scenario('an_unlearned_magnetometer_never_steers',
                   'test_magnetic_yaw: with no track ever seen the offset is unlearned -- a zero that would assert '
                   'magnetic north IS the track -- so the magnetometer must leave yaw alone.', _MAGNETIC_CONFIG)
    run.step(20, fused=_primary(12345, 0, 0), mag=[0, 1000, 0])
    for _ in range(20):
        run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0])

    run = scenario('the_level_gate_refuses_a_banked_magnetometer',
                   'test_magnetic_yaw: past mag_level_deg a flat atan2 is not a heading, so a banked or pitched '
                   'airframe learns nothing from the track; level again, it learns.', _MAGNETIC_CONFIG)
    run.step(20, fused=_primary(9000, 3000, 0))
    for _ in range(3):
        run.step(20, fused=None, accel_cg=list(_NO_GRAVITY), rate_cd_s=[0, 0, 0], mag=[1000, 0, 0], course_cd=9000,
                 speed=14.0)
    run.step(20, fused=_primary(9000, 0, 4000))
    for _ in range(3):
        run.step(20, fused=None)
    run.step(20, fused=_primary(9000, 0, 0))
    for _ in range(3):
        run.step(20, fused=None)

    run = scenario('the_level_gate_reads_the_roll_before_the_gyro_step',
                   'test_magnetic_yaw\'s level gate through the tick: plan B gates the magnetometer on the roll it had '
                   'BEFORE this tick\'s gyro step (_magnetic_heading runs before blend_cd). Mirrored at roll 14.5 and '
                   'rolling at 50 degrees/s, the first tick learns -- gated on 14.5 -- although it ends at 15.5; the '
                   'next starts at 15.5 and refuses. Rolling back, the ticks that start at 16.5 and 15.5 refuse and '
                   'the one that starts at 14.5 learns again.', _MAGNETIC_CONFIG)
    run.step(20, fused=_primary(0, 1450, 0), rate_cd_s=[0, 0, 0])
    run.step(20, fused=None, rate_cd_s=[5000, 0, 0], mag=[1000, 0, 0], course_cd=9000, speed=14.0)
    run.step(20)
    for _ in range(3):
        run.step(20, rate_cd_s=[-5000, 0, 0])

    run = scenario('the_track_needs_the_course_and_the_speed',
                   'Corner: the GNSS track needs BOTH the course and the ground speed fresh -- plan B drops both when '
                   'either has no source. The course alone (the speed stale, an extrapolated 14 m/s), then the speed '
                   'alone (the course stale): no yaw pull, and nothing learned against the magnetometer. Both fresh: '
                   'the track pulls and the offset is learned.', _MAGNETIC_CONFIG)
    run.step(20, fused=_primary(4500, 0, 0), rate_cd_s=[0, 0, 0], accel_cg=list(_LEVEL), mag=[1000, 0, 0])
    for _ in range(3):
        run.step(20, fused=None, course_cd=9000, speed=None, stale={'speed': 14.0})
    for _ in range(3):
        run.step(20, course_cd=None, speed=14.0, stale={'course_cd': 90.0})
    for _ in range(2):
        run.step(20, course_cd=9000, speed=14.0)

    run = scenario('the_accel_pull_goes_the_long_way_round',
                   'A plan B hazard the port keeps: the accelerometer pull (fixed.blend_cd) takes its error UNWRAPPED. '
                   'Mirrored at roll 170, then gravity at roll -170 (inverted, inside the 1 g band): the short way is '
                   '20 degrees, but the error is -340, so at corr_shift 2 the roll sweeps 85, 21, -27, -62, -89, -109 '
                   'through wings-level, and only then settles on -170.')
    run.step(20, fused=_primary(0, 17000, 0), rate_cd_s=[0, 0, 0])
    for _ in range(12):
        run.step(20, fused=None, accel_cg=[0, -17, -98])

    run = scenario('a_long_free_run_is_exact',
                   'Corner: 20 s of pure gyro integration -- 1000 ticks of 20 ms at (30, -20, 30) degrees/s, the '
                   'accelerometer outside the band, no track, no magnetometer -- sweeping roll and yaw 600 degrees and '
                   'pitch 400. Plan B integrates whole-degree rates over 10 ms multiples exactly, so the 0.1 degree '
                   'tolerance resolves a gyro step wrong by under 0.02 %. It starts at roll 0.3 and pitch 0.2, so no '
                   'published roll or pitch sits on the +/-180 wrap.')
    run.step(20, fused=_primary(0, 30, 20))
    run.step(20, fused=None, accel_cg=list(_NO_GRAVITY), rate_cd_s=[3000, -2000, 3000])
    for _ in range(999):
        run.step(20)

    component = _default_component()
    run = scenario('config_default_on_the_pad',
                   'config_default\'s own settings (accel_period_ms 50, corr_shift 4) on a level pad: a source seeds '
                   'the attitude above level and the 1/16 pull brings it down; the blind window, with no backup and '
                   'then with a fresh one.', component)
    run.step(20, fused=_primary(9000, 300, 200))
    run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[0, 0, 0])
    for _ in range(30):
        run.step(20)
    for _ in range(6):
        run.step(20, rate_cd_s=None)
    run.step(20, rate_cd_s=[0, 0, 0])
    for _ in range(6):
        run.step(20, rate_cd_s=None, backup=True)
    run.step(20, rate_cd_s=[0, 0, 0])

    run = scenario('config_default_a_slow_roll',
                   'config_default\'s settings: a 5 degree/s roll on a level gravity vector, under the 4 degree/s turn '
                   'gate on yaw, so every third tick pulls it back toward level.', component)
    run.step(20, fused=_primary(0, 0, 0))
    run.step(20, fused=None, accel_cg=list(_LEVEL), rate_cd_s=[500, 0, 0])
    for _ in range(40):
        run.step(20)
    return runs


"""The seeded flights' knobs: what a tick draws, as probabilities per tick."""
_GYRO_DROP: float = 0.03  # a gyro dropout starts (1..8 ticks)
_ACCEL_DROP: float = 0.05  # the accelerometer is stale this tick
_BOOST_G: float = 0.04  # a high-g reading (outside the band)
_GNSS_DROP: float = 0.04  # a GNSS dropout starts (1..30 ticks)
_MAG_PRESENT: float = 0.7  # the magnetometer is fresh this tick
_GAP: float = 0.02  # a long gap (an asyncio stall)


def _sweep(generator: random.Random, name: str) -> _Run:
    """
    One seeded flight: a true attitude driven by whole-degree rates, the sensors that see it, and their faults.

    The gyro reads the true rate; the accelerometer the true gravity vector (centi-g, a usable vector near it), or a
    high-g boost reading, or nothing; the GNSS the true heading plus a crab, at a drawn speed, with dropouts; the
    magnetometer the true heading plus a constant lump (declination and mounting). A priority-0 source may seed the
    flight at the start (the HITL sim) and drop out; the fused winner is otherwise the filter itself, sometimes the
    backup or a peer; a backup below is fresh on some ticks. A reading whose decision sits on a threshold is dropped
    for that tick.

    Args:
        generator - the seeded random source.
        name - the scenario name.

    Returns:
        The finished run.

    Raises:
        angle_vetting.NearThreshold - a decision that dropping a reading cannot avoid (the caller draws again).
    """
    config = {'name': 'attitude', 'activity': 'attitude', 'enabled': True, 'period_ms': 20,
              'accel_period_ms': generator.choice((0, 20, 40, 50)), 'corr_shift': generator.choice((1, 2)),
              'grav_low_g': 0.7, 'grav_high_g': 1.3, 'turn_gate_deg_s': generator.choice((2, 4, 6)),
              'course_gate_mps': generator.choice((3.0, 5.0)), 'course_shift': generator.choice((1, 2)),
              'mag_shift': generator.choice((1, 2)), 'mag_offset_shift': 1,
              'mag_level_deg': generator.choice((10, 15, 20)),
              'provides': {'attitude': {'priority': 1, 'timeout_ms': 40}}}
    run = _Run(name, 'Seeded flight: whole-degree rates, gravity, track and field from a true attitude, with dropouts, '
                     'boosts, gaps, a seeding source and a backup below.', config)
    roll, pitch, heading = generator.uniform(-20, 20), generator.uniform(-10, 10), generator.uniform(0, 360)
    rates = [generator.randrange(-10, 11) * 100, generator.randrange(-5, 6) * 100, generator.randrange(-10, 11) * 100]
    lump, field, crab = generator.uniform(-180, 180), generator.randint(300, 1000), generator.uniform(-5, 5)
    seeding = generator.choice((0, 0, 10, 30))
    gyro_gone = gnss_gone = 0
    speed = generator.uniform(2.0, 20.0)
    last = {}
    for tick in range(_SWEEP_STEPS):
        dt_ms = 600 if generator.random() < _GAP else generator.choice((10, 20, 20, 20, 30, 40))
        limits = (50, 35, None)
        for axis in range(3):
            rates[axis] += generator.choice((-100, 0, 0, 100))
            rates[axis] = max(-3000, min(3000, rates[axis]))
        for axis, value in enumerate((roll, pitch)):
            if abs(value) > limits[axis]:
                rates[axis] = -100 * int(math.copysign(10, value))
        step_s = (20 if dt_ms > _MAXIMUM_GAP_MS else dt_ms) / 1000.0
        roll += rates[0] / _SCALE * step_s
        pitch += rates[1] / _SCALE * step_s
        heading = (heading + rates[2] / _SCALE * step_s) % 360.0
        if gyro_gone == 0 and generator.random() < _GYRO_DROP:
            gyro_gone = generator.randint(1, 8)
        rate = None if gyro_gone else list(rates)
        gyro_gone = max(0, gyro_gone - 1)
        draw = generator.random()
        if draw < _ACCEL_DROP:
            accel = None
        elif draw < _ACCEL_DROP + _BOOST_G:
            accel = [0, 0, generator.randint(150, 400)]
        else:
            r, p = math.radians(roll), math.radians(pitch)
            gravity = (-100 * math.sin(p), 100 * math.cos(p) * math.sin(r), 100 * math.cos(p) * math.cos(r))
            found = angle_vetting.nearest(gravity, angle_vetting.accel_usable)
            accel = None if found is None else list(found)
        if gnss_gone == 0 and generator.random() < _GNSS_DROP:
            gnss_gone = generator.randint(1, 30)
        speed = max(0.5, min(25.0, speed + generator.uniform(-0.5, 0.5)))
        course = speed_now = None
        if not gnss_gone:
            course = angle_vetting.kept_near(int(round(((heading + crab) % 360.0) * _SCALE)) % 36000)
            speed_now = speed
        gnss_gone = max(0, gnss_gone - 1)
        mag = None
        if generator.random() < _MAG_PRESENT:
            magnetic = math.radians(heading + lump)
            horizontal = (field * math.cos(magnetic), -field * math.sin(magnetic), -400)
            found = angle_vetting.nearest(horizontal, angle_vetting.magnetic_usable)
            mag = None if found is None else list(found)
        truth = [angle_vetting.kept_near(int(round(heading * _SCALE)) % 36000), int(round(roll * _SCALE)),
                 int(round(pitch * _SCALE))]
        draw = generator.random()
        if tick < seeding:
            fused = _primary(*truth)
        elif draw < 0.05:
            fused = _from('backup', *truth)
        elif draw < 0.08:
            fused = _from('peer', *truth)
        elif draw < 0.7 and run.unit._attitude.pushed:
            fused = _from(_OWN_SOURCE, *truth)
        else:
            fused = None
        reading = {'rate_cd_s': rate, 'accel_cg': accel, 'course_cd': course, 'speed': speed_now, 'mag': mag,
                   'fused': fused, 'backup': generator.random() < 0.3}
        stale = {channel: last.get(channel) for channel in reading if reading[channel] is None}
        stale['course_cd'] = None if last.get('course_cd') is None else last['course_cd'] / _SCALE
        stale['accel_cg'] = None if last.get('accel_cg') is None else tuple(axis / _SCALE for axis in last['accel_cg'])
        for _attempt in range(4):
            try:
                run.step(dt_ms, stale=stale, **reading)
                break
            except angle_vetting.NearThreshold as refusal:
                if refusal.channel == 'fused':
                    raise
                for channel in (('course_cd', 'speed') if refusal.channel == 'speed' else (refusal.channel,)):
                    reading[channel] = None
        else:
            raise angle_vetting.NearThreshold('fused', '%s: tick %d stays near a threshold' % (name, tick))
        for channel, value in reading.items():
            if value is not None and channel in run.parameters:
                last[channel] = value
    return run


def build() -> dict:
    """
    The attitude fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, measured, scenarios}.

    Raises:
        ValueError - plan B and the law disagree on a flag, or plan B strays past angle_vetting.BUDGET.
        angle_vetting.NearThreshold - a scripted scenario sits on a threshold.
    """
    recorder.Recorder.log = staticmethod(_capture_log)
    runs = _explicit()
    generator = random.Random(_SEED)
    redrawn = 0
    for index in range(_SWEEPS):
        while True:
            try:
                runs.append(_sweep(generator, 'seeded_flight_%02d' % index))
                break
            except angle_vetting.NearThreshold:
                redrawn += 1
    if shims._ticks_us() < shims._BOOT_UPTIME_US:
        raise ValueError('attitude: the virtual clock wrapped (2**30 us): the timers would no longer be exact')
    steps = sum(len(run.steps) for run in runs)
    return {
        'meta': {'source': 'src/glider/tasks/attitude.py, src/glider/fixed.py'},
        'compare': {'exact': {'kind': 'exact'}, 'degrees': {'kind': 'angle_deg', 'tol': angle_vetting.TOLERANCE}},
        'note': ('a tick per step; inputs in plan B units: rate_cd_s / 100 degrees per second, accel_cg / 100 g, '
                 'course_cd / 100 degrees, fused value [heading_cd, roll_cd, pitch_cd] / 100 degrees, mag counts, '
                 'now_ms after setup; flags exact, angles in degrees within 0.1 the short way around'),
        'measured': {'worst_plan_b_against_the_law_degrees': round(max(run.worst for run in runs), 5),
                     'longest_scenario_steps': max(len(run.steps) for run in runs), 'steps': steps,
                     'seeded_flights_redrawn': redrawn},
        'scenarios': {'note': 'stateful: {name, note, init: {config, settings}, steps: [{in, out}], inspect}',
                      'cases': [run.document() for run in runs]},
    }
