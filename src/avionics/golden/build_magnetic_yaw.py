"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture magnetic_yaw.json: plan B's magnetic yaw (src/glider/tasks/attitude.py, Attitude._magnetic_heading
and Attitude._magnetic_yaw) as the Rust port (crates/control, module attitude, MagneticYaw) must answer it, call by
call. This is the path that exists for the GNSS dropout at boost: the magnetometer is measured against the ground
track while the track is good (one offset learned: declination plus mounting), and carries the yaw on that offset
when the track is gone. test_magnetic_yaw.py drives the two methods on their own, against synthetic fields, and so
does this fixture; attitude.json covers the same path through the filter's tick.

Each scenario starts from `init` -- the magnetometer knobs as config keys (mag_shift, mag_offset_shift,
mag_level_deg), the settings plan B resolved from them, and the starting yaw in centidegrees -- with the offset
unlearned. Its steps, in plan B's own units (`mag` counts, `roll_cd`/`pitch_cd` the filter's attitude in
centidegrees, `course_cd` the ground track in centidegrees, handed to plan B as course_cd / 100, a float its
from_float() reads back exactly), are one of:
  heading  _magnetic_heading() at that attitude -- out: degrees.heading, the magnetic heading in degrees or null
           (no fresh field, or a banked or pitched airframe past the level gate);
  yaw      _magnetic_yaw(tracking, course) -- out: exact.known (the offset has been learned) and degrees.yaw (the
           yaw after the call; the scenario carries it into the next step) and degrees.offset.
A null `mag` is a magnetometer with no fresh source.

Compared as attitude.json is: `exact` exactly, `degrees` within angle_deg 0.1, the plan's tolerance. The builder
runs the real-number law beside plan B (_Law: what the port computes, here in f64) and refuses a fixture in which
plan B strays past BUDGET (angle_vetting.py) or answers `known` differently. Plan B's integer error is held inside the
tolerance as in attitude.json: every field used is one whose plan B heading is within TARGET_ERROR of the true one,
and the pulls are shifts of 1 and 2, as test_magnetic_yaw.py uses (a shift floors, so a pull approaching from below
stalls up to 2^shift - 1 centidegrees short: 0.31 degree at the default mag_offset_shift 5, 0.63 at mag_shift 6).
No roll or pitch sits within GATE_MARGIN of the level gate and no pull error within WRAP_MARGIN of +/-180; a seeded
scenario that would is drawn again.

Scenarios: test_magnetic_yaw.py's four tests, then seeded runs: a heading that turns, a constant lump between the
field and the track, the attitude wandering across the level gate, the track coming and going.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random

import angle_vetting
from tasks import attitude

_SEED: int = 20261006  # the seeded runs' random.Random seed: the fixture is a function of the code alone
_SEEDED_RUNS: int = 8
_SEEDED_STEPS: int = 80
_SCALE: int = angle_vetting.SCALE  # centi-units per unit (fixed.SCALE)

"""test_magnetic_yaw.py's knobs: fast, so a few steps converge."""
_TEST_KNOBS: dict = {'mag_shift': 2, 'mag_offset_shift': 1, 'mag_level_deg': 15}


class _Channel:
    """A databoard channel stand-in: read() returns (value, source, age), as test_magnetic_yaw's stub does."""

    def __init__(self) -> None:
        """No field yet."""
        self.value: tuple | None = None

    def read(self) -> tuple:
        """The field, with a source when there is one."""
        return self.value, 'stub' if self.value is not None else None, 0


class _Task:
    """The parts of plan B's Attitude the magnetic path touches, without a Controller or a databoard."""

    _magnetic_heading = attitude.Attitude._magnetic_heading
    _magnetic_yaw = attitude.Attitude._magnetic_yaw

    def __init__(self, knobs: dict, yaw_cd: int) -> None:
        """
        The state test_magnetic_yaw's stub carries, resolved from the knobs as Attitude.setup() resolves them.

        Args:
            knobs - mag_shift, mag_offset_shift, mag_level_deg.
            yaw_cd - the starting yaw, centidegrees.
        """
        self._mag = _Channel()
        self._roll_cd = self._pitch_cd = 0
        self._yaw_cd: int = yaw_cd
        self._mag_offset_cd: int = 0
        self._mag_known: bool = False
        self._mag_shift: int = knobs.get('mag_shift', 6)
        self._mag_offset_shift: int = knobs.get('mag_offset_shift', 5)
        self._mag_level_cd: int = int(knobs.get('mag_level_deg', 15) * _SCALE)


class _Law:
    """The real-number magnetic yaw the port computes, in f64, only to vet plan B (see the module notes)."""

    def __init__(self, unit: _Task) -> None:
        """The law with plan B's resolved knobs and starting yaw."""
        self.pull: float = 2.0**-unit._mag_shift
        self.offset_pull: float = 2.0**-unit._mag_offset_shift
        self.level: float = unit._mag_level_cd / _SCALE
        self.yaw: float = unit._yaw_cd / _SCALE
        self.offset: float = 0.0
        self.known: bool = False

    def heading(self, mag: list | None, roll: float, pitch: float) -> float | None:
        """
        The magnetic heading in degrees, or None (no field, or past the level gate).

        Raises:
            angle_vetting.NearThreshold - the attitude sits on the level gate, or plan B misreads the field.
        """
        if mag is None:
            return None
        for name, value in (('roll', roll), ('pitch', pitch)):
            if abs(abs(angle_vetting.wrap180(value)) - self.level) < angle_vetting.GATE_MARGIN:
                raise angle_vetting.NearThreshold('attitude', '%s %r on the level gate' % (name, value))
        if abs(angle_vetting.wrap180(roll)) > self.level or abs(angle_vetting.wrap180(pitch)) > self.level:
            return None
        if not angle_vetting.magnetic_usable(tuple(mag)):
            raise angle_vetting.NearThreshold('mag', 'plan B misreads %r by more than %g degree' % (
                mag, angle_vetting.TARGET_ERROR))
        return angle_vetting.true_magnetic(tuple(mag))

    def yaw_step(self, mag: list | None, roll: float, pitch: float, course: float | None) -> None:
        """
        One _magnetic_yaw call: learn the offset while tracking (course given), else pull the yaw once learned.

        Raises:
            angle_vetting.NearThreshold - a decision or a pull error sits on its threshold.
        """
        magnetic = self.heading(mag, roll, pitch)
        if magnetic is None:
            return
        if course is not None:
            target = angle_vetting.wrap180(course - magnetic)
            error = self._error(target - self.offset)
            self.offset = angle_vetting.wrap360(self.offset + error * self.offset_pull)
            self.known = True
            return
        if not self.known:
            return
        self.yaw = angle_vetting.wrap360(self.yaw + self._error(magnetic + self.offset - self.yaw) * self.pull)

    @staticmethod
    def _error(difference: float) -> float:
        """A pull error wrapped to [-180, 180), refused within WRAP_MARGIN of the wrap."""
        if angle_vetting.around(difference, 180.0) < angle_vetting.WRAP_MARGIN:
            raise angle_vetting.NearThreshold('mag', 'a pull error of %r degrees sits on the wrap' % difference)
        return angle_vetting.wrap180(difference)


class _Scenario:
    """One scenario: plan B's stand-in task, the vetting law, and the steps so far."""

    def __init__(self, name: str, note: str, knobs: dict, yaw_cd: int = 0) -> None:
        """A scenario from the knobs and the starting yaw."""
        self.name: str = name
        self.note: str = note
        self.knobs: dict = knobs
        self.yaw_cd: int = yaw_cd
        self.unit = _Task(knobs, yaw_cd)
        self.law = _Law(self.unit)
        self.steps: list = []
        self.worst: float = 0.0

    def _attitude(self, mag: list | None, roll_cd: int, pitch_cd: int) -> None:
        """Hand plan B's stand-in the field and the attitude of a call."""
        self.unit._mag.value = None if mag is None else tuple(mag)
        self.unit._roll_cd, self.unit._pitch_cd = roll_cd, pitch_cd

    def heading(self, mag: list | None, roll_cd: int = 0, pitch_cd: int = 0) -> None:
        """
        A heading call on both, recorded.

        Raises:
            angle_vetting.NearThreshold - the call sits on a threshold.
            ValueError - plan B and the law disagree on whether there is a heading, or by more than BUDGET.
        """
        expected = self.law.heading(mag, roll_cd / _SCALE, pitch_cd / _SCALE)
        self._attitude(mag, roll_cd, pitch_cd)
        answer = self.unit._magnetic_heading()
        heading = None if answer is None else answer / _SCALE
        if (heading is None) != (expected is None):
            raise ValueError('%s: plan B heading %r, the law %r' % (self.name, heading, expected))
        if heading is not None:
            self._measure([(heading, expected)])
        self.steps.append({'in': {'action': 'heading', 'mag': mag, 'roll_cd': roll_cd, 'pitch_cd': pitch_cd},
                           'out': {'degrees': {'heading': heading}}})

    def yaw(self, mag: list | None, course_cd: int | None, roll_cd: int = 0, pitch_cd: int = 0) -> None:
        """
        A _magnetic_yaw call on both, recorded: tracking when `course_cd` is given.

        Raises:
            angle_vetting.NearThreshold - the call sits on a threshold, or the course is not kept by from_float.
            ValueError - plan B and the law disagree on `known`, or by more than BUDGET.
        """
        if course_cd is not None and not angle_vetting.kept(course_cd):
            raise angle_vetting.NearThreshold('course', '%r is not kept by from_float' % course_cd)
        law = _Law.__new__(_Law)
        law.__dict__.update(self.law.__dict__)  # a dry run first: a refused decision raises before plan B moves
        law.yaw_step(mag, roll_cd / _SCALE, pitch_cd / _SCALE, None if course_cd is None else course_cd / _SCALE)
        self.law = law
        self._attitude(mag, roll_cd, pitch_cd)
        tracking = course_cd is not None
        self.unit._magnetic_yaw(tracking, course_cd / _SCALE if tracking else None)
        unit = self.unit
        if unit._mag_known != law.known:
            raise ValueError('%s: plan B known %r, the law %r' % (self.name, unit._mag_known, law.known))
        yaw, offset = unit._yaw_cd / _SCALE, unit._mag_offset_cd / _SCALE
        self._measure([(yaw, law.yaw), (offset, law.offset)])
        self.steps.append({'in': {'action': 'yaw', 'mag': mag, 'roll_cd': roll_cd, 'pitch_cd': pitch_cd,
                                  'tracking': tracking, 'course_cd': course_cd},
                           'out': {'exact': {'known': unit._mag_known}, 'degrees': {'yaw': yaw, 'offset': offset}}})

    def _measure(self, pairs: list) -> None:
        """Track the worst plan B-against-law distance; refuse past BUDGET."""
        self.worst = max([self.worst] + [angle_vetting.around(mine, theirs) for mine, theirs in pairs])
        if self.worst > angle_vetting.BUDGET:
            raise ValueError('%s: plan B strays %.4f degree from the real-number law (budget %g)' % (
                self.name, self.worst, angle_vetting.BUDGET))

    def document(self) -> dict:
        """The scenario as the fixture writes it."""
        unit = self.unit
        settings = {'mag_shift': unit._mag_shift, 'mag_offset_shift': unit._mag_offset_shift,
                    'mag_level_cd': unit._mag_level_cd}
        return {'name': self.name, 'note': self.note,
                'init': {'config': self.knobs, 'settings': settings, 'yaw_cd': self.yaw_cd}, 'steps': self.steps}


def _explicit() -> list:
    """
    test_magnetic_yaw.py's four tests, as scenarios.

    Returns:
        The finished scenarios.
    """
    scenarios = []

    def scenario(name: str, note: str, yaw_cd: int = 0) -> _Scenario:
        built = _Scenario(name, note, _TEST_KNOBS, yaw_cd)
        scenarios.append(built)
        return built

    heading = scenario('heading_comes_from_the_horizontal_axes',
                       'A field along +x reads 0 degrees and along +y -90 (270); past the level gate a flat atan2 is '
                       'not a heading, so a banked or a pitched airframe -- or no fresh field -- yields none.')
    heading.heading([1000, 0, 0])
    heading.heading([0, 1000, 0])
    heading.heading([1000, 0, 0], roll_cd=3000)
    heading.heading([1000, 0, 0], pitch_cd=4000)
    heading.heading([1000, 0, 0], roll_cd=-1400, pitch_cd=1400)
    heading.heading([1000, 0, 0], roll_cd=-1600)
    heading.heading(None)
    learned = scenario('offset_is_learned_from_the_track_then_used_without_it',
                       'The airframe tracks 90 degrees while the field reads 0: the offset learns 90. Then the GNSS '
                       'drops and the yaw, wrong at 0, is pulled to field + offset with no track at all.')
    for _ in range(40):
        learned.yaw([1000, 0, 0], 9000)
    for _ in range(40):
        learned.yaw([1000, 0, 0], None)
    unlearned = scenario('an_unlearned_offset_never_steers',
                         'With no track ever seen the offset is a zero that would claim magnetic north IS the track: '
                         'a freely drifting gyro beats a confidently wrong pull, so yaw is left alone.', yaw_cd=12345)
    for _ in range(20):
        unlearned.yaw([1000, 0, 0], None)
    wins = scenario('the_track_wins_while_it_is_available',
                    'Learned (one tracking call along the field), yaw at 90: while tracking the magnetometer is only '
                    'measured, never steers -- two references fighting is worse than one.', yaw_cd=9000)
    wins.yaw([1000, 0, 0], 0)
    wins.yaw([1000, 0, 0], 0)
    wins.yaw([0, -1000, 0], 4500)
    wins.yaw(None, None)
    wins.yaw([1000, 0, 0], None, roll_cd=2000)
    return scenarios


def _seeded(generator: random.Random, name: str) -> _Scenario:
    """
    One seeded run: a turning heading, a constant lump between field and track, a wandering attitude, a track that
    comes and goes.

    Args:
        generator - the seeded random source.
        name - the scenario name.

    Returns:
        The finished scenario.

    Raises:
        angle_vetting.NearThreshold - a call sits on a threshold (the caller draws again).
    """
    knobs = {'mag_shift': generator.choice((1, 2)), 'mag_offset_shift': generator.choice((1, 2)),
             'mag_level_deg': generator.choice((10, 15, 20))}
    built = _Scenario(name, 'Seeded: a turning heading, a constant field-to-track lump, an attitude wandering across '
                            'the level gate, the track coming and going.', knobs,
                      yaw_cd=generator.randrange(36000))
    heading = generator.uniform(0.0, 360.0)
    turn = generator.uniform(-3.0, 3.0)
    lump = generator.uniform(-180.0, 180.0)
    field = generator.randint(300, 1000)
    roll = pitch = 0.0
    tracking = True
    for _ in range(_SEEDED_STEPS):
        heading = (heading + turn) % 360.0
        roll = max(-30.0, min(30.0, roll + generator.uniform(-3.0, 3.0)))
        pitch = max(-30.0, min(30.0, pitch + generator.uniform(-2.0, 2.0)))
        if generator.random() < 0.08:
            tracking = not tracking
        magnetic = angle_vetting.wrap360(heading + lump)
        radians = math.radians(magnetic)
        mag = None
        if generator.random() < 0.85:
            target = (field * math.cos(radians), -field * math.sin(radians), -400)
            found = angle_vetting.nearest(target, angle_vetting.magnetic_usable)
            mag = None if found is None else list(found)
        roll_cd, pitch_cd = int(round(roll * _SCALE)), int(round(pitch * _SCALE))
        if generator.random() < 0.15:
            built.heading(mag, roll_cd, pitch_cd)
            continue
        course_cd = angle_vetting.kept_near(int(round(heading * _SCALE)) % 36000) if tracking else None
        built.yaw(mag, course_cd, roll_cd, pitch_cd)
    return built


def build() -> dict:
    """
    The magnetic_yaw fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, measured, scenarios}.

    Raises:
        ValueError - plan B and the law disagree on `known` or a heading's presence, or stray past BUDGET.
        angle_vetting.NearThreshold - a scripted scenario sits on a threshold.
    """
    scenarios = _explicit()
    generator = random.Random(_SEED)
    redrawn = 0
    for index in range(_SEEDED_RUNS):
        while True:
            try:
                scenarios.append(_seeded(generator, 'seeded_%02d' % index))
                break
            except angle_vetting.NearThreshold:
                redrawn += 1
    return {
        'meta': {'source': 'src/glider/tasks/attitude.py, src/glider/fixed.py'},
        'compare': {'exact': {'kind': 'exact'}, 'degrees': {'kind': 'angle_deg', 'tol': angle_vetting.TOLERANCE}},
        'note': ('a call per step; inputs in plan B units: mag counts, roll_cd/pitch_cd/course_cd / 100 degrees; '
                 'known exact, angles in degrees within 0.1 the short way around'),
        'measured': {'worst_plan_b_against_the_law_degrees': round(max(scenario.worst for scenario in scenarios), 5),
                     'steps': sum(len(scenario.steps) for scenario in scenarios), 'seeded_runs_redrawn': redrawn},
        'scenarios': {'note': 'stateful: {name, note, init: {config, settings, yaw_cd}, steps: [{in, out}]}',
                      'cases': [scenario.document() for scenario in scenarios]},
    }
