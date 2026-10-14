"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture guidance.json: plan B's stage-dependent guidance law (src/glider/guidance.py) as the Rust port
(crates/control, module guidance) must answer it -- GuidanceConfig, heading_error, the Heading patterns, the zone
geometry the law reads (mission.zone_points), the steering filter, the turn-radius and endgame-bank physics,
reachability, and the law itself over stateful scenarios. The Mission's own rule for the pattern 'auto' resolves
to (mission.endgame_heading) is the mission crate's, so it is pinned in mission.json; a scenario's init carries
plan B's answer for its zone, `endgame_heading`, as the Mission hands it to the Rust Landing.

Plan B's Guidance reads injected handles: the Mission, the governor, the wind estimator and databoard Parameters.
Here they are stand-ins set per step from the fixture's inputs, so the Rust test hands the same answers in as its
`Inputs`: a position, agl or elevation is null when no fresh source vouches for it (plan B reads it with source
None), the airspeed is the governor's, the wind the estimator's components. The Mission is plan B's own class
with its zone and launch point set directly, so zone_points(), endgame_heading() and geometry() are plan B's code
(geometry() needs a launch point, which is why reachability does). Reachability's wind and airspeed, plan B's
arguments, are the Rust `Inputs` the same values ride in (plan B's only caller passes the estimator's and the
governor's). Positions are 1e-7 degree units, written as
integers; plan B is called with units / 1e7. Zones have even corner sums, so plan B's centre is a whole unit and
the Rust integer centre is the same point. Time is the shims' tick space: every now_us stays far below the 2**29
microseconds where plan B's tick difference would wrap.

Units: plan B holds the setpoints as centidegree fixnums (fixed.from_float, truncating toward zero), the
fixture writes them in degrees (fixnum / 100), and the Rust port keeps them as f32 degrees. The heading error is
integer degrees on both sides.

Every output is grouped by how it compares (`compare`, and the `tolerance` text):
  * exact -- integers, flags and the reach dict (its metres are rounded to whole ones and its headwind to 0.1,
    as plan B reports them): the heading error, active, reckoning, pattern ids, filter outputs.
  * degrees -- angle_deg 0.05, the plan's navigation angle: the target heading the law steers to. f32 trig on
    the same integer positions is good to ~1e-4 degree; a dead-reckoned position is a whole unit in Rust (up
    to 8 mm from plan B's), which no case lets move the heading by more than ~0.02 degree (below).
  * setpoint -- abs 0.011 degree: the roll and pitch setpoints and the endgame bank. Plan B truncates a setpoint
    to the centidegree (less than 0.01 below the Rust f32); the rest is f32 rounding (~1e-5).
  * metres -- abs 0.01 m: the dead-reckoned offset and the turn radii. f32 holds them to ~1e-6 of themselves
    (radii stay under ~1 km: banks of 5 degrees or more), and a dead reckoning integrates a few hundred steps.
  * config -- rel 1e-6: a config number, which f32 holds to 6e-8 of itself.

Cases stay off the decision thresholds, where one ulp could flip a discrete answer: the integer truncation of
the heading error, the 0/360 wrap of the target heading, the nearer-gate tie, the loiter capture radius, the 'oo'
lobe radius, the 'ov' leg ends, the final-approach agl, the endgame elevation, the stall-margin airspeed, the
boost engage speed, the reach comparison and its roundings. Rather than re-derive each one, every step is re-run
by plan B from the SAME state with its inputs jittered (heading +/-0.01 degree, positions +/-2 units, agl and
elevation +/-0.01 m, airspeed and wind +/-0.001 m/s, the dead-reckoned offset +/-0.03 m), far more than the
f32 answer can differ. A step whose discrete outcome moves under the jitter (the heading error, the flags, the
lobe, the filter state, the cache), whose target heading sits within 0.02 degree of the 0/360 wrap, or whose
dead-reckoned target heading moves by more than 0.1 degree (a waypoint within ~24 m, where the 8 mm the Rust
whole-unit position may differ by starts to show), is on a threshold: an explicit scenario raises (its
inputs are then moved, a heading of 0.5 rather than 0 for example), and a seeded flight nudges that step's inputs
and tries again (after ten tries with the GNSS fix back), or ends there.

Some thresholds are pinned ON purpose all the same, where both sides compute the very same number: the clock and
a fix's age are integers the jitter never moves (the dead-reckoning step ceiling, the freshness gate), the
steering filter is all-integer (its 90 degree jump), and reachability's half-to-even roundings are reached with
inputs exact in f32 as in f64 (a fix ON the zone's centre, a quarter m/s of wind straight along the bearing):
those cases say so, and the jitter there must move nothing but the tie itself. The comparisons on float
thresholds (the boost engage speed, the final-approach agl, the endgame band, the stall floor) are pinned by the
Rust unit tests, at values f32 holds exactly.

The seeded flights are closed loops: a point-mass glider (a coordinated turn at the commanded roll, a constant
glide ratio, a steady wind, GNSS noise and dropouts, a short laser range) flown by plan B's own law. Only the
inputs and plan B's answers are written; the Rust test replays the inputs open-loop.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import copy
import math
import random
import struct

import commons
import config_default
import controller
import fixed
import guidance
import mission
import navigation

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_UNITS_PER_DEGREE: float = 1e7  # a Rust Position counts 1e-7 degrees
_WINDOW_MS: int = 1000  # the GNSS channels' databoard window the flight task hands GuidanceConfig
_MICROSECONDS_PER_SECOND: int = 1_000_000
_STAGES: dict = controller.Stage.NAMES  # stage name -> plan B id

_COMPARE: dict = {
    'exact': {'kind': 'exact'},
    'degrees': {'kind': 'angle_deg', 'tol': 0.05},
    'setpoint': {'kind': 'abs', 'tol': 0.011},
    'metres': {'kind': 'abs', 'tol': 0.01},
    'config': {'kind': 'rel', 'tol': 1e-06},
}

"""The robustness re-runs (see the module notes): how far each input is jittered, and how many re-runs a step gets."""
_JITTER_HEADING: float = 0.01
_JITTER_UNITS: int = 2
_JITTER_METRES: float = 0.01
_JITTER_SPEED: float = 0.001
_JITTER_RECKONED: float = 0.03
_JITTER_RUNS: int = 8
_WRAP_MARGIN: float = 0.02  # a target heading this close to 0/360 is on the wrap
"""
A dead-reckoned target heading that moves more than this (degrees) under the 3 cm jitter of the reckoned offset
points at a waypoint within ~24 m. The Rust reckoned position is a whole unit, at most 8 mm from plan B's float
one: a jitter five times that moving the heading by 0.1 degree bounds the Rust difference to ~0.02 degree.
"""
_RECKONED_STABLE: float = 0.1

"""test_guidance's zone: 0.001 degree tall by 0.010 wide near 48 N, so gates on the left and right edges."""
_ZONE: tuple = ((480_010_000, 110_000_000), (480_000_000, 110_100_000))
_CENTRE: tuple = (480_005_000, 110_050_000)
_EAST_OF_ZONE: tuple = (480_005_000, 110_200_000)
_WEST_OF_ZONE: tuple = (480_005_000, 109_900_000)
_SQUARE: tuple = ((480_010_000, 110_000_000), (480_000_000, 110_015_000))  # aspect ~1 -> 'o'


def _degrees(position: tuple) -> tuple:
    """Plan B's float (latitude, longitude) for a position in units."""
    return position[0] / _UNITS_PER_DEGREE, position[1] / _UNITS_PER_DEGREE


def _metres_moved(position: tuple, east_m: float, north_m: float) -> tuple:
    """A position moved by a metre offset, in whole units (input generation only)."""
    latitude = position[0] + round(north_m / commons.M_PER_DEG * _UNITS_PER_DEGREE)
    scale = commons.M_PER_DEG * math.cos(math.radians(position[0] / _UNITS_PER_DEGREE))
    return latitude, position[1] + round(east_m / scale * _UNITS_PER_DEGREE)


class _Handle:
    """A databoard Parameter's read() stand-in: (value, source, age_ms)."""

    def __init__(self) -> None:
        self.reading = (None, None, None)

    def read(self) -> tuple:
        return self.reading


class _Governor:
    """The governor's airspeed() stand-in."""

    def __init__(self) -> None:
        self.speed = 0.0

    def airspeed(self) -> float:
        return self.speed


class _Wind:
    """The wind estimator's components() stand-in."""

    def __init__(self) -> None:
        self.parts = (0.0, 0.0)

    def components(self) -> tuple:
        return self.parts


class _Mission(mission.Mission):
    """
    Plan B's Mission with its zone and launch point set directly.

    zone_points(), zone_aspect(), endgame_heading() and geometry() are plan B's own; only the constructor (which
    reads launch.config and registers with the Inspector) and launch_point() (which reads the databoard) are
    stood in.
    """

    def __init__(self, zone: tuple | None) -> None:
        self.zone = None if zone is None else (_degrees(zone[0]), _degrees(zone[1]))
        self.max_range_m = 1.0e9  # geometry()'s range gate plays no part in guidance
        self._launch = None
        self._zone_key = None
        self._zone_points = None
        self._zone_aspect = 1.0

    def launch_point(self) -> tuple | None:
        return self._launch


class _Harness:
    """One plan B Guidance and the stand-ins it reads."""

    def __init__(self, config: dict, zone: tuple | None) -> None:
        self.position = _Handle()
        self.agl = _Handle()
        self.elevation = _Handle()
        self.governor = _Governor()
        self.wind = _Wind()
        self.mission = _Mission(zone)
        self.unit = guidance.Guidance(guidance.GuidanceConfig(config, _WINDOW_MS), self.mission, self.governor,
                                      self.position, self.agl, self.elevation, self.wind)

    def set_inputs(self, step: dict) -> None:
        """Point the stand-ins at a step's inputs (see the module notes for what null means)."""
        fix = step.get('position')
        self.position.reading = ((None, None, None) if fix is None
                                 else (_degrees(fix['position']), 'gnss', fix['age_ms']))
        agl = step.get('agl')
        self.agl.reading = [agl, 'laser' if agl is not None else None, 0]
        elevation = step.get('elevation')
        self.elevation.reading = [elevation, 'baro' if elevation is not None else None, 0]
        self.governor.speed = step.get('airspeed', 0.0)
        self.wind.parts = tuple(step.get('wind', (0.0, 0.0)))
        launch = step.get('launch')
        self.mission._launch = None if launch is None else _degrees(launch)


def _clone(unit: guidance.Guidance) -> guidance.Guidance:
    """
    A copy of a plan B Guidance that runs on its own: every state attribute copied, the handles shared.

    The law table holds methods bound to the original, so the copy gets its own.
    """
    twin = copy.copy(unit)
    twin._laws = {stage: getattr(twin, law.__name__) for stage, law in unit._laws.items()}
    return twin


def _run(harness: _Harness, unit: guidance.Guidance, step: dict) -> object:
    """
    Run one step's action on `unit` under the step's inputs.

    Returns:
        compute's verdict (bool); None for an enter.
    """
    harness.set_inputs(step)
    if step['action'] == 'enter':
        unit.enter(step['heading'], step['roll_centidegrees'], step['pitch_centidegrees'])
        return None
    stage = _STAGES[step['stage']]
    setpoint = unit.setpoint(stage)
    if setpoint is None and stage != _STAGES['boosting']:
        raise ValueError('compute %s: not a control stage of this config' % step['stage'])
    return unit.compute(stage, setpoint, step['heading'], step['now_us'])


def _signature(unit: guidance.Guidance, verdict: object) -> tuple:
    """A step's discrete outcome: what must not move under the jitter."""
    return (verdict, unit.heading_error, unit.reckoning, unit._leg_dir, unit._was_near, unit._error_filtered16,
            unit._nav_heading is None)


def _jittered(step: dict, generator: random.Random, direction: int = 0) -> dict:
    """
    The step's inputs, each moved by its jitter (see the module notes).

    Args:
        step - the inputs.
        generator - the coin for each input's direction.
        direction - +1 or -1 moves every input that way; 0 tosses a coin per input.
    """
    sign = lambda: direction or generator.choice((-1, 1))  # noqa: E731 -- one coin per input
    moved = dict(step)
    if 'heading' in step:
        moved['heading'] = step['heading'] + sign() * _JITTER_HEADING
    if step.get('position') is not None:
        position = step['position']['position']
        moved['position'] = dict(step['position'], position=[position[0] + sign() * _JITTER_UNITS,
                                                               position[1] + sign() * _JITTER_UNITS])
    if step.get('launch') is not None:
        moved['launch'] = [step['launch'][0] + sign() * _JITTER_UNITS, step['launch'][1] + sign() * _JITTER_UNITS]
    for key in ('agl', 'elevation'):
        if step.get(key) is not None:
            moved[key] = step[key] + sign() * _JITTER_METRES
    if step.get('airspeed'):  # an airspeed of exactly 0 is exact on both sides: the 'no airspeed' branches
        moved['airspeed'] = step['airspeed'] + sign() * _JITTER_SPEED
    if 'wind' in step:
        moved['wind'] = [part + sign() * _JITTER_SPEED for part in step['wind']]
    return moved


def _turn(first: float, second: float) -> float:
    """How far apart two headings are, the short way around, in degrees."""
    around = (first - second) % 360.0
    return min(around, 360.0 - around)


def _robust(harness: _Harness, step: dict, generator: random.Random) -> str:
    """
    Whether a step sits clear of every decision threshold (see the module notes).

    Runs the step on copies of the current state: once as given, then _JITTER_RUNS times jittered (the
    dead-reckoned offset included).

    Returns:
        '' when it is clear; else what moved.
    """
    nominal = _clone(harness.unit)
    reference = _signature(nominal, _run(harness, nominal, step))
    target = nominal._nav_heading
    on_the_wrap = target is not None and min(target % 360.0, 360.0 - target % 360.0) < _WRAP_MARGIN
    if step['action'] == 'compute' and on_the_wrap:
        return 'the target heading %r sits on the 0/360 wrap' % target
    for run in range(_JITTER_RUNS):
        direction = (1, -1)[run] if run < 2 else 0  # every input up, every input down, then mixed
        twin = _clone(harness.unit)
        twin._reckon_east += (direction or generator.choice((-1, 1))) * _JITTER_RECKONED
        twin._reckon_north += (direction or generator.choice((-1, 1))) * _JITTER_RECKONED
        signature = _signature(twin, _run(harness, twin, _jittered(step, generator, direction)))
        if signature != reference:
            return 'the jitter moves %r to %r' % (reference, signature)
        if nominal.reckoning and target is not None and twin._nav_heading is not None and \
                _turn(twin._nav_heading, target) > _RECKONED_STABLE:
            return 'the dead-reckoned target heading moves %.4f degree' % _turn(twin._nav_heading, target)
    return ''


def _outputs(unit: guidance.Guidance, verdict: object) -> dict:
    """The step's answer, grouped: the Rust test builds the same from its Guidance."""
    exact = {'heading_error': unit.heading_error, 'reckoning': unit.reckoning}
    if verdict is not None:
        exact['active'] = verdict
    return {'exact': exact,
            'setpoint': {'roll': unit.roll_setpoint / fixed.SCALE, 'pitch': unit.pitch_setpoint / fixed.SCALE},
            'degrees': {'target_heading': unit._nav_heading},
            'metres': {'reckoned_offset': [unit._reckon_east, unit._reckon_north]}}


def _record(harness: _Harness, step: dict) -> dict:
    """Run one step for real, one already found clear of the thresholds, and record it."""
    verdict = _run(harness, harness.unit, step)
    return {'in': step, 'out': _outputs(harness.unit, verdict)}


def _step(harness: _Harness, step: dict, generator: random.Random) -> dict:
    """
    Check one step against the thresholds, then run it for real and record it.

    Raises:
        ValueError - the step sits on a threshold.
    """
    trouble = _robust(harness, step, generator)
    if trouble:
        raise ValueError(trouble)
    return _record(harness, step)


def _enter(heading: float, roll_centidegrees: int = 0, pitch_centidegrees: int = 0) -> dict:
    """An enter step: the heading to hold blind and the rod-vertical attitude in centidegrees, as plan B's flight
    task hands them over."""
    return {'action': 'enter', 'heading': heading, 'roll_centidegrees': roll_centidegrees,
            'pitch_centidegrees': pitch_centidegrees}


def _compute(stage: str, heading: float, now_us: int, position: tuple | None = None, age_ms: int = 0,
             agl: float | None = None, elevation: float | None = None, airspeed: float = 0.0,
             wind: tuple = (0.0, 0.0), launch: tuple | None = None) -> dict:
    """A compute step and the inputs it runs under; an input no source vouches for is left out (null)."""
    step = {'action': 'compute', 'stage': stage, 'heading': heading, 'now_us': now_us, 'airspeed': airspeed,
            'wind': list(wind)}
    if position is not None:
        step['position'] = {'position': list(position), 'age_ms': age_ms}
    if agl is not None:
        step['agl'] = agl
    if elevation is not None:
        step['elevation'] = elevation
    if launch is not None:
        step['launch'] = list(launch)
    return step


def _scenario(name: str, config: dict, zone: tuple | None, steps: list, generator: random.Random) -> dict:
    """
    One explicit scenario on a fresh Guidance.

    Raises:
        ValueError - a step sits on a threshold (the scenario's inputs must move off it).
    """
    harness = _Harness(config, zone)
    recorded = []
    for index, step in enumerate(steps):
        try:
            recorded.append(_step(harness, step, generator))
        except ValueError as error:
            raise ValueError('scenario %s step %d: %s' % (name, index, error)) from None
    return {'name': name, 'init': _init(harness, config, zone), 'steps': recorded}


def _init(harness: _Harness, config: dict, zone: tuple | None) -> dict:
    """
    A scenario's init: the config, the zone in units, and the pattern plan B's Mission resolves 'auto' to for that
    zone (null without one): the Mission's answer, which the Rust caller hands guidance in its Landing.
    """
    return {'config': config, 'zone': None if zone is None else [list(zone[0]), list(zone[1])],
            'endgame_heading': None if zone is None else harness.mission.endgame_heading()}


def _south_of_centre(metres: float) -> tuple:
    """The _ZONE centre moved `metres` due south, in whole units."""
    return _CENTRE[0] - round(metres / commons.M_PER_DEG * _UNITS_PER_DEGREE), _CENTRE[1]


def _oval_leg(metres: float) -> tuple:
    """A point `metres` along _ZONE's long axis (east positive, toward gate_b), 5.6 m north of the centreline."""
    return _metres_moved((_CENTRE[0] + 500, _CENTRE[1]), metres, 0.0)


def _explicit_scenarios(generator: random.Random) -> list:
    """
    test_guidance.py's scenarios, as steps.

    Where the plan B test pokes private state (`_nav_heading = None` to drop the cache, `position_age_max_ms = 0`,
    `_leg_dir = 1`), a scenario reaches the same state through the public surface: time past the navigation
    period, a fix older than the gate, enter(). Headings of 0 become 0.5 and the like, off the integer the heading
    error truncates at; the behaviour each test asserts is unchanged.
    """
    period = 100_000  # the default navigation period, microseconds
    scenarios = []
    scenarios.append(_scenario('heading_tiers_blind', {}, _ZONE, [
        _enter(200.0),
        _compute('gliding', 0.5, 0),  # tier 4: no fix, no launch point -> the captured heading
    ], generator))
    scenarios.append(_scenario('heading_tiers', {}, _ZONE, [
        _enter(200.0),
        _compute('gliding', 0.5, 0, launch=_WEST_OF_ZONE),  # tier 3: the launch point -> the left gate, ~90
        _compute('gliding', 0.5, period, position=_EAST_OF_ZONE, launch=_WEST_OF_ZONE),  # tier 1: ~270
        # a fix older than the gate: dead reckoning from the fix just seen (east, ~270), not the launch point
        _compute('gliding', 0.5, 2 * period, position=_EAST_OF_ZONE, age_ms=1500, launch=_WEST_OF_ZONE),
        _compute('gliding', 0.5, 3 * period, launch=_WEST_OF_ZONE),
        # the freshness gate is strict (`age < position_age_max_ms`), and an age is an integer no jitter moves: a
        # fix 999 ms old steers (tier 1), one exactly the 1000 ms window old is already reckoned past (tier 2)
        _compute('gliding', 0.5, 4 * period, position=_EAST_OF_ZONE, age_ms=_WINDOW_MS - 1, launch=_WEST_OF_ZONE),
        _compute('gliding', 0.5, 5 * period, position=_EAST_OF_ZONE, age_ms=_WINDOW_MS, launch=_WEST_OF_ZONE),
    ], generator))
    scenarios.append(_scenario('no_zone_holds_the_captured_heading', {}, None, [
        _enter(200.0),
        _compute('gliding', 0.5, 0, position=_EAST_OF_ZONE),
        _compute('gliding', 0.5, period, position=_EAST_OF_ZONE),
    ], generator))
    scenarios.append(_scenario('navigation_cache', {'nav_period_ms': 100}, _ZONE, [
        _enter(200.0),
        _compute('gliding', 0.5, 0, position=_EAST_OF_ZONE),  # ~270
        _compute('gliding', 0.5, 50_000, position=(480_005_000, 109_800_000)),  # 50 ms: cached, still ~270
        _compute('gliding', 0.5, 150_000, position=(480_005_000, 109_800_000)),  # past the period: ~90
        _enter(200.0),  # entering control drops the cache outright
        _compute('gliding', 0.5, 160_000, position=(480_005_000, 109_800_000)),
    ], generator))
    scenarios.append(_scenario('bank_to_turn', {'nav_bank_gain': 1.5, 'bank_limit': 30,
                                                'stages': {'gliding': {}, 'landing': {}}}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_WEST_OF_ZONE),  # +89: bank_demand hits the 30 cap
        _compute('landing', 0.5, period, position=_WEST_OF_ZONE),  # LANDING: the land gains, the 45 cap
    ], generator))
    scenarios.append(_scenario('rudder_only_keeps_the_configured_roll', {'nav_bank_gain': 0,
                                                                         'stages': {'gliding': {'roll': 2.0}}},
                               _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_WEST_OF_ZONE),
    ], generator))
    scenarios.append(_scenario('final_approach', {'final_approach_agl': 8, 'nav_bank_gain': 1.5,
                                                  'land_bank_gain': 1.5, 'land_bank_limit': 45}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 270.5, 0, position=_EAST_OF_ZONE, agl=20.0),  # high: homing
        _compute('gliding', 270.5, period, position=_EAST_OF_ZONE, agl=4.0),  # low: the centreline
        _compute('gliding', 270.5, 2 * period, position=_EAST_OF_ZONE),  # a stale agl never engages it
        _compute('gliding', 275.5, 3 * period, position=(480_003_000, 110_080_000), agl=3.0),
        _compute('gliding', 275.5, 4 * period, position=(480_003_000, 110_080_000), agl=8.5),  # just above: homing
        _compute('gliding', 275.5, 5 * period, position=(480_003_000, 110_080_000), agl=7.5),  # just below: final
    ], generator))
    scenarios.append(_scenario('final_approach_off', {'final_approach_agl': 0}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 270.5, 0, position=_EAST_OF_ZONE, agl=1.0),
    ], generator))
    """
    Plan B stores a knob as given and gates the feature on its TRUTH, so a present null or false switches it OFF:
    it is not a missing key, whose default would switch it on. Each step tells the two apart: low on the laser
    and in the endgame band, final approach would track the centreline (270) where the endgame lobes say 300; a
    null bank gain holds the configured roll, not a 45 degree bank; a 0.0 shift passes the raw 69 through, where
    the default filter answers 86 (and a shift of true, which flies as 1, answers 79).
    """
    for value in (None, False):
        scenarios.append(_scenario('final_approach_agl_%s_is_off' % str(value).lower(), {'final_approach_agl': value},
                                   _ZONE, [
            _enter(0.5),
            _compute('gliding', 270.5, 0, position=_EAST_OF_ZONE, agl=4.0, elevation=10.0, airspeed=14.0),
        ], generator))
    scenarios.append(_scenario('nav_bank_gain_null_is_rudder_only', {'nav_bank_gain': None,
                                                                     'stages': {'gliding': {'roll': 2.0}}}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_WEST_OF_ZONE),
    ], generator))
    scenarios.append(_scenario('land_bank_gain_null_is_off', {'land_bank_gain': None,
                                                              'stages': {'gliding': {}, 'landing': {'roll': -1.5}}},
                               _ZONE, [
        _enter(0.5),
        _compute('landing', 0.5, 0, position=_WEST_OF_ZONE),
    ], generator))
    scenarios.append(_scenario('endgame_alt_m_null_is_off', {'endgame_alt_m': None}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_EAST_OF_ZONE, elevation=10.0, airspeed=14.0),
    ], generator))
    # a true agl gate flies as 1 m: at 4 m the endgame lobes hold (300), where the 8 m default would track (270)
    scenarios.append(_scenario('final_approach_agl_true_is_one', {'final_approach_agl': True}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 270.5, 0, position=_EAST_OF_ZONE, agl=4.0, elevation=10.0, airspeed=14.0),
    ], generator))
    for name, shift in (('steer_filter_shift_float_zero_is_off', 0.0), ('steer_filter_shift_null_is_off', None),
                        ('steer_filter_shift_true_is_one', True)):
        scenarios.append(_scenario(name, {'steer_filter_shift': shift}, _ZONE, [
            _enter(0.5),
            _compute('gliding', 0.5, 0, position=_WEST_OF_ZONE),  # seeds at 89
            _compute('gliding', 20.5, period, position=_WEST_OF_ZONE),  # a raw 69
        ], generator))
    scenarios.append(_scenario('a_stale_baro_never_opens_the_endgame', {'endgame_alt_m': 50, 'land_bank_gain': 3.0},
                               _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_EAST_OF_ZONE),  # stale baro: ordinary steering, a real bank
        _compute('gliding', 0.5, period, position=_EAST_OF_ZONE, elevation=10.0),  # fresh: endgame, no airspeed
    ], generator))
    loiter = {'loiter_radius_m': 30, 'loiter_capture_m': 120, 'loiter_gain': 3.0, 'endgame_alt_m': 50,
              'endgame_pattern': 'o', 'nav_bank_gain': 1.5, 'land_bank_gain': 3.0}
    scenarios.append(_scenario('loiter_and_endgame_spiral', loiter, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=_south_of_centre(60), elevation=200.0),  # the tangent, cut capped
        _compute('gliding', 0.5, period, position=_south_of_centre(30), elevation=200.0),  # on the circle
        _compute('gliding', 0.5, 2 * period, position=_south_of_centre(30), elevation=25.0),  # endgame: R 15
        # far: the gates (a little east of the centre's meridian, off the tie where both gates are as near)
        _compute('gliding', 0.5, 3 * period, position=(_south_of_centre(500)[0], _CENTRE[1] + 2000), elevation=200.0),
        # either side of the 120 m capture radius (a little off the centre's meridian, clear of the gate tie)
        _compute('gliding', 0.5, 4 * period, position=(_south_of_centre(122)[0], _CENTRE[1] + 3000), elevation=200.0),
        _compute('gliding', 0.5, 5 * period, position=(_south_of_centre(118)[0], _CENTRE[1] + 3000), elevation=200.0),
        _compute('gliding', 40.5, 6 * period, position=_south_of_centre(45), elevation=30.0, airspeed=14.0),
        _compute('gliding', 40.5, 7 * period, position=_south_of_centre(45), elevation=30.0, airspeed=9.0),
    ], generator))
    two_lobes = {'loiter_radius_m': 30, 'loiter_capture_m': 120, 'loiter_gain': 3.0, 'endgame_alt_m': 50,
                 'land_bank_gain': 3.0}
    metres_east = 60.0 / (commons.M_PER_DEG * math.cos(math.radians(_CENTRE[0] / _UNITS_PER_DEGREE)))
    east_of_centre = (_CENTRE[0], _CENTRE[1] + round(metres_east * _UNITS_PER_DEGREE))
    near_centre = (_CENTRE[0] + 300, _CENTRE[1] + 200)  # 3.6 m from the centre, off the exact 0/360 answer
    scenarios.append(_scenario('two_lobes_flip_once_per_crossing', two_lobes, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=near_centre, elevation=25.0, airspeed=14.0),  # a crossing flips
        _compute('gliding', 0.5, period, position=near_centre, elevation=25.0, airspeed=14.0),  # no re-flip
        _compute('gliding', 0.5, 2 * period, position=east_of_centre, elevation=25.0, airspeed=14.0),
        _compute('gliding', 0.5, 3 * period, position=near_centre, elevation=25.0, airspeed=14.0),  # flips back
        _compute('gliding', 0.5, 4 * period, position=east_of_centre, elevation=25.0, airspeed=14.0),
        _compute('gliding', 0.5, 5 * period, position=near_centre, elevation=25.0, airspeed=14.0),  # to gate_a
        # a re-entry on the gate_a lobe with the near-latch set starts on gate_b again, unlatched: out and back
        # near, the crossing flips to gate_a as the first one did (not back to gate_b, as a kept lobe would)
        _enter(0.5),
        _compute('gliding', 0.5, 6 * period, position=east_of_centre, elevation=25.0, airspeed=14.0),
        _compute('gliding', 0.5, 7 * period, position=near_centre, elevation=25.0, airspeed=14.0),
    ], generator))
    oval = dict(two_lobes, endgame_pattern='ov', final_cross_gain=2.0, final_intercept_deg=40)
    scenarios.append(_scenario('oval_reverses_at_the_leg_ends', oval, _ZONE, [
        _enter(90.5),
        _compute('gliding', 90.5, 0, position=(480_004_000, 110_060_000), elevation=40.0, airspeed=14.0),
        _compute('gliding', 90.5, period, position=(480_005_500, 110_092_000), elevation=40.0, airspeed=14.0),
        _compute('gliding', 270.5, 2 * period, position=(480_005_500, 110_010_000), elevation=10.0,
                 airspeed=14.0),
        _compute('gliding', 270.5, 3 * period, position=(480_005_500, 110_040_000), elevation=2.0, airspeed=14.0),
        # low in the band the legs shrink only to the 15 % floor: 65 m along reverses there (and would not at 20 %)
        _compute('gliding', 90.5, 4 * period, position=(480_005_500, 110_058_720), elevation=2.0, airspeed=14.0),
        _compute('gliding', 270.5, 5 * period, position=(480_005_500, 110_041_280), elevation=1.5, airspeed=14.0),
        # the floor itself: 0.15 x the 372.4 m half-length is 55.9 m (0.14 would be 52.1, 0.16 59.6), so 54 m
        # along runs on and 57.5 m reverses, either way round; the +/-2 unit jitter is 1.5 cm
        _compute('gliding', 90.5, 6 * period, position=_oval_leg(54.0), elevation=2.0, airspeed=14.0),
        _compute('gliding', 90.5, 7 * period, position=_oval_leg(57.5), elevation=2.0, airspeed=14.0),
        _compute('gliding', 270.5, 8 * period, position=_oval_leg(-54.0), elevation=2.0, airspeed=14.0),
        _compute('gliding', 270.5, 9 * period, position=_oval_leg(-57.5), elevation=2.0, airspeed=14.0),
        # a re-entry after a reversal starts the leg toward gate_b again: 54 m along does not reverse it
        _compute('gliding', 90.5, 10 * period, position=_oval_leg(57.5), elevation=2.0, airspeed=14.0),
        _enter(90.5),
        _compute('gliding', 90.5, 11 * period, position=_oval_leg(54.0), elevation=2.0, airspeed=14.0),
    ], generator))
    scenarios.append(_scenario('auto_flies_a_square_zone_round', {'endgame_alt_m': 50}, _SQUARE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, position=(479_998_000, 110_007_000), elevation=20.0, airspeed=15.0),
    ], generator))
    scenarios.append(_scenario('hold_law', {'stages': {'setting': {'roll': 1.0, 'pitch': -2.0}}}, _ZONE, [
        _enter(100.25),
        _compute('setting', 90.0, 0, position=_EAST_OF_ZONE),  # a live fix must not steer this law: 10
    ], generator))
    scenarios.append(_scenario('boost_hold', {'stages': {'boosting': {}, 'gliding': {}}, 'boost_engage_speed': 15.0},
                               _ZONE, [
        _enter(0.5, 1250, 9000),  # the rod holds the airframe 12.5 degrees rolled
        _compute('boosting', 0.5, 0, airspeed=5.0),  # still on the rod: neutral, the slots untouched
        _compute('boosting', 0.5, 10_000, airspeed=30.0),  # past the rod: the captured attitude
        _compute('gliding', 0.5, 20_000, position=_WEST_OF_ZONE, airspeed=30.0),  # leaves roll 45 and error 89
        _compute('boosting', 0.5, 30_000, airspeed=5.0),  # neutral again: the gliding slots stay
        _compute('boosting', 0.5, 40_000, airspeed=30.0),  # past the rod: the CAPTURED roll back, error 0
    ], generator))
    pad = (480_100_000, 110_053_000)  # north of the zone, a little east of its centre: off the gate tie
    scenarios.append(_scenario('dead_reckoning', {}, _ZONE, [
        _enter(0.5),
        _compute('gliding', 0.5, 0, airspeed=20.0, launch=pad),  # no fix ever: the launch point
        _compute('gliding', 180.5, 1_000_000, position=pad, airspeed=20.0, launch=pad),  # a fix seeds
        _compute('gliding', 180.5, 1_200_000, airspeed=20.0, launch=pad),  # 0.2 s at 20 m/s south: 4 m
        _compute('gliding', 180.5, 1_400_000, airspeed=20.0, wind=(10.0, 0.0), launch=pad),  # the wind adds
        _compute('gliding', 180.5, 6_400_000, airspeed=20.0, wind=(10.0, 0.0), launch=pad),  # a 5 s gap: none
        _compute('gliding', 180.5, 6_300_000, airspeed=20.0, wind=(10.0, 0.0), launch=pad),  # earlier: none
        _compute('gliding', 180.5, 6_400_000, airspeed=20.0, wind=(10.0, 0.0), launch=pad),
        _compute('gliding', 200.75, 6_500_000, airspeed=20.0, wind=(-3.0, 4.0), launch=pad),
        _enter(200.75),  # a re-entry keeps the seed: still reckoning
        _compute('gliding', 200.75, 6_600_000, airspeed=20.0, wind=(-3.0, 4.0), launch=pad),
        _compute('gliding', 200.75, 6_700_000, position=pad, airspeed=20.0, launch=pad),  # a fix again
        # the step ceiling is half a second, exclusive; the clock is never jittered, so these steps are exact
        _compute('gliding', 180.5, 7_200_000, airspeed=20.0, launch=pad),  # exactly 500 ms after that fix: none
        _compute('gliding', 180.5, 7_650_000, airspeed=20.0, launch=pad),  # 450 ms: 9 m south
        _compute('gliding', 180.5, 8_650_000, airspeed=20.0, launch=pad),  # a whole second: none
    ], generator))
    slow = [_enter(0.5), _compute('gliding', 180.5, 0, position=pad, airspeed=2.0)]
    slow += [_compute('gliding', 180.5, step * 100_000, airspeed=2.0) for step in range(1, 51)]  # 10 m in 0.2 m steps
    scenarios.append(_scenario('dead_reckoning_keeps_slow_steps', {}, _ZONE, slow, generator))
    scenarios.append(_scenario('no_fix_no_launch_holds_the_captured_heading', {}, _ZONE, [
        _enter(77.25),
        _compute('gliding', 77.25, 0, airspeed=20.0),
    ], generator))
    return scenarios


"""
The seeded flights' physics (input generation only; see the module notes): a coordinated turn at the commanded
roll, the measured polar's glide ratio, a 1 m GNSS scatter and a +/-0.8 degree heading noise.
"""
_GRAVITY: float = 9.81
_GLIDE_RATIO: float = 5.5
_GNSS_SCATTER_M: float = 1.0
_HEADING_NOISE: float = 0.8
_REPAIRS: int = 40  # a step that keeps landing on a threshold after this many nudges ends its flight
_FLIGHT_STEPS: int = 240  # a flight's length cap: 24 s, enough for the endgame, final and landing
"""
The seeded flights: one per endgame pattern, as config names it, and whether it starts boosting off the rod. Three
of four do, each with its own random rod roll, so the boost hold restores a captured attitude that is not level.
"""
_FLIGHTS: tuple = (('o', True), ('ov', False), ('O-O', True), ('auto', True))


def _flight_config(generator: random.Random, pattern: str) -> dict:
    """The shipped flight config flying `pattern` in the endgame, with the knobs the law reads varied."""
    shipped = [component for component in config_default.default()['components'] if component['name'] == 'flight'][0]
    config = {key: value for key, value in shipped.items() if key not in ('gains', 'wind')}
    config.update({
        'endgame_pattern': pattern,
        'loiter_radius_m': generator.choice((25, 30, 40)),
        'loiter_capture_m': generator.choice((80, 120, 150)),
        'nav_period_ms': generator.choice((50, 100, 150)),
        'steer_filter_shift': generator.choice((0, 2, 3, 4)),
        'final_approach_agl': generator.choice((0, 4, 8)),
        'stall_speed_1g': generator.choice((0, 9.0, 9.0)),
        'nav_bank_gain': generator.choice((1.0, 1.5, 2.0)),
        'position_age_max_ms': generator.choice((400, 1000)),
    })
    return config


def _random_zone(generator: random.Random, centre: tuple) -> tuple:
    """A zone around `centre`, sides of 30 to 400 m, both corner sums even."""
    height = generator.uniform(30.0, 400.0)
    width = generator.uniform(30.0, 400.0)
    north_east = _metres_moved(centre, width / 2.0, height / 2.0)
    south_west = _metres_moved(centre, -width / 2.0, -height / 2.0)
    north_east = (north_east[0] + (north_east[0] + south_west[0]) % 2,
                  north_east[1] + (north_east[1] + south_west[1]) % 2)
    return (north_east[0], south_west[1]), (south_west[0], north_east[1])


def _nudged(step: dict, generator: random.Random) -> dict:
    """A seeded flight's step moved off a threshold: its heading, airspeed, elevation (and agl) and fix nudged."""
    step = dict(step, heading=round((step['heading'] + generator.uniform(-0.4, 0.4)) % 360.0, 4),
                airspeed=round(step['airspeed'] + generator.uniform(-0.3, 0.3), 3),
                elevation=round(step['elevation'] + generator.uniform(-0.3, 0.3), 3))
    if 'agl' in step:
        step['agl'] = step['elevation']
    if 'position' in step:
        position = step['position']['position']
        step['position'] = dict(step['position'], position=[position[0] + generator.randint(-6, 6),
                                                             position[1] + generator.randint(-6, 6)])
    return step


def _flight(index: int, pattern: str, boost: bool, generator: random.Random) -> dict:
    """
    One seeded closed-loop flight: plan B's law flies a point-mass glider into a random zone, holding `pattern` in
    the endgame (a config name: 'o', 'ov', 'O-O', 'auto'), from a boost off the rod when `boost`.

    Returns:
        The scenario: {name, init, steps}, the steps as recorded (a step on a threshold nudged off it).
    """
    centre = (generator.randint(-500_000_000, 500_000_000), generator.randint(-1_700_000_000, 1_700_000_000))
    zone = _random_zone(generator, centre)
    config = _flight_config(generator, pattern)
    if not boost:
        config['stages'] = {name: setpoint for name, setpoint in config['stages'].items() if name != 'boosting'}
    harness = _Harness(config, zone)
    zone_centre = ((zone[0][0] + zone[1][0]) // 2, (zone[0][1] + zone[1][1]) // 2)
    distance = generator.uniform(100.0, 300.0)
    bearing = math.radians(generator.uniform(0.0, 360.0))
    east, north = distance * math.sin(bearing), distance * math.cos(bearing)  # metres from the zone centre
    heading = generator.uniform(0.0, 360.0)
    airspeed = generator.uniform(13.0, 17.0)
    elevation = generator.uniform(40.0, 70.0)
    wind_speed, wind_toward = generator.uniform(0.0, 5.0), math.radians(generator.uniform(0.0, 360.0))
    wind = (wind_speed * math.sin(wind_toward), wind_speed * math.cos(wind_toward))
    estimate = (wind[0] + generator.uniform(-0.5, 0.5), wind[1] + generator.uniform(-0.5, 0.5))
    launch = _metres_moved(zone_centre, east, north) if generator.random() < 0.6 else None
    laser_range = generator.uniform(2.0, 6.0)
    step_s = 0.1  # 10 Hz: the GNSS rate; a 150 ms navigation period still caches every other step
    dropouts = []
    start = generator.uniform(0.0, 4.0)
    while start < 60.0:
        length = generator.uniform(1.0, 8.0)
        dropouts.append((start, start + length))
        start += length + generator.uniform(3.0, 15.0)
    if generator.random() < 0.5:
        dropouts.insert(0, (0.0, generator.uniform(0.5, 3.0)))  # no fix at all at the start: the launch point
    roll = 0.0
    now_us = 1_000_000
    stage = 'boosting' if boost else 'gliding'
    rod_roll = generator.randint(-1500, 1500)  # centidegrees: the rod never holds the airframe wings level
    steps = [_step(harness, _enter(round(heading, 3), rod_roll, 8000 if boost else 0), generator)]
    boost_left = generator.uniform(0.6, 1.5) if boost else 0.0
    seconds = 0.0
    while elevation > 0.3 and len(steps) < _FLIGHT_STEPS:
        seconds += step_s
        now_us += round(step_s * _MICROSECONDS_PER_SECOND)
        if stage == 'boosting':
            boost_left -= step_s
            speed = 5.0 + 25.0 * min(1.0, seconds / 1.2)
            if boost_left <= 0.0:
                stage = 'gliding'
        else:
            speed = airspeed
        if stage == 'gliding' and elevation < 3.0 and 'landing' in config['stages']:
            stage = 'landing'
        in_dropout = any(begin <= seconds < end for begin, end in dropouts)
        fix = None if in_dropout else _metres_moved(zone_centre, east + generator.gauss(0.0, _GNSS_SCATTER_M),
                                                    north + generator.gauss(0.0, _GNSS_SCATTER_M))
        age_ms = generator.choice((0, 50, 120, 250, 380, 450, 700, 900)) if fix is not None else 0
        measured = (heading + generator.uniform(-_HEADING_NOISE, _HEADING_NOISE)) % 360.0
        step = _compute(stage, round(measured, 4), now_us, fix, age_ms,
                        round(elevation, 3) if elevation < laser_range else None,
                        round(elevation, 3), round(speed, 3), (round(estimate[0], 3), round(estimate[1], 3)), launch)
        for attempt in range(_REPAIRS):
            if not _robust(harness, step, generator):
                break
            step = _nudged(step, generator)
            if attempt >= _REPAIRS // 4:
                # dead reckoning that stays ill-conditioned (circling within metres of its waypoint): the fix returns
                step['position'] = {'position': list(_metres_moved(zone_centre, east, north)), 'age_ms': 0}
        else:
            break  # stuck on a threshold: the flight ends here
        steps.append(_record(harness, step))
        active = steps[-1]['out']['exact']['active']
        roll = harness.unit.roll_setpoint / fixed.SCALE if active else 0.0
        if stage != 'boosting':
            heading = (heading + math.degrees(_GRAVITY * math.tan(math.radians(roll)) / airspeed) * step_s) % 360.0
            east += (airspeed * math.sin(math.radians(heading)) + wind[0]) * step_s
            north += (airspeed * math.cos(math.radians(heading)) + wind[1]) * step_s
            elevation -= airspeed / _GLIDE_RATIO / math.cos(math.radians(roll)) * step_s
    return {'name': 'flight_%02d' % index, 'init': _init(harness, config, zone), 'steps': steps}


def _config_case(name: str, config: dict, window_ms: int = _WINDOW_MS) -> dict:
    """One GuidanceConfig case: plan B's resolved knobs, grouped."""
    resolved = guidance.GuidanceConfig(config, window_ms)
    stage_names = controller.Stage.STAGES
    numbers = {key: float(getattr(resolved, key)) for key in (
        'bank_gain', 'bank_limit', 'land_bank_gain', 'land_bank_limit', 'endgame_alt_m', 'stall_speed_1g',
        'stall_margin', 'endgame_max_bank', 'loiter_radius_m', 'loiter_capture_m', 'loiter_gain', 'final_agl',
        'final_cross_gain', 'final_intercept', 'boost_engage')}
    return {'name': name, 'input': {'config': config, 'position_window_ms': window_ms},
            'output': {'config': numbers,
                       'setpoint': {'stages': {stage_names[stage]: [roll / fixed.SCALE, pitch / fixed.SCALE]
                                               for stage, (roll, pitch) in resolved.setpoints_fx.items()}},
                       'exact': {'endgame_pattern': resolved.endgame_pattern,
                                 'steer_filter_shift': resolved.steer_filter_shift,
                                 'nav_period_us': resolved.nav_period_us,
                                 'position_age_max_ms': resolved.position_age_max_ms}}}


def _config_cases() -> list:
    """GuidanceConfig over the empty config (every fallback), the shipped one, and the variations tests use."""
    shipped = [component for component in config_default.default()['components'] if component['name'] == 'flight'][0]
    return [
        _config_case('empty_takes_every_fallback', {}),
        _config_case('shipped', shipped),
        _config_case('shipped_with_a_gnss_window_of_400', shipped, 400),
        _config_case('bank_to_turn', {'nav_bank_gain': 1.5, 'bank_limit': 30,
                                      'stages': {'gliding': {}, 'landing': {}}}),
        _config_case('setpoints_truncate_to_the_centidegree', {'stages': {'gliding': {'roll': 2.005, 'pitch': -6.3},
                                                                           'setting': {'roll': 1, 'pitch': -2}}}),
        _config_case('unknown_stage_names_are_dropped', {'stages': {'cruise': {'roll': 5.0}, 'Gliding': {},
                                                                     'landing': {'pitch': -4.5}}}),
        _config_case('no_control_stage_at_all', {'stages': {}}),
        _config_case('pattern_upper_case', {'endgame_pattern': 'O'}),
        _config_case('pattern_o_dash_o', {'endgame_pattern': 'o-o'}),
        _config_case('pattern_oval', {'endgame_pattern': 'ov'}),
        _config_case('pattern_unknown_is_auto', {'endgame_pattern': 'figure-eight'}),
        _config_case('pattern_not_text_is_auto', {'endgame_pattern': 7}),
        _config_case('tighter_position_gate_and_period', {'position_age_max_ms': 250, 'nav_period_ms': 40,
                                                          'steer_filter_shift': 0}),
        _config_case('every_knob', {'nav_bank_gain': 2.5, 'bank_limit': 35.5, 'land_bank_gain': 4.0,
                                    'land_bank_limit': 50, 'endgame_alt_m': 65.0, 'stall_speed_1g': 8.5,
                                    'stall_margin': 1.3, 'endgame_max_bank': 55, 'loiter_radius_m': 35,
                                    'loiter_capture_m': 140, 'loiter_gain': 2.5, 'final_approach_agl': 6,
                                    'final_cross_gain': 2.0, 'final_intercept_deg': 35, 'boost_engage_speed': 18.5,
                                    'steer_filter_shift': 5, 'nav_period_ms': 200, 'endgame_pattern': 'oo'}),
    ]


def _heading_error_cases(generator: random.Random) -> list:
    """heading_error over test_guidance's cases, the wrap edges and a seeded sweep off the integer boundaries."""
    explicit = [('across_north', 10.0, 350.0), ('across_north_back', 350.0, 10.0), ('none', 100.0, 100.0),
                ('a_half_turn_keeps_its_sign', 180.0, 0.0), ('minus_a_half_turn', 0.0, 180.0),
                ('truncates_toward_zero', 10.75, 0.0), ('truncates_negative_toward_zero', 0.0, 10.75),
                ('just_past_a_half_turn', 0.0, 180.5), ('wraps_once', 0.0, 359.5), ('wraps_back', 359.5, 0.0),
                ('two_turns', 720.25, 0.0), ('one_and_a_half_turns', -540.5, 0.0)]
    cases = [{'name': name, 'input': {'target': target, 'current': current},
              'output': {'exact': guidance.heading_error(target, current)}} for name, target, current in explicit]
    while len(cases) < len(explicit) + 100:
        target, current = generator.uniform(0.0, 360.0), generator.uniform(-400.0, 760.0)
        fraction = abs(target - current) % 1.0
        if min(fraction, 1.0 - fraction) < _JITTER_HEADING:
            continue
        cases.append({'name': 'sweep_%03d' % (len(cases) - len(explicit)),
                      'input': {'target': target, 'current': current},
                      'output': {'exact': guidance.heading_error(target, current)}})
    return cases


def _heading_resolve_cases() -> list:
    """Heading.resolve over config names: case, dashes, and the unknown ones that fall back to auto."""
    names = ['auto', 'AUTO', 'o', 'O', 'oo', 'o-o', 'O-O', 'ov', 'OV', 'o-v', '-o-', 'o--o', 'a-u-t-o', '', 'circle',
             'ooo', 'figure-8', 'Ov']
    return [{'name': 'resolve_%s' % (name or 'empty'), 'input': {'name': name},
             'output': {'exact': {'id': guidance.Heading.resolve(name),
                                  'name': guidance.Heading.PATTERNS[guidance.Heading.resolve(name)]}}}
            for name in names]


def _landing_case(name: str, zone: tuple) -> dict:
    """
    One zone's resolved landing geometry, what Landing::new answers: plan B's Mission zone_points().

    The pattern the Mission resolves 'auto' to is the mission crate's rule, pinned in mission.json.
    """
    launch = _Mission(zone)
    target, gate_a, gate_b = launch.zone_points()
    whole = lambda point: [round(point[0] * _UNITS_PER_DEGREE), round(point[1] * _UNITS_PER_DEGREE)]  # noqa: E731
    return {'name': name, 'input': {'top_left': list(zone[0]), 'bottom_right': list(zone[1])},
            'output': {'exact': {'target': whole(target), 'gate_a': whole(gate_a), 'gate_b': whole(gate_b)}}}


def _landing_cases(generator: random.Random) -> list:
    """The landing geometry: test_guidance's strip and square, a wide and a tall strip, and a sweep."""
    cases = [_landing_case('the_test_strip_gates_left_and_right', _ZONE),
             _landing_case('a_square', _SQUARE),
             _landing_case('aspect_three', ((480_010_000, 110_000_000), (480_000_000, 110_044_800))),
             _landing_case('a_tall_strip_gates_top_and_bottom',
                           ((480_080_000, 110_000_000), (480_000_000, 110_010_000)))]
    while len(cases) < 4 + 50:
        centre = (generator.randint(-600_000_000, 600_000_000), generator.randint(-1_700_000_000, 1_700_000_000))
        zone = _random_zone(generator, centre)
        if abs(navigation.zone_aspect(_degrees(zone[0]), _degrees(zone[1])) - 1.0) < 1e-3:
            continue  # the orientation tie: a zone as tall as it is wide could gate either way in f32
        cases.append(_landing_case('sweep_%03d' % (len(cases) - 4), zone))
    return cases


def _filter_case(name: str, shift: int, errors: list) -> dict:
    """The steering filter over a sequence of raw errors, from a fresh seed."""
    unit = _Harness({'steer_filter_shift': shift}, _ZONE).unit
    unit.enter(0.0, 0, 0)
    return {'name': name, 'input': {'shift': shift, 'errors': errors},
            'output': {'exact': [unit._filter_error(error) for error in errors]}}


def _filter_cases(generator: random.Random) -> list:
    """test_guidance's filter cases (seed, smoothing, the jump, the off switch, the symmetric rounding) and sweeps."""
    cases = [_filter_case('seeds_smooths_and_follows_a_jump', 3, [40, 48] + [48] * 60 + [-120, -118, 60]),
             _filter_case('shift_zero_passes_through', 0, [40, 48, -170, 3]),
             _filter_case('a_symmetric_oscillation_has_no_bias', 3,
                          [8 if step % 2 == 0 else -8 for step in range(600)])]
    for constant in (1, -1, 5, -5, 12, -12, 45, -45):
        cases.append(_filter_case('a_constant_%+d_settles_on_itself' % constant, 3, [0] + [constant] * 80))
    cases.append(_filter_case('a_huge_shift', 40, [10, 30, -20, 100, 95]))
    # the re-seed takes a jump OVER 90 degrees: exactly 90 (1440 sixteenths) is smoothed, 91 followed at once
    for jump in (90, 91, -90, -91):
        cases.append(_filter_case('a_jump_of_%+d' % jump, 3, [0, jump]))
    for index in range(8):
        shift = generator.choice((1, 2, 3, 4, 5))
        errors, error = [], generator.randint(-180, 180)
        for _ in range(100):
            error = max(-180, min(180, error + generator.randint(-12, 12)))
            if generator.random() < 0.03:
                error = generator.randint(-180, 180)  # a genuine target change now and then
            errors.append(error)
        cases.append(_filter_case('sweep_%02d' % index, shift, errors))
    return cases


def _turn_case(name: str, config: dict, airspeed: float, bank: float) -> dict:
    """
    One physics case: min_turn_radius(bank), endgame_bank() and landing_turn_radius() at an airspeed.

    Raises:
        ValueError - the airspeed sits on the stall-margin floor.
    """
    resolved = guidance.GuidanceConfig(config, _WINDOW_MS)
    floor = resolved.stall_speed_1g * resolved.stall_margin
    if floor > 0.0 and abs(airspeed - floor) < 10 * _JITTER_SPEED:
        raise ValueError('turn %s: the airspeed sits on the stall floor' % name)
    harness = _Harness(config, _ZONE)
    harness.governor.speed = airspeed
    unit = harness.unit
    return {'name': name, 'input': {'config': config, 'airspeed': airspeed, 'bank': bank},
            'output': {'metres': {'min_turn_radius': unit.min_turn_radius(bank),
                                  'landing_turn_radius': unit.landing_turn_radius()},
                       # float(): a disabled gate or the cap answers the config's own number, an int in config_default
                       'setpoint': {'endgame_bank': float(unit.endgame_bank())}}}


def _turn_cases(generator: random.Random) -> list:
    """test_guidance's min_turn_radius and endgame_bank cases, and a seeded sweep."""
    cases = [_turn_case('trim_at_45', {}, 14.0, 45.0), _turn_case('trim_at_30', {}, 14.0, 30.0),
             _turn_case('too_slow_to_bank', {}, 5.0, 45.0),
             _turn_case('gate_disabled', {'stall_speed_1g': 0}, 14.0, 45.0),
             _turn_case('fast_is_capped', {}, 20.0, 45.0), _turn_case('no_airspeed', {}, 0.0, 45.0),
             _turn_case('under_the_floor', {}, 10.0, 45.0), _turn_case('a_level_bank', {}, 14.0, 0.0),
             _turn_case('a_negative_bank', {}, 14.0, -10.0), _turn_case('negative_airspeed', {}, -3.0, 45.0)]
    while len(cases) < 10 + 60:
        config = {'stall_speed_1g': generator.choice((0, 7.5, 9.0, 10.0)), 'stall_margin': generator.uniform(1.05, 1.4),
                  'endgame_max_bank': generator.uniform(45.0, 70.0), 'land_bank_limit': generator.uniform(30.0, 60.0)}
        try:
            cases.append(_turn_case('sweep_%03d' % (len(cases) - 10), config, generator.uniform(0.0, 30.0),
                                    generator.uniform(5.0, 75.0)))
        except ValueError:
            continue
    return cases


def _reach(harness: _Harness, case: dict) -> object:
    """Plan B's reachability for a case's inputs: the dict, or None."""
    harness.set_inputs(case)
    return harness.unit.reachability(case['glide_ratio'], case['wind_east'], case['wind_north'],
                                     case['reach_airspeed'])


def _reachability_case(name: str, case: dict, generator: random.Random, tie: str = '') -> dict:
    """
    One reachability case: plan B's dict, or null.

    Args:
        name - the case name.
        case - the inputs (_reach_inputs).
        generator - the jitter's coin.
        tie - '' for an ordinary case; else the member that sits ON its rounding tie by construction (see
            _reachability_ties), which the jitter must move while every other member holds.

    Raises:
        ValueError - the jitter changes the answer (a rounding boundary or the reach comparison); for a tie case,
            it changes another member, never moves the tie, or an input is not exact in f32.
    """
    harness = _Harness({}, case['zone'])
    answer = _reach(harness, case)
    if tie and any(struct.unpack('<f', struct.pack('<f', case[key]))[0] != case[key]
                   for key in ('elevation', 'glide_ratio', 'wind_east', 'wind_north', 'reach_airspeed')):
        raise ValueError('reachability %s: a tie needs inputs exact in f32' % name)
    others = lambda reach: reach and {key: value for key, value in reach.items() if key != tie}  # noqa: E731
    tie_moved = False
    for _ in range(_JITTER_RUNS):
        moved = _jittered(case, generator)
        # an airspeed of exactly 0 skips the wind projection on both sides (0.0 is exact in f32): no jitter there
        moved.update({key: case[key] + generator.choice((-1, 1)) * _JITTER_SPEED
                      for key in ('wind_east', 'wind_north', 'reach_airspeed') if case[key] != 0.0})
        jittered = _reach(harness, moved)
        if others(jittered) != others(answer):
            raise ValueError('reachability %s: the jitter moves the answer' % name)
        tie_moved = tie_moved or (tie and jittered[tie] != answer[tie])
    if tie and not tie_moved:
        raise ValueError('reachability %s: %s is not on a tie' % (name, tie))
    inputs = dict(case, zone=None if case['zone'] is None else [list(case['zone'][0]), list(case['zone'][1])])
    return {'name': name, 'input': inputs, 'output': {'exact': answer}}


def _reachability_ties(generator: random.Random) -> list:
    """
    Reachability ON its half-to-even roundings, which plan B's round() decides and the jitter keeps every other
    case away from.

    Both sides compute the tie itself: the inputs are halves and quarters f32 holds exactly, a fix ON the zone's
    centre is exactly 0 m from it, and a fix due north of the centre has a bearing of exactly 180 degrees, whose
    cosine is exactly -1 in f32 and f64 alike. So 0.5 m x 5 is a reach of 2.5 m, which half to even rounds to 2
    (3 half away from zero), 1.5 m x 5 rounds to 8, and a quarter m/s straight along the bearing is a headwind of
    0.25, which rounds to 0.2 either way round (0.3 half away from zero). The reach comparison's own tie is a reach
    of 0 for a distance of 0. The distance's rounding has no such case: a distance is a product with the
    metres-per-unit, never exactly k.5 in f32 and f64 at once, so the Rust unit tests pin that one alone.
    """
    north_of_centre = (_CENTRE[0] + 10_000, _CENTRE[1])  # 111 m due north
    ties = [
        ('a_reach_of_2_5_rounds_to_even', _reach_inputs(_ZONE, _CENTRE, 0.5, 5.0), 'margin_m'),
        ('a_reach_of_7_5_rounds_to_even', _reach_inputs(_ZONE, _CENTRE, 1.5, 5.0), 'margin_m'),
        ('a_headwind_of_0_25_rounds_to_even', _reach_inputs(_ZONE, north_of_centre, 100.0, 3.0, 0.0, 0.25, 14.0),
         'headwind_mps'),
        ('a_tailwind_of_0_25_rounds_to_even', _reach_inputs(_ZONE, north_of_centre, 100.0, 3.0, 0.0, -0.25, 14.0),
         'headwind_mps'),
        # and the reach comparison is inclusive: no height left ON the centre still reaches (`reach >= distance`)
        ('no_height_on_the_centre_still_reaches', _reach_inputs(_ZONE, _CENTRE, 0.0, 5.0), 'reachable'),
    ]
    return [_reachability_case(name, inputs, generator, tie) for name, inputs, tie in ties]


def _reach_inputs(zone: tuple | None, position: tuple | None, elevation: float | None, glide_ratio: float = 3.0,
                  wind_east: float = 0.0, wind_north: float = 0.0, airspeed: float = 0.0,
                  launch: tuple | None = (480_100_000, 110_053_000)) -> dict:
    """A reachability case's inputs: the handles' answers and reachability()'s own arguments."""
    return {'zone': zone, 'position': None if position is None else {'position': list(position), 'age_ms': 0},
            'elevation': elevation, 'launch': None if launch is None else list(launch), 'glide_ratio': glide_ratio,
            'wind_east': wind_east, 'wind_north': wind_north, 'reach_airspeed': airspeed}


def _reachability_cases(generator: random.Random) -> list:
    """test_guidance's reachability cases (and what makes it None), and a seeded sweep."""
    far = (480_200_000, 110_050_000)
    explicit = [
        ('no_fix', _reach_inputs(_ZONE, None, 100.0)),
        ('near_the_centre', _reach_inputs(_ZONE, (480_005_700, 110_050_300), 100.0)),
        ('far_north', _reach_inputs(_ZONE, far, 100.0)),
        ('far_north_headwind', _reach_inputs(_ZONE, far, 100.0, 3.0, 0.0, 8.0, 14.0)),
        ('far_north_tailwind', _reach_inputs(_ZONE, far, 100.0, 3.0, 0.0, -8.0, 14.0)),
        ('a_gale_cannot_make_the_reach_negative', _reach_inputs(_ZONE, far, 100.0, 3.0, 0.0, 30.0, 14.0)),
        ('no_elevation', _reach_inputs(_ZONE, far, None)),
        ('no_zone', _reach_inputs(None, far, 100.0)),
        ('no_launch_point', _reach_inputs(_ZONE, far, 100.0, launch=None)),
    ]
    cases = [_reachability_case(name, inputs, generator) for name, inputs in explicit] + _reachability_ties(generator)
    first_sweep = len(cases)
    while len(cases) < first_sweep + 60:
        centre = (generator.randint(-600_000_000, 600_000_000), generator.randint(-1_700_000_000, 1_700_000_000))
        zone = _random_zone(generator, centre)
        distance = generator.uniform(0.0, 1500.0)
        bearing = math.radians(generator.uniform(0.0, 360.0))
        position = _metres_moved(centre, distance * math.sin(bearing), distance * math.cos(bearing))
        inputs = _reach_inputs(zone, position, generator.uniform(5.0, 250.0), generator.uniform(2.0, 8.0),
                               generator.uniform(-8.0, 8.0), generator.uniform(-8.0, 8.0),
                               generator.choice((0.0, generator.uniform(8.0, 25.0))), centre)
        try:
            cases.append(_reachability_case('sweep_%03d' % (len(cases) - first_sweep), inputs, generator))
        except ValueError:
            continue
    return cases


def build() -> dict:
    """
    The guidance fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, tolerance, config, heading_error, heading_resolve, landing, filter, turn, reachability,
        scenarios}: each section {note, cases}, every output grouped by `compare`.
    """
    generator = random.Random(_SEED)
    return {
        'meta': {'source': 'src/glider/guidance.py, src/glider/navigation.py, src/glider/mission.py, '
                           'src/glider/commons.py, src/glider/fixed.py'},
        'compare': _COMPARE,
        'tolerance': ('exact for integers, flags and the reach dict; degrees angle_deg 0.05 (the plan\'s navigation '
                      'angle); setpoints abs 0.011 degree (plan B truncates to the centidegree); metres abs 0.01 '
                      '(f32 on figures under ~1 km); config rel 1e-6 (a number held in f32). No step lies within '
                      'the jitter of a decision threshold, but for the integer ones (clock, age, filter) and the '
                      'reach ties reached with inputs exact on both sides (see the builder).'),
        'config': {'note': 'GuidanceConfig: the knobs, the control stages\' setpoints in degrees, the pattern id',
                   'cases': _config_cases()},
        'heading_error': {'note': 'the shortest signed heading error target - current, whole degrees',
                          'cases': _heading_error_cases(generator)},
        'heading_resolve': {'note': 'Heading.resolve: a config name -> the pattern id and its name',
                            'cases': _heading_resolve_cases()},
        'landing': {'note': ('the Mission\'s zone_points(): the target and gates Landing::new resolves (the pattern '
                             '\'auto\' resolves to is the mission crate\'s rule, in mission.json)'),
                    'cases': _landing_cases(generator)},
        'filter': {'note': 'the steering filter\'s output for each raw heading error, from a fresh seed',
                   'cases': _filter_cases(generator)},
        'turn': {'note': 'min_turn_radius(bank), endgame_bank() and landing_turn_radius() at an airspeed',
                 'cases': _turn_cases(generator)},
        'reachability': {'note': ('reachability(glide_ratio, wind_east, wind_north, airspeed) under the inputs; '
                                  'the Rust inputs carry reach_airspeed and the wind'),
                         'cases': _reachability_cases(generator)},
        'scenarios': {'note': ('stateful: init {config, zone, endgame_heading (the Mission\'s pattern for the '
                               'zone)}, then steps: enter or compute under the step\'s inputs, and plan B\'s slots '
                               'after it'),
                      'cases': _explicit_scenarios(generator) + [_flight(index, pattern, boost, generator)
                                                                 for index, (pattern, boost) in enumerate(_FLIGHTS)]},
    }
