"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture wind.json: plan B's wind estimator (src/glider/wind.py) as the Rust port (crates/control, module
wind) must answer it -- the wind triangle, its EMA, the physical envelope that rejects a GNSS jump, the NaN guard,
the calm floor, and the operator surface (inspect, update, stats).

Stateful: each scenario starts an estimator from a config block and runs steps, each an `observe` (course, ground
speed, airspeed, heading) or an `update` (the properties of a CC `update wind`), and records plan B's answers after
it. A non-finite observation is written as the text 'nan', 'inf' or '-inf' (JSON has no such numbers).

Every output is grouped by how it compares (`compare`):
  * exact -- inspect() and stats() (their speeds rounded to 0.01 m/s and directions to the degree, as plan B
    reports them), and the names update() changed.
  * speed -- abs 0.001 m/s: components() and speed(). f32 trig on ~15 m/s speeds is good to ~2e-6 m/s, and the
    EMA is contractive, so the error does not grow with the steps.
  * degrees -- angle_deg 0.01: direction(), from a wind at least as strong as the calm floor (>= 0.1 m/s in
    every case), so f32 holds it to ~1e-3 degree.

Cases stay off the decision thresholds, where one ulp could flip an answer: the envelope's rejection, the calm
floor, the roundings in inspect and stats, and the 0/360 wrap of the direction. Every step is re-run by plan B from
the same state with its inputs jittered (course and heading +/-0.001 degree, speeds +/-1e-4 m/s) and the EMA
state itself jittered (+/-1e-4 m/s); a step whose exact answers move under that, or whose direction sits within
0.01 degree of north, is skipped from a seeded run (an explicit one raises). Config and update numbers carry at
most four decimals, so the shortest text of their f32 is the same decimal and inspect() reads back exactly.

The Rust port differs from plan B on purpose in three ways the cases avoid (see its module notes): an update value
that is not a number is ignored, the changed names come in key order (every update here names its keys sorted),
and the tunables read back as JSON floats (every config number here is a float).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import copy
import math
import random

import wind

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SEEDED_RUNS: int = 6
_SEEDED_STEPS: int = 50
_JITTER_DEGREES: float = 0.001
_JITTER_SPEED: float = 1e-4
_JITTER_RUNS: int = 8
_WRAP_MARGIN: float = 0.01

_COMPARE: dict = {
    'exact': {'kind': 'exact'},
    'speed': {'kind': 'abs', 'tol': 0.001},
    'degrees': {'kind': 'angle_deg', 'tol': 0.01},
}

_OBSERVED: tuple = ('course', 'ground_speed', 'airspeed', 'heading')


def _ground(airspeed: float, heading: float, wind_east: float, wind_north: float) -> tuple:
    """The (course, ground_speed) a GNSS reads for the air velocity plus the wind (test_wind `_ground`)."""
    radians = math.radians(heading)
    east = airspeed * math.sin(radians) + wind_east
    north = airspeed * math.cos(radians) + wind_north
    return math.degrees(math.atan2(east, north)) % 360.0, math.sqrt(east * east + north * north)


def _number(value: object) -> float:
    """An observation as plan B takes it: the texts 'nan', 'inf' and '-inf' are those floats."""
    return float(value)


def _observe(course: object, ground_speed: object, airspeed: object, heading: object) -> dict:
    """An observe step."""
    return {'action': 'observe', 'course': course, 'ground_speed': ground_speed, 'airspeed': airspeed,
            'heading': heading}


def _observe_ground(airspeed: float, heading: float, wind_east: float, wind_north: float,
                    measured_airspeed: float | None = None) -> dict:
    """An observe step whose GNSS reading comes from a true wind (the airspeed may be measured wrong)."""
    course, ground_speed = _ground(airspeed, heading, wind_east, wind_north)
    return _observe(course, ground_speed, airspeed if measured_airspeed is None else measured_airspeed, heading)


def _update(properties: dict) -> dict:
    """An update step, its properties in key order (the Rust map is sorted)."""
    return {'action': 'update', 'properties': dict(sorted(properties.items()))}


def _run(estimator: wind.WindEstimator, step: dict) -> object:
    """Run a step; update's changed names, or None for an observe."""
    if step['action'] == 'update':
        return estimator.update(step['properties'])
    estimator.observe(*(_number(step[name]) for name in _OBSERVED))
    return None


def _answers(estimator: wind.WindEstimator, changed: object) -> dict:
    """The estimator's answers after a step, grouped."""
    exact = {'inspect': estimator.inspect(), 'stats': estimator.stats()}
    if changed is not None:
        exact['changed'] = changed
    return {'exact': exact, 'speed': {'components': list(estimator.components()), 'speed': estimator.speed()},
            'degrees': {'direction': estimator.direction()}}


def _jittered(step: dict, generator: random.Random) -> dict:
    """An observe step's finite inputs, each moved by its jitter in a random direction."""
    if step['action'] != 'observe':
        return step
    moved = dict(step)
    for name in _OBSERVED:
        value = step[name]
        if isinstance(value, float) and math.isfinite(value):
            size = _JITTER_DEGREES if name in ('course', 'heading') else _JITTER_SPEED
            moved[name] = value + generator.choice((-1, 1)) * size
    return moved


def _robust(estimator: wind.WindEstimator, step: dict, generator: random.Random) -> str:
    """
    Whether a step sits clear of every decision threshold (see the module notes).

    Returns:
        '' when it is clear; else what moved.
    """
    nominal = copy.copy(estimator)
    reference = _answers(nominal, _run(nominal, step))
    direction = reference['degrees']['direction']
    if (nominal._we or nominal._wn) and min(direction, 360.0 - direction) < _WRAP_MARGIN and \
            nominal.components() != (0.0, 0.0):
        return 'the direction %r sits on the 0/360 wrap' % direction
    for _ in range(_JITTER_RUNS):
        twin = copy.copy(estimator)
        if twin._seen:
            twin._we += generator.choice((-1, 1)) * _JITTER_SPEED
            twin._wn += generator.choice((-1, 1)) * _JITTER_SPEED
        answer = _answers(twin, _run(twin, _jittered(step, generator)))
        if answer['exact'] != reference['exact']:
            return 'the jitter moves %r to %r' % (reference['exact'], answer['exact'])
    return ''


def _scenario(name: str, config: dict | None, steps: list, generator: random.Random) -> dict:
    """
    One explicit scenario on a fresh estimator.

    Raises:
        ValueError - a step sits on a threshold (move its inputs off it).
    """
    estimator = wind.WindEstimator(config)
    recorded = []
    for index, step in enumerate(steps):
        trouble = _robust(estimator, step, generator)
        if trouble:
            raise ValueError('scenario %s step %d: %s' % (name, index, trouble))
        recorded.append({'in': step, 'out': _answers(estimator, _run(estimator, step))})
    return {'name': name, 'init': {'config': config}, 'steps': recorded}


def _explicit(generator: random.Random) -> list:
    """test_wind.py's cases as scenarios, and the edges it does not reach (non-finite fixes, update's rules)."""
    instant = {'triangle_alpha': 1.0}  # alpha 1: one fix is the estimate, no EMA lag
    return [
        _scenario('still_air_reads_calm', instant, [_observe_ground(14.0, 0.0, 0.0, 0.0)], generator),
        _scenario('an_east_wind_from_the_west', instant, [_observe_ground(14.0, 0.0, 5.0, 0.0)], generator),
        # an over-read of 2.5 m/s (test_wind's 2 puts the speed, sqrt(29), 1.6e-4 from a rounding boundary)
        _scenario('the_crosswind_is_bias_free', instant, [_observe_ground(14.0, 0.0, 5.0, 0.0, 16.5)], generator),
        _scenario('a_gnss_jump_beyond_the_envelope_is_rejected', {'triangle_alpha': 1.0, 'max_speed': 15.0}, [
            _observe_ground(14.0, 0.0, 5.0, 0.0),
            _observe(200.0, 99.0, 14.0, 0.0),
        ], generator),
        _scenario('a_wind_under_the_calm_floor_reads_calm', {'triangle_alpha': 1.0, 'calm_speed': 0.5}, [
            _observe_ground(14.0, 0.0, 0.2, 0.0),
        ], generator),
        _scenario('non_finite_fixes_never_poison_the_estimate', {'triangle_alpha': 0.25}, [
            _observe('nan', 15.0, 14.0, 10.0),  # before any fix: not even the first one
            _observe_ground(14.0, 30.0, 3.0, -2.0),
            _observe('nan', 15.0, 14.0, 30.0),
            _observe(35.0, 'inf', 14.0, 30.0),
            _observe(35.0, 15.0, '-inf', 30.0),
            _observe(35.0, 15.0, 14.0, 'nan'),
            _observe(0.0, 'inf', 14.0, 30.0),  # one component NaN (inf x sin 0), the other infinite
            _observe(90.0, 'inf', 14.0, 30.0),
            _observe_ground(14.0, 40.0, 3.0, -2.0),
        ], generator),
        # either side of the envelope (15 m/s) and of the calm floor (0.5 m/s), a few percent away
        _scenario('the_envelope_and_the_floor_sit_where_the_config_puts_them',
                  {'triangle_alpha': 1.0, 'calm_speed': 0.5, 'max_speed': 15.0}, [
            _observe_ground(14.0, 20.0, 12.31, 8.81),  # 15.14 m/s: rejected, even as the first fix
            _observe_ground(14.0, 20.0, 11.83, 8.47),  # 14.55 m/s: taken
            _observe_ground(14.0, 20.0, 0.31, 0.37),  # 0.483 m/s: calm
            _observe_ground(14.0, 20.0, 0.33, 0.39),  # 0.511 m/s: a wind
        ], generator),
        _scenario('the_ema_blends_each_fix_in', {'triangle_alpha': 0.25, 'calm_speed': 0.5, 'max_speed': 15.0}, [
            _observe_ground(14.0, 90.0, 0.0, 4.0),
            _observe_ground(14.0, 90.0, 0.0, 8.0),
            _observe_ground(15.0, 120.0, -2.13, 6.07),
            _observe_ground(13.0, 250.0, -2.37, 6.19),
        ], generator),
        _scenario('inspect_publishes_and_update_retunes', {'triangle_alpha': 0.05, 'max_speed': 15.0}, [
            _update({'max_speed': 12.0, 'calm_speed': 1.0}),
            _update({'max_speed': 12.0, 'triangle_alpha': 0.05}),  # unchanged values are not listed
            _update({'speed': 3.0, 'method': 'gps', 'from': 90.0}),  # nothing else is writable
            _observe_ground(14.0, 10.0, 6.0, 1.0),
            _update({'calm_speed': 7.0}),  # the floor now hides the 6 m/s estimate
            _update({'calm_speed': 0.5, 'max_speed': 5.0}),  # the narrower envelope rejects the next 6 m/s fix
            _observe_ground(14.0, 12.0, 6.5, 1.0),
            _update({'triangle_alpha': 0.5}),
            _observe_ground(14.0, 12.0, 4.0, 1.0),
        ], generator),
        _scenario('no_config_takes_the_defaults', None, [_observe_ground(15.0, 200.0, -1.55, 2.45)], generator),
    ]


def _seeded(index: int, generator: random.Random) -> dict:
    """
    One seeded run: a random config, then fixes from a steady true wind seen through GNSS noise, an airspeed bias,
    a wandering heading, GNSS jumps now and then, and a live retune or two. Steps on a threshold are skipped.
    """
    config = {'triangle_alpha': round(generator.uniform(0.02, 0.5), 4),
              'calm_speed': round(generator.uniform(0.1, 1.0), 4),
              'max_speed': round(generator.uniform(8.0, 20.0), 4)}
    estimator = wind.WindEstimator(config)
    true_speed, toward = generator.uniform(0.0, 9.0), math.radians(generator.uniform(0.0, 360.0))
    wind_east, wind_north = true_speed * math.sin(toward), true_speed * math.cos(toward)
    airspeed, bias = generator.uniform(12.0, 18.0), generator.uniform(-1.0, 1.0)
    heading = generator.uniform(0.0, 360.0)
    recorded = []
    while len(recorded) < _SEEDED_STEPS:
        if generator.random() < 0.04:
            step = _update({key: round(generator.uniform(*span), 4) for key, span in
                            (('triangle_alpha', (0.02, 0.5)), ('calm_speed', (0.1, 1.0)), ('max_speed', (8.0, 20.0)))
                            if generator.random() < 0.6})
        else:
            heading = (heading + generator.uniform(-25.0, 25.0)) % 360.0
            course, ground_speed = _ground(airspeed, heading, wind_east, wind_north)
            course = (course + generator.gauss(0.0, 2.0)) % 360.0
            ground_speed = max(0.0, ground_speed + generator.gauss(0.0, 0.3))
            if generator.random() < 0.05:  # a GNSS jump
                course, ground_speed = (course + generator.uniform(60.0, 180.0)) % 360.0, ground_speed * 2.0
            step = _observe(course, ground_speed, airspeed + bias, heading)
        if _robust(estimator, step, generator):
            continue
        recorded.append({'in': step, 'out': _answers(estimator, _run(estimator, step))})
    return {'name': 'seeded_%02d' % index, 'init': {'config': config}, 'steps': recorded}


def build() -> dict:
    """
    The wind fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, tolerance, scenarios}: scenarios {note, cases}, every output grouped by `compare`.
    """
    generator = random.Random(_SEED)
    return {
        'meta': {'source': 'src/glider/wind.py, src/glider/inspector.py'},
        'compare': _COMPARE,
        'tolerance': ('inspect, stats and changed exactly (their roundings are plan B\'s); components and speed abs '
                      '0.001 m/s; direction angle_deg 0.01. No step lies within the jitter of a decision threshold.'),
        'scenarios': {'note': ('stateful: init {config}, then observe or update steps and plan B\'s answers after '
                               'each'),
                      'cases': _explicit(generator) + [_seeded(index, generator) for index in range(_SEEDED_RUNS)]},
    }
