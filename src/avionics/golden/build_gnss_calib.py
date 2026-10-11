"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture gnss_calib.json: plan B's pad-drift calibration (src/glider/tasks/gnss_calib.py) as the Rust port
(crates/control, module drift) must answer it, pass by pass.

Plan B's run() wakes every period_ms and, until it froze: in SETTING it restarts its sums at the first armed pass and
folds the fused ground velocity in (speed and course both fresh by read(), east = speed x sin(course), north =
speed x cos(course)); off the pad (BOOSTING or later, a warm start into GLIDING included) it freezes the mean when it
has min_samples, else a zero drift. The builder runs plan B's own setup() and run(), with:
  * a stub Controller holding the stage and the arming of each pass;
  * the `speed` and `course` parameters as stand-ins whose read() answers each pass's scripted [value, source, age]
    (a stale reading has no source, and may keep a value: plan B's read() of an expired channel);
  * asyncio.sleep_ms as the pass boundary: the builder records the calibrator's state after each pass, then sets the
    next pass's inputs.

Plan B divides by its sample count when min_samples is 0 and no sample came (ZeroDivisionError, the task dies):
no scenario runs that, and the port freezes a zero drift there (crates/control drift).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import math
import random

import controller
import databoard
import recorder
from tasks import gnss_calib

_SEED: int = 20261010  # the seeded scenarios' random.Random seed: the fixture is a function of the code alone
_SEEDED: int = 16  # seeded scenarios

_STAGE = controller.Stage


class _Controller:
    """The stage and the arming plan B's run() reads each pass."""

    def __init__(self) -> None:
        self.stage: int = _STAGE.SETTING
        self.armed: bool = False


class _Parameter:
    """A databoard parameter stand-in: read() answers the pass's scripted [value, source, age]."""

    def __init__(self) -> None:
        self.reading: list = [None, None, None]

    def read(self) -> tuple:
        return tuple(self.reading)


def _scenario(name: str, steps: list, min_samples: int = 5) -> dict:
    """
    One scenario through plan B's run().

    Args:
        name - the case name.
        steps - the passes: {stage, armed, speed: [value, source, age], course: [value, source, age]}.
        min_samples - the entry's `min_samples`.

    Returns:
        The case: input {min_samples, steps}, output the state after each pass {drift: [east, north], samples,
        frozen}.
    """
    stub = _Controller()
    parameters = {'speed': _Parameter(), 'course': _Parameter()}
    original_parameter = databoard.Databoard.parameter
    databoard.Databoard.parameter = staticmethod(lambda quantity: parameters[quantity])
    try:
        calibrator = gnss_calib.GnssCalib('gnss_calib', {'min_samples': min_samples, 'period_ms': 1000}, stub)
        asyncio.run(calibrator.setup())
    finally:
        databoard.Databoard.parameter = original_parameter
    states: list = []

    def apply(step: dict) -> None:
        """Set one pass's inputs."""
        stub.stage = _STAGE.NAMES[step['stage']]
        stub.armed = step['armed']
        parameters['speed'].reading = list(step['speed'])
        parameters['course'].reading = list(step['course'])

    class _Done(Exception):
        """The last pass ran."""

    async def sleep_ms(_milliseconds: int) -> None:
        """The pass boundary: record the state, then the next pass's inputs (or the end)."""
        drift_east, drift_north = calibrator.drift()
        states.append({'drift': [drift_east, drift_north], 'samples': calibrator._count,
                       'frozen': calibrator._frozen})
        if len(states) == len(steps):
            raise _Done()
        apply(steps[len(states)])

    original_sleep, original_log = asyncio.sleep_ms, recorder.Recorder.log
    asyncio.sleep_ms = sleep_ms
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    apply(steps[0])
    try:
        asyncio.run(calibrator.run())
    except _Done:
        pass
    finally:
        asyncio.sleep_ms, recorder.Recorder.log = original_sleep, original_log
    return {'name': name, 'input': {'min_samples': min_samples, 'steps': steps}, 'output': states}


def _fresh(speed: float, course: float) -> dict:
    """A pass's GNSS inputs, both fresh."""
    return {'speed': [speed, 'gnss', 0], 'course': [course, 'gnss', 0]}


def _stale(kept: bool = False) -> dict:
    """A pass's GNSS inputs, both stale: no source, the last value kept or none."""
    return {'speed': [0.4 if kept else None, None, 900 if kept else None],
            'course': [90.0 if kept else None, None, 900 if kept else None]}


def _pass(stage: str, armed: bool, inputs: dict) -> dict:
    """One pass."""
    return dict(stage=stage, armed=armed, **inputs)


def _drift(speed: float, course: float, count: int, stage: str = 'setting', armed: bool = True) -> list:
    """`count` passes of one steady drift."""
    return [_pass(stage, armed, _fresh(speed, course)) for _ in range(count)]


def _seeded(generator: random.Random, index: int) -> dict:
    """A seeded scenario: a walk disarmed, the arm, a jittered drift with dropouts, then the launch and a while after."""
    steps = [_pass('setting', False, _fresh(generator.uniform(0.5, 2.0), generator.uniform(0, 360)))
             for _ in range(generator.randint(0, 6))]
    speed, course = generator.uniform(0.0, 0.6), generator.uniform(0.0, 360.0)
    for _ in range(generator.randint(0, 14)):
        roll = generator.random()
        if roll < 0.15:
            steps.append(_pass('setting', True, _stale(kept=generator.random() < 0.5)))
        elif roll < 0.22:  # one side stale
            inputs = _fresh(speed, course)
            inputs[generator.choice(('speed', 'course'))] = [None, None, None]
            steps.append(_pass('setting', True, inputs))
        else:
            steps.append(_pass('setting', True, _fresh(max(0.0, speed + generator.gauss(0, 0.05)),
                                                       (course + generator.gauss(0, 8)) % 360.0)))
    stage = generator.choice(('boosting', 'boosting', 'gliding'))
    steps += [_pass(stage, True, _fresh(30.0, generator.uniform(0, 360))) for _ in range(generator.randint(1, 3))]
    return _scenario('seeded_%02d' % index, steps, min_samples=generator.choice((1, 3, 5, 5, 8)))


def build() -> dict:
    """
    The gnss_calib fixture document.

    Returns:
        {meta, compare, note, cases}.
    """
    walk = [_pass('setting', False, _fresh(1.4, 200.0)) for _ in range(4)]
    cases = [
        _scenario('the_arm_restarts_the_mean_and_boosting_freezes_it',
                  walk + _drift(0.3, 45.0, 6) + _drift(30.0, 10.0, 2, stage='boosting')),
        _scenario('too_few_samples_freeze_zero',
                  walk + _drift(0.3, 45.0, 4) + _drift(30.0, 10.0, 1, stage='boosting')),
        _scenario('stale_readings_are_never_sampled',
                  _drift(0.2, 300.0, 3) + [_pass('setting', True, _stale()), _pass('setting', True, _stale(True))]
                  + [_pass('setting', True, {'speed': [5.0, 'gnss', 0], 'course': [None, None, None]}),
                     _pass('setting', True, {'speed': [None, None, None], 'course': [10.0, 'gnss', 0]})]
                  + _drift(0.2, 300.0, 2) + _drift(30.0, 10.0, 1, stage='boosting')),
        _scenario('never_armed_samples_the_whole_pad',
                  walk + _drift(0.3, 45.0, 3, armed=False) + _drift(30.0, 10.0, 1, stage='boosting', armed=False)),
        _scenario('a_warm_start_into_gliding_freezes_zero', _drift(25.0, 90.0, 3, stage='gliding')),
        _scenario('frozen_stays_frozen_back_in_setting',
                  _drift(0.5, 180.0, 5) + _drift(30.0, 0.0, 1, stage='boosting') + _drift(2.0, 0.0, 3)),
        _scenario('a_disarm_and_rearm_never_restarts_twice',
                  _drift(0.3, 90.0, 3) + _drift(0.3, 90.0, 2, armed=False) + _drift(0.6, 270.0, 3)
                  + _drift(30.0, 0.0, 1, stage='boosting'), min_samples=3),
        _scenario('min_samples_one', _drift(0.25, 359.0, 1) + _drift(30.0, 0.0, 1, stage='landing'), min_samples=1),
        _scenario('the_compass_points',
                  [_pass('setting', True, _fresh(1.0, course)) for course in (0.0, 90.0, 180.0, 270.0, 45.0)]
                  + _drift(30.0, 0.0, 1, stage='done'), min_samples=5),
    ]
    generator = random.Random(_SEED)
    cases += [_seeded(generator, index) for index in range(_SEEDED)]
    for case in cases:
        for state in case['output']:
            if not all(math.isfinite(value) for value in state['drift']):
                raise ValueError('gnss_calib: %s froze a non-finite drift' % case['name'])
    return {
        'meta': {'source': 'src/glider/tasks/gnss_calib.py'},
        'compare': {'kind': 'abs', 'tol': 1e-4},
        'note': ('per case: input {min_samples, steps}, a step per pass of plan B\'s run(): {stage, armed, speed and '
                 'course as read() answers them: [value, source or null when stale, age]}; output the state after '
                 'each pass: {drift: [east, north] m/s, samples since the restart, frozen}'),
        'cases': cases,
    }
