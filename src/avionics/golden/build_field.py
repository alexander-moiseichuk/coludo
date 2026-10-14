"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture field.json: plan B's CC-less field agent (src/glider/tasks/field.py) and the two Mission methods it
drives (src/glider/mission.py select_site, fallback_zone), as the Rust port (crates/mission, module field) must
answer them, tick by tick.

Plan B's _tick(now) is sync and tick-driven so a test controls the time: in SETTING, a fresh fix picks the site once
(the nearest known site within max_range_m, else a synthesized fallback zone at the clear-sector bearing); with
auto_arm on, unarmed and a zone set, it arms once the board sat inside the still band (fresh accel) with a fresh fix
for auto_arm_dwell_s. The builder runs plan B's own setup() and _tick(), with:
  * a plan B Mission loaded from a launch.config text (sites, maybe a zone) written to a temporary file;
  * the `position` and `accel` parameters as stand-ins whose read() answers each tick's scripted reading;
  * a stub Controller holding the stage and the arming; its arm() arms and counts.

The site coordinates are made up (no real pad but the default config's public HPRC).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import json
import os
import random
import tempfile

import controller
import databoard
import inspector
import mission
import recorder
from tasks import field

_SEED: int = 20261011  # the seeded scenarios' random.Random seed: the fixture is a function of the code alone
_SEEDED: int = 16  # seeded scenarios
_MAX_RANGE_M: float = 200.0  # the board config's field.max_range_m in every scenario

_STAGE = controller.Stage

# made-up sites: two pads 150 m apart on one field, and a far one
_NORTH = {'name': 'north_field', 'pad': [45.50000, -73.50000],
          'zone': [[45.50300, -73.50200], [45.50200, -73.50050]]}
_SOUTH = {'name': 'south_field', 'pad': [45.49865, -73.50000],
          'zone': [[45.49800, -73.49900], [45.49700, -73.49750]]}
_FAR = {'name': 'far_field', 'pad': [48.10000, 11.50000], 'zone': [[48.10300, 11.49800], [48.10200, 11.50100]]}


class _Controller:
    """The stage and the arming plan B's _tick() reads; arm() as the Controller's, counted."""

    def __init__(self) -> None:
        self.stage: int = _STAGE.SETTING
        self.armed: bool = False
        self.arms: int = 0

    def arm(self) -> None:
        self.armed = True
        self.arms += 1


class _Parameter:
    """A databoard parameter stand-in: read() answers the tick's scripted (value, source, age)."""

    def __init__(self) -> None:
        self.reading: tuple = (None, None, None)

    def read(self) -> tuple:
        return self.reading


def _zone(value) -> list:
    """A Mission zone as JSON: [[lat, lon], [lat, lon]], or None."""
    return None if value is None else [list(value[0]), list(value[1])]


def _scenario(name: str, launch: dict, settings: dict, steps: list) -> dict:
    """
    One scenario through plan B's _tick().

    Args:
        name - the case name.
        launch - the launch.config document the Mission loads (sites, maybe a zone).
        settings - the field entry's keys.
        steps - the ticks: {now_ms, stage, armed (set before the tick: the operator's own arm or disarm), fix:
            [lat, lon] fresh or null, accel: [x, y, z] g fresh or null}.

    Returns:
        The case: input {launch, settings, max_range_m, steps}, output after each tick {site, zone, armed, arms}.
    """
    descriptor, path = tempfile.mkstemp(suffix='.config')
    with os.fdopen(descriptor, 'w') as file:
        json.dump(launch, file)
    stub = _Controller()
    parameters = {'position': _Parameter(), 'accel': _Parameter()}
    board_mission = mission.Mission(path, _MAX_RANGE_M)
    originals = databoard.Databoard.parameter, inspector.Inspector.get, recorder.Recorder.log
    databoard.Databoard.parameter = staticmethod(lambda quantity: parameters[quantity])
    inspector.Inspector.get = staticmethod(lambda name: board_mission if name == 'mission' else None)
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    states = []
    try:
        agent = field.Field('field', dict(settings), stub)
        asyncio.run(agent.setup())
        for step in steps:
            stub.stage = _STAGE.NAMES[step['stage']]
            if step['armed']:  # the operator's arm; false keeps whatever the field made
                stub.armed = True
            parameters['position'].reading = ((tuple(step['fix']), 'gnss', 0) if step['fix'] is not None
                                              else (None, None, None))
            parameters['accel'].reading = ((tuple(step['accel']), 'imu', 0) if step['accel'] is not None
                                           else (None, None, None))
            agent._tick(step['now_ms'])
            states.append({'site': board_mission.site, 'zone': _zone(board_mission.zone), 'armed': stub.armed,
                           'arms': stub.arms})
    finally:
        databoard.Databoard.parameter, inspector.Inspector.get, recorder.Recorder.log = originals
        os.remove(path)
    return {'name': name, 'input': {'launch': launch, 'settings': settings, 'max_range_m': _MAX_RANGE_M,
                                    'steps': steps}, 'output': states}


def _ticks(count: int, fix, accel=(0.0, 0.0, 1.0), stage: str = 'setting', start_ms: int = 1000,
           period_ms: int = 1000, armed=False) -> list:
    """`count` ticks a period apart, one fix and one accel throughout; armed False keeps the stub's own state."""
    return [{'now_ms': start_ms + index * period_ms, 'stage': stage, 'armed': armed,
             'fix': None if fix is None else list(fix), 'accel': None if accel is None else list(accel)}
            for index in range(count)]


def _seeded(generator: random.Random, index: int) -> dict:
    """A seeded pad: a fix somewhere around the two near sites (or nowhere near), stillness broken now and then."""
    centre = generator.choice((_NORTH['pad'], _SOUTH['pad'], [45.5100, -73.5000], [45.4950, -73.4800]))
    fix = [centre[0] + generator.uniform(-0.001, 0.001), centre[1] + generator.uniform(-0.001, 0.001)]
    dwell_s = generator.choice((3, 5, 2.5))
    settings = {'auto_arm': generator.random() < 0.8, 'auto_arm_dwell_s': dwell_s,
                'still_g': generator.choice((0.3, 0.1)), 'site_select': generator.random() < 0.85,
                'fallback_bearing_deg': generator.uniform(0, 360), 'fallback_near_m': generator.uniform(30, 80),
                'fallback_width_m': generator.uniform(60, 140), 'fallback_depth_m': generator.uniform(50, 120)}
    launch = {'sites': [_NORTH, _SOUTH, _FAR]}
    if generator.random() < 0.3:
        launch['zone'] = _FAR['zone']
    steps, now = [], 1000
    for _ in range(generator.randint(4, 14)):
        roll = generator.random()
        if roll < 0.1:
            reading = {'fix': None, 'accel': [0.0, 0.0, 1.0]}
        elif roll < 0.2:
            reading = {'fix': fix, 'accel': None}
        elif roll < 0.35:
            reading = {'fix': fix, 'accel': [generator.uniform(-1, 1), 0.2, generator.uniform(0.2, 1.8)]}
        else:
            reading = {'fix': fix, 'accel': [generator.gauss(0, 0.05), generator.gauss(0, 0.05),
                                             1.0 + generator.gauss(0, 0.05)]}
        steps.append(dict(now_ms=now, stage='setting' if generator.random() < 0.9 else 'boosting', armed=False,
                          **reading))
        now += generator.choice((500, 1000, 1000, 1500))
    return _scenario('seeded_%02d' % index, launch, settings, steps)


def build() -> dict:
    """
    The field fixture document.

    Returns:
        {meta, compare, note, cases}.
    """
    sites = {'sites': [_NORTH, _SOUTH, _FAR]}
    near_north = (45.50010, -73.50010)
    nowhere = (45.52000, -73.47000)
    auto = {'auto_arm': True, 'auto_arm_dwell_s': 5}
    cases = [
        _scenario('the_nearest_site_in_range_is_picked', sites, {}, _ticks(2, near_north)),
        _scenario('between_two_sites_the_nearer_wins', sites, {}, _ticks(1, (45.49880, -73.50000))),
        _scenario('no_site_in_range_synthesizes_the_fallback', sites,
                  {'fallback_bearing_deg': 135.0, 'fallback_near_m': 40.0, 'fallback_width_m': 120.0,
                   'fallback_depth_m': 80.0}, _ticks(2, nowhere)),
        _scenario('the_fallback_defaults_face_north', {}, {}, _ticks(1, nowhere)),
        _scenario('site_select_off_keeps_the_mission', dict(sites, zone=_FAR['zone']), {'site_select': False},
                  _ticks(2, near_north)),
        _scenario('no_fix_decides_nothing_until_one_comes', sites, {},
                  _ticks(3, None) + _ticks(1, near_north, start_ms=4000)),
        _scenario('off_the_pad_decides_nothing', sites, auto, _ticks(8, near_north, stage='boosting')),
        _scenario('a_still_dwell_arms_once', sites, auto, _ticks(9, near_north)),
        _scenario('moving_restarts_the_dwell', sites, auto,
                  _ticks(4, near_north) + _ticks(1, near_north, accel=(0.9, 0.0, 1.2), start_ms=5000)
                  + _ticks(7, near_north, start_ms=6000)),
        _scenario('a_stale_accel_or_fix_restarts_the_dwell', sites, auto,
                  _ticks(4, near_north) + _ticks(1, near_north, accel=None, start_ms=5000)
                  + _ticks(3, near_north, start_ms=6000) + _ticks(1, None, start_ms=9000)
                  + _ticks(7, near_north, start_ms=10000)),
        _scenario('no_zone_never_arms', {}, dict(auto, site_select=False), _ticks(9, near_north)),
        _scenario('the_operator_armed_first', sites, auto,
                  _ticks(2, near_north) + _ticks(8, near_north, start_ms=3000, armed=True)),
        _scenario('auto_arm_off_never_arms', sites, {'auto_arm': False}, _ticks(9, near_north)),
        # just off the band's edges: exactly on one, f32 and f64 round 1.3 to opposite sides of it
        _scenario('the_still_band_edges', sites, dict(auto, still_g=0.3),
                  _ticks(3, near_north, accel=(0.0, 0.0, 1.31)) + _ticks(3, near_north, accel=(0.0, 0.0, 0.69),
                                                                         start_ms=4000)
                  + _ticks(7, near_north, accel=(0.0, 0.0, 1.29), start_ms=7000)),
        _scenario('a_fractional_dwell', sites, {'auto_arm': True, 'auto_arm_dwell_s': 2.5},
                  _ticks(6, near_north, period_ms=500)),
    ]
    generator = random.Random(_SEED)
    cases += [_seeded(generator, index) for index in range(_SEEDED)]
    return {
        'meta': {'source': 'src/glider/tasks/field.py, src/glider/mission.py'},
        'compare': {'kind': 'abs', 'tol': 1e-6},
        'note': ('per case: input {launch (the launch.config the Mission loads), settings (the field entry), '
                 'max_range_m, steps}, a step per _tick(): {now_ms, stage, armed (the operator arms before the tick; '
                 'false keeps the arming the field made), fix [lat, lon] fresh or null, accel [x, y, z] g fresh or '
                 'null}; output after each tick: {site, zone [[lat, lon], [lat, lon]] degrees or null, armed, arms '
                 '(the arm() calls so far)}'),
        'cases': cases,
    }
