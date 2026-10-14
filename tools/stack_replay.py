#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The 10-03 flights replayed on the 6-DoF boost (tools/stack_dynamics.py, doc/specs/boost-control.md BC1): each stack
in that day's ~4.5 m/s crosswind, fins neutral, its events beside what the flight measured (launches/20261003, the
judge's numbers). The acceptance is the event and its order of magnitude, not a curve fit: 1.3 s (7C) and 6.4 s (7D)
of one flight each cannot carry more.

    python3 tools/stack_replay.py            # every replay, model against measured
    python3 tools/stack_replay.py tms7d      # one
"""

import math
import sys

import stack_dynamics

WIND_MS: float = 4.5  # 10-03's crosswind, fitted on 7D's rod exit (9 m/s, 26 deg angle of attack)

"""
REPLAYS: per stack, how long to fly and what the flight measured. The wind blows toward east and the stack's dorsal
side faces east: the side force of 7C's 9.3 g was along the board's +z, the plane normal to the folded wings.
"""
REPLAYS: dict = {
    'tms7c': {'seconds': 1.6, 'measured': 'departed 0.31 s, nose past horizontal under thrust ~1.06 s, side load '
                                          '9.3 g, roll -372 deg/s at 0.47 s'},
    'tms7d': {'seconds': 7.0, 'measured': 'angle of attack 17-19 deg 0.4-1.2 s, 2 deg by 2.2 s; path minimum 32 deg; '
                                          'burnout 53.5 m/s at 2.53 s, 54 m up; apogee 91-98 m, ~145 m out; coast '
                                          'roll ~20 deg/s per m/s'},
    'tms7': {'seconds': 10.0, 'measured': 'burnout 125 m/s at 2.68 s, tilt ~12 deg through the burn, apogee '
                                          '464-479 m'},
}


def replay(name: str, profile: dict = None) -> dict:
    """
    Fly one replay and read its events.

    Args:
        name - a REPLAYS (and STACKS) name.
        profile - the stack to fly instead of STACKS[name] (a variant); None flies the named one.

    Returns:
        The events: the angle of attack early (0.5-1.2 s, mean) and late (2.2-2.6 s, mean), the burn's path minimum,
        the burnout speed, height and range, the apogee and its range, the time the nose first passed horizontal
        (None: never) and whether the motor still burned, the largest side load in g, the roll rate at 0.5 s, and
        the coast's roll per m/s at the end.
    """
    stack = stack_dynamics.Stack(profile or stack_dynamics.STACKS[name], WIND_MS, 0.0, 90.0)
    early, late = [], []
    events = {'path_minimum': 90.0, 'apogee': 0.0, 'apogee_range': 0.0, 'horizontal': None, 'burning': None,
              'side_g': 0.0, 'roll_at_half': 0.0, 'burnout': None}
    burnout = stack_dynamics.F15[-1][0]
    while stack.seconds < REPLAYS[name]['seconds']:
        stack.step(0.01)
        speed = math.sqrt(sum(component * component for component in stack.velocity))
        if 0.5 <= stack.seconds <= 1.2:
            early.append(stack.angle_of_attack)
        if 2.2 <= stack.seconds <= 2.6:
            late.append(stack.angle_of_attack)
        if 0.4 < stack.seconds < burnout:
            events['path_minimum'] = min(events['path_minimum'], stack.path_angle())
        if events['burnout'] is None and stack.seconds >= burnout:
            events['burnout'] = (speed, stack.position[2], math.hypot(stack.position[0], stack.position[1]))
        if stack.position[2] > events['apogee']:
            events['apogee'] = stack.position[2]
            events['apogee_range'] = math.hypot(stack.position[0], stack.position[1])
        if events['horizontal'] is None and stack.nose_elevation() < 0.0:
            events['horizontal'] = stack.seconds
            events['burning'] = stack_dynamics.thrust_at(stack.seconds) > 0.0
        events['side_g'] = max(events['side_g'], math.hypot(stack.specific[1], stack.specific[2]) / stack_dynamics.G)
        if abs(stack.seconds - 0.5) < 0.005:
            events['roll_at_half'] = math.degrees(stack.rates[0])
        if stack.position[2] < -0.5:
            break
    speed = math.sqrt(sum(component * component for component in stack.velocity))
    events['early_angle'] = sum(early) / len(early) if early else None
    events['late_angle'] = sum(late) / len(late) if late else None
    events['roll_per_speed'] = math.degrees(stack.rates[0]) / speed if speed > 1.0 else None
    return events


def _shown(value) -> str:
    """A figure for the table: one decimal, or a dash for none."""
    if value is None:
        return '-'
    if isinstance(value, tuple):
        return '/'.join('%.1f' % part for part in value)
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    return '%.1f' % value


def main() -> int:
    names = sys.argv[1:] or list(REPLAYS)
    for name in names:
        if name not in REPLAYS:
            print('no replay %r (%s)' % (name, ', '.join(REPLAYS)))
            return 2
        events = replay(name)
        print('== %s (margin %s cal)' % (name, stack_dynamics.STACKS[name]['margin_cal'][0]))
        print('   model:    ' + ', '.join('%s %s' % (key, _shown(value)) for key, value in events.items()))
        print('   measured: ' + REPLAYS[name]['measured'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
