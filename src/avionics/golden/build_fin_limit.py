"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture fin_limit.json: plan B's dynamic-pressure fin deflection table (src/glider/commons.py
fin_deflection_limit) as the Rust port (crates/control, module commons) must answer it -- the largest fin
deflection, in whole degrees, at an airspeed in m/s.

Compare EXACT: the table is integers (Python's round() of 12500 / v^2, half to even, clamped to [5, 45]), and the
lookup truncates the speed toward zero with int(). The cases are the speeds test_commons.py, bench_cases.py and the
governor tests use, every whole speed of the table and past it, and both sides of every step of the table -- 1e-3
m/s under and over each whole speed where the entry changes, plus the midpoint. A speed must keep clear of a whole
number it is not: the Rust port reads the speed as f32, and rounding a decimal to f32 moves it by half an ulp
(under 4e-6 m/s up to 80), so 1e-3 is a 250-fold margin; the seeded sweep skips anything closer (_NEAR_INTEGER).

commons.json (the sequencer port's fixture) samples the same table among the other commons primitives; this one is
the control core's, exhaustive at the steps the governor's cap moves on.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import commons

_SEED: int = 20261006  # the sweep's random.Random seed: the fixture is a function of the code alone
_SWEEP: int = 120  # seeded speeds
_NEAR_INTEGER: float = 1e-03  # how far a speed keeps from a whole number it is not (see the module notes)
_STEP_OFFSET: float = 1e-03  # how far under and over a table step its two cases sit
_TOP_SPEED: int = 80  # commons._FIN_VMAX: the table saturates here

"""The speeds plan B's tests and benches use (m/s): test_commons, bench_cases, test_governor's 40 and 30."""
_SPEEDS: tuple = (0, 15, 30, 50, 200, -5, 0.0, 5.0, 15.0, 30.0, 90.0, 40.0, 4.0, 14.0, 28.0, 60.0, -0.5, -1.5, 0.5,
                  1e6, -1e6)


def _case(name: str, speed: float) -> dict:
    """One case: the speed, plan B's whole-degree limit."""
    return {'name': name, 'input': speed, 'output': commons.fin_deflection_limit(speed)}


def build() -> dict:
    """
    The fin_limit fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, cases}.
    """
    cases = [_case('plan_b_speed_%r' % speed, speed) for speed in _SPEEDS]
    cases += [_case('whole_%02d' % speed, float(speed)) for speed in range(_TOP_SPEED + 2)]
    for speed in range(1, _TOP_SPEED + 1):
        if commons.fin_deflection_limit(speed - 1) == commons.fin_deflection_limit(speed):
            continue
        cases.append(_case('step_%02d_under' % speed, speed - _STEP_OFFSET))
        cases.append(_case('step_%02d_over' % speed, speed + _STEP_OFFSET))
        cases.append(_case('step_%02d_midpoint_below' % speed, speed - 0.5))
    generator = random.Random(_SEED)
    sweep = []
    while len(sweep) < _SWEEP:
        speed = generator.uniform(-10.0, 120.0)
        if abs(speed - round(speed)) < _NEAR_INTEGER:
            continue
        sweep.append(_case('sweep_%03d' % len(sweep), speed))
    cases += sweep
    return {
        'meta': {'source': 'src/glider/commons.py'},
        'compare': {'kind': 'exact'},
        'note': 'whole degrees; int() truncates the speed; no speed within 1e-3 of a whole number it is not',
        'cases': cases,
    }
