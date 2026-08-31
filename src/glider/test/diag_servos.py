"""
How well are the three fin servos assembled? A per-surface mechanical + electrical assessment.

diag_devices' servo probe answers only "did it draw current". After a rebuild the useful questions are
comparative: does each surface reach both ends, do the two elerons behave like each other, and does any
one of them draw noticeably more than its siblings -- which is what binding, a mis-splined horn or a
linkage fouling the body actually look like from here.

Each servo is driven ALONE (the others held at neutral) so a draw belongs to exactly one surface, and
the sweep is stepped rather than commanded end-to-end, so a servo that stalls part-way shows up as a
current plateau instead of a single peak.

WARNING: THE FINS MOVE, through their full configured travel. Keep fingers and clothing clear.

    mpremote connect $PORT run src/glider/test/diag_servos.py
"""

import asyncio

import config
import databoard
import main

_SETTLE_MS: int = 600    # let a surface come to rest before the baseline is taken
_SAMPLE_MS: int = 50     # sampling interval within a step
_TRAVEL_MS: int = 250    # motion window: an SG90 needs ~56 ms for a 22 deg step, so this is generous
_REST_MS: int = 250      # the AT-REST window that follows it -- 2+ fresh INA226 reads (100 ms period)
_STEPS: int = 9          # positions across the travel; odd, so neutral is one of them
_HOLD_MW: int = 150      # draw above baseline that counts as "still fighting something"

# The servo actually BOLTED ON each fin, which may differ from the config while a rig is being reworked.
# The bench now carries three SG90s so it matches TMS-7D; the config's mg90s yaw is the design intent.
_FITTED: dict = {'servo_yaw': 'sg90', 'servo_eleron_left': 'sg90', 'servo_eleron_right': 'sg90'}


def _power() -> int:
    """
    Rail power in mW, or 0 when no INA226 is fitted.

    read(), NOT value(). value() EXTRAPOLATES a stale channel without bound, and the INA226 runs at a
    100 ms period -- so sampling it faster than that through value() invents readings, and a reading
    invented mid-motion looks exactly like a servo that never stopped pulling. That mistake is what made
    the first version of this script report all three surfaces as binding when all three were healthy.
    """
    value, source, _age = databoard.Databoard.parameter('power').read()
    return value if source is not None and value is not None else 0


async def _sweep(unit, label: str, targets: list) -> dict:
    """
    Step one servo through `targets`, separating the MOTION window from the AT-REST window.

    Each step allows _TRAVEL_MS to move, then watches for _REST_MS. Peak draw comes from the whole
    step (that is the surge worth knowing); the binding verdict comes ONLY from the rest window, where
    a healthy servo has stopped pulling and a fighting one has not.

    Returns:
        peak/mean mW above baseline, plus `holding` -- the number of steps still drawing at rest.
    """
    unit._apply(unit._neutral)
    await asyncio.sleep_ms(_SETTLE_MS)
    baseline = _power()
    peak, total, samples, holding, rest_peak = 0, 0, 0, 0, 0
    for target in targets:
        unit._apply(target)
        for elapsed in range(0, _TRAVEL_MS + _REST_MS, _SAMPLE_MS):
            rise = _power() - baseline
            peak = max(peak, rise)
            total += rise
            samples += 1
            if elapsed >= _TRAVEL_MS:        # the at-rest window: motion is long over by here
                rest_peak = max(rest_peak, rise)
                if rise > _HOLD_MW:
                    holding += 1
            await asyncio.sleep_ms(_SAMPLE_MS)
    unit._apply(unit._neutral)
    await asyncio.sleep_ms(_SETTLE_MS)
    return {'peak': peak, 'mean': total // max(samples, 1), 'holding': holding,
            'rest_peak': rest_peak, 'steps': len(targets), 'baseline': baseline, 'label': label}


async def run():
    board, source, _errors = config.load()
    print('config : %s   board: %s' % (source, board.get('board', {}).get('id')))

    """
    Judge every fin as the servo that is actually FITTED, not the one the config names.

    `mg90s` is a thin SG90 subclass differing in three constants, and two of them would corrupt this
    measurement: its probe floor is 600 mW against the SG90's 500, so a healthy SG90 drawing between the
    two reads as DEAD; and its slew gate assumes 0.1 s/60deg against the SG90's 0.15, so the driver
    commands steps faster than the horn can follow. A board whose config still says mg90s while an SG90
    is bolted on would produce a confident, wrong verdict -- which is the exact failure this whole script
    exists to avoid.
    """
    for component in board['components']:
        if component['name'] in _FITTED and component.get('driver') != _FITTED[component['name']]:
            print('NOTE: %s declared %r, judging it as %r (what is fitted on this rig)'
                  % (component['name'], component['driver'], _FITTED[component['name']]))
            component['driver'] = _FITTED[component['name']]
    flight = await main.bringup(board, log=lambda line: None)
    await asyncio.sleep_ms(800)

    if databoard.Databoard.value('power') is None:
        print('NOTE: no INA226 on this airframe -- travel is still exercised, but the draw column')
        print('      will read 0 and cannot tell a live servo from a lost PWM pin.')

    results = []
    for name in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right'):
        unit = flight.active(name)
        if unit is None:
            print('%-20s NOT UP (disabled or failed setup)' % name)
            continue
        span = unit._max_deg - unit._min_deg
        targets = [unit._min_deg + (span * step) // (_STEPS - 1) for step in range(_STEPS)]
        targets += list(reversed(targets))          # and back, so a one-way binding is visible
        print('>>> %s  sweeping %d..%d deg (trim %s) -- WATCH THIS FIN'
              % (name, unit._min_deg, unit._max_deg, unit._trim_deg))
        results.append(await _sweep(unit, name, targets))

    print()
    print('%-20s %10s %10s %10s  %s' % ('surface', 'peak mW', 'mean mW', 'baseline', 'verdict'))
    for row in results:
        if row['peak'] < 100:
            verdict = '*** NO DRAW -- dead servo / lost PWM pin / unpowered rail ***'
        elif row['holding'] > row['steps'] // 4:
            verdict = '*** BINDING: still pulling %d mW at rest on %d/%d steps ***' % (
                row['rest_peak'], row['holding'], row['steps'])
        else:
            verdict = 'moves and SETTLES (rest draw %d mW)' % row['rest_peak']
        print('%-20s %10d %10d %10d  %s' % (row['label'], row['peak'], row['mean'], row['baseline'], verdict))

    """
    The ELERONS are the comparison that matters: they are the same part doing a mirrored job, so their
    draw should match. A large gap is one horn on the wrong spline, one linkage fouling the body, or one
    servo simply weaker -- none of which a per-servo pass/fail can see, because each one passes alone.
    """
    pair = [row for row in results if 'eleron' in row['label']]
    if len(pair) == 2 and min(row['peak'] for row in pair) > 0:
        low, high = sorted(row['peak'] for row in pair)
        ratio = high / low
        print()
        print('eleron symmetry: %d vs %d mW peak -> %.2fx  %s'
              % (high, low, ratio,
                 'matched' if ratio < 1.5 else '*** ASYMMETRIC: check horn spline / linkage / binding ***'))
    print('DONE')

asyncio.run(run())
