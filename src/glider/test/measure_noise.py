"""
Measure the REAL sensor noise the control loop sees, standing still on the bench.

The sim perturbs every channel by +/- frac*(|value|+1) -- noise proportional to MAGNITUDE, with the
"5 % noise" studies as the standard hostile case. Nothing ever checked that against the actual parts,
and the shape is wrong for an angle (heading 350 is not more uncertain than heading 10). This samples
the fused databoard channels the flight loop reads and reports the 1-sigma spread in real units, so the
sim's noise can be set from measurement instead of a guess. Glider STILL and level while it runs.
"""

import asyncio
import time

import config_default
import controller
import databoard
import drivers
import fixed
import tasks

_SECONDS = 20


def _spread(samples: list) -> tuple:
    """(mean, 1-sigma) of a sample list; (0, 0) when empty."""
    if not samples:
        return 0.0, 0.0
    mean = sum(samples) / len(samples)
    variance = sum((value - mean) ** 2 for value in samples) / len(samples)
    return mean, variance ** 0.5


async def main():
    drivers.load()
    tasks.load()
    cfg = config_default.default()
    for component in cfg['components']:  # the control loop must NOT run -- we only want the sensors
        if component['name'] in ('flight', 'sequencer', 'hitl', 'wifi', 'cc', 'watchdog'):
            component['enabled'] = False
    board = controller.Controller(cfg, log=lambda *a: None)
    await board.setup()
    await board.start()
    await asyncio.sleep_ms(3000)  # let the drivers settle

    attitude = databoard.Databoard.parameter('attitude')
    rate = databoard.Databoard.parameter('rate')
    series = {name: [] for name in ('heading', 'roll', 'pitch', 'gx', 'gy', 'gz')}
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < _SECONDS * 1000:
        await asyncio.sleep_ms(20)
        value, source, _age = attitude.read()
        if source is not None and value is not None:
            series['heading'].append(value[0] if isinstance(value[0], float) else fixed.to_float(value[0]))
            series['roll'].append(fixed.to_float(value[1]))
            series['pitch'].append(fixed.to_float(value[2]))
        value, source, _age = rate.read()
        if source is not None and value is not None:
            for name, slot in (('gx', 0), ('gy', 1), ('gz', 2)):
                series[name].append(fixed.to_float(value[slot]))

    print('\nREAL sensor noise, %d s stationary (1-sigma):' % _SECONDS)
    for name in ('heading', 'roll', 'pitch'):
        mean, sigma = _spread(series[name])
        equivalent = sigma / (abs(mean) + 1.0) if series[name] else 0.0
        print('  %-8s n=%4d  mean %8.2f deg   1-sigma %6.3f deg   == sim frac %.4f'
              % (name, len(series[name]), mean, sigma, equivalent))
    for name in ('gx', 'gy', 'gz'):
        mean, sigma = _spread(series[name])
        print('  %-8s n=%4d  mean %8.2f dps   1-sigma %6.3f dps' % (name, len(series[name]), mean, sigma))
    print('\n  the sim\'s "5 %% noise" = frac 0.05; compare the `sim frac` column above\n')

asyncio.run(main())
