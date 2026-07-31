"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BOARD-ONLY leak: the real drivers, no HITL sim at all.

The published figure of 187 KB/s is the intercept of leak-vs-inject_hz, i.e. HITL with the sim's
PUBLISHING removed -- but the sim still steps sim_model's float physics at 50 Hz underneath it, and a
real flight has allocation the HITL does not (ten drivers doing I2C/SPI reads, NMEA string parsing).
The two are not nested, so neither bounds the other. This runs the DEFAULT config -- real sensors,
no sim -- with GC forced off, and measures the slope that a real flight would actually see.

    mpremote run test/diag_real_leak.py
"""

import asyncio
import gc
import time

import config_default
import controller
import drivers
import mission
import tasks

_SECONDS = 24  # long enough for the slope to converge (it settles by ~20 s), short enough to sweep


async def _go(off=()) -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_default.default()
    for component in cfg['components']:            # no flight loop, no watchdog: sensors + recorder only
        # cc_link OFF always: with a hub reachable it reconnect-loops (OSError 104) and its churn
        # -- plus the collects it provoked -- swamped the very slope being measured. watchdog off so a
        # deliberate leak cannot reset the board mid-measurement.
        if component['name'] in ('watchdog', 'hitl', 'cc', 'cc_link', 'wifi') or component['name'] in off:
            component['enabled'] = False
    for sensor in cfg['sensors']:
        if sensor['name'] in off:
            sensor['enabled'] = False
    board = controller.Controller(cfg, log=lambda message: None)
    await board.setup()
    await board.start()
    print('SETUP %d tasks | disabled: %s' % (len(board.tasks), ','.join(off) or 'nothing'))
    await asyncio.sleep_ms(3000)                   # let every driver reach steady state first
    gc.collect()
    gc.disable()                                   # the airborne policy, forced here on the bench
    start, first = time.ticks_ms(), gc.mem_free()
    while True:
        await asyncio.sleep_ms(4000)
        now, free = time.ticks_ms(), gc.mem_free()
        seconds = time.ticks_diff(now, start) / 1000.0
        if free > first:  # a collect ran despite gc.disable() -> the sample is not measuring a leak
            continue
        if seconds >= _SECONDS:
            break
    gc.enable()
    slope = (first - gc.mem_free()) / 1024.0 / seconds
    print('LEAK %7.1f KB/s | OOM %5.0f s | off: %s'
          % (slope, (first // 1024) / slope if slope > 0 else -1, ','.join(off) or 'nothing'))
    await board.finish()


def probe(off=()) -> None:
    """Measure the leak with `off` disabled; the DIFFERENCE against the baseline attributes it."""
    asyncio.run(_go(off))

