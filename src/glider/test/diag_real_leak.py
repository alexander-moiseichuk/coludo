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

_SECONDS = 40


async def _go() -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_default.default()
    for component in cfg['components']:            # no flight loop, no watchdog: sensors + recorder only
        # cc_link OFF too: with a hub reachable it reconnect-loops (OSError 104) and its churn
        # -- plus the collects it provoked -- swamped the very slope being measured
        if component['name'] in ('flight', 'watchdog', 'hitl', 'cc', 'cc_link', 'wifi'):
            component['enabled'] = False
    board = controller.Controller(cfg, log=lambda message: None)
    await board.setup()
    await board.start()
    print('SETUP: %d tasks running (real drivers, no sim)' % len(board.tasks))
    await asyncio.sleep_ms(3000)                   # let every driver reach steady state first
    gc.collect()
    gc.disable()                                   # the airborne policy, forced here on the bench
    start, first = time.ticks_ms(), gc.mem_free()
    print('%6s %12s %12s' % ('t_s', 'free_KB', 'KB/s'))
    while True:
        await asyncio.sleep_ms(4000)
        now, free = time.ticks_ms(), gc.mem_free()
        seconds = time.ticks_diff(now, start) / 1000.0
        if free > first:  # a collect ran despite gc.disable() -> the sample is not measuring a leak
            print('%6.0f %12d   (heap ROSE -- collect, sample discarded)' % (seconds, free // 1024))
            continue
        print('%6.0f %12d %12.1f' % (seconds, free // 1024, (first - free) / 1024.0 / seconds))
        if seconds >= _SECONDS:
            break
    gc.enable()
    slope = (first - gc.mem_free()) / 1024.0 / seconds
    print('BOARD-ONLY LEAK %.1f KB/s -> OOM in %.0f s from a %d KB heap'
          % (slope, (first // 1024) / slope if slope > 0 else -1, first // 1024))
    await board.finish()


asyncio.run(_go())
