"""
Why does `rate` publish at ~1 Hz and `attitude` read frozen? Diagnose the real sensor path.

measure_noise.py reported attitude 1-sigma of exactly 0.000 deg (a real sensor never does that) and
only 24 `rate` samples in 20 s where the LSM6DSO32 should give ~100. This lists who PROVIDES each
control channel, how often each source actually pushes (by watching its stamp advance), and what each
driver says about itself -- so the answer is measured, not inferred.
"""

import asyncio
import time

import config_default
import controller
import databoard
import drivers
import tasks

_SECONDS = 10


async def main():
    drivers.load()
    tasks.load()
    cfg = config_default.default()
    for component in cfg['components']:  # sensors only -- no control loop, no radios
        if component['name'] in ('flight', 'sequencer', 'hitl', 'wifi', 'cc', 'watchdog'):
            component['enabled'] = False
    board = controller.Controller(cfg, log=lambda *a: None)
    await board.setup()
    await board.start()
    await asyncio.sleep_ms(2000)

    print('\n-- setup verdicts (a driver that failed setup never publishes) --')
    for name, unit in sorted(board.tasks.items()):
        if any(key in name for key in ('imu', 'accel', 'baro', 'laser', 'gnss', 'airspeed')):
            print('  %-20s ok=%-5s healthy=%s' % (name, getattr(unit, '_ok', '?'), getattr(unit, '_healthy', '?')))

    print('\n-- who provides each control channel, and how fast does it really push --')
    for channel in ('attitude', 'rate', 'accel', 'altitude', 'airspeed'):
        param = databoard.Databoard.parameter(channel)
        if param is None or not param.channels:
            print('  %-10s NO PROVIDER' % channel)
            continue
        marks = {}
        for chan in param.channels:
            marks[chan.source] = [0, chan.t1]
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < _SECONDS * 1000:
            await asyncio.sleep_ms(5)
            for chan in param.channels:
                if chan.t1 != marks[chan.source][1]:
                    marks[chan.source][0] += 1
                    marks[chan.source][1] = chan.t1
        value, source, age = param.read()
        detail = ', '.join('%s %.1f Hz' % (name, count / _SECONDS) for name, (count, _t) in marks.items())
        print('  %-10s fused_source=%-12s age=%-6s | %s' % (channel, source, age, detail))

asyncio.run(main())
