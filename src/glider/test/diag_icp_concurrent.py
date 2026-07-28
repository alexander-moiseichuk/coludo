"""The ORIGINAL bad methodology on purpose: poll _read() alongside the driver's own run loop.

That produced a 20.8 % failure rate before _read() was serialised -- two measure/read conversations
interleaving on one device. With the guard the competing caller must now be served from the in-flight
result instead, so this should read ~0 %.
"""

import asyncio
import time

import config_default
import controller
import drivers
import recorder
import tasks


class FakeUart:
    async def drain(self):
        pass

    def write(self, data):
        return len(data)


async def main():
    drivers.load()
    tasks.load()
    cfg = config_default.default()
    for component in cfg['components']:
        if component['name'] in ('flight', 'sequencer', 'hitl', 'wifi', 'cc', 'watchdog'):
            component['enabled'] = False
    recorder.Recorder.setup(cfg, FakeUart())
    board = controller.Controller(cfg, log=lambda *a: None)
    await board.setup()
    await board.start()
    imu = board.tasks.get('baro_icp10111')
    await asyncio.sleep_ms(500)
    ok = errs = 0
    seen = {}
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 30000:
        await asyncio.sleep_ms(20)
        try:
            await imu._read()          # deliberately concurrent with run()
            ok += 1
        except Exception as error:
            errs += 1
            key = '%s(%s)' % (type(error).__name__, getattr(error, 'errno', ''))
            seen[key] = seen.get(key, 0) + 1
    print('  concurrent _read(): ok %4d  errors %3d  (%.1f %%)  %s'
          % (ok, errs, 100.0 * errs / (ok + errs) if (ok + errs) else 0, seen))

asyncio.run(main())
