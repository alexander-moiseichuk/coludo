"""
Are the icp10111 read timeouts contention, or the part? Count them alone vs with its i2c:0 peers.

OSError(116) = ETIMEDOUT appeared in bench runs. A healthy part on a quiet bus should never time out,
so this isolates the variable: the SAME driver, same rate, with the other four i2c:0 devices (bno055,
bmp280, sdp810, laser) enabled and then disabled. `errors` is per 30 s of run loop.
"""

import asyncio

import config_default
import controller
import drivers
import recorder
import tasks

_PEERS = ('imu_bno055', 'baro_bmp280', 'airspeed_sdp810', 'laser_agl')


class FakeUart:
    """A drain-able sink so telemetry works: without it EVERY driver's push raises and note() logs
    it, which counted 249 AttributeErrors as I2C failures on the first attempt at this."""

    async def drain(self):
        pass

    def write(self, data):
        return len(data)
_SECONDS = 30


async def measure(with_peers: bool) -> tuple:
    cfg = config_default.default()
    for component in cfg['components']:
        if component['name'] in ('flight', 'sequencer', 'hitl', 'wifi', 'cc', 'watchdog'):
            component['enabled'] = False
    recorder.Recorder.setup(cfg, FakeUart())
    for sensor in cfg['sensors']:
        if sensor['name'] in _PEERS and not with_peers:
            sensor['enabled'] = False
    board = controller.Controller(cfg, log=lambda *a: None)
    await board.setup()
    await board.start()
    imu = board.tasks.get('baro_icp10111')
    if imu is None:
        return (0, 0, 'icp10111 failed setup')
    await asyncio.sleep_ms(500)

    """
    Count from the driver's OWN run loop, never by calling _read() alongside it: two interleaved
    measure/read sequences on one address NAK each other, which is a property of the test, not the bus
    (it produced a fictional 20.8 % error rate before this was fixed). note() is the driver's single
    error path, so wrapping it counts exactly what production would log.
    """
    tally = {'ok': 0, 'errs': 0, 'seen': {}}
    original = imu.note

    def counting_note(template=None, arg=None):
        if template is None:
            tally['ok'] += 1
        else:
            tally['errs'] += 1
            key = '%s(%s)' % (type(arg).__name__, getattr(arg, 'errno', ''))
            tally['seen'][key] = tally['seen'].get(key, 0) + 1
        return original(template, arg)

    imu.note = counting_note
    await asyncio.sleep_ms(_SECONDS * 1000)
    imu.note = original
    for task in board.tasks.values():
        await task.finish()
    return (tally['ok'], tally['errs'], tally['seen'])


async def main():
    drivers.load()
    tasks.load()
    for with_peers in (True, False):
        ok, errs, seen = await measure(with_peers)
        total = ok + errs
        print('  peers %-3s : ok %4d  errors %3d  (%.1f %%)  %s'
              % ('ON' if with_peers else 'OFF', ok, errs,
                 100.0 * errs / total if total else 0.0, seen))
        await asyncio.sleep_ms(500)

asyncio.run(main())
