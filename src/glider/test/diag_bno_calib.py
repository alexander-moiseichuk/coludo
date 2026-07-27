"""Watch BNO055 calibration converge and the fusion start moving. Move the board in a slow figure-8."""

import asyncio

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
    await asyncio.sleep_ms(800)
    imu = board.tasks['imu_bno055']
    print('  MOVE THE BOARD in a slow figure-8 -- watching 40 s')
    print('  %-6s %-28s %-22s %s' % ('t', 'calib (sys gyr acc mag)', 'euler bytes', 'moving?'))
    last = None
    for step in range(40):
        calib = (await imu._bus.read(imu._addr, 0x35, 1))[0]
        eul = await imu._bus.read(imu._addr, 0x1A, 6)
        raw = ' '.join('%02X' % b for b in eul)
        print('  %-6s sys %d gyr %d acc %d mag %d        %-22s %s'
              % ('%ds' % step, calib >> 6, (calib >> 4) & 3, (calib >> 2) & 3, calib & 3,
                 raw, 'MOVING' if (last is not None and raw != last) else 'frozen'))
        last = raw
        await asyncio.sleep_ms(1000)

asyncio.run(main())
