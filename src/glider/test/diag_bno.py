"""Is the BNO055 fused attitude actually moving, or frozen? Read the device directly + its calib state."""

import asyncio

import config_default
import controller
import drivers
import recorder
import tasks

_CALIB_STAT = 0x35  # bits: sys[7:6] gyr[5:4] acc[3:2] mag[1:0], 3 = fully calibrated
_OPR_MODE = 0x3D


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
    await asyncio.sleep_ms(1500)

    imu = board.tasks.get('imu_bno055')
    if imu is None:
        print('  bno055 absent / failed setup')
        return
    mode = (await imu._bus.read(imu._addr, _OPR_MODE, 1))[0]
    calib = (await imu._bus.read(imu._addr, _CALIB_STAT, 1))[0]
    print('  OPR_MODE = 0x%02X (0x0C = NDOF fusion)' % mode)
    print('  CALIB_STAT = 0x%02X -> sys %d gyr %d acc %d mag %d (3 = calibrated)'
          % (calib, calib >> 6, (calib >> 4) & 3, (calib >> 2) & 3, calib & 3))

    print('\n  ten DIRECT samples 200 ms apart (move the board if they never change):')
    for _ in range(10):
        sample = await imu.sample()
        print('    %s' % (tuple(round(v, 3) if isinstance(v, float) else v for v in sample),))
        await asyncio.sleep_ms(200)

asyncio.run(main())
