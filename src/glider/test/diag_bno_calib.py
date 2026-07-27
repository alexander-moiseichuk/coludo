"""Watch BNO055 calibration converge and the fusion start moving. Move the board in a slow figure-8."""

import asyncio
import struct

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
    print('  %-5s %-26s %-9s %-20s %s'
          % ('t', 'calib (sys gyr acc mag)', 'gyro dps', 'euler bytes', 'euler'))
    last = None
    for step in range(40):
        calib = (await imu._bus.read(imu._addr, 0x35, 1))[0]
        eul = await imu._bus.read(imu._addr, 0x1A, 6)
        # the GYRO is printed so a 'frozen euler' verdict can never rest on an UNVERIFIED assumption
        # that the board was being moved -- that assumption invalidated an earlier faulty/healthy call
        gyro = await imu._bus.read(imu._addr, 0x14, 6)
        gx, gy, gz = struct.unpack('<hhh', gyro)
        dps = (abs(gx) + abs(gy) + abs(gz)) / 16.0
        raw = ' '.join('%02X' % b for b in eul)
        print('  %-5s sys %d gyr %d acc %d mag %d       %-9.1f %-20s %s'
              % ('%ds' % step, calib >> 6, (calib >> 4) & 3, (calib >> 2) & 3, calib & 3,
                 dps, raw, ('MOVING' if (last is not None and raw != last) else 'frozen')
                 + ('' if dps > 5.0 else '  (STILL -- proves nothing)')))
        last = raw
        await asyncio.sleep_ms(1000)

asyncio.run(main())
