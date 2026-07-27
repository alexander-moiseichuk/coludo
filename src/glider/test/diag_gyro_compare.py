"""BNO055's own gyro vs the INDEPENDENT LSM6DSO32 (SPI). Move the board -- both should react."""

import asyncio
import struct

import config_default
import controller
import databoard
import drivers
import fixed
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
    imu = board.tasks['imu_bno055']
    rate = databoard.Databoard.parameter('rate')
    print('  MOVE THE BOARD -- 20 s. BNO055 gyro (its own) vs LSM6DSO32 rate (independent, SPI)')
    for step in range(10):
        await asyncio.sleep_ms(2000)
        gx, gy, gz = struct.unpack_from('<hhh', imu._buf, 12)
        bno = abs(gx) + abs(gy) + abs(gz)
        value, source, _age = rate.read()
        lsm = 0.0
        if value is not None:
            lsm = sum(abs(fixed.to_float(v)) for v in value)
        print('    %2ds  BNO055 gyro|sum|=%-6d (LSB, ~16/dps)   LSM6DSO32 |rate|=%.1f dps  [%s]'
              % ((step + 1) * 2, bno, lsm, source))

asyncio.run(main())
