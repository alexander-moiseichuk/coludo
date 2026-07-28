"""BNO055 self-report: what does the chip say about its own fusion engine?"""
import asyncio

import config_default
import controller
import drivers
import recorder
import tasks

_ST_RESULT = 0x36   # bit0 acc, bit1 mag, bit2 gyr, bit3 mcu -- 1 = self-test PASSED
_SYS_STATUS = 0x39  # 5 = running WITH fusion, 6 = running WITHOUT fusion
_SYS_ERR = 0x3A
_OPR_MODE = 0x3D
_CHIP_ID = 0x00
_STATUS = {0: 'idle', 1: 'SYSTEM ERROR', 2: 'initialising peripherals', 3: 'system initialisation',
           4: 'executing self-test', 5: 'running WITH fusion', 6: 'running WITHOUT fusion'}
_ERRS = {0: 'no error', 1: 'peripheral init error', 2: 'system init error', 3: 'self-test FAILED',
         4: 'register map value out of range', 5: 'register map address out of range',
         6: 'register map write error', 7: 'low-power mode not available',
         8: 'accel power mode not available', 9: 'fusion config error', 10: 'sensor config error'}


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
    imu = board.tasks['imu_bno055']

    async def reg(address):
        return (await imu._bus.read(imu._addr, address, 1))[0]

    print('  CHIP_ID    = 0x%02X (expect 0xA0)' % await reg(_CHIP_ID))
    print('  OPR_MODE   = 0x%02X' % await reg(_OPR_MODE))
    status, err, st = await reg(_SYS_STATUS), await reg(_SYS_ERR), await reg(_ST_RESULT)
    print('  SYS_STATUS = %d  -> %s' % (status, _STATUS.get(status, '?')))
    print('  SYS_ERR    = %d  -> %s' % (err, _ERRS.get(err, '?')))
    print('  ST_RESULT  = 0x%02X -> acc %d mag %d gyr %d mcu %d  (1 = passed)'
          % (st, st & 1, (st >> 1) & 1, (st >> 2) & 1, (st >> 3) & 1))

asyncio.run(main())
