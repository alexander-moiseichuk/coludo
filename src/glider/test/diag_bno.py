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

    for name, address in (('SYS_STATUS', 0x39), ('SYS_ERR', 0x3A), ('ST_RESULT', 0x36),
                          ('SYS_TRIGGER', 0x3F)):
        print('  %-12s = 0x%02X' % (name, (await imu._bus.read(imu._addr, address, 1))[0]))
    print('  (SYS_STATUS 5 = running WITH fusion, 6 = without; ST_RESULT 0x0F = all self-tests passed)')

    """
    The RAW block is what settles a frozen-attitude argument: acc / mag / gyr / eul are four fields of
    ONE 24-byte read, so if acc and gyr bytes move while eul does not, the bus is fine and the fusion
    core is the fault -- no wiring explanation can be selective per register inside one transaction.
    """
    print('\n  raw ACC | MAG | GYR | EUL from one block read (move the board):')
    for _ in range(6):
        await imu._bus.read_into(imu._addr, 0x08, imu._buf)
        row = imu._buf
        print('    %s | %s | %s | %s'
              % (' '.join('%02X' % b for b in row[0:6]), ' '.join('%02X' % b for b in row[6:12]),
                 ' '.join('%02X' % b for b in row[12:18]), ' '.join('%02X' % b for b in row[18:24])))
        await asyncio.sleep_ms(250)
    print('\n  fusion_stalled flag: %s' % imu.inspect().get('fusion_stalled'))

asyncio.run(main())
