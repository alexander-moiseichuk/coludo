"""Does the fusion-stall detector CATCH a known-faulty part under motion? Keep the board moving."""

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
    imu = board.tasks['imu_bno055']
    attitude = __import__('databoard').Databoard.parameter('attitude')
    print('  KEEP MOVING the board -- 25 s, watching the driver\'s own run loop')
    for step in range(5):
        await asyncio.sleep_ms(5000)
        value, source, age = attitude.read()
        # report the gyro too: frozen_count only advances while the part is actually ROTATING, so a
        # zero count with a still board means the detector correctly abstained, not that it missed
        import struct
        gx, gy, gz = struct.unpack_from('<hhh', imu._buf, 12)
        print('    %2ds  gyro|sum|=%-5d  fusion_stalled=%-5s  frozen=%-4s  source=%s'
              % ((step + 1) * 5, abs(gx) + abs(gy) + abs(gz), imu._stalled, imu._frozen, source))
    print('\n  VERDICT: %s' % ('detector CAUGHT it -- attitude withheld, backup carries it'
                               if imu._stalled else 'detector did NOT fire'))

asyncio.run(main())
