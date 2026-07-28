"""Why is LSM6DSO32 INT1 silent? Read back its routing/ODR registers and watch the GPIO toggle."""

import asyncio
import time

import config_default
import controller
import drivers
import tasks
from machine import Pin

_CTRL1_XL = 0x10   # accel ODR/scale
_CTRL2_G = 0x11    # gyro ODR/scale
_INT1_CTRL = 0x0D  # INT1 routing
_STATUS = 0x1E     # data-ready flags


async def main():
    drivers.load()
    tasks.load()
    cfg = config_default.default()
    for component in cfg['components']:
        if component['name'] in ('flight', 'sequencer', 'hitl', 'wifi', 'cc', 'watchdog'):
            component['enabled'] = False
    board = controller.Controller(cfg, log=lambda *a: None)
    await board.setup()
    await board.start()
    await asyncio.sleep_ms(1500)

    imu = board.tasks.get('imu_lsm6dso32')
    dev = imu._dev
    for name, reg in (('CTRL1_XL (accel ODR)', _CTRL1_XL), ('CTRL2_G (gyro ODR)', _CTRL2_G),
                      ('INT1_CTRL (routing)', _INT1_CTRL), ('STATUS', _STATUS)):
        raw = await dev.read(reg, 1)
        print('  %-22s = 0x%02X' % (name, raw[0]))

    print('\n  STATUS over 1 s (bit0 XLDA, bit1 GDA -- should flicker if data is flowing):')
    seen = set()
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 1000:
        raw = await dev.read(_STATUS, 1)
        seen.add(raw[0])
        await asyncio.sleep_ms(5)
    print('    distinct STATUS values: %s' % sorted('0x%02X' % v for v in seen))

    gpio = imu._pin_gpio('int_pin')
    print('\n  int_pin GPIO = %s' % gpio)
    if gpio is not None:
        pin = Pin(gpio, Pin.IN)
        levels = set()
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 1000:
            levels.add(pin.value())
        print('    levels seen in 1 s: %s  ->  %s' % (sorted(levels),
              'TOGGLING' if len(levels) > 1 else 'STUCK -- no edges, so the IRQ can never fire'))

asyncio.run(main())
