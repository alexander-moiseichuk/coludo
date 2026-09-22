"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BMP280 driver (drivers/bmp280.py): @task.driver('bmp280') registration,
graceful setup when absent, and that the Bosch compensation is sane. Deterministic whether or not a
BMP280 is wired. Run by `make test`.
"""

import asyncio
import time

import config_default
import databoard
import task
from drivers import bmp280


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('bmp280') is bmp280.Bmp280  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bmp280.Bmp280('baro', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bmp280.Bmp280('baro', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    # compensation against the Bosch datasheet reference calibration + raw values -> ~100653 Pa.
    # (dig_T1..P9 and adc_t/adc_p from the BMP280 datasheet worked example.)
    probe = bmp280.Bmp280('baro', {}, _StubController())
    probe._cal = (27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000)
    pa, temp_c = probe._compensate(519888, 415148)
    assert 99000.0 < pa < 102000.0, pa  # datasheet expects ~100653 Pa
    assert 20.0 < temp_c < 30.0, temp_c  # datasheet worked example ≈ 25.08 °C


    """
    `update {"rezero": true}` re-zeroes from the driver's OWN channel, fresh within 3 periods. It called
    read() -- the Parameter's method, which a channel does not have -- so every re-zero raised
    AttributeError and the documented pre-launch re-zero answered `err internal`.
    """
    unit = bmp280.Bmp280('baro', {}, _StubController())
    unit._period_ms = 100
    unit._altitude = databoard._Channel('baro', 0)  # born stale: never pushed
    unit._ground = 7.0
    for label in ('never pushed', 'silent for 4 periods'):
        try:
            unit.update({'rezero': True})
            raise AssertionError('re-zero accepted from a channel that is %s' % label)
        except ValueError as error:
            assert 'no fresh altitude' in str(error), error
        assert unit._ground == 7.0, 'a refused re-zero must leave the ground untouched'
        unit._altitude.push(123.5)
        assert unit.update({'rezero': True}) == ['ground'] and unit._ground == 123.5
        unit._ground = 7.0
        unit._altitude.t1 = time.ticks_add(time.ticks_us(), -400000)  # the part goes quiet

    print('ok: bmp280 registered; graceful-absent; compensation ~%d Pa / %.1f C; re-zero fresh-only' % (pa, temp_c))


asyncio.run(amain())
