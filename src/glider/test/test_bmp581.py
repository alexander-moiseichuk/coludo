"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BMP581 driver (drivers/bmp581.py): @task.driver('bmp581') registration, graceful
setup when absent, and that the raw-to-engineering conversion is right in BOTH signs. Deterministic
whether or not a BMP581 is wired (the SEN0697: every revision but v1.0). Run by `make test`.
"""

import asyncio
import time

import commons
import config_default
import databoard
import task
from drivers import bmp581


class _StubController:
    config = config_default.default()


class _FakeBus:
    """Feeds _read() a known 6-byte frame without touching hardware."""

    def __init__(self, frame):
        self._frame = frame

    async def read_into(self, _addr, _reg, buf):
        for index in range(len(buf)):
            buf[index] = self._frame[index]


async def amain():
    assert task.ACTIVITIES.get('bmp581') is bmp581.Bmp581  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bmp581.Bmp581('baro', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bmp581.Bmp581('baro', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    """
    Conversion, POSITIVE case: the BMP581 reports 1/64 Pa and 1/65536 degC, both little-endian 24-bit.
    101325 Pa -> 6484800 counts; 25 degC -> 1638400 counts. No compensation to apply -- unlike the BMP280
    this part outputs compensated values, which is the one thing that makes this driver shorter.
    """
    probe = bmp581.Bmp581('baro', {}, _StubController())
    probe._buf = bytearray(6)
    probe._addr = 0x47
    probe._bus = _FakeBus(bytes((0x00, 0x00, 0x19,        # temperature 1638400 = 25.0 C
                                 0x00, 0xF6, 0x62)))      # pressure 6485504 = 101336 Pa
    altitude, temp_c, pressure = await probe._read()
    assert abs(temp_c - 25.0) < 0.01, temp_c
    assert abs(pressure - 101336.0) < 1.0, pressure
    assert abs(altitude - commons.altitude_m(pressure)) < 0.001, 'altitude must come from the shared helper'

    """
    NEGATIVE case, and the one worth having: temperature is TWO'S COMPLEMENT while pressure is unsigned.
    Reading a sub-zero temperature as unsigned yields ~+256 C, which is exactly the kind of wrong that
    still renders as a plausible number on a chart. -10 C is 0xFF60_0000 truncated to 24 bits = 0xF60000.
    """
    probe._bus = _FakeBus(bytes((0x00, 0x00, 0xF6,        # temperature -655360 = -10.0 C
                                 0x00, 0xF6, 0x62)))
    _altitude, cold_c, _pressure = await probe._read()
    assert abs(cold_c - -10.0) < 0.01, 'sub-zero temperature must sign-extend, got %r' % cold_c

    # a dead read (zero pressure) must not produce an altitude: the shared helper refuses it
    assert commons.altitude_m(0.0) == 0.0, 'zero pressure is a dead read, not sea level'


    """
    `update {"rezero": true}` re-zeroes from the driver's OWN channel, fresh within 3 periods. It called
    read() -- the Parameter's method, which a channel does not have -- so every re-zero raised
    AttributeError and the documented pre-launch re-zero answered `err internal`.
    """
    unit = bmp581.Bmp581('baro', {}, _StubController())
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

    print('ok: bmp581 registered; graceful-absent; re-zero fresh-only; %.0f Pa / %.1f C, sub-zero %.1f C' % (
        pressure, temp_c, cold_c))


asyncio.run(amain())
