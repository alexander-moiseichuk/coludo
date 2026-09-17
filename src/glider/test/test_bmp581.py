"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BMP581 driver (drivers/bmp581.py): @task.driver('bmp581') registration, graceful
setup when absent, and that the raw-to-engineering conversion is right in BOTH signs. Deterministic
whether or not a BMP581 is wired (it is fitted only on v1.1 boards). Run by `make test`.
"""

import asyncio

import commons
import config_default
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

    print('ok: bmp581 registered; graceful-absent; %.0f Pa / %.1f C, sub-zero %.1f C' % (
        pressure, temp_c, cold_c))


asyncio.run(amain())
