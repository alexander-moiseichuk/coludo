"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BMI323 driver (drivers/bmi323.py): @task.driver('bmi323') registration, graceful
setup when absent, the TWO DUMMY BYTES every I2C read prepends, exact gyro scaling in both signs, and the
0x8000 start-up marker. Deterministic whether or not a BMI323 is wired (the SEN0697: every revision but v1.0).
Run by `make test`.
"""

import asyncio

import config_default
import task
from drivers import bmi323
from fixed import SCALE


class _StubController:
    config = config_default.default()


class _FakeBus:
    """Feeds _read() a known 14-byte frame (2 dummy + 6 words) without touching hardware."""

    def __init__(self, frame):
        self._frame = frame

    async def read_into(self, _addr, _reg, buf):
        for index in range(len(buf)):
            buf[index] = self._frame[index]


def _frame(ax, ay, az, gx, gy, gz):
    """A BMI323 I2C read: two dummy bytes, then six little-endian int16."""
    out = bytearray(b'\xA5\x5A')  # deliberately NOT zero: reading from offset 0 must corrupt the result
    for value in (ax, ay, az, gx, gy, gz):
        out += bytes(((value & 0xFF), (value >> 8) & 0xFF))
    return bytes(out)


async def amain():
    assert task.ACTIVITIES.get('bmi323') is bmi323.Bmi323  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bmi323.Bmi323('imu', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bmi323.Bmi323('imu', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    probe = bmi323.Bmi323('imu', {}, _StubController())
    probe._buf = bytearray(14)
    probe._addr = 0x69

    """
    Scaling, and the dummy bytes. +/-16 g is 2048 counts per g, so 1 g rests at 2048; +/-2000 dps in
    centideg/s is raw * 2000 * SCALE / 32768 exactly. The frame's first two bytes are 0xA5 0x5A: if the
    driver ever reads from offset 0 instead of 2 the accel comes back as garbage rather than gravity,
    which is the failure this part's I2C quirk actually produces.
    """
    probe._bus = _FakeBus(_frame(0, 0, 2048, 1000, -1000, 0))
    ax, ay, az, gx, gy, gz = await probe._read()
    assert ax == 0.0 and ay == 0.0, (ax, ay)
    assert abs(az - 1.0) < 1e-6, 'z should read exactly 1 g at 2048 counts, got %r' % az
    assert gx == 1000 * 2000 * SCALE // 32768, gx
    assert gy == -1000 * 2000 * SCALE // 32768, gy
    assert gz == 0, gz
    assert gx > 0 and gy < 0, 'the gyro must keep its sign through the fixnum conversion'

    # full scale, both ends: -32768 is a legitimate -16 g rail here, not a marker
    probe._bus = _FakeBus(_frame(32767, -32768, 0, 32767, -32768, 0))
    ax, ay, _az, gx, gy, _gz = await probe._read()
    assert 15.9 < ax < 16.0, ax
    assert abs(ay - -16.0) < 1e-6, 'the negative rail is -16 g exactly, got %r' % ay
    assert gx == 199993 and gy == -200000, (gx, gy)  # +/-2000 dps in centideg/s

    """
    NEGATIVE: the gyro reports 0x8000 until it has started, and that scales to a full -2000 dps -- a value
    the PID's D term would act on. _await_gyro must refuse to finish while the marker is present, so a
    part stuck there fails setup() instead of publishing a phantom rotation.
    """
    probe._bus = _FakeBus(_frame(0, 0, 2048, -32768, -32768, -32768))
    assert await probe._await_gyro() is False, 'a gyro stuck at 0x8000 must not be reported as ready'
    probe._bus = _FakeBus(_frame(0, 0, 2048, 5, -5, 5))
    assert await probe._await_gyro() is True, 'real samples must satisfy the wait'

    print('ok: bmi323 registered; graceful-absent; dummy-byte offset, +/-16 g rails, '
          'exact centideg/s both signs, 0x8000 start-up refused')


asyncio.run(amain())
