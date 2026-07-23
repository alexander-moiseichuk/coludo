"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the SDP810 driver (drivers/sdp810.py): @task.driver('sdp810') registration, graceful
setup when the bus is undefined, the Sensirion CRC-8, and that the raw -> dynamic-pressure -> airspeed
conversion (with tare / span / density) is sane. Deterministic whether or not an SDP810 is wired.
Run by `make test`.
"""

import asyncio
import struct

import config_default
import task
from drivers import sdp810


class _StubController:
    config = config_default.default()


def _frame(dp_raw, temp_raw=5000, scale=60):
    """Build a valid 9-byte SDP8xx frame (DP, Temp, Scale each + a correct CRC) for the conversion tests."""
    body = bytearray(9)
    body[0:2] = struct.pack('>h', dp_raw)
    body[3:5] = struct.pack('>h', temp_raw)
    body[6:8] = struct.pack('>H', scale)
    for base in (0, 3, 6):
        body[base + 2] = sdp810._crc8(body, base)
    return bytes(body)


async def amain():
    assert task.ACTIVITIES.get('sdp810') is sdp810.Sdp810  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = sdp810.Sdp810('airspeed', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    """
    Sensirion CRC-8 (poly 0x31, seed 0xFF): the datasheet worked example 0xBEEF -> 0x92, and a
    round-trip through the frame builder must validate while a single flipped bit must not.
    """
    assert sdp810._crc8(b'\xbe\xef', 0) == 0x92
    good = _frame(8100)  # +135 Pa at scale 60
    assert sdp810._frame_ok(good)
    bad = bytearray(good)
    bad[1] ^= 0x01  # corrupt the DP word -> CRC must reject
    assert not sdp810._frame_ok(bytes(bad))
    assert not sdp810._frame_ok(good[:8])  # short frame -> rejected

    # conversion: +135 Pa dynamic pressure (raw 8100 / scale 60) -> ~15 m/s at ISA density
    probe = sdp810.Sdp810('airspeed', {}, _StubController())
    probe._scale, probe._density, probe._gain, probe._zero_pa = 60.0, 1.225, 1.0, 0.0
    pressure, airspeed, temp = probe._convert(_frame(8100, temp_raw=5000))
    assert abs(pressure - 135.0) < 0.5, pressure
    assert 14.5 < airspeed < 15.5, airspeed  # sqrt(2*135/1.225) ~ 14.85
    assert abs(temp - 25.0) < 0.1, temp  # 5000 / 200

    # negative dynamic pressure (reverse/near-zero flow) -> 0 m/s, never a complex root
    pressure, airspeed, _t = probe._convert(_frame(-600))  # -10 Pa
    assert pressure < 0.0 and airspeed == 0.0, (pressure, airspeed)

    """
    Calibration: a pad tare captures the at-rest bias so a still glider reads ~0 Pa / 0 m/s, and the
    span trim scales the reading. update() reports the changed property names for the CC round-trip.
    """
    probe._convert(_frame(120))  # +2 Pa at rest -> becomes the tare source (_raw_pa)
    assert probe.update({'zero': True}) == ['zero_offset_pa']
    pressure, airspeed, _t = probe._convert(_frame(120))  # same reading, now tared
    assert abs(pressure) < 0.01 and airspeed == 0.0, (pressure, airspeed)
    assert probe.update({'pressure_scale': 1.05}) == ['pressure_scale'] and probe._gain == 1.05

    print('ok: sdp810 driver registered; graceful-absent; crc + conversion ~%.0f Pa / %.1f m/s' % (135.0, 14.85))


asyncio.run(amain())
