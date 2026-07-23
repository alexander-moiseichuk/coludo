"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the SDP810 driver (drivers/sdp810.py): @task.driver('sdp810') registration, graceful
setup when the bus is undefined, the @viper Sensirion CRC-8, the integer raw -> dynamic-pressure fixnum
scaling + pad tare, and the airspeed = sqrt(2q/rho) conversion the driver derives once per read.
Deterministic whether or not an SDP810 is wired. The fusion of that airspeed (the estimator's direct
source + the governor's in-band/saturation gate) is tested in test_airspeed / test_governor. Run by
`make test`.
"""

import asyncio
import struct

import config_default
import fixed
import task
from drivers import sdp810


class _StubController:
    config = config_default.default()


def _frame(dp_raw, temp_raw=5000, scale=60):
    """Build a valid 9-byte SDP8xx frame (DP, Temp, Scale each + a correct CRC) for the scaling tests."""
    body = bytearray(9)
    body[0:2] = struct.pack('>h', dp_raw)
    body[3:5] = struct.pack('>h', temp_raw)
    body[6:8] = struct.pack('>H', scale)
    for base in (0, 3, 6):
        body[base + 2] = sdp810._crc8(body[base], body[base + 1])  # @viper crc over the word's two bytes
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
    assert sdp810._crc8(0xBE, 0xEF) == 0x92
    good = _frame(8100)  # +135 Pa at scale 60
    assert sdp810._frame_ok(good)
    bad = bytearray(good)
    bad[1] ^= 0x01  # corrupt the DP word -> CRC must reject
    assert not sdp810._frame_ok(bytes(bad))
    assert not sdp810._frame_ok(good[:8])  # short frame -> rejected

    # signed-16 field decode + integer scaling: raw 8100 / scale 60 -> 135.00 Pa -> 13500 fixnum
    assert sdp810._signed16(0x1f, 0xa4) == 8100 and sdp810._signed16(0xfd, 0xa8) == -600
    probe = sdp810.Sdp810('airspeed', {}, _StubController())
    probe._scale, probe._zero, probe._density = 60, 0, 1.225
    assert probe._pressure(8100) == 13500 and abs(fixed.to_float(probe._pressure(8100)) - 135.0) < 0.01
    assert probe._pressure(-600) == -1000  # -10 Pa; a negative (reverse/near-zero flow) reading is kept signed

    # the ONE float: airspeed = sqrt(2q/rho); 135 Pa -> ~14.85 m/s, and a negative q clamps to 0 (no root)
    assert 14.5 < probe._airspeed(13500) < 15.5, probe._airspeed(13500)
    assert probe._airspeed(-1000) == 0.0 and probe._airspeed(0) == 0.0

    """
    Calibration: a pad tare captures the at-rest bias (a Pa fixnum) so a still glider reads 0; a direct
    set applies a Pa offset; air_density is the q->v span knob. update() reports the changed names (CC).
    """
    probe._pressure(200)  # +200 raw -> _raw = 333 fixnum -> becomes the tare source
    assert probe.update({'zero': True}) == ['zero_offset_pa'] and probe._zero == 333  # 200*100//60
    assert probe._pressure(200) == 0  # same reading, now tared to zero
    assert probe.update({'zero_offset_pa': 1.5}) == ['zero_offset_pa'] and probe._zero == fixed.from_float(1.5)
    assert probe.update({'air_density': 1.2}) == ['air_density'] and probe._density == 1.2

    print('ok: sdp810 driver registered; graceful-absent; @viper crc + Pa-fixnum scaling + airspeed')


asyncio.run(amain())
