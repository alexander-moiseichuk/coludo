"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BNO055 driver (drivers/bno055.py): @task.driver('bno055') registration and
graceful setup when the device is absent. Deterministic whether or not a BNO055 is wired (it probes a
bus/address with nothing on it). Run by `make test`.
"""

import asyncio
import struct

import config_default
import task
from drivers import bno055


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('bno055') is bno055.Bno055  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bno055.Bno055('imu', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bno055.Bno055('imu', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    """
    FUSION-STALL detector, both directions, deterministically -- no one has to wave a breadboard.

    The failure it exists for: a BNO055 whose fusion core latches while its raw sensors keep streaming.
    The channel stays FRESH, so every staleness guard passes it and the priority-1 attitude backup never
    engages; the PID would be handed a constant. Measured on a real faulty part, which held a
    bit-identical Euler triple through 40 s of motion with mag calibration stuck at 0.

    The gate is ROTATION, from the part's own gyro in the same block read. A stationary BNO055
    legitimately repeats its output, so an earlier version keyed on accel dither fired on a HEALTHY
    sensor -- which on the pad would have withheld attitude and launched the glider on the backup.
    """
    detector = bno055.Bno055('imu', {}, _StubController())
    detector._buf = bytearray(24)
    detector._last_euler = None
    detector._frozen = 0
    detector._stalled = False

    def turning(dps):
        """Write a rotation rate into the gyro slot of the block (16 LSB per deg/s)."""
        struct.pack_into('<hhh', detector._buf, 12, int(dps * 16), 0, 0)

    frozen = (148.5, 837, -16875)           # the exact triple the faulty part latched at
    turning(0.0)                            # STILL: a repeated reading proves nothing
    for _ in range(500):
        assert detector._fusion_alive(frozen + (0.1, 0.2, 0.9)) is True
    assert detector._frozen == 0, 'a still part must never accumulate toward a stall'

    turning(30.0)                           # ROTATING, yet the fusion output does not move
    alive = [detector._fusion_alive(frozen + (0.1, 0.2, 0.9)) for _ in range(60)]
    assert alive[0] is True and alive[-1] is False, 'a frozen fusion under rotation must be caught'
    # first False at 49, not 50: the still-loop above already primed _last_euler, so the rotating
    # sequence starts counting on its FIRST call rather than spending one on the comparison
    assert alive.index(False) == 49, alive.index(False)  # exactly _STALL_SAMPLES rotating reads

    # a fusion that DOES move under rotation is never flagged, and clears any partial count
    detector._frozen = 40
    assert detector._fusion_alive((149.0, 840, -16870) + (0.1, 0.2, 0.9)) is True
    assert detector._frozen == 0

    print('ok: bno055 driver registered; setup fails gracefully when no device answers; '
          'fusion-stall detector fires under rotation, never when still')


asyncio.run(amain())
