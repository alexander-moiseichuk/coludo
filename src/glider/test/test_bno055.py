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
    detector._stalled = False

    def turning(dps):
        """Write a rotation rate into the gyro slot of the block (16 LSB per deg/s)."""
        struct.pack_into('<hhh', detector._buf, 12, int(dps * 16), 0, 0)

    frozen = (148.5, 837, -16875)           # the exact triple the faulty part latched at
    turning(0.0)                            # STILL: a repeated reading proves nothing
    for _ in range(500):
        assert detector._fusion_alive(frozen + (0.1, 0.2, 0.9)) is True
    assert detector._strikes == 0, 'a still part must never accumulate toward a stall'

    turning(30.0)                           # ROTATING, yet the fusion output does not move
    alive = [detector._fusion_alive(frozen + (0.1, 0.2, 0.9)) for _ in range(60)]
    assert alive[0] is True and alive[-1] is False, 'a frozen fusion under rotation must be caught'
    # first False at 49, not 50: the still-loop above already primed _last_euler, so the rotating
    # sequence starts counting on its FIRST call rather than spending one on the comparison
    assert alive.index(False) == 49, alive.index(False)  # exactly _STALL_SAMPLES rotating reads

    # a fusion that DOES move under rotation is never flagged, clears any partial count AND releases
    # the latch -- without that release a recovered part stays withheld forever (caught in review)
    detector._strikes = 40
    assert detector._fusion_alive((149.0, 840, -16870) + (0.1, 0.2, 0.9)) is True
    assert detector._strikes == 0 and detector._stalled is False

    """
    The CONVERGENCE LATCH, and the regression that made it necessary.

    CALIB_STAT is the chip's confidence in its RECENT magnetometer data, not a record of what it has
    learned. Measured on TMS-7C: the operator's figure-8 drove mag to 3, and it fell back to 2 within
    a minute of the airframe sitting still -- while the learned offsets, and the heading, were
    unchanged. Reading it live made the ready gate a coin toss and told an operator who had done the
    figure-8 correctly to do it again.
    """
    class _CalibBus:
        """Minimal stub: serves one CALIB_STAT byte, so the latch is driven through the real path."""
        def __init__(self):
            self.raw = 0x00

        async def read(self, addr, reg, count, addrsize=8):
            return bytes([self.raw])

    bus = _CalibBus()
    imu = bno055.Bno055('imu_bno055', {}, _StubController())
    imu._bus, imu._addr, imu._period_ms = bus, 0x28, 20
    imu.calibration_state, imu._converged, imu._restored = None, False, False
    imu._calib_due = 1

    bus.raw = 0b00_11_01_00                 # sys 0 gyr 3 acc 1 mag 0 -- the state 7C booted in
    await imu._poll_calibration()
    assert imu.calibration_state == (0, 3, 1, 0)
    assert imu.calibrated() is False
    assert 'FIGURE-8' in imu.calibration()

    """
    `calibrate` must not answer SUCCESS for a device it cannot calibrate.

    The base task.Task.calibrate() returns None, which cc_client documents as success -- so
    `calibrate imu_bno055` replied `ok {"imu_bno055": null}` however many times it was run, while the
    magnetometer sat at 0 and nothing had changed. That reply is why the figure-8 was repeated.
    """
    outstanding = await imu.calibrate()
    assert outstanding is not None, 'an inert calibrate must not report success'
    assert 'FIGURE-8' in outstanding

    imu._calib_due = 1
    bus.raw = 0b11_11_01_11                 # the figure-8 lands: mag 3
    await imu._poll_calibration()
    assert imu.calibrated() is True and imu.calibration() == ''

    imu._calib_due = 1
    bus.raw = 0b11_11_01_10                 # ...and mag regresses to 2 sitting still
    await imu._poll_calibration()
    assert imu.calibration_state[3] == 2, 'the LIVE register must still show the regression'
    assert imu.calibrated() is True, 'the latch must survive it -- offsets do not un-learn'
    assert imu.calibration() == '', 'and the operator must not be re-asked for the figure-8'

    print('ok: bno055 driver registered; setup fails gracefully when no device answers; '
          'fusion-stall detector fires under rotation, never when still; '
          'calibration latches through a mag regression and inert calibrate reports what is owed')


asyncio.run(amain())
