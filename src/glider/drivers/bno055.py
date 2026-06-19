# drivers/bno055.py — BNO055 9-DOF IMU (on the SEN0253) over the shared I2C bus: the attitude
# channel. @task.driver('bno055'). In NDOF fusion mode the chip computes absolute orientation
# on-chip; run() reads the Euler angles (heading, roll, pitch in degrees) to the blackboard
# 'attitude' slot. Graceful: a wrong/absent chip id -> setup False -> the Controller skips it.
#
# BNO055's INT pin signals motion/threshold events, not a fusion data-ready, so this driver polls at
# period_ms (the fusion engine runs at 100 Hz internally); the wired int_pin is reserved for future
# event detection (e.g. high-g). Uses the shared locked bus (i2cbus) since it shares i2c:0 with the
# ADXL375 and BMP280.

import asyncio
import struct

import blackboard
import config
import i2cbus
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_REG_CHIP_ID = const(0x00)  # = 0xA0
_REG_OPR_MODE = const(0x3D)  # operating mode
_REG_PWR_MODE = const(0x3E)  # power mode
_REG_EUL = const(0x1A)  # EUL heading/roll/pitch -- 6 bytes int16 LE, 16 LSB/degree
_CHIP_ID = const(0xA0)
_MODE_CONFIG = const(0x00)
_MODE_NDOF = const(0x0C)  # full 9-DOF absolute-orientation fusion
_PWR_NORMAL = const(0x00)
_DEG = 1.0 / 16.0


@task.driver('bno055')
class Bno055(task.Task):
    """9-DOF attitude: polls (heading, roll, pitch) deg to the blackboard 'attitude' slot."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x28)
        self._period_ms: int = self.config.get('period_ms', 20)  # 50 Hz (fusion runs at 100 Hz)
        self._buf = bytearray(6)
        try:
            if (await self._bus.read(self._addr, _REG_CHIP_ID, 1))[0] != _CHIP_ID:
                return False  # not a BNO055 at this address
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_CONFIG]))
            await asyncio.sleep_ms(25)  # mode switch settle
            await self._bus.write(self._addr, _REG_PWR_MODE, bytes([_PWR_NORMAL]))
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_NDOF]))
            await asyncio.sleep_ms(25)  # config -> fusion settle
        except Exception as error:
            print('bno055 :: %r' % error)
            return False
        blackboard.Blackboard.declare('attitude')
        self._ok = True
        return True

    async def sample(self) -> tuple:
        """Read the fused orientation as (heading, roll, pitch) in degrees."""
        await self._bus.read_into(self._addr, _REG_EUL, self._buf)
        heading, roll, pitch = struct.unpack('<hhh', self._buf)
        return (heading * _DEG, roll * _DEG, pitch * _DEG)

    async def run(self) -> None:
        while True:
            try:
                blackboard.Blackboard.write('attitude', await self.sample(), self.name)
            except Exception as error:
                print('bno055 :: read %r' % error)
            await asyncio.sleep_ms(self._period_ms)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        slot = blackboard.Blackboard.raw('attitude', self.name)  # our own latest (no hot-path I2C)
        status['attitude_deg'] = slot.value if slot is not None else None
        return status
