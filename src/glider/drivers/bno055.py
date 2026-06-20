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
import recorder
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_REG_CHIP_ID = const(0x00)  # = 0xA0
_REG_OPR_MODE = const(0x3D)  # operating mode
_REG_PWR_MODE = const(0x3E)  # power mode
_REG_DATA = const(0x08)  # ACC..EUL block: acc(6) mag(6) gyro(6) eul(6) = 24 bytes, all int16 LE
_OFF_EUL = const(18)  # EUL heading/roll/pitch within the block (16 LSB/degree)
_CHIP_ID = const(0xA0)
_MODE_CONFIG = const(0x00)
_MODE_NDOF = const(0x0C)  # full 9-DOF absolute-orientation fusion
_PWR_NORMAL = const(0x00)
_DEG = 1.0 / 16.0
_ACC_G = 1.0 / 980.665  # ACC_DATA is m/s² at 100 LSB/(m/s²); /100/9.80665 -> g (incl gravity)


@task.driver('bno055')
class Bno055(task.Task):
    """9-DOF: attitude (heading, roll, pitch) deg -> 'attitude', plus the calibrated accelerometer
    (g, incl gravity) -> 'accel' as a low-g backup to the ADXL375 (priority 1)."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x28)
        self._period_ms: int = self.config.get('period_ms', 20)  # 50 Hz (fusion runs at 100 Hz)
        self._buf = bytearray(24)  # ACC..EUL block
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
        channels = blackboard.Blackboard.provide(self.name, self.config.get('provides', {}))
        self._attitude, self._accel = channels['attitude'], channels['accel']
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                             ('heading', 'roll', 'pitch', 'ax', 'ay', 'az'),
                                             decimate_us=self.config.get('telemetry_us', 0))
        self._ok = True
        return True

    async def sample(self) -> tuple:
        """Read the block and return (attitude (heading, roll, pitch) deg, accel (x, y, z) g)."""
        await self._bus.read_into(self._addr, _REG_DATA, self._buf)
        ax, ay, az = struct.unpack_from('<hhh', self._buf, 0)
        heading, roll, pitch = struct.unpack_from('<hhh', self._buf, _OFF_EUL)
        return ((heading * _DEG, roll * _DEG, pitch * _DEG), (ax * _ACC_G, ay * _ACC_G, az * _ACC_G))

    async def run(self) -> None:
        while True:
            try:
                attitude, accel = await self.sample()
                self._attitude.push(attitude)  # one step: push our channels directly
                self._accel.push(accel)  # low-g backup to the ADXL375
                self._telemetry.push(attitude + accel)
            except Exception as error:
                print('bno055 :: read %r' % error)
            await asyncio.sleep_ms(self._period_ms)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['attitude_deg'] = self._attitude.value()  # our channels' latest (no hot-path I2C)
        status['accel_g'] = self._accel.value()
        return status
