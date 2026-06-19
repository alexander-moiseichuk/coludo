# drivers/adxl375.py — ADXL375 ±200 g high-G accelerometer over I2C: the boost-phase accel channel.
# @task.driver('adxl375'). setup() probes the device id and configures it; run() samples at the
# component's rate and writes the latest (x, y, z) acceleration in g to the blackboard 'accel' slot.
# If the device is absent (no I2C ack / wrong device id) setup() returns False and the Controller
# skips it, so the board boots fine with the sensor unplugged.
#
# NOTE: this driver opens its own machine.I2C on the component's bus. When several I2C sensors share
# `i2c:0`, a shared (locked) bus manager is the right next step — see specs/coludo.md.

import asyncio
import struct

import blackboard
import config
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_REG_DEVID = const(0x00)  # reads 0xE5 on the whole ADXL34x/375 family
_REG_BW_RATE = const(0x2C)  # output data rate
_REG_POWER_CTL = const(0x2D)  # measure bit = 0x08
_REG_DATA_FORMAT = const(0x31)
_REG_DATAX0 = const(0x32)  # X0,X1,Y0,Y1,Z0,Z1 -- 6 bytes, signed LE
_DEVID = const(0xE5)
_SCALE_G = 0.049  # ADXL375 ≈ 49 mg/LSB (full-resolution, fixed ±200 g)


@task.driver('adxl375')
class Adxl375(task.Task):
    """High-G accel: samples (x, y, z) in g to the blackboard 'accel' slot."""

    async def setup(self) -> bool:
        from machine import I2C, Pin

        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c:0'))
        if spec is None:
            return False
        self._addr: int = self.config.get('addr', 0x53)
        self._period_ms: int = self.config.get('period_ms', 10)  # ~100 Hz default
        self._buf = bytearray(6)
        try:
            bus_id = int(self.config.get('bus', 'i2c:0').split(':')[1])
            self._i2c = I2C(bus_id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))
            if self._i2c.readfrom_mem(self._addr, _REG_DEVID, 1)[0] != _DEVID:
                return False  # not an ADXL375 at this address
            self._i2c.writeto_mem(self._addr, _REG_DATA_FORMAT, b'\x0b')  # full-res, right-justified
            self._i2c.writeto_mem(self._addr, _REG_BW_RATE, b'\x0d')  # 800 Hz ODR
            self._i2c.writeto_mem(self._addr, _REG_POWER_CTL, b'\x08')  # measure mode
        except Exception as error:
            print('adxl375 :: %r' % error)
            return False
        blackboard.Blackboard.declare('accel')
        self._ok = True
        return True

    def sample(self) -> tuple:
        """Read and return (x, y, z) acceleration in g."""
        self._i2c.readfrom_mem_into(self._addr, _REG_DATAX0, self._buf)
        x, y, z = struct.unpack('<hhh', self._buf)
        return (x * _SCALE_G, y * _SCALE_G, z * _SCALE_G)

    async def run(self) -> None:
        while True:
            try:
                blackboard.Blackboard.write('accel', self.sample(), self.name)
            except Exception as error:
                print('adxl375 :: read %r' % error)
            await asyncio.sleep_ms(self._period_ms)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        if self._ok:
            try:
                status['accel_g'] = self.sample()
            except Exception:
                status['accel_g'] = None
        return status
