# drivers/bmp280.py — BMP280 barometric pressure sensor (on the SEN0253) over the shared I2C bus:
# the backup altitude channel. @task.driver('bmp280'). setup() probes the chip id, reads the factory
# calibration and starts normal-mode conversion; run() reads pressure, applies Bosch compensation
# and writes barometric altitude (m AMSL) to the blackboard 'altitude' slot. Graceful: wrong/absent
# chip id -> setup False -> the Controller skips it.
#
# Polled at period_ms (the BMP280 conversion is ~tens of ms, far slower than the IMU). Uses the
# shared locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and BNO055.

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


_REG_CHIP_ID = const(0xD0)  # = 0x58 for BMP280
_REG_CALIB = const(0x88)  # 24 bytes: dig_T1..T3, dig_P1..P9
_REG_CTRL_MEAS = const(0xF4)
_REG_CONFIG = const(0xF5)
_REG_DATA = const(0xF7)  # press msb/lsb/xlsb, temp msb/lsb/xlsb -- 6 bytes
_CHIP_ID = const(0x58)
_CTRL_NORMAL = const(0x2F)  # osrs_t x1, osrs_p x4, mode = normal
_CONFIG_FILTER = const(0x08)  # t_sb 0.5 ms, IIR filter x4
_SEA_LEVEL_PA = 101325.0  # reference for the barometric-altitude formula


@task.driver('bmp280')
class Bmp280(task.Task):
    """Barometric altitude: polls compensated pressure -> altitude (m) to the blackboard 'altitude'."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x76)
        self._period_ms: int = self.config.get('period_ms', 100)  # ~10 Hz (conversion is slow)
        self._buf = bytearray(6)
        try:
            if (await self._bus.read(self._addr, _REG_CHIP_ID, 1))[0] != _CHIP_ID:
                return False  # not a BMP280 at this address
            cal = await self._bus.read(self._addr, _REG_CALIB, 24)
            # dig_T1 + dig_P1 are unsigned (H), the rest signed (h)
            self._cal = struct.unpack('<HhhHhhhhhhhh', cal)
            await self._bus.write(self._addr, _REG_CONFIG, bytes([_CONFIG_FILTER]))
            await self._bus.write(self._addr, _REG_CTRL_MEAS, bytes([_CTRL_NORMAL]))
        except Exception as error:
            print('bmp280 :: %r' % error)
            return False
        blackboard.Blackboard.declare('altitude')
        self._ok = True
        return True

    def _compensate(self, adc_t: int, adc_p: int) -> float:
        """Bosch fixed-point compensation: raw ADC -> pressure in Pa (int64 math; MicroPython ints
        are arbitrary precision). Returns 0.0 if the calibration is degenerate."""
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9 = self._cal
        var1 = (((adc_t >> 3) - (t1 << 1)) * t2) >> 11
        var2 = (((((adc_t >> 4) - t1) * ((adc_t >> 4) - t1)) >> 12) * t3) >> 14
        t_fine = var1 + var2
        var1 = t_fine - 128000
        var2 = var1 * var1 * p6
        var2 = var2 + ((var1 * p5) << 17)
        var2 = var2 + (p4 << 35)
        var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12)
        var1 = (((1 << 47) + var1) * p1) >> 33
        if var1 == 0:
            return 0.0
        p = 1048576 - adc_p
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (p9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (p8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (p7 << 4)
        return p / 256.0  # Pa

    async def sample(self) -> float:
        """Read pressure and return barometric altitude in metres (AMSL, std sea-level reference)."""
        await self._bus.read_into(self._addr, _REG_DATA, self._buf)
        adc_p = (self._buf[0] << 12) | (self._buf[1] << 4) | (self._buf[2] >> 4)
        adc_t = (self._buf[3] << 12) | (self._buf[4] << 4) | (self._buf[5] >> 4)
        pressure = self._compensate(adc_t, adc_p)
        if pressure <= 0.0:
            return 0.0
        return 44330.0 * (1.0 - (pressure / _SEA_LEVEL_PA) ** 0.190294957)

    async def run(self) -> None:
        while True:
            try:
                blackboard.Blackboard.write('altitude', await self.sample(), self.name)
            except Exception as error:
                print('bmp280 :: read %r' % error)
            await asyncio.sleep_ms(self._period_ms)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        slot = blackboard.Blackboard.read('altitude')  # latest sample (no hot-path I2C in inspect)
        status['altitude_m'] = slot.value if (slot is not None and slot.source == self.name) else None
        return status
