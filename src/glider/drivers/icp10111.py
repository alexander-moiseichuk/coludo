# drivers/icp10111.py — ICP-10111 barometric pressure sensor (TDK ICP-101xx, on the SEN0517) over
# the shared I2C bus: the PRIMARY altitude channel (8.5 cm accuracy). @task.driver('icp10111').
# Command-based, not register-mapped: setup() verifies the product id and reads the 4 OTP calibration
# constants; run() issues a measure command, reads pressure+temperature, applies the TDK polynomial
# conversion and writes barometric altitude (m AMSL) to the blackboard 'altitude' slot. Graceful:
# wrong/absent id -> setup False -> the Controller skips it.
#
# Polled at period_ms. Uses the shared locked bus (i2cbus); shares i2c:0 with the other sensors.

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


_CMD_ID = b'\xef\xc8'  # read product id -> (word & 0x3f) == 0x08 for ICP-101xx
_CMD_OTP_UNLOCK = b'\xc5\x95\x00\x66\x9c'  # unlock OTP, then 4x read
_CMD_OTP_READ = b'\xc7\xf7'
_CMD_MEASURE = b'\x48\xa3'  # "measure pressure first", normal mode (6.3 ms, 1.6 Pa RMS)
_MEASURE_MS = const(12)  # conversion wait for normal mode (with margin)
_ID_MASK = const(0x3F)
_ID_VALUE = const(0x08)
_SEA_LEVEL_PA = 101325.0
_QUADR = 1.0 / 16777216.0  # 1 / 2**24
_LUT_LOWER = 3.5 * (1 << 20)
_LUT_UPPER = 11.5 * (1 << 20)
_OFFSET = 2048.0
_PCAL = (45000.0, 80000.0, 105000.0)  # calibration pressures for the conversion LUT


@task.driver('icp10111')
class Icp10111(task.Task):
    """Primary barometric altitude: polls compensated pressure -> altitude (m) to 'altitude'."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x63)
        self._period_ms: int = self.config.get('period_ms', 100)  # ~10 Hz
        try:
            await self._bus.writeto(self._addr, _CMD_ID)
            ident = await self._bus.readfrom(self._addr, 3)
            if (((ident[0] << 8) | ident[1]) & _ID_MASK) != _ID_VALUE:
                return False  # not an ICP-101xx at this address
            await self._bus.writeto(self._addr, _CMD_OTP_UNLOCK)
            otp = []
            for _ in range(4):
                await self._bus.writeto(self._addr, _CMD_OTP_READ)
                word = await self._bus.readfrom(self._addr, 3)
                otp.append(struct.unpack('>h', word[0:2])[0])  # signed 16-bit
            self._otp = otp
        except Exception as error:
            print('icp10111 :: %r' % error)
            return False
        blackboard.Blackboard.declare('altitude')
        blackboard.Blackboard.declare('temperature')
        self._ok = True
        return True

    def _pressure(self, p_raw: int, t_raw: int) -> float:
        """TDK ICP-101xx polynomial: raw pressure + temperature -> pressure in Pa."""
        c0, c1, c2, c3 = self._otp
        t = t_raw - 32768
        s1 = _LUT_LOWER + c0 * t * t * _QUADR
        s2 = _OFFSET * c3 + c1 * t * t * _QUADR
        s3 = _LUT_UPPER + c2 * t * t * _QUADR
        p0, p1, p2 = _PCAL
        c = (s1 * s2 * (p0 - p1) + s2 * s3 * (p1 - p2) + s3 * s1 * (p2 - p0)) / (
            s3 * (p0 - p1) + s1 * (p1 - p2) + s2 * (p2 - p0))
        a = (p0 * s1 - p1 * s2 - (p1 - p0) * c) / (s1 - s2)
        b = (p0 - a) * (s1 + c)
        return a + b / (c + p_raw)

    async def sample(self) -> tuple:
        """Measure and return (barometric altitude m AMSL, temperature °C)."""
        await self._bus.writeto(self._addr, _CMD_MEASURE)
        await asyncio.sleep_ms(_MEASURE_MS)
        data = await self._bus.readfrom(self._addr, 9)  # P[0,1],CRC, P[3],_,CRC, T[6,7],CRC
        p_raw = (data[0] << 16) | (data[1] << 8) | data[3]
        t_raw = (data[6] << 8) | data[7]
        temp_c = -45.0 + 175.0 / 65536.0 * t_raw
        pressure = self._pressure(p_raw, t_raw)
        altitude = 0.0 if pressure <= 0.0 else 44330.0 * (1.0 - (pressure / _SEA_LEVEL_PA) ** 0.190294957)
        return altitude, temp_c

    async def run(self) -> None:
        while True:
            try:
                altitude, temp_c = await self.sample()
                blackboard.Blackboard.write('altitude', altitude, self.name)
                blackboard.Blackboard.write('temperature', temp_c, self.name)
            except Exception as error:
                print('icp10111 :: read %r' % error)
            await asyncio.sleep_ms(self._period_ms)

    def _mine(self, quantity: str):
        """This sensor's own latest raw value for `quantity` (None if never written)."""
        slot = blackboard.Blackboard.raw(quantity, self.name)
        return slot.value if slot is not None else None

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # latest samples from the blackboard (no hot-path I2C here)
        status['altitude_m'] = self._mine('altitude')
        status['temperature_c'] = self._mine('temperature')
        return status
