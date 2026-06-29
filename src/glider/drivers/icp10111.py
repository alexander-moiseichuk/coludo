# drivers/icp10111.py — ICP-10111 barometric pressure sensor (TDK ICP-101xx, on the SEN0517) over
# the shared I2C bus: the PRIMARY altitude channel (8.5 cm accuracy). @task.driver('icp10111').
# Command-based, not register-mapped: setup() verifies the product id and reads the 4 OTP calibration
# constants; run() issues a measure command, reads pressure+temperature, applies the TDK polynomial
# conversion and writes pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the
# per-sensor startup ground zero) to the databoard. Graceful: wrong/absent id -> setup False -> skipped.
#
# Polled at period_ms. Uses the shared locked bus (i2cbus); shares i2c:0 with the other sensors.

import asyncio
import struct

import commons
import config
import databoard
import i2cbus
import recorder
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
_GROUND_SAMPLES = const(8)  # readings averaged at startup to fix the ground-zero reference


@task.driver('icp10111')
class Icp10111(task.Task):
    """Primary baro: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the
    startup ground zero, captured per-sensor so it is offset-free) to the databoard. `update`
    {"rezero": true} re-captures ground zero (e.g. after warm-up, just before launch)."""

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
        self._ground = await self._ground_zero()
        self._altitude, self._temperature, self._pressure, self._elevation = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'altitude', 'temperature', 'pressure', 'elevation')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                       ('altitude', 'temperature', 'pressure', 'elevation'),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._ok = True
        return True

    async def _ground_zero(self) -> float:
        """Average a short burst of altitude readings -> the per-sensor ground reference (m AMSL)."""
        total = 0.0
        for _ in range(_GROUND_SAMPLES):
            altitude, _temp, _pressure = await self._read()
            total += altitude
        return total / _GROUND_SAMPLES

    def _compensate(self, p_raw: int, t_raw: int) -> float:
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

    async def _read(self) -> tuple:
        """Measure and return (altitude m AMSL, temperature °C, pressure Pa)."""
        await self._bus.writeto(self._addr, _CMD_MEASURE)
        await asyncio.sleep_ms(_MEASURE_MS)
        data = await self._bus.readfrom(self._addr, 9)  # P[0,1],CRC, P[3],_,CRC, T[6,7],CRC
        p_raw = (data[0] << 16) | (data[1] << 8) | data[3]
        t_raw = (data[6] << 8) | data[7]
        temp_c = -45.0 + 175.0 / 65536.0 * t_raw
        pressure = self._compensate(p_raw, t_raw)
        altitude = 0.0 if pressure <= 0.0 else 44330.0 * (1.0 - (pressure / _SEA_LEVEL_PA) ** 0.190294957)
        return altitude, temp_c, pressure

    async def run(self) -> None:
        while True:
            try:
                altitude, temp_c, pressure = await self._read()
                elevation = altitude - self._ground
                self._altitude.push(altitude)  # one step: push our channels directly
                self._temperature.push(temp_c)
                self._pressure.push(pressure)
                self._elevation.push(elevation)
                self._telemetry.push((altitude, temp_c, pressure, elevation))
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('icp10111 :: read %r' % error)  # deduped: a persistent error logs once, not every tick
            await asyncio.sleep_ms(self._period_ms)

    def update(self, props: dict) -> list:
        """`{"rezero": true}` re-captures ground zero from the latest altitude (sync; operator does
        it when the reading is stable)."""
        if props.get('rezero') and self._altitude.value() is not None:
            self._ground = self._altitude.value()
            return ['ground']
        return []

    async def probe(self) -> str:
        """On-demand self-test: the run loop is producing pressure. We issue NO I2C here -- the
        command-based ICP (write-measure then read) would race the run loop's own measure/read
        sequence on the shared bus; a present, healthy sensor instead keeps its channel fresh."""
        try:
            recorder.Recorder.log(self.name, 'probe: data ...')
            await asyncio.sleep_ms(300)  # let the run loop produce a fresh reading
            pressure = self._pressure.value()
            if pressure is None:
                raise ValueError('no pressure from run loop (i2c:%s 0x%02x)' % (self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: data ok %.0f Pa' % pressure)
        except Exception as error:
            message = 'data: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """Deeper analysis when setup() failed: re-issue the product-id command and classify via
        commons.id_classify (masked 2-byte word & 0x3F, expecting 0x08). icp10111 is command-based (not
        register-mapped) so it cannot use i2cbus._Device.diagnose(), but the shared classifier still
        produces the same wire-level categories."""
        if getattr(self, '_bus', None) is None:
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        try:
            await self._bus.writeto(self._addr, _CMD_ID)
            ident = await self._bus.readfrom(self._addr, 3)
        except Exception:
            read = None
        else:
            read = (((ident[0] << 8) | ident[1]) & _ID_MASK)
        return commons.id_classify(read, _ID_VALUE)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status['altitude_m'] = self._altitude.value()
        status['temperature_c'] = self._temperature.value()
        status['pressure_pa'] = self._pressure.value()
        status['elevation_m'] = self._elevation.value()
        return status
