"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BMP280 barometric pressure sensor (on the SEN0253) over the shared I2C bus: the backup altitude
channel. @task.driver('bmp280'). setup() probes the chip id, reads the factory calibration and starts
normal-mode conversion; run() reads pressure, applies Bosch compensation and writes pressure (Pa),
temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor startup ground zero) to the
databoard. Graceful: wrong/absent chip id -> setup False -> skipped.

Polled at period_ms (the BMP280 conversion is ~tens of ms, far slower than the IMU). Uses the shared
locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and BNO055.
"""

import asyncio
import struct

import config
import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_ADDR = const(0x76)  # default I2C address (SDO low; 0x77 when high)
_REG_CHIP_ID = const(0xD0)  # = 0x58 for BMP280
_REG_CALIB = const(0x88)  # 24 bytes: dig_T1..T3, dig_P1..P9
_REG_CTRL_MEAS = const(0xF4)
_REG_CONFIG = const(0xF5)
_REG_DATA = const(0xF7)  # press msb/lsb/xlsb, temp msb/lsb/xlsb -- 6 bytes
_CHIP_ID = const(0x58)
_CTRL_NORMAL = const(0x2F)  # osrs_t x1, osrs_p x4, mode = normal
_CONFIG_FILTER = const(0x08)  # t_sb 0.5 ms, IIR filter x4
_SEA_LEVEL_PA = 101325.0  # reference for the barometric-altitude formula (AMSL)
_GROUND_SAMPLES = const(8)  # readings averaged at startup to fix the ground-zero reference


@task.driver('bmp280')
class Bmp280(task.Task):
    """
    Backup baro to the databoard: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation.

    Elevation is metres above the startup ground zero, captured per-sensor so it is offset-free.
    `update {"rezero": true}` re-captures ground zero (e.g. after warm-up, just before launch).
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', _ADDR)
        self._period_ms: int = self.config.get('period_ms', 100)  # ~10 Hz (conversion is slow)
        self._buf = bytearray(6)
        try:
            if await self._bus.read_chip_id(self._addr, _REG_CHIP_ID) != _CHIP_ID:
                return False  # not a BMP280 at this address
            cal = await self._bus.read(self._addr, _REG_CALIB, 24)
            # dig_T1 + dig_P1 are unsigned (H), the rest signed (h)
            self._cal = struct.unpack('<HhhHhhhhhhhh', cal)
            await self._bus.write(self._addr, _REG_CONFIG, bytes([_CONFIG_FILTER]))
            await self._bus.write(self._addr, _REG_CTRL_MEAS, bytes([_CTRL_NORMAL]))
            await asyncio.sleep_ms(50)  # let the first normal-mode conversion complete
            self._ground = await self._ground_zero()  # inside the try: an I2C error here -> graceful False
        except Exception as error:
            print('bmp280 :: %r' % error)
            return False
        self._altitude, self._temperature, self._pressure, self._elevation = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'altitude', 'temperature', 'pressure', 'elevation')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                             ('altitude', 'temperature', 'pressure', 'elevation'),
                                             decimate_us=self.config.get('telemetry_us', 0))  # 0 -> global rate
        self._ok = True
        return True

    async def _ground_zero(self) -> float:
        """Average a short burst of altitude readings -> the per-sensor ground reference (m AMSL)."""
        total = 0.0
        for _ in range(_GROUND_SAMPLES):
            altitude, _temp, _pressure = await self._read()
            total += altitude
            await asyncio.sleep_ms(20)
        return total / _GROUND_SAMPLES

    def _compensate(self, adc_t: int, adc_p: int) -> tuple:
        """
        Bosch fixed-point compensation: raw ADC counts -> (pressure Pa, temperature °C).

        int64 math throughout (MicroPython ints are arbitrary precision, so it cannot overflow).

        Args:
            adc_t - raw temperature ADC reading.
            adc_p - raw pressure ADC reading.

        Returns:
            (pressure Pa, temperature °C); pressure is 0.0 when the calibration is degenerate.
        """
        t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9 = self._cal
        tv1 = (((adc_t >> 3) - (t1 << 1)) * t2) >> 11
        tv2 = (((((adc_t >> 4) - t1) * ((adc_t >> 4) - t1)) >> 12) * t3) >> 14
        t_fine = tv1 + tv2
        temp_c = ((t_fine * 5 + 128) >> 8) / 100.0
        var1 = t_fine - 128000
        var2 = var1 * var1 * p6
        var2 = var2 + ((var1 * p5) << 17)
        var2 = var2 + (p4 << 35)
        var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12)
        var1 = (((1 << 47) + var1) * p1) >> 33
        if var1 == 0:
            return 0.0, temp_c
        p = 1048576 - adc_p
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (p9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (p8 * p) >> 19
        p = ((p + var1 + var2) >> 8) + (p7 << 4)
        return p / 256.0, temp_c

    async def _read(self) -> tuple:
        """Read the sensor and return (altitude m AMSL, temperature °C, pressure Pa)."""
        await self._bus.read_into(self._addr, _REG_DATA, self._buf)
        adc_p = (self._buf[0] << 12) | (self._buf[1] << 4) | (self._buf[2] >> 4)
        adc_t = (self._buf[3] << 12) | (self._buf[4] << 4) | (self._buf[5] >> 4)
        pressure, temp_c = self._compensate(adc_t, adc_p)
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
                self.note('bmp280 :: read %r', error)  # deduped: a persistent error logs once, not every tick
            await asyncio.sleep_ms(self._period_ms)

    def update(self, props: dict) -> list:
        """
        Apply an operator property change: re-zero or directly set the ground reference.

        `{"rezero": true}` re-captures ground zero from the latest altitude (sync; the operator does it
        when the reading is stable). `{"ground": <m>}` SETS the zero directly -- the warm start rebases a
        rebooted baro to the breadcrumb's pad altitude (a mid-air re-zero would make `elevation` read ~0
        at altitude, faking an immediate landing).

        Args:
            props - the property dict; honours 'ground' (a float, metres) and 'rezero' (a truthy flag).

        Returns:
            The list of changed property names (['ground'] on a set or re-zero, [] otherwise).
        """
        if 'ground' in props:
            self._ground = float(props['ground'])
            return ['ground']
        if props.get('rezero') and self._altitude.value() is not None:
            self._ground = self._altitude.value()
            return ['ground']
        return []

    async def probe(self) -> str:
        """
        On-demand self-test: the chip id reads back, then one conversion reads (each step logged).

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) at the first failing step.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: chip id ...')
            chip = await self._bus.read_chip_id(self._addr, _REG_CHIP_ID)
            if chip != _CHIP_ID:
                raise ValueError('BMP280 id 0x%02x != 0x%02x at i2c:%s 0x%02x' % (
                    chip, _CHIP_ID, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: chip id ok 0x%02x' % chip)
        except Exception as error:
            message = 'chip id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: read ...')
            _altitude, _temp, pressure = await self._read()
            recorder.Recorder.log(self.name, 'probe: read ok %.0f Pa' % pressure)
        except Exception as error:
            message = 'read: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: classify the wire-level fault.

        The bus reads the chip id and classifies the fault (no ack / wrong device / present-but-init), so
        the Controller can fold it into the failure reason and `verify`/`probe` show the 'why', not just
        'absent / miswired?'.

        Args:
            (none)

        Returns:
            A wire-level fault description; a config-fault message when setup never built the bus.
        """
        bus = self._bus  # None until setup builds the transport
        if bus is None:  # setup never built the bus -> a config fault
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        return await bus.device(self._addr).diagnose(_REG_CHIP_ID, _CHIP_ID)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status.update({'addr': hex(self._addr),
                       'altitude_m': self._altitude.value(),
                       'temperature_c': self._temperature.value(),
                       'pressure_pa': self._pressure.value(),
                       'elevation_m': self._elevation.value()})
        return status
