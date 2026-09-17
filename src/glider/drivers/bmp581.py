"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BMP581 barometric pressure sensor (on the SEN0697) over the shared I2C bus: the backup altitude channel.

Provides pressure (Pa), temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor startup
ground zero) to the databoard, exactly as the BMP280 it replaces -- so nothing downstream learns a new
name. `update {"rezero": true}` re-captures ground zero (e.g. after warm-up, just before launch).

Simpler than the BMP280 in one way that matters: the BMP581 outputs COMPENSATED values, so there is no
factory calibration to read and no fixed-point compensation to carry. Pressure is a 24-bit unsigned count
of 1/64 Pa; temperature a 24-bit two's-complement count of 1/65536 °C.

Register values are Bosch's own (bmp5_defs.h), and the configuration is the one the TMS-7 nose logger
flew: pressure enabled at 4x oversampling, 50 Hz, normal mode. @task.driver('bmp581').
"""

import asyncio

import commons
import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:
    from commons import const

_ADDR = const(0x47)  # default I2C address (0x46 when the address pin is pulled the other way)
_REG_CHIP_ID = const(0x01)  # = 0x50 for BMP581
_REG_TEMP_DATA = const(0x1D)  # temp xlsb/lsb/msb then press xlsb/lsb/msb -- 6 contiguous bytes
_REG_OSR_CONFIG = const(0x36)
_REG_ODR_CONFIG = const(0x37)
_CHIP_ID = const(0x50)
_OSR_PRESSURE_4X = const(0x50)  # press_en (bit 6) | osr_p = 4x (bits 5:3) | osr_t = 1x
_ODR_NORMAL_50HZ = const(0xBD)  # deep standby disabled (bit 7) | odr 50 Hz (bits 6:2) | normal mode
_GROUND_SAMPLES = const(8)  # readings averaged at startup to fix the ground-zero reference


@task.driver('bmp581')
class Bmp581(task.Task):
    """
    Backup baro to the databoard: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation.

    Elevation is metres above the startup ground zero, captured per-sensor so it is offset-free.
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        self._period_ms: int = self.config.get('period_ms', 100)  # ~10 Hz; the part runs faster than we read
        self._buf = bytearray(6)
        try:
            if await self._bus.read_chip_id(self._addr, _REG_CHIP_ID) != _CHIP_ID:
                return False  # not a BMP581 at this address
            await self._configure()
            await asyncio.sleep_ms(50)  # let the first normal-mode conversion complete
            self._ground = await self._ground_zero()  # inside the try: an I2C error here -> graceful False
        except Exception as error:
            print('bmp581 :: %r' % error)
            return False
        self._altitude, self._temperature, self._pressure, self._elevation = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'altitude', 'temperature', 'pressure', 'elevation')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                             ('altitude', 'temperature', 'pressure', 'elevation'),
                                             decimate_us=self.config.get('telemetry_ms', 0) * 1000)  # 0 -> global rate
        self._ok = True
        return True

    async def _configure(self) -> None:
        """Enable pressure at 4x oversampling and start 50 Hz normal-mode conversion."""
        await self._bus.write(self._addr, _REG_OSR_CONFIG, bytes([_OSR_PRESSURE_4X]))
        await self._bus.write(self._addr, _REG_ODR_CONFIG, bytes([_ODR_NORMAL_50HZ]))

    async def _ground_zero(self) -> float:
        """Average a short burst of altitude readings -> the per-sensor ground reference (m AMSL)."""
        total = 0.0
        for _ in range(_GROUND_SAMPLES):
            altitude, _temp, _pressure = await self._read()
            total += altitude
            await asyncio.sleep_ms(20)
        return total / _GROUND_SAMPLES

    async def _read(self) -> tuple:
        """Read the sensor and return (altitude m AMSL, temperature °C, pressure Pa)."""
        await self._bus.read_into(self._addr, _REG_TEMP_DATA, self._buf)
        raw_t = self._buf[0] | (self._buf[1] << 8) | (self._buf[2] << 16)
        raw_p = self._buf[3] | (self._buf[4] << 8) | (self._buf[5] << 16)
        if raw_t & 0x800000:  # temperature is two's complement; pressure is unsigned
            raw_t -= 1 << 24
        temp_c = raw_t / 65536.0
        pressure = raw_p / 64.0
        return commons.altitude_m(pressure), temp_c, pressure

    async def rearm(self) -> None:
        """
        Re-apply the mode this driver set at setup, after something reset the part underneath it.

        The BMP581 powers on in STANDBY, so anything that resets it behind our back stops it converting
        and it returns its last sample forever -- an altitude that quietly stops moving.

        On v1.1 the specific culprit the BMP280 driver guards against cannot reach here: the ICP-10111's
        recovery sends an I2C GENERAL-CALL reset, and the two baros are on DIFFERENT buses (ICP on i2c:1,
        this part on i2c:0 with the rest of the SEN0697). That separation is much of the redundancy's
        value -- a backup sharing the primary's bus shares its bus faults. This stays as a cheap guard
        against anything else that resets the part, and so a later rewire onto one bus cannot silently
        reintroduce the failure.

        Args:
            (none)

        Returns:
            None; best-effort -- a part that is still gone fails its next read as before.
        """
        try:
            await self._configure()
        except Exception as error:
            recorder.Recorder.log(self.name, 'rearm failed: %r' % error)

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
                self.note('bmp581 :: read %r', error)  # deduped: a persistent error logs once, not every tick
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
        if props.get('rezero'):
            altitude, altitude_source, _altitude_age = self._altitude.read()
            # read(), NOT value(): a re-zero LATCHES a number that every later `elevation` is reported
            # against, so an extrapolated altitude here biases the channel for the rest of the flight.
            if altitude_source is None:
                raise ValueError('no fresh altitude to re-zero from')
            self._ground = altitude
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
                raise ValueError('BMP581 id 0x%02x != 0x%02x at i2c:%s 0x%02x' % (
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
