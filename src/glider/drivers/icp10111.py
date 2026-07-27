"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

ICP-10111 barometric pressure sensor (TDK ICP-101xx, on the SEN0517) over the shared I2C bus: the
PRIMARY altitude channel (8.5 cm accuracy). @task.driver('icp10111'). Command-based, not
register-mapped: setup() verifies the product id and reads the 4 OTP calibration constants; run() issues
a measure command, reads pressure+temperature, applies the TDK polynomial conversion and writes pressure
(Pa), temperature (°C), altitude (m AMSL) and elevation (m above the per-sensor startup ground zero) to
the databoard. Graceful: wrong/absent id -> setup False -> skipped.

Polled at period_ms. Uses the shared locked bus (i2cbus); shares i2c:0 with the other sensors.
"""

import asyncio
import struct
import time

import commons
import config
import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_ADDR = const(0x63)  # the fixed I2C address
_CMD_ID = b'\xef\xc8'  # read product id -> (word & 0x3f) == 0x08 for ICP-101xx
_GENERAL_CALL_RESET = b'\x06'  # I2C general-call reset (to addr 0x00): reaches a wedged core
_CMD_OTP_UNLOCK = b'\xc5\x95\x00\x66\x9c'  # unlock OTP, then 4x read
_CMD_OTP_READ = b'\xc7\xf7'
_CMD_MEASURE = b'\x48\xa3'  # "measure pressure first", normal mode (6.3 ms, 1.6 Pa RMS)
_RESET_TRIES = const(4)  # first-touch attempts spanning one max-length conversion window (~95 ms)
_RECOVER_AFTER = const(3)  # consecutive run-loop read failures before escalating (see _recover)
_RESET_RETRY_MS = const(30)
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
    """
    Primary baro to the databoard: pressure (Pa), temperature (°C), altitude (m AMSL) and elevation.

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
        self._busy: bool = False  # one owner of the measure->read conversation (see _read)
        self._sample = None
        self._sample_ms: int = 0
        self._addr: int = self.config.get('addr', _ADDR)
        self._period_ms: int = self.config.get('period_ms', 100)  # ~10 Hz
        """
        OSError(19) hardening (measured after the 7/06 OOM-soak panic): a mid-conversion
        sensor NAKs until the conversion drains, and an unclean reboot can LATCH the digital
        core -- the address still acks (scan sees 0x63) but every command NAKs. The addressed
        soft reset (0x805d) RE-WEDGED the latched bench part; the I2C GENERAL-CALL reset
        (0x00 0x06) fully recovered it (5/5 measures after). So: touch the part plainly (a
        clean boot never needs a reset), drain a possible busy window on the first NAK, and
        only from the second NAK fire the general call. It also resets peers that honour it
        (bmp280, ina226) -- acceptable: they re-apply their config-declared state when set up
        after us, and it fires only when the PRIMARY baro would otherwise be lost.
        """
        for attempt in range(_RESET_TRIES):
            try:
                await self._bus.writeto(self._addr, _CMD_ID)
                await self._bus.readfrom(self._addr, 3)  # drain -- the real id read follows
                break
            except OSError:
                if attempt == _RESET_TRIES - 1:
                    break  # truly absent / dead -- the id read below reports it as before
                if attempt:  # not just busy -- assume latched, escalate
                    try:
                        await self._bus.writeto(0x00, _GENERAL_CALL_RESET)
                    except OSError:
                        pass  # nothing honours general call -- keep waiting it out
                await asyncio.sleep_ms(_RESET_RETRY_MS)
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
            # inside the try: a half-recovered part (id/OTP ack, first measure NAKs -- the
            # post-latch-up state needing a power cycle) must fail setup GRACEFULLY, not crash it
            self._ground = await self._ground_zero()
        except Exception as error:
            print('icp10111 :: %r' % error)
            return False
        self._altitude, self._temperature, self._pressure, self._elevation = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'altitude', 'temperature', 'pressure', 'elevation')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                       ('altitude', 'temperature', 'pressure', 'elevation'),
                                       decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        self._ok = True
        return True

    async def _ground_zero(self) -> float:
        """Average a short burst of altitude readings -> the per-sensor ground reference (m AMSL)."""
        total = 0.0
        for _ in range(_GROUND_SAMPLES):
            altitude, _temp, _pressure = await self._read(cached_ok=False)  # AVERAGE distinct samples
            total += altitude
        return total / _GROUND_SAMPLES

    def _compensate(self, p_raw: int, t_raw: int) -> float:
        """
        TDK ICP-101xx polynomial conversion: raw pressure + temperature -> pressure in Pa.

        Args:
            p_raw - raw pressure reading from the sensor.
            t_raw - raw temperature reading from the sensor.

        Returns:
            Pressure in Pa.
        """
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

    async def _read(self, cached_ok: bool = True) -> tuple:
        """
        Measure and return (altitude m AMSL, temperature °C, pressure Pa) -- SERIALISED and cached.

        This is a MULTI-STEP sequence: write-measure, await the conversion, then read. That await is a
        yield point, so a second caller entering here mid-sequence issues its own measure command into
        the middle of ours and both come back NAKing -- measured at a 20.8 % failure rate when a
        diagnostic polled alongside the run loop. It is not a bus-sharing problem (peers on i2c:0 cost
        nothing); it is this device's own state machine having exactly one conversation at a time.
        probe() dodges it by issuing no I2C at all, which was a workaround for the missing guard here.

        So: one owner at a time, and a caller arriving while a sample is already in flight gets the
        RESULT rather than starting a competing conversation. A sample newer than period_ms is served
        from cache untouched -- it is exactly as fresh as the run loop would have handed out anyway,
        for zero bus traffic. A plain flag rather than asyncio.Lock: `async with lock` measures ~288 B
        on this port and MicroPython's loop is cooperative, so the flag can only change across an await.

        Args:
            cached_ok - False forces a genuine conversion (ground_zero AVERAGES samples, so serving it
                the same cached tuple repeatedly would collapse the average to one reading).

        Returns:
            (altitude m AMSL, temperature degC, pressure Pa).
        """
        if cached_ok and self._sample is not None \
                and time.ticks_diff(time.ticks_ms(), self._sample_ms) < self._period_ms:
            return self._sample
        while self._busy:  # someone else owns the conversation -- wait for their answer, do not compete
            await asyncio.sleep_ms(1)
            if cached_ok and self._sample is not None \
                    and time.ticks_diff(time.ticks_ms(), self._sample_ms) < self._period_ms:
                return self._sample
        self._busy = True
        try:
            return await self._measure()
        finally:
            self._busy = False  # a raising read must not wedge every future caller

    async def _measure(self) -> tuple:
        """The bus conversation itself; only ever entered by the single owner _read() admits."""
        await self._bus.writeto(self._addr, _CMD_MEASURE)
        await asyncio.sleep_ms(_MEASURE_MS)
        data = await self._bus.readfrom(self._addr, 9)  # P[0,1],CRC, P[3],_,CRC, T[6,7],CRC
        p_raw = (data[0] << 16) | (data[1] << 8) | data[3]
        t_raw = (data[6] << 8) | data[7]
        temp_c = -45.0 + 175.0 / 65536.0 * t_raw
        pressure = self._compensate(p_raw, t_raw)
        altitude = 0.0 if pressure <= 0.0 else 44330.0 * (1.0 - (pressure / _SEA_LEVEL_PA) ** 0.190294957)
        self._sample = (altitude, temp_c, pressure)
        self._sample_ms = time.ticks_ms()
        return self._sample

    async def _recover(self) -> None:
        """
        Un-wedge a part that has stopped answering MID-FLIGHT, with the escalation setup() proved.

        setup() hardens the boot path against a latched core, but the run loop had no recovery at all:
        a read that started failing simply logged once and kept failing, so a mid-air latch-up cost the
        PRIMARY baro for the rest of the flight -- elevation drives the endgame band, the landing
        fallback and the launch backup, so that is expensive even with bmp280 behind it.

        Measured escalation (7/06 OOM soak, unchanged here): a NAK is usually just a conversion still
        draining, so wait one period first; only a PERSISTENT failure gets the I2C GENERAL-CALL reset
        (0x00 0x06), which is what actually recovered the latched bench part when the addressed soft
        reset re-wedged it. The general call also resets peers that honour it (bmp280, ina226) -- they
        re-apply their config-declared state, and this fires only when the primary baro is already lost.

        Args:
            (none)

        Returns:
            None; best-effort -- a still-dead part just fails the next read and tries again later.
        """
        await asyncio.sleep_ms(self._period_ms)  # a busy conversion drains on its own
        try:
            await self._bus.writeto(0x00, _GENERAL_CALL_RESET)
        except OSError:
            pass  # nothing honours general call -- nothing lost by asking
        await asyncio.sleep_ms(_RESET_RETRY_MS)
        recorder.Recorder.log(self.name, 'read recovery: general-call reset after %d failures'
                              % _RECOVER_AFTER)

    def calibration(self) -> str:
        """The ground-reference instruction; '' once a ground zero is held."""
        if self._ground is not None:
            return ''
        return 'stand the glider STILL on the pad -- this captures its ground reference'

    async def calibrate(self) -> str:
        """Re-capture ground zero from the live reading -- do it with the glider ON THE PAD."""
        try:
            self._ground = await self._ground_zero()
        except Exception as error:
            return 'ground zero: %s' % error
        recorder.Recorder.log(self.name, 'calibrated: ground %.1f m AMSL' % self._ground)
        return None

    async def run(self) -> None:
        while True:
            try:
                altitude, temp_c, pressure = await self._read()
                self.strike(False, _RECOVER_AFTER)  # good read rearms the run
                elevation = altitude - self._ground
                self._altitude.push(altitude)  # one step: push our channels directly
                self._temperature.push(temp_c)
                self._pressure.push(pressure)
                self._elevation.push(elevation)
                self._telemetry.push((altitude, temp_c, pressure, elevation))
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('icp10111 :: read %r', error)  # deduped: a persistent error logs once, not every tick
                if self.strike(True, _RECOVER_AFTER):  # once, not every tick while it stays dead
                    await self._recover()
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
        On-demand self-test: confirm the run loop is producing pressure.

        We issue NO I2C here -- the command-based ICP (write-measure then read) would race the run loop's
        own measure/read sequence on the shared bus; a present, healthy sensor instead keeps its channel
        fresh, so probe just waits for and checks it.

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) when no pressure appears.
        """
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
        """
        Deeper analysis when setup() failed: re-issue the product-id command and classify the fault.

        commons.id_classify masks the 2-byte word (& 0x3F, expecting 0x08). icp10111 is command-based
        (not register-mapped) so it cannot use i2cbus._Device.diagnose(), but the shared classifier still
        produces the same wire-level categories.

        Args:
            (none)

        Returns:
            A wire-level fault category; a config-fault message when setup never built the transport.
        """
        if self._bus is None:  # setup never built the transport
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
