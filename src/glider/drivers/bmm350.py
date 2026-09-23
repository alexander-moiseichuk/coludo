"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BMM350 3-axis magnetometer (on the SEN0697) over the shared I2C bus: the `mag` channel.

Why it is driven at all, having been dismissed once: heading. `tasks/attitude.py` integrates gyro-z for
heading and bounds the drift with the GNSS ground TRACK -- which only works above the course gate, and
not at all while the fix is out. Boost is exactly when a consumer GNSS module loses lock, so the one
reference that corrects heading is missing during the phase that introduces the most drift. A
magnetometer is the only sensor on this board that bounds heading WITHOUT a fix.

RAW, deliberately. Turning counts into microtesla needs each part's OTP compensation coefficients, and
that is worth skipping here: heading needs a DIRECTION, and the hard/soft-iron ellipsoid fit this
airframe needs anyway -- carbon, servo currents, a booster -- also absorbs per-axis sensitivity
differences. So this publishes counts, and the field calibration that must happen regardless does the
rest. Absolute field strength in microtesla is the only thing given up, and nothing here wants it.

Two BMM350 details worth stating: every I2C register read returns TWO DUMMY BYTES before the data (the
same quirk as the BMI323 beside it), and each axis is 24-bit two's complement, little-endian.

Register values are Bosch's own (bmm350_defs.h). @task.driver('bmm350').
"""

import asyncio
import struct

import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:
    from commons import const

try:
    from esp32 import NVS
except Exception:  # host / no NVS partition -- the calibration simply is not persisted
    _nvs = None
else:
    try:
        _nvs = NVS('coludo')
    except Exception:
        _nvs = None

_ADDR = const(0x15)  # I2C_ADSEL high (0x14 when pulled low)
_REG_CHIP_ID = const(0x00)
_REG_PMU_CMD_AGGR_SET = const(0x04)
_REG_PMU_CMD = const(0x06)
_REG_MAG_X = const(0x31)  # mag x/y/z then temperature, 3 bytes each -- 12 contiguous bytes
_REG_OTP_CMD = const(0x50)
_REG_CMD = const(0x7E)
_OTP_POWER_OFF = const(0x80)  # OTP_CMD_REG PWR_OFF_OTP -- until it is written, the part never measures
_NO_DATA = const(0x7F7F7F)  # what every axis reads while the OTP is still powered: not a field, no sample
_CHIP_ID = const(0x33)
_CMD_SOFT_RESET = const(0xB6)
_PMU_NORMAL = const(0x01)  # PMU_CMD_NM
_PMU_UPDATE_ODR = const(0x02)  # PMU_CMD_UPD_OAE -- applies a new ODR/averaging setting
_AGGR_100HZ_AVG4 = const(0x24)  # avg 4 (bits 5:4) | ODR 100 Hz (bits 3:0)
_DUMMY = const(2)  # bytes the part prepends to every I2C register read
_FRAME = const(14)  # _DUMMY + 3 axes x 3 bytes + temperature x 3
_NVS_KEY: str = 'bmm350_cal'  # 4 x int32: x centre, y centre, x radius, y radius
_CAL_BYTES = const(16)
_UNIT = const(1000)  # corrected axes are normalised to +/-_UNIT, so atan2 sees equal-scaled integers
_MIN_SPAN = const(2000)  # counts: a level turn spans ~8000-10000; still-board noise ~200 (see calibrate())
_MIN_REACH = const(1000)  # counts from the centre before a sample may set a sector (noise is ~+/-100)
_SECTORS = const(8)  # the circle is split into octants; coverage of them is what proves a real turn
_NEED_SECTORS = const(7)  # 7 of 8 = 315 degrees -- a lap that stops just short still counts
_MIN_SAMPLES = const(100)


@task.driver('bmm350')
class Bmm350(task.Task):
    """Raw magnetic field to the databoard as `mag` -- (x, y, z) counts, for heading."""

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        self._period_ms: int = self.config.get('period_ms', 100)  # 10 Hz; heading moves slowly
        self._buf = bytearray(_FRAME)
        try:
            if await self._chip_id() != _CHIP_ID:
                return False  # not a BMM350 at this address
            await self._configure()
        except Exception as error:
            print('bmm350 :: %r' % error)
            return False
        self._mag = databoard.Databoard.provide(self.name, self.config.get('provides', {}), 'mag')
        self._span = None  # [x_min, x_max, y_min, y_max] observed since boot -- the calibration evidence
        self._sectors: int = 0  # bitmask of octants visited around the provisional centre
        self._centre = None  # provisional centre the sector bits were accumulated against
        self._seen: int = 0
        self._calibration = None  # (x_centre, y_centre, x_radius, y_radius) once known
        self._restore()
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('mx', 'my', 'mz'),
                                             decimate_us=self.config.get('telemetry_ms', 0) * 1000)
        self._ok = True
        return True

    async def _chip_id(self) -> int:
        """
        The chip id, read PAST the two dummy bytes this part prepends to every I2C register read.

        The shared `read_chip_id()` helper reads one byte at the register, which for this part is a dummy
        -- so it never matches and setup() rejects a perfectly good sensor. That is exactly what happened
        on the first hardware bring-up, while the BMP581 beside it (no dummy bytes) came up fine.
        """
        buf = bytearray(_DUMMY + 1)
        await self._bus.read_into(self._addr, _REG_CHIP_ID, buf)
        return buf[_DUMMY]

    async def _configure(self) -> None:
        """
        Soft reset, power the OTP off, set the rate, then normal mode. The rate needs its own apply command.

        The OTP power-off is Bosch's own init step and it is NOT optional: without it every data register
        reads 0x7F and the part never produces a sample -- while its chip id, and so the old probe, read
        perfectly. That is how this driver shipped and passed bring-up on the SEN0697 without its
        magnetometer ever having measured a field: `mag` published a constant, the calibration refused
        it as a dead part, and the filter's magnetic yaw would have pulled toward a fixed vector.
        """
        await self._bus.write(self._addr, _REG_CMD, bytes([_CMD_SOFT_RESET]))
        await asyncio.sleep_ms(25)  # the part reloads its OTP after a reset
        await self._bus.write(self._addr, _REG_OTP_CMD, bytes([_OTP_POWER_OFF]))
        await asyncio.sleep_ms(5)
        await self._bus.write(self._addr, _REG_PMU_CMD_AGGR_SET, bytes([_AGGR_100HZ_AVG4]))
        await self._bus.write(self._addr, _REG_PMU_CMD, bytes([_PMU_UPDATE_ODR]))  # apply the rate
        await asyncio.sleep_ms(5)
        await self._bus.write(self._addr, _REG_PMU_CMD, bytes([_PMU_NORMAL]))
        await asyncio.sleep_ms(10)

    def _restore(self) -> None:
        """Load a saved hard/soft-iron calibration, so a power cycle does not cost the operator a turn."""
        if _nvs is None:
            return
        buffer = bytearray(_CAL_BYTES)
        try:
            if _nvs.get_blob(_NVS_KEY, buffer) != _CAL_BYTES:
                return  # a short blob is not a calibration -- leave the axes raw
        except Exception:
            return  # never calibrated on this board yet
        self._calibration = struct.unpack('<4i', buffer)

    def calibrated(self) -> bool:
        """Whether a hard/soft-iron calibration is in force (restored or just captured)."""
        return self._calibration is not None

    def _covered(self) -> int:
        """How many of the eight octants the turn has visited -- the progress the operator is working on."""
        return sum(1 for bit in range(_SECTORS) if self._sectors & (1 << bit))

    def calibration(self) -> str:
        """The outstanding instruction, or '' once calibrated -- what CC shows in the calibration column."""
        if self._calibration is not None:
            return ''
        if self._span is None:
            return 'turn the airframe through a full circle, LEVEL, then calibrate (no field seen yet)'
        return ('turn the airframe slowly through a FULL CIRCLE, keeping it level -- %d of %d sectors '
                'covered, %d samples' % (self._covered(), _SECTORS, self._seen))

    async def calibrate(self) -> str:
        """
        Capture hard/soft iron from the turn the operator has just done.

        Nothing is collected HERE: run() accumulates the extrema continuously, so by the time the operator
        presses this the evidence already exists. That is what makes it fit the one-press-per-device flow
        -- the physical act is theirs, and this is the half the board can do, exactly as the BNO055's is.

        What it removes is what a single learned offset cannot: the circle's CENTRE (hard iron, a fixed
        field from servos and steel) and its OUT-OF-ROUNDNESS (soft iron and per-axis scale). Both shift
        heading by an amount that depends on which way the nose points, so attitude.py's one learned
        track offset cannot absorb either.

        Args:
            (none)

        Returns:
            None once captured and persisted; otherwise the instruction still outstanding, as a failure
            string -- never a bare success for a turn that never happened.
        """
        if self._span is None or self._seen < _MIN_SAMPLES:
            return self.calibration()
        x_min, x_max, y_min, y_max = self._span
        x_span, y_span = x_max - x_min, y_max - y_min
        """
        Refuse a partial turn -- on ANGULAR COVERAGE, not on the shape of the spans.

        Measured: a quarter turn from 0 to 90 degrees has x span == y span exactly, so it scores a
        PERFECT roundness of 1.00 while putting the centre 450 counts off (a 30 degree heading error).
        Span shape cannot see it. Nor can it be allowed to: a real soft iron makes the circle genuinely
        elliptical, and that ellipse is the thing this calibration CORRECTS, so rejecting it would refuse
        exactly the boards that need the correction most. Octant coverage separates the two -- a partial
        turn misses sectors however round its bounding box looks.

        _MIN_SPAN is the floor for a motionless or broken magnetometer, now set from a MEASURED field: on
        the v1.1 bench (2026-09-23) the part's field sphere fitted at ~10 000 counts radius, so a level turn
        spans roughly 8 000-10 000 counts while a still board's noise spans ~200. It used to be 50 --
        below that noise -- which let a board that never moved pass as calibrated.
        """
        if max(x_span, y_span) < _MIN_SPAN:
            return 'no field change seen (x span %d, y span %d) -- is the magnetometer alive?' % (
                x_span, y_span)
        if self._covered() < _NEED_SECTORS:
            # built here, NOT via calibration(): that returns '' once a calibration exists (restored from
            # NVS), so a refused RE-calibration read as success and the old one silently stayed
            return 'refused -- only %d of %d sectors covered: turn the airframe through a full LEVEL circle' % (
                self._covered(), _SECTORS)
        calibration = ((x_min + x_max) // 2, (y_min + y_max) // 2, x_span // 2, y_span // 2)
        if _nvs is not None:
            try:
                _nvs.set_blob(_NVS_KEY, struct.pack('<4i', *calibration))
                _nvs.commit()
            except Exception as error:
                return 'calibration computed but not persisted: %r' % error
        self._calibration = calibration
        recorder.Recorder.log(self.name, 'calibrated: centre (%d, %d) radii (%d, %d)' % calibration)
        return None

    def _cover(self, x: int, y: int) -> None:
        """
        Record which octant of the circle this sample falls in, around the CURRENT centre estimate.

        The centre is only known once the turn is done, so the estimate moves while the operator turns
        and early bits would be recorded against a centre that no longer exists. So whenever the estimate
        MOVES materially the mask is thrown away and coverage restarts -- the bits always describe one
        settled centre. The cost is that a lap which shifts the estimate late asks for a bit more turning
        rather than accepting a half-measured circle; the instruction shows the count, so the operator
        can see it climbing instead of guessing.

        Octants come from three comparisons (two signs and |dx| vs |dy|), so there is no atan2 in the
        50 Hz path.

        Args:
            x, y - the raw horizontal axes of this sample.
        """
        x_min, x_max, y_min, y_max = self._span
        centre = ((x_min + x_max) // 2, (y_min + y_max) // 2)
        span = max(x_max - x_min, y_max - y_min)
        if self._centre is None or (abs(centre[0] - self._centre[0]) > span // 8 or
                                    abs(centre[1] - self._centre[1]) > span // 8):
            self._centre, self._sectors = centre, 0
        delta_x, delta_y = x - centre[0], y - centre[1]
        # samples hugging the centre have no meaningful angle. The floor is ABSOLUTE: relative to the span
        # alone, a board sitting still (span = its own noise, ~200 counts) reached 8 of 8 sectors in a few
        # hundred samples and calibrate() would have saved that noise as the hard-iron calibration
        reach = max(span // 4, _MIN_REACH)
        if delta_x * delta_x + delta_y * delta_y < reach * reach:
            return
        octant = ((1 if delta_y >= 0 else 0) << 2 | (1 if delta_x >= 0 else 0) << 1 |
                  (1 if abs(delta_x) >= abs(delta_y) else 0))
        self._sectors |= 1 << octant

    def _axis(self, offset: int) -> int:
        """One 24-bit two's-complement axis, little-endian, from the frame at `offset`."""
        raw = self._buf[offset] | (self._buf[offset + 1] << 8) | (self._buf[offset + 2] << 16)
        return raw - (1 << 24) if raw & 0x800000 else raw

    async def _read(self) -> tuple:
        """
        (mx, my, mz): raw counts, or hard/soft-iron CORRECTED and normalised once calibrated.

        Correction re-centres each axis on the circle's centre and scales both to +/-_UNIT, which is all
        a heading needs -- atan2 cares about the ratio, not the magnitude. Uncalibrated, the raw counts
        pass through unchanged, so the channel works from the first boot and improves when the operator
        does the turn.
        """
        await self._bus.read_into(self._addr, _REG_MAG_X, self._buf)
        x, y, z = self._axis(_DUMMY), self._axis(_DUMMY + 3), self._axis(_DUMMY + 6)
        if self._span is None:
            self._span = [x, x, y, y]
        else:
            self._span[0], self._span[1] = min(self._span[0], x), max(self._span[1], x)
            self._span[2], self._span[3] = min(self._span[2], y), max(self._span[3], y)
        self._seen += 1
        self._cover(x, y)
        if self._calibration is None:
            return x, y, z
        x_centre, y_centre, x_radius, y_radius = self._calibration
        return ((x - x_centre) * _UNIT // (x_radius or 1),
                (y - y_centre) * _UNIT // (y_radius or 1), z)

    async def rearm(self) -> None:
        """
        Re-apply the mode after something reset the part underneath it.

        A reset leaves it SUSPENDED, where it answers reads with its last sample forever -- a heading
        that quietly stops moving, which is worse than one that stops arriving. Same guard, same reason,
        as the baro drivers carry.

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
                sample = await self._read()
                self._mag.push(sample)
                self._telemetry.push(sample)
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('bmm350 :: read %r', error)  # deduped: a persistent error logs once
            await asyncio.sleep_ms(self._period_ms)

    async def probe(self) -> str:
        """
        On-demand self-test: the chip id reads back, then one sample reads (each step logged).

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) at the first failing step.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: chip id ...')
            chip = await self._chip_id()
            if chip != _CHIP_ID:
                raise ValueError('BMM350 id 0x%02x != 0x%02x at i2c:%s 0x%02x' % (
                    chip, _CHIP_ID, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: chip id ok 0x%02x' % chip)
        except Exception as error:
            message = 'chip id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: read ...')
            mx, my, mz = await self._read()
            """
            A suspended part answers every read with the same counts, which a bare "it answered" probe
            passes. All-zero is the signature worth refusing: the field is never zero on Earth, so zero
            means the part is not measuring rather than measuring nothing.
            """
            if mx == 0 and my == 0 and mz == 0:
                raise ValueError('all axes zero -- the part is not measuring')
            # the RAW frame, not the corrected axes: 0x7F on every byte is the part's own "no sample" --
            # what it reads until its OTP is powered off -- and a calibration would disguise it
            if self._axis(_DUMMY) == _NO_DATA and self._axis(_DUMMY + 3) == _NO_DATA:
                raise ValueError('axes read 0x7F7F7F -- the part is not measuring (OTP still powered?)')
            recorder.Recorder.log(self.name, 'probe: read ok (%d, %d, %d)' % (mx, my, mz))
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
        # NOT bus.device().diagnose(): that helper reads one byte at the register, which on this part
        # is a dummy byte, so it would report a healthy sensor as the wrong device.
        try:
            chip = await self._chip_id()
        except Exception as error:
            return 'no ack at i2c:%s 0x%02x -- %r' % (self.config.get('id'), self._addr, error)
        if chip != _CHIP_ID:
            return 'wrong device at i2c:%s 0x%02x: id 0x%02x, expected 0x%02x' % (
                self.config.get('id'), self._addr, chip, _CHIP_ID)
        return 'present and answering, but setup did not complete'

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status.update({'addr': hex(self._addr), 'mag_counts': self._mag.value(),
                       'calibrated': self.calibrated(), 'span': self._span, 'samples': self._seen,
                       'sectors': self._covered()})
        return status
