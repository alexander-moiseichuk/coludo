"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Attitude REDUNDANCY: a complementary-filter backup for the BNO055 (coludo.md "Sensors Fusion/Backup").
The BNO055 is the sole fused-attitude source; losing it mid-flight would leave the flight loop with
stale/absent attitude -> neutral fins -> ballistic. This task derives (heading, roll, pitch) from the
LSM6DSO32 gyro `rate` + accel gravity vector and PROVIDES it on the databoard at PRIORITY 1, so the
existing timeout-handoff fusion swaps to it automatically the moment the BNO055 (priority 0) stops -- no
change to flight.py.

@task.activity('attitude'). Two regimes, checked each cycle by the fused-attitude SOURCE:
  * BNO055 alive (it is the fused source): MIRROR it -- copy roll/pitch (already fixnum cd) + heading,
    staying warm and FRESH so the handoff is seamless, no atan2/accel math (the BNO055 is trusted).
  * BNO055 lost (source is us / extrapolated): FREE-RUN -- integrate the gyro rate (integer) and, when
    |accel| ~ 1 g (a trustworthy gravity vector), pull roll/pitch toward the accel angle via the integer
    CORDIC fixed.atan2_cd (throttled -- drift correction is slow). Heading is gyro-z only (it drifts: the
    LSM6DSO32 has no magnetometer) -- roll/pitch stay solid (gravity-referenced), so the glider holds
    wings-level + pitch; nav heading degrades gracefully. Integer/fixnum throughout; the only boxed float
    is the heading value the channel format requires (nav consumes heading as float degrees).

Mounting (the gyro-D-term convention, HITL-validated): gx->roll, gy->pitch, gz->yaw; accel roll =
atan2(ay, az), pitch = atan2(-ax, |ay,az|). Field calibration flips a sign like the mixer gains.
"""

import asyncio
import time

import databoard
import fixed
import recorder
import task
from commons import const
from fixed import fixnum  # centidegree fixed-point -- the one control scale (roll/pitch/yaw AND accel)

_MAX_DT_MS = const(500)  # a gyro-integration gap longer than this (asyncio stall) -> clamp dt to nominal


@task.activity('attitude')
class Attitude(task.Task):
    """Complementary-filter attitude backup (heading, roll, pitch) at priority 1 behind the BNO055."""

    async def setup(self) -> bool:
        cfg = self.config
        self._period_ms: int = cfg.get('period_ms', 20)  # 50 Hz -- the backup publish/track rate
        self._accel_period_us: int = cfg.get('accel_period_ms', 50) * 1000  # accel correction throttle
        self._corr_shift: int = cfg.get('corr_shift', 4)  # accel pull = err >> shift (4 -> 1/16 per step)
        self._turn_gate: int = int(cfg.get('turn_gate_deg_s', 4) * fixed.SCALE)  # |yaw rate| cd/s -> gate accel
        self._course_gate: float = cfg.get('course_gate_mps', 5.0)  # min ground speed for a meaningful course
        """
        Magnetometer knobs. The mag is the only absolute yaw reference that does NOT need a GNSS fix, and
        boost is exactly when a consumer module loses lock -- so without it, heading runs free through the
        phase that generates the most drift.

        `mag_level_deg` is a LEVEL GATE, not a tuning nicety. Heading from a flat atan2(mag) is valid only
        while the airframe is near level: tilt-compensating it needs sin/cos, which the integer `fixed`
        module deliberately does not carry, and at this latitude's field inclination a 20 deg bank puts
        more than 10 deg into an uncompensated heading. Gating is honest where compensating would be
        quietly wrong.
        """
        self._mag_shift: int = cfg.get('mag_shift', 6)  # yaw pull toward magnetic heading (weak: 1/64)
        self._mag_offset_shift: int = cfg.get('mag_offset_shift', 5)  # how fast the offset is learned
        self._mag_level_cd: int = int(cfg.get('mag_level_deg', 15) * fixed.SCALE)  # |roll|,|pitch| limit
        self._course_shift: int = cfg.get('course_shift', 5)  # yaw pull toward the GNSS track (weak: 1/32)
        # trust the accel gravity vector only near 1 g -- store the squared centi-g band (no per-cycle sqrt)
        low = fixed.from_float(cfg.get('grav_low_g', 0.7))    # g -> centi-g fixnum (the standard boundary)
        high = fixed.from_float(cfg.get('grav_high_g', 1.3))
        """
        RECORD the output. On v1.1 (TMS-7F) this filter is the ONLY attitude source -- no BNO055, and a
        passive profile has no flight.csv -- yet it created no stream, so the flight meant to validate
        it as the sole source returned none of its output. Integers only (centidegrees, 0/1 flags): a
        float in the row is heap-boxed on a GC-off flight. Decimated to telemetry_ms (default 100 ms).
        """
        self._telemetry = recorder.Telemetry(
            'attitude.csv', ('heading_cd', 'roll_cd', 'pitch_cd', 'free', 'mag_known', 'mag_offset_cd'),
            decimate_us=cfg.get('telemetry_ms', 100) * 1000)
        self._grav_lo_sq: int = low * low  # a centi-g SQUARED magnitude (not a fixnum itself)
        self._grav_hi_sq: int = high * high
        self._roll_cd: fixnum = 0   # centidegree fixnum (matches the BNO055 attitude slot)
        self._pitch_cd: fixnum = 0
        self._yaw_cd: fixnum = 0
        self._mag_offset_cd: fixnum = 0  # magnetic heading -> ground track, LEARNED while the GNSS is good
        self._mag_known: bool = False    # ...and only usable once learned; see _magnetic_yaw()
        self._seeded: bool = False  # has the primary ever seeded us? (else free-run from 0)
        self._free: bool = False    # currently free-running (primary lost) -> inspect/telemetry
        self._last_us: int = time.ticks_us()
        self._accel_us: int = self._last_us
        self._attitude_param = databoard.Databoard.parameter('attitude')  # the FUSED attitude (source check)
        self._accel = databoard.Databoard.parameter('accel')
        self._rate = databoard.Databoard.parameter('rate')
        self._course = databoard.Databoard.parameter('course')  # GNSS ground-track bearing -> absolute yaw
        self._speed = databoard.Databoard.parameter('speed')    # ground speed -> course only when moving
        self._mag = databoard.Databoard.parameter('mag')        # raw magnetometer counts (v1.1 boards)
        self._attitude = databoard.Databoard.provide(self.name, cfg.get('provides', {}), 'attitude')
        self._ok = True
        return True

    def _mirror(self, value: tuple) -> None:
        """
        Track the higher-priority source while it is the fused winner, so the handoff is seamless.

        The winner is the BNO055 in flight, the sim's attitude in HITL. Copy its (already-fixnum)
        roll/pitch and heading, so we stay fresh and hand over the moment it dies next cycle.

        Args:
            value - the winning source's (heading FLOAT deg, roll cd, pitch cd).

        Returns:
            None; copies the source into our own state as a side effect.
        """
        heading, roll_cd, pitch_cd = value
        self._roll_cd = roll_cd            # already a centidegree fixnum
        self._pitch_cd = pitch_cd
        self._yaw_cd = fixed.from_float(heading)  # heading (float deg) -> centidegree fixnum for our state
        self._seeded = True

    def _magnetic_heading(self):
        """
        Heading from the magnetometer, or None when it cannot be trusted this step.

        FLAT atan2 of the horizontal axes -- which is why the level gate exists (see `mag_level_deg`).
        The result is in the SENSOR's magnetic frame: it carries declination and the mounting angle,
        which `_magnetic_yaw` learns as ONE constant offset, so no absolute reference is needed here.

        Static hard iron is NOT in that lump and must not be left to it: it shifts the heading by an
        amount that depends on where the nose points, so no single offset can remove it. The driver
        does, from a level circle the operator turns (bmm350.calibrate()); until that is done the
        device sits on the not-ready list.

        Args:
            (none)

        Returns:
            A centidegree fixnum, or None when there is no fresh mag or the airframe is not level.
        """
        mag, mag_source, _mag_age = self._mag.read()
        if mag_source is None or mag is None:
            return None
        if abs(((self._roll_cd + 18000) % 36000) - 18000) > self._mag_level_cd:
            return None
        if abs(((self._pitch_cd + 18000) % 36000) - 18000) > self._mag_level_cd:
            return None
        return fixed.atan2_cd(-mag[1], mag[0]) % 36000

    def _magnetic_yaw(self, tracking: bool, course) -> None:
        """
        Learn the magnetic-to-track offset while the GNSS is good; USE it when the GNSS is not.

        Two states, and the order matters. While a track is available the mag steers nothing -- it is
        being measured against the track, and the difference (declination + mounting + hard iron, as one
        lump) is low-passed into `_mag_offset_cd`. The moment the track goes -- a boost dropout, or a
        phase below the course gate -- that learned offset turns the mag into a standalone absolute
        reference, and yaw is pulled toward it exactly as it was toward the track.

        This learns ONE CONSTANT, and that is only sound because the driver removes the part that is not
        constant. Hard and soft iron shift the heading by an amount that DEPENDS on which way the nose
        points (measured at up to 214 degrees on the bench fixture), so no single offset can absorb
        them -- bmm350.calibrate() does, from a level circle the operator turns, leaving declination
        plus mounting, which genuinely is one number.

        On an UNCALIBRATED magnetometer this still runs, and is still worth having: the pull is bounded
        by a reference that is wrong-but-stationary rather than a gyro that is free-running. It is
        simply worth much less, which is why the device stays on the not-ready list until calibrated.

        Args:
            tracking - whether the GNSS track was usable this step (it takes precedence).
            course - the GNSS ground track in float degrees, when tracking.

        Returns:
            None; updates the learned offset, or nudges `self._yaw_cd` when the track is gone.
        """
        magnetic = self._magnetic_heading()
        if magnetic is None:
            return
        if tracking:
            target = ((fixed.from_float(course) - magnetic + 18000) % 36000) - 18000
            error = ((target - self._mag_offset_cd + 18000) % 36000) - 18000
            self._mag_offset_cd = (self._mag_offset_cd + (error >> self._mag_offset_shift)) % 36000
            self._mag_known = True
            return
        if not self._mag_known:
            return  # an unlearned offset is not a reference: free-running gyro beats a confidently wrong pull
        error = ((magnetic + self._mag_offset_cd - self._yaw_cd + 18000) % 36000) - 18000
        self._yaw_cd = (self._yaw_cd + (error >> self._mag_shift)) % 36000

    def _integrate(self, dt_ms: int) -> None:
        """
        Free-run: gyro-integrate roll/pitch/yaw, then re-anchor roll/pitch to the accel gravity vector.

        The accel pull fires ONLY when that vector is trustworthy: near 1 g AND not in a turn. In a
        coordinated turn the accel reads gravity+centripetal DOWN THE BODY AXIS, so its roll/pitch look
        level however hard the glider is banked; correcting to that would roll the estimate flat. The
        gyro integrates the true bank through the turn, so the accel correction is gated off there (yaw
        rate over turn_gate) and only re-anchors roll/pitch in straight-ish flight. The per-axis
        gyro-integrate + accel-blend is fixed.blend_cd (viper, zero float boxed).

        Args:
            dt_ms - the step interval in integer milliseconds, for the gyro integration.

        Returns:
            None; advances the free-run roll/pitch/yaw state as a side effect.
        """
        """
        read(), not value(), for every input to the BACKUP attitude.

        This task exists to keep an attitude when the BNO055 has failed -- the case where its inputs
        are most likely to be stale too. value() extrapolates without bound, so the backup would blend
        an invented gyro rate or gravity vector and report an attitude with the same confidence as a
        real one. A backup that cannot tell fresh from invented is not redundancy.

        gnss_calib and the wind feed were migrated for the same reason; this task was missed.
        """
        rate, rate_source, _rate_age = self._rate.read()
        if rate_source is None:
            rate = None
        roll_d = pitch_d = yaw_d = 0  # gyro-integration deltas this step (centidegree fixnum)
        turning = True
        if rate is not None:
            roll_d = rate[0] * dt_ms // 1000
            pitch_d = rate[1] * dt_ms // 1000
            yaw_d = rate[2] * dt_ms // 1000
            turning = abs(rate[2]) > self._turn_gate  # |yaw rate| -> coordinated-turn detector
        """
        yaw: gyro-integrate (wrapped), then pull toward the GNSS ground track when moving -- an ABSOLUTE
        reference that bounds the gyro drift (no magnetometer), and it is the TRACK, which is what the nav
        steers by anyway. Weak blend (course_shift) so a crosswind crab averages out.
        """
        self._yaw_cd = (self._yaw_cd + yaw_d) % 36000
        course, course_source, _course_age = self._course.read()
        speed, speed_source, _speed_age = self._speed.read()
        if course_source is None or speed_source is None:
            course = speed = None
        tracking = course is not None and speed is not None and speed > self._course_gate
        if tracking:
            err = ((fixed.from_float(course) - self._yaw_cd + 18000) % 36000) - 18000  # wrapped (-180,180] cd
            self._yaw_cd = (self._yaw_cd + (err >> self._course_shift)) % 36000
        self._magnetic_yaw(tracking, course)
        roll_accel: fixnum = 0
        pitch_accel: fixnum = 0
        correct = False
        now = time.ticks_us()
        if not turning and time.ticks_diff(now, self._accel_us) >= self._accel_period_us:
            self._accel_us = now
            accel, accel_source, _accel_age = self._accel.read()   # (ax, ay, az) float g, or None
            if accel_source is None:
                accel = None
            if accel is not None:
                axi = fixed.from_float(accel[0])  # the one float boundary: g -> centi-g fixnum for the CORDIC
                ayi = fixed.from_float(accel[1])  # (~0.5 deg typical over the glide envelope -- coludo.md)
                azi = fixed.from_float(accel[2])
                planar = ayi * ayi + azi * azi  # the (ay,az) magnitude squared, centi-g^2
                if self._grav_lo_sq <= axi * axi + planar <= self._grav_hi_sq:  # near 1 g -> trustworthy
                    roll_accel = fixed.atan2_cd(ayi, azi)
                    pitch_accel = fixed.atan2_cd(-axi, fixed.isqrt(planar))
                    correct = True
        self._roll_cd = fixed.blend_cd(self._roll_cd, roll_d, roll_accel, self._corr_shift, correct)
        self._pitch_cd = fixed.blend_cd(self._pitch_cd, pitch_d, pitch_accel, self._corr_shift, correct)

    async def run(self) -> None:
        """
        Mirror the primary while it is the fused source; free-run the filter when it is lost.

        Publish (heading, roll, pitch) at priority 1 every cycle to stay fresh for the handoff.

        Returns:
            None; loops forever, publishing the backup attitude once per cycle.
        """
        while True:
            await asyncio.sleep_ms(self._period_ms)
            now = time.ticks_us()
            dt_ms = time.ticks_diff(now, self._last_us) // 1000
            self._last_us = now
            if dt_ms > _MAX_DT_MS or dt_ms < 0:  # a long asyncio gap (I2C contention) -> nominal, not a huge
                dt_ms = self._period_ms  # single gyro-integration jump the yaw would then hold permanently
            value, source, _age = self._attitude_param.read()
            # source None = nothing fresh: read() then hands back an EXTRAPOLATED old value. Mirroring
            # that snapped a dead primary's last attitude back in after every >40 ms gap (a GC pause is
            # enough) and dropped the gyro integration it replaced -- free-run instead.
            if value is not None and source is not None and source != self.name:
                self._mirror(value)  # a higher-priority source is winning -> mirror it (stay warm/fresh)
                self._free = False
            elif self._seeded or self._accel.read()[1] is not None:
                self._integrate(dt_ms)  # the primary is gone (source is us / stale) -> free-run
                self._free = True
            else:
                continue  # never seeded and no accel yet -> nothing trustworthy to publish
            self._roll_cd = ((self._roll_cd + 18000) % 36000) - 18000   # wrap to (-180, 180] cd
            self._pitch_cd = ((self._pitch_cd + 18000) % 36000) - 18000
            self._yaw_cd %= 36000                                        # heading to [0, 360) cd
            self._attitude.push((fixed.to_float(self._yaw_cd), self._roll_cd, self._pitch_cd))  # heading float; r/p cd
            if self._telemetry.due(now):  # due() first: no row tuple built on the 50 Hz path unless it emits
                try:
                    self._telemetry.push((self._yaw_cd, self._roll_cd, self._pitch_cd, 1 if self._free else 0,
                                          1 if self._mag_known else 0, self._mag_offset_cd))
                except Exception as error:  # a full ring must never stop the only attitude source
                    self.note('attitude :: record %r', error)

    async def probe(self) -> str:
        """
        On-demand self-test: the gyro `rate` is present (the backup's core input).

        A dead gyro means no attitude backup -- surfaced pre-flight.

        Returns:
            None when the gyro rate is present; an error message string when it is missing.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: gyro rate ...')
            if self._rate.read()[1] is None:
                raise ValueError('no gyro rate -- attitude backup blind')
        except Exception as error:
            message = 'attitude backup: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    """Inspectable: the operator-facing backup-attitude snapshot (inspect/stats)."""

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status.update({'free_running': self._free, 'seeded': self._seeded,
                       'roll': fixed.to_str(self._roll_cd), 'pitch': fixed.to_str(self._pitch_cd),
                       'heading': fixed.to_str(self._yaw_cd),
                       # the magnetic reference, which is otherwise invisible: whether the track offset
                       # has been LEARNED (the mag steers nothing until it has) and what it converged to
                       'mag_known': self._mag_known, 'mag_offset': fixed.to_str(self._mag_offset_cd)})
        return status

    def stats(self) -> dict:
        return self.inspect()
