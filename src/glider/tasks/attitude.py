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
from fixed import fixnum  # centidegree fixed-point -- the one control scale (roll/pitch/yaw AND accel)


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
        self._course_shift: int = cfg.get('course_shift', 5)  # yaw pull toward the GNSS track (weak: 1/32)
        # trust the accel gravity vector only near 1 g -- store the squared centi-g band (no per-cycle sqrt)
        low = fixed.from_float(cfg.get('grav_low_g', 0.7))    # g -> centi-g fixnum (the standard boundary)
        high = fixed.from_float(cfg.get('grav_high_g', 1.3))
        self._grav_lo_sq: int = low * low  # a centi-g SQUARED magnitude (not a fixnum itself)
        self._grav_hi_sq: int = high * high
        self._roll_cd: fixnum = 0   # centidegree fixnum (matches the BNO055 attitude slot)
        self._pitch_cd: fixnum = 0
        self._yaw_cd: fixnum = 0
        self._seeded: bool = False  # has the primary ever seeded us? (else free-run from 0)
        self._free: bool = False    # currently free-running (primary lost) -> inspect/telemetry
        self._last_us: int = time.ticks_us()
        self._accel_us: int = self._last_us
        self._attitude_param = databoard.Databoard.parameter('attitude')  # the FUSED attitude (source check)
        self._accel = databoard.Databoard.parameter('accel')
        self._rate = databoard.Databoard.parameter('rate')
        self._course = databoard.Databoard.parameter('course')  # GNSS ground-track bearing -> absolute yaw
        self._speed = databoard.Databoard.parameter('speed')    # ground speed -> course only when moving
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
        rate = self._rate.value()  # (gx, gy, gz) centideg/s fixnum, or None
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
        course = self._course.value()
        speed = self._speed.value()
        if course is not None and speed is not None and speed > self._course_gate:
            err = ((fixed.from_float(course) - self._yaw_cd + 18000) % 36000) - 18000  # wrapped (-180,180] cd
            self._yaw_cd = (self._yaw_cd + (err >> self._course_shift)) % 36000
        roll_accel: fixnum = 0
        pitch_accel: fixnum = 0
        correct = False
        now = time.ticks_us()
        if not turning and time.ticks_diff(now, self._accel_us) >= self._accel_period_us:
            self._accel_us = now
            accel = self._accel.value()  # (ax, ay, az) float g, or None
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
            value, source, _age = self._attitude_param.read()
            if value is not None and source != self.name:
                self._mirror(value)  # a higher-priority source is winning -> mirror it (stay warm/fresh)
                self._free = False
            elif self._seeded or self._accel.value() is not None:
                self._integrate(dt_ms)  # the primary is gone (source is us / stale) -> free-run
                self._free = True
            else:
                continue  # never seeded and no accel yet -> nothing trustworthy to publish
            self._roll_cd = ((self._roll_cd + 18000) % 36000) - 18000   # wrap to (-180, 180] cd
            self._pitch_cd = ((self._pitch_cd + 18000) % 36000) - 18000
            self._yaw_cd %= 36000                                        # heading to [0, 360) cd
            self._attitude.push((fixed.to_float(self._yaw_cd), self._roll_cd, self._pitch_cd))  # heading float; r/p cd

    async def probe(self) -> str:
        """
        On-demand self-test: the gyro `rate` is present (the backup's core input).

        A dead gyro means no attitude backup -- surfaced pre-flight.

        Returns:
            None when the gyro rate is present; an error message string when it is missing.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: gyro rate ...')
            if self._rate.value() is None:
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
                       'heading': fixed.to_str(self._yaw_cd)})
        return status

    def stats(self) -> dict:
        return self.inspect()
