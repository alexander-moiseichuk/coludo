# drivers/sg90.py — SG90 micro fin servo on a PWM pin. @task.driver('sg90'), one instance per fin
# (yaw / left eleron / right eleron), each naming its `pin`. 50 Hz frame; the command unit is INTEGER
# DEGREES, linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg, integer math) and
# CLAMPED to the range so a bad command can never drive the horn past the linkage.
#
# OPEN-LOOP -- NO POSITION FEEDBACK. A 3-wire SG90 (signal / V+ / GND) only RECEIVES a PWM command;
# the signal pin is input-only on the servo and there is no wire back, so the board CANNOT read where
# the horn actually is. Everything this driver reports (inspect()/telemetry `angle`, `pulse_us`) is
# the LAST COMMANDED value it tracks in software -- what we asked for, NOT a measurement. A stalled,
# force-held or jammed surface would still read the commanded target. inspect() carries
# `feedback: None` to make that explicit. (Real feedback would need a feedback servo, or tapping the
# internal pot to an ADC, or a current-sense on the rail.) Separately, this MicroPython-P4 build's PWM
# duty_u16()/duty_ns() GETTERS are broken (return a constant), so we cannot even read the commanded
# duty back from the peripheral -- the driver only ever WRITES it and remembers what it set.
#
# This class is SG90-specific on purpose. Other servos (MG90S, MG996R, ...) differ in pulse range and
# behaviour and would be their own @task.driver -- a new drivers/<type>.py subclassing this or
# standalone -- selected by the component's `driver` field. The shared slew gate + degree->pulse math
# live here for now; factor them into a servo base when a second type lands.
#
# Two ways to command a fin:
#   update {"angle": d}  -- IMMEDIATE, ungated: the operator override (sync, returns at once).
#   await move(d)        -- GATED + settle-aware: passes through a SHARED slew gate so at most
#                           `servo_concurrency` (board config, default 3 = no limit) fins slew at
#                           once, then awaits the estimated travel so the caller knows it has (open-
#                           loop, no feedback) arrived. The flight control loop uses this.
# Both record the command to per-fin telemetry (<name>.csv: angle, pulse_us, done) -- done=0 when a
# command is ISSUED, done=1 when a move() has (estimated) COMPLETED. probe() is the on-demand self-
# test (CC `probe`, pre-flight -- never at boot, so a reboot never sweeps fins): it sweeps the full
# range and returns to neutral, logging each step.
#
# Power: servos run off their own boost rail (per-pin diode protected); the board sources only the
# low-current signal on the PWM pin, never the servo supply.

import asyncio

import commons
import recorder
import servo
import task

_PERIOD_US: int = 20000  # 50 Hz servo frame (20 ms)
_SLEW_MS_PER_60: int = 150  # ~0.15 s / 60deg SG90 slew estimate (open-loop -- no position feedback)
_SETTLE_MARGIN_MS: int = 60  # added to the slew estimate so move() returns after it has settled
_DEFAULT_CONCURRENCY: int = 3  # fins allowed to slew at once (== fin count -> no limit)

# The N-slew concurrency gate is shared infrastructure (servo.py) so future servo types share one
# board-wide `servo_concurrency` budget.


@task.driver('sg90')
class SG90(task.Task):
    """One PWM SG90 fin servo, commanded in integer degrees (clamped to [min_deg, max_deg]). OPEN-LOOP
    -- reported angle is the last command, never a measurement (see module header; inspect carries
    `feedback: None`). `update {"angle": d}` moves it immediately; `await move(d)` moves it through the
    shared slew gate; probe() sweeps it on demand."""

    async def setup(self) -> bool:
        gpio = self.controller.config.get('pins', {}).get(self.config.get('pin'))
        if gpio is None:
            return False
        from machine import PWM, Pin

        self._min_us: int = self.config.get('min_us', 500)  # pulse at min_deg (SG90 ~500..2500 us)
        self._max_us: int = self.config.get('max_us', 2500)  # pulse at max_deg
        self._min_deg: int = self.config.get('min_deg', 0)
        self._max_deg: int = self.config.get('max_deg', 180)
        self._neutral: int = (self._min_deg + self._max_deg) // 2  # the zero position
        self._gate = servo.Gate.slew(self.controller.config.get('servo_concurrency', _DEFAULT_CONCURRENCY))
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('angle', 'pulse_us', 'done'),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._pwm = PWM(Pin(gpio), freq=50, duty_u16=0)
        self._apply(self.config.get('angle', self._neutral))  # neutral by default
        self._ok = True
        return True

    async def probe(self) -> str:
        """On-demand self-test (CC `probe`, pre-flight -- never at boot): sweep min -> max -> neutral so
        the fin is seen to travel, logging each step. Open-loop (no feedback) -> a step can only fail
        on a PWM/hardware error, and returns that step's message."""
        for label, target in (('min', self._min_deg), ('max', self._max_deg), ('neutral', self._neutral)):
            try:
                recorder.Recorder.log(self.name, 'probe: sweep to %s %d ...' % (label, target))
                await self.move(target)
                recorder.Recorder.log(self.name, 'probe: at %s %d ok' % (label, target))
            except Exception as error:
                message = 'sweep to %s %d: %s' % (label, target, error)
                recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
                return message
        return None

    def _clamp(self, angle) -> int:
        return commons.clamp_int(self._min_deg, round(angle), self._max_deg)

    def _apply(self, angle, done: int = 0) -> int:
        """Clamp `angle` to integer degrees, map to a pulse width (integer math), drive the PWM, and
        record the command to telemetry (done=0 issued / 1 completed). Stores + returns the angle."""
        angle = self._clamp(angle)
        span = self._max_deg - self._min_deg
        if span:
            self._pulse_us = self._min_us + (angle - self._min_deg) * (self._max_us - self._min_us) // span
        else:
            self._pulse_us = self._min_us
        self._pwm.duty_u16(self._pulse_us * 65535 // _PERIOD_US)
        self.angle = angle
        self._telemetry.push((angle, self._pulse_us, done))
        return angle

    async def move(self, angle) -> int:
        """Drive to `angle` (clamped, integer degrees) through the shared slew gate -- at most
        servo_concurrency fins slew at once -- then await the estimated travel so the caller knows it
        has arrived (open-loop: the wait is a slew estimate, not feedback). Records the command
        (done=0) and, after settling, the completion (done=1). Returns the angle."""
        target = self._clamp(angle)
        travel_ms = abs(target - self.angle) * _SLEW_MS_PER_60 // 60 + _SETTLE_MARGIN_MS
        async with self._gate:
            self._apply(target)  # done=0: commanded
            await asyncio.sleep_ms(travel_ms)
        self._telemetry.push((target, self._pulse_us, 1))  # done=1: (estimated) completed
        return target

    def update(self, props: dict) -> list:
        """`{"angle": d}` moves the servo IMMEDIATELY (integer degrees, clamped) -- the operator
        override. Returns ['angle'] if set."""
        if 'angle' in props:
            self._apply(props['angle'])
            return ['angle']
        return []

    async def finish(self) -> None:
        """Release the PWM (stop driving the pin) on shutdown."""
        self._pwm.deinit()
        await task.Task.finish(self)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['angle'] = self.angle  # COMMANDED, not measured -- SG90 is open-loop
        status['pulse_us'] = self._pulse_us
        status['feedback'] = None  # no position feedback on a 3-wire SG90
        return status
