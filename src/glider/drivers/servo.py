# drivers/servo.py — one fin servo (SG90) on a PWM pin. @task.driver('servo'), one instance per fin
# (yaw / left eleron / right eleron), each naming its `pin`. 50 Hz frame; the command unit is DEGREES,
# linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg) and CLAMPED to the range so
# a bad command can never drive the horn past the linkage. Set-and-hold like the bluetooth driver: no
# run loop -- setup() drives the servo to its neutral angle (transparent state).
#
# Two ways to command a fin:
#   update {"angle": d}  -- IMMEDIATE, ungated: the operator override (sync, returns at once).
#   await move(d)        -- GATED + settle-aware: passes through a SHARED slew gate so at most
#                           `servo_concurrency` (board config, default 3 = no limit) fins slew at
#                           once, then awaits the estimated travel so the caller knows it has (open-
#                           loop, no feedback) arrived. The flight control loop uses this; limiting N
#                           bounds the boost-rail transient when several fins would move together.
#
# Power: servos run off their own boost rail (per-pin diode protected); the board sources only the
# low-current signal on the PWM pin, never the servo supply.

import asyncio

import task

_PERIOD_US: int = 20000  # 50 Hz servo frame (20 ms)
_MS_PER_DEG: float = 2.5  # ~0.15 s/60deg SG90 slew estimate (open-loop -- no position feedback)
_SETTLE_MARGIN_MS: int = 60  # added to the slew estimate so move() returns after it has settled
_DEFAULT_CONCURRENCY: int = 3  # fins allowed to slew at once (== fin count -> no limit)

_gate = None  # the shared slew gate, created on the first servo setup


class _Gate:
    """A tiny FIFO counting semaphore (MicroPython asyncio has no Semaphore, only Lock/Event): at most
    `permits` holders at once, the rest queue and are handed a permit in order on release."""

    def __init__(self, permits: int):
        self._free: int = permits
        self._waiters: list = []

    async def acquire(self) -> None:
        if self._free > 0:
            self._free -= 1
        else:
            event = asyncio.Event()
            self._waiters.append(event)
            await event.wait()  # release() hands this waiter the permit directly; _free unchanged

    def release(self) -> None:
        if self._waiters:
            self._waiters.pop(0).set()  # direct hand-off to the next in line (FIFO)
        else:
            self._free += 1

    async def __aenter__(self):
        await self.acquire()
        return self

    async def __aexit__(self, *exception):
        self.release()


def _slew_gate(permits: int) -> _Gate:
    """The process-wide slew gate, created once (the first servo's `permits` wins)."""
    global _gate
    if _gate is None:
        _gate = _Gate(permits)
    return _gate


@task.driver('servo')
class Servo(task.Task):
    """One PWM fin servo, commanded in degrees (clamped to [min_deg, max_deg]). `update {"angle": d}`
    moves it immediately; `await move(d)` moves it through the shared slew gate. Inspect reports the
    current angle + pulse width."""

    async def setup(self) -> bool:
        gpio = self.controller.config.get('pins', {}).get(self.config.get('pin'))
        if gpio is None:
            return False
        from machine import PWM, Pin

        self._min_us: int = self.config.get('min_us', 500)  # pulse at min_deg (SG90 ~500..2500 us)
        self._max_us: int = self.config.get('max_us', 2500)  # pulse at max_deg
        self._min_deg: float = self.config.get('min_deg', 0)
        self._max_deg: float = self.config.get('max_deg', 180)
        self._gate = _slew_gate(self.controller.config.get('servo_concurrency', _DEFAULT_CONCURRENCY))
        self._pwm = PWM(Pin(gpio), freq=50, duty_u16=0)
        self._apply(self.config.get('angle', (self._min_deg + self._max_deg) / 2))  # neutral by default
        self._ok = True
        return True

    def _clamp(self, angle: float) -> float:
        return min(max(angle, self._min_deg), self._max_deg)

    def _apply(self, angle: float) -> float:
        """Clamp `angle`, map it to a pulse width and drive the PWM. Stores + returns the angle set."""
        angle = self._clamp(angle)
        span = self._max_deg - self._min_deg
        fraction = (angle - self._min_deg) / span if span else 0.0
        self._pulse_us = self._min_us + fraction * (self._max_us - self._min_us)
        self._pwm.duty_u16(round(self._pulse_us / _PERIOD_US * 65535))
        self.angle = angle
        return angle

    async def move(self, angle: float) -> float:
        """Drive to `angle` (clamped) through the shared slew gate -- at most servo_concurrency fins
        slew at once -- then await the estimated travel so the caller knows it has arrived (open-loop:
        the wait is a slew-rate estimate, not feedback). Returns the angle set."""
        target = self._clamp(angle)
        travel_ms = round(abs(target - self.angle) * _MS_PER_DEG) + _SETTLE_MARGIN_MS
        async with self._gate:
            self._apply(target)
            await asyncio.sleep_ms(travel_ms)
        return target

    def update(self, props: dict) -> list:
        """`{"angle": d}` moves the servo IMMEDIATELY (degrees, clamped) -- the operator override.
        Returns ['angle'] if set."""
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
        status['angle'] = self.angle
        status['pulse_us'] = round(self._pulse_us)
        return status
