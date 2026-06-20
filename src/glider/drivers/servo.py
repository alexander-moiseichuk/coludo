# drivers/servo.py — one fin servo (SG90) on a PWM pin. @task.driver('servo'), one instance per fin
# (yaw / left eleron / right eleron), each naming its `pin`. 50 Hz frame; the command unit is DEGREES,
# linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg) and CLAMPED to the range so
# a bad command can never drive the horn past the linkage. Set-and-hold like the bluetooth driver: no
# run loop -- setup() drives the servo to its neutral angle (transparent state) and update {"angle":
# d} moves it live; the PWM keeps holding between commands.
#
# Power: servos run off their own boost rail (per-pin diode protected) and are driven sequentially
# under load -- the board only sources the low-current signal on the PWM pin, never the servo supply.

import task

_PERIOD_US: int = 20000  # 50 Hz servo frame (20 ms)


@task.driver('servo')
class Servo(task.Task):
    """One PWM fin servo, commanded in degrees (clamped to [min_deg, max_deg]). `update {"angle": d}`
    moves it; inspect reports the current angle + pulse width."""

    async def setup(self) -> bool:
        gpio = self.controller.config.get('pins', {}).get(self.config.get('pin'))
        if gpio is None:
            return False
        from machine import PWM, Pin

        self._min_us: int = self.config.get('min_us', 500)  # pulse at min_deg (SG90 ~500..2500 us)
        self._max_us: int = self.config.get('max_us', 2500)  # pulse at max_deg
        self._min_deg: float = self.config.get('min_deg', 0)
        self._max_deg: float = self.config.get('max_deg', 180)
        self._pwm = PWM(Pin(gpio), freq=50, duty_u16=0)
        self._apply(self.config.get('angle', (self._min_deg + self._max_deg) / 2))  # neutral by default
        self._ok = True
        return True

    def _apply(self, angle: float) -> float:
        """Clamp `angle` to the configured range, map it to a pulse width and drive the PWM. Stores +
        returns the angle actually set."""
        angle = min(max(angle, self._min_deg), self._max_deg)
        span = self._max_deg - self._min_deg
        fraction = (angle - self._min_deg) / span if span else 0.0
        self._pulse_us = self._min_us + fraction * (self._max_us - self._min_us)
        self._pwm.duty_u16(round(self._pulse_us / _PERIOD_US * 65535))
        self.angle = angle
        return angle

    def update(self, props: dict) -> list:
        """`{"angle": d}` moves the servo (degrees, clamped to the range). Returns ['angle'] if set."""
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
