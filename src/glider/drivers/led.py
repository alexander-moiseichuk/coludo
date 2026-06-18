# led.py — status LED driver. One GPIO shows the board state at a glance: fast blink when a task is
# unhealthy (error), slow blink while setting up / standing by, solid once flying. The pin role
# (default 'led_status') comes from the component's `pin` field, resolved against the config `pins`
# section. Registered as @task.driver('led') so the Controller creates and supervises it.

import asyncio

import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_BLINK_ERROR_MS = const(100)  # fast: a task is unhealthy
_BLINK_SETTING_MS = const(500)  # slow: setting up / standby
_SOLID_TICK_MS = const(200)  # re-check cadence while held solid (flying)


@task.driver('led')
class LedStatus(task.Task):
    """Blink a status pattern on one GPIO derived from the controller's state + health."""

    async def setup(self) -> bool:
        from machine import Pin

        pins = self.controller.config.get('pins', {})
        gpio = pins.get(self.config.get('pin', 'led_status'))
        if gpio is None:
            return False
        self._pin = Pin(gpio, Pin.OUT)
        self._pin.value(0)
        self._ok = True
        return True

    def _half_period_ms(self):
        """Blink half-period for the current status, or None to hold the LED solid (flying)."""
        if not self.controller.validate():
            return _BLINK_ERROR_MS  # an unhealthy task wins -> fast blink
        if self.controller.state == 'setting':
            return _BLINK_SETTING_MS
        return None

    async def run(self) -> None:
        level = 0
        while True:
            half = self._half_period_ms()
            if half is None:
                self._pin.value(1)
                await asyncio.sleep_ms(_SOLID_TICK_MS)
            else:
                level ^= 1
                self._pin.value(level)
                await asyncio.sleep_ms(half)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['state'] = self.controller.state
        status['error'] = not self.controller.validate()
        return status
