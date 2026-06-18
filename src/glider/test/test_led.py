# On-board test for the status LED driver (led.py): driver registration, pin setup from config,
# blink-period selection by state/health, inspect, and the missing-pin negative. Run by `make test`.

import asyncio

import config_default
import led
import task


class _StubController:
    """Stand-in for the Controller: just the bits the LED reads (config / state / validate)."""

    def __init__(self, config, state='setting', healthy=True):
        self.config = config
        self.state = state
        self._healthy = healthy

    def validate(self):
        return self._healthy


async def amain():
    # registered as a driver the Controller can build
    assert task.DRIVERS.get('led') is led.LedStatus

    cfg = config_default.default()
    component = {'name': 'led', 'driver': 'led', 'pin': 'led_status', 'enabled': True}

    # setup resolves the pin from config.pins and comes up healthy
    controller = _StubController(cfg, state='setting', healthy=True)
    blinker = led.LedStatus('led', component, controller)
    assert await blinker.setup() is True and blinker.validate()

    # status -> blink half-period: a positive period while setting, solid (None) when flying
    setting = blinker._half_period_ms()
    assert isinstance(setting, int) and setting > 0
    controller.state = 'gliding'
    assert blinker._half_period_ms() is None
    # an unhealthy task wins over flying, and blinks faster than standby (error is most urgent)
    controller._healthy = False
    error = blinker._half_period_ms()
    assert isinstance(error, int) and 0 < error < setting

    # inspect surfaces the live state + error
    snapshot = blinker.inspect()
    assert snapshot['name'] == 'led' and snapshot['state'] == 'gliding' and snapshot['error'] is True

    # negative: a missing pin role fails setup gracefully (no crash)
    no_pin = led.LedStatus('led', {'name': 'led', 'driver': 'led', 'pin': 'absent'}, _StubController({'pins': {}}))
    assert await no_pin.setup() is False

    print('ok: led driver registered, pin setup, blink-by-state/health, inspect, missing-pin negative')


asyncio.run(amain())
