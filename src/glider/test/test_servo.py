# On-board test for the servo driver (drivers/servo.py): @task.driver('servo') registration, the
# degrees->pulse mapping, range clamping, neutral-at-boot, and update/finish. Constructs real PWM on
# the configured fin pins (no servo needs to be attached). Run by `make test`.

import asyncio

import config_default
import task
from drivers import servo


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('servo') is servo.Servo  # registered driver

    # no pin -> graceful False, no PWM touched
    no_pin = servo.Servo('s', {}, _StubController())
    assert await no_pin.setup() is False

    # default range 0..180: neutral (90) at boot maps to the mid pulse (~1500 us)
    fin = servo.Servo('servo_yaw', {'pin': 'servo_yaw'}, _StubController())
    assert await fin.setup() is True and fin.angle == 90
    assert 1400 < fin.inspect()['pulse_us'] < 1600

    # update moves it (degrees); out-of-range commands clamp to the limits, not jam the linkage
    assert fin.update({'angle': 45}) == ['angle'] and fin.angle == 45
    fin.update({'angle': 999})
    assert fin.angle == 180  # clamped to max_deg
    fin.update({'angle': -50})
    assert fin.angle == 0  # clamped to min_deg
    assert fin.update({}) == []  # no 'angle' -> no-op
    await fin.finish()

    # a limited-throw fin (min/max -30..30, neutral 0): 0 is the mid -> ~1500 us; 90 clamps to 30
    geared = servo.Servo('servo_eleron_left', {'pin': 'servo_eleron_left', 'min_deg': -30,
                                               'max_deg': 30, 'angle': 0}, _StubController())
    assert await geared.setup() is True and geared.angle == 0
    assert 1400 < geared.inspect()['pulse_us'] < 1600
    geared.update({'angle': 90})
    assert geared.angle == 30
    await geared.finish()

    print('ok: servo registered; neutral at boot, degrees->pulse, clamp to range, update/finish')


asyncio.run(amain())
