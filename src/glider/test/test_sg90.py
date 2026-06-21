# On-board test for the SG90 servo driver (drivers/sg90.py): @task.driver('sg90') registration, the
# integer degrees->pulse mapping, range clamping, neutral-at-boot, update/finish, move() + the N-slew
# gate, the open-loop `feedback: None` marker, and the probe() sweep. Constructs real PWM on the
# configured fin pins (no servo needs to be attached). Run by `make test`.

import asyncio

import config_default
import recorder
import task
from drivers import sg90


class _FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _StubController:
    config = config_default.default()


async def amain():
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())  # the servo logs to <name>.csv
    assert task.ACTIVITIES.get('sg90') is sg90.SG90  # registered driver

    # no pin -> graceful False, no PWM touched
    no_pin = sg90.SG90('s', {}, _StubController())
    assert await no_pin.setup() is False

    # default range 0..180: neutral (90) at boot maps to the mid pulse (~1500 us); open-loop marker
    fin = sg90.SG90('servo_yaw', {'pin': 'servo_yaw'}, _StubController())
    assert await fin.setup() is True and fin.angle == 90
    assert 1400 < fin.inspect()['pulse_us'] < 1600
    assert fin.inspect()['feedback'] is None  # SG90 has no position feedback

    # update moves it (degrees); out-of-range commands clamp to the limits, not jam the linkage
    assert fin.update({'angle': 45}) == ['angle'] and fin.angle == 45
    fin.update({'angle': 999})
    assert fin.angle == 180  # clamped to max_deg
    fin.update({'angle': -50})
    assert fin.angle == 0  # clamped to min_deg
    assert fin.update({}) == []  # no 'angle' -> no-op
    await fin.finish()

    # a limited-throw fin (min/max -30..30, neutral 0): 0 is the mid -> ~1500 us; 90 clamps to 30
    geared = sg90.SG90('servo_eleron_left', {'pin': 'servo_eleron_left', 'min_deg': -30,
                                             'max_deg': 30, 'angle': 0}, _StubController())
    assert await geared.setup() is True and geared.angle == 0
    assert 1400 < geared.inspect()['pulse_us'] < 1600
    geared.update({'angle': 90})
    assert geared.angle == 30
    await geared.finish()

    # move() drives to the clamped angle through the slew gate and returns it (settle-aware)
    fin2 = sg90.SG90('servo_yaw', {'pin': 'servo_yaw'}, _StubController())
    await fin2.setup()
    assert await fin2.move(120) == 120 and fin2.angle == 120
    assert await fin2.move(999) == 180  # clamped to max
    await fin2.finish()

    # (the N-slew gate itself is covered by test_servo; move() above exercises it via the driver)

    # probe() sweeps the range and fixes at neutral (zero), returning None (open-loop self-test)
    fin3 = sg90.SG90('servo_yaw', {'pin': 'servo_yaw'}, _StubController())
    await fin3.setup()
    assert await fin3.probe() is None
    assert fin3.angle == 90 and isinstance(fin3.angle, int)  # ended at neutral, integer degrees
    await fin3.finish()

    print('ok: sg90 int degrees/clamp/update/finish + move() (gated), feedback:None, probe() sweep')


asyncio.run(amain())
