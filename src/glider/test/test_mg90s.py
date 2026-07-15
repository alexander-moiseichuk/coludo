"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the MG90S servo driver (drivers/mg90s.py): @task.driver('mg90s') registration, that
it is a thin SG90 subclass (trim / clamp / move / probe inherited), the metal-gear TYPE defaults
(quicker slew, higher probe draw window), and the 360deg-travel yaw mapping (min_deg -90 / max_deg 270
-> neutral 90 = servo mid = 1500 us, 1 command-deg = 1 rotation-deg). Builds real PWM on the yaw fin
pin (no servo needs to be attached). Run by `make test`.
"""

import asyncio

import config_default
import recorder
import task
from drivers import mg90s, sg90


class _FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _StubController:
    config = config_default.default()


async def amain():
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())  # the servo logs to <name>.csv
    assert task.ACTIVITIES.get('mg90s') is mg90s.MG90S  # registered driver
    assert issubclass(mg90s.MG90S, sg90.SG90)  # thin subclass -- inherits the SG90 logic

    # metal-gear TYPE defaults override SG90's (quicker slew, draw window shifted up)
    assert mg90s.MG90S._SLEW_MS_PER_60 == 100 and sg90.SG90._SLEW_MS_PER_60 == 150
    assert mg90s.MG90S._ENGINE_MIN_MW == 600 and mg90s.MG90S._ENGINE_MAX_MW == 5000

    # the 360deg-travel yaw instance, centred on the mixer neutral (min_deg -90 / max_deg 270): neutral
    # 90 = servo mid = 1500 us; the type's draw window is inherited (no component override)
    fin = mg90s.MG90S('servo_yaw', {'pin': 'servo_yaw', 'min_deg': -90, 'max_deg': 270}, _StubController())
    assert await fin.setup() is True and fin.angle == 90  # neutral = (-90 + 270) // 2
    assert fin.inspect()['pulse_us'] == 1500  # command 90 -> servo mid
    assert fin._engine_min_mw == 600 and fin._engine_max_mw == 5000  # from the MG90S class defaults
    assert fin.inspect()['feedback'] is None  # inherited: open-loop

    # 1 command-deg = 1 rotation-deg over the centred 360: +/-45 command -> +/-250 us from mid
    fin.update({'angle': 135})
    assert fin.angle == 135 and fin.inspect()['pulse_us'] == 1750  # 500 + (135+90)*2000//360
    fin.update({'angle': 45})
    assert fin.angle == 45 and fin.inspect()['pulse_us'] == 1250
    fin.update({'angle': 999})
    assert fin.angle == 270  # clamped to max_deg (full one-way travel)
    fin.update({'angle': -999})
    assert fin.angle == -90  # clamped to min_deg

    # per-fin trim is inherited (degrees): +9 deg over the 360 span = +50 us
    fin.update({'angle': 90, 'trim': 9})
    assert fin.inspect()['trim'] == 9 and fin.inspect()['pulse_us'] == 1550  # 500 + (90+9+90)*2000//360
    await fin.finish()

    # a 180deg MG90S drops in like an SG90 (default travel): neutral 90 -> mid pulse, only the slew differs
    stock = mg90s.MG90S('servo_yaw', {'pin': 'servo_yaw'}, _StubController())
    assert await stock.setup() is True and stock.angle == 90
    assert 1400 < stock.inspect()['pulse_us'] < 1600
    assert await stock.move(135) == 135 and stock.angle == 135  # inherited slew gate + move()
    await stock.finish()

    print('ok: mg90s -- registered SG90 subclass, metal-gear defaults, centred 360deg travel, trim/move inherited')


asyncio.run(amain())
