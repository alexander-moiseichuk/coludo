# On-board test for the ADXL375 driver (drivers/adxl375.py): @task.driver('adxl375') registration
# and graceful setup when the device is absent. Deterministic whether or not an ADXL375 is wired
# (it probes a bus/address that has nothing on it). Run by `make test`.

import asyncio

import config_default
import task
from drivers import adxl375


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('adxl375') is adxl375.Adxl375  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = adxl375.Adxl375('accel', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = adxl375.Adxl375('accel', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    print('ok: adxl375 driver registered; setup fails gracefully when no device answers')


asyncio.run(amain())
