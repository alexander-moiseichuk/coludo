# On-board test for the BNO055 driver (drivers/bno055.py): @task.driver('bno055') registration and
# graceful setup when the device is absent. Deterministic whether or not a BNO055 is wired (it probes
# a bus/address with nothing on it). Run by `make test`.

import asyncio

import config_default
import task
from drivers import bno055


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('bno055') is bno055.Bno055  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bno055.Bno055('imu', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bno055.Bno055('imu', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    print('ok: bno055 driver registered; setup fails gracefully when no device answers')


asyncio.run(amain())
