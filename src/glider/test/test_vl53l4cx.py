# On-board test for the VL53L4CX driver (drivers/vl53l4cx.py): @task.driver('vl53l4cx') registration
# and graceful setup when the device is absent. Deterministic whether or not a VL53L4CX is wired (it
# probes a bus/address that has nothing on it). Run by `make test`.

import asyncio

import config_default
import task
from drivers import vl53l4cx


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('vl53l4cx') is vl53l4cx.Vl53l4cx  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = vl53l4cx.Vl53l4cx('laser', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = vl53l4cx.Vl53l4cx('laser', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    print('ok: vl53l4cx registered; setup fails gracefully when no device answers')


asyncio.run(amain())
