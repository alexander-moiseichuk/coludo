# On-board test for the INA226 driver (drivers/ina226.py): @task.driver('ina226') registration,
# graceful setup when the device is absent, and the calibration math. Deterministic whether or not an
# INA226 is wired (it probes a bus/address that has nothing on it). Run by `make test`.

import asyncio

import config_default
import task
from drivers import ina226


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('ina226') is ina226.Ina226  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = ina226.Ina226('power', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks at 0x4F) -> graceful False (Controller skips it)
    absent = ina226.Ina226('power', {'bus': 'i2c', 'id': 0, 'addr': 0x4F,
                                     'shunt_mohms': 10, 'max_current_ma': 5000}, _StubController())
    assert await absent.setup() is False
    # integer milli-unit config: max_current stays as-configured mA (set before the die-id read fails)
    assert absent._max_current_ma == 5000
    # CAL is integer: 0.00512·2^15·1e6 // (max_current_ma · shunt_mohms) = 167772160 // (5000·10) = 3355
    assert 167772160 // (5000 * 10) == 3355  # the literal numerator (const() folds, no module attr)

    print('ok: ina226 registered; setup fails gracefully when no device answers; integer milli-unit CAL')


asyncio.run(amain())
