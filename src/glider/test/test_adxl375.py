# On-board test for the ADXL375 driver (drivers/adxl375.py): @task.driver('adxl375') registration
# and graceful setup when the device is absent. Deterministic whether or not an ADXL375 is wired
# (it probes a bus/address that has nothing on it). Run by `make test`.

import asyncio

import config_default
import task
from drivers import adxl375


class _StubController:
    config = config_default.default()
    config['pins']['nc_cs'] = 52  # a free, unconnected GPIO for the spi-absent probe (nothing wired)


async def amain():
    assert task.ACTIVITIES.get('adxl375') is adxl375.Adxl375  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = adxl375.Adxl375('accel', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = adxl375.Adxl375('accel', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    # SPI bus but the chip-select points at a free, unconnected GPIO -> nothing answers -> False
    # (uses a non-ADXL cs so the test stays deterministic now that the real ADXL is wired on cs 49)
    spi_absent = adxl375.Adxl375('accel', {'bus': 'spi', 'id': 1, 'cs_pin': 'nc_cs'}, _StubController())
    assert await spi_absent.setup() is False

    # SPI selected but no cs_pin declared -> graceful False (no chip-select to drive)
    no_cs = adxl375.Adxl375('accel', {'bus': 'spi', 'id': 1}, _StubController())
    assert await no_cs.setup() is False

    print('ok: adxl375 registered; graceful setup over i2c (no ack) and spi (id mismatch / no cs)')


asyncio.run(amain())
