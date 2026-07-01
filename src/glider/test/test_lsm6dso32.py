# On-board test for the LSM6DSO32 driver (drivers/lsm6dso32.py): @task.driver('lsm6dso32') registration
# and graceful setup when the device is absent (negative, deterministic), plus a POSITIVE case against the
# real device if one answers at the configured chip-select (accel ~1 g at rest, finite gyro). Run by
# `make test`.

import asyncio

import config_default
import task
from drivers import lsm6dso32


class _StubController:
    config = config_default.default()
    config['pins']['nc_cs'] = 52  # a free, unconnected GPIO for the spi-absent probe (nothing wired)


async def amain():
    assert task.ACTIVITIES.get('lsm6dso32') is lsm6dso32.Lsm6dso32  # registered driver

    # --- negative (deterministic whether or not the IMU is wired) ---
    no_bus = lsm6dso32.Lsm6dso32('imu', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()  # undefined bus -> graceful False

    absent = lsm6dso32.Lsm6dso32('imu', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False  # real bus, bogus address (nothing acks)

    spi_absent = lsm6dso32.Lsm6dso32('imu', {'bus': 'spi', 'id': 1, 'cs_pin': 'nc_cs'}, _StubController())
    assert await spi_absent.setup() is False  # SPI cs on a free, unconnected GPIO -> wrong WHO_AM_I

    no_cs = lsm6dso32.Lsm6dso32('imu', {'bus': 'spi', 'id': 1}, _StubController())
    assert await no_cs.setup() is False  # SPI selected, no cs_pin -> graceful False

    # --- positive (only when the real LSM6DSO32 answers on cs 50; skipped cleanly if unplugged) ---
    real = lsm6dso32.Lsm6dso32('imu', {'bus': 'spi', 'id': 1, 'cs_pin': 'lsm6dso32_cs',
                                       'int_pin': 'lsm6dso32_int1',
                                       'provides': {'accel': {'priority': 0}, 'rate': {'priority': 0}}},
                               _StubController())
    if await real.setup():
        await asyncio.sleep_ms(50)  # let the first 104 Hz conversion land before sampling
        ax, ay, az, gx, gy, gz = await real.sample()  # flat 6-tuple
        magnitude = (ax * ax + ay * ay + az * az) ** 0.5
        assert 0.5 < magnitude < 2.0, 'accel |a|=%.2f g not ~1 g at rest' % magnitude  # gravity
        assert max(abs(gx), abs(gy), abs(gz)) < 100.0, 'gyro %r dps too high at rest' % ((gx, gy, gz),)
        assert await real.probe() is None  # who_am_i + sample self-test clean
        print('ok: lsm6dso32 registered; graceful absent; REAL device |a|=%.2f g, gyro ok' % magnitude)
    else:
        print('ok: lsm6dso32 registered; graceful absent setup (no device wired on cs 50 -- positive skipped)')


asyncio.run(amain())
