# On-board test for the ICP-10111 driver (drivers/icp10111.py): @task.driver('icp10111')
# registration, graceful setup when absent, and that the TDK polynomial conversion is sane.
# Deterministic whether or not an ICP-10111 is wired. Run by `make test`.

import asyncio

import config_default
import task
from drivers import icp10111


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('icp10111') is icp10111.Icp10111  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = icp10111.Icp10111('baro', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = icp10111.Icp10111('baro', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    # conversion against real OTP + raw values captured live from the wired sensor -> ~101797 Pa
    probe = icp10111.Icp10111('baro', {}, _StubController())
    probe._otp = [211, 371, 553, 3833]
    pa = probe._compensate(11441968, 27034)
    assert 100000.0 < pa < 103000.0, pa  # matches the live ~1017.97 hPa reading

    print('ok: icp10111 driver registered; graceful-absent; conversion ~%d Pa' % pa)


asyncio.run(amain())
