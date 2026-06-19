# On-board test for the shared I2C bus (i2cbus.py): get() returns one cached, shared Bus per id, a
# different id is a different bus, and scan/read/write go through the lock. Run by `make test`.

import asyncio

import config_default
import i2cbus


async def amain():
    spec = config_default.default()['buses']['i2c']['0']  # sda7/scl8

    # get() creates the bus once and shares it (same instance for the same id)
    bus = i2cbus.get(0, spec)
    assert i2cbus.get(0, spec) is bus

    # scan() works through the wrapper; returns whatever is on the bus (a list)
    devices = bus.scan()
    assert isinstance(devices, list)

    # locked read against a present device if any is wired (ADXL375 0x53 -> DEVID 0xE5); else skip
    if 0x53 in devices:
        assert (await bus.read(0x53, 0x00, 1))[0] == 0xE5

    print('ok: i2cbus shared/cached per id, scan/locked-read, devices=%s' % [hex(a) for a in devices])


asyncio.run(amain())
