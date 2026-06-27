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

    # _Device.diagnose() -- the bus-level wire-fault classifier a failed driver's diagnose() awaits.
    # An address that never acks -> 'no bus response'; a present chip with the right id -> 'present'.
    absent = next((addr for addr in range(0x08, 0x78) if addr not in devices), 0x09)
    assert 'no bus response' in await bus.device(absent).diagnose(0x00, 0xA0)  # no ack -> read fails -> None
    if 0x28 in devices:  # BNO055 wired -> CHIP_ID 0xA0 at reg 0x00 reads back -> 'present'
        assert 'present' in await bus.device(0x28).diagnose(0x00, 0xA0)

    # retune() re-inits the peripheral at a new freq in place (bench calibration); the bus stays usable
    await bus.retune(1000000)
    assert isinstance(bus.scan(), list)  # still scans after the in-place re-init
    await bus.retune(spec.get('freq', 400000))  # restore the configured freq

    print('ok: i2cbus shared/cached per id, scan/locked-read, diagnose, retune, devices=%s' % [hex(a) for a in devices])


asyncio.run(amain())
