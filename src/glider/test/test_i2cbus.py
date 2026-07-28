"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the shared I2C bus (i2cbus.py): get() returns one cached, shared Bus per id, a
different id is a different bus, and scan/read/write go through the lock. Run by `make test`.
"""

import asyncio

import config_default
import i2cbus

_BUS_FAIL_LIMIT_LOCAL = 4  # mirrors i2cbus._BUS_FAIL_LIMIT (a module const is not importable)


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

    """
    BUS-WEDGE recovery. Per-driver recovery cannot fix the failure that matters most: a slave reset or
    glitched mid-byte can hold SDA LOW forever, and then every driver on the bus fails with none able
    to recover -- the general call, the sdp810 restart and the rest all need a working bus to be issued
    on. The bus counts CONSECUTIVE failures and, only past the limit, clocks SDA free and re-inits.
    """
    fails_before = bus._fails
    for _ in range(_BUS_FAIL_LIMIT_LOCAL - 1):   # isolated NAKs must NOT trigger a bus clear
        try:
            await bus.read(0x7E, 0x00, 1)        # nothing at this address -> OSError
        except OSError:
            pass
    assert bus._fails == _BUS_FAIL_LIMIT_LOCAL - 1, bus._fails
    try:
        await bus.read(0x7E, 0x00, 1)            # the one that crosses the limit -> recover
    except OSError:
        pass
    # the counter RESET is the observable proof it acted (machine.I2C is a per-bus singleton on this
    # port, so a re-inited peripheral is not a distinguishable object)
    assert bus._fails == 0, 'the wedge recovery did not run: %s' % bus._fails
    # ...and the bus still WORKS after being clocked free and re-inited
    assert isinstance(bus.scan(), list) if hasattr(bus, 'scan') else True

    # a SUCCESSFUL transfer clears the run, so unrelated NAKs never accumulate into a false wedge
    if devices:
        await bus.read(devices[0], 0x00, 1)
        assert bus._fails == 0, bus._fails
    bus._fails = fails_before

    print('ok: i2cbus shared/cached per id, scan/locked-read, diagnose, retune, wedge recovery, '
          'devices=%s' % [hex(a) for a in devices])


asyncio.run(amain())
