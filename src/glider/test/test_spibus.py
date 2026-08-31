"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the shared SPI bus (spibus.py) -- the only bus module that had no test, while it is
the one carrying BOTH IMUs. get() caches one Bus per id, bind() resolves a device block to it,
device(cs) frames a transaction, diagnose() classifies a wire fault, and `mb_bit` drives the
auto-increment convention that DIFFERS between the two devices on this bus. Run by `make test`.

Why mb_bit gets a functional test rather than a value assertion: it selects whether the command byte
carries an address bit that means "auto-increment". The ADXL family uses bit 6; the LSM6DSO32 does not
(it increments from CTRL3_C.IF_INC instead, so spibus is passed mb_bit=None). Get it wrong and a
multi-byte read silently returns THE SAME REGISTER repeated -- values that look plausible, never an
error. So the test reads two registers in one transfer and asserts the second byte differs from the
first, which is exactly what repeats-instead-of-increments would break.
"""

import asyncio

import config_default
import spibus

_ADXL_DEVID: int = 0x00  # ADXL375 device-id register -> 0xE5
_ADXL_ID: int = 0xE5
_LSM_WHOAMI: int = 0x0F  # LSM6DSO32 WHO_AM_I -> 0x6C
_LSM_ID: int = 0x6C
_SETTLE_READS: int = 24  # bounded; spibus discards up to 16 itself, this covers the residue
_LSM_CTRL3_C: int = 0x12  # BDU + IF_INC (auto-increment) + SIM (4-wire SPI)
_LSM_CFG_C: int = 0x44    # the value the driver writes at setup -- idempotent, so safe to repeat here


async def _settled(window, reg: int, expected: int) -> int:
    """
    Read `reg` until it returns `expected`, up to _SETTLE_READS times; returns how many reads it took.

    This is an ASSERTION, not a retry-until-green: a device that is absent, mis-wired or on the wrong
    chip-select never returns its id, so the caller still fails. What it tolerates is the one documented
    behaviour of this bus -- after a peripheral is created or retuned, a part hands back 0x00 for a
    handful of transactions before it locks on (measured 1..11, hence spibus._RESYNC_READS). The count
    is RETURNED so the test can report it: a device needing more than the driver-side resync covers is
    itself worth seeing.
    """
    for attempt in range(_SETTLE_READS):
        if (await window.read(reg, 1))[0] == expected:
            return attempt
    return -1


async def amain():
    board = config_default.default()
    spec = board['buses']['spi']['1']
    pins = board['pins']

    # get() creates the bus once and shares it -- two devices on spi:1 must not each open a peripheral
    bus = spibus.get(1, spec)
    assert spibus.get(1, spec) is bus

    """
    bind() resolves a component's own config block to the bus. POSITIVE: a block naming spi:1 finds it.
    NEGATIVE: a block naming a bus the board does not declare returns None rather than raising -- the
    driver then reports a clean setup failure instead of taking the controller down at bring-up.
    """
    assert spibus.bind(board, {'bus': 'spi', 'id': 1}) is bus
    assert spibus.bind(board, {'bus': 'spi', 'id': 7}) is None
    assert spibus.bind(board, {}) is bus  # defaults to spi:1, the only SPI bus declared

    adxl = bus.device(pins['adxl375_cs'])                    # mb_bit 6 (ADXL family default)
    lsm = bus.device(pins['lsm6dso32_cs'], mb_bit=None)      # increments via CTRL3_C.IF_INC instead

    """
    Put the LSM into the state this test is about to make assertions against.

    Two reasons, both learned the hard way when this test failed on the board at a DIFFERENT line on
    each run. First, the auto-increment being tested below IS CTRL3_C.IF_INC -- asserting that a 2-byte
    read increments while leaving IF_INC clear tests nothing and passes only by luck. Second, the part
    answers on BOTH I2C (0x6A) and SPI and does not commit to one until the interface is pinned, so a
    cold chip returns 0x00 for its first several SPI transactions; measured 6-11 of them here before it
    locked on and stayed locked. The driver writes exactly this at setup, so repeating it is idempotent
    and leaves a running driver undisturbed.
    """
    await lsm.write(_LSM_CTRL3_C, bytes([_LSM_CFG_C]))

    # single-register reads: both parts answer with their documented id
    adxl_settle = await _settled(adxl, _ADXL_DEVID, _ADXL_ID)
    assert adxl_settle >= 0, 'ADXL375 DEVID never read 0xE5'
    lsm_settle = await _settled(lsm, _LSM_WHOAMI, _LSM_ID)
    assert lsm_settle >= 0, 'LSM6DSO32 WHO_AM_I never read 0x6C'

    """
    mb_bit, functionally. A 2-byte read starts at the id register; byte 0 is the id and byte 1 is the
    NEXT register, which is reserved/zero on both parts. If auto-increment were misconfigured the
    second byte would repeat the id -- so `!=` is the assertion that catches a wrong mb_bit.
    """
    pair = await adxl.read(_ADXL_DEVID, 2)
    assert pair[0] == _ADXL_ID and pair[1] != _ADXL_ID, pair
    pair = await lsm.read(_LSM_WHOAMI, 2)
    assert pair[0] == _LSM_ID and pair[1] != _LSM_ID, pair

    # read_into() fills a caller buffer without allocating a return value (the GC-off path drivers use)
    buf = bytearray(1)
    await adxl.read_into(_ADXL_DEVID, buf)
    assert buf[0] == _ADXL_ID

    """
    diagnose() -- the wire-fault classifier a failed driver's diagnose() awaits, and the one that
    matters most on this bus: a mis-wired SPI device returns a PLAUSIBLE byte, never an exception, so
    the verdict string is the only thing separating "absent" from "crosswired" from "CS not asserting".
    POSITIVE: a healthy part with its true id -> 'ok'. NEGATIVE: the same healthy part against a WRONG
    expected id must be called out as the wrong device rather than passed.
    """
    assert 'ok' in await adxl.diagnose(_ADXL_DEVID, _ADXL_ID)
    assert 'ok' in await lsm.diagnose(_LSM_WHOAMI, _LSM_ID)
    verdict = await adxl.diagnose(_ADXL_DEVID, 0x42)
    assert 'wrong device' in verdict, verdict

    # retune() re-inits the peripheral in place (bench frequency calibration, no reboot); the shared
    # device windows keep working because they transact through the bus, not a captured peripheral
    await bus.retune(1_000_000)
    assert await _settled(adxl, _ADXL_DEVID, _ADXL_ID) >= 0, 'ADXL after retune'
    await bus.retune(spec.get('baud', 5_000_000))
    assert await _settled(lsm, _LSM_WHOAMI, _LSM_ID) >= 0, 'LSM after retune restored the configured baud'

    print('ok: spibus cached per id, bind +/-, framed read/read_into, mb_bit auto-increment both '
          'conventions, diagnose +/-, retune  (settle reads: adxl %d, lsm %d)' % (adxl_settle, lsm_settle))


asyncio.run(amain())
