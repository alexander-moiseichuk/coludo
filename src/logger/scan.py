"""
First thing to run after soldering: does every chip on the SEN0697 answer, and is it the chip we think?

A bare I2C scan proves the bus is wired; it does not prove WHICH parts are on it, and on this board the
three addresses come in strap-selectable pairs where the default is the upper one. So this scans, then
reads each chip's identity register and names what it found -- a board strapped to the lower addresses
is a perfectly good board, and should not read as a fault.

Nothing here writes to a sensor. It is safe to run repeatedly on a half-finished rig.

    mpremote connect /dev/ttyACM0 run scan.py
"""

from machine import I2C, Pin

_SDA: int = 19          # see README -- the five-wire harness on the left header
_SCL: int = 20
_FREQ: int = 100000     # slow on purpose: a marginal solder joint fails at 400 kHz and passes here,
                        # and a part that answers only at 100 kHz is something you want to know NOW

"""
(name, default address, alternate address, identity register, expected id, dummy bytes).

`dummy` is the Bosch BMI323 and BMM350 quirk and the reason a naive scan script reports garbage for it: over I2C the
part prepends two dummy bytes to every register read, so the identity lands at offset 2, not 0. Reading
it like the other two returns whatever happened to be in the pipeline and looks like a dead chip.
"""
_CHIPS: tuple = (
    ('BMI323 accel+gyro', 0x69, 0x68, 0x00, 0x43, 2),
    ('BMM350 mag       ', 0x15, 0x14, 0x00, 0x33, 2),
    ('BMP581 baro      ', 0x47, 0x46, 0x01, 0x50, 0),
)


def _identity(bus, addr: int, reg: int, dummy: int) -> int:
    """The identity byte, or -1 when the read itself failed."""
    try:
        raw = bus.readfrom_mem(addr, reg, dummy + 1)
    except Exception:
        return -1
    return raw[dummy]


def scan() -> bool:
    """
    Scan the bus and identify each expected chip.

    Returns:
        True when all three answered and identified; False otherwise (the detail is printed).
    """
    bus = I2C(0, scl=Pin(_SCL), sda=Pin(_SDA), freq=_FREQ)
    found = set(bus.scan())
    print('i2c sda=GPIO%d scl=GPIO%d @ %d Hz' % (_SDA, _SCL, _FREQ))
    print('addresses answering: %s' % (sorted('0x%02x' % a for a in found) or 'NONE'))
    if not found:
        print('')
        print('NOTHING ANSWERED. In order of likelihood: SDA/SCL swapped, no 3V3 at the sensor,')
        print('a missing ground between the boards, or absent pull-ups (measure SDA->3V3).')
        return False

    print('')
    ok = True
    for name, default, alternate, reg, expect, dummy in _CHIPS:
        addr = default if default in found else (alternate if alternate in found else None)
        if addr is None:
            print('%s  MISSING -- neither 0x%02x nor 0x%02x answered' % (name, default, alternate))
            ok = False
            continue
        got = _identity(bus, addr, reg, dummy)
        where = 'default' if addr == default else 'ALTERNATE strap'
        if got == expect:
            print('%s  0x%02x (%s)  id 0x%02x  OK' % (name, addr, where, got))
        elif got < 0:
            print('%s  0x%02x (%s)  answered the scan but the id read FAILED' % (name, addr, where))
            ok = False
        else:
            # not automatically a fault: report it rather than assert, since a wrong id here is more
            # often a datasheet revision than a dead part
            print('%s  0x%02x (%s)  id 0x%02x, expected 0x%02x -- CHECK' % (name, addr, where, got, expect))
            ok = False

    extra = found - {c[1] for c in _CHIPS} - {c[2] for c in _CHIPS}
    if extra:
        print('')
        print('unexpected responders: %s' % sorted('0x%02x' % a for a in extra))
    print('')
    print('ALL THREE PRESENT' if ok else 'INCOMPLETE -- see above')
    return ok


scan()
