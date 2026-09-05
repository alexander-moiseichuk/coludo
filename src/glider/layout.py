"""
Which board is this -- v0.1 or v1.0 -- decided by I2C scan, before any driver is set up.

The two layouts differ only in which bus each device hangs off (doc/hardware.md, "Transition"), so one
firmware can serve both if it can tell them apart. It can: four addresses swap buses between the
revisions, which is four independent votes rather than one hinge, so a single dead device cannot flip
the verdict.

                       i2c:0                              i2c:1
    v0.1   0x28 0x63 0x76 0x25 0x29             0x40
    v1.0   0x28 0x76 0x40                       0x63 0x25 0x29

`0x28` (BNO055) and `0x76` (BMP280) sit on i2c:0 in BOTH, so they say nothing about the layout -- they
are the sanity check that the scan worked at all rather than returning an empty bus.

Scanning does NOT go through i2cbus.get(): that caches a Bus per id, and the cached frequency would then
outlive detection -- a scan at 100 kHz would pin the fast bus at 100 kHz for the whole flight. Raw I2C
objects are built here, scanned, and dropped, so the drivers create the real buses afterwards at
whatever speed the chosen layout declares.
"""

import config

try:
    from machine import I2C, Pin
except ImportError:  # host (CPython): detection is board-only, detect() reports unknown
    I2C = None
    Pin = None

_SCAN_HZ: int = 100000  # every part tolerates 100 kHz; the v1.0 front bus runs here permanently

"""
The devices that MOVE, by name, with the bus each layout puts them on. Names rather than addresses,
because apply() has to rewrite the config blocks and the config is keyed by name -- and because an
address appearing twice in this table would be a silent bug where a name cannot be.
"""
_MOVED: dict = {
    # `only`: this address is evidence ONLY for the revision named, because it could be present on the
    # other bus in BOTH revisions. DEFENSIVE for the ICP-10111 rather than planned -- a second one on
    # i2c:0 was considered and retired (its ~120 ms general-call recovery must not land on the bus
    # carrying attitude). If one is ever fitted anyway, 0x63 still says v1.0 from i2c:1 and says
    # nothing from i2c:0, instead of voting both ways and cancelling itself out.
    'baro_icp10111': {'addr': 0x63, 'v0.1': 0, 'v1.0': 1, 'only': 'v1.0'},
    'airspeed_sdp810': {'addr': 0x25, 'v0.1': 0, 'v1.0': 1},
    'laser_agl': {'addr': 0x29, 'v0.1': 0, 'v1.0': 1},
    'power_ina226': {'addr': 0x40, 'v0.1': 1, 'v1.0': 0},
}

_ANCHORS: tuple = (0x28, 0x76)  # on i2c:0 in both revisions -- presence proves the scan reached devices

# The i2c:1 clock per layout. v1.0 runs the front harness slow on purpose: nothing on it exceeds 50 Hz,
# so a quarter of the clock costs no sample rate and buys edge margin on a long unterminated run.
_BUS1_HZ: dict = {'v0.1': 400000, 'v1.0': 100000}

_ABSENT: dict = {'v0.1': (), 'v1.0': ('accel_adxl375',)}  # not fitted on that revision

_LASER_PINS: tuple = ('int_pin', 'xshut_pin')  # both freed on v1.0; the driver treats them as optional

# What resolve() concluded this boot, for the health payload -- the operator should be able to see which
# revision the firmware decided it is running on WITHOUT reading the boot log, since a wrong verdict and
# a miswired board look identical from the device list.
RESOLVED: str = None


def _scan(cfg: dict, bus_id: int) -> set:
    """
    Addresses answering on one I2C bus, or an empty set when the bus cannot be opened.

    Args:
        cfg - the board config (for the bus pin spec).
        bus_id - 0 or 1.

    Returns:
        A set of 7-bit addresses; empty on any failure, which detect() treats as "no votes from here"
        rather than as evidence for either layout.
    """
    spec = config.bus(cfg, 'i2c', bus_id)
    if spec is None or I2C is None:
        return set()
    try:
        bus = I2C(bus_id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=_SCAN_HZ)
        found = set(bus.scan())
    except Exception:
        return set()  # a shorted or unpopulated bus votes for nothing
    return found


def detect(cfg: dict) -> tuple:
    """
    Decide the layout from the buses themselves.

    Each moved device votes for whichever revision puts it on the bus it actually answered on. A device
    that answers on neither expected bus, or not at all, abstains -- so an unfitted or dead part costs a
    vote instead of casting a wrong one.

    Args:
        cfg - the board config, read for bus pins only (nothing is mutated).

    Returns:
        (name, detail) where name is 'v0.1' / 'v1.0' / None. None means undecided, and the caller must
        then leave the config exactly as written -- a guess here mis-buses every sensor at once.
    """
    seen = {0: _scan(cfg, 0), 1: _scan(cfg, 1)}
    if not (seen[0] | seen[1]):
        return None, 'both buses scanned empty -- no devices answered'
    if not [addr for addr in _ANCHORS if addr in seen[0]]:
        return None, 'neither anchor (0x28/0x76) on i2c:0 -- the scan is not trustworthy'

    votes = {'v0.1': 0, 'v1.0': 0}
    for name in _MOVED:
        entry = _MOVED[name]
        for revision in votes:
            if entry.get('only', revision) != revision:
                continue  # this address is not evidence for that revision -- see `only` above
            if entry['addr'] in seen[entry[revision]]:
                votes[revision] += 1
    detail = 'i2c0=%s i2c1=%s votes %s' % (
        sorted('0x%02x' % a for a in seen[0]), sorted('0x%02x' % a for a in seen[1]), votes)
    if votes['v0.1'] == votes['v1.0']:
        return None, 'ambiguous -- ' + detail
    return ('v1.0' if votes['v1.0'] > votes['v0.1'] else 'v0.1'), detail


def apply(cfg: dict, revision: str) -> list:
    """
    Rewrite the config in place for a revision: bus membership, the i2c:1 clock, and what is not fitted.

    Only the four moving devices, one bus frequency and the not-fitted list differ between revisions --
    no pin is renumbered, so `pins` is untouched. The laser's optional control pins are dropped on v1.0
    because those GPIOs are freed there; the driver already treats both as optional.

    Args:
        cfg - the board config, MUTATED.
        revision - 'v0.1' or 'v1.0'.

    Returns:
        A list of human-readable change strings, for the boot log. Empty when the config already
        matched, which is the normal case on a board whose profile was written for it.
    """
    changes = []
    for name in _MOVED:
        want = _MOVED[name][revision]
        device = config.device(cfg, name=name)
        if device is not None and device.get('id') != want:
            changes.append('%s i2c:%s->%s' % (name, device.get('id'), want))
            device['bus'], device['id'] = 'i2c', want

    spec = config.bus(cfg, 'i2c', 1)
    if spec is not None and spec.get('freq') != _BUS1_HZ[revision]:
        changes.append('i2c:1 %d->%d Hz' % (spec.get('freq', 0), _BUS1_HZ[revision]))
        spec['freq'] = _BUS1_HZ[revision]

    for name in _ABSENT[revision]:
        device = config.device(cfg, name=name)
        if device is not None and device.get('enabled', True):
            changes.append('%s not fitted -> disabled' % name)
            device['enabled'] = False

    if revision == 'v1.0':
        laser = config.device(cfg, name='laser_agl')
        for key in _LASER_PINS:
            if laser is not None and laser.get(key) is not None:
                changes.append('laser_agl %s dropped' % key)
                laser[key] = None
    return changes


def resolve(cfg: dict, log=print) -> str:
    """
    The boot entry point: honour an explicit `board.layout`, else detect, else change nothing.

    An explicit 'v0.1' / 'v1.0' always wins over the scan, so a board can be pinned when a sensor is
    unfitted and would otherwise abstain its way to a wrong verdict. 'auto' (the default) scans.

    Args:
        cfg - the board config, mutated when a revision is applied.
        log - line logger.

    Returns:
        The revision applied, or None when nothing was changed.
    """
    global RESOLVED
    """
    A config with NO `layout` key means v0.1, not 'auto'.

    config.load() replaces rather than merges -- a valid board.config is used wholesale -- so a profile
    written before this field existed genuinely arrives without it. Every such profile belongs to a
    board built the old way, because there are no v1.0 boards yet; TMS-7C and TMS-7D are assembled and
    packed on v0.1 and will be updated over OTA later. Defaulting them to 'auto' would put a built,
    closed airframe's bus map at the mercy of a scan, and 7C in particular has `power_ina226` in its
    absent list, so it can only cast three votes -- fewer as more parts are declared absent. Absent
    key -> the revision those boards actually are.
    """
    declared = cfg.get('board', {}).get('layout', 'v0.1')
    if declared in ('v0.1', 'v1.0'):
        changes = apply(cfg, declared)
        log('layout :: %s (declared) %s' % (declared, ', '.join(changes) if changes else 'already matched'))
        RESOLVED = declared + ' (declared)'
        return declared

    revision, detail = detect(cfg)
    if revision is None:
        # Deliberately no fallback guess: applying the wrong revision re-buses every moving sensor at
        # once, which is a worse failure than running the config as written and saying so.
        log('layout :: UNDECIDED, config left as written -- %s' % detail)
        RESOLVED = 'undecided'
        return None
    changes = apply(cfg, revision)
    RESOLVED = revision
    log('layout :: %s detected -- %s' % (revision, detail))
    if changes:
        log('layout :: applied %s' % ', '.join(changes))
    return revision
