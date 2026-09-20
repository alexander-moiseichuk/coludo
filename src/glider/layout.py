"""
Which board is this -- v0.1 or v1.0 -- decided by I2C scan, before any driver is set up.

The two layouts differ only in which bus each device hangs off (doc/hardware.md, "Transition"), so one
firmware can serve both if it can tell them apart. It can: four addresses swap buses between the
revisions, which is four independent votes rather than one hinge, so a single dead device cannot flip
the verdict.

                       i2c:0                                   i2c:1
    v0.1   0x18 0x28 0x76 0x63 0x25 0x29                 0x40
    v1.0   0x18 0x28 0x76 0x40                           0x63 0x25 0x29
    v1.1   0x18 0x69 0x47 0x15 0x40                      0x63 0x25 0x29

Three revisions, and they separate on two independent axes. The four devices that MOVE BUS separate
v0.1 from the v1.x pair; the ATTITUDE PARTS separate v1.0 from v1.1, because the SEN0253 (BNO055 0x28 +
BMP280 0x76, one module) gives way to the SEN0697 (BMI323 0x69 + BMP581 0x47 + BMM350 0x15). The
ADXL375 is on SPI and invisible here, so "no ADXL375" is carried by the revision, never scanned for.

`0x18` is the ANCHOR: the ES8311 audio codec soldered to the WaveShare board itself. It says nothing
about the revision -- it is present on all of them -- which is exactly what an anchor is for: proving the
scan reached a live bus rather than returning an empty set. It replaces the old pair of anchors (0x28 +
0x76), and it is a better one precisely because it cannot be unplugged: those two were the SEN0253, so
fitting a SEN0697 removed both and detection would have refused on a perfectly good board. The other
known-present addresses stay in the list as fallbacks in case a board ever ships without the codec.

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
    # `only`: this address is evidence ONLY for the revisions named, because it could be present on the
    # other bus in more than one. DEFENSIVE for the ICP-10111 rather than planned -- a second one on
    # i2c:0 was considered and retired (its ~120 ms general-call recovery must not land on the bus
    # carrying attitude). If one is ever fitted anyway, 0x63 still says v1.x from i2c:1 and says nothing
    # from i2c:0, instead of voting both ways and cancelling itself out.
    'baro_icp10111': {'addr': 0x63, 'v0.1': 0, 'v1.0': 1, 'v1.1': 1, 'only': ('v1.0', 'v1.1')},
    'airspeed_sdp810': {'addr': 0x25, 'v0.1': 0, 'v1.0': 1, 'v1.1': 1},
    'laser_agl': {'addr': 0x29, 'v0.1': 0, 'v1.0': 1, 'v1.1': 1},
    'power_ina226': {'addr': 0x40, 'v0.1': 1, 'v1.0': 0, 'v1.1': 0},
}

"""
The attitude parts, which is what separates v1.0 from v1.1.

Presence votes; absence does not vote against. A part that is dead or unfitted therefore costs its
revision one vote rather than casting one for the other -- the same tolerance the moving devices have,
and the reason a single failure cannot flip the verdict.
"""
_FITTED: dict = {
    'imu_bno055': {'addr': 0x28, 'in': ('v0.1', 'v1.0')},    # SEN0253, one module with the BMP280
    'baro_bmp280': {'addr': 0x76, 'in': ('v0.1', 'v1.0')},
    'imu_bmi323': {'addr': 0x69, 'in': ('v1.1',)},           # SEN0697, one module with the BMP581+BMM350
    'baro_bmp581': {'addr': 0x47, 'in': ('v1.1',)},
    'mag_bmm350': {'addr': 0x15, 'in': ('v1.1',)},           # driven on v1.1, and evidence on any board
}

_REVISIONS: tuple = ('v0.1', 'v1.0', 'v1.1')

_ANCHORS: tuple = (0x18, 0x28, 0x76, 0x69, 0x47)  # ANY one proves the scan reached a live i2c:0.
                                                 # 0x18 is the on-board ES8311 codec: unpluggable,
                                                 # revision-independent, and the only one of these
                                                 # a sensor swap cannot take away.

# The i2c:1 clock per layout. v1.0 runs the front harness slow on purpose: nothing on it exceeds 50 Hz,
# so a quarter of the clock costs no sample rate and buys edge margin on a long unterminated run.
_BUS1_HZ: dict = {'v0.1': 400000, 'v1.0': 100000, 'v1.1': 100000}

"""
What is NOT fitted on each revision. apply() also ENABLES everything outside this list that appears in
another revision's -- a config that arrives with the SEN0697 disabled must have it switched ON when a
v1.1 board is detected, not merely left alone.
"""
_ABSENT: dict = {
    'v0.1': ('imu_bmi323', 'mag_bmm350', 'baro_bmp581'),
    'v1.0': ('accel_adxl375', 'imu_bmi323', 'mag_bmm350', 'baro_bmp581'),
    'v1.1': ('accel_adxl375', 'imu_bno055', 'baro_bmp280'),
}

_LASER_PINS: tuple = ('int_pin', 'xshut_pin')  # both freed on v1.0; the driver treats them as optional

"""
Devices that occupy the SAME SOCKET as another and must follow it, without voting for themselves.

`laser_agl_l1x` is a VL53L1X in the same footprint and at the same 0x29 as the VL53L4CX `laser_agl`;
only one is ever soldered, and each driver's model-id check decides which comes up. They must share
the bus and the pin treatment -- but the address must be counted ONCE, or a socket with two candidate
drivers would cast two votes for the same physical evidence and outweigh the parts that are really
there. So followers are moved by apply() and ignored by detect().
"""
_FOLLOWS: dict = {'laser_agl': ('laser_agl_l1x',)}

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

    Two independent kinds of evidence, counted into one tally per revision:

      * a MOVED device votes for whichever revisions put it on the bus it actually answered on. This
        separates v0.1 from the v1.x pair and says nothing within it.
      * a FITTED part votes for the revisions that carry it. This is what separates v1.0 from v1.1,
        since the SEN0253 and the SEN0697 occupy different addresses on the same bus.

    Absence never votes AGAINST. A dead or unfitted part costs its revision one vote rather than casting
    one for another, so no single failure can flip a verdict -- the whole reason the tally is spread over
    several addresses instead of hinging on one.

    Args:
        cfg - the board config, read for bus pins only (nothing is mutated).

    Returns:
        (name, detail) where name is one of _REVISIONS, or None. None means undecided, and the caller
        must then leave the config exactly as written -- a guess mis-buses every sensor at once.
    """
    seen = {0: _scan(cfg, 0), 1: _scan(cfg, 1)}
    if not (seen[0] | seen[1]):
        return None, 'both buses scanned empty -- no devices answered'
    if not [addr for addr in _ANCHORS if addr in seen[0]]:
        return None, 'no anchor (%s) on i2c:0 -- the scan is not trustworthy' % (
            ' '.join('0x%02x' % addr for addr in _ANCHORS))

    votes = {}
    for revision in _REVISIONS:
        votes[revision] = 0
    for name in _MOVED:
        entry = _MOVED[name]
        for revision in _REVISIONS:
            only = entry.get('only')
            if only is not None and revision not in only:
                continue  # this address is not evidence for that revision -- see `only` above
            if entry['addr'] in seen[entry[revision]]:
                votes[revision] += 1
    for name in _FITTED:
        entry = _FITTED[name]
        if entry['addr'] in seen[0]:
            for revision in entry['in']:
                votes[revision] += 1

    detail = 'i2c0=%s i2c1=%s votes %s' % (
        sorted('0x%02x' % a for a in seen[0]), sorted('0x%02x' % a for a in seen[1]), votes)
    ranked = sorted(_REVISIONS, key=lambda revision: votes[revision], reverse=True)
    if votes[ranked[0]] == votes[ranked[1]]:
        return None, 'ambiguous -- ' + detail
    return ranked[0], detail


def fitted(device: str, revision: str) -> bool:
    """
    Whether `device` is physically present on `revision` -- the ONE place that question is answered.

    Every caller that instead wrote its own revision literal has eventually been wrong: resolve() gated
    on ('v0.1', 'v1.0') and so ignored v1.1 entirely, and test_spibus asked `revision != 'v1.0'` and so
    demanded an ADXL375 from a v1.1 board that has none. Both read fine until a third revision existed.
    Ask here instead, and adding a revision updates every caller at once.

    An UNDECIDED detection (revision not in _REVISIONS) answers False for the parts a revision removes:
    unknown is not evidence of presence, and a test that demands a part on an unidentifiable board
    reports a hardware fault when what it found was an inconclusive scan.

    Args:
        device - the config device name, e.g. 'accel_adxl375'.
        revision - the board revision, as detect() / RESOLVED gives it.

    Returns:
        True when the part is fitted on that revision.
    """
    if device in _ABSENT.get(revision, ()):
        return False
    if revision not in _REVISIONS:
        return device not in _MOVED and not any(device in absent for absent in _ABSENT.values())
    requirement = _FITTED.get(device)
    return requirement is None or revision in requirement['in']


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
        for device_name in (name,) + _FOLLOWS.get(name, ()):  # the socket's other candidate moves too
            device = config.device(cfg, name=device_name)
            if device is not None and device.get('id') != want:
                changes.append('%s i2c:%s->%s' % (device_name, device.get('id'), want))
                device['bus'], device['id'] = 'i2c', want

    spec = config.bus(cfg, 'i2c', 1)
    if spec is not None and spec.get('freq') != _BUS1_HZ[revision]:
        changes.append('i2c:1 %d->%d Hz' % (spec.get('freq', 0), _BUS1_HZ[revision]))
        spec['freq'] = _BUS1_HZ[revision]

    """
    Fit or unfit every revision-dependent part, in BOTH directions.

    This used only to disable. That was enough while the revisions differed by a part going away, but
    v1.1 swaps one attitude module for another: a config written for v1.0 arrives with the SEN0697
    disabled, and leaving it alone would bring up a board whose IMU is present, wired and ignored. The
    set is the union of every revision's absent list, so nothing outside it is ever touched.
    """
    revision_dependent = set()
    for names in _ABSENT.values():
        revision_dependent.update(names)
    for name in sorted(revision_dependent):
        device = config.device(cfg, name=name)
        if device is None:
            continue
        fitted = name not in _ABSENT[revision]
        if device.get('enabled', True) != fitted:
            changes.append('%s %s' % (name, 'fitted -> enabled' if fitted else 'not fitted -> disabled'))
            device['enabled'] = fitted

    if revision == 'v1.0':
        for laser_name in ('laser_agl',) + _FOLLOWS.get('laser_agl', ()):
            laser = config.device(cfg, name=laser_name)
            for key in _LASER_PINS:
                if laser is not None and laser.get(key) is not None:
                    changes.append('%s %s dropped' % (laser_name, key))
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
    if declared in _REVISIONS:  # not a literal pair: a revision added below must not silently
                               # fall through to the scan, which is what made a declared v1.1 ignored
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
