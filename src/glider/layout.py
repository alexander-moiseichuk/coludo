"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Which board is this, decided by I2C scan before any driver is set up: the BUS MAP from where the moving
devices answer; the attitude parts then follow from the revision and from what the scan found.

There are two bus maps, and they differ only in which bus four devices hang off (doc/hardware.md,
"Transition"), so one firmware can serve both if it can tell them apart. It can: four addresses swap
buses between the maps, which is four independent votes rather than one hinge, so a single dead device
cannot flip the verdict.

                   i2c:0                          i2c:1
    v0.1 map   0x18 0x63 0x25 0x29  + module      0x40
    v1.x map   0x18 0x40            + module      0x63 0x25 0x29

    SEN0697  BMI323 0x69  BMP581 0x47  BMM350 0x15   i2c:0   PRIMARY: expected on every scanned board
    SEN0253  BNO055 0x28  BMP280 0x76                i2c:0   BACKUP: enabled only when the scan finds it

THE MAP IS DECIDED BY PLACEMENT ALONE. The attitude module sits on i2c:0 on every board, so where it
answers says nothing about where the front devices are. It used to vote anyway, counted into one tally
with the placement votes, and that put a v0.1 board on the wrong map: TMS-7C, the one v0.1 board left
and now carrying a SEN0697, read v1.1 on three module votes against two placement votes, and its laser,
pitot and ICP-10111 were moved to an i2c:1 with nothing on it (scan 2026-10-05: i2c:0 0x15 0x18 0x25
0x29 0x47 0x63 0x69, i2c:1 empty).

The three revision names stay, because configs and captures carry them:

    v0.1   the v0.1 map + the SEN0697 + the ADXL375 on SPI    TMS-7C
    v1.1   the v1.x map + the SEN0697                         the taster, TMS-7F
    v1.0   the v1.x map + the SEN0253 alone                   legacy: TMS-7E as built -- DECLARED only

A SCAN NEVER ANSWERS v1.0. The v1.x map always scans as v1.1, because the SEN0697 is the module every
board is expected to carry: when it is missing, its setup fails and `arm` refuses. A scanned v1.0 used to
switch it off instead -- a two-module board whose SEN0697 connector had worked loose read v1.0 and would
have flown on the backup with nothing refusing. v1.0 survives as a DECLARED revision for the legacy build
and the configs saved for it, and there the SEN0253 IS the module: apply() enables it.

Everywhere else THE SEN0253 IS A BACKUP: a taster or an experimental board may carry both modules
(different addresses on the same bus), while a flight board normally carries the SEN0697 alone. After a
scan, apply() enables the SEN0253 parts that answered and disables the rest; a declared v0.1 / v1.1 runs
no scan, so the config's own `enabled` stands. Wherever it ends up enabled, apply() RANKS it below the
SEN0697 and the attitude filter on every quantity they share -- in code, whatever the config says,
because every config saved before 2026-10-05 carries the BNO055 at attitude priority 0 (see _BELOW).

The ADXL375 is on SPI and invisible here, so "no ADXL375" is carried by the map, never scanned for.

`0x18` is the ANCHOR: the ES8311 audio codec soldered to the WaveShare board itself. It says nothing
about the revision (every board has it), and that is exactly what an anchor is for: proving the scan
reached a live bus rather than returning an empty set. Because it is soldered down, no sensor swap can
remove it. The other known-present addresses stay in the list as fallbacks, in case a board ever ships
without the codec.

Scanning does NOT go through i2cbus.get(): that caches a Bus per id, and the cached frequency would then
outlive detection -- a scan at 100 kHz would pin the fast bus at 100 kHz for the whole flight. Raw I2C
objects are built here, scanned, and dropped, so the drivers create the real buses afterwards at
whatever speed the chosen layout declares.
"""

import config
import i2cbus

try:
    from machine import I2C, Pin
except ImportError:  # host (CPython): detection is board-only, detect() reports unknown
    I2C = None
    Pin = None

_SCAN_HZ: int = 100000  # every part tolerates 100 kHz; the v1.x front bus runs here permanently

_REVISIONS: tuple = ('v0.1', 'v1.0', 'v1.1')

_MAPS: tuple = ('v0.1', 'v1.x')  # the two bus maps; placement votes between these and nothing else

_MAP: dict = {'v0.1': 'v0.1', 'v1.0': 'v1.x', 'v1.1': 'v1.x'}  # the bus map each revision lays out

_SCANNED: dict = {'v0.1': 'v0.1', 'v1.x': 'v1.1'}  # the revision a SCAN names for each map: never v1.0

"""
The devices that MOVE, by name, with the bus each MAP puts them on. These are the only votes for a map.
Names rather than addresses, because apply() has to rewrite the config blocks and the config is keyed
by name -- and because an address appearing twice in this table would be a silent bug where a name
cannot be.

Every one votes from BOTH buses. The ICP-10111 used to count from i2c:1 only, as a guard against a
second ICP on i2c:0 -- considered and retired, because its ~120 ms general-call recovery must not land
on the bus carrying attitude. That guard cost TMS-7C, whose one ICP answers on i2c:0, its third vote. A
stray second ICP would now add one v0.1 vote beside the v1.x map's four, which cannot flip it.
"""
_MOVED: dict = {
    'baro_icp10111': {'addr': 0x63, 'v0.1': 0, 'v1.x': 1},
    'airspeed_sdp810': {'addr': 0x25, 'v0.1': 0, 'v1.x': 1},
    'laser_agl': {'addr': 0x29, 'v0.1': 0, 'v1.x': 1},
    'power_ina226': {'addr': 0x40, 'v0.1': 1, 'v1.x': 0},
}

"""
The two attitude modules, by config name -> address on i2c:0. Neither ever votes for a map.

PRIMARY (SEN0697) parts are expected on every revision except the declared legacy v1.0. A scan counts
them for the boot log only: their absence changes no verdict, it fails their setup.
BACKUP (SEN0253) parts are enabled only when a scan finds them -- see apply()'s `found` -- except on
_BACKUP_REVISIONS, where the SEN0253 is the module itself.
"""
_PRIMARY: dict = {'imu_bmi323': 0x69, 'baro_bmp581': 0x47, 'mag_bmm350': 0x15}
_BACKUP: dict = {'imu_bno055': 0x28, 'baro_bmp280': 0x76}
_BACKUP_REVISIONS: tuple = ('v1.0',)  # the revisions whose attitude module IS the SEN0253 (declared only)

_ANCHORS: tuple = (0x18, 0x28, 0x76, 0x69, 0x47)  # ANY one proves the scan reached a live i2c:0.
                                                 # 0x18 is the on-board ES8311 codec: unpluggable,
                                                 # revision-independent, and the only one of these
                                                 # a sensor swap cannot take away.

# The i2c:1 clock per MAP. The v1.x front harness runs slow on purpose: nothing on it exceeds 50 Hz, so
# a quarter of the clock costs no sample rate and buys edge margin on a long unterminated run.
_BUS1_HZ: dict = {'v0.1': 400000, 'v1.x': 100000}

"""
What is NOT fitted on each revision. These parts are the REVISION-DEPENDENT set: apply() disables
the ones listed for the revision and ENABLES the rest of the set. A config that arrives with the
SEN0697 disabled (anything saved before 2026-10-05) must have it switched ON, not merely left alone.

v0.1 lists nothing: since 2026-10-05 it carries the SEN0697 AND the ADXL375. The SEN0253 is in no list:
_BACKUP_REVISIONS fits it, and on any other revision only a scan does.
"""
_ABSENT: dict = {
    'v0.1': (),
    'v1.0': ('accel_adxl375', 'imu_bmi323', 'mag_bmm350', 'baro_bmp581'),
    'v1.1': ('accel_adxl375',),
}

"""
RANKING apply() enforces, whatever priorities the config carries: (device, the devices it must rank
strictly BELOW). Wherever the device is enabled, each quantity it provides that an ENABLED superior also
provides is demoted to one past the worst-ranked such superior. It only ever demotes, and a quantity no
enabled superior provides is left alone.

In code rather than in config_default alone, because a config is used wholesale: every config saved
before 2026-10-05 -- tms7f.config, tms7e.config, a taster's board.config, anything launch_config.py
--base derives from them -- carries the BNO055 at attitude priority 0 and the BMP280 at priority 1, tied
with the BMP581. A SEN0253 found beside the SEN0697 on such a config would fly as the PRIMARY, with the
filter mirroring it. CONFIG_VERSION reports the mismatch but changes nothing that runs.

  * the ADXL375 below the BMI323 on `accel`: it is the +/-200 g backstop, 49 mg a count, and both are
    fitted on v0.1 only. The LSM6DSO32 stays the primary over both.
  * the SEN0253 below the SEN0697 and the attitude filter on everything they share -- and the BNO055 below
    the ADXL375 on `accel` as well, which config_default has always ranked it behind; without that the
    ADXL375's demotion above would tie the two.

ORDER MATTERS: the ADXL375 is settled before the BNO055 is ranked against it.
"""
_BELOW: tuple = (
    ('accel_adxl375', ('imu_bmi323',)),
    ('imu_bno055', ('attitude', 'imu_bmi323', 'mag_bmm350', 'baro_bmp581', 'accel_adxl375')),
    ('baro_bmp280', ('attitude', 'imu_bmi323', 'mag_bmm350', 'baro_bmp581')),
)

"""
Control pins per MAP, by device: apply() writes each value (None = not routed; every driver below
treats its pin as optional).

v1.x: the front harness carries no laser INT/XSHUT, for either laser entry. The BMI323's data-ready is
accel_int1 (GPIO4), which the v1.x map frees because it has no ADXL375.
v0.1: GPIO4 IS the ADXL375's INT1, routed on the PCB, and the attitude socket carries four wires only
(SDA, SCL, 3V3, GND), so the BMI323 has no data-ready line there. If both parts kept accel_int1, two IRQ
handlers would sit on one GPIO and the later setup would silently take over the ADXL375's interrupt.
"""
_CONTROL_PINS: dict = {
    'v0.1': {'imu_bmi323': {'int_pin': None}},
    'v1.x': {'imu_bmi323': {'int_pin': 'accel_int1'},
             'laser_agl': {'int_pin': None, 'xshut_pin': None},
             'laser_agl_l1x': {'int_pin': None, 'xshut_pin': None}},
}

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
        found = set()  # a shorted or unpopulated bus votes for nothing
    """
    A RUNTIME scan (the `detect` command) must hand the bus back at its own clock. machine.I2C(id) is ONE
    peripheral per id, so the scan above re-clocked the bus every driver on it is using -- to 100 kHz,
    until reboot, silently: a pre-flight `detect` flew i2c:0 at a quarter speed. At boot no driver holds
    a bus yet, so there is nothing to restore (the real Bus is built afterwards at its own speed).
    The scan and this restore run with no await between them, so no driver transfer can land in the
    100 kHz window.
    """
    live = i2cbus.live(bus_id)
    if live is not None:
        try:
            live.reclock()
        except Exception:
            pass  # the bus-clear path re-inits on the next wedge; a scan must never raise into CC
    return found


def _verdict(seen: dict) -> tuple:
    """
    The decision on one scan: the map by placement, the revision the scan names for it, the backup found.

    Placement is the only evidence for a map. A MOVED device votes for whichever map puts it on the
    bus it actually answered on; absence never votes AGAINST, so a dead or unfitted device costs its
    map one vote rather than casting one for the other, and no single failure flips a verdict. A TIE
    (0-0 included: a scan that saw only the anchor and the attitude module) is undecided, because the
    module cannot break it -- it is on i2c:0 whichever map this is.

    The modules are counted for the detail line and nothing else. The v1.x map is v1.1 whichever
    module answered, so a missing SEN0697 fails its setup rather than reading as legacy v1.0.

    Args:
        seen - {bus_id: set of addresses} for i2c:0 and i2c:1.

    Returns:
        (revision, detail, found): revision is 'v0.1', 'v1.1' or None (undecided: the caller must leave
        the bus map exactly as written -- a guess mis-buses every sensor at once); detail is the
        human-readable evidence; found is the tuple of _BACKUP names that answered on i2c:0, empty when
        undecided.
    """
    if not (seen[0] | seen[1]):
        return None, 'both buses scanned empty -- no devices answered', ()
    if not [addr for addr in _ANCHORS if addr in seen[0]]:
        return None, 'no anchor (%s) on i2c:0 -- the scan is not trustworthy' % (
            ' '.join('0x%02x' % addr for addr in _ANCHORS)), ()

    placement = {}
    for bus_map in _MAPS:
        placement[bus_map] = 0
    for name in _MOVED:
        entry = _MOVED[name]
        for bus_map in _MAPS:
            if entry['addr'] in seen[entry[bus_map]]:
                placement[bus_map] += 1
    primary = [name for name in _PRIMARY if _PRIMARY[name] in seen[0]]
    found = tuple(name for name in sorted(_BACKUP) if _BACKUP[name] in seen[0])

    detail = 'i2c0=%s i2c1=%s placement v0.1:%d v1.x:%d SEN0697 %d/%d SEN0253 %d/%d' % (
        sorted('0x%02x' % a for a in seen[0]), sorted('0x%02x' % a for a in seen[1]),
        placement['v0.1'], placement['v1.x'], len(primary), len(_PRIMARY), len(found), len(_BACKUP))
    if placement['v0.1'] == placement['v1.x']:
        return None, 'ambiguous placement -- ' + detail, ()
    return _SCANNED['v0.1' if placement['v0.1'] > placement['v1.x'] else 'v1.x'], detail, found


def detect(cfg: dict) -> tuple:
    """
    Decide the layout from the buses themselves (see _verdict for the rules).

    Args:
        cfg - the board config, read for bus pins only (nothing is mutated).

    Returns:
        (name, detail) where name is 'v0.1' or 'v1.1' (a scan never names v1.0), or None. None means
        undecided, and the caller must then leave the bus map as written -- a guess mis-buses every
        sensor at once.
    """
    revision, detail, _found = _verdict({0: _scan(cfg, 0), 1: _scan(cfg, 1)})
    return revision, detail


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

    The SEN0253 (BNO055 + BMP280) answers True on _BACKUP_REVISIONS only, where it is the module. No
    other revision guarantees it: only a scan finds it there, and the resolved config's `enabled` is
    what records the result.

    Args:
        device - the config device name, e.g. 'accel_adxl375'.
        revision - the board revision, as detect() / RESOLVED gives it.

    Returns:
        True when the part is fitted on that revision.
    """
    if device in _BACKUP:
        return revision in _BACKUP_REVISIONS
    if device in _ABSENT.get(revision, ()):
        return False
    if revision not in _REVISIONS:
        return device not in _MOVED and not any(device in absent for absent in _ABSENT.values())
    return True


def apply(cfg: dict, revision: str, found: tuple = None) -> list:
    """
    Rewrite the config in place for a revision: bus membership, the i2c:1 clock, what is fitted, ranks.

    Only the four moving devices, one bus frequency, the revision-dependent parts, the backup module,
    a few optional control pins and the backup ranking (_BELOW) differ -- no pin is renumbered, so
    `pins` is untouched.

    Args:
        cfg - the board config, MUTATED.
        revision - one of _REVISIONS.
        found - the _BACKUP (SEN0253) parts a scan found: those are enabled and the rest disabled. None
            means there was no scan (a declared revision), and the config's own `enabled` stands --
            except on _BACKUP_REVISIONS, which fit the whole SEN0253 either way.

    Returns:
        A list of human-readable change strings, for the boot log. Empty when the config already
        matched, which is the normal case on a board whose profile was written for it.
    """
    bus_map = _MAP[revision]
    changes = []
    for name in _MOVED:
        want = _MOVED[name][bus_map]
        for device_name in (name,) + _FOLLOWS.get(name, ()):  # the socket's other candidate moves too
            device = config.device(cfg, name=device_name)
            if device is not None and device.get('id') != want:
                changes.append('%s i2c:%s->%s' % (device_name, device.get('id'), want))
                device['bus'], device['id'] = 'i2c', want

    spec = config.bus(cfg, 'i2c', 1)
    if spec is not None and spec.get('freq') != _BUS1_HZ[bus_map]:
        changes.append('i2c:1 %d->%d Hz' % (spec.get('freq', 0), _BUS1_HZ[bus_map]))
        spec['freq'] = _BUS1_HZ[bus_map]

    """
    Fit or unfit every revision-dependent part, in BOTH directions.

    The set is the union of every revision's absent list, so nothing outside it is ever touched -- the
    SEN0253 included, which has its own rule below.
    """
    revision_dependent = set()
    for names in _ABSENT.values():
        revision_dependent.update(names)
    for name in sorted(revision_dependent):
        _fit(cfg, name, name not in _ABSENT[revision], changes)

    if revision in _BACKUP_REVISIONS:
        found = tuple(_BACKUP)  # the SEN0253 IS this revision's module: fitted, scan or no scan
    if found is not None:  # a scan ran (or the revision says): the backup module is exactly what answered
        for name in sorted(_BACKUP):
            _fit(cfg, name, name in found, changes)

    pins = _CONTROL_PINS[bus_map]
    for name in sorted(pins):
        device = config.device(cfg, name=name)
        for key in sorted(pins[name]):
            if device is not None and device.get(key) != pins[name][key]:
                changes.append('%s %s %s->%s' % (name, key, device.get(key), pins[name][key]))
                device[key] = pins[name][key]
    _rank(cfg, changes, revision)
    return changes


def _rank(cfg: dict, changes: list, revision: str = None) -> None:
    """
    Demote each ENABLED device in _BELOW to strictly below its enabled superiors, per shared quantity.

    Bus-independent, which is why resolve() runs it on an undecided scan too: the ranking says which
    source the databoard prefers, not where anything is wired.

    NOT on _BACKUP_REVISIONS: there the SEN0253 IS the module, and the board carries no BMM350, so the
    filter has no heading reference of its own -- demoted below it, the BNO055 would never seed it and
    the heading would start at 0 and stay gyro-integrated. The SEN0253 keeps the rank its config gives
    it, as it always flew there.

    Args:
        cfg - the board config, MUTATED.
        changes - the boot-log list, appended to for every priority moved.
        revision - the revision being applied; None on an undecided scan.

    Returns:
        None.
    """
    for name, superiors in _BELOW:
        if revision in _BACKUP_REVISIONS and name in _BACKUP:
            continue  # the SEN0253 is this revision's own module, not a backup -- see above
        device = config.device(cfg, name=name)
        if device is None or not device.get('enabled', True):
            continue
        provides = device.get('provides') or {}
        for quantity in sorted(provides):
            floor = None  # the worst (highest-numbered) priority among the enabled superiors
            for superior_name in superiors:
                superior = config.device(cfg, name=superior_name)
                if superior is None or not superior.get('enabled', True):
                    continue
                entry = (superior.get('provides') or {}).get(quantity)
                if entry is not None:
                    priority = entry.get('priority', 0)
                    floor = priority if floor is None else max(floor, priority)
            entry = provides[quantity]
            if floor is not None and entry.get('priority', 0) <= floor:
                changes.append('%s %s p%d->p%d' % (name, quantity, entry.get('priority', 0), floor + 1))
                entry['priority'] = floor + 1


def _fit(cfg: dict, name: str, present: bool, changes: list) -> None:
    """
    Enable a device the board carries, disable one it does not -- unless the operator unfitted it.

    `fitted: false` is the operator saying THIS part is dead or removed. Without it, a part the board is
    supposed to carry would be re-enabled here: an `enabled: false` was undone, and verify/arm refused
    on a part nobody could take out of the config.

    Args:
        cfg - the board config, MUTATED.
        name - the device's config name; a device the config does not declare is skipped.
        present - whether the revision (or the scan) says the part is there.
        changes - the boot-log list, appended to when `enabled` changes.

    Returns:
        None.
    """
    device = config.device(cfg, name=name)
    if device is None:
        return
    enabled = present and device.get('fitted', True) is not False
    if device.get('enabled', True) != enabled:
        changes.append('%s %s' % (name, 'fitted -> enabled' if enabled else 'not fitted -> disabled'))
        device['enabled'] = enabled


def resolve(cfg: dict, log=print) -> str:
    """
    The boot entry point: honour an explicit `board.layout`, else detect, else change the bus map not.

    An explicit 'v0.1' / 'v1.0' / 'v1.1' always wins over the scan, so a board can be pinned when a
    sensor is unfitted and would otherwise abstain its way to a wrong verdict. 'auto' (the default)
    scans. A declared v0.1 / v1.1 never touches the SEN0253's `enabled`: with no scan there is nothing to
    say whether it is fitted, so the config's own value stands (a declared v1.0 fits it -- it IS v1.0).
    Whatever the path, an enabled SEN0253 is ranked below the SEN0697 (_BELOW).

    Args:
        cfg - the board config, mutated when a revision is applied (and by the ranking on any path).
        log - line logger.

    Returns:
        The revision applied, or None when the scan was undecided.
    """
    global RESOLVED
    """
    A config with NO `layout` key means v0.1, not 'auto'.

    config.load() replaces rather than merges -- a valid board.config is used wholesale -- so a profile
    written before this field existed genuinely arrives without it, and every such profile belongs to a
    board built the old way. TMS-7C, the one v0.1 board left, flies keyless profiles from
    make_telemetry_config.py. Defaulting it to 'auto' would put a built, closed airframe's bus map at the
    mercy of a scan: 7C casts three placement votes (ICP, pitot, laser on i2c:0; its INA226 is absent),
    so a front harness fault is all it takes to make it undecided. Absent key -> the revision it is.
    """
    declared = cfg.get('board', {}).get('layout', 'v0.1')
    if declared in _REVISIONS:  # not a literal pair: a revision added below must not silently
                               # fall through to the scan, which is what made a declared v1.1 ignored
        changes = apply(cfg, declared)
        log('layout :: %s (declared) %s' % (declared, ', '.join(changes) if changes else 'already matched'))
        RESOLVED = declared + ' (declared)'
        return declared

    revision, detail, found = _verdict({0: _scan(cfg, 0), 1: _scan(cfg, 1)})
    if revision is None:
        """
        Deliberately no fallback guess: applying the wrong revision re-buses every moving sensor at once,
        which is a worse failure than running the config as written and saying so. The ranking still
        applies -- it depends on no bus, and an old profile's p0 BNO055 is as wrong here as anywhere.
        """
        changes = []
        _rank(cfg, changes)
        log('layout :: UNDECIDED, bus map left as written -- %s' % detail)
        if changes:
            log('layout :: ranked %s' % ', '.join(changes))
        RESOLVED = 'undecided'
        return None
    changes = apply(cfg, revision, found)
    RESOLVED = revision
    log('layout :: %s detected -- %s' % (revision, detail))
    if changes:
        log('layout :: applied %s' % ', '.join(changes))
    return revision
