"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture layout.json: plan B's board detection (src/glider/layout.py) as the Rust port (crates/layout) must
answer it. Everything compares EXACTLY: the verdicts, the evidence text, the boot-log lines, every change string,
every edit to the config, and the config_id of the config the board then runs.

Each case is {name, input, output}, and `input.function` names the plan B entry point:
  verdict  _verdict(seen) -- out: {revision, detail, found}; revision is null when undecided
  detect   detect(cfg) on scripted hardware -- out: {revision, detail, scanned}
  resolve  resolve(cfg) on scripted hardware (the boot path) -- out: {revision, resolved (layout.RESOLVED), log
           (the lines it logs), scanned, edits, config_id}
  apply    apply(cfg, revision, found) once per entry of `applies`, in order -- out: {changes (one list per apply),
           edits}
  rank     _rank(cfg, changes, revision) -- out: {changes, edits}
  fit      _fit(cfg, device, present, changes) -- out: {changes, edits}
  fitted   fitted(device, revision) for each device -- out: {device: bool}
A scan is {"0": [addresses], "1": [addresses]}, i2c:0 and i2c:1, and `raises` (when present) lists the bus ids whose
scan raises. The HARDWARE is scripted, not the scan: layout.I2C and layout.Pin are fakes (_Hardware), so plan B's own
_scan runs -- its `config.bus(...) is None` guard, the pins it reads before opening a bus, and its `except` -- and
`scanned` is the bus ids an I2C was opened on, in order: hardware touched. The Rust test's closure stands for that
I2C (it is called only where the crate's guard lets plan B open a bus), answers the scripted set, or fails for a bus
in `raises`.

A config is named as golden/config_corpus.py describes (a committed board file, or the `default` base the fixture
carries, then edits), keys sorted. `edits` is what the call changed in it: every member whose value differs after
the call, as {path, before, after}, depth first in sorted key and list order; a member added has no `before`. Two
values are the same only when their JSON text is (an int never equals a float or a bool here).

`config_id` checks the post-layout ids the port plan publishes for M5 (doc/specs/avionics-port.md): tms7c.config
as-is, with no board.layout key, resolves as a declared v0.1 and hashes to f6a9c35d6372 (an explicit 'v0.1' key
hashes to 92d537918c7c, so the builder must never add it); tms7f.config scanned as the taster (v1.1, no SEN0253)
hashes to d18751e70f6c. A corpus that disagrees stops the builder.

Inputs: test_layout.py's recorded scans (_TMS7C, _TASTER derived from its verdict, _LEGACY) and every case it runs,
the M5 board proof's live taster scan (_TASTER_M5, cases named m5_*), the five launches/20261003 board files against
each scan, every single dead address of each real scan, configs that define one bus or none and buses that fail,
priorities at the integer ceilings, float bus clocks, and seeded random scans through the verdict and the whole boot
path.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import copy
import json
import random

import config
import config_corpus
import config_default
import layout

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_VERDICT_SWEEPS: int = 300  # random scans through _verdict
_RESOLVE_SWEEPS: int = 60  # random scans through the boot path, on the default and the old profile

"""Scans as the boards answered, {bus_id: addresses}: test_layout.py:26-30, which says where each comes from."""
_TMS7C: dict = {0: {0x15, 0x18, 0x25, 0x29, 0x47, 0x63, 0x69}, 1: set()}
_TASTER: dict = {0: {0x15, 0x18, 0x40, 0x47, 0x69}, 1: {0x25, 0x29, 0x63}}
_LEGACY: dict = {0: {0x18, 0x28, 0x40, 0x76}, 1: {0x25, 0x29, 0x63}}

"""
The taster as the M5 board proof scans it (doc/specs/avionics-port.md M5): _TASTER without 0x29 (the VL53L1X, now on
7F) and with 0x36 (the OV5647's SCCB while the camera is powered). The on-board `detect` reply is compared with
m5_detect_taster word for word, so this is the scan the board must answer for.
"""
_TASTER_M5: dict = {0: {0x15, 0x18, 0x36, 0x40, 0x47, 0x69}, 1: {0x25, 0x63}}
_SEN0253: set = {0x28, 0x76}
_SEN0697: set = {0x15, 0x47, 0x69}

"""The real scans by name, for the board-file and dead-device sweeps."""
_REAL: tuple = (('tms7c', _TMS7C), ('taster', _TASTER), ('legacy', _LEGACY))

"""Addresses the random scans draw from: every one layout reads, the camera SCCB, and a few nobody reads."""
_POOL: tuple = (0x15, 0x18, 0x25, 0x28, 0x29, 0x36, 0x40, 0x47, 0x63, 0x69, 0x76, 0x08, 0x3C, 0x50, 0x77)

"""The post-layout config_ids the port plan publishes (M5): the builder refuses a corpus that disagrees."""
_PUBLISHED: dict = {
    'm5_tms7c_as_is_resolves_declared_v0_1': 'f6a9c35d6372',
    'm5_tms7c_with_an_explicit_layout_v0_1': '92d537918c7c',
    'm5_tms7f_scanned_on_the_taster': 'd18751e70f6c',
}

_TMS7C_FILE: str = 'launches/20261003/TMS-7C/tms7c.config'

_KNOWN: dict = config_corpus.bases(['default'])  # the fixture's `bases`


def _scan_json(seen: dict) -> dict:
    """A scan as the fixture writes it: both buses, addresses sorted."""
    return {'0': sorted(seen.get(0, set())), '1': sorted(seen.get(1, set()))}


class _Hardware:
    """
    machine.I2C and machine.Pin for layout._scan, scripted: an I2C opened on a bus answers that bus's addresses, or
    raises from scan() for a bus in `raises`. Used as a context manager that installs the fakes in layout (where the
    shims leave I2C and Pin None) and records every bus an I2C was opened on.
    """

    def __init__(self, seen: dict, raises: tuple = ()):
        self.seen: dict = seen
        self.raises: tuple = raises
        self.scanned: list = []  # the bus ids an I2C was opened on, in order

    def __enter__(self):
        hardware = self

        class I2C:
            """One bus opened for a scan, at the clock plan B scans at."""

            def __init__(self, bus_id: int, scl: object, sda: object, freq: int):
                if freq != layout._SCAN_HZ:
                    raise ValueError('a scan at %r Hz, not %r' % (freq, layout._SCAN_HZ))
                hardware.scanned.append(bus_id)
                self.bus_id: int = bus_id

            def scan(self) -> list:
                if self.bus_id in hardware.raises:
                    raise OSError(116, 'ETIMEDOUT: a shorted bus')
                return sorted(hardware.seen.get(self.bus_id, set()))

        self.saved = (layout.I2C, layout.Pin)
        layout.I2C, layout.Pin = I2C, lambda number: number
        return self

    def __exit__(self, *exception) -> bool:
        layout.I2C, layout.Pin = self.saved
        return False


def _with(seen: dict, addresses: set) -> dict:
    """The scan with extra addresses answering on i2c:0 (test_layout.py _with)."""
    return {0: set(seen[0]) | addresses, 1: set(seen[1])}


def _same(left: object, right: object) -> bool:
    """Whether two JSON values are the same: by their JSON text, so 1, 1.0 and True all differ."""
    return json.dumps(left, sort_keys=True) == json.dumps(right, sort_keys=True)


def _diff(before: object, after: object, path: tuple = ()) -> list:
    """
    What changed between two documents: {path, before, after} per differing member, depth first.

    Objects recurse by sorted key (a member only on one side has only that side), lists of one length by index;
    anything else that differs is one entry.
    """
    if isinstance(before, dict) and isinstance(after, dict):
        edits = []
        for key in sorted(set(before) | set(after)):
            if key not in after:
                edits.append({'path': list(path) + [key], 'before': before[key]})
            elif key not in before:
                edits.append({'path': list(path) + [key], 'after': after[key]})
            else:
                edits += _diff(before[key], after[key], path + (key,))
        return edits
    if isinstance(before, list) and isinstance(after, list) and len(before) == len(after):
        edits = []
        for index, (left, right) in enumerate(zip(before, after)):
            edits += _diff(left, right, path + (index,))
        return edits
    return [] if _same(before, after) else [{'path': list(path), 'before': before, 'after': after}]


def _default(*edits) -> dict:
    """The default config (board.layout 'auto') with edits, each (device, [keys...], value) or (path, value)."""
    return _spec({'base': 'default'}, edits)


def _file(path: str, *edits) -> dict:
    """A committed board file with edits, as _default."""
    return _spec({'file': path}, edits)


def _spec(specification: dict, edits: tuple) -> dict:
    """
    A config specification with its edits resolved to paths (a device is found by name in its section).

    Args:
        specification - {base} or {file}.
        edits - (device name, [keys...], value) to set inside a device, or (['path', ...], value) for any member;
            a value of _DELETE deletes the member.

    Returns:
        The config_corpus specification.
    """
    document = config_corpus.resolve(specification, _KNOWN)
    resolved = []
    for edit in edits:
        if len(edit) == 3:
            device, keys, value = edit
            section = 'sensors' if any(item.get('name') == device for item in document['sensors']) else 'components'
            path = [section, config_corpus.index_of(document, section, device)] + list(keys)
        else:
            path, value = edit
        resolved.append({'path': path, 'delete': True} if value is _DELETE else {'path': path, 'value': value})
    if resolved:
        specification = dict(specification, edits=resolved)
    return specification


_DELETE: object = object()  # an edit value that deletes the member


def _device_path(name: str) -> list:
    """The path of a device in the default config, to delete it whole."""
    document = config_corpus.resolve({'base': 'default'}, _KNOWN)
    section = 'sensors' if any(item.get('name') == name for item in document['sensors']) else 'components'
    return [section, config_corpus.index_of(document, section, name)]

"""test_layout.py _old_profile(): tms7f.config's priorities, as every config saved before 2026-10-05 carries them."""
_OLD_PROFILE: tuple = (
    ('imu_bno055', ['provides', 'attitude', 'priority'], 0),
    ('imu_bno055', ['provides', 'accel', 'priority'], 2),
    ('accel_adxl375', ['provides', 'accel', 'priority'], 1),
) + tuple(('baro_bmp280', ['provides', quantity, 'priority'], 1)
          for quantity in ('altitude', 'elevation', 'pressure', 'temperature'))


def _backups_enabled(value: bool = True) -> tuple:
    """Edits setting the SEN0253 parts' `enabled`."""
    return tuple((name, ['enabled'], value) for name in sorted(layout._BACKUP))


class _Cases:
    """The cases being built, and the config_ids the published vectors are checked against."""

    def __init__(self):
        self.cases: list = []
        self.ids: dict = {}

    def add(self, name: str, specification: dict, output: dict) -> None:
        """Keep one case."""
        if any(case['name'] == name for case in self.cases):
            raise ValueError('duplicate case %s' % name)
        self.cases.append({'name': name, 'input': specification, 'output': output})

    def verdict(self, name: str, seen: dict) -> None:
        """A _verdict case."""
        revision, detail, found = layout._verdict({0: set(seen.get(0, set())), 1: set(seen.get(1, set()))})
        self.add(name, {'function': 'verdict', 'scan': _scan_json(seen)},
                 {'revision': revision, 'detail': detail, 'found': list(found)})

    @staticmethod
    def _scripted(specification: dict, seen: dict, raises: tuple) -> dict:
        """A case input's config, scan and (when any bus fails) `raises`."""
        scripted = {'config': specification, 'scan': _scan_json(seen)}
        if raises:
            scripted['raises'] = list(raises)
        return scripted

    def detect(self, name: str, seen: dict, specification: dict = None, raises: tuple = ()) -> None:
        """A detect case, on the default config unless another is named."""
        specification = specification or _default()
        document = config_corpus.resolve(specification, _KNOWN)
        with _Hardware(seen, raises) as hardware:
            revision, detail = layout.detect(document)
        self.add(name, dict(function='detect', **self._scripted(specification, seen, raises)),
                 {'revision': revision, 'detail': detail, 'scanned': hardware.scanned})

    def resolve(self, name: str, specification: dict, seen: dict, raises: tuple = ()) -> str:
        """A resolve case; returns the config_id it ends on."""
        document = config_corpus.resolve(specification, _KNOWN)
        before = copy.deepcopy(document)
        lines = []
        with _Hardware(seen, raises) as hardware:
            revision = layout.resolve(document, log=lines.append)
        identity = config.config_id(document)
        self.ids[name] = identity
        self.add(name, dict(function='resolve', **self._scripted(specification, seen, raises)),
                 {'revision': revision, 'resolved': layout.RESOLVED, 'log': lines, 'scanned': hardware.scanned,
                  'edits': _diff(before, document), 'config_id': identity})
        return identity

    def apply(self, name: str, specification: dict, applies: list) -> None:
        """An apply case: apply(cfg, revision, found) for each (revision, found), in order."""
        document = config_corpus.resolve(specification, _KNOWN)
        before = copy.deepcopy(document)
        changes = [layout.apply(document, revision, None if found is None else tuple(found))
                   for revision, found in applies]
        self.add(name, {'function': 'apply', 'config': specification,
                        'applies': [{'revision': revision, 'found': found} for revision, found in applies]},
                 {'changes': changes, 'edits': _diff(before, document)})

    def rank(self, name: str, specification: dict, revision) -> None:
        """A _rank case."""
        document = config_corpus.resolve(specification, _KNOWN)
        before = copy.deepcopy(document)
        changes = []
        layout._rank(document, changes, revision)
        self.add(name, {'function': 'rank', 'config': specification, 'revision': revision},
                 {'changes': changes, 'edits': _diff(before, document)})

    def fit(self, name: str, specification: dict, device: str, present: bool) -> None:
        """A _fit case."""
        document = config_corpus.resolve(specification, _KNOWN)
        before = copy.deepcopy(document)
        changes = []
        layout._fit(document, device, present, changes)
        self.add(name, {'function': 'fit', 'config': specification, 'device': device, 'present': present},
                 {'changes': changes, 'edits': _diff(before, document)})

    def fitted(self, name: str, revision: str, devices: list) -> None:
        """A fitted case: one revision, every device named."""
        self.add(name, {'function': 'fitted', 'revision': revision, 'devices': devices},
                 {device: layout.fitted(device, revision) for device in devices})


def _verdicts(cases: _Cases) -> None:
    """test_layout.py's scans through _verdict and detect, then every dead address, then random scans."""
    for name, seen in _REAL:
        cases.verdict('verdict_%s' % name, seen)
        cases.verdict('verdict_%s_with_the_sen0253' % name, _with(seen, _SEN0253))
        cases.verdict('verdict_%s_without_the_sen0697' % name, {0: seen[0] - _SEN0697, 1: set(seen[1])})
        cases.verdict('verdict_%s_with_a_second_icp' % name, _with(seen, {0x63}))
    cases.verdict('verdict_empty', {0: set(), 1: set()})
    cases.verdict('verdict_the_anchor_alone', {0: {0x18}, 1: set()})
    cases.verdict('verdict_no_anchor', {0: {0x63, 0x25, 0x29}, 1: {0x40}})
    cases.verdict('verdict_the_anchor_on_i2c1_proves_nothing', {0: {0x63, 0x25, 0x29}, 1: {0x18, 0x40}})
    cases.verdict('verdict_the_icp_alone_on_i2c0', {0: {0x18, 0x63}, 1: set()})
    cases.verdict('verdict_the_sen0253_on_i2c1_is_not_the_backup', {0: set(_TASTER[0]), 1: _TASTER[1] | _SEN0253})
    for label, module in (('sen0697', _SEN0697), ('sen0253', _SEN0253), ('both', _SEN0697 | _SEN0253)):
        cases.verdict('verdict_module_%s_alone_is_undecided' % label, {0: {0x18} | module, 1: set()})
    tie = {0: {0x18, 0x29, 0x40}, 1: set()}  # 0x29 on i2c:0 says v0.1, 0x40 on i2c:0 says v1.x
    for label, module in (('bare', set()), ('sen0697', _SEN0697), ('both', _SEN0697 | _SEN0253)):
        cases.verdict('verdict_placement_tie_%s' % label, _with(tie, module))
    for anchor in layout._ANCHORS:
        cases.verdict('verdict_anchor_0x%02x_with_one_vote' % anchor, {0: {anchor, 0x25}, 1: set()})

    # test_one_dead_device_never_flips_a_verdict: every address of every real scan lost, one at a time
    for name, seen in _REAL + (('tms7c_with_the_sen0253', _with(_TMS7C, _SEN0253)),
                                ('taster_with_the_sen0253', _with(_TASTER, _SEN0253))):
        for bus_id in (0, 1):
            for address in sorted(seen[bus_id]):
                lost = {0: set(seen[0]), 1: set(seen[1])}
                lost[bus_id].discard(address)
                cases.verdict('verdict_%s_without_i2c%d_0x%02x' % (name, bus_id, address), lost)

    for name, seen in (('tms7c', _TMS7C), ('taster', _TASTER), ('empty', {0: set(), 1: set()})):
        cases.detect('detect_%s' % name, seen)

    # plan B's _scan guard: a bus the config does not define (or cannot give pins for) is not opened, a failing one
    # answers nothing; either way it casts no votes
    for label, edit in (('no_i2c1_spec', (['buses', 'i2c', '1'], _DELETE)),
                        ('a_null_i2c1_spec', (['buses', 'i2c', '1'], None)),
                        ('an_i2c1_spec_that_is_not_an_object', (['buses', 'i2c', '1'], 'fast')),
                        ('an_i2c1_spec_without_scl', (['buses', 'i2c', '1', 'scl'], _DELETE)),
                        ('an_i2c1_spec_without_sda', (['buses', 'i2c', '1', 'sda'], _DELETE)),
                        ('no_i2c0_spec', (['buses', 'i2c', '0'], _DELETE)),
                        ('no_i2c_buses', (['buses', 'i2c'], _DELETE)),
                        ('no_buses', (['buses'], _DELETE))):
        cases.detect('detect_%s' % label, _TASTER, _default(edit))
    for bus_id in (0, 1):
        cases.detect('detect_a_failing_i2c%d' % bus_id, _TASTER, raises=(bus_id,))
    cases.detect('detect_both_buses_failing', _TASTER, raises=(0, 1))

    # the M5 board proof: the taster's live scan, word for word
    cases.verdict('m5_verdict_taster', _TASTER_M5)
    cases.detect('m5_detect_taster', _TASTER_M5)

    generator = random.Random(_SEED)
    for index in range(_VERDICT_SWEEPS):
        cases.verdict('verdict_random_%03d' % index, _random_scan(generator))


def _random_scan(generator: random.Random) -> dict:
    """A random scan over _POOL: each address answers on i2c:0, i2c:1, both or neither."""
    seen = {0: set(), 1: set()}
    for address in _POOL:
        for bus_id in (0, 1):
            if generator.random() < 0.35:
                seen[bus_id].add(address)
    return seen


def _resolves(cases: _Cases) -> None:
    """test_layout.py's boot paths, the board files against each scan, the published ids, and random scans."""
    for name, seen in _REAL:
        cases.resolve('resolve_%s' % name, _default(), seen)
        cases.resolve('resolve_%s_with_the_sen0253' % name, _default(), _with(seen, _SEN0253))
        cases.resolve('resolve_%s_on_an_old_profile' % name, _default(*_OLD_PROFILE), _with(seen, _SEN0253))
        cases.resolve('resolve_%s_old_profile_without_the_sen0253' % name, _default(*_OLD_PROFILE), seen)
        for label, remnant in (('nothing', set()), ('the_sen0253', _SEN0253)):
            cases.resolve('resolve_%s_missing_the_sen0697_with_%s' % (name, label), _default(),
                          {0: (seen[0] - _SEN0697) | remnant, 1: set(seen[1])})
    cases.resolve('resolve_half_a_sen0253', _default(), _with(_TASTER, {0x28}))
    cases.resolve('resolve_a_stale_profile_claiming_the_sen0253', _default(*_backups_enabled()), _TMS7C)
    cases.resolve('resolve_fitted_false_beats_the_scan', _default(('baro_bmp280', ['fitted'], False)),
                  _with(_TASTER, _SEN0253))
    cases.resolve('resolve_the_sen0253_on_i2c1', _default(), {0: set(_TASTER[0]), 1: _TASTER[1] | _SEN0253})
    for declared in ('v0.1', 'v1.0', 'v1.1'):
        layout_edit = (['board', 'layout'], declared)
        cases.resolve('resolve_declared_%s_beats_the_scan' % declared, _default(layout_edit),
                      _with(_TMS7C, _SEN0253))
        cases.resolve('resolve_declared_%s_old_profile_backups_enabled' % declared,
                      _default(*(_OLD_PROFILE + _backups_enabled() + (layout_edit,))), _TASTER)
        cases.resolve('resolve_declared_%s_old_profile' % declared, _default(*(_OLD_PROFILE + (layout_edit,))),
                      _TASTER)
    cases.resolve('resolve_undecided_still_ranks', _default(*(_OLD_PROFILE + (('imu_bno055', ['enabled'], True),))),
                  {0: {0x18} | _SEN0697 | _SEN0253, 1: set()})
    cases.resolve('resolve_undecided_leaves_the_config_as_written', _default(), {0: {0x18} | _SEN0697, 1: set()})
    cases.resolve('resolve_never_promotes', _default(*(_OLD_PROFILE + (('imu_bno055', ['provides', 'attitude',
                                                                                        'priority'], 5),))),
                  _with(_TASTER, _SEN0253))
    cases.resolve('resolve_a_keyless_config_is_v0_1', _default((['board', 'layout'], _DELETE)), _TASTER)
    cases.resolve('resolve_a_null_layout_scans', _default((['board', 'layout'], None)), _TASTER)
    cases.resolve('resolve_an_unknown_layout_scans', _default((['board', 'layout'], 'v2.0')), _TMS7C)
    cases.resolve('resolve_empty_scan', _default(), {0: set(), 1: set()})
    cases.resolve('resolve_no_anchor', _default(), {0: {0x63, 0x25, 0x29}, 1: {0x40}})

    # the _scan guard on the boot path: one bus defined, a bus without pins, none, and failing buses
    cases.resolve('resolve_no_i2c1_spec', _default((['buses', 'i2c', '1'], _DELETE)), _TASTER)
    cases.resolve('resolve_no_i2c1_spec_on_tms7c', _default((['buses', 'i2c', '1'], _DELETE)), _TMS7C)
    cases.resolve('resolve_a_null_i2c1_spec', _default((['buses', 'i2c', '1'], None)), _TASTER)
    cases.resolve('resolve_an_i2c1_spec_without_scl', _default((['buses', 'i2c', '1', 'scl'], _DELETE)), _TASTER)
    cases.resolve('resolve_no_i2c0_spec', _default((['buses', 'i2c', '0'], _DELETE)), _TASTER)
    cases.resolve('resolve_no_buses', _default((['buses'], _DELETE)), _TASTER)
    for bus_id in (0, 1):
        for name, seen in _REAL:
            cases.resolve('resolve_%s_with_a_failing_i2c%d' % (name, bus_id), _default(), seen, raises=(bus_id,))
    cases.resolve('resolve_both_buses_failing', _default(), _TASTER, raises=(0, 1))

    # the M5 board proof's live taster scan, through the boot path on 7F's file and on the default
    cases.resolve('m5_resolve_tms7f_on_the_live_taster_scan', _file(config_corpus.TMS7F_FILE), _TASTER_M5)
    cases.resolve('m5_resolve_default_on_the_live_taster_scan', _default(), _TASTER_M5)

    # the published M5 ids, then every board file against every scan
    cases.resolve('m5_tms7c_as_is_resolves_declared_v0_1', _file(_TMS7C_FILE), _TASTER)
    cases.resolve('m5_tms7c_with_an_explicit_layout_v0_1', _file(_TMS7C_FILE, (['board', 'layout'], 'v0.1')), _TASTER)
    cases.resolve('m5_tms7f_scanned_on_the_taster', _file(config_corpus.TMS7F_FILE), _TASTER)
    for path in config_corpus.LAUNCH_FILES:
        board = path.rsplit('/', 1)[1][:-len('.config')]
        for name, seen in _REAL:
            cases.resolve('board_file_%s_on_the_%s_scan' % (board, name), _file(path), seen)

    generator = random.Random(_SEED + 1)
    for index in range(_RESOLVE_SWEEPS):
        profile = _default(*_OLD_PROFILE) if index % 2 else _default()
        cases.resolve('resolve_random_%02d' % index, profile, _random_scan(generator))


def _applies(cases: _Cases) -> None:
    """test_layout.py's apply cases, every revision with every `found`, the board files, then _rank, _fit, fitted."""
    cases.apply('apply_the_default_is_v0_1', _default(), [('v0.1', None)])
    cases.apply('apply_v1_0_then_v1_1_then_back', _default(),
                [('v1.0', None), ('v1.1', None), ('v0.1', None), ('v0.1', None)])
    for revision in layout._REVISIONS:
        for label, found in (('no_scan', None), ('found_nothing', []), ('found_both', ['baro_bmp280', 'imu_bno055']),
                             ('found_the_bno055', ['imu_bno055']), ('found_the_bmp280', ['baro_bmp280'])):
            cases.apply('apply_%s_%s' % (revision, label), _default(), [(revision, found)])
            cases.apply('apply_%s_%s_old_profile' % (revision, label), _default(*_OLD_PROFILE), [(revision, found)])
        for wanted in (False, True):
            cases.apply('apply_declared_%s_backups_%s' % (revision, 'on' if wanted else 'off'),
                        _default(*_backups_enabled(wanted)), [(revision, None)])
        cases.apply('apply_%s_twice_is_a_no_op' % revision, _default(*_OLD_PROFILE),
                    [(revision, ['imu_bno055']), (revision, ['imu_bno055'])])
    cases.apply('apply_v1_0_fitted_false_beats_v1_0', _default(('baro_bmp280', ['fitted'], False)), [('v1.0', None)])
    cases.apply('apply_v1_1_fitted_false_keeps_the_bmp581_off',
                _default(('baro_bmp581', ['fitted'], False), ('baro_bmp581', ['enabled'], False)), [('v1.1', None)])
    cases.apply('apply_v1_1_fitted_zero_is_not_false', _default(('baro_bmp581', ['fitted'], 0),
                                                               ('baro_bmp581', ['enabled'], False)), [('v1.1', None)])
    cases.apply('apply_v0_1_adxl_not_demoted_without_the_bmi323',
                _default(*(_OLD_PROFILE + (('imu_bmi323', ['fitted'], False),))), [('v0.1', None)])
    cases.apply('apply_v1_1_a_disabled_backup_is_not_ranked', _default(*_OLD_PROFILE), [('v1.1', [])])
    cases.apply('apply_v1_1_moves_a_device_on_spi', _default(('laser_agl', ['bus'], 'spi')), [('v1.1', None)])
    cases.apply('apply_v1_1_keeps_a_device_already_on_its_bus', _default(('laser_agl', ['id'], 1),
                                                                         ('laser_agl', ['bus'], 'spi')),
                [('v1.1', None)])
    cases.apply('apply_v1_1_without_the_laser_l1x', _default((_device_path('laser_agl_l1x'), _DELETE)),
                [('v1.1', None)])
    cases.apply('apply_v1_1_an_i2c1_without_a_clock', _default((['buses', 'i2c', '1', 'freq'], _DELETE)),
                [('v1.1', None)])
    cases.apply('apply_v1_1_no_i2c1', _default((['buses', 'i2c', '1'], _DELETE)), [('v1.1', None)])
    cases.apply('apply_v0_1_a_float_clock', _default((['buses', 'i2c', '1', 'freq'], 100000.0)), [('v0.1', None)])
    for label, clock in (('huge', 1e40), ('negative_fraction', -0.5), ('fraction', 99999.9)):
        cases.apply('apply_v1_1_a_%s_float_clock' % label, _default((['buses', 'i2c', '1', 'freq'], clock)),
                    [('v1.1', None)])
    cases.apply('apply_v1_1_an_unrouted_pin_absent', _default(('laser_agl', ['int_pin'], _DELETE)),
                [('v1.1', None)])
    for path in config_corpus.LAUNCH_FILES:
        board = path.rsplit('/', 1)[1][:-len('.config')]
        for revision in layout._REVISIONS:
            cases.apply('apply_board_file_%s_%s' % (board, revision), _file(path), [(revision, None)])

    for revision in (None, 'v0.1', 'v1.0', 'v1.1'):
        label = revision or 'undecided'
        cases.rank('rank_old_profile_%s' % label, _default(*(_OLD_PROFILE + _backups_enabled())), revision)
        cases.rank('rank_default_%s' % label, _default(), revision)
    cases.rank('rank_a_missing_priority_reads_0', _default(*(_backups_enabled() + (
        ('imu_bno055', ['provides', 'attitude', 'priority'], _DELETE),))), 'v1.1')
    cases.rank('rank_a_superior_without_the_quantity', _default(*(_backups_enabled() + (
        ('attitude', ['provides'], {}),))), 'v1.1')
    cases.rank('rank_null_provides', _default(*(_backups_enabled() + (('imu_bno055', ['provides'], None),))), 'v1.1')
    # priorities at the integer ceilings: plan B's int has none, so one past i64::MAX is written as it is
    for label, superior in (('at_the_i64_ceiling', (1 << 63) - 1), ('past_the_i64_ceiling', (1 << 63) + 5)):
        cases.rank('rank_a_superior_%s' % label, _default(*(_backups_enabled() + (
            ('attitude', ['provides', 'attitude', 'priority'], superior),))), 'v1.1')
    cases.rank('rank_a_backup_past_the_i64_ceiling_stays', _default(*(_backups_enabled() + (
        ('imu_bno055', ['provides', 'attitude', 'priority'], 1 << 63),))), 'v1.1')
    cases.rank('rank_negative_priorities', _default(*(_backups_enabled() + (
        ('imu_bno055', ['provides', 'attitude', 'priority'], -(1 << 63)),
        ('attitude', ['provides', 'attitude', 'priority'], -7)))), 'v1.1')

    cases.fit('fit_enables_a_present_part', _default(('imu_bmi323', ['enabled'], False)), 'imu_bmi323', True)
    cases.fit('fit_disables_an_absent_part', _default(), 'imu_bmi323', False)
    cases.fit('fit_already_matching', _default(), 'imu_bmi323', True)
    cases.fit('fit_an_absent_enabled_key_reads_true', _default(('imu_bmi323', ['enabled'], _DELETE)), 'imu_bmi323',
              True)
    cases.fit('fit_fitted_false_disables_a_present_part', _default(('imu_bmi323', ['fitted'], False)), 'imu_bmi323',
              True)
    cases.fit('fit_fitted_true_is_the_default', _default(('imu_bmi323', ['fitted'], True),
                                                         ('imu_bmi323', ['enabled'], False)), 'imu_bmi323', True)
    cases.fit('fit_an_unknown_device_is_skipped', _default(), 'no_such_device', True)

    devices = sorted(item['name'] for item in config_default.default()['sensors']
                     + config_default.default()['components'])
    devices += ['laser_ground', 'no_such_device']
    for revision in ('v0.1', 'v1.0', 'v1.1', 'undecided', 'auto'):
        cases.fitted('fitted_%s' % revision.replace('.', '_'), revision, devices)


def build() -> dict:
    """
    The layout fixture document.

    Returns:
        {meta, compare, bases, cases}.

    Raises:
        ValueError - a published M5 config_id disagrees with plan B.
    """
    cases = _Cases()
    _verdicts(cases)
    _resolves(cases)
    _applies(cases)
    wrong = {name: (cases.ids.get(name), wanted) for name, wanted in _PUBLISHED.items()
             if cases.ids.get(name) != wanted}
    if wrong:
        raise ValueError('layout: the published M5 config_ids disagree (got, published): %r' % wrong)
    return {
        'meta': {'source': 'src/glider/layout.py, src/glider/config.py, src/glider/config_default.py, '
                           'launches/20261003/*/*.config'},
        'compare': {'kind': 'exact'},
        'bases': _KNOWN,
        'cases': cases.cases,
    }
