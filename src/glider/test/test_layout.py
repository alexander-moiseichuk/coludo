"""
layout.py: the bus map by placement, the SEN0253 backup by scan and ranked below the SEN0697, what it rewrites.

Runs on the board and exercises the pure decision logic against REAL scans recorded from the fleet, so
every verdict is checked without needing every board on the bench. The live end -- detect() against the
real buses -- is the last case, which asserts the board this runs on is recognised at all rather than
which revision, so the file does not go red the moment the breadboard is rewired.
"""

import config
import config_default
import i2cbus
import layout

"""
Scans as the boards actually answered, {bus_id: addresses}.

TMS-7C: the one v0.1 board left, re-fitted with a SEN0697 (2026-10-05, HEAD 4be46b6). Its INA226 is
absent, so it casts THREE placement votes (ICP, pitot, laser on i2c:0) -- two while the ICP counted from
i2c:1 only, and the old single tally let its three SEN0697 parts outvote those two and called it v1.1.
TASTER: the v1.1 bench, derived from its recorded verdict (2026-09-16, votes v0.1 0 / v1.0 4 / v1.1 7:
four placement votes + three SEN0697 parts, no SEN0253) plus the ES8311 anchor.
LEGACY: the taster before the SEN0697, as scanned 2026-09-13 (TMS-7E's build). A scan now reads it as
v1.1 with a missing SEN0697 -- v1.0 is declarable only.
"""
_TMS7C: dict = {0: {0x15, 0x18, 0x25, 0x29, 0x47, 0x63, 0x69}, 1: set()}
_TASTER: dict = {0: {0x15, 0x18, 0x40, 0x47, 0x69}, 1: {0x25, 0x29, 0x63}}
_LEGACY: dict = {0: {0x18, 0x28, 0x40, 0x76}, 1: {0x25, 0x29, 0x63}}
_SEN0253: set = {0x28, 0x76}
_SEN0697: set = {0x15, 0x47, 0x69}

_PRIMARY_SOURCES: tuple = ('imu_bmi323', 'baro_bmp581', 'mag_bmm350', 'attitude')  # attitude = their filter
_BACKUP_SOURCES: tuple = ('imu_bno055', 'baro_bmp280')


def _cfg() -> dict:
    """A fresh default config to mutate per case, set to scan."""
    cfg = config_default.default()
    cfg['board']['layout'] = 'auto'
    return cfg


def _copy(seen: dict) -> dict:
    """A scan to mutate without touching the fixture."""
    return {0: set(seen[0]), 1: set(seen[1])}


def _with(seen: dict, addresses: set) -> dict:
    """The scan with extra addresses answering on i2c:0 (where both attitude modules sit)."""
    grown = _copy(seen)
    grown[0] |= addresses
    return grown


def _vote(cfg: dict, seen: dict) -> tuple:
    """Run detect() against a synthetic scan by swapping _scan out."""
    original = layout._scan
    layout._scan = lambda _cfg, bus_id: seen.get(bus_id, set())
    try:
        return layout.detect(cfg)
    finally:
        layout._scan = original


def _resolve(seen: dict, cfg: dict = None) -> tuple:
    """The boot path (resolve) against a synthetic scan: (revision, the config it rewrote)."""
    cfg = _cfg() if cfg is None else cfg
    original = layout._scan
    layout._scan = lambda _cfg, bus_id: seen.get(bus_id, set())
    try:
        return layout.resolve(cfg, log=lambda _line: None), cfg
    finally:
        layout._scan = original


def _enabled(cfg: dict, name: str) -> bool:
    return config.device(cfg, name=name).get('enabled', True)


def _priority(cfg: dict, name: str, quantity: str) -> int:
    return config.device(cfg, name=name)['provides'][quantity].get('priority', 0)


def _old_profile() -> dict:
    """
    A config as saved before 2026-10-05 -- tms7f.config's priorities: the BNO055 at attitude p0 (accel
    p2), the BMP280 at p1 tied with the BMP581, the ADXL375 at accel p1 tied with the BMI323. What every
    taster / 7E / 7F board.config and launch profile still carries, and what layout must rank correctly
    whatever it says.
    """
    cfg = _cfg()
    priorities = (('imu_bno055', 'attitude', 0), ('imu_bno055', 'accel', 2), ('accel_adxl375', 'accel', 1))
    for name, quantity, priority in priorities:
        config.device(cfg, name=name)['provides'][quantity]['priority'] = priority
    for quantity in ('altitude', 'elevation', 'pressure', 'temperature'):
        config.device(cfg, name='baro_bmp280')['provides'][quantity]['priority'] = 1
    return cfg


def _assert_ranked(cfg: dict, label: str) -> int:
    """
    Every ENABLED backup quantity sits strictly below every enabled SEN0697 / filter provider of it, the
    ADXL375 below the BMI323, the BNO055 below the ADXL375; returns how many pairs were checked.
    """
    pairs = 0
    rules = [(name, _PRIMARY_SOURCES) for name in _BACKUP_SOURCES]
    rules += [('accel_adxl375', ('imu_bmi323',)), ('imu_bno055', ('accel_adxl375',))]
    for lower, uppers in rules:
        if not _enabled(cfg, lower):
            continue
        for quantity in config.device(cfg, name=lower)['provides']:
            for upper in uppers:
                provided = config.device(cfg, name=upper)['provides'].get(quantity)
                if provided is None or not _enabled(cfg, upper):
                    continue
                pairs += 1
                assert _priority(cfg, lower, quantity) > provided.get('priority', 0), (
                    '%s: %s %s p%d must rank below %s p%d' % (
                        label, lower, quantity, _priority(cfg, lower, quantity), upper, provided.get('priority', 0)))
    return pairs


def _bus_id(cfg: dict, name: str) -> int:
    return config.device(cfg, name=name)['id']


def test_real_scans():
    """
    Every board in the fleet resolves to its own map and modules, with the config rewritten to match.

    TMS-7C is the regression this file was rewritten for: three SEN0697 votes beat two placement votes,
    v1.1 was applied, and the laser, pitot and ICP-10111 were hunted on an empty i2c:1 -- ENODEV on all
    three, and test_vl53l1x the one red test in the board suite.
    """
    revision, cfg = _resolve(_TMS7C)
    assert revision == 'v0.1' and layout.RESOLVED == 'v0.1', revision
    for name in ('baro_icp10111', 'airspeed_sdp810', 'laser_agl', 'laser_agl_l1x'):
        assert _bus_id(cfg, name) == 0, '7C: %s must stay on i2c:0' % name
    assert _bus_id(cfg, 'power_ina226') == 1 and config.bus(cfg, 'i2c', 1)['freq'] == 400000
    for name in layout._PRIMARY:
        assert _enabled(cfg, name), '7C carries the SEN0697: %s must be enabled' % name
    for name in layout._BACKUP:
        assert not _enabled(cfg, name), '7C has no SEN0253: %s must be disabled' % name
    assert _enabled(cfg, 'accel_adxl375'), 'the ADXL375 stays fitted on v0.1'
    # the ADXL375 owns GPIO4 on v0.1; the BMI323 sits in a 4-wire socket and must not claim it too
    assert config.device(cfg, name='imu_bmi323').get('int_pin') is None
    assert config.device(cfg, name='accel_adxl375')['int_pin'] == 'accel_int1'
    assert config.device(cfg, name='laser_agl')['xshut_pin'] == 'laser_xshut', 'v0.1 routes the laser pins'

    revision, cfg = _resolve(_TASTER)
    assert revision == 'v1.1', revision
    for name in ('baro_icp10111', 'airspeed_sdp810', 'laser_agl', 'laser_agl_l1x'):
        assert _bus_id(cfg, name) == 1, 'taster: %s must move to i2c:1' % name
    assert _bus_id(cfg, 'power_ina226') == 0 and config.bus(cfg, 'i2c', 1)['freq'] == 100000
    for name in layout._PRIMARY:
        assert _enabled(cfg, name), name
    for name in layout._BACKUP:
        assert not _enabled(cfg, name), name
    assert not _enabled(cfg, 'accel_adxl375'), 'no ADXL375 on the v1.x map'
    assert config.device(cfg, name='imu_bmi323')['int_pin'] == 'accel_int1', 'v1.x routes GPIO4 to the BMI323'
    assert config.device(cfg, name='laser_agl')['xshut_pin'] is None, 'v1.x does not route the laser pins'

    """
    LEGACY: a scan never names v1.0. The SEN0253 alone on the v1.x map is v1.1 with its SEN0697 MISSING:
    the SEN0697 stays enabled, so its setup fails and `arm` refuses, and the SEN0253 found is the
    backup -- ranked below the filter, not flying as the primary.
    """
    revision, cfg = _resolve(_LEGACY)
    assert revision == 'v1.1', 'a scan must never name v1.0, got %r' % revision
    for name in layout._PRIMARY:
        assert _enabled(cfg, name), 'a missing SEN0697 must stay expected (fail loudly): %s' % name
    for name in layout._BACKUP:
        assert _enabled(cfg, name), 'the SEN0253 the scan found is the backup: %s' % name
    assert _bus_id(cfg, 'laser_agl') == 1 and not _enabled(cfg, 'accel_adxl375')
    assert _assert_ranked(cfg, 'legacy') >= 6


def test_both_modules_primary_and_backup():
    """
    A board carrying BOTH modules (a taster, an experimental board) on either map: the SEN0697 is the
    primary, the SEN0253 is enabled because the scan found it -- and ranked BELOW the SEN0697 sources on
    every quantity they share, or "backup" would mean nothing to the databoard.
    """
    for seen, expected in ((_TMS7C, 'v0.1'), (_TASTER, 'v1.1')):
        revision, cfg = _resolve(_with(seen, _SEN0253))
        assert revision == expected, 'a SEN0253 beside the SEN0697 moved the verdict: %r' % revision
        for name in tuple(layout._PRIMARY) + tuple(layout._BACKUP):
            assert _enabled(cfg, name), '%s: both modules answer, %s must be enabled' % (expected, name)

        shared = _assert_ranked(cfg, expected)
        assert shared >= 6, 'attitude, accel, altitude, elevation, pressure, temperature: %d checked' % shared

    # only what ANSWERED is enabled: half a SEN0253 (a dead BMP280) enables only its BNO055
    revision, cfg = _resolve(_with(_TASTER, {0x28}))
    assert revision == 'v1.1' and _enabled(cfg, 'imu_bno055') and not _enabled(cfg, 'baro_bmp280')

    # NEGATIVE: a config that claims the SEN0253 (saved before the swap) is overruled by a scan that
    # does not find it -- the backup is off when absent, whatever the profile says
    stale = _cfg()
    for name in layout._BACKUP:
        config.device(stale, name=name)['enabled'] = True
    revision, cfg = _resolve(_TMS7C, stale)
    assert revision == 'v0.1' and not _enabled(cfg, 'imu_bno055') and not _enabled(cfg, 'baro_bmp280')

    # NEGATIVE: `fitted: false` keeps a FOUND backup part off -- the operator's word beats the scan
    unfitted = _cfg()
    config.device(unfitted, name='baro_bmp280')['fitted'] = False
    revision, cfg = _resolve(_with(_TASTER, _SEN0253), unfitted)
    assert not _enabled(cfg, 'baro_bmp280') and _enabled(cfg, 'imu_bno055')


def test_backup_ranks_below_whatever_the_config_says():
    """
    The SEN0253 ranks below the SEN0697 and the filter IN CODE, not by config_default alone.

    The regression: a config saved before 2026-10-05 (tms7f.config, any taster board.config) carries
    the BNO055 at attitude p0, so a SEN0253 found beside the SEN0697 flew as the PRIMARY and the filter
    mirrored it. Replayed here on the taster and on 7C, scanned and declared.
    """
    for seen, expected in ((_TASTER, 'v1.1'), (_TMS7C, 'v0.1')):
        revision, cfg = _resolve(_with(seen, _SEN0253), _old_profile())
        assert revision == expected, revision
        assert _priority(cfg, 'imu_bno055', 'attitude') == 2, 'the BNO055 must drop below the filter (p1)'
        assert _priority(cfg, 'baro_bmp280', 'altitude') == 2, 'the BMP280 must drop below the BMP581 (p1)'
        assert _assert_ranked(cfg, 'old profile on %s' % expected) >= 6

    """
    The ADXL375 (+/-200 g backstop) ranks one below the BMI323 wherever both are enabled -- v0.1 --
    whatever the profile said (p1, tied); the LSM6DSO32 stays the primary. accel on v0.1 with both
    modules: LSM6DSO32 p0 -> BMI323 p1 -> ADXL375 p2 -> BNO055 p3.
    """
    revision, cfg = _resolve(_with(_TMS7C, _SEN0253), _old_profile())
    order = [(_priority(cfg, name, 'accel'), name)
             for name in ('imu_lsm6dso32', 'imu_bmi323', 'accel_adxl375', 'imu_bno055')]
    assert sorted(order) == order and len(set(p for p, _name in order)) == 4, order
    revision, cfg = _resolve(_TMS7C, _old_profile())
    assert _priority(cfg, 'accel_adxl375', 'accel') == 2 and _priority(cfg, 'imu_bmi323', 'accel') == 1

    # DECLARED with an explicitly enabled SEN0253: the same demotion, no scan needed
    for declared in ('v0.1', 'v1.1'):
        cfg = _old_profile()
        cfg['board']['layout'] = declared
        for name in layout._BACKUP:
            config.device(cfg, name=name)['enabled'] = True
        _revision, cfg = _resolve(_TASTER, cfg)
        assert _enabled(cfg, 'imu_bno055') and _priority(cfg, 'imu_bno055', 'attitude') == 2, declared
        assert _assert_ranked(cfg, 'declared %s' % declared) >= 6

    # NEGATIVE: a DECLARED v1.0 is the SEN0253 alone, with no BMM350 -- the BNO055 keeps its own rank (p0
    # in the old profile) so it seeds the filter's heading, exactly as it flew there; nothing is demoted
    cfg = _old_profile()
    cfg['board']['layout'] = 'v1.0'
    _revision, cfg = _resolve(_TASTER, cfg)
    assert _enabled(cfg, 'imu_bno055') and _priority(cfg, 'imu_bno055', 'attitude') == 0, 'v1.0 demoted its module'
    assert _priority(cfg, 'baro_bmp280', 'altitude') == 1, 'v1.0 demoted its baro'

    # UNDECIDED leaves the bus map as written but still ranks: the rule depends on no bus
    cfg = _old_profile()
    config.device(cfg, name='imu_bno055')['enabled'] = True
    revision, cfg = _resolve({0: {0x18} | _SEN0697 | _SEN0253, 1: set()}, cfg)
    assert revision is None and _priority(cfg, 'imu_bno055', 'attitude') == 2

    # NEGATIVE: never PROMOTES -- a backup the config already put further down stays there
    cfg = _old_profile()
    config.device(cfg, name='imu_bno055')['provides']['attitude']['priority'] = 5
    _revision, cfg = _resolve(_with(_TASTER, _SEN0253), cfg)
    assert _priority(cfg, 'imu_bno055', 'attitude') == 5
    # NEGATIVE: a backup that is NOT enabled is not touched (no boot-log noise for a part that is absent)
    cfg = _old_profile()
    changes = layout.apply(cfg, 'v1.1', ())
    assert _priority(cfg, 'imu_bno055', 'attitude') == 0 and not [c for c in changes if 'p0->' in c], changes
    # NEGATIVE: the ADXL375 is not demoted where the BMI323 is not enabled (nothing to rank against)
    cfg = _old_profile()
    config.device(cfg, name='imu_bmi323')['fitted'] = False
    layout.apply(cfg, 'v0.1')
    assert _priority(cfg, 'accel_adxl375', 'accel') == 1
    # the default is already ranked: apply() changes nothing on it, with or without a SEN0253 found
    assert layout.apply(config_default.default(), 'v0.1') == []
    found = config_default.default()
    layout.apply(found, 'v1.1', tuple(layout._BACKUP))
    assert not [change for change in layout.apply(found, 'v1.1', tuple(layout._BACKUP)) if '->p' in change]
    assert not [change for change in layout.apply(config_default.default(), 'v1.1', tuple(layout._BACKUP))
                if '->p' in change], 'config_default must already rank the SEN0253 below the SEN0697'



def test_placement_alone_decides_the_map():
    """
    The attitude module NEVER votes for a map: it is on i2c:0 whichever map this is.

    So a scan with no placement evidence is undecided however many module parts answer, and a placement
    TIE stays undecided -- the module cannot break it. Undecided changes nothing, which is the safe
    failure: a wrong map re-buses every moving sensor at once.
    """
    cfg = _cfg()
    for module in (_SEN0697, _SEN0253, _SEN0697 | _SEN0253):
        got, detail = _vote(cfg, {0: {0x18} | module, 1: set()})
        assert got is None, 'module parts alone picked %r (%s)' % (got, detail)

    tie = {0: {0x18, 0x29, 0x40}, 1: set()}  # 0x29 on i2c:0 says v0.1, 0x40 on i2c:0 says v1.x
    for module in (set(), _SEN0697, _SEN0697 | _SEN0253):
        got, detail = _vote(cfg, _with(tie, module))
        assert got is None and 'ambiguous' in detail, 'a 1-1 placement tie resolved to %r (%s)' % (got, detail)

    # the 7C verdict is the placement's, and it says so: 3-0, the three module parts counted separately
    got, detail = _vote(cfg, _TMS7C)
    assert got == 'v0.1' and 'v0.1:3 v1.x:0' in detail and 'SEN0697 3/3' in detail, detail


def test_votes():
    """
    NEGATIVE, and the one that matters most: an EMPTY or ANCHOR-ONLY scan must decide NOTHING, and a
    scan with no anchor is not trustworthy even when the rest looks like a board.
    """
    cfg = _cfg()
    got, _ = _vote(cfg, {0: set(), 1: set()})
    assert got is None, 'an empty scan must not pick a revision'
    got, _ = _vote(cfg, {0: {0x18}, 1: set()})
    assert got is None, 'the anchor alone must not pick a revision'
    got, detail = _vote(cfg, {0: {0x63, 0x25, 0x29}, 1: {0x40}})
    assert got is None and 'anchor' in detail, 'no anchor on i2c:0 -> the scan is not trustworthy'
    # the anchor counts on i2c:0 ONLY -- the codec is soldered there; an 0x18 on i2c:1 proves nothing
    got, detail = _vote(cfg, {0: {0x63, 0x25, 0x29}, 1: {0x18, 0x40}})
    assert got is None and 'anchor' in detail, 'an anchor on i2c:1 must not make the scan trustworthy'
    # the SEN0253 is looked for on i2c:0 only: 0x28/0x76 answering on i2c:1 are not the backup
    revision, cfg = _resolve({0: _TASTER[0], 1: _TASTER[1] | _SEN0253})
    assert revision == 'v1.1' and not _enabled(cfg, 'imu_bno055') and not _enabled(cfg, 'baro_bmp280')


def test_one_dead_device_never_flips_a_verdict():
    """
    Losing any SINGLE address from any real scan must leave the verdict standing -- the reason the map
    is spread over four movers and the module over three parts. The anchor included: 7C and the taster
    still have SEN0697 anchors without the codec, the legacy board its SEN0253 ones.
    """
    cfg = _cfg()
    for seen in (_TMS7C, _TASTER, _LEGACY, _with(_TMS7C, _SEN0253), _with(_TASTER, _SEN0253)):
        expected, _detail = _vote(cfg, seen)
        assert expected in layout._REVISIONS, seen
        for bus_id in (0, 1):
            for addr in seen[bus_id]:
                lost = _copy(seen)
                lost[bus_id].discard(addr)
                got, detail = _vote(cfg, lost)
                assert got == expected, 'losing 0x%02x flipped %s to %r (%s)' % (addr, expected, got, detail)

    """
    A WHOLE SEN0697 missing never changes the revision and never switches it off: it stays enabled and
    its setup fails loudly, on either map -- with or without a SEN0253 still answering. A scanned v1.0
    used to disable it there, and a two-module board with a loose SEN0697 connector armed on the backup.
    """
    for seen, expected in ((_TASTER, 'v1.1'), (_TMS7C, 'v0.1')):
        for remnant in (set(), _SEN0253):
            revision, cfg = _resolve({0: (seen[0] - _SEN0697) | remnant, 1: seen[1]})
            assert revision == expected, (expected, remnant, revision)
            for name in layout._PRIMARY:
                assert _enabled(cfg, name), 'a missing SEN0697 was switched off: %s' % name
            for name in layout._BACKUP:
                assert _enabled(cfg, name) is (layout._BACKUP[name] in remnant), name


def test_icp_votes_from_both_buses():
    """
    The ICP-10111 votes from EITHER bus: 0x63 on i2c:0 is evidence for the v0.1 map. Its old i2c:1-only
    rule guarded a second ICP on i2c:0 (retired) and cost 7C its third vote. A stray second ICP now adds
    one v0.1 vote beside the v1.x map's four -- which cannot flip it.
    """
    cfg = _cfg()
    got, detail = _vote(cfg, {0: {0x18, 0x63}, 1: set()})
    assert got == 'v0.1' and 'v0.1:1 v1.x:0' in detail, 'an ICP on i2c:0 must vote v0.1: %r (%s)' % (got, detail)
    for seen in (_TASTER, _LEGACY):
        got, detail = _vote(cfg, _with(seen, {0x63}))
        assert got == 'v1.1' and 'v0.1:1 v1.x:4' in detail, 'a second ICP broke the verdict: %r (%s)' % (got, detail)


def test_apply_fits_the_right_parts_in_both_directions():
    """
    apply() moves the four devices, sets the i2c:1 clock and the control pins, and fits/unfits the
    revision-dependent parts both ways. A DECLARED v0.1 / v1.1 (found=None) never touches the SEN0253's
    `enabled`; a declared v1.0 fits it, because there it IS the module.
    """
    cfg = config_default.default()
    assert layout.apply(cfg, 'v0.1') == [], 'the default IS the v0.1 layout: %s' % layout.apply(cfg, 'v0.1')
    assert all(_enabled(cfg, name) for name in layout._PRIMARY), 'the default expects the SEN0697'
    assert not any(_enabled(cfg, name) for name in layout._BACKUP), 'the default leaves the SEN0253 off'

    pins_before = dict(cfg['pins'])
    layout.apply(cfg, 'v1.0')
    assert _bus_id(cfg, 'baro_icp10111') == 1 and _bus_id(cfg, 'airspeed_sdp810') == 1
    assert _bus_id(cfg, 'laser_agl') == 1 and _bus_id(cfg, 'power_ina226') == 0
    assert config.bus(cfg, 'i2c', 1)['freq'] == 100000, 'the v1.x front bus runs at 100 kHz'
    assert not _enabled(cfg, 'accel_adxl375') and not _enabled(cfg, 'imu_bmi323'), 'v1.0: no ADXL, no SEN0697'
    assert _enabled(cfg, 'imu_bno055') and _enabled(cfg, 'baro_bmp280'), 'v1.0 IS the SEN0253: fitted'
    assert config.device(cfg, name='laser_agl')['int_pin'] is None, 'laser int_pin is freed on v1.x'
    assert cfg['pins'] == pins_before, 'no pin is renumbered between revisions'

    """
    A config written for v1.0 arrives with the SEN0697 off; apply() must switch it ON for v1.1 rather
    than leave a present, wired IMU ignored -- silent, and exactly what a bus scan cannot tell you later.
    """
    layout.apply(cfg, 'v1.1')
    for name in layout._PRIMARY:
        assert _enabled(cfg, name), 'v1.1 must FIT %s' % name
    assert config.device(cfg, name='imu_bmi323')['int_pin'] == 'accel_int1'

    # ...and back: v0.1 restores the map, keeps the SEN0697, refits the ADXL and takes GPIO4 off the BMI323
    layout.apply(cfg, 'v0.1')
    assert _bus_id(cfg, 'baro_icp10111') == 0 and _bus_id(cfg, 'power_ina226') == 1
    assert config.bus(cfg, 'i2c', 1)['freq'] == 400000
    assert _enabled(cfg, 'accel_adxl375') and _enabled(cfg, 'imu_bmi323')
    assert config.device(cfg, name='imu_bmi323')['int_pin'] is None
    assert layout.apply(cfg, 'v0.1') == [], 'apply must be a no-op when the config already matches'

    # DECLARED v0.1 / v1.1: the config's own `enabled` stands for the SEN0253, in both directions; a
    # declared v1.0 fits it either way, unless the operator unfitted a part
    for wanted in (False, True):
        for revision in layout._REVISIONS:
            declared = config_default.default()
            for name in layout._BACKUP:
                config.device(declared, name=name)['enabled'] = wanted
            layout.apply(declared, revision)
            for name in layout._BACKUP:
                assert _enabled(declared, name) is (wanted or revision == 'v1.0'), (revision, name, wanted)
    legacy = config_default.default()
    config.device(legacy, name='baro_bmp280')['fitted'] = False
    layout.apply(legacy, 'v1.0')
    assert _enabled(legacy, 'imu_bno055') and not _enabled(legacy, 'baro_bmp280'), 'fitted:false beats v1.0'


def test_missing_key_means_v01():
    """
    A config with no `layout` key resolves to v0.1 WITHOUT scanning: TMS-7C flies keyless profiles
    (make_telemetry_config.py), and a closed airframe's bus map must not depend on how many of its
    sensors happen to answer. v0.1 now means the SEN0697 is expected.
    """
    cfg = config_default.default()
    cfg['board'].pop('layout', None)
    scanned = []
    original = layout._scan
    layout._scan = lambda _c, bus_id: scanned.append(bus_id) or set()
    try:
        assert layout.resolve(cfg, log=lambda _line: None) == 'v0.1', 'no key must mean v0.1'
    finally:
        layout._scan = original
    assert scanned == [], 'a keyless config must not be scanned at all'
    assert _bus_id(cfg, 'baro_icp10111') == 0, 'and the v0.1 bus map applied'
    assert _enabled(cfg, 'imu_bmi323') and not _enabled(cfg, 'imu_bno055')


def test_resolve_declared_wins():
    """
    An explicit board.layout overrides the scan -- the escape hatch for an unfitted sensor. With no scan
    a declared v1.1 cannot enable a SEN0253 the hardware happens to carry, nor disable one the config
    enables; a declared v1.0 fits it, because it IS the SEN0253 build.
    """
    for declared in ('v1.0', 'v1.1'):
        cfg = _cfg()
        cfg['board']['layout'] = declared
        scanned = []
        seen = _with(_TMS7C, _SEN0253)  # the hardware looks like v0.1 with both modules...
        original = layout._scan
        layout._scan = (lambda _c, bus_id, _seen=seen, _scanned=scanned:  # bind per iteration
                        _scanned.append(bus_id) or _seen.get(bus_id, set()))
        try:
            assert layout.resolve(cfg, log=lambda _line: None) == declared, 'declared must beat the scan'
        finally:
            layout._scan = original
        assert scanned == [] and layout.RESOLVED == declared + ' (declared)'
        assert _bus_id(cfg, 'baro_icp10111') == 1, '...and the declared revision applied'
        assert _enabled(cfg, 'imu_bno055') is (declared == 'v1.0'), (
            'only a declared v1.0 enables the SEN0253 itself, got %s' % declared)

    # through resolve(): a declared v1.1 KEEPS a SEN0253 its config enables, though the scan never ran
    cfg = _cfg()
    cfg['board']['layout'] = 'v1.1'
    for name in layout._BACKUP:
        config.device(cfg, name=name)['enabled'] = True
    _revision, cfg = _resolve(_TASTER, cfg)
    assert all(_enabled(cfg, name) for name in layout._BACKUP), 'a declared layout dropped a configured backup'


def test_undecided_leaves_the_config_as_written():
    """
    UNDECIDED changes nothing -- and "as written" is the default, which still expects the SEN0697.
    """
    cfg = _cfg()
    revision, cfg = _resolve({0: {0x18} | _SEN0697, 1: set()}, cfg)
    assert revision is None and layout.RESOLVED == 'undecided'
    written = _cfg()
    assert cfg == written, 'an undecided scan rewrote the config'
    assert all(_enabled(cfg, name) for name in layout._PRIMARY)
    assert not any(_enabled(cfg, name) for name in layout._BACKUP)


def test_live_scan():
    """
    The real buses answer something recognisable on whatever board this is running on.

    Asserts a verdict exists, NOT which one: this file has to pass before and after a rewire, and pinning
    the revision here would turn a successful one into a red test.
    """
    cfg = _cfg()
    revision, detail = layout.detect(cfg)
    assert revision in layout._REVISIONS, 'live scan did not recognise this board: %s' % detail
    print('   live scan -> %s (%s)' % (revision, detail))


def _clock(i2c) -> int:
    """The frequency an I2C peripheral reports in its repr, e.g. I2C(0, scl=8, sda=7, freq=400000)."""
    text = repr(i2c)
    start = text.index('freq=') + 5
    end = start
    while end < len(text) and text[end].isdigit():
        end += 1
    return int(text[start:end])


def test_laser_pins_and_operator_unfit():
    """
    The laser INT/XSHUT are freed on EVERY revision of the map that does not route them. Only literal
    'v1.0' did, so on v1.1 the L1X armed an IRQ on unrouted GPIO3 and toggled GPIO5. And `fitted: false`
    is the operator's word that a revision part is dead: apply() used to re-enable it, undoing
    `enabled: false` and leaving verify/arm refusing on a part nobody could remove from the config.
    """
    for revision in ('v1.0', 'v1.1'):
        cfg = config_default.default()
        layout.apply(cfg, revision)
        for name in ('laser_agl', 'laser_agl_l1x'):
            laser = config.device(cfg, name=name)
            assert laser.get('int_pin') is None and laser.get('xshut_pin') is None, (revision, name, laser)
    kept = config_default.default()
    layout.apply(kept, 'v0.1')  # v0.1 routes them: kept
    assert config.device(kept, name='laser_agl').get('int_pin') is not None

    cfg = config_default.default()
    config.device(cfg, name='baro_bmp581')['fitted'] = False  # the operator: this one is dead
    config.device(cfg, name='baro_bmp581')['enabled'] = False
    layout.apply(cfg, 'v1.1')  # v1.1 carries a BMP581, so it would be re-enabled
    assert not _enabled(cfg, 'baro_bmp581'), 'fitted:false was overridden'
    assert _enabled(cfg, 'imu_bmi323')  # its neighbours still fit


def test_runtime_scan_restores_the_live_clock():
    """
    `detect` at runtime must leave the drivers' bus at ITS clock. machine.I2C(id) is one peripheral per
    id, so the 100 kHz scan re-clocked the bus every driver shares -- until reboot, silently, and a
    pre-flight detect flew i2c:0 at a quarter speed.
    """
    cfg = _cfg()
    revision, _detail = layout.detect(cfg)
    layout.apply(cfg, revision or 'v1.0')
    for bus_id in (0, 1):
        spec = config.bus(cfg, 'i2c', bus_id)
        configured = spec.get('freq', 400000)
        if configured == layout._SCAN_HZ:
            continue  # a bus that already runs at the scan clock cannot show the difference
        bus = i2cbus.get(bus_id, spec)  # the drivers' shared Bus, as a running board has it
        before = _clock(bus._i2c)
        layout.detect(cfg)  # the runtime `detect` command
        after = _clock(bus._i2c)
        assert after == before and after > layout._SCAN_HZ, (
            'i2c:%d left at %d Hz after a runtime scan (configured %d)' % (bus_id, after, configured))


def test_fitted_answers_every_revision_and_the_undecided_case():
    """
    fitted() is what callers must ask INSTEAD of writing a revision literal, so it has to be right for
    all three revisions and honest when there is no verdict.
    """
    # the ADXL375 is on the v0.1 map only
    assert layout.fitted('accel_adxl375', 'v0.1')
    assert not layout.fitted('accel_adxl375', 'v1.0')
    assert not layout.fitted('accel_adxl375', 'v1.1'), 'the literal-pair bug this helper replaces'

    # the SEN0697 is the primary everywhere but legacy v1.0 -- on the v0.1 map too, since 2026-10-05
    for name in layout._PRIMARY:
        assert layout.fitted(name, 'v0.1') and layout.fitted(name, 'v1.1'), name
        assert not layout.fitted(name, 'v1.0'), name

    # the SEN0253 is v1.0's module (declared only) and no other revision's: elsewhere only a scan finds it
    for name in layout._BACKUP:
        assert layout.fitted(name, 'v1.0'), name
        for revision in ('v0.1', 'v1.1', 'undecided'):
            assert not layout.fitted(name, revision), (name, revision)

    # a device no revision removes is fitted everywhere, including on an unrecognised board
    for revision in ('v0.1', 'v1.0', 'v1.1', 'undecided'):
        assert layout.fitted('laser_ground', revision), revision

    # UNDECIDED: unknown must not answer "fitted" for a part some revision removes
    assert not layout.fitted('accel_adxl375', 'undecided')
    assert not layout.fitted('imu_bmi323', 'undecided')


test_real_scans()
test_both_modules_primary_and_backup()
test_backup_ranks_below_whatever_the_config_says()
test_placement_alone_decides_the_map()
test_votes()
test_one_dead_device_never_flips_a_verdict()
test_icp_votes_from_both_buses()
test_apply_fits_the_right_parts_in_both_directions()
test_missing_key_means_v01()
test_resolve_declared_wins()
test_undecided_leaves_the_config_as_written()
test_fitted_answers_every_revision_and_the_undecided_case()
test_live_scan()
test_runtime_scan_restores_the_live_clock()
test_laser_pins_and_operator_unfit()
print('ok: layout -- real scans (7C v0.1 on 3 votes, taster v1.1, legacy scans v1.1), both modules (SEN0697 '
      'primary, SEN0253 backup), backup ranked below in code (old profiles, declared, undecided; ADXL375 '
      'below BMI323), placement alone decides the map, ICP votes from both buses, a missing SEN0697 never '
      'switched off, dead-device tolerance, apply both ways, declared override (v1.0 fits the SEN0253), '
      'undecided as written, fitted() incl. undecided, live scan, live clock kept')
