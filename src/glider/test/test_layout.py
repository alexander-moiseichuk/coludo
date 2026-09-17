"""
layout.py: the v0.1 / v1.0 / v1.1 revision vote, and what it rewrites.

Runs on the board and exercises the pure decision logic against synthetic scans, so every verdict is
checked without needing all three physical revisions to exist. The live end -- detect() against the real
buses -- is the last case, which asserts the board this runs on is recognised at all rather than which
revision, so the file does not go red the moment the breadboard is rewired.
"""

import config
import config_default
import layout


def _cfg() -> dict:
    """A fresh default config to mutate per case."""
    return config_default.default()


def _seen(revision: str) -> dict:
    """The scan a healthy board of `revision` would produce: anchor, moved devices, fitted parts."""
    found = {0: {0x18}, 1: set()}  # the ES8311 is soldered to the board -- present on every revision
    for name in layout._MOVED:
        entry = layout._MOVED[name]
        found[entry[revision]].add(entry['addr'])
    for name in layout._FITTED:
        entry = layout._FITTED[name]
        if revision in entry['in']:
            found[0].add(entry['addr'])
    return found


def _vote(cfg: dict, seen: dict) -> tuple:
    """Run detect() against a synthetic scan by swapping _scan out."""
    original = layout._scan
    layout._scan = lambda _cfg, bus_id: seen.get(bus_id, set())
    try:
        return layout.detect(cfg)
    finally:
        layout._scan = original


def test_votes():
    """Every clean revision is recognised, and evidence alone is never enough to guess."""
    cfg = _cfg()
    for revision in layout._REVISIONS:
        got, detail = _vote(cfg, _seen(revision))
        assert got == revision, 'clean %s scan read as %r (%s)' % (revision, got, detail)

    """
    NEGATIVE, and the one that matters most: an EMPTY or ANCHOR-ONLY scan must decide NOTHING. Guessing
    re-buses every moving sensor at once and can swap the whole attitude module, which is worse than
    running the config as written -- so "undecided" has to stay a real outcome. The anchor in particular
    proves only that the bus is alive; it is on every revision and must never cast a vote.
    """
    got, _ = _vote(cfg, {0: set(), 1: set()})
    assert got is None, 'an empty scan must not pick a revision'
    got, _ = _vote(cfg, {0: {0x18}, 1: set()})
    assert got is None, 'the anchor alone must not pick a revision'

    # a scan with no anchor at all is not trustworthy even when the rest looks like a revision
    got, _ = _vote(cfg, {0: {0x63, 0x25, 0x29}, 1: {0x40}})
    assert got is None, 'no anchor on i2c:0 -> the scan is not trustworthy'


def test_one_dead_device_never_flips_a_verdict():
    """
    Losing any SINGLE address must leave the verdict standing.

    That is the whole reason the tally is spread over four moving devices plus the fitted parts rather
    than hinging on one address: a dead or unfitted part costs its revision a vote instead of casting one
    for another. v1.0 and v1.1 are the pair most at risk, since they differ only in the attitude module.
    """
    cfg = _cfg()
    for revision in layout._REVISIONS:
        for table in (layout._MOVED, layout._FITTED):
            for name in table:
                seen = _seen(revision)
                addr = table[name]['addr']
                seen[0].discard(addr)
                seen[1].discard(addr)
                got, detail = _vote(cfg, seen)
                assert got == revision, 'losing %s flipped %s to %r (%s)' % (name, revision, got, detail)


def test_the_attitude_module_is_what_separates_v10_from_v11():
    """
    v1.0 and v1.1 have IDENTICAL bus layouts; only the attitude module differs.

    So the moving devices tie between them by construction, and the SEN0253/SEN0697 addresses break it.
    Lose the whole module and the tie must stand as undecided rather than resolve by luck.
    """
    cfg = _cfg()
    bare = _seen('v1.0')
    for name in layout._FITTED:
        bare[0].discard(layout._FITTED[name]['addr'])
    got, detail = _vote(cfg, bare)
    assert got is None, 'with no attitude module, v1.0 vs v1.1 must be undecided, got %r (%s)' % (got, detail)

    # and one part of either module is enough to break it, in both directions
    for revision, name in (('v1.0', 'imu_bno055'), ('v1.1', 'imu_bmi323')):
        seen = {0: set(bare[0]), 1: set(bare[1])}
        seen[0].add(layout._FITTED[name]['addr'])
        got, detail = _vote(cfg, seen)
        assert got == revision, '%s alone should say %s, got %r (%s)' % (name, revision, got, detail)


def test_second_icp_does_not_confuse_the_vote():
    """
    A SECOND ICP-10111 on i2c:0 must not break detection.

    Fitting one per bus is a planned v2 option and puts 0x63 on BOTH buses. Treated symmetrically that
    address would vote for v0.1 and the v1.x pair at once and cancel itself; marked `only` it stays
    evidence for the revisions it can still prove.
    """
    cfg = _cfg()
    for revision in ('v1.0', 'v1.1'):
        seen = _seen(revision)
        seen[0].add(0x63)  # the second ICP, on the on-board bus
        got, detail = _vote(cfg, seen)
        assert got == revision, 'a second ICP broke the %s verdict: %r (%s)' % (revision, got, detail)
    got, detail = _vote(cfg, _seen('v0.1'))
    assert got == 'v0.1', 'v0.1 misread as %r (%s)' % (got, detail)


def test_apply_fits_the_right_parts_in_both_directions():
    """apply() moves the four devices, sets the i2c:1 clock, and fits/unfits the revision-dependent parts."""
    cfg = _cfg()
    pins_before = dict(cfg['pins'])
    layout.apply(cfg, 'v1.0')
    assert config.device(cfg, name='baro_icp10111')['id'] == 1, 'icp10111 must move to i2c:1'
    assert config.device(cfg, name='airspeed_sdp810')['id'] == 1, 'sdp810 must move to i2c:1'
    assert config.device(cfg, name='laser_agl')['id'] == 1, 'laser must move to i2c:1'
    assert config.device(cfg, name='power_ina226')['id'] == 0, 'ina226 must move to i2c:0'
    assert config.bus(cfg, 'i2c', 1)['freq'] == 100000, 'the v1.x front bus runs at 100 kHz'
    assert config.device(cfg, name='accel_adxl375')['enabled'] is False, 'adxl375 is not fitted on v1.0'
    assert config.device(cfg, name='imu_bno055')['enabled'] is True, 'v1.0 keeps the SEN0253'
    assert config.device(cfg, name='imu_bmi323')['enabled'] is False, 'v1.0 has no SEN0697'
    assert config.device(cfg, name='laser_agl')['int_pin'] is None, 'laser int_pin is freed on v1.x'
    assert cfg['pins'] == pins_before, 'no pin is renumbered between revisions'

    """
    v1.1 SWAPS the attitude module, so apply() has to enable as well as disable. A config written for
    v1.0 arrives with the SEN0697 off; leaving it alone would bring up a board whose IMU is present,
    wired and ignored -- silent, and exactly the kind of thing a bus scan cannot tell you afterwards.
    """
    layout.apply(cfg, 'v1.1')
    assert config.device(cfg, name='imu_bmi323')['enabled'] is True, 'v1.1 must FIT the SEN0697 IMU'
    assert config.device(cfg, name='baro_bmp581')['enabled'] is True, 'v1.1 must FIT the SEN0697 baro'
    assert config.device(cfg, name='imu_bno055')['enabled'] is False, 'v1.1 has no SEN0253'
    assert config.device(cfg, name='baro_bmp280')['enabled'] is False, 'v1.1 has no SEN0253'

    # ...and back: v0.1 restores every one of them
    layout.apply(cfg, 'v0.1')
    assert config.device(cfg, name='baro_icp10111')['id'] == 0
    assert config.device(cfg, name='power_ina226')['id'] == 1
    assert config.bus(cfg, 'i2c', 1)['freq'] == 400000
    assert config.device(cfg, name='accel_adxl375')['enabled'] is True, 'v0.1 carries the ADXL375'
    assert config.device(cfg, name='imu_bno055')['enabled'] is True, 'v0.1 carries the SEN0253'
    assert config.device(cfg, name='imu_bmi323')['enabled'] is False, 'v0.1 has no SEN0697'
    assert layout.apply(cfg, 'v0.1') == [], 'apply must be a no-op when the config already matches'


def test_missing_key_means_v01():
    """
    A config with no `layout` key resolves to v0.1 WITHOUT scanning.

    TMS-7C and TMS-7D are built, packed and flying profiles written before this field existed, and
    config.load() replaces rather than merges, so those configs arrive with no key at all. They must not
    fall through to a scan: a closed airframe's bus map should not depend on how many of its sensors
    happen to answer.
    """
    cfg = _cfg()
    cfg['board'].pop('layout', None)
    scanned = []
    original = layout._scan
    layout._scan = lambda _c, bus_id: scanned.append(bus_id) or set()
    try:
        assert layout.resolve(cfg, log=lambda _line: None) == 'v0.1', 'no key must mean v0.1'
    finally:
        layout._scan = original
    assert scanned == [], 'a keyless config must not be scanned at all'
    assert config.device(cfg, name='baro_icp10111')['id'] == 0, 'and the v0.1 bus map applied'


def test_resolve_declared_wins():
    """An explicit board.layout overrides the scan -- the escape hatch for an unfitted sensor."""
    for declared in ('v1.0', 'v1.1'):
        cfg = _cfg()
        cfg['board']['layout'] = declared
        seen = _seen('v0.1')  # the hardware looks like v0.1...
        original = layout._scan
        layout._scan = lambda _c, bus_id, _seen=seen: _seen.get(bus_id, set())  # bind per iteration
        try:
            assert layout.resolve(cfg, log=lambda _line: None) == declared, 'declared must beat the scan'
        finally:
            layout._scan = original
        assert config.device(cfg, name='baro_icp10111')['id'] == 1, '...and the declared revision applied'


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


def test_fitted_answers_every_revision_and_the_undecided_case():
    """
    fitted() is what callers must ask INSTEAD of writing a revision literal, so it has to be right for
    all three revisions and honest when there is no verdict.

    The bug it exists to prevent, twice over: `revision != 'v1.0'` reads as "the ADXL is on v0.1" and
    silently became "the ADXL is on v0.1 AND v1.1" the day a third revision appeared -- a test then
    demanded a part from a board that has none, and reported it as a hardware fault.
    """
    # the ADXL375 is a v0.1 part only
    assert layout.fitted('accel_adxl375', 'v0.1')
    assert not layout.fitted('accel_adxl375', 'v1.0')
    assert not layout.fitted('accel_adxl375', 'v1.1'), 'the literal-pair bug this helper replaces'

    # the two attitude modules are mutually exclusive, and each is fitted on exactly its revisions
    assert layout.fitted('imu_bno055', 'v0.1') and layout.fitted('imu_bno055', 'v1.0')
    assert not layout.fitted('imu_bno055', 'v1.1')
    assert layout.fitted('imu_bmi323', 'v1.1') and layout.fitted('mag_bmm350', 'v1.1')
    assert not layout.fitted('imu_bmi323', 'v1.0') and not layout.fitted('baro_bmp581', 'v0.1')

    # a device no revision removes is fitted everywhere, including on an unrecognised board
    for revision in ('v0.1', 'v1.0', 'v1.1', 'undecided'):
        assert layout.fitted('laser_ground', revision), revision

    """
    UNDECIDED: unknown must not answer "fitted" for a part some revision removes. A test that demands
    the ADXL because the scan was inconclusive blames the hardware for a bad scan.
    """
    assert not layout.fitted('accel_adxl375', 'undecided')
    assert not layout.fitted('imu_bmi323', 'undecided')


test_votes()
test_one_dead_device_never_flips_a_verdict()
test_the_attitude_module_is_what_separates_v10_from_v11()
test_second_icp_does_not_confuse_the_vote()
test_apply_fits_the_right_parts_in_both_directions()
test_missing_key_means_v01()
test_resolve_declared_wins()
test_fitted_answers_every_revision_and_the_undecided_case()
test_live_scan()
print('ok: layout -- three-revision vote, dead-device tolerance, attitude-module split, apply both ways, '
      'declared override, fitted() incl. undecided, live scan')
