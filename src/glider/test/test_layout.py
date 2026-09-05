"""
layout.py: the v0.1 / v1.0 revision vote, and what it rewrites.

Runs on the board and exercises the pure decision logic against synthetic scans, so the verdicts are
checked without needing both physical revisions to exist. The live end -- detect() against the real
buses -- is covered by the last case, which asserts the board this runs on is recognised at all rather
than asserting WHICH revision, so the test does not go red the moment the breadboard is rewired.
"""

import config
import config_default
import layout


def _cfg() -> dict:
    """A fresh default config to mutate per case."""
    return config_default.default()


def _seen(revision: str) -> dict:
    """The scan a healthy board of `revision` would produce (anchors + every moved device)."""
    found = {0: {0x28, 0x76}, 1: set()}
    for name in layout._MOVED:
        entry = layout._MOVED[name]
        found[entry[revision]].add(entry['addr'])
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
    """A clean board of either revision is recognised; the anchors alone are not enough to decide."""
    cfg = _cfg()
    for revision in ('v0.1', 'v1.0'):
        got, detail = _vote(cfg, _seen(revision))
        assert got == revision, 'clean %s scan read as %r (%s)' % (revision, got, detail)

    """
    NEGATIVE, and the one that matters most: an EMPTY or anchor-only scan must decide NOTHING. Guessing
    here re-buses every moving sensor at once, which is a worse failure than running the config as
    written -- so "undecided" has to be a real outcome, not a fallback to a default revision.
    """
    got, _ = _vote(cfg, {0: set(), 1: set()})
    assert got is None, 'an empty scan must not pick a revision'
    got, _ = _vote(cfg, {0: {0x28, 0x76}, 1: set()})
    assert got is None, 'anchors with no moving device must not pick a revision'

    # a scan with no anchor is not trustworthy even when the moving devices look like a revision
    got, _ = _vote(cfg, {0: {0x63, 0x25, 0x29}, 1: {0x40}})
    assert got is None, 'no anchor on i2c:0 -> the scan is not trustworthy'

    """
    A single DEAD moving device must not flip the verdict -- that is the whole reason the vote is over
    four addresses rather than one. Drop each in turn from a v1.0 board and the answer must hold.
    """
    for name in layout._MOVED:
        seen = _seen('v1.0')
        seen[layout._MOVED[name]['v1.0']].discard(layout._MOVED[name]['addr'])
        got, detail = _vote(cfg, seen)
        assert got == 'v1.0', 'losing %s flipped the verdict to %r (%s)' % (name, got, detail)


def test_apply():
    """apply() moves exactly the four devices, sets the i2c:1 clock, and leaves pins alone."""
    cfg = _cfg()
    pins_before = dict(cfg['pins'])
    layout.apply(cfg, 'v1.0')
    assert config.device(cfg, name='baro_icp10111')['id'] == 1, 'icp10111 must move to i2c:1'
    assert config.device(cfg, name='airspeed_sdp810')['id'] == 1, 'sdp810 must move to i2c:1'
    assert config.device(cfg, name='laser_agl')['id'] == 1, 'laser must move to i2c:1'
    assert config.device(cfg, name='power_ina226')['id'] == 0, 'ina226 must move to i2c:0'
    assert config.bus(cfg, 'i2c', 1)['freq'] == 100000, 'the v1.0 front bus runs at 100 kHz'
    assert config.device(cfg, name='accel_adxl375')['enabled'] is False, 'adxl375 is not fitted on v1.0'
    assert config.device(cfg, name='laser_agl')['int_pin'] is None, 'laser int_pin is freed on v1.0'
    assert cfg['pins'] == pins_before, 'no pin is renumbered between revisions'

    # ...and back: applying v0.1 to the same dict restores every one of them
    layout.apply(cfg, 'v0.1')
    assert config.device(cfg, name='baro_icp10111')['id'] == 0
    assert config.device(cfg, name='power_ina226')['id'] == 1
    assert config.bus(cfg, 'i2c', 1)['freq'] == 400000
    # idempotent: a second apply of the same revision reports no changes
    assert layout.apply(cfg, 'v0.1') == [], 'apply must be a no-op when the config already matches'


def test_missing_key_means_v01():
    """
    A config with no `layout` key resolves to v0.1 WITHOUT scanning.

    TMS-7C and TMS-7D are built, packed and flying profiles written before this field existed, and
    config.load() replaces rather than merges, so those configs arrive with no key at all. They must
    not fall through to a scan: 7C declares `power_ina226` absent, so it can cast only three votes, and a
    closed airframe's bus map should not depend on how many of its sensors happen to answer.
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
    cfg = _cfg()
    cfg['board']['layout'] = 'v1.0'
    seen = _seen('v0.1')  # the hardware looks like v0.1...
    original = layout._scan
    layout._scan = lambda _c, bus_id: seen.get(bus_id, set())
    try:
        assert layout.resolve(cfg, log=lambda _line: None) == 'v1.0', 'declared must beat the scan'
    finally:
        layout._scan = original
    assert config.device(cfg, name='baro_icp10111')['id'] == 1, '...and the declared revision applied'


def test_live_scan():
    """
    The real buses answer something recognisable on whatever board this is running on.

    Asserts a verdict exists, NOT which one: this file has to pass both before and after the breadboard
    is rewired, and pinning the revision here would turn a successful rewire into a red test.
    """
    cfg = _cfg()
    revision, detail = layout.detect(cfg)
    assert revision in ('v0.1', 'v1.0'), 'live scan did not recognise this board: %s' % detail
    print('   live scan -> %s (%s)' % (revision, detail))


test_votes()
test_apply()
test_missing_key_means_v01()
test_resolve_declared_wins()
test_live_scan()
print('ok: layout -- revision vote (+ dead-device tolerance), apply/restore, declared override, live scan')
