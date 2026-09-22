"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the board config loader/validator (config.py), new schema: nested
buses (uart/i2c/spi -> id), `sensors` + `components` with 'type:id' bus refs. Run by `make test`.
"""

import json
import os

import config
import config_default


def main():
    # the default config validates clean
    assert config.validate(config_default.default()) == [], config.validate(config_default.default())

    # config_id is stable and sensitive
    a, b = config_default.default(), config_default.default()
    assert config.config_id(a) == config.config_id(b)
    b['board']['rev'] = 99
    assert config.config_id(a) != config.config_id(b)
    assert isinstance(config.config_id(a), str) and len(config.config_id(a)) >= 8

    """
    A bus missing its DRIVING pins must fail validation, not fail at bring-up.

    `{i2c: {9: {freq: 400000}}}` used to validate cleanly -- validation claimed only the pins that
    happened to be PRESENT -- and then raised KeyError at Pin(spec['scl']) during bring-up, losing the
    whole bus and every device on it. validate() exists to catch that before a config is saved and
    flown, so passing it WAS the failure.
    """
    for kind, needed in (('i2c', ('scl', 'sda')), ('spi', ('sck', 'mosi', 'miso')), ('uart', ('tx',))):
        broken = config_default.default()
        broken['buses'][kind]['9'] = {'freq': 400000, 'baud': 9600}
        errs = config.validate(broken)
        for key in needed:
            assert any(key in e and '9' in e for e in errs), '%s without %r validated: %r' % (kind, key, errs)

    # NEGATIVE: uart with tx and NO rx must still pass -- that is the shipped recorder link, tx-only
    tx_only = config_default.default()
    tx_only['buses']['uart']['9'] = {'tx': 44, 'baud': 9600}
    assert not [e for e in config.validate(tx_only) if 'uart:9' in e], config.validate(tx_only)

    """
    Fields the CONTROL PATH does arithmetic on must be numbers, checked while a human is still present.

    A JSON config carries any type, and a string that looks like a number survives save() and load()
    untouched -- then TypeErrors at 100 Hz, in flight, inside the governor or a servo write. `validate()`
    is the last moment anyone can see it.

    fins.limit_multiplier is checked separately because it is a TOP-LEVEL section, not a component
    field: it reaches the governor via flight.py's `board.get('fins', {})`, so a component-only sweep
    misses exactly the field that multiplies the fin cap.
    """
    def _find(cfg, name):
        return [item for item in cfg['components'] if item['name'] == name][0]

    def _find_sensor(cfg, name):
        return [item for item in cfg['sensors'] if item['name'] == name][0]

    for mutate, needle in (
            (lambda c: _find(c, 'flight').update({'bank_limit': '45'}), 'bank_limit'),
            (lambda c: _find(c, 'servo_yaw').update({'trim': '2.5'}), 'trim'),
            (lambda c: _find(c, 'flight').setdefault('gains', {})
                .setdefault('roll', {}).update({'kp': '0.8'}), 'gains.roll.kp'),
            (lambda c: c.setdefault('fins', {}).update({'limit_multiplier': '1.0'}), 'fins.limit_multiplier')):
        broken = config_default.default()
        mutate(broken)
        errs = config.validate(broken)
        assert any(needle in e and 'must be a number' in e for e in errs), '%s not caught: %r' % (needle, errs)

    """
    Device TIMING fields, and the recorder's own rate. Both were missed by the first numeric pass.

    period_ms goes straight into asyncio.sleep_ms(); telemetry_ms is multiplied by 1000 for
    decimate_us -- and in Python `"20" * 1000` is a valid 2000-character STRING, so it constructs
    without error and only fails later at a comparison, far from the config that caused it.

    recorder.telemetry_ms is checked separately because ZERO is legal there and is the shipped value
    ("no decimation"). Folding it into the positive-int loop would have rejected the config that flies,
    which is why the test pins 0 as accepted alongside the rejections.
    """
    for mutate, needle in (
            (lambda c: _find_sensor(c, 'baro_icp10111').update({'period_ms': '100'}), 'period_ms'),
            (lambda c: _find_sensor(c, 'accel_adxl375').update({'telemetry_ms': '20'}), 'telemetry_ms')):
        broken = config_default.default()
        mutate(broken)
        assert any(needle in e and 'must be a number' in e for e in config.validate(broken)), needle
    for bad in ('500', -1, True):
        broken = config_default.default()
        broken['recorder']['telemetry_ms'] = bad
        assert any('recorder.telemetry_ms' in e for e in config.validate(broken)), bad
    zeroed = config_default.default()
    zeroed['recorder']['telemetry_ms'] = 0          # the SHIPPED value: no decimation
    assert not [e for e in config.validate(zeroed) if 'recorder.telemetry_ms' in e]

    # NEGATIVE: bool is an int subclass -- True must NOT pass as a number for a gain or a multiplier
    truthy = config_default.default()
    truthy.setdefault('fins', {})['limit_multiplier'] = True
    assert any('limit_multiplier' in e for e in config.validate(truthy))
    # ...and a legitimate float must still pass
    good = config_default.default()
    good.setdefault('fins', {})['limit_multiplier'] = 0.5
    assert not [e for e in config.validate(good) if 'limit_multiplier' in e]

    # pin uniqueness across nested buses + pins
    dup = config_default.default()
    dup['pins']['servo_yaw'] = dup['buses']['i2c']['0']['sda']  # collide with GPIO7
    assert any('used by both' in e for e in config.validate(dup))

    # reserved pin (GPIO18 is a C6 Wi-Fi line)
    res = config_default.default()
    res['pins']['servo_yaw'] = 18
    assert any('reserved GPIO18' in e for e in config.validate(res))

    # DISABLED optional pins: null (kept as a placeholder) or ANY negative -> feature wired off,
    # not a GPIO claim -> validates clean; a real collision is still caught alongside them
    off = config_default.default()
    off['pins']['laser_xshut'] = None  # placeholder kept, feature off
    off['pins']['laser_int'] = -1
    off['pins']['accel_int1'] = -7    # any negative, not just -1
    assert config.validate(off) == [], config.validate(off)
    off['pins']['servo_yaw'] = off['buses']['i2c']['0']['sda']  # a genuine collide still flagged
    assert any('used by both' in e for e in config.validate(off))

    # unknown bus reference on a sensor (a valid kind, but an id with no defined bus)
    badref = config_default.default()
    badref['sensors'][0]['id'] = 9  # i2c:9 is not defined
    assert any('is not defined' in e for e in config.validate(badref))

    # a device naming a bus must give an int id
    badid = config_default.default()
    badid['sensors'][0]['id'] = 'x'
    assert any('.id must be the int bus id' in e for e in config.validate(badid))

    # bad bus type
    badtype = config_default.default()
    badtype['buses']['oops'] = {'0': {'tx': 99, 'rx': 98}}
    assert any('not one of uart/i2c/spi' in e for e in config.validate(badtype))

    # SPI mode must be 0..3 (machine.SPI polarity/phase are each 0/1)
    badmode = config_default.default()
    badmode['buses']['spi']['1']['mode'] = 4
    assert any('.mode must be 0..3' in e for e in config.validate(badmode))

    # board.id must be a bare wire token (no spaces)
    spaced = config_default.default()
    spaced['board']['id'] = 'glider 1'
    assert any('must not contain whitespace' in e for e in config.validate(spaced))

    # a component must name its implementation: `driver` (drivers/) or `activity` (tasks/)
    noimpl = config_default.default()
    del noimpl['components'][0]['activity']  # the recorder component
    assert any('driver` (drivers/) or `activity`' in e for e in config.validate(noimpl))

    # bus() / device() helpers -- addressed by (kind, id), no string parsing
    cfg = config_default.default()
    assert config.bus(cfg, 'i2c', 0) == {'sda': 7, 'scl': 8, 'freq': 400000}
    assert config.bus(cfg, 'uart', 2)['baud'] == 9600
    assert config.bus(cfg, 'i2c', 9) is None  # undefined id
    assert config.bus(cfg, 'nope', 0) is None  # undefined kind
    assert config.device(cfg, driver='recorder')['name'] == 'recorder'
    gnss = config.device(cfg, name='gnss')
    assert gnss['bus'] == 'uart' and gnss['id'] == 2
    assert config.device(cfg, name='absent') is None

    # save / load round-trip on the board filesystem
    path = 'test_board.config'
    config.reset(path)
    cid = config.save(config_default.default(), path)
    cfg, source, errs = config.load(path, defaults=config_default.default())
    assert source == 'active' and cfg == config_default.default() and config.config_id(cfg) == cid

    # corrupt file -> fallback to defaults (never crash)
    f = open(path, 'w')
    f.write('{ not json ')
    f.close()
    cfg, source, errs = config.load(path, defaults=config_default.default())
    assert cfg == config_default.default() and 'fallback' in source

    # invalid config is never written
    bad = config_default.default()
    bad['pins']['servo_yaw'] = bad['buses']['i2c']['0']['scl']
    raised = False
    try:
        config.save(bad, path)
    except ValueError:
        raised = True
    assert raised

    # reset removes the active file
    assert config.reset(path) is True
    cfg, source, errs = config.load(path, defaults=config_default.default())
    assert source == 'default'

    """
    Config SCHEMA VERSION (findings §27.13): a saved config carries the version it was produced from,
    forever. The saved config still WINS as-is -- what you saved is what flies -- but a mismatch against
    the firmware is reported through `source`, so a config predating a new sensor is visible instead of
    silently dropping it.
    """
    fresh = config_default.default()
    assert fresh['version'] == config_default.CONFIG_VERSION and config.outdated(fresh, fresh) is None
    assert config.schema_version(fresh) == config_default.CONFIG_VERSION
    assert config.schema_version(None) == '' and config.schema_version({}) == ''
    assert config.outdated({'version': '19700101'}, fresh) == ('19700101', config_default.CONFIG_VERSION)
    assert config.outdated({}, fresh) == ('unversioned', config_default.CONFIG_VERSION)  # predates versioning
    # an OLD saved config still loads and still wins -- only the source string flags it
    stale = config_default.default()
    stale['version'] = '19700101'
    stale['board']['id'] = 'stale-board'
    config.save(stale, path)
    cfg, source, errs = config.load(path, defaults=fresh)
    assert not errs and cfg['board']['id'] == 'stale-board'  # the SAVED config ran (bar firmware_version)
    assert source.startswith('active(config 19700101, firmware ') and 're-save' in source, source
    config.reset(path)

    """
    firmware_version reports the RUNNING firmware, never the one that saved the file.

    It is the single field load() overrides, because it is not configuration: a saved board.config
    freezes whatever build wrote it, so a freshly deployed board would announce the build it replaced
    -- which is exactly what TMS-7D did, reporting an old commit through CC minutes after a clean
    deploy of a newer one. Negative half matters just as much: restamping must touch NOTHING else, or
    "what you saved is what flies" quietly stops being true.
    """
    running = config_default.default()
    running['board']['firmware_version'] = '2026.09.06.running'
    written = config_default.default()
    written['board']['firmware_version'] = '2026.08.31.whenSaved'
    written['board']['id'] = 'restamp-board'
    written['board']['setup_retries'] = 7
    config.save(written, path)
    cfg, source, errs = config.load(path, defaults=running)
    assert not errs and source == 'active', source
    assert cfg['board']['firmware_version'] == '2026.09.06.running', cfg['board']['firmware_version']
    # ...and every other saved field survived untouched
    assert cfg['board']['id'] == 'restamp-board' and cfg['board']['setup_retries'] == 7
    expected = dict(written['board'], firmware_version='2026.09.06.running')
    assert cfg['board'] == expected, cfg['board']
    # the restamp self-heals: saving the loaded config persists the running version
    config.save(cfg, path)
    again, _source, _errs = config.load(path, defaults=running)
    assert again['board']['firmware_version'] == '2026.09.06.running'

    """
    config_id identifies the CONFIGURATION, not the build.

    The restamp above puts the running firmware into the loaded config, and config_id hashes that dict --
    so unless firmware_version is excluded, a byte-identical board.config reports a different id after
    every deploy, and the id save() returned stops matching the one the board announces in `iam`.
    """
    build_a = config_default.default()
    build_a['board']['firmware_version'] = '2026.01.01.aaaaaaaaaaaa'
    build_b = config_default.default()
    build_b['board']['firmware_version'] = '2026.12.31.bbbbbbbbbbbb'
    assert config.config_id(build_a) == config.config_id(build_b), 'firmware_version must not move the id'
    # ...while a REAL configuration change still must
    build_b['board']['setup_retries'] = 9
    assert config.config_id(build_a) != config.config_id(build_b), 'a config change must move the id'
    # and the whole point: the id save() hands back survives load()'s restamp
    config.reset(path)
    saved_id = config.save(build_a, path)
    reloaded, _source, _errs = config.load(path, defaults=running)
    assert config.config_id(reloaded) == saved_id, 'the saved id must survive the restamp'

    # NEGATIVE: a config whose board section is missing or not a dict must not crash the loader
    config.reset(path)
    assert config.load(path, defaults=running)[1] == 'default'  # no file at all
    config.reset(path)

    """
    An ENABLED watchdog below the measured boot floor is refused: 1000 ms boot-loops the board every
    ~8.5 s, and with servos enabled each boot re-centres every fin (how a servo died). The flight value
    passes, and so does a disabled watchdog whatever it says -- it never arms.
    """
    """
    A malformed section is an ERROR, never a raise: `fins` as a list made validate() raise
    AttributeError, and load() -- which must never raise, it runs before the watchdog and WiFi exist --
    raised with it. It must refuse the file and fall back.
    """
    broken = config_default.default()
    broken['fins'] = [1, 2]
    assert 'fins is not an object' in config.validate(broken), config.validate(broken)
    with open('test_malformed.config', 'w') as handle:
        handle.write(json.dumps(broken))
    try:
        loaded, source, errors = config.load('test_malformed.config')
        assert source.startswith('default(fallback'), source
        assert loaded['board']['id'] == config_default.default()['board']['id']
    finally:
        os.remove('test_malformed.config')

    floor_cfg = config_default.default()
    watchdog = [device for device in floor_cfg['components'] if device.get('activity') == 'watchdog'][0]
    watchdog['enabled'], watchdog['wdt_timeout_ms'] = True, 5000
    assert not [e for e in config.validate(floor_cfg) if 'boot floor' in e]
    watchdog['wdt_timeout_ms'] = 1000
    assert [e for e in config.validate(floor_cfg) if 'boot floor' in e], config.validate(floor_cfg)
    watchdog['enabled'] = False
    assert not [e for e in config.validate(floor_cfg) if 'boot floor' in e]

    print('ok: config validate/config_id/save/load/reset + nested buses, sensors, bus()/device(), '
          'schema version + outdated(), firmware_version restamp, watchdog boot floor')


main()
