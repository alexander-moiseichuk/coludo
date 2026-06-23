# On-board (MicroPython) test for the board config loader/validator (config.py), new schema:
# nested buses (uart/i2c/spi -> id), `sensors` + `components` with 'type:id' bus refs.
# Run by `make test`.

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

    # pin uniqueness across nested buses + pins
    dup = config_default.default()
    dup['pins']['servo_yaw'] = dup['buses']['i2c']['0']['sda']  # collide with GPIO7
    assert any('used by both' in e for e in config.validate(dup))

    # reserved pin (GPIO18 is a C6 Wi-Fi line)
    res = config_default.default()
    res['pins']['servo_yaw'] = 18
    assert any('reserved GPIO18' in e for e in config.validate(res))

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

    # board.id must be a bare wire token (no spaces)
    spaced = config_default.default()
    spaced['board']['id'] = 'glider 1'
    assert any('must not contain whitespace' in e for e in config.validate(spaced))

    # a component must name its implementation: `driver` (drivers/) or `activity` (tasks/)
    noimpl = config_default.default()
    del noimpl['components'][0]['activity']  # the recorder component
    assert any('driver` (drivers/) or `activity`' in e for e in config.validate(noimpl))

    # bus() / device() helpers — addressed by (kind, id), no string parsing
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

    print('ok: config validate/config_id/save/load/reset + nested buses, sensors, bus()/device()')


main()
