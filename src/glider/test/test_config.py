# On-board (MicroPython) test for the board config loader/validator (config.py), new schema:
# nested buses (uart/i2c/spi -> id), `sensors` + `components` with 'type:id' bus refs.
# Run by `make test`.

import config
from config_default import default


def main():
    # the default config validates clean
    assert config.validate(default()) == [], config.validate(default())

    # config_id is stable and sensitive
    a, b = default(), default()
    assert config.config_id(a) == config.config_id(b)
    b['board']['rev'] = 99
    assert config.config_id(a) != config.config_id(b)
    assert isinstance(config.config_id(a), str) and len(config.config_id(a)) >= 8

    # pin uniqueness across nested buses + pins
    dup = default()
    dup['pins']['servo_yaw'] = dup['buses']['i2c']['0']['sda']  # collide with GPIO7
    assert any('used by both' in e for e in config.validate(dup))

    # reserved pin (GPIO18 is a C6 Wi-Fi line)
    res = default()
    res['pins']['servo_yaw'] = 18
    assert any('reserved GPIO18' in e for e in config.validate(res))

    # unknown bus reference on a sensor
    badref = default()
    badref['sensors'][0]['bus'] = 'nope'
    assert any('not a defined bus' in e for e in config.validate(badref))

    # bad bus type
    badtype = default()
    badtype['buses']['oops'] = {'0': {'tx': 99, 'rx': 98}}
    assert any('not one of uart/i2c/spi' in e for e in config.validate(badtype))

    # board.id must be a bare wire token (no spaces)
    spaced = default()
    spaced['board']['id'] = 'glider 1'
    assert any('must not contain whitespace' in e for e in config.validate(spaced))

    # bus() / device() helpers
    cfg = default()
    assert config.bus(cfg, 'i2c:0') == {'sda': 7, 'scl': 8, 'freq': 400000}
    assert config.bus(cfg, 'uart:2')['baud'] == 9600
    assert config.bus(cfg, 'nope') is None
    assert config.device(cfg, driver='uart_sink')['name'] == 'recorder'
    assert config.device(cfg, name='gnss')['bus'] == 'uart:2'
    assert config.device(cfg, name='absent') is None

    # save / load round-trip on the board filesystem
    path = 'test_board.json'
    config.reset(path)
    cid = config.save(default(), path)
    cfg, source, errs = config.load(path, defaults=default())
    assert source == 'active' and cfg == default() and config.config_id(cfg) == cid

    # corrupt file -> fallback to defaults (never crash)
    f = open(path, 'w')
    f.write('{ not json ')
    f.close()
    cfg, source, errs = config.load(path, defaults=default())
    assert cfg == default() and 'fallback' in source

    # invalid config is never written
    bad = default()
    bad['pins']['servo_yaw'] = bad['buses']['i2c']['0']['scl']
    raised = False
    try:
        config.save(bad, path)
    except ValueError:
        raised = True
    assert raised

    # reset removes the active file
    assert config.reset(path) is True
    cfg, source, errs = config.load(path, defaults=default())
    assert source == 'default'

    print('ok: config validate/config_id/save/load/reset + nested buses, sensors, bus()/device()')


main()
