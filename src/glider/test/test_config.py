# On-board (MicroPython) test for the board config loader/validator (config.py).
# Run by run_tests.sh / `make test`, which deploys the glider modules to the board first so
# this can import them. Raises (-> runner reports FAIL) on any failed assertion.

import config
from config_default import default

PATH = 'test_board.json'  # a throwaway path; never touches a real board.json


def main():
    # the default config validates clean
    errs = config.validate(default())
    assert errs == [], errs

    # config_id is stable and sensitive to changes
    a, b = default(), default()
    assert config.config_id(a) == config.config_id(b)
    b['board']['rev'] = 99
    assert config.config_id(a) != config.config_id(b)
    cid = config.config_id(a)
    assert isinstance(cid, str) and len(cid) >= 8

    # pin uniqueness
    dup = default()
    dup['pins']['servo_yaw'] = dup['buses']['i2c0']['sda']
    assert any('used by both' in e for e in config.validate(dup))

    # reserved pin (GPIO18 is a C6 Wi-Fi line)
    res = default()
    res['pins']['servo_yaw'] = 18
    assert any('reserved GPIO18' in e for e in config.validate(res))

    # unknown bus reference
    bb = default()
    bb['components'][0]['bus'] = 'nope'
    assert any('not a defined bus' in e for e in config.validate(bb))

    # board.id must be a bare wire token (no spaces)
    spc = default()
    spc['board']['id'] = 'glider 1'
    assert any('must not contain whitespace' in e for e in config.validate(spc))

    # save / load round-trip on the board filesystem
    config.reset(PATH)
    cid2 = config.save(default(), PATH)
    cfg, src, errs = config.load(PATH, defaults=default())
    assert src == 'active', src
    assert cfg == default()
    assert config.config_id(cfg) == cid2

    # corrupt file -> fallback to defaults (never crash)
    f = open(PATH, 'w')
    f.write('{ not json ')
    f.close()
    cfg, src, errs = config.load(PATH, defaults=default())
    assert cfg == default() and 'fallback' in src

    # invalid config is never written
    bad = default()
    bad['pins']['servo_yaw'] = bad['buses']['i2c0']['scl']
    raised = False
    try:
        config.save(bad, PATH)
    except ValueError:
        raised = True
    assert raised

    # reset removes the active file
    assert config.reset(PATH) is True
    cfg, src, errs = config.load(PATH, defaults=default())
    assert src == 'default'

    print('ok: validate/config_id/save/load/reset on MicroPython, id=%s' % cid)


main()
