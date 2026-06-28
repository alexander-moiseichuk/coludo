# On-board test for driver diagnose() -- the deeper wire-level self-analysis the Controller folds into a
# failed device's reason (and the operator sees in verify/probe). Covers every new diagnose(): the
# config-fault path (deterministic, no hardware) + the present-device path where one is wired. diagnose()
# reads transport state that setup() builds, so the bus cases seed _bus/_addr to mimic the post-setup
# instance the Controller diagnoses. Run by `make test`.

import asyncio

import config_default
import i2cbus
import task
from drivers import atgm336h, icp10111, separation, sg90, vl53l4cx, wifi


class _Stub:
    config = config_default.default()
    config['pins']['nc'] = 52  # a free, unconnected GPIO for the pin checks


def _bus0():
    return i2cbus.get(0, config_default.default()['buses']['i2c']['0'])


async def amain():
    stub = _Stub()
    for name, cls in (('sg90', sg90.SG90), ('separation', separation.Separation),
                      ('icp10111', icp10111.Icp10111), ('vl53l4cx', vl53l4cx.Vl53l4cx),
                      ('atgm336h', atgm336h.Atgm336h), ('wifi', wifi.Wifi)):
        assert task.ACTIVITIES.get(name) is cls and hasattr(cls, 'diagnose')  # registered + has diagnose()

    # sg90 -- PWM (self-contained: resolves the pin, brings a PWM up to confirm it is alive)
    assert 'no PWM' in await sg90.SG90('s', {}, stub).diagnose()                 # no pin in config
    assert 'alive' in await sg90.SG90('s', {'pin': 'nc'}, stub).diagnose()       # free PWM-capable pin

    # separation -- GPIO that should be HIGH (nested) at check; a free pull-down pin reads LOW -> flagged
    assert 'no pin' in await separation.Separation('x', {'pin': 'absent'}, stub).diagnose()
    low = await separation.Separation('x', {'pin': 'nc'}, stub).diagnose()
    assert 'LOW' in low and 'expected HIGH' in low

    # icp10111 -- command-based product id: undefined bus -> no transport; a dead address -> no response
    assert 'no transport' in await icp10111.Icp10111('b', {'bus': 'i2c', 'id': 9}, stub).diagnose()
    icp = icp10111.Icp10111('b', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, stub)
    icp._bus, icp._addr = _bus0(), 0x7F
    assert 'no I2C response' in await icp.diagnose()

    # vl53l4cx -- 16-bit MODEL_ID via commons.id_classify: undefined bus / dead address
    assert 'no transport' in await vl53l4cx.Vl53l4cx('l', {'bus': 'i2c', 'id': 9}, stub).diagnose()
    vl = vl53l4cx.Vl53l4cx('l', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, stub)
    vl._bus, vl._addr = _bus0(), 0x7F
    assert 'no bus response' in await vl.diagnose()

    # gnss base (atgm336h + neo6mv2 share it) -- undefined uart -> no transport
    assert 'no transport' in await atgm336h.Atgm336h('g', {'bus': 'uart', 'id': 9}, stub).diagnose()

    # present devices, only when wired: the real icp (0x63) / vl53 (0x29) report present/ok
    on = _bus0().scan()
    if 0x63 in on:
        d = icp10111.Icp10111('b', {'bus': 'i2c', 'id': 0, 'addr': 0x63}, stub)
        d._bus, d._addr = _bus0(), 0x63
        assert 'ok' in await d.diagnose()
    if 0x29 in on:
        d = vl53l4cx.Vl53l4cx('l', {'bus': 'i2c', 'id': 0, 'addr': 0x29}, stub)
        d._bus, d._addr = _bus0(), 0x29
        assert 'present' in await d.diagnose()

    # wifi -- dumps a 'wifi ::' summary (radio up or no-interface) to print + the recorder log
    w = wifi.Wifi('wifi', {}, stub)
    await w.setup()
    assert 'wifi ::' in await w.diagnose()

    print('ok: diagnose() -- sg90 PWM / separation HIGH / icp10111 + vl53l4cx id / gnss NMEA / wifi dump',
          '| i2c present:', [hex(a) for a in on])


asyncio.run(amain())
