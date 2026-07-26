"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the Wi-Fi station driver (drivers/wifi.py): @task.driver('wifi') registration,
setup (brings the STA interface up), config parsing, and inspect. Does NOT join a network (that
needs the AP up) -- only checks construction. Run by `make test`.
"""

import asyncio

import config_default
import task
from drivers import wifi


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('wifi') is wifi.Wifi  # registered driver

    radio = wifi.Wifi('wifi', {}, _StubController())
    assert await radio.setup() is True and radio.validate()

    # params come from the `wifi` config section; the default networks are the panda+coludo pair,
    # each a FULL entry inheriting the top-level keys (the proven panda shape, replicated)
    assert radio.ssid == 'panda' and radio.tx_power == 11 and radio._policy == 'auto'
    assert [n['ssid'] for n in radio._networks] == ['panda', 'coludo']
    assert radio._networks[0]['retry_ms'] == 10000 and radio._networks[0]['tx_power_dbm'] == 11

    # per-network parsing: strings are ssid sugar, missing keys inherit the top level, entry keys
    # override, enabled:false and policy:'disabled' park an entry, empty ssids are dropped
    class _ManyController:
        config = dict(config_default.default(),
                      wifi={'ssid': 'lab', 'policy': 'disabled', 'retry_ms': 7000,
                            'tx_power_dbm': 9, 'networks': [
                                'hotspot',
                                {'ssid': 'field', 'retry_ms': 30000, 'tx_power_dbm': 5},
                                {'ssid': 'parked', 'enabled': False},
                                {'ssid': 'off', 'policy': 'disabled'},
                                {'ssid': ''},
                            ]})

    many = wifi.Wifi('wifi', {}, _ManyController())
    assert await many.setup() is True
    assert many._policy == 'disabled'  # the SESSION policy (run() never touches the radio)
    assert [n['ssid'] for n in many._networks] == ['hotspot', 'field']  # parked/off/empty dropped
    assert many._networks[0]['retry_ms'] == 7000  # inherited from the top level...
    assert many._networks[1]['retry_ms'] == 30000 and many._networks[1]['tx_power_dbm'] == 5  # ...or overridden

    # per-network backoff clocks: never-tried is eligible at once; a tried network is quiet until
    # ITS OWN retry_ms elapsed; rotation round-robins the eligible ones; None while all backing off
    first = many._next_network(1000)
    assert first['ssid'] == 'hotspot' and first['last_ms'] == 1000
    second = many._next_network(1500)
    assert second['ssid'] == 'field'  # round-robin advanced (hotspot inside its 7 s window anyway)
    assert many._next_network(2000) is None  # both inside their windows -> nothing eligible
    assert many._next_network(8100)['ssid'] == 'hotspot'  # hotspot's 7 s elapsed; field's 30 s has not
    assert many._next_network(9000) is None  # hotspot re-stamped at 8100, field still quiet
    assert many._next_network(31600)['ssid'] == 'field'  # field's 30 s from 1500 finally elapsed

    # interface up but not joined (no connect() called) -> inspect reflects it
    assert radio.isconnected() is False
    snapshot = radio.inspect()
    assert set(snapshot.keys()) == {'name', 'ok', 'healthy',  # the common Task.inspect base
                                    'ssid', 'tx_power', 'connected', 'rssi', 'ip', 'networks'}
    assert snapshot['ssid'] == 'panda' and snapshot['connected'] is False

    # update: re-applying the same tx_power changes nothing
    assert radio.update({'tx_power': radio.tx_power}) == []

    print('ok: wifi task registered, setup brings the STA up, params/inspect, not-connected')


asyncio.run(amain())
