# On-board test for the Wi-Fi station driver (drivers/wifi.py): @task.driver('wifi') registration,
# setup (brings the STA interface up), config parsing, and inspect. Does NOT join a network (that
# needs the AP up) -- only checks construction. Run by `make test`.

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

    # params come from the `wifi` config section
    assert radio.ssid == 'panda' and radio.tx_power == 11
    assert radio._policy == 'auto' and radio._networks == ['panda']  # the CC-less field policy defaults

    # several networks: parsed in order (round-robin candidates), empties dropped; the single-ssid
    # fallback covers a config without a list; policy 'disabled' is carried to run()'s guard
    class _ManyController:
        config = dict(config_default.default(),
                      wifi={'ssid': 'lab', 'networks': ['hotspot', '', 'lab'], 'policy': 'disabled', 'mode': 'sta'})

    many = wifi.Wifi('wifi', {}, _ManyController())
    assert await many.setup() is True
    assert many._networks == ['hotspot', 'lab'] and many._policy == 'disabled'

    # interface up but not joined (no connect() called) -> inspect reflects it
    assert radio.isconnected() is False
    snapshot = radio.inspect()
    assert set(snapshot.keys()) == {'ssid', 'tx_power', 'connected', 'rssi', 'ip'}
    assert snapshot['ssid'] == 'panda' and snapshot['connected'] is False

    # update: re-applying the same tx_power changes nothing
    assert radio.update({'tx_power': radio.tx_power}) == []

    print('ok: wifi task registered, setup brings the STA up, params/inspect, not-connected')


asyncio.run(amain())
