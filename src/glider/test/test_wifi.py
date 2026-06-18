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

    # interface up but not joined (no connect() called) -> inspect reflects it
    assert radio.isconnected() is False
    snapshot = radio.inspect()
    assert set(snapshot.keys()) == {'ssid', 'tx_power', 'connected', 'rssi', 'ip'}
    assert snapshot['ssid'] == 'panda' and snapshot['connected'] is False

    # update: re-applying the same tx_power changes nothing
    assert radio.update({'tx_power': radio.tx_power}) == []

    print('ok: wifi task registered, setup brings the STA up, params/inspect, not-connected')


asyncio.run(amain())
