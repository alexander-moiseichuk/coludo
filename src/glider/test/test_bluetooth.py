# On-board test for the Bluetooth radio driver (drivers/bluetooth.py): @task.driver('bluetooth')
# registration and that it applies the configured `radio` state (default off) + live toggle via
# update(). Run by `make test`.

import asyncio

import task
from drivers import bluetooth


async def amain():
    assert task.ACTIVITIES.get('bluetooth') is bluetooth.Bluetooth  # registered driver

    # default (no `radio` field) -> off; inspect reports both requested + actual
    off = bluetooth.Bluetooth('bluetooth', {}, None)
    assert await off.setup() is True and off.validate()
    snapshot = off.inspect()
    assert snapshot['radio'] is False and snapshot['active'] in (False, None)

    # explicit radio: True -> on (None where the board has no BLE)
    on = bluetooth.Bluetooth('bluetooth', {'radio': True}, None)
    assert await on.setup() is True
    assert on.radio is True and on.active in (True, None)

    # update() toggles live and reports the change only when it actually changes
    assert on.update({'radio': False}) == ['radio'] and on.active in (False, None)
    assert on.update({'radio': False}) == []

    print('ok: bluetooth driver registered, applies configured radio state + live toggle')


asyncio.run(amain())
