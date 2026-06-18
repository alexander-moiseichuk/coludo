# On-board test for the Control link task (tasks/cc_link.py): @task.driver('cc') registration and
# that setup builds the client when a cc_host is configured, and skips otherwise. Run by `make test`.

import asyncio

import config_default
import task
import tasks.cc_link


class _StubController:
    config = config_default.default()


class _NoCcController:
    config = {'board': {'id': 'x', 'mcu': 'esp32p4'}, 'wifi': {}}


async def amain():
    assert task.ACTIVITIES.get('cc') is tasks.cc_link.ControlLink  # registered driver

    # cc_host configured -> setup builds the reconnecting client toward the hub
    link = tasks.cc_link.ControlLink('cc', {}, _StubController())
    assert await link.setup() is True and link.validate()
    assert link._client.host == '192.168.10.1' and link._client.port == 1234

    # negative: no cc_host -> setup skips (board runs standalone, no CC)
    standalone = tasks.cc_link.ControlLink('cc', {}, _NoCcController())
    assert await standalone.setup() is False

    print('ok: cc link task registered, setup builds the client, skips without cc_host')


asyncio.run(amain())
