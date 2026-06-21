# On-board test for the Control link task (tasks/cc_link.py): @task.activity('cc') registration, that
# setup builds the client, and the hub-address rule -- explicit cc_host honored, else the `.1` of the
# board's own subnet is derived at dial time. Run by `make test`.

import asyncio

import config_default
import task
import tasks.cc_link


class _StubController:
    config = config_default.default()  # default has no cc_host -> derive at dial


class _ExplicitController:
    config = {'board': {'id': 'x', 'mcu': 'esp32p4'}, 'wifi': {'cc_host': '10.0.0.5', 'cc_port': 1234}}


async def amain():
    assert task.ACTIVITIES.get('cc') is tasks.cc_link.ControlLink  # registered driver

    # default (no cc_host) -> setup builds the client; host is derived at dial time, so None for now
    link = tasks.cc_link.ControlLink('cc', {}, _StubController())
    assert await link.setup() is True and link.validate()
    assert link._client.host is None and link._client.port == 1234

    # explicit cc_host -> used verbatim
    explicit = tasks.cc_link.ControlLink('cc', {}, _ExplicitController())
    assert await explicit.setup() is True
    assert explicit._client.host == '10.0.0.5'

    # derive: no explicit host -> the `.1` of the board's own subnet (the hub by convention)
    assert tasks.cc_link._network_host('192.168.102.152') == '192.168.102.1'
    assert tasks.cc_link._network_host('10.4.7.88') == '10.4.7.1'
    assert tasks.cc_link._network_host(None) is None  # no lease yet
    assert tasks.cc_link._network_host('0.0.0.0') is None

    print('ok: cc link task registered; default derives <subnet>.1, explicit cc_host honored')


asyncio.run(amain())
