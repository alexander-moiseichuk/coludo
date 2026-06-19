# On-board test for the Recorder's task adapter (tasks/recorder.py): @task.driver('recorder')
# registration, and that it proxies + wires the global Recorder singleton. Run by `make test`.

import asyncio

import config_default
import recorder
import task
import tasks.recorder


class _FakeWriter:
    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


class _StubController:
    config = config_default.default()


async def amain():
    # registered as the 'recorder' driver so the Controller builds the recorder component
    assert task.ACTIVITIES.get('recorder') is tasks.recorder.RecorderTask

    component = {'name': 'recorder', 'activity': 'recorder', 'bus': 'uart', 'id': 1, 'enabled': True}

    # the task proxies the singleton's Inspectable surface to the operator
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    proxy = tasks.recorder.RecorderTask('recorder', component, None)
    snapshot = proxy.inspect()
    assert snapshot['name'] == 'recorder' and snapshot['ok'] is False and 'session' in snapshot
    assert proxy.stats() == recorder.Recorder.stats()
    assert proxy.update({'stats_ms': 500}) == ['stats_ms'] and recorder.Recorder._stats_ms == 500

    # setup() wires the singleton from the controller's full config (resolves the recorder bus uart:1)
    wired = tasks.recorder.RecorderTask('recorder', component, _StubController())
    assert await wired.setup() is True and wired.validate() is True
    assert recorder.Recorder._tlm is not None  # singleton initialized through the task

    print('ok: tasks.recorder adapter registered, proxies + wires the Recorder singleton')


asyncio.run(amain())
