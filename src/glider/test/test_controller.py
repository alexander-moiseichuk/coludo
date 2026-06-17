# On-board (MicroPython) test for the Task base + Controller skeleton.
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import asyncio

from controller import Controller
from task import Task


class FakeSensor(Task):
    async def setup(self):
        self.ran = 0
        self._ok = True
        return True

    async def run(self):
        for _ in range(3):
            self.ran += 1
            await asyncio.sleep_ms(1)

    def inspect(self):
        status = Task.inspect(self)
        status['ran'] = self.ran
        return status


class FailSensor(Task):
    async def setup(self):
        return False


def make_config():
    return {
        'board': {'id': 't', 'mcu': 'esp32p4'},
        'buses': {},
        'pins': {},
        'components': [
            {'name': 's1', 'driver': 'fake', 'enabled': True},
            {'name': 's2', 'driver': 'fake', 'enabled': True},
            {'name': 'off', 'driver': 'fake', 'enabled': False},
            {'name': 'bad', 'driver': 'nodriver', 'enabled': True},
            {'name': 'failing', 'driver': 'fail', 'enabled': True},
        ],
    }


async def amain():
    logs = []
    reg = {'fake': FakeSensor, 'fail': FailSensor}
    c = Controller(make_config(), registry=reg, log=lambda m: logs.append(m))

    # directory() excludes disabled, keeps config order
    assert c.directory() == ['s1', 's2', 'bad', 'failing'], c.directory()

    assert await c.setup() is True
    # s1/s2 created; 'off' disabled; 'bad' has no driver; 'failing' setup() -> False
    assert set(c.tasks.keys()) == set(['s1', 's2']), c.tasks.keys()

    # active()
    assert c.active('s1') is c.tasks['s1']
    assert c.active('missing') is None
    assert len(c.active()) == 2

    assert c.validate() is True

    # run the task loops, then check stats()
    await c.start()
    await asyncio.sleep_ms(50)
    rep = c.stats()
    assert rep['state'] == 'setting'
    assert rep['tasks']['s1']['ran'] >= 1, rep

    # tasks are individually inspectable through the Inspector
    from inspector import Inspector

    assert Inspector.inspect('s1')['ran'] >= 1

    # notify/emit
    seen = []
    c.tasks['s1'].notify(lambda task, ev: seen.append(ev))
    c.tasks['s1'].emit('hello')
    assert seen == ['hello']

    # state machine
    c.set_state('boosting')
    assert c.state == 'boosting'
    raised = False
    try:
        c.set_state('nope')
    except ValueError:
        raised = True
    assert raised

    # close one, then finish all
    await c.close('s1')
    assert 's1' not in c.tasks
    await c.finish()
    assert c.tasks == {}
    assert c.state == 'done'

    print('ok: controller directory/create/setup/run/active/inspect/stats/validate/close/finish')


asyncio.run(amain())
