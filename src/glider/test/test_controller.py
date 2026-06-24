# On-board (MicroPython) test for the Task base + Controller skeleton.
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import asyncio

import controller
import inspector
import task


class FakeSensor(task.Task):
    async def setup(self):
        self.ran = 0
        self._ok = True
        return True

    async def run(self):
        for _ in range(3):
            self.ran += 1
            await asyncio.sleep_ms(1)

    def inspect(self):
        status = task.Task.inspect(self)
        status['ran'] = self.ran
        return status


class FailSensor(task.Task):
    async def setup(self):
        return False


class MessySensor(task.Task):
    """setup() raises mid-init AND finish() raises on the half-set-up device -- the controller must
    still record the failure and bring up the rest of the board, not abort boot (finding 1.2.1)."""

    async def setup(self):
        raise RuntimeError('setup boom')

    async def finish(self):
        raise RuntimeError('cleanup boom')


class FlakySensor(task.Task):
    """Fails its first setup, succeeds on a retry (a fresh instance per attempt, so the attempt count
    is class-level) -- models a breadboard contact that makes on the second try."""

    attempts: int = 0

    async def setup(self):
        FlakySensor.attempts += 1
        self._ok = FlakySensor.attempts >= 2
        return self._ok

    async def run(self):
        await asyncio.sleep_ms(1)


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
    c = controller.Controller(make_config(), registry=reg, log=lambda m: logs.append(m))

    # directory() excludes disabled, keeps config order
    assert c.directory() == ['s1', 's2', 'bad', 'failing'], c.directory()

    assert await c.setup() is True
    # s1/s2 created; 'off' disabled; 'bad' has no driver; 'failing' setup() -> False
    assert set(c.tasks.keys()) == set(['s1', 's2']), c.tasks.keys()

    # failures collects every enabled device that did not come up (not the disabled 'off')
    assert set(c.failures.keys()) == set(['bad', 'failing']), c.failures
    assert c.failures['bad'] == 'no driver/activity' and 'setup failed' in c.failures['failing']
    assert c.inspect()['failures'] == c.failures  # exposed for the operator (probe / inspect)
    assert any('2 device(s) not up' in m for m in logs), logs

    # setup_retries: a device that fails its first setup comes up on a retry (breadboard contacts)
    FlakySensor.attempts = 0
    retry_cfg = {'board': {'id': 'r', 'mcu': 'esp32p4'}, 'setup_retries': 2,
                 'components': [{'name': 'flaky', 'driver': 'flaky', 'enabled': True}]}
    rc = controller.Controller(retry_cfg, registry={'flaky': FlakySensor}, log=lambda m: None)
    assert await rc.setup() is True
    assert 'flaky' in rc.tasks and rc.failures == {} and FlakySensor.attempts == 2  # up on the 2nd try
    await rc.finish()

    # 1.2.1: a device whose setup AND cleanup both raise must not abort boot -- it is recorded and the
    # rest of the board still comes up.
    messy_cfg = {'board': {'id': 'm', 'mcu': 'esp32p4'},
                 'components': [{'name': 'messy', 'driver': 'messy', 'enabled': True},
                                {'name': 'ok', 'driver': 'fake', 'enabled': True}]}
    mc = controller.Controller(messy_cfg, registry={'messy': MessySensor, 'fake': FakeSensor},
                               log=lambda m: None)
    assert await mc.setup() is True  # boot completes despite the messy cleanup raise
    assert 'ok' in mc.tasks and 'messy' in mc.failures  # good task up, messy one recorded (not crashed)
    await mc.finish()

    # active()
    assert c.active('s1') is c.tasks['s1']
    assert c.active('missing') is None
    assert len(c.active()) == 2

    # find(): non-blocking dependency lookup (None for any not up); Task.find delegates
    assert c.find(['s1', 's2']) == [c.tasks['s1'], c.tasks['s2']]
    assert c.find(['s1', 'missing']) == [c.tasks['s1'], None]
    assert c.tasks['s1'].find(['s2']) == [c.tasks['s2']]

    # query(waiting=False) == find; query(waiting=True) returns once all are present
    assert await c.query(['s1'], waiting=False) == [c.tasks['s1']]
    assert await c.tasks['s1'].query(['s2']) == [c.tasks['s2']]

    # query(waiting=True) parks until a not-yet-present task appears
    late = FakeSensor('late', {}, c)

    async def appear():
        await asyncio.sleep_ms(20)
        c.tasks['late'] = late

    asyncio.create_task(appear())
    assert await c.query(['s1', 'late'], waiting=True) == [c.tasks['s1'], late]
    c.tasks.pop('late')  # drop the never-set-up fixture so it doesn't skew validate()/stats()

    assert c.validate() is True

    # run the task loops, then check stats()
    await c.start()
    await asyncio.sleep_ms(50)
    rep = c.stats()
    assert rep['stage'] == 'setting'  # the operator-facing stage name
    assert rep['tasks']['s1']['ran'] >= 1, rep

    # tasks are individually inspectable through the Inspector
    assert inspector.Inspector.inspect('s1')['ran'] >= 1

    # notify/emit
    seen = []
    c.tasks['s1'].notify(lambda emitter, ev: seen.append(ev))
    c.tasks['s1'].emit('hello')
    assert seen == ['hello']

    # stage machine: int ids internally, name on the wire
    c.set_stage(controller.Stage.BOOSTING)
    assert c.stage == controller.Stage.BOOSTING and c.stage_name() == 'boosting'
    raised = False
    try:
        c.set_stage(99)  # not a defined stage
    except ValueError:
        raised = True
    assert raised

    # arming + manual hold (the actuation safety gate + ground-test override)
    assert c.armed is False and c.manual is False  # disarmed / auto by default
    c.arm()
    assert c.armed is True and c.inspect()['armed'] is True
    c.disarm()
    assert c.armed is False
    assert c.hold('gliding') is True and c.stage_name() == 'gliding' and c.manual is True
    assert c.hold('nope') is False  # unknown stage name
    c.resume()
    assert c.manual is False and c.inspect()['manual'] is False

    # close one, then finish all
    await c.close('s1')
    assert 's1' not in c.tasks
    await c.finish()
    assert c.tasks == {}
    assert c.stage == controller.Stage.DONE

    print('ok: controller directory/create/setup/run/active/inspect/stats/validate/close/finish')


asyncio.run(amain())
