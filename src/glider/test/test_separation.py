# On-board test for the separation switch (drivers/separation.py): @task.driver('separation')
# registration, pin setup, the Boosting->Gliding transition on a confirmed separation, the
# not-Boosting guard, and graceful-absent. Run by `make test`.

import asyncio

import config_default
import controller
import recorder
import task
from drivers import separation


class _StubController:
    def __init__(self, stage):
        self.config = config_default.default()
        self.stage = stage
        self.transitions = []

    def set_stage(self, stage):
        self.stage = stage
        self.transitions.append(stage)


class _FakeWriter:
    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


async def amain():
    assert task.ACTIVITIES.get('separation') is separation.Separation  # registered
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())  # _apply logs the event

    # separation during Boosting -> Gliding
    boosting = _StubController(controller.Stage.BOOSTING)
    sep = separation.Separation('separation', {'pin': 'separation_switch'}, boosting)
    assert await sep.setup() is True and sep.validate()
    sep._separated = False  # start from a known nested state for the logic check
    sep._apply(True)  # confirmed separation
    assert sep._separated is True and boosting.stage == controller.Stage.GLIDING
    assert sep.inspect()['separated'] is True

    # the event is recorded to telemetry (durable separation.csv), not only the best-effort log
    await recorder.Recorder.drain()
    rows = [bytes(i) for i in recorder.Recorder._uart.items]
    assert any(b'_separation.csv@' in r and b'separated;gliding' in r for r in rows), rows

    # idempotent (same level) + a re-nest emits but never reverses the stage
    sep._apply(True)
    sep._apply(False)
    assert sep._separated is False and boosting.stage == controller.Stage.GLIDING
    assert boosting.transitions == [controller.Stage.GLIDING]

    # the guard: separation while NOT Boosting (e.g. a ground test in Setting) does not transition
    setting = _StubController(controller.Stage.SETTING)
    ground = separation.Separation('separation', {'pin': 'separation_switch'}, setting)
    assert await ground.setup() is True
    ground._separated = False
    ground._apply(True)
    assert ground._separated is True and setting.stage == controller.Stage.SETTING

    # graceful: an unknown pin role fails setup
    bad = separation.Separation('separation', {'pin': 'nope'}, setting)
    assert await bad.setup() is False

    print('ok: separation registered, setup, Boosting->Gliding (guarded), graceful-absent')


asyncio.run(amain())
