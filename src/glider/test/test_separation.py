"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the separation switch (drivers/separation.py): @task.driver('separation')
registration, pin setup, the Boosting->Gliding transition on a confirmed separation, the
not-Boosting guard, and graceful-absent. Run by `make test`.
"""

import asyncio
import time

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

    """
    THE DEBOUNCE ITSELF, driven through run() -- the part no test exercised before, which is how a
    single-sample "debounce" shipped. A scripted pin replays a timeline of levels (milliseconds since
    the edge -> level); run() is woken once, as the IRQ would, and the stage is read afterwards.
    """
    class _ScriptedPin:
        def __init__(self, timeline):
            self._timeline = timeline  # [(from_ms, level), ...] ascending; level 1 = nested, 0 = open
            self._t0 = time.ticks_ms()

        def value(self):
            elapsed = time.ticks_diff(time.ticks_ms(), self._t0)
            level = self._timeline[0][1]
            for start, value in self._timeline:
                if elapsed >= start:
                    level = value
            return level

    async def _run_with(timeline, window_ms=300):
        stage = _StubController(controller.Stage.BOOSTING)
        driver = separation.Separation('separation', {'pin': 'separation_switch'}, stage)
        assert await driver.setup() is True
        driver._separated = False
        driver._pin = _ScriptedPin(timeline)
        runner = asyncio.create_task(driver.run())
        await asyncio.sleep_ms(0)
        driver._on_edge(None)
        await asyncio.sleep_ms(window_ms)
        runner.cancel()
        return stage.stage

    # NEGATIVE: one ~7 ms open blip must NOT deploy -- this exact case forced GLIDING on the board
    assert await _run_with([(0, 1), (19, 0), (26, 1)]) == controller.Stage.BOOSTING, 'a 7 ms blip deployed'
    # NEGATIVE: contact chatter that never holds open must not deploy either
    chatter = [(0, 1)] + [(start, start // 8 % 2) for start in range(8, 200, 8)] + [(200, 1)]
    assert await _run_with(chatter) == controller.Stage.BOOSTING, 'chatter deployed'
    # POSITIVE: a real separation -- open and staying open -- must still deploy
    assert await _run_with([(0, 1), (5, 0)]) == controller.Stage.GLIDING, 'a held separation was missed'
    # POSITIVE: bounce first, then settle open -> deploys once the level holds
    bouncy = [(0, 1), (3, 0), (6, 1), (9, 0), (12, 1), (15, 0)]
    assert await _run_with(bouncy) == controller.Stage.GLIDING, 'bounce-then-hold was missed'

    print('ok: separation registered, setup, Boosting->Gliding (guarded), graceful-absent, '
          'debounce holds: blip + chatter rejected, held + bounce-then-hold accepted')


asyncio.run(amain())
