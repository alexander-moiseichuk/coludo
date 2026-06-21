# On-board test for the flight-stage sequencer (tasks/sequencer.py): the guarded, forward-only
# transitions driven by synthetic databoard accel/agl with a controlled `now` (ticks_ms) fed to
# _tick(). Run by `make test`.

import asyncio

import config_default
import controller
import databoard
import recorder
import task
from tasks import sequencer

Stage = controller.Stage


class FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _StubController:
    def __init__(self):
        self.config = config_default.default()
        self.stage = Stage.SETTING
        self.manual = False  # operator hold -> the sequencer pauses (tested below)

    def set_stage(self, stage):
        self.stage = stage


# small thresholds for a fast, deterministic test
SPEC = {'period_ms': 10, 'launch_g': 3.0, 'launch_ms': 100, 'boost_timeout_ms': 500,
        'land_agl_m': 5.0, 'still_g': 0.3, 'ground_ms': 300}


async def amain():
    assert task.ACTIVITIES.get('sequencer') is sequencer.Sequencer  # registered driver
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())  # sequencer.csv telemetry
    ctrl = _StubController()
    seq = sequencer.Sequencer('sequencer', SPEC, ctrl)
    assert await seq.setup() is True
    accel = databoard.Databoard.provide('imu', {'accel': {'priority': 0, 'timeout_ms': 1000}}, 'accel')
    agl = databoard.Databoard.provide('laser', {'agl': {'priority': 0, 'timeout_ms': 1000}}, 'agl')

    # SETTING: 1 g at rest -> no launch
    accel.push((0.0, 0.0, 1.0))
    seq._tick(0)
    assert ctrl.stage == Stage.SETTING

    # launch must be SUSTAINED launch_ms before BOOSTING (a transient is not enough)
    accel.push((0.0, 0.0, 8.0))
    seq._tick(1000)
    assert ctrl.stage == Stage.SETTING  # the condition just started
    seq._tick(1100)  # 100 ms sustained
    assert ctrl.stage == Stage.BOOSTING

    # BOOSTING: the separation switch never fired -> burnout-timeout fallback to GLIDING
    seq._tick(1110)  # boost-entry tick
    assert ctrl.stage == Stage.BOOSTING
    seq._tick(1620)  # 510 ms > boost_timeout 500
    assert ctrl.stage == Stage.GLIDING

    # GLIDING: no agl (out of laser range) holds; agl below land_agl -> LANDING
    seq._tick(1630)
    assert ctrl.stage == Stage.GLIDING
    agl.push(2.0)
    seq._tick(1640)
    assert ctrl.stage == Stage.LANDING

    # LANDING: ~1 g stationary sustained ground_ms -> done
    accel.push((0.0, 0.0, 1.0))
    seq._tick(1650)
    assert ctrl.stage == Stage.LANDING
    seq._tick(1960)  # 310 ms > ground_ms 300
    assert ctrl.stage == Stage.DONE

    # guard: a transient (high g that drops before launch_ms) does NOT trip launch
    ctrl.stage, seq._stage_seen = Stage.SETTING, None
    accel.push((0.0, 0.0, 8.0))
    seq._tick(2000)  # condition starts
    accel.push((0.0, 0.0, 1.0))  # drops back before launch_ms
    seq._tick(2050)
    assert ctrl.stage == Stage.SETTING

    # operator hold (ground test): manual pauses auto-sequencing -- a sustained launch is ignored
    ctrl.stage, ctrl.manual, seq._stage_seen = Stage.SETTING, True, None
    accel.push((0.0, 0.0, 8.0))
    seq._tick(3000)
    seq._tick(3200)  # well past launch_ms
    assert ctrl.stage == Stage.SETTING  # held -> no auto-advance

    print('ok: sequencer -- launch detect, boost-timeout, agl landing, on-ground, guard, manual hold')


asyncio.run(amain())
