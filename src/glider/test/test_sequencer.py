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


# small thresholds for a fast, deterministic test; gc_flight off here so the stage-logic checks do not
# also toggle the interpreter's GC (the GC policy has its own focused test below)
SPEC = {'period_ms': 10, 'launch_g': 3.0, 'launch_ms': 100, 'boost_timeout_ms': 500,
        'land_agl_m': 5.0, 'land_ms': 100, 'still_g': 0.3, 'ground_ms': 300, 'gc_flight': False}


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

    # GLIDING: out of laser range holds; below land_agl must be SUSTAINED land_ms (g12: not a spike)
    seq._tick(1630)
    assert ctrl.stage == Stage.GLIDING
    agl.push(2.0)
    seq._tick(1640)  # below land_agl -> the sustained timer starts, not yet elapsed
    assert ctrl.stage == Stage.GLIDING
    agl.push(50.0)   # a single spurious-low sample bounces back up -> timer resets, no premature flare
    seq._tick(1650)
    assert ctrl.stage == Stage.GLIDING
    agl.push(2.0)
    seq._tick(1700)  # below again -> restart the timer
    seq._tick(1810)  # 110 ms > land_ms 100 -> LANDING
    assert ctrl.stage == Stage.LANDING

    # LANDING: ~1 g stationary sustained ground_ms -> done
    accel.push((0.0, 0.0, 1.0))
    seq._tick(1650)
    assert ctrl.stage == Stage.LANDING
    seq._tick(1960)  # 310 ms > ground_ms 300
    assert ctrl.stage == Stage.DONE

    # guard: a transient (high g that drops before launch_ms) does NOT trip launch
    ctrl.stage = Stage.SETTING
    seq._stage_seen = None
    accel.push((0.0, 0.0, 8.0))
    seq._tick(2000)  # condition starts
    accel.push((0.0, 0.0, 1.0))  # drops back before launch_ms
    seq._tick(2050)
    assert ctrl.stage == Stage.SETTING

    # operator hold (ground test): manual pauses auto-sequencing -- a sustained launch is ignored
    ctrl.stage = Stage.SETTING
    ctrl.manual = True
    seq._stage_seen = None
    accel.push((0.0, 0.0, 8.0))
    seq._tick(3000)
    seq._tick(3200)  # well past launch_ms
    assert ctrl.stage == Stage.SETTING  # held -> no auto-advance

    # missing accel reading: _magnitude returns None (guarded, no raise) and _tick skips via an explicit
    # is-not-None check -- a dropped sample does not advance, no crash, no GC-churning exception (g9).
    assert sequencer._magnitude(None) is None

    class _NoReading:
        def value(self):
            return None  # a stale / absent databoard parameter

    ctrl.stage = Stage.SETTING
    ctrl.manual = False
    seq._stage_seen = None
    seq._accel = _NoReading()
    seq._tick(4000)  # accel absent -> guarded -> tick does nothing
    assert ctrl.stage == Stage.SETTING  # no crash, no advance

    # g14: GC policy -- compacted + DISABLED at BOOSTING, re-enabled at LANDING (coludo.md), and finish()
    # never leaves it off. gc_flight True here (the only test that exercises the toggle).
    import gc
    gseq = sequencer.Sequencer('sequencer', {'gc_flight': True}, _StubController())
    assert await gseq.setup() is True
    assert gc.isenabled()
    gseq._advance(Stage.BOOSTING, 'launch')
    assert not gc.isenabled()           # GC off while airborne -> no collection can stall a control slice
    gseq._advance(Stage.LANDING, 'agl')
    assert not gc.isenabled()           # STILL off through the flare (a collect at <5 m could crash it)
    gseq._advance(Stage.DONE, 'stationary')
    assert gc.isenabled()               # re-enabled + collected only once stationary on the ground
    gc.disable()
    await gseq.finish()
    assert gc.isenabled()               # defensive: a mid-flight stop must not leave GC disabled

    print('ok: sequencer -- launch detect, boost-timeout, agl landing, on-ground, guard, manual hold, '
          'no-accel skip, GC flight policy')


asyncio.run(amain())
