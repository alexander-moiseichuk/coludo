"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the flight-stage sequencer (tasks/sequencer.py): the guarded, forward-only transitions
driven by synthetic databoard accel/agl with a controlled `now` (ticks_ms) fed to _tick(). Run by
`make test`.
"""

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
        self.armed = False  # only an ARMED flight drops the warm-start breadcrumb (tested below)

    def set_stage(self, stage):
        self.stage = stage


"""
small thresholds for a fast, deterministic test; disable_gc_flight off here so the stage-logic checks do not
also toggle the interpreter's GC (the GC policy has its own focused test below). apogee_arm_ms is
tiny so the apogee checks run promptly; the arming window has its own focused test below.
"""
SPEC = {'period_ms': 10, 'launch_g': 3.0, 'launch_ms': 100, 'boost_timeout_ms': 500,
        'apogee_arm_ms': 50, 'land_agl_m': 5.0, 'land_ms': 100, 'still_g': 0.3, 'ground_ms': 300,
        'disable_gc_flight': False}


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

    # GLIDING: out of laser range holds; below land_agl must be SUSTAINED land_ms (not a spike)
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

    # launch BACKUP: the baro climbing past launch_alt_m (10 m) trips BOOSTING even with a marginal,
    # sub-launch_g accel -- a heavy stack boosting near the threshold still detects once it left the pad.
    ctrl.stage = Stage.SETTING
    seq._stage_seen = None
    elevation = databoard.Databoard.provide(
        'baro', {'elevation': {'priority': 0, 'timeout_ms': 1000}}, 'elevation')
    accel.push((0.0, 0.0, 2.0))  # only 2 g -- below this SPEC's 3 g launch_g
    elevation.push(3.0)          # still on/near the pad
    seq._tick(2100)
    assert ctrl.stage == Stage.SETTING  # neither signal trips
    elevation.push(15.0)         # climbed 15 m off the pad -> launched (no accel sustain needed)
    seq._tick(2110)
    assert ctrl.stage == Stage.BOOSTING

    """
    apogee detect: in BOOSTING the baro peaks then falls apogee_drop_m (5 m) -> deploy at the TOP of the
    arc (mass/motor-independent), before the long burnout-timeout fallback. The detector ARMS
    apogee_arm_ms (50 here) after the entry TICK -- a reading inside the window is ignored.
    """
    seq._tick(2120)   # BOOSTING entry seen -> the arming clock starts here
    elevation.push(80.0)
    seq._tick(2140)   # 20 ms in: a burn spike INSIDE the arming window...
    assert seq._apogee_max is None           # ...does not poison the peak tracker
    elevation.push(150.0)
    seq._tick(2200)   # armed (80 ms past entry): climbing -> peak tracks up
    elevation.push(240.0)
    seq._tick(2210)   # new peak
    elevation.push(233.0)
    seq._tick(2220)   # fell 7 m below the 240 m peak -> the apogee dwell starts
    assert ctrl.stage == Stage.BOOSTING      # not yet SUSTAINED (a single dip is not apogee)
    elevation.push(232.0)
    seq._tick(2330)   # still down, >100 ms later -> deploy
    assert ctrl.stage == Stage.GLIDING

    # operator hold (ground test): manual pauses auto-sequencing -- a sustained launch is ignored
    ctrl.stage = Stage.SETTING
    ctrl.manual = True
    seq._stage_seen = None
    accel.push((0.0, 0.0, 8.0))
    seq._tick(3000)
    seq._tick(3200)  # well past launch_ms
    assert ctrl.stage == Stage.SETTING  # held -> no auto-advance

    # missing accel reading: _magnitude_sq returns None (guarded, no raise) and _tick skips via an
    # explicit is-not-None check -- a dropped sample does not advance, no crash, no GC exception.
    assert sequencer._magnitude_sq(None) is None

    class _NoReading:
        def value(self):
            return None  # a stale / absent databoard parameter

    ctrl.stage = Stage.SETTING
    ctrl.manual = False
    seq._stage_seen = None
    seq._accel = _NoReading()
    seq._elevation = _NoReading()  # both launch signals absent -> no advance
    seq._tick(4000)  # accel + elevation absent -> guarded -> tick does nothing
    assert ctrl.stage == Stage.SETTING  # no crash, no advance

    """
    apogee ARMING window: the motor exhaust pressure wave corrupts the in-airframe baro
    DURING BURN, so the whole detector (peak tracking included) is blind for apogee_arm_ms after
    BOOSTING entry -- a burn spike must neither deploy GLIDING under thrust nor poison the peak.
    """
    burn_ctrl = _StubController()
    bseq = sequencer.Sequencer('sequencer', dict(SPEC, apogee_arm_ms=100000), burn_ctrl)
    assert await bseq.setup() is True
    burn_ctrl.stage = Stage.BOOSTING
    bseq._tick(5000)   # BOOSTING entry seen -> the arming clock starts
    elevation.push(80.0)
    bseq._tick(5010)   # exhaust pressure spike reads +80 m...
    elevation.push(20.0)
    bseq._tick(5020)   # ...then a 60 m 'drop' -- a textbook false apogee
    bseq._tick(5150)   # past the detect dwell: unarmed, this WOULD have deployed
    assert burn_ctrl.stage == Stage.BOOSTING  # blind during burn -> no deploy under thrust
    assert bseq._apogee_max is None           # and the spike did not poison the peak tracker

    # RSO flight timeout: with every landing sensor dead the glider must not circle until
    # the battery dies -- flight_timeout_ms after BOOSTING entry the stage forces DONE.
    rso_ctrl = _StubController()
    rseq = sequencer.Sequencer('sequencer', dict(SPEC, flight_timeout_ms=1000), rso_ctrl)
    assert await rseq.setup() is True
    rso_ctrl.stage = Stage.BOOSTING
    rseq._tick(0)      # entry -> the flight clock starts
    rso_ctrl.stage = Stage.GLIDING  # separation happened; then all landing sensors go silent
    rseq._tick(500)
    assert rso_ctrl.stage == Stage.GLIDING  # under the timeout -> untouched
    rseq._tick(1100)
    assert rso_ctrl.stage == Stage.DONE     # the backstop landed the stage machine

    # externally-driven transitions land in sequencer.csv too: the separation driver (or an
    # operator command) moves the stage outside _advance() -- post-flight tooling needs ONE source.
    class _TelemetryLog:
        def __init__(self):
            self.rows = []

        def push(self, row):
            self.rows.append(row)

    ext_ctrl = _StubController()
    eseq = sequencer.Sequencer('sequencer', SPEC, ext_ctrl)
    assert await eseq.setup() is True
    eseq._telemetry = _TelemetryLog()
    ext_ctrl.stage = Stage.BOOSTING  # moved from OUTSIDE the sequencer
    eseq._tick(0)
    assert eseq._telemetry.rows[-1] == ('boosting', 'external')
    eseq._advance(Stage.GLIDING, 'test')  # the sequencer's own move...
    eseq._tick(10)
    assert eseq._telemetry.rows[-1] == ('gliding', 'test')  # ...is NOT double-logged as external

    """
    warm-start breadcrumb (specs/coludo.md): an ARMED flight with a zone drops it at BOOSTING and
    clears it at DONE; unarmed (passive telemetry) flights never do -- a warm start must not arm
    a flight that was never armed. Round-trips the real NVS on the board (no-op on CPython).
    """
    import warmstart

    class _StubMission:
        zone = ((48.001, 11.000), (48.000, 11.010))

        def launch_point(self):
            return (48.0005, 11.005)

        def freeze_launch(self):
            pass  # the real Mission pins a late fix here; nothing to pin in the rig

    crumb_ctrl = _StubController()
    cseq = sequencer.Sequencer('sequencer', SPEC, crumb_ctrl)
    assert await cseq.setup() is True
    cseq._mission = _StubMission()
    warmstart.clear()
    cseq._advance(Stage.BOOSTING, 'launch')  # NOT armed -> no breadcrumb
    assert warmstart.load() is None
    crumb_ctrl.armed = True
    cseq._advance(Stage.BOOSTING, 'launch')  # armed + zone -> the breadcrumb drops
    crumb = warmstart.load()
    if warmstart._nvs is not None:  # board: round-trip the real NVS
        assert crumb is not None and abs(crumb['launch'][0] - 48.0005) < 1e-6
        assert abs(crumb['zone'][1][1] - 11.010) < 1e-6 and crumb['pad_altitude'] == 0.0
        cseq._advance(Stage.DONE, 'stationary')  # flight over -> the next boot is cold
        assert warmstart.load() is None
    # the warm-start gate itself is pure (host + board): ALL five signals must agree
    made = {'launch': [48.0, 11.0], 'zone': [[48.001, 11.0], [48.0, 11.01]],
            'pad_altitude': 500.0, 'stamp': 0}
    assert warmstart.should_restore(None, True, 600.0, True, 30)[0] is False   # no breadcrumb
    assert warmstart.should_restore(made, False, 600.0, True, 30)[0] is False  # switch reads nested
    assert warmstart.should_restore(made, True, None, True, 30)[0] is False    # baro never came up
    assert warmstart.should_restore(made, True, 505.0, True, 30)[0] is False   # too close to the pad
    assert warmstart.should_restore(made, True, 560.0, False, 30)[0] is False  # power-on = human hands
    assert warmstart.should_restore(made, True, 560.0, True, -5)[0] is False   # RTC restarted (power cycle)
    assert warmstart.should_restore(made, True, 560.0, True, 700)[0] is False  # crumb older than a flight
    assert warmstart.should_restore(made, True, 560.0, True, 30)[0] is True    # the genuine mid-air reset

    # GC policy -- compacted + DISABLED at BOOSTING, re-enabled at LANDING (coludo.md), and finish()
    # never leaves it off. disable_gc_flight True here (the only test that exercises the toggle).
    import gc
    gseq = sequencer.Sequencer('sequencer', {'disable_gc_flight': True}, _StubController())
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
          'no-accel skip, apogee arming, RSO flight timeout, external-transition log, warm-start '
          'breadcrumb, GC flight policy')


asyncio.run(amain())
