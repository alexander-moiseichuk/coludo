"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the Phase 3 stabilization loop (tasks/flight.py): registration, GLIDING gating,
degraded->neutral on stale attitude, the PID->mixer->fin path, both scheduling modes (asyncio at
schedule_hz=0, machine.Timer at schedule_hz>0), and the WIRING of the extracted governor + guidance
(their laws are unit-tested in test_governor.py / test_guidance.py). Uses fake fins + a stub
controller; attitude comes from the databoard. Run by `make test`.
"""

import asyncio

import config_default
import databoard
import fixed
import guidance
import pid
import task
from controller import Stage
from tasks import flight


class _FakeFin:
    def __init__(self):
        self.angle = None

    def update(self, props):
        self.angle = props['angle']

    def set_angle(self, angle):  # the flight loop's hot-path entry (no dict; compare-and-set in sg90)
        self.angle = angle
        return angle


class _StubController:
    def __init__(self, stage):
        self.config = config_default.default()  # carries the fins/mixer block
        self.stage = stage  # a Stage id (int) -- the flight loop reads controller.stage, not strings
        self.armed = True  # the gate: disarmed -> the loop holds neutral (tested below)
        self.fins = {n: _FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def stage_name(self):
        return Stage.STAGES[self.stage]

    def find(self, names):
        return [self.fins.get(n) for n in names]


class _StubMission:
    zone = ((48.001, 11.000), (48.000, 11.010))  # longitude-stretched -> gates on the left/right edges

    def __init__(self, launch=None):
        self._launch = launch

    def launch_point(self):
        return self._launch

    def zone_points(self):
        import navigation
        return navigation.zone(self.zone[0], self.zone[1])

    def zone_aspect(self):
        import navigation
        return navigation.zone_aspect(self.zone[0], self.zone[1])

    def endgame_heading(self):
        import guidance
        wide = self.zone_aspect() > guidance.Heading.OO_ASPECT
        return guidance.Heading.FIG_OO if wide else guidance.Heading.FIG_O


async def amain():
    assert task.ACTIVITIES.get('flight') is flight.Flight  # registered driver

    ctrl = _StubController(Stage.SETTING)
    unit = flight.Flight('flight', {'schedule_hz': 0, 'period_ms': 20, 'gains': {'roll': {'kp': 1.0}}}, ctrl)
    assert await unit.setup() is True
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 1000}}, 'attitude')

    # not gliding -> the loop is gated, no actuation
    unit._tick()
    assert ctrl.fins['servo_yaw'].angle is None

    # gliding but attitude born-stale (not pushed) -> degraded -> fins neutral, not engaged
    ctrl.stage = Stage.GLIDING
    unit._tick()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # gliding + fresh attitude -> engage: kp=1 on roll=10 -> roll cmd -10 -> elevons differential
    attitude.push((100.0, fixed.from_float(10), fixed.from_float(-5)))  # heading, roll, pitch
    unit._tick()
    assert unit._active is True and unit._stage == Stage.GLIDING and unit._steps == 1
    assert ctrl.fins['servo_eleron_left'].angle == 80 and ctrl.fins['servo_eleron_right'].angle == 100
    assert ctrl.fins['servo_yaw'].angle == 90  # heading hold captured at 100 -> error 0 -> rudder neutral

    # landing is NOT a control stage by default (only gliding) -> centre the fins + disengage
    ctrl.stage = Stage.LANDING
    unit._tick()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # disarmed -> neutral even in a control stage (the arming safety gate)
    ctrl.stage = Stage.GLIDING
    ctrl.armed = False
    unit._tick()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False
    ctrl.armed = True  # re-arm for the scheduling-mode checks below

    # asyncio mode (schedule_hz 0): run() loops and runs control steps
    ctrl.stage = Stage.GLIDING
    runner = asyncio.create_task(unit.run())
    await asyncio.sleep_ms(80)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    assert unit._steps > 1  # the asyncio loop ticked

    # timer mode (schedule_hz > 0): a machine.Timer drives the step deterministically
    timed = flight.Flight('flight', {'schedule_hz': 100, 'gains': {}}, _StubController(Stage.GLIDING))
    assert await timed.setup() is True
    timer_runner = asyncio.create_task(timed.run())
    await asyncio.sleep_ms(120)
    timer_runner.cancel()
    try:
        await timer_runner
    except asyncio.CancelledError:
        pass
    await timed.finish()  # deinit the timer
    assert timed._steps > 0 and timed.inspect()['schedule'] == 'timer'

    # per-stage behaviour: LANDING declared as a control stage -> continuous control glide->landing
    # (stays engaged, setpoint switches), then a non-control stage centres the fins
    attitude.push((100.0, fixed.from_float(10), fixed.from_float(-5)))  # refresh
    pctrl = _StubController(Stage.GLIDING)
    staged = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'pitch': {'kp': 1.0}},
                                      'stages': {'gliding': {'pitch': 0}, 'landing': {'pitch': 0}}}, pctrl)
    assert await staged.setup() is True
    staged._tick()  # gliding -> engage
    assert staged._active is True and staged._stage == Stage.GLIDING
    pctrl.stage = Stage.LANDING
    staged._tick()  # still a control stage -> stays engaged (no neutral between)
    assert staged._active is True and staged._stage == Stage.LANDING
    # pitch=-5, landing setpoint 0 -> error 5 -> kp=1 -> elevons 90+5 (controlling, not neutral)
    assert pctrl.fins['servo_eleron_left'].angle == 95 and pctrl.fins['servo_eleron_right'].angle == 95
    pctrl.stage = Stage.BOOSTING
    staged._tick()  # non-control stage -> fins neutral, disengaged
    assert all(fin.angle == 90 for fin in pctrl.fins.values()) and staged._active is False

    """
    bank-to-turn WIRING through the whole pipeline: a heading error becomes a BANK (guidance) and
    the roll PID drives the elevons differentially. The tier/cache/bank law details are
    test_guidance.py's; here the databoard fix must reach it through the running task.
    """
    position = databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': 1000}}, 'position')
    position.push((48.0005, 10.990))   # west of the zone -> steer ~east (90) to the left gate
    attitude.push((0.0, 0, 0))     # facing north, wings level -> a +90 heading error
    bank_ctrl = _StubController(Stage.GLIDING)
    bankflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'roll': {'kp': 1.0}},
                                          'nav_bank_gain': 1.5, 'bank_limit': 30}, bank_ctrl)
    assert await bankflight.setup() is True
    bankflight._guidance._mission = _StubMission(launch=None)  # zone present -> tier-1 live fix
    bankflight._tick()
    # error +90 -> bank_demand(+90, 1.5, 30) = +30 -> roll PID (kp 1) -> elevons 90+/-30 (a right bank)
    assert bank_ctrl.fins['servo_eleron_left'].angle == 120 and bank_ctrl.fins['servo_eleron_right'].angle == 60
    assert bank_ctrl.fins['servo_eleron_left'].angle != bank_ctrl.fins['servo_eleron_right'].angle  # banked

    # crosswind landing: LANDING keeps steering to the zone (not a blind wings-level flare), using
    # the FULL fin authority (land_bank_limit 45) to crab the crosswind out -- keep it gliding.
    land_ctrl = _StubController(Stage.LANDING)
    landflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'roll': {'kp': 1.0}},
                                          'stages': {'gliding': {}, 'landing': {}}}, land_ctrl)
    assert await landflight.setup() is True
    landflight._guidance._mission = _StubMission(launch=None)
    position.push((48.0005, 10.990))   # west of the zone -> steer ~east (90); agl absent -> not 'final'
    attitude.push((0.0, 0, 0))
    landflight._tick()
    # error +90 -> bank_demand(+90, land_bank_gain 1.5, land_bank_limit 45) = +45 -> elevons 90+/-45 (full)
    assert land_ctrl.fins['servo_eleron_left'].angle == 135 and land_ctrl.fins['servo_eleron_right'].angle == 45

    """
    integer-degree heading error quantises the yaw D-term -- characterise the on-device impact.
    A smooth turn feeds a kd-only PID (its step() output IS the D term). Float wrap gives a smooth
    de/dt ~= the turn rate; the production int wrap holds flat then jumps a whole degree, so the D term
    spikes to ~1deg/dt at each integer crossing -- larger peaks, but bounded and sparse.
    """
    dt_ms = 10  # 100 Hz
    sweep = [30.0 - 0.27 * i for i in range(80)]  # heading error sweeping smoothly (~27 deg/s turn)

    def peak_dterm(wrap):
        controller = pid.Pid(kd=1.0)  # step(): fixnum error in, fixnum out -> / fixed.SCALE back to deg/s
        return max(abs(controller.step(fixed.from_float(wrap(e)), dt_ms)) for e in sweep) / fixed.SCALE

    peak_float = peak_dterm(lambda e: ((e + 180.0) % 360.0) - 180.0)  # smooth (float) heading error
    peak_int = peak_dterm(lambda e: guidance.heading_error(e, 0.0))  # the production int wrap
    print('yaw D-term peak over a smooth ~27deg/s turn @100Hz -- float=%.0f, int=%.0f deg/s'
          % (peak_float, peak_int))
    assert abs(peak_float - 27) < 2          # float: ~ the turn rate, no quantisation
    assert peak_int >= 1000 / dt_ms - 1      # int: spikes of ~1deg/dt (~100 deg/s) at degree crossings
    """
    Verdict: the spike is bounded by one degree-per-tick. With yaw kd kept small (sub-degree heading
    precision is irrelevant for fin authority over a 100-200 m approach) it is negligible; if a large
    kd is ever needed, switch the yaw error to float or low-pass the D term.
    """

    """
    dynamic-pressure fin governor WIRING: the governor caps mixer.limit EVERY step (even in a
    non-control stage). The schedule/throttle/override logic is test_governor.py's; here the
    databoard accel channel and the step pipeline must reach it through the running task.
    """
    gov = flight.Flight('flight', {'schedule_hz': 0, 'gains': {}}, _StubController(Stage.SETTING))
    assert await gov.setup() is True
    accel = databoard.Databoard.provide('accel_gov', {'accel': {'priority': 0, 'timeout_ms': 1000}}, 'accel')
    accel.push((0.0, 0.0, 1.0))  # exactly 1 g -> net accel 0 -> predict() is a no-op, value() = what we set
    gov._governor._estimator._speed = 0.0
    gov._tick()
    assert gov._mixer.limit == 45  # 0 m/s -> full 45 deg authority (and SETTING still ran the governor)
    gov._governor._estimator._speed = 40.0
    gov._tick()
    assert gov._mixer.limit == 8  # fin_deflection_limit(40) -> 8 deg
    # the accel channel feeds the integral through the databoard handle: >1 g builds airspeed from zero
    gov._governor._estimator._speed = 0.0
    accel.push((0.0, 0.0, 6.0))  # 6 g -> ~49 m/s^2 net along the path
    gov._tick()
    assert gov._governor.airspeed() > 0.0  # integrated off zero

    # airspeed governor THROTTLE wiring in the glide: the fresh pitch (dive detector) measured by the
    # PREVIOUS step must reach the governor's full-rate override on the next one.
    glide_ctrl = _StubController(Stage.GLIDING)
    thr = flight.Flight('flight', {'schedule_hz': 0, 'gains': {}, 'stages': {'gliding': {}},
                        'airspeed_dive_pitch': -45.0}, glide_ctrl)
    assert await thr.setup() is True
    attitude.push((0.0, 0, fixed.from_float(-60)))  # a steep nose-down the next step must pick up
    accel.push((0.0, 0.0, 1.0))  # 1 g -> net 0 -> predict() no-op
    thr._governor._interval_s = 1.0  # long throttle interval -> only an override can fire the update
    thr._governor._estimator._speed = 14.0  # settled low speed -> no absolute-speed override
    thr._tick()  # reads the dive pitch into _pitch_cd (this step may still be throttled)
    thr._governor._accum_s = 0.5
    thr._tick()
    assert thr._governor._accum_s == 0.0, 'the measured dive pitch must re-arm the governor full rate'

    # boost stage: BOOSTING is a control stage that holds the captured rod-vertical attitude, but only
    # PAST THE ROD (airspeed > boost_engage); below it the fins stay neutral (the rod holds it vertical).
    boost_ctrl = _StubController(Stage.BOOSTING)
    boostflight = flight.Flight('flight', {'schedule_hz': 0, 'boost_engage_speed': 15.0,
                                           'gains': {'roll': {'kp': 1.0}, 'pitch': {'kp': 1.0}},
                                           'stages': {'boosting': {}, 'gliding': {}}}, boost_ctrl)
    assert await boostflight.setup() is True
    accel.push((0.0, 0.0, 1.0))  # 1 g -> net 0 -> predict() no-op so the poked airspeed survives
    attitude.push((0.0, 0, fixed.from_float(90)))  # vertical on the rod (heading 0, roll 0, pitch 90)
    boostflight._governor._estimator._speed = 5.0  # still on the rod (below boost_engage)
    boostflight._tick()
    assert all(fin.angle == 90 for fin in boost_ctrl.fins.values())  # rod gate -> neutral
    assert boostflight._guidance._pitch_hold == fixed.from_float(90)  # captured the vertical hold
    assert boostflight._guidance._roll_hold == 0
    # past the rod + leaned 10 deg off vertical -> elevons deflect to restore pitch toward the hold
    boostflight._governor._estimator._speed = 30.0
    attitude.push((0.0, 0, fixed.from_float(80)))
    boostflight._tick()
    # pitch error = hold(90) - 80 = +10 -> kp 1 -> pitch_cmd 10 -> elevons 90+10, capped by the governor
    assert boost_ctrl.fins['servo_eleron_left'].angle == 100 and boost_ctrl.fins['servo_yaw'].angle == 90

    """
    crash safety: an uncaught exception inside the control step must CENTRE the fins on the
    way out (run()'s finally -> finish()), never leave the last deflection standing through the
    watchdog window.
    """
    crash_ctrl = _StubController(Stage.GLIDING)
    crashing = flight.Flight('flight', {'schedule_hz': 0, 'period_ms': 10,
                                        'gains': {'roll': {'kp': 1.0}}}, crash_ctrl)
    assert await crashing.setup() is True
    attitude.push((100.0, fixed.from_float(10), fixed.from_float(-5)))
    crashing._tick()  # engage: the elevons leave neutral
    assert crash_ctrl.fins['servo_eleron_left'].angle != 90
    crashing._guidance = None  # poison the pipeline -> the next step raises AttributeError
    try:
        await crashing.run()  # awaited directly (a bg task would print 'exception wasn't retrieved')
    except AttributeError:
        pass  # the run loop died on the poisoned step...
    assert all(fin.angle == 90 for fin in crash_ctrl.fins.values())  # ...and centred the fins going out

    print('ok: flight -- control stages, degraded->neutral, PID->mix->fins, governor + guidance wiring, '
          'boost hold, scheduling, crash->neutral')


asyncio.run(amain())
