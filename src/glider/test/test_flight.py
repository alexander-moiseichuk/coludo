# On-board test for the Phase 3 stabilization loop (tasks/flight.py): registration, GLIDING gating,
# degraded->neutral on stale attitude, the PID->mixer->fin path, and both scheduling modes (asyncio at
# schedule_hz=0, machine.Timer at schedule_hz>0). Uses fake fins + a stub controller; attitude comes from the
# databoard. Run by `make test`.

import asyncio

import config_default
import databoard
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
        self.config = config_default.default()  # carries the mixer block
        self.stage = stage  # a Stage id (int) -- the flight loop reads controller.stage, not strings
        self.armed = True  # the gate: disarmed -> the loop holds neutral (tested below)
        self.fins = {n: _FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def stage_name(self):
        return Stage.STAGES[self.stage]

    def find(self, names):
        return [self.fins.get(n) for n in names]


async def amain():
    assert task.ACTIVITIES.get('flight') is flight.Flight  # registered driver

    ctrl = _StubController(Stage.SETTING)
    unit = flight.Flight('flight', {'schedule_hz': 0, 'period_ms': 20, 'gains': {'roll': {'kp': 1.0}}}, ctrl)
    assert await unit.setup() is True
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 1000}}, 'attitude')

    # not gliding -> the loop is gated, no actuation
    unit._step()
    assert ctrl.fins['servo_yaw'].angle is None

    # gliding but attitude born-stale (not pushed) -> degraded -> fins neutral, not engaged
    ctrl.stage = Stage.GLIDING
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # gliding + fresh attitude -> engage: kp=1 on roll=10 -> roll cmd -10 -> elevons differential
    attitude.push((100.0, 10.0, -5.0))  # heading, roll, pitch
    unit._step()
    assert unit._active is True and unit._stage == Stage.GLIDING and unit._steps == 1
    assert ctrl.fins['servo_eleron_left'].angle == 80 and ctrl.fins['servo_eleron_right'].angle == 100
    assert ctrl.fins['servo_yaw'].angle == 90  # heading hold captured at 100 -> error 0 -> rudder neutral

    # landing is NOT a control stage by default (only gliding) -> centre the fins + disengage
    ctrl.stage = Stage.LANDING
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # disarmed -> neutral even in a control stage (the arming safety gate)
    ctrl.stage = Stage.GLIDING
    ctrl.armed = False
    unit._step()
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
    attitude.push((100.0, 10.0, -5.0))  # refresh
    pctrl = _StubController(Stage.GLIDING)
    staged = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'pitch': {'kp': 1.0}},
                                      'stages': {'gliding': {'pitch': 0}, 'landing': {'pitch': 0}}}, pctrl)
    assert await staged.setup() is True
    staged._step()  # gliding -> engage
    assert staged._active is True and staged._stage == Stage.GLIDING
    pctrl.stage = Stage.LANDING
    staged._step()  # still a control stage -> stays engaged (no neutral between)
    assert staged._active is True and staged._stage == Stage.LANDING
    # pitch=-5, landing setpoint 0 -> error 5 -> kp=1 -> elevons 90+5 (controlling, not neutral)
    assert pctrl.fins['servo_eleron_left'].angle == 95 and pctrl.fins['servo_eleron_right'].angle == 95
    pctrl.stage = Stage.BOOSTING
    staged._step()  # non-control stage -> fins neutral, disengaged
    assert all(fin.angle == 90 for fin in pctrl.fins.values()) and staged._active is False

    # landing-zone nav, three GPS-degrading tiers of the yaw heading setpoint (GLIDING only)
    class _StubMission:
        zone = ((48.001, 11.000), (48.000, 11.010))  # longitude-stretched -> gates on the left/right edges

        def __init__(self, launch=None):
            self._launch = launch

        def launch_point(self):
            return self._launch

    nav_ctrl = _StubController(Stage.GLIDING)
    navflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {}}, nav_ctrl)
    assert await navflight.setup() is True
    navflight._heading_hold = 200.0  # the blind fallback heading
    # the cache holds the steer() result for nav_period_ms; this test changes the inputs faster than
    # that on purpose, so it clears _nav_heading before each tier to force a fresh recompute.

    # tier 3: no fix, no launch point -> hold the captured heading (blind)
    navflight._mission = _StubMission(launch=None)
    navflight._nav_heading = None
    assert navflight._target_heading(0.0, False) == 200.0
    # tier 2: no fix, CC-set launch point (west of the zone) -> launch->left-gate bearing (~east, 90)
    navflight._mission = _StubMission(launch=(48.0005, 10.990))
    navflight._nav_heading = None
    assert abs(navflight._target_heading(0.0, False) - 90.0) < 5.0
    # tier 1: a fresh fix overrides -> steer from the CURRENT position (east of the zone -> right gate, ~270)
    position = databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': 1000}}, 'position')
    position.push((48.0005, 11.020))
    navflight._nav_heading = None
    assert abs(navflight._target_heading(0.0, False) - 270.0) < 5.0  # current position, not the launch point
    # tier-1 freshness gate: a position_age_max_ms below the fix age skips tier 1 even on a LIVE fix,
    # falling to the launch-point bearing (tier 2). 0 ms rejects even a just-pushed fix (~270 -> ~90).
    navflight._position_age_max_ms = 0
    navflight._nav_heading = None
    assert abs(navflight._target_heading(0.0, False) - 90.0) < 5.0  # gated off tier-1 -> tier 2 launch bearing
    navflight._position_age_max_ms = max(navflight._position.window_us, navflight._gnss_speed.window_us) // 1000
    #/LANDING now STEERS too (it no longer locks straight-and-level) -> same tier-1 fix, ~270
    nav_ctrl.stage = Stage.LANDING
    navflight._nav_heading = None
    assert abs(navflight._target_heading(0.0, False) - 270.0) < 5.0
    # a NON-control stage (BOOSTING) holds the captured heading (blind)
    nav_ctrl.stage = Stage.BOOSTING
    navflight._nav_heading = None
    assert navflight._target_heading(0.0, False) == 200.0

    # a second call within nav_period returns the CACHED heading (no recompute) even if the fix moves
    nav_ctrl.stage = Stage.GLIDING
    navflight._nav_heading = None
    first = navflight._target_heading(0.0, False)           # fresh steer() from (48.0005, 11.020) -> ~270
    position.push((48.0005, 10.980))              # move the fix west; without the cache this -> ~90
    assert navflight._target_heading(0.0, False) == first   # cached -> unchanged until nav_period elapses

    # bank-to-turn: in GLIDING a heading error commands a BANK (roll setpoint = nav_bank_gain*error,
    # capped at bank_limit), so the glider banks into the turn (differential elevons) instead of only
    # yawing -- the fix for over-ranging the zone on a flat rudder skid.
    position.push((48.0005, 10.990))   # west of the zone -> steer ~east (90) to the left gate
    attitude.push((0.0, 0.0, 0.0))     # facing north, wings level -> a +90 heading error
    bank_ctrl = _StubController(Stage.GLIDING)
    bankflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'roll': {'kp': 1.0}},
                                          'nav_bank_gain': 1.5, 'bank_limit': 30}, bank_ctrl)
    assert await bankflight.setup() is True
    bankflight._mission = _StubMission(launch=None)  # zone present -> tier-1 uses the live fix above
    bankflight._step()
    # error +90 -> bank_demand(+90, 1.5, 30) = +30 -> roll PID (kp 1) -> elevons 90+/-30 (a right bank)
    assert bank_ctrl.fins['servo_eleron_left'].angle == 120 and bank_ctrl.fins['servo_eleron_right'].angle == 60
    assert bank_ctrl.fins['servo_eleron_left'].angle != bank_ctrl.fins['servo_eleron_right'].angle  # banked

    # crosswind landing: LANDING keeps steering to the zone (not a blind wings-level flare), using
    # the FULL fin authority (land_bank_limit 45) to crab the crosswind out -- keep it gliding.
    land_ctrl = _StubController(Stage.LANDING)
    landflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'roll': {'kp': 1.0}},
                                          'stages': {'gliding': {}, 'landing': {}}}, land_ctrl)
    assert await landflight.setup() is True
    landflight._mission = _StubMission(launch=None)
    position.push((48.0005, 10.990))   # west of the zone -> steer ~east (90); agl absent -> not 'final'
    attitude.push((0.0, 0.0, 0.0))
    landflight._step()
    # error +90 -> bank_demand(+90, land_bank_gain 1.5, land_bank_limit 45) = +45 -> elevons 90+/-45 (full)
    assert land_ctrl.fins['servo_eleron_left'].angle == 135 and land_ctrl.fins['servo_eleron_right'].angle == 45

    # integer-degree heading error quantises the yaw D-term -- characterise the on-device impact.
    # A smooth turn feeds a kd-only PID (its step() output IS the D term). Float wrap gives a smooth
    # de/dt ~= the turn rate; the production int wrap holds flat then jumps a whole degree, so the D term
    # spikes to ~1deg/dt at each integer crossing -- larger peaks, but bounded and sparse.
    dt = 0.01  # 100 Hz
    sweep = [30.0 - 0.27 * i for i in range(80)]  # heading error sweeping smoothly (~27 deg/s turn)

    def peak_dterm(wrap):
        controller = pid.Pid(kd=1.0)
        return max(abs(controller.step(wrap(e), dt)) for e in sweep)

    peak_float = peak_dterm(lambda e: ((e + 180.0) % 360.0) - 180.0)  # smooth (float) heading error
    peak_int = peak_dterm(lambda e: flight.Flight._heading_error(e, 0.0))  # the production int wrap
    print('yaw D-term peak over a smooth ~27deg/s turn @100Hz -- float=%.0f, int=%.0f deg/s'
          % (peak_float, peak_int))
    assert abs(peak_float - 27) < 2          # float: ~ the turn rate, no quantisation
    assert peak_int >= 1.0 / dt - 1          # int: spikes of ~1deg/dt (~100 deg/s) at degree crossings
    # Verdict: the spike is bounded by one degree-per-tick. With yaw kd kept small (sub-degree heading
    # precision is irrelevant for fin authority over a 100-200 m approach) it is negligible; if a large
    # kd is ever needed, switch the yaw error to float or low-pass the D term.

    # dynamic-pressure fin governor: the airspeed estimate caps mixer.limit EVERY step (even in a
    # non-control stage), scaled by fin_limit_multiplier. The estimator itself is covered by test_airspeed;
    # here we drive its value directly (a steady 1 g -> zero net accel -> predict adds nothing) and check
    # the cap wiring: commons.fin_deflection_limit(v) * multiplier -> mixer.limit.
    gov = flight.Flight('flight', {'schedule_hz': 0, 'gains': {}}, _StubController(Stage.SETTING))
    assert await gov.setup() is True
    accel = databoard.Databoard.provide('accel_gov', {'accel': {'priority': 0, 'timeout_ms': 1000}}, 'accel')
    accel.push((0.0, 0.0, 1.0))  # exactly 1 g -> net accel 0 -> predict() is a no-op, value() = what we set
    gov._airspeed._speed = 0.0
    gov._step()
    assert gov._mixer.limit == 45  # 0 m/s -> full 45 deg authority (and SETTING still ran the governor)
    gov._airspeed._speed = 40.0
    gov._step()
    assert gov._mixer.limit == 8  # fin_deflection_limit(40) -> 8 deg
    gov._fin_limit_multiplier = 0.5  # the safety dial halves the whole schedule
    gov._airspeed._speed = 0.0
    gov._step()
    assert gov._mixer.limit == 22  # int(45 * 0.5)
    # the accel channel feeds the integral: a sustained >1 g reading builds airspeed up from zero
    gov._fin_limit_multiplier = 1.0
    gov._airspeed._speed = 0.0
    accel.push((0.0, 0.0, 6.0))  # 6 g -> ~49 m/s^2 net along the path
    gov._step()
    assert gov._airspeed.value() > 0.0  # integrated off zero

    # boost stage: BOOSTING is a control stage that holds the captured rod-vertical attitude, but only
    # PAST THE ROD (airspeed > boost_engage); below it the fins stay neutral (the rod holds it vertical).
    boost_ctrl = _StubController(Stage.BOOSTING)
    boostflight = flight.Flight('flight', {'schedule_hz': 0, 'boost_engage_speed': 15.0,
                                           'gains': {'roll': {'kp': 1.0}, 'pitch': {'kp': 1.0}},
                                           'stages': {'boosting': {}, 'gliding': {}}}, boost_ctrl)
    assert await boostflight.setup() is True
    accel.push((0.0, 0.0, 1.0))  # 1 g -> net 0 -> predict() no-op so the poked airspeed survives
    attitude.push((0.0, 0.0, 90.0))  # vertical on the rod (heading 0, roll 0, pitch 90)
    boostflight._airspeed._speed = 5.0  # still on the rod (below boost_engage)
    boostflight._step()
    assert all(fin.angle == 90 for fin in boost_ctrl.fins.values())  # rod gate -> neutral
    assert boostflight._pitch_hold == 90.0 and boostflight._roll_hold == 0.0  # captured the vertical hold
    # past the rod + leaned 10 deg off vertical -> elevons deflect to restore pitch toward the hold
    boostflight._airspeed._speed = 30.0
    attitude.push((0.0, 0.0, 80.0))
    boostflight._step()
    # pitch error = hold(90) - 80 = +10 -> kp 1 -> pitch_cmd 10 -> elevons 90+10, capped by the governor
    assert boost_ctrl.fins['servo_eleron_left'].angle == 100 and boost_ctrl.fins['servo_yaw'].angle == 90

    print('ok: flight -- control stages, nav, degraded->neutral, PID->mix->fins, fin governor, boost hold, scheduling')


asyncio.run(amain())
