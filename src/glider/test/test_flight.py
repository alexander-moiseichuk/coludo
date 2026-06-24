# On-board test for the Phase 3 stabilization loop (tasks/flight.py): registration, GLIDING gating,
# degraded->neutral on stale attitude, the PID->mixer->fin path, and both scheduling modes (asyncio at
# schedule_hz=0, machine.Timer at schedule_hz>0). Uses fake fins + a stub controller; attitude comes from the
# databoard. Run by `make test`.

import asyncio

import config_default
import databoard
import pid
import task
from tasks import flight


class _FakeFin:
    def __init__(self):
        self.angle = None

    def update(self, props):
        self.angle = props['angle']


class _StubController:
    def __init__(self, stage):
        self.config = config_default.default()  # carries the mixer block
        self._stage = stage
        self.armed = True  # the gate: disarmed -> the loop holds neutral (tested below)
        self.fins = {n: _FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def stage_name(self):
        return self._stage

    def find(self, names):
        return [self.fins.get(n) for n in names]


async def amain():
    assert task.ACTIVITIES.get('flight') is flight.Flight  # registered driver

    ctrl = _StubController('setting')
    unit = flight.Flight('flight', {'schedule_hz': 0, 'period_ms': 20, 'gains': {'roll': {'kp': 1.0}}}, ctrl)
    assert await unit.setup() is True
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 1000}}, 'attitude')

    # not gliding -> the loop is gated, no actuation
    unit._step()
    assert ctrl.fins['servo_yaw'].angle is None

    # gliding but attitude born-stale (not pushed) -> degraded -> fins neutral, not engaged
    ctrl._stage = 'gliding'
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # gliding + fresh attitude -> engage: kp=1 on roll=10 -> roll cmd -10 -> elevons differential
    attitude.push((100.0, 10.0, -5.0))  # heading, roll, pitch
    unit._step()
    assert unit._active is True and unit._stage == 'gliding' and unit._steps == 1
    assert ctrl.fins['servo_eleron_left'].angle == 80 and ctrl.fins['servo_eleron_right'].angle == 100
    assert ctrl.fins['servo_yaw'].angle == 90  # heading hold captured at 100 -> error 0 -> rudder neutral

    # landing is NOT a control stage by default (only gliding) -> centre the fins + disengage
    ctrl._stage = 'landing'
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False

    # disarmed -> neutral even in a control stage (the arming safety gate)
    ctrl._stage = 'gliding'
    ctrl.armed = False
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._active is False
    ctrl.armed = True  # re-arm for the scheduling-mode checks below

    # asyncio mode (schedule_hz 0): run() loops and runs control steps
    ctrl._stage = 'gliding'
    runner = asyncio.create_task(unit.run())
    await asyncio.sleep_ms(80)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    assert unit._steps > 1  # the asyncio loop ticked

    # timer mode (schedule_hz > 0): a machine.Timer drives the step deterministically
    timed = flight.Flight('flight', {'schedule_hz': 100, 'gains': {}}, _StubController('gliding'))
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
    pctrl = _StubController('gliding')
    staged = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'pitch': {'kp': 1.0}},
                                      'stages': {'gliding': {'pitch': 0}, 'landing': {'pitch': 0}}}, pctrl)
    assert await staged.setup() is True
    staged._step()  # gliding -> engage
    assert staged._active is True and staged._stage == 'gliding'
    pctrl._stage = 'landing'
    staged._step()  # still a control stage -> stays engaged (no neutral between)
    assert staged._active is True and staged._stage == 'landing'
    # pitch=-5, landing setpoint 0 -> error 5 -> kp=1 -> elevons 90+5 (controlling, not neutral)
    assert pctrl.fins['servo_eleron_left'].angle == 95 and pctrl.fins['servo_eleron_right'].angle == 95
    pctrl._stage = 'boosting'
    staged._step()  # non-control stage -> fins neutral, disengaged
    assert all(fin.angle == 90 for fin in pctrl.fins.values()) and staged._active is False

    # landing-zone nav, three GPS-degrading tiers of the yaw heading setpoint (GLIDING only)
    class _StubMission:
        zone = ((48.001, 11.000), (48.000, 11.010))  # longitude-stretched -> gates on the left/right edges

        def __init__(self, launch=None):
            self._launch = launch

        def launch_point(self):
            return self._launch

    nav_ctrl = _StubController('gliding')
    navflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {}}, nav_ctrl)
    assert await navflight.setup() is True
    navflight._heading_hold = 200.0  # the blind fallback heading

    # tier 3: no fix, no launch point -> hold the captured heading (blind)
    navflight._mission = _StubMission(launch=None)
    assert navflight._target_heading() == 200.0
    # tier 2: no fix, CC-set launch point (west of the zone) -> launch->left-gate bearing (~east, 90)
    navflight._mission = _StubMission(launch=(48.0005, 10.990))
    assert abs(navflight._target_heading() - 90.0) < 5.0
    # tier 1: a fresh fix overrides -> steer from the CURRENT position (east of the zone -> right gate, ~270)
    position = databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': 1000}}, 'position')
    position.push((48.0005, 11.020))
    assert abs(navflight._target_heading() - 270.0) < 5.0  # current position, not the launch point
    nav_ctrl._stage = 'landing'  # nav steers only in GLIDING -> LANDING holds (straight-and-level)
    assert navflight._target_heading() == 200.0

    # bank-to-turn: in GLIDING a heading error commands a BANK (roll setpoint = nav_bank_gain*error,
    # capped at bank_limit), so the glider banks into the turn (differential elevons) instead of only
    # yawing -- the fix for over-ranging the zone on a flat rudder skid.
    position.push((48.0005, 10.990))   # west of the zone -> steer ~east (90) to the left gate
    attitude.push((0.0, 0.0, 0.0))     # facing north, wings level -> a +90 heading error
    bank_ctrl = _StubController('gliding')
    bankflight = flight.Flight('flight', {'schedule_hz': 0, 'gains': {'roll': {'kp': 1.0}},
                                          'nav_bank_gain': 1.5, 'bank_limit': 30}, bank_ctrl)
    assert await bankflight.setup() is True
    bankflight._mission = _StubMission(launch=None)  # zone present -> tier-1 uses the live fix above
    bankflight._step()
    # error +90 -> bank_demand(+90, 1.5, 30) = +30 -> roll PID (kp 1) -> elevons 90+/-30 (a right bank)
    assert bank_ctrl.fins['servo_eleron_left'].angle == 120 and bank_ctrl.fins['servo_eleron_right'].angle == 60
    assert bank_ctrl.fins['servo_eleron_left'].angle != bank_ctrl.fins['servo_eleron_right'].angle  # banked

    # g6: integer-degree heading error quantises the yaw D-term -- characterise the on-device impact.
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
    print('g6: yaw D-term peak over a smooth ~27deg/s turn @100Hz -- float=%.0f, int=%.0f deg/s'
          % (peak_float, peak_int))
    assert abs(peak_float - 27) < 2          # float: ~ the turn rate, no quantisation
    assert peak_int >= 1.0 / dt - 1          # int: spikes of ~1deg/dt (~100 deg/s) at degree crossings
    # Verdict: the spike is bounded by one degree-per-tick. With yaw kd kept small (sub-degree heading
    # precision is irrelevant for fin authority over a 100-200 m approach) it is negligible; if a large
    # kd is ever needed, switch the yaw error to float or low-pass the D term.

    print('ok: flight -- per-stage control stages, nav (3 GPS tiers), degraded->neutral, PID->mix->fins, scheduling')


asyncio.run(amain())
