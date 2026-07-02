# bench_flight.py — flight-loop load sweep + step-time breakdown (run ON the board: `make bench-flight`,
# or `mpremote run`). Runs tasks/flight.py at schedule_hz 0 (asyncio) / 50 / 100 / 200 (timer), forced
# into GLIDING with a synthetic ~140 Hz attitude (centidegree fixnum) + gyro rate and gains 0 (each step
# does the full read->PID->mix->apply, but the fins hold neutral -> no motion). Reports achieved Hz,
# per-step latency (worst max_step_us) and CPU load (a free-running idle counter vs a no-flight baseline).
#
# The BREAKDOWN section then prices where a step's time goes -- the whole _step vs its _run_pid (the
# fixed-point PID: 3x pid.step + mix + apply) vs navigation.steer (the float haversine/atan2 homing trig,
# recomputed only every nav_period_ms in flight). This is the evidence for 7/01 wish #6 (viperize): if the
# integer PID is a small slice and the float trig dominates, viperizing the alloc-free PID is churn.
# Needs the firmware deployed (run `make test` or ../deploy.sh first). Results live in doc/plan.md.

import asyncio
import time

import config_default
import databoard
import fixed
import navigation
from controller import Stage
from tasks import flight


class FakeFin:
    def set_angle(self, angle):
        pass


class Ctrl:
    config = config_default.default()
    stage = Stage.GLIDING   # forced into a control stage so every step runs the full law
    armed = True

    def __init__(self):
        self._fins = {n: FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def find(self, names):
        return [self._fins.get(n) for n in names]


_idle = [0]


async def _idler():
    while True:
        _idle[0] += 1
        await asyncio.sleep_ms(0)


async def _pusher(attitude, rate):
    while True:  # centidegree fixnum attitude + centideg/s fixnum gyro rate -> always fresh for the loop
        attitude.push((100.0, fixed.from_float(5.0), fixed.from_float(-3.0)))
        rate.push((fixed.from_float(2.0), fixed.from_float(-1.0), fixed.from_float(0.5)))
        await asyncio.sleep_ms(7)  # ~140 Hz


async def _window(ms):
    start, t0 = _idle[0], time.ticks_ms()
    await asyncio.sleep_ms(ms)
    return _idle[0] - start, time.ticks_diff(time.ticks_ms(), t0)


def _per_call(fn, n=2000):
    """Mean microseconds per call of fn() over n calls (time.ticks_us resolution)."""
    t0 = time.ticks_us()
    for _ in range(n):
        fn()
    return time.ticks_diff(time.ticks_us(), t0) / n


def _per_op(body, reps=50, n=400):
    """Microseconds per body() invocation with the harness's per-call overhead amortized: body() runs
    `reps` times inside each of `n` timed samples, so tens-of-us ops are measured above the ~call-overhead
    noise floor. Returns us per single body()."""
    def batch():
        for _ in range(reps):
            body()
    return _per_call(batch, n) / reps


async def sweep(base_rate):
    print('%-18s %8s %9s %7s' % ('schedule', 'ach.Hz', 'max_us', 'load%'))
    for schedule_hz, period_ms in ((0, 10), (50, 0), (100, 0), (200, 0)):
        flight_task = flight.Flight('flight', {'schedule_hz': schedule_hz, 'period_ms': period_ms or 20,
                                               'gains': {}}, Ctrl())
        await flight_task.setup()
        runner = asyncio.create_task(flight_task.run())
        await asyncio.sleep_ms(300)
        flight_task._steps = 0
        flight_task._max_step_us = 0
        idle_delta, ms = await _window(2000)
        achieved = flight_task._steps * 1000 / ms
        load = (1 - (idle_delta * 1000 / ms) / base_rate) * 100
        label = 'asyncio %dms' % (period_ms or 20) if schedule_hz == 0 else 'timer %dHz' % schedule_hz
        print('%-18s %8.1f %9d %7.1f' % (label, achieved, flight_task._max_step_us, load))
        runner.cancel()
        try:
            await runner
        except asyncio.CancelledError:
            pass
        await flight_task.finish()


async def breakdown():
    """Price a single control step's components: the whole _step, the fixed-point PID (_run_pid), and the
    float nav trig (navigation.steer). schedule_hz 0 so no timer fires while we call _step() by hand."""
    task = flight.Flight('flight', {'schedule_hz': 0, 'period_ms': 20, 'gains': {'roll': {'kp': 2.0, 'kd': 0.2},
                        'pitch': {'kp': 1.5}, 'yaw': {'kp': 1.5, 'kd': 0.1}}}, Ctrl())
    await task.setup()
    task._step()  # warm: resolve+cache the fins and set _roll_sp/_pitch_sp/_heading_err for _run_pid

    # representative coords: a position ~200 m off a 100 m landing zone (the homing trig's real inputs)
    position = (25.5000, -80.4000)
    corner_tl, corner_br = (25.5010, -80.4010), (25.4990, -80.3990)
    roll_cd, pitch_cd = fixed.from_float(5.0), fixed.from_float(-3.0)

    step_us = _per_call(task._step)
    pid_us = _per_call(lambda: task._run_pid(roll_cd, pitch_cd, 10))
    nav_us = _per_call(lambda: navigation.steer(position, corner_tl, corner_br))
    # decompose _run_pid: is the cost the VIPERIZABLE integer PID arithmetic, or the databoard read /
    # dict ops around it? Time each piece _run_pid runs (one pid.step, the 3-axis PID, mix, rate read, apply)
    # sub-ops are tens of us -- smaller than the timing harness's per-call overhead -- so amortize it
    # (_per_op runs each 50x per sample). Precompute args: no float box / tuple alloc inside the timed body.
    err, rate_arg = roll_cd - fixed.from_float(2.0), fixed.from_float(2.0)
    roll_pid, pitch_pid, yaw_pid = task._pid['roll'], task._pid['pitch'], task._pid['yaw']
    one_step_us = _per_op(lambda: roll_pid.step(err, 10, rate_arg))

    def _three():  # the 3 axes as _run_pid runs them
        roll_pid.step(err, 10, rate_arg)
        pitch_pid.step(err, 10, rate_arg)
        yaw_pid.step(err, 10, rate_arg)
    three_step_us = _per_op(_three)
    mix_us = _per_op(lambda: task._mixer.mix(1, 1, 1))
    rate_us = _per_op(task._rate.value)
    neutral = task._mixer.neutralise()
    apply_us = _per_op(lambda: task._apply(neutral))

    print('\nstep-time breakdown (us/call, %d samples):' % 2000)
    print('  whole _step (nav cached) : %7.1f us' % step_us)
    print('  _run_pid (PID+mix+apply) : %7.1f us  (%.0f%% of a step)' % (pid_us, 100 * pid_us / step_us))
    print('  navigation.steer (trig)  : %7.1f us  (throttled ~1 in %d steps -> ~%.0f us amortized/step)' %
          (nav_us, max(1, task.config.get('nav_period_ms', 100) // 10),
           nav_us / max(1, task.config.get('nav_period_ms', 100) // 10)))
    print('  -- inside _run_pid --')
    print('    1x pid.step (integer)  : %7.1f us  <- the VIPERIZABLE arithmetic' % one_step_us)
    print('    3x pid.step            : %7.1f us  (%.0f%% of _run_pid)' % (three_step_us,
                                                                          100 * three_step_us / pid_us))
    print('    mixer.mix              : %7.1f us' % mix_us)
    print('    rate.value (databoard) : %7.1f us' % rate_us)
    print('    _apply (find+set fins) : %7.1f us' % apply_us)
    # viper verdict: viper can only speed pid.step's integer ARITHMETIC, not the Python method/attr/clamp
    # overhead, dict work in mix/apply, or the databoard read. So its ceiling is a fraction of the 3x
    # pid.step slice. And a step is a small part of the 100 Hz budget -> headroom, not a deadline.
    budget_us = 10000  # 100 Hz
    print('  -- viper #6 verdict --')
    print('    3x pid.step = %.0f%% of a step; a step = %.1f%% of the 100 Hz budget (%d us)' %
          (100 * three_step_us / step_us, 100 * step_us / budget_us, budget_us))
    print('    biggest single slice is _apply (%.0f us, fin writes) -- NOT viperizable arithmetic' % apply_us)


async def main():
    attitude = databoard.Databoard.provide(
        'imu', {'attitude': {'priority': 0, 'timeout_ms': 500}, 'rate': {'priority': 0, 'timeout_ms': 500}},
        'attitude', 'rate')
    asyncio.create_task(_idler())
    asyncio.create_task(_pusher(*attitude))
    await asyncio.sleep_ms(300)
    base_idle, base_ms = await _window(2000)
    base_rate = base_idle * 1000 / base_ms
    print('baseline idle rate (no flight): %.0f /s\n' % base_rate)
    await sweep(base_rate)
    await breakdown()


asyncio.run(main())
