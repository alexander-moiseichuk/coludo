# On-board test for the Phase 3 stabilization loop (tasks/flight.py): registration, GLIDING gating,
# degraded->neutral on stale attitude, the PID->mixer->fin path, and both scheduling modes (asyncio at
# rate_hz=0, machine.Timer at rate_hz>0). Uses fake fins + a stub controller; attitude comes from the
# databoard. Run by `make test`.

import asyncio

import config_default
import databoard
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
        self.fins = {n: _FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def stage_name(self):
        return self._stage

    def find(self, names):
        return [self.fins.get(n) for n in names]


async def amain():
    assert task.ACTIVITIES.get('flight') is flight.Flight  # registered driver

    ctrl = _StubController('setting')
    unit = flight.Flight('flight', {'rate_hz': 0, 'period_ms': 20, 'gains': {'roll': {'kp': 1.0}}}, ctrl)
    assert await unit.setup() is True
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 1000}}, 'attitude')

    # not gliding -> the loop is gated, no actuation
    unit._step()
    assert ctrl.fins['servo_yaw'].angle is None

    # gliding but attitude born-stale (not pushed) -> degraded -> fins neutral, not engaged
    ctrl._stage = 'gliding'
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._gliding is False

    # gliding + fresh attitude -> engage: kp=1 on roll=10 -> roll cmd -10 -> elevons differential
    attitude.push((100.0, 10.0, -5.0))  # heading, roll, pitch
    unit._step()
    assert unit._gliding is True and unit._steps == 1
    assert ctrl.fins['servo_eleron_left'].angle == 80 and ctrl.fins['servo_eleron_right'].angle == 100
    assert ctrl.fins['servo_yaw'].angle == 90  # heading hold captured at 100 -> error 0 -> rudder neutral

    # leaving gliding -> centre the fins + drop the glide state
    ctrl._stage = 'landing'
    unit._step()
    assert all(fin.angle == 90 for fin in ctrl.fins.values()) and unit._gliding is False

    # asyncio mode (rate_hz 0): run() loops and runs control steps
    ctrl._stage = 'gliding'
    runner = asyncio.create_task(unit.run())
    await asyncio.sleep_ms(80)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    assert unit._steps > 1  # the asyncio loop ticked

    # timer mode (rate_hz > 0): a machine.Timer drives the step deterministically
    timed = flight.Flight('flight', {'rate_hz': 100, 'gains': {}}, _StubController('gliding'))
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

    print('ok: flight -- gliding gating, degraded->neutral, PID->mix->fins, asyncio + timer scheduling')


asyncio.run(amain())
