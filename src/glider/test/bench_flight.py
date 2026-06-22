# bench_flight.py — flight-loop load sweep (run ON the board: `make bench-flight`, or `mpremote run`).
# Runs tasks/flight.py at schedule_hz 0 (asyncio) / 50 / 100 / 200 (timer), forced into GLIDING with a
# synthetic ~140 Hz attitude and gains 0 (each step does the full read->PID->mix->apply, but the fins
# hold neutral -> no motion). Reports achieved Hz, per-step latency (worst max_step_us), and CPU load
# (from a free-running idle counter vs a no-flight baseline). Needs the firmware deployed (run
# `make test` or ../deploy.sh first). Results live in doc/plan.md (Phase 3 task 2).

import asyncio
import time

import config_default
import databoard
from tasks import flight


class FakeFin:
    def update(self, props):
        pass


class Ctrl:
    config = config_default.default()

    def __init__(self):
        self._fins = {n: FakeFin() for n in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}

    def stage_name(self):
        return 'gliding'

    def find(self, names):
        return [self._fins.get(n) for n in names]


_idle = [0]


async def _idler():
    while True:
        _idle[0] += 1
        await asyncio.sleep_ms(0)


async def _pusher(channel):
    while True:
        channel.push((100.0, 5.0, -3.0))
        await asyncio.sleep_ms(7)  # ~140 Hz -> always fresh for the control loop


async def _window(ms):
    start, t0 = _idle[0], time.ticks_ms()
    await asyncio.sleep_ms(ms)
    return _idle[0] - start, time.ticks_diff(time.ticks_ms(), t0)


async def main():
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 500}}, 'attitude')
    asyncio.create_task(_idler())
    asyncio.create_task(_pusher(attitude))
    await asyncio.sleep_ms(300)

    base_idle, base_ms = await _window(2000)
    base_rate = base_idle * 1000 / base_ms
    print('baseline idle rate (no flight): %.0f /s\n' % base_rate)
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


asyncio.run(main())
