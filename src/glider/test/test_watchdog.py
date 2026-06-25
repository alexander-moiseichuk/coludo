# On-board test for the watchdog/heartbeat supervisor (tasks/watchdog.py): the control-loop stall
# decision (now TIME-based, via the flight progress() timestamp) and that run() feeds an (injected)
# WDT while healthy but resets on a stall. A stub WDT + reset are injected so nothing actually resets
# the board. Run by `make test`.

import asyncio
import time

import config_default
import task
from tasks import watchdog


class StubWDT:
    def __init__(self):
        self.feeds = 0

    def feed(self):
        self.feeds += 1


class FakeFlight:
    def __init__(self):
        self._active = False
        self._steps = 0
        self._stage = None
        self._updated_us = time.ticks_us()

    def step(self):  # a healthy control update: advance steps + stamp the time
        self._steps += 1
        self._updated_us = time.ticks_us()

    def progress(self):  # the public heartbeat the watchdog reads (3.6.1): (active, steps, stage, updated_us)
        return self._active, self._steps, self._stage, self._updated_us


class _StubController:
    config = config_default.default()

    def __init__(self, flight):
        self._flight = flight

    def find(self, names):
        return [self._flight]

    def stage_name(self):  # the watchdog logs this in the stall message (int-stage refactor)
        return 'gliding'


async def amain():
    assert task.ACTIVITIES.get('watchdog') is watchdog.Watchdog  # registered driver

    flight = FakeFlight()
    wd = watchdog.Watchdog('watchdog', {'period_ms': 5, 'wdt_timeout_ms': 2000, 'stall_ms': 20},
                           _StubController(flight))
    assert await wd.setup() is True
    resets = []
    wd._wdt = StubWDT()
    wd._reset = lambda: resets.append(1)

    # not in a control stage -> nothing to supervise (never "stalled")
    flight._active = False
    assert wd._stalled(flight) is False
    assert wd._stalled(None) is False  # disabled flight task

    # in a control stage + a fresh update -> healthy
    flight._active = True
    flight.step()
    assert wd._stalled(flight) is False
    # in a control stage but the last step is older than stall_ms -> stalled
    flight._updated_us = time.ticks_add(time.ticks_us(), -50000)  # 50 ms ago > stall_ms 20
    assert wd._stalled(flight) is True

    # run(): a healthy (stepping) control loop -> WDT fed, no reset
    flight._active = True
    flight.step()
    ticker = asyncio.create_task(_tick(flight))
    runner = asyncio.create_task(wd.run())
    await asyncio.sleep_ms(40)
    assert wd._wdt.feeds > 0 and resets == []

    # stall it: the ticker stops -> the timestamp goes stale -> full reset
    ticker.cancel()
    try:
        await ticker
    except asyncio.CancelledError:
        pass
    await asyncio.sleep_ms(40)  # > stall_ms since the last step
    assert resets == [1]  # reset fired on the stall (run() returned after)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass

    print('ok: watchdog -- time-based stall decision, WDT fed when healthy, control-loop stall -> reset')


async def _tick(flight):
    while True:
        flight.step()
        await asyncio.sleep_ms(1)


asyncio.run(amain())
