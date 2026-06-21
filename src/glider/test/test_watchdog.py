# On-board test for the watchdog/heartbeat supervisor (tasks/watchdog.py): the control-loop stall
# decision and that run() feeds an (injected) WDT while healthy but resets on a stall. A stub WDT +
# reset are injected so nothing actually resets the board. Run by `make test`.

import asyncio

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
        self._phase = None


class _StubController:
    config = config_default.default()

    def __init__(self, flight):
        self._flight = flight

    def find(self, names):
        return [self._flight]


async def amain():
    assert task.ACTIVITIES.get('watchdog') is watchdog.Watchdog  # registered driver

    flight = FakeFlight()
    wd = watchdog.Watchdog('watchdog', {'period_ms': 5, 'wdt_timeout_ms': 2000}, _StubController(flight))
    assert await wd.setup() is True
    resets = []
    wd._wdt, wd._reset = StubWDT(), lambda: resets.append(1)

    # not in a control phase -> nothing to supervise (never "stalled")
    flight._active = False
    assert wd._stalled(flight) is False

    # in a control phase + the step counter advancing -> healthy
    flight._active, flight._steps = True, 10
    assert wd._stalled(flight) is False  # 0 -> 10
    flight._steps = 20
    assert wd._stalled(flight) is False  # 10 -> 20
    # in a control phase + NOT advancing -> stalled
    assert wd._stalled(flight) is True  # 20 == 20

    # run(): a healthy (advancing) control loop -> WDT fed, no reset
    flight._active, flight._steps = True, 0
    ticker = asyncio.create_task(_tick(flight))
    runner = asyncio.create_task(wd.run())
    await asyncio.sleep_ms(40)
    assert wd._wdt.feeds > 0 and resets == []

    # stall it: the ticker stops -> the next heartbeat sees no progress -> full reset
    ticker.cancel()
    try:
        await ticker
    except asyncio.CancelledError:
        pass
    await asyncio.sleep_ms(30)
    assert resets == [1]  # reset fired on the stall (run() returned after)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass

    print('ok: watchdog -- stall decision, WDT fed when healthy, control-loop stall -> reset')


async def _tick(flight):
    while True:
        flight._steps += 1
        await asyncio.sleep_ms(1)


asyncio.run(amain())
