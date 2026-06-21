# On-board test for the board vitals task (tasks/board_health.py): @task.activity('health')
# registration, vitals sampling, telemetry (first row at startup), int load, and that the load
# estimate tracks real CPU load. Run by `make test`.

import asyncio

import config_default
import recorder
import task
from tasks import board_health


class _FakeWriter:
    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


async def test_basics():
    # registered as the 'health' activity the Controller builds from config
    assert task.ACTIVITIES.get('health') is board_health.BoardHealth

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth('health', {'period_ms': 20}, None)
    assert await health.setup() is True and health.validate()

    # sample()/inspect() report the vitals; load is an int percent 0..100
    vitals = health.sample()
    assert set(vitals.keys()) == {'temp', 'mem_free', 'load'}
    assert isinstance(vitals['mem_free'], int) and vitals['mem_free'] > 0
    assert isinstance(vitals['load'], int) and 0 <= vitals['load'] <= 100
    assert set(health.inspect().keys()) == {'temp', 'mem_free', 'load'}

    # the FIRST telemetry row lands at startup -- not one period late
    runner = asyncio.create_task(health.run())
    await asyncio.sleep_ms(5)  # << period (20 ms): only the startup row so far
    await recorder.Recorder.drain()
    rows = [bytes(i) for i in recorder.Recorder._uart.items]
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    assert any(b'uptime;temp;mem_free;load' in r for r in rows)  # header emitted
    assert sum(1 for r in rows if b'_health.csv@' in r) >= 2  # header + the startup data row


async def test_load_tracking():
    # load tracks real CPU load: idle -> low, a CPU hog -> higher. The estimate is relative to the
    # peak idle rate (_max_rate), so calibrate idle first, then load the board up.
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth('health', {'period_ms': 100}, None)
    await health.setup()
    runner = asyncio.create_task(health.run())
    await asyncio.sleep_ms(500)  # a few idle periods -> _max_rate calibrates, load near 0
    idle_load = health.load

    async def hog():  # burn cycles between minimal yields so the idle task runs far less
        while True:
            total = 0
            for _ in range(30000):
                total += 1
            await asyncio.sleep_ms(0)

    hogger = asyncio.create_task(hog())
    await asyncio.sleep_ms(500)  # board now busy
    busy_load = health.load

    hogger.cancel()
    runner.cancel()
    for stopping in (hogger, runner):
        try:
            await stopping
        except asyncio.CancelledError:
            pass

    assert 0 <= idle_load <= 100 and 0 <= busy_load <= 100
    assert busy_load > idle_load, (idle_load, busy_load)  # load rises with real CPU load
    print('  load: idle=%d%% busy=%d%%' % (idle_load, busy_load))


async def amain():
    await test_basics()
    await test_load_tracking()
    print('ok: board_health registered, sample/inspect, first-row-at-startup, int load tracks CPU')


asyncio.run(amain())
