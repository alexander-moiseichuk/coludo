# On-board test for the board vitals task (tasks/board_health.py): @task.driver('health')
# registration, vitals sampling, telemetry push, load estimation, and inspect. Run by `make test`.

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


async def amain():
    # registered as the 'health' driver the Controller builds from config
    assert task.DRIVERS.get('health') is board_health.BoardHealth

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth('health', {'period_ms': 20}, None)
    assert await health.setup() is True and health.validate()

    # sample() reports the vitals; inspect() exposes exactly them
    vitals = health.sample()
    assert 'temp' in vitals and 'load' in vitals
    assert isinstance(vitals['mem_free'], int) and vitals['mem_free'] > 0
    assert set(health.inspect().keys()) == {'temp', 'mem_free', 'load'}

    # run a few periods: rows land in telemetry routed to the health.csv file
    runner = asyncio.create_task(health.run())
    await asyncio.sleep_ms(120)
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    await recorder.Recorder.drain()
    rows = [bytes(i) for i in recorder.Recorder._uart.items]
    assert any(b'_health.csv@' in r for r in rows), rows
    assert any(b'uptime;temp;mem_free;load' in r for r in rows)  # header emitted
    assert 0.0 <= health.load <= 1.0

    print('ok: board_health task driver registered, sample/telemetry/load/inspect')


asyncio.run(amain())
