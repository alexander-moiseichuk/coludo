# On-board (MicroPython) test for BoardHealth (board_health.py): vitals sampling, telemetry
# push, load estimation, and Inspectable. Run by `make test`.

import asyncio

import board_health
import config_default
import inspector
import recorder


class _FakeWriter:
    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


async def amain():
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth(period_ms=20)

    # sample() reports the vitals
    vitals = health.sample()
    assert 'temp' in vitals and 'load' in vitals
    assert isinstance(vitals['mem_free'], int) and vitals['mem_free'] > 0

    # inspectable + registered
    assert 'health' in inspector.Inspector.names()
    assert set(inspector.Inspector.inspect('health').keys()) == {'temp', 'mem_free', 'load'}

    # run a few periods: rows land in telemetry routed to the health.csv file
    task = asyncio.create_task(health.run())
    await asyncio.sleep_ms(120)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    await recorder.Recorder.drain()
    rows = [bytes(i) for i in recorder.Recorder._uart.items]
    assert any(b'_health.csv@' in r for r in rows), rows
    assert any(b'uptime;temp;mem_free;load' in r for r in rows)  # header emitted
    assert 0.0 <= health.load <= 1.0

    print('ok: board_health sample/telemetry/load/inspectable')


asyncio.run(amain())
