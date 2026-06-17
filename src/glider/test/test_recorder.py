# On-board (MicroPython) test for the Recorder + PSRAM ring + Telemetry (recorder.py).
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import asyncio

from config_default import default
from recorder import Recorder, Ring, Telemetry, _RecorderError


class FakeWriter:
    """Stands in for the asyncio.StreamWriter over the recorder UART."""

    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


def _config(tlm_capacity, log_capacity, cell_size):
    cfg = default()
    cfg['recorder'] = {'tlm_capacity': tlm_capacity, 'log_capacity': log_capacity, 'cell_size': cell_size}
    return cfg


def test_ring():
    # SPSC write/read; holds capacity-1 records
    ring = Ring(3, 32)
    assert ring.write(b'a') and ring.write(b'b') and ring.count() == 2
    assert ring.write(b'c') is False and ring.dropped == 1  # full -> skip, no overwrite
    assert ring.read() == b'a' and ring.read() == b'b'
    assert ring.read() is None and ring.count() == 0
    assert ring.write(b'x' * 31) is False and ring.dropped == 2  # too big for a 32-byte cell


async def test_recorder():
    Recorder.setup(default(), uart=FakeWriter())
    assert Recorder._session is None  # lazy until first tlm/session
    session = Recorder.session()
    assert len(session) == 15  # YYYYMMDD_HHMMSS

    # telemetry first, then logs; @<session>_file@ routing
    assert Recorder.log('Controller', 'setup started') is True
    Recorder.tlm('cpu.csv', '40;51')
    assert await Recorder.drain() == 2
    out = Recorder._uart.items
    assert out[0] == ('@%s_cpu.csv@40;51\n' % session).encode(), out[0]
    assert b' Controller :: setup started\n' in out[1]
    assert await Recorder.drain() == 0

    # report exposes count/max/dropped
    rep = Recorder.report()
    assert rep['session'] == session and rep['tlm']['max'] >= 1 and 'dropped' in rep['log']

    # Telemetry: header first (uptime + fields), then timestamped rows
    Recorder.setup(default(), uart=FakeWriter())
    stream = Telemetry('imu.csv', ('yaw', 'pitch', 'roll'))
    stream.push((1.0, 2.0, 3.0))
    stream.push((4, 5, 6))
    await Recorder.drain()
    rows = Recorder._uart.items
    prefix = ('@%s_imu.csv@' % Recorder.session()).encode()
    assert rows[0] == prefix + b'uptime;yaw;pitch;roll\n', rows[0]
    assert rows[1].startswith(prefix) and rows[1].endswith(b';1.0;2.0;3.0\n')
    assert rows[2].endswith(b';4;5;6\n')


async def test_error_policy():
    # logs are best-effort: a too-long message is truncated to the cell and still stored
    Recorder.setup(_config(8, 8, 64), uart=FakeWriter())
    assert Recorder.log('X', 'y' * 300) is True
    await Recorder.drain()
    assert len(Recorder._uart.items[0]) <= 64

    # logs drop (return False) when the buffer is full -- no raise
    Recorder.setup(_config(8, 2, 64), uart=FakeWriter())  # log ring holds 1
    assert Recorder.log('A', 'one') is True
    assert Recorder.log('A', 'two') is False  # full -> dropped, best-effort

    # telemetry is important: raises when full
    Recorder.setup(_config(2, 8, 64), uart=FakeWriter())  # tlm ring holds 1
    Recorder.tlm('t.csv', '1')
    raised = False
    try:
        Recorder.tlm('t.csv', '2')
    except _RecorderError:
        raised = True
    assert raised

    # telemetry raises when a record is too big for a cell
    Recorder.setup(_config(8, 8, 64), uart=FakeWriter())
    raised = False
    try:
        Recorder.tlm('t.csv', 'v' * 200)
    except _RecorderError:
        raised = True
    assert raised


async def test_run_loop():
    # run() loops forever; cancellation stops it (no stop flag)
    Recorder.setup(default(), uart=FakeWriter())
    Recorder.log('X', 'y')
    task = asyncio.create_task(Recorder.run())
    await asyncio.sleep_ms(120)
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    assert len(Recorder._uart.items) >= 1


async def _amain():
    await test_recorder()
    await test_error_policy()
    await test_run_loop()


test_ring()
asyncio.run(_amain())
print('ok: recorder SPSC ring, async drain/priority, log-drop vs tlm-raise, Telemetry, run loop')
