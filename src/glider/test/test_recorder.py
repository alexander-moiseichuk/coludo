# On-board (MicroPython) test for the Recorder + PSRAM ring + Telemetry (recorder.py).
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import asyncio

import config_default
import recorder


class FakeWriter:
    """Stands in for the asyncio.StreamWriter over the recorder UART."""

    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


def _config(tlm_capacity, log_capacity, cell_size):
    cfg = config_default.default()
    cfg['recorder'] = {'tlm_capacity': tlm_capacity, 'log_capacity': log_capacity, 'cell_size': cell_size}
    return cfg


def test_ring():
    # SPSC write/read; holds capacity-1 records
    ring = recorder.Ring(3, 32)
    assert ring.write(b'a') and ring.write(b'b') and ring.count() == 2
    assert ring.write(b'c') is False and ring.dropped == 1  # full -> skip, no overwrite
    assert ring.read() == b'a' and ring.read() == b'b'
    assert ring.read() is None and ring.count() == 0
    assert ring.write(b'x' * 31) is False and ring.dropped == 2  # too big for a 32-byte cell


async def test_recorder():
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    assert recorder.Recorder._session is None  # lazy until first tlm/session
    session = recorder.Recorder.session()
    assert len(session) >= 19 and session[15] == '_'  # YYYYMMDD_HHMMSS_<rand>, rand disambiguates boots

    # telemetry first, then logs; @<session>_file@ routing
    assert recorder.Recorder.log('Controller', 'setup started') is True
    recorder.Recorder.tlm('cpu.csv', '40;51')
    assert await recorder.Recorder.drain() == 2
    out = recorder.Recorder._uart.items
    assert out[0] == ('@%s_cpu.csv@40;51\n' % session).encode(), out[0]
    assert b' Controller :: setup started\n' in out[1]
    assert await recorder.Recorder.drain() == 0

    # report exposes count/max/dropped
    rep = recorder.Recorder.report()
    assert rep['session'] == session and rep['tlm']['max'] >= 1 and 'dropped' in rep['log']

    # Telemetry: header first (uptime + fields), then timestamped rows
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    recorder.Recorder.telemetry_decimate_us = 0  # this block tests row FORMAT -> emit every push
    stream = recorder.Telemetry('imu.csv', ('yaw', 'pitch', 'roll'))
    stream.push((1.0, 2.0, 3.0))
    stream.push((4, 5, 6))
    await recorder.Recorder.drain()
    rows = recorder.Recorder._uart.items
    prefix = ('@%s_imu.csv@' % recorder.Recorder.session()).encode()
    assert rows[0] == prefix + b'uptime;yaw;pitch;roll\n', rows[0]
    assert rows[1].startswith(prefix) and rows[1].endswith(b';1.0;2.0;3.0\n')
    assert rows[2].endswith(b';4;5;6\n')

    # decimate_us: a fast stream is decimated -- bursts within the window collapse to one row
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    rated = recorder.Telemetry('fast.csv', ('v',), decimate_us=50000)  # >= 50 ms between rows
    rated.push((1,))  # first push always emits (header + row)
    rated.push((2,))  # immediately after -> decimated away
    rated.push((3,))
    await recorder.Recorder.drain()
    out = recorder.Recorder._uart.items
    assert len(out) == 2 and out[0].endswith(b'uptime;v\n') and out[1].endswith(b';1\n'), out

    # global rate: a stream with decimate_us=0 inherits Recorder.telemetry_decimate_us (the board-wide knob)
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    recorder.Recorder.telemetry_decimate_us = 50000
    glob = recorder.Telemetry('g.csv', ('v',))  # no per-stream rate -> the global
    assert glob.decimate_us == 50000
    glob.push((1,))  # header + first row
    glob.push((2,))  # within the global window -> decimated
    await recorder.Recorder.drain()
    assert len(recorder.Recorder._uart.items) == 2, recorder.Recorder._uart.items


async def test_error_policy():
    # logs are best-effort: a too-long message is truncated to the cell and still stored
    recorder.Recorder.setup(_config(8, 8, 64), uart=FakeWriter())
    assert recorder.Recorder.log('X', 'y' * 300) is True
    await recorder.Recorder.drain()
    assert len(recorder.Recorder._uart.items[0]) <= 64

    # logs drop (return False) when the buffer is full -- no raise
    recorder.Recorder.setup(_config(8, 2, 64), uart=FakeWriter())  # log ring holds 1
    assert recorder.Recorder.log('A', 'one') is True
    assert recorder.Recorder.log('A', 'two') is False  # full -> dropped, best-effort

    # telemetry is important: raises when full
    recorder.Recorder.setup(_config(2, 8, 64), uart=FakeWriter())  # tlm ring holds 1
    recorder.Recorder.tlm('t.csv', '1')
    raised = False
    try:
        recorder.Recorder.tlm('t.csv', '2')
    except recorder._RecorderError:
        raised = True
    assert raised

    # telemetry raises when a record is too big for a cell
    recorder.Recorder.setup(_config(8, 8, 64), uart=FakeWriter())
    raised = False
    try:
        recorder.Recorder.tlm('t.csv', 'v' * 200)
    except recorder._RecorderError:
        raised = True
    assert raised


async def test_cc_stream():
    # poll-model `log <ms>` streaming: a tee of log() onto a lazily-allocated CC ring (_cc_log), gated
    # by a deadline; the UART/Luckfox path must be untouched throughout.
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    tee = recorder.Recorder._cc_log
    assert tee._ring is None  # nothing allocated until the first request

    # off by default: log() does NOT collect for CC, but still goes to the UART ring
    assert recorder.Recorder.log('A', 'before') is True
    assert recorder.Recorder.cc_logs(0) == {'lines': []}  # disabled -> empty batch
    assert tee._ring is None  # nothing allocated while off

    # `log 1000` sizes + allocates the ring, returns the (empty) batch so far, and arms a 1 s window
    assert recorder.Recorder.cc_logs(1000)['lines'] == []
    assert tee._ring is not None and tee._deadline  # ring sized, armed
    recorder.Recorder.log('B', 'one')
    recorder.Recorder.log('B', 'two')
    batch = recorder.Recorder.cc_logs(1000)['lines']  # drain + re-arm
    assert len(batch) == 2 and batch[0].endswith('B :: one') and batch[1].endswith('B :: two'), batch
    assert recorder.Recorder.cc_logs(1000)['lines'] == []  # already drained -> empty next time

    # the UART log path is unaffected: every log() above still reached the _log ring (3 lines)
    assert await recorder.Recorder.drain() == 3
    assert all(b' :: ' in item for item in recorder.Recorder._uart.items)

    # the ring capacity is derived from the window (10 records/ms) and capped at 4x the default (1024)
    assert tee._ring.capacity == min(1000 * 10, 4 * 1024)

    # `log 0` hands back the final batch and stops streaming (deadline cleared)
    recorder.Recorder.cc_logs(1000)  # re-arm, ring empty
    recorder.Recorder.log('C', 'last')
    final = recorder.Recorder.cc_logs(0)['lines']
    assert len(final) == 1 and final[0].endswith('C :: last') and tee._deadline == 0

    # window lapse: a deadline already in the past -> the next log() discards + disables, no collection
    recorder.Recorder.cc_logs(1000)  # arm
    tee._deadline = recorder.time.ticks_add(recorder.time.ticks_us(), -1)  # already past
    recorder.Recorder.log('D', 'after-lapse')
    assert tee._deadline == 0  # log() saw the lapse and disabled
    assert recorder.Recorder.cc_logs(0)['lines'] == []  # nothing collected after the lapse


async def test_cc_telemetry():
    # poll-model `tlm <ms>` streaming: the same tee mechanism mirrors tlm() onto _cc_tlm, returning
    # {'samples': [...]}; the primary telemetry ring (and its raise-on-overflow policy) is untouched.
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    tee = recorder.Recorder._cc_tlm
    assert tee._ring is None  # off until requested

    # off by default: tlm() does NOT collect for CC, but still reaches the primary _tlm ring
    recorder.Recorder.tlm('t.csv', 'a')
    assert recorder.Recorder.cc_telemetry(0) == {'samples': []}  # disabled -> empty batch
    assert tee._ring is None

    # `tlm 1000` arms the window; subsequent tlm() rows are mirrored and drained on the next poll
    assert recorder.Recorder.cc_telemetry(1000)['samples'] == []
    assert tee._ring is not None and tee._deadline
    recorder.Recorder.tlm('t.csv', 'b')
    recorder.Recorder.tlm('t.csv', 'c')
    samples = recorder.Recorder.cc_telemetry(1000)['samples']  # drain + re-arm
    assert len(samples) == 2 and samples[0].endswith('@b') and samples[1].endswith('@c'), samples

    # the primary telemetry path is unaffected: every tlm() above still reached the _tlm ring
    assert await recorder.Recorder.drain() == 3  # a, b, c
    assert all(item.startswith(b'@') for item in recorder.Recorder._uart.items)

    # `tlm 0` hands back the final batch and stops streaming
    recorder.Recorder.cc_telemetry(1000)  # re-arm, ring empty
    recorder.Recorder.tlm('t.csv', 'z')
    final = recorder.Recorder.cc_telemetry(0)['samples']
    assert len(final) == 1 and final[0].endswith('@z') and tee._deadline == 0


async def test_run_loop():
    # run() loops forever; cancellation stops it (no stop flag)
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    recorder.Recorder.log('X', 'y')
    drain_task = asyncio.create_task(recorder.Recorder.run())
    await asyncio.sleep_ms(120)
    drain_task.cancel()
    try:
        await drain_task
    except asyncio.CancelledError:
        pass
    assert len(recorder.Recorder._uart.items) >= 1


async def _amain():
    await test_recorder()
    await test_error_policy()
    await test_cc_stream()
    await test_cc_telemetry()
    await test_run_loop()


test_ring()
asyncio.run(_amain())
print('ok: recorder SPSC ring, async drain/priority, log-drop vs tlm-raise, Telemetry, cc log+tlm stream, run loop')
