"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the Recorder + PSRAM ring + Telemetry (recorder.py). Run by
`make test`. Raises (-> runner reports FAIL) on any failed assertion.

Every line the drain writes carries the wire wrapper (doc/specs/recorder-wire.md). The exact vectors
below are literals produced by the host reference, tools/recorder_wire.py; _wrapped() is an independent
bitwise re-implementation for lines whose uptime the test cannot know in advance.
"""

import asyncio
import gc
import time

import config as config_module
import config_default
import recorder

"""
Raw record -> wire line, as tools/recorder_wire.py writes them (OPEN = crc32 of '<routing>@<payload>',
CLOSE = ~OPEN ^ the leading uptime). Literals, never recomputed here: they pin the board to the host.
"""
_ROUTED_VECTORS = (
    (b'@000123_imu_lsm6dso32.csv@884029;0.98;0.05;0.01\n',
     b'@000123_imu_lsm6dso32.csv@{cb34fe60};884029;0.98;0.05;0.01;<34c67ca2>\n'),
    (b'@000123_imu_lsm6dso32.csv@uptime;ax;ay;az\n',  # a header: no leading digits -> CLOSE = ~OPEN
     b'@000123_imu_lsm6dso32.csv@{229ee3fb};uptime;ax;ay;az;<dd611c04>\n'),
    (b'@session.csv@884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;\n',
     b'@session.csv@{9b56a10c};884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;'
     b';<64a423cc>\n'),
    (b'@a.csv@4294967296;0\n',  # an uptime of 2**32 wraps to U = 0
     b'@a.csv@{0c9ea9b2};4294967296;0;<f361564d>\n'),
)
_LOG_VECTORS = (
    (b'884030 sequencer :: stage -> boosting\n',
     b'{fe52ab73};884030 sequencer :: stage -> boosting;<01a029b2>\n'),
    (b'123456789\n',  # the CRC-32 check value: crc32(b'123456789') == 0xcbf43926
     b'{cbf43926};123456789;<33500bcc>\n'),
    (b'99999999999 x :: y\n',  # ticks past 2**32 wrap mod 2**32
     b'{12085fce};99999999999 x :: y;<a58147ce>\n'),
    (b'boot :: no digits here\n',  # no leading digits -> U = 0
     b'{7afd6024};boot :: no digits here;<85029fdb>\n'),
)


def _crc32(data: bytes) -> int:
    """Bitwise CRC-32 (the zlib polynomial) -- independent of the table-driven viper one under test."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _bit in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


def _routing_end(line: bytes) -> int:
    """Index of the first payload byte: past a routing's closing '@', else 0 (a log line)."""
    if line.startswith(b'@'):
        end = line.find(b'@', 1)
        if end > 0:
            return end + 1
    return 0


def _wrapped(raw: bytes) -> bytes:
    """The wire line for one raw record, per doc/specs/recorder-wire.md."""
    text = raw[:-1] if raw.endswith(b'\n') else raw
    start = _routing_end(text)
    payload = text[start:]
    number = 0
    for byte in payload:
        if not 48 <= byte <= 57:
            break
        number = (number * 10 + byte - 48) & 0xFFFFFFFF
    opening = _crc32(text[1:] if start else text)
    closing = opening ^ 0xFFFFFFFF ^ number
    return text[:start] + ('{%08x};' % opening).encode() + payload + (';<%08x>\n' % closing).encode()


def _unwrap(line: bytes) -> bytes:
    """The raw record a wire line carries, after checking its wrapper (AssertionError when it fails)."""
    start = _routing_end(line)
    body = line[start:-1]
    assert line.endswith(b'\n') and body[:1] == b'{' and body[9:11] == b'};', line
    assert body[-11:-9] == b';<' and body[-1:] == b'>', line
    raw = line[:start] + body[11:-11] + b'\n'
    assert _wrapped(raw) == line, line
    return raw


class FakeWriter:
    """
    Stands in for the asyncio.StreamWriter over the recorder UART.

    Carries `out_buf` because the real StreamWriter does and Recorder.drain() reads it to decide when to
    flush -- a stub without it makes the high-water path both untestable and fatal. Counts flushes so a
    test can assert back-pressure actually happened rather than assuming it.
    """

    def __init__(self):
        self.items = []
        self.out_buf = b''      # pending bytes, exactly as the real StreamWriter exposes them
        self.flushes = 0

    def write(self, data):
        self.items.append(bytes(data))
        self.out_buf += bytes(data)

    async def drain(self):
        self.out_buf = b''
        self.flushes += 1


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

    # readinto: the drain's allocation-free read -- the record lands at the head of the caller's buffer
    buffer = bytearray(30)
    assert ring.write(b'hello') and ring.write(b'')
    assert ring.readinto(buffer) == 5 and buffer[:5] == b'hello'
    assert ring.readinto(buffer) == 0  # an empty RECORD is not an empty RING
    assert ring.readinto(buffer) == -1 and ring.count() == 0  # NEGATIVE: empty ring
    # NEGATIVE: a record longer than the buffer raises and STAYS queued (viper would write past it)
    assert ring.write(b'z' * 30)
    raised = False
    try:
        ring.readinto(bytearray(29))
    except ValueError:
        raised = True
    assert raised and ring.count() == 1
    assert ring.readinto(buffer) == 30 and buffer == b'z' * 30


async def test_recorder():
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    assert recorder.Recorder._session is None  # lazy until first tlm/session
    session = recorder.Recorder.session()
    assert session  # its shape per boot id / config / legacy is test_session()'s

    # telemetry first, then logs; @<session>_file@ routing, each line wrapped on the way out
    assert recorder.Recorder.log('Controller', 'setup started') is True
    recorder.Recorder.tlm('cpu.csv', '40;51')
    assert await recorder.Recorder.drain() == 2
    out = recorder.Recorder._uart.items
    assert out[0] == _wrapped(('@%s_cpu.csv@40;51\n' % session).encode()), out[0]
    assert _unwrap(out[1]).endswith(b' Controller :: setup started\n'), out[1]
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
    rows = [_unwrap(item) for item in recorder.Recorder._uart.items]
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
    out = [_unwrap(item) for item in recorder.Recorder._uart.items]
    assert len(out) == 2 and out[0].endswith(b'uptime;v\n') and out[1].endswith(b';1\n'), out

    # global rate: a stream with decimate_us=0 inherits Recorder.telemetry_decimate_us (the board-wide knob)
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    recorder.Recorder.telemetry_decimate_us = 50000
    glob = recorder.Telemetry('g.csv', ('v',))  # no per-stream rate -> the global
    assert glob.decimate_us == 0, glob.decimate_us   # its OWN rate: 0 means inherit
    assert glob.window == 50000, glob.window         # ...and `window` is what it actually decimates by
    glob.push((1,))  # header + first row
    glob.push((2,))  # within the global window -> decimated
    await recorder.Recorder.drain()
    assert len(recorder.Recorder._uart.items) == 2, recorder.Recorder._uart.items

    """
    The CONFIG's global rate must actually reach the Recorder. Nothing here asserted that: every test
    set Recorder.telemetry_decimate_us by hand, so a knob written into the wrong dict was invisible.
    It was -- config_default carried telemetry_ms on the recorder COMPONENT while setup() reads the
    top-level SECTION, so the rate never took effect and the 20 ms class default silently won. Drive
    setup() with the real default config and check the result.

    The expectation is DERIVED from the config, not written as a literal. It was 40000 and broke the
    moment the default legitimately changed to 0 (uncapped) -- a test that has to be edited every time
    the value it guards moves is testing the value, when what matters is the WIRING. The guard that
    keeps it honest is the assert above it: if the config ever equals the class default, this test
    could not distinguish "read correctly" from "fell back", so it says so instead of passing.
    """
    expected_us = config_default.default()['recorder']['telemetry_ms'] * 1000
    class_default_us = 20000  # recorder._DEFAULT_TELEMETRY_MS: a const(), so folded at compile time
    assert expected_us != class_default_us, 'config equals the class default -- this test cannot fail'
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    assert recorder.Recorder.telemetry_decimate_us == expected_us, recorder.Recorder.telemetry_decimate_us
    # and a stream that declares no rate of its own must inherit exactly that
    assert recorder.Telemetry('inherit.csv', ('v',)).window == expected_us

    """
    The global must reach a stream BUILT BEFORE IT WAS SET. That ordering is the real one: drivers
    construct their Telemetry during setup(), and the controller runs every device's setup before the
    recorder task's, so on a real boot every stream predates the configured global. Telemetry used to
    fold the global into decimate_us in __init__, so those streams silently kept the 50 Hz class
    default and `recorder.telemetry_ms` did nothing -- a config asking for full-rate logging held a
    100 Hz accelerometer at 50, and nothing failed. No test covered this order, which is why it shipped.
    """
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    early = recorder.Telemetry('early.csv', ('v',))          # built while the global is the default
    recorder.Recorder.telemetry_decimate_us = 0              # ...then the config turns decimation OFF
    assert early.window == 0, early.window
    for value in range(4):
        early.push((value,))                                 # every push must emit: no window at all
    await recorder.Recorder.drain()
    assert len(recorder.Recorder._uart.items) == 5, recorder.Recorder._uart.items  # header + 4 rows


async def test_error_policy():
    # logs are best-effort: a too-long message is truncated to the cell and still stored
    recorder.Recorder.setup(_config(8, 8, 64), uart=FakeWriter())
    assert recorder.Recorder.log('X', 'y' * 300) is True
    await recorder.Recorder.drain()
    line = recorder.Recorder._uart.items[0]
    assert len(_unwrap(line)) == 64 - 2, len(line)  # the record filled the cell's payload exactly...
    assert len(line) == 64 - 2 + 22  # ...and the wrapper still fits: the drain sizes for it
    """
    ...and it MUST still end with a newline. The wire protocol is line-framed, so truncating the '\n'
    away merges the over-long log line with whatever record follows it on the UART -- and when that is
    a telemetry row, the row is swallowed into the log line and lost from its CSV, silently, on the
    channel the error policy promises is durable. The length check alone passed either way, which is
    why this went unnoticed.
    """
    assert _unwrap(line).endswith(b'\n'), line[-16:]

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
    assert recorder.Recorder.cc_logs(0) == {'lines': [], 'dropped': 0}  # disabled -> empty batch
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

    """
    The tee DISCARDS when its ring fills (best-effort by policy -- it must never raise on a log path),
    so a live stream can have a hole that looks exactly like a quiet sensor. The reply carries the
    discarded count, which is what lets the hub say "this window is incomplete" instead of the operator
    reading absence as calm.
    """
    recorder.Recorder.cc_logs(1000)                       # arm with a fresh ring
    for index in range(tee._ring.capacity + 5):           # overrun it deliberately
        recorder.Recorder.log('E', 'flood %d' % index)
    flooded = recorder.Recorder.cc_logs(1000)
    assert flooded['dropped'] > 0, flooded['dropped']     # the loss is REPORTED, not silent
    assert recorder.Recorder.cc_logs(1000)['dropped'] == 0  # ...and the count resets per window

    """
    Window lapse: a deadline already in the past -> the next log() discards + disables, no collection.

    Lapsed with a LOADED ring, not an empty one. The discard runs on the PRODUCER's path (tee() is
    reached from every log()/tlm_raw(), i.e. inside Telemetry.push at 100 Hz with GC off), and it used
    to call _take(), which builds a list and decodes every buffered record into a str purely to throw
    it away -- a burst of allocation at the worst possible moment. An empty ring made that free, which
    is why the old version of this test could not see it. Load the ring first.
    """
    recorder.Recorder.cc_logs(1000)  # arm
    for index in range(20):
        recorder.Recorder.log('D', 'buffered %d' % index)   # real records waiting in the tee ring
    assert tee._ring.count() > 0, 'the ring must be LOADED for the discard to mean anything'
    tee._deadline = recorder.time.ticks_add(recorder.time.ticks_us(), -1)  # already past
    recorder.Recorder.log('D', 'after-lapse')
    assert tee._deadline == 0  # log() saw the lapse and disabled
    assert tee._ring.count() == 0, 'the lapse must empty the ring, not leave it holding records'
    assert recorder.Recorder.cc_logs(0)['lines'] == []  # nothing collected after the lapse


async def test_cc_telemetry():
    # poll-model `tlm <ms>` streaming: the same tee mechanism mirrors tlm() onto _cc_tlm, returning
    # {'samples': [...]}; the primary telemetry ring (and its raise-on-overflow policy) is untouched.
    recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
    tee = recorder.Recorder._cc_tlm
    assert tee._ring is None  # off until requested

    # off by default: tlm() does NOT collect for CC, but still reaches the primary _tlm ring
    recorder.Recorder.tlm('t.csv', 'a')
    assert recorder.Recorder.cc_telemetry(0) == {'samples': [], 'dropped': 0}  # disabled -> empty batch
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
    # the wrapper is the UART's alone: the CC tee above saw the raw '...@b', the wire the wrapped line
    assert _unwrap(recorder.Recorder._uart.items[1]).endswith(b'@b\n') and b'@{' in recorder.Recorder._uart.items[1]

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


async def test_back_pressure():
    """
    drain() must flush MID-PASS once pending bytes reach the high-water mark, not buffer a whole ring.

    The UART was created with MicroPython's 256-byte default txbuf while the ring holds 256 KB, so one
    pass could hand the peripheral orders of magnitude more than it can take -- and an overrun does not
    raise, it silently drops the tail of whatever record is in flight. That is the exact shape of the
    corruption measured on real captures: a record cut mid-field, its newline gone, and the next record
    running onto the same line (20 in 1,004,804 rows across 48 flights).

    Flushing on FILL rather than on a record count adapts to record size and link speed: a quiet stream
    never flushes early, a burst flushes as often as it needs. A fixed sleep per line was measured
    against instead -- 4.2 s of sleep per second of flight at this board's ~10 ms asyncio floor.
    """
    writer = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    payload = b'y' * 200
    for _ in range(60):                     # 12000 bytes: must cross the 2048 mark several times
        recorder.Recorder._tlm.write(payload)
    drained = await recorder.Recorder.drain()
    assert drained == 60, drained
    assert writer.flushes >= 2, 'back-pressure never engaged: %d flush(es)' % writer.flushes
    assert len(writer.items) == 60, len(writer.items)          # nothing dropped while flushing

    # NEGATIVE: a small pass must NOT flush early -- back-pressure only when there is pressure
    quiet = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=quiet)
    recorder.Recorder._tlm.write(b'z' * 50)
    await recorder.Recorder.drain()
    assert quiet.flushes == 1, 'a 50-byte pass flushed %d times, not once' % quiet.flushes


async def test_wire_vectors():
    """
    The drain's wrapper reproduces the host reference byte for byte.

    Raw records go into the rings exactly as producers leave them and come out of drain(), so this is the
    code path a flight uses -- readinto, the viper CRC + hex, the per-length view -- not a side door. The
    expected lines are tools/recorder_wire.py output, pasted as literals.
    """
    writer = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    for raw, _line in _ROUTED_VECTORS:
        assert recorder.Recorder._tlm.write(raw)
    for raw, _line in _LOG_VECTORS:
        assert recorder.Recorder._log.write(raw)
    assert await recorder.Recorder.drain() == len(_ROUTED_VECTORS) + len(_LOG_VECTORS)
    expected = [line for _raw, line in _ROUTED_VECTORS] + [line for _raw, line in _LOG_VECTORS]  # tlm first
    assert writer.items == expected, writer.items
    # the independent reference agrees with the host on every vector (so _wrapped() can judge the rest)
    assert all(_wrapped(raw) == line for raw, line in _ROUTED_VECTORS + _LOG_VECTORS)

    # a header (no leading digits) closes with plain ~OPEN
    header = _ROUTED_VECTORS[1][1]
    body = header[_routing_end(header):]
    assert int(body[1:9].decode(), 16) ^ int(body[-10:-2].decode(), 16) == 0xFFFFFFFF, header

    # a record without its '\n' still goes out as exactly one line
    assert recorder.Recorder._tlm.write(b'@a.csv@7;x')
    await recorder.Recorder.drain()
    assert writer.items[-1] == _wrapped(b'@a.csv@7;x\n') and writer.items[-1].endswith(b'>\n')


async def test_wrap_negative():
    """Empty rings, over-long records and a too-small output buffer: refused, never written past."""
    writer = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    # an empty ring drains nothing and writes nothing
    assert await recorder.Recorder.drain() == 0 and writer.items == []

    # the longest record a cell holds -- with and without its '\n' -- still fits the drain's output buffer
    max_payload = recorder.Recorder._tlm.max_payload
    longest = b'@a.csv@' + b'9' * (max_payload - 8) + b'\n'
    bare = longest[:-1] + b'9'
    assert len(longest) == len(bare) == max_payload
    assert recorder.Recorder._tlm.write(longest) and recorder.Recorder._tlm.write(bare)
    await recorder.Recorder.drain()
    assert writer.items == [_wrapped(longest), _wrapped(bare)], writer.items
    assert len(writer.items[1]) == max_payload + 22 + 1  # + the '\n' it came without: the buffer's exact size

    # one byte more and the ring refuses it (telemetry raises -- test_error_policy)
    assert recorder.Recorder._tlm.write(longest + b'x') is False

    # _wrap itself never writes past the buffer it is given: -1, and the buffer untouched
    small = bytearray(24)
    table = recorder.Recorder._crc
    assert recorder._wrap(b'ab\n', 3, small, len(small), table, b'0123456789abcdef') == -1
    assert small == bytearray(24)
    assert recorder._wrap(b'a\n', 2, small, len(small), table, b'0123456789abcdef') == 24  # exactly fits
    assert bytes(small) == _wrapped(b'a\n')

    """
    NEGATIVE: a record the drain cannot wrap is never silent. setup() sizes the line buffer so this cannot
    happen; a shrunken buffer forces it. The refused record counts in its ring's `dropped`, the drain
    goes on with the next one, and the cause is logged ONCE -- the CC tee sees the line even though the
    log record itself is refused too (it is longer than the shrunken buffer).
    """
    writer = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    recorder.Recorder._line = bytearray(30)
    view = memoryview(recorder.Recorder._line)
    recorder.Recorder._lines = [view[:length] for length in range(31)]
    recorder.Recorder.cc_logs(1000)  # arm the CC log tee: it sees log lines before the drain refuses them
    assert recorder.Recorder._tlm.write(b'@a.csv@1;2\n')  # 10 B + the wrapper and '\n' > 30: refused
    assert recorder.Recorder._tlm.write(b'7\n')  # 1 B + 23 B: fits, and still goes out after the refusal
    assert recorder.Recorder._tlm.write(b'@b.csv@3;4\n')  # refused again: counted, not logged again
    assert await recorder.Recorder.drain() == 1  # only the record it wrote
    assert writer.items == [_wrapped(b'7\n')], writer.items
    assert recorder.Recorder._tlm.dropped == 2 and recorder.Recorder._log.dropped == 1  # the log line too
    lines = [line for line in recorder.Recorder.cc_logs(0)['lines'] if 'wire wrapper refused' in line]
    assert len(lines) == 1 and 'counted in dropped' in lines[0], lines


class _QuietWriter:
    """A writer that allocates nothing, so a measurement sees the drain's own allocation and no more."""

    def __init__(self):
        self.lines = 0

    def write(self, data):
        self.lines += 1

    async def drain(self):
        pass


async def _drain_cost(count: int) -> int:
    """Bytes the heap grows by while drain() wraps and writes `count` queued rows, with the GC off."""
    writer = _QuietWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    for _row in range(count):
        assert recorder.Recorder._tlm.write(b'@000123_imu_lsm6dso32.csv@884029;0.98;0.05;0.01\n')
    gc.collect()
    gc.disable()  # as in flight, BOOSTING to DONE
    try:
        before = gc.mem_alloc()
        drained = await recorder.Recorder.drain()
        spent = gc.mem_alloc() - before
    finally:
        gc.enable()
    assert drained == count and writer.lines == count, (drained, writer.lines)
    return spent


async def test_drain_allocation():
    """
    The drain allocates NOTHING per line -- the reason the wrapper is viper and the views are made at setup.

    The GC is off from BOOSTING to DONE, so every byte the drain allocates per row is leaked for the rest
    of the flight. The old read() path cost a bytes copy, a bytearray slice and a tuple per row (~200 B at
    ~400 rows/s); a naive wrapper would add a boxed CRC and two hex strings. Per-drain overhead (the
    coroutine, the ring tuple) is the same for 10 rows and 200, so the difference is the per-line cost.
    """
    few = await _drain_cost(10)
    many = await _drain_cost(200)
    assert many - few < 190, 'drain allocates per line: %d B for 10 rows, %d B for 200' % (few, many)


def test_session():
    """
    The session prefix: config label, else the NVS boot id as '%06u', else the legacy date + random.

    Recorder.boot_id is set here by hand and put back -- main.py's NVS counter is never touched, so the
    test cannot bump the board's real boot count.
    """
    booted = recorder.Recorder.boot_id
    try:
        recorder.Recorder.boot_id = 123
        recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
        assert recorder.Recorder.session() == '000123'
        assert recorder.Recorder.boot_id == 123  # setup() leaves the boot's id alone
        recorder.Recorder.boot_id = 1234567  # wider than six digits: never truncated
        recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
        assert recorder.Recorder.session() == '1234567'

        # a config label wins over the boot id, spaces made file-safe
        labelled = config_default.default()
        labelled['recorder']['session'] = 'catapult run3'
        recorder.Recorder.setup(labelled, uart=FakeWriter())
        assert recorder.Recorder.session() == 'catapult-run3'
        assert recorder.Recorder._log.count() == 0  # a valid label logs nothing

        """
        NEGATIVE: a label must be ASCII letters, digits and '-' with a letter in it. A '_' would make
        `<session>_<stream>.csv` ambiguous to split, digits alone would read as a boot id, and other bytes
        do not belong in a file name. Each is ignored (the boot id names the files) and logged.
        """
        recorder.Recorder.boot_id = 123
        for label in ('catapult_run3', '20261003', '123-456', 'run/3', 'r\u00e9union', 'run;3'):
            labelled['recorder']['session'] = label
            recorder.Recorder.setup(labelled, uart=FakeWriter())
            assert recorder.Recorder.session() == '000123', (label, recorder.Recorder.session())
            logged = recorder.Recorder._log.read()
            assert logged is not None and b'ignored' in logged and label.encode() in logged, (label, logged)
        assert recorder._is_label('A-1') and recorder._is_label('x') and not recorder._is_label('')

        """
        NEGATIVE: 32 characters at most (recorder._LABEL_MAX, a const() and so not readable here). The label
        heads the routing of every telemetry line, so an unbounded one overflows the 254-byte cell and every
        tlm() raises; the longest allowed still leaves a wide row its room.
        """
        longest = 'run-' + 'x' * 28
        assert len(longest) == 32 and recorder._is_label(longest) and not recorder._is_label(longest + 'x')
        labelled['recorder']['session'] = longest
        recorder.Recorder.setup(labelled, uart=FakeWriter())
        assert recorder.Recorder.session() == longest and recorder.Recorder._log.count() == 0
        recorder.Recorder.tlm('imu_lsm6dso32.csv', '4294967295;' + ';'.join(['-1234.567'] * 12))  # raises if not
        labelled['recorder']['session'] = longest + 'x'
        recorder.Recorder.setup(labelled, uart=FakeWriter())
        assert recorder.Recorder.session() == '000123', recorder.Recorder.session()
        logged = recorder.Recorder._log.read()
        assert logged is not None and b'ignored' in logged and b'32 at most' in logged, logged

        # nothing counted the boot (tests, HITL): the legacy YYYYMMDD_HHMMSS_<6-digit random>
        recorder.Recorder.boot_id = None
        recorder.Recorder.setup(config_default.default(), uart=FakeWriter())
        legacy = recorder.Recorder.session()
        assert len(legacy) == 22 and legacy[8] == '_' and legacy[15] == '_', legacy
        assert (legacy[:8] + legacy[9:15] + legacy[16:]).isdigit(), legacy
        assert recorder.Recorder.session() is legacy  # settled once, fixed for the boot
    finally:
        recorder.Recorder.boot_id = booted


_CLOCK_SET = 844352467  # 2026-10-03T14:21:07Z on the RTC's 2000 epoch (Unix 1791037267): a set clock
_FIRST_SET = 31622400  # 2001-01-01T00:00:00Z on the RTC's epoch: its first set second (recorder._CLOCK_SET_S)


async def test_session_index():
    """
    session.csv: routed bare, header first, then a row per call -- dated from the RTC, wrapped on the wire.
    """
    booted = recorder.Recorder.boot_id
    try:
        recorder.Recorder.boot_id = 123
        cfg = config_default.default()
        writer = FakeWriter()
        recorder.Recorder.setup(cfg, uart=writer)
        assert recorder.Recorder.index_session(_CLOCK_SET, -240, 'cc-auto', None) is True
        assert recorder.Recorder.index_session(_CLOCK_SET + 233, None, 'dashboard', (25.5, -80.25)) is True  # 14:25:00
        assert await recorder.Recorder.drain() == 3  # header + 2 rows: the header went out ONCE
        raw = [_unwrap(line) for line in writer.items]  # every line checks out on the wire
        assert raw[0] == (b'@session.csv@uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;'
                          b'cc_lat;cc_lon\n'), raw[0]
        identity = [cfg['board']['id'], cfg['board']['firmware_version'], config_module.config_id(cfg)]
        first = raw[1][len(b'@session.csv@'):-1].decode().split(';')
        assert first[0].isdigit() and first[1:4] == ['123', '000123', '2026-10-03T14:21:07Z'], first
        assert first[4:] == ['-240'] + identity + ['cc-auto', '', ''], first  # unknown -> empty cells
        second = raw[2][len(b'@session.csv@'):-1].decode().split(';')
        assert second[3:] == ['2026-10-03T14:25:00Z', ''] + identity + ['dashboard', '25.5', '-80.25'], second

        """
        NEGATIVE: a clock nobody set dates nothing. While the RTC reads before 2001 -- 2000-01-01 at
        power-on, or a time set from a setter's own bad clock -- utc AND utc_offset are empty on the row,
        whatever the caller passed; the first second of 2001 is set (test_mission pins the RTC's epoch).
        """
        writer = FakeWriter()
        recorder.Recorder.setup(cfg, uart=writer)
        for seconds in (0, 3600, _FIRST_SET - 1, _FIRST_SET):  # power-on, an hour in, the last unset second
            assert recorder.Recorder.index_session(seconds, -240, 'dashboard', None) is True
        assert await recorder.Recorder.drain() == 5
        rows = [_unwrap(line)[len(b'@session.csv@'):-1].decode().split(';') for line in writer.items[1:]]
        assert [row[3:5] for row in rows] == [['', '']] * 3 + [['2001-01-01T00:00:00Z', '-240']], rows
        assert all(row[1:3] == ['123', '000123'] and row[8] == 'dashboard' for row in rows), rows

        # a boot nothing counted leaves the boot cell empty; the session is the legacy prefix
        recorder.Recorder.boot_id = None
        recorder.Recorder.setup(cfg, uart=FakeWriter())
        recorder.Recorder.index_session(_CLOCK_SET, None, '', None)
        recorder.Recorder._tlm.read()  # this setup's header
        cells = recorder.Recorder._tlm.read()[len(b'@session.csv@'):-1].decode().split(';')
        assert cells[1] == '' and cells[2] == recorder.Recorder.session(), cells

        # NEGATIVE: no Recorder running -> nothing to write to, and no AttributeError either
        rings = recorder.Recorder._tlm
        recorder.Recorder._tlm = None
        try:
            assert recorder.Recorder.index_session(_CLOCK_SET, 0, 'cc-auto', None) is False
        finally:
            recorder.Recorder._tlm = rings

        """
        Text cells are printable ASCII without ';', at most 32 characters, else empty: a ';' splits the
        cell, a TAB is a byte the Luckfox discards (the row's CRC then fails), non-ASCII has no place in
        the file, and a longer cell would lose the whole row to the ring-cell size. That
        holds for the board name and firmware from config too, and a config without a firmware version
        reads 'dev', as whoami reports it.
        """
        for text in ('a;b', 'a\tb', 'a\nb', 'caf\u00e9', 7, None):
            assert recorder._cell(text) == '', text
        assert recorder._cell(' cc-auto ~') == ' cc-auto ~' and recorder._cell('') == ''
        assert recorder._cell('x' * 32) == 'x' * 32 and recorder._cell('x' * 33) == ''  # literal: _CELL_MAX
        unnamed = config_default.default()
        unnamed['board']['id'] = 'tms;7'
        del unnamed['board']['firmware_version']
        recorder.Recorder.boot_id = 123
        recorder.Recorder.setup(unnamed, uart=FakeWriter())
        recorder.Recorder.index_session(_CLOCK_SET, None, 'cc\tauto', None)
        recorder.Recorder._tlm.read()  # this setup's header
        cells = recorder.Recorder._tlm.read()[len(b'@session.csv@'):-1].decode().split(';')
        assert len(cells) == 11 and cells[5:9] == ['', 'dev', config_module.config_id(unnamed), ''], cells

        """
        NEGATIVE: a full telemetry ring raises (the caller logs and loses the row), and a header that did
        not fit goes out with the next row -- a row must never reach the file without its header.
        """
        writer = FakeWriter()
        recorder.Recorder.setup(_config(2, 8, 256), uart=writer)  # the telemetry ring holds ONE record
        assert recorder.Recorder._tlm.write(b'@a.csv@1\n')  # ...and it is taken
        raised = False
        try:
            recorder.Recorder.index_session(_CLOCK_SET, None, 'cc-auto', None)
        except recorder._RecorderError:
            raised = True
        assert raised and recorder.Recorder._index_header_sent is False  # the header was refused
        await recorder.Recorder.drain()
        raised = False
        try:  # the header now fits, the row behind it does not
            recorder.Recorder.index_session(_CLOCK_SET + 53, None, 'cc-auto', None)
        except recorder._RecorderError:
            raised = True
        assert raised and recorder.Recorder._index_header_sent is True
        await recorder.Recorder.drain()
        assert recorder.Recorder.index_session(_CLOCK_SET + 113, None, 'dashboard', None) is True
        await recorder.Recorder.drain()
        raw = [_unwrap(line) for line in writer.items[1:]]  # past the record that filled the ring
        assert len(raw) == 2 and raw[0].startswith(b'@session.csv@uptime;'), raw  # one header, then...
        assert b';2026-10-03T14:23:00Z;' in raw[1] and b';dashboard;' in raw[1], raw  # ...the row that fit
    finally:
        recorder.Recorder.boot_id = booted


async def test_session_anchor():
    """
    The 'anchor' row: once per boot, a minute after setup(), whatever the clock -- header again first.

    It lists the boots the earlier rows miss: a 'boot' row and first sync sent before the Luckfox
    listened, and boots nobody syncs -- a soft reset or a warm start kept the RTC, or no CC was there.
    `now` and the RTC go in as arguments, so the minute and the clock are the test's, not the board's.
    """
    booted = recorder.Recorder.boot_id
    try:
        recorder.Recorder.boot_id = 123
        cfg = config_default.default()
        writer = FakeWriter()
        recorder.Recorder.setup(cfg, uart=writer)
        start = recorder.Recorder._setup_ms
        assert recorder.Recorder.index_session(_CLOCK_SET - 7, -240, 'cc-auto', None)  # a time set, 14:21:00
        # NEGATIVE: under a minute in -> not due
        assert recorder.Recorder._anchor(time.ticks_add(start, 59999), _CLOCK_SET) is False
        assert recorder.Recorder._anchored is False
        assert recorder.Recorder._anchor(time.ticks_add(start, 60000), _CLOCK_SET) is True
        # NEGATIVE: one-shot -- a later tick with a later clock writes nothing more
        assert recorder.Recorder._anchor(time.ticks_add(start, 120000), _CLOCK_SET + 60) is False
        assert await recorder.Recorder.drain() == 4
        raw = [_unwrap(line) for line in writer.items]
        header = b'@session.csv@' + recorder._SESSION_INDEX_HEADER.encode() + b'\n'
        assert raw[0] == header and raw[2] == header, raw  # the header again, right before the anchor
        cells = raw[3][len(b'@session.csv@'):-1].decode().split(';')
        identity = [cfg['board']['id'], cfg['board']['firmware_version'], config_module.config_id(cfg)]
        assert cells[0].isdigit() and cells[1:5] == ['123', '000123', '2026-10-03T14:21:07Z', ''], cells
        assert cells[5:] == identity + ['anchor', '', ''], cells

        """
        A clock nobody set does not hold the anchor back -- a boot without CC is listed too: on time and
        undated, utc and utc_offset empty, the boot id placing it between its dated neighbours.
        """
        for seconds in (0, _FIRST_SET - 1):  # power-on; 2000-12-31T23:59:59, the last unset second
            writer = FakeWriter()
            recorder.Recorder.setup(cfg, uart=writer)
            assert recorder.Recorder._anchor(time.ticks_add(recorder.Recorder._setup_ms, 60000), seconds) is True
            assert await recorder.Recorder.drain() == 2
            raw = [_unwrap(line) for line in writer.items]
            cells = raw[1][len(b'@session.csv@'):-1].decode().split(';')
            assert raw[0] == header and cells[1:5] == ['123', '000123', '', ''], (seconds, raw)
            assert cells[5:] == identity + ['anchor', '', ''], (seconds, cells)

        # NEGATIVE: a full ring loses the anchor -- logged, never raised (it runs inside run()), still spent
        recorder.Recorder.setup(_config(2, 8, 256), uart=FakeWriter())
        assert recorder.Recorder._tlm.write(b'@a.csv@1\n')
        assert recorder.Recorder._anchor(time.ticks_add(recorder.Recorder._setup_ms, 60000), _CLOCK_SET) is True
        logged = recorder.Recorder._log.read()
        assert logged is not None and b'session.csv anchor row lost' in logged, logged
        assert recorder.Recorder._anchor(time.ticks_add(recorder.Recorder._setup_ms, 61000), _CLOCK_SET) is False
    finally:
        recorder.Recorder.boot_id = booted


async def _stopped(drain_task) -> bool:
    """
    Cancel a run() task; True when it was still running.

    run() never returns, so a task already done has raised -- and MicroPython's loop may have consumed that
    error already, after which awaiting the task returns quietly. Hence done() rather than the await.

    Args:
        drain_task - the asyncio task running Recorder.run().

    Returns:
        True when run() was still running at the cancel; False when it had died.
    """
    running = not drain_task.done()
    drain_task.cancel()
    try:
        await drain_task
    except asyncio.CancelledError:
        pass
    return running


async def test_index_run_loop():
    """
    run() writes the 'boot' row as it starts and the 'anchor' off its stats tick: each once, whatever the RTC.

    The minute is faked by moving setup()'s stamp back, each stats tick by moving the last one back. The
    clock is the board's own and is left alone: it decides only whether the rows are dated, never whether
    they are written, so an unset bench clock cannot hide a broken run().
    """
    writer = FakeWriter()
    recorder.Recorder.setup(config_default.default(), uart=writer)
    recorder.Recorder._setup_ms = time.ticks_add(time.ticks_ms(), -61000)
    drain_task = asyncio.create_task(recorder.Recorder.run())
    for _tick in range(2):  # the anchor's tick, then one more that must write nothing
        recorder.Recorder._last_stats_ms = time.ticks_add(time.ticks_ms(), -1000)  # the next pass is a stats tick
        recorder.Recorder.log('X', 'wake')
        await asyncio.sleep_ms(120)
    assert await _stopped(drain_task), 'run() died'
    dated = time.time() >= _FIRST_SET
    raw = [_unwrap(line) for line in writer.items]
    header = b'@session.csv@' + recorder._SESSION_INDEX_HEADER.encode() + b'\n'
    rows = [index for index, line in enumerate(raw) if line.startswith(b'@session.csv@') and line != header]
    cells = [raw[index][len(b'@session.csv@'):-1].decode().split(';') for index in rows]
    assert [row[8] for row in cells] == ['boot', 'anchor'], cells
    assert all(raw[index - 1] == header for index in rows) and raw.count(header) == 2, raw  # header before each
    assert all((row[3] != '') is dated and row[4] == '' for row in cells), (dated, cells)  # no time set behind


async def test_boot_row_lost():
    """
    NEGATIVE: a 'boot' row that finds the telemetry ring full is logged and lost, and run() runs on.

    run() is the drain loop: raising there would end the boot's capture over one index row.
    """
    writer = FakeWriter()
    recorder.Recorder.setup(_config(2, 8, 256), uart=writer)  # the telemetry ring holds ONE record...
    assert recorder.Recorder._tlm.write(b'@a.csv@1\n')  # ...and it is taken
    drain_task = asyncio.create_task(recorder.Recorder.run())
    await asyncio.sleep_ms(120)
    recorder.Recorder.tlm('b.csv', '2')  # the drain made room: telemetry flows on
    await asyncio.sleep_ms(120)
    assert await _stopped(drain_task), 'run() died'
    raw = [_unwrap(line) for line in writer.items]
    assert raw[0] == b'@a.csv@1\n' and not any(line.startswith(b'@session.csv@') for line in raw), raw
    assert any(b'session.csv boot row lost' in line for line in raw), raw
    assert raw[-1] == ('@%s_b.csv@2\n' % recorder.Recorder.session()).encode(), raw


async def _amain():
    await test_recorder()
    await test_error_policy()
    await test_cc_stream()
    await test_cc_telemetry()
    await test_run_loop()
    await test_back_pressure()
    await test_wire_vectors()
    await test_wrap_negative()
    await test_drain_allocation()
    test_session()
    await test_session_index()
    await test_session_anchor()
    await test_index_run_loop()
    await test_boot_row_lost()


test_ring()
asyncio.run(_amain())
print('ok: recorder SPSC ring + readinto, async drain/priority, log-drop vs tlm-raise, Telemetry, cc log+tlm '
      'stream, run loop, high-water back-pressure, wire wrapper vectors, zero-allocation drain, session prefix '
      '(label rule + cap), session.csv boot/time-set/anchor rows, dated or not +/-')
