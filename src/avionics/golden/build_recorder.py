"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture recorder.json: plan B's Telemetry decimation (src/glider/recorder.py `Telemetry.due` and `push`) as
the Rust port (crates/recorder `Telemetry`) must answer it. The line text is recorder_wire.json's; this fixture pins
which pushes emit a row at all.

Each case is one stream on a fresh Recorder: its own `decimate_us` (> 0 its window, 0 the global
`recorder.telemetry_ms`, < 0 never decimated), the global `telemetry_ms`, and the uptimes it is pushed at, in us,
each a whole millisecond past the stream's construction. For every push, plan B's answer: whether `due()` said a row
was due just before, and whether the push emitted one. Plan B's first push always emits (it seeds its last row one
whole window back), and so does the Rust one (it has no last row yet).

The clock is the shims' virtual one, moved by whole milliseconds, so the uptimes stay far below the 2**30 ticks wrap
(the Rust uptime is a u64 and never wraps). asyncio.ThreadSafeFlag, which CPython lacks and Recorder.setup builds, is
stood in for below, as build_recorder_wire.py does; shims.py says the first builder that needs it should add it there,
and these builders may not edit it, so it is still to move.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import random
import time

import config_default
import recorder
import shims

_SEED: int = 20261006  # the jittered cases' random.Random seed
_STREAM: str = 'imu.csv'  # every case's stream file
_FIELDS: tuple = ('index',)  # its one field: the push's index, so an emitted row names its push


class _Wire:
    """Stands in for the recorder UART's StreamWriter, as test_recorder's FakeWriter does."""

    def __init__(self) -> None:
        self.lines: list = []
        self.out_buf: bytes = b''

    def write(self, data: bytes) -> None:
        """Take one wrapped line, as the UART would."""
        self.lines.append(bytes(data))

    async def drain(self) -> None:
        """The UART caught up."""
        self.out_buf = b''


class _Flag:
    """asyncio.ThreadSafeFlag, which CPython lacks: Recorder.setup builds one, producers set it, nothing waits."""

    def set(self) -> None:
        """A producer queued a record."""


def _rows(wire: _Wire) -> int:
    """
    How many data rows a drain writes now (the header aside); the stand-in UART is emptied.

    Args:
        wire - the stand-in UART.

    Returns:
        The count of drained lines whose payload starts with a digit (a row's uptime; the header starts 'uptime').
    """
    asyncio.run(recorder.Recorder.drain())
    lines, wire.lines = wire.lines, []
    return sum(1 for line in lines if line.split(b'@', 2)[2].split(b';', 2)[1][:1].isdigit())


def _case(name: str, decimate_us: int, telemetry_ms: int, offsets_ms: list) -> dict:
    """
    One stream through plan B's Telemetry.

    Args:
        name - the case name.
        decimate_us - the stream's own setting.
        telemetry_ms - the global recorder.telemetry_ms.
        offsets_ms - when each push happens, in ms after the stream is built; never decreasing.

    Returns:
        The case: input {decimate_us, telemetry_ms, pushes_us}, output [{due, emitted}] per push.
    """
    wire = _Wire()
    recorder.Recorder.boot_id = 123
    config = config_default.default()
    config['recorder']['telemetry_ms'] = telemetry_ms
    recorder.Recorder.setup(config, uart=wire)
    _rows(wire)  # nothing queued yet, but a setup log line would be
    telemetry = recorder.Telemetry(_STREAM, _FIELDS, decimate_us)
    built = time.ticks_us()
    pushes, steps, elapsed = [], [], 0
    for index, offset in enumerate(offsets_ms):
        shims.advance(offset - elapsed)
        elapsed = offset
        pushes.append(time.ticks_us())
        due = telemetry.due(time.ticks_us())
        telemetry.push((index,))
        steps.append({'due': due, 'emitted': _rows(wire) == 1})
    if pushes[-1] - built >= 1 << 29:
        raise ValueError('%s: the pushes reach the ticks wrap' % name)
    return {'name': name, 'input': {'decimate_us': decimate_us, 'telemetry_ms': telemetry_ms, 'pushes_us': pushes},
            'output': steps}


def _jitter(seed: int, pushes: int, longest_ms: int) -> list:
    """
    Push times with seeded gaps of 0 to `longest_ms` ms (0: two pushes at one uptime).

    Args:
        seed - the case's seed, offset from _SEED.
        pushes - how many.
        longest_ms - the longest gap.

    Returns:
        The offsets in ms, from 0.
    """
    draw = random.Random(_SEED + seed)
    offsets, at = [], 0
    for _push in range(pushes):
        offsets.append(at)
        at += draw.randint(0, longest_ms)
    return offsets


"""(name, decimate_us, telemetry_ms, offsets_ms): one of each setting, then the edges and seeded jitter."""
_CASES: tuple = (
    ('own_window_every_10_ms', 50000, 20, list(range(0, 201, 10))),
    ('global_window_every_7_ms', 0, 20, list(range(0, 141, 7))),
    ('no_global_window_every_push_emits', 0, 0, [0, 0, 1, 1, 1, 5, 9]),
    ('never_decimated_beside_a_global_window', -1, 20, [0, 0, 1, 2, 3, 30, 31]),
    ('own_window_wins_over_a_longer_global', 30000, 100, list(range(0, 151, 10))),
    ('own_window_wins_over_a_shorter_global', 30000, 5, list(range(0, 151, 10))),
    ('first_push_long_after_construction', 25000, 0, [5000, 5001, 5025, 5049, 5050]),
    ('first_push_at_construction', 25000, 0, [0, 24, 25, 26, 50]),
    ('exactly_one_window_apart', 1000, 0, [0, 1, 2, 3, 4]),
    ('a_microsecond_window', 1, 0, [0, 0, 1, 1]),
    ('jitter_25_ms_window', 25000, 0, _jitter(1, 60, 30)),
    ('jitter_global_10_ms', 0, 10, _jitter(2, 60, 12)),
    ('jitter_never_decimated', -1000, 50, _jitter(3, 30, 20)),
)


def build() -> dict:
    """
    The recorder fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, cases}.
    """
    asyncio.ThreadSafeFlag = _Flag
    return {
        'meta': {'source': 'src/glider/recorder.py'},
        'compare': {'kind': 'exact'},
        'cases': [_case(*case) for case in _CASES],
    }
