# Host (CPython) test for the `calibrate` operator command: it drives a fake board's `bustune` ladder
# and must (a) keep the ceiling when the sweep is clean, (b) back off `margin` steps below a real
# failure and name the limiting device, (c) restore the bus to its configured freq, (d) guard bad args
# / offline boards. No hardware -- a fake board answers get-config + bustune.

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from commands import calibrate  # noqa: E402


class _Resp:
    def __init__(self, command, args):
        self.command = command
        self.args = args


class _FakeBoard:
    """Answers get-config + bustune. `fails_at` is the freq where `culprit` stops being healthy."""

    def __init__(self, kind, ident, configured, fails_at=None, culprit=None):
        self.online = True
        self.kind, self.ident, self.configured = kind, ident, configured
        self.fails_at, self.culprit = fails_at, culprit
        self.calls = []

    async def command(self, cmd, *args):
        self.calls.append((cmd, tuple(args)))
        if cmd == 'get-config':
            key = 'freq' if self.kind == 'i2c' else 'baud'
            return _Resp('ok', [json.dumps({'buses': {self.kind: {self.ident: {key: self.configured}}}})])
        if cmd == 'bustune':
            kind, ident, freq = args[0], args[1], int(args[2])
            ok = self.fails_at is None or freq < self.fails_at
            devices = {'dev_a': 'ok', (self.culprit or 'dev_b'): 'ok' if ok else 'who_am_i mismatch'}
            return _Resp('ok', [json.dumps({'kind': kind, 'id': ident, 'freq': freq,
                                            'devices': devices, 'all_ok': ok})])
        return _Resp('ok', [])


class _Hub:
    def __init__(self, board):
        self.boards = {'taster': board}


def _run(hub, tokens):
    [line] = asyncio.run(calibrate.calibrate_command(hub, tokens, {}))
    return line


def test_clean_sweep_keeps_ceiling():
    # the whole ladder passes -> chosen == ceiling (top rung), no backoff, no limiter
    board = _FakeBoard('i2c', '0', 400000)
    out = json.loads(_run(_Hub(board), ['calibrate', 'taster', 'i2c', '0'])[len('from cc ok '):])
    assert out['ceiling'] == 1000000 and out['chosen'] == 1000000 and out['limiter'] is None
    assert board.calls[-1] == ('bustune', ('i2c', '0', '400000'))  # restored to the configured freq


def test_limited_sweep_backs_off_and_names_limiter():
    # a device fails at 20M -> ceiling is the last pass (16M); chosen is `margin` steps below it
    board = _FakeBoard('spi', '1', 5000000, fails_at=20000000, culprit='accel_adxl375')
    out = json.loads(_run(_Hub(board), ['calibrate', 'taster', 'spi', '1'])[len('from cc ok '):])
    assert out['ceiling'] == 16000000 and out['chosen'] == 10000000  # MAX-1 step below the ceiling
    assert out['limiter'] == {'freq': 20000000, 'failed': ['accel_adxl375']}
    assert board.calls[-1] == ('bustune', ('spi', '1', '5000000'))  # restored

    # margin=2 backs off two steps (16M ceiling -> 8M)
    board2 = _FakeBoard('spi', '1', 5000000, fails_at=20000000, culprit='accel_adxl375')
    out2 = json.loads(_run(_Hub(board2), ['calibrate', 'taster', 'spi', '1', '2'])[len('from cc ok '):])
    assert out2['chosen'] == 8000000


def test_guards():
    # bad kind / missing args -> badargs; an offline board -> noboard
    assert 'badargs' in _run(_Hub(_FakeBoard('i2c', '0', 400000)), ['calibrate', 'taster', 'usb', '0'])
    assert 'badargs' in _run(_Hub(_FakeBoard('i2c', '0', 400000)), ['calibrate', 'taster'])
    hub = _Hub(_FakeBoard('i2c', '0', 400000))
    hub.boards['taster'].online = False
    assert 'noboard' in _run(hub, ['calibrate', 'taster', 'i2c', '0'])


test_clean_sweep_keeps_ceiling()
test_limited_sweep_backs_off_and_names_limiter()
test_guards()
print('ok: calibrate -- clean-sweep keeps ceiling, limited-sweep backs off + names limiter, restore, guards')
