"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for board.py: the Board lockstep (command / identify / disconnect / timeout)
over fake streams. Run by `make test` in this dir (or python3 test_board.py).
"""

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board as board_module  # noqa: E402
import cc_protocol as cc  # noqa: E402
from board import Board  # noqa: E402  (subject under test)


class _Reader:
    def __init__(self, lines):
        self.lines = [line.encode() for line in lines]
        self.index = 0

    async def readline(self):
        if self.index < len(self.lines):
            value = self.lines[self.index]
            self.index += 1
            return value
        return b''


class _HangReader:
    async def readline(self):
        await asyncio.sleep(60)
        return b''


class _Writer:
    def __init__(self):
        self.out = []

    def write(self, data):
        self.out.append(data)

    async def drain(self):
        pass

    def get_extra_info(self, key):
        return ('1.2.3.4', 5)

    def close(self):
        pass


async def main():
    # command() returns the parsed response
    board = Board(_Reader(['pong\n']), _Writer())
    assert (await board.command('ping')).command == 'pong'

    # command() returns None on disconnect (empty readline)
    assert await Board(_Reader([]), _Writer()).command('ping') is None

    """
    A GARBLED reply costs the reply, not the board.

    cc.parse() raises binascii.Error on a corrupt `base64:` token, and exchange() called it bare. That
    exception is not in server._handle's caught set, so it reached the generic handler: traceback
    logged, `finally` ran, stream dropped, board marked OFFLINE. One flipped bit on the link removed a
    board from the hub.

    The board side already guards the mirror case on purpose -- a garbled line over a lossy field radio
    must survive -- so the asymmetry was the bug: identical corruption was survivable inbound and fatal
    outbound. A dropped reply reads as None, which every caller already handles because a timeout
    produces the same thing.
    """
    garbled = Board(_Reader(['ok base64:!!!not-base64!!!\n']), _Writer())
    assert await garbled.command('health') is None       # survived; no exception escaped
    # NEGATIVE: a WELL-FORMED base64 reply must still parse, or the guard is hiding real replies
    good = Board(_Reader([cc.build('ok', [json.dumps({'temp': 41})]) + '\n']), _Writer())
    assert (await good.command('health')).command == 'ok'

    # identify() learns the id + info from iam
    iam = cc.build('iam', ['glider2', json.dumps({'mcu': 'esp32p4'})])
    board = Board(_Reader([iam + '\n']), _Writer())
    assert await board.identify() == 'glider2' and board.info['mcu'] == 'esp32p4'

    # identify() returns None when the reply is not iam
    assert await Board(_Reader(['pong\n']), _Writer()).identify() is None

    # exchange() logs both directions (tx '->' then rx '<-') through the optional log hook
    captured = []
    board = Board(_Reader(['pong\n']), _Writer(), log=captured.append)
    board.id = 'glider2'
    await board.command('ping')
    assert any('glider2 -> ping' in m for m in captured), captured
    assert any('glider2 <- pong' in m for m in captured), captured

    # a disconnect during exchange is logged as a received '<disconnected>' marker
    captured = []
    assert await Board(_Reader([]), _Writer(), log=captured.append).command('ping') is None
    assert any('<- <disconnected>' in m for m in captured), captured

    # exchange caches board-state replies for the dashboard: get-config / inspect / stats
    cfg = cc.build('ok', [json.dumps({'board': {'id': 'glider2'}})])
    insp = cc.build('ok', [json.dumps({'name': 'wifi', 'ok': True})])
    stat = cc.build('ok', [json.dumps({'rx': 5})])
    board = Board(_Reader([cfg + '\n', insp + '\n', stat + '\n']), _Writer())
    await board.command('get-config')
    await board.command('inspect', 'wifi')
    await board.command('stats', 'wifi')
    props = board.properties()
    assert props['config'] == {'board': {'id': 'glider2'}}, props
    assert props['inspect']['wifi'] == {'name': 'wifi', 'ok': True}, props
    assert props['stats']['wifi'] == {'rx': 5}, props

    # `get-config default` is the built-in default, NOT the running config -> not cached as config
    board = Board(_Reader([cfg + '\n']), _Writer())
    await board.command('get-config', 'default')
    assert board.properties()['config'] is None

    # an err reply never pollutes the cache
    board = Board(_Reader([cc.build('err', ['badargs', 'no object x']) + '\n']), _Writer())
    await board.command('inspect', 'x')
    assert board.properties()['inspect'] == {}

    # command() times out (raises) on a wedged board instead of hanging
    raised = False
    try:
        await Board(_HangReader(), _Writer()).command('ping', timeout=0.1)
    except asyncio.TimeoutError:
        raised = True
    assert raised

    """
    A TIMEOUT MUST GIVE THE LINK UP, not just mark it. It set online=False and left the socket open, so
    nothing reconnected: the board kept its end (possibly armed) while the hub refused every command to
    an "offline" board -- disarm included.
    """
    class _TrackingWriter(_Writer):
        closed = False

        def close(self):
            self.closed = True

    tracked = _TrackingWriter()
    stuck = Board(_HangReader(), tracked)
    try:
        await stuck.command('ping', timeout=0.1)
        raise AssertionError('a hung reply did not time out')
    except asyncio.TimeoutError:
        pass
    assert stuck.online is False and tracked.closed is True, 'timed out but kept the socket open'

    """
    The active self-tests get a longer default. probe_all sweeps every servo; on three servos it was
    estimated at 8.5-10 s against the 10 s default, so an `arm` could time out while still landing.
    """
    seen = {}

    async def _capture(line, timeout, quiet=False):
        seen[line.split()[0]] = timeout
        return cc.parse('ok')

    probe = Board(_Reader([]), _Writer())
    probe.exchange = _capture
    for verb in ('arm', 'verify', 'probe', 'calibrate'):
        await probe.command(verb)
        assert seen[verb] >= 30.0, '%s got %s s' % (verb, seen[verb])
    await probe.command('ping')
    assert seen['ping'] == 10.0, 'ordinary commands keep the 10 s default'
    await probe.command('arm', timeout=2.0)
    assert seen['arm'] == 2.0, 'an explicit timeout still wins'
    # ...and a RAW line, as the operator console forwards it, gets the same deadline: a typed `arm`
    # used to get the plain 10 s, and a timeout drops the link
    assert board_module.timeout_for('arm') >= 30.0 and board_module.timeout_for('probe servo_yaw') >= 30.0
    assert board_module.timeout_for('ping') == 10.0 and board_module.timeout_for('armed') == 10.0

    """
    health_seen tracks HEALTH replies only. last_seen moves on any reply, and the heartbeat used to
    treat a running log stream as proof of health -- freezing armed/stage on the dashboard.
    """
    live = Board(_Reader(['ok e30=', 'ok']), _Writer())
    assert live.health_seen == 0.0
    await live.command('health')
    assert live.health_seen > 0.0
    before = live.health_seen
    await live.command('log', 1000)  # stream traffic refreshes last_seen, NOT health_seen
    assert live.health_seen == before

    print('ok: board lockstep command / identify / disconnect / timeout / garbled reply +/- '
          '/ timeout closes the socket / slow-command timeouts / health_seen is health-only')


asyncio.run(main())
