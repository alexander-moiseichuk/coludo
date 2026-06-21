# Host (CPython) test for board.py: the Board lockstep (command / identify / disconnect / timeout)
# over fake streams. Run by `make test` in this dir (or python3 test_board.py).

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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

    print('ok: board lockstep command / identify / disconnect / timeout')


asyncio.run(main())
