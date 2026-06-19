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

    # command() times out (raises) on a wedged board instead of hanging
    raised = False
    try:
        await Board(_HangReader(), _Writer()).command('ping', timeout=0.1)
    except asyncio.TimeoutError:
        raised = True
    assert raised

    print('ok: board lockstep command / identify / disconnect / timeout')


asyncio.run(main())
