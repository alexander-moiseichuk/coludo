# Host (CPython) test for the Control ground station (control.py): Board lockstep + Server accept
# over a real loopback. Run by `make test` in this dir (or python3 test_control.py).

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cc_protocol as cc  # noqa: E402

import control  # noqa: E402

PORT = 18234


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


async def _unit():
    # command() returns the parsed response
    board = control.Board(_Reader(['pong\n']), _Writer())
    assert (await board.command('ping')).command == 'pong'

    # command() returns None on disconnect (empty readline)
    assert await control.Board(_Reader([]), _Writer()).command('ping') is None

    # identify() learns the id + info from iam
    iam = cc.build('iam', ['glider2', json.dumps({'mcu': 'esp32p4'})])
    board = control.Board(_Reader([iam + '\n']), _Writer())
    assert await board.identify() == 'glider2' and board.info['mcu'] == 'esp32p4'

    # identify() returns None when the reply is not iam
    assert await control.Board(_Reader(['pong\n']), _Writer()).identify() is None

    # command() times out (raises) on a wedged board instead of hanging
    raised = False
    try:
        await control.Board(_HangReader(), _Writer()).command('ping', timeout=0.1)
    except asyncio.TimeoutError:
        raised = True
    assert raised


async def _fake_board(reader, writer):
    """A minimal board: answers whoami/ping/inspect over the socket."""
    while True:
        line = await reader.readline()
        if not line:
            return
        msg = cc.parse(line.decode().strip())
        if msg.command == 'whoami':
            reply = cc.build('iam', ['glider9', json.dumps({'mcu': 'esp32p4', 'fw': '0.1'})])
        elif msg.command == 'ping':
            reply = cc.build('pong')
        elif msg.command == 'inspect':
            reply = cc.build('ok', [json.dumps({'name': msg.args[0], 'ok': True})])
        else:
            reply = cc.build('err', ['badcmd', msg.command])
        writer.write((reply + '\n').encode())
        await writer.drain()


async def _loopback():
    result = {}
    done = asyncio.Event()

    async def on_board(board):
        try:
            assert board.id == 'glider9' and board.info['mcu'] == 'esp32p4'
            result['pong'] = (await board.command('ping')).command
            result['wifi'] = await board.inspect('wifi')
        finally:
            done.set()

    server = control.Server(host='127.0.0.1', port=PORT, on_board=on_board, log=lambda message: None)
    server_task = asyncio.create_task(server.serve_forever())
    await asyncio.sleep(0.1)

    reader, writer = await asyncio.open_connection('127.0.0.1', PORT)
    board_task = asyncio.create_task(_fake_board(reader, writer))
    try:
        await asyncio.wait_for(done.wait(), timeout=5)
    finally:
        server_task.cancel()
        board_task.cancel()

    assert result['pong'] == 'pong'
    assert result['wifi'] == {'name': 'wifi', 'ok': True}


async def main():
    await _unit()
    await _loopback()
    print('ok: control Board identify/command/inspect + Server accept (loopback)')


asyncio.run(main())
