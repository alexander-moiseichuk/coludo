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
BOARD_PORT = 18235
OPERATOR_PORT = 18236


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
            info = {'mcu': 'esp32p4', 'fw': '0.1', 'state': 'setting', 'config_id': 'abc123'}
            reply = cc.build('iam', ['glider9', json.dumps(info)])
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


async def _operator_console():
    """A board dials in; an operator drives it through the telnet console: list / route / select /
    broadcast / Control commands, with replies tagged by source."""
    server = control.Server(host='127.0.0.1', port=BOARD_PORT, operator_port=OPERATOR_PORT,
                            log=lambda message: None, heartbeat_s=0.05)
    hub_task = asyncio.create_task(server.run())
    await asyncio.sleep(0.1)

    board_reader, board_writer = await asyncio.open_connection('127.0.0.1', BOARD_PORT)
    board_task = asyncio.create_task(_fake_board(board_reader, board_writer))
    for _ in range(50):  # wait for the handshake to register it
        if 'glider9' in server.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider9' in server.boards

    operator_reader, operator_writer = await asyncio.open_connection('127.0.0.1', OPERATOR_PORT)

    async def ask(text):
        operator_writer.write((text + '\n').encode())
        await operator_writer.drain()
        return (await asyncio.wait_for(operator_reader.readline(), 2)).decode().strip()

    try:
        # Control command: list shows the online board with its iam-reported state/config_id
        listing = await ask('list')
        assert listing.startswith('from cc ok ')
        rows = json.loads(listing[len('from cc ok '):])
        assert rows[0] == {'id': 'glider9', 'online': True, 'state': 'setting', 'config_id': 'abc123'}

        # an unknown first token (no selection yet) is a bad Control command, never sent to a board
        assert await ask('bogus') == 'from cc err badcmd bogus'
        # help is served from the commands/ registry (every registered command appears)
        helped = await ask('help')
        assert helped.startswith('from cc ok ')
        assert {'help', 'list', 'select', 'who'} <= set(json.loads(helped[len('from cc ok '):]))

        # explicit-target routing, reply tagged by source
        assert await ask('glider9 ping') == 'from glider9 pong'
        # structured payloads render as readable JSON (base64 decoded by Control)
        inspected = await ask('glider9 inspect wifi')
        assert inspected.startswith('from glider9 ok ') and '"name": "wifi"' in inspected

        # sticky select -> a bare command routes to the selected board
        assert await ask('select glider9') == 'from cc ok {"selected": "glider9"}'
        assert await ask('who') == 'from cc ok {"selected": "glider9"}'
        assert await ask('ping') == 'from glider9 pong'

        # broadcast to every online board
        assert await ask('all ping') == 'from glider9 pong'
    finally:
        operator_writer.close()
        hub_task.cancel()
        board_task.cancel()


async def main():
    await _unit()
    await _loopback()
    await _operator_console()
    print('ok: control Board lockstep + Server accept + operator console (list/route/select/broadcast)')


asyncio.run(main())
