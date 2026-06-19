# Host (CPython) test for server.py: the hub — board accept/handshake (loopback), the operator
# console (list / route / select / broadcast / Control commands), and the web bridge (api/boards,
# api/cmd, events) — all over a real loopback. Run by `make test`.

import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cc_protocol as cc  # noqa: E402
import server  # noqa: E402

PORT = 18234
BOARD_PORT = 18235
OPERATOR_PORT = 18236
WEB_PORT = 18237
WEB_BOARD_PORT = 18238
WEB_OPERATOR_PORT = 18239


async def _fake_board(reader, writer):
    """A minimal board: answers whoami/ping/inspect over the socket."""
    while True:
        line = await reader.readline()
        if not line:
            return
        msg = cc.parse(line.decode().strip())
        if msg.command == 'whoami':
            info = {'mcu': 'esp32p4', 'firmware_version': 'a1b2c3', 'stage': 'setting', 'config_id': 'abc123'}
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

    hub = server.Server(host='127.0.0.1', port=PORT, on_board=on_board, log=lambda message: None)
    server_task = asyncio.create_task(hub.serve_forever())
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
    hub = server.Server(host='127.0.0.1', port=BOARD_PORT, operator_port=OPERATOR_PORT,
                        web_port=WEB_PORT, log=lambda message: None, heartbeat_s=0.05)
    hub_task = asyncio.create_task(hub.run())
    await asyncio.sleep(0.1)

    board_reader, board_writer = await asyncio.open_connection('127.0.0.1', BOARD_PORT)
    board_task = asyncio.create_task(_fake_board(board_reader, board_writer))
    for _ in range(50):  # wait for the handshake to register it
        if 'glider9' in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider9' in hub.boards

    operator_reader, operator_writer = await asyncio.open_connection('127.0.0.1', OPERATOR_PORT)

    async def ask(text):
        operator_writer.write((text + '\n').encode())
        await operator_writer.drain()
        return (await asyncio.wait_for(operator_reader.readline(), 2)).decode().strip()

    try:
        # Control command: list shows the online board with its iam-reported stage/config_id
        listing = await ask('list')
        assert listing.startswith('from cc ok ')
        rows = json.loads(listing[len('from cc ok '):])
        assert rows[0] == {'id': 'glider9', 'online': True, 'stage': 'setting', 'config_id': 'abc123'}

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

        # broadcast to every online board (only `all` -- `*` is gone)
        assert await ask('all ping') == 'from glider9 pong'
    finally:
        operator_writer.close()
        hub_task.cancel()
        board_task.cancel()


async def _http(port, method, path, body=None):
    """A tiny raw HTTP/1.1 client: send one request, read the (Connection: close) response."""
    reader, writer = await asyncio.open_connection('127.0.0.1', port)
    data = body.encode() if isinstance(body, str) else (body or b'')
    request = '%s %s HTTP/1.1\r\nHost: t\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n' % (
        method, path, len(data))
    writer.write(request.encode() + data)
    await writer.drain()
    raw = await asyncio.wait_for(reader.read(), 2)  # Connection: close -> read to EOF
    writer.close()
    head, _, payload = raw.partition(b'\r\n\r\n')
    return int(head.split()[1]), payload


async def _web():
    """The browser bridge on 8080: dashboard, /api/boards, /api/cmd routing, and /events SSE."""
    hub = server.Server(host='127.0.0.1', port=WEB_BOARD_PORT, operator_port=WEB_OPERATOR_PORT,
                        web_port=WEB_PORT, log=lambda message: None, heartbeat_s=0.05)
    hub_task = asyncio.create_task(hub.run())
    await asyncio.sleep(0.1)

    board_reader, board_writer = await asyncio.open_connection('127.0.0.1', WEB_BOARD_PORT)
    board_task = asyncio.create_task(_fake_board(board_reader, board_writer))
    for _ in range(50):
        if 'glider9' in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider9' in hub.boards

    try:
        # GET / serves the dashboard page
        status, page = await _http(WEB_PORT, 'GET', '/')
        assert status == 200 and b'Coludo Control' in page

        # GET /api/boards is the registry as JSON (same data as the `list` command)
        status, payload = await _http(WEB_PORT, 'GET', '/api/boards')
        assert status == 200
        rows = json.loads(payload)
        assert rows[0]['id'] == 'glider9' and rows[0]['online'] is True and rows[0]['stage'] == 'setting'

        # POST /api/cmd routes to the board and returns its reply
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'ping'}))
        assert status == 200 and json.loads(payload)['status'] == 'pong'

        # POST to an unknown board is a 404
        status, _payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                       json.dumps({'board': 'ghost', 'command': 'ping'}))
        assert status == 404

        # GET /events streams the board list as Server-Sent Events
        events_reader, events_writer = await asyncio.open_connection('127.0.0.1', WEB_PORT)
        events_writer.write(b'GET /events HTTP/1.1\r\nHost: t\r\n\r\n')
        await events_writer.drain()
        frame = await asyncio.wait_for(events_reader.readuntil(b'\n\n'), 2)
        assert b'text/event-stream' in frame and b'data: [' in frame and b'glider9' in frame
        events_writer.close()
    finally:
        hub_task.cancel()
        board_task.cancel()


async def main():
    await _loopback()
    await _operator_console()
    await _web()
    print('ok: server accept (loopback) + operator console + web bridge (api/boards, api/cmd, events)')


asyncio.run(main())
