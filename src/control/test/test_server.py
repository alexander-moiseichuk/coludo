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
GPS_BOARD_PORT = 18240
GPS_OPERATOR_PORT = 18241
GPS_WEB_PORT = 18242
LOG_BOARD_PORT = 18243
LOG_OPERATOR_PORT = 18244
LOG_WEB_PORT = 18245


def _nmea(body):
    """`$<body>*hh` with a correct XOR checksum — for feeding a synthetic fix to gps.Gps in tests."""
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '$%s*%02X' % (body, checksum)


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
        elif msg.command == 'get-config':
            reply = cc.build('ok', [json.dumps({'board': {'id': 'glider9', 'mcu': 'esp32p4'}, 'sensors': []})])
        elif msg.command == 'save-config':
            json.loads(msg.args[0])  # the config arrives base64-decoded by parse
            reply = cc.build('ok', [json.dumps({'config_id': 'newcfg'})])
        elif msg.command == 'update':
            reply = cc.build('ok', [json.dumps({'changed': sorted(json.loads(msg.args[1]))})])
        elif msg.command == 'log':  # poll-model log streaming: one canned line per armed window
            window = int(msg.args[0]) if msg.args else 0
            lines = ['100 test :: tick'] if window > 0 else []
            reply = cc.build('ok', [json.dumps({'lines': lines})])
        elif msg.command in ('reset-config', 'reboot', 'save-mission'):
            reply = cc.build('ok')
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

        # that inspect reply was cached Control-side; `cache` shows it without re-polling the board
        cached = await ask('cache glider9')
        assert cached.startswith('from cc ok '), cached
        props = json.loads(cached[len('from cc ok '):])
        assert props['id'] == 'glider9' and props['inspect']['wifi'] == {'name': 'wifi', 'ok': True}

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

        # dashboard config flow over /api/cmd: get-config -> edit the draft -> save-config -> reboot
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'get-config'}))
        assert status == 200
        config = json.loads(json.loads(payload)['args'][0])  # the board's config, ready to edit
        assert config['board']['id'] == 'glider9'
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'save-config',
                                                  'params': [json.dumps(config)]}))
        assert status == 200 and json.loads(payload) == {'board': 'glider9', 'status': 'ok',
                                                         'args': [json.dumps({'config_id': 'newcfg'})]}
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'reboot'}))
        assert status == 200 and json.loads(payload)['status'] == 'ok'

        # /api/board/<id> serves the Control-side cache (config was cached by the get-config above)
        status, payload = await _http(WEB_PORT, 'GET', '/api/board/glider9')
        assert status == 200
        props = json.loads(payload)
        assert props['id'] == 'glider9' and props['config']['board']['id'] == 'glider9'
        status, _payload = await _http(WEB_PORT, 'GET', '/api/board/ghost')  # unknown board -> 404
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


async def _gps_assist():
    """A host GPS with a usable 3D fix: `gps` reports it and `assist <board>` pushes the position to
    the board's mission (update mission -> save-mission)."""
    import gps as gps_mod
    host_gps = gps_mod.Gps(log=lambda message: None)
    host_gps.feed(_nmea('GPGSA,A,3,01,02,03,04,05,06,,,,,,,2.0,1.0,1.5'))  # 3D fix
    host_gps.feed(_nmea('GPGGA,123519,4807.038,N,01131.000,E,1,06,0.9,545.4,M,46.9,M,,'))  # 6 sats
    assert host_gps.position() is not None

    hub = server.Server(host='127.0.0.1', port=GPS_BOARD_PORT, operator_port=GPS_OPERATOR_PORT,
                        web_port=GPS_WEB_PORT, log=lambda message: None, heartbeat_s=0.05, gps=host_gps)
    hub_task = asyncio.create_task(hub.run())
    await asyncio.sleep(0.1)

    board_reader, board_writer = await asyncio.open_connection('127.0.0.1', GPS_BOARD_PORT)
    board_task = asyncio.create_task(_fake_board(board_reader, board_writer))
    for _ in range(50):
        if 'glider9' in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider9' in hub.boards

    operator_reader, operator_writer = await asyncio.open_connection('127.0.0.1', GPS_OPERATOR_PORT)

    async def ask(text):
        operator_writer.write((text + '\n').encode())
        await operator_writer.drain()
        return (await asyncio.wait_for(operator_reader.readline(), 2)).decode().strip()

    try:
        # gps status shows the usable 3D fix and satellite count
        reply = await ask('gps')
        assert reply.startswith('from cc ok '), reply
        status = json.loads(reply[len('from cc ok '):])
        assert status['usable'] and status['fix_3d'] and status['satellites'] == 6, status

        # assist pushes the host position to the board mission and persists it (save-mission)
        reply = await ask('assist glider9')
        assert reply.startswith('from cc ok '), reply
        out = json.loads(reply[len('from cc ok '):])
        assert out['assisted'] == 'glider9' and out['saved'] is True, out
        assert abs(out['position']['latitude'] - 48.1173) < 1e-3, out
    finally:
        operator_writer.close()
        hub_task.cancel()
        board_task.cancel()


async def _log_stream():
    """Operator enables `<board> log <ms>` (board-first, like `<board> ping`): the hub polls the
    board's `log` buffer and surfaces each line to the console (`<id>: <line>`) and the /logs SSE
    feed; `<board> log off` stops it (and tells the board to stop collecting with a final `log 0`)."""
    seen = []
    hub = server.Server(host='127.0.0.1', port=LOG_BOARD_PORT, operator_port=LOG_OPERATOR_PORT,
                        web_port=LOG_WEB_PORT, log=seen.append, heartbeat_s=5.0)
    hub_task = asyncio.create_task(hub.run())
    await asyncio.sleep(0.1)

    board_reader, board_writer = await asyncio.open_connection('127.0.0.1', LOG_BOARD_PORT)
    board_task = asyncio.create_task(_fake_board(board_reader, board_writer))
    for _ in range(50):
        if 'glider9' in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider9' in hub.boards

    operator_reader, operator_writer = await asyncio.open_connection('127.0.0.1', LOG_OPERATOR_PORT)

    async def ask(text):
        operator_writer.write((text + '\n').encode())
        await operator_writer.drain()
        return (await asyncio.wait_for(operator_reader.readline(), 2)).decode().strip()

    try:
        # subscribe to /logs SSE first, so it sees the same lines the console gets
        sse_reader, sse_writer = await asyncio.open_connection('127.0.0.1', LOG_WEB_PORT)
        sse_writer.write(b'GET /logs HTTP/1.1\r\nHost: t\r\n\r\n')
        await sse_writer.drain()
        await asyncio.wait_for(sse_reader.readuntil(b'\r\n\r\n'), 2)  # consume the SSE response headers

        # dashboard path: POST /api/log starts the same hub stream (and /logs SSE carries the lines)
        status, payload = await _http(LOG_WEB_PORT, 'POST', '/api/log',
                                      json.dumps({'board': 'glider9', 'interval_ms': 40}))
        assert status == 200 and json.loads(payload) == {'board': 'glider9', 'streaming': True,
                                                         'interval_ms': 40}, payload
        assert 'glider9' in hub.streams
        status, payload = await _http(LOG_WEB_PORT, 'POST', '/api/log',
                                      json.dumps({'board': 'glider9', 'interval_ms': 0}))
        assert status == 200 and json.loads(payload) == {'board': 'glider9', 'streaming': False}
        assert 'glider9' not in hub.streams

        reply = await ask('glider9 log 30')
        assert reply.startswith('from glider9 ok ') and '"interval_ms": 30' in reply, reply

        # the hub polls the board and surfaces each line as `<id>: <line>` on the console
        for _ in range(50):
            if 'glider9: 100 test :: tick' in seen:
                break
            await asyncio.sleep(0.02)
        assert 'glider9: 100 test :: tick' in seen, seen[-5:]

        # the same line arrives on the /logs SSE feed
        frame = await asyncio.wait_for(sse_reader.readuntil(b'\n\n'), 2)
        assert b'"board": "glider9"' in frame and b'test :: tick' in frame, frame

        # stop -> the board is told to stop collecting (log 0) and no streaming task remains
        reply = await ask('glider9 log off')
        assert reply.startswith('from glider9 ok ') and '"log": "off"' in reply, reply
        assert 'glider9' not in hub.streams
        sse_writer.close()
    finally:
        operator_writer.close()
        hub_task.cancel()
        board_task.cancel()


async def main():
    await _loopback()
    await _operator_console()
    await _web()
    await _gps_assist()
    await _log_stream()
    print('ok: server accept (loopback) + operator console + web bridge (api/boards, api/cmd, events) '
          '+ gps assist + log streaming')


asyncio.run(main())
