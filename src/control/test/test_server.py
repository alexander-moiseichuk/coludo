"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for server.py: the hub -- board accept/handshake (loopback), the operator
console (list / route / select / broadcast / Control commands), and the web bridge (api/boards,
api/cmd, events) -- all over a real loopback. Run by `make test`.
"""

import asyncio
import json
import os
import sys
import tempfile
import time
from collections.abc import Callable

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import board  # noqa: E402
import cc_protocol as cc  # noqa: E402
import gps  # noqa: E402
import server  # noqa: E402

PORT = 18234
BIG_BOARD_PORT = 18291
BIG_OPERATOR_PORT = 18292
BIG_WEB_PORT = 18293
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
COLD_EPOCH: int = 946684800 + 12  # a cold board's RTC: 2000-01-01 plus 12 s of uptime
CC_ZONE: str = 'Asia/Kolkata'  # CC's zone in the clock-sync cases: half-hour, no DST, never UTC by accident
CC_ZONE_OFFSET: int = 330  # its offset EAST of UTC in minutes, the sign utc_offset carries
_ROSTERS: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # each hub's gliders.json; gone at exit


def _isolated_hub(**options) -> server.Server:
    """
    A hub with a roster of its own inside _ROSTERS: no test may write the live hub's src/control/gliders.json.

    Args:
        options - server.Server's keyword arguments, roster_path aside.

    Returns:
        The hub, its gliders.json in a fresh folder.
    """
    return server.Server(roster_path=os.path.join(tempfile.mkdtemp(dir=_ROSTERS.name), 'gliders.json'), **options)


def _nmea(body):
    """`$<body>*hh` with a correct XOR checksum -- for feeding a synthetic fix to gps.Gps in tests."""
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
        elif msg.command == 'health':  # the heartbeat polls this; carries the vitals + board clock + position
            reply = cc.build('ok', [json.dumps({'temp': 40, 'mem_free': 1000, 'uptime': 12345,
                                                'stage': 'setting', 'clock': '2026-06-22T20:00:00',
                                                'position': [48.117, 11.517],
                                                'launchpad': [48.117, 11.517], 'launchpad_set': False,
                                                'site': 'field', 'armed': False,
                                                'agl': 3.2, 'flight': {'airspeed': 14.2, 'fin_cap': 30,
                                                                       'active': True},
                                                'degraded': ['attitude-backup']})])
        elif msg.command == 'inspect':
            reply = cc.build('ok', [json.dumps({'name': msg.args[0], 'ok': True})])
        elif msg.command == 'get-config':
            name = msg.args[0] if msg.args else 'board'
            payload = ({'launch_id': 'l1', 'latitude': None} if name == 'launch'
                       else {'board': {'id': 'glider9', 'mcu': 'esp32p4'}, 'sensors': []})
            reply = cc.build('ok', [json.dumps(payload)])
        elif msg.command == 'set-config':  # <name> <json>
            json.loads(msg.args[1])  # the payload arrives base64-decoded by parse
            reply = cc.build('ok', [json.dumps({'config_id': 'newcfg'})] if msg.args[0] == 'board' else [])
        elif msg.command == 'update':
            reply = cc.build('ok', [json.dumps({'changed': sorted(json.loads(msg.args[1]))})])
        elif msg.command == 'log':  # poll-model log streaming: one canned line per armed window
            window = int(msg.args[0]) if msg.args else 0
            lines = ['100 test :: tick'] if window > 0 else []
            reply = cc.build('ok', [json.dumps({'lines': lines})])
        elif msg.command == 'tlm':  # poll-model telemetry streaming: one canned sample per armed window
            window = int(msg.args[0]) if msg.args else 0
            samples = ['@s_t.csv@1;2;3'] if window > 0 else []
            reply = cc.build('ok', [json.dumps({'samples': samples})])
        elif msg.command in ('reset-config', 'reboot'):
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

    hub = _isolated_hub(host='127.0.0.1', port=PORT, on_board=on_board, log=lambda message: None)
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
    hub = _isolated_hub(host='127.0.0.1', port=BOARD_PORT, operator_port=OPERATOR_PORT,
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
        assert rows[0]['id'] == 'glider9' and rows[0]['online'] is True
        assert rows[0]['stage'] == 'setting' and rows[0]['config_id'] == 'abc123'  # vitals keys also present

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
    hub = _isolated_hub(host='127.0.0.1', port=WEB_BOARD_PORT, operator_port=WEB_OPERATOR_PORT,
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

        # the heartbeat polls health -> board_rows carries live vitals (uptime, clock, position) + version
        for _ in range(50):
            rows = json.loads((await _http(WEB_PORT, 'GET', '/api/boards'))[1])
            if rows[0].get('uptime') is not None:
                break
            await asyncio.sleep(0.02)
        assert rows[0]['uptime'] == 12345 and rows[0]['clock'] == '2026-06-22T20:00:00', rows[0]
        assert rows[0]['version'] == 'a1b2c3' and rows[0]['position'] == [48.117, 11.517], rows[0]
        # the launchpad safety cell fields ride the same heartbeat (dashboard 'no launchpad' warning)
        assert rows[0]['launchpad'] == [48.117, 11.517] and rows[0]['launchpad_set'] is False, rows[0]
        assert rows[0]['site'] == 'field' and rows[0]['armed'] is False, rows[0]
        # the live flight panel fields ride the heartbeat (dashboard airspeed / fin-cap / agl)
        assert rows[0]['agl'] == 3.2 and rows[0]['flight']['airspeed'] == 14.2, rows[0]
        assert rows[0]['flight']['fin_cap'] == 30 and rows[0]['flight']['active'] is True, rows[0]
        assert rows[0]['degraded'] == ['attitude-backup'], rows[0]  # degraded annunciation passes through

        # POST /api/cmd routes to the board and returns its reply
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'ping'}))
        assert status == 200 and json.loads(payload)['status'] == 'pong'

        # POST to an unknown board is a 404
        status, _payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                       json.dumps({'board': 'ghost', 'command': 'ping'}))
        assert status == 404

        """
        POST /api/op runs an operator-console line through the registry (calibrate, list, ...) over the
        same dispatch the telnet console uses -- so an operator command runs, and a board-id-first line
        still routes to the board.
        """
        status, payload = await _http(WEB_PORT, 'POST', '/api/op', json.dumps({'line': 'list'}))
        assert status == 200 and json.loads(payload)['lines'][0].startswith('from cc ok ')  # `list` ran
        status, payload = await _http(WEB_PORT, 'POST', '/api/op', json.dumps({'line': 'glider9 ping'}))
        assert json.loads(payload)['lines'] == ['from glider9 pong']  # board routing via /api/op
        status, _payload = await _http(WEB_PORT, 'POST', '/api/op', json.dumps({}))
        assert status == 400  # an empty line is rejected

        # dashboard config flow over /api/cmd: get-config <name> -> edit the draft -> set-config <name> -> reboot
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'get-config', 'params': ['board']}))
        assert status == 200
        config = json.loads(json.loads(payload)['args'][0])  # the board's config, ready to edit
        assert config['board']['id'] == 'glider9'
        status, payload = await _http(WEB_PORT, 'POST', '/api/cmd',
                                      json.dumps({'board': 'glider9', 'command': 'set-config',
                                                  'params': ['board', json.dumps(config)]}))
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

        # GET /events streams {cc, boards} as Server-Sent Events
        events_reader, events_writer = await asyncio.open_connection('127.0.0.1', WEB_PORT)
        events_writer.write(b'GET /events HTTP/1.1\r\nHost: t\r\n\r\n')
        await events_writer.drain()
        frame = await asyncio.wait_for(events_reader.readuntil(b'\n\n'), 2)
        assert b'text/event-stream' in frame and b'data: {' in frame
        payload = json.loads(frame.split(b'data: ', 1)[1])
        assert 'time' in payload['cc'] and payload['boards'][0]['id'] == 'glider9'
        events_writer.close()
    finally:
        hub_task.cancel()
        board_task.cancel()


async def _gps_assist():
    """A host GPS with a usable 3D fix: `gps` reports it and `assist <board>` pushes the position to
    the board mission (set-config launch: merge + persist)."""
    import gps as gps_module
    host_gps = gps_module.Gps(log=lambda message: None)
    host_gps.feed(_nmea('GPGSA,A,3,01,02,03,04,05,06,,,,,,,2.0,1.0,1.5'))  # 3D fix
    host_gps.feed(_nmea('GPGGA,123519,4807.038,N,01131.000,E,1,06,0.9,545.4,M,46.9,M,,'))  # 6 sats
    assert host_gps.position() is not None

    hub = _isolated_hub(host='127.0.0.1', port=GPS_BOARD_PORT, operator_port=GPS_OPERATOR_PORT,
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

        # `gps <board>` compares the host fix with the board's on-board GNSS (inspect gnss)
        reply = await ask('gps glider9')
        assert reply.startswith('from cc ok '), reply
        compare = json.loads(reply[len('from cc ok '):])
        assert compare['host']['usable'] and compare['board'] == 'glider9'
        assert compare['onboard'] == {'name': 'gnss', 'ok': True}, compare  # the board's inspect gnss

        # assist pushes the host position to the board mission and persists it (set-config launch)
        reply = await ask('assist glider9')
        assert reply.startswith('from cc ok '), reply
        out = json.loads(reply[len('from cc ok '):])
        assert out['assisted'] == 'glider9' and out['saved'] is True, out
        assert abs(out['position']['latitude'] - 48.1173) < 1e-3, out

        # the dashboard 'gps' button: POST /api/assist does the same push (set-config launch)
        status, payload = await _http(GPS_WEB_PORT, 'POST', '/api/assist', json.dumps({'board': 'glider9'}))
        assert status == 200, payload
        pushed = json.loads(payload)
        assert pushed['assisted'] is True and abs(pushed['position']['latitude'] - 48.1173) < 1e-3, pushed
    finally:
        operator_writer.close()
        hub_task.cancel()
        board_task.cancel()


async def _log_stream():
    """
    Operator enables `<board> log <ms>` (board-first, like `<board> ping`): the hub polls the
    board's `log` buffer and surfaces each line to the console (`<id>: <line>`) and the /logs SSE
    feed; `<board> log off` stops it (and tells the board to stop collecting with a final `log 0`).
    """
    seen = []
    hub = _isolated_hub(host='127.0.0.1', port=LOG_BOARD_PORT, operator_port=LOG_OPERATOR_PORT,
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
                                                         'kind': 'log', 'interval_ms': 40}, payload
        assert 'glider9' in hub.streams

        # telemetry stream: the SAME endpoint with kind=tlm polls the board's tlm buffer instead
        status, payload = await _http(LOG_WEB_PORT, 'POST', '/api/log',
                                      json.dumps({'board': 'glider9', 'kind': 'tlm', 'interval_ms': 40}))
        assert status == 200 and json.loads(payload)['kind'] == 'tlm', payload
        for _ in range(50):
            if any('@s_t.csv@' in line for line in seen):
                break
            await asyncio.sleep(0.02)
        assert any('@s_t.csv@' in line for line in seen), seen[-5:]  # tlm samples flowed to the feed

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


def _gps_device_resolve():
    """main._resolve_gps_device: explicit/off deterministic; 'auto' picks a /dev/ttyUSB* or None."""
    import main as control_main

    assert control_main._resolve_gps_device('off') is None
    assert control_main._resolve_gps_device('') is None
    assert control_main._resolve_gps_device('/dev/ttyUSB7') == '/dev/ttyUSB7'
    auto = control_main._resolve_gps_device('auto')  # host-dependent: a ttyUSB path or None
    assert auto is None or auto.startswith('/dev/ttyUSB'), auto


def _log_drops_are_reported():
    """
    A /logs listener that falls behind is TOLD how many lines it missed, once it has room again.

    A full subscriber queue used to drop lines silently, so the browser showed a log with a hole in
    it and no sign of one. Negative: a listener that kept up sees exactly the lines, no notice.
    """
    hub = _isolated_hub(log=lambda message: None)
    slow, fast = asyncio.Queue(maxsize=2), asyncio.Queue(maxsize=100)
    hub.log_subscribers.update((slow, fast))
    for number in range(5):
        hub._emit_log('taster', 'line %d' % number)
    assert [slow.get_nowait()['line'] for _ in range(2)] == ['line 0', 'line 1']
    hub._emit_log('taster', 'line 5')  # room again -> the count, then the line
    notice, line = slow.get_nowait()['line'], slow.get_nowait()['line']
    assert notice == '[3 log line(s) DROPPED: this view fell behind]' and line == 'line 5', (notice, line)
    assert [fast.get_nowait()['line'] for _ in range(6)] == ['line %d' % n for n in range(6)]
    assert fast.empty(), 'a listener that kept up gets no DROPPED notice'


async def _handler_crash():
    """A registered command handler that raises must return an error reply, NOT drop the operator
    session -- server.py _dispatch wraps the handler call, logs the crash, and replies with an err line."""
    import types
    hub = _isolated_hub(log=lambda message: None)

    def boom(_hub, _tokens, _session):
        raise RuntimeError('boom')

    hub.commands['kaboom'] = types.SimpleNamespace(handler=boom)
    reply = await hub._dispatch('kaboom', {'selected': None})
    assert any('crashed' in line for line in reply), reply


def _glider_roster():
    """
    The roster answers a question live state cannot: is this glider ABSENT, or was it never here?

    The board dials the hub, not the reverse, and it does so only at BOOT -- so a known glider that is
    not connected will not turn up however long the operator waits; it has to be power-cycled. Without
    a persisted roster an absent glider is just a row that is not there, indistinguishable from one
    never set up, and a hub restart forgets every glider it ever saw.
    """
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, 'gliders.json')
        hub = server.Server(roster_path=path)
        hub.log = lambda *args: None

        hub._roster_seen('taster', '192.168.102.152:60384')
        assert hub.roster['taster']['ip'] == '192.168.102.152'      # port stripped: the host is the identity
        assert [g['id'] for g in hub.absent()] == ['taster']        # known, no live link -> absent
        assert 'reboot taster' in hub.absent()[0]['hint']           # ...and says what to do about it

        # SAME NAME FROM A NEW IP replaces the old entry -- a glider that moved network, or a rebuilt board
        # reusing the id. Two records for one glider would make the reboot hint ambiguous.
        hub._roster_seen('taster', '10.0.0.9:5000')
        assert hub.roster['taster']['ip'] == '10.0.0.9' and len(hub.roster) == 1

        class _Live:
            online = True

        hub.boards['taster'] = _Live()      # a LIVE board drops out: the hint is only for gliders needing action
        assert hub.absent() == []

        # it SURVIVES a hub restart, which is the whole point of persisting it
        assert server.Server(roster_path=path).roster['taster']['ip'] == '10.0.0.9'

        # a corrupt roster must not stop the hub starting -- a lost roster is a nuisance, a dead hub is not
        with open(path, 'w') as handle:
            handle.write('{ this is not json')
        assert server.Server(roster_path=path).roster == {}



async def _large_reply():
    """
    A board reply LARGER than asyncio's default 64 KiB stream limit must survive.

    Every reply here is one line read with readline(), and `tlm`/`log` batches are a single base64
    JSON token by design. The board's tee ring holds 1024 cells x 256 B = 256 KiB of records, which
    base64 inflates to ~350 KiB -- so a full window overruns the default limit, readline raises
    ValueError, the reply is discarded and the stream task dies. The operator loses exactly the data
    they asked for, because they asked for a lot of it. This drives a reply past the old ceiling.
    """
    hub = _isolated_hub(host='127.0.0.1', port=BIG_BOARD_PORT, operator_port=BIG_OPERATOR_PORT,
                        web_port=BIG_WEB_PORT, log=lambda message: None, heartbeat_s=5.0)
    hub_task = asyncio.create_task(hub.run())
    await asyncio.sleep(0.1)

    payload = 'x' * 200_000          # 200 KB: over the 64 KiB default, under the new 2 MiB limit
    reader, writer = await asyncio.open_connection('127.0.0.1', BIG_BOARD_PORT)

    async def board():
        """Answers the hub's whoami/health normally, and `ping` with a huge SINGLE line."""
        while True:
            raw = await reader.readline()
            if not raw:
                return
            msg = cc.parse(raw.decode().strip())
            if msg.command == 'whoami':
                info = {'mcu': 'esp32p4', 'firmware_version': 'big', 'stage': 'setting'}
                reply = cc.build('iam', ['glider-big', json.dumps(info)])
            elif msg.command == 'ping':
                reply = cc.build('pong', [payload])     # the oversized one
            else:
                reply = cc.build('ok', [json.dumps({'stage': 'setting', 'mem_free': 1000})])
            writer.write((reply + '\n').encode())
            await writer.drain()

    board_task = asyncio.create_task(board())
    for _ in range(80):
        if 'glider-big' in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'glider-big' in hub.boards, 'the board never registered'

    reply = await hub.boards['glider-big'].exchange('ping', timeout=3)
    assert reply is not None, 'a 200 KB reply was dropped -- the stream limit is too low'
    assert reply.args and len(reply.args[0]) == len(payload), \
        len(reply.args[0]) if reply.args else 'no args'

    board_task.cancel()
    hub_task.cancel()
    writer.close()


def _set(_payload: dict) -> str:
    """The board's answer to a time set it applied: `epoch` changed."""
    return cc.build('ok', [json.dumps({'changed': ['epoch']})])


class _CcClock:
    """Stand-in for server.py's `time` module whose wall clock reads `epoch`; the rest is the real one."""

    def __init__(self, epoch: int):
        self._epoch = epoch

    def time(self) -> float:
        """CC's wall clock, frozen at `epoch`."""
        return float(self._epoch)

    def __getattr__(self, name: str) -> object:
        """Everything else (gmtime, localtime, strftime, monotonic) from the real time module."""
        return getattr(time, name)


async def _clock_case(info: dict, answer: Callable = _set, host_gps: gps.Gps = None,
                      cc_epoch: int = None) -> tuple:
    """
    Register one board announcing `info` in its whoami and record what the hub sends it.

    The board answers `update` with `answer(payload)` (None = never answers, so the exchange times out)
    and anything else with a plain ok. Returns once the board reached `on_board` -- which runs after the
    clock sync -- or the link was lost, or the handler failed, or 2 s passed. The hub listens on a port
    the OS picks.

    Args:
        info - the whoami JSON.
        answer - builds the reply line to `update` from its decoded payload.
        host_gps - an optional host gps.Gps, CC's own fix.
        cc_epoch - CC's wall clock for this case, Unix seconds; None keeps the real one.

    Returns:
        (updates, log, kept, hub): the (object, payload) of each `update` received, the hub log lines,
        whether the handler got past the clock sync (`on_board` ran) with the board still registered
        and online -- so a case that never reached the sync cannot pass as one that sent nothing --
        and the hub.
    """
    updates, seen = [], []
    reached = asyncio.Event()

    async def on_board(_client: board.Board) -> None:
        reached.set()

    with tempfile.TemporaryDirectory() as directory:  # the hub's gliders.json, gone with the case
        hub = server.Server(host='127.0.0.1', on_board=on_board, log=seen.append, heartbeat_s=5.0, gps=host_gps,
                            roster_path=os.path.join(directory, 'gliders.json'))
        listener = await asyncio.start_server(hub._handle, '127.0.0.1', 0)
        reader, writer = await asyncio.open_connection('127.0.0.1', listener.sockets[0].getsockname()[1])

        async def fake() -> None:
            while True:
                raw = await reader.readline()
                if not raw:
                    return
                msg = cc.parse(raw.decode().strip())
                if msg.command == 'whoami':
                    reply = cc.build('iam', ['clock9', json.dumps(info)])
                elif msg.command == 'update':
                    payload = json.loads(msg.args[1])
                    updates.append((msg.args[0], payload))
                    reply = answer(payload)
                    if reply is None:
                        continue  # silence: the hub's exchange times out
                else:
                    reply = cc.build('ok', [json.dumps({'stage': info.get('stage')})])
                writer.write((reply + '\n').encode())
                await writer.drain()

        if cc_epoch is not None:
            server.time = _CcClock(cc_epoch)
        board_task = asyncio.create_task(fake())
        try:
            for _ in range(100):
                if reached.is_set() or any('link lost' in line or line.startswith('error ') for line in seen):
                    break
                await asyncio.sleep(0.02)
            kept = reached.is_set() and 'clock9' in hub.boards and hub.boards['clock9'].online
        finally:
            server.time = time
            board_task.cancel()
            writer.close()
            listener.close()
    return updates, seen, kept, hub


async def _clock_sync() -> None:
    """
    CC sets an UNSET board clock on connect, in SETTING only, and never costs the board the link.

    A cold board's RTC reads 2000-01-01, which named every 2026-10-03 session that date. The sync is one
    `update mission` with CC's UTC epoch, local offset, GPS fix and source; the board appends it to
    session.csv. CC's zone is pinned to +05:30 so a sign slip in the offset, or a local-time epoch,
    cannot pass on a UTC host. Negative: DONE and flight stages (a 26-year jump voids the warm start),
    a set clock, no epoch (older firmware), a bool / float / string / negative / huge epoch, and CC's own
    clock before 2020 get no command and keep the board; a refused, garbled or unapplied reply (a
    non-ok naming epoch, an ok with no `changed`) keeps the board registered; a silent board is dropped
    like any timed-out link.
    """
    host_gps = gps.Gps(log=lambda message: None)
    host_gps.feed(_nmea('GPGSA,A,3,01,02,03,04,05,06,,,,,,,2.0,1.0,1.5'))
    host_gps.feed(_nmea('GPGGA,123519,4807.038,N,01131.000,E,1,06,0.9,545.4,M,46.9,M,,'))
    saved_zone = os.environ.get('TZ')
    os.environ['TZ'] = CC_ZONE
    time.tzset()
    try:
        await _clock_sync_cases(host_gps)
    finally:
        if saved_zone is None:
            os.environ.pop('TZ', None)
        else:
            os.environ['TZ'] = saved_zone
        time.tzset()


async def _clock_sync_cases(host_gps: gps.Gps) -> None:
    """
    The cases of _clock_sync, run under the pinned CC zone.

    Args:
        host_gps - CC's host GPS holding a usable fix.

    Returns:
        None; asserts each case.
    """
    # an unset clock in SETTING: exactly one update mission, CC's UTC epoch + offset east + GPS fix
    updates, seen, kept, _hub = await _clock_case({'stage': 'setting', 'epoch': COLD_EPOCH}, host_gps=host_gps)
    assert len(updates) == 1 and updates[0][0] == 'mission', updates
    payload = updates[0][1]
    assert sorted(payload) == ['cc_position', 'epoch', 'source', 'utc_offset'], payload
    assert type(payload['epoch']) is int and abs(payload['epoch'] - time.time()) < 5, payload  # UTC, not local
    assert payload['utc_offset'] == CC_ZONE_OFFSET and payload['source'] == 'cc-auto', payload
    position = host_gps.position()
    assert payload['cc_position'] == [position['latitude'], position['longitude']], payload
    assert any(line.startswith('clock9 clock set 2000-01-01T00:00:12Z -> 20') for line in seen), seen
    assert kept, seen

    # the first second of 2000 is unset too; no host GPS -> cc_position is null, never launch lat/lon
    updates, seen, kept, _hub = await _clock_case({'stage': 'setting', 'epoch': 946684800})
    assert len(updates) == 1 and updates[0][1]['cc_position'] is None, updates
    assert 'latitude' not in updates[0][1] and 'longitude' not in updates[0][1], updates
    assert kept, seen

    # CC's own clock: 2020-01-01 exactly is valid and sent as is; before 2020 it is unset, logged once
    updates, seen, kept, _hub = await _clock_case({'stage': 'setting', 'epoch': COLD_EPOCH},
                                                  cc_epoch=1577836800)
    assert [payload['epoch'] for _object, payload in updates] == [1577836800], updates
    assert any(line == 'clock9 clock set 2000-01-01T00:00:12Z -> 2020-01-01T00:00:00Z' for line in seen), seen
    assert kept, seen
    updates, seen, kept, _hub = await _clock_case({'stage': 'setting', 'epoch': COLD_EPOCH},
                                                  cc_epoch=1577836799)
    assert updates == [] and kept, (updates, seen)
    notes = [line for line in seen if line.startswith('clock9 clock')]
    assert notes == ['clock9 clock unset (2000-01-01T00:00:12Z) -- NOT synced: the CC clock reads '
                     '2019-12-31T23:59:59Z, before 2020'], notes

    # DONE, flight and an unknown stage: reported, not set -- DONE's 26-year jump would void the warm start
    for stage in ('done', 'boosting', None):
        info = {'epoch': COLD_EPOCH} if stage is None else {'stage': stage, 'epoch': COLD_EPOCH}
        updates, seen, kept, _hub = await _clock_case(info)
        assert updates == [] and kept, (stage, updates, seen)
        notes = [line for line in seen if line.startswith('clock9 clock')]
        assert notes == ['clock9 clock unset (2000-01-01T00:00:12Z) in stage %s -- set only in setting' % stage], notes

    # a set clock (2001-01-01 on), before 2000, no epoch (older firmware), bool, float, string, and epochs
    # no RTC reads (negative, huge): no command, nothing logged, no handler error, the board kept
    for info in ({'stage': 'setting', 'epoch': 1791000000},
                 {'stage': 'setting', 'epoch': 978307200},
                 {'stage': 'setting', 'epoch': 946684799},
                 {'stage': 'setting'},
                 {'stage': 'setting', 'epoch': True},
                 {'stage': 'setting', 'epoch': False},
                 {'stage': 'setting', 'epoch': 946684812.5},
                 {'stage': 'setting', 'epoch': '946684812'},
                 {'stage': 'setting', 'epoch': -1},
                 {'stage': 'setting', 'epoch': -10 ** 20},
                 {'stage': 'setting', 'epoch': 10 ** 20}):
        updates, seen, kept, _hub = await _clock_case(info)
        assert updates == [] and kept, (info, updates, seen)
        assert not any('clock9 clock' in line or line.startswith('error ') for line in seen), (info, seen)

    # refused (no mission object), a non-ok naming epoch, garbled (unparseable -> None), malformed, no
    # `changed`, unapplied (no RTC) and a `changed` that is not a list: logged, no handler error, board kept
    for answer in (lambda _payload: cc.build('err', ['badargs', 'no', 'object', 'mission']),
                   lambda _payload: cc.build('err', [json.dumps({'changed': ['epoch']})]),
                   lambda _payload: 'ok base64:a',
                   lambda _payload: 'ok',
                   lambda _payload: cc.build('ok', [json.dumps({})]),
                   lambda _payload: cc.build('ok', [json.dumps({'changed': []})]),
                   lambda _payload: cc.build('ok', [json.dumps({'changed': ['latitude']})]),
                   lambda _payload: cc.build('ok', [json.dumps({'changed': 'epoch'})]),
                   lambda _payload: cc.build('ok', [json.dumps({'changed': {'epoch': 1}})]),
                   lambda _payload: cc.build('ok', [json.dumps(['epoch'])])):
        updates, seen, kept, _hub = await _clock_case({'stage': 'setting', 'epoch': COLD_EPOCH}, answer=answer)
        assert len(updates) == 1 and kept, (updates, seen)
        assert any(line.startswith('clock9 clock sync') for line in seen), seen
        assert not any('clock set' in line or line.startswith('error ') for line in seen), seen

    # a board that never answers the set times out, and the hub drops it like any lost link
    saved = board.EXCHANGE_TIMEOUT_S
    board.EXCHANGE_TIMEOUT_S = 0.3
    try:
        updates, seen, kept, hub = await _clock_case({'stage': 'setting', 'epoch': COLD_EPOCH},
                                                     answer=lambda _payload: None)
    finally:
        board.EXCHANGE_TIMEOUT_S = saved
    assert len(updates) == 1 and any('clock9 link lost' in line for line in seen), seen
    for _ in range(50):  # the handler's finally runs once the timeout unwinds
        if 'clock9' not in hub.boards:
            break
        await asyncio.sleep(0.02)
    assert 'clock9' not in hub.boards, seen


async def _heartbeat():
    """
    The heartbeat polls HEALTH on health age, not on link traffic, and stops on a link given up.

    It skipped the poll whenever ANY reply was recent, and a running log stream replies every second,
    so during a stream health was never polled and armed/stage/degraded on the dashboard froze. And
    after an exchange timeout marked the link down, it kept polling a dead socket instead of ending
    so the board could re-dial.
    """
    class _Client:
        def __init__(self, online):
            self.id = 'hb'
            self.online = online
            self.last_seen = time.monotonic()  # a stream is keeping the link busy
            self.health_seen = 0.0             # ...but no health reply has arrived
            self.sent = []

        async def command(self, verb, *args, quiet=False, timeout=None):
            self.sent.append(verb)
            self.last_seen = time.monotonic()
            if verb == 'health':
                self.health_seen = time.monotonic()
            return object()

    hub = _isolated_hub(host='127.0.0.1', port=0, operator_port=0, web_port=0,
                        log=lambda message: None, heartbeat_s=0.05)
    busy = _Client(online=True)
    poller = asyncio.create_task(hub._poll(busy))
    await asyncio.sleep(0.4)
    poller.cancel()
    assert busy.sent.count('health') >= 2, 'a busy link suppressed the health poll: %r' % busy.sent

    dead = _Client(online=False)
    await asyncio.wait_for(hub._poll(dead), 1.0)  # must RETURN, not poll a link that was given up
    assert dead.sent == [], dead.sent


async def main():
    await _heartbeat()
    await _loopback()
    await _operator_console()
    await _web()
    await _gps_assist()
    await _log_stream()
    await _handler_crash()
    await _large_reply()
    await _clock_sync()
    _gps_device_resolve()
    _glider_roster()
    _log_drops_are_reported()
    print('ok: server heartbeat polls health on health age + stops on a dead link '
          '+ accept (loopback) + operator console + web bridge (api/boards, api/cmd, events) '
          '+ glider roster (persist, same-name-new-ip, absent hint) '
          '+ gps assist/compare + log streaming + gps auto-detect + oversized reply '
          '+ clock sync on connect (unset + SETTING + CC clock valid; refused/garbled keep the board, silent drops it)')


asyncio.run(main())
