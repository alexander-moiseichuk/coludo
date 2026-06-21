# Web bridge — the browser face of the Control hub (specs/cc-protocol.md "Browser bridge").
#
# A minimal HTTP/1.1 + SSE server on 8080 over the same stdlib asyncio loop as the board listener
# and operator console (no extra dependency, no framework). Plain HTTP: the LAN is trusted and
# encryption is out of scope (cc-protocol.md "Transport & ports"). Routes:
#   GET  /             -> the one-page dashboard (static/index.html)
#   GET  /api/boards   -> hub.board_rows() as JSON (same data as the `list` command)
#   POST /api/cmd      -> {board, command, params} -> run it on the board, reply as JSON
#   GET  /events       -> Server-Sent Events: the board list pushed every heartbeat (live table)

import asyncio
import json
import os

_REASON = {200: 'OK', 400: 'Bad Request', 404: 'Not Found', 502: 'Bad Gateway'}
_PAGE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static', 'index.html')


def _load_page() -> str:
    try:
        with open(_PAGE_PATH) as page:
            return page.read()
    except OSError:
        return '<!doctype html><title>Coludo</title><h1>Coludo Control</h1><p>static/index.html missing</p>'


async def _send(writer, status: int, content_type: str, body) -> None:
    if isinstance(body, str):
        body = body.encode()
    head = (
        'HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\n\r\n'
        % (status, _REASON.get(status, 'OK'), content_type, len(body))
    )
    writer.write(head.encode() + body)
    await writer.drain()


async def _send_json(writer, status: int, payload) -> None:
    await _send(writer, status, 'application/json', json.dumps(payload))


class Web:
    """The HTTP/SSE server. Holds the hub for the board registry + routing; one per hub."""

    def __init__(self, hub, host: str = '0.0.0.0', port: int = 8080, log=print):
        self.hub = hub
        self.host = host
        self.port = port
        self.log = log
        self.page = _load_page()

    async def serve(self) -> None:
        server = await asyncio.start_server(self._handle, self.host, self.port)
        self.log('control :: web on %s:%d' % (self.host, self.port))
        async with server:
            await server.serve_forever()

    async def _handle(self, reader, writer) -> None:
        try:
            request_line = await reader.readline()
            if not request_line:
                return
            method, path, _ = request_line.decode().split(' ', 2)
            length = 0
            while True:
                header = await reader.readline()
                if header in (b'\r\n', b'\n', b''):
                    break
                name, _, value = header.decode().partition(':')
                if name.strip().lower() == 'content-length':
                    length = int(value.strip())
            body = await reader.readexactly(length) if length else b''
            await self._route(method, path, body, writer)
        except (ConnectionError, asyncio.IncompleteReadError, asyncio.CancelledError):
            pass
        finally:
            writer.close()

    async def _route(self, method: str, path: str, body: bytes, writer) -> None:
        route = path.split('?', 1)[0]
        if method == 'GET' and route == '/':
            return await _send(writer, 200, 'text/html; charset=utf-8', self.page)
        if method == 'GET' and route == '/api/boards':
            return await _send_json(writer, 200, self.hub.board_rows())
        if method == 'GET' and route.startswith('/api/board/'):
            return await self._api_board(route[len('/api/board/'):], writer)
        if method == 'POST' and route == '/api/cmd':
            return await self._api_cmd(body, writer)
        if method == 'POST' and route == '/api/log':
            return await self._api_log(body, writer)
        if method == 'GET' and route == '/events':
            return await self._events(writer)
        if method == 'GET' and route == '/logs':
            return await self._logs(writer)
        await _send(writer, 404, 'text/plain', 'not found')

    async def _api_board(self, board_id: str, writer) -> None:
        """The Control-side cached properties for one board (config / inspect / stats / health) —
        last-known values, served without touching the board."""
        board = self.hub.boards.get(board_id)
        if board is None:
            return await _send_json(writer, 404, {'error': 'no board %r' % board_id})
        return await _send_json(writer, 200, board.properties())

    async def _api_cmd(self, body: bytes, writer) -> None:
        try:
            request = json.loads(body or b'{}')
        except ValueError:
            return await _send_json(writer, 400, {'error': 'bad json'})
        board = self.hub.boards.get(request.get('board'))
        command = request.get('command')
        if board is None or not board.online:
            return await _send_json(writer, 404, {'error': 'no online board %r' % request.get('board')})
        if not command:
            return await _send_json(writer, 400, {'error': 'no command'})
        resp = await board.command(command, *request.get('params', []))
        if resp is None:
            return await _send_json(writer, 502, {'board': board.id, 'error': 'offline'})
        return await _send_json(writer, 200, {'board': board.id, 'status': resp.command, 'args': resp.args})

    async def _api_log(self, body: bytes, writer) -> None:
        """Start/stop the hub's log stream for a board from the dashboard: body `{board, interval_ms}`
        (interval_ms <= 0 stops). The streamed lines arrive on the /logs SSE feed, same as when an
        operator types `<board> log <ms>`."""
        try:
            request = json.loads(body or b'{}')
        except ValueError:
            return await _send_json(writer, 400, {'error': 'bad json'})
        board = self.hub.boards.get(request.get('board'))
        if board is None or not board.online:
            return await _send_json(writer, 404, {'error': 'no online board %r' % request.get('board')})
        interval_ms = request.get('interval_ms', 1000)
        if not isinstance(interval_ms, int) or interval_ms <= 0:
            await self.hub.stop_stream(board.id)
            return await _send_json(writer, 200, {'board': board.id, 'streaming': False})
        self.hub.start_stream(board, interval_ms)
        return await _send_json(writer, 200, {'board': board.id, 'streaming': True, 'interval_ms': interval_ms})

    async def _events(self, writer) -> None:
        writer.write(
            b'HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n'
            b'Cache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n'
        )
        await writer.drain()
        while True:  # thin view over the hub: push the board list every heartbeat
            writer.write(('data: %s\n\n' % json.dumps(self.hub.board_rows())).encode())
            await writer.drain()
            await asyncio.sleep(self.hub.heartbeat_s)

    async def _logs(self, writer) -> None:
        """Server-Sent Events of streamed board log lines (`{board, line}`), pushed as the hub emits
        them while `log <board>` is active. One queue per connection, dropped from the hub on close."""
        queue = asyncio.Queue(maxsize=1000)
        self.hub.log_subscribers.add(queue)
        try:
            writer.write(
                b'HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n'
                b'Cache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n'
            )
            await writer.drain()
            while True:
                writer.write(('data: %s\n\n' % json.dumps(await queue.get())).encode())
                await writer.drain()
        finally:
            self.hub.log_subscribers.discard(queue)
