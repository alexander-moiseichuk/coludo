# server.py — the Control hub: a board listener (1234) + per-board heartbeat + a telnet operator
# console (1235), plus the web bridge (web.py, 8080). Boards dial in, Control learns each id via
# whoami/iam and owns every exchange. An operator line whose first token is a board id or `all`
# routes to that board (id stripped, the rest forwarded verbatim) and the reply is tagged
# `from <board> ...`; any other first token is a Control command from the drop-in commands/ registry.
# CPython 3.12, stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

import asyncio
import json
import time

import board
import commands
import web

HEARTBEAT_S: float = 2.0  # poll an idle board this often to prove it is alive
BROADCAST: str = 'all'  # the one broadcast target -- a clean token for scripting (no '*')


def _render(resp) -> str:
    """Render a board reply (_Msg) as a human-readable `status [args...]` line for the console.
    Args are already base64-decoded by cc.parse, so structured payloads show as plain JSON."""
    if not resp.args:
        return resp.command
    return '%s %s' % (resp.command, ' '.join(str(a) for a in resp.args))


class Server:
    """The hub: a board listener + per-board heartbeat + an operator console. `on_board` is an
    optional async hook invoked once, right after a board identifies (used by integration tests)."""

    def __init__(self, host: str = '0.0.0.0', port: int = 1234, operator_port: int = 1235,
                 web_port: int = 8080, on_board=None, log=print, heartbeat_s: float = HEARTBEAT_S,
                 gps=None):
        self.host = host
        self.port = port
        self.operator_port = operator_port
        self.web_port = web_port
        self.boards = {}  # id -> board.Board (kept after disconnect with online=False)
        self.on_board = on_board
        self.log = log
        self.heartbeat_s = heartbeat_s
        self.gps = gps  # optional host GPS (gps.Gps) for launch-position assist; None if unattached
        self.commands = commands.load()  # operator command registry, loaded from commands/ at start
        self.streams = {}  # board id -> the log-streaming Task while `log <board>` is active
        self.log_subscribers = set()  # asyncio.Queue per /logs SSE listener (streamed log lines)

    def board_rows(self) -> list:
        """The registry as json-able rows — shared by the `list` operator command and the web
        /api/boards + /events feeds. Carries the last-known stage/config_id plus the heartbeat vitals
        (uptime / clock / temp / mem_free) cached from `health`, so the dashboard top table is live."""
        rows = []
        for client in self.boards.values():
            health = client.cache.get('health') or {}
            rows.append({
                'id': client.id, 'online': client.online,
                'stage': health.get('stage') or client.info.get('stage'),  # health is fresher than the handshake
                'version': client.info.get('firmware_version'),
                'config_id': client.info.get('config_id'),
                'uptime': health.get('uptime'), 'clock': health.get('clock'),
                'position': health.get('position'),  # board GNSS fix (lat, lon) or None
                'temp': health.get('temp'), 'mem_free': health.get('mem_free'),
            })
        return rows

    def cc_status(self) -> dict:
        """The Control host's own status for the dashboard header: the wall clock and the host GPS
        (None if no GPS device is attached; otherwise gps.status() -- usable / fix_3d / lat / lon)."""
        return {'time': time.strftime('%Y-%m-%dT%H:%M:%S'),
                'gps': self.gps.status() if self.gps is not None else None}

    # ----------------------------------------------------------------- board side
    async def _handle(self, reader, writer) -> None:
        """Identify a freshly connected board, register it, then poll it until it drops."""
        client = board.Board(reader, writer, log=self.log)  # log every CC<->board line exchanged
        self.log('board connected %s' % client.peer)
        try:
            board_id = await client.identify()
            if not board_id:
                self.log('whoami failed from %s' % client.peer)
                return
            self.boards[board_id] = client
            self.log('%s online %s' % (board_id, client.info))
            if self.on_board is not None:
                await self.on_board(client)
            await self._poll(client)
        except (asyncio.TimeoutError, ConnectionError, asyncio.IncompleteReadError) as error:
            self.log('%s link lost %r' % (client.id or client.peer, error))
        except Exception as error:
            self.log('error %r' % error)
        finally:
            self._drop_stream(client.id)  # stop any log stream for this board
            client.online = False
            client.close()
            self.log('%s offline' % (client.id or client.peer))

    # --------------------------------------------------------------- log streaming
    def _emit_log(self, board_id, line) -> None:
        """Surface one streamed board log line: to the console as `<id>: <line>` and to every /logs
        SSE subscriber (a full subscriber queue drops the line, never blocks the poll)."""
        self.log('%s: %s' % (board_id, line))
        for queue in list(self.log_subscribers):
            try:
                queue.put_nowait({'board': board_id, 'line': line})
            except asyncio.QueueFull:
                pass

    async def _stream(self, client, interval_ms) -> None:
        """Poll a board's `log` buffer every `interval_ms` and emit each returned line. The board
        window is 2x the interval so its deadline never lapses between polls; if this task stops, the
        window lapses and the board self-disables (no consumer -> no collection). Ends on disconnect."""
        window_ms = max(1, interval_ms) * 2
        while True:
            resp = await client.command('log', window_ms)
            if resp is None:
                return  # board dropped -> _handle marks it offline and drops the stream
            if resp.command == 'ok' and resp.args:
                try:
                    lines = json.loads(resp.args[0]).get('lines', [])
                except ValueError:
                    lines = []
                for line in lines:
                    self._emit_log(client.id, line)
            await asyncio.sleep(interval_ms / 1000.0)

    def start_stream(self, client, interval_ms) -> None:
        """(Re)start streaming a board's logs at `interval_ms`, replacing any running stream for it."""
        self._drop_stream(client.id)
        self.streams[client.id] = asyncio.create_task(self._stream(client, interval_ms))

    def _drop_stream(self, board_id):
        """Cancel and forget any streaming task for a board; return its Board (or None)."""
        task = self.streams.pop(board_id, None)
        if task is not None:
            task.cancel()
        return self.boards.get(board_id)

    async def stop_stream(self, board_id) -> None:
        """Stop streaming a board's logs and tell the board to stop collecting (a final `log 0`
        drain), so it does not keep teeing once nobody is polling."""
        client = self._drop_stream(board_id)
        if client is not None and client.online:
            await client.command('log', 0)

    async def _stream_toggle(self, client, args) -> str:
        """`<board> log [ms|off]`: start/refresh (default 1000 ms) or stop the hub's log stream for one
        board. Returns a source-tagged reply line, like any board-first command."""
        arg = args[0] if args else '1000'
        if arg in ('off', '0'):
            await self.stop_stream(client.id)
            return 'from %s ok %s' % (client.id, json.dumps({'log': 'off'}))
        try:
            interval_ms = int(arg)
        except ValueError:
            return 'from %s err badargs log [ms|off]' % client.id
        self.start_stream(client, interval_ms)
        return 'from %s ok %s' % (client.id, json.dumps({'log': 'on', 'interval_ms': interval_ms}))

    async def _poll(self, client) -> None:
        """Heartbeat: poll an idle board's `health` every `heartbeat_s` -- it proves liveness AND
        refreshes the vitals (uptime / clock / temp / mem) the dashboard top table shows, cached
        Control-side. A recent exchange within the window already proves liveness, so it is skipped.
        The poll is `quiet` (no per-beat tx/rx spam) -- only a CHANGE in liveness is logged (the first
        'ok', the first 'lost'). Returns on disconnect."""
        alive = None  # last heartbeat outcome (None until the first poll) -> log only on transition
        while True:
            await asyncio.sleep(self.heartbeat_s)
            if time.monotonic() - client.last_seen < self.heartbeat_s:
                continue  # a recent exchange already proved liveness
            ok = await client.command('health', quiet=True) is not None
            if ok != alive:
                self.log('%s heartbeat %s' % (client.id, 'ok' if ok else 'lost'))
                alive = ok
            if not ok:
                return  # disconnected -> _handle marks it offline

    # -------------------------------------------------------------- operator side
    async def _operator(self, reader, writer) -> None:
        """One telnet/dev operator session: read lines, dispatch, write tagged replies."""
        session = {'selected': None}
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    return
                text = raw.decode().strip()
                if not text:
                    continue
                for line in await self._dispatch(text, session):
                    writer.write((line + '\n').encode())
                await writer.drain()
        except ConnectionError:
            pass
        finally:
            writer.close()

    async def _dispatch(self, text, session) -> list:
        """Route one operator line. A known board id / `all` first token routes to a board (id
        stripped); a registered Control command (from commands/) handles its own; otherwise a sticky
        `select` target (if any) takes the whole line."""
        tokens = text.split()
        first = tokens[0]
        if first in self.boards or first == BROADCAST:
            return await self._route(first, tokens[1:])
        spec = self.commands.get(first)
        if spec is not None:
            result = spec.handler(self, tokens, session)
            return await result if asyncio.iscoroutine(result) else result
        if session['selected']:
            return await self._route(session['selected'], tokens)
        return ['from cc err badcmd %s' % first]

    async def _route(self, target, command_tokens) -> list:
        """Run `command_tokens` against one board or every online board, tagging each reply with its
        source. `log [ms|off]` is intercepted as hub-orchestrated streaming (start/stop a poll task);
        every other command is forwarded verbatim (already board-facing) as a one-shot exchange."""
        if not command_tokens:
            return ['from cc err badargs empty-command']
        if target == BROADCAST:
            targets = [c for c in self.boards.values() if c.online]
            if not targets:
                return ['from cc err noboard all']
        else:
            client = self.boards.get(target)
            if client is None or not client.online:
                return ['from cc err noboard %s' % target]
            targets = [client]
        if command_tokens[0] == 'log':  # hub streaming toggle, not a one-shot forward
            return [await self._stream_toggle(client, command_tokens[1:]) for client in targets]
        line = ' '.join(command_tokens)
        out = []
        for client in targets:
            resp = await client.exchange(line)
            out.append('from %s %s' % (client.id, _render(resp)) if resp else 'from %s err offline' % client.id)
        return out

    # ------------------------------------------------------------------ listeners
    async def serve_forever(self) -> None:
        """Accept board connections on `port` (board-facing listener)."""
        server = await asyncio.start_server(self._handle, self.host, self.port)
        self.log('boards on %s:%d' % (self.host, self.port))
        async with server:
            await server.serve_forever()

    async def serve_operators(self) -> None:
        """Accept operator connections on `operator_port` (telnet-friendly console)."""
        server = await asyncio.start_server(self._operator, self.host, self.operator_port)
        self.log('operators on %s:%d' % (self.host, self.operator_port))
        async with server:
            await server.serve_forever()

    async def run(self) -> None:
        """Run the board listener, operator console, and web bridge until cancelled."""
        bridge = web.Web(self, self.host, self.web_port, self.log)
        await asyncio.gather(self.serve_forever(), self.serve_operators(), bridge.serve())
