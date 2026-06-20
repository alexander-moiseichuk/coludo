# server.py — the Control hub: a board listener (1234) + per-board heartbeat + a telnet operator
# console (1235), plus the web bridge (web.py, 8080). Boards dial in, Control learns each id via
# whoami/iam and owns every exchange. An operator line whose first token is a board id or `all`
# routes to that board (id stripped, the rest forwarded verbatim) and the reply is tagged
# `from <board> ...`; any other first token is a Control command from the drop-in commands/ registry.
# CPython 3.12, stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

import asyncio
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

    def board_rows(self) -> list:
        """The registry as json-able rows (id, online, last-known stage/config_id) — shared by the
        `list` operator command and the web /api/boards + /events feeds."""
        return [
            {'id': client.id, 'online': client.online, 'stage': client.info.get('stage'),
             'config_id': client.info.get('config_id')}
            for client in self.boards.values()
        ]

    # ----------------------------------------------------------------- board side
    async def _handle(self, reader, writer) -> None:
        """Identify a freshly connected board, register it, then poll it until it drops."""
        client = board.Board(reader, writer, log=self.log)  # log every CC<->board line exchanged
        self.log('control :: board connected %s' % client.peer)
        try:
            board_id = await client.identify()
            if not board_id:
                self.log('control :: whoami failed from %s' % client.peer)
                return
            self.boards[board_id] = client
            self.log('control :: %s online %s' % (board_id, client.info))
            if self.on_board is not None:
                await self.on_board(client)
            await self._poll(client)
        except (asyncio.TimeoutError, ConnectionError, asyncio.IncompleteReadError) as error:
            self.log('control :: %s link lost %r' % (client.id or client.peer, error))
        except Exception as error:
            self.log('control :: error %r' % error)
        finally:
            client.online = False
            client.close()
            self.log('control :: %s offline' % (client.id or client.peer))

    async def _poll(self, client) -> None:
        """Heartbeat: ping an idle board every `heartbeat_s`; a successful exchange (operator or
        ping) within the window already proves liveness, so it is skipped. Returns on disconnect."""
        while True:
            await asyncio.sleep(self.heartbeat_s)
            if time.monotonic() - client.last_seen < self.heartbeat_s:
                continue  # a recent exchange already proved liveness
            if await client.command('ping') is None:
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
        """Forward `command_tokens` (verbatim — already board-facing) to one board or every online
        board, and tag each reply with its source."""
        if not command_tokens:
            return ['from cc err badargs empty-command']
        line = ' '.join(command_tokens)
        if target == BROADCAST:
            targets = [c for c in self.boards.values() if c.online]
            if not targets:
                return ['from cc err noboard all']
        else:
            client = self.boards.get(target)
            if client is None or not client.online:
                return ['from cc err noboard %s' % target]
            targets = [client]
        out = []
        for client in targets:
            resp = await client.exchange(line)
            out.append('from %s %s' % (client.id, _render(resp)) if resp else 'from %s err offline' % client.id)
        return out

    # ------------------------------------------------------------------ listeners
    async def serve_forever(self) -> None:
        """Accept board connections on `port` (board-facing listener)."""
        server = await asyncio.start_server(self._handle, self.host, self.port)
        self.log('control :: boards on %s:%d' % (self.host, self.port))
        async with server:
            await server.serve_forever()

    async def serve_operators(self) -> None:
        """Accept operator connections on `operator_port` (telnet-friendly console)."""
        server = await asyncio.start_server(self._operator, self.host, self.operator_port)
        self.log('control :: operators on %s:%d' % (self.host, self.operator_port))
        async with server:
            await server.serve_forever()

    async def run(self) -> None:
        """Run the board listener, operator console, and web bridge until cancelled."""
        bridge = web.Web(self, self.host, self.web_port, self.log)
        await asyncio.gather(self.serve_forever(), self.serve_operators(), bridge.serve())
