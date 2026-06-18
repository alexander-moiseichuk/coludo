# Control — host-side ground station / hub for the Coludo boards (specs/cc-protocol.md).
#
# Board-first: boards dial in (port 1234), Control learns each board's id via whoami/iam and then
# owns every exchange over the board socket (which sees `command params`, no id; only `iam` carries
# the id). Control polls each online board (~2 s heartbeat) to prove liveness, and exposes a
# telnet-friendly operator console (port 1235): a line whose first token is a board id / `all` / `*`
# is routed to that board (the id stripped, the rest forwarded verbatim) and the reply tagged
# `from <board> ...`; any other first token is a Control command (`help`/`list`/`select`/`who`),
# served from a drop-in registry loaded from the `commands/` package at start.
#
# CPython 3.12, stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

import asyncio
import json
import time

import cc_protocol as cc
import commands

HEARTBEAT_S: float = 2.0  # poll an idle board this often to prove it is alive


def _render(resp) -> str:
    """Render a board reply (_Msg) as a human-readable `status [args...]` line for the console.
    Args are already base64-decoded by cc.parse, so structured payloads show as plain JSON."""
    if not resp.args:
        return resp.command
    return '%s %s' % (resp.command, ' '.join(str(a) for a in resp.args))


class Board:
    """One connected board: lockstep request/response over its socket. The per-board lock makes
    every exchange strictly sequential (Control never injects a second command mid-exchange), so
    the heartbeat and operator traffic to one board can never overlap."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self._reader = reader
        self._writer = writer
        self._lock = asyncio.Lock()
        self.id = None
        self.info = {}
        self.online: bool = True
        self.last_seen: float = time.monotonic()

    @property
    def peer(self) -> str:
        host, port = self._writer.get_extra_info('peername')[:2]
        return '%s:%d' % (host, port)

    async def exchange(self, line: str, timeout: float = 5.0):
        """Send a ready board-facing line and return its parsed reply (None if disconnected).
        `timeout` bounds the wait so a wedged board raises asyncio.TimeoutError, not hangs."""
        async with self._lock:
            self._writer.write((line + '\n').encode())
            await self._writer.drain()
            raw = await asyncio.wait_for(self._reader.readline(), timeout)
        if not raw:
            return None
        self.last_seen = time.monotonic()
        return cc.parse(raw.decode().strip())

    async def command(self, command: str, *args, timeout: float = 5.0):
        """Build `command args...` and exchange it. Returns the parsed reply or None."""
        return await self.exchange(cc.build(command, list(args)), timeout)

    async def identify(self) -> str:
        resp = await self.command('whoami')
        if resp and resp.command == 'iam' and len(resp.args) >= 2:
            self.id = resp.args[0]
            self.info = json.loads(resp.args[1])
        return self.id

    async def inspect(self, name: str) -> dict:
        resp = await self.command('inspect', name)
        return json.loads(resp.args[0]) if resp and resp.command == 'ok' else {}

    def close(self) -> None:
        self._writer.close()


class Server:
    """The hub: a board listener + per-board heartbeat + an operator console. `on_board` is an
    optional async hook invoked once, right after a board identifies (used by integration tests)."""

    def __init__(self, host: str = '0.0.0.0', port: int = 1234, operator_port: int = 1235,
                 on_board=None, log=print, heartbeat_s: float = HEARTBEAT_S):
        self.host = host
        self.port = port
        self.operator_port = operator_port
        self.boards = {}  # id -> Board (kept after disconnect with online=False)
        self.on_board = on_board
        self.log = log
        self.heartbeat_s = heartbeat_s
        self.commands = commands.load()  # operator command registry, loaded from commands/ at start

    # ----------------------------------------------------------------- board side
    async def _handle(self, reader, writer):
        """Identify a freshly connected board, register it, then poll it until it drops."""
        board = Board(reader, writer)
        self.log('control :: board connected %s' % board.peer)
        try:
            board_id = await board.identify()
            if not board_id:
                self.log('control :: whoami failed from %s' % board.peer)
                return
            self.boards[board_id] = board
            self.log('control :: %s online %s' % (board_id, board.info))
            if self.on_board is not None:
                await self.on_board(board)
            await self._poll(board)
        except (asyncio.TimeoutError, ConnectionError, asyncio.IncompleteReadError) as error:
            self.log('control :: %s link lost %r' % (board.id or board.peer, error))
        except Exception as error:
            self.log('control :: error %r' % error)
        finally:
            board.online = False
            board.close()
            self.log('control :: %s offline' % (board.id or board.peer))

    async def _poll(self, board):
        """Heartbeat: ping an idle board every `heartbeat_s`; a successful exchange (operator or
        ping) within the window already proves liveness, so it is skipped. Returns on disconnect."""
        while True:
            await asyncio.sleep(self.heartbeat_s)
            if time.monotonic() - board.last_seen < self.heartbeat_s:
                continue  # a recent exchange already proved liveness
            if await board.command('ping') is None:
                return  # disconnected -> _handle marks it offline

    # -------------------------------------------------------------- operator side
    async def _operator(self, reader, writer):
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
        """Route one operator line. A known board id / `all` / `*` first token routes to a board
        (id stripped); a registered Control command (from commands/) handles its own; otherwise a
        sticky `select` target (if any) takes the whole line."""
        tokens = text.split()
        first = tokens[0]
        if first in self.boards or first in ('all', '*'):
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
        if target in ('all', '*'):
            targets = [b for b in self.boards.values() if b.online]
            if not targets:
                return ['from cc err noboard *']
        else:
            board = self.boards.get(target)
            if board is None or not board.online:
                return ['from cc err noboard %s' % target]
            targets = [board]
        out = []
        for board in targets:
            resp = await board.exchange(line)
            out.append('from %s %s' % (board.id, _render(resp)) if resp else 'from %s err offline' % board.id)
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
        """Run both listeners until cancelled — the hub entry point."""
        await asyncio.gather(self.serve_forever(), self.serve_operators())


if __name__ == '__main__':
    asyncio.run(Server().run())
