# Control — host-side ground station for the Coludo boards (specs/cc-protocol.md). Board-first:
# boards dial in, Control learns each board's id via whoami/iam, and drives commands over the
# board socket (which sees `command params`, no id; only `iam` carries the id). CPython 3.12,
# stdlib asyncio only. cc_protocol.py is shared with the firmware (symlinked).

import asyncio
import json

import cc_protocol as cc


class Board:
    """One connected board: lockstep request/response over its socket."""

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        self._reader = reader
        self._writer = writer
        self._lock = asyncio.Lock()
        self.id = None
        self.info = {}

    @property
    def peer(self) -> str:
        host, port = self._writer.get_extra_info('peername')[:2]
        return '%s:%d' % (host, port)

    async def command(self, command: str, *args):
        """Send `command args...` to the board and return its parsed response (or None on drop)."""
        line = cc.build(command, list(args))
        async with self._lock:
            self._writer.write((line + '\n').encode())
            await self._writer.drain()
            raw = await self._reader.readline()
        if not raw:
            return None
        return cc.parse(raw.decode().strip())

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
    def __init__(self, host: str = '0.0.0.0', port: int = 1234, on_board=None, log=print):
        self.host = host
        self.port = port
        self.boards = {}  # id -> Board
        self.on_board = on_board  # optional async callback(board) once identified
        self.log = log

    async def _handle(self, reader, writer):
        board = Board(reader, writer)
        self.log('control :: board connected %s' % board.peer)
        try:
            board_id = await board.identify()
            if not board_id:
                self.log('control :: whoami failed from %s' % board.peer)
                return
            self.boards[board_id] = board
            self.log('control :: %s identified %s' % (board_id, board.info))
            if self.on_board is not None:
                await self.on_board(board)
        except Exception as error:
            self.log('control :: error %r' % error)
        finally:
            board.close()
            self.boards.pop(board.id, None)
            self.log('control :: %s disconnected' % (board.id or board.peer))

    async def serve_forever(self) -> None:
        server = await asyncio.start_server(self._handle, self.host, self.port)
        self.log('control :: listening on %s:%d' % (self.host, self.port))
        async with server:
            await server.serve_forever()
