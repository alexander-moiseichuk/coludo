# board.py — one connected Coludo board as seen by the hub: lockstep request/response over its
# socket (specs/cc-protocol.md). The per-board lock makes every exchange strictly sequential, so the
# heartbeat and operator traffic to one board can never overlap. CPython 3.12, stdlib asyncio only.

import asyncio
import json
import time

import cc_protocol as cc

EXCHANGE_TIMEOUT_S: float = 10.0  # bound every board exchange so a wedged board raises, never hangs


class Board:
    """One connected board: lockstep request/response over its socket."""

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

    async def exchange(self, line: str, timeout: float = EXCHANGE_TIMEOUT_S) -> cc._Msg:
        """Send a ready board-facing line and return its parsed reply (None if disconnected).
        `timeout` (default 10 s) bounds the wait so a wedged board raises asyncio.TimeoutError."""
        async with self._lock:
            self._writer.write((line + '\n').encode())
            await self._writer.drain()
            raw = await asyncio.wait_for(self._reader.readline(), timeout)
        if not raw:
            return None
        self.last_seen = time.monotonic()
        return cc.parse(raw.decode().strip())

    async def command(self, command: str, *args, timeout: float = EXCHANGE_TIMEOUT_S) -> cc._Msg:
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
