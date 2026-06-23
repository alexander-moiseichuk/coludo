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

    def __init__(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter, log=None):
        self._reader = reader
        self._writer = writer
        self._log = log or (lambda message: None)  # CC<->board exchange log (no-op unless wired)
        self._lock = asyncio.Lock()
        self.id = None
        self.info = {}  # the iam handshake: mcu / firmware_version / stage / config_id
        # Control-side cache of board properties, filled as a side effect of real exchanges so the
        # dashboard can show last-known values without re-polling. config <- get-config (running);
        # inspect/stats <- per-object; health <- the vitals reply.
        self.cache = {'config': None, 'inspect': {}, 'stats': {}, 'health': None}
        self.online: bool = True
        self.last_seen: float = time.monotonic()

    @property
    def peer(self) -> str:
        host, port = self._writer.get_extra_info('peername')[:2]
        return '%s:%d' % (host, port)

    async def exchange(self, line: str, timeout: float = EXCHANGE_TIMEOUT_S, quiet: bool = False) -> cc._Msg:
        """Send a ready board-facing line and return its parsed reply (None if disconnected).
        `timeout` (default 10 s) bounds the wait so a wedged board raises asyncio.TimeoutError. `quiet`
        suppresses the tx/rx console log -- used for the heartbeat ping, which the hub summarises itself
        (only its first success / first failure) rather than spamming a line every beat."""
        tag = self.id or self.peer
        async with self._lock:
            if not quiet:
                self._log('%s -> %s' % (tag, line))  # CC sends (tx); logged before the wait
            self._writer.write((line + '\n').encode())
            await self._writer.drain()
            raw = await asyncio.wait_for(self._reader.readline(), timeout)
        reply = raw.decode().strip()
        if not quiet:
            self._log('%s <- %s' % (tag, reply if raw else '<disconnected>'))  # board replies (rx)
        if not raw:
            return None
        self.last_seen = time.monotonic()
        msg = cc.parse(reply)
        self._remember(line, msg)
        return msg

    def _remember(self, line: str, msg: cc._Msg) -> None:
        """Cache a board-state reply (config / inspect / stats / health) keyed by the command sent,
        so the dashboard shows last-known values without re-polling. Only successful `ok` replies
        carrying JSON update the cache; everything else (ping, errors) is ignored."""
        if msg.command != 'ok' or not msg.args:
            return
        tokens = line.split()
        command = tokens[0]
        try:
            payload = json.loads(msg.args[0])  # already base64-decoded by cc.parse
        except ValueError:
            return
        if command == 'get-config' and tokens[1:] in ([], ['board'], ['running']):  # the running board config only
            self.cache['config'] = payload
        elif command == 'inspect' and len(tokens) >= 2:
            self.cache['inspect'][tokens[1]] = payload
        elif command == 'stats' and len(tokens) >= 2:
            self.cache['stats'][tokens[1]] = payload
        elif command == 'health':
            self.cache['health'] = payload

    def properties(self) -> dict:
        """The Control-side snapshot of this board: identity + the cached config/inspect/stats/health
        (json-able). Served by `/api/board/<id>` and the `cache` operator command."""
        return {'id': self.id, 'online': self.online, 'info': self.info,
                'config': self.cache['config'], 'inspect': self.cache['inspect'],
                'stats': self.cache['stats'], 'health': self.cache['health']}

    async def command(self, command: str, *args, timeout: float = EXCHANGE_TIMEOUT_S,
                      quiet: bool = False) -> cc._Msg:
        """Build `command args...` and exchange it. Returns the parsed reply or None."""
        return await self.exchange(cc.build(command, list(args)), timeout, quiet=quiet)

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
