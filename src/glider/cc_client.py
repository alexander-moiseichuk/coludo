# CC client — the board side of the Control Center protocol (specs/cc-protocol.md).
#
# The board is a TCP client that dials out to CC; CC drives every exchange and the board only
# answers (the poll model). On connect, CC sends `whoami`; thereafter it issues board-addressed
# commands and the board replies. Dispatcher turns a parsed line into a response (pure logic,
# unit-testable); Client is the thin networking that reads lines and writes responses.

import asyncio
import json

import cc_protocol as cc


class Dispatcher:
    """Maps a command to an async handler(msg) -> response line. Enforces that board-addressed
    commands match this board (whoami has no board-id)."""

    def __init__(self, board_id):
        self.board_id = board_id
        self.handlers = {}

    def on(self, command, fn):
        self.handlers[command] = fn
        return fn

    async def handle(self, line):
        msg = cc.parse(line)
        if msg.command is None:
            return None
        if msg.board is not None and msg.board != self.board_id:
            return cc.build('err', [self.board_id, 'badboard', msg.board])
        fn = self.handlers.get(msg.command)
        if fn is None:
            return cc.build('err', [self.board_id, 'badcmd', msg.command])
        try:
            return await fn(msg)
        except Exception as e:
            return cc.build('err', [self.board_id, 'internal', repr(e)])


class Client:
    def __init__(self, config, dispatcher, log=None, backoff_ms=1000):
        w = config['wifi']
        self.host = w['cc_host']
        self.port = w['cc_port']
        self.dispatcher = dispatcher
        self.log = log if log is not None else (lambda m: None)
        self.backoff_ms = backoff_ms

    async def run(self, stop=None):
        """Connect to CC and serve until stopped, reconnecting with backoff on drop."""
        while stop is None or not stop[0]:
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                self.log('cc_client :: connected %s:%d' % (self.host, self.port))
                await self.serve(reader, writer)
            except Exception as e:
                self.log('cc_client :: %r' % e)
            await asyncio.sleep_ms(self.backoff_ms)

    async def serve(self, reader, writer):
        """Read commands from CC, dispatch, write responses. Returns on disconnect."""
        while True:
            line = await reader.readline()
            if not line:
                return
            resp = await self.dispatcher.handle(line.decode().strip())
            if resp is not None:
                writer.write((resp + '\n').encode())
                await writer.drain()


def standard_dispatcher(cfg, controller=None, on_reboot=None, fw='0.1', config_path='board.json'):
    """Build a Dispatcher with the standard command handlers, wired to the running config and
    (optionally) the Controller. `on_reboot` lets tests intercept the reset."""
    import gc
    import time

    import config as config_mod

    board_id = cfg['board']['id']
    d = Dispatcher(board_id)

    def _state():
        return controller.state if controller is not None else 'setting'

    async def h_whoami(msg):
        info = {
            'mcu': cfg['board'].get('mcu'),
            'fw': fw,
            'config_id': config_mod.config_id(cfg),
            'state': _state(),
            'uptime': time.ticks_ms(),
        }
        return cc.build('iam', [board_id, json.dumps(info)])

    async def h_ping(msg):
        return cc.build('pong', [board_id])

    async def h_health(msg):
        try:
            import esp32

            temp = esp32.mcu_temperature()
        except Exception:
            temp = None
        h = {'temp': temp, 'mem_free': gc.mem_free(), 'uptime': time.ticks_ms(), 'state': _state()}
        if controller is not None:
            h['tasks'] = [{'name': t.name, 'ok': t.validate()} for t in controller.active()]
        return cc.build('ok', [board_id, json.dumps(h)])

    async def h_state(msg):
        return cc.build('ok', [board_id, json.dumps({'state': _state()})])

    async def h_report(msg):
        r = controller.report() if controller is not None else {}
        return cc.build('ok', [board_id, json.dumps(r)])

    async def h_get_config(msg):
        which = msg.params[0] if msg.params else 'running'
        c = config_mod._builtin_default() if which == 'default' else cfg
        return cc.build('ok', [board_id, json.dumps(c)])

    async def h_save_config(msg):
        if not msg.params:
            return cc.build('err', [board_id, 'badargs', 'no config'])
        try:
            newcfg = json.loads(msg.params[0])
        except Exception:
            return cc.build('err', [board_id, 'badargs', 'bad json'])
        try:
            cid = config_mod.save(newcfg, config_path)
        except ValueError as e:
            return cc.build('err', [board_id, 'invalid', str(e)])
        return cc.build('ok', [board_id, json.dumps({'config_id': cid})])

    async def h_reset_config(msg):
        config_mod.reset(config_path)
        return cc.build('ok', [board_id])

    async def h_reboot(msg):
        reset = on_reboot if on_reboot is not None else _machine_reset

        async def _do():
            await asyncio.sleep_ms(200)
            reset()

        asyncio.create_task(_do())
        return cc.build('ok', [board_id])

    d.on('whoami', h_whoami)
    d.on('ping', h_ping)
    d.on('health', h_health)
    d.on('state', h_state)
    d.on('report', h_report)
    d.on('get-config', h_get_config)
    d.on('save-config', h_save_config)
    d.on('reset-config', h_reset_config)
    d.on('reboot', h_reboot)
    return d


def _machine_reset():
    import machine

    machine.reset()
