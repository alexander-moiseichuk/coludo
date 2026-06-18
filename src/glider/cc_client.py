# cc_client — board side of the Control protocol (specs/cc-protocol.md). Board-first routing:
# Control strips the routing board id, so the board receives `command params` and replies
# `status params` (no id; only `iam` carries the board id, so Control can learn it on a new
# socket). Dispatcher turns a parsed line into a response (pure logic, unit-testable); Client is
# the thin networking that reads lines and writes responses.

import asyncio
import json

import cc_protocol as cc
from inspector import Inspector


class Dispatcher:
    """Maps a command to an async handler(msg) -> response line."""

    def __init__(self):
        self.handlers = {}

    def on(self, command, fn):
        self.handlers[command] = fn

    async def handle(self, line):
        msg = cc.parse(line)
        if msg.command is None:
            return None
        fn = self.handlers.get(msg.command)
        if fn is None:
            return cc.build('err', ['badcmd', msg.command])
        try:
            return await fn(msg)
        except Exception as error:
            return cc.build('err', ['internal', repr(error)])


class Client:
    def __init__(self, config, dispatcher, log=None, backoff_ms=1000):
        wifi = config['wifi']
        self.host = wifi['cc_host']
        self.port = wifi['cc_port']
        self.dispatcher = dispatcher
        self.log = log if log is not None else (lambda message: None)
        self.backoff_ms = backoff_ms

    async def run(self):
        """Connect to Control and serve forever, reconnecting with backoff on drop."""
        while True:
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                self.log('cc_client :: connected %s:%d' % (self.host, self.port))
                await self.serve(reader, writer)
            except Exception as error:
                self.log('cc_client :: %r' % error)
            await asyncio.sleep_ms(self.backoff_ms)

    async def serve(self, reader, writer):
        """Read commands from Control, dispatch, write responses. Returns on disconnect."""
        while True:
            line = await reader.readline()
            if not line:
                return
            response = await self.dispatcher.handle(line.decode().strip())
            if response is not None:
                writer.write((response + '\n').encode())
                await writer.drain()


def create_dispatcher(cfg, controller=None, on_reboot=None, fw='0.1', config_path='board.json'):
    """Build a Dispatcher with the standard command handlers, wired to the running config, the
    Inspector, and (optionally) the Controller. `on_reboot` lets tests intercept the reset."""
    import gc
    import time

    import config as config_mod

    board_id = cfg['board']['id']
    dispatcher = Dispatcher()

    def state():
        return controller.state if controller is not None else 'setting'

    async def whoami(msg):
        info = {
            'mcu': cfg['board'].get('mcu'),
            'fw': fw,
            'config_id': config_mod.config_id(cfg),
            'state': state(),
            'uptime': time.ticks_ms(),
        }
        return cc.build('iam', [board_id, json.dumps(info)])  # the one reply carrying the id

    async def ping(msg):
        return cc.build('pong')

    async def health(msg):
        try:
            import esp32

            temp = esp32.mcu_temperature()
        except Exception:
            temp = None
        info = {'temp': temp, 'mem_free': gc.mem_free(), 'uptime': time.ticks_ms(), 'state': state()}
        if controller is not None:
            info['tasks'] = [{'name': t.name, 'ok': t.validate()} for t in controller.active()]
        return cc.build('ok', [json.dumps(info)])

    async def get_state(msg):
        return cc.build('ok', [json.dumps({'state': state()})])

    async def report(msg):
        return cc.build('ok', [json.dumps(controller.stats() if controller is not None else {})])

    async def objects(msg):
        return cc.build('ok', [json.dumps(Inspector.names())])

    async def inspect(msg):
        if not msg.args:
            return cc.build('err', ['badargs', 'inspect <object>'])
        try:
            return cc.build('ok', [json.dumps(Inspector.inspect(msg.args[0]))])
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])

    async def update(msg):
        if len(msg.args) < 2:
            return cc.build('err', ['badargs', 'update <object> <json>'])
        try:
            changed = Inspector.update(msg.args[0], json.loads(msg.args[1]))
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])
        return cc.build('ok', [json.dumps({'changed': changed})])

    async def stats(msg):
        if not msg.args:
            return cc.build('err', ['badargs', 'stats <object>'])
        try:
            return cc.build('ok', [json.dumps(Inspector.stats(msg.args[0]))])
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])

    async def get_config(msg):
        which = msg.args[0] if msg.args else 'running'
        config = config_mod._builtin_default() if which == 'default' else cfg
        return cc.build('ok', [json.dumps(config)])

    async def save_config(msg):
        if not msg.args:
            return cc.build('err', ['badargs', 'no config'])
        try:
            new_config = json.loads(msg.args[0])
        except Exception:
            return cc.build('err', ['badargs', 'bad json'])
        try:
            config_id = config_mod.save(new_config, config_path)
        except ValueError as error:
            return cc.build('err', ['invalid', str(error)])
        return cc.build('ok', [json.dumps({'config_id': config_id})])

    async def reset_config(msg):
        config_mod.reset(config_path)
        return cc.build('ok')

    async def save_mission(msg):
        mission = Inspector.get('mission')
        if mission is None:
            return cc.build('err', ['unsupported', 'no mission'])
        mission.save()  # persist the live mission (set via `update mission`) to launch.config
        return cc.build('ok')

    async def reboot(msg):
        reset = on_reboot or (lambda: __import__('machine').reset())  # imported only when it fires

        async def do_reset():
            await asyncio.sleep_ms(200)
            reset()

        asyncio.create_task(do_reset())
        return cc.build('ok')

    dispatcher.on('whoami', whoami)
    dispatcher.on('ping', ping)
    dispatcher.on('health', health)
    dispatcher.on('state', get_state)
    dispatcher.on('report', report)
    dispatcher.on('objects', objects)
    dispatcher.on('inspect', inspect)
    dispatcher.on('update', update)
    dispatcher.on('stats', stats)
    dispatcher.on('get-config', get_config)
    dispatcher.on('save-config', save_config)
    dispatcher.on('reset-config', reset_config)
    dispatcher.on('save-mission', save_mission)
    dispatcher.on('reboot', reboot)
    return dispatcher
