# cc_client — board side of the Control protocol (specs/cc-protocol.md). Board-first routing:
# Control strips the routing board id, so the board receives `command params` and replies
# `status params` (no id; only `iam` carries the board id, so Control can learn it on a new
# socket). Dispatcher turns a parsed line into a response (pure logic, unit-testable); Client is
# the thin networking that reads lines and writes responses.

import asyncio
import json

import cc_protocol as cc
import inspector


class Dispatcher:
    """Maps a command to an async handler(msg) -> response line."""

    def __init__(self):
        self.handlers = {}

    def on(self, command: str, fn) -> None:
        self.handlers[command] = fn

    async def handle(self, line: str) -> str:
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
    def __init__(self, config: dict, dispatcher, log=None, backoff_ms: int = 1000):
        wifi = config['wifi']
        self.host = wifi.get('cc_host')  # None -> cc_link derives the `.1` of the board's subnet at dial
        self.port = wifi.get('cc_port', 1234)
        self.dispatcher = dispatcher
        self.log = log if log is not None else (lambda message: None)
        self.backoff_ms = backoff_ms

    async def run(self) -> None:
        """Connect to Control and serve forever, reconnecting with backoff on drop."""
        while True:
            try:
                reader, writer = await asyncio.open_connection(self.host, self.port)
                self.log('cc_client :: connected %s:%d' % (self.host, self.port))
                await self.serve(reader, writer)
            except Exception as error:
                self.log('cc_client :: %r' % error)
            await asyncio.sleep_ms(self.backoff_ms)

    async def serve(self, reader, writer) -> None:
        """Read commands from Control, dispatch, write responses. Returns on disconnect."""
        while True:
            line = await reader.readline()
            if not line:
                return
            response = await self.dispatcher.handle(line.decode().strip())
            if response is not None:
                writer.write((response + '\n').encode())
                await writer.drain()


def create_dispatcher(cfg: dict, controller=None, on_reboot=None,
                      config_path: str = 'board.json') -> Dispatcher:
    """Build a Dispatcher with the standard command handlers, wired to the running config, the
    Inspector, and (optionally) the Controller. `on_reboot` lets tests intercept the reset."""
    import gc
    import time

    import config as config_mod

    board_id = cfg['board']['id']
    dispatcher = Dispatcher()

    def stage() -> str:
        return controller.stage_name() if controller is not None else 'setting'

    async def whoami(msg) -> str:
        info = {
            'mcu': cfg['board'].get('mcu'),
            'firmware_version': cfg['board'].get('firmware_version', 'dev'),
            'config_id': config_mod.config_id(cfg),
            'stage': stage(),
            'uptime': time.ticks_ms(),
        }
        return cc.build('iam', [board_id, json.dumps(info)])  # the one reply carrying the id

    async def ping(msg) -> str:
        return cc.build('pong')

    async def health(msg) -> str:
        try:
            import esp32

            temp = esp32.mcu_temperature()
        except Exception:
            temp = None
        info = {'temp': temp, 'mem_free': gc.mem_free(), 'uptime': time.ticks_ms(), 'stage': stage()}
        mission = inspector.Inspector.get('mission')
        if mission is not None:  # the board wall-clock (RTC) -> the dashboard top table shows it live
            info['clock'] = mission.clock()
            info['epoch'] = mission.epoch()
        if controller is not None:
            info['tasks'] = [{'name': t.name, 'ok': t.validate()} for t in controller.active()]
        return cc.build('ok', [json.dumps(info)])

    async def get_stage(msg) -> str:
        """`stage` -> the current stage; `stage <name>` holds it (operator override, pauses the
        sequencer -- ground test); `stage auto` resumes auto-sequencing."""
        manual = controller.manual if controller is not None else False
        if msg.args and controller is not None:
            if msg.args[0] == 'auto':
                controller.resume()
            elif not controller.hold(msg.args[0]):
                return cc.build('err', ['badargs', 'no stage ' + msg.args[0]])
            manual = controller.manual
        return cc.build('ok', [json.dumps({'stage': stage(), 'manual': manual})])

    async def arm(msg) -> str:
        """Enable actuation -- but only when board verify is clean (every device up + probe healthy,
        incl. the mission launch-position): a refused arm returns the problems. Disarmed by default;
        the control loop holds the fins neutral until armed."""
        if controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        problems = dict(controller.failures)  # not-connected devices
        for name in inspector.Inspector.names():
            run = getattr(inspector.Inspector.get(name), 'probe', None)
            if run is not None:
                result = await run()
                if result is not None:
                    problems[name] = result
        if problems:
            return cc.build('err', ['unsafe', json.dumps(problems)])  # refuse to arm
        controller.arm()
        return cc.build('ok', [json.dumps({'armed': True})])

    async def disarm(msg) -> str:
        if controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        controller.disarm()
        return cc.build('ok', [json.dumps({'armed': False})])

    async def report(msg) -> str:
        return cc.build('ok', [json.dumps(controller.stats() if controller is not None else {})])

    async def objects(msg) -> str:
        return cc.build('ok', [json.dumps(inspector.Inspector.names())])

    async def inspect(msg) -> str:
        if not msg.args:
            return cc.build('err', ['badargs', 'inspect <object>'])
        try:
            return cc.build('ok', [json.dumps(inspector.Inspector.inspect(msg.args[0]))])
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])

    async def update(msg) -> str:
        if len(msg.args) < 2:
            return cc.build('err', ['badargs', 'update <object> <json>'])
        try:
            changed = inspector.Inspector.update(msg.args[0], json.loads(msg.args[1]))
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])
        return cc.build('ok', [json.dumps({'changed': changed})])

    async def stats(msg) -> str:
        if not msg.args:
            return cc.build('err', ['badargs', 'stats <object>'])
        try:
            return cc.build('ok', [json.dumps(inspector.Inspector.stats(msg.args[0]))])
        except KeyError:
            return cc.build('err', ['badargs', 'no object ' + msg.args[0]])

    # The named configs the operator can read/write through one pair of commands (get-config <name> /
    # set-config <name> <json>), instead of a get-/save- pair per config:
    #   board    the running board config (hardware; config.py, validated + atomically saved)
    #   default  the built-in board default (read-only)
    #   launch   the per-launch mission (launch.config; mission.py, merge-applied + saved)
    async def get_config(msg) -> str:
        """`get-config [name]` -- the named config (default `board`)."""
        name = msg.args[0] if msg.args else 'board'
        if name in ('board', 'running'):
            return cc.build('ok', [json.dumps(cfg)])
        if name == 'default':
            return cc.build('ok', [json.dumps(config_mod._builtin_default())])
        if name == 'launch':
            mission = inspector.Inspector.get('mission')
            if mission is None:
                return cc.build('err', ['unsupported', 'no mission'])
            return cc.build('ok', [json.dumps(mission.persisted())])
        return cc.build('err', ['badargs', 'unknown config %s' % name])

    async def set_config(msg) -> str:
        """`set-config <name> <json>` -- save the named config. `board` validates + atomically replaces
        the running config; `launch` merge-applies the fields into the mission and persists them."""
        if len(msg.args) < 2:
            return cc.build('err', ['badargs', 'set-config <name> <json>'])
        name = msg.args[0]
        try:
            payload = json.loads(msg.args[1])
        except ValueError:
            return cc.build('err', ['badargs', 'bad json'])
        if name == 'board':
            try:
                config_id = config_mod.save(payload, config_path)
            except ValueError as error:
                return cc.build('err', ['invalid', str(error)])
            return cc.build('ok', [json.dumps({'config_id': config_id})])
        if name == 'launch':
            mission = inspector.Inspector.get('mission')
            if mission is None:
                return cc.build('err', ['unsupported', 'no mission'])
            mission.update(payload)  # key-wise merge (a missing field is left as-is, never wiped)
            mission.save()
            return cc.build('ok', [json.dumps(mission.persisted())])
        return cc.build('err', ['badargs', 'unknown config %s' % name])

    async def reset_config(msg) -> str:
        config_mod.reset(config_path)
        return cc.build('ok')

    async def probe(msg) -> str:
        """Run self-tests ON DEMAND over the inspectable objects (tasks + mission + ...) that implement
        probe(): `probe <name>` for one, `probe`/`probe all` for every one. Per object None when
        healthy, else its error string. `probe all` also lists the devices that never set up (absent /
        miswired -> not inspectable), from the Controller's failures, so one command shows the whole
        connected/not picture. Costly ACTIVE checks (e.g. the servo range sweep) live in probe(), never
        at boot -- so a mid-flight reboot never sweeps the fins; the operator runs it pre-flight.
        Sequential, so fins self-test one at a time."""
        target = msg.args[0] if msg.args else 'all'
        if target == 'all':
            results = {}
            for name in inspector.Inspector.names():
                run = getattr(inspector.Inspector.get(name), 'probe', None)
                if run is not None:
                    results[name] = await run()
            if controller is not None:  # devices that failed setup aren't inspectable -> not connected
                for name, reason in controller.failures.items():
                    results.setdefault(name, 'not connected: ' + reason)
            return cc.build('ok', [json.dumps(results)])
        run = getattr(inspector.Inspector.get(target), 'probe', None)
        if run is None:
            return cc.build('err', ['badargs', 'no probe for ' + target])
        return cc.build('ok', [json.dumps({target: await run()})])

    async def verify(msg) -> str:
        """Verify board setup and report PASS or the problems: dump every configured device (up vs
        down) and run the probe self-tests, with an overall `pass`. The on-the-pad / pre-flight
        re-check -- catches anything disconnected in transport. Needs the Controller (the configured
        device list + setup failures). NOTE: probe is active (sweeps the servos)."""
        if controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        devices = {name: ('up' if controller.active(name) is not None
                          else 'down: ' + controller.failures.get(name, '?'))
                   for name in controller.directory()}
        problems = dict(controller.failures)  # not-connected devices
        for name in inspector.Inspector.names():
            run = getattr(inspector.Inspector.get(name), 'probe', None)
            if run is not None:
                result = await run()
                if result is not None:
                    problems[name] = result
        return cc.build('ok', [json.dumps({'pass': not problems, 'devices': devices, 'problems': problems})])

    async def log(msg) -> str:
        """`log <duration_ms>` -- poll-model log streaming. Reply with the log lines the board buffered
        since the last `log`, and keep teeing log() for another `duration_ms` (the operator re-sends it
        each tick; `log 0` stops, default 1000 ms). The batch rides back as one base64 token (a JSON
        list). An EXTRA route: the UART/Luckfox log path is untouched, and with no `log` request the
        board collects nothing -- a lost link cannot grow memory once the window lapses."""
        import recorder

        try:
            duration_ms = int(msg.args[0]) if msg.args else 1000
        except ValueError:
            return cc.build('err', ['badargs', 'log <duration_ms>'])
        return cc.build('ok', [json.dumps(recorder.Recorder.cc_logs(duration_ms))])

    async def tlm(msg) -> str:
        """`tlm <duration_ms>` -- poll-model telemetry streaming (mirrors `log`). Reply with the
        telemetry rows the board buffered since the last `tlm`, and keep teeing tlm() for another
        `duration_ms` (`tlm 0` stops, default 1000 ms). One base64 JSON token. An EXTRA route: the
        UART/Luckfox telemetry is untouched, and with no `tlm` request the board collects nothing."""
        import recorder

        try:
            duration_ms = int(msg.args[0]) if msg.args else 1000
        except ValueError:
            return cc.build('err', ['badargs', 'tlm <duration_ms>'])
        return cc.build('ok', [json.dumps(recorder.Recorder.cc_telemetry(duration_ms))])

    async def reboot(msg) -> str:
        reset = on_reboot or (lambda: __import__('machine').reset())  # imported only when it fires

        async def do_reset() -> None:
            await asyncio.sleep_ms(200)
            reset()

        asyncio.create_task(do_reset())
        return cc.build('ok')

    dispatcher.on('whoami', whoami)
    dispatcher.on('ping', ping)
    dispatcher.on('health', health)
    dispatcher.on('stage', get_stage)
    dispatcher.on('arm', arm)
    dispatcher.on('disarm', disarm)
    dispatcher.on('report', report)
    dispatcher.on('objects', objects)
    dispatcher.on('inspect', inspect)
    dispatcher.on('update', update)
    dispatcher.on('stats', stats)
    dispatcher.on('probe', probe)
    dispatcher.on('verify', verify)
    dispatcher.on('log', log)
    dispatcher.on('tlm', tlm)
    dispatcher.on('get-config', get_config)
    dispatcher.on('set-config', set_config)
    dispatcher.on('reset-config', reset_config)
    dispatcher.on('reboot', reboot)
    return dispatcher
