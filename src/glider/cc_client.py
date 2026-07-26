"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Board side of the Control protocol (doc/specs/cc-protocol.md). Board-first routing: Control strips the
routing board id, so the board receives `command params` and replies `status params` (no id; only
`iam` carries the board id, so Control can learn it on a new socket). Dispatcher turns a parsed line
into a response (pure logic, unit-testable); Client is the thin networking that reads lines and
writes responses.
"""

import asyncio
import gc
import json
import time

import cc_protocol as cc
import config as config_mod
import databoard
import inspector
import recorder

try:
    import esp32
except ImportError:  # host (CPython): board-only; the health MCU-temp read degrades to None
    esp32 = None


def _enabled(tasks: dict, name: str):
    """
    The named task dict when it is present and enabled, else None.

    Args:
        tasks - name -> task dict, indexed from the config 'components' list.
        name - the task name to look up.

    Returns:
        The task dict when it exists and its 'enabled' flag is truthy, otherwise None.
    """
    task = tasks.get(name)
    return task if task is not None and task.get('enabled') else None


def _readiness(cfg: dict) -> dict:
    """
    Flight-readiness gate: field-dangerous CONFIG states no device probe can see.

    These are config choices that leave the board ballistic or unrecoverable in flight while every
    peripheral still probes healthy: the watchdog off (a wedged flight loop never reboots), the
    flight loop off or its gains zero (fins hold neutral -- ballistic), a bench fin derating still
    applied, or no landing zone and no CC-less site to pick one from. Bench and HITL sessions
    legitimately trip these, so `verify` reports them as a separate `ready` verdict without failing
    the hardware `pass`.

    Args:
        cfg - the active board config (top level, with 'components' and the 'fins' section).

    Returns:
        {check: problem} for each tripped gate; an empty dict means flight-ready.
    """
    problems = {}
    tasks = {task.get('name'): task for task in cfg.get('components', [])}

    # Watchdog off: a wedged flight loop never reboots.
    if _enabled(tasks, 'watchdog') is None:
        problems['watchdog'] = 'disabled: a wedged flight loop never reboots'

    # Flight loop off, or running with no gains -- either way the fins hold neutral (ballistic).
    flight = _enabled(tasks, 'flight')
    if flight is None:
        problems['flight'] = 'disabled: fins hold neutral -- ballistic'
    else:
        gains = flight.get('gains', {})
        slack = [axis for axis in ('roll', 'pitch', 'yaw') if not any(gains.get(axis, {}).values())]
        if slack:
            problems['gains'] = 'zero/unset on ' + ', '.join(slack) + ' -- the loop computes but cannot act'

    # Bench fin derating still applied.
    multiplier = cfg.get('fins', {}).get('limit_multiplier', 1.0)
    if multiplier != 1.0:
        problems['fins.limit_multiplier'] = '%g: a bench derating is still applied' % multiplier

    # No landing zone and no CC-less site to select one from.
    mission = inspector.Inspector.get('mission')
    if mission is not None and mission.zone is None and not mission.sites:
        problems['zone'] = 'no landing zone set and no CC-less sites to select one from'

    return problems


class Dispatcher:
    """
    Command name -> async handler(msg) registry that turns a request line into a response line.

    Pure routing with no networking (Client owns the socket), so the command logic stays
    unit-testable off a live connection. Register handlers with on(); dispatch a line with handle().
    """

    def __init__(self):
        self.handlers = {}

    def on(self, command: str, fn) -> None:
        self.handlers[command] = fn

    async def handle(self, line: str) -> str:
        msg = cc.parse(line)
        if msg.command is None:
            return None
        handler = self.handlers.get(msg.command)
        if handler is None:
            return cc.build('err', ['badcmd', msg.command])
        try:
            return await handler(msg)
        except Exception as error:
            return cc.build('err', ['internal', repr(error)])


class Client:
    """The thin CC networking: dial Control, read request lines, write the dispatcher's responses."""

    def __init__(self, config: dict, dispatcher, log=None, backoff_ms: int = 1000):
        wifi = config.get('wifi', {})  # .get like every other config read -- a minimal config (no wifi
        # section) falls back to the cc_host/cc_port defaults below instead of KeyErroring before them
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


class _Context:
    """
    Shared state the CC handlers close over.

    Bundles the running config, the optional Controller, the reboot hook, the config path, and the
    board id (cached from the config) so the _register_* groups below each take one object instead of
    a long argument list.
    """

    def __init__(self, cfg: dict, controller, on_reboot, config_path: str):
        self.cfg = cfg
        self.controller = controller
        self.on_reboot = on_reboot
        self.config_path = config_path
        self.board_id = cfg['board']['id']

    def stage(self) -> str:
        """Current stage name, or 'setting' when running without a Controller (unit tests)."""
        return self.controller.stage_name() if self.controller is not None else 'setting'


def _register_identity(dispatcher, ctx) -> None:
    """whoami / ping / health -- who the board is and how it is doing."""
    async def whoami(_unused_msg) -> str:
        info = {
            'mcu': ctx.cfg['board'].get('mcu'),
            'firmware_version': ctx.cfg['board'].get('firmware_version', 'dev'),
            'config_id': config_mod.config_id(ctx.cfg),
            'stage': ctx.stage(),
            'uptime': time.ticks_ms(),
        }
        return cc.build('iam', [ctx.board_id, json.dumps(info)])  # the one reply carrying the id

    async def ping(_unused_msg) -> str:
        return cc.build('pong')

    async def health(_unused_msg) -> str:
        try:
            temp = esp32.mcu_temperature()
        except Exception:  # host (esp32 None) or a probe failure -> no temperature
            temp = None
        info = {'temp': temp, 'mem_free': gc.mem_free(), 'uptime': time.ticks_ms(), 'stage': ctx.stage()}
        position = databoard.Databoard.parameter('position')  # board GNSS fix -> dashboard (None until a fix)
        if position is not None:
            value, source, _age = position.read()
            info['position'] = value if source is not None and value is not None else None
        mission = inspector.Inspector.get('mission')
        if mission is not None:  # the board wall-clock (RTC) -> the dashboard top table shows it live
            info['clock'] = mission.clock()
            info['epoch'] = mission.epoch()
            """
            the launchpad the board would fly with, for the dashboard safety cell: the effective origin
            (CC-set/frozen, else the live fix -- which freeze_launch pins at arm), whether it is already
            persistent, and the selected site name (CC-less site-by-GPS)
            """
            info['launchpad'] = mission.launch_point()
            info['launchpad_set'] = mission.latitude is not None and mission.longitude is not None
            info['site'] = mission.site or None
        agl = databoard.Databoard.parameter('agl')  # low-altitude laser AGL -> the flight panel
        if agl is not None:  # FRESH only -- out of the laser's ~4 m range an extrapolated agl is fiction
            value, source, _age = agl.read()
            info['agl'] = value if source is not None else None
        """
        degraded-mode annunciation: the non-nominal states, gathered into one operator signal so a glance
        shows the board is NOT flying clean (attitude on the backup, memory rescued, warm restart, CC-less
        fallback zone). Empty list = nominal.
        """
        degraded = []
        attitude = databoard.Databoard.parameter('attitude')
        if attitude is not None and attitude.read()[1] == 'attitude':  # fused source IS the backup
            degraded.append('attitude-backup')
        health = inspector.Inspector.get('health')
        if getattr(health, 'rescues', 0) > 0:
            degraded.append('memory-rescued')
        mission = inspector.Inspector.get('mission')
        if mission is not None and mission.site == 'fallback':
            degraded.append('cc-less-fallback')
        if ctx.controller is not None:
            info['armed'] = ctx.controller.armed
            if getattr(ctx.controller, 'warm_started', False):
                degraded.append('warm-started')
            flight = ctx.controller.active('flight')  # live flight panel: airspeed + fin cap + reach
            if flight is not None and hasattr(flight, 'vitals'):
                info['flight'] = flight.vitals()
            info['tasks'] = [{'name': task.name, 'ok': task.validate()} for task in ctx.controller.active()]
        info['degraded'] = degraded
        return cc.build('ok', [json.dumps(info)])

    dispatcher.on('whoami', whoami)
    dispatcher.on('ping', ping)
    dispatcher.on('health', health)


def _register_control(dispatcher, ctx) -> None:
    """stage / arm / disarm / report -- flight-stage hold + actuation enable."""

    async def get_stage(msg) -> str:
        """
        Read or hold the flight stage.

        `stage` reports the current stage; `stage <name>` holds it (an operator override that pauses
        the auto-sequencer, for ground test); `stage auto` resumes auto-sequencing.

        Args:
            msg - the request; msg.args[0], when present, is the stage name to hold or 'auto'.

        Returns:
            ok with {stage, manual}; err badargs when the named stage does not exist.
        """
        manual = ctx.controller.manual if ctx.controller is not None else False
        if msg.args and ctx.controller is not None:
            if msg.args[0] == 'auto':
                ctx.controller.resume()
            elif not ctx.controller.hold(msg.args[0]):
                return cc.build('err', ['badargs', 'no stage ' + msg.args[0]])
            manual = ctx.controller.manual
        return cc.build('ok', [json.dumps({'stage': ctx.stage(), 'manual': manual})])

    async def arm(_unused_msg) -> str:
        """
        Enable actuation, but only when board verify is clean.

        Arms only when every device is up and probes healthy (including the mission launch position);
        a refused arm returns the problems instead. The board is disarmed by default -- the control
        loop holds the fins neutral until armed.

        Args:
            msg - the request (unused).

        Returns:
            ok {armed: true} on success; err unsupported when there is no controller; err unsafe
            carrying the problems dict when a device is down or fails its probe.
        """
        if ctx.controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        problems = dict(ctx.controller.failures)  # not-connected devices
        for name, result in (await inspector.Inspector.probe_all()).items():  #
            if result is not None:
                problems[name] = result
        if problems:
            return cc.build('err', ['unsafe', json.dumps(problems)])  # refuse to arm
        ctx.controller.arm()
        return cc.build('ok', [json.dumps({'armed': True})])

    async def disarm(_unused_msg) -> str:
        if ctx.controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        ctx.controller.disarm()
        return cc.build('ok', [json.dumps({'armed': False})])

    async def report(_unused_msg) -> str:
        return cc.build('ok', [json.dumps(ctx.controller.stats() if ctx.controller is not None else {})])

    dispatcher.on('stage', get_stage)
    dispatcher.on('arm', arm)
    dispatcher.on('disarm', disarm)
    dispatcher.on('report', report)


def _register_inspection(dispatcher) -> None:
    """objects / inspect / update / stats -- the Inspector surface for operator-facing objects."""

    async def objects(_unused_msg) -> str:
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

    dispatcher.on('objects', objects)
    dispatcher.on('inspect', inspect)
    dispatcher.on('update', update)
    dispatcher.on('stats', stats)


def _register_config(dispatcher, ctx) -> None:
    """
    get-config / set-config / reset-config -- read and write the named configs.

    One command pair (get-config <name> / set-config <name> <json>) covers every config instead of a
    get-/save- pair per config:
      board   the running board config (hardware; config.py, validated + atomically saved)
      default the built-in board default (read-only)
      launch  the per-launch mission (launch.config; mission.py, merge-applied + saved)
    """
    async def get_config(msg) -> str:
        """
        Read the named config.

        Args:
            msg - the request; msg.args[0], when present, is the config name (default 'board').

        Returns:
            ok with the config JSON (board/running the running config, default the built-in default,
            launch the persisted mission); err unsupported when launch is asked for with no mission;
            err badargs on an unknown name.
        """
        name = msg.args[0] if msg.args else 'board'
        if name in ('board', 'running'):
            return cc.build('ok', [json.dumps(ctx.cfg)])
        if name == 'default':
            return cc.build('ok', [json.dumps(config_mod._builtin_default())])
        if name == 'launch':
            mission = inspector.Inspector.get('mission')
            if mission is None:
                return cc.build('err', ['unsupported', 'no mission'])
            return cc.build('ok', [json.dumps(mission.persisted())])
        return cc.build('err', ['badargs', 'unknown config %s' % name])

    async def set_config(msg) -> str:
        """
        Save the named config.

        `board` validates the JSON and atomically replaces the running config; `launch` merge-applies
        the fields into the mission and persists them.

        Args:
            msg - the request; msg.args[0] is the config name, msg.args[1] the JSON payload.

        Returns:
            ok with the saved result ({config_id} for board, the persisted mission for launch); err
            badargs on a missing arg / bad JSON / unknown name; err invalid when board validation
            fails; err unsupported when launch is asked for with no mission.
        """
        if len(msg.args) < 2:
            return cc.build('err', ['badargs', 'set-config <name> <json>'])
        name = msg.args[0]
        try:
            payload = json.loads(msg.args[1])
        except ValueError:
            return cc.build('err', ['badargs', 'bad json'])
        if name == 'board':
            try:
                config_id = config_mod.save(payload, ctx.config_path)
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

    async def reset_config(_unused_msg) -> str:
        config_mod.reset(ctx.config_path)
        return cc.build('ok')

    dispatcher.on('get-config', get_config)
    dispatcher.on('set-config', set_config)
    dispatcher.on('reset-config', reset_config)


def _register_diagnostics(dispatcher, ctx) -> None:
    """probe / verify -- on-demand self-tests + the pre-flight pass/fail check."""

    async def probe(msg) -> str:
        """
        Run device self-tests on demand over the inspectable objects.

        Runs probe() on the inspectable objects (tasks + mission + ...) that implement it:
        `probe <name>` for one, `probe` / `probe all` for every one. Each object reports None when
        healthy, else its error string. `probe all` also lists the devices that never set up (absent /
        miswired -> not inspectable), from the Controller's failures, so one command shows the whole
        connected/not picture. Costly ACTIVE checks (e.g. the servo range sweep) live in probe(),
        never at boot -- so a mid-flight reboot never sweeps the fins; the operator runs it pre-flight.
        Sequential, so fins self-test one at a time.

        Args:
            msg - the request; msg.args[0], when present, is the object to probe (default 'all').

        Returns:
            ok with {name: result} (None healthy, else the error string); err badargs when a named
            object has no probe.
        """
        target = msg.args[0] if msg.args else 'all'
        if target == 'all':
            results = await inspector.Inspector.probe_all()  #
            if ctx.controller is not None:  # devices that failed setup aren't inspectable -> not connected
                for name, reason in ctx.controller.failures.items():
                    results.setdefault(name, 'not connected: ' + reason)
            return cc.build('ok', [json.dumps(results)])
        run = getattr(inspector.Inspector.get(target), 'probe', None)
        if run is None:
            # a device that failed setup isn't inspectable -> surface its (diagnosed) failure reason
            if ctx.controller is not None and target in ctx.controller.failures:
                return cc.build('ok', [json.dumps({target: 'not connected: ' + ctx.controller.failures[target]})])
            return cc.build('err', ['badargs', 'no probe for ' + target])
        return cc.build('ok', [json.dumps({target: await run()})])

    async def verify(_unused_msg) -> str:
        """
        Verify board setup and report PASS or the problems.

        Dumps every configured device (up vs down) and runs the probe self-tests, with an overall
        `pass`, plus the flight-readiness CONFIG gate as a separate `ready`/`readiness` verdict (a
        bench session is healthy but not flight-ready). This is the on-the-pad / pre-flight re-check
        -- it catches anything disconnected in transport. Needs the Controller (the configured device
        list + setup failures). Note the probe is active (it sweeps the servos).

        Args:
            msg - the request (unused).

        Returns:
            ok with {pass, devices, problems, ready, readiness}; err unsupported when there is no
            controller.
        """
        if ctx.controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        devices = {name: ('up' if ctx.controller.active(name) is not None
                          else 'down: ' + ctx.controller.failures.get(name, '?'))
                   for name in ctx.controller.directory()}
        problems = dict(ctx.controller.failures)  # not-connected devices
        for name, result in (await inspector.Inspector.probe_all()).items():  #
            if result is not None:
                problems[name] = result
        readiness = _readiness(ctx.cfg)
        return cc.build('ok', [json.dumps({'pass': not problems, 'devices': devices, 'problems': problems,
                                           'ready': not readiness, 'readiness': readiness})])

    async def bustune(msg) -> str:
        """
        Retune a sensor bus frequency live and report which devices stay healthy.

        `bustune <kind> <id> <freq>` retunes an i2c/spi bus to <freq> Hz without a reboot and reports
        which devices on it stay healthy. The CC-side `calibrate` sweep drives this to find each bus's
        maximum stable frequency and the limiting device; it never persists (CC saves the chosen freq
        via set-config board + reboot).

        Args:
            msg - the request; msg.args are the bus kind, the bus id, and the frequency in Hz.

        Returns:
            ok with the per-device health after the retune; err unsupported when there is no
            controller; err badargs on missing args or a non-integer frequency.
        """
        if ctx.controller is None:
            return cc.build('err', ['unsupported', 'no controller'])
        if len(msg.args) < 3:
            return cc.build('err', ['badargs', 'bustune <kind> <id> <freq>'])
        try:
            freq = int(msg.args[2])
        except ValueError:
            return cc.build('err', ['badargs', 'freq must be an int (Hz)'])
        return cc.build('ok', [json.dumps(await ctx.controller.bustune(msg.args[0], msg.args[1], freq))])

    dispatcher.on('probe', probe)
    dispatcher.on('verify', verify)
    dispatcher.on('bustune', bustune)


def _register_streaming(dispatcher) -> None:
    """log / tlm -- poll-model log + telemetry streaming (recorder only; no controller/config)."""

    async def log(msg) -> str:
        """
        Poll-model log streaming.

        `log <duration_ms>` replies with the log lines the board buffered since the last `log`, and
        keeps teeing log() for another `duration_ms` (the operator re-sends it each tick; `log 0`
        stops, default 1000 ms). The batch rides back as one base64 token (a JSON list). An EXTRA
        route: the UART/Luckfox log path is untouched, and with no `log` request the board collects
        nothing -- a lost link cannot grow memory once the window lapses.

        Args:
            msg - the request; msg.args[0], when present, is the window in milliseconds (default
            1000).

        Returns:
            ok with the buffered log lines; err badargs when the duration is not an integer.
        """
        try:
            duration_ms = int(msg.args[0]) if msg.args else 1000
        except ValueError:
            return cc.build('err', ['badargs', 'log <duration_ms>'])
        return cc.build('ok', [json.dumps(recorder.Recorder.cc_logs(duration_ms))])

    async def tlm(msg) -> str:
        """
        Poll-model telemetry streaming (mirrors `log`).

        `tlm <duration_ms>` replies with the telemetry rows the board buffered since the last `tlm`,
        and keeps teeing tlm() for another `duration_ms` (`tlm 0` stops, default 1000 ms). One base64
        JSON token. An EXTRA route: the UART/Luckfox telemetry is untouched, and with no `tlm` request
        the board collects nothing.

        Args:
            msg - the request; msg.args[0], when present, is the window in milliseconds (default
            1000).

        Returns:
            ok with the buffered telemetry rows; err badargs when the duration is not an integer.
        """
        try:
            duration_ms = int(msg.args[0]) if msg.args else 1000
        except ValueError:
            return cc.build('err', ['badargs', 'tlm <duration_ms>'])
        return cc.build('ok', [json.dumps(recorder.Recorder.cc_telemetry(duration_ms))])

    dispatcher.on('log', log)
    dispatcher.on('tlm', tlm)


def _register_system(dispatcher, ctx) -> None:
    """reboot -- restart the board (delayed so the `ok` reply flushes before the reset)."""

    async def reboot(_unused_msg) -> str:
        reset = ctx.on_reboot or (lambda: __import__('machine').reset())  # imported only when it fires

        async def do_reset() -> None:
            await asyncio.sleep_ms(200)
            reset()

        asyncio.create_task(do_reset())
        return cc.build('ok')

    dispatcher.on('reboot', reboot)


def create_dispatcher(cfg: dict, controller=None, on_reboot=None,
                      config_path: str = 'board.config') -> Dispatcher:
    """
    Build a Dispatcher with the standard command handlers.

    Wires the handlers to the running config, the Inspector, and (optionally) the Controller.
    `on_reboot` lets tests intercept the reset. The handlers are grouped by concern into the
    _register_* helpers; each closes over one shared _Context, so this stays a short orchestrator.

    Args:
        cfg - the running board config.
        controller - the flight Controller, or None (unit tests / recorder-only nodes).
        on_reboot - a reset hook the reboot handler calls, or None to fall back to machine.reset().
        config_path - where set-config board / reset-config persist the board config.

    Returns:
        The wired Dispatcher.
    """
    dispatcher = Dispatcher()
    ctx = _Context(cfg, controller, on_reboot, config_path)
    _register_identity(dispatcher, ctx)
    _register_control(dispatcher, ctx)
    _register_inspection(dispatcher)
    _register_config(dispatcher, ctx)
    _register_diagnostics(dispatcher, ctx)
    _register_streaming(dispatcher)
    _register_system(dispatcher, ctx)
    return dispatcher
