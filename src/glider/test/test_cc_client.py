"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the CC client (cc_client.py). Board-first: the board socket sees
`command params` (no id) and replies `status params` (no id except iam). Run by `make test`.
"""

import asyncio
import json
import os

import cc_client
import cc_protocol as cc
import config_default
import inspector
import mission


class _FakeReader:
    def __init__(self, lines):
        self.queue = [item if isinstance(item, bytes) else (item + '\n').encode() for item in lines]
        self.index = 0

    async def readline(self):
        if self.index < len(self.queue):
            value = self.queue[self.index]
            self.index += 1
            return value
        return b''


class _FakeWriter:
    def __init__(self):
        self.out = []

    def write(self, data):
        self.out.append(data)

    async def drain(self):
        pass


class _Knob(inspector.Inspectable):
    name = 'knob'
    kind = 'knob'
    _inspect = ('level',)
    _writable = ('level',)

    def __init__(self):
        self.level = 1


async def amain():
    # generic Dispatcher: no board-id handling; command -> handler
    dispatcher = cc_client.Dispatcher()

    async def ping(msg):
        return cc.build('pong')

    dispatcher.on('ping', ping)
    assert await dispatcher.handle('ping') == 'pong'
    assert 'badcmd' in await dispatcher.handle('nope')  # unknown command
    assert await dispatcher.handle('   ') is None  # empty line

    async def boom(msg):
        raise ValueError('x')

    dispatcher.on('boom', boom)
    assert 'internal' in await dispatcher.handle('boom')  # handler exception

    # standard handlers
    sd = cc_client.create_dispatcher(config_default.default())

    msg = cc.parse(await sd.handle('whoami'))
    assert msg.command == 'iam' and msg.args[0] == 'taster'
    info = json.loads(msg.args[1])
    assert info['mcu'] == 'esp32p4' and 'config_id' in info and info['stage'] == 'setting'

    assert cc.parse(await sd.handle('ping')).command == 'pong'
    health = json.loads(cc.parse(await sd.handle('health')).args[0])
    assert 'mem_free' in health and 'uptime' in health
    cfg = json.loads(cc.parse(await sd.handle('get-config')).args[0])
    assert cfg['board']['id'] == 'taster'

    # Inspector-backed inspect/update/stats
    inspector.Inspector.register(_Knob())
    assert 'knob' in json.loads(cc.parse(await sd.handle('objects')).args[0])
    assert json.loads(cc.parse(await sd.handle('inspect knob')).args[0]) == {'level': 1}
    changed = cc.parse(await sd.handle(cc.build('update', ['knob', json.dumps({'level': 5})])))
    assert json.loads(changed.args[0]) == {'changed': ['level']}
    assert json.loads(cc.parse(await sd.handle('inspect knob')).args[0])['level'] == 5
    assert 'badargs' in await sd.handle('inspect nope')  # unknown object

    # Client.serve over fake streams
    writer = _FakeWriter()
    await cc_client.Client(config_default.default(), sd).serve(_FakeReader(['whoami', 'ping']), writer)
    resp = [b.decode().strip() for b in writer.out]
    assert cc.parse(resp[0]).command == 'iam' and cc.parse(resp[1]).command == 'pong'

    # set-config board: invalid rejected; reset-config ok; bad args rejected
    sd2 = cc_client.create_dispatcher(config_default.default(), config_path='test_cc_board.config')
    bad = config_default.default()
    bad['pins']['servo_yaw'] = 18  # reserved Wi-Fi pin -> invalid
    assert 'invalid' in await sd2.handle(cc.build('set-config', ['board', json.dumps(bad)]))
    reply = cc.parse(await sd2.handle(cc.build('set-config', ['board', json.dumps(config_default.default())])))
    assert reply.command == 'ok' and 'config_id' in json.loads(reply.args[0])
    assert cc.parse(await sd2.handle('reset-config')).command == 'ok'
    assert 'badargs' in await sd2.handle('set-config board')  # no json
    assert 'badargs' in await sd2.handle(cc.build('set-config', ['nope', '{}']))  # unknown config name
    assert 'badargs' in await sd2.handle('get-config nope')  # unknown config name

    # config 'launch' target: get-config/set-config launch are unsupported until a Mission registers
    inspector.Inspector.unregister('mission')
    assert 'unsupported' in await sd.handle('get-config launch')
    assert 'unsupported' in await sd.handle(cc.build('set-config', ['launch', '{}']))
    mission.Mission('test_cc_launch.config').update({'launch_id': 'cc-t1'})  # registers itself

    # health now carries the board wall-clock (RTC) for the dashboard top table, plus the
    # launchpad safety fields (the effective origin / persistent-vs-live / selected site)
    vitals = json.loads(cc.parse(await sd.handle('health')).args[0])
    assert 'clock' in vitals and 'epoch' in vitals
    assert 'launchpad' in vitals and 'launchpad_set' in vitals and 'site' in vitals

    # the live flight panel: with a controller reporting a flight task, health carries its vitals
    # (airspeed / fin cap / active) + agl for the dashboard glance
    _VITALS = {'airspeed': 14.2, 'fin_cap': 30, 'active': True,
               'reach': {'reachable': True, 'margin_m': 120, 'distance_m': 40}}

    class _FlightTask:
        name = 'flight'

        def validate(self):
            return True

        def vitals(self):
            return _VITALS

    class _Pitot:
        """A device the board can calibrate itself -- the operator still has to hold it still."""

        def calibration(self):
            return 'keep the pitot in STILL AIR -- this captures the zero tare'

        async def calibrate(self):
            return None

    class _UncalibratedImu:
        """A healthy BNO055 that simply has not been moved yet -- mag calibration still 0."""

        calibration_state = (0, 3, 3, 0)
        calibration_value = (0, 3, 3, 0)

        @property
        def calibration(self):
            return self.calibration_value

        def calibrated(self):
            return self.calibration_value[3] >= 3

    class _FlightController:
        armed = True
        warm_started = True  # a degraded state -> annunciated
        failures = {}

        def stage_name(self):
            return 'gliding'

        def active(self, name=None):
            if name is None:
                return [_FlightTask()]
            if name == 'flight':
                return _FlightTask()
            return _UncalibratedImu() if name == 'imu_bno055' else None

    sd_flight = cc_client.create_dispatcher(config_default.default(), controller=_FlightController())
    panel = json.loads(cc.parse(await sd_flight.handle('health')).args[0])
    assert panel['armed'] is True and panel['flight'] == _VITALS  # airspeed / fin cap / reach ride along
    assert 'agl' in panel  # low-altitude laser AGL rides the same heartbeat
    # degraded-mode annunciation: warm-started (from the controller flag) is surfaced
    assert 'warm-started' in panel['degraded']
    """
    An UNCALIBRATED BNO055 must reach the operator. It is invisible to probe() (the part answers
    perfectly) and to _readiness() (it is not a config choice), yet NDOF fusion never converges without
    motion -- so a still glider reaches launch with a frozen attitude on HEALTHY hardware. Not
    reporting it is what got a working module condemned on this bench.
    """
    assert 'imu-uncalibrated' in panel['degraded']
    assert panel['imu_calibration'] == [0, 3, 3, 0]  # (sys, gyr, acc, mag) -- mag 0 names the fix
    # a clean board (no controller) reports an empty degraded list
    assert json.loads(cc.parse(await sd.handle('health')).args[0])['degraded'] == []

    # get-config launch returns the editable launch.config (persisted fields only, no computed geometry)
    got = json.loads(cc.parse(await sd.handle('get-config launch')).args[0])
    assert got['launch_id'] == 'cc-t1' and 'zone' in got and 'target' not in got and 'clock' not in got

    # set-config launch merge-applies a draft and persists it (like set-config board)
    draft = json.dumps({'launch_id': 'cc-t2', 'site': 'pad-z'})
    assert cc.parse(await sd.handle(cc.build('set-config', ['launch', draft]))).command == 'ok'
    saved = json.load(open('test_cc_launch.config'))
    assert saved['launch_id'] == 'cc-t2' and saved['site'] == 'pad-z'
    os.remove('test_cc_launch.config')

    # reboot returns ok and fires the (intercepted) reset
    fired = []
    sd3 = cc_client.create_dispatcher(config_default.default(), on_reboot=lambda: fired.append(1))
    assert await sd3.handle('reboot') == 'ok'
    await asyncio.sleep_ms(260)
    assert fired == [1]

    # probe: on-demand self-tests over the inspectable objects that implement probe() (None = healthy,
    # else an error string); objects without probe() (the _Knob above) are skipped
    class _Probeable(inspector.Inspectable):
        def __init__(self, name, result):
            self.name = name
            self._result = result

        async def probe(self):
            return self._result

    inspector.Inspector.register(_Probeable('p_good', None))
    inspector.Inspector.register(_Probeable('p_bad', 'X not found on i2c:0'))
    sd4 = cc_client.create_dispatcher(config_default.default())
    allres = json.loads(cc.parse(await sd4.handle('probe')).args[0])  # 'probe' / 'probe all'
    assert allres.get('p_good') is None and allres.get('p_bad') == 'X not found on i2c:0'
    assert 'knob' not in allres  # an inspectable without probe() is skipped
    assert json.loads(cc.parse(await sd4.handle('probe p_good')).args[0]) == {'p_good': None}
    assert 'badargs' in await sd4.handle('probe knob')  # registered, but has no probe()
    assert 'badargs' in await sd4.handle('probe nope')  # unknown object

    # `probe all` also surfaces devices that never set up (absent/miswired) from Controller.failures,
    # so one command shows the whole connected/not picture (probe checks wiring + setup)
    class _FaultyController:
        failures = {'baro_icp10111': 'setup failed (absent / miswired?)'}

    sd_fail = cc_client.create_dispatcher(config_default.default(), controller=_FaultyController())
    allres = json.loads(cc.parse(await sd_fail.handle('probe')).args[0])
    assert allres.get('p_good') is None  # an inspectable device still probed live
    assert allres.get('baro_icp10111', '').startswith('not connected: ')  # never set up -> reported

    # `verify`: dump every configured device (up/down) + probe self-tests + an overall PASS/fail verdict
    class _VerifyController:
        failures = {'baro_icp10111': 'setup failed (absent / miswired?)'}

        def directory(self):
            return ['imu_bno055', 'baro_icp10111']

        def active(self, name):
            return object() if name == 'imu_bno055' else None  # imu up, baro never set up

    sd_verify = cc_client.create_dispatcher(config_default.default(), controller=_VerifyController())
    report = json.loads(cc.parse(await sd_verify.handle('verify')).args[0])
    assert report['devices']['imu_bno055'] == 'up'
    assert report['devices']['baro_icp10111'].startswith('down: ')  # configured but not connected
    assert 'baro_icp10111' in report['problems'] and report['pass'] is False  # a problem -> not PASS
    # the flight-readiness config gate rides along: the DEFAULT config is a bench config -- watchdog
    # and flight both disabled -> not ready, each named (hardware `pass` is judged separately)
    assert report['ready'] is False
    assert report['readiness']['watchdog'].startswith('disabled') and 'flight' in report['readiness']
    assert 'unsupported' in await cc_client.create_dispatcher(config_default.default()).handle('verify')

    # a flight-ready config (watchdog + flight on, gains set, nominal fin limit, a zone source) is
    # clean; each field-dangerous knob then trips its own named flag
    ready_cfg = config_default.default()
    tasks = {task['name']: task for task in ready_cfg['components']}
    tasks['watchdog']['enabled'] = True
    tasks['flight']['enabled'] = True
    tasks['flight']['gains'] = {axis: {'kp': 1.0} for axis in ('roll', 'pitch', 'yaw')}

    class _SitedMission:
        name = 'mission'
        zone = None
        sites = [('field', ((48.001, 11.0), (48.0, 11.01)))]

    inspector.Inspector.register(_SitedMission())
    assert cc_client._readiness(ready_cfg) == {}
    tasks['flight']['gains']['yaw'] = {'kp': 0.0}  # a zero gain on one axis -> named
    assert 'yaw' in cc_client._readiness(ready_cfg)['gains']
    tasks['flight']['gains']['yaw'] = {'kp': 1.0}
    ready_cfg['fins']['limit_multiplier'] = 0.5  # a bench derating left applied
    assert '0.5' in cc_client._readiness(ready_cfg)['fins.limit_multiplier']
    _SitedMission.sites = []  # no zone AND no sites to select one from -> named
    assert 'zone' in cc_client._readiness(ready_cfg)

    # log streaming: `log <ms>` arms collection + returns the batch buffered since the last call.
    # Poll model -- the operator re-sends `log` each tick; the batch rides back as one base64 token.
    import recorder

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    sd5 = cc_client.create_dispatcher(config_default.default())
    assert json.loads(cc.parse(await sd5.handle('log 1000')).args[0])['lines'] == []  # arm, empty
    recorder.Recorder.log('test', 'hello')
    batch = json.loads(cc.parse(await sd5.handle('log 1000')).args[0])  # drain + re-arm
    assert batch['lines'][0].endswith('test :: hello'), batch
    assert json.loads(cc.parse(await sd5.handle('log 0')).args[0])['lines'] == []  # stop, drained
    assert recorder.Recorder._cc_log._deadline == 0
    assert 'badargs' in await sd5.handle('log notanumber')  # non-integer duration rejected

    # telemetry streaming: `tlm <ms>` mirrors `log` -- arms collection + returns {'samples': [...]}.
    assert json.loads(cc.parse(await sd5.handle('tlm 1000')).args[0])['samples'] == []  # arm, empty
    recorder.Recorder.tlm('t.csv', 'row')
    samples = json.loads(cc.parse(await sd5.handle('tlm 1000')).args[0])  # drain + re-arm
    assert samples['samples'][0].endswith('@row'), samples
    assert json.loads(cc.parse(await sd5.handle('tlm 0')).args[0])['samples'] == []  # stop, drained
    assert recorder.Recorder._cc_tlm._deadline == 0
    assert 'badargs' in await sd5.handle('tlm notanumber')  # non-integer duration rejected

    # arming: refused while a probe fails, clean board -> armed; disarm; manual stage hold + auto resume
    class _ArmController:
        failures = {}

        def __init__(self):
            self.armed = False
            self.manual = False
            self._stage = 'setting'

        def arm(self):
            self.armed = True

        def disarm(self):
            self.armed = False

        def stage_name(self):
            return self._stage

        def resume(self):
            self.manual = False

        def hold(self, name):
            if name not in ('setting', 'boosting', 'gliding', 'landing', 'done'):
                return False
            self._stage = name
            self.manual = True
            return True

    arm_ctrl = _ArmController()
    sd_arm = cc_client.create_dispatcher(config_default.default(), controller=arm_ctrl)
    assert 'unsafe' in await sd_arm.handle('arm') and arm_ctrl.armed is False  # p_bad probe fails -> refused

    inspector.Inspector.unregister('p_bad')  # clear the failing probes -> a clean board
    inspector.Inspector.unregister('mission')
    assert json.loads(cc.parse(await sd_arm.handle('arm')).args[0])['armed'] is True and arm_ctrl.armed is True
    assert json.loads(cc.parse(await sd_arm.handle('disarm')).args[0])['armed'] is False

    held = json.loads(cc.parse(await sd_arm.handle('stage gliding')).args[0])  # operator hold (ground test)
    assert held['stage'] == 'gliding' and held['manual'] is True
    assert 'badargs' in await sd_arm.handle('stage nope')  # unknown stage name
    assert json.loads(cc.parse(await sd_arm.handle('stage auto')).args[0])['manual'] is False  # resume
    assert 'unsupported' in await cc_client.create_dispatcher(config_default.default()).handle('arm')

    print('ok: cc_client dispatch/serve/standard + inspect/update/stats + probe + verify + log + tlm + arm')


asyncio.run(amain())
