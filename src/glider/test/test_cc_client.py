"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the CC client (cc_client.py). Board-first: the board socket sees
`command params` (no id) and replies `status params` (no id except iam). Run by `make test`.
"""

import asyncio
import json
import os
import time

import cc_client
import cc_protocol as cc
import config as config_module
import config_default
import controller
import inspector
import layout
import mission
import recorder
import task


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


_DRIVERS: dict = {'imu_bno055': 'bno055', 'baro_icp10111': 'icp10111'}  # the stub controllers' config entries
_CROSSWIRED: str = 'id reads 0xEA, expected 0xEB -- wrong device on this bus/select (crosswired)'
_NOT_CONNECTED: str = 'not connected: setup failed (absent / miswired?) -- ' + _CROSSWIRED
_UNFITTED: str = 'vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:0 0x29'
_AGL: dict = {'agl': {'priority': 0, 'timeout_ms': 100}}  # what both laser entries feed


class _Answers(task.Task):
    """A part that is soldered: setup succeeds and the probe passes."""

    async def setup(self) -> bool:
        self._ok = True
        return True


class _Absent(task.Task):
    """A part that is not there: its model-id check reads the other part, as the VL53L4CX did on the taster."""

    async def setup(self) -> bool:
        return False

    async def diagnose(self) -> str:
        return _CROSSWIRED


class _Silent(_Answers):
    """A part that set up and then fails its self-test."""

    async def probe(self) -> str:
        return 'range read timed out'


class _Crashes(_Answers):
    """A part that set up and whose run loop then dies."""

    async def run(self) -> None:
        raise RuntimeError('loop died')


def _laser_rig(l4cx: type, l1x: type, l1x_addr: int = 0x29, l1x_first: bool = False):
    """
    A REAL Controller over the two laser entries config_default declares for one socket.

    Args:
        l4cx - the task class standing in for the VL53L4CX entry (_Answers fitted, _Absent not).
        l1x - the same for the VL53L1X entry.
        l1x_addr - the VL53L1X entry's address; 0x29 is the shared socket.
        l1x_first - declare (and so set up) the VL53L1X entry BEFORE the VL53L4CX one.

    Returns:
        The Controller, not yet set up.
    """
    entries = [{'name': 'laser_agl', 'driver': 'vl53l4cx', 'bus': 'i2c', 'id': 0, 'addr': 0x29, 'enabled': True,
                'provides': _AGL},
               {'name': 'laser_agl_l1x', 'driver': 'vl53l1x', 'bus': 'i2c', 'id': 0, 'addr': l1x_addr,
                'enabled': True, 'provides': _AGL}]
    if l1x_first:
        entries.reverse()
    return controller.Controller({'board': {'id': 'taster', 'mcu': 'esp32p4'}, 'sensors': entries},
                                 registry={'vl53l4cx': l4cx, 'vl53l1x': l1x})


async def _answer(rig, command: str) -> list:
    """Run one command through a dispatcher over `rig`; returns [status, *args] with the JSON args decoded."""
    reply = cc.parse(await cc_client.create_dispatcher(config_default.default(), controller=rig).handle(command))
    decoded = []
    for arg in reply.args:
        try:
            decoded.append(json.loads(arg))
        except ValueError:
            decoded.append(arg)
    return [reply.command] + decoded


async def _alternatives() -> None:
    """
    Two parts declared for ONE socket: the one that answers wins, the other is NOT FITTED, not a fault.

    config_default declares laser_agl (vl53l4cx) and laser_agl_l1x (vl53l1x) both on i2c:0 0x29, so the
    part that is soldered wins. On the taster the VL53L4CX entry's failure landed in controller.failures
    and every consumer called it a fault: `probe all` printed FAIL, verify failed, and `arm` refused
    every board with an L1X fitted. A socket NOBODY answers on must still block arming.
    """
    inspector.Inspector.unregister('p_good')  # the earlier probe fixture: the maps below hold the rig alone
    for l1x_first in (False, True):  # the loser set up before its winner, and after it
        rig = _laser_rig(_Absent, _Answers, l1x_first=l1x_first)
        await rig.setup()
        assert rig.failures == {}, rig.failures
        assert rig.alternatives == {'laser_agl': 'laser_agl_l1x'}, rig.alternatives
        assert rig.unfitted('laser_agl') == _UNFITTED and rig.inspect()['alternatives'] == rig.alternatives
        status, results, alternatives = await _answer(rig, 'probe')
        assert status == 'ok' and results['laser_agl_l1x'] is None and 'laser_agl' not in results, results
        assert alternatives == {'laser_agl': _UNFITTED}, alternatives
        # a single probe keeps probe's contract: null in the first map (tools/cc.py exits 0, the
        # dashboard prints no FAIL), the not-fitted line in the second -- exactly as `probe all` carries it
        assert await _answer(rig, 'probe laser_agl') == ['ok', {'laser_agl': None}, {'laser_agl': _UNFITTED}]
        status, report = await _answer(rig, 'verify')
        assert report['pass'] is True and report['problems'] == {}, report['problems']
        assert report['devices'] == {'laser_agl': _UNFITTED, 'laser_agl_l1x': 'vl53l1x -- up'}, report['devices']
        assert report['alternatives'] == {'laser_agl': _UNFITTED}, report
        assert await _answer(rig, 'arm') == ['ok', {'armed': True}] and rig.armed is True
        rig.disarm()
        await rig.finish()

    # NEGATIVE: neither part answers -> both are failures, each naming its driver, and arm is refused
    rig = _laser_rig(_Absent, _Absent)
    await rig.setup()
    assert sorted(rig.failures) == ['laser_agl', 'laser_agl_l1x'] and rig.alternatives == {}, rig.alternatives
    both = {'laser_agl': 'vl53l4cx -- ' + _NOT_CONNECTED, 'laser_agl_l1x': 'vl53l1x -- ' + _NOT_CONNECTED}
    reply = await _answer(rig, 'probe')
    assert reply == ['ok', both], 'no second map when there are no alternatives: %s' % reply
    assert await _answer(rig, 'probe laser_agl') == ['ok', {'laser_agl': both['laser_agl']}]
    assert await _answer(rig, 'arm') == ['err', 'unsafe', both] and rig.armed is False
    status, report = await _answer(rig, 'verify')
    assert report['pass'] is False and report['problems'] == both and report['alternatives'] == {}, report
    assert report['devices']['laser_agl'] == ('vl53l4cx -- down: setup failed (absent / miswired?) -- '
                                              + _CROSSWIRED), report['devices']
    await rig.finish()

    # NEGATIVE: the part that answers sits on ANOTHER address -> the dead one is still a fault
    rig = _laser_rig(_Absent, _Answers, l1x_addr=0x30)
    await rig.setup()
    assert list(rig.failures) == ['laser_agl'] and rig.alternatives == {}, (rig.failures, rig.alternatives)
    assert await _answer(rig, 'arm') == ['err', 'unsafe', {'laser_agl': both['laser_agl']}]
    await rig.finish()

    # NEGATIVE: a UART device has no address, so it is never anyone's alternative
    gnss = controller.Controller(
        {'board': {'id': 'taster', 'mcu': 'esp32p4'},
         'sensors': [{'name': 'gnss', 'driver': 'atgm336h', 'bus': 'uart', 'id': 2, 'addr': None, 'enabled': True},
                     {'name': 'gnss_spare', 'driver': 'neo6mv2', 'bus': 'uart', 'id': 2, 'enabled': True}]},
        registry={'atgm336h': _Answers, 'neo6mv2': _Absent})
    await gnss.setup()
    assert list(gnss.failures) == ['gnss_spare'] and gnss.alternatives == {}, (gnss.failures, gnss.alternatives)
    assert await _answer(gnss, 'arm') == ['err', 'unsafe', {'gnss_spare': 'neo6mv2 -- ' + _NOT_CONNECTED}]
    await gnss.finish()

    # a winner that fails its PROBE is named by its driver in probe, single probe and arm alike
    rig = _laser_rig(_Absent, _Silent)
    await rig.setup()
    silent = 'vl53l1x -- range read timed out'
    status, results, alternatives = await _answer(rig, 'probe')
    assert results['laser_agl_l1x'] == silent and alternatives == {'laser_agl': _UNFITTED}, results
    assert await _answer(rig, 'probe laser_agl_l1x') == ['ok', {'laser_agl_l1x': silent}]
    assert await _answer(rig, 'arm') == ['err', 'unsafe', {'laser_agl_l1x': silent}]
    await rig.finish()

    # a winner whose run loop CRASHES becomes a failure (connected, so not 'not connected') -- probe no
    # longer calls it healthy because its probe() still answers, verify no longer calls it up because it
    # is still registered -- and the loser stays an alternative whose line no longer says it "answers"
    rig = _laser_rig(_Absent, _Crashes)
    await rig.setup()
    await rig.start()
    await asyncio.sleep_ms(50)  # let the loop run and die
    assert list(rig.failures) == ['laser_agl_l1x'] and rig.alternatives == {'laser_agl': 'laser_agl_l1x'}
    assert rig.failures['laser_agl_l1x'].startswith('run loop crashed: RuntimeError('), rig.failures
    crashed = 'vl53l1x -- ' + rig.failures['laser_agl_l1x']  # the repr differs between MicroPython and CPython
    went_down = 'vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answered i2c:0 0x29 at setup, now down'
    assert await _answer(rig, 'probe') == ['ok', {'laser_agl_l1x': crashed}, {'laser_agl': went_down}]
    status, report = await _answer(rig, 'verify')
    assert report['devices'] == {'laser_agl': went_down,
                                 'laser_agl_l1x': 'vl53l1x -- down: ' + rig.failures['laser_agl_l1x']}, report
    assert report['pass'] is False and report['problems'] == {'laser_agl_l1x': crashed}, report['problems']
    assert report['alternatives'] == {'laser_agl': went_down}, report['alternatives']
    assert await _answer(rig, 'arm') == ['err', 'unsafe', {'laser_agl_l1x': crashed}] and rig.armed is False
    await rig.finish()

    # NEGATIVE: a baro declared at the laser's 0x29 by mistake feeds OTHER data than the L1X that answers
    # there -> it is a failure, not "not fitted", and arm is refused as it was before alternatives existed
    baro = controller.Controller(
        {'board': {'id': 'taster', 'mcu': 'esp32p4'},
         'sensors': [{'name': 'laser_agl_l1x', 'driver': 'vl53l1x', 'bus': 'i2c', 'id': 0, 'addr': 0x29,
                      'enabled': True, 'provides': _AGL},
                     {'name': 'baro_icp10111', 'driver': 'icp10111', 'bus': 'i2c', 'id': 0, 'addr': 0x29,
                      'enabled': True, 'provides': {'altitude': {'priority': 0, 'timeout_ms': 200}}}]},
        registry={'vl53l1x': _Answers, 'icp10111': _Absent})
    await baro.setup()
    assert list(baro.failures) == ['baro_icp10111'] and baro.alternatives == {}, baro.alternatives
    refused = {'baro_icp10111': 'icp10111 -- ' + _NOT_CONNECTED}
    assert await _answer(baro, 'probe') == ['ok', dict(refused, laser_agl_l1x=None)]
    assert await _answer(baro, 'arm') == ['err', 'unsafe', refused] and baro.armed is False
    await baro.finish()

    # NEGATIVE: two SPI accels on spi:1 share the i2c-fallback addr 0x53 but not a chip-select -- the
    # chip-select is the socket on SPI, so the dead one is a failure, never an alternative, and arm refuses
    accel = {'accel': {'priority': 0, 'timeout_ms': 20}}
    spi = controller.Controller(
        {'board': {'id': 'taster', 'mcu': 'esp32p4'},
         'sensors': [{'name': 'accel_adxl375', 'driver': 'adxl375', 'bus': 'spi', 'id': 1, 'addr': 0x53,
                      'cs_pin': 'adxl_cs', 'enabled': True, 'provides': accel},
                     {'name': 'accel_spare', 'driver': 'adxl375_spare', 'bus': 'spi', 'id': 1, 'addr': 0x53,
                      'cs_pin': 'spare_cs', 'enabled': True, 'provides': accel}]},
        registry={'adxl375': _Answers, 'adxl375_spare': _Absent})
    await spi.setup()
    assert list(spi.failures) == ['accel_spare'] and spi.alternatives == {}, spi.alternatives
    status, report = await _answer(spi, 'verify')
    assert report['devices'] == {'accel_adxl375': 'adxl375 -- up', 'accel_spare': 'adxl375_spare -- down: '
                                 'setup failed (absent / miswired?) -- ' + _CROSSWIRED}, report['devices']
    assert report['pass'] is False and report['alternatives'] == {}, report
    assert await _answer(spi, 'arm') == ['err', 'unsafe', {'accel_spare': 'adxl375_spare -- ' + _NOT_CONNECTED}]
    await spi.finish()


_SOCKET: dict = {'fitted': 'vl53l1x', 'ranging': False}  # the one part soldered at the laser socket


class _Laser(task.Task):
    """
    A laser entry on the socket _SOCKET holds -- STATEFUL, as the real part is.

    setup() pulses XSHUT when its entry routes one, as both real drivers do before their model-id check,
    and XSHUT low reboots whichever part is soldered: it stops ranging. Only the entry whose driver
    matches the soldered part comes up, and that one starts ranging. probe() is the base one, healthy:
    both real probes read the model id alone, which a reset part still answers.
    """

    async def setup(self) -> bool:
        if self._pin_gpio('xshut_pin') is not None:
            _SOCKET['ranging'] = False  # XSHUT low: the soldered part reboots to its defaults
        if self.config.get('driver') != _SOCKET['fitted']:
            return False
        _SOCKET['ranging'] = True
        self._ok = True
        return True


async def _shared_reset() -> None:
    """
    Two laser entries that share a routed XSHUT are NEVER alternatives: the loser's setup resets the winner.

    On v0.1 (or an undecided layout) both entries keep `xshut_pin: laser_xshut` (GPIO5) and each setup
    pulses it before its model-id check. Set up after the winner, the loser rebooted the fitted laser,
    which still answered its model id and never ranged again -- and with the loser paired as "not
    fitted", `arm` passed. Refused in either order (a later setup() retries the loser and pulses it
    again), with the fix in the reason. On v1.1 the layout drops the pins: paired, and the board arms.
    """
    for revision in ('v0.1', 'v1.1'):
        board_config = config_default.default()
        layout.apply(board_config, revision)
        for fitted, winner, loser in (('vl53l4cx', 'laser_agl', 'laser_agl_l1x'),
                                      ('vl53l1x', 'laser_agl_l1x', 'laser_agl')):
            for l1x_first in (False, True):
                lasers = [dict(config_module.device(board_config, name=name))
                          for name in ('laser_agl', 'laser_agl_l1x')]
                if l1x_first:
                    lasers.reverse()
                rig = controller.Controller({'board': {'id': 'taster', 'mcu': 'esp32p4'},
                                             'pins': board_config['pins'], 'sensors': lasers},
                                            registry={'vl53l4cx': _Laser, 'vl53l1x': _Laser})
                _SOCKET.update(fitted=fitted, ranging=False)
                await rig.setup()
                case = (revision, fitted, l1x_first)
                if revision == 'v0.1':
                    # the hazard itself: a loser set up AFTER the winner left the fitted laser not ranging
                    assert _SOCKET['ranging'] is (lasers[0]['name'] == loser), case
                    reason = ('%s -- not connected: setup failed (absent / miswired?); shares XSHUT GPIO5 with '
                              '%s (%s): its setup resets the fitted laser -- enable exactly one of laser_agl, '
                              'laser_agl_l1x on this board' % (rig.driver(loser), winner, fitted))
                    assert rig.alternatives == {} and list(rig.failures) == [loser], (case, rig.alternatives)
                    assert await _answer(rig, 'arm') == ['err', 'unsafe', {loser: reason}], case
                    assert rig.armed is False, case
                else:
                    assert _SOCKET['ranging'] is True, case
                    assert rig.alternatives == {loser: winner} and rig.failures == {}, (case, rig.failures)
                    assert await _answer(rig, 'arm') == ['ok', {'armed': True}] and rig.armed is True, case
                    rig.disarm()
                await rig.finish()


async def _whoami_identity(sd):
    """
    whoami carries this boot's identity: the NVS boot id and the session prefix its files carry.

    Called with NO Mission registered, so it also covers the negative half of the clock: a board with no
    clock to report sends no `epoch`, and CC leaves such a board's time alone. Recorder.boot_id is set by
    hand and put back -- the board's real NVS boot count is never touched.
    """
    booted = recorder.Recorder.boot_id
    try:
        recorder.Recorder.boot_id = 123
        recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())  # settles the session afresh
        info = json.loads(cc.parse(await sd.handle('whoami')).args[1])
        assert info['boot_id'] == 123 and info['session'] == '000123', info
        assert 'epoch' not in info, info
        # nothing counted the boot (tests, HITL): no id, and the legacy date + random prefix
        recorder.Recorder.boot_id = None
        recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
        info = json.loads(cc.parse(await sd.handle('whoami')).args[1])
        assert info['boot_id'] is None and len(info['session']) == 22, info
    finally:
        recorder.Recorder.boot_id = booted


async def _whoami_clock(sd):
    """With a Mission registered, whoami carries the board clock (Unix UTC seconds): CC's set-on-connect cue."""
    info = json.loads(cc.parse(await sd.handle('whoami')).args[1])
    epoch = inspector.Inspector.get('mission').epoch()
    assert isinstance(info['epoch'], int) and not isinstance(info['epoch'], bool), info  # CC's gate
    assert abs(info['epoch'] - epoch) <= 2, info
    assert info['epoch'] >= 946684800, info  # never before 2000-01-01: the RTC's own floor


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
    """
    A MALFORMED LINE must answer, not raise. cc.parse() throws binascii.Error on a bad `base64:`
    token, and it used to sit outside handle()'s guard -- so the exception propagated past the read
    loop and dropped the CC link. One mistyped operator token disconnecting the board is the wrong
    failure: the link is how the operator recovers from mistakes, so it has to survive them.
    """
    assert 'badargs' in await dispatcher.handle('ping base64:!!!not-base64!!!')
    assert 'badargs' in await dispatcher.handle('ping base64:AB')  # incorrect padding
    assert await dispatcher.handle('ping') == 'pong'  # ...and the dispatcher still works afterwards

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

    """
    A hub that vanished without a FIN (host crash, power loss) sends nothing -- and a bare readline()
    waited on the dead socket forever, so the board never re-dialled. serve() must give up after
    _SILENT_MS of silence; a live hub (a line every ~2 s) never gets near it.
    """
    class _SilentReader:
        async def readline(self):
            await asyncio.sleep_ms(60000)
            return b'ping'

    real_silent = cc_client._SILENT_MS
    cc_client._SILENT_MS = 50
    try:
        started = time.ticks_ms()
        await cc_client.Client(config_default.default(), sd).serve(_SilentReader(), _FakeWriter())
        assert time.ticks_diff(time.ticks_ms(), started) < 2000, 'serve() waited on a silent hub'
    finally:
        cc_client._SILENT_MS = real_silent

    # set-config board: invalid rejected; reset-config ok; bad args rejected
    sd2 = cc_client.create_dispatcher(config_default.default(), config_path='test_cc_board.config')
    bad = config_default.default()
    bad['pins']['servo_yaw'] = 18  # reserved Wi-Fi pin -> invalid
    assert 'invalid' in await sd2.handle(cc.build('set-config', ['board', json.dumps(bad)]))
    reply = cc.parse(await sd2.handle(cc.build('set-config', ['board', json.dumps(config_default.default())])))
    assert reply.command == 'ok' and 'config_id' in json.loads(reply.args[0])
    assert cc.parse(await sd2.handle('reset-config')).command == 'ok'
    assert 'badargs' in await sd2.handle('set-config board')  # no json

    """
    `get-config saved` is what the NEXT boot runs, `get-config board` what this one does. A UI
    read-modify-write that started from the running config reverted every un-rebooted save: lower
    fins.concurrency for the bench, save the fin zeros, and concurrency 3 was back on the bench supply.
    """
    running_concurrency = config_default.default()['fins']['concurrency']
    pending = 3 if running_concurrency != 3 else 2  # any valid value the running config does not have
    changed = config_default.default()
    changed['fins']['concurrency'] = pending
    assert cc.parse(await sd2.handle(cc.build('set-config', ['board', json.dumps(changed)]))).command == 'ok'
    saved = json.loads(cc.parse(await sd2.handle('get-config saved')).args[0])
    running = json.loads(cc.parse(await sd2.handle('get-config board')).args[0])
    assert saved['fins']['concurrency'] == pending, 'saved must return the pending save'
    assert running['fins']['concurrency'] == running_concurrency, 'board must stay the RUNNING config'
    # no saved file -> what the next boot would load: the default, never an error
    assert cc.parse(await sd2.handle('reset-config')).command == 'ok'
    fallback = json.loads(cc.parse(await sd2.handle('get-config saved')).args[0])
    assert fallback['fins']['concurrency'] == running_concurrency

    """
    A board.config that fails to load boots the BENCH default -- id 'taster', watchdog off -- and that
    used to reach the serial console only. health must annunciate it and whoami must carry the source;
    a clean boot must not.
    """
    booted = config_module.BOOT_SOURCE
    try:
        config_module.BOOT_SOURCE = 'default(fallback: invalid board.config)'
        panel = json.loads(cc.parse(await sd2.handle('health')).args[0])
        assert any(item.startswith('CONFIG FALLBACK') for item in panel['degraded']), panel['degraded']
        assert 'invalid board.config' in json.loads(cc.parse(await sd2.handle('whoami')).args[1])['config_source']
        for clean in ('active', 'default'):
            config_module.BOOT_SOURCE = clean
            panel = json.loads(cc.parse(await sd2.handle('health')).args[0])
            assert not any(item.startswith('CONFIG FALLBACK') for item in panel['degraded']), (clean, panel)
    finally:
        config_module.BOOT_SOURCE = booted
    assert 'badargs' in await sd2.handle(cc.build('set-config', ['nope', '{}']))  # unknown config name
    assert 'badargs' in await sd2.handle('get-config nope')  # unknown config name

    # config 'launch' target: get-config/set-config launch are unsupported until a Mission registers
    inspector.Inspector.unregister('mission')
    assert 'unsupported' in await sd.handle('get-config launch')
    assert 'unsupported' in await sd.handle(cc.build('set-config', ['launch', '{}']))
    await _whoami_identity(sd)
    mission.Mission('test_cc_launch.config').update({'launch_id': 'cc-t1'})  # registers itself
    await _whoami_clock(sd)

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

        name = 'airspeed_sdp810'

        def calibration(self):
            return 'keep the pitot in STILL AIR -- this captures the zero tare'

        async def calibrate(self):
            return None

    class _UncalibratedImu:
        """A healthy BNO055 that simply has not been moved yet -- mag calibration still 0."""

        calibration_state = (0, 3, 3, 0)
        calibration_value = (0, 3, 3, 0)

        def calibration(self):
            """
            A METHOD returning a STRING, because that is what drivers/bno055.py:214 is.

            This stub used to be a @property returning the tuple, and that shape difference hid a real
            bug: cc_client read `imu.calibration` without calling it, so the board rendered
            "<bound_method>" into the readiness line while the test -- reading a property -- saw a
            tidy tuple and passed. A double that cannot express the failure cannot catch it.
            """
            return ('move the airframe in a slow FIGURE-8 until mag reads 3 '
                    '(now sys %d gyr %d acc %d mag %d)' % self.calibration_value)

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

    # the sweep reads REGISTERED inspectables, so the stub has to be one for the heartbeat to see it
    inspector.Inspector.register(_Pitot())
    try:
        sd_flight = cc_client.create_dispatcher(config_default.default(), controller=_FlightController())
        panel = json.loads(cc.parse(await sd_flight.handle('health')).args[0])
    finally:
        inspector.Inspector.unregister('airspeed_sdp810')
    assert panel['armed'] is True and panel['flight'] == _VITALS  # airspeed / fin cap / reach ride along
    assert 'agl' in panel  # low-altitude laser AGL rides the same heartbeat
    # degraded-mode annunciation: warm-started (from the controller flag) is surfaced
    assert 'WARM-STARTED (rebooted in flight)' in panel['degraded']
    """
    An UNCALIBRATED BNO055 must reach the operator. It is invisible to probe() (the part answers
    perfectly) and to _readiness() (it is not a config choice), yet NDOF fusion never converges without
    motion -- so a still glider reaches launch with a frozen attitude on HEALTHY hardware. Not
    reporting it is what got a working module condemned on this bench.
    """
    assert 'needs-calibration' in panel['degraded']
    assert panel['imu_calibration'] == [0, 3, 3, 0]  # (sys, gyr, acc, mag) -- mag 0 shows the progress
    # the pending sweep rides the heartbeat as {device: instruction}: the dashboard counts it for the
    # `calibrate N` button and clears the not-ready row when it empties, with no extra round trip
    assert isinstance(panel['calibration'], dict) and panel['calibration']
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
        alternatives = {}

        def stage_name(self):
            return 'setting'  # on the pad: the ground gate lets the probes run

        def active(self, name=None):
            return None  # nothing is up: the baro never set up

        def driver(self, name):
            return _DRIVERS.get(name)

    sd_fail = cc_client.create_dispatcher(config_default.default(), controller=_FaultyController())
    reply = cc.parse(await sd_fail.handle('probe'))
    allres = json.loads(reply.args[0])
    assert allres.get('p_good') is None  # an inspectable device still probed live
    # never set up -> reported, naming the DRIVER: "FAIL baro_icp10111" alone did not say which part
    assert allres['baro_icp10111'] == 'icp10111 -- not connected: setup failed (absent / miswired?)', allres
    assert len(reply.args) == 1, 'no alternatives -> no second map: %s' % reply.args

    # `verify`: dump every configured device (up/down) + probe self-tests + an overall PASS/fail verdict
    class _VerifyController:
        failures = {'baro_icp10111': 'setup failed (absent / miswired?)'}
        alternatives = {}
        stage = 1  # SETTING, as a real board on the pad reports
        manual = False

        def stage_name(self):
            return 'setting'

        def directory(self):
            return ['imu_bno055', 'baro_icp10111']

        def active(self, name):
            return object() if name == 'imu_bno055' else None  # imu up, baro never set up

        def driver(self, name):
            return _DRIVERS.get(name)

    sd_verify = cc_client.create_dispatcher(config_default.default(), controller=_VerifyController())
    report = json.loads(cc.parse(await sd_verify.handle('verify')).args[0])
    assert report['devices']['imu_bno055'] == 'bno055 -- up', report['devices']
    # configured but not connected, naming the driver
    assert report['devices']['baro_icp10111'] == 'icp10111 -- down: setup failed (absent / miswired?)'
    assert report['problems']['baro_icp10111'] == 'icp10111 -- not connected: setup failed (absent / miswired?)'
    assert report['pass'] is False and report['alternatives'] == {}  # a problem -> not PASS
    # the flight-readiness config gate rides along: the DEFAULT config is a bench config -- watchdog
    # and flight both disabled -> not ready, each named (hardware `pass` is judged separately)
    assert report['ready'] is False

    """
    The readiness line has to TELL the operator something. It shipped reading `imu.calibration` without
    calling it, so a real board rendered "BNO055 not calibrated <bound_method ...>" -- the one message
    whose entire job is to say which axes are still short said nothing, and it reached a live airframe.
    Assert the driver's text arrives and the repr does not.
    """
    class _UncalibratedVerifyController(_VerifyController):
        def active(self, name):
            return _UncalibratedImu() if name == 'imu_bno055' else None

    sd_calib = cc_client.create_dispatcher(config_default.default(),
                                           controller=_UncalibratedVerifyController())
    calib_report = json.loads(cc.parse(await sd_calib.handle('verify')).args[0])
    line = calib_report['readiness']['imu_calibration']
    assert 'bound_method' not in line and '<' not in line, 'readiness rendered a repr: %r' % line
    assert 'FIGURE-8' in line and 'mag 0' in line, 'readiness lost the instruction/counts: %r' % line
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
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    sd5 = cc_client.create_dispatcher(config_default.default())
    assert json.loads(cc.parse(await sd5.handle('log 1000')).args[0])['lines'] == []  # arm, empty
    recorder.Recorder.log('test', 'hello')
    batch = json.loads(cc.parse(await sd5.handle('log 1000')).args[0])  # drain + re-arm
    assert batch['lines'][0].endswith('test :: hello'), batch
    assert json.loads(cc.parse(await sd5.handle('log 0')).args[0])['lines'] == []  # stop, drained
    assert recorder.Recorder._cc_log._deadline == 0
    """
    The tee is best-effort -- a full ring DISCARDS rather than raising -- so a live stream with a gap
    looks exactly like a quiet sensor. The reply carries the discarded count so the hub can say the
    window is incomplete instead of the operator reading absence as calm.
    """
    assert 'dropped' in batch, batch
    assert json.loads(cc.parse(await sd5.handle('tlm 1000')).args[0])['dropped'] == 0
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
        alternatives = {}
        config = config_default.default()  # `detect` scans with it

        def driver(self, name):
            return _DRIVERS.get(name)

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

        @property
        def stage(self):
            return controller.Stage.NAMES[self._stage]  # the id a real Controller exposes

        def resume(self):
            self.manual = False

        def directory(self):
            return []  # `verify` lists the configured devices: none here

        def active(self, name=None):
            return None

        def hold(self, name):  # mirrors Controller.hold: `setting` returns to the ground, not a hold
            if name not in ('setting', 'boosting', 'gliding', 'landing', 'done'):
                return False
            self._stage = name
            self.manual = name != 'setting'
            return True

    arm_ctrl = _ArmController()
    sd_arm = cc_client.create_dispatcher(config_default.default(), controller=arm_ctrl)
    refused = cc.parse(await sd_arm.handle('arm'))
    assert refused.args[0] == 'unsafe' and arm_ctrl.armed is False  # p_bad probe fails -> refused
    # p_bad is no configured device, so its verdict has no driver to name and stays as probed
    assert json.loads(refused.args[1])['p_bad'] == 'X not found on i2c:0', refused.args

    inspector.Inspector.unregister('p_bad')  # clear the failing probes -> a clean board
    inspector.Inspector.unregister('mission')
    assert json.loads(cc.parse(await sd_arm.handle('arm')).args[0])['armed'] is True and arm_ctrl.armed is True
    assert json.loads(cc.parse(await sd_arm.handle('disarm')).args[0])['armed'] is False

    held = json.loads(cc.parse(await sd_arm.handle('stage gliding')).args[0])  # operator hold (ground test)
    assert held['stage'] == 'gliding' and held['manual'] is True
    """
    ARM MUST REFUSE A BOARD THAT IS NOT ON THE GROUND UNDER AUTOMATIC CONTROL. A held or forced stage
    suppresses the stage detectors, so the flight's core output -- the stage record -- would be wrong,
    and on 10-03 every profile flies passive, where arm and verify are the gates the operator sees.
    """
    refused = cc.parse(await sd_arm.handle('arm'))  # the problems travel base64-encoded: decode them
    assert refused.args[0] == 'unsafe' and arm_ctrl.armed is False, refused.args
    assert 'held at gliding' in json.loads(refused.args[1])['stage'], refused.args
    assert 'badargs' in await sd_arm.handle('stage nope')  # unknown stage name
    assert json.loads(cc.parse(await sd_arm.handle('stage auto')).args[0])['manual'] is False  # resume
    assert 'unsafe' in await sd_arm.handle('arm'), 'auto but still GLIDING on the pad must not arm'
    back = json.loads(cc.parse(await sd_arm.handle('stage setting')).args[0])  # back to the ground
    assert back['stage'] == 'setting' and back['manual'] is False
    assert json.loads(cc.parse(await sd_arm.handle('arm')).args[0])['armed'] is True
    assert json.loads(cc.parse(await sd_arm.handle('disarm')).args[0])['armed'] is False
    assert 'unsupported' in await cc_client.create_dispatcher(config_default.default()).handle('arm')

    """
    AIRBORNE: the CC link stays up through the flight and the hub fans `all` out to every online board,
    so an ACTIVE command must refuse rather than run -- a probe sweeps every fin to its limits, a baro
    calibrate re-zeroes elevation, a reboot is a warm-start gamble. The reads and the recovery commands
    (`stage`, `disarm`) must stay open, or the operator loses the airframe's only remote control.
    """
    class _Sweep(inspector.Inspectable):
        """Counts probe() calls -- a servo probe is a fin sweep, so in flight the count must stay 0."""

        def __init__(self):
            self.name = 'fin_sweep'
            self.sweeps = 0

        async def probe(self):
            self.sweeps += 1
            return None

        async def calibrate(self):
            self.sweeps += 1
            return None

    sweep = _Sweep()
    inspector.Inspector.register(sweep)
    rebooted = []
    flying = _ArmController()
    sd_air = cc_client.create_dispatcher(config_default.default(), controller=flying,
                                         on_reboot=lambda: rebooted.append(1))
    for stage in ('boosting', 'gliding', 'landing'):
        flying._stage, flying.manual = stage, False  # AUTO sequencing, genuinely airborne
        for command in ('probe', 'probe fin_sweep', 'detect', 'bustune i2c 0 100000', 'calibrate fin_sweep',
                        'reset-config', 'reboot'):
            reply = cc.parse(await sd_air.handle(command))
            assert reply.command == 'err' and reply.args[0] == 'unsafe', (stage, command, reply.args)
            assert 'in flight' in reply.args[1], (stage, command, reply.args)
        reply = cc.parse(await sd_air.handle(cc.build('set-config', ['launch', '{}'])))
        assert reply.command == 'err' and reply.args[0] == 'unsafe', (stage, reply.args)
        armed = cc.parse(await sd_air.handle('arm'))  # refused on the stage, WITHOUT sweeping first
        assert armed.args[0] == 'unsafe' and 'stage' in json.loads(armed.args[1]) and not flying.armed
        report = json.loads(cc.parse(await sd_air.handle('verify')).args[0])  # still reports, no sweep
        assert 'in flight' in report['problems']['probe'] and report['pass'] is False, report['problems']
        assert cc.parse(await sd_air.handle('calibrate')).command == 'ok'  # the sweep list is a read
        assert json.loads(cc.parse(await sd_air.handle('stage')).args[0])['stage'] == stage  # reads open
        assert json.loads(cc.parse(await sd_air.handle('disarm')).args[0])['armed'] is False  # recovery open
    await asyncio.sleep_ms(260)
    assert sweep.sweeps == 0 and rebooted == [], 'an airborne command ran: %d sweeps, %s' % (
        sweep.sweeps, rebooted)
    # ...and on the ground the same commands run: SETTING before a flight, DONE after it
    for stage in ('setting', 'done'):
        flying._stage = stage
        assert json.loads(cc.parse(await sd_air.handle('probe fin_sweep')).args[0]) == {'fin_sweep': None}
        assert cc.parse(await sd_air.handle('calibrate fin_sweep')).command == 'ok'
        assert cc.parse(await sd_air.handle('detect')).command == 'ok'
    assert sweep.sweeps == 4, sweep.sweeps
    assert await sd_air.handle('reboot') == 'ok'
    await asyncio.sleep_ms(260)
    assert rebooted == [1]
    inspector.Inspector.unregister('fin_sweep')

    """
    `attitude-backup` must mean a FALLBACK, not "this revision has one source".

    v0.1/v1.0 publish fused attitude from the BNO055 at p0, so running on tasks/attitude.py means
    something died. v1.1 has no p0 attitude provider at all, so the flat check marked every v1.1 board
    permanently degraded -- and an always-amber panel stops being read. Both directions are asserted
    here because the flag is worthless if it never fires and worse than worthless if it always does.
    """
    class _Ctx:
        pass

    for revision, expected in (('v1.0', True), ('v1.1', False)):
        cfg = config_default.default()
        layout.apply(cfg, revision)
        ctx = _Ctx()
        ctx.controller = _Ctx()
        ctx.controller.config = cfg
        assert cc_client._has_primary(ctx, 'attitude') is expected, (
            '%s: expected a p0 attitude provider to be %s' % (revision, expected))

    # a DISABLED primary is not a primary -- it was never going to publish
    cfg = config_default.default()
    layout.apply(cfg, 'v1.0')
    for device in cfg['sensors']:
        if device['name'] == 'imu_bno055':
            device['enabled'] = False
    ctx = _Ctx()
    ctx.controller = _Ctx()
    ctx.controller.config = cfg
    assert cc_client._has_primary(ctx, 'attitude') is False

    # an unreadable config must NOT hide a real fallback
    blind = _Ctx()
    blind.controller = None
    assert cc_client._has_primary(blind, 'attitude') is True

    await _alternatives()
    await _shared_reset()

    print('ok: cc_client dispatch/serve/standard + inspect/update/stats + probe + verify + log + tlm + arm '
          '+ active commands refused airborne + attitude-backup only where a primary exists '
          '+ an unfitted alternative is no fault + every device verdict names its driver '
          '+ a shared XSHUT refuses arming')


asyncio.run(amain())
