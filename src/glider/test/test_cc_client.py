# On-board (MicroPython) test for the CC client (cc_client.py). Board-first: the board socket
# sees `command params` (no id) and replies `status params` (no id except iam). Run by `make test`.

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

    m = cc.parse(await sd.handle('whoami'))
    assert m.command == 'iam' and m.args[0] == 'glider1'
    info = json.loads(m.args[1])
    assert info['mcu'] == 'esp32p4' and 'config_id' in info and info['stage'] == 'setting'

    assert cc.parse(await sd.handle('ping')).command == 'pong'
    health = json.loads(cc.parse(await sd.handle('health')).args[0])
    assert 'mem_free' in health and 'uptime' in health
    cfg = json.loads(cc.parse(await sd.handle('get-config')).args[0])
    assert cfg['board']['id'] == 'glider1'

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

    # save-config: invalid rejected; reset-config ok
    sd2 = cc_client.create_dispatcher(config_default.default(), config_path='test_cc_board.json')
    bad = config_default.default()
    bad['pins']['servo_yaw'] = 18  # reserved Wi-Fi pin -> invalid
    assert 'invalid' in await sd2.handle(cc.build('save-config', [json.dumps(bad)]))
    ok = cc.parse(await sd2.handle(cc.build('save-config', [json.dumps(config_default.default())])))
    assert ok.command == 'ok' and 'config_id' in json.loads(ok.args[0])
    assert cc.parse(await sd2.handle('reset-config')).command == 'ok'

    # save-mission: unsupported until a Mission is registered, then persists launch.config
    inspector.Inspector.unregister('mission')
    assert 'unsupported' in await sd.handle('save-mission')
    mission.Mission('test_cc_launch.config').update({'launch_id': 'cc-t1'})  # registers itself
    assert cc.parse(await sd.handle('save-mission')).command == 'ok'
    assert json.load(open('test_cc_launch.config'))['launch_id'] == 'cc-t1'
    os.remove('test_cc_launch.config')

    # reboot returns ok and fires the (intercepted) reset
    fired = []
    sd3 = cc_client.create_dispatcher(config_default.default(), on_reboot=lambda: fired.append(1))
    assert await sd3.handle('reboot') == 'ok'
    await asyncio.sleep_ms(260)
    assert fired == [1]

    print('ok: cc_client board-first dispatch/serve/standard + inspect/update/stats')


asyncio.run(amain())
