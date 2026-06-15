# On-board (MicroPython) test for the CC client (cc_client.py): dispatch logic + serve loop
# with fake streams + the standard command handlers. Run by `make test`.

import asyncio
import json

import cc_protocol as cc
from cc_client import Dispatcher, Client, standard_dispatcher
from config_default import default


class FakeReader:
    def __init__(self, lines):
        self.q = [l if isinstance(l, bytes) else (l + '\n').encode() for l in lines]
        self.i = 0

    async def readline(self):
        if self.i < len(self.q):
            v = self.q[self.i]
            self.i += 1
            return v
        return b''


class FakeWriter:
    def __init__(self):
        self.out = []

    def write(self, b):
        self.out.append(b)

    async def drain(self):
        pass


async def amain():
    # --- generic Dispatcher ---------------------------------------------
    d = Dispatcher('glider1')

    async def ping_h(msg):
        return cc.build('pong', ['glider1'])
    d.on('ping', ping_h)

    assert await d.handle('ping glider1') == 'pong glider1'
    assert 'badcmd' in await d.handle('nope glider1')            # unknown command
    assert 'badboard' in await d.handle('ping glider2')          # wrong board id

    async def boom(msg):
        raise ValueError('x')
    d.on('boom', boom)
    assert 'internal' in await d.handle('boom glider1')          # handler exception
    assert await d.handle('   ') is None                         # empty line

    # --- Client.serve over fake streams ---------------------------------
    sd = standard_dispatcher(default())
    client = Client(default(), sd)
    r = FakeReader(['whoami', 'ping glider1'])
    w = FakeWriter()
    await client.serve(r, w)
    resp = [x.decode().strip() for x in w.out]
    assert cc.parse(resp[0]).command == 'iam'
    assert cc.parse(resp[1]).command == 'pong'

    # --- standard handlers ----------------------------------------------
    m = cc.parse(await sd.handle('whoami'))
    info = json.loads(m.params[0])
    assert m.command == 'iam' and m.board == 'glider1'
    assert info['mcu'] == 'esp32p4' and 'config_id' in info and info['state'] == 'setting'

    h = json.loads(cc.parse(await sd.handle('health glider1')).params[0])
    assert 'mem_free' in h and 'uptime' in h

    g = json.loads(cc.parse(await sd.handle('get-config glider1')).params[0])
    assert g['board']['id'] == 'glider1'

    # save-config: invalid rejected, valid persisted; reset-config removes it
    sd2 = standard_dispatcher(default(), config_path='test_cc_board.json')
    bad = default(); bad['pins']['servo_yaw'] = 18              # reserved Wi-Fi pin -> invalid
    r = await sd2.handle(cc.build('save-config', ['glider1', json.dumps(bad)]))
    assert 'invalid' in r, r
    r = await sd2.handle(cc.build('save-config', ['glider1', json.dumps(default())]))
    assert cc.parse(r).command == 'ok' and 'config_id' in json.loads(cc.parse(r).params[0])
    assert cc.parse(await sd2.handle('reset-config glider1')).command == 'ok'

    # reboot: returns ok and fires the (intercepted) reset after a short delay
    fired = []
    sd3 = standard_dispatcher(default(), on_reboot=lambda: fired.append(1))
    assert await sd3.handle('reboot glider1') == 'ok glider1'
    await asyncio.sleep_ms(260)
    assert fired == [1]

    print('ok: cc_client dispatcher/serve/standard handlers (whoami/ping/health/config/reboot)')


asyncio.run(amain())
