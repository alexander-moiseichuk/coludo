# Integration test: a real wifi exchange between Control (this host) and the board.
#
# Control runs a TCP server; the board (over wifi) dials in and serves. Control does the
# board-first handshake (whoami -> iam) and then ping + `inspect wifi`, and verifies the board
# reports itself connected with an IP. Needs the board on USB (mpremote) and on the `panda`
# network (panda.creds deployed); run `src/glider/deploy.sh` first so modules + creds are on board.
#
#   python3 src/control/itest_wifi.py          (PORT_DEV env overrides the serial port)

import asyncio
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # src/control (control.py + cc_protocol symlink)
import control  # noqa: E402

PORT_DEV = os.environ.get('PORT_DEV', '/dev/ttyACM0')
BOARD_SCRIPT = '/tmp/coludo_board_probe.py'

# Runs on the board: bring the Wi-Fi task up, dial the gateway (= Control host) and serve the
# protocol. Uses the real wifi task (tasks/wifi.py) but dials the gateway directly so the test
# works on any network regardless of the configured cc_host.
BOARD_SRC = """
import asyncio
import cc_client
import config
from tasks import wifi

class _Stub:
    pass

async def main():
    cfg, source, errs = config.load()
    stub = _Stub(); stub.config = cfg
    radio = wifi.Wifi('wifi', {}, stub)
    if not await radio.setup() or not await radio.connect():
        print('WIFI_FAIL')
        return
    gateway = radio.ifconfig()[2]
    print('WIFI_OK ip=%s gw=%s' % (radio.ip(), gateway))
    dispatcher = cc_client.create_dispatcher(cfg)
    reader, writer = await asyncio.open_connection(gateway, 1234)
    print('DIALED %s:1234' % gateway)
    await cc_client.Client(cfg, dispatcher).serve(reader, writer)
    print('SERVE_DONE')

asyncio.run(main())
"""

RESULT = {}


async def main():
    with open(BOARD_SCRIPT, 'w') as script:
        script.write(BOARD_SRC)

    done = asyncio.Event()

    async def on_board(board):
        try:
            pong = await board.command('ping')
            RESULT['pong'] = pong.command if pong else None
            RESULT['wifi'] = await board.inspect('wifi')
            objects = await board.command('objects')
            RESULT['objects'] = objects.args[0] if objects else None
        finally:
            done.set()

    server = control.Server(port=1234, on_board=on_board)
    server_task = asyncio.create_task(server.serve_forever())
    await asyncio.sleep(0.3)

    print('launching board over %s ...' % PORT_DEV)
    proc = subprocess.Popen(
        ['mpremote', 'connect', PORT_DEV, 'run', BOARD_SCRIPT],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    board_out = ''
    try:
        await asyncio.wait_for(done.wait(), timeout=45)
    except asyncio.TimeoutError:
        print('TIMEOUT waiting for the board to connect')
    finally:
        await asyncio.sleep(0.3)
        proc.terminate()
        try:
            board_out = proc.communicate(timeout=5)[0]
        except Exception:
            pass
        server_task.cancel()

    print('--- board output ---\n%s' % board_out)
    print('--- exchange result ---\n%s' % RESULT)
    ok = RESULT.get('pong') == 'pong' and RESULT.get('wifi', {}).get('connected') is True
    print('WIFI EXCHANGE %s' % ('PASS' if ok else 'FAIL'))
    return 0 if ok else 1


sys.exit(asyncio.run(main()))
