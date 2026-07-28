"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) END-TO-END test of the chain a flight actually travels:
BOARD telemetry -> the real cc_protocol over a real socket -> the CC hub's stream poller -> a recorder
capture -> flight_telemetry -> the analysis tools.

findings §27.12: every leg of this was tested in isolation and none of it together, so a shape or
protocol change could pass every unit test and still produce a capture nothing could render -- which is
exactly the §27.1 failure. The board leg is a fake that speaks the REAL protocol and serves REAL
recorder-format rows (including the board-only per-servo streams); everything downstream is the actual
production code. What CANNOT be scripted -- real drivers, a physical Luckfox, servos -- is the walk test
in doc/field_test.md; this covers the software chain around it. Run by `make test` / `make check`.
"""

import asyncio
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
sys.path.insert(0, os.path.join(_HERE, '..', '..', '..', 'tools'))
import cc_protocol as cc  # noqa: E402
import flight_kpi  # noqa: E402
import flight_telemetry  # noqa: E402
import server  # noqa: E402

BOARD_PORT, OPERATOR_PORT, WEB_PORT = 18301, 18302, 18303
_SESSION = '20260726_090000_1'

# What the board would record for a short flight -- in the BOARD's shapes, including the per-servo
# streams a real board writes instead of the sim's fused fins.csv (findings §27.1).
_HEADERS = [
    'servo_eleron_left.csv;uptime;angle;pulse_us;done',
    'servo_eleron_right.csv;uptime;angle;pulse_us;done',
    'servo_yaw.csv;uptime;angle;pulse_us;done',
    'flight.csv;uptime;stage;active;airspeed_cms;fin_cap;roll_sp;pitch_sp;heading_err;'
    'roll_cmd;pitch_cmd;yaw_cmd;wind_cms;wind_from',
    'gnss.csv;uptime;lat;lon;speed_kn;course',
]


def _board_samples() -> list:
    """The recorder-format rows the board would hand over, newest last."""
    rows = []
    for header in _HEADERS:
        name, _, fields = header.partition(';')
        rows.append('@%s_%s@%s' % (_SESSION, name, fields))
    for step in range(12):
        stamp = 1_000_000 + step * 100_000
        rows.append('@%s_servo_eleron_left.csv@%d;%d;1500;1' % (_SESSION, stamp, 90 + step))
        if step % 3 == 0:  # the right elevon moves less often -- forward-fill must cover the gaps
            rows.append('@%s_servo_eleron_right.csv@%d;%d;1500;1' % (_SESSION, stamp, 90 - step))
        rows.append('@%s_servo_yaw.csv@%d;90;1500;1' % (_SESSION, stamp))
        rows.append('@%s_flight.csv@%d;3;1;14%02d;%d;0;-600;2;%d;0;0;0;0'
                    % (_SESSION, stamp, step, 45 - step, step))
        rows.append('@%s_gnss.csv@%d;25.5146%02d;-80.3920;27.2;30.0' % (_SESSION, stamp, step))
    return rows


async def _fake_board(reader, writer):
    """A board that speaks the real protocol and serves its telemetry through the poll model."""
    pending = list(_board_samples())
    while True:
        line = await reader.readline()
        if not line:
            return
        message = cc.parse(line.decode().strip())
        if message.command == 'whoami':
            info = {'mcu': 'esp32p4', 'firmware_version': 'e2e01', 'stage': 'gliding',
                    'config_id': 'cfg1'}
            reply = cc.build('iam', ['e2eboard', json.dumps(info)])
        elif message.command == 'ping':
            reply = cc.build('pong')
        elif message.command == 'health':
            reply = cc.build('ok', [json.dumps({'stage': 'gliding', 'armed': True, 'agl': 12.0,
                                                'flight': {'airspeed': 14.2, 'fin_cap': 30,
                                                           'active': True}})])
        elif message.command == 'tlm':  # the poll model: hand over everything buffered since last time
            window = int(message.args[0]) if message.args else 0
            batch = []
            if window > 0:  # copy THEN clear -- aliasing the same list would hand back an empty batch
                batch = list(pending)
                pending.clear()
            reply = cc.build('ok', [json.dumps({'samples': batch})])
        elif message.command == 'log':
            reply = cc.build('ok', [json.dumps({'lines': []})])
        else:
            reply = cc.build('ok', [])
        writer.write((reply + '\n').encode())
        await writer.drain()


async def _chain() -> str:
    """Run the hub with a board attached, collect the streamed telemetry, return it as a capture."""
    hub = server.Server(host='127.0.0.1', port=BOARD_PORT, operator_port=OPERATOR_PORT,
                        web_port=WEB_PORT, log=lambda *a: None)
    listener = await asyncio.start_server(hub._handle, '127.0.0.1', BOARD_PORT)
    board = await asyncio.start_server(_fake_board, '127.0.0.1', BOARD_PORT + 50)

    collected = []
    hub._emit_log = lambda board_id, line: collected.append(line)  # the hub emits BOTH kinds here
    reader, writer = await asyncio.open_connection('127.0.0.1', BOARD_PORT + 50)

    # drive the hub's own client object over the live socket pair, exactly as an accepted board is driven
    import board as board_mod
    client = board_mod.Board(reader, writer)
    assert await client.identify() is not None, 'the handshake must succeed over the real protocol'
    assert client.id == 'e2eboard'
    hub.boards[client.id] = client

    hub.start_stream(client, 60, kind='tlm')
    for _ in range(40):  # let the poller drain the board's buffer
        await asyncio.sleep(0.03)
        if len(collected) >= len(_board_samples()):
            break
    await hub.stop_stream(client.id)
    writer.close()
    listener.close()
    board.close()
    return '\n'.join(collected)


def test_board_to_capture_to_analysis():
    """
    The whole chain: a board's telemetry survives the protocol and renders.

    Asserts on what the ANALYSIS sees, not on bytes -- the failure this guards against is a capture that
    transports perfectly and then renders as nothing.
    """
    capture = asyncio.run(_chain())
    assert capture, 'the hub must have streamed the board telemetry'
    streams, _logs = flight_telemetry.parse(capture)

    # the board's per-servo streams must rebuild into the fused shape every renderer wants (§27.1)
    fins = flight_telemetry.find_stream(streams, 'eleron_left', 'eleron_right', 'yaw')
    assert fins is not None, 'per-servo streams did not rebuild: %s' % sorted(streams)
    assert fins.column('eleron_left')[1][-1] == 101.0  # 90 + 11, the last commanded angle
    assert fins.column('eleron_right')[1][-1] == 81.0  # held between its sparser updates (forward-fill)

    # the control state survives, so the airspeed/authority panels have their data (§27.2)
    control = flight_telemetry.find_stream(streams, 'fin_cap')
    assert control is not None and len(control.rows) == 12
    caps = control.column('fin_cap')[1]
    assert caps[0] == 45 and caps[-1] == 34, caps[:3]  # the governor's schedule came through intact

    # and the analysis layer produces real numbers from it end to end
    rows, span = flight_kpi._fin_activity(fins)
    assert span > 0 and len(rows) == 3
    named = {row[0]: row for row in rows}  # (fin, samples, moves, travel, max_step)
    assert named['eleron_left'][2] > 0, 'the left elevon moved every step -- KPI must see the activity'
    assert named['yaw'][2] == 0, 'the yaw fin held all flight -- KPI must report no moves, not noise'
    miss, _inside = flight_kpi._touchdown(flight_telemetry.find_stream(streams, 'lat', 'lon'),
                                          ((25.514944, -80.392972), (25.514583, -80.391111)))
    assert miss > 0.0


test_board_to_capture_to_analysis()
print('ok: e2e -- board telemetry -> cc_protocol over a socket -> hub stream -> capture -> '
      'per-servo rebuild -> KPI numbers')
