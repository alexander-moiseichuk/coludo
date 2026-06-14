# On-board (MicroPython) test for the Recorder + PSRAM ring + Telemetry (recorder.py).
# Run by `make test`. Raises (-> runner reports FAIL) on any failed assertion.

import asyncio

from recorder import Ring, Recorder, Telemetry
from config_default import default


class Collector:
    def __init__(self):
        self.items = []

    def write(self, b):
        self.items.append(bytes(b))


def main():
    # Ring: write / read (read returns a copy and advances)
    r = Ring(4, 32)
    assert r.write(b'hello') and r.count == 1
    assert r.read() == b'hello' and r.count == 0
    assert r.read() is None

    # too-big -> dropped; overflow -> drop oldest
    assert r.write(b'x' * 31) is False and r.dropped == 1
    r = Ring(2, 16)
    r.write(b'a'); r.write(b'b'); r.write(b'c')
    assert r.count == 2 and r.dropped == 1 and r.read() == b'b'

    # Recorder setup from config
    Recorder.setup(default())
    assert len(Recorder.session()) == 15            # YYYYMMDD_HHMMSS
    assert Recorder.uptime_us() >= 0

    # the session prefix is fixed for the boot -> stable file names across streams
    sess = Recorder.session()
    Recorder.log('Controller', 'setup started')
    Recorder.tlm('cpu.csv', '40;51')
    sink = Collector()
    n = Recorder.drain(sink)
    assert n == 2
    # telemetry first, with @<session>_file@ routing
    assert sink.items[0] == ('@%s_cpu.csv@40;51\n' % sess).encode(), sink.items[0]
    # logs second, "<uptime_us> Controller :: setup started"
    assert b' Controller :: setup started\n' in sink.items[1]
    assert Recorder.drain(Collector()) == 0

    # uart-before-subscriber ordering
    trace = []

    class Sink:
        def write(self, b):
            trace.append(('uart', bytes(b)))

    Recorder.tlm('x.csv', '1')
    Recorder.drain(Sink(), also=lambda b: trace.append(('cc', bytes(b))))
    assert trace[0][0] == 'uart' and trace[1][0] == 'cc' and trace[0][1] == trace[1][1]

    # Telemetry: header first (uptime + fields), then timestamped rows
    Recorder.drain(Collector())                     # clear
    t = Telemetry('imu.csv', ('yaw', 'pitch', 'roll'))
    t.push((1.0, 2.0, 3.0))
    t.push((4, 5, 6))
    out = Collector()
    Recorder.drain(out)
    assert out.items[0] == ('@%s_imu.csv@uptime;yaw;pitch;roll\n' % sess).encode(), out.items[0]
    assert out.items[1].startswith(('@%s_imu.csv@' % sess).encode())
    assert out.items[1].endswith(b';1.0;2.0;3.0\n')
    assert out.items[2].endswith(b';4;5;6\n')

    # set_clock back-dates the session to boot time (and keeps it 15 chars)
    Recorder.set_clock(800000000)
    assert len(Recorder.session()) == 15

    # async drain loop drains queued records then stops
    async def amain():
        Recorder.log('X', 'y')
        s = Collector()
        stop = [False]
        task = asyncio.create_task(Recorder.run(sink=s, stop=stop))
        await asyncio.sleep_ms(120)
        stop[0] = True
        await task
        assert len(s.items) >= 1
    asyncio.run(amain())

    print('ok: recorder ring/log/tlm/priority/session-prefix, Telemetry header+rows, async run')


main()
