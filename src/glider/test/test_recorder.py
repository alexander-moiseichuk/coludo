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
    # Ring: SPSC write / read (read returns a copy and advances); holds slots-1 records
    r = Ring(3, 32)
    assert r.write(b'a') and r.write(b'b') and r.count() == 2
    assert r.write(b'c') is False and r.dropped == 1        # full -> skip, no overwrite
    assert r.read() == b'a' and r.read() == b'b'
    assert r.read() is None and r.count() == 0

    # too-big record is skipped
    assert r.write(b'x' * 31) is False and r.dropped == 2

    # Recorder setup (inject a dummy sink so no real UART is opened in the unit test)
    Recorder.setup(default(), uart=Collector())
    assert Recorder._session is None                        # lazy: not produced until first tlm
    assert len(Recorder.session()) == 15                    # YYYYMMDD_HHMMSS
    sess = Recorder.session()

    # log + telemetry, telemetry first, with @<session>_file@ routing
    Recorder.log('Controller', 'setup started')
    Recorder.tlm('cpu.csv', '40;51')
    sink = Collector()
    assert Recorder.drain(sink) == 2
    assert sink.items[0] == ('@%s_cpu.csv@40;51\n' % sess).encode(), sink.items[0]
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
    Recorder.drain(Collector())                             # clear
    t = Telemetry('imu.csv', ('yaw', 'pitch', 'roll'))
    t.push((1.0, 2.0, 3.0))
    t.push((4, 5, 6))
    out = Collector()
    Recorder.drain(out)
    assert out.items[0] == ('@%s_imu.csv@uptime;yaw;pitch;roll\n' % sess).encode(), out.items[0]
    assert out.items[1].startswith(('@%s_imu.csv@' % sess).encode())
    assert out.items[1].endswith(b';1.0;2.0;3.0\n')
    assert out.items[2].endswith(b';4;5;6\n')

    # drain tracks high-water marks; report exposes count()/max/dropped
    rep = Recorder.report()
    assert rep['session'] == sess
    assert rep['tlm']['max'] >= 1 and 'count' in rep['tlm'] and 'dropped' in rep['log']
    assert 'max=' in Recorder._stats()

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

    print('ok: recorder SPSC ring/log/tlm/priority/session, Telemetry header+rows, async run')


main()
