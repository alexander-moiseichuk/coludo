# Host (CPython) test for the flight telemetry parser (tools/flight_telemetry.py): demux a recorder
# capture into streams + logs, against both an explicit fixture and the synthetic flight. Stdlib only
# (no plotly). Run by `make test`.

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import flight_synth_capture  # noqa: E402
import flight_telemetry  # noqa: E402

# new session shape YYYYMMDD_HHMMSS_<rand>_<file> -- the parser strips it to the bare file name
FIXTURE = '\n'.join([
    '@20260609_101715_500_accel.csv@uptime;ax;ay;az',
    '@20260609_101715_500_accel.csv@1000000;0.0;0.0;1.0',
    '@20260609_101715_500_accel.csv@1010000;0.1;-0.2;1.0',
    '@20260609_101715_500_atgm336h.csv@uptime;lat;lon;speed_kn;course',
    '@20260609_101715_500_atgm336h.csv@1000000;48.1173;11.5167;0.0;0.0',
    '161221274 health :: probe: vitals ok (mem_free 31480912, temp 35)',
    '161300000 controller :: stage -> gliding',
])


def test_fixture():
    streams, logs = flight_telemetry.parse(FIXTURE)
    assert set(streams) == {'accel.csv', 'atgm336h.csv'}, list(streams)  # session prefix stripped

    accel = streams['accel.csv']
    assert accel.fields == ['ax', 'ay', 'az']  # header row consumed, not data
    assert len(accel.rows) == 2 and accel.rows[0][0] == 1000000  # uptime is integer us
    times, az = accel.column('az')
    assert az == [1.0, 1.0] and abs(times[0] - 1.0) < 1e-9  # us -> seconds

    assert streams['atgm336h.csv'].column('lat')[1][0] == 48.1173  # GNSS numeric parse
    assert flight_telemetry.parse('@20260609_101715_x.csv@1;notanumber')[0]['x.csv'].rows[0][1] == 'notanumber'
    # a non-numeric UPTIME drops the whole row (else column() would divide a str by 1e6 -> TypeError)
    gated = flight_telemetry.parse('@20260609_101715_x.csv@uptime;v\n'
                                   '@20260609_101715_x.csv@badtime;5\n'
                                   '@20260609_101715_x.csv@2000000;7')[0]['x.csv']
    assert len(gated.rows) == 1 and gated.rows[0][0] == 2000000  # bad-uptime row skipped, good one kept

    assert logs[0] == (161221274, FIXTURE.splitlines()[5])  # log line with its ticks_us
    assert any('stage -> gliding' in line for _ts, line in logs)


def test_synthetic_flight():
    streams, logs = flight_telemetry.parse(flight_synth_capture.generate())
    assert {'accel_adxl375.csv', 'baro_icp10111.csv', 'imu_bno055.csv', 'gnss.csv', 'laser_agl.csv'} <= set(streams)
    assert streams['imu_bno055.csv'].fields == ['heading', 'roll', 'pitch']  # real BNO055 field names
    _times, az = streams['accel_adxl375.csv'].column('az')
    assert max(az) > 5.0  # the boost spike is present
    _times, elevation = streams['baro_icp10111.csv'].column('elevation')
    assert max(elevation) > 100.0 and min(elevation) <= 0.0  # climb to apogee, back to ground
    assert len(streams['gnss.csv'].rows) < len(streams['accel_adxl375.csv'].rows)  # GNSS slower than accel
    assert any('stage -> gliding' in line for _ts, line in logs)


test_fixture()
test_synthetic_flight()
print('ok: flight telemetry parser — streams + logs from fixture and synthetic flight')
