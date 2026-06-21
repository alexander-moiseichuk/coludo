# flight_synth_capture.py — generate a synthetic Coludo recorder capture (a believable E/F-motor boost
# -> coast -> glide -> land), in the exact wire format flight_telemetry.parse() reads. Lets the report
# be demoed before any real flight, and gives the parser test realistic data.
# `python3 flight_synth_capture.py` prints a capture to stdout; pipe it into flight_report.py.

import math

_SESSION = '20260621_120000_500'  # YYYYMMDD_HHMMSS_<rand>, matching recorder.session()
_GROUND_M = 520.0  # launch-site elevation (m AMSL)
_LAT0, _LON0 = 48.1173, 11.5167


def generate() -> str:
    """A ~16 s flight at 20 Hz: ~2 s boost (accel spike), apogee ~120 m, glide back to ground."""
    lines = []

    def tlm(file, row):
        lines.append('@%s_%s@%s' % (_SESSION, file, row))

    tlm('accel.csv', 'uptime;ax;ay;az')
    tlm('baro_icp10111.csv', 'uptime;altitude;temperature;pressure;elevation')
    tlm('imu_bno055.csv', 'uptime;yaw;pitch;roll')
    tlm('atgm336h.csv', 'uptime;lat;lon;speed_kn;course')
    tlm('vl53l4cx.csv', 'uptime;agl')

    step, t = 0.05, 0.0
    while t < 16.0:
        microseconds = int(t * 1e6)
        if t < 2.0:  # boost: az climbs to ~8 g, altitude accelerates up
            az = 1.0 + 7.0 * math.sin(math.pi * t / 2.0)
            elevation = 30.0 * t * t
        elif t < 4.0:  # coast to apogee ~120 m
            az = 1.0
            elevation = 120.0 - 10.0 * (t - 3.5) ** 2
        else:  # glide down to ground by ~14 s
            az = 1.0
            elevation = max(0.0, 120.0 - 12.0 * (t - 4.0))
        altitude = _GROUND_M + elevation
        latitude = _LAT0 + 0.00010 * math.sin(t / 3.0)  # a gentle ground-track arc
        longitude = _LON0 + 0.00010 * (t / 16.0)
        yaw, pitch, roll = (t * 20.0) % 360.0, 10.0 * math.sin(t), 5.0 * math.cos(t / 2.0)

        tlm('accel.csv', '%u;%.3f;%.3f;%.3f' % (microseconds, 0.1 * math.sin(t), 0.1 * math.cos(t), az))
        tlm('baro_icp10111.csv', '%u;%.2f;21.0;%.0f;%.2f' % (microseconds, altitude, 100000.0, elevation))
        tlm('imu_bno055.csv', '%u;%.1f;%.1f;%.1f' % (microseconds, yaw, pitch, roll))
        if int(t / 0.1) != int((t - step) / 0.1):  # GNSS ~10 Hz
            tlm('atgm336h.csv', '%u;%.6f;%.6f;0.0;0.0' % (microseconds, latitude, longitude))
        if elevation < 4.0:  # laser only resolves the last few metres
            tlm('vl53l4cx.csv', '%u;%.3f' % (microseconds, elevation))
        t += step

    lines.append('2000000 separation :: separated -> gliding')
    lines.append('2000100 controller :: stage -> gliding')
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    print(generate(), end='')
