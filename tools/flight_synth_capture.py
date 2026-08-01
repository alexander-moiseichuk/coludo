"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate a synthetic Coludo recorder capture (a believable E/F-motor boost -> coast -> glide -> land),
in the exact wire format flight_telemetry.parse() reads. Lets the report be demoed before any real
flight, and gives the parser test realistic data.

`python3 flight_synth_capture.py` prints a capture to stdout; pipe it into flight_report.py.
"""

import math

_SESSION = '20260621_120000_500'  # YYYYMMDD_HHMMSS_<rand>, matching recorder.session()
_GROUND_M = 520.0  # launch-site elevation (m AMSL)
_LAT0, _LON0 = 48.1173, 11.5167


def generate() -> str:
    """A ~16 s flight at 20 Hz: ~2 s boost (accel spike), apogee ~120 m, glide back to ground."""
    lines = []

    def tlm(file, row):
        lines.append('@%s_%s@%s' % (_SESSION, file, row))

    # the real telemetry file + field names (recorder writes '<component-name>.csv')
    tlm('accel_adxl375.csv', 'uptime;ax;ay;az')
    tlm('baro_icp10111.csv', 'uptime;altitude;temperature;pressure;elevation')
    tlm('imu_bno055.csv', 'uptime;heading;roll;pitch')
    tlm('gnss.csv', 'uptime;lat;lon;speed_kn;course')
    tlm('laser_agl.csv', 'uptime;agl')
    # the fixture must look like a REAL capture -- it is what the tool tests and the demo render
    # against, so a stream the tools depend on must be present here too (findings §27.1/§27.8)
    tlm('fins.csv', 'uptime;eleron_left;eleron_right;yaw')
    tlm('flight.csv', 'uptime;stage;active;airspeed_cms;fin_cap;roll_sp;pitch_sp;heading_err;'
                      'roll_cmd;pitch_cmd;yaw_cmd;wind_cms;wind_from')
    tlm('airspeed_sdp810.csv', 'uptime;dynamic_pressure;airspeed_cms;temperature')

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
        heading, pitch, roll = (t * 20.0) % 360.0, 10.0 * math.sin(t), 5.0 * math.cos(t / 2.0)

        tlm('accel_adxl375.csv', '%u;%.3f;%.3f;%.3f' % (microseconds, 0.1 * math.sin(t), 0.1 * math.cos(t), az))
        tlm('baro_icp10111.csv', '%u;%.2f;21.0;%.0f;%.2f' % (microseconds, altitude, 100000.0, elevation))
        # roll/pitch: RAW centidegree fixnum, matching drivers/bno055.py
        tlm('imu_bno055.csv', '%u;%.1f;%d;%d'
            % (microseconds, heading, round(roll * 100), round(pitch * 100)))
        if int(t / 0.1) != int((t - step) / 0.1):  # GNSS ~10 Hz
            tlm('gnss.csv', '%u;%.6f;%.6f;0.0;0.0' % (microseconds, latitude, longitude))
        if elevation < 4.0:  # laser only resolves the last few metres
            tlm('laser_agl.csv', '%u;%.3f' % (microseconds, elevation))
        gliding = t >= 2.0
        left = 90 + int(6.0 * math.sin(t * 1.3)) if gliding else 90
        right = 180 - left if gliding else 90
        rudder = 90 + int(4.0 * math.cos(t * 0.9)) if gliding else 90
        tlm('fins.csv', '%u;%d;%d;%d' % (microseconds, left, right, rudder))
        airspeed = 14.0 + 1.5 * math.sin(t / 2.0) if gliding else 0.0
        tlm('airspeed_sdp810.csv', '%u;%d;%d;2500'
            % (microseconds, int(0.5 * 1.225 * airspeed * airspeed * 100), int(airspeed * 100)))
        cap = 45 if airspeed < 12.0 else max(8, int(45.0 * (12.0 / airspeed) ** 2))
        tlm('flight.csv', '%u;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;0;0'
            % (microseconds, 3 if gliding else 2, 1 if gliding else 0, int(airspeed * 100), cap,
               int(100 * 5.0 * math.cos(t / 2.0)), -600, int(3.0 * math.sin(t)),
               left - 90, 0, rudder - 90))
        t += step

    lines.append('2000000 separation :: separated -> gliding')
    lines.append('2000100 controller :: stage -> gliding')
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    print(generate(), end='')
