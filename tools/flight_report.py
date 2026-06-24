# flight_report.py — render a Coludo flight capture as one self-contained interactive HTML (plotly):
# a 3D trajectory (GNSS ground-track + baro altitude) plus linked time-series (accel magnitude,
# altitude/elevation, attitude, agl) with stage/separation events marked. Streams are matched by their
# field names, not file names, so it survives config renames.
#
#   pip install plotly
#   python3 synth_capture.py > demo.txt && python3 flight_report.py demo.txt -o demo.html
#   python3 flight_report.py <luckfox-capture> -o flight.html

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402


def _require_plotly():
    try:
        import plotly.graph_objects as go
        import plotly.io as pio
        from plotly.subplots import make_subplots
    except ImportError:
        sys.exit('flight_report needs plotly:  pip install plotly')
    return go, pio, make_subplots


def find_stream(streams, *fields, prefer=None):
    """The stream carrying all the given fields; when several match, one whose name contains `prefer`
    wins (e.g. the dedicated ADXL high-g accel over the IMU's low-g accel). None if none match."""
    matches = [stream for stream in streams.values() if all(field in stream.fields for field in fields)]
    if prefer:
        for stream in matches:
            if prefer in stream.name:
                return stream
    return matches[0] if matches else None


def stage_events(logs):
    """(time_s, label) for each stage transition logged (e.g. separation -> gliding)."""
    events = []
    for microseconds, line in logs:
        if microseconds is not None and 'stage ->' in line:
            events.append((microseconds / 1e6, line.split('::', 1)[-1].strip()))
    return events


def _nearest(times, values, targets):
    """Sample (times,values) at each target time (step-hold of the latest prior value)."""
    out, index = [], 0
    for target in targets:
        while index + 1 < len(times) and times[index + 1] <= target:
            index += 1
        out.append(values[index] if values else 0.0)
    return out


def build(streams, logs, go, make_subplots):
    accel = find_stream(streams, 'ax', 'ay', 'az', prefer='adxl')  # high-g, not the IMU's low-g accel
    attitude = find_stream(streams, 'roll', 'pitch', prefer='bno')  # BNO055 emits heading/roll/pitch
    baro = find_stream(streams, 'elevation', prefer='icp') or find_stream(streams, 'altitude')
    laser = find_stream(streams, 'agl')
    gnss = find_stream(streams, 'lat', 'lon')
    fins = find_stream(streams, 'eleron_left', 'eleron_right', 'yaw')  # commanded servo angles (sim/board)
    health = find_stream(streams, 'load')  # board_health.csv: temp (C), mem_free (bytes), load (%)

    trajectory = go.Figure()
    if gnss is not None:
        times, latitude = gnss.column('lat')
        _, longitude = gnss.column('lon')
        height_field = 'elevation' if (baro and 'elevation' in baro.fields) else 'altitude'
        height = _nearest(*baro.column(height_field), targets=times) if baro else [0.0] * len(times)
        speed = [k / 1.94384 for k in gnss.column('speed_kn')[1]] if 'speed_kn' in gnss.fields else [0.0] * len(times)
        _, course = gnss.column('course') if 'course' in gnss.fields else (times, [0.0] * len(times))
        # per-point hover so a click on the 3D track reads out everything known at that instant
        text = ['t=%.1fs<br>height=%.0f m<br>speed=%.1f m/s<br>heading=%.0f deg' % point
                for point in zip(times, height, speed, course)]
        trajectory.add_trace(go.Scatter3d(
            x=longitude, y=latitude, z=height, mode='lines+markers', name='trajectory',
            text=text, hoverinfo='text',
            line=dict(width=4), marker=dict(size=2, color=times, colorscale='Viridis',
                                            colorbar=dict(title='t (s)'))))
        trajectory.update_layout(title='trajectory — GNSS ground-track + baro height (hover/click a point)',
                                 scene=dict(xaxis_title='lon', yaxis_title='lat', zaxis_title='height (m)'))
    else:
        trajectory.update_layout(title='trajectory — no GNSS fix in this capture')

    series = make_subplots(rows=7, cols=1, shared_xaxes=True, vertical_spacing=0.025,
                           subplot_titles=('|accel| (g)', 'altitude / elevation (m)', 'speed (m/s)',
                                           'attitude (deg)', 'fins — commanded (deg)',
                                           'board health — load %, temp °C, mem MB', 'agl (m)'))
    if accel is not None:
        times, ax = accel.column('ax')
        _, ay = accel.column('ay')
        _, az = accel.column('az')
        magnitude = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
        series.add_trace(go.Scatter(x=times, y=magnitude, name='|a|'), row=1, col=1)
    if baro is not None:
        for field in ('altitude', 'elevation'):
            if field in baro.fields:
                times, values = baro.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=field), row=2, col=1)
    if gnss is not None and 'speed_kn' in gnss.fields:  # GPS ground speed (knots) -> m/s
        times, knots = gnss.column('speed_kn')
        series.add_trace(go.Scatter(x=times, y=[k / 1.94384 for k in knots], name='speed'), row=3, col=1)
    if attitude is not None:
        for field in ('heading', 'yaw', 'roll', 'pitch'):
            if field in attitude.fields:
                times, values = attitude.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=field), row=4, col=1)
    if fins is not None:
        for field in ('eleron_left', 'eleron_right', 'yaw'):
            if field in fins.fields:
                times, values = fins.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=field), row=5, col=1)
    if health is not None:
        if 'load' in health.fields:
            series.add_trace(go.Scatter(x=health.column('load')[0], y=health.column('load')[1],
                                        name='load %'), row=6, col=1)
        if 'temp' in health.fields:
            series.add_trace(go.Scatter(x=health.column('temp')[0], y=health.column('temp')[1],
                                        name='temp °C'), row=6, col=1)
        if 'mem_free' in health.fields:  # bytes -> MB so it shares the panel's scale
            times, mem = health.column('mem_free')
            series.add_trace(go.Scatter(x=times, y=[m / 1e6 for m in mem], name='mem MB'), row=6, col=1)
    if laser is not None:
        times, values = laser.column('agl')
        series.add_trace(go.Scatter(x=times, y=values, name='agl', mode='markers'), row=7, col=1)
    for time_s, label in stage_events(logs):
        series.add_vline(x=time_s, line_dash='dash', line_color='crimson',
                         annotation_text=label, annotation_position='top left')
    # 'x unified' -> hovering (or clicking) any time shows every panel's value at that instant
    series.update_layout(height=1450, title='flight parameters', showlegend=True, hovermode='x unified')
    series.update_xaxes(title_text='time (s)', row=7, col=1)
    return trajectory, series


def write_html(trajectory, series, out, pio, plotlyjs=True):
    """One HTML with both figures. plotlyjs True -> embed plotly.js (self-contained, ~4.5 MB); 'cdn' ->
    load it from the CDN (tiny file, needs internet to view)."""
    body = (pio.to_html(trajectory, include_plotlyjs=plotlyjs, full_html=False)
            + pio.to_html(series, include_plotlyjs=False, full_html=False))
    with open(out, 'w') as handle:
        handle.write('<!doctype html><html><head><meta charset="utf-8">'
                     '<title>Coludo flight report</title></head><body>'
                     '<h1>Coludo flight report</h1>' + body + '</body></html>')


def main():
    parser = argparse.ArgumentParser(description='Render a Coludo flight capture as an interactive HTML report.')
    parser.add_argument('capture', help='recorder capture (the UART stream saved by the Luckfox)')
    parser.add_argument('-o', '--out', default='flight.html', help='output HTML (default flight.html)')
    parser.add_argument('--cdn', action='store_true', help='load plotly.js from the CDN (tiny file, needs net)')
    args = parser.parse_args()
    go, pio, make_subplots = _require_plotly()
    with open(args.capture) as handle:
        streams, logs = flight_telemetry.parse(handle.read())
    if not streams:
        sys.exit('no telemetry streams found in %s' % args.capture)
    trajectory, series = build(streams, logs, go, make_subplots)
    write_html(trajectory, series, args.out, pio, 'cdn' if args.cdn else True)
    print('wrote %s (%d streams, %d log lines)' % (args.out, len(streams), len(logs)))


if __name__ == '__main__':
    main()
