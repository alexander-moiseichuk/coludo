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


def find_stream(streams, *fields):
    """The first stream that carries all the given field names (None if none does)."""
    for stream in streams.values():
        if all(field in stream.fields for field in fields):
            return stream
    return None


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
    accel = find_stream(streams, 'ax', 'ay', 'az')
    attitude = find_stream(streams, 'yaw', 'pitch', 'roll')
    baro = find_stream(streams, 'elevation') or find_stream(streams, 'altitude')
    laser = find_stream(streams, 'agl')
    gnss = find_stream(streams, 'lat', 'lon')

    trajectory = go.Figure()
    if gnss is not None:
        times, latitude = gnss.column('lat')
        _, longitude = gnss.column('lon')
        height_field = 'elevation' if (baro and 'elevation' in baro.fields) else 'altitude'
        height = _nearest(*baro.column(height_field), targets=times) if baro else [0.0] * len(times)
        trajectory.add_trace(go.Scatter3d(
            x=longitude, y=latitude, z=height, mode='lines+markers', name='trajectory',
            line=dict(width=4), marker=dict(size=2, color=times, colorscale='Viridis',
                                            colorbar=dict(title='t (s)'))))
        trajectory.update_layout(title='trajectory — GNSS ground-track + baro height',
                                 scene=dict(xaxis_title='lon', yaxis_title='lat', zaxis_title='height (m)'))
    else:
        trajectory.update_layout(title='trajectory — no GNSS fix in this capture')

    series = make_subplots(rows=4, cols=1, shared_xaxes=True, vertical_spacing=0.05,
                           subplot_titles=('|accel| (g)', 'altitude / elevation (m)',
                                           'attitude (deg)', 'agl (m)'))
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
    if attitude is not None:
        for field in ('yaw', 'pitch', 'roll'):
            times, values = attitude.column(field)
            series.add_trace(go.Scatter(x=times, y=values, name=field), row=3, col=1)
    if laser is not None:
        times, values = laser.column('agl')
        series.add_trace(go.Scatter(x=times, y=values, name='agl', mode='markers'), row=4, col=1)
    for time_s, label in stage_events(logs):
        series.add_vline(x=time_s, line_dash='dash', line_color='crimson',
                         annotation_text=label, annotation_position='top left')
    series.update_layout(height=900, title='flight parameters', showlegend=True)
    series.update_xaxes(title_text='time (s)', row=4, col=1)
    return trajectory, series


def write_html(trajectory, series, out, pio):
    """One self-contained HTML: plotly.js embedded once, then both figures."""
    body = (pio.to_html(trajectory, include_plotlyjs=True, full_html=False)
            + pio.to_html(series, include_plotlyjs=False, full_html=False))
    with open(out, 'w') as handle:
        handle.write('<!doctype html><html><head><meta charset="utf-8">'
                     '<title>Coludo flight report</title></head><body>'
                     '<h1>Coludo flight report</h1>' + body + '</body></html>')


def main():
    parser = argparse.ArgumentParser(description='Render a Coludo flight capture as an interactive HTML report.')
    parser.add_argument('capture', help='recorder capture (the UART stream saved by the Luckfox)')
    parser.add_argument('-o', '--out', default='flight.html', help='output HTML (default flight.html)')
    args = parser.parse_args()
    go, pio, make_subplots = _require_plotly()
    with open(args.capture) as handle:
        streams, logs = flight_telemetry.parse(handle.read())
    if not streams:
        sys.exit('no telemetry streams found in %s' % args.capture)
    trajectory, series = build(streams, logs, go, make_subplots)
    write_html(trajectory, series, args.out, pio)
    print('wrote %s (%d streams, %d log lines)' % (args.out, len(streams), len(logs)))


if __name__ == '__main__':
    main()
