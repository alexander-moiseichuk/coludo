"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Render a Coludo flight capture as one self-contained interactive HTML (plotly): a 3D trajectory (GNSS
ground-track + baro altitude) plus linked time-series (accel magnitude, altitude/elevation, attitude,
agl) with stage/separation events marked. Streams are matched by their field names, not file names, so
it survives config renames.

  pip install plotly
  python3 synth_capture.py > demo.txt && python3 flight_report.py demo.txt -o demo.html
  python3 flight_report.py <luckfox-capture> -o flight.html
"""

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


# role-based stream lookup now lives with the parser, so every renderer resolves streams the same way
find_stream = flight_telemetry.find_stream


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


def leak_estimate(health, events):
    """
    The GC-off PSRAM leak and the extrapolated time-to-OOM.

    Measured from the mem_free slope over BOOSTING->DONE (GC is disabled airborne, so mem_free falls
    monotonically); time-to-OOM is free-at-boost / leak.

    Returns:
        (leak_kbps, oom_s, free_boost_mb, free_low_mb), or None when there is no health/stage data.
    """
    if health is None or 'mem_free' not in health.fields:
        return None
    boost = next((t for t, label in events if 'boosting' in label.lower()), None)
    done = next((t for t, label in events if 'done' in label.lower()), None)
    times, mem = health.column('mem_free')
    if boost is None or not times:
        return None
    end = done if done is not None else times[-1]
    window = [(t, m) for t, m in zip(times, mem) if boost <= t <= end - 1.0]  # drop the DONE snap-back
    if len(window) < 2:
        return None
    (t0, m0), (t1, m1) = window[0], window[-1]
    span = t1 - t0
    leak_bps = (m0 - m1) / span if span > 0 else 0.0
    oom_s = m0 / leak_bps if leak_bps > 0 else float('inf')
    return (leak_bps / 1000.0, oom_s, m0 / 1e6, m1 / 1e6)


def build(streams, logs, go, make_subplots):
    accel = find_stream(streams, 'ax', 'ay', 'az', prefer='adxl')  # high-g, not the IMU's low-g accel
    attitude = find_stream(streams, 'roll', 'pitch', prefer='bno')  # BNO055 emits heading/roll/pitch
    baro = find_stream(streams, 'elevation', prefer='icp') or find_stream(streams, 'altitude')
    laser = find_stream(streams, 'agl')
    gnss = find_stream(streams, 'lat', 'lon')
    fins = find_stream(streams, 'eleron_left', 'eleron_right', 'yaw')  # commanded servo angles (sim/board)
    health = find_stream(streams, 'load')  # board_health.csv: temp (C), mem_free (bytes), load (%)
    power = find_stream(streams, 'voltage_mv', 'current_ma', 'power_mw')  # power_ina226.csv: integer mV/mA/mW
    gyro = find_stream(streams, 'gx', 'gy', 'gz', prefer='lsm')  # imu_lsm6dso32.csv: gyro rate (deg/s) -> PID D term
    pitot = find_stream(streams, 'dynamic_pressure')  # airspeed_sdp810.csv: the DIRECT pitot measurement
    control = find_stream(streams, 'fin_cap')  # flight.csv: the control state (findings §27.2)
    # telemetry carries centi-units (fixnums), never floats -- a float in a row heap-boxes on
    # MicroPython, so every rate is x100 on the wire and scaled here
    centi = 100.0

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

    series = make_subplots(rows=11, cols=1, shared_xaxes=True, vertical_spacing=0.017,
                           subplot_titles=('|accel| (g)', 'altitude / elevation (m)', 'speed (m/s)',
                                           'attitude (deg)', 'fins — commanded (deg)',
                                           'board health — load %, temp °C, mem MB', 'agl (m)',
                                           'engine — mV / mA / mW / over-current alerts (INA226)',
                                           'gyro rate — LSM6DSO32 (deg/s) → PID D term',
                                           'airspeed (m/s) — pitot vs governor estimate vs GNSS ground',
                                           'control authority (deg) — fin cap vs per-axis demand'))
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
    if power is not None:  # real INA226 servo-rail draw (the servos physically move during HITL)
        for field in ('voltage_mv', 'current_ma', 'power_mw', 'alerts'):  # alerts = cumulative over-current trips
            if field in power.fields:
                times, values = power.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=field), row=8, col=1)
    if gyro is not None:  # the gyro rate the PID reads as its D term (roll->gx, pitch->gy, yaw->gz)
        for field, label in (('gx', 'roll rate'), ('gy', 'pitch rate'), ('gz', 'yaw rate')):
            if field in gyro.fields:
                times, values = gyro.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=label), row=9, col=1)
    """
    AIRSPEED (findings §27.4): the pitot is the direct measurement, the governor estimate is what the fin
    cap was actually computed from, and GNSS ground speed is the third opinion -- overlaid, their spread
    IS the calibration signal (a calm pass should collapse pitot onto ground speed; see
    tools/airspeed_calibrate.py) and their divergence flags wind, saturation or a fallback to the accel
    backbone.
    """
    if pitot is not None and 'airspeed_cms' in pitot.fields:
        times, values = pitot.column('airspeed_cms')
        series.add_trace(go.Scatter(x=times, y=[v / centi for v in values], name='pitot'), row=10, col=1)
    if control is not None and 'airspeed_cms' in control.fields:
        times, values = control.column('airspeed_cms')
        series.add_trace(go.Scatter(x=times, y=[v / centi for v in values],
                                    name='estimate (governor)'), row=10, col=1)
    if gnss is not None and 'speed_kn' in gnss.fields:
        times, knots = gnss.column('speed_kn')
        series.add_trace(go.Scatter(x=times, y=[k / 1.94384 for k in knots], name='GNSS ground',
                                    line=dict(dash='dot')), row=10, col=1)
    """
    CONTROL AUTHORITY (findings §27.16): fin_cap is the 1/v² limit the governor imposed, and the per-axis
    demands are what the PID asked for. Where a demand rides the cap the loop was CLIPPED -- previously
    invisible, since only the resulting fin angles were ever recorded.
    """
    if control is not None:
        times, cap = control.column('fin_cap')
        series.add_trace(go.Scatter(x=times, y=cap, name='fin cap', line=dict(width=3)), row=11, col=1)
        series.add_trace(go.Scatter(x=times, y=[-value for value in cap], name='fin cap (−)',
                                    line=dict(width=3), showlegend=False), row=11, col=1)
        for field, label in (('roll_cmd', 'roll demand'), ('pitch_cmd', 'pitch demand'),
                             ('yaw_cmd', 'yaw demand')):
            if field in control.fields:
                times, values = control.column(field)
                series.add_trace(go.Scatter(x=times, y=values, name=label), row=11, col=1)

    events = stage_events(logs)
    for time_s, label in events:
        series.add_vline(x=time_s, line_dash='dash', line_color='crimson',
                         annotation_text=label, annotation_position='top left')
    # GC-off leak + time-to-OOM headline (mem_free slope over the airborne, GC-disabled window)
    leak = leak_estimate(health, events)
    title = 'flight parameters'
    if leak is not None:
        leak_kbps, oom_s, free_boost, free_low = leak
        oom_txt = '%.0f s' % oom_s if oom_s != float('inf') else 'n/a'
        title = ('flight parameters — GC-off leak %.0f KB/s, time-to-OOM ~%s '
                 '(free %.1f→%.1f MB)' % (leak_kbps, oom_txt, free_boost, free_low))
        series.add_annotation(row=6, col=1, x=0.0, xref='x domain', y=1.0, yref='y6 domain',
                              text='leak %.0f KB/s · OOM ~%s' % (leak_kbps, oom_txt),
                              showarrow=False, xanchor='left', yanchor='bottom',
                              font=dict(color='crimson', size=12))
    # 'x unified' -> hovering (or clicking) any time shows every panel's value at that instant
    series.update_layout(height=2250, title=title, showlegend=True, hovermode='x unified')
    series.update_xaxes(title_text='time (s)', row=9, col=1)
    return trajectory, series


def write_html(trajectory, series, out, pio, plotlyjs=True):
    """
    Write one HTML file holding both figures.

    Args:
        trajectory - the 3D trajectory figure.
        series - the linked time-series figure.
        out - path to write the HTML to.
        pio - the plotly.io module.
        plotlyjs - True embeds plotly.js (self-contained, ~4.5 MB); 'cdn' loads it from the CDN (tiny
            file, needs internet to view).

    Returns:
        None (writes the HTML to `out`).
    """
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
