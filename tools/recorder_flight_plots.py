"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Plot a flight that tools/recorder_flight.py cut from a recorder dump: <launch>/flight/<stream>.csv, t_s from
ignition. Needs plotly, plus kaleido for the SVGs (`pipx inject plotly kaleido`, then `plotly_get_chrome -y`):

    ~/.local/share/pipx/venvs/plotly/bin/python tools/recorder_flight_plots.py launches/20261003/TMS-7C

Writes <launch>/plots/launch.svg (forces, rates, height, airspeed), attitude.svg (the BNO055's own fusion)
and flight.html with both, interactive. Every stream ends at its own last flush when a crash cuts the
recorder's power, so each figure marks where each stream's data stops.

Units as the firmware records them, converted here: LSM6DSO32 gyro centi-deg/s -> deg/s, BNO055 roll and
pitch centi-deg -> deg, SDP810 airspeed cm/s -> m/s. The ADXL375 is drawn against the LSM6DSO32's pad
reading: its per-axis zero offset (~0.8 g on x) is the pad mean difference between the two, removed.
A row with any acceleration beyond its sensor's full scale is a corrupted row (UART) and is not drawn;
the CSVs keep it as recorded.
"""

import argparse
import csv
import os
import statistics
import sys

_PAD_S: tuple = (-5.0, -0.5)
_FULL_SCALE_G: dict = {'imu_lsm6dso32': 32.0, 'accel_adxl375': 200.0, 'imu_bno055': 16.0}
_BEFORE_S: float = 0.5
_WIDTH: int = 1100
_ROW_PX: int = 230
_COLOURS: dict = {'x': '#1f77b4', 'y': '#ff7f0e', 'z': '#2ca02c', 'lsm': '#d62728', 'adxl': '#7f7f7f',
                  'bmp280': '#1f77b4', 'icp10111': '#9467bd', 'airspeed': '#8c564b'}


def _require_plotly():
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        sys.exit('recorder_flight_plots needs plotly:  pipx inject plotly kaleido  (see the module docstring)')
    return go, make_subplots


def load(folder: str, stream: str) -> list:
    """
    The stream's rows as dicts of floats (None for an empty cell), or [] when it was not recorded. Rows
    with an acceleration past the sensor's full scale are corrupted and dropped.
    """
    path = os.path.join(folder, stream + '.csv')
    if not os.path.exists(path):
        return []
    with open(path, newline='') as handle:
        rows = [{key: (float(value) if value not in ('', None) else None) for key, value in row.items()}
                for row in csv.DictReader(handle)]
    limit = _FULL_SCALE_G.get(stream)
    if limit is None:
        return rows
    return [r for r in rows if all(r[axis] is None or abs(r[axis]) <= limit for axis in ('ax', 'ay', 'az'))]


def _pad_mean(rows: list, field: str) -> float:
    """The field's mean over the pad window."""
    return statistics.mean(r[field] for r in rows if _PAD_S[0] <= r['t_s'] <= _PAD_S[1] and r[field] is not None)


def _figure(make_subplots, titles: list, title: str):
    """A stacked, x-linked figure: one y title per row, the legend in the right margin."""
    fig = make_subplots(rows=len(titles), cols=1, shared_xaxes=True, vertical_spacing=0.04)
    fig.update_layout(template='plotly_white', width=_WIDTH, height=_ROW_PX * len(titles) + 130,
                      title={'text': title, 'x': 0.01, 'y': 0.99, 'yanchor': 'top', 'font': {'size': 17}},
                      margin={'l': 80, 'r': 190, 't': 110, 'b': 50},
                      legend={'x': 1.01, 'xanchor': 'left', 'y': 1.0, 'yanchor': 'top'})
    for row, text in enumerate(titles, 1):
        fig.update_yaxes(title_text=text, row=row, col=1)
    fig.update_xaxes(title_text='t from ignition (s)', row=len(titles), col=1)
    return fig


def _line(go, rows: list, field: str, name: str, colour: str, scale: float = 1.0, offset: float = 0.0,
          dash: str = None):
    """One trace of rows[field] * scale - offset against t_s."""
    return go.Scatter(x=[r['t_s'] for r in rows], y=[None if r[field] is None else r[field] * scale - offset
                                                     for r in rows],
                      mode='lines', name=name, line={'color': colour, 'width': 1.5, 'dash': dash})


def _ends(fig, streams: dict, end: float) -> None:
    """A dotted line where each stream's data stops (the crash cut the recorder mid-buffer)."""
    stops = sorted((rows[-1]['t_s'], name) for name, rows in streams.items() if rows)
    lift = 0
    for index, (t, name) in enumerate(stops):
        if t < 0 or t > end:
            continue
        lift = (lift + 1) % 4 if index and t - stops[index - 1][0] < 0.08 else 0
        fig.add_vline(x=t, line_dash='dot', line_color='#aaaaaa', line_width=1)
        fig.add_annotation(x=t, y=1.0, yref='paper', yanchor='bottom', yshift=2 + 14 * lift,
                           text=name + ' ends', showarrow=False, font={'size': 10, 'color': '#666666'})


def launch(go, make_subplots, streams: dict, name: str):
    """Axial and lateral specific force, body rates, height, airspeed."""
    lsm, adxl = streams['imu_lsm6dso32'], streams['accel_adxl375']
    end = max(rows[-1]['t_s'] for rows in streams.values() if rows) + 0.05
    window = {key: [r for r in rows if r['t_s'] >= -_BEFORE_S] for key, rows in streams.items()}
    fig = _figure(make_subplots, ['axial force (g)', 'lateral force (g)', 'body rate (deg/s)', 'height (m)',
                                  'airspeed (m/s)'],
                  '%s launch: the recorded 1.3 s (the crash cut the recorder; data stops per stream)' % name)
    offsets = {axis: _pad_mean(adxl, axis) - _pad_mean(lsm, axis) for axis in ('ax', 'ay', 'az')} if adxl and lsm \
        else {'ax': 0.0, 'ay': 0.0, 'az': 0.0}
    if lsm:
        fig.add_trace(_line(go, window['imu_lsm6dso32'], 'ax', 'LSM6DSO32 x', _COLOURS['lsm']), 1, 1)
    if adxl:
        fig.add_trace(_line(go, window['accel_adxl375'], 'ax', 'ADXL375 x', _COLOURS['adxl'],
                            offset=offsets['ax']), 1, 1)
    for axis in ('y', 'z'):
        if adxl:
            fig.add_trace(_line(go, window['accel_adxl375'], 'a' + axis, 'ADXL375 ' + axis, _COLOURS[axis],
                                offset=offsets['a' + axis]), 2, 1)
        if lsm:
            fig.add_trace(_line(go, window['imu_lsm6dso32'], 'a' + axis, 'LSM6DSO32 ' + axis, _COLOURS[axis],
                                dash='dot'), 2, 1)
    for axis, label in (('x', 'roll'), ('y', 'pitch'), ('z', 'yaw')):
        if lsm:
            fig.add_trace(_line(go, window['imu_lsm6dso32'], 'g' + axis, 'gyro %s (%s)' % (axis, label),
                                _COLOURS[axis], scale=0.01), 3, 1)
    for stream in ('baro_bmp280', 'baro_icp10111'):
        rows = window.get(stream) or []
        if rows:
            fig.add_trace(_line(go, rows, 'altitude', stream.split('_')[1], _COLOURS[stream.split('_')[1]],
                                offset=_pad_mean(streams[stream], 'altitude')), 4, 1)
    if window.get('airspeed_sdp810'):
        fig.add_trace(_line(go, window['airspeed_sdp810'], 'airspeed_cms', 'pitot', _COLOURS['airspeed'],
                            scale=0.01), 5, 1)
    fig.update_xaxes(range=[-_BEFORE_S, end])
    _ends(fig, streams, end)
    return fig


def attitude(go, make_subplots, streams: dict, name: str):
    """The BNO055's own fused attitude and its (saturating) accelerometer."""
    bno = [r for r in streams['imu_bno055'] if r['t_s'] >= -_BEFORE_S]
    end = max(rows[-1]['t_s'] for rows in streams.values() if rows) + 0.05
    fig = _figure(make_subplots, ['heading (deg)', 'roll, pitch (deg)', 'BNO055 accel (g)'],
                  '%s attitude by the BNO055 fusion -- which assumes ~1 g and saturates at 4 g: indicative only'
                  % name)
    fig.add_trace(_line(go, bno, 'heading', 'heading', _COLOURS['x']), 1, 1)
    fig.add_trace(_line(go, bno, 'roll', 'roll', _COLOURS['y'], scale=0.01), 2, 1)
    fig.add_trace(_line(go, bno, 'pitch', 'pitch', _COLOURS['z'], scale=0.01), 2, 1)
    for axis in ('x', 'y', 'z'):
        fig.add_trace(_line(go, bno, 'a' + axis, 'accel ' + axis, _COLOURS[axis]), 3, 1)
    fig.update_xaxes(range=[-_BEFORE_S, end])
    _ends(fig, {'imu_bno055': streams['imu_bno055']}, end)
    return fig


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('launch', help='the launch folder holding flight/ (e.g. launches/20261003/TMS-7C)')
    args = parser.parse_args()
    go, make_subplots = _require_plotly()
    folder = os.path.join(args.launch, 'flight')
    streams = {stream: load(folder, stream) for stream in ('imu_lsm6dso32', 'accel_adxl375', 'imu_bno055',
                                                          'baro_bmp280', 'baro_icp10111', 'airspeed_sdp810')}
    name = os.path.basename(os.path.normpath(args.launch))
    out = os.path.join(args.launch, 'plots')
    os.makedirs(out, exist_ok=True)
    parts = []
    for title, build in (('launch', launch), ('attitude', attitude)):
        fig = build(go, make_subplots, streams, name)
        fig.write_image(os.path.join(out, title + '.svg'))
        parts.append(fig.to_html(full_html=False, include_plotlyjs='cdn' if not parts else False))
        print('wrote', os.path.join(out, title + '.svg'))
    with open(os.path.join(out, 'flight.html'), 'w') as handle:
        handle.write('<!DOCTYPE html><html><head><meta charset="utf-8"><title>%s flight</title></head>'
                     '<body>%s</body></html>\n' % (name, '\n'.join(parts)))
    print('wrote', os.path.join(out, 'flight.html'))


if __name__ == '__main__':
    main()
