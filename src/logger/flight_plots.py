"""
Plot a nose-logger flight: the CSV flight.py cut, measured by the same flight.analyse().

Host-side, needs plotly -- and kaleido for the SVGs (a README can only show static images):

    pipx inject plotly kaleido && plotly_get_chrome -y        # once: kaleido renders through Chrome
    ~/.local/share/pipx/venvs/plotly/bin/python src/logger/flight_plots.py \\
        launches/20261003/TMS-7/flight.csv -o launches/20261003/TMS-7/plots --mass 0.2283 --propellant 0.060

Writes one SVG per figure and flight.html with all of them, interactive. Long windows plot the
accelerometer and gyro as a per-0.1 s min/max envelope -- every peak survives, the SVG stays small;
the HTML and the short windows (boost, landing) carry every sample.
"""

import argparse
import os
import sys

_HERE: str = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'glider'))
import flight  # noqa: E402
import sim_model  # noqa: E402 -- the motor the simulator flies, so the comparison cannot drift from it

_WIDTH: int = 1100
_ROW_PX: int = 250
_ENVELOPE_S: float = 0.1
_CROWDED: float = 0.04            # labels nearer than this share of the window are stacked
_ACCEL_FULL_SCALE_G: float = 16.0
_GYRO_FULL_SCALE_DPS: float = 2000.0
_COLOURS: dict = {'baro': '#1f77b4', 'inertial': '#d62728', 'axial': '#2ca02c', 'magnitude': '#7f7f7f',
                  'x': '#1f77b4', 'y': '#ff7f0e', 'z': '#2ca02c', 'vertical': '#8c564b',
                  'horizontal': '#9467bd', 'speed': '#000000', 'thrust': '#d62728', 'model': '#7f7f7f'}


def _require_plotly():
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        sys.exit('flight_plots needs plotly:  pipx inject plotly kaleido  (see the module docstring)')
    return go, make_subplots


def _window(samples: list, start: float, end: float) -> list:
    """Indices of the samples inside [start, end] s."""
    return [i for i, s in enumerate(samples) if start <= s.t <= end]


def _envelope(times: list, values: list, bucket: float) -> tuple:
    """Per-`bucket` min and max of `values`, in time order: a decimation that keeps every peak."""
    out_t, out_v, current, group = [], [], None, []

    def flush():
        if group:
            low, high = min(group, key=lambda p: p[1]), max(group, key=lambda p: p[1])
            for point in sorted({low, high}):
                out_t.append(point[0])
                out_v.append(point[1])

    for t, v in zip(times, values):
        if v is None:
            continue
        key = int((t + 1000.0) // bucket)
        if key != current:
            flush()
            current, group = key, []
        group.append((t, v))
    flush()
    return out_t, out_v


def _series(samples: list, indices: list, pick, envelope: bool) -> tuple:
    """(times, values) of pick(sample) over `indices`, enveloped when asked."""
    times = [samples[i].t for i in indices]
    values = [pick(samples[i]) for i in indices]
    return _envelope(times, values, _ENVELOPE_S) if envelope else (times, values)


def _events(result: dict) -> list:
    """(t, label) of every event the figures mark."""
    return [(0.0, 'ignition'), (result['burnout'], 'burnout'), (result['ejection'][0], 'ejection'),
            (result['apogee']['inertial'][1], 'apogee'), (result['shock'][0], 'shock'),
            (result['touchdown'], 'touchdown')]


def _mark(fig, events: list, start: float, end: float) -> None:
    """
    A dotted line per event inside [start, end], labelled once above the plot. A label closer than
    _CROWDED of the window to the one before it goes a line higher, so neighbours never print over each
    other (ejection and apogee are half a second apart).
    """
    previous, lift = None, 0
    for t, label in sorted(e for e in events if start <= e[0] <= end):
        lift = (lift + 1) % 3 if previous is not None and t - previous < _CROWDED * (end - start) else 0
        fig.add_vline(x=t, line_dash='dot', line_color='#888888', line_width=1)
        fig.add_annotation(x=t, y=1.0, yref='paper', yanchor='bottom', yshift=2 + 15 * lift, text=label,
                           showarrow=False, font={'size': 11, 'color': '#555555'})
        previous = t


def _figure(make_subplots, titles: list, title: str):
    """A stacked, x-linked figure in the house style: one y title per row, the legend in the right margin."""
    fig = make_subplots(rows=len(titles), cols=1, shared_xaxes=True, vertical_spacing=0.04)
    fig.update_layout(template='plotly_white', width=_WIDTH, height=_ROW_PX * len(titles) + 130,
                      title={'text': title, 'x': 0.01, 'y': 0.99, 'yanchor': 'top', 'font': {'size': 17}},
                      margin={'l': 80, 'r': 170, 't': 110, 'b': 50},
                      legend={'x': 1.01, 'xanchor': 'left', 'y': 1.0, 'yanchor': 'top'})
    for row, text in enumerate(titles, 1):
        fig.update_yaxes(title_text=text, row=row, col=1)
    fig.update_xaxes(title_text='t from ignition (s)', row=len(titles), col=1)
    return fig


def _line(go, times, values, name: str, colour: str, legend: bool = True, dash: str = None, width: float = 1.5):
    """One trace."""
    return go.Scatter(x=times, y=values, mode='lines', name=name, showlegend=legend,
                      line={'color': colour, 'width': width, 'dash': dash})


def overview(go, make_subplots, result: dict):
    """The whole flight: height, |a|, gyro."""
    samples = result['samples']
    start, end = -2.0, samples[-1].t
    indices = _window(samples, start, end)
    track = result['track']
    fig = _figure(make_subplots, ['height above the pad (m)', '|a| (g)', 'gyro (deg/s)'],
                  'TMS-7 nose logger: the whole flight (accel and gyro as a 0.1 s min/max envelope)')
    times, heights = _series(samples, indices, lambda s: s.height, False)
    filtered = [result['filtered'][i] for i in indices]
    fig.add_trace(_line(go, times[::5], filtered[::5], 'baro (spike-filtered)', _COLOURS['baro']), 1, 1)
    fig.add_trace(_line(go, [r[0] for r in track], [r[1] for r in track], 'inertial (to ejection)',
                        _COLOURS['inertial']), 1, 1)
    fig.add_trace(_line(go, *_series(samples, indices, lambda s: s.accel and flight._magnitude(s.accel), True),
                        '|a|', _COLOURS['magnitude'], legend=False, width=1), 2, 1)
    for axis, name in enumerate('xyz'):
        fig.add_trace(_line(go, *_series(samples, indices, lambda s, a=axis: s.gyro and s.gyro[a], True),
                            'gyro ' + name, _COLOURS[name], width=1), 3, 1)
    for level in (_GYRO_FULL_SCALE_DPS, -_GYRO_FULL_SCALE_DPS):
        fig.add_hline(y=level, line_dash='dash', line_color='#bbbbbb', row=3, col=1)
    _mark(fig, _events(result), start, end)
    return fig


def boost(go, make_subplots, result: dict):
    """Ignition to ejection: specific force, speeds, height (inertial vs baro), attitude."""
    samples, track = result['samples'], result['track']
    start, end = -0.5, result['ejection'][0] + 0.3
    indices = _window(samples, start, end)
    fig = _figure(make_subplots, ['axial specific force (g)', 'speed (m/s, inertial)', 'height (m)',
                                  'thrust axis from vertical (deg)'],
                  'TMS-7 boost and coast: the inertial solution against the nose-cone baro')
    axial = result['axial']
    fig.add_trace(_line(go, [samples[i].t for i in indices], [axial[i] for i in indices], 'axial',
                        _COLOURS['axial'], legend=False), 1, 1)
    fig.add_hline(y=_ACCEL_FULL_SCALE_G, line_dash='dash', line_color='#d62728', row=1, col=1,
                  annotation_text='BMI323 full scale', annotation_position='bottom right')
    for column, name in ((2, 'vertical'), (3, 'horizontal'), (4, 'speed')):
        fig.add_trace(_line(go, [r[0] for r in track], [r[column] for r in track], name + ' speed',
                            _COLOURS[name]), 2, 1)
    fig.add_trace(_line(go, [samples[i].t for i in indices], [samples[i].height for i in indices], 'baro height',
                        _COLOURS['baro']), 3, 1)
    fig.add_trace(_line(go, [r[0] for r in track], [r[1] for r in track], 'inertial height',
                        _COLOURS['inertial']), 3, 1)
    fig.add_trace(_line(go, [r[0] for r in track], [r[7] for r in track], 'zenith', _COLOURS['inertial'],
                        legend=False), 4, 1)
    _mark(fig, _events(result), start, end)
    return fig


def thrust(go, make_subplots, result: dict):
    """The thrust estimate against the simulator's F15, with the impulse delivered."""
    estimate = result['thrust']
    average, burn = sim_model.MOTORS['F15']
    fig = _figure(make_subplots, ['thrust (N)', 'impulse (N s)'],
                  'TMS-7 F15: estimated thrust (%.0f g liftoff, %.0f g propellant) vs the simulator\'s motor' % (
                      estimate['mass'] * 1e3, estimate['propellant'] * 1e3))
    fig.add_trace(_line(go, estimate['t'], estimate['newton'], 'estimate: mass x axial + drag',
                        _COLOURS['thrust']), 1, 1)
    fig.add_trace(_line(go, [0.0, 0.0, burn, burn], [0.0, average, average, 0.0],
                        'sim_model F15: %.1f N for %.2f s' % (average, burn), _COLOURS['model'], dash='dash'), 1, 1)
    delivered, total = [0.0], 0.0
    for i in range(1, len(estimate['t'])):
        total += (estimate['newton'][i] + estimate['newton'][i - 1]) / 2.0 * (estimate['t'][i] - estimate['t'][i - 1])
        delivered.append(total)
    fig.add_trace(_line(go, estimate['t'], delivered, 'estimate', _COLOURS['thrust'], legend=False), 2, 1)
    fig.add_trace(_line(go, [0.0, burn], [0.0, average * burn], 'sim_model', _COLOURS['model'], legend=False,
                        dash='dash'), 2, 1)
    return fig


def descent(go, make_subplots, result: dict):
    """Under the chute: height, descent rate, the tumble."""
    samples = result['samples']
    start, end = result['ejection'][0] - 1.0, result['touchdown'] + 1.0
    indices = _window(samples, start, end)
    fig = _figure(make_subplots, ['height (m, baro)', 'descent rate (m/s)', 'gyro (deg/s)'],
                  'TMS-7 under the chute (descent rate: 2 s regression of the spike-filtered baro)')
    filtered = [result['filtered'][i] for i in indices]
    fig.add_trace(_line(go, [samples[i].t for i in indices][::5], filtered[::5], 'baro', _COLOURS['baro'],
                        legend=False), 1, 1)
    fig.add_trace(_line(go, [t for t, _r in result['rate']], [r for _t, r in result['rate']], 'rate',
                        _COLOURS['vertical'], legend=False), 2, 1)
    for axis, name in enumerate('xyz'):
        fig.add_trace(_line(go, *_series(samples, indices, lambda s, a=axis: s.gyro and s.gyro[a], True),
                            'gyro ' + name, _COLOURS[name], width=1), 3, 1)
    _mark(fig, _events(result), start, end)
    return fig


def landing(go, make_subplots, result: dict):
    """The last seconds: every sample, glitches included."""
    samples = result['samples']
    start, end = result['shock'][0] - 2.0, result['rest'][0] + 2.0
    indices = _window(samples, start, end)
    times = [samples[i].t for i in indices]
    fig = _figure(make_subplots, ['height (m, baro)', '|a| (g)', 'gyro (deg/s)'],
                  'TMS-7 landing: every sample, glitches included')
    fig.add_trace(go.Scatter(x=times, y=[samples[i].height for i in indices], mode='markers', name='raw',
                             marker={'size': 3, 'color': '#aec7e8'}), 1, 1)
    fig.add_trace(_line(go, times, [result['filtered'][i] for i in indices], 'spike-filtered', _COLOURS['baro']),
                  1, 1)
    # the y range follows the spike-filtered height: an impact glitch reads tens of metres off
    top = max(result['filtered'][i] for i in indices if result['filtered'][i] is not None)
    fig.update_yaxes(range=[result['rest'][1] - 4.0, top + 2.0], row=1, col=1)
    fig.add_trace(_line(go, times, [samples[i].accel and flight._magnitude(samples[i].accel) for i in indices],
                        '|a|', _COLOURS['magnitude'], legend=False), 2, 1)
    for axis, name in enumerate('xyz'):
        fig.add_trace(_line(go, times, [samples[i].gyro and samples[i].gyro[axis] for i in indices], 'gyro ' + name,
                            _COLOURS[name], width=1), 3, 1)
    _mark(fig, _events(result) + [(result['rest'][0], 'at rest')], start, end)
    return fig


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('csv', help='the flight CSV flight.py wrote')
    parser.add_argument('-o', '--out', required=True, help='directory for the SVGs and flight.html')
    parser.add_argument('--mass', type=float, help='liftoff mass, kg (with --propellant: the thrust figure)')
    parser.add_argument('--propellant', type=float, help='propellant mass, kg')
    args = parser.parse_args()
    go, make_subplots = _require_plotly()
    result = flight.analyse(flight.read_csv(args.csv), args.mass, args.propellant)
    figures = [('overview', overview), ('boost', boost), ('descent', descent), ('landing', landing)]
    if result['thrust']:
        figures.insert(2, ('thrust', thrust))
    os.makedirs(args.out, exist_ok=True)
    parts = []
    for name, build in figures:
        fig = build(go, make_subplots, result)
        fig.write_image(os.path.join(args.out, name + '.svg'))
        parts.append(fig.to_html(full_html=False, include_plotlyjs='cdn' if not parts else False))
        print('wrote', os.path.join(args.out, name + '.svg'))
    with open(os.path.join(args.out, 'flight.html'), 'w') as handle:
        handle.write('<!DOCTYPE html><html><head><meta charset="utf-8"><title>TMS-7 flight</title></head>'
                     '<body>%s</body></html>\n' % '\n'.join(parts))
    print('wrote', os.path.join(args.out, 'flight.html'))


if __name__ == '__main__':
    main()
