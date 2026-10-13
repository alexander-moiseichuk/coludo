#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The boost's envelope over the SITL (doc/specs/boost-control.md BC2): the Rust flight code flown lockstep on the 6-DoF
boost (tools/stack_dynamics.py, `hitl_host.py --boost 6dof`), every combination of BOOSTING's law (a board config
each: passive, BOOSTING not a control stage; hold, the existing rod-vertical hold; lean, the flight entry's
`boost_law: "lean"`) x the stack's static margin x the crosswind against its folded wings x a seeded run. Scored
by the physics: the burn's path minimum and its worst lean, whether the nose went below the horizon under thrust (a
loop), the apogee, the landing.

  ~/.local/share/pipx/venvs/plotly/bin/python tools/boost_sweep.py --laws passive=<config> hold=<config> -o <outdir>
        [--stack tms7d] [--glider-g 287] [--margins 0.2 0.5 ...] [--winds 0 3 4.5 6] [--runs 2] [--workers 6]

The configs are the same board config but for the flight entry: `boosting` absent from its `stages` (passive) or
present (hold), `boost_law` set (lean). Writes <outdir>/runs.csv and summary.csv (';'), and, where plotly imports,
<outdir>/envelope.html and envelope.svg: per law, the margin x wind grid of the path minimum (LOOP where the stack
looped) and the apogee.
"""

import argparse
import concurrent.futures
import os
import queue
import re
import statistics
import subprocess
import sys

_ROOT: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
_SITL: str = os.path.join(_ROOT, 'src', 'avionics', 'target', 'release', 'sitl')
_LANDED: re.Pattern = re.compile(r'^LANDED miss=([\d.]+) inside=(True|False) aloft=([\d.]+) apogee=(\d+)$', re.M)
_BOOST: re.Pattern = re.compile(r'^BOOST path_min=(-?[\d.]+) tilt_max=([\d.]+) looped=(True|False) '
                                r'burnout=(-?[\d.]+)$', re.M)
_COLUMNS: tuple = ('law', 'margin_cal', 'wind_ms', 'run', 'outcome', 'path_min_deg', 'tilt_max_deg', 'looped',
                   'burnout_ms', 'apogee_m', 'aloft_s', 'miss_m', 'inside')
# the wind blows toward east and the stack's dorsal side faces east: the wind meets the folded wings' plane
_WIND_TOWARD_DEG: float = 90.0
_NOISE: float = 0.10  # the matrix's nominal sensor noise


def fly(config: str, law: str, stack: str, glider_g: int, margin: float, wind: float, run: int, port: int) -> dict:
    """One SITL flight on `port`; its row (`_COLUMNS`). A flight with no LANDED or BOOST line reads -1."""
    command = [sys.executable, os.path.join(_ROOT, 'tools', 'hitl_host.py'), '--sitl', _SITL, '--config', config,
               '--motor', 'F15', '--glider-g', str(glider_g), '--noise', str(_NOISE), '--wind', str(wind),
               '--wind-dir', str(_WIND_TOWARD_DEG), '--seed', str(run), '--sitl-port', str(port), '--boost', '6dof',
               '--stack', stack, '--stack-margin', str(margin)]
    out = subprocess.run(command, capture_output=True, text=True, timeout=900).stdout
    landed, burn = _LANDED.search(out), _BOOST.search(out)
    return {'law': law, 'margin_cal': margin, 'wind_ms': wind, 'run': run,
            'outcome': 'done' if '\nDONE' in '\n' + out else ('timeout' if 'TIMEOUT' in out else 'none'),
            'path_min_deg': float(burn.group(1)) if burn else -1.0,
            'tilt_max_deg': float(burn.group(2)) if burn else -1.0,
            'looped': burn is not None and burn.group(3) == 'True',
            'burnout_ms': float(burn.group(4)) if burn else -1.0,
            'apogee_m': int(landed.group(4)) if landed else -1,
            'aloft_s': float(landed.group(3)) if landed else -1.0,
            'miss_m': float(landed.group(1)) if landed else -1.0,
            'inside': landed is not None and landed.group(2) == 'True'}


def summary(rows: list) -> list:
    """Per law, margin and wind: the flights, the loops, the median path minimum, lean, apogee and time aloft."""
    keys = sorted({(row['law'], row['margin_cal'], row['wind_ms']) for row in rows})
    out = []
    for law, margin, wind in keys:
        cell = [row for row in rows if (row['law'], row['margin_cal'], row['wind_ms']) == (law, margin, wind)]
        out.append({'law': law, 'margin_cal': margin, 'wind_ms': wind, 'flights': len(cell),
                    'loops': sum(row['looped'] for row in cell),
                    'median_path_min_deg': round(statistics.median(row['path_min_deg'] for row in cell), 1),
                    'median_tilt_max_deg': round(statistics.median(row['tilt_max_deg'] for row in cell), 1),
                    'median_apogee_m': statistics.median(row['apogee_m'] for row in cell),
                    'median_aloft_s': round(statistics.median(row['aloft_s'] for row in cell), 1),
                    'in_zone': sum(row['inside'] for row in cell)})
    return out


def _write(path: str, rows: list, columns: tuple) -> None:
    """`rows` as a ';' CSV, numeric tokens verbatim."""
    with open(path, 'w') as handle:
        handle.write(';'.join(columns) + '\n')
        for row in rows:
            handle.write(';'.join(str(row[column]) for column in columns) + '\n')


def render(lines: list, laws: list, out: str) -> bool:
    """The envelope figure (html + svg) from the summary, one column per law; False when plotly does not import."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        return False
    margins = sorted({line['margin_cal'] for line in lines})
    winds = sorted({line['wind_ms'] for line in lines})
    titles = ['%s: the burn\'s path minimum, deg' % law for law in laws] + ['%s: apogee, m' % law for law in laws]
    figure = make_subplots(rows=2, cols=len(laws), horizontal_spacing=0.06 + 0.06 / len(laws), vertical_spacing=0.14,
                           subplot_titles=titles)
    for column, law in enumerate(laws, start=1):
        cell = {(line['margin_cal'], line['wind_ms']): line for line in lines if line['law'] == law}
        panels = ((1, 'median_path_min_deg', 'RdYlGn', 90), (2, 'median_apogee_m', 'Viridis', None))
        for row, field, scale, high in panels:
            values = [[cell[(margin, wind)][field] if (margin, wind) in cell else None for wind in winds]
                      for margin in margins]
            text = [['LOOP %d/%d' % (cell[(margin, wind)]['loops'], cell[(margin, wind)]['flights'])
                     if (margin, wind) in cell and cell[(margin, wind)]['loops'] else
                     ('%.0f' % cell[(margin, wind)][field] if (margin, wind) in cell else '') for wind in winds]
                    for margin in margins]
            figure.add_trace(go.Heatmap(z=values, x=['%g m/s' % wind for wind in winds],
                                        y=['%g cal' % margin for margin in margins], text=text,
                                        texttemplate='%{text}', colorscale=scale, zmin=0, zmax=high,
                                        showscale=column == len(laws),
                                        colorbar={'len': 0.42, 'y': 0.79 if row == 1 else 0.21}),
                             row=row, col=column)
    figure.update_layout(title='Boost envelope over the SITL: the 6-DoF stack (tms7d), F15, crosswind against the '
                               'folded wings', height=900, width=600 * len(laws), template='plotly_white')
    figure.write_html(out + '.html', include_plotlyjs='cdn')
    try:
        figure.write_image(out + '.svg')
    except Exception as error:  # kaleido or its browser missing: the html is the figure
        print('no svg: %s' % error)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--laws', nargs='+', required=True, metavar='NAME=CONFIG',
                        help='each law flown, by name and its board config: passive=..., hold=..., lean=...')
    parser.add_argument('-o', '--out', required=True)
    parser.add_argument('--stack', default='tms7d')
    parser.add_argument('--glider-g', type=int, default=287)
    parser.add_argument('--margins', type=float, nargs='+', default=[0.2, 0.5, 0.8, 1.0, 1.5, 2.0])
    parser.add_argument('--winds', type=float, nargs='+', default=[0.0, 3.0, 4.5, 6.0])
    parser.add_argument('--runs', type=int, default=2)
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    laws = [law.split('=', 1) for law in args.laws]
    jobs = [(config, law, margin, wind, run) for law, config in laws
            for margin in args.margins for wind in args.winds for run in range(1, args.runs + 1)]
    ports: queue.Queue = queue.Queue()
    for index in range(args.workers):
        ports.put(1300 + index)

    def one(job: tuple) -> dict:
        port = ports.get()
        try:
            config, law, margin, wind, run = job
            return fly(config, law, args.stack, args.glider_g, margin, wind, run, port)
        finally:
            ports.put(port)

    rows = []
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        for count, row in enumerate(pool.map(one, jobs), start=1):
            rows.append(row)
            print('%3d/%d %-7s margin %.1f wind %.1f run %d: %s path_min %.1f looped %s apogee %d' % (
                count, len(jobs), row['law'], row['margin_cal'], row['wind_ms'], row['run'], row['outcome'],
                row['path_min_deg'], row['looped'], row['apogee_m']), flush=True)
    _write(os.path.join(args.out, 'runs.csv'), rows, _COLUMNS)
    lines = summary(rows)
    _write(os.path.join(args.out, 'summary.csv'), lines, tuple(lines[0]))
    if not render(lines, [law for law, _ in laws], os.path.join(args.out, 'envelope')):
        print('plotly does not import here: no figure (run with the plotly venv)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
