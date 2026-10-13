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
present (hold), `boost_law` set (lean).

On boards (BC4): `--boards taster=192.168.102.152:/dev/ttyACM0 TMS-7F=192.168.102.162:/dev/ttyACM1:no-power` flies
the same grid in realtime HITL, each board taking the next flight from one queue, the laws by name (`--laws passive
hold lean`: hitl_host.py `--boost-law` over each board's own config; `lean@10` adds a 10-degree boost ceiling,
`--boost-limit`); every flight goes through
tools/hitl_collect_host.sh (pulled, cut) into <outdir>/flights/<law>_m<margin>_w<wind>_r<run>/, plots deferred
(plotting during another board's realtime flight would load the host). `no-power` is a board whose servo rail is
off (hitl_host.py --no-power). `--scenarios` adds the 12-scenario matrix (tools/hitl_matrix_host.sh's table) at the
stack's own margin, per law. Writes <outdir>/runs.csv and summary.csv (';'), and, where plotly imports,
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
import threading

_ROOT: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
_SITL: str = os.path.join(_ROOT, 'src', 'avionics', 'target', 'release', 'sitl')
_LANDED: re.Pattern = re.compile(r'^LANDED miss=([\d.]+) inside=(True|False) aloft=([\d.]+) apogee=(\d+)$', re.M)
_BOOST: re.Pattern = re.compile(r'^BOOST path_min=(-?[\d.]+) tilt_max=([\d.]+) looped=(True|False) '
                                r'burnout=(-?[\d.]+)$', re.M)
_COLUMNS: tuple = ('law', 'margin_cal', 'wind_ms', 'run', 'outcome', 'path_min_deg', 'tilt_max_deg', 'looped',
                   'burnout_ms', 'apogee_m', 'aloft_s', 'miss_m', 'inside', 'board', 'scenario')
_COLLECT: str = os.path.join(_ROOT, 'tools', 'hitl_collect_host.sh')
# tools/hitl_matrix_host.sh's scenario table: name, noise, wind, its direction, spike
_MATRIX: tuple = (('noise05', 0.05, 0.0, 210.0, False), ('noise10', 0.10, 0.0, 210.0, False),
                  ('noise25', 0.25, 0.0, 210.0, False), ('noise50', 0.50, 0.0, 210.0, False),
                  ('noise100', 1.00, 0.0, 210.0, False), ('wind00', 0.10, 0.0, 210.0, False),
                  ('wind03', 0.10, 3.0, 210.0, False), ('wind06', 0.10, 6.0, 210.0, False),
                  ('wind09', 0.10, 9.0, 210.0, False), ('wind12', 0.10, 12.0, 210.0, False),
                  ('corner_spike', 0.10, 0.0, 210.0, True), ('corner_stress', 0.50, 12.0, 210.0, True))
# the wind blows toward east and the stack's dorsal side faces east: the wind meets the folded wings' plane
_WIND_TOWARD_DEG: float = 90.0
_NOISE: float = 0.10  # the matrix's nominal sensor noise


def _score(out: str) -> dict:
    """A flight's score from hitl_host.py's output: the BOOST and LANDED lines (-1 where a line is missing)."""
    landed, burn = _LANDED.search(out), _BOOST.search(out)
    return {'outcome': 'done' if '\nDONE' in '\n' + out else ('timeout' if 'TIMEOUT' in out else 'none'),
            'path_min_deg': float(burn.group(1)) if burn else -1.0,
            'tilt_max_deg': float(burn.group(2)) if burn else -1.0,
            'looped': burn is not None and burn.group(3) == 'True',
            'burnout_ms': float(burn.group(4)) if burn else -1.0,
            'apogee_m': int(landed.group(4)) if landed else -1,
            'aloft_s': float(landed.group(3)) if landed else -1.0,
            'miss_m': float(landed.group(1)) if landed else -1.0,
            'inside': landed is not None and landed.group(2) == 'True'}


def fly_board(board: tuple, job: dict, stack: str, glider_g: int, out: str) -> dict:
    """
    One realtime HITL flight of `job` on `board` (id, ip, console, servo rail off) through hitl_collect_host.sh;
    its row (`_COLUMNS`), scored from the flight's host.log. A flight that never flew reads -1, outcome none.
    """
    name, ip, console, no_power = board
    law, _, limit = job['law'].partition('@')  # lean@10: the lean law under a 10-degree boost ceiling
    flags = ['--boost', '6dof', '--stack', stack, '--seed', str(job['run']), '--boost-law', law]
    if limit:
        flags += ['--boost-limit', limit]
    if job['margin_cal'] is not None:
        flags += ['--stack-margin', str(job['margin_cal'])]
    if no_power:
        flags.append('--no-power')
    scenario = job['scenario'] or 'm%g_w%g_r%d' % (job['margin_cal'], job['wind_ms'], job['run'])
    folder = os.path.join(out, 'flights', job['law'])
    environment = dict(os.environ, PORT=console, HOST_FLAGS=' '.join(flags), PLOTLY='/nonexistent')
    command = ['bash', _COLLECT, name, ip, 'F15', scenario, str(job['noise']), str(job['wind_ms']),
               str(job['wind_dir']), str(job['spike']), folder, str(glider_g)]
    verdict = subprocess.run(command, capture_output=True, text=True, timeout=1800, env=environment).stdout.strip()
    try:
        with open(os.path.join(folder, scenario, 'host.log')) as handle:
            log = handle.read()
    except OSError:
        log = ''
    row = {'law': job['law'], 'margin_cal': job['margin_cal'] if job['margin_cal'] is not None else -1.0,
           'wind_ms': job['wind_ms'], 'run': job['run'], 'board': name, 'scenario': scenario}
    row.update(_score(log))
    if not verdict.startswith('OK'):
        row['outcome'] = 'failed'
    return row


def fly(config: str, law: str, stack: str, glider_g: int, margin: float, wind: float, run: int, port: int) -> dict:
    """One SITL flight on `port`; its row (`_COLUMNS`). A flight with no LANDED or BOOST line reads -1."""
    command = [sys.executable, os.path.join(_ROOT, 'tools', 'hitl_host.py'), '--sitl', _SITL, '--config', config,
               '--motor', 'F15', '--glider-g', str(glider_g), '--noise', str(_NOISE), '--wind', str(wind),
               '--wind-dir', str(_WIND_TOWARD_DEG), '--seed', str(run), '--sitl-port', str(port), '--boost', '6dof',
               '--stack', stack, '--stack-margin', str(margin)]
    out = subprocess.run(command, capture_output=True, text=True, timeout=900).stdout
    row = {'law': law, 'margin_cal': margin, 'wind_ms': wind, 'run': run, 'board': 'sitl', 'scenario': ''}
    row.update(_score(out))
    return row


def summary(rows: list) -> list:
    """
    Per law, margin and wind: the flights, the loops, the median path minimum, lean, apogee and time aloft. The grid's
    flights only: a matrix scenario's (`scenario` set by --scenarios) goes to `matrix_summary`.
    """
    rows = [row for row in rows if row['margin_cal'] >= 0]
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


def matrix_summary(rows: list) -> list:
    """Per law, over the matrix's scenarios (the stack's own margin): flights, loops, median miss, in the zone."""
    rows = [row for row in rows if row['margin_cal'] < 0 and row['outcome'] == 'done']
    out = []
    for law in sorted({row['law'] for row in rows}):
        cell = [row for row in rows if row['law'] == law]
        out.append({'law': law, 'flights': len(cell), 'loops': sum(row['looped'] for row in cell),
                    'median_path_min_deg': round(statistics.median(row['path_min_deg'] for row in cell), 1),
                    'median_miss_m': round(statistics.median(row['miss_m'] for row in cell), 1),
                    'in_zone': sum(row['inside'] for row in cell),
                    'median_apogee_m': statistics.median(row['apogee_m'] for row in cell)})
    return out


def _write(path: str, rows: list, columns: tuple) -> None:
    """`rows` as a ';' CSV, numeric tokens verbatim."""
    with open(path, 'w') as handle:
        handle.write(';'.join(columns) + '\n')
        for row in rows:
            handle.write(';'.join(str(row[column]) for column in columns) + '\n')


def render(lines: list, laws: list, out: str, where: str = 'over the SITL') -> bool:
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
    figure.update_layout(title='Boost envelope %s: the 6-DoF stack (tms7d), F15, crosswind against the folded wings'
                               % where, height=900, width=600 * len(laws), template='plotly_white')
    figure.write_html(out + '.html', include_plotlyjs='cdn')
    try:
        figure.write_image(out + '.svg')
    except Exception as error:  # kaleido or its browser missing: the html is the figure
        print('no svg: %s' % error)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--laws', nargs='+', required=True, metavar='NAME=CONFIG',
                        help='each law flown, by name and its board config: passive=..., hold=..., lean=... (on '
                             '--boards: the names alone, passive hold lean)')
    parser.add_argument('--boards', nargs='+', metavar='ID=IP:CONSOLE[:no-power]',
                        help='fly realtime HITL on these boards instead of the SITL, one queue')
    parser.add_argument('--scenarios', action='store_true',
                        help="--boards: also the 12-scenario matrix per law, at the stack's own margin")
    parser.add_argument('-o', '--out', required=True)
    parser.add_argument('--stack', default='tms7d')
    parser.add_argument('--glider-g', type=int, default=287)
    parser.add_argument('--margins', type=float, nargs='+', default=[0.2, 0.5, 0.8, 1.0, 1.5, 2.0])
    parser.add_argument('--winds', type=float, nargs='+', default=[0.0, 3.0, 4.5, 6.0])
    parser.add_argument('--runs', type=int, default=2)
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    if args.boards:
        return _boards(args)
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


def _boards(args: argparse.Namespace) -> int:
    """
    The grid (and with --scenarios the matrix) flown on --boards, one queue: each board's thread takes the next flight.
    runs.csv is rewritten after every flight, so a run cut short keeps what it flew.
    """
    boards = []
    for spec in args.boards:
        name, rest = spec.split('=', 1)
        parts = rest.split(':')
        boards.append((name, parts[0], ':'.join(part for part in parts[1:] if part != 'no-power'),
                       'no-power' in parts[1:]))
    laws = [law.split('=', 1)[0] for law in args.laws]
    jobs: queue.Queue = queue.Queue()
    total = 0
    # run by run, so a run cut short has whole grids
    for run in range(1, args.runs + 1):
        for margin in args.margins:
            for wind in args.winds:
                for law in laws:
                    jobs.put({'law': law, 'margin_cal': margin, 'wind_ms': wind, 'wind_dir': _WIND_TOWARD_DEG,
                              'noise': _NOISE, 'spike': False, 'run': run, 'scenario': ''})
                    total += 1
    if args.scenarios:
        for name, noise, wind, direction, spike in _MATRIX:
            for law in laws:
                jobs.put({'law': law, 'margin_cal': None, 'wind_ms': wind, 'wind_dir': direction, 'noise': noise,
                          'spike': spike, 'run': 1, 'scenario': name})
                total += 1
    rows: list = []
    lock = threading.Lock()

    def worker(board: tuple) -> None:
        while True:
            try:
                job = jobs.get_nowait()
            except queue.Empty:
                return
            row = fly_board(board, job, args.stack, args.glider_g, args.out)
            with lock:
                rows.append(row)
                _write(os.path.join(args.out, 'runs.csv'), rows, _COLUMNS)
                print('%3d/%d %-7s %-8s %-14s wind %.1f: %s path_min %.1f looped %s apogee %d miss %.0f' % (
                    len(rows), total, board[0], row['law'], row['scenario'], row['wind_ms'], row['outcome'],
                    row['path_min_deg'], row['looped'], row['apogee_m'], row['miss_m']), flush=True)

    threads = [threading.Thread(target=worker, args=(board,)) for board in boards]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    lines = summary(rows)
    if lines:
        _write(os.path.join(args.out, 'summary.csv'), lines, tuple(lines[0]))
        if not render(lines, laws, os.path.join(args.out, 'envelope'), 'in realtime HITL (%s)' %
                      ', '.join(board[0] for board in boards)):
            print('plotly does not import here: no figure (run with the plotly venv)')
    matrix = matrix_summary(rows)
    if matrix:
        _write(os.path.join(args.out, 'matrix_summary.csv'), matrix, tuple(matrix[0]))
    failed = [row for row in rows if row['outcome'] != 'done']
    print('flown %d of %d, %d not done' % (len(rows) - len(failed), total, len(failed)))
    return 1 if failed or len(rows) < total else 0


if __name__ == '__main__':
    sys.exit(main())
