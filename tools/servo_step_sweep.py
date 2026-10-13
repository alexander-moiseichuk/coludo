#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The fins' step (`step_deg`: a grid of that many degrees about the neutral, left only past half a step and a degree;
crates/drivers sg90 `Servo::set_angle`) against what it costs and what it buys: each fin's writes, the moves among
them (a write that changes the angle: a step of 0 rewrites held ones) and the degrees they move it, the servo rail's
energy, the landing.

  sitl    the Rust SITL (src/avionics/sitl flies the board's servos) lockstep on tools/hitl_host.py, every motor x
          step x scenario x seed: the miss, the zone and the apogee by the physics, the writes, moves and degrees by
          the SITL's DONE tally (hitl_host's SERVOS line). The SITL has no power rail: its energy is the board's fit
          on the moves and degrees (`_JOULES_PER_MOVE`, `_JOULES_PER_DEGREE`).
  board   tools/hitl_collect_host.sh flights from a board, `<dir>/<scenario>_s<step>_r<run>/flight/flight/`: each fin's
          writes, moves and degrees from its servo_*.csv, the rail's energy integrated from power_ina226.csv, both
          from the launch (t = 0) to DONE, and the miss as tools/hitl_matrix_compare measures it; then the energy's
          least-squares fit, joules per move and per degree, over every flight given.

  ~/.local/share/pipx/venvs/plotly/bin/python tools/servo_step_sweep.py sitl --sitl src/avionics/target/release/sitl \
        --config board.json -o <out> [--motors F15 E16] [--steps 0 1 2 3 5] [--seeds 20] [--workers 12]
  ~/.local/share/pipx/venvs/plotly/bin/python tools/servo_step_sweep.py board <dir> -o <out>
  ~/.local/share/pipx/venvs/plotly/bin/python tools/servo_step_sweep.py render <out>

sitl writes <out>/runs.csv and summary.csv (';'), board <out>/board.csv, and each, where plotly imports, its figure
(<out>/steps.html + .svg, <out>/board.html + .svg); render draws a sitl out flown where plotly is not. The
summary's miss change is each seed's against the same seed at step 1, over the scenarios whose miss
tools/hitl_matrix_compare gates (the wind and the low-noise families).
"""

import argparse
import concurrent.futures
import csv
import os
import queue
import re
import statistics
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hitl_matrix_compare  # noqa: E402

_ROOT: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
_FINS: tuple = ('servo_eleron_left', 'servo_eleron_right', 'servo_yaw')
_COUNTS: tuple = ('writes', 'moves', 'degrees')  # each fin's tally, the SITL's SERVOS order
_SCENARIOS: tuple = ('noise05', 'noise10', 'noise25', 'noise50', 'noise100', 'wind03', 'wind06', 'wind09')
"""The scenarios whose miss is gated: the summary's miss change and zone count are over these."""
_NOMINAL: tuple = tuple(scenario for members, tolerance in hitl_matrix_compare.FAMILIES.values()
                        if tolerance is not None for scenario in members if scenario in _SCENARIOS)
"""The taster's servo rail (2026-10-13, F15 noise10 and wind06 at steps 0, 1, 3 and 5, 16 flights): the energy over
the pad's idle draw, fitted per move and per degree commanded. A move costs nothing measurable; the degrees carry it.
The r4 matrix's step-1 flights alone could not tell the two apart (their moves and degrees rise together), and at
high noise the commanded degrees outrun what the horn follows, so this holds for the nominal scenarios."""
_JOULES_PER_MOVE: float = 0.0
_JOULES_PER_DEGREE: float = 0.00246
_LANDED: re.Pattern = re.compile(r'^LANDED miss=([\d.]+) inside=(True|False) aloft=([\d.]+) apogee=(\d+)$', re.M)
_SERVOS: re.Pattern = re.compile(r'^SERVOS (.*)$', re.M)
_BOARD_FLIGHT: re.Pattern = re.compile(r'^(\w+?)_s(\d+)_r(\d+)$')
_PAD_S: tuple = (-15.0, -1.0)  # the pad window the idle draw is read over, s from the launch


def _scenario(name: str) -> tuple:
    """`name`'s matrix row: (name, noise, wind, toward-bearing, spike)."""
    return next(row for row in hitl_matrix_compare.SCENARIOS if row[0] == name)


def fly(sitl: str, config: str, motor: str, step: int, scenario: str, seed: int, port: int) -> dict:
    """One SITL flight on `port`; its row. A flight with no LANDED or SERVOS line reads -1."""
    name, noise, wind, direction, spike = _scenario(scenario)
    command = [sys.executable, os.path.join(_ROOT, 'tools', 'hitl_host.py'), '--sitl', sitl, '--config', config,
               '--motor', motor, '--glider-g', str(hitl_matrix_compare._GLIDER_G), '--noise', str(noise), '--wind',
               str(wind), '--wind-dir', str(direction), '--seed', str(seed), '--sitl-port', str(port),
               '--servo-step', str(step)] + (['--spike'] if spike else [])
    out = subprocess.run(command, capture_output=True, text=True, timeout=900).stdout
    landed, servos = _LANDED.search(out), _SERVOS.search(out)
    row = {'motor': motor, 'step': step, 'scenario': name, 'seed': seed, 'done': '\nDONE' in '\n' + out,
           'miss_m': float(landed.group(1)) if landed else -1.0,
           'inside': landed is not None and landed.group(2) == 'True',
           'apogee_m': int(landed.group(4)) if landed else -1}
    tally = dict(item.split('=') for item in servos.group(1).split()) if servos else {}
    for fin in _FINS:
        for what, value in zip(_COUNTS, tally.get(fin, '-1/-1/-1').split('/')):
            row[fin + '_' + what] = int(value)
    for what in _COUNTS:
        row[what] = sum(row[fin + '_' + what] for fin in _FINS)
    row['energy_j'] = round(_JOULES_PER_MOVE * row['moves'] + _JOULES_PER_DEGREE * row['degrees'], 1)
    return row


def summary(rows: list) -> list:
    """Per motor and step, over the nominal scenarios: the mean writes, moves, degrees and energy, the miss change
    against step 1 (each seed against itself: median, 10th and 90th percentile), the flights in the zone."""
    base = {(row['motor'], row['scenario'], row['seed']): row['miss_m'] for row in rows if row['step'] == 1}
    out = []
    for motor, step in sorted({(row['motor'], row['step']) for row in rows}):
        cell = [row for row in rows if (row['motor'], row['step']) == (motor, step) and row['scenario'] in _NOMINAL]
        changes = sorted(row['miss_m'] - base[(motor, row['scenario'], row['seed'])] for row in cell
                         if (motor, row['scenario'], row['seed']) in base)
        out.append({'motor': motor, 'step': step, 'flights': len(cell),
                    'writes': round(statistics.mean(row['writes'] for row in cell)),
                    'moves': round(statistics.mean(row['moves'] for row in cell)),
                    'degrees': round(statistics.mean(row['degrees'] for row in cell)),
                    'energy_j': round(statistics.mean(row['energy_j'] for row in cell), 1),
                    'miss_change_median_m': round(statistics.median(changes), 1) if changes else '',
                    'miss_change_p10_m': round(changes[len(changes) // 10], 1) if changes else '',
                    'miss_change_p90_m': round(changes[9 * len(changes) // 10], 1) if changes else '',
                    'in_zone': sum(row['inside'] for row in cell)})
    return out


def board_flight(cut: str) -> dict:
    """One board flight's cut (`.../flight/flight`): the writes, moves, degrees and energy from the launch to DONE,
    the pad's idle draw, the miss."""
    def rows(stream: str) -> list:
        with open(os.path.join(cut, stream + '.csv'), newline='') as handle:
            return list(csv.DictReader(handle, delimiter=';'))

    stages = rows('sequencer')
    done = [float(row['t_s']) for row in stages if row['stage'] == 'done']
    end = done[0] if done else float(stages[-1]['t_s'])
    out = {'flight_s': round(end, 1)}
    for fin in _FINS:
        commands = [row for row in rows(fin) if row['done'] == '0']
        before = [int(row['angle']) for row in commands if float(row['t_s']) < 0.0]
        angle = before[-1] if before else None
        writes = moves = degrees = 0
        for row in commands:
            if 0.0 <= float(row['t_s']) <= end:
                moved = abs(int(row['angle']) - angle) if angle is not None else 0
                writes, moves, degrees = writes + 1, moves + (moved > 0), degrees + moved
                angle = int(row['angle'])
        out[fin + '_writes'], out[fin + '_moves'], out[fin + '_degrees'] = writes, moves, degrees
    for what in _COUNTS:
        out[what] = sum(out[fin + '_' + what] for fin in _FINS)
    power = [(float(row['t_s']), float(row['power_mw']) / 1000.0) for row in rows('power_ina226')]
    energy = sum((t - earlier) * (watts + before) / 2.0 for (earlier, before), (t, watts) in zip(power, power[1:])
                 if 0.0 <= earlier and t <= end)
    pad = [watts for t, watts in power if _PAD_S[0] <= t <= _PAD_S[1]]
    out['pad_w'] = round(statistics.mean(pad), 3) if pad else -1.0
    out['energy_j'] = round(energy, 1)
    out['motion_j'] = round(energy - out['pad_w'] * end, 1) if pad else -1.0
    found = hitl_matrix_compare.metrics(hitl_matrix_compare.hitl_overlay.rust_flight(os.path.dirname(cut), 'board'))
    out['miss_m'] = round(found['miss_m'], 1) if found['miss_m'] is not None else -1.0
    out['inside'] = bool(found['inside'])
    return out


def energy_fit(rows: list) -> tuple:
    """The least-squares (joules per move, joules per degree) through the origin of the flights' motion energy."""
    moves_squared = sum(row['moves'] ** 2 for row in rows)
    degrees_squared = sum(row['degrees'] ** 2 for row in rows)
    cross = sum(row['moves'] * row['degrees'] for row in rows)
    moves_joules = sum(row['moves'] * row['motion_j'] for row in rows)
    degrees_joules = sum(row['degrees'] * row['motion_j'] for row in rows)
    determinant = moves_squared * degrees_squared - cross * cross
    return ((degrees_squared * moves_joules - cross * degrees_joules) / determinant,
            (moves_squared * degrees_joules - cross * moves_joules) / determinant)


def _write(path: str, rows: list) -> None:
    """`rows` as a ';' CSV, numeric tokens verbatim."""
    with open(path, 'w') as handle:
        handle.write(';'.join(rows[0]) + '\n')
        for row in rows:
            handle.write(';'.join(str(value) for value in row.values()) + '\n')


def _plotly() -> tuple | None:
    """plotly's graph_objects and make_subplots, or None where it does not import."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        return None
    return go, make_subplots


def _save(figure: object, out: str) -> None:
    """The figure as html, and as svg where kaleido works."""
    figure.write_html(out + '.html', include_plotlyjs='cdn')
    try:
        figure.write_image(out + '.svg')
    except Exception as error:  # kaleido or its browser missing: the html is the figure
        print('no svg: %s' % error)


def render(lines: list, rows: list, out: str) -> bool:
    """The sitl figure: per motor, the energy and writes by step, each seed's miss change, the zone count."""
    plotly = _plotly()
    if plotly is None:
        return False
    go, make_subplots = plotly
    motors = sorted({line['motor'] for line in lines})
    titles = []
    for motor in motors:
        titles += ['%s: energy (J), writes and moves per flight' % motor, '%s: miss change against step 1, m' % motor,
                   '%s: flights in the zone (of %d)' % (motor, lines[0]['flights'])]
    figure = make_subplots(rows=len(motors), cols=3, subplot_titles=titles, horizontal_spacing=0.07,
                           vertical_spacing=0.16, specs=[[{'secondary_y': True}, {}, {}]] * len(motors))
    for index, motor in enumerate(motors, start=1):
        mine = [line for line in lines if line['motor'] == motor]
        steps = ['step %d' % line['step'] for line in mine]
        figure.add_trace(go.Bar(x=steps, y=[line['energy_j'] for line in mine], name='energy, J',
                                marker_color='#d62728', showlegend=index == 1,
                                text=[line['energy_j'] for line in mine]),
                         row=index, col=1, secondary_y=False)
        for field, colour in (('writes', '#1f77b4'), ('moves', '#ff7f0e')):
            figure.add_trace(go.Scatter(x=steps, y=[line[field] for line in mine], name=field, mode='lines+markers',
                                        marker_color=colour, showlegend=index == 1),
                             row=index, col=1, secondary_y=True)
        base = {(row['scenario'], row['seed']): row['miss_m'] for row in rows
                if row['motor'] == motor and row['step'] == 1}
        for line in mine:
            changes = [row['miss_m'] - base[(row['scenario'], row['seed'])] for row in rows
                       if row['motor'] == motor and row['step'] == line['step'] and row['scenario'] in _NOMINAL]
            figure.add_trace(go.Box(y=changes, name='step %d' % line['step'], marker_color='#2ca02c',
                                    showlegend=False, boxpoints='outliers'), row=index, col=2)
        figure.add_trace(go.Bar(x=steps, y=[line['in_zone'] for line in mine], marker_color='#9467bd',
                                showlegend=False, text=[line['in_zone'] for line in mine]), row=index, col=3)
    figure.update_layout(title='The fins\' step over the SITL: the grid with hysteresis, the nominal scenarios x '
                               'seeds (energy: the taster\'s fit)', height=430 * len(motors), width=1500,
                         template='plotly_white')
    _save(figure, out)
    return True


def render_board(rows: list, out: str) -> bool:
    """The board figure: each flight's measured motion energy, moves and degrees by step, per scenario."""
    plotly = _plotly()
    if plotly is None:
        return False
    go, make_subplots = plotly
    figure = make_subplots(rows=1, cols=3, subplot_titles=('servo rail energy over the pad\'s idle, J',
                                                           'moves (writes that changed the angle)',
                                                           'degrees moved'), horizontal_spacing=0.07)
    for scenario in sorted({row['scenario'] for row in rows}):
        mine = sorted((row for row in rows if row['scenario'] == scenario), key=lambda row: row['step'])
        for column, field in enumerate(('motion_j', 'moves', 'degrees'), start=1):
            figure.add_trace(go.Scatter(x=[row['step'] for row in mine], y=[row[field] for row in mine],
                                        mode='markers', name=scenario, legendgroup=scenario,
                                        showlegend=column == 1, marker={'size': 10}), row=1, col=column)
    figure.update_xaxes(title_text='step_deg', tickvals=sorted({row['step'] for row in rows}))
    figure.update_layout(title='The fins\' step on the board (HITL): measured from launch to DONE', height=450,
                         width=1500, template='plotly_white')
    _save(figure, out)
    return True


def _sitl(args: argparse.Namespace) -> int:
    """The sitl command: fly, write, render."""
    os.makedirs(args.out, exist_ok=True)
    config = os.path.abspath(args.config)
    jobs = [(motor, step, scenario, seed) for motor in args.motors for scenario in _SCENARIOS
            for seed in range(1, args.seeds + 1) for step in args.steps]
    ports: queue.Queue = queue.Queue()
    for index in range(args.workers):
        ports.put(1400 + index)

    def one(job: tuple) -> dict:
        port = ports.get()
        try:
            motor, step, scenario, seed = job
            return fly(os.path.abspath(args.sitl), config, motor, step, scenario, seed, port)
        finally:
            ports.put(port)

    rows = []
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        for count, row in enumerate(pool.map(one, jobs), start=1):
            rows.append(row)
            if count % 50 == 0 or count == len(jobs):
                print('%d/%d flown' % (count, len(jobs)), flush=True)
    failed = [row for row in rows if not row['done'] or row['writes'] < 0]
    _write(os.path.join(args.out, 'runs.csv'), rows)
    _summarise(rows, args.out)
    if failed:
        print('%d flights without DONE or a SERVOS line' % len(failed))
    return 1 if failed else 0


def _summarise(rows: list, out: str) -> None:
    """The summary of `rows` to <out>/summary.csv and stdout, and the figure."""
    lines = summary(rows)
    _write(os.path.join(out, 'summary.csv'), lines)
    for line in lines:
        print(';'.join(str(value) for value in line.values()))
    if not render(lines, rows, os.path.join(out, 'steps')):
        print('plotly does not import here: no figure (run with the plotly venv)')


def _render(args: argparse.Namespace) -> int:
    """The render command: the summary and the figure again from <out>/runs.csv (flown where plotly is not)."""
    integers = ('step', 'seed', 'apogee_m') + _COUNTS + tuple(fin + '_' + what for fin in _FINS for what in _COUNTS)
    with open(os.path.join(args.out, 'runs.csv'), newline='') as handle:
        rows = list(csv.DictReader(handle, delimiter=';'))
    for row in rows:
        for key in integers:
            row[key] = int(row[key])
        row['miss_m'], row['energy_j'] = float(row['miss_m']), float(row['energy_j'])
        row['done'], row['inside'] = row['done'] == 'True', row['inside'] == 'True'
    _summarise(rows, args.out)
    return 0


def _board(args: argparse.Namespace) -> int:
    """The board command: tally the cuts, fit the energy, render."""
    rows = []
    for name in sorted(os.listdir(args.dir)):
        matched = _BOARD_FLIGHT.match(name)
        cut = os.path.join(args.dir, name, 'flight', 'flight')
        if not matched or not os.path.isdir(cut):
            continue
        rows.append({'scenario': matched.group(1), 'step': int(matched.group(2)), 'run': int(matched.group(3)),
                     **board_flight(cut)})
    if not rows:
        print('no <scenario>_s<step>_r<run>/flight/flight cuts under %s' % args.dir)
        return 1
    os.makedirs(args.out, exist_ok=True)
    _write(os.path.join(args.out, 'board.csv'), rows)
    for row in rows:
        print('%-8s step %d run %d: %5d writes %5d moves %6d deg %6.1f J (pad %.3f W) miss %.1f' % (
            row['scenario'], row['step'], row['run'], row['writes'], row['moves'], row['degrees'], row['motion_j'],
            row['pad_w'], row['miss_m']))
    per_move, per_degree = energy_fit(rows)
    print('energy fit: %.2f mJ a move + %.3f mJ a degree' % (per_move * 1000, per_degree * 1000))
    if not render_board(rows, os.path.join(args.out, 'board')):
        print('plotly does not import here: no figure (run with the plotly venv)')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    commands = parser.add_subparsers(dest='command', required=True)
    flown = commands.add_parser('sitl', help='fly the step grid through the Rust SITL')
    flown.add_argument('--sitl', required=True, help='src/avionics/target/release/sitl')
    flown.add_argument('--config', required=True, help='the board config JSON it runs')
    flown.add_argument('-o', '--out', required=True)
    flown.add_argument('--motors', nargs='+', default=['F15', 'E16'])
    flown.add_argument('--steps', type=int, nargs='+', default=[0, 1, 2, 3, 5])
    flown.add_argument('--seeds', type=int, default=20)
    flown.add_argument('--workers', type=int, default=12)
    measured = commands.add_parser('board', help="tally a board's collected HITL flights")
    measured.add_argument('dir', help='hitl_collect_host.sh outdir: <scenario>_s<step>_r<run>/ per flight')
    measured.add_argument('-o', '--out', required=True)
    drawn = commands.add_parser('render', help="a sitl out's summary and figure again from its runs.csv")
    drawn.add_argument('out')
    args = parser.parse_args()
    return {'sitl': _sitl, 'board': _board, 'render': _render}[args.command](args)


if __name__ == '__main__':
    sys.exit(main())
