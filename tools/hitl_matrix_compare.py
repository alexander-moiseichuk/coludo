#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The Rust board's HITL matrix against plan B's control stack on the same physics (M14, doc/specs/avionics-port.md).

The reference is plan B's own flight code at its tag -- its Controller, stage machine and flight task -- run by
tools/planb_sitl.py behind the same sim-link and the same tools/hitl_host.py frames the Rust board and the Rust SITL
meet, lockstep, on today's src/glider/sim_model.py. Not tools/virtual_flight.py: its loop feeds no wind estimator and
hands its law another GNSS than a board's (the total speed, clean), so in the wind its misses were the harness's. Not
the board matrices in doc/sims either: they flew earlier sims (the measured polar and the endgame work came after).
Each scenario is flown with several seeds (hitl_host --seed): the seeds' spread is the stack's own run-to-run
variation on that scenario, and a seed's frames are the same for both stacks while their flights agree.

  reference  plan B's matrix through its SITL, each seed a run: `<out>/s<seed>/<motor>/<scenario>/flight/`
  sitl       the Rust matrix through the Rust SITL (src/avionics/sitl), the same layout
  compare    Rust runs -- the SITL's seeds, or tools/hitl_matrix_host.sh outdirs `<run>/<motor>/<scenario>/flight/`
             from the board -- against the reference's seeds, scenario by scenario; a table to stdout, `--json` and
             `--markdown` to files, `--figure` the miss and apogee against the seeds' range (plotly)

What is gated, each Rust flight against plan B's seeds (`low..high` their range):
  * the stage machine, every scenario: the apogee within 3 % of the range, GLIDING (the apogee detection) within
    0.5 s of it, and DONE reached;
  * the touchdown miss, the wind and the low-noise families only: within the range widened by the family's own
    run-to-run spread, measured over the post-audit board matrix's three runs (doc/sims/TMS-7-postaudit_regression:
    wind median 7.7 m, low noise 11.9 m). The high-noise and corner families are reported, not gated: their misses
    spread 80-100 m run to run, so no single flight says anything there.
Reported, not gated: LANDING's time, the peak |a|, in-zone, the left eleron's travel per glide second.

  tools/hitl_matrix_compare.py reference --config board.json -o /tmp/planb
  tools/hitl_matrix_compare.py sitl --binary src/avionics/target/release/sitl --config board.json -o /tmp/sitl
  tools/hitl_matrix_compare.py compare --planb /tmp/planb s1=/tmp/sitl/s1 r1=/tmp/m14/r1 --markdown out.md
"""

import argparse
import concurrent.futures
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hitl_overlay  # noqa: E402

"""
The 12 scenarios: name, noise, wind (m/s), its toward-bearing (deg), spike. hitl_matrix.sh's table and
hitl_matrix_host.sh's (src/control/test/test_tools.py checks all three agree).
"""
SCENARIOS: tuple = (
    ('noise05', 0.05, 0.0, 210.0, False),
    ('noise10', 0.10, 0.0, 210.0, False),
    ('noise25', 0.25, 0.0, 210.0, False),
    ('noise50', 0.50, 0.0, 210.0, False),
    ('noise100', 1.00, 0.0, 210.0, False),
    ('wind00', 0.10, 0.0, 210.0, False),
    ('wind03', 0.10, 3.0, 210.0, False),
    ('wind06', 0.10, 6.0, 210.0, False),
    ('wind09', 0.10, 9.0, 210.0, False),
    ('wind12', 0.10, 12.0, 210.0, False),
    ('corner_spike', 0.10, 0.0, 210.0, True),
    ('corner_stress', 0.50, 12.0, 210.0, True),
)

"""Each scenario's family and the family's miss tolerance in metres (None: the miss is not gated)."""
FAMILIES: dict = {
    'wind': (('wind00', 'wind03', 'wind06', 'wind09', 'wind12'), 7.7),
    'low noise': (('noise05', 'noise10', 'noise25'), 11.9),
    'high noise': (('noise50', 'noise100'), None),
    'corner': (('corner_spike', 'corner_stress'), None),
}

_MOTORS: tuple = ('E16', 'F15')
_APOGEE_FRACTION: float = 0.03
_GLIDING_S: float = 0.5
_GLIDER_G: int = 285
_PLANB_SITL: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'planb_sitl.py')


def family(scenario: str) -> tuple:
    """(family name, its miss tolerance m or None) of `scenario`."""
    for name, (members, tolerance) in FAMILIES.items():
        if scenario in members:
            return name, tolerance
    raise KeyError(scenario)


def _sitl_flight(job: tuple) -> str:
    """One SITL flight (binary, config, motor, scenario row, seed, out folder, port); the folder, or an error text."""
    binary, config, motor, (name, noise, wind, direction, spike), seed, folder, port = job
    arguments = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hitl_host.py'),
                 '--sitl', binary, '--config', config, '--sitl-port', str(port), '--motor', motor,
                 '--glider-g', str(_GLIDER_G), '--noise', str(noise), '--wind', str(wind), '--wind-dir', str(direction),
                 '--seed', str(seed), '--record', folder]
    if spike:
        arguments.append('--spike')
    done = subprocess.run(arguments, capture_output=True, text=True, timeout=600, check=False)
    ok = done.returncode == 0 and os.path.exists(os.path.join(folder, 'flight', 'sequencer.csv'))
    return folder if ok else 'FAILED %s: %s' % (folder, (done.stdout + done.stderr)[-300:])


def sitl(binary: str, config: str, out: str, seeds: int, motors: tuple) -> int:
    """Fly the matrix against the SITL `binary` (the Rust one, or tools/planb_sitl.py) into `out`; 0 when every flight
    reached DONE and wrote its streams."""
    jobs = []
    for motor in motors:
        for row in SCENARIOS:
            for seed in range(1, seeds + 1):
                folder = os.path.join(out, 's%d' % seed, motor, row[0], 'flight')
                jobs.append((binary, config, motor, row, seed, folder, 1300 + len(jobs)))
    with concurrent.futures.ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        results = list(pool.map(_sitl_flight, jobs))
    failed = [result for result in results if result.startswith('FAILED')]
    for line in failed:
        print(line)
    print('sitl: %d of %d flights -> %s' % (len(results) - len(failed), len(results), out))
    return 1 if failed else 0


def _glide_travel(flight: hitl_overlay.Flight) -> float | None:
    """The left eleron's travel per second of glide (deg/s), GLIDING to the series' end."""
    gliding = flight.milestones()['gliding_s']
    times, angles = flight.fin
    pairs = [(t, angle) for t, angle in zip(times, angles) if gliding is not None and t >= gliding]
    if len(pairs) < 2 or pairs[-1][0] <= pairs[0][0]:
        return None
    travel = sum(abs(later - earlier) for (_, earlier), (_, later) in zip(pairs, pairs[1:]))
    return travel / (pairs[-1][0] - pairs[0][0])


def metrics(flight: hitl_overlay.Flight) -> dict:
    """The compared numbers of one flight."""
    found = flight.milestones()
    return {'gliding_s': found['gliding_s'], 'landing_s': found['landing_s'], 'done': found['done_s'] is not None,
            'apogee_m': found['apogee_m'], 'peak_g': found['peak_g'], 'miss_m': found['miss_m'],
            'inside': found['inside'], 'travel_dps': _glide_travel(flight)}


def judge(scenario: str, rust: dict, planb: list) -> list:
    """
    The gates one Rust flight fails against plan B's seeds on `scenario` (empty when it passes).

    Args:
        scenario - the scenario's name (its family decides the miss gate).
        rust - the Rust flight's metrics.
        planb - the plan B seeds' metrics.

    Returns:
        One text per failed gate.
    """
    failures = []
    if not rust['done']:
        failures.append('no DONE')

    def outside(key: str, margin: float, relative: bool) -> str | None:
        values = [flight[key] for flight in planb if flight[key] is not None]
        if not values:
            return None
        if rust[key] is None:
            return '%s missing' % key
        low, high = min(values), max(values)
        widen = margin * (abs(low) + abs(high)) / 2 if relative else margin
        if low - widen <= rust[key] <= high + widen:
            return None
        return '%s %.1f outside plan B %.1f..%.1f (+-%.1f)' % (key, rust[key], low, high, widen)

    for failure in (outside('apogee_m', _APOGEE_FRACTION, True), outside('gliding_s', _GLIDING_S, False)):
        if failure:
            failures.append(failure)
    _, tolerance = family(scenario)
    if tolerance is not None:
        failure = outside('miss_m', tolerance, False)
        if failure:
            failures.append(failure)
    return failures


def compare(runs: dict, planb_dir: str, motors: tuple) -> dict:
    """Every Rust run's every flight against plan B's seeds; {rows, failures, missing}."""
    rows, failures, missing = [], [], []
    for motor in motors:
        for name, *_ in SCENARIOS:
            planb = []
            for seed in sorted(os.listdir(planb_dir)):
                folder = os.path.join(planb_dir, seed, motor, name, 'flight')
                if os.path.exists(os.path.join(folder, 'flight', 'sequencer.csv')):
                    planb.append(metrics(hitl_overlay.rust_flight(folder, seed)))
            if not planb:
                missing.append('%s/%s: no plan B reference' % (motor, name))
                continue
            for label, run in runs.items():
                folder = os.path.join(run, motor, name, 'flight')
                if not os.path.exists(os.path.join(folder, 'flight', 'sequencer.csv')):
                    missing.append('%s/%s: no flight in %s' % (motor, name, label))
                    continue
                rust = metrics(hitl_overlay.rust_flight(folder, label))
                failed = judge(name, rust, planb)
                rows.append({'motor': motor, 'scenario': name, 'family': family(name)[0], 'run': label,
                             'rust': rust, 'planb': planb, 'failed': failed})
                failures.extend('%s/%s %s: %s' % (motor, name, label, text) for text in failed)
    return {'rows': rows, 'failures': failures, 'missing': missing}


def _range(planb: list, key: str) -> str:
    """Plan B's seeds' `low..high` of `key`."""
    values = [flight[key] for flight in planb if flight[key] is not None]
    return '%.1f..%.1f' % (min(values), max(values)) if values else '-'


def table(result: dict) -> str:
    """The comparison as a markdown table, one row per Rust flight."""
    lines = ['| motor | scenario | run | miss Rust | miss plan B | apogee Rust | apogee plan B | GLIDING Rust | '
             'GLIDING plan B | LANDING Rust | LANDING plan B | verdict |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']

    def cell(value: float | None, unit: str = '') -> str:
        return '-' if value is None else '%.1f%s' % (value, unit)

    for row in result['rows']:
        rust, planb = row['rust'], row['planb']
        verdict = 'pass' if not row['failed'] else 'FAIL: ' + '; '.join(row['failed'])
        if row['family'] in ('high noise', 'corner') and not row['failed']:
            verdict = 'pass (miss not gated)'
        lines.append('| %s | %s | %s | %s%s | %s | %s | %s | %s | %s | %s | %s | %s |' % (
            row['motor'], row['scenario'], row['run'], cell(rust['miss_m']), ' (in)' if rust['inside'] else '',
            _range(planb, 'miss_m'), cell(rust['apogee_m']), _range(planb, 'apogee_m'), cell(rust['gliding_s']),
            _range(planb, 'gliding_s'), cell(rust['landing_s']), _range(planb, 'landing_s'), verdict))
    return '\n'.join(lines)


def figure(result: dict, out: str) -> bool:
    """
    The comparison as a figure (`<out>.html`, `<out>.svg`): per motor, each scenario's touchdown miss and apogee,
    plan B's seeds as a bar from the lowest to the highest with their median marked, each Rust run as its own marker.

    Returns:
        False when plotly does not import here (run with the plotly venv); True once the html is written.
    """
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError:
        return False
    motors = sorted({row['motor'] for row in result['rows']})
    runs = sorted({row['run'] for row in result['rows']})
    panels = make_subplots(rows=2, cols=len(motors), shared_xaxes=True, vertical_spacing=0.08,
                           subplot_titles=['%s: touchdown miss, m' % motor for motor in motors]
                           + ['%s: apogee, m' % motor for motor in motors])
    symbols = ('circle', 'diamond', 'square', 'triangle-up', 'x')
    for column, motor in enumerate(motors, start=1):
        rows = [row for row in result['rows'] if row['motor'] == motor]
        scenarios = list(dict.fromkeys(row['scenario'] for row in rows))
        for line, key in ((1, 'miss_m'), (2, 'apogee_m')):
            seeds = {row['scenario']: [flight[key] for flight in row['planb'] if flight.get(key) is not None]
                     for row in rows}
            low = [min(seeds[scenario]) if seeds[scenario] else None for scenario in scenarios]
            high = [max(seeds[scenario]) if seeds[scenario] else None for scenario in scenarios]
            middle = [sorted(seeds[scenario])[len(seeds[scenario]) // 2] if seeds[scenario] else None
                      for scenario in scenarios]
            panels.add_trace(go.Bar(x=scenarios, y=[None if a is None else b - a for a, b in zip(low, high)],
                                    base=low, marker_color='rgba(120,120,120,0.35)', name='plan B, seeds',
                                    legendgroup='planb', showlegend=column == 1 and line == 1), row=line, col=column)
            panels.add_trace(go.Scatter(x=scenarios, y=middle, mode='markers', marker={'symbol': 'line-ew-open',
                                        'size': 18, 'color': 'black'}, name='plan B, median', legendgroup='median',
                                        showlegend=column == 1 and line == 1), row=line, col=column)
            for index, run in enumerate(runs):
                values = {row['scenario']: row['rust'].get(key) for row in rows if row['run'] == run}
                panels.add_trace(go.Scatter(x=scenarios, y=[values.get(scenario) for scenario in scenarios],
                                            mode='markers', marker={'symbol': symbols[index % len(symbols)],
                                                                    'size': 10}, name='Rust ' + run,
                                            legendgroup=run, showlegend=column == 1 and line == 1),
                                 row=line, col=column)
    panels.update_layout(title='The Rust HITL matrix against plan B on the same physics', height=900,
                         width=700 * len(motors), template='plotly_white', barmode='overlay')
    panels.write_html(out + '.html', include_plotlyjs='cdn')
    try:
        panels.write_image(out + '.svg')
    except Exception as error:  # kaleido or its browser missing: the html is the figure
        print('no svg: %s' % error)
    return True


def main() -> int:
    """reference or compare; 0 on success (compare: every flight present and every gate passed)."""
    parser = argparse.ArgumentParser(description="The Rust HITL matrix against plan B's on the same physics")
    commands = parser.add_subparsers(dest='command', required=True)
    flown = commands.add_parser('reference', help="fly plan B's matrix through its SITL (tools/planb_sitl.py)")
    flown.add_argument('--config', required=True, help='the board config JSON it runs')
    flown.add_argument('-o', '--out', required=True)
    flown.add_argument('--seeds', type=int, default=3)
    flown.add_argument('--motors', default=','.join(_MOTORS))
    simulated = commands.add_parser('sitl', help='fly the Rust matrix against the SITL')
    simulated.add_argument('--binary', required=True, help='src/avionics/target/release/sitl')
    simulated.add_argument('--config', required=True, help='the board config JSON it runs')
    simulated.add_argument('-o', '--out', required=True)
    simulated.add_argument('--seeds', type=int, default=3)
    simulated.add_argument('--motors', default=','.join(_MOTORS))
    compared = commands.add_parser('compare', help='Rust runs against the reference')
    compared.add_argument('runs', nargs='+', help='label=<hitl_matrix_host.sh outdir root>')
    compared.add_argument('--planb', required=True, help='the reference directory')
    compared.add_argument('--motors', default=','.join(_MOTORS))
    compared.add_argument('--json', help='write the whole result here')
    compared.add_argument('--markdown', help='write the table here')
    compared.add_argument('--figure', help='draw the miss and apogee against plan B\'s seeds: <prefix>.html, .svg '
                          '(run with the plotly venv)')
    args = parser.parse_args()
    motors = tuple(motor for motor in args.motors.split(',') if motor)
    if args.command == 'reference':
        return sitl(_PLANB_SITL, os.path.abspath(args.config), args.out, args.seeds, motors)
    if args.command == 'sitl':
        return sitl(os.path.abspath(args.binary), os.path.abspath(args.config), args.out, args.seeds, motors)
    runs = dict(spec.split('=', 1) for spec in args.runs)
    result = compare(runs, args.planb, motors)
    text = table(result)
    print(text)
    for line in result['missing']:
        print('MISSING: ' + line)
    print('%d flights compared, %d gate failure(s), %d missing' % (len(result['rows']), len(result['failures']),
                                                                  len(result['missing'])))
    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(result, handle, indent=1)
    if args.markdown:
        with open(args.markdown, 'w') as handle:
            handle.write(text + '\n')
    if args.figure and not figure(result, args.figure):
        print('plotly does not import here: no figure (run with the plotly venv)')
    return 1 if result['failures'] or result['missing'] else 0


if __name__ == '__main__':
    sys.exit(main())
