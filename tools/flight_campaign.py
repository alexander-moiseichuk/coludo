"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Compare a WHOLE CAMPAIGN of flights in one view (findings §27.18).

Per-flight tools answer "how did this one go?". A campaign asks a different question -- "which
configuration is better, and is the difference real or just scatter?" -- and nothing answered it:
doc/sims/ is per-run HTML plus hand-written README tables, and flight_kpi prints one block per capture.
That is precisely the question the catapult ladder generates, where the whole point of a repeatable
launcher is that run-to-run comparison MEANS something.

So this aggregates the existing per-flight measurements (glide_polar's L/D, flight_kpi's G envelope,
the baro traces) into one table plus an overlay, and -- importantly -- reports the SPREAD within a
group. Three launches of the same airframe establish the noise floor; a difference between two
airframes only counts if it clears it.

  python3 tools/flight_campaign.py a:run1.txt a:run2.txt b:run3.txt -o campaign.html

Labels repeat on purpose: same label = same configuration = one group, and the group's mean +/- spread
is what gets compared.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_kpi  # noqa: E402
import flight_telemetry  # noqa: E402
import glide_polar  # noqa: E402
import preflight  # noqa: E402


def _spread(values: list) -> tuple:
    """(mean, half-spread) for a group; half-spread is 0 for a single run (no scatter measurable)."""
    if not values:
        return float('nan'), 0.0
    mean = sum(values) / len(values)
    return mean, (max(values) - min(values)) / 2.0


def measure(path: str) -> dict:
    """
    Everything comparable about one capture, reusing the per-flight tools rather than re-deriving.

    Args:
        path - the capture file.

    Returns:
        A dict of metrics; missing ones are None so a partial capture still lines up in the table.
    """
    with open(path) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    find = flight_telemetry.find_stream
    baro = find(streams, 'elevation', prefer='icp') or find(streams, 'elevation') or find(streams, 'altitude')
    result = {'apogee': None, 'duration': None, 'lift_drag': None, 'sink': None,
              'airspeed': None, 'peak_g': None, 'trace': ([], [])}
    if baro is not None:
        field = 'elevation' if 'elevation' in baro.fields else 'altitude'
        times, elevation = baro.column(field)
        if times:
            result['apogee'] = max(elevation)
            result['duration'] = times[-1] - times[0]
            result['trace'] = (times, elevation)
    glide = glide_polar.analyse(streams, None, None)
    if 'error' not in glide:
        result.update({'lift_drag': glide['lift_drag'], 'sink': glide['sink'],
                       'airspeed': glide['airspeed'], 'r2': glide.get('r2')})
    primary = find(streams, 'ax', 'ay', 'az', 'gx', prefer='lsm') or find(streams, 'ax', 'ay', 'az')
    peak, _when, count = flight_kpi._peak_g(primary)
    if count:
        result['peak_g'] = peak
    return result


def render(groups: dict, path: str) -> None:
    """Write the campaign overlay: every flight's altitude trace, coloured by group."""
    try:
        import plotly.graph_objects as go
        import plotly.io as pio
        from plotly.subplots import make_subplots
    except ImportError:
        print('  (no plotly -- table only; pip install plotly for the overlay)')
        return
    figure = make_subplots(rows=2, cols=1, vertical_spacing=0.10,
                           subplot_titles=('altitude (m) — every flight, coloured by configuration',
                                           'glide quality L/D — per flight (higher is better)'))
    palette = ['#1f77b4', '#2ca02c', '#d62728', '#9467bd', '#ff7f0e', '#17becf']
    for index, (label, runs) in enumerate(sorted(groups.items())):
        colour = palette[index % len(palette)]
        for number, run in enumerate(runs):
            times, elevation = run['trace']
            if times:
                figure.add_trace(go.Scatter(x=times, y=elevation, name='%s #%d' % (label, number + 1),
                                            legendgroup=label, line=dict(color=colour)), row=1, col=1)
        ratios = [r['lift_drag'] for r in runs if r['lift_drag'] is not None]
        if ratios:
            figure.add_trace(go.Bar(x=['%s #%d' % (label, i + 1) for i in range(len(ratios))], y=ratios,
                                    name=label, legendgroup=label, showlegend=False,
                                    marker=dict(color=colour)), row=2, col=1)
    figure.update_layout(height=900, title='flight campaign — configurations compared', hovermode='x unified')
    pio.write_html(figure, file=path, include_plotlyjs='cdn', auto_open=False)
    print('wrote %s' % path)


def main() -> int:
    parser = argparse.ArgumentParser(description='Compare a campaign of flights (repeat labels to group).')
    parser.add_argument('captures', nargs='+', help='LABEL:path.txt (repeat a LABEL to group runs)')
    parser.add_argument('-o', '--out', default=None, help='campaign overlay HTML')
    args = parser.parse_args()
    preflight.gate('campaign comparison')

    groups = {}
    for entry in args.captures:
        label, _, path = entry.rpartition(':')
        if not path:
            label, path = os.path.splitext(os.path.basename(entry))[0], entry
        if os.path.isdir(path):
            """
            A DIRECTORY is the natural mistake here, because flight_metrics.py takes one -- so the
            operator reasonably tries the same argument on this tool. os.path.exists() said yes and
            open() then raised IsADirectoryError with a traceback, which reads like a tool bug rather
            than a usage one. Name the difference instead: this tool groups labelled runs.
            """
            print('%s is a directory -- this tool takes LABEL:capture.txt entries, not a folder.\n'
                  '  try:  %s/*.txt        (or flight_metrics.py %s for a whole directory)'
                  % (path, path.rstrip('/'), path), file=sys.stderr)
            continue
        if not os.path.exists(path):
            print('missing: %s' % path, file=sys.stderr)
            continue
        groups.setdefault(label or os.path.basename(path), []).append(measure(path))
    if not groups:
        print('no readable captures', file=sys.stderr)
        return 1

    print('%-12s %5s  %8s  %9s  %7s  %8s  %7s'
          % ('config', 'runs', 'apogee m', 'duration s', 'L/D', 'sink m/s', 'peak g'))
    print('-' * 72)
    ranking = []
    for label, runs in sorted(groups.items()):
        cells = []
        for key in ('apogee', 'duration', 'lift_drag', 'sink', 'peak_g'):
            mean, half = _spread([r[key] for r in runs if r[key] is not None])
            cells.append((mean, half))
        print('%-12s %5d  %8s  %9s  %7s  %8s  %7s' % (
            label, len(runs),
            _cell(cells[0]), _cell(cells[1]), _cell(cells[2]), _cell(cells[3]), _cell(cells[4])))
        if not math.isnan(cells[2][0]):
            ranking.append((cells[2][0], cells[2][1], label, len(runs)))

    """
    A difference only counts if it clears the SCATTER. With repeat runs the half-spread is a measured
    noise floor, so say plainly whether the gap between the best two configurations is bigger than it --
    otherwise a 0.2 L/D "improvement" from single runs gets believed.
    """
    if len(ranking) > 1:
        ranking.sort(reverse=True)
        best, second = ranking[0], ranking[1]
        gap = best[0] - second[0]
        noise = max(best[1], second[1])
        print()
        print('best glide : %s (L/D %.2f) over %s (L/D %.2f) -- gap %.2f'
              % (best[2], best[0], second[2], second[0], gap))
        """
        A group flown ONCE has no measurable scatter, and comparing a gap against a spread of zero makes
        ANY difference look significant -- a 0.01 L/D gap was cheerfully reported as REAL while testing.
        So the repeat count gates the verdict before the arithmetic does: without repeats on BOTH sides
        there is simply nothing to judge against.
        """
        if best[3] < 2 or second[3] < 2:
            print('  UNPROVEN: %s has %d run(s) and %s has %d -- with no repeats there is no measured'
                  % (best[2], best[3], second[2], second[3]))
            print('            scatter to compare the gap against. Fly each configuration >=3 times.')
        elif gap > noise:
            print('  REAL: the gap (%.2f) exceeds the run-to-run spread (%.2f).' % (gap, noise))
        else:
            print('  NOT PROVEN: the gap (%.2f) is inside the run-to-run spread (%.2f) -- same, so far.'
                  % (gap, noise))
    if args.out:
        render(groups, args.out)
    return 0


def _cell(pair: tuple) -> str:
    """Format a (mean, half-spread) pair; '-' when the metric is absent from every run in the group."""
    mean, half = pair
    if math.isnan(mean):
        return '-'
    return '%.1f' % mean if half == 0 else '%.1f±%.1f' % (mean, half)


if __name__ == '__main__':
    sys.exit(main())
