#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Compare the software stacks behind HITL flights on the same board (plan B's MicroPython, the Rust avionics): what each
leaves in its own recordings, each aligned on its own BOOSTING entry and read from BOOSTING to DONE.

  * health.csv: the die temperature, the free heap and the CPU load. The columns share names, not meanings: plan B's
    mem_free is MicroPython's heap and its load its scheduler's, Rust's mem_free the whole 8-bit heap (internal and
    PSRAM) and its load the busier core's;
  * the cadence of its streams: the flight law's samples (flight.csv), the fin commands (plan B's fins.csv), the
    sensors it reads (the LSM6DSO32 and ICP-10111 streams, which HITL feeds).

The table goes to stdout (markdown). With -o, a figure of the health over the flight: <out>.html, and <out>.svg with
kaleido.

  tools/stack_compare.py 'plan B=<prefix>' 'Rust=<prefix>' [-o <out_prefix>]

A prefix names one session's files without the stream: `<dir>/<session>`, whose streams are
`<prefix>_<stream>.csv` (a plan B capture's names, and a Rust session's recordings). Plan B's uptime wraps at 2^30 us
(MicroPython's ticks), Rust's never; a Rust row's integrity wrapper is stripped.
"""
import argparse
import glob
import re
import statistics
import sys

UPTIME_WRAP: int = 1 << 30  # plan B's ticks_us wrap
_WRAPPED: re.Pattern = re.compile(r'^\{[0-9a-f]{8}\};(.*);<[0-9a-f]{8}>$')
_STREAMS: tuple = ('flight', 'fins', 'imu_lsm6dso32', 'baro_icp10111')
_HEALTH: tuple = ('temp', 'mem_free', 'load')
_BOOSTING: tuple = ('boosting', '2')
_GLIDING: tuple = ('gliding', '3')
_DONE: tuple = ('done', '5')


def read_stream(prefix: str, name: str) -> list:
    """
    One stream's rows as dicts, `t` its uptime unwrapped (us).

    Args:
        prefix - the session's files without the stream name.
        name - the stream (`flight`, `health`, ...).

    Returns:
        The rows, header columns as keys; [] when the file is missing. A row of another width than its header (a
        damaged line) is left out.
    """
    paths = glob.glob('%s_%s.csv' % (prefix, name))
    if not paths:
        return []
    rows, header, base, last = [], None, 0, None
    with open(paths[0], errors='replace') as stream:
        for line in stream.read().splitlines():
            match = _WRAPPED.match(line)
            cells = (match.group(1) if match else line).split(';')
            if cells[0] == 'uptime':
                header = cells
                continue
            if header is None or not cells[0].isdigit() or len(cells) != len(header):
                continue
            uptime = int(cells[0])
            if last is not None and uptime + base < last - UPTIME_WRAP // 2:
                base += UPTIME_WRAP
            row = dict(zip(header, cells))
            row['t'] = last = uptime + base
            rows.append(row)
    return rows


def flight_window(prefix: str) -> tuple:
    """
    The flight's window and stage times from sequencer.csv.

    Returns:
        (boost_us, done_us, {stage: seconds from boost}); done_us is the last stage change when DONE never came.

    Raises:
        ValueError - the session has no BOOSTING.
    """
    stages = [(row['t'], row['stage']) for row in read_stream(prefix, 'sequencer')]
    boost = next((t for t, stage in stages if stage in _BOOSTING), None)
    if boost is None:
        raise ValueError('%s: no BOOSTING in sequencer.csv' % prefix)
    done = next((t for t, stage in stages if stage in _DONE and t >= boost), stages[-1][0])
    times = {}
    for t, stage in stages:
        if t >= boost:
            times.setdefault(stage, (t - boost) / 1e6)
    return boost, done, times


def cadence(times: list) -> dict | None:
    """
    How regularly a stream's rows came: their rate and the gaps between them.

    Args:
        times - the rows' times, us, in order.

    Returns:
        {hz, median, p95, p99, max} (the gaps in ms), or None for fewer than three rows.
    """
    if len(times) < 3:
        return None
    gaps = sorted((later - earlier) / 1000 for earlier, later in zip(times, times[1:]))

    def quantile(fraction):
        return gaps[min(len(gaps) - 1, int(fraction * len(gaps)))]

    span_s = (times[-1] - times[0]) / 1e6
    return {'hz': (len(times) - 1) / span_s if span_s else 0.0, 'median': quantile(0.5), 'p95': quantile(0.95),
            'p99': quantile(0.99), 'max': gaps[-1]}


def summary(prefix: str) -> dict:
    """One flight's figures: its stage times, each stream's cadence and each health column's min/median/max."""
    boost, done, stages = flight_window(prefix)
    streams = {}
    for name in _STREAMS:
        streams[name] = cadence([row['t'] for row in read_stream(prefix, name) if boost <= row['t'] <= done])
    health = {}
    rows = [row for row in read_stream(prefix, 'health') if boost <= row['t'] <= done]
    for key in _HEALTH:
        values = [float(row[key]) for row in rows if row.get(key) not in (None, '', 'None')]
        health[key] = (min(values), statistics.median(values), max(values)) if values else None
    series = {key: [((row['t'] - boost) / 1e6, float(row[key])) for row in rows
                    if row.get(key) not in (None, '', 'None')] for key in _HEALTH}
    return {'stages': stages, 'streams': streams, 'health': health, 'series': series}


def table(flights: dict) -> str:
    """The markdown table of the flights' figures, a column per flight."""
    names = list(flights)
    lines = ['| | %s |' % ' | '.join(names), '|---|' + '---|' * len(names)]

    def row(label, cell):
        lines.append('| %s | %s |' % (label, ' | '.join(cell(flights[name]) for name in names)))

    for label, keys in (('GLIDING (s)', _GLIDING), ('DONE (s)', _DONE)):
        row(label, lambda f, keys=keys: next(('%.1f' % f['stages'][k] for k in keys if k in f['stages']), '-'))
    for name in _STREAMS:
        row('%s rate (Hz)' % name, lambda f, n=name: '%.1f' % f['streams'][n]['hz'] if f['streams'][n] else '-')
        row('%s gap median / p99 / max (ms)' % name,
            lambda f, n=name: ('%.1f / %.1f / %.1f' % tuple(f['streams'][n][k] for k in ('median', 'p99', 'max'))
                               if f['streams'][n] else '-'))
    for key, unit, scale in (('temp', 'deg C', 1.0), ('mem_free', 'MB', 1e-6), ('load', '%', 1.0)):
        row('%s min / median / max (%s)' % (key, unit),
            lambda f, k=key, s=scale: ('%.1f / %.1f / %.1f' % tuple(v * s for v in f['health'][k])
                                       if f['health'][k] else '-'))
    return '\n'.join(lines)


def figure(flights: dict, out: str) -> None:
    """The health over each flight (load, free heap, die temperature) as <out>.html, and <out>.svg with kaleido."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    panels = (('load', 'CPU load (%)', 1.0), ('mem_free', 'free heap (MB)', 1e-6), ('temp', 'die temperature (C)', 1.0))
    fig = make_subplots(rows=len(panels), cols=1, shared_xaxes=True, vertical_spacing=0.06,
                        subplot_titles=[title for _, title, _ in panels])
    for index, (key, _, scale) in enumerate(panels, start=1):
        for name, flight in flights.items():
            points = flight['series'][key]
            fig.add_trace(go.Scatter(x=[t for t, _ in points], y=[v * scale for _, v in points], mode='lines',
                                     name=name, legendgroup=name, showlegend=index == 1), row=index, col=1)
    fig.update_xaxes(title_text='seconds from BOOSTING', row=len(panels), col=1)
    fig.update_layout(height=900, width=1100, title='HITL F15 clean on the taster: plan B vs the Rust avionics')
    fig.write_html(out + '.html', include_plotlyjs='cdn')
    try:
        fig.write_image(out + '.svg')
    except (ValueError, RuntimeError, ImportError) as error:
        print('no svg: %s' % error, file=sys.stderr)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('flights', nargs='+', help="'<name>=<prefix>' per flight")
    parser.add_argument('-o', '--out', help='the figure, <out>.html and <out>.svg')
    args = parser.parse_args()
    flights = {}
    for item in args.flights:
        name, _, prefix = item.partition('=')
        if not prefix:
            parser.error('%r is not <name>=<prefix>' % item)
        flights[name] = summary(prefix)
    print(table(flights))
    if args.out:
        figure(flights, args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
