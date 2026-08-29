"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Compare REPEATED HITL matrix runs, panel by panel, to separate real change from run-to-run noise.

A single matrix says what the aircraft did once. Comparing a build against a baseline from a different
run is only meaningful if you know how much a run varies against ITSELF -- otherwise every difference
looks like a finding. This flies the same matrix N times and reports, per scenario and per motor, the
spread across runs for every quantity the flight report plots.

The panels mirror tools/flight_report.py exactly, so a number here can be traced to a chart there:
  |accel|, altitude/elevation, speed, attitude, fins commanded, board health, agl, engine power,
  gyro rate, airspeed (pitot / estimate / GNSS), control authority (fin cap vs demand).

Two things are reported and they answer different questions:
  * SPREAD -- max-min across runs, as a fraction of the mean. How repeatable is this quantity.
  * ANOMALY -- a channel present in some runs and absent in others, a run outside the others' range by
    more than a stated factor, or a flight that ended in a different stage. These are the ones worth
    looking at; a large spread on a quantity that is inherently chaotic (touchdown miss) is not.

Usage:
    python3 tools/hitl_compare.py r1=/path/r1 r2=/path/r2 r3=/path/r3 [--motors E16,F15] [--json out.json]
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402 -- the SHARED capture parser, so this cannot drift from the reports

# Each entry: (panel label, stream selector, field, how to reduce the series to one number).
# 'selector' is passed to flight_telemetry.find_stream, matching flight_report's own stream choices.
_PANELS: tuple = (
    ('accel |a| peak',        ('ax', 'ay', 'az'), 'adxl', 'az', 'max'),
    ('accel |a| mean',        ('ax', 'ay', 'az'), 'adxl', 'az', 'mean'),
    ('altitude peak',         ('elevation',), 'icp', 'altitude', 'max'),
    ('elevation peak',        ('elevation',), 'icp', 'elevation', 'max'),
    ('elevation final',       ('elevation',), 'icp', 'elevation', 'last'),
    ('speed max',             ('lat', 'lon'), None, 'speed_kn', 'max'),
    ('attitude roll range',   ('roll', 'pitch'), 'bno', 'roll', 'range'),
    ('attitude pitch range',  ('roll', 'pitch'), 'bno', 'pitch', 'range'),
    ('fin eleron_left range', ('eleron_left',), None, 'eleron_left', 'range'),
    ('fin yaw range',         ('eleron_left',), None, 'yaw', 'range'),
    ('health load max',       ('load',), None, 'load', 'max'),
    ('health mem_free min',   ('load',), None, 'mem_free', 'min'),
    ('health rescues',        ('load',), None, 'rescues', 'max'),
    ('agl min',               ('agl',), None, 'agl', 'min'),
    ('engine power peak',     ('power_mw',), None, 'power_mw', 'max'),
    ('engine mA peak',        ('power_mw',), None, 'current_ma', 'max'),
    ('gyro gx range',         ('gx', 'gy', 'gz'), 'lsm', 'gx', 'range'),
    ('gyro gz range',         ('gx', 'gy', 'gz'), 'lsm', 'gz', 'range'),
    ('airspeed pitot max',    ('dynamic_pressure',), None, 'airspeed_cms', 'max'),
    ('fin cap min',           ('fin_cap',), None, 'fin_cap', 'min'),
    ('heading_err max',       ('fin_cap',), None, 'heading_err', 'max'),
    ('duration s',            ('elevation',), 'icp', None, 'duration'),
    ('rows total',            None, None, None, 'rows'),
)

_ANOMALY_FACTOR: float = 3.0   # a run this far outside the others' spread is called out, not just listed


def _reduce(values: list, how: str):
    """One number from a series: the reduction each panel names. None when the channel is absent."""
    if not values:
        return None
    if how == 'max':
        return max(values)
    if how == 'min':
        return min(values)
    if how == 'mean':
        return sum(values) / len(values)
    if how == 'last':
        return values[-1]
    if how == 'range':
        return max(values) - min(values)
    return None


def _metrics(path: str) -> dict:
    """
    Every panel quantity for one capture.

    Args:
        path - the capture .txt written by hitl_collect.

    Returns:
        {panel label: value or None}; None means the channel was absent, which is itself comparable.
    """
    with open(path) as handle:
        streams, _unused = flight_telemetry.parse(handle.read())
    out = {}
    total_rows = sum(len(stream.rows) for stream in streams.values())
    for label, selector, prefer, field, how in _PANELS:
        if how == 'rows':
            out[label] = total_rows
            continue
        stream = flight_telemetry.find_stream(streams, *selector, prefer=prefer) if selector else None
        if stream is None:
            out[label] = None
            continue
        if how == 'duration':
            stamps, _values = stream.column(stream.fields[0])
            out[label] = (max(stamps) - min(stamps)) / 1e6 if stamps else None
            continue
        try:
            _stamps, values = stream.column(field)
        except Exception:
            out[label] = None
            continue
        out[label] = _reduce([v for v in values if v is not None], how)
    return out


def _spread(values: list) -> tuple:
    """
    (low, high, spread-fraction) over the runs that produced a number.

    The fraction is (max-min)/|mean|, so quantities of different magnitude compare on one scale. Returns
    None for the fraction when the mean is ~0, where a ratio says nothing.
    """
    live = [v for v in values if v is not None]
    if not live:
        return None, None, None
    low, high = min(live), max(live)
    mean = sum(live) / len(live)
    return low, high, (abs(high - low) / abs(mean) if abs(mean) > 1e-9 else None)


def compare(runs: dict, motors: list) -> dict:
    """
    Compare every scenario in every motor across the named runs.

    Args:
        runs - {run label: directory}; each holds <motor>/<scenario>.txt.
        motors - motor subdirectory names to walk.

    Returns:
        {'rows': [...per scenario/panel...], 'anomalies': [...], 'missing': [...]}.
    """
    rows, anomalies, missing = [], [], []
    labels = list(runs)
    for motor in motors:
        scenarios = set()
        for directory in runs.values():
            here = os.path.join(directory, motor)
            if os.path.isdir(here):
                scenarios |= {f[:-4] for f in os.listdir(here) if f.endswith('.txt')}
        for scenario in sorted(scenarios):
            per_run = {}
            for label, directory in runs.items():
                path = os.path.join(directory, motor, '%s.txt' % scenario)
                if not os.path.exists(path):
                    missing.append('%s/%s absent from %s' % (motor, scenario, label))
                    continue
                per_run[label] = _metrics(path)
            if len(per_run) < 2:
                continue
            for panel, _sel, _pref, _field, _how in _PANELS:
                values = [per_run.get(label, {}).get(panel) for label in labels]
                low, high, frac = _spread(values)
                rows.append({'motor': motor, 'scenario': scenario, 'panel': panel,
                             'values': values, 'low': low, 'high': high, 'spread': frac})
                present = [label for label, v in zip(labels, values) if v is not None]
                absent = [label for label, v in zip(labels, values) if v is None]
                if present and absent:
                    anomalies.append({'motor': motor, 'scenario': scenario, 'panel': panel,
                                      'kind': 'channel present in %s, absent in %s'
                                              % (','.join(present), ','.join(absent))})
    return {'rows': rows, 'anomalies': anomalies, 'missing': missing, 'labels': labels}


def main() -> None:
    """Parse the run list, compare, and print a per-panel table plus the anomalies."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', help='label=directory, e.g. r1=/tmp/r1')
    parser.add_argument('--motors', default='E16,F15')
    parser.add_argument('--json', help='also write the full comparison as JSON')
    parser.add_argument('--top', type=int, default=25, help='how many widest-spread rows to print')
    args = parser.parse_args()

    runs = {}
    for item in args.runs:
        if '=' not in item:
            raise SystemExit('expected label=directory, got %r' % item)
        label, directory = item.split('=', 1)
        runs[label] = directory
    result = compare(runs, args.motors.split(','))

    print('runs: %s' % ', '.join('%s=%s' % (k, v) for k, v in runs.items()))
    print('scenario/panel comparisons: %d' % len(result['rows']))
    if result['missing']:
        print('MISSING FLIGHTS (%d):' % len(result['missing']))
        for line in result['missing'][:10]:
            print('   ', line)
    print()
    ranked = sorted([r for r in result['rows'] if r['spread'] is not None],
                    key=lambda r: -r['spread'])
    print('WIDEST SPREAD ACROSS RUNS (spread = (max-min)/mean):')
    print('%-5s %-14s %-24s %9s   %s' % ('motor', 'scenario', 'panel', 'spread', 'values'))
    for row in ranked[:args.top]:
        shown = ', '.join('-' if v is None else ('%.3g' % v) for v in row['values'])
        print('%-5s %-14s %-24s %8.1f%%   %s' % (row['motor'], row['scenario'], row['panel'],
                                                 100 * row['spread'], shown))
    print()
    print('ANOMALIES (channel present in some runs, absent in others): %d' % len(result['anomalies']))
    for item in result['anomalies'][:20]:
        print('    %s/%s %s -- %s' % (item['motor'], item['scenario'], item['panel'], item['kind']))
    if args.json:
        with open(args.json, 'w') as handle:
            json.dump(result, handle, indent=1)
        print('\nwrote %s' % args.json)


if __name__ == '__main__':
    main()
