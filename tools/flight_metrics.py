"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Campaign view: one row per capture in a directory — miss / in-zone / max-from-pad / duration / apogee /
peak accel — so a whole matrix (E16 vs F15 × weights × repeats × seeds) can be read and ranked at once.

That matrix is exactly what the passive-telemetry flights will produce, and nothing rendered it
(findings §27.18): there were per-run HTML reports plus hand-written README tables. Every capture in the
directory now appears — an unrecognised name is listed, never silently dropped (the previous version
printed only a hardcoded scenario list, so a new case simply vanished from the table).

    python3 tools/flight_metrics.py <dir-of-captures>            # ranked by miss
    python3 tools/flight_metrics.py <dir> --sort name            # or by name / apogee / duration
"""

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import flight_telemetry  # noqa: E402

_M_PER_DEG = 111320.0
_PAD = (25.514379, -80.391795)
_TL, _BR = (25.514944, -80.392972), (25.514583, -80.391111)
_CENTER = ((_TL[0] + _BR[0]) / 2.0, (_TL[1] + _BR[1]) / 2.0)
_COSLAT = math.cos(math.radians(_PAD[0]))
# a preferred order for the scenario names the sweeps use; anything else sorts after, never disappears
_ORDER = ('noise05', 'noise10', 'noise25', 'noise50', 'noise100',
          'wind00', 'wind03', 'wind06', 'wind09', 'wind12', 'corner_spike', 'corner_stress')


def _meters(a: tuple, b: tuple) -> float:
    return math.hypot((a[0] - b[0]) * _M_PER_DEG, (a[1] - b[1]) * _M_PER_DEG * _COSLAT)


def metrics(path: str):
    """
    The comparable numbers for one capture.

    Parsed through flight_telemetry (not a private line scan), so this inherits its stream handling --
    including the per-servo fins rebuild and the malformed-row guards.

    Args:
        path - the capture file.

    Returns:
        A dict of metrics, or None when the capture carries no GNSS track to measure.
    """
    with open(path) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    gnss = flight_telemetry.find_stream(streams, 'lat', 'lon')
    if gnss is None:
        return None
    times, latitude = gnss.column('lat')
    _, longitude = gnss.column('lon')
    if not times:
        return None
    touchdown = (latitude[-1], longitude[-1])
    baro = (flight_telemetry.find_stream(streams, 'elevation', prefer='icp')
            or flight_telemetry.find_stream(streams, 'altitude'))
    apogee = 0.0
    if baro is not None:
        field = 'elevation' if 'elevation' in baro.fields else 'altitude'
        values = baro.column(field)[1]
        if values:  # an AMSL altitude is re-based to the pad so both shapes report height above ground
            apogee = max(values) - (min(values) if field == 'altitude' else 0.0)
    accel = flight_telemetry.find_stream(streams, 'ax', 'ay', 'az', prefer='adxl')
    peak_g = 0.0
    if accel is not None:
        magnitudes = [math.sqrt(x * x + y * y + z * z) for x, y, z in
                      zip(accel.column('ax')[1], accel.column('ay')[1], accel.column('az')[1])]
        peak_g = max(magnitudes) if magnitudes else 0.0
    return {
        'miss': _meters(touchdown, _CENTER),
        'in_zone': (_BR[0] <= touchdown[0] <= _TL[0]) and (_TL[1] <= touchdown[1] <= _BR[1]),
        'maxpad': max(_meters((a, b), _PAD) for a, b in zip(latitude, longitude)),
        'duration': times[-1] - times[0],
        'apogee': apogee,
        'peak_g': peak_g,
    }


def _rank(name: str) -> tuple:
    """Sort key: the known scenario order first, then everything else alphabetically."""
    return (_ORDER.index(name), name) if name in _ORDER else (len(_ORDER), name)


def main() -> int:
    parser = argparse.ArgumentParser(description='Campaign metrics for every capture in a directory.')
    parser.add_argument('directory', help='a directory of <scenario>.txt captures')
    parser.add_argument('--sort', default='miss', choices=('miss', 'name', 'apogee', 'duration'),
                        help='ranking column (default miss -- best landing first)')
    args = parser.parse_args()

    rows, skipped = [], []
    for entry in sorted(os.listdir(args.directory)):
        if not entry.endswith('.txt'):
            continue
        result = metrics(os.path.join(args.directory, entry))
        if result is None:
            skipped.append(entry[:-4])  # reported, never silently dropped
            continue
        result['name'] = entry[:-4]
        rows.append(result)
    if not rows:
        print('no captures with a GNSS track in %s' % args.directory, file=sys.stderr)
        return 1

    if args.sort == 'name':
        rows.sort(key=lambda row: _rank(row['name']))
    elif args.sort == 'miss':
        rows.sort(key=lambda row: row['miss'])
    else:
        rows.sort(key=lambda row: -row[args.sort])

    print('%-22s %8s %6s %9s %8s %9s %7s' % ('scenario', 'miss', 'zone', 'maxpad', 'dur', 'apogee', 'peak'))
    print('-' * 76)
    for row in rows:
        print('%-22s %7.0f m %6s %8.0f m %7.1fs %8.0f m %6.1fg'
              % (row['name'], row['miss'], 'yes' if row['in_zone'] else 'no',
                 row['maxpad'], row['duration'], row['apogee'], row['peak_g']))
    best = min(rows, key=lambda row: row['miss'])
    print('-' * 76)
    print('%d capture(s): %d in-zone, best %.0f m (%s), median %.0f m'
          % (len(rows), sum(1 for row in rows if row['in_zone']), best['miss'], best['name'],
             sorted(row['miss'] for row in rows)[len(rows) // 2]))
    if skipped:
        print('skipped (no GNSS track): %s' % ', '.join(skipped))
    return 0


if __name__ == '__main__':
    sys.exit(main())
