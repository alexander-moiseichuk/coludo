#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Compute the SDP810 airspeed span trim (`air_density`) from a CALM-DAY recording.

On a calm pass the GNSS GROUND speed equals the true AIRSPEED (no wind), and dynamic pressure is
q = ½·rho·v². So each paired sample gives a density estimate rho = 2q / v², and the robust median over
the steady, airborne segment is the `air_density` that makes the pitot airspeed match GNSS. This absorbs
BOTH the real air density and the pitot/static position-span error into the one knob (the driver derives
airspeed = sqrt(2q/rho), so the fix is a single number).

Inputs are the on-board telemetry CSVs (`recorder.Telemetry`, ';'-separated, uptime in µs):
  * the pitot stream  `airspeed_sdp810.csv`  -> `dynamic_pressure` (a Pa fixnum, ÷ SCALE=100 -> Pa);
  * the GNSS stream   `<gnss>.csv`           -> `speed_kn` (knots, × 0.514444 -> m/s).
Point it at a recording directory (both auto-detected by their columns) or pass the files explicitly.

    python3 tools/airspeed_calibrate.py <recording_dir>            # auto-detect both CSVs
    python3 tools/airspeed_calibrate.py pitot.csv --gnss gnss.csv  # explicit
    # then set the printed value in config `airspeed_sdp810.air_density` (or CC `update {air_density: X}`)

Fly the pass STEADY (a constant glide) in still air; the tool reports the sample scatter so a windy or
ragged pass shows up as a wide spread rather than a confident-but-wrong number.
"""

import argparse
import bisect
import glob
import os
import statistics
import sys

_KNOTS_TO_MS = 0.514444  # NMEA RMC knots -> m/s (matches gnss._KNOTS_TO_MS)
_SCALE = 100  # fixed.SCALE: the dynamic_pressure fixnum is Pa × SCALE
_MAX_PAIR_US = 200_000  # reject a GNSS sample with no pitot row within 200 ms (time-alignment guard)


def _read_csv(path: str) -> list:
    """Parse a ';'-separated telemetry CSV into a list of {column: value} dicts (header row first)."""
    with open(path) as handle:
        header = handle.readline().strip().split(';')
        rows = []
        for line in handle:
            parts = line.strip().split(';')
            if len(parts) == len(header):
                rows.append(dict(zip(header, parts)))
    return rows


def _find(directory: str, column: str) -> str:
    """The first *.csv in `directory` whose header carries `column` (auto-detect a stream by its field)."""
    for path in sorted(glob.glob(os.path.join(directory, '*.csv'))):
        with open(path) as handle:
            if column in handle.readline().strip().split(';'):
                return path
    return None


def _nearest(stamps: list, values: list, when: int):
    """The value whose timestamp is closest to `when` (or None if the nearest is beyond _MAX_PAIR_US)."""
    index = bisect.bisect_left(stamps, when)
    best = None
    for candidate in (index - 1, index):
        if 0 <= candidate < len(stamps) and (best is None or abs(stamps[candidate] - when) < abs(stamps[best] - when)):
            best = candidate
    if best is None or abs(stamps[best] - when) > _MAX_PAIR_US:
        return None
    return values[best]


def calibrate(pitot_rows: list, gnss_rows: list, min_speed: float, current: float) -> dict:
    """
    Pair each airborne GNSS sample with the nearest pitot dynamic pressure and fit `air_density`.

    Args:
        pitot_rows - the airspeed_sdp810.csv rows (uptime + dynamic_pressure fixnum).
        gnss_rows - the GNSS csv rows (uptime + speed_kn).
        min_speed - ignore samples below this ground speed (m/s): only the steady airborne glide.
        current - the air_density used during the recording (for the before/after airspeed error).

    Returns:
        A result dict: recommended air_density, sample count, robust scatter, and the airspeed error
        before/after the trim; or {'samples': 0} when nothing usable pairs up.
    """
    pitot = sorted((int(row['uptime']), int(row['dynamic_pressure']) / _SCALE) for row in pitot_rows)
    stamps = [stamp for stamp, _q in pitot]
    pressures = [q for _stamp, q in pitot]
    densities, pairs = [], []
    for row in gnss_rows:
        speed = float(row['speed_kn']) * _KNOTS_TO_MS
        if speed < min_speed:
            continue  # not the steady airborne segment (taxi / pad / stall)
        q = _nearest(stamps, pressures, int(row['uptime']))
        if q is None or q <= 0.0:
            continue  # no time-aligned pitot sample, or a sub-zero (reverse/noise) reading
        densities.append(2.0 * q / (speed * speed))
        pairs.append((speed, q))
    if len(densities) < 5:
        return {'samples': len(densities)}
    densities.sort()
    recommended = statistics.median(densities)
    quartile = (densities[len(densities) // 4], densities[(3 * len(densities)) // 4])
    return {
        'samples': len(densities),
        'air_density': recommended,
        'iqr': (quartile[0], quartile[1]),
        'error_before': _rms_error(pairs, current),
        'error_after': _rms_error(pairs, recommended),
    }


def _rms_error(pairs: list, density: float) -> float:
    """RMS of the fractional airspeed error |sqrt(2q/rho) − v| / v across the paired samples (percent)."""
    total = sum((((2.0 * q / density) ** 0.5 - v) / v) ** 2 for v, q in pairs)
    return 100.0 * (total / len(pairs)) ** 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description='SDP810 air_density trim from a calm-day GNSS-vs-q pass.')
    parser.add_argument('recording', help='a recording directory (auto-detects both CSVs) or the pitot CSV')
    parser.add_argument('--gnss', help='the GNSS CSV (auto-detected in a directory by its speed_kn column)')
    parser.add_argument('--current', type=float, default=1.225, help='air_density in the recording (default 1.225)')
    parser.add_argument('--min-speed', type=float, default=8.0, help='ignore ground speed below this m/s (default 8)')
    args = parser.parse_args()

    if os.path.isdir(args.recording):
        pitot_path = _find(args.recording, 'dynamic_pressure')
        gnss_path = args.gnss or _find(args.recording, 'speed_kn')
    else:
        pitot_path = args.recording
        gnss_path = args.gnss or _find(os.path.dirname(pitot_path) or '.', 'speed_kn')
    if not pitot_path or not gnss_path:
        print('error: could not find the pitot (dynamic_pressure) and/or GNSS (speed_kn) CSV', file=sys.stderr)
        return 2

    result = calibrate(_read_csv(pitot_path), _read_csv(gnss_path), args.min_speed, args.current)
    if result['samples'] < 5:
        print('error: only %d usable paired samples (need >=5) -- fly a longer steady calm pass above %g m/s'
              % (result['samples'], args.min_speed), file=sys.stderr)
        return 1

    print('pitot : %s' % pitot_path)
    print('gnss  : %s' % gnss_path)
    print('samples (steady, >= %g m/s) : %d' % (args.min_speed, result['samples']))
    print('sample air_density spread (IQR) : %.3f .. %.3f kg/m^3' % result['iqr'])
    print('airspeed error vs GNSS : %.1f%% (current %.3f) -> %.1f%% (recommended)'
          % (result['error_before'], args.current, result['error_after']))
    print()
    print('RECOMMENDED air_density = %.3f' % result['air_density'])
    print("apply: config airspeed_sdp810 'air_density': %.3f   (or CC: update {\"air_density\": %.3f})"
          % (result['air_density'], result['air_density']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
