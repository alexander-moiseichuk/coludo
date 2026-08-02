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

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixed  # noqa: E402 -- the firmware's fixed-point scale, so this cannot drift from the board
import flight_telemetry  # noqa: E402

_KNOTS_TO_MS = 0.514444  # NMEA RMC knots -> m/s (matches gnss._KNOTS_TO_MS)
_SCALE = fixed.SCALE  # the dynamic_pressure fixnum is Pa x SCALE -- read it, never restate it
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


def _is_capture(path: str) -> bool:
    """
    Is this an assembled capture? Decided by the WIRE MARKER, not the file extension.

    `.txt` matched any stray log or config copy in the directory and produced a cryptic parse error
    instead of a clear one. Every capture line carries the `@<session>_<stream>.csv@` prefix, so one
    line is enough to tell.

    Args:
        path - the file to test.

    Returns:
        True when the first readable line carries a capture marker.
    """
    try:
        with open(path) as handle:
            for line in handle:
                if line.startswith('@') and '.csv@' in line:
                    return True
                if line.strip():
                    return False  # a real first line that is not a capture row -> not a capture
    except OSError:
        return False
    return False


def _read_capture(path: str) -> tuple:
    """
    Pull the pitot and GNSS rows out of an ASSEMBLED capture (.txt) -- the artifact every other tool eats.

    Without this the calibrator was the odd one out: it read only loose per-stream CSVs, so trimming
    `air_density` from a field recording meant hand-splitting the capture first. Reuses the shared
    flight_telemetry parser (findings §27.5) rather than re-implementing the wire format, and rebuilds
    the same {column: value} row dicts the CSV path yields, so calibrate() is untouched.

    Args:
        path - the assembled capture file.

    Returns:
        (pitot_rows, gnss_rows) -- each a list of {column: value} dicts, empty when the stream is absent.
    """
    with open(path) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    pitot = flight_telemetry.find_stream(streams, 'dynamic_pressure')
    gnss = flight_telemetry.find_stream(streams, 'speed_kn')

    def rows(stream):
        if stream is None:
            return []
        names = ['uptime'] + list(stream.fields)
        return [dict(zip(names, [str(cell) for cell in row])) for row in stream.rows]

    return rows(pitot), rows(gnss)


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
    # float() both: the CSV path yields integer strings, the assembled-capture path yields parsed
    # floats ('0.0'), and int() would reject the latter
    pitot = sorted((int(float(row['uptime'])), float(row['dynamic_pressure']) / _SCALE)
                   for row in pitot_rows)
    stamps = [stamp for stamp, _q in pitot]
    pressures = [q for _stamp, q in pitot]
    densities, pairs = [], []
    for row in gnss_rows:
        speed = float(row['speed_kn']) * _KNOTS_TO_MS
        if speed < min_speed:
            continue  # not the steady airborne segment (taxi / pad / stall)
        q = _nearest(stamps, pressures, int(float(row['uptime'])))
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
        'pairs': pairs,          # (gnss_speed, q) per accepted sample -- what the plot draws
        'densities': densities,  # the per-sample rho estimates, sorted (the scatter's spread)
    }


def plot(result: dict, path: str, current: float) -> None:
    """
    Draw the calibration so a bad pass is VISIBLE, not just a number (findings §27.17).

    The fit returns one density, but whether to TRUST it is a visual question: a calm, steady pass
    collapses pitot airspeed onto GNSS ground speed along the 1:1 line and leaves a tight residual band;
    wind shows up as a consistent offset, a ragged pass as scatter, and a gust or a turn as outliers no
    median can warn you about. Two panels, stdlib SVG only (no plotly needed at the field).

    Args:
        result - a calibrate() result carrying 'pairs' and 'densities'.
        path - the SVG to write.
        current - the air_density the recording was made with (drawn for comparison).

    Returns:
        None; writes the SVG.
    """
    pairs, densities = result['pairs'], result['densities']
    fitted = result['air_density']
    width, height, pad = 900, 420, 58
    panel = (width - 3 * pad) / 2
    speeds = [speed for speed, _q in pairs]
    fitted_speeds = [(2.0 * q / fitted) ** 0.5 for _s, q in pairs]
    lo = min(min(speeds), min(fitted_speeds))
    hi = max(max(speeds), max(fitted_speeds))
    span = (hi - lo) or 1.0
    body = ['<rect width="%d" height="%d" fill="white"/>' % (width, height),
            '<text x="%d" y="26" font-size="16" font-family="sans-serif">'
            'airspeed calibration &#8212; %d samples, rho %.3f (was %.3f), '
            'error %.1f%% &#8594; %.1f%%</text>'
            % (pad, result['samples'], fitted, current, result['error_before'], result['error_after'])]

    # panel 1: pitot-derived airspeed vs GNSS ground speed, against the 1:1 line a calm pass sits on
    x0, y0 = pad, 56
    fx = lambda v: x0 + (v - lo) / span * panel          # noqa: E731
    fy = lambda v: y0 + panel - (v - lo) / span * panel  # noqa: E731
    body.append('<rect x="%.0f" y="%.0f" width="%.0f" height="%.0f" fill="#fbfbfb" stroke="#ddd"/>'
                % (x0, y0, panel, panel))
    body.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#999" stroke-dasharray="5 4"/>'
                % (fx(lo), fy(lo), fx(hi), fy(hi)))
    for ground, q in pairs:
        body.append('<circle cx="%.1f" cy="%.1f" r="2" fill="#1f77b4" opacity="0.55"/>'
                    % (fx(ground), fy((2.0 * q / fitted) ** 0.5)))
    body.append('<text x="%.0f" y="%.0f" font-size="12" font-family="sans-serif">'
                'GNSS ground speed (m/s) &#8594;</text>' % (x0, y0 + panel + 20))
    body.append('<text x="%.0f" y="%.0f" font-size="12" font-family="sans-serif" '
                'transform="rotate(-90 %.0f %.0f)">pitot airspeed (m/s)</text>'
                % (x0 - 14, y0 + panel, x0 - 14, y0 + panel))
    body.append('<text x="%.0f" y="%.0f" font-size="11" fill="#666" font-family="sans-serif">'
                'dashed = 1:1 (a calm, well-trimmed pass sits on it)</text>' % (x0 + 6, y0 + 14))

    # panel 2: the per-sample density estimates -- the spread IS the confidence in the number
    x1 = pad * 2 + panel
    body.append('<rect x="%.0f" y="%.0f" width="%.0f" height="%.0f" fill="#fbfbfb" stroke="#ddd"/>'
                % (x1, y0, panel, panel))
    d_lo, d_hi = densities[0], densities[-1]
    d_span = (d_hi - d_lo) or 1.0
    gx = lambda i: x1 + i / max(len(densities) - 1, 1) * panel                 # noqa: E731
    gy = lambda d: y0 + panel - (d - d_lo) / d_span * panel                    # noqa: E731
    for index, density in enumerate(densities):
        body.append('<circle cx="%.1f" cy="%.1f" r="1.6" fill="#2ca02c" opacity="0.5"/>'
                    % (gx(index), gy(density)))
    for value, colour, label in ((fitted, '#c22', 'fit %.3f' % fitted),
                                 (result['iqr'][0], '#999', 'IQR'),
                                 (result['iqr'][1], '#999', '')):
        if d_lo <= value <= d_hi:
            body.append('<line x1="%.0f" y1="%.1f" x2="%.0f" y2="%.1f" stroke="%s" stroke-dasharray="4 3"/>'
                        % (x1, gy(value), x1 + panel, gy(value), colour))
            if label:
                body.append('<text x="%.0f" y="%.1f" font-size="11" fill="%s" font-family="sans-serif">'
                            '%s</text>' % (x1 + panel - 62, gy(value) - 4, colour, label))
    body.append('<text x="%.0f" y="%.0f" font-size="12" font-family="sans-serif">'
                'per-sample rho, sorted &#8212; a WIDE band means a windy or ragged pass</text>'
                % (x1, y0 + panel + 20))

    with open(path, 'w') as handle:
        handle.write('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
                     'viewBox="0 0 %d %d">%s</svg>\n' % (width, height, width, height, ''.join(body)))


def _rms_error(pairs: list, density: float) -> float:
    """RMS of the fractional airspeed error |sqrt(2q/rho) − v| / v across the paired samples (percent)."""
    total = sum((((2.0 * q / density) ** 0.5 - v) / v) ** 2 for v, q in pairs)
    return 100.0 * (total / len(pairs)) ** 0.5


def main() -> int:
    parser = argparse.ArgumentParser(description='SDP810 air_density trim from a calm-day GNSS-vs-q pass.')
    parser.add_argument('recording',
                        help='an assembled capture .txt, a recording directory (auto-detects both '
                             'CSVs), or the pitot CSV')
    parser.add_argument('--gnss', help='the GNSS CSV (auto-detected in a directory by its speed_kn column)')
    parser.add_argument('--current', type=float, default=1.18, help='air_density in the recording (default 1.18)')
    parser.add_argument('--plot', help='also write an SVG of the fit (pitot vs GNSS + the rho spread)')
    parser.add_argument('--min-speed', type=float, default=8.0, help='ignore ground speed below this m/s (default 8)')
    args = parser.parse_args()

    if os.path.isdir(args.recording):
        pitot_path = _find(args.recording, 'dynamic_pressure')
        gnss_path = args.gnss or _find(args.recording, 'speed_kn')
        if not pitot_path or not gnss_path:
            print('error: could not find the pitot (dynamic_pressure) and/or GNSS (speed_kn) CSV',
                  file=sys.stderr)
            return 2
        pitot_rows, gnss_rows = _read_csv(pitot_path), _read_csv(gnss_path)
    elif _is_capture(args.recording):  # an assembled capture -- what every other tool consumes
        pitot_path = gnss_path = args.recording
        pitot_rows, gnss_rows = _read_capture(args.recording)
        if not pitot_rows or not gnss_rows:
            print('error: the capture carries no pitot (dynamic_pressure) and/or GNSS (speed_kn) stream',
                  file=sys.stderr)
            return 2
    else:
        pitot_path = args.recording
        gnss_path = args.gnss or _find(os.path.dirname(pitot_path) or '.', 'speed_kn')
        if not gnss_path:
            print('error: could not find the GNSS (speed_kn) CSV', file=sys.stderr)
            return 2
        pitot_rows, gnss_rows = _read_csv(pitot_path), _read_csv(gnss_path)

    result = calibrate(pitot_rows, gnss_rows, args.min_speed, args.current)
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
    if args.plot:
        plot(result, args.plot, args.current)
        print('wrote %s' % args.plot)
    return 0


if __name__ == '__main__':
    sys.exit(main())
