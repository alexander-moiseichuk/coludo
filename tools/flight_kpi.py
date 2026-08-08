"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Guidance-effectiveness KPIs for one or more flight captures, comparable across firmware versions (used
for the doc/sims set READMEs). Per capture:
  * fin ACTIVITY -- commanded-angle changes between consecutive fins.csv samples ('moves'; sg90
    compare-and-sets, so an unchanged command is no PWM write), total travel (deg), max single step;
  * CONTROL EFFORT -- total travel per second of flight;
  * SERVO ENERGY -- the INA226 power integral (J) and average power = energy / flight duration
    (real measured actuation work, including holding torque);
  * GUIDANCE OUTCOME -- touchdown distance from the landing-zone centre + inside-zone flag.
Handles both the integer milli-unit power stream (power_mw, fixnums firmware on) and the older float
watts (power) so pre-fixnums captures compare on the same axis.

Usage: flight_kpi.py LABEL:capture.txt [LABEL:capture.txt ...] [--zone lat1,lon1,lat2,lon2]
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flight_telemetry  # noqa: E402

_FINS: tuple = ('eleron_left', 'eleron_right', 'yaw')
_ZONE_DEFAULT: str = '25.514944,-80.392972,25.514583,-80.391111'  # HPRC zone (TL, BR), as in hitl_matrix
_M_PER_DEG: float = 111320.0
_PRIMARY_RANGE_G: float = 32.0    # LSM6DSO32 full scale -- the range the backstop has to beat to matter
_BACKSTOP_RANGE_G: float = 200.0  # ADXL375 full scale
_CLIP_FRACTION: float = 0.95      # within this of the primary's rail counts as clipped


def _fin_activity(fins) -> tuple:
    """Per-fin (moves, travel_deg, max_step) rows + the flight span in seconds from the fins stream."""
    if fins is None:
        return [], 1.0  # no fin stream in this capture -- report nothing rather than crash
    rows = []
    starts, ends = [], []
    for fin in _FINS:
        times, angles = fins.column(fin)
        moves = travel = biggest = 0
        for previous, current in zip(angles, angles[1:]):
            delta = abs(current - previous)
            if delta > 0:
                moves += 1
                travel += delta
                biggest = max(biggest, delta)
        if len(times) > 1:  # collect each fin's window; the flight span is their combined range
            starts.append(times[0])
            ends.append(times[-1])
        rows.append((fin, len(angles), moves, travel, biggest))
    span = max(max(ends) - min(starts), 1e-9) if starts else 1.0  # single non-zero span for all fins
    return rows, span


def _servo_energy(power) -> tuple:
    """
    Energy (joules) and duration from the INA226 stream.

    Trapezoid integral of power over the capture. Reads power_mw (integer milli-units) or the
    pre-fixnums float `power` watts, so old and new captures land on the same axis.

    Returns:
        (joules, duration_s) over the captured window; (0, 0) when the stream is absent or empty.
    """
    if power is None:
        return 0.0, 0.0  # no INA226 in this capture
    field = 'power_mw' if 'power_mw' in power.fields else 'power'
    times, values = power.column(field)
    if not times:  # empty power stream -- nothing to integrate
        return 0.0, 0.0
    milliwatts = values if field == 'power_mw' else [watts * 1000.0 for watts in values]
    duration = times[-1] - times[0]
    joules = sum((t1 - t0) * (p0 + p1) / 2000.0
                 for t0, t1, p0, p1 in zip(times, times[1:], milliwatts, milliwatts[1:]))
    return joules, duration


def _touchdown(gnss, zone: tuple) -> tuple:
    """
    The last GNSS fix measured against the zone rectangle.

    Args:
        gnss - the parsed GNSS stream (its final lat/lon is the touchdown point).
        zone - the landing rectangle as ((lat, lon) TL, (lat, lon) BR).

    Returns:
        (miss_m, inside): metres from the zone centre and whether the fix is inside the rectangle;
        (0.0, False) when there is no GNSS stream or it holds no fix.
    """
    if gnss is None:
        return 0.0, False  # no GNSS in this capture -- nothing to measure against the zone
    _times, latitudes = gnss.column('lat')
    _times, longitudes = gnss.column('lon')
    if not latitudes:  # empty GNSS stream -- no touchdown fix
        return 0.0, False
    latitude, longitude = latitudes[-1], longitudes[-1]
    (lat_t, lon_l), (lat_b, lon_r) = zone
    centre_lat, centre_lon = (lat_t + lat_b) / 2, (lon_l + lon_r) / 2
    north = (latitude - centre_lat) * _M_PER_DEG
    east = (longitude - centre_lon) * _M_PER_DEG * math.cos(math.radians(centre_lat))
    inside = (min(lat_t, lat_b) <= latitude <= max(lat_t, lat_b)
              and min(lon_l, lon_r) <= longitude <= max(lon_l, lon_r))
    return math.hypot(north, east), inside


def _peak_g(stream) -> tuple:
    """
    Peak |a| (g) in one accel stream and when it happened.

    Args:
        stream - an accel stream carrying ax/ay/az, or None.

    Returns:
        (peak_g, time_s, samples); (0.0, 0.0, 0) for an absent or empty stream.
    """
    if stream is None or 'ax' not in stream.fields:
        return 0.0, 0.0, 0
    times, ax = stream.column('ax')
    _, ay = stream.column('ay')
    _, az = stream.column('az')
    """
    Non-finite samples are DROPPED before anything is computed. A NaN here is a corrupted telemetry
    record (flight_telemetry maps an unparseable cell to nan rather than letting a string reach the
    arithmetic), and NaN poisons every comparison silently: `min`/`max` return whichever operand they
    saw first, so a median filter built on them passes the garbage straight through. Found on a real
    board capture -- a lone 21 g on the ADXL375 sat directly beside a nan, i.e. it was the corrupt
    record's neighbour, not a shock. Left in, it would have argued to KEEP the +/-200 g backstop.
    """
    magnitudes, stamps = [], []
    for moment, x, y, z in zip(times, ax, ay, az):
        magnitude = math.sqrt(x * x + y * y + z * z)
        if magnitude == magnitude and magnitude != float('inf'):  # nan != nan
            magnitudes.append(magnitude)
            stamps.append(moment)
    if not magnitudes:
        return 0.0, 0.0, 0
    times = stamps
    """
    MEDIAN-FILTER the peak. A raw max is one sample, and one sample is exactly what a bad SPI read
    looks like -- found on a real board capture where the ADXL375 (known-intermittent on SPI, and
    logging `setup attempt 1/3 failed` that boot) reported a lone 21 g while the LSM6DSO32 beside it,
    sampling the same window at the same rate, never exceeded 3.3 g. A genuine shock moves both.
    Since this number decides whether the +/-200 g backstop stays on the board, an isolated glitch must
    not cast the vote: a 3-sample median keeps any event that lasts more than one sample and discards
    the ones that do not.
    """
    filtered = [max(min(magnitudes[i - 1], magnitudes[i]), min(max(magnitudes[i - 1], magnitudes[i]),
                magnitudes[i + 1])) for i in range(1, len(magnitudes) - 1)] or magnitudes
    peak = max(filtered)
    when = times[magnitudes.index(peak)] if peak in magnitudes else times[0]
    return peak, when, len(times)


def _raw_peak(stream) -> float:
    """The UNFILTERED max |a| -- compared against the filtered peak to expose single-sample glitches."""
    if stream is None or 'ax' not in stream.fields:
        return 0.0
    _t, ax = stream.column('ax')
    _t, ay = stream.column('ay')
    _t, az = stream.column('az')
    values = [math.sqrt(x * x + y * y + z * z) for x, y, z in zip(ax, ay, az)]
    values = [v for v in values if v == v and v != float('inf')]  # drop corrupt (nan) records
    return max(values) if values else 0.0


def _accel_envelope(streams) -> None:
    """
    Report the measured G envelope and turn it into a KEEP/DROP verdict for the high-g backstop.

    The ADXL375 (±200 g) exists for ONE reason: to survive a shock the LSM6DSO32's ±32 g would clip.
    Whether it earns its mass, its SPI chip-select and its PCB area is a MEASUREMENT, not an opinion --
    so fly both, then read it off here. Only two outcomes matter:
      * the primary CLIPPED (peak at/near its rail) -> the backstop is load-bearing, keep it;
      * the backstop never saw more than the primary's range -> it recorded nothing the primary could
        not, and it is a candidate to drop when simplifying the board.

    Args:
        streams - the parsed capture streams.

    Returns:
        None; prints the envelope and the verdict.
    """
    find = flight_telemetry.find_stream
    primary = find(streams, 'ax', 'ay', 'az', 'gx', prefer='lsm') or find(streams, 'ax', 'ay', 'az', 'gx')
    backstop = find(streams, 'ax', 'ay', 'az', prefer='adxl')
    if backstop is primary:
        backstop = None
    primary_peak, primary_when, primary_n = _peak_g(primary)
    backstop_peak, backstop_when, backstop_n = _peak_g(backstop)
    if not primary_n and not backstop_n:
        return
    if primary_n:
        print('  peak |a| lsm  : %6.1f g at t=%.1fs (%d samples, +/-%.0f g range)'
              % (primary_peak, primary_when, primary_n, _PRIMARY_RANGE_G))
    if backstop_n:
        print('  peak |a| adxl : %6.1f g at t=%.1fs (%d samples, +/-%.0f g range)'
              % (backstop_peak, backstop_when, backstop_n, _BACKSTOP_RANGE_G))
        raw = _raw_peak(backstop)
        if raw > backstop_peak * 1.5 + 1.0:  # the max is far above anything that lasted 2 samples
            print('    (raw max %.1f g was a SINGLE sample -- treated as a glitch, not a shock;'
                  % raw)
            print('     a real event registers on consecutive samples and on the other accel too)')
    if primary_n and primary_peak >= _PRIMARY_RANGE_G * _CLIP_FRACTION:
        print('  high-g verdict: KEEP the +/-200 g backstop -- the primary reached %.1f g, at/near its '
              '+/-%.0f g rail (clipping)' % (primary_peak, _PRIMARY_RANGE_G))
    elif backstop_n and backstop_peak <= _PRIMARY_RANGE_G:
        print('  high-g verdict: DROP candidate -- the backstop never exceeded %.1f g, inside the '
              'primary\'s +/-%.0f g range (it recorded nothing the primary could not)'
              % (backstop_peak, _PRIMARY_RANGE_G))
    elif backstop_n:
        print('  high-g verdict: KEEP -- the backstop saw %.1f g, beyond the primary\'s +/-%.0f g range'
              % (backstop_peak, _PRIMARY_RANGE_G))


def report(label: str, path: str, zone: tuple) -> None:
    """Print the KPI block for one capture."""
    with open(path) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    fins = next((s for name, s in streams.items() if 'fins' in name), None)
    print(label)
    _accel_envelope(streams)  # the G envelope + the high-g KEEP/DROP verdict (device-count decision)
    if fins is None:
        print('  (no fins stream)')
        return
    rows, span = _fin_activity(fins)
    total_moves = sum(moves for _f, _n, moves, _t, _b in rows)
    total_travel = sum(travel for _f, _n, _m, travel, _b in rows)
    for fin, samples, moves, travel, biggest in rows:
        print('  %-13s: %5d samples, %5d moves (%4.1f/s), travel %6.0f deg, max step %3.0f deg'
              % (fin, samples, moves, moves / span, travel, biggest))
    print('  TOTAL        : %17d moves,          travel %6.0f deg = %5.0f deg/s of flight'
          % (total_moves, total_travel, total_travel / span))
    power = next((s for name, s in streams.items() if 'power' in name), None)
    if power is not None:
        joules, duration = _servo_energy(power)
        watts = joules / duration if duration > 0 else 0.0  # single sample has no window to average over
        print('  servo energy : %6.1f J over %.1f s -> average %4.2f W' % (joules, duration, watts))
    gnss = next((s for name, s in streams.items() if 'gnss' in name), None)
    if gnss is not None:
        miss, inside = _touchdown(gnss, zone)
        print('  touchdown    : %6.1f m from zone centre, inside zone: %s' % (miss, inside))


def main() -> None:
    parser = argparse.ArgumentParser(description='Guidance-effectiveness KPIs for flight captures.')
    parser.add_argument('captures', nargs='+', help='LABEL:capture.txt (label shown as the block header)')
    parser.add_argument('--zone', default=_ZONE_DEFAULT,
                        help='landing zone lat1,lon1,lat2,lon2 (TL, BR; default: the HPRC test zone)')
    args = parser.parse_args()
    lat_t, lon_l, lat_b, lon_r = (float(value) for value in args.zone.split(','))
    zone = ((lat_t, lon_l), (lat_b, lon_r))
    for spec in args.captures:
        label, _, path = spec.partition(':')
        report(label if path else os.path.basename(spec), path or spec, zone)


if __name__ == '__main__':
    main()
