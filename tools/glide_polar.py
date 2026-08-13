"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Measure the airframe's GLIDE QUALITY (L/D) from a flight capture -- the number the whole simulation
rests on.

`sim_model.AIR_QUALITY` was a guess for most of this project's life: L/D 2, the deliberately
pessimistic worst-case floor (7.0 m/s of trim sink). That difference decides real things -- the
endgame study (doc/sims/TMS-7_endgame_x3) found the converging 'ov' pattern WINS at quality 5 and
LOSES at quality 2 -- so every landing-accuracy claim rested on it.

It is now measured: **5.5**, set on 2026-08-12 from a hand-toss glide ratio (TMS-7B, 8.0 m of glide
from a 1.40 m release against a 2.70 m ballistic reference thrown with an inert dummy of equal mass).
That measurement is crude by this tool's standards -- one number from a 1.5 s flight, below trim speed
-- which is exactly why this tool still matters: the first capture with a few seconds of steady glide
supersedes it, and the printout below flags any disagreement with the shipped constant.

L/D = horizontal speed / sink rate, and crucially that needs only a STEADY SEGMENT, not a long glide:
a catapult launch from 1-2 m gives a couple of seconds, which is plenty when the sink is ~1.8 m/s and
the baro resolves ~0.085 m. Distance-on-the-ground cannot do this at low altitude -- the catapult's own
ballistic throw is several metres, so a glide and a thrown brick look identical.

AIRSPEED, not ground speed: the pitot (airspeed_sdp810) measures through the AIR, so the result is
wind-independent. Falling back to GNSS ground speed works but folds the wind straight into the answer,
so it is reported as degraded.

  python3 tools/glide_polar.py LABEL:capture.txt [LABEL:capture.txt ...]
  python3 tools/glide_polar.py run1:a.txt --from 2.5 --to 6.0     # manual window
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
import flight_telemetry  # noqa: E402
import sim_model  # noqa: E402

_MIN_SPAN_S: float = 0.8      # below this a sink fit is noise, not a measurement
_SETTLE_S: float = 0.4        # skip after the apex: the launch transient is not a glide
_SIM_TRIM_MS: float = sim_model.TRIM_SPEED_MS  # the sim's trim speed -- trim_sink = it / (L/D)


def _fit_sink(times: list, elevation: list) -> tuple:
    """
    Least-squares sink rate over a segment.

    Args:
        times - sample times (s).
        elevation - height (m) at each sample.

    Returns:
        (sink_ms, r_squared): sink is POSITIVE for descending flight; r² reports how straight the
        descent was, which is the honest way to see a segment that was not actually a steady glide.
    """
    count = len(times)
    mean_t = sum(times) / count
    mean_h = sum(elevation) / count
    covariance = sum((t - mean_t) * (h - mean_h) for t, h in zip(times, elevation))
    variance = sum((t - mean_t) ** 2 for t in times)
    slope = covariance / variance if variance else 0.0
    residual = sum((h - (mean_h + slope * (t - mean_t))) ** 2 for t, h in zip(times, elevation))
    total = sum((h - mean_h) ** 2 for h in elevation)
    return -slope, (1.0 - residual / total) if total else 0.0


def _glide_window(times: list, elevation: list, start: float, end: float) -> tuple:
    """
    The steady-descent segment: from just after the apex to the last still-falling sample.

    A catapult launch climbs, arcs over, then glides. The apex splits the two, and the first moments
    after it are still the launch transient (the airframe is trading the launch overspeed for trim), so
    they are skipped rather than averaged in.

    Args:
        times - sample times (s).
        elevation - height (m).
        start - manual window start (s), or None to auto-detect from the apex.
        end - manual window end (s), or None to run to the end of the descent.

    Returns:
        (from_s, to_s) or (None, None) when there is no usable descent.
    """
    if start is not None and end is not None:
        return start, end
    apex = max(range(len(elevation)), key=lambda i: elevation[i])
    begin = times[apex] + _SETTLE_S if start is None else start
    if end is None:  # run to the lowest point -- past that it is on the ground, not gliding
        lowest = min(range(apex, len(elevation)), key=lambda i: elevation[i]) if apex < len(elevation) else apex
        end = times[lowest]
    return (begin, end) if end - begin >= _MIN_SPAN_S else (None, None)


def analyse(streams, start: float, end: float) -> dict:
    """
    Extract the glide-quality numbers from one capture.

    Args:
        streams - parsed telemetry streams.
        start - manual window start (s) or None.
        end - manual window end (s) or None.

    Returns:
        A result dict (see report); {'error': ...} when the capture cannot answer.
    """
    find = flight_telemetry.find_stream
    baro = find(streams, 'elevation', prefer='icp') or find(streams, 'elevation') or find(streams, 'altitude')
    if baro is None:
        return {'error': 'no baro stream -- cannot measure sink'}
    field = 'elevation' if 'elevation' in baro.fields else 'altitude'
    times, elevation = baro.column(field)
    if len(times) < 4:
        return {'error': 'baro stream too short (%d samples)' % len(times)}
    begin, finish = _glide_window(times, elevation, start, end)
    if begin is None:
        return {'error': 'no steady descent of at least %.1f s found' % _MIN_SPAN_S}
    window = [(t, h) for t, h in zip(times, elevation) if begin <= t <= finish]
    if len(window) < 4:
        return {'error': 'only %d baro samples inside the glide window' % len(window)}
    sink, quality = _fit_sink([t for t, _h in window], [h for _t, h in window])

    pitot = find(streams, 'dynamic_pressure')
    source = 'pitot (true airspeed, wind-independent)'
    speeds = []
    if pitot is not None and 'airspeed_cms' in pitot.fields:      # cm/s integer (doc/telemetry.md)
        speeds = [v / 100.0 for t, v in zip(*pitot.column('airspeed_cms')) if begin <= t <= finish]
    elif pitot is not None and 'airspeed' in pitot.fields:        # pre-fixnum captures, m/s float
        speeds = [v for t, v in zip(*pitot.column('airspeed')) if begin <= t <= finish]
    if not speeds:
        gnss = find(streams, 'lat', 'lon')
        if gnss is not None and 'speed_kn' in gnss.fields:
            speeds = [k / 1.94384 for t, k in zip(*gnss.column('speed_kn')) if begin <= t <= finish]
            source = 'GNSS ground speed (DEGRADED: wind folds into the result)'
    if not speeds:
        return {'error': 'no airspeed or ground speed in the glide window'}
    airspeed = sum(speeds) / len(speeds)
    if sink <= 0:
        return {'error': 'the window is not descending (sink %.2f m/s) -- pick one with --from/--to' % sink}
    horizontal = math.sqrt(max(airspeed ** 2 - sink ** 2, 0.0))  # airspeed is along the path, not level
    """
    A BANKED glide sinks faster: a turn pulls load factor n = 1/cos(bank) and induced drag scales
    n^1.5, so averaging through turns UNDER-reports L/D. Report the mean bank so a turning segment is
    visible, and (when attitude is present) the straight-flight equivalent alongside the raw number.
    """
    attitude = find(streams, 'roll', 'pitch', prefer='bno')
    banks = []
    if attitude is not None:
        rolls = attitude.column('roll')
        # attitude is CENTIDEGREE fixnum (doc/telemetry.md); detect by magnitude so a degrees-valued
        # capture still reads correctly rather than silently reporting 100x the bank
        scale = 100.0 if rolls[1] and max(abs(v) for v in rolls[1]) > 200 else 1.0
        banks = [abs(r) / scale for t, r in zip(*rolls) if begin <= t <= finish]
    bank = sum(banks) / len(banks) if banks else 0.0
    load = 1.0 / max(math.cos(math.radians(min(bank, 60.0))), 0.3)
    return {'from': begin, 'to': finish, 'span': finish - begin, 'samples': len(window),
            'sink': sink, 'r2': quality, 'airspeed': airspeed, 'source': source, 'bank': bank,
            'lift_drag': horizontal / sink, 'angle': math.degrees(math.atan2(sink, horizontal)),
            'level_lift_drag': horizontal / (sink / load ** 1.5),
            'dropped': elevation[0] - elevation[-1]}


def report(label: str, path: str, start: float, end: float) -> None:
    """Print the glide-quality block for one capture."""
    with open(path) as handle:
        streams, _logs = flight_telemetry.parse(handle.read())
    print(label)
    result = analyse(streams, start, end)
    if 'error' in result:
        print('  cannot measure: %s' % result['error'])
        return
    print('  glide window : %.2f - %.2f s (%.2f s, %d baro samples)'
          % (result['from'], result['to'], result['span'], result['samples']))
    print('  airspeed     : %5.2f m/s   [%s]' % (result['airspeed'], result['source']))
    print('  sink rate    : %5.2f m/s   (straight-line fit r2 = %.3f)' % (result['sink'], result['r2']))
    print('  glide angle  : %5.2f deg below horizontal' % result['angle'])
    print('  mean bank    : %5.1f deg in the window' % result['bank'])
    print('  ---> L/D     : %5.2f  (as flown)' % result['lift_drag'])
    if result['bank'] > 5.0:
        print('       L/D     : %5.2f  (straight-flight equivalent, de-rating the n^1.5 turn drag)'
              % result['level_lift_drag'])
        print('  NOTE: %.0f deg of bank -- a turning glide sinks faster, so the as-flown number is a\n'
              '        FLOOR. Measure a straight glide for the airframe figure.' % result['bank'])
    if result['r2'] < 0.9:
        print('  WARNING: r2 %.2f -- that segment was not a steady glide, so L/D is unreliable.'
              % result['r2'])
    if result['span'] < 2.0:
        print('  NOTE: only %.1f s of glide. Launch from more height for a tighter number.'
              % result['span'])
    """
    Close the loop on the sim: `VF_QUALITY` IS the L/D (sim_model.trim_sink = TRIM_SPEED_MS / quality),
    so a measured L/D drops straight into every study. This is what retired the old guess -- a hand-toss
    measurement set sim_model.AIR_QUALITY to 5.5 on 2026-08-12; print the delta so a later capture that
    disagrees with the shipped number is visible rather than quietly divergent.
    """
    measured_ld = result['lift_drag']
    print('  sim calibration: VF_QUALITY=%.1f  (sim_model.trim_sink = %.2f at its %.0f m/s trim)'
          % (measured_ld, _SIM_TRIM_MS / measured_ld, _SIM_TRIM_MS))
    print('  shipped sim_model.AIR_QUALITY = %.1f  -> this capture is %+.1f (%s)'
          % (sim_model.AIR_QUALITY, measured_ld - sim_model.AIR_QUALITY,
             'sim is pessimistic, the safe direction' if measured_ld >= sim_model.AIR_QUALITY
             else 'sim is OPTIMISTIC -- it promises more glide than the airframe delivers'))


def main() -> int:
    parser = argparse.ArgumentParser(description='Measure airframe glide quality (L/D) from a capture.')
    parser.add_argument('captures', nargs='+', help='LABEL:path.txt (or just path.txt)')
    parser.add_argument('--from', dest='start', type=float, default=None, help='glide window start (s)')
    parser.add_argument('--to', dest='end', type=float, default=None, help='glide window end (s)')
    args = parser.parse_args()
    for entry in args.captures:
        label, _, path = entry.rpartition(':')
        if not path:
            label, path = os.path.basename(entry), entry
        if not os.path.exists(path):
            print('%s\n  missing: %s' % (label or path, path), file=sys.stderr)
            continue
        report(label or os.path.basename(path), path, args.start, args.end)
    return 0


if __name__ == '__main__':
    sys.exit(main())
