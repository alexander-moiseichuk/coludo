"""
Cut one flight out of a nose-logger boot, zero its clock at ignition, and measure it.

Host-side (CPython, stdlib only). decode.py turns each ~15 s segment into its own CSV on its own clock; a
flight wants the opposite -- ONE file on ONE time axis, t = 0 at ignition, from a few seconds of pad to
a few seconds of rest after landing -- so no analysis has to stitch segments or find the launch again.
Pass every segment that holds the flight (all of one boot); the pad before and the recovery after are
cut away.

    python3 src/logger/flight.py launches/20261003/TMS-7/logger/*.bin -o launches/20261003/TMS-7/flight.csv \\
        --mass 0.2283 --propellant 0.060

Prints the flight's numbers: events, peaks, speeds, apogee, descent. --mass (liftoff, kg) and
--propellant (kg) add the thrust estimate. Plots come from flight_plots.py, which reads the CSV back with
read_csv() and measures it with the same analyse().

INERTIAL vs BARO. The barometer sits inside the nose cone, and at speed the air in there is not at the
static pressure outside: on TMS-7 it read 70 m HIGH three seconds into the flight. So speeds and heights
up to the ejection come from strapdown integration -- gyro attitude from the pad's gravity vector,
accelerometer integrated in that frame -- and the baro is trusted where the airframe is slow: apogee,
the descent, the ground. The strapdown stops at the ejection: past it the airframe tumbles faster than
the gyro's +/-2000 dps range.
"""

import argparse
import csv
import itertools
import math
import os
import statistics
import sys
from collections import namedtuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode  # noqa: E402 -- the logger's record format and scaling, never restated here

_GRAVITY: float = 9.80665       # m/s^2
_R_AIR: float = 287.05          # J/(kg K), dry air
_GAMMA_AIR: float = 1.4         # ratio of specific heats
_BOOST_G: float = 3.0           # |a| only a burning motor sustains ...
_BOOST_HOLD_S: float = 0.3      # ... for this long
_ONSET_G: float = 1.1           # the boost starts at the first sample of its rise above this
_PAD_S: tuple = (-5.0, -0.5)    # the window before ignition every reference comes from
_STILL_G: float = 0.05          # at rest: |a| within this of 1 g,
_STILL_DPS: float = 30.0        # every gyro axis under this,
_STILL_M: float = 1.5           # the height inside this band,
_STILL_S: float = 2.0           # for this long
_EJECTION_G: float = 3.0        # the first shock this size a second after burnout is the ejection charge
_GYRO_RAIL_DPS: float = 1999.0  # BMI323 full scale is +/-2000 dps
_TOUCHDOWN_M: float = 0.5       # touchdown: back within this of the elevation it comes to rest at
_MEDIAN: int = 5                # samples in the spike filter on the baro height (an impact glitches it)
_APOGEE_MEDIAN: int = 25        # ~0.5 s: the baro apogee is read from this smoothing, never one sample
_RATE_S: float = 2.0            # descent-rate regression window
_BAND_S: float = 10.0           # descent reported per slice of this length
_DRAG_MIN_MS: float = 30.0      # the drag constant is fitted from coast samples faster than this

_COLUMNS: tuple = ('t_s', 'ax_g', 'ay_g', 'az_g', 'a_g', 'gx_dps', 'gy_dps', 'gz_dps', 'pressure_pa',
                   'temp_c', 'alt_m', 'segment', 'tick_ms')

Sample = namedtuple('Sample', 't accel gyro pascal celsius height segment tick')
Sample.__doc__ = """
One record: t s from ignition, accel g and gyro dps as (x, y, z) or None where the chip had no sample,
pressure Pa, temperature C, height m above the pad (baro; None without a pressure), the segment it was
saved in and its sensor-clock tick in ms.
"""


def _magnitude(vector) -> float:
    """Euclidean length."""
    return math.sqrt(sum(component * component for component in vector))


def _sustained(samples: list, index: int) -> bool:
    """True when |a| stays above _BOOST_G for _BOOST_HOLD_S from `index` -- a motor, not a knock."""
    start = samples[index].t
    for sample in itertools.islice(samples, index, None):
        if sample.t - start > _BOOST_HOLD_S:
            return True
        if sample.accel is not None and _magnitude(sample.accel) <= _BOOST_G:
            return False
    return False


def _ignition(samples: list) -> int:
    """Index of the first sample of the boost: the first sustained _BOOST_G, walked back to where |a| left 1 g."""
    for index, sample in enumerate(samples):
        if sample.accel is not None and _magnitude(sample.accel) > _BOOST_G and _sustained(samples, index):
            while index and samples[index - 1].accel is not None and \
                    _magnitude(samples[index - 1].accel) > _ONSET_G:
                index -= 1
            return index
    raise ValueError('no boost: |a| never held %.1f g for %.1f s' % (_BOOST_G, _BOOST_HOLD_S))


def _still(sample: Sample) -> bool:
    """At rest by accel and gyro alone (the height band is checked over the window)."""
    return sample.accel is not None and sample.gyro is not None and \
        abs(_magnitude(sample.accel) - 1.0) < _STILL_G and max(abs(w) for w in sample.gyro) < _STILL_DPS


def _rest(samples: list, start: int) -> int:
    """
    Index from which the airframe is at rest for _STILL_S: still by accel and gyro, AND not descending.

    The height band is what separates rest from a calm stretch under the chute: hanging still at 1 g is
    exactly what a stable parachute descent looks like to an accelerometer.
    """
    first = None
    for index in range(start, len(samples)):
        sample = samples[index]
        if not _still(sample) or sample.height is None:
            first = None
            continue
        if first is None:
            first = index
        heights = [s.height for s in samples[first:index + 1] if s.height is not None]
        if max(heights) - min(heights) > _STILL_M:
            first = index
        elif sample.t - samples[first].t >= _STILL_S:
            return first
    raise ValueError('the airframe never comes to rest')


def cut(paths: list, before_s: float, after_s: float) -> list:
    """
    The flight from `paths` (segments of ONE boot) as Samples, t = 0 at ignition, from `before_s` ahead of
    it to `after_s` past the moment it comes to rest. Heights are against the mean pad pressure.
    """
    loaded = []
    for path in paths:
        with open(path, 'rb') as handle:
            loaded.append(decode.records(handle.read()))
    boots = sorted({header['boot'] for header, _rows in loaded})
    if len(boots) != 1:
        raise ValueError('a flight is one boot; these files hold boots %s' % boots)
    loaded.sort(key=lambda item: item[0]['segment'])
    samples = []
    for header, rows in loaded:
        for row in rows:
            ms, accel, gyro, pascal, celsius = decode.physical(row)
            samples.append(Sample(ms / 1000.0, accel, gyro, pascal, celsius, None, header['segment'], ms))
    start = _ignition(samples)
    zero = samples[start].t
    samples = [sample._replace(t=sample.t - zero) for sample in samples]
    pad = [s for s in samples if _PAD_S[0] <= s.t <= _PAD_S[1] and s.pascal is not None]
    if not pad:
        raise ValueError('no pressure on the pad %.1f..%.1f s before ignition' % _PAD_S)
    reference = statistics.mean(s.pascal for s in pad)
    reference_celsius = statistics.mean(s.celsius for s in pad if s.celsius is not None)
    samples = [s._replace(height=None if s.pascal is None else
                          decode.altitude(s.pascal, reference, s.celsius, reference_celsius)) for s in samples]
    rest = samples[_rest(samples, start)].t
    return [s for s in samples if -before_s <= s.t <= rest + after_s]


def _cell(value, fmt: str) -> str:
    """A formatted reading, or an empty cell where there was none."""
    return '' if value is None else fmt % value


def write_csv(samples: list, path: str) -> None:
    """One row per sample, _COLUMNS."""
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(_COLUMNS)
        for s in samples:
            accel = s.accel or (None, None, None)
            gyro = s.gyro or (None, None, None)
            writer.writerow(['%.3f' % s.t] + [_cell(v, '%.4f') for v in accel] +
                            [_cell(None if s.accel is None else _magnitude(s.accel), '%.4f')] +
                            [_cell(v, '%.2f') for v in gyro] +
                            [_cell(s.pascal, '%.2f'), _cell(s.celsius, '%.2f'), _cell(s.height, '%.2f'),
                             s.segment, s.tick])


def read_csv(path: str) -> list:
    """The Samples write_csv() wrote."""
    def number(text):
        return float(text) if text else None

    samples = []
    with open(path, newline='') as handle:
        for row in csv.DictReader(handle):
            accel = None if not row['ax_g'] else tuple(float(row[k]) for k in ('ax_g', 'ay_g', 'az_g'))
            gyro = None if not row['gx_dps'] else tuple(float(row[k]) for k in ('gx_dps', 'gy_dps', 'gz_dps'))
            samples.append(Sample(float(row['t_s']), accel, gyro, number(row['pressure_pa']),
                                  number(row['temp_c']), number(row['alt_m']), int(row['segment']),
                                  int(row['tick_ms'])))
    return samples


def _multiply(a: tuple, b: tuple) -> tuple:
    """Hamilton product of two quaternions (w, x, y, z)."""
    return (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])


def _rotate(quaternion: tuple, vector) -> tuple:
    """`vector` (body) in the world frame."""
    conjugate = (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])
    return _multiply(_multiply(quaternion, (0.0,) + tuple(vector)), conjugate)[1:]


def _levelling(up: list) -> tuple:
    """The quaternion turning the body's measured `up` onto world z (heading arbitrary: no magnetometer)."""
    axis = (up[1], -up[0], 0.0)                  # up x z
    size = _magnitude(axis)
    if size < 1e-9:
        return (1.0, 0.0, 0.0, 0.0) if up[2] > 0 else (0.0, 1.0, 0.0, 0.0)
    half = math.acos(max(-1.0, min(1.0, up[2]))) / 2.0
    return (math.cos(half),) + tuple(c / size * math.sin(half) for c in axis)


def _world(quaternion: tuple, accel) -> tuple:
    """Acceleration in the world frame, m/s^2: the specific force rotated out of the body, minus gravity."""
    force = _rotate(quaternion, accel)
    return (force[0] * _GRAVITY, force[1] * _GRAVITY, force[2] * _GRAVITY - _GRAVITY)


def _strapdown(samples: list, start: int, stop: int, up: list, bias: list, axis: list) -> list:
    """
    Dead reckoning from the pad (index `start`, at rest) up to `stop`, exclusive. Per sample: (t, height,
    vertical speed, horizontal speed, speed, horizontal distance, distance travelled, zenith angle of the
    thrust axis). Gyro bias from the pad is removed; a sample missing a sensor reuses its last reading.

    Trapezoidal between samples -- the mean of the two rates turns the attitude, the mean of the two
    accelerations moves the velocity -- because end-of-step values lead on the 15 g/0.3 s ramp: the
    off-rod speeds read ~1 m/s high that way.
    """
    quaternion = _levelling(up)
    velocity, position, travelled = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0
    accel, gyro = samples[start].accel, samples[start].gyro
    rate = [math.radians(w - b) for w, b in zip(gyro, bias)]
    acceleration = _world(quaternion, accel)
    track = []
    for index in range(start, stop):
        sample = samples[index]
        if index > start:
            accel, gyro = sample.accel or accel, sample.gyro or gyro
            step = sample.t - samples[index - 1].t
            previous_rate, previous_acceleration, previous_velocity = rate, acceleration, velocity
            rate = [math.radians(w - b) for w, b in zip(gyro, bias)]
            turn = [(a + b) / 2.0 for a, b in zip(previous_rate, rate)]
            angle = _magnitude(turn) * step
            if angle:
                half = angle / 2.0
                quaternion = _multiply(quaternion, (math.cos(half),) + tuple(
                    r / _magnitude(turn) * math.sin(half) for r in turn))
            acceleration = _world(quaternion, accel)
            velocity = tuple(v + (a + b) / 2.0 * step
                             for v, a, b in zip(previous_velocity, previous_acceleration, acceleration))
            position = tuple(p + (a + b) / 2.0 * step for p, a, b in zip(position, previous_velocity, velocity))
            travelled += (_magnitude(previous_velocity) + _magnitude(velocity)) / 2.0 * step
        nose = _rotate(quaternion, axis)
        track.append((sample.t, position[2], velocity[2], math.hypot(velocity[0], velocity[1]),
                      _magnitude(velocity), math.hypot(position[0], position[1]), travelled,
                      math.degrees(math.acos(max(-1.0, min(1.0, nose[2]))))))
    return track


def _at_travel(track: list, metres: float):
    """The track row interpolated to `metres` travelled, or None if it never got that far."""
    for before, after in zip(track, track[1:]):
        if after[6] >= metres:
            share = (metres - before[6]) / (after[6] - before[6]) if after[6] > before[6] else 1.0
            return tuple(b + (a - b) * share for b, a in zip(before, after))
    return None


def _median_filter(values: list, width: int) -> list:
    """Running median over `width` samples (None-tolerant), centred."""
    half = width // 2
    out = []
    for index in range(len(values)):
        window = [v for v in values[max(0, index - half):index + half + 1] if v is not None]
        out.append(statistics.median(window) if window else None)
    return out


def _slope(points: list) -> float:
    """Least-squares slope of (x, y) points."""
    mean_x = statistics.mean(x for x, _y in points)
    mean_y = statistics.mean(y for _x, y in points)
    spread = sum((x - mean_x) ** 2 for x, _y in points)
    return sum((x - mean_x) * (y - mean_y) for x, y in points) / spread if spread else 0.0


def _thrust(samples: list, axial: list, zero: int, burnout: int, track: list, coast: list,
            mass: float, propellant: float, density: float) -> dict:
    """
    Thrust = mass x specific force along the thrust axis + drag. The accelerometer cannot see gravity, so
    its axial reading is (thrust - drag) / mass; drag is k v^2 with k fitted from the coast, where thrust
    is zero and the reading IS -drag / mass. Mass falls from `mass` to `mass - propellant` in proportion
    to the impulse delivered so far (constant specific impulse) -- solved by iterating twice.
    """
    burnout_mass = mass - propellant
    fits = [-axial[index] * _GRAVITY * burnout_mass / speed ** 2 for index, speed in coast
            if speed > _DRAG_MIN_MS and axial[index] is not None]
    drag = statistics.median(fits)
    times = [samples[index].t for index in range(zero, burnout + 1)]
    speeds = [track[index - zero][4] for index in range(zero, burnout + 1)]
    share = [(t - times[0]) / (times[-1] - times[0]) for t in times]   # first pass: linear in time
    for _iteration in range(2):
        thrust = [(mass - propellant * fraction) * _GRAVITY * (axial[zero + i] or 0.0) + drag * speed ** 2
                  for i, (fraction, speed) in enumerate(zip(share, speeds))]
        delivered = [0.0]
        for i in range(1, len(times)):
            delivered.append(delivered[-1] + (thrust[i] + thrust[i - 1]) / 2.0 * (times[i] - times[i - 1]))
        share = [d / delivered[-1] for d in delivered]
    return {'t': times, 'newton': thrust, 'impulse': delivered[-1], 'peak': max(thrust),
            'average': delivered[-1] / (times[-1] - times[0]), 'drag_k': drag, 'drag_area': 2.0 * drag / density,
            'mass': mass, 'propellant': propellant}


def analyse(samples: list, mass: float = None, propellant: float = None) -> dict:
    """
    Every number and series the summary and the plots use. See the module docstring for which sensor
    each comes from; `mass` and `propellant` (kg) are only for the thrust estimate.
    """
    zero = next(index for index, s in enumerate(samples) if s.t >= 0.0)
    pad = [s for s in samples if _PAD_S[0] <= s.t <= _PAD_S[1]]
    gravity = [statistics.mean(s.accel[axis] for s in pad if s.accel) for axis in range(3)]
    up = [component / _magnitude(gravity) for component in gravity]
    bias = [statistics.mean(s.gyro[axis] for s in pad if s.gyro) for axis in range(3)]
    kelvin = statistics.mean(s.celsius for s in pad if s.celsius is not None) + 273.15
    pascal = statistics.mean(s.pascal for s in pad if s.pascal is not None)

    def along(axis):
        return [None if s.accel is None else sum(a * u for a, u in zip(s.accel, axis)) for s in samples]

    # the thrust axis: first take `up` (the airframe stands on its tail), find the burn, then refine it to
    # the mean direction of the specific force while the motor dominates -- the airframe's own long axis
    axial = along(up)
    burnout = next(i for i in range(zero, len(samples)) if samples[i].t > 0.1 and axial[i] is not None
                   and axial[i] < 0.0)
    burning = [s.accel for s in samples[zero:burnout] if s.accel and s.t > 0.1]
    mean = [statistics.mean(a[axis] for a in burning) for axis in range(3)]
    axis = [component / _magnitude(mean) for component in mean]
    axial = along(axis)
    burnout = next(i for i in range(zero, len(samples)) if samples[i].t > 0.1 and axial[i] is not None
                   and axial[i] < 0.0)
    peak = max(range(zero, burnout), key=lambda i: axial[i] or 0.0)
    ejection = next(i for i in range(burnout, len(samples)) if samples[i].t - samples[burnout].t > 1.0
                    and samples[i].accel is not None and _magnitude(samples[i].accel) > _EJECTION_G)
    track = _strapdown(samples, zero, ejection, up, bias, axis)
    fastest = max(range(len(track)), key=lambda i: track[i][4])
    sound = math.sqrt(_GAMMA_AIR * _R_AIR * kelvin)
    off_rod = {metres: _at_travel(track, metres) for metres in (1.0, 2.0)}
    last = track[-1]
    climbing = max(last[2], 0.0)
    inertial_apogee = (last[1] + climbing ** 2 / (2.0 * _GRAVITY), last[0] + climbing / _GRAVITY)

    heights = [s.height for s in samples]
    smooth = _median_filter(heights, _APOGEE_MEDIAN)
    window = [i for i in range(burnout, len(samples)) if samples[i].t <= samples[ejection].t + 5.0
              and smooth[i] is not None]
    top = max(window, key=lambda i: smooth[i])
    rest = _rest(samples, ejection)
    elevation = statistics.median(s.height for s in samples[rest:]
                                  if s.height is not None and s.t <= samples[rest].t + _STILL_S)
    filtered = _median_filter(heights, _MEDIAN)
    above = [i for i in range(ejection, rest) if filtered[i] is not None and
             filtered[i] > elevation + _TOUCHDOWN_M]
    touchdown = above[-1] + 1

    # the descent: from the chute opening to just before the ground
    descent = [i for i in range(ejection, touchdown) if samples[ejection].t + 2.0 <= samples[i].t
               <= samples[touchdown].t - 0.5]
    rate = []
    for index in descent[::10]:
        points = [(samples[i].t, filtered[i]) for i in descent
                  if abs(samples[i].t - samples[index].t) <= _RATE_S / 2.0 and filtered[i] is not None]
        rate.append((samples[index].t, -_slope(points)))
    bands = []
    start = samples[descent[0]].t
    while start < samples[descent[-1]].t:
        inside = [i for i in descent if start <= samples[i].t < start + _BAND_S and filtered[i] is not None]
        if len(inside) > 1:
            first, final = inside[0], inside[-1]
            bands.append((samples[first].t, samples[final].t, filtered[first], filtered[final],
                          (filtered[first] - filtered[final]) / (samples[final].t - samples[first].t)))
        start += _BAND_S
    first, final = descent[0], descent[-1]
    tumble = [s.gyro for s in samples[first:final + 1] if s.gyro]
    shock = max(range(first, touchdown), key=lambda i: _magnitude(samples[i].accel) if samples[i].accel else 0.0)

    result = {
        'samples': samples, 'axial': axial, 'filtered': filtered, 'smooth': smooth, 'track': track,
        'rate': rate, 'bands': bands,
        'pad': {'up': up, 'tilt': math.degrees(math.acos(abs(sum(u * a for u, a in zip(up, axis))))),
                'zenith': track[0][7], 'bias': bias, 'pascal': pascal, 'celsius': kelvin - 273.15},
        'axis': axis,
        'peak': (samples[peak].t, axial[peak], _magnitude(samples[peak].accel)),
        'burnout': samples[burnout].t,
        'fastest': track[fastest], 'mach': track[fastest][4] / sound,
        'off_rod': off_rod,
        'ejection': (samples[ejection].t, _magnitude(samples[ejection].accel), last),
        'apogee': {'baro': (smooth[top], samples[top].t), 'inertial': inertial_apogee},
        'descent': ((filtered[first] - filtered[final]) / (samples[final].t - samples[first].t),
                    samples[first].t, samples[final].t),
        'tumble': {'mean': [statistics.mean(g[a] for g in tumble) for a in range(3)],
                   'rms': [math.sqrt(statistics.mean(g[a] ** 2 for g in tumble)) for a in range(3)],
                   'railed': sum(1 for g in tumble if max(abs(w) for w in g) >= _GYRO_RAIL_DPS) / len(tumble)},
        'shock': (samples[shock].t, _magnitude(samples[shock].accel), filtered[shock]),
        'touchdown': samples[touchdown].t, 'rest': (samples[rest].t, elevation),
        'thrust': None,
    }
    if mass is not None and propellant is not None:
        coast = [(index, track[index - zero][4]) for index in range(burnout + 5, ejection)
                 if samples[index].t < samples[ejection].t - 1.0]
        result['thrust'] = _thrust(samples, axial, zero, burnout, track, coast, mass, propellant,
                                   pascal / (_R_AIR * kelvin))
    return result


def summary(flight: dict) -> str:
    """The flight's numbers as text."""
    samples, pad = flight['samples'], flight['pad']
    lines = ['%d samples, %+.1f..%+.1f s, t = 0 at ignition (segment %d, sensor tick %d ms)' % (
        len(samples), samples[0].t, samples[-1].t,
        *next((s.segment, s.tick) for s in samples if s.t >= 0.0))]
    lines.append('pad       %.1f deg from vertical (thrust axis); %.1f Pa, %.1f C; gyro bias %s dps' % (
        pad['zenith'], pad['pascal'], pad['celsius'], ', '.join('%+.2f' % b for b in pad['bias'])))
    t, axial, total = flight['peak']
    lines.append('boost     peak %.2f g along the thrust axis (|a| %.2f) at %.2f s; thrust < drag at %.2f s'
                 % (axial, total, t, flight['burnout']))
    for metres, row in flight['off_rod'].items():
        if row:
            lines.append('          %.0f m of travel at %.2f s, %.1f m/s' % (metres, row[0], row[4]))
    fast = flight['fastest']
    lines.append('speed     %.0f m/s at %.2f s (Mach %.2f): %.0f vertical, %.0f horizontal' % (
        fast[4], fast[0], flight['mach'], fast[2], fast[3]))
    t, shock, last = flight['ejection']
    lines.append('ejection  %.2f s (%.1f g), %.1f m/s still climbing, %.0f m up, %.0f m downrange, '
                 '%.0f deg from vertical' % (t, shock, last[2], last[1], last[5], last[7]))
    baro, inertial = flight['apogee']['baro'], flight['apogee']['inertial']
    lines.append('apogee    baro %.0f m at %.2f s; inertial %.0f m at %.2f s' % (baro[0], baro[1], *inertial))
    rate, start, end = flight['descent']
    lines.append('descent   %.1f m/s mean over %.1f..%.1f s' % (rate, start, end))
    for band in flight['bands']:
        lines.append('          %5.1f..%5.1f s  %5.0f -> %5.0f m  %4.1f m/s' % band)
    tumble = flight['tumble']
    lines.append('tumble    mean %s dps, rms %s dps; %.1f%% of samples at the gyro rail' % (
        ', '.join('%+.0f' % m for m in tumble['mean']), ', '.join('%.0f' % r for r in tumble['rms']),
        100.0 * tumble['railed']))
    t, shock, height = flight['shock']
    lines.append('shock     %.1f g at %.2f s, %.1f m above the pad (largest of the descent)' % (shock, t, height))
    lines.append('ground    touchdown %.2f s; at rest from %.2f s, %+.1f m vs the pad' % (
        flight['touchdown'], *flight['rest']))
    thrust = flight['thrust']
    if thrust:
        lines.append('thrust    (%.1f g at liftoff, %.1f g propellant) peak %.1f N, impulse %.1f N s over %.2f s = '
                     '%.1f N average; drag k %.2e kg/m, CdA %.2f cm^2' % (
                         thrust['mass'] * 1e3, thrust['propellant'] * 1e3, thrust['peak'], thrust['impulse'],
                         thrust['t'][-1], thrust['average'], thrust['drag_k'], thrust['drag_area'] * 1e4))
    return '\n'.join(lines)


def main() -> None:
    """Command line: cut, write, measure."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('segments', nargs='+', help='the boot\'s .bin segments')
    parser.add_argument('-o', '--out', required=True, help='the flight CSV to write')
    parser.add_argument('--before', type=float, default=10.0, help='seconds of pad kept before ignition')
    parser.add_argument('--after', type=float, default=10.0, help='seconds kept after coming to rest')
    parser.add_argument('--mass', type=float, help='liftoff mass, kg (with --propellant: thrust estimate)')
    parser.add_argument('--propellant', type=float, help='propellant mass, kg')
    args = parser.parse_args()
    samples = cut(sorted(args.segments), args.before, args.after)
    write_csv(samples, args.out)
    print('%s:' % args.out)
    print(summary(analyse(samples, args.mass, args.propellant)))


if __name__ == '__main__':
    main()
