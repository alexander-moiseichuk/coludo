"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

findings.md 31.6 validation: can the navigation/guidance/wind float trig move onto fixed.atan2_cd
and fixed.isqrt without hurting the control path? Analysis only -- this changes nothing, it MEASURES
the two things the finding says to check before porting:

  (a) ROUNDING BIAS. atan2_cd documents ~0.5 deg typical / 1.8 deg worst at the accel centi-scale,
      but the doc also says magnitude only trades precision -- and navigation feeds it metres, which
      are 100-1000x bigger than centi-g. So the error at NAVIGATION magnitudes is the number that
      matters, and a systematic bias (a mean error that does not average out) is worse than noise:
      it would steer the glider consistently to one side of the target.
  (b) RANGE. The CORDIC and isqrt are @micropython.viper, i.e. 32-bit machine ints. distance()
      squares its inputs, so the question is at what offset the square overflows and starts
      returning nonsense instead of raising.

Run on the board (viper semantics are the point -- a host run would not test the 32-bit wrap):
    mpremote run test/diag_fixed_nav.py
"""

import math

import fixed
import navigation

_RANGES = (10.0, 50.0, 200.0, 500.0, 1000.0, 2000.0)  # metres from target: pad to far downrange
_BEARINGS = tuple(range(0, 360, 3))  # 120 directions per range -- catches quadrant-boundary bias


def bias() -> None:
    """Signed + absolute bearing error of atan2_cd against math.atan2, swept over the flight envelope."""
    print('(a) BEARING ERROR vs math.atan2 -- east/north as centimetre fixnums')
    print('    %8s %10s %10s %10s %10s' % ('range_m', 'mean_deg', 'max_deg', 'p95_deg', 'cross_m'))
    worst_overall = 0.0
    for metres in _RANGES:
        errors = []
        for degrees in _BEARINGS:
            radians = math.radians(degrees)
            east, north = metres * math.sin(radians), metres * math.cos(radians)
            truth = math.degrees(math.atan2(east, north)) % 360.0
            got = fixed.atan2_cd(int(east * fixed.SCALE), int(north * fixed.SCALE)) / 100.0 % 360.0
            error = (got - truth + 180.0) % 360.0 - 180.0  # signed, wrapped to +/-180
            errors.append(error)
        errors.sort()
        mean = sum(errors) / len(errors)
        worst = max(abs(errors[0]), abs(errors[-1]))
        p95 = sorted(abs(error) for error in errors)[int(len(errors) * 0.95)]
        # the number that decides the port: what the worst angular error costs in metres off track
        print('    %8.0f %10.4f %10.4f %10.4f %10.2f'
              % (metres, mean, worst, p95, metres * math.radians(worst)))
        worst_overall = max(worst_overall, worst)
    print('    worst bearing error over the envelope: %.4f deg' % worst_overall)


def near_zero() -> None:
    """
    The CLOSE-IN regime -- where the CORDIC's right-shifts have the fewest bits left to discard.

    atan2_cd's precision comes from input MAGNITUDE, so the interesting failure is not a big offset
    (which only gets more precise) but a small one: on short final the east/north offset collapses
    toward zero and the centimetre fixnum runs out of significant bits. This sweeps down to 10 cm.
    """
    print()
    print('(d) NEAR-ZERO -- bearing error as the offset collapses on short final')
    print('    %9s %9s %10s %10s %10s' % ('range_m', 'range_cu', 'mean_deg', 'max_deg', 'cross_m'))
    for metres in (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0):
        errors = []
        for degrees in _BEARINGS:
            radians = math.radians(degrees)
            east, north = metres * math.sin(radians), metres * math.cos(radians)
            truth = math.degrees(math.atan2(east, north)) % 360.0
            got = fixed.atan2_cd(int(east * fixed.SCALE), int(north * fixed.SCALE)) / 100.0 % 360.0
            errors.append((got - truth + 180.0) % 360.0 - 180.0)
        mean = sum(errors) / len(errors)
        worst = max(abs(error) for error in errors)
        print('    %9.2f %9d %10.4f %10.4f %10.3f'
              % (metres, int(metres * fixed.SCALE), mean, worst, metres * math.radians(worst)))
    print('    degenerate inputs:  atan2_cd(0,0)=%d  (1,0)=%d  (0,1)=%d  (-1,0)=%d  (0,-1)=%d'
          % (fixed.atan2_cd(0, 0), fixed.atan2_cd(1, 0), fixed.atan2_cd(0, 1),
             fixed.atan2_cd(-1, 0), fixed.atan2_cd(0, -1)))


def envelope() -> None:
    """The stated flight envelope against the 32-bit ceiling: <100 s at 15 m/s -> 1500 m."""
    print()
    print('(e) STATED ENVELOPE -- 100 s at 15 m/s = 1500 m extent')
    for metres in (1500.0,):
        for name, unit in (('cm', fixed.SCALE), ('m', 1)):
            leg = int(metres * unit)
            total = leg * leg * 2  # the worst case: a 45 deg offset of the full extent
            print('    %6s scale: east=north=%d -> east^2+north^2 = %d  (2**31 = %d)  %s'
                  % (name, leg, total, 2 ** 31, 'OVERFLOWS' if total >= 2 ** 31 else 'fits'))


def overflow() -> None:
    """Where the viper isqrt stops being able to hold east^2 + north^2."""
    print()
    print('(b) RANGE -- isqrt(east^2 + north^2), @viper == 32-bit signed (limit 2**31-1 = 2147483647)')
    print('    %10s %6s %16s %14s %12s' % ('offset_m', 'scale', 'east^2+north^2', 'isqrt', 'truth'))
    for metres in (100.0, 500.0, 1000.0, 2000.0):
        for name, unit in (('cm', fixed.SCALE), ('m', 1)):
            east = north = int(metres * unit / 1.4142135)  # a 45 deg offset of this range
            total = east * east + north * north
            try:
                got = fixed.isqrt_opt(total)
            except Exception as error:
                got = repr(error)
            truth = int(math.sqrt(total))
            flag = '' if got == truth else '   <-- WRONG'
            print('    %10.0f %6s %16d %14s %12d%s' % (metres, name, total, got, truth, flag))


def lat_factor() -> None:
    """How much cos(lat_mid) -- offset()'s only trig call -- actually moves during a flight."""
    print()
    print('(c) offset() calls cos(lat_mid) per invocation. How constant is it over a flight?')
    base = 60.17  # Helsinki-ish; the factor is worst (steepest) at high latitude
    reference = math.cos(math.radians(base))
    for span_km in (0.5, 2.0, 10.0):
        delta = span_km * 1000.0 / navigation.M_PER_DEG
        drift = abs(math.cos(math.radians(base + delta)) - reference) / reference
        print('    lat %.2f -> +%.1f km: cos factor moves %.3e relative (%.3f m per km east)'
              % (base, span_km, drift, drift * 1000.0))


print('=== findings 31.6: integer trig on the navigation path -- MEASUREMENT ONLY ===')
print('fixed.SCALE = %d, M_PER_DEG = %.1f' % (fixed.SCALE, navigation.M_PER_DEG))
print()
bias()
near_zero()
envelope()
overflow()
lat_factor()
print()
print('=== done ===')


def normalise() -> None:
    """
    Would PRE-SCALING the vector fix the low-magnitude floor? atan2 is ratio-free, so shifting both
    components left is lossless -- it just hands the CORDIC more bits to shift away. This measures
    what that would buy, and whether the error is really about magnitude or only about the axes.
    """
    print()
    print('(f) PRE-NORMALISATION -- atan2 is ratio-free, so y,x << k is lossless')

    def shifted(y, x, target=1 << 14):
        """Left-shift both components until the larger is near `target` (what a fixed atan2_cd would do)."""
        while (-target < y < target) and (-target < x < target) and (y or x):
            y, x = y << 1, x << 1
        return fixed.atan2_cd(y, x)

    print('    %10s %12s %12s %12s' % ('magnitude', 'worst_now', 'worst_shifted', 'improvement'))
    for magnitude in (1, 3, 10, 30, 100, 300, 1000):
        now, better = 0.0, 0.0
        for degrees in _BEARINGS:
            radians = math.radians(degrees)
            y, x = int(magnitude * math.sin(radians)), int(magnitude * math.cos(radians))
            if not (y or x):
                continue
            truth = math.degrees(math.atan2(y, x))
            for value, keep in ((fixed.atan2_cd(y, x), 'now'), (shifted(y, x), 'shift')):
                error = abs((value / 100.0 - truth + 180.0) % 360.0 - 180.0)
                if keep == 'now':
                    now = max(now, error)
                else:
                    better = max(better, error)
        print('    %10d %12.4f %12.4f %11.1fx'
              % (magnitude, now, better, (now / better) if better else 0.0))
    print('    axes after shifting:  (1,0)=%d  (0,1)=%d  (-1,0)=%d  (0,-1)=%d   [want 9000/0/-9000/18000]'
          % (shifted(1, 0), shifted(0, 1), shifted(-1, 0), shifted(0, -1)))
    print('    is the error only on the axes? magnitude 1, (1,1) -> %d  [want 4500]' % fixed.atan2_cd(1, 1))


normalise()
