# On-board test for the fixed-point helpers (fixed.py): boundary conversions (from_float / to_float),
# integer decimal formatting (to_str, no float boxed), clamp, and -- the point of the SCALE choice -- an
# accuracy + overflow SWEEP over the real control-path ranges, reported so we pick SCALE (100 vs 1000)
# from data. Run by `make test`.

import math

import fixed


def test_atan2_cd():
    """Integer CORDIC atan2 (centidegrees) tracks math.atan2 to <= ~0.2 deg over all four quadrants,
    and the accel-gravity-vector -> roll/pitch composition (with isqrt) reproduces banked/pitched
    attitudes -- the attitude-backup filter's core, zero float boxed."""
    # cardinal + quadrant sanity
    assert fixed.atan2_cd(0, 0) == 0
    assert abs(fixed.atan2_cd(0, 1000) - 0) <= 17       # +x axis -> 0
    assert abs(fixed.atan2_cd(1000, 0) - 9000) <= 17    # +y axis -> +90
    assert abs(fixed.atan2_cd(0, -1000) - 18000) <= 17  # -x, y>=0 -> +180
    assert abs(fixed.atan2_cd(-1000, 0) + 9000) <= 17   # -y axis -> -90
    # full sweep vs math.atan2 (accel scaled x1000)
    worst = 0
    for adeg in range(-179, 181):
        a = math.radians(adeg)
        got = fixed.atan2_cd(round(math.sin(a) * 1000), round(math.cos(a) * 1000))
        ref = round(adeg * 100)
        worst = max(worst, abs(((got - ref + 18000) % 36000) - 18000))
    assert worst <= 20, worst  # ~0.2 deg -- plenty for a backup attitude
    # isqrt exact vs the reference across the range the filter feeds it
    for n in (0, 1, 2, 4, 1000000, 1999999, 2000000):
        assert fixed.isqrt(n) == fixed.isqrt_upy(n) == int(n ** 0.5)
    # accel -> roll/pitch at a banked, nose-down attitude (glide): recovers to ~0.1 deg
    for roll_d, pitch_d in ((45, -6), (-30, 5), (0, 20)):
        r, p = math.radians(roll_d), math.radians(pitch_d)
        ax, ay, az = -math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)
        axi, ayi, azi = round(ax * 1000), round(ay * 1000), round(az * 1000)
        roll_cd = fixed.atan2_cd(ayi, azi)
        pitch_cd = fixed.atan2_cd(-axi, fixed.isqrt(ayi * ayi + azi * azi))
        assert abs(roll_cd - roll_d * 100) <= 20 and abs(pitch_cd - pitch_d * 100) <= 20


def test_convert():
    # round-trip within the resolution: from_float truncates toward zero, so the error is < 1/SCALE
    tol = 1.0 / fixed.SCALE
    for x in (0.0, 1.23, -1.23, 45.67, -180.0, 179.99, 8.5):
        f = fixed.from_float(x)
        assert isinstance(f, int), (x, f)  # a fixnum is a plain int (no float, no box)
        assert abs(fixed.to_float(f) - x) <= tol, (x, f, fixed.to_float(f))

    # to_str: the true decimal via integer divmod (no float), sign + zero-padded fraction
    assert fixed.to_str(fixed.from_float(1.23)) in ('1.23', '1.22')  # 1.23*100 may truncate to 122
    assert fixed.to_str(123) == '1.23'
    assert fixed.to_str(-45) == '-0.45'
    assert fixed.to_str(1200) == '12.00'
    assert fixed.to_str(0) == '0.00'
    assert fixed.to_str(-1) == '-0.01'

    # millis: fixnum (×SCALE) -> milli-int (×1000), pure integer rescale (no float)
    assert fixed.millis(fixed.from_float(1.23)) == fixed.from_float(1.23) * (1000 // fixed.SCALE)
    assert fixed.millis(fixed.from_float(1.0)) == 1000  # 1.0 unit -> 1000 milli, any SCALE that divides 1000
    assert isinstance(fixed.millis(500), int)

    # clamp: symmetric ±x and one-sided
    assert fixed.clamp(-100, 250, 100) == 100
    assert fixed.clamp(-100, -250, 100) == -100
    assert fixed.clamp(-100, 50, 100) == 50


def test_limits():
    # The heaviest control-path shapes are a scaled angle (max ±180° -> ±180·SCALE) times another scaled
    # quantity. Assert the worst case stays a SMALL INT (< 2**30) so nothing promotes to a 16-byte mpz,
    # and print the margins so the SCALE=100-vs-1000 decision is data-driven.
    ceiling = 1 << 30  # RV32 small-int limit
    angle = 180 * fixed.SCALE                 # widest scaled angle
    angle_sq = angle * angle                  # scaled × scaled (the heaviest product shape)
    angle_gain = angle * (5 * fixed.SCALE)    # scaled angle × a gain up to ~5.0 (also scaled)
    print('fixed SCALE=%d: max_angle=%d  angle^2=%d  angle*gain=%d  ceiling(2^30)=%d  headroom=%.0fx'
          % (fixed.SCALE, angle, angle_sq, angle_gain, ceiling, ceiling / angle_sq))
    assert angle_sq < ceiling, 'scaled angle^2 overflows 2^30 at SCALE=%d -> mpz' % fixed.SCALE
    assert angle_gain < ceiling

    # accuracy: worst truncation error across the angle range stays within one sub-unit
    worst = 0.0
    x = -180.0
    while x <= 180.0:
        worst = max(worst, abs(fixed.to_float(fixed.from_float(x)) - x))
        x += 0.137  # an irregular step so we sample between grid points
    print('fixed SCALE=%d: worst round-trip error=%.4f (resolution=%.4f)' % (
        fixed.SCALE, worst, 1.0 / fixed.SCALE))
    assert worst <= 1.0 / fixed.SCALE


test_convert()
test_limits()
test_atan2_cd()
print('ok: fixed -- from_float/to_float round-trip, to_str integer formatting, clamp, SCALE limit sweep, '
      'CORDIC atan2 + isqrt')
