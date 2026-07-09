# fixed.py — fixed-point helpers for the flight hot paths. MicroPython boxes a heap float on EVERY float
# operation, and GC is disabled through the airborne phase, so every boxed float leaks toward OOM. The
# control path therefore works in scaled integers ("fixnum") and crosses to/from float only at the
# isolated sensor boundary.
#
# `fixnum` is `int`, aliased -- a SEMANTIC marker that a value is a scaled fixed-point quantity (×SCALE),
# not a plain count and not a float. Annotating with it documents the convention and makes an accidental
# float / whole-number mix obvious at the call site; there is no runtime cost.
#
# SCALE is the fractional resolution (100 -> 0.01 unit: centidegrees / centimetres / centi-(m/s)). It is
# kept small on purpose: the RV32 small-int ceiling is 2**30, so a product of two scaled quantities is
# (val·SCALE)² -- at SCALE 100 a ±180° angle squared is 3.2e8 (safe); at 1000 it is 3.2e10 (a 16-byte
# mpz). Start at 100; raise to 1000 only if the accuracy sweep in test_fixed.py shows 0.01 is too coarse.
#
# Convert at the BOUNDARY only -- from_float once on the way in, to_float / to_str once on the way out --
# and stay integer in between. There is deliberately NO fixnum mul/div rescale here: a fixed-point rescale
# invites float->fixnum->float chains mid-computation, which is exactly what this exists to remove.

import array

from commons import clamp_int

try:
    import micropython  # real module on the board (micropython.viper)
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const, micropython  # the shim: @viper degrades to plain Python
    ptr32 = int  # dummy so the shim'd @viper function's `ptr32` annotation evaluates (unused off-board)


fixnum = int  # a scaled fixed-point integer (×SCALE=100, centi-units); NOT a plain count and NOT a float
SCALE: int = const(100)  # sub-units per unit -> fractional resolution 0.01 (centidegrees / centimetres / ...)

_FRAC: int = len(str(SCALE - 1))  # fractional digits for to_str: 2 at SCALE 100, 3 at 1000
_STR_FMT: str = '%d.%0' + str(_FRAC) + 'd'  # e.g. '%d.%02d' -- built once at import, not per call


def from_float(value) -> fixnum:
    """Whole unit (float or int: degrees / metres / m·s⁻¹) -> fixnum. The one boxed-float spot, kept at
    the sensor boundary. Truncates toward zero -- the residual is < 1/SCALE (below actuator resolution)."""
    return int(value * SCALE)


def to_float(scaled: fixnum) -> float:
    """fixnum -> whole-unit float. Boxes a float, so use ONLY where a float is genuinely required (trig,
    the airspeed integrator) and keep it at the boundary -- never inside a hot loop."""
    return scaled / SCALE


def to_millis(value: fixnum) -> int:
    """A fixnum (×SCALE) -> integer MILLI-units (×1000), independent of SCALE -- e.g. at SCALE=100 a
    centidegree fixnum becomes millidegrees. For telemetry/logs that fix a milli representation regardless
    of the control SCALE. Pure integer rescale (SCALE divides 1000), so no float is boxed."""
    return value * (1000 // SCALE)


def to_str(scaled: fixnum) -> str:
    """fixnum -> its decimal string ('12.34' at SCALE 100) via INTEGER divmod -- NO float is boxed. For
    telemetry / display: a scaled value prints as its true decimal without a float round-trip."""
    sign = '-' if scaled < 0 else ''
    whole, frac = divmod(-scaled if scaled < 0 else scaled, SCALE)
    return sign + (_STR_FMT % (whole, frac))


def clamp(low: fixnum, value: fixnum, high: fixnum) -> fixnum:
    """Integer clamp to [low, high] (a symmetric ±x clamp with low=-x, high=+x). Routes to commons.
    clamp_int -- the `@micropython.viper` integer clamp (~2.1-2.8x the float `between`). Safe here because
    fixnum is always a finite int (no math.inf), which is exactly what the fixed-point transition buys:
    the whole control-path clamp is now viper-native, not the inf-tolerant @native float path."""
    return clamp_int(low, value, high)


# ------------------------------------------------------------------------- integer trig (@viper)
#
# The attitude-backup complementary filter (tasks/attitude.py) turns the accel gravity vector into
# roll/pitch EVERY control step with GC off, so it must not box a float. atan2_cd is an integer CORDIC
# (vectoring mode: ~0.17 deg over 12 iterations) and isqrt a division-free integer square root (the
# pitch denominator sqrt(ay^2+az^2)). Both @micropython.viper, both zero-alloc. Angles are centidegree
# fixnum like the rest of the control path.

_ATAN_CD = array.array('i', (4500, 2657, 1404, 713, 358, 179, 90, 45, 22, 11, 6, 3))  # atan(2^-i), cd


def _cordic_vec_upy(y: int, x: int, atan) -> int:
    """CORDIC vectoring reference: rotate (x, y) onto +x, accumulating the angle (cd). x must be >= 0."""
    z = 0
    for i in range(12):
        if y < 0:
            x, y, z = x - (y >> i), y + (x >> i), z - atan[i]
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + atan[i]
    return z


@micropython.viper
def _cordic_vec_opt(y: int, x: int, atan: ptr32) -> int:
    z = 0
    i = 0
    while i < 12:
        if y < 0:
            nx = x - (y >> i)
            ny = y + (x >> i)
            z = z - atan[i]
        else:
            nx = x + (y >> i)
            ny = y - (x >> i)
            z = z + atan[i]
        x = nx
        y = ny
        i += 1
    return z


_cordic_vec = _cordic_vec_opt  # viper is safe on this firmware -> bind the optimised variant


def atan2_cd(y: int, x: int) -> fixnum:
    """atan2(y, x) as a CENTIDEGREE fixnum, four-quadrant, via integer CORDIC -- NO float boxed. y and x
    are a RATIO-FREE integer direction vector: only their ratio sets the angle, and their MAGNITUDE only
    trades precision (the CORDIC's right-shifts discard low bits, so bigger inputs keep more). Fed the
    control's centi-fixnum scale (accel g via from_float, ~x100) the error is ~0.5 deg typical / 1.8 deg
    worst over the glide envelope -- fine for the attitude backup; x1000 would tighten to ~0.16 deg if a
    caller ever needs it. Range (-18000, 18000]. CORDIC needs x >= 0, so x < 0 reflects into the right
    half-plane and the 180 deg is added back per quadrant."""
    if x >= 0:
        return _cordic_vec(y, x, _ATAN_CD) if (x or y) else 0  # (0, 0) is undefined -> 0
    base = _cordic_vec(y, -x, _ATAN_CD)
    return (18000 - base) if y >= 0 else (-18000 - base)


def _blend_upy(state: int, delta: int, target: int, shift: int, correct: int) -> int:
    """Reference: advance a filter state by `delta` (gyro integration), then, when `correct`, pull it
    `1/2^shift` of the way toward `target` (the accel gravity angle) -- one complementary-filter step."""
    state += delta
    return state + ((target - state) >> shift) if correct else state


@micropython.viper
def _blend_opt(state: int, delta: int, target: int, shift: int, correct: int) -> int:
    state = state + delta
    if correct:
        state = state + ((target - state) >> shift)
    return state


blend = _blend_opt  # viper is safe on this firmware -> bind the optimised variant


def blend_cd(state: fixnum, delta: fixnum, target: fixnum, shift: int, correct: bool) -> fixnum:
    """One complementary-filter step in centidegrees (viper): `state + delta` (gyro integration), then
    optionally a `1/2^shift` pull toward `target` (the accel angle). Pure integer -> zero float boxed;
    the attitude backup runs it per axis each control step (tasks/attitude.py)."""
    return blend(state, delta, target, shift, 1 if correct else 0)


def isqrt_upy(n: int) -> int:
    """Integer floor(sqrt(n)) reference, division-free (bit-by-bit); n < 2**31."""
    if n <= 0:
        return 0
    res = 0
    bit = 1 << 30
    while bit > n:
        bit >>= 2
    while bit:
        if n >= res + bit:
            n -= res + bit
            res = (res >> 1) + bit
        else:
            res >>= 1
        bit >>= 2
    return res


@micropython.viper
def isqrt_opt(n: int) -> int:
    if n <= 0:
        return 0
    res = 0
    bit = int(1 << 30)  # int() cast: viper constant-folds `1 << 30` to a Python object otherwise
    while bit > n:
        bit = bit >> 2
    while bit != 0:
        if n >= res + bit:
            n = n - res - bit
            res = (res >> 1) + bit
        else:
            res = res >> 1
        bit = bit >> 2
    return res


isqrt = isqrt_opt  # viper is safe on this firmware -> bind the optimised variant
