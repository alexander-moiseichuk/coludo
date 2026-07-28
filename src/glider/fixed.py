"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Fixed-point helpers for the flight hot paths. MicroPython boxes a heap float on EVERY float operation,
and GC is disabled through the airborne phase, so every boxed float leaks toward OOM. The control path
therefore works in scaled integers ('fixnum') and crosses to/from float only at the isolated sensor
boundary.

`fixnum` is `int`, aliased -- a SEMANTIC marker that a value is a scaled fixed-point quantity (×SCALE),
not a plain count and not a float. Annotating with it documents the convention and makes an accidental
float / whole-number mix obvious at the call site; there is no runtime cost.

SCALE is the fractional resolution (100 -> 0.01 unit: centidegrees / centimetres / centi-(m/s)). It is
kept small on purpose: the RV32 small-int ceiling is 2**30, so a product of two scaled quantities is
(val·SCALE)² -- at SCALE 100 a ±180° angle squared is 3.2e8 (safe); at 1000 it is 3.2e10 (a 16-byte
mpz). Start at 100; raise to 1000 only if the accuracy sweep in test_fixed.py shows 0.01 is too coarse.

Convert at the BOUNDARY only -- from_float once on the way in, to_float / to_str once on the way out --
and stay integer in between. There is deliberately NO fixnum mul/div rescale here: a fixed-point rescale
invites float->fixnum->float chains mid-computation, which is exactly what this exists to remove.
"""

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
    """
    Whole unit (float or int: degrees / metres / m·s⁻¹) -> fixnum.

    The one boxed-float spot, kept at the sensor boundary. Truncates toward zero -- the residual is
    < 1/SCALE (below actuator resolution).

    Args:
        value - the whole-unit quantity (degrees / metres / m·s⁻¹), float or int.

    Returns:
        The value as a fixnum (the whole unit scaled by SCALE).
    """
    return int(value * SCALE)


def to_float(scaled: fixnum) -> float:
    """
    fixnum -> whole-unit float.

    Boxes a float, so use ONLY where a float is genuinely required (trig, the airspeed integrator) and
    keep it at the boundary -- never inside a hot loop.

    Args:
        scaled - the fixnum to convert.

    Returns:
        The whole-unit float value (scaled / SCALE).
    """
    return scaled / SCALE


def to_millis(value: fixnum) -> int:
    """
    A fixnum (×SCALE) -> integer MILLI-units (×1000), independent of SCALE.

    For telemetry/logs that fix a milli representation regardless of the control SCALE -- e.g. at
    SCALE=100 a centidegree fixnum becomes millidegrees. Pure integer rescale (SCALE divides 1000), so
    no float is boxed.

    Args:
        value - the fixnum to rescale.

    Returns:
        The value in integer milli-units (×1000).
    """
    return value * (1000 // SCALE)


def to_str(scaled: fixnum) -> str:
    """
    fixnum -> its decimal string ('12.34' at SCALE 100) via INTEGER divmod -- NO float is boxed.

    For telemetry / display: a scaled value prints as its true decimal without a float round-trip.

    Args:
        scaled - the fixnum to format.

    Returns:
        The decimal string (e.g. '12.34' at SCALE 100), sign preserved.
    """
    sign = '-' if scaled < 0 else ''
    whole, frac = divmod(-scaled if scaled < 0 else scaled, SCALE)
    return sign + (_STR_FMT % (whole, frac))


def clamp(low: fixnum, value: fixnum, high: fixnum) -> fixnum:
    """
    Integer clamp to [low, high] (a symmetric ±x clamp with low=-x, high=+x).

    Routes to commons.clamp_int -- the `@micropython.viper` integer clamp (~2.1-2.8x the float
    `between`). Safe here because fixnum is always a finite int (no math.inf), which is exactly what the
    fixed-point transition buys: the whole control-path clamp is now viper-native, not the inf-tolerant
    @native float path.

    Args:
        low - the lower bound (fixnum).
        value - the value to clamp (fixnum).
        high - the upper bound (fixnum).

    Returns:
        value clamped to [low, high] (fixnum).
    """
    return clamp_int(low, value, high)


"""
Integer trig (@viper): zero-alloc CORDIC + square root for the control path.

The attitude-backup complementary filter (tasks/attitude.py) turns the accel gravity vector into
roll/pitch EVERY control step with GC off, so it must not box a float. atan2_cd is an integer CORDIC
(vectoring mode: ~0.17 deg over 12 iterations) and isqrt a division-free integer square root (the
pitch denominator sqrt(ay^2+az^2)). Both @micropython.viper, both zero-alloc. Angles are centidegree
fixnum like the rest of the control path.
"""

_ATAN_CD = array.array('i', (4500, 2657, 1404, 713, 358, 179, 90, 45, 22, 11, 6, 3))  # atan(2^-i), cd
"""
PRE-NORMALISATION. The CORDIC's accuracy is set by how many bits its right-shifts have to give away,
so a SMALL input vector is the inaccurate case, not a large one -- and atan2 is ratio-free, so
scaling the vector up costs nothing and buys those bits back. Both cores below double (x, y) until
one component reaches _NORM_MIN before iterating.

Measured on the board (test/diag_fixed_nav.py), worst error over 120 directions, before -> after:

    magnitude    3 : 36.45 deg -> 0.025    (1455x)
    magnitude  100 :  0.74 deg -> 0.048      (15x)   <- attitude.py's centi-g accel products
    magnitude 1000 :  0.11 deg -> 0.040     (2.8x)

i.e. a flat ~0.05 deg floor at every magnitude instead of an error that explodes as the vector
shrinks (9.88 deg at magnitude 1, where atan2_cd(0, 1) returned -988 instead of 0). Special-casing
the axes would not have done this: the error is about magnitude, not direction -- atan2_cd(1, 1) was
4326 rather than 4500, nowhere near an axis.

_NORM_MIN is 2**14 so the normalised vector peaks near 2**15; the CORDIC's ~1.647 gain then leaves it
around 2**16, far inside the 32-bit range the viper core works in.
"""
_NORM_MIN = const(1 << 14)


def _cordic_vec_upy(y: int, x: int, atan) -> int:
    """
    CORDIC vectoring reference: rotate (x, y) onto +x, accumulating the angle (centidegrees).

    Args:
        y - the vector's y component (integer direction, any magnitude).
        x - the vector's x component; must be >= 0.
        atan - the atan(2^-i) lookup table (centidegrees).

    Returns:
        The accumulated angle in centidegrees.
    """
    if x or y:  # scale up into the CORDIC's accurate range -- lossless, atan2 depends only on y/x
        while x < _NORM_MIN and -_NORM_MIN < y < _NORM_MIN:
            x, y = x << 1, y << 1
    z = 0
    for i in range(12):
        if y < 0:
            x, y, z = x - (y >> i), y + (x >> i), z - atan[i]
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + atan[i]
    return z


@micropython.viper
def _cordic_vec_opt(y: int, x: int, atan: ptr32) -> int:
    if x != 0 or y != 0:  # pre-normalise (see _ATAN_CD note); x >= 0 always -- atan2_cd reflects first
        while x < 16384 and y < 16384 and y > -16384:
            x = x << 1
            y = y << 1
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
    """
    atan2(y, x) as a CENTIDEGREE fixnum, four-quadrant, via integer CORDIC -- NO float boxed.

    y and x are a RATIO-FREE integer direction vector: only their ratio sets the angle, and their
    MAGNITUDE only trades precision (the CORDIC's right-shifts discard low bits, so bigger inputs keep
    more). Fed the control's centi-fixnum scale (accel g via from_float, ~x100) the error is ~0.5 deg
    typical / 1.8 deg worst over the glide envelope -- fine for the attitude backup; x1000 would tighten
    to ~0.16 deg if a caller ever needs it. CORDIC needs x >= 0, so x < 0 reflects into the right
    half-plane and the 180 deg is added back per quadrant.

    The old magnitude floor is GONE -- the core pre-normalises (see the _ATAN_CD note), so the error
    is a flat ~0.05 deg at every magnitude instead of exploding as the vector shrinks. Measured on
    the board (test/diag_fixed_nav.py), worst error by magnitude: 1 -> 0.02 deg, 3 -> 0.025,
    100 -> 0.048, 1000 -> 0.040. atan2_cd(1, 1) is now exactly 4500; the axes land within 0.02 deg.

    What a caller still cannot get back is precision its INPUTS already threw away: two components
    quantised to a coarse integer no longer carry the true ratio (a 10 cm vector at centimetre scale
    holds ~10 % per component, so ~4 deg -- but only ~7 mm of position). Feed it the largest-magnitude
    form of the vector available, and prefer raw sensor counts over pre-scaled values.

    Args:
        y - the direction vector's y component (integer, any magnitude).
        x - the direction vector's x component (integer, any magnitude).

    Returns:
        The angle in centidegrees, range (-18000, 18000]; 0 for the undefined (0, 0) input.
    """
    if x >= 0:
        return _cordic_vec(y, x, _ATAN_CD) if (x or y) else 0  # (0, 0) is undefined -> 0
    base = _cordic_vec(y, -x, _ATAN_CD)
    return (18000 - base) if y >= 0 else (-18000 - base)


def _blend_upy(state: int, delta: int, target: int, shift: int, correct: int) -> int:
    """
    Complementary-filter step reference: advance `state` by `delta`, then optionally pull toward `target`.

    Advance a filter state by `delta` (gyro integration), then, when `correct`, pull it `1/2^shift` of
    the way toward `target` (the accel gravity angle) -- one complementary-filter step.

    Args:
        state - the current filter state.
        delta - the increment to advance by (gyro integration).
        target - the value to pull toward (the accel gravity angle).
        shift - the pull is 1/2^shift of the way toward target.
        correct - non-zero to apply the pull toward target; 0 to only advance.

    Returns:
        The updated filter state.
    """
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
    """
    One complementary-filter step in centidegrees (viper).

    `state + delta` (gyro integration), then optionally a `1/2^shift` pull toward `target` (the accel
    angle). Pure integer -> zero float boxed; the attitude backup runs it per axis each control step
    (tasks/attitude.py).

    Args:
        state - the current filter state (centidegree fixnum).
        delta - the gyro-integration increment (centidegree fixnum).
        target - the accel angle to pull toward (centidegree fixnum).
        shift - the pull is 1/2^shift of the way toward target.
        correct - True to apply the pull toward target; False to only advance.

    Returns:
        The updated state (centidegree fixnum).
    """
    return blend(state, delta, target, shift, 1 if correct else 0)


def isqrt_upy(n: int) -> int:
    """
    Integer floor(sqrt(n)) reference, division-free (bit-by-bit).

    Args:
        n - the value to root; must be < 2**31.

    Returns:
        floor(sqrt(n)); 0 for n <= 0.
    """
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
"""
DOMAIN: n < 2**31. Past that it does not raise -- it returns a WRONG answer, quietly, because viper
arithmetic is 32-bit machine ints. Measured on the board (test/diag_fixed_nav.py):

    n = 2_499_952_050  -> 0      (truth 49_999)
    n = 9_999_808_200  -> 37_548 (truth 99_999)

A zero, not a small error. That is why navigation distance() cannot simply move to fixnum: east and
north as CENTIMETRES square past 2**31 at only ~330 m of offset, so a 500 m range would report 0 m
and every downstream range gate would read as "at the target". At METRE scale the same expression is
exact to 32 km, which is the scale any integer distance() must use. attitude.py, the one caller
today, feeds it centi-g accel products (~2e4) and is nowhere near the bound.
"""
