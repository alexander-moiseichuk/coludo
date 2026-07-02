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

from commons import clamp_int

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


fixnum = int  # a scaled fixed-point integer (×SCALE); NOT a plain count and NOT a float
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


def millis(value: fixnum) -> int:
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
