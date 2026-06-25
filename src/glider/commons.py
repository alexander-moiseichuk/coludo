# commons.py — small, dependency-free primitives shared across the control-math modules (mixer / pid /
# navigation / sequencer / flight / sg90). The bundle module for the g14/g15 plan: the hot, isolated
# INTEGER operations are gathered here with a portable bytecode version AND a `@micropython.viper`
# variant (integer-only native code — the on-device RV32 emitter compiles it with no FPU needed;
# measured ~2.1x vs bytecode on the P4). `between()` stays plain bytecode (it is float-/inf-valued, which
# viper cannot type). `select()` swaps viper<->bytecode at bring-up per board.config. Everything here is
# pure and host-testable so both versions can be validated + benchmarked against each other.
#
# NOTE on viper portability: `@micropython.viper` is a COMPILER directive recognised by the literal
# `micropython.viper` decorator name — it cannot be aliased. The shim below makes the module importable
# on CPython (tooling/tests), where the decorator degrades to an identity (the function runs as plain
# Python); on the board it emits native integer code.

try:
    import micropython  # real module on the board: micropython.viper / .native / .const
except ImportError:  # CPython (off-board tooling / tests) — decorators + const become no-ops

    class micropython:  # noqa: N801 — deliberately shadows the absent stdlib name with a shim
        @staticmethod
        def viper(function):
            return function

        @staticmethod
        def native(function):
            return function

        @staticmethod
        def const(value):
            return value


def between(low, value, high):
    """Clamp `value` to the inclusive range [low, high]: `low` if below, `high` if above, else `value`.
    With low=-x, high=+x it is a symmetric +/-x clamp; either bound may be math.inf for an open side
    (between(-inf, v, inf) == v). Float-/inf-valued -> stays bytecode (not viper). Assumes low <= high."""
    return low if value < low else (high if value > high else value)


# --- clamp_int: integer clamp to [low, high]. Hot via sg90 fin clamping (round(angle), min/max deg). ---


def _clamp_int_upy(low, value, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


@micropython.viper
def _clamp_int_viper(low: int, value: int, high: int) -> int:
    if value < low:
        return low
    if value > high:
        return high
    return value


# --- wrap180: wrap an integer-degree value to (-180, 180]. Hot via the yaw heading error each step. ---


def _wrap180_upy(degrees):
    return degrees if -180 <= degrees <= 180 else (degrees + 180) % 360 - 180


@micropython.viper
def _wrap180_viper(degrees: int) -> int:
    if -180 <= degrees <= 180:
        return degrees
    return (degrees + 180) % 360 - 180


# --- magnitude_sq: squared 3-vector magnitude |(x,y,z)|^2. FLOAT, so not viper -- kept bytecode here as
# the centralised leaf and the prime candidate for a real-FPU C natmod (mpy-ld) later (on-device @native
# is float-boxing-limited to ~1.2x; a C natmod with hardware FPU is the path to a worthwhile speedup). ---


def magnitude_sq(x, y, z):
    """|(x, y, z)|^2 (no sqrt — callers compare against squared thresholds; g7). Pure float."""
    return x * x + y * y + z * z


# Default to the viper variants (this firmware's RV32 emitter supports them; the CPython shim runs them
# as plain bytecode). Consumers that reference these as module attributes (commons.clamp_int) see select().
clamp_int = _clamp_int_viper
wrap180 = _wrap180_viper


def select(viper: bool) -> None:
    """Bind clamp_int / wrap180 to the @micropython.viper variants (viper=True, the default) or the
    portable bytecode versions (False — a board without the viper emitter, or A/B benchmarking). Call
    once at bring-up from a board.config flag, before the control tasks start."""
    global clamp_int, wrap180
    clamp_int = _clamp_int_viper if viper else _clamp_int_upy
    wrap180 = _wrap180_viper if viper else _wrap180_upy
