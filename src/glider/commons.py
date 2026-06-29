# commons.py — small, dependency-free primitives shared across the control-math modules (mixer / pid /
# navigation / sequencer / flight / sg90). The bundle module for the plan.
#
# Naming convention:
# plain name -- a leaf with no _opt variant at all (none currently).
# NAME_upy / NAME_opt + `NAME = <winner>`
# -- a function with an optimised variant. NAME_upy is the
# portable bytecode reference; NAME_opt is the optimised build (viper for ints, native for floats,
# future asm). The module binds NAME to whichever the on-board bench FAVOURS -- usually _opt; switch
# the one alias line if a measurement changes. Both forms stay public so benchmarks/tests call them
# DIRECTLY (no runtime selector). Bound here: clamp_int, wrap180 (@viper, ~2.1-2.8x); between,
# magnitude_sq (@native, ~1.2-1.6x); bank_demand -> _upy for now (its @native measured 1.03x -- a
# thin wrapper over native between; switch to _opt when a bench shows a gain).
#
# `@micropython.viper` / `@micropython.native` are compiler directives keyed on the literal decorator
# name (not aliasable); the shim below keeps the module importable on CPython (the decorator degrades to
# identity, runs as plain Python). On the board the RV32 emitter compiles viper to integer-only native
# code (~2.1-2.5x vs bytecode, no FPU) and native to FPU float code (~1.2-1.6x — float boxing caps it).

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


try:
    from micropython import const  # real on the board; compiler catches this one for any
except ImportError:                # module that does `from commons import const` on CPython
    def const(value):
        """CPython const fallback — identity function."""
        return value

M_PER_DEG = 111320.0  # metres per degree of latitude (and per degree longitude * cos(lat)); shared
                      # by navigation + sim_model (flat-earth geo) -- one definition, not three.


def between_upy(low, value, high):
    """Clamp `value` to the inclusive range [low, high]: `low` if below, `high` if above, else `value`.
    With low=-x, high=+x it is a symmetric +/-x clamp; either bound may be math.inf for an open side
    (between(-inf, v, inf) == v). Float-/inf-valued (so @native, not viper). Assumes low <= high."""
    return low if value < low else (high if value > high else value)


@micropython.native
def between_opt(low, value, high):
    return low if value < low else (high if value > high else value)


between = between_opt  # @native -- the most-called primitive; a free ~1.6x (handles inf the same way)


def magnitude_sq_upy(x, y, z):
    """|(x, y, z)|^2 (no sqrt — callers compare against squared thresholds). Pure float -> @native."""
    return x * x + y * y + z * z


@micropython.native
def magnitude_sq_opt(x, y, z):
    return x * x + y * y + z * z


magnitude_sq = magnitude_sq_opt  # @native


def bank_demand_upy(heading_error, gain, limit):
    """Bank-to-turn: the roll angle (deg, right +) to hold for a heading error (deg) -- proportional with
    a symmetric hard clamp (gain 0 -> no bank, rudder-only). A banked turn is tight (~v^2/(g*tan(bank)))
    where a flat rudder skid is wide and weak, so the glider does not over-RANGE a small zone and the
    overshoot loop becomes an altitude-bleeding orbit."""
    return between(-limit, gain * heading_error, limit)


@micropython.native
def bank_demand_opt(heading_error, gain, limit):
    return between(-limit, gain * heading_error, limit)


bank_demand = bank_demand_upy  # @native measured 1.03x here -> keep _upy; switch to _opt when a bench shows a gain


# --- fin_deflection_limit: the dynamic-pressure fin governor (coludo.md "Fin authority"). Max fin
# deflection (deg from neutral) the airframe can safely take at a given airspeed. Aero torque scales with
# dynamic pressure q ∝ v², so a fixed angle is too weak slow / too violent fast; the cap goes ∝ 1/v² to
# hold ~constant angular authority, clamped to [5°, 45°] (always-some authority / fin mechanical throw).
# K=12500 anchors 50 m/s -> 5°. Precomputed ONCE at import (no per-step 1/v² on the 100 Hz path); the
# board.config `fin_limit_multiplier` (default 1.0) is applied by the caller, not baked into the table. ---
_FIN_VMAX = 80  # m/s -- table saturates here (well past any expected airspeed)
_FIN_LIMIT = tuple(45 if v == 0 else min(45, max(5, round(12500 / (v * v)))) for v in range(_FIN_VMAX + 1))


def fin_deflection_limit(speed_ms):
    """Max fin deflection in degrees for airspeed `speed_ms` (m/s) -- the dynamic-pressure governor table
    lookup (saturates at _FIN_VMAX). Multiply by the config fin_limit_multiplier at the caller."""
    return _FIN_LIMIT[max(0, min(int(speed_ms), _FIN_VMAX))]


def atomic_write_json(path, data):
    """Persist `data` as JSON to `path` atomically (shared by config.save + mission.save): write a
    temp file then rename it over the target, with a remove-then-rename fallback for a VFS (FAT) that
    won't rename onto an existing file. os/json are imported lazily so the hot-path importers of commons
    do not pull them in."""
    import json
    import os
    tmp = path + '.tmp'
    with open(tmp, 'w') as handle:
        handle.write(json.dumps(data))
    try:
        os.rename(tmp, path)
    except OSError:  # some VFS (FAT) won't rename onto an existing file
        try:
            os.remove(path)
        except OSError:
            pass
        os.rename(tmp, path)


def id_classify(read, expected: int) -> str:
    """Classify a chip WHO_AM_I / device-id byte against the expected value into an operator-readable
    wire-level diagnosis. The deeper 'why' a bus driver's diagnose() returns when setup() failed, so
    `verify`/`probe` report e.g. 'chip-select not asserting' instead of just 'absent / miswired?'.
    `read` is None when the bus read itself failed (no I2C ack / SPI error). Shared by every ID-based
    driver (adxl375 / lsm6dso32 / bno055 / bmp280), so it lives here, not in one driver."""
    if read is None:
        return 'no bus response -- device not acking (absent / unpowered / miswired)'
    if read == expected:
        return 'id 0x%02X ok -- device present; setup failed AFTER detect (init / config / timing)' % read
    if read == 0x00:
        return 'id reads 0x00 -- chip-select not asserting, or device unpowered'
    if read == 0xFF:
        return 'id reads 0xFF -- bus idle-high: no device driving MISO (absent / MISO miswired)'
    return 'id reads 0x%02X, expected 0x%02X -- wrong device on this bus/select (crosswired)' % (read, expected)


# --- clamp_int: integer clamp to [low, high]. Hot via sg90 fin clamping (round(angle), min/max deg). ---


def clamp_int_upy(low, value, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


@micropython.viper
def clamp_int_opt(low: int, value: int, high: int) -> int:
    if value < low:
        return low
    if value > high:
        return high
    return value


clamp_int = clamp_int_opt  # viper is safe on this firmware -> bind the optimised variant


# --- wrap180: wrap an integer-degree value to (-180, 180]. Hot via the yaw heading error each step. ---


def wrap180_upy(degrees):
    return degrees if -180 <= degrees <= 180 else (degrees + 180) % 360 - 180


@micropython.viper
def wrap180_opt(degrees: int) -> int:
    if -180 <= degrees <= 180:
        return degrees
    return (degrees + 180) % 360 - 180


wrap180 = wrap180_opt  # viper is safe on this firmware -> bind the optimised variant
