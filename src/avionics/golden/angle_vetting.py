"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

What the attitude builders (build_attitude.py, build_magnetic_yaw.py) share: the angle arithmetic of plan B's
src/glider/tasks/attitude.py, both as plan B computes it (integer CORDIC, integer square root, centidegree wraps) and
as the Rust port computes it (the real-number angle), so a builder can refuse an input on which the two part by more
than the fixture's tolerance can hold.

Plan B's integer angle of a vector is good to ~0.05 degree (the CORDIC) and, for the pitch, to ~0.3 degree (the
square root floors the denominator at centi-g scale). The fixtures compare the port within 0.1 degree, so they use
only vectors whose plan B angle is within TARGET_ERROR of the true angle of the same integers: the tolerance then
tests the port's filter, not plan B's integer error.

Imported by the builders, which generate.py runs after shims.install(): the imports below need src/glider on sys.path.
"""

import math

import fixed

TOLERANCE: float = 0.1  # degrees: the plan's trajectory tolerance for the attitude fixtures
BUDGET: float = 0.09  # the most plan B may stray from the real-number filter, leaving the port's f32 its margin
TARGET_ERROR: float = 0.025  # degrees: how far plan B's integer angle of a used vector may be from the true angle
GATE_MARGIN: float = 0.15  # degrees from the level gate, and from the +/-180 wrap of a published roll or pitch
WRAP_MARGIN: float = 0.5  # degrees from +/-180 for a pull's wrapped error
SCALE: int = 100  # fixed.SCALE: centi-units per unit


class NearThreshold(Exception):
    """A decision sits too close to its threshold to compare across f32; `channel` names the reading behind it."""

    def __init__(self, channel: str, detail: str) -> None:
        """
        A refusal.

        Args:
            channel - the reading behind the decision (a seeded run drops it for the tick).
            detail - what is too close to what.
        """
        super().__init__('%s: %s' % (channel, detail))
        self.channel: str = channel


def kept(units: int) -> bool:
    """Whether plan B's from_float() reads units / 100 back as exactly `units` (int(0.29 * 100) is 28)."""
    return fixed.from_float(units / SCALE) == units


def wrap360(degrees: float) -> float:
    """A heading in [0, 360), as plan B's `% 36000` keeps one."""
    return degrees % 360.0


def wrap180(degrees: float) -> float:
    """A signed angle in [-180, 180), as plan B's `((x + 18000) % 36000) - 18000` keeps one."""
    return (degrees + 180.0) % 360.0 - 180.0


def around(first: float, second: float) -> float:
    """How far apart two angles are in degrees, the short way around the circle."""
    return abs(wrap180(first - second))


def plan_b_accel_angles(accel_cg: tuple) -> tuple:
    """Plan B's (roll, pitch) gravity target of a centi-g vector, in degrees: its integer CORDIC and square root."""
    x, y, z = accel_cg
    planar = y * y + z * z
    return fixed.atan2_cd(y, z) / SCALE, fixed.atan2_cd(-x, fixed.isqrt(planar)) / SCALE


def true_accel_angles(accel_cg: tuple) -> tuple:
    """The true (roll, pitch) of the same vector, in degrees: what the port computes."""
    x, y, z = accel_cg
    return math.degrees(math.atan2(y, z)), math.degrees(math.atan2(-x, math.hypot(y, z)))


def plan_b_magnetic(mag: tuple) -> float:
    """Plan B's magnetic heading of a field, in degrees: its integer CORDIC, in [0, 360)."""
    return (fixed.atan2_cd(-mag[1], mag[0]) % 36000) / SCALE


def true_magnetic(mag: tuple) -> float:
    """The true magnetic heading of the same field, in degrees, in [0, 360)."""
    return wrap360(math.degrees(math.atan2(-mag[1], mag[0])))


def accel_usable(accel_cg: tuple) -> bool:
    """Whether from_float() keeps every axis of a centi-g vector and plan B's angles of it are within TARGET_ERROR."""
    if not all(kept(axis) for axis in accel_cg):
        return False
    plan_b = plan_b_accel_angles(accel_cg)
    true = true_accel_angles(accel_cg)
    return all(around(mine, theirs) <= TARGET_ERROR for mine, theirs in zip(plan_b, true))


def magnetic_usable(mag: tuple) -> bool:
    """Whether plan B's heading of a field is within TARGET_ERROR of the true one."""
    return around(plan_b_magnetic(mag), true_magnetic(mag)) <= TARGET_ERROR


def nearest(target: tuple, usable) -> tuple | None:
    """
    The integer vector nearest `target` that `usable` accepts: the rounded vector, then its +/-1 neighbours.

    Args:
        target - the real-valued vector (centi-g, or counts).
        usable - the acceptance test (accel_usable, magnetic_usable).

    Returns:
        The accepted integer vector, or None when none of the seven is.
    """
    rounded = tuple(round(axis) for axis in target)
    candidates = [rounded] + [tuple(axis + delta * (index == which) for index, axis in enumerate(rounded))
                              for which in range(3) for delta in (1, -1)]
    for candidate in candidates:
        if usable(candidate):
            return candidate
    return None


def kept_near(units: int) -> int:
    """
    `units`, or the nearest integer from_float() keeps.

    Raises:
        ValueError - none within two (from_float drops at most one unit, so it cannot happen).
    """
    for delta in (0, 1, -1, 2, -2):
        if kept(units + delta):
            return units + delta
    raise ValueError('no kept value near %d' % units)
