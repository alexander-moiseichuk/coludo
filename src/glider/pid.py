# pid.py — a minimal fixed-step PID controller for the flight stabilization loop (Phase 3), sibling of
# mixer.py. One instance per control axis. Integral anti-windup clamp + output clamp; reset() on
# (re)entering a control phase. Floats internally (gains/integral are inherently fractional); the
# caller rounds the output to integer degrees for the mixer/servos.

import math

from commons import between


class Pid:
    """error -> control output. step(error, dt): kp*e + ki*integral(e) + kd*de/dt, each clamped."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: float = math.inf, output_limit: float = math.inf):
        # limits default to inf (unbounded): _clamp(x, inf) == x, so step() needs no `is not None`
        # guard -- an unbounded PID just clamps to +/- inf, which is a no-op.
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.integral_limit: float = integral_limit
        self.output_limit: float = output_limit
        self._integral: float = 0.0
        self._previous = None  # last error; None until the first step -> no derivative kick on entry

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """Symmetric clamp of `value` to +/- `limit` (limit inf -> a no-op). Thin wrapper over the shared
        between() primitive."""
        return between(-limit, value, limit)

    def reset(self) -> None:
        """Clear the integral + derivative history -- on entering a control phase, so a fresh glide
        does not inherit wind-up from a previous one. `_previous = None` so the FIRST step after reset
        takes no derivative term (finding 1.14.1: a 0 baseline would make de/dt = error/dt, a large
        spurious D kick on entry)."""
        self._integral = 0.0
        self._previous = None

    def step(self, error: float, dt: float) -> float:
        self._integral = self._clamp(self._integral + error * dt, self.integral_limit)
        if self._previous is None or dt <= 0:  # first step after reset (or dt 0) -> no derivative
            derivative = 0.0
        else:
            derivative = (error - self._previous) / dt
        self._previous = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return self._clamp(output, self.output_limit)
