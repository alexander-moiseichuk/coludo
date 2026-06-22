# pid.py — a minimal fixed-step PID controller for the flight stabilization loop (Phase 3), sibling of
# mixer.py. One instance per control axis. Integral anti-windup clamp + output clamp; reset() on
# (re)entering a control phase. Floats internally (gains/integral are inherently fractional); the
# caller rounds the output to integer degrees for the mixer/servos.


def _clamp(value: float, limit: float) -> float:
    return -limit if value < -limit else (limit if value > limit else value)


class Pid:
    """error -> control output. step(error, dt): kp*e + ki*integral(e) + kd*de/dt, each clamped."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: float = None, output_limit: float = None):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self._integral: float = 0.0
        self._previous: float = 0.0

    def reset(self) -> None:
        """Clear the integral + derivative history -- on entering a control phase, so a fresh glide
        does not inherit wind-up from a previous one."""
        self._integral = 0.0
        self._previous = 0.0

    def step(self, error: float, dt: float) -> float:
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = _clamp(self._integral, self.integral_limit)
        derivative = (error - self._previous) / dt if dt > 0 else 0.0
        self._previous = error
        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return _clamp(output, self.output_limit) if self.output_limit is not None else output
