# pid.py — a minimal fixed-point PID controller for the flight stabilization loop (Phase 3), sibling of
# mixer.py. One instance per control axis. Integral anti-windup clamp + output clamp; reset() on
# (re)entering a control phase.
#
# INTEGER fixed-point (millidegrees in/out, integer-millisecond dt) so a step allocates NOTHING on the
# heap. The flight loop runs with GC DISABLED (sequencer disables it on BOOSTING), so every heap byte
# accumulates toward OOM; the old float PID boxed a fresh float on every * + / -- measured 176 B/step,
# ×3 axes ×100 Hz ≈ 56 KB/s of leak. This version measures 0 B/step (even at a ±180° heading swing, the
# worst case for the derivative), leaving only the isolated call-site conversion int((setpoint-actual)*
# 1000) at the sensor boundary (~16-32 B/axis). Net saving ≈ 47 KB/s. See findings 17 (memory refactor).
#
# Fixed-point contract (measured alloc-free on the ESP32-P4, worst case included):
#   error   in millidegrees (int)   -- the caller scales: int((setpoint - actual) * 1000)
#   dt      in milliseconds  (int)
#   gains   as floats (kp/ki/kd)    -- scaled by _KU=100 (0.01 resolution) at construction
#   limits  in degrees              -- scaled to millidegree(-seconds) at construction
#   output  in millidegrees (int)   -- the caller reduces: output // 1000 -> integer degrees for the mixer
# Every intermediate product stays < 2**30 (the RV32 small-int ceiling: sys.maxsize is 2**31-1 but a
# product past ~2**30 boxes a 16-byte mpz) for any realistic gain -- worst term kp_k·e = 150·180000 =
# 2.7e7, derivative swing 360000·1000 = 3.6e8, both far under it. Gains stay small-int up to kp ≈ 59.

from commons import between

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_KU = const(100)  # gain scale: kp 1.50 -> 150 (0.01 resolution, ample for a flight PID)
_MDEG = const(1000)  # millidegrees per degree: scales the degree-unit limits to the error/output unit
_UNBOUNDED_DEG = const(1000000)  # default 'no limit' -- ×_MDEG stays a small int, so the clamp is a no-op


class Pid:
    """error (millidegrees) -> control output (millidegrees). step(error, dt_ms):
    kp*e + ki*integral(e) + kd*de/dt, each clamped -- all integer, no heap allocation."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: int = _UNBOUNDED_DEG, output_limit: int = _UNBOUNDED_DEG):
        # gains scaled by _KU; limits (degrees) scaled to the millidegree units the loop runs in, so a
        # step needs no unit conversion. An unbounded default (1e6 deg) stays a small int -> clamp no-op.
        self.kp: int = int(kp * _KU)
        self.ki: int = int(ki * _KU)
        self.kd: int = int(kd * _KU)
        self.integral_limit: int = int(integral_limit * _MDEG)  # millidegree-seconds
        self.output_limit: int = int(output_limit * _MDEG)  # millidegrees
        self._integral: int = 0
        self._previous = None  # last error (mdeg); None until the first step -> no derivative kick on entry

    def reset(self) -> None:
        """Clear the integral + derivative history -- on entering a control phase, so a fresh glide does
        not inherit wind-up from a previous one. `_previous = None` so the FIRST step after reset takes no
        derivative term (finding 1.14.1: a 0 baseline would make de/dt = error/dt, a large spurious D kick
        on entry)."""
        self._integral = 0
        self._previous = None

    def step(self, error: int, dt_ms: int) -> int:
        # integral += error*dt in millidegree-seconds (//1000 converts dt_ms -> s); clamped for anti-windup
        integral = between(-self.integral_limit, self._integral + error * dt_ms // 1000, self.integral_limit)
        self._integral = integral
        previous = self._previous
        if previous is None or dt_ms <= 0:  # first step after reset (or a sub-ms slice) -> no derivative
            derivative = 0
        else:
            derivative = (error - previous) * 1000 // dt_ms  # millidegrees per second
        self._previous = error
        output = (self.kp * error + self.ki * integral + self.kd * derivative) // _KU
        return between(-self.output_limit, output, self.output_limit)
