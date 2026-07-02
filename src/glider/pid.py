# pid.py — a minimal fixed-point PID controller for the flight stabilization loop (Phase 3), sibling of
# mixer.py. One instance per control axis. Integral anti-windup clamp + output clamp; reset() on
# (re)entering a control phase.
#
# INTEGER fixed-point (fixed.fixnum in/out, integer-millisecond dt) so a step allocates NOTHING on the
# heap. The flight loop runs with GC DISABLED (sequencer disables it on BOOSTING), so every heap byte
# accumulates toward OOM; the old float PID boxed a fresh float on every * + / -- measured 176 B/step,
# ×3 axes ×100 Hz ≈ 56 KB/s of leak. This version measures 0 B/step (even at a ±180° heading swing, the
# worst case for the derivative), leaving only the isolated call-site conversion fixed.from_float(setpoint
# - actual) at the sensor boundary. Net saving ≈ 47 KB/s. See findings 17 (memory refactor).
#
# Fixed-point contract (error/output in fixed.fixnum -- degrees × fixed.SCALE; measured alloc-free):
#   error   fixnum  -- the caller scales at the boundary: fixed.from_float(setpoint - actual)
#   dt      ms (int)
#   gains   floats (kp/ki/kd) -- scaled by _KU=100 (0.01 gain resolution) at construction
#   limits  degrees -- scaled by fixed.SCALE (to the error/output unit) at construction
#   output  fixnum  -- the caller reduces: output // fixed.SCALE -> integer degrees for the mixer
# The two 1000s inside step() are TIME (ms<->s), not the angle scale -- they are independent of SCALE.
# Every intermediate product stays < 2**30 (the RV32 small-int ceiling; past it boxes a 16-byte mpz): at
# SCALE=100 the worst term kp_k·e = 500·18000 = 9e6 and the derivative swing 36000·1000 = 3.6e7, both far
# under it (SCALE=100 keeps ~3x headroom even on a scaled angle², which SCALE=1000 would overflow).

from fixed import SCALE, clamp, fixnum  # fixed-point convention: error/output in SCALE-units, integer clamp

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_KU = const(100)  # GAIN scale: kp 1.50 -> 150 (0.01 gain resolution) -- distinct from fixed.SCALE (angle)
_UNBOUNDED_DEG = const(1000000)  # default 'no limit' -- ×SCALE stays a small int, so the clamp is a no-op


class Pid:
    """error (fixnum, degrees × SCALE) -> control output (fixnum). step(error, dt_ms[, rate]):
    kp*e + ki*integral(e) + kd*derivative, each clamped -- all integer, no heap allocation. The
    derivative is the measured `rate` (gyro, SCALE-deg/s) when given -- derivative-on-measurement, clean
    + no setpoint kick -- else d(error)/dt (differentiated on the error)."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: int = _UNBOUNDED_DEG, output_limit: int = _UNBOUNDED_DEG):
        # gains scaled by _KU; limits (degrees) scaled by SCALE to the error/output unit the loop runs in,
        # so a step needs no unit conversion. An unbounded default (1e6 deg) stays a small int -> no-op.
        self.kp: int = int(kp * _KU)
        self.ki: int = int(ki * _KU)
        self.kd: int = int(kd * _KU)
        self.integral_limit: fixnum = int(integral_limit * SCALE)  # SCALE-degree-seconds
        self.output_limit: fixnum = int(output_limit * SCALE)  # SCALE-degrees
        self._integral: int = 0
        self._previous = None  # last error (mdeg); None until the first step -> no derivative kick on entry

    def reset(self) -> None:
        """Clear the integral + derivative history -- on entering a control phase, so a fresh glide does
        not inherit wind-up from a previous one. `_previous = None` so the FIRST step after reset takes no
        derivative term (finding 1.14.1: a 0 baseline would make de/dt = error/dt, a large spurious D kick
        on entry)."""
        self._integral = 0
        self._previous = None

    def step(self, error: fixnum, dt_ms: int, rate: fixnum = None) -> fixnum:
        # integral += error*dt in SCALE-degree-seconds (the //1000 is TIME, ms -> s); clamped for anti-windup
        integral = clamp(-self.integral_limit, self._integral + error * dt_ms // 1000, self.integral_limit)
        self._integral = integral
        if rate is not None:
            # DERIVATIVE-ON-MEASUREMENT: the gyro's angular rate (SCALE-deg/s), used directly. For a
            # constant setpoint d(error)/dt = -d(measured)/dt, so the D term is -rate -- but the gyro is
            # far cleaner than differentiating a customer-level attitude signal, and it has no derivative
            # kick when the setpoint steps. Always valid, so no first-step guard.
            derivative = -rate
        elif self._previous is None or dt_ms <= 0:  # no gyro -> derivative on error; skip the first step
            derivative = 0
        else:
            derivative = (error - self._previous) * 1000 // dt_ms  # SCALE-degrees per second (1000 is TIME)
        self._previous = error
        output = (self.kp * error + self.ki * integral + self.kd * derivative) // _KU
        return clamp(-self.output_limit, output, self.output_limit)
