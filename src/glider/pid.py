"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

A minimal fixed-point PID controller for the flight stabilization loop (Phase 3), sibling of mixer.py.
One instance per control axis. Anti-windup is TWO mechanisms: the integral/output clamps bound the
magnitude (and track the governor's live fin cap via set_limit()), and BACK-CALCULATION bleeds the
integral by whatever demand the saturated fin could not fly. reset() on (re)entering a control phase.

INTEGER fixed-point (fixed.fixnum in/out, integer-millisecond dt) so a step allocates NOTHING on the
heap. The flight loop runs with GC DISABLED (sequencer disables it on BOOSTING), so every heap byte
accumulates toward OOM; the old float PID boxed a fresh float on every * + / -- measured 176 B/step,
×3 axes ×100 Hz ≈ 56 KB/s of leak. This version measures 0 B/step (even at a ±180° heading swing, the
worst case for the derivative), leaving only the isolated call-site conversion
fixed.from_float(setpoint - actual) at the sensor boundary. Net saving ≈ 47 KB/s (from the
memory-refactor work).

Fixed-point contract (error/output in fixed.fixnum -- degrees × fixed.SCALE; measured alloc-free):
  error   fixnum  -- the caller scales at the boundary: fixed.from_float(setpoint - actual)
  dt      ms (int)
  gains   floats (kp/ki/kd) -- scaled by _KU=100 (0.01 gain resolution) at construction
  limits  degrees -- scaled by fixed.SCALE (to the error/output unit) at construction
  output  fixnum  -- the caller reduces: output // fixed.SCALE -> integer degrees for the mixer
The two 1000s inside step() are TIME (ms<->s), not the angle scale -- they are independent of SCALE.
Every intermediate product stays < 2**30 (the RV32 small-int ceiling; past it boxes a 16-byte mpz): at
SCALE=100 the worst term kp_k·e = 500·18000 = 9e6 and the derivative swing 36000·1000 = 3.6e7, both far
under it (SCALE=100 keeps ~3x headroom even on a scaled angle², which SCALE=1000 would overflow).
"""

from fixed import SCALE, clamp, fixnum  # fixed-point convention: error/output in SCALE-units, integer clamp

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_KU = const(100)  # GAIN scale: kp 1.50 -> 150 (0.01 gain resolution) -- distinct from fixed.SCALE (angle)
_UNBOUNDED_DEG = const(1000000)  # default 'no limit' -- ×SCALE stays a small int, so the clamp is a no-op
_ANTI_WINDUP_SHIFT = const(2)  # back-calculation gain = 1/4 of the unflyable demand per step (see step())


class Pid:
    """
    A minimal fixed-point PID controller for one control axis: error (fixnum) -> control output (fixnum).

    step(error, dt_ms[, rate]) is kp*e + ki*integral(e) + kd*derivative, each clamped -- all integer,
    no heap allocation. Error and output are fixnums (degrees × SCALE). The derivative is the measured
    `rate` (gyro, SCALE-deg/s) when given -- derivative-on-measurement, clean + no setpoint kick --
    else d(error)/dt (differentiated on the error).
    """

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: int = _UNBOUNDED_DEG, output_limit: int = _UNBOUNDED_DEG,
                 anti_windup_shift: int = _ANTI_WINDUP_SHIFT):
        # gains scaled by _KU; limits (degrees) scaled by SCALE to the error/output unit the loop runs in,
        # so a step needs no unit conversion. An unbounded default (1e6 deg) stays a small int -> no-op.
        self.kp: int = int(kp * _KU)
        self.ki: int = int(ki * _KU)
        self.kd: int = int(kd * _KU)
        self.anti_windup_shift: int = anti_windup_shift  # back-calculation strength; see step()
        self.integral_limit: fixnum = int(integral_limit * SCALE)  # SCALE-degree-seconds
        self.output_limit: fixnum = int(output_limit * SCALE)  # SCALE-degrees
        self._integral: int = 0
        self._previous = None  # last error (mdeg); None until the first step -> no derivative kick on entry

    def reset(self) -> None:
        """
        Clear the integral + derivative history.

        On entering a control phase, so a fresh glide does not inherit wind-up from a previous one.
        `_previous = None` so the FIRST step after reset takes no derivative term (a 0 baseline would
        make de/dt = error/dt, a large spurious D kick on entry).

        Args:
            (none)

        Returns:
            None -- resets self._integral and self._previous in place.
        """
        self._integral = 0
        self._previous = None

    def set_limit(self, limit_deg: int) -> None:
        """
        Retune the output clamp + anti-windup integral clamp to a live authority limit (whole degrees).

        The governor rewrites the fin cap (mixer.limit) by dynamic pressure every update; driving BOTH
        PID clamps from that same cap stops the integral winding up to a stale 45deg behind a tight
        (e.g. 5deg) cap and then dumping a saturating deflection when the cap reopens (overshoot). Scales
        to the SCALE-unit the loop runs in, matching the constructor. Cheap (two int stores, no alloc).

        Args:
            limit_deg - the current authority limit in whole degrees (the mixer's live cap).

        Returns:
            None -- updates self.output_limit and self.integral_limit in place.
        """
        self.output_limit = limit_deg * SCALE
        self.integral_limit = limit_deg * SCALE

    def step(self, error: fixnum, dt_ms: int, rate: fixnum = None) -> fixnum:
        # integral += error*dt in SCALE-degree-seconds (the //1000 is TIME, ms -> s); clamped for anti-windup
        integral = clamp(-self.integral_limit, self._integral + error * dt_ms // 1000, self.integral_limit)
        self._integral = integral
        if rate is not None:
            """
            DERIVATIVE-ON-MEASUREMENT: the gyro's angular rate (SCALE-deg/s), used directly. For a
            constant setpoint d(error)/dt = -d(measured)/dt, so the D term is -rate -- but the gyro is
            far cleaner than differentiating a customer-level attitude signal, and it has no derivative
            kick when the setpoint steps. Always valid, so no first-step guard.
            """
            derivative = -rate
        elif self._previous is None or dt_ms <= 0:  # no gyro -> derivative on error; skip the first step
            derivative = 0
        else:
            derivative = (error - self._previous) * 1000 // dt_ms  # SCALE-degrees per second (1000 is TIME)
        self._previous = error
        output = (self.kp * error + self.ki * integral + self.kd * derivative) // _KU
        limited = clamp(-self.output_limit, output, self.output_limit)
        """
        BACK-CALCULATION anti-windup (findings §23.5). The clamps above bound the integral's MAGNITUDE,
        but they do not unwind it: while the fin is saturated the integral simply sits at its limit and
        keeps demanding what the airframe cannot fly, so the moment authority returns it dumps.
        Back-calculation actively bleeds it by the part of the demand that did NOT reach the fin,
        converted back into integral units through ki (`* _KU // ki` inverts the `ki * integral // _KU`
        above) and damped by a shift so it eases out rather than snapping.

        This matters most on a WARM START: after a mid-air reset the fins sit whereever the servos
        landed while the GPIO was floating -- potentially near 90deg to the airflow, where a fin is an
        AIRBRAKE, not a control surface. The loop then restarts against a large, sustained disturbance
        with a tight `unconfident` cap, i.e. deep saturation for many steps: exactly the case where a
        clamp-only integral winds to its bound and a back-calculated one does not.

        ki == 0 -> no integral to unwind (the conversion would divide by zero); the clamp still bounds.
        All-integer, every product < 2**30, so the step stays 0-allocation (bench_pid_alloc guards it).
        """
        if limited != output and self.ki:
            unwound = integral - (((output - limited) * _KU // self.ki) >> self.anti_windup_shift)
            """
            STOP AT ZERO -- a deliberate deviation from textbook back-calculation. Unmodified, the law
            drives the integral until it CANCELS the excess, so a demand the P term alone oversaturates
            (30deg of error against a 5deg cap) settles the integral at the opposite sign: measured
            -1067 where the error was +3000, which commands a REVERSED fin the instant the cap reopens.
            Trading windup for counter-windup is not a fix. Bleeding only toward zero removes the
            wind-up §23.5 is about and can never invert the command.
            """
            integral = max(0, unwound) if integral > 0 else min(0, unwound)
            self._integral = integral
        return limited
