"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture pid.json: plan B's PID controller (src/glider/pid.py) as the Rust port (crates/control, module pid)
must answer it, step by step.

Plan B runs the law in FIXED POINT -- centidegree errors and outputs, integer-millisecond time, gains times 100 --
a MicroPython workaround (a boxed float per float operation, with the GC off in flight). The Rust port computes
the real-number law those integers stand for, in f32 degrees and seconds. So every input here is written in plan
B's own integer units, which both sides read exactly: `error_cd` (centidegrees: the Rust error is error_cd / 100
degrees), `dt_ms` (the Rust elapsed time is dt_ms / 1000 seconds), `rate_cd_s` (centidegrees per second, or null
for no gyro). Plan B's output, a centidegree integer, is written in degrees (fixed.to_float).

The fixture is stateful: each scenario is {name, init, steps}. `init` holds the constructor's arguments (the gains
kp/ki/kd, integral_limit and output_limit in degrees, anti_windup_shift); each step is {in, out}, and `in.action`
is one of
  step       pid.step(error_cd, dt_ms, rate_cd_s) -- out: the output in degrees
  set_limit  pid.set_limit(limit), whole degrees -- out: null
  reset      pid.reset() -- out: null

TOLERANCE abs 0.02 degrees a step, the plan's. Against the real-number law, plan B's fixed point floors every
result to 0.01: the output (under 0.01 degrees), the integral (under 0.01 degree-seconds, times ki) and the error
derivative (under 0.01 degrees per second, times kd), and back-calculation floors twice more and snaps its fine
accumulator to the coarse one -- a state error the 3/4 contraction of a saturated step keeps bounded. With ki and
kd at most 0.4 the builder measures the worst of all that over the fixture (`measured`) and refuses a fixture past
_BUDGET, which leaves the f32 rounding of the port (~1e-5 degrees) its margin. The measurement uses the real-number
law, written out below (_Law), only to vet: every expected value is plan B's.

VETTED, not fitted: the law has one discontinuity, its stop-at-zero guard -- `max(0, unwound) if integral > 0
else min(0, unwound)` -- which sends an integral at or under 0 to the opposite branch from one just above it. Plan
B floors a positive integral under 0.01 degree-seconds to 0, so near zero its branch differs from the real law's
(plan B then winds the integral AGAINST the error; a fixed-point artifact the port does not reproduce), and an f32
sum that should cancel to 0 can land a hair either side. A sweep scenario that saturates with the law's integral
within (_ZERO_BAND_LOW, _ZERO_BAND_HIGH) of zero is drawn again; an integral exactly 0 because every error since the
last zero was exactly 0 is exact on both sides and kept (the on-setpoint kick scenario relies on it). An explicit
scenario that lands in the band is an error. Gains are drawn as k/100 that int(gain x 100) keeps (plan B truncates
0.29 to 0.28), and no scenario reaches plan B's integral accumulator ceiling (2**28 centidegree-milliseconds).

Explicit scenarios are test_pid.py's, bench_cases.py's pid_steps, and two that show where the stop-at-zero guard
does not hold (the on-setpoint D kick, and an integral of the opposite sign winding on until the reopened cap
reverses the fin). Then seeded sweeps: random gains and limits, an error random walk with jumps, jittered dt,
with and without a gyro rate, live set_limit and the occasional reset.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import pid

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SWEEPS: int = 10  # seeded scenarios
_SWEEP_STEPS: int = 60  # control steps per seeded scenario
_TOLERANCE: float = 0.02  # degrees a step: the plan's PID tolerance
_BUDGET: float = 0.019  # the most plan B may differ from the real-number law, leaving the port's f32 its margin
_ZERO_BAND_LOW: float = -1e-04  # degree-seconds: a saturated integral this close below zero is a coin toss...
_ZERO_BAND_HIGH: float = 0.1  # ...and this close above it (plan B's floor of 0.01, plus its bleed state error)
_ACCUMULATOR_CEILING: int = 1 << 28  # plan B's _ACCUM_LIMIT_MAX, centidegree-milliseconds
_GAIN_SCALE: int = 100  # plan B's _KU: gains are kept as int(gain x 100)
_SCALE: int = 100  # fixed.SCALE: centidegrees per degree
_UNBOUNDED: int = 1000000  # plan B's _UNBOUNDED_DEG, the constructor's default limits


def _between(low: float, value: float, high: float) -> float:
    """commons.between's rule: low if below, high if above, else value."""
    return low if value < low else (high if value > high else value)


class _Law:
    """
    The real-number law plan B's fixnums stand for: what the Rust port computes, here in f64, used only to vet.

    Its state is the integral (degree-seconds) and the previous error; `exact_zero` tracks whether an integral of 0
    is exactly 0 in any arithmetic (no nonzero error since the last zero).
    """

    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float, output_limit: float, shift: int):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit, self.output_limit = float(integral_limit), float(output_limit)
        self.factor: float = 2.0 ** -shift
        self.integral: float = 0.0
        self.previous = None
        self.exact_zero: bool = True

    def reset(self) -> None:
        """pid.reset()."""
        self.integral, self.previous, self.exact_zero = 0.0, None, True

    def set_limit(self, limit: int) -> None:
        """pid.set_limit()."""
        self.integral_limit = self.output_limit = float(limit)

    def step(self, error: float, elapsed: float, rate) -> tuple:
        """
        One step of the law.

        Args:
            error - degrees.
            elapsed - seconds.
            rate - degrees per second, or None.

        Returns:
            (output degrees, whether the step saturated with the integral in the coin-toss band around zero).
        """
        integral = _between(-self.integral_limit, self.integral + error * elapsed, self.integral_limit)
        self.exact_zero = self.exact_zero and error == 0
        if rate is not None:
            derivative = -rate
        elif self.previous is None or elapsed <= 0:
            derivative = 0.0
        else:
            derivative = (error - self.previous) / elapsed
        self.previous = error
        output = self.kp * error + self.ki * integral + self.kd * derivative
        limited = _between(-self.output_limit, output, self.output_limit)
        risky = False
        if limited != output and self.ki:
            risky = _ZERO_BAND_LOW < integral < _ZERO_BAND_HIGH and not (integral == 0 and self.exact_zero)
            unwound = integral - (output - limited) / self.ki * self.factor
            integral = max(0.0, unwound) if integral > 0 else min(0.0, unwound)
            if integral == 0:
                self.exact_zero = True  # the guard's 0 is a literal 0 on both sides
        self.integral = integral
        return limited, risky


class _Scenario:
    """One scenario being built: plan B's controller, the vetting law, and the steps written so far."""

    def __init__(self, name: str, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0,
                 integral_limit: int = _UNBOUNDED, output_limit: int = _UNBOUNDED,
                 anti_windup_shift: int = pid._ANTI_WINDUP_SHIFT):
        for gain in (kp, ki, kd):
            if int(gain * _GAIN_SCALE) != round(gain * _GAIN_SCALE):
                raise ValueError('%s: plan B truncates the gain %r (int(gain x 100))' % (name, gain))
        self.name: str = name
        self.init: dict = {'kp': kp, 'ki': ki, 'kd': kd, 'integral_limit': integral_limit,
                           'output_limit': output_limit, 'anti_windup_shift': anti_windup_shift}
        self.plan_b = pid.Pid(kp=kp, ki=ki, kd=kd, integral_limit=integral_limit, output_limit=output_limit,
                              anti_windup_shift=anti_windup_shift)
        self.law = _Law(kp, ki, kd, integral_limit, output_limit, anti_windup_shift)
        self.steps: list = []
        self.risky: bool = False
        self.worst: float = 0.0

    def step(self, error_cd: int, dt_ms: int, rate_cd_s: int | None = None) -> int:
        """
        One control step on both: plan B's answer is recorded, the law's only measured against it.

        Returns:
            plan B's output, centidegrees.
        """
        output = self.plan_b.step(error_cd, dt_ms, rate_cd_s)
        if abs(self.plan_b._integral_ms) >= _ACCUMULATOR_CEILING:
            raise ValueError('%s: plan B hit its integral accumulator ceiling' % self.name)
        expected, risky = self.law.step(error_cd / _SCALE, dt_ms / 1000.0,
                                        None if rate_cd_s is None else rate_cd_s / _SCALE)
        self.risky = self.risky or risky
        self.worst = max(self.worst, abs(output / _SCALE - expected))
        self.steps.append({'in': {'action': 'step', 'error_cd': error_cd, 'dt_ms': dt_ms, 'rate_cd_s': rate_cd_s},
                           'out': output / _SCALE})
        return output

    def set_limit(self, limit: int) -> None:
        """A live cap change on both."""
        self.plan_b.set_limit(limit)
        self.law.set_limit(limit)
        self.steps.append({'in': {'action': 'set_limit', 'limit': limit}, 'out': None})

    def reset(self) -> None:
        """A reset on both."""
        self.plan_b.reset()
        self.law.reset()
        self.steps.append({'in': {'action': 'reset'}, 'out': None})

    def document(self) -> dict:
        """The scenario as the fixture writes it."""
        return {'name': self.name, 'init': self.init, 'steps': self.steps}


def _explicit() -> list:
    """
    test_pid.py's cases, bench_cases.py's pid_steps, and the two guard scenarios, as scenarios.

    Returns:
        The built _Scenario objects.
    """
    scenarios = []

    def scenario(name: str, **init) -> _Scenario:
        built = _Scenario(name, **init)
        scenarios.append(built)
        return built

    scenario('proportional', kp=2.0).step(300, 100)
    integral = scenario('integral_accumulates', ki=1.0)
    integral.step(200, 500)
    integral.step(200, 500)
    derivative = scenario('derivative_on_the_error', kd=1.0)
    derivative.step(0, 100)
    derivative.step(100, 100)
    rated = scenario('derivative_on_the_measured_rate', kd=1.0)
    rated.step(0, 100, 1000)
    rated.step(0, 100, -400)
    spike = scenario('no_derivative_kick_after_construction_or_reset', kd=1.0)
    spike.step(500, 100)
    spike.reset()
    spike.step(900, 100)
    swing = scenario('a_full_heading_swing', kp=1.5)
    swing.step(18000, 10)
    swing.step(-18000, 10)
    pinned = scenario('the_integral_clamp', ki=1.0, integral_limit=5)
    for _ in range(20):
        pinned.step(1000, 1000)
    clamped = scenario('the_output_clamp', kp=100.0, output_limit=45)
    clamped.step(1000, 100)
    clamped.step(-1000, 100)
    reset = scenario('reset_clears_the_integral', ki=1.0)
    reset.step(500, 1000)
    reset.reset()
    reset.step(0, 1000)
    live = scenario('set_limit_retunes_the_output_clamp', kp=100.0, output_limit=45)
    live.step(1000, 100)
    live.set_limit(5)
    live.step(1000, 100)
    live.set_limit(45)
    live.step(1000, 100)
    held = scenario('set_limit_retunes_the_integral_clamp', ki=1.0, integral_limit=45)
    held.set_limit(5)
    for _ in range(20):
        held.step(1000, 1000)
    held.set_limit(45)
    held.step(0, 1000)
    for shift in (2, 30):
        saturated = scenario('back_calculation_shift_%d' % shift, kp=1.0, ki=2.0, integral_limit=45, output_limit=5,
                             anti_windup_shift=shift)
        for _ in range(50):
            saturated.step(3000, 20)
        saturated.set_limit(45)
        saturated.step(0, 20)
    reversal = scenario('a_positive_integral_stops_at_zero', kp=1.0, ki=2.0, integral_limit=45, output_limit=5)
    for _ in range(200):
        reversal.step(3000, 20)
    reversal.set_limit(45)  # reopened: an integral wound below zero would now command a reversed fin
    reversal.step(0, 20)
    free = scenario('an_unsaturated_integral_is_left_alone', ki=2.0, integral_limit=45, output_limit=45)
    for _ in range(51):
        free.step(100, 20)
    balanced = scenario('a_symmetric_error', ki=1.0, integral_limit=45, output_limit=45)
    for index in range(200):
        balanced.step(50 if index % 2 == 0 else -50, 10)
    creeping = scenario('a_small_standing_error', ki=1.0, integral_limit=45, output_limit=45)
    for _ in range(101):
        creeping.step(50, 10)
    scenario('no_ki_no_back_calculation', kp=100.0, output_limit=5).step(3000, 20)
    bench = scenario('bench_cases_pid_steps', kp=1.5, ki=0.2, kd=0.1, integral_limit=20, output_limit=30)
    for index in range(5):
        bench.step(500 + index * 900, 10, -300)
    kicked = scenario('an_on_setpoint_kick_winds_an_empty_integral_against_it', kp=1.0, ki=1.0, kd=1.0,
                      integral_limit=45, output_limit=5)
    for _ in range(5):
        kicked.step(0, 10, -1000)
    kicked.step(0, 10, 0)
    opposite = scenario('an_opposite_integral_winds_on_and_reverses_the_fin', kp=1.0, ki=2.0, integral_limit=45,
                        output_limit=5)
    for _ in range(100):
        opposite.step(100, 20)
    for _ in range(60):
        opposite.step(-3000, 20)
    opposite.set_limit(45)
    opposite.step(-100, 20)
    return scenarios


def _gain(generator: random.Random, top: int) -> float:
    """A gain k/100 for k in [0, top] that plan B keeps as written (int(gain x 100) == k)."""
    while True:
        units = generator.randint(0, top)
        if int(units / _GAIN_SCALE * _GAIN_SCALE) == units:
            return units / _GAIN_SCALE


def _sweep(generator: random.Random, name: str) -> _Scenario:
    """
    One seeded scenario: random gains and limits, an error random walk with occasional jumps, jittered dt, a gyro
    rate on most, live caps and the occasional reset.

    Args:
        generator - the seeded random source.
        name - the scenario name.

    Returns:
        The built scenario (possibly risky: the caller draws again).
    """
    built = _Scenario(name, kp=_gain(generator, 300), ki=_gain(generator, 40), kd=_gain(generator, 40),
                      integral_limit=generator.choice((5, 10, 20, 45)), output_limit=generator.randint(5, 45),
                      anti_windup_shift=generator.choice((2, 2, 2, 1, 3)))
    with_rate = generator.random() < 0.6
    error = generator.randint(-3000, 3000)
    for _ in range(_SWEEP_STEPS):
        draw = generator.random()
        if draw < 0.04:
            built.set_limit(generator.randint(5, 45))
        elif draw < 0.05:
            built.reset()
        jump = generator.randint(-3000, 3000) if generator.random() < 0.03 else 0
        error = max(-4500, min(4500, error + generator.randint(-300, 300) + jump))
        rate = generator.randint(-20000, 20000) if with_rate else None
        built.step(error, generator.randint(8, 25), rate)
    return built


def build() -> dict:
    """
    The pid fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, measured, scenarios}.

    Raises:
        ValueError - an explicit scenario lands in the zero band, or plan B strays past _BUDGET from the law.
    """
    scenarios = _explicit()
    risky = [scenario.name for scenario in scenarios if scenario.risky]
    if risky:
        raise ValueError('pid: explicit scenarios saturate next to a zero integral: %s' % ', '.join(risky))
    generator = random.Random(_SEED)
    redrawn = 0
    for index in range(_SWEEPS):
        while True:
            scenario = _sweep(generator, 'sweep_%02d' % index)
            if not scenario.risky:
                break
            redrawn += 1
        scenarios.append(scenario)
    worst = max(scenario.worst for scenario in scenarios)
    if worst > _BUDGET:
        raise ValueError('pid: plan B strays %.5f degrees from the real-number law (budget %.3f)' % (worst, _BUDGET))
    return {
        'meta': {'source': 'src/glider/pid.py, src/glider/fixed.py'},
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'note': ('degrees a step; plan B floors to 0.01 everywhere, the real-number law does not; inputs in plan B '
                 'units: error_cd / 100 degrees, dt_ms / 1000 s, rate_cd_s / 100 degrees per second'),
        'measured': {'worst_plan_b_against_the_law_degrees': round(worst, 5), 'sweeps_redrawn': redrawn},
        'scenarios': [scenario.document() for scenario in scenarios],
    }
