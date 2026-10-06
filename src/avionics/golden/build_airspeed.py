"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture airspeed.json: plan B's hybrid airspeed estimator (src/glider/airspeed.py) as the Rust port
(crates/control, module airspeed) must answer it, operation by operation.

The fixture is stateful: each scenario is {name, init, steps}. `init` holds the constructor's arguments (ceiling,
gnss_gain); each step is {in, out}, and `in.action` is one of
  predict  estimator.predict(acceleration_along, elapsed)
  correct  estimator.correct(gnss_speed, has_fix)
  measure  estimator.measure(airspeed, gain)
  seed     estimator.seed(airspeed)
and `out` is {value, confident}: the estimate in m/s and whether it is trusted.

TOLERANCE abs 1e-3 m/s on the value; `confident` is a flag and compares exactly. Plan B ran this float path on
CPython in f64; the port runs it in f32, as plan B's board does. The difference is rounding: an f32 ulp is 3.8e-6
m/s at the 60 m/s ceiling, and an integration of a few hundred steps stays within ~1e-5 (the Rust test prints the
worst it measures). A millimetre per second is two orders past that, and still a thousand times finer than the cap
steps the governor reads off the estimate (whole m/s).

VETTED: the one decision a float can flip is the confidence latch, `speed >= 5.0` after predict() or measure()
while not yet confident. A sweep scenario that lands within _LATCH_MARGIN of 5 m/s there is drawn again; an
explicit scenario that does is an error, unless it lands EXACTLY on 5.0 through exact arithmetic, which holds on
both sides -- test_airspeed's 40 steps of predict(50.0, 0.01) do: 50 x 0.01 is exactly 0.5 in f64 and in f32, so
the tenth step reaches 5.0 exactly in both; and a first pitot reading of exactly 5.0 seeds exactly 5.0 (both listed
in _EXACT_LATCHES).

The inclusive edges are pinned on their exact values, each with its other side: the latch at exactly 5 m/s
(`>=`), and a GNSS fix and a warm-start seed at exactly the ceiling, accepted (`<=`), half a m/s past it refused.

Explicit scenarios are test_airspeed.py's, step for step; the sweeps are seeded flights: a pad, a boost, a coast
and a glide fed by every source in turn, with dropouts, jumps and garbage seeds.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import airspeed

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SWEEPS: int = 2  # seeded scenarios
_TOLERANCE: float = 1e-03  # m/s on the value (see the module notes)
_LATCH_MARGIN: float = 1e-03  # m/s: how far from the 5 m/s confidence latch a vetted latch decision keeps
_EXACT_LATCHES: frozenset = frozenset({('gnss_is_gated_then_blended', 9),
                                       ('a_pitot_reading_of_exactly_five_latches', 0)})  # (scenario, step): by design


class _Scenario:
    """One scenario being built: plan B's estimator and the steps written so far."""

    def __init__(self, name: str, ceiling: float = 60.0, gnss_gain: float = 0.2):
        self.name: str = name
        self.init: dict = {'ceiling': ceiling, 'gnss_gain': gnss_gain}
        self.estimator = airspeed.AirspeedEstimator(ceiling_ms=ceiling, gnss_gain=gnss_gain)
        self.steps: list = []
        self.risky: bool = False

    def _record(self, request: dict, latch: bool) -> None:
        """
        Append the step plan B just took, and vet its latch decision.

        Args:
            request - the step's `in`.
            latch - whether this step decided the confidence latch (predict or measure while unconfident).

        Returns:
            None.
        """
        value = self.estimator.value()
        if latch and abs(value - airspeed._CONFIDENT_MS) < _LATCH_MARGIN:
            exact = value == airspeed._CONFIDENT_MS and (self.name, len(self.steps)) in _EXACT_LATCHES
            self.risky = self.risky or not exact
        self.steps.append({'in': request, 'out': {'value': value, 'confident': self.estimator.confident()}})

    def predict(self, acceleration_along: float, elapsed: float) -> None:
        """predict() on plan B."""
        latch = not self.estimator.confident()
        self.estimator.predict(acceleration_along, elapsed)
        self._record({'action': 'predict', 'acceleration_along': acceleration_along, 'elapsed': elapsed}, latch)

    def correct(self, gnss_speed: float, has_fix: bool) -> None:
        """correct() on plan B (it latches by a fix, not by a threshold: nothing to vet)."""
        self.estimator.correct(gnss_speed, has_fix)
        self._record({'action': 'correct', 'gnss_speed': gnss_speed, 'has_fix': has_fix}, False)

    def measure(self, value: float, gain: float) -> None:
        """measure() on plan B."""
        latch = not self.estimator.confident()
        self.estimator.measure(value, gain)
        self._record({'action': 'measure', 'airspeed': value, 'gain': gain}, latch)

    def seed(self, value: float) -> None:
        """seed() on plan B (it latches by its range check on the input: nothing to vet)."""
        self.estimator.seed(value)
        self._record({'action': 'seed', 'airspeed': value}, False)

    def document(self) -> dict:
        """The scenario as the fixture writes it."""
        return {'name': self.name, 'init': self.init, 'steps': self.steps}


def _explicit() -> list:
    """
    test_airspeed.py's cases as scenarios.

    Returns:
        The built _Scenario objects.
    """
    scenarios = []

    def scenario(name: str, **init) -> _Scenario:
        built = _Scenario(name, **init)
        scenarios.append(built)
        return built

    integration = scenario('the_accelerometer_integrates_and_floors_at_zero')
    for _ in range(50):
        integration.predict(49.0, 0.01)
    integration.predict(-3000.0, 0.01)
    ceiling = scenario('the_integral_clamps_at_the_ceiling', ceiling=60.0)
    for _ in range(100):
        ceiling.predict(100.0, 0.1)
    gated = scenario('gnss_is_gated_then_blended', ceiling=60.0, gnss_gain=0.2)
    for _ in range(40):
        gated.predict(50.0, 0.01)
    gated.correct(14.0, False)
    gated.correct(120.0, True)
    gated.correct(10.0, True)
    for _ in range(60):
        gated.correct(10.0, True)
    confidence = scenario('confidence_latches_and_the_first_fix_seeds', gnss_gain=0.2)
    confidence.predict(1.0, 0.1)
    confidence.correct(24.0, True)
    confidence.correct(20.0, True)
    charged = scenario('the_accelerometer_alone_earns_trust')
    for _ in range(20):
        charged.predict(49.0, 0.01)
    warm = scenario('a_seed_restores_and_the_pitot_overrides_it')
    warm.seed(16.5)
    warm.measure(25.0, 1.0)
    slow = scenario('a_slow_seed_is_trusted_and_integrated_on')
    slow.seed(3.0)
    slow.predict(9.81, 1.0)
    for garbage in (-1.0, 1e6):
        scenario('a_garbage_seed_is_refused_%r' % garbage, ceiling=60.0).seed(garbage)
    pitot = scenario('the_pitot_seeds_then_blends_and_a_pad_zero_earns_no_trust')
    pitot.measure(0.0, 0.5)
    pitot.measure(15.0, 0.5)
    pitot.measure(17.0, 0.5)
    below = scenario('an_unconfident_pitot_reading_below_five_seeds_without_trust')
    below.measure(4.0, 0.5)
    below.measure(3.5, 0.5)
    below.predict(20.0, 0.1)
    latch = scenario('a_pitot_reading_of_exactly_five_latches')
    latch.measure(5.0, 0.5)  # seeds 5.0 exactly: confident (>=)
    fix = scenario('a_fix_at_exactly_the_ceiling_is_accepted', ceiling=60.0)
    fix.correct(60.5, True)  # past the ceiling: a glitch, refused
    fix.correct(60.0, True)  # exactly the ceiling: accepted, and it seeds (<=)
    restored = scenario('a_seed_at_exactly_the_ceiling_is_accepted', ceiling=60.0)
    restored.seed(60.5)  # past the ceiling: garbage, refused
    restored.seed(60.0)  # exactly the ceiling: restored (<=)
    return scenarios


def _sweep(generator: random.Random, name: str) -> _Scenario:
    """
    One seeded flight: a pad, a boost, a coast and a glide, each source in turn, with dropouts, jumps and garbage.

    Args:
        generator - the seeded random source.
        name - the scenario name.

    Returns:
        The built scenario (possibly risky: the caller draws again).
    """
    built = _Scenario(name, ceiling=generator.choice((60.0, 60.0, 45.0)), gnss_gain=generator.choice((0.2, 0.1, 0.35)))
    for _ in range(generator.randint(3, 10)):  # the pad: a jittering accelerometer and a still-air pitot
        built.predict(generator.gauss(0.0, 0.4), generator.uniform(0.008, 0.012))
        if generator.random() < 0.3:
            built.measure(generator.uniform(0.0, 2.0), 0.5)
    if generator.random() < 0.3:
        built.seed(generator.choice((-3.0, 1e3, generator.uniform(0.0, 40.0))))
    for _ in range(generator.randint(25, 40)):  # the boost: several g along the path
        built.predict(generator.uniform(20.0, 70.0), generator.uniform(0.008, 0.012))
    for _ in range(generator.randint(15, 25)):  # the coast: drag and gravity take it back
        built.predict(generator.uniform(-25.0, -2.0), generator.uniform(0.008, 0.012))
        if generator.random() < 0.15:
            built.correct(generator.uniform(-5.0, 90.0), generator.random() < 0.7)
    for _ in range(generator.randint(40, 60)):  # the glide: every source, throttled updates
        draw = generator.random()
        elapsed = generator.uniform(0.02, 0.2)
        built.predict(generator.gauss(0.0, 1.5), elapsed)
        if draw < 0.4:
            built.measure(generator.uniform(8.0, 26.0), generator.choice((0.5, 0.3)))
        elif draw < 0.8:
            built.correct(generator.uniform(4.0, 30.0) if generator.random() < 0.9 else 75.0, generator.random() < 0.8)
    return built


def build() -> dict:
    """
    The airspeed fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, note, scenarios}.

    Raises:
        ValueError - an explicit scenario decides the confidence latch within _LATCH_MARGIN of 5 m/s.
    """
    scenarios = _explicit()
    risky = [scenario.name for scenario in scenarios if scenario.risky]
    if risky:
        raise ValueError('airspeed: explicit scenarios latch next to 5 m/s: %s' % ', '.join(risky))
    generator = random.Random(_SEED)
    for index in range(_SWEEPS):
        while True:
            scenario = _sweep(generator, 'sweep_%02d' % index)
            if not scenario.risky:
                break
        scenarios.append(scenario)
    return {
        'meta': {'source': 'src/glider/airspeed.py'},
        'compare': {'kind': 'abs', 'tol': _TOLERANCE},
        'note': 'm/s on the value, f32 against f64 rounding; confident is a flag and exact; no latch within 1e-3 of 5',
        'scenarios': [scenario.document() for scenario in scenarios],
    }
