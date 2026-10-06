"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture governor.json: plan B's dynamic-pressure fin governor (src/glider/governor.py) as the Rust port
(crates/control, module governor) must answer it -- the airspeed estimate it keeps and the fin cap it writes into
the mixer, control slice by control slice.

Two sections, each a golden fixture of its own:
  * update_interval -- {config, speed} -> seconds: the distance-constant throttle's table, abs 1e-6 s (an f32
    reciprocal of a whole number of Hz is within 4e-9 of plan B's f64 one).
  * scenarios -- stateful, {name, init, steps}. `init` is {config, multiplier}: the flight component config plan B's
    GovernorConfig reads and the board's fins.limit_multiplier. Each step is {in, out}; `in.action` is
      step  governor.step(dt, full_rate_override, pitch) over the readings `accel` ([x, y, z] in g), `gnss` (m/s)
            and `pitot` (m/s), each null when that sensor is absent or stale (plan B's `source is None`);
            `pitch_cd` is plan B's centidegree pitch, the Rust port's pitch_cd / 100 degrees
      seed  governor.seed_airspeed(airspeed)
    and `out` is {airspeed, confident, cap}: the estimate in m/s, whether it is trusted, and mixer.limit -- the
    whole-degree cap. The mixer starts at the default limit, 45, as plan B's stub and the default mixer do.

TOLERANCE abs 1e-3 m/s on the airspeed, the same justification as airspeed.json (f32 against f64 rounding, two
orders under it); `confident` and `cap` are a flag and an integer and compare exactly. That holds only because no
discrete decision sits where the two roundings could part, so the builder watches every one inside plan B's
update (it swaps in an instrumented AirspeedEstimator subclass that records each call; plan B's logic is
untouched) and refuses a step that takes one closer than its margin:
  * the confidence latch -- predict()/measure() reaching 5 m/s while unconfident (_SPEED_MARGIN);
  * the GNSS wind bound -- |gnss - estimate| against max_wind while confident (_SPEED_MARGIN), unless it lands
    EXACTLY on the bound through exact operands, which holds on both sides: the explicit edge scenario's 35.0
    against a seeded 20.0 under max_wind 15.0 (listed in _EXACT_WIND_BOUNDS);
  * the whole m/s the cap and the update interval are read at, int(estimate), when the estimate lies within
    _SPEED_MARGIN of a whole number and the two neighbouring whole speeds give a different cap, or intervals that
    would decide a later throttled step differently (an exact whole number -- a seed, a pitot reading or a fix
    taken as is -- is exact on both sides);
  * the throttle, accumulated time against the interval when not at full rate (_TIME_MARGIN);
  * the pitot band, a reading against pitot_min_ms / pitot_max_ms (_READING_MARGIN, or exactly on it).
A sweep scenario that comes closer is drawn again; an explicit one is an error. Multipliers are dyadic (1, 0.75,
0.5, 1.25), so int(table x multiplier) is exact on both sides; a decimal one is not -- 45 x 1.4 truncates to 62 on
CPython and to 63 in f32, as on plan B's single-precision board -- which is plan B's own CPython/board split, kept
out of the fixture.

Every inclusive or exclusive edge of those decisions is ALSO pinned on its exact value, where both sides agree by
construction (each operand and each difference is exact in f32 and f64): the pitot band's floor (in) and ceiling
(out), the steep gate at +/-45 degrees (gated) and a centidegree inside (blends), the wind bound exactly at
max_wind (blends) and half a m/s past (rejected), and the unconfident estimator taking a fix far past max_wind as
its seed -- the safety path after a mid-air reset. A mutation of any one comparison fails the fixture.

Explicit scenarios are test_governor.py's, reached through the public API (plan B's tests poke _speed and
_confident; here seed_airspeed or an accelerometer charge sets the same state). The sweeps are seeded flights at
~100 Hz: the pad, a boost (full rate, steep pitch, a railed pitot), the deceleration, and a throttled glide with a
dive, every source dropping out, GNSS jumps and a dead pitot.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import airspeed
import commons
import governor

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SWEEPS: int = 2  # seeded flights
_TOLERANCE: float = 1e-03  # m/s on the airspeed (see the module notes)
_INTERVAL_TOLERANCE: float = 1e-06  # seconds on update_interval
_SPEED_MARGIN: float = 1e-03  # m/s: how far a speed decision keeps from its threshold (f32 drifts ~1e-5)
_TIME_MARGIN: float = 1e-06  # seconds: how far the accumulated time keeps from the interval
_READING_MARGIN: float = 1e-04  # m/s: how far a pitot reading keeps from its band's edges
_LATCH: float = airspeed._CONFIDENT_MS  # the confidence latch speed
_TOP_SPEED: int = 81  # int() of a speed matters below this (the tables saturate at 80)
_EXACT_WIND_BOUNDS: frozenset = frozenset({('the_wind_bound_includes_its_bound', 2)})  # (scenario, step): by design

"""The config the explicit scenarios start from: plan B's defaults, written out as config_default ships them."""
_DEFAULTS: dict = {'airspeed_floor_hz': 5, 'airspeed_ceiling_hz': 50, 'airspeed_dive_pitch': -45.0,
                   'gnss_steep_pitch': 45.0, 'airspeed_unconfident_ms': 30.0, 'pitot_gain': 0.5,
                   'pitot_max_ms': 28.0, 'pitot_min_ms': 3.0, 'wind': {'max_speed': 15.0}}
_LEVEL: list = [0.0, 0.0, 1.0]  # exactly 1 g: net acceleration 0, so predict() changes nothing


class _Watched(airspeed.AirspeedEstimator):
    """
    Plan B's estimator, recording each call as (name, speed before, confident before, speed after) for the
    builder's vetting.

    Every method calls plan B's own; nothing about the estimate changes.
    """

    def __init__(self):
        super().__init__()
        self.calls: list = []

    def predict(self, accel_along: float, dt: float) -> float:
        before = (self._speed, self._confident)
        result = super().predict(accel_along, dt)
        self.calls.append(('predict',) + before + (result,))
        return result

    def correct(self, gnss_speed: float, has_fix: bool) -> float:
        before = (self._speed, self._confident)
        result = super().correct(gnss_speed, has_fix)
        self.calls.append(('correct',) + before + (result,))
        return result

    def measure(self, value: float, gain: float) -> float:
        before = (self._speed, self._confident)
        result = super().measure(value, gain)
        self.calls.append(('measure',) + before + (result,))
        return result


class _Handle:
    """A databoard handle stand-in: read() -> (value, source, age), source None when absent or stale."""

    def __init__(self):
        self.value = None

    def read(self) -> tuple:
        return self.value, None if self.value is None else 'golden', 0


class _Mixer:
    """The mixer stand-in: the limit is all the governor touches. 45 is the default mixer's."""

    def __init__(self):
        self.limit: int = 45


def _neighbours(speed: float) -> tuple | None:
    """
    The two whole speeds int(speed) could read when f32 rounds differently, or None when it cannot.

    Returns:
        (whole below, whole above) when speed lies within _SPEED_MARGIN of a whole number it is not exactly, in the
        tables' range; else None.
    """
    whole = round(speed)
    if not 0.0 <= speed < _TOP_SPEED or speed == whole or abs(speed - whole) >= _SPEED_MARGIN:
        return None
    return whole - 1, whole


class _Scenario:
    """One scenario being built: plan B's governor over stand-in handles, and the steps written so far."""

    def __init__(self, name: str, config: dict, multiplier: float = 1.0):
        self.name: str = name
        self.init: dict = {'config': config, 'multiplier': multiplier}
        self.config = governor.GovernorConfig(config)
        self.mixer = _Mixer()
        self.accel, self.gnss, self.pitot = _Handle(), _Handle(), _Handle()
        self.governor = governor.Governor(self.config, self.mixer, self.accel, self.gnss, self.pitot, multiplier)
        self.governor._estimator = _Watched()  # the same fresh estimator plan B built, instrumented
        self.steps: list = []
        self.risks: list = []
        self.alternative_interval = None  # the interval f32 could have derived instead, until the next update

    def _risk(self, reason: str) -> None:
        """Record why the step just taken is too close to a decision."""
        self.risks.append('step %d: %s' % (len(self.steps), reason))

    def _vet(self, accumulated: float, interval: float, elapsed: float, full: bool, pitch_cd: int) -> None:
        """
        Vet the step just taken against every decision plan B made in it (see the module notes).

        Args:
            accumulated - the governor's accumulated time before the step.
            interval - its update interval before the step.
            elapsed - the step's dt.
            full - whether the step was at full rate (the override or a dive).
            pitch_cd - the step's pitch, centidegrees.

        Returns:
            None; appends to self.risks.
        """
        due = accumulated + elapsed
        if not full and abs(due - interval) < _TIME_MARGIN:
            self._risk('the throttle: %r against %r' % (due, interval))
        if not full and self.alternative_interval is not None and (due >= interval) != (
                due >= self.alternative_interval):
            self._risk('the throttle under the interval of the neighbouring whole speed')
        reading = self.pitot.value
        if reading is not None:
            for edge in (self.config.pitot_min_ms, self.config.pitot_max_ms):
                if reading != edge and abs(reading - edge) < _READING_MARGIN:
                    self._risk('the pitot band edge %r' % edge)
        estimator = self.governor._estimator
        steep = abs(pitch_cd) >= self.config.steep_pitch
        for name, speed, confident, after in estimator.calls:
            if name in ('predict', 'measure') and not confident and abs(after - _LATCH) < _SPEED_MARGIN:
                self._risk('the confidence latch at %r' % after)
            if name == 'correct' and confident and self.gnss.value is not None and not steep:
                off = abs(self.gnss.value - speed)
                exact = off == self.config.max_wind and (self.name, len(self.steps)) in _EXACT_WIND_BOUNDS
                if abs(off - self.config.max_wind) < _SPEED_MARGIN and not exact:
                    self._risk('the wind bound: %r against %r' % (self.gnss.value, speed))
        updated = bool(estimator.calls)
        estimator.calls.clear()
        if updated:
            self._vet_whole_speed(estimator._speed, estimator.confident(), True)

    def _vet_whole_speed(self, speed: float, cap_from_estimate: bool, interval_derived: bool) -> None:
        """
        Vet int(speed) after an update or a seed: the cap (when it comes off the estimate) must not depend on which
        side of a whole number f32 lands, and the interval derived from it is carried until the next update.

        Args:
            speed - the estimate the cap and the interval were read at.
            cap_from_estimate - whether the cap came off it (an update caps off the unconfident speed instead while
                the estimate is not trusted; a seed always caps off the estimate).
            interval_derived - whether the interval was derived from it (an update; a seed leaves the interval).

        Returns:
            None; appends to self.risks and sets self.alternative_interval.
        """
        if interval_derived:
            self.alternative_interval = None
        neighbours = _neighbours(speed)
        if neighbours is None:
            return
        caps = [max(1, int(commons.fin_deflection_limit(whole) * self.governor._multiplier))
                for whole in neighbours]
        if cap_from_estimate and caps[0] != caps[1]:
            self._risk('the cap at a whole m/s: %r' % speed)
        if interval_derived:
            below, above = (self.config.update_interval(float(whole)) for whole in neighbours)
            if below != above:
                self.alternative_interval = below if int(speed) == neighbours[1] else above

    def _record(self, request: dict) -> None:
        """Append the step with plan B's state after it."""
        self.steps.append({'in': request, 'out': {'airspeed': self.governor.airspeed(),
                                                  'confident': self.governor._estimator.confident(),
                                                  'cap': self.mixer.limit}})

    def step(self, dt: float, full_rate_override: bool, pitch_cd: int, accel: list | None = None,
             gnss: float | None = None, pitot: float | None = None) -> None:
        """One governor.step on plan B, vetted and recorded."""
        self.accel.value, self.gnss.value, self.pitot.value = accel, gnss, pitot
        accumulated, interval = self.governor._accum_s, self.governor._interval_s
        full = full_rate_override or pitch_cd <= self.config.dive_pitch
        self.governor.step(dt, full_rate_override, pitch_cd)
        self._vet(accumulated, interval, dt, full, pitch_cd)
        self._record({'action': 'step', 'dt': dt, 'full_rate_override': full_rate_override, 'pitch_cd': pitch_cd,
                      'accel': accel, 'gnss': gnss, 'pitot': pitot})

    def seed(self, value: float) -> None:
        """One governor.seed_airspeed on plan B, recorded."""
        self.governor.seed_airspeed(value)
        self._vet_whole_speed(self.governor.airspeed(), True, False)
        self._record({'action': 'seed', 'airspeed': value})

    def document(self) -> dict:
        """The scenario as the fixture writes it."""
        return {'name': self.name, 'init': self.init, 'steps': self.steps}


def _explicit() -> list:
    """
    test_governor.py's cases as scenarios, through the public API.

    Returns:
        The built _Scenario objects.
    """
    scenarios = []

    def scenario(name: str, config: dict | None = None, multiplier: float = 1.0) -> _Scenario:
        built = _Scenario(name, dict(_DEFAULTS, **(config or {})), multiplier)
        scenarios.append(built)
        return built

    cap = scenario('the_cap_follows_a_trusted_speed')
    cap.seed(0.0)
    cap.step(0.01, True, 0, _LEVEL)
    cap.seed(40.0)
    cap.step(0.01, True, 0, _LEVEL)
    halved = scenario('the_safety_dial_halves_the_schedule', multiplier=0.5)
    halved.seed(0.0)
    halved.step(0.01, True, 0, _LEVEL)
    unconfident = scenario('an_unconfident_estimate_caps_off_the_conservative_speed', {'airspeed_unconfident_ms': 30.0})
    unconfident.step(0.01, True, 0, _LEVEL)
    unconfident.step(0.01, True, 0, _LEVEL, gnss=4.0)
    wiring = scenario('the_estimator_wiring')
    wiring.step(0.1, True, 0, [0.0, 0.0, 6.0])
    wiring.step(0.1, True, 0, [0.0, 0.0, 6.0])
    wiring.step(0.1, True, 0, _LEVEL, gnss=10.0)
    wiring.step(0.1, True, 0, None, gnss=None)
    wiring.step(0.1, True, 0, None)  # a stale accelerometer is no reading: nothing reaches predict()
    pitot = scenario('an_in_band_pitot_wins_and_a_railed_one_falls_back', {'pitot_gain': 0.5, 'pitot_max_ms': 28.0})
    pitot.step(0.1, True, 0, _LEVEL, pitot=15.0)
    pitot.step(0.1, True, 0, _LEVEL, gnss=40.0, pitot=16.0)
    pitot.step(0.1, True, 0, _LEVEL, gnss=15.5 - 3.0, pitot=30.0)
    dead = scenario('a_dead_pitot_must_not_open_the_cap', {'pitot_min_ms': 3.0, 'pitot_max_ms': 28.0})
    for _ in range(8):
        dead.step(0.1, True, 0, [0.0, 0.0, 3.0])
    for _ in range(20):
        dead.step(0.1, True, 0, _LEVEL, pitot=0.0)
    dead.step(0.1, True, 0, _LEVEL, pitot=2.9)
    dead.step(0.1, True, 0, _LEVEL, pitot=3.1)
    jump = scenario('a_gnss_jump_is_rejected', {'wind': {'max_speed': 15.0}})
    for _ in range(8):
        jump.step(0.1, True, 0, [0.0, 0.0, 3.0])
    settled = jump.governor.airspeed()
    jump.step(0.1, True, 0, _LEVEL, gnss=settled + 60.0)
    jump.step(0.1, True, 0, _LEVEL, gnss=settled - 4.0)
    throttle = scenario('the_throttle_and_its_overrides', {'airspeed_dive_pitch': -45.0})
    throttle.seed(14.0)
    for _ in range(3):
        throttle.step(0.01, False, -600, [0.0, 0.0, 2.0])  # a settled glide throttles: the charge waits
    throttle.step(0.01, False, -6000, [0.0, 0.0, 2.0])  # a dive forces the update at once
    throttle.step(0.01, False, -4500, [0.0, 0.0, 2.0])  # exactly the dive pitch counts as a dive
    throttle.step(0.01, False, -4499, [0.0, 0.0, 2.0])  # a centidegree above it does not
    throttle.step(0.01, True, 0, [0.0, 0.0, 2.0])  # the caller's override never throttles
    rederive = scenario('the_interval_rederives_from_the_fresh_estimate', {'airspeed_floor_hz': 5,
                                                                           'airspeed_ceiling_hz': 50})
    rederive.seed(14.0)
    for _ in range(30):  # due after 0.2 s (the floor at rest), then every 1/16 s off the blended 16.8 m/s
        rederive.step(0.011, False, -600, _LEVEL, gnss=28.0)
    steep = scenario('steep_attitude_gates_the_gnss')
    steep.seed(14.0)
    steep.step(0.1, True, 8500, _LEVEL, gnss=0.0)
    steep.step(0.1, True, -8000, _LEVEL, gnss=0.0)
    steep.step(0.1, True, -600, _LEVEL, gnss=0.0)
    tight = scenario('a_tighter_steep_pitch_gates_earlier', {'gnss_steep_pitch': 10.0})
    tight.seed(14.0)
    tight.step(0.1, True, -1500, _LEVEL, gnss=0.0)
    seeded = scenario('a_warm_start_seed_caps_at_once')
    seeded.step(0.01, True, 0, _LEVEL)
    seeded.seed(16.5)
    seeded.seed(-1.0)  # garbage: refused, the cap re-derives off the kept estimate
    garbage = scenario('a_garbage_seed_on_a_fresh_governor_caps_off_zero')
    garbage.seed(-1.0)  # refused, yet the cap comes off the unseeded 0 m/s: the full 45 degrees
    garbage.step(0.01, True, 0, _LEVEL)  # the next update re-caps off the unconfident speed
    # every decision edge on its exact value (see the module notes), and one step to its other side
    seeding = scenario('an_unconfident_estimate_takes_any_fix_as_the_seed')
    seeding.step(0.01, True, 0, _LEVEL, gnss=25.0)  # 25 m/s off the uncharged 0, past max_wind: the seed, cap 20
    band = scenario('the_pitot_band_includes_its_floor_and_excludes_its_ceiling')
    band.seed(20.0)
    band.step(0.01, True, 0, _LEVEL, pitot=28.0)  # exactly pitot_max_ms: railed, dropped (no fix: nothing moves)
    band.step(0.01, True, 0, _LEVEL, pitot=3.0)  # exactly pitot_min_ms: in band, blended to 11.5
    gate = scenario('the_steep_gate_includes_its_bound_both_ways')
    gate.seed(14.0)
    gate.step(0.01, True, 4500, _LEVEL, gnss=0.0)  # exactly +45 degrees: gated
    gate.step(0.01, True, -4500, _LEVEL, gnss=0.0)  # exactly -45 degrees: gated
    gate.step(0.01, True, 4499, _LEVEL, gnss=0.0)  # a centidegree inside: blends to 11.2
    bound = scenario('the_wind_bound_includes_its_bound')
    bound.seed(20.0)
    bound.step(0.01, True, 0, _LEVEL, gnss=35.5)  # 15.5 m/s off: a jump, rejected
    bound.step(0.01, True, 0, _LEVEL, gnss=35.0)  # exactly max_wind off: plausible, blended to 23.0
    return scenarios


def _accelerometer(generator: random.Random, magnitude_g: float) -> list:
    """An (x, y, z) reading of about this magnitude in g, in a random direction near the body's z axis."""
    lateral_x, lateral_y = generator.gauss(0.0, 0.08), generator.gauss(0.0, 0.08)
    axial = (max(magnitude_g * magnitude_g - lateral_x * lateral_x - lateral_y * lateral_y, 0.0)) ** 0.5
    return [round(lateral_x, 4), round(lateral_y, 4), round(axial, 4)]


def _sweep(generator: random.Random, name: str) -> _Scenario:
    """
    One seeded flight at ~100 Hz: pad, boost, deceleration and a throttled glide, with every source misbehaving.

    Args:
        generator - the seeded random source.
        name - the scenario name.

    Returns:
        The built scenario (with risks: the caller draws again).
    """
    config = dict(_DEFAULTS, airspeed_floor_hz=generator.choice((5, 4, 8)),
                  airspeed_ceiling_hz=generator.choice((50, 40, 60)),
                  pitot_gain=generator.choice((0.5, 0.4)), wind={'max_speed': generator.choice((15.0, 10.0))})
    built = _Scenario(name, config, generator.choice((1.0, 0.75, 0.5, 1.25)))

    def dt() -> float:
        return round(generator.uniform(0.0085, 0.0115), 5)

    for _ in range(generator.randint(5, 10)):  # the pad: 1 g with noise, a still-air pitot, a crawling fix
        built.step(dt(), True, generator.randint(8500, 8900), _accelerometer(generator, generator.gauss(1.0, 0.02)),
                   gnss=round(generator.uniform(0.0, 0.6), 3) if generator.random() < 0.5 else None,
                   pitot=round(generator.uniform(0.0, 1.5), 3))
    for _ in range(generator.randint(35, 50)):  # the boost: several g, steep, a railed pitot, a lagging fix
        built.step(dt(), True, generator.randint(8000, 8900), _accelerometer(generator, generator.uniform(3.0, 8.0)),
                   gnss=round(generator.uniform(0.0, 12.0), 3) if generator.random() < 0.3 else None,
                   pitot=round(generator.uniform(28.5, 32.0), 3) if generator.random() < 0.8 else None)
    for _ in range(generator.randint(15, 25)):  # the deceleration: under 1 g, pitching over, still full rate
        built.step(dt(), True, generator.randint(0, 6000), _accelerometer(generator, generator.uniform(0.2, 0.9)),
                   gnss=round(generator.uniform(10.0, 40.0), 3) if generator.random() < 0.3 else None,
                   pitot=round(generator.uniform(20.0, 31.0), 3) if generator.random() < 0.6 else None)
    pitch = -600
    for index in range(generator.randint(90, 120)):  # the glide: throttled, a dive, dropouts, jumps, a dead pitot
        pitch = max(-8000, min(2000, pitch + generator.randint(-150, 150)))
        if 30 <= index < 40 and generator.random() < 0.5:
            pitch = generator.randint(-7000, -4600)  # a steep nose-down: full rate at once
        accel = None if generator.random() < 0.05 else _accelerometer(generator, generator.gauss(1.0, 0.15))
        gnss_draw = generator.random()
        gnss = (None if gnss_draw < 0.3 else round(generator.uniform(8.0, 24.0), 3) if gnss_draw < 0.9
                else round(generator.uniform(40.0, 70.0), 3))
        pitot_draw = generator.random()
        pitot = (None if pitot_draw < 0.25 else round(generator.uniform(9.0, 22.0), 3) if pitot_draw < 0.8
                 else round(generator.uniform(0.0, 2.5), 3))
        built.step(dt(), False, pitch, accel, gnss=gnss, pitot=pitot)
    return built


def _interval_cases() -> list:
    """update_interval over test_governor's speeds and the whole range, at the default and two other rate bands."""
    cases = []
    for config in ({'airspeed_floor_hz': 5, 'airspeed_ceiling_hz': 50}, {'airspeed_floor_hz': 4,
                                                                          'airspeed_ceiling_hz': 40},
                   {'airspeed_floor_hz': 8, 'airspeed_ceiling_hz': 60}):
        resolved = governor.GovernorConfig(config)
        for speed in (0.0, 4.0, 14.0, 14.9, 30.0, 50.0, 75.0, -3.0, 0.5, 79.5, 80.0, 200.0) + tuple(
                float(whole) for whole in range(0, 82, 3)):
            cases.append({'name': 'floor_%d_ceiling_%d_speed_%r' % (config['airspeed_floor_hz'],
                                                                   config['airspeed_ceiling_hz'], speed),
                          'input': {'config': config, 'speed': speed},
                          'output': resolved.update_interval(speed)})
    return cases


def build() -> dict:
    """
    The governor fixture document.

    Args:
        (none)

    Returns:
        {meta, update_interval, scenarios}.

    Raises:
        ValueError - an explicit scenario takes a decision closer than its margin (the reasons listed).
    """
    scenarios = _explicit()
    risky = ['%s %s' % (scenario.name, risk) for scenario in scenarios for risk in scenario.risks]
    if risky:
        raise ValueError('governor: explicit scenarios on a decision edge:\n  %s' % '\n  '.join(risky))
    generator = random.Random(_SEED)
    redrawn = 0
    for index in range(_SWEEPS):
        while True:
            scenario = _sweep(generator, 'sweep_%02d' % index)
            if not scenario.risks:
                break
            redrawn += 1
        scenarios.append(scenario)
    return {
        'meta': {'source': 'src/glider/governor.py, src/glider/airspeed.py, src/glider/commons.py'},
        'update_interval': {
            'compare': {'kind': 'abs', 'tol': _INTERVAL_TOLERANCE},
            'note': 'seconds: 1 / clamp(int(speed), floor, ceiling); f32 is within 4e-9 of plan B',
            'cases': _interval_cases(),
        },
        'scenarios': {
            'compare': {'kind': 'abs', 'tol': _TOLERANCE},
            'note': ('airspeed m/s within 1e-3 (f32 against f64); confident and cap exact; every decision vetted '
                     'clear of its edge; pitch_cd / 100 degrees'),
            'redrawn': redrawn,
            'cases': [scenario.document() for scenario in scenarios],
        },
    }
