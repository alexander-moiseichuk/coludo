"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture commons.json: the four src/glider/commons.py primitives the Rust port splits between two crates
-- the stage machine's detectors (crates/sequencer, module commons: dwell_step, apogee_step) and the flight math
(crates/control, module commons: fin_deflection_limit, altitude_m).

Each function answers in its own unit, so the fixture has four sections, each a golden fixture of its own
({compare, note, cases}):
  * dwell_step -- exact. Integers and flags only: the net credit in ms, the clock of the last sample and whether
    the dwell is satisfied. Explicit cases name each rule (start, accrue, cap, drain, reset, time running
    backwards), then chains -- a noisy launch signal at 100 % noise, each case's state being plan B's answer to
    the one before -- and a seeded sweep of random states.
  * apogee_step -- abs 1e-3 m, the plan's tolerance. The Rust smoother runs in f32: at the 500 m a TMS flight
    never reaches, an f32 ulp is 3e-5 m, so a millimetre is a 30-fold margin. since_ms and fired are integers
    and flags and compare exactly. Cases stay off the one decision a float can flip, the fall below
    `peak - drop_m`: a case whose smoothed elevation lies within _DECISION_MARGIN_M of that line is skipped
    (the climb-vs-band decision cannot flip an answer beyond the tolerance: both sides clear the dwell and
    leave the peak within an ulp). Chains are lifted from test_sequencer.py (the 240 m peak and the 233 m
    ramp) and from seeded noisy arcs; the state of each case is plan B's previous answer, never re-derived.
  * fin_deflection_limit -- exact (an integer table). The test_commons, bench_cases and governor speeds, every
    whole speed of the table and past it, and a seeded sweep. int() truncates toward zero, so a speed within
    _NEAR_INTEGER of a whole number other than one is skipped: rounding it to f32 can carry it across.
  * altitude_m -- abs 0.01 m, NOT the plan's 1e-3 m. f32 cannot hold a millimetre here: the input pressure
    alone rounds by 0.0078 Pa near 100 kPa (0.65 mm), and the 0.19 power of a ratio near 1 is resolved to
    6e-8, which 44330 m turns into 2.6 mm; the worst measured over 30-110 kPa is 3.4 mm. A centimetre is a
    3-fold margin on that, and still 8 times finer than the barometers resolve (about 1 Pa, 8.4 cm), while
    every decision taken on an altitude is in metres (launch_alt_m 10, apogee_drop_m 5, land_agl_m 5). f64
    is not spent on it: decision D18 keeps f64 for the absolutely critical, and a millimetre of altitude is
    not that.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random

import commons

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_DWELL_SWEEP: int = 100
_APOGEE_SWEEP: int = 100
_FIN_SWEEP: int = 80
_ALTITUDE_SWEEP: int = 100
_DECISION_MARGIN_M: float = 1e-03  # how far an apogee case keeps from the drop line (see the module notes)
_NEAR_INTEGER: float = 1e-03  # how far a fin speed keeps from a whole number it is not (see the module notes)
_ALTITUDE_TOLERANCE_M: float = 0.01  # see the module notes: f32 measures 3.4 mm at worst
_APOGEE_TOLERANCE_M: float = 1e-03  # the plan's tolerance for the apogee tracker
_THRESHOLDS_MS: tuple = (0, 50, 100, 300, 3000)  # the dwell thresholds the sweep draws from (launch_ms 100 ...)

"""
dwell_step explicit cases: (name, active, now_ms, credit_ms, last_ms, threshold_ms). The leaky dwell's rules,
one per case, plus the corners plan B reaches when its clock steps by zero or backwards.
"""
_DWELLS: tuple = (
    ('idle_inactive_stays_idle', False, 1000, 0, None, 100),
    ('idle_active_starts_the_clock_at_zero_credit', True, 1000, 0, None, 100),
    ('idle_ignores_a_leftover_credit', True, 1000, 70, None, 100),
    ('idle_inactive_ignores_a_leftover_credit', False, 1000, 70, None, 100),
    ('active_accrues_net_time', True, 1050, 0, 1000, 100),
    ('active_reaches_the_threshold_and_fires', True, 1100, 50, 1050, 100),
    ('credit_is_capped_at_the_threshold', True, 1300, 90, 1100, 100),
    ('a_dip_drains_rather_than_resets', False, 1110, 80, 1100, 100),
    ('a_long_dip_drains_to_nothing_and_ends_the_dwell', False, 1200, 80, 1100, 100),
    ('draining_to_exactly_zero_ends_the_dwell', False, 1180, 80, 1100, 100),
    ('a_repeated_instant_with_no_credit_ends_the_dwell', True, 1000, 0, 1000, 100),
    ('a_repeated_instant_keeps_a_credit', True, 1000, 40, 1000, 100),
    ('a_zero_threshold_fires_on_the_second_sample', True, 1010, 0, 1000, 0),
    ('time_running_backwards_drains_an_active_dwell', True, 990, 50, 1000, 100),
    ('time_running_backwards_feeds_an_inactive_dwell', False, 950, 80, 1000, 100),
    ('one_huge_step_satisfies_any_dwell', True, 10_000_000, 1, 1000, 100),
    ('the_ground_dwell_of_three_seconds', True, 4000, 2950, 3950, 3000),
)

"""
apogee_step explicit cases: (name, elevation, now_ms, peak, since_ms, smooth, drop_m, dwell_ms). The plan B
apogee rules, one per case: no reading, seeding, climbing, the drop band, the fall, the dwell.
"""
_APOGEES: tuple = (
    ('no_reading_keeps_the_state', None, 2200, 120.0, None, 118.5, 5.0, 100),
    ('no_reading_keeps_an_open_dwell', None, 2300, 120.0, 2250, 110.0, 5.0, 100),
    ('no_reading_before_any_reading', None, 2300, None, None, None, 5.0, 100),
    ('the_first_reading_seeds_the_smoother_and_the_peak', 240.0, 2200, None, None, None, 5.0, 100),
    ('climbing_raises_the_peak_and_clears_the_dwell', 260.0, 2210, 240.0, 2150, 240.0, 5.0, 100),
    ('a_spike_moves_the_smoother_a_quarter_of_the_way', 180.0, 2210, 100.0, None, 100.0, 5.0, 100),
    ('one_sample_below_stays_inside_the_drop_band', 233.0, 2210, 240.0, None, 240.0, 5.0, 100),
    ('a_level_reading_stays_inside_the_band', 240.0, 2210, 240.0, None, 240.0, 5.0, 100),
    ('falling_off_the_peak_opens_the_dwell', 200.0, 2300, 240.0, None, 236.0, 5.0, 100),
    ('the_dwell_not_yet_held', 200.0, 2350, 240.0, 2300, 226.0, 5.0, 100),
    ('the_dwell_held_fires', 200.0, 2400, 240.0, 2300, 226.0, 5.0, 100),
    ('a_zero_dwell_fires_on_the_first_fall', 200.0, 2300, 240.0, None, 236.0, 5.0, 0),
    ('rising_back_into_the_band_clears_the_dwell', 260.0, 2400, 240.0, 2300, 230.0, 5.0, 100),
    ('below_the_pad', -14.0, 5000, -1.0, None, -4.0, 5.0, 100),
    ('a_wider_drop_band', 200.0, 2300, 240.0, None, 236.0, 40.0, 100),
    ('time_running_backwards_never_satisfies_the_dwell', 200.0, 2200, 240.0, 2300, 226.0, 5.0, 100),
)

"""fin_deflection_limit explicit speeds (m/s): test_commons, bench_cases, the governor tests, and the table's edges."""
_FIN_SPEEDS: tuple = (0, 15, 30, 50, 200, -5, 0.0, 5.0, 15.0, 30.0, 90.0, 40.0, 4.0, 0.5, 0.99, 16.5, 29.99, 30.5,
                      49.9, 79.9, 80.0, 80.5, 81.0, -0.5, -1.5, -1e6, 1e6, 1e30)

"""altitude_m explicit pressures (Pa): the datum, test_bmp581's 101336 and its dead read, and pads up to 3 km."""
_PRESSURES: tuple = (101325.0, 101336.0, 0.0, -5.0, 100000.0, 98000.0, 95000.0, 89875.0, 84556.0, 79495.0,
                     70109.0, 105000.0, 110000.0, 30000.0, 1.0, 101325)


def _dwell_case(name: str, active: bool, now_ms: int, credit_ms: int, last_ms: int | None,
                threshold_ms: int) -> dict:
    """One dwell_step case: the arguments, plan B's (credit_ms, last_ms, fired)."""
    credit, last, fired = commons.dwell_step(active, now_ms, credit_ms, last_ms, threshold_ms)
    return {'name': name,
            'input': {'active': active, 'now_ms': now_ms, 'credit_ms': credit_ms, 'last_ms': last_ms,
                      'threshold_ms': threshold_ms},
            'output': {'credit_ms': credit, 'last_ms': last, 'fired': fired}}


def _dwell_chain(generator: random.Random, name: str, samples: int, active_odds: float, threshold_ms: int) -> list:
    """
    A launch dwell under noise: each case's state is plan B's answer to the one before.

    The 100 % noise study (doc/sims/TMS-7-noise_tolerance) is why the dwell leaks instead of resetting: the
    magnitude crosses the threshold both ways every few samples, and only a net credit ever adds up.

    Args:
        generator - the seeded random source.
        name - the chain's case-name prefix.
        samples - how many 10 ms samples.
        active_odds - the chance a sample is over the threshold.
        threshold_ms - the dwell's threshold.

    Returns:
        The cases, in order; the chain restarts its clock where a sample ends the dwell, as the sequencer's does.
    """
    cases = []
    credit, last, now = 0, None, 50_000
    for index in range(samples):
        now += generator.choice((10, 10, 10, 11, 9, 20))
        active = generator.random() < active_odds
        cases.append(_dwell_case('%s_%03d' % (name, index), active, now, credit, last, threshold_ms))
        credit, last, fired = commons.dwell_step(active, now, credit, last, threshold_ms)
        if fired:  # the sequencer advances on a fired dwell: a fresh one starts afresh
            credit, last = 0, None
    return cases


def _dwell_sweep(generator: random.Random) -> list:
    """
    Random dwell states: idle or running, any credit up to the threshold, a step of -50..400 ms.

    Args:
        generator - the seeded random source.

    Returns:
        The cases.
    """
    cases = []
    for index in range(_DWELL_SWEEP):
        threshold = generator.choice(_THRESHOLDS_MS)
        last = None if generator.random() < 0.2 else generator.randint(0, 1_000_000)
        now = (last if last is not None else generator.randint(0, 1_000_000)) + generator.randint(-50, 400)
        now = max(now, 0)
        credit = generator.randint(0, threshold) if last is not None else 0
        cases.append(_dwell_case('sweep_%03d' % index, generator.random() < 0.6, now, credit, last, threshold))
    return cases


def _near_the_drop_line(elevation: float | None, peak: float | None, smooth: float | None, drop_m: float) -> bool:
    """
    Whether an apogee case's smoothed elevation lands within _DECISION_MARGIN_M of `peak - drop_m`.

    That is the one comparison an f32 smoother can answer differently from plan B's f64 (see the module notes).

    Args:
        elevation, peak, smooth, drop_m - the case's arguments.

    Returns:
        True when the case sits too close to the line to compare.
    """
    if elevation is None or peak is None:
        return False
    smoothed = elevation if smooth is None else smooth + (elevation - smooth) / commons.APOGEE_SMOOTH
    return abs(smoothed - (peak - drop_m)) < _DECISION_MARGIN_M


def _apogee_case(name: str, elevation: float | None, now_ms: int, peak: float | None, since_ms: int | None,
                 smooth: float | None, drop_m: float, dwell_ms: int) -> dict:
    """One apogee_step case: the arguments, plan B's (peak, since_ms, smooth, fired)."""
    new_peak, new_since, new_smooth, fired = commons.apogee_step(elevation, now_ms, peak, since_ms, smooth, drop_m,
                                                                 dwell_ms)
    return {'name': name,
            'input': {'elevation': elevation, 'now_ms': now_ms, 'peak': peak, 'since_ms': since_ms,
                      'smooth': smooth, 'drop_m': drop_m, 'dwell_ms': dwell_ms},
            'output': {'peak': new_peak, 'since_ms': new_since, 'smooth': new_smooth, 'fired': fired}}


def _apogee_chain(name: str, samples: list, drop_m: float, dwell_ms: int) -> list:
    """
    Feed apogee_step a run of (now_ms, elevation) samples, each case's state being plan B's previous answer.

    A case too close to the drop line is left out; the chain still carries plan B's state through it.

    Args:
        name - the chain's case-name prefix.
        samples - (now_ms, elevation or None) in order.
        drop_m - the drop band.
        dwell_ms - the dwell (the sequencer passes launch_ms).

    Returns:
        The cases, in order.
    """
    cases = []
    peak = since = smooth = None
    for index, (now, elevation) in enumerate(samples):
        if not _near_the_drop_line(elevation, peak, smooth, drop_m):
            cases.append(_apogee_case('%s_%03d' % (name, index), elevation, now, peak, since, smooth, drop_m,
                                      dwell_ms))
        peak, since, smooth, fired = commons.apogee_step(elevation, now, peak, since, smooth, drop_m, dwell_ms)
        if fired:
            break
    return cases


def _test_sequencer_ramp() -> list:
    """test_sequencer.py's apogee ramp: armed at 240 m, one 233 m sample, a 233 m ramp, then 232 m."""
    return [(2200, 240.0), (2210, 233.0)] + [(tick, 233.0) for tick in range(2220, 2270, 10)] + [(2400, 232.0)]


def _noisy_arc(generator: random.Random, apogee_m: float, noise_m: float, dropout_odds: float) -> list:
    """
    A ballistic arc over the top at 20 Hz, with gaussian baro noise and dropped samples.

    Args:
        generator - the seeded random source.
        apogee_m - the arc's top.
        noise_m - the baro noise, one sigma.
        dropout_odds - the chance a sample is missing (a stale baro: no reading).

    Returns:
        (now_ms, elevation or None) samples from 3 s before the top to 5 s after it.
    """
    samples = []
    for index in range(160):
        seconds = index * 0.05 - 3.0
        height = apogee_m - 0.5 * 9.81 * seconds * seconds
        reading = None if generator.random() < dropout_odds else height + generator.gauss(0.0, noise_m)
        samples.append((100_000 + index * 50, reading))
    return samples


def _apogee_sweep(generator: random.Random) -> list:
    """
    Random single apogee states: any phase of the tracker, elevations from below the pad to 500 m.

    Args:
        generator - the seeded random source.

    Returns:
        The cases, none within _DECISION_MARGIN_M of the drop line.
    """
    cases = []
    while len(cases) < _APOGEE_SWEEP:
        peak = generator.uniform(-20.0, 500.0) if generator.random() < 0.9 else None
        smooth = None if peak is None else peak - generator.uniform(0.0, 30.0)
        elevation = None if generator.random() < 0.1 else (smooth if smooth is not None else 0.0) + generator.gauss(
            0.0, 10.0)
        now = generator.randint(0, 1_000_000)
        since = None if generator.random() < 0.5 else now - generator.randint(-20, 400)
        drop_m = generator.choice((5.0, 2.5, 10.0))
        dwell_ms = generator.choice(_THRESHOLDS_MS)
        if _near_the_drop_line(elevation, peak, smooth, drop_m):
            continue
        cases.append(_apogee_case('sweep_%03d' % len(cases), elevation, now, peak, since, smooth, drop_m, dwell_ms))
    return cases


def _fin_case(name: str, speed_ms: float) -> dict:
    """One fin_deflection_limit case: the speed, plan B's integer limit."""
    return {'name': name, 'input': speed_ms, 'output': commons.fin_deflection_limit(speed_ms)}


def _fin_sweep(generator: random.Random) -> list:
    """
    Random speeds from -10 to 120 m/s, none within _NEAR_INTEGER of a whole number (see the module notes).

    Args:
        generator - the seeded random source.

    Returns:
        The cases.
    """
    cases = []
    while len(cases) < _FIN_SWEEP:
        speed = generator.uniform(-10.0, 120.0)
        if abs(speed - round(speed)) < _NEAR_INTEGER:
            continue
        cases.append(_fin_case('sweep_%03d' % len(cases), speed))
    return cases


def _altitude_case(name: str, pressure_pa: float) -> dict:
    """One altitude_m case: the pressure, plan B's metres."""
    return {'name': name, 'input': pressure_pa, 'output': commons.altitude_m(pressure_pa)}


def build() -> dict:
    """
    The commons fixture document.

    Args:
        (none)

    Returns:
        {meta, dwell_step, apogee_step, fin_deflection_limit, altitude_m}: each section a {compare, note, cases}
        golden fixture of its own.
    """
    generator = random.Random(_SEED)
    dwells = [_dwell_case(*case) for case in _DWELLS]
    dwells += _dwell_chain(generator, 'launch_noise_100_percent', 100, 0.62, 100)
    dwells += _dwell_chain(generator, 'ground_rocking', 60, 0.8, 300)
    dwells += _dwell_chain(generator, 'lone_spikes', 40, 0.08, 100)
    dwells += _dwell_sweep(generator)
    close = [case[0] for case in _APOGEES if _near_the_drop_line(case[1], case[3], case[5], case[6])]
    if close:
        raise SystemExit('commons: explicit apogee cases on the drop line: %s' % ', '.join(close))
    apogees = [_apogee_case(*case) for case in _APOGEES]
    apogees += _apogee_chain('test_sequencer_ramp', _test_sequencer_ramp(), 5.0, 100)
    apogees += _apogee_chain('arc_clean', _noisy_arc(generator, 268.0, 0.05, 0.0), 5.0, 100)
    apogees += _apogee_chain('arc_noise_1m', _noisy_arc(generator, 168.0, 1.0, 0.05), 5.0, 100)
    apogees += _apogee_chain('arc_noise_3m_dropouts', _noisy_arc(generator, 120.0, 3.0, 0.3), 5.0, 300)
    apogees += _apogee_sweep(generator)
    fins = [_fin_case('speed_%r' % speed, speed) for speed in _FIN_SPEEDS]
    fins += [_fin_case('whole_%d' % speed, float(speed)) for speed in range(0, 82)]
    fins += _fin_sweep(generator)
    altitudes = [_altitude_case('pressure_%r' % pressure, pressure) for pressure in _PRESSURES]
    altitudes += [_altitude_case('sweep_%03d' % index, generator.uniform(30000.0, 110000.0))
                  for index in range(_ALTITUDE_SWEEP)]
    if any(not math.isfinite(case['output']) for case in altitudes):
        raise SystemExit('commons: altitude_m answered a non-finite metre')
    return {
        'meta': {'source': 'src/glider/commons.py'},
        'dwell_step': {
            'compare': {'kind': 'exact'},
            'note': 'integers and flags: the net credit in ms, the last sample clock, fired',
            'cases': dwells,
        },
        'apogee_step': {
            'compare': {'kind': 'abs', 'tol': _APOGEE_TOLERANCE_M},
            'note': ('metres; an f32 smoother is within an ulp (3e-5 m at 500 m) of plan B; no case within 1 mm '
                     'of the drop line; since_ms and fired exact'),
            'cases': apogees,
        },
        'fin_deflection_limit': {
            'compare': {'kind': 'exact'},
            'note': 'the integer table; no speed within 1e-3 of a whole number it is not',
            'cases': fins,
        },
        'altitude_m': {
            'compare': {'kind': 'abs', 'tol': _ALTITUDE_TOLERANCE_M},
            'note': ('metres; 1 cm, not the plan 1e-3 m: f32 measures 3.4 mm at worst (input rounding 0.65 mm '
                     'at 100 kPa, the 0.19 power near 1 2.6 mm); the baros resolve about 8 cm'),
            'cases': altitudes,
        },
    }
