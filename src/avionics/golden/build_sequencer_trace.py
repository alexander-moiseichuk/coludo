"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture sequencer_trace.json: plan B's stage machine (src/glider/tasks/sequencer.py, Sequencer._tick) as
the Rust port (crates/sequencer, StageMachine::step) must answer it, tick by tick. Stages and reasons compare
EXACTLY: a reason is the sequencer.csv text, and every one the post-flight tools grep must read the same.

Each scenario is stateful, {name, note, init, steps}:
  * init -- the sequencer's config (the keys the board config carries; plan B's setup() defaults fill the rest),
    the settings plan B's setup() resolved from it (so the Rust defaults are checked against plan B's own), and
    the stage the Controller holds before the first tick (a warm start restores a stage that way:
    warmstart.apply's set_stage, before any task runs).
  * steps -- {in, out} per tick. `in` is what the tick sees: now_ms, the Controller's stage and manual flag
    (after the step's events), and the three reads. `out` is plan B's answer: the stage after the tick and the
    sequencer.csv rows the tick pushed, [stage, reason] each -- none, one, or two (an external move seen on the
    same tick an own advance fires). A step the port answers differently ON PURPOSE also carries a `divergence`,
    {note, out}, the out the port owes instead: only the altitude launch's rate gate makes one (avionics-port.md
    §6; the port's `launch_alt_window_ms`, which plan B does not have), pinned by _RATE_GATE.

The stand-ins, none of which touches the stage machine's logic:
  * the Controller is plan B's own (controller.Controller): set_stage, and hold()/resume() for an operator
    `stage <name>` / `stage auto`, so a hold's manual flag is plan B's rule, not a re-statement of it;
  * the separation switch is plan B's own driver (drivers/separation.Separation._apply on a debounced level):
    it moves BOOSTING -> GLIDING and nothing else, outside the sequencer, which then records it as external;
  * the databoard parameters are stubs answering read() with the step's scripted [value, source, age]. A read
    is null (no reading at all) or {value, source}; a null source is a stale channel, and its value is what
    plan B's read() would have extrapolated (the sequencer must ignore it -- the Rust databoard has no
    extrapolation, so the Rust side sees no reading). Sources are device names: 'gnss' is the one the launch
    detector refuses;
  * sequencer.csv is captured instead of written (no Recorder runs);
  * disable_gc_flight is False: the GC switch is not ported (avionics-port.md, not ported), and toggling the
    builder's own collector would only add a log line plan B cannot write without a Recorder.

Inputs are lifted from test_sequencer.py block by block (its direct pokes at the sequencer's internals become
the real paths that produce the same state: an operator hold and release, the separation driver, a warm-start
restore), then the regressions the plan names (a stale agl at apogee, GNSS elevation on the pad, a baro dropout,
the warm-entry rebase, a manual hold) and the corners found while porting (the backstop from BOOSTING and a
separation under an operator hold among them), then a scenario for each decision no
other one pins (each comparison's strictness, the arming edge, each stage-entry reset, the launch reason text),
then the rate gate's two climbs (one fast, which both launch on; one slow, which only plan B does), then seeded
noisy flights.

Cases stay off decision thresholds, where an f32 can answer differently from plan B's f64 (_Run.check,
_Run.check_reasons, _watch_the_drop_line): no fresh reading within _MARGIN of a threshold it is compared with, no
smoothed apogee elevation within _MARGIN of the drop line, and no printed float within _MARGIN of the boundary its
rounding turns on. A scripted scenario that breaks the rule fails the build; a seeded flight that breaks it is
drawn again. One exception makes the comparisons' strictness testable: a value and a threshold that are f32s
EXACTLY, made by arithmetic that rounds nowhere (_exact), are the same numbers in both precisions, so they may
sit anywhere -- ON the threshold included, the one place a strict `>` and a `>=` part.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import math
import random
import struct
from fractions import Fraction

import commons
import config_default
import controller
from drivers import separation
from tasks import sequencer

_SEED: int = 20261006  # the seeded flights' random.Random seed: the fixture is a function of the code alone
_MARGIN: float = 1e-03  # how far a compared or printed value keeps from its threshold (see the module notes)
_DRAWS: int = 200  # a seeded flight is drawn at most this many times before the build gives up
_TICK_MS: int = 50  # the sequencer's period on the board (config period_ms 50, 20 Hz)
_GRAVITY: float = 9.81  # m/s^2: the seeded flights' simple vertical kinematics
_LASER_RANGE_M: float = 4.0  # the laser publishes only below this (the VL53 reaches about 4 m)

_STAGE = controller.Stage

"""test_sequencer.py's SPEC: small thresholds for a fast, deterministic test (period_ms is the loop's, not _tick's)."""
_SPEC: dict = {'launch_g': 3.0, 'launch_ms': 100, 'boost_timeout_ms': 500, 'apogee_arm_ms': 50, 'land_agl_m': 5.0,
               'land_ms': 100, 'still_g': 0.3, 'ground_ms': 300}

"""
The settings plan B's Sequencer.setup() resolves, by its attribute: what init.settings reports. The loop period is
among them: the port reads it with the thresholds (its Settings), so its default is checked against plan B's too.
"""
_SETTINGS: tuple = (('period_ms', '_period_ms'), ('launch_g', '_launch_g'), ('launch_ms', '_launch_ms'),
                    ('launch_alt_m', '_launch_alt_m'), ('boost_timeout_ms', '_boost_timeout_ms'),
                    ('apogee_drop_m', '_apogee_drop_m'), ('apogee_arm_ms', '_apogee_arm_ms'),
                    ('flight_timeout_ms', '_flight_timeout_ms'), ('land_agl_m', '_land_agl_m'), ('land_ms', '_land_ms'),
                    ('still_g', '_still_g'), ('ground_ms', '_ground_ms'), ('land_timeout_ms', '_land_timeout_ms'))

"""The board-config keys the port's Settings do not read: the entry's own, and the GC switch (not ported)."""
_NOT_SETTINGS: tuple = ('name', 'activity', 'enabled', 'disable_gc_flight')


class _NearThreshold(Exception):
    """A value sits within _MARGIN of a threshold or a rounding boundary: the case cannot compare across f32."""


def _imu(x: float, y: float, z: float) -> dict:
    """A fresh accel reading in g, from the IMU."""
    return {'value': [x, y, z], 'source': 'imu'}


def _barometer(metres: float) -> dict:
    """A fresh elevation from a barometer."""
    return {'value': metres, 'source': 'baro'}


def _gnss(metres: float) -> dict:
    """A fresh elevation from the GNSS (zeroed at its first fix, metres of noise): never a launch trigger."""
    return {'value': metres, 'source': 'gnss'}


def _laser(metres: float) -> dict:
    """A fresh height above ground from the laser."""
    return {'value': metres, 'source': 'laser'}


def _stale(value: object) -> dict:
    """A stale channel: the value plan B's read() extrapolates, with no source vouching for it."""
    return {'value': value, 'source': None}


"""Shorthands for the scripted scenarios."""
_STILL: dict = _imu(0.0, 0.0, 1.0)  # 1 g at rest
_BOOST: dict = _imu(0.0, 0.0, 8.0)  # a sustained boost, well over launch_g 3


class _Parameter:
    """A databoard parameter stand-in: read() answers the step's scripted reading."""

    def __init__(self) -> None:
        """No reading until a step scripts one."""
        self.reading: dict | None = None

    def read(self) -> list:
        """[value, source, age_ms] as plan B's Parameter.read() answers it."""
        if self.reading is None:
            return [None, None, None]
        value = self.reading['value']
        source = self.reading['source']
        return [tuple(value) if isinstance(value, list) else value, source, 0 if source is not None else None]

    def value(self) -> object:
        """The scripted value (the sequencer never calls this; a stand-in answers it all the same)."""
        return None if self.reading is None else self.reading['value']


class _Rows:
    """A telemetry stream stand-in: keeps the rows pushed."""

    def __init__(self) -> None:
        """No rows yet."""
        self.rows: list = []

    def push(self, row: tuple) -> None:
        """Keep one row."""
        self.rows.append(list(row))


def _rounding_margin(value: float, decimals: int) -> float:
    """How far `value` is from the boundary its rounding to `decimals` turns on (a half of the last digit)."""
    scaled = value * 10**decimals
    return abs(scaled - math.floor(scaled) - 0.5) / 10**decimals


def _as_f32(value: float) -> float:
    """`value` as the port holds it: rounded to the nearest f32, then widened back to a float exactly."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _exact(*values: float | Fraction) -> bool:
    """
    Whether every value is an f32 exactly: an input as the fixture writes it, or an intermediate result as the
    exact rational it really is (a Fraction, so plan B's own f64 rounding cannot hide).

    When every operand of a comparison, and every intermediate result that made them, passes, neither side
    rounds anywhere: the port's f32 arithmetic and plan B's f64 arithmetic hold the SAME numbers, so the
    comparison answers alike however near the threshold.

    Args:
        values - the floats and exact intermediate results to test.

    Returns:
        True when no value needs rounding to fit an f32.
    """
    return all(Fraction(_as_f32(float(value))) == Fraction(value) for value in values)


def _exact_magnitude(vector: list) -> bool:
    """Whether |vector|^2 is computed exactly in f32 and f64 alike (x*x + y*y + z*z, every partial result)."""
    x, y, z = (Fraction(axis) for axis in vector)
    return _exact(x, y, z, x * x, y * y, z * z, x * x + y * y, x * x + y * y + z * z)


def _check_near(what: str, value: float, threshold: float, exact: bool) -> None:
    """
    Refuse a value within _MARGIN of a threshold it is compared with, unless both are exact (see _exact).

    Args:
        what - the value's name, for the refusal.
        value - plan B's value.
        threshold - the threshold it is compared with.
        exact - whether the value and the threshold are f32s exactly, made without rounding.

    Raises:
        _NearThreshold - the value is too close to call.
    """
    if not exact and abs(value - threshold) < _MARGIN:
        raise _NearThreshold('%s %r is within %g of %r' % (what, value, _MARGIN, threshold))


def _check_printed(what: str, value: float, decimals: int) -> None:
    """Refuse a printed value within _MARGIN of the boundary its rounding turns on."""
    if _rounding_margin(value, decimals) < _MARGIN:
        raise _NearThreshold('%s %r prints too close to a rounding boundary' % (what, value))


class _Run:
    """One scenario on plan B: the Controller, the sequencer under test, the separation switch and the stubs."""

    def __init__(self, config: dict, stage: str) -> None:
        """
        Build the scenario's plan B objects.

        Args:
            config - the sequencer's config keys.
            stage - the stage the Controller holds before the first tick.

        Raises:
            SystemExit - plan B's setup() refused the config.
        """
        self.controller = controller.Controller({})
        if stage != 'setting':
            self.controller.set_stage(_STAGE.NAMES[stage])  # a warm start restores the stage before any task runs
        self.sequencer = sequencer.Sequencer('sequencer', dict(config, disable_gc_flight=False), self.controller)
        if not asyncio.run(self.sequencer.setup()):
            raise SystemExit('sequencer_trace: plan B setup() failed for %r' % config)
        self.rows = _Rows()
        self.sequencer._telemetry = self.rows
        self.parameters = {'accel': _Parameter(), 'elevation': _Parameter(), 'agl': _Parameter()}
        self.sequencer._accel = self.parameters['accel']
        self.sequencer._elevation = self.parameters['elevation']
        self.sequencer._agl = self.parameters['agl']
        self.switch = separation.Separation('separation', {}, self.controller)
        self.switch._separated = False  # nested on the pad
        self.switch._telemetry = _Rows()
        machine = self.sequencer
        launch_g = Fraction(machine._launch_g)
        still_g = Fraction(machine._still_g)
        """Whether each threshold is an f32 exactly, made without rounding (see _exact): plan B's own formulas."""
        self.exact: dict = {
            'launch_g_sq': _exact(launch_g, launch_g * launch_g),
            'still_lo_sq': _exact(still_g, 1 - still_g, (1 - still_g) ** 2) if still_g < 1 else True,
            'still_hi_sq': _exact(still_g, 1 + still_g, (1 + still_g) ** 2),
            'launch_alt_m': _exact(machine._launch_alt_m),
            'land_agl_m': _exact(machine._land_agl_m),
        }

    def settings(self) -> dict:
        """The settings plan B's setup() resolved."""
        return {key: getattr(self.sequencer, attribute) for key, attribute in _SETTINGS}

    def event(self, event: str) -> None:
        """
        Apply one event before a tick.

        Args:
            event - 'separation' (the switch opens: the driver's own rule moves BOOSTING to GLIDING),
                'stage <name>' (an operator hold: Controller.hold) or 'stage auto' (Controller.resume).

        Returns:
            None; the Controller's stage or manual flag may change.
        """
        if event == 'separation':
            self.switch._apply(True)
        elif event == 'stage auto':
            self.controller.resume()
        elif event.startswith('stage ') and self.controller.hold(event[len('stage '):]):
            pass
        else:
            raise SystemExit('sequencer_trace: unknown event %r' % event)

    def check(self, reads: dict) -> None:
        """
        Refuse fresh reads within _MARGIN of a threshold they are compared with (see the module notes).

        Args:
            reads - the step's reads.

        Returns:
            None.

        Raises:
            _NearThreshold - a read is too close to call.
        """
        machine = self.sequencer
        accel = reads['accel']
        if accel is not None and accel['source'] is not None:
            magnitude = commons.magnitude_sq(*accel['value'])
            exact = _exact_magnitude(accel['value'])
            for threshold in ('launch_g_sq', 'still_lo_sq', 'still_hi_sq'):
                _check_near('|a|^2', magnitude, getattr(machine, '_' + threshold), exact and self.exact[threshold])
        elevation = reads['elevation']
        if elevation is not None and elevation['source'] is not None:
            exact = _exact(elevation['value'])
            for threshold in ('launch_alt_m', 'land_agl_m'):
                _check_near('elevation', elevation['value'], getattr(machine, '_' + threshold),
                            exact and self.exact[threshold])
        agl = reads['agl']
        if agl is not None and agl['source'] is not None:
            _check_near('agl', agl['value'], machine._land_agl_m, _exact(agl['value']) and self.exact['land_agl_m'])

    def check_reasons(self, rows: list, reads: dict) -> None:
        """
        Refuse a reason whose printed float sits within _MARGIN of a rounding boundary (see the module notes).

        Args:
            rows - the rows this tick pushed.
            reads - the step's reads.

        Returns:
            None.

        Raises:
            _NearThreshold - a printed value is too close to call.
        """
        accel = reads['accel']
        magnitude = math.sqrt(commons.magnitude_sq(*accel['value'])) if (
            accel is not None and accel['source'] is not None) else 0.0
        elevation = reads['elevation']
        fresh_elevation = elevation['value'] if elevation is not None and elevation['source'] is not None else None
        """
        The launch reason prints the elevation the LAUNCH detector took -- a fresh baro's, else the 0.0 plan B
        prints when the completing sample had none (a GNSS one is refused there): an inactive sample completes
        the leaky dwell when the clock steps backwards.
        """
        launch_elevation = fresh_elevation if (
            fresh_elevation is not None and elevation['source'] != sequencer._GNSS_SOURCE) else 0.0
        agl = reads['agl']
        height = agl['value'] if agl is not None and agl['source'] is not None else fresh_elevation
        for _stage, reason in rows:
            if reason.startswith('launch alt='):
                _check_printed('launch elevation', launch_elevation, 0)
            elif reason.startswith('launch |a|=') or reason.startswith('stationary '):
                _check_printed('|a|', magnitude, 1)
            elif reason.startswith('apogee '):
                _check_printed('apogee peak', self.sequencer._apogee_max, 0)
            elif reason.startswith('agl '):
                _check_printed('height', height, 1)


def _watch_the_drop_line() -> None:
    """
    Wrap commons.apogee_step so every call the sequencer makes refuses a smoothed elevation on the drop line.

    The sequencer calls it through the module (commons.apogee_step), so the wrapper sees every call and answers
    exactly what plan B answers: it only looks.

    The smoothed elevation is carried state, so it is exact (see _exact) only while EVERY smoothing step since
    the seed was: one rounded step leaves the port's f32 smoother a hair off plan B's for the rest of the
    climb. The wrapper keeps that verdict across calls and starts it over at every seed.

    Args:
        (none)

    Returns:
        None; commons.apogee_step is wrapped for this process.
    """
    original = commons.apogee_step
    weight = Fraction(commons.APOGEE_SMOOTH)
    chain = {'exact': False}  # whether every smoothed value since the seed is an f32 exactly

    def watched(elevation: float | None, now_ms: int, peak: float | None, since_ms: int | None,
                smooth: float | None, drop_m: float, dwell_ms: int) -> tuple:
        """commons.apogee_step, after refusing a smoothed elevation on the drop line."""
        if elevation is not None:
            if smooth is None:
                smoothed = elevation
                chain['exact'] = _exact(elevation)
            else:
                smoothed = smooth + (elevation - smooth) / commons.APOGEE_SMOOTH
                difference = Fraction(elevation) - Fraction(smooth)
                chain['exact'] = chain['exact'] and _exact(elevation, difference, difference / weight,
                                                           Fraction(smooth) + difference / weight)
            if peak is not None:
                line_exact = chain['exact'] and _exact(drop_m, Fraction(peak) - Fraction(drop_m))
                _check_near('smoothed elevation', smoothed, peak - drop_m, line_exact)
        return original(elevation, now_ms, peak, since_ms, smooth, drop_m, dwell_ms)

    commons.apogee_step = watched


def _scenario(name: str, note: str, config: dict, steps: list, stage: str = 'setting') -> dict:
    """
    Run one scenario on plan B.

    Args:
        name - the scenario name.
        note - what it shows, for a reader of the fixture.
        config - the sequencer's config keys.
        steps - (now_ms, events, read changes) per tick: events is a tuple of event strings applied in order
            before the tick; read changes update the reads carried from the step before (all null at first).
        stage - the stage the Controller holds before the first tick ('setting' unless a warm start).

    Returns:
        {name, note, init: {config, settings, stage}, steps: [{in, out}]}.

    Raises:
        _NearThreshold - a value sits too close to a threshold or a rounding boundary.
    """
    run = _Run(config, stage)
    reads = {'accel': None, 'elevation': None, 'agl': None}
    steps_out = []
    for now, events, changes in steps:
        for event in events:
            run.event(event)
        reads = dict(reads, **changes)
        run.check(reads)
        for channel, parameter in run.parameters.items():
            parameter.reading = reads[channel]
        step_in = {'now_ms': now, 'stage': _STAGE.STAGES[run.controller.stage], 'manual': run.controller.manual,
                   'reads': reads}
        if events:
            step_in['events'] = list(events)
        before = len(run.rows.rows)
        run.sequencer._tick(now)
        pushed = run.rows.rows[before:]
        run.check_reasons(pushed, reads)
        steps_out.append({'in': step_in, 'out': {'stage': _STAGE.STAGES[run.controller.stage], 'records': pushed}})
    return {'name': name, 'note': note, 'init': {'config': config, 'settings': run.settings(), 'stage': stage},
            'steps': steps_out}


def _ticks(start: int, end: int, step: int = _TICK_MS) -> range:
    """Tick clocks from start up to (not including) end."""
    return range(start, end, step)


def _scripted() -> list:
    """
    The scripted scenarios: test_sequencer.py block by block, then the regressions and the porting corners.

    Returns:
        The scenario documents.
    """
    scenarios = []
    add = scenarios.append
    add(_scenario(
        'launch_burnout_landing_still',
        'test_sequencer: a sustained 8 g launch, the burnout timeout, a laser landing past one spike, then still. '
        'The first tick records setting as external: plan B starts with no stage seen.',
        _SPEC, [
            (0, (), {'accel': _STILL}),
            (1000, (), {'accel': _BOOST}),
            (1100, (), {}),
            (1110, (), {}),
            (1600, (), {}),
            (1620, (), {}),
            (1630, (), {'accel': _STILL}),
            (1640, (), {'agl': _laser(2.0)}),
            (1650, (), {'agl': _laser(50.0)}),
            (1700, (), {'agl': _laser(2.0)}),
            (1790, (), {}),
            (1810, (), {}),
            (1820, (), {}),
            (2000, (), {}),
            (2110, (), {}),
            (2130, (), {}),
            (2200, (), {}),
        ]))
    add(_scenario(
        'stale_agl_at_apogee',
        'test_sequencer: the flight lost at apogee -- a stale laser extrapolating -9.6 m must not land the glider '
        'at 274 m; the baro fallback lands it once the elevation really comes down.',
        _SPEC, [
            (0, (), {'accel': _STILL, 'elevation': _barometer(0.4), 'agl': _laser(0.2)}),
            (100, (), {'accel': _BOOST}),
            (150, (), {}),
            (200, (), {'elevation': _barometer(30.0), 'agl': _stale(-0.4)}),
            (250, (), {'elevation': _barometer(60.0)}),
            (300, ('separation',), {'elevation': _barometer(120.0), 'agl': _stale(-9.6)}),
            (350, (), {'accel': _STILL, 'elevation': _barometer(274.0)}),
            (400, (), {}),
            (900, (), {'agl': _stale(-30.0)}),
            (1500, (), {'agl': _stale(-62.5)}),
            (5000, (), {'elevation': _barometer(1.0), 'agl': _stale(-250.0)}),
            (5050, (), {}),
            (5100, (), {}),
        ]))
    add(_scenario(
        'stale_barometer_never_launches',
        'test_sequencer: a stale baro extrapolating past launch_alt_m is no launch; one fresh 15 m sample is no '
        'launch either; 15 m held launch_ms is.',
        _SPEC, [
            (40000, (), {'accel': _STILL, 'elevation': _stale(12.0)}),
            (40100, (), {'elevation': _stale(25.0)}),
            (40200, (), {'elevation': _stale(40.0)}),
            (40300, (), {'elevation': _stale(55.0)}),
            (40400, (), {'elevation': _barometer(15.0)}),
            (40450, (), {'elevation': _barometer(3.0)}),
            (40500, (), {'elevation': _barometer(15.0)}),
            (40550, (), {}),
            (40650, (), {}),
        ]))
    add(_scenario(
        'transient_and_leaky_launch',
        'test_sequencer: a transient high g is no launch; then the leaky dwell -- dips drain the credit instead '
        'of wiping it, so a boost with dropouts still launches on 100 ms of NET time.',
        _SPEC, [
            (2000, (), {'accel': _BOOST}),
            (2050, (), {'accel': _STILL}),
            (2100, (), {}),
            (3000, (), {'accel': _BOOST}),
            (3050, (), {}),
            (3060, (), {'accel': _STILL}),
            (3070, (), {'accel': _BOOST}),
            (3110, (), {}),
            (3120, (), {'accel': None}),
            (3130, (), {'accel': _BOOST}),
            (3140, (), {'accel': _stale([0.0, 0.0, 8.0])}),
            (3150, (), {'accel': _BOOST}),
            (3160, (), {}),
            (3200, (), {}),
        ]))
    add(_scenario(
        'barometer_launch_backup_then_apogee',
        'test_sequencer: a marginal 2 g boost is caught by the baro backup (held launch_ms), then the apogee: the '
        'arming window blinds a burn spike, the IIR smoother is seeded by the first armed reading, one sample '
        'below the peak is inside the drop band, a ramp opens the dwell and launch_ms later it deploys.',
        _SPEC, [
            (2100, (), {'accel': _imu(0.0, 0.0, 2.0), 'elevation': _barometer(3.0)}),
            (2110, (), {'elevation': _barometer(15.0)}),
            (2210, (), {}),
            (2220, (), {}),
            (2240, (), {'elevation': _barometer(80.0)}),
            (2300, (), {'elevation': _barometer(240.0)}),
            (2310, (), {'elevation': _barometer(233.0)}),
            (2320, (), {}),
            (2330, (), {}),
            (2340, (), {}),
            (2350, (), {}),
            (2360, (), {}),
            (2450, (), {}),
            (2500, (), {'elevation': _barometer(232.0)}),
            (2550, (), {}),
        ]))
    add(_scenario(
        'operator_hold',
        'test_sequencer: a held stage ignores a sustained launch; `stage setting` is a return to the ground, not '
        'a hold, and the launch is detected afresh; `stage auto` releases a hold in place.',
        _SPEC, [
            (3000, (), {'accel': _STILL}),
            (3050, ('stage gliding',), {'accel': _BOOST}),
            (3100, (), {}),
            (3300, (), {}),
            (3350, ('stage setting',), {}),
            (3400, (), {}),
            (3450, (), {}),
            (3500, ('stage landing',), {'accel': _STILL}),
            (3700, (), {}),
            (3750, ('stage auto',), {}),
            (3800, (), {}),
            (4100, (), {}),
        ]))
    add(_scenario(
        'missing_readings_never_advance',
        'test_sequencer: no reading on any channel -- no launch, no crash, however long; a held flight with no '
        'readings stays in GLIDING until the RSO backstop.',
        dict(_SPEC, flight_timeout_ms=2000), [
            (4000, (), {}),
            (4050, (), {}),
            (4100, (), {}),
            (6000, (), {}),
            (6050, ('stage gliding', 'stage auto'), {}),
            (7000, (), {}),
            (8000, (), {}),
            (8050, (), {}),
        ]))
    add(_scenario(
        'apogee_arming_window',
        'test_sequencer: entering BOOSTING (a warm start here) starts the arming clock; a burn spike then a 60 m '
        '"drop" inside the window neither deploys nor seeds the peak; the burnout timeout runs regardless.',
        dict(_SPEC, apogee_arm_ms=100000), [
            (5000, (), {'accel': _BOOST}),
            (5010, (), {'elevation': _barometer(80.0)}),
            (5020, (), {'elevation': _barometer(20.0)}),
            (5150, (), {}),
            (5490, (), {}),
            (5500, (), {}),
            (5510, (), {}),
        ], stage='boosting'))
    add(_scenario(
        'flight_timeout_backstop',
        'test_sequencer: the separation switch moves BOOSTING to GLIDING (recorded as external), every landing '
        'sensor is silent, and flight_timeout_ms after BOOSTING entry the RSO backstop forces DONE.',
        dict(_SPEC, flight_timeout_ms=1000), [
            (0, (), {}),
            (500, ('separation',), {}),
            (999, (), {}),
            (1100, (), {}),
            (1200, (), {}),
        ], stage='boosting'))
    add(_scenario(
        'external_moves_are_recorded_once',
        'test_sequencer: a move from outside the sequencer is recorded as external once; the sequencer\'s own '
        'advance is recorded with its reason and never again as external; a separation outside BOOSTING moves '
        'nothing.',
        _SPEC, [
            (0, (), {'accel': _STILL}),
            (10, ('separation',), {}),
            (20, ('stage boosting', 'stage auto'), {}),
            (30, (), {}),
            (520, (), {}),
            (530, (), {}),
            (540, (), {}),
        ]))
    add(_scenario(
        'landing_timeout',
        'test_sequencer: warm into LANDING, carried at 1.6 g (never still) -- the landing timeout fires at '
        'land_timeout_ms after the entry tick, and not a tick early.',
        dict(_SPEC, land_timeout_ms=1000), [
            (50000, (), {'accel': _imu(0.0, 0.0, 1.6)}),
            (50900, (), {}),
            (50999, (), {}),
            (51000, (), {}),
            (51100, (), {}),
        ], stage='landing'))
    add(_scenario(
        'setting_forgets_the_last_flight',
        'test_sequencer: warm into GLIDING bases the RSO backstop there; `stage setting` puts the board back on '
        'the pad and forgets it, so the backstop never fires from SETTING.',
        dict(_SPEC, flight_timeout_ms=1000), [
            (70000, (), {'accel': _STILL, 'elevation': _barometer(0.5)}),
            (70100, ('stage setting',), {}),
            (71500, (), {}),
            (72500, (), {}),
        ], stage='gliding'))
    add(_scenario(
        'gnss_elevation_on_the_pad',
        'test_sequencer: the baros stale, the GNSS elevation (zeroed at its first fix, metres of noise) reads '
        '15 m held well past launch_ms -- never a launch. The same 15 m from a baro is one.',
        _SPEC, [
            (80000, (), {'accel': _STILL, 'elevation': _gnss(15.0)}),
            (80100, (), {}),
            (80200, (), {}),
            (80300, (), {}),
            (80400, (), {}),
            (80500, (), {'elevation': _barometer(15.0)}),
            (80600, (), {}),
        ]))
    add(_scenario(
        'warm_start_into_gliding_rebases_the_backstop',
        'Regression: a warm start straight into GLIDING never saw BOOSTING; the RSO backstop is based on the '
        'first tick, so it still bounds the restored flight -- flight_timeout_ms after that tick, not after boot.',
        dict(_SPEC, flight_timeout_ms=2000), [
            (10000, (), {'accel': _STILL, 'elevation': _barometer(80.0)}),
            (10050, (), {}),
            (11950, (), {}),
            (11999, (), {}),
            (12000, (), {}),
        ], stage='gliding'))
    add(_scenario(
        'warm_start_into_boosting_rearms_everything',
        'Regression: a warm start into BOOSTING bases the arming window, the burnout timeout and the backstop on '
        'the first tick; the apogee then deploys from the restored climb.',
        dict(_SPEC, apogee_arm_ms=200, boost_timeout_ms=5000), [
            (20000, (), {'accel': _imu(0.0, 0.0, 0.2), 'elevation': _barometer(150.0)}),
            (20150, (), {'elevation': _barometer(160.0)}),
            (20200, (), {'elevation': _barometer(170.0)}),
            (20250, (), {'elevation': _barometer(171.0)}),
            (20300, (), {'elevation': _barometer(150.0)}),
            (20350, (), {'elevation': _barometer(140.0)}),
            (20400, (), {}),
            (20450, (), {}),
            (20500, (), {}),
        ], stage='boosting'))
    add(_scenario(
        'barometer_dropout',
        'Regression: a baro dropout. On the pad it drains the altitude dwell; in BOOSTING the apogee tracker '
        'keeps its open dwell through the gap (no reading is not a rise) and deploys on the first fresh sample; '
        'in GLIDING with the laser out of range it resets the landing dwell.',
        dict(_SPEC, apogee_arm_ms=0, boost_timeout_ms=5000), [
            (0, (), {'accel': _imu(0.0, 0.0, 2.0), 'elevation': _barometer(12.0)}),
            (60, (), {'elevation': _stale(14.0)}),
            (120, (), {'elevation': _barometer(12.7)}),
            (200, (), {}),
            (300, (), {}),
            (350, (), {'accel': _imu(0.0, 0.0, 0.1), 'elevation': _barometer(200.0)}),
            (400, (), {}),
            (450, (), {'elevation': _barometer(150.0)}),
            (500, (), {'elevation': _stale(140.0)}),
            (550, (), {'elevation': _stale(120.0)}),
            (600, (), {'elevation': _barometer(150.0)}),
            (650, (), {'agl': _stale(-3.0)}),
            (700, (), {'elevation': _barometer(3.0)}),
            (750, (), {'elevation': _stale(2.0)}),
            (800, (), {'elevation': _barometer(3.0)}),
            (850, (), {}),
            (900, (), {}),
        ]))
    add(_scenario(
        'gnss_elevation_in_flight',
        'Corner: plan B gates the GNSS elevation out of the LAUNCH detector only. With both baros stale it feeds '
        'the apogee tracker and the landing fallback.',
        dict(_SPEC, apogee_arm_ms=0), [
            (0, (), {'accel': _BOOST, 'elevation': _barometer(0.3)}),
            (50, (), {}),
            (100, (), {}),
            (150, (), {'accel': _imu(0.0, 0.0, 0.1), 'elevation': _gnss(90.0)}),
            (200, (), {'elevation': _gnss(110.0)}),
            (250, (), {'elevation': _gnss(70.0)}),
            (300, (), {'elevation': _gnss(60.0)}),
            (350, (), {}),
            (400, (), {}),
            (450, (), {'elevation': _gnss(3.2)}),
            (500, (), {}),
            (550, (), {}),
        ]))
    add(_scenario(
        'apogee_dwell_is_launch_ms',
        'Regression: the apogee dwell reuses launch_ms (plan B passes it to apogee_step; ported faithfully). '
        'With launch_ms 300 the fall must hold 300 ms.',
        dict(_SPEC, launch_ms=300, apogee_arm_ms=0), [
            (0, (), {'accel': _imu(0.0, 0.0, 0.1), 'elevation': _barometer(200.0)}),
            (50, (), {}),
            (100, (), {'elevation': _barometer(150.0)}),
            (150, (), {}),
            (300, (), {}),
            (399, (), {}),
            (400, (), {}),
            (450, (), {}),
        ], stage='boosting'))
    add(_scenario(
        'a_long_hold_ends_in_two_records',
        'Corner: BOOSTING detected, then an operator holds GLIDING past the flight timeout. A held stage is not '
        'looked at, so `stage auto` shows the move as external AND fires the backstop on the same tick: two '
        'sequencer.csv rows from one tick.',
        dict(_SPEC, flight_timeout_ms=1000), [
            (0, (), {'accel': _BOOST}),
            (50, (), {}),
            (100, (), {}),
            (150, (), {}),
            (200, ('stage gliding',), {'accel': _STILL}),
            (1500, (), {}),
            (1550, ('stage auto',), {}),
            (1600, (), {}),
        ]))
    add(_scenario(
        'done_until_the_operator',
        'Corner: DONE has no detector and the backstop does not apply to it; `stage setting` starts a new pad '
        'session whose launch dwell starts afresh.',
        dict(_SPEC, flight_timeout_ms=1000), [
            (0, (), {'accel': _BOOST}),
            (100, ('stage done', 'stage auto'), {}),
            (5000, (), {}),
            (5050, ('stage setting',), {}),
            (5100, (), {}),
            (5150, (), {}),
        ], stage='gliding'))
    add(_scenario(
        'still_band_edges',
        'Corner: the still band is 1 +/- still_g (0.3) on |a|, compared squared. 0.69 g and 1.31 g held past '
        'ground_ms are not still; one out-of-band sample resets a running dwell (0.71 g from 850 confirms at '
        '1300, not 1150); 0.71 g and 1.29 g, each held ground_ms on its own, confirm (`stage landing` + `stage '
        'auto` re-enter LANDING between them); and the band is on the vector: |(0.3, 0.4, 0.95)| is 1.07 g.',
        dict(_SPEC, land_timeout_ms=3000), [
            (0, (), {'accel': _imu(0.0, 0.0, 0.69)}),
            (100, (), {}),
            (400, (), {}),
            (450, (), {'accel': _imu(0.0, 0.0, 1.31)}),
            (550, (), {}),
            (800, (), {}),
            (850, (), {'accel': _imu(0.0, 0.0, 0.71)}),
            (950, (), {'accel': _imu(0.0, 0.0, 1.31)}),
            (1000, (), {'accel': _imu(0.0, 0.0, 0.71)}),
            (1150, (), {}),
            (1299, (), {}),
            (1300, (), {}),
            (1350, ('stage landing', 'stage auto'), {'accel': _imu(0.0, 0.0, 1.29)}),
            (1500, (), {}),
            (1649, (), {}),
            (1650, (), {}),
            (1700, ('stage landing', 'stage auto'), {'accel': _imu(0.3, 0.4, 0.95)}),
            (1999, (), {}),
            (2000, (), {}),
        ], stage='landing'))
    add(_scenario(
        'stale_accel_never_settles',
        'Corner: a dead accel extrapolating a plausible 1 g reads as no reading, so LANDING ends on its timeout, '
        'not on a stillness nobody measured; a stale 8 g on the pad is no launch either.',
        dict(_SPEC, land_timeout_ms=1000), [
            (0, (), {'accel': _stale([0.0, 0.0, 1.0])}),
            (400, (), {}),
            (999, (), {}),
            (1000, (), {}),
            (1100, ('stage setting',), {'accel': _stale([0.0, 0.0, 8.0])}),
            (1300, (), {}),
            (1500, (), {}),
        ], stage='landing'))
    add(_scenario(
        'barometer_landing_below_the_pad',
        'Corner: the baro fallback lands on a negative elevation (a landing below the pad) and prints its sign.',
        _SPEC, [
            (0, (), {'elevation': _barometer(30.0)}),
            (50, (), {'elevation': _barometer(-1.5)}),
            (150, (), {}),
        ], stage='gliding'))
    add(_scenario(
        'shipped_defaults',
        'Corner: an empty config takes plan B setup() defaults -- boost_timeout_ms 6000 there, though '
        'config_default ships 12000 -- and the launch threshold is 2.5 g.',
        {}, [
            (0, (), {'accel': _imu(0.0, 0.0, 2.6)}),
            (50, (), {}),
            (100, (), {}),
            (150, (), {}),
            (5000, (), {}),
            (6150, (), {}),
            (6200, (), {}),
        ]))
    add(_scenario(
        'flight_timeout_from_boosting',
        'Corner: the RSO backstop bounds BOOSTING too. A launch, then no elevation at all (no apogee) and a burnout '
        'fallback set past the backstop: flight_timeout_ms after the tick that first sees BOOSTING, DONE.',
        dict(_SPEC, boost_timeout_ms=5000, flight_timeout_ms=1000), [
            (0, (), {'accel': _BOOST}),
            (50, (), {}),
            (100, (), {}),
            (150, (), {'accel': _imu(0.0, 0.0, 0.1)}),
            (1100, (), {}),
            (1149, (), {}),
            (1150, (), {}),
            (1200, (), {}),
        ]))
    add(_scenario(
        'separation_while_held',
        'Corner: an operator holds BOOSTING (`stage boosting` on the bench) and the separation switch opens. Plan '
        "B's driver advances a held BOOSTING to GLIDING and the hold stays, so the stage machine sees nothing -- not "
        'even a low laser -- until `stage auto`, which records gliding as external and starts the landing dwell.',
        _SPEC, [
            (0, (), {'accel': _STILL}),
            (50, ('stage boosting',), {}),
            (100, (), {}),
            (150, ('separation',), {}),
            (200, (), {'agl': _laser(2.0)}),
            (400, (), {}),
            (450, ('stage auto',), {}),
            (500, (), {}),
            (550, (), {}),
        ]))
    scenarios += _on_the_lines()
    scenarios += _stage_entries()
    scenarios += _launch_reasons()
    scenarios += _rate_gate()
    return scenarios


def _on_the_lines() -> list:
    """
    Every threshold comparison with a value ON its line: each is strict, and only an exact value (see _exact)
    can show it, so these use small binary fractions that an f32 holds exactly.

    Returns:
        The scenario documents.
    """
    return [
        _scenario(
            'on_the_launch_lines',
            'Strictness: |a| exactly launch_g (3.0 g, 9.0 squared) held 300 ms is no launch, nor is the baro '
            'exactly launch_alt_m (10.0 m) held as long; 3.5 g is one.',
            _SPEC, [
                (0, (), {'accel': _imu(0.0, 0.0, 3.0)}),
                (50, (), {}),
                (100, (), {}),
                (200, (), {}),
                (300, (), {}),
                (350, (), {'accel': _STILL, 'elevation': _barometer(10.0)}),
                (400, (), {}),
                (450, (), {}),
                (550, (), {}),
                (650, (), {}),
                (700, (), {'accel': _imu(0.0, 0.0, 3.5)}),
                (750, (), {}),
                (800, (), {}),
            ]),
        _scenario(
            'on_the_landing_height',
            'Strictness: a height exactly land_agl_m (5.0 m) held well past land_ms is no landing, from the baro '
            'fallback or from the laser; 4.5 m is one.',
            _SPEC, [
                (0, (), {'elevation': _barometer(5.0)}),
                (100, (), {}),
                (200, (), {}),
                (250, (), {'agl': _laser(5.0)}),
                (350, (), {}),
                (450, (), {}),
                (500, (), {'agl': _laser(4.5)}),
                (550, (), {}),
                (600, (), {}),
            ], stage='gliding'),
        _scenario(
            'on_the_still_band_edges',
            'Strictness: with still_g 0.5 the band is (0.25, 2.25) in g^2, open at both ends -- |a| exactly 0.5 g '
            'and exactly 1.5 g held past ground_ms are not still; 1.0 g is.',
            dict(_SPEC, still_g=0.5, land_timeout_ms=3000), [
                (0, (), {'accel': _imu(0.0, 0.0, 0.5)}),
                (100, (), {}),
                (400, (), {}),
                (450, (), {'accel': _imu(0.0, 0.0, 1.5)}),
                (550, (), {}),
                (800, (), {}),
                (850, (), {'accel': _STILL}),
                (1149, (), {}),
                (1150, (), {}),
            ], stage='landing'),
        _scenario(
            'on_the_drop_line',
            'Strictness: the apogee needs the smoothed elevation BELOW peak - apogee_drop_m. Seeded at 240 m, the '
            'reads 224 and 232 smooth to exactly 235.0 (240 - 5) and 235 holds it there: no apogee however long. '
            '230 smooths to 233.75, below the line, and launch_ms later it deploys.',
            dict(_SPEC, apogee_arm_ms=0, boost_timeout_ms=5000), [
                (0, (), {'elevation': _barometer(240.0)}),
                (50, (), {'elevation': _barometer(224.0)}),
                (100, (), {'elevation': _barometer(232.0)}),
                (150, (), {'elevation': _barometer(235.0)}),
                (200, (), {}),
                (300, (), {}),
                (400, (), {}),
                (450, (), {'elevation': _barometer(230.0)}),
                (500, (), {}),
                (550, (), {}),
            ], stage='boosting'),
        _scenario(
            'a_tolerance_of_a_g_or_more',
            'Corner: still_g is a tolerance around 1 g, and at 1 g or more the band has no lower edge (plan B '
            'clamps it to 0): 0.2 g -- near free fall -- reads as still.',
            dict(_SPEC, still_g=1.5), [
                (0, (), {'accel': _imu(0.0, 0.0, 0.2)}),
                (150, (), {}),
                (300, (), {}),
            ], stage='landing'),
    ]


def _stage_entries() -> list:
    """
    What entering a stage resets -- each scenario fails if its one reset is lost -- and the two ways an
    operator move goes unseen.

    Returns:
        The scenario documents.
    """
    return [
        _scenario(
            'setting_restarts_the_launch_dwell',
            'Entry: a launch dwell half-credited on the pad, then `stage gliding` + `stage auto` (seen) and much '
            'later `stage setting`. SETTING entry restarts the dwell, so the launch needs launch_ms afresh '
            '(5100); a kept dwell would span the 4950 ms gap and fire on the first sample at 5000.',
            _SPEC, [
                (0, (), {'accel': _BOOST}),
                (50, (), {}),
                (60, ('stage gliding', 'stage auto'), {}),
                (70, (), {}),
                (5000, ('stage setting',), {}),
                (5050, (), {}),
                (5100, (), {}),
            ]),
        _scenario(
            'setting_restarts_the_altitude_dwell',
            'Entry: the same for the baro launch backup -- 15 m half-credited, a seen GLIDING, then `stage '
            'setting`: the altitude dwell starts over and launches at 5100.',
            _SPEC, [
                (0, (), {'accel': _STILL, 'elevation': _barometer(15.0)}),
                (50, (), {}),
                (60, ('stage gliding', 'stage auto'), {}),
                (70, (), {}),
                (5000, ('stage setting',), {}),
                (5050, (), {}),
                (5100, (), {}),
            ]),
        _scenario(
            'boosting_restarts_the_apogee_tracker',
            'Entry: a warm BOOSTING tracks a 240 m peak; back to the pad, then `stage boosting` + `stage auto` '
            'at 100 m. BOOSTING entry forgets the old peak, so 100 m seeds a new one and holds (a kept 240 m peak '
            'would call apogee at once); the fresh tracker deploys on its own peak.',
            dict(_SPEC, apogee_arm_ms=0, boost_timeout_ms=5000), [
                (0, (), {'elevation': _barometer(240.0)}),
                (50, (), {}),
                (100, ('stage setting',), {'elevation': _barometer(0.5)}),
                (150, (), {}),
                (200, ('stage boosting', 'stage auto'), {'elevation': _barometer(100.0)}),
                (250, (), {}),
                (300, (), {}),
                (400, (), {}),
                (450, (), {'elevation': _barometer(70.0)}),
                (500, (), {}),
                (550, (), {}),
            ], stage='boosting'),
        _scenario(
            'warm_landing_rebases_the_backstop',
            'Entry: a warm start straight into LANDING never saw BOOSTING, so the RSO backstop is based on the '
            'first tick -- flight_timeout_ms (500) after it, before the landing timeout (1000) could fire.',
            dict(_SPEC, flight_timeout_ms=500, land_timeout_ms=1000), [
                (0, (), {'accel': _imu(0.0, 0.0, 1.6)}),
                (250, (), {}),
                (499, (), {}),
                (500, (), {}),
                (1000, (), {}),
            ], stage='landing'),
        _scenario(
            'apogee_arming_window_opens_on_time',
            'Entry: the apogee detector arms exactly apogee_arm_ms (100) after BOOSTING entry. 300 m at 99 ms is '
            'ignored, so 200 m from 100 ms seeds the peak and holds (armed a millisecond early it would call '
            'apogee from 300 m); the fall to 170 m deploys at 200 m.',
            dict(_SPEC, apogee_arm_ms=100, boost_timeout_ms=5000), [
                (0, (), {}),
                (99, (), {'elevation': _barometer(300.0)}),
                (100, (), {'elevation': _barometer(200.0)}),
                (150, (), {}),
                (250, (), {}),
                (300, (), {}),
                (350, (), {'elevation': _barometer(170.0)}),
                (400, (), {}),
                (450, (), {}),
            ], stage='boosting'),
        _scenario(
            'a_hold_hides_the_return_to_setting',
            'Hazard (plan B, ported as is): a hold observes nothing, so `stage gliding` (a fin check) and then '
            '`stage setting` never show the machine a stage change and nothing is recorded. The launch dwell '
            'half-credited before the hold survives it, and the first boosting sample after it spans the whole '
            "4950 ms gap: launch on ONE sample. In Rust that detected launch also ends the link's reconnects.",
            _SPEC, [
                (0, (), {'accel': _BOOST}),
                (50, (), {}),
                (60, ('stage gliding',), {}),
                (1000, (), {}),
                (5000, ('stage setting',), {}),
                (5050, (), {}),
            ]),
        _scenario(
            'an_advance_undone_before_the_next_tick',
            'Corner (plan B, ported as is): the machine lands the glider, and `stage gliding` + `stage auto` put '
            'it back before the next tick. The stage equals the last one seen, so the move is not recorded; the '
            'advance restarted the landing dwell, so the landing needs land_ms afresh.',
            _SPEC, [
                (0, (), {'agl': _laser(2.0)}),
                (100, (), {}),
                (150, ('stage gliding', 'stage auto'), {}),
                (200, (), {}),
                (250, (), {}),
                (300, (), {}),
            ], stage='gliding'),
    ]


def _launch_reasons() -> list:
    """
    The launch reason text: which trigger wins a tie, and the 0.0 plan B prints when the sample that completed
    the dwell had no reading of its own (an inactive sample completes a leaky dwell when the clock steps
    backwards, as test_sequencer.py's clock does).

    Returns:
        The scenario documents.
    """
    return [
        _scenario(
            'launch_altitude_wins_a_tie',
            'Reason: an 8 g boost and a 15 m baro complete their dwells on the same tick; the baro backup is '
            'checked first and its reason is the one recorded.',
            _SPEC, [
                (0, (), {'accel': _BOOST, 'elevation': _barometer(15.0)}),
                (50, (), {}),
                (100, (), {}),
            ]),
        _scenario(
            'a_backwards_clock_completes_a_launch',
            'Reason: the clock steps back 20 ms onto a sample with no reading, which FEEDS the 80 ms credit to '
            '100: the accel launch prints |a|=0.0g, the baro one alt=0m -- and so does a GNSS elevation, which '
            'the launch detector refuses.',
            _SPEC, [
                (1000, (), {'accel': _BOOST}),
                (1080, (), {}),
                (1060, (), {'accel': None}),
                (1100, (), {}),
                (2000, ('stage setting',), {'accel': _STILL, 'elevation': _barometer(15.0)}),
                (2080, (), {}),
                (2060, (), {'elevation': None}),
                (2100, (), {}),
                (3000, ('stage setting',), {'elevation': _barometer(15.0)}),
                (3080, (), {}),
                (3060, (), {'elevation': _gnss(15.0)}),
            ]),
    ]


def _climb(rate_m_per_ms: float, start_ms: int, end_ms: int) -> list:
    """
    A climb off the pad, one tick every _TICK_MS from 0 through end_ms: still, the baro at 0 m until start_ms, then
    rising at rate_m_per_ms.

    Args:
        rate_m_per_ms - the climb rate, in m per ms.
        start_ms - when the climb starts.
        end_ms - the last tick.

    Returns:
        The steps, as _scenario takes them.
    """
    return [(now, (), {'accel': _STILL, 'elevation': _barometer(round(max(0.0, rate_m_per_ms * (now - start_ms)), 3))})
            for now in _ticks(0, end_ms + 1)]


def _rate_gate() -> list:
    """
    The altitude launch's rate gate, the port's (plan B has none): a climb off the pad fast enough for it, which plan
    B and the port launch on alike, and a slow one, which only plan B launches on (a divergence, _RATE_GATE).

    Returns:
        The scenario documents.
    """
    return [
        _scenario(
            'a_fast_climb_off_the_pad',
            'Rate gate: the baro climbs 24 m/s off a 0 m pad, over launch_alt_m from 550 (10.8 m): plan B launches '
            'launch_ms later, and so does the port, the whole rise within its 1 s window of the pad held launch_ms.',
            _SPEC, _climb(0.024, 100, 650)),
        _scenario(
            'a_slow_climb_off_the_pad',
            'Rate gate: the baro climbs 8 m/s off a 0 m pad, over launch_alt_m from 1400 (10.32 m): plan B launches '
            'launch_ms later. It never rises 10 m within 1 s, so the port does not -- weather drift of this shape, '
            'only slower, launched TMS-7F unarmed on the bench overnight.',
            _SPEC, _climb(0.008, 110, 1500)),
    ]


"""
The altitude launches the port's rate gate removes, by scenario: the clock of each tick where plan B launches by
altitude though its fresh baro elevation never rose launch_alt_m within the gate's default window (1 s) above a level
it held for launch_ms, and the reason the port records on that tick instead -- None when it stays on the pad. Six read
over the line from their first fresh baro sample, so no level below is seen at all; two rise from one low sample, no
level (a glitched read on a drifted pad is one); and one is the slow climb. _pin_rate_gate makes them divergences.
"""
_RATE_GATE: dict = {
    'stale_barometer_never_launches': {40650: None},
    'barometer_launch_backup_then_apogee': {2210: None},
    'gnss_elevation_on_the_pad': {80600: None},
    'barometer_dropout': {300: None},
    'setting_restarts_the_altitude_dwell': {5100: None},
    'launch_altitude_wins_a_tie': {100: 'launch |a|=8.0g dwell=100ms'},
    'a_backwards_clock_completes_a_launch': {2060: None, 3060: None},
    'a_slow_climb_off_the_pad': {1500: None},
}


def _pin_rate_gate(scenario: dict) -> None:
    """
    Pin a scenario's rate-gate divergences (_RATE_GATE) on its steps, as the `divergence` the port owes there.

    On a pinned tick the port makes no altitude launch: it records the accel launch's reason when that completes on
    the same tick, and otherwise stays on the pad. On the tick after a launch it did not make, plan B's BOOSTING
    reaches the port as a move from outside, recorded `external` before plan B's own rows; from there on, every
    step is plan B's again.

    Args:
        scenario - the scenario document; its pinned steps gain a `divergence`.

    Returns:
        None.

    Raises:
        SystemExit - a pin no longer describes the scenario: its tick is missing or no plan B altitude launch, or
            the tick after a launch the port did not make is held or back in SETTING (the move would go unseen).
    """
    name = scenario['name']
    pins = dict(_RATE_GATE.get(name, {}))
    steps = scenario['steps']
    for index, step in enumerate(steps):
        if step['in']['now_ms'] not in pins:
            continue
        reason = pins.pop(step['in']['now_ms'])
        out = step['out']
        kept = [row for row in out['records'] if not row[1].startswith('launch alt=')]
        if len(kept) == len(out['records']):
            raise SystemExit('sequencer_trace: %s at %d is no plan B altitude launch' % (name, step['in']['now_ms']))
        if reason is not None:
            step['divergence'] = {
                'note': ('the rate gate: the baro never rose launch_alt_m within 1 s above a level it held for '
                         'launch_ms before this altitude launch, so the port does not make it; its accel launch '
                         'completes on the same tick'),
                'out': {'stage': out['stage'], 'records': kept + [[out['stage'], reason]]}}
            continue
        step['divergence'] = {
            'note': ('the rate gate: the baro never rose launch_alt_m within 1 s above a level it held for launch_ms '
                     'before this altitude launch, so the port does not make it and stays on the pad'),
            'out': {'stage': step['in']['stage'], 'records': kept}}
        if index + 1 < len(steps):
            after = steps[index + 1]
            if after['in']['manual'] or after['in']['stage'] == 'setting':
                raise SystemExit('sequencer_trace: %s at %d: the port would not see plan B\'s launch'
                                 % (name, after['in']['now_ms']))
            after['divergence'] = {
                'note': ("plan B's BOOSTING reaches the port as a move from outside, recorded external; every later "
                         "step is plan B's"),
                'out': {'stage': after['out']['stage'],
                        'records': [[after['in']['stage'], 'external']] + after['out']['records']}}
    if pins:
        raise SystemExit('sequencer_trace: %s has no tick %r' % (name, sorted(pins)))


def _shipped_config() -> dict:
    """The sequencer's entry in config_default (what a board without overrides runs), minus the keys not read."""
    for component in config_default.default()['components']:
        if component.get('name') == 'sequencer':
            return {key: value for key, value in component.items() if key not in _NOT_SETTINGS}
    raise SystemExit('sequencer_trace: config_default has no sequencer')


def _flight_steps(generator: random.Random, profile: dict) -> tuple:
    """
    One seeded flight, open loop: pad, boost, coast, glide, ground -- the reads only, at 20 Hz with jitter.

    Simple vertical kinematics: the boost accelerates at (boost_g - 1) g, the coast is ballistic to the top, the
    glide sinks at sink_ms. The accel reads the specific force (boost_g in the burn, ~0 g coasting, 1 g gliding
    and on the ground) plus gaussian noise per axis; the baro reads the height plus its noise, stale through the
    profile's dropout windows (where the GNSS, when fitted, stands in); the laser reads only below its range.

    Args:
        generator - the seeded random source.
        profile - the flight's parameters (see _FLIGHTS).

    Returns:
        (steps, top_ms): (now_ms, events, read changes) per tick, as _scenario takes them, and the clock of the
        tick at the true top of the arc (the highest height, before any noise).
    """
    steps = []
    top_ms = top_m = None
    now = 1000
    height = velocity = 0.0
    burn_s = profile['burn_s']
    boost_g = profile['boost_g']
    pad_s = profile['pad_s']
    clock = 0.0
    phase = 'pad'
    ground_since = None
    separated = False
    last_barometer = 0.0
    while True:
        interval = (_TICK_MS + generator.randint(-3, 8)) / 1000.0
        now += int(interval * 1000)
        clock += interval
        if phase == 'pad' and clock >= pad_s:
            phase = 'boost'
        elif phase == 'boost' and clock >= pad_s + burn_s:
            phase = 'coast'
        elif phase == 'coast' and velocity <= 0.0:
            phase = 'glide'
        elif phase == 'glide' and height <= 0.0:
            phase, height, ground_since = 'ground', 0.0, clock
        if phase == 'boost':
            velocity += (boost_g - 1.0) * _GRAVITY * interval
            force = boost_g
        elif phase == 'coast':
            velocity -= _GRAVITY * interval
            force = 0.0
        elif phase == 'glide':
            velocity = -profile['sink_ms']
            force = 1.0
        else:
            velocity = 0.0
            force = 1.0
        height = max(0.0, height + velocity * interval) if phase != 'pad' else 0.0
        if top_m is None or height > top_m:
            top_m, top_ms = height, now
        noise = profile['ground_noise_g'] if phase in ('pad', 'ground') else profile['acceleration_noise_g']
        if phase == 'ground' and profile['rocking']:
            noise = 0.6
        accel = _imu(*(round(generator.gauss(0.0, noise), 4) for _ in range(2)),
                     round(force + generator.gauss(0.0, noise), 4))
        dropout = any(start <= clock < end for start, end in profile['barometer_dropouts'])
        if not dropout:
            last_barometer = round(height + generator.gauss(0.0, profile['barometer_noise_m']), 3)
            elevation = _barometer(last_barometer)
        elif profile['gnss']:
            elevation = _gnss(round(height + generator.gauss(0.0, 2.0), 2))
        else:
            elevation = _stale(round(last_barometer + generator.uniform(-40.0, 40.0), 2))
        if profile['laser'] and height < _LASER_RANGE_M:
            agl = _laser(round(max(0.02, height + generator.gauss(0.0, 0.02)), 3))
        elif profile['laser']:
            agl = _stale(round(generator.uniform(-20.0, 20.0), 2))
        else:
            agl = None
        events = ()
        if profile['separation'] and not separated and phase == 'glide':
            events, separated = ('separation',), True
        steps.append((now, events, {'accel': accel, 'elevation': elevation, 'agl': agl}))
        if ground_since is not None and clock - ground_since >= profile['ground_s']:
            return steps, top_ms


"""
The seeded flights: (name, note, config, profile). Short and steep on purpose -- the trace is a tick log, and a
real 40 s glide would only repeat the same GLIDING tick some 800 times. A profile's `apogee` flag promises the
flight deploys by the baro apogee THROUGH the top: armed before the true top, deploying after it (_check_the_top).
"""
_FLIGHTS: tuple = (
    ('flight_shipped_config_separation',
     'Seeded: the config_default sequencer, a clean 4 g boost, the separation switch at the top, a laser '
     'landing, still on the ground.',
     None,
     {'pad_s': 1.0, 'burn_s': 0.8, 'boost_g': 4.0, 'sink_ms': 9.0, 'acceleration_noise_g': 0.3, 'ground_noise_g': 0.04,
      'barometer_noise_m': 0.2, 'barometer_dropouts': (), 'gnss': False, 'laser': True, 'separation': True,
      'rocking': False, 'ground_s': 4.5, 'apogee': False}),
    ('flight_noise_100_percent',
     'Seeded: plan B setup() defaults, a 3.2 g x 1.5 s boost under ~100 % accel noise (the leaky dwell must '
     'still launch), 1.5 m baro noise with dropped samples, no separation switch, no laser. The arc tops out '
     'about 5.8 s into the flight, after the detector arms (apogee_arm_ms 4000 past BOOSTING, about 5.1 s), so '
     'the smoothed peak is tracked through the noisy top and its high samples (a raw running maximum would latch '
     'one: the measured early-apogee incident), and a baro dropout falls just past the top.',
     {},
     {'pad_s': 1.0, 'burn_s': 1.5, 'boost_g': 3.2, 'sink_ms': 9.0, 'acceleration_noise_g': 1.6, 'ground_noise_g': 0.12,
      'barometer_noise_m': 1.5, 'barometer_dropouts': ((1.4, 1.6), (3.0, 3.3), (6.0, 6.2)), 'gnss': False,
      'laser': False, 'separation': False, 'rocking': False, 'ground_s': 8.0, 'apogee': True}),
    ('flight_dropouts_gnss_rocking',
     'Seeded: test_sequencer-like thresholds, long baro dropouts covered by the GNSS elevation, a laser '
     'landing, then rocking on the ground: never still, so the landing timeout ends it.',
     dict(_SPEC, apogee_arm_ms=1500, boost_timeout_ms=8000, land_timeout_ms=2500, land_ms=150),
     {'pad_s': 1.0, 'burn_s': 0.8, 'boost_g': 5.0, 'sink_ms': 12.0, 'acceleration_noise_g': 0.5, 'ground_noise_g': 0.05,
      'barometer_noise_m': 0.4, 'barometer_dropouts': ((2.0, 3.0), (4.5, 5.5)), 'gnss': True, 'laser': True,
      'separation': False, 'rocking': True, 'ground_s': 2.8, 'apogee': True}),
)


def _check_the_top(scenario: dict, top_ms: int) -> None:
    """
    Hold a flight to its `apogee` promise: the apogee detector armed before the true top of the arc, and the
    apogee row after it -- otherwise the trace never runs the peak tracker through the top it is named for.

    Args:
        scenario - the scenario document.
        top_ms - the clock of the tick at the true top.

    Raises:
        SystemExit - the profile does not keep its promise (a profile error, not an unlucky draw).
    """
    steps = scenario['steps']
    entry_ms = next(step['in']['now_ms'] for step in steps if step['in']['stage'] == 'boosting')
    armed_ms = entry_ms + scenario['init']['settings']['apogee_arm_ms']
    apogee_ms = [step['in']['now_ms'] for step in steps
                 for _stage, reason in step['out']['records'] if reason.startswith('apogee ')]
    if not (armed_ms < top_ms and len(apogee_ms) == 1 and apogee_ms[0] > top_ms):
        raise SystemExit('sequencer_trace: %s armed at %d, topped at %d, apogee at %r: the peak is not tracked '
                         'through the top' % (scenario['name'], armed_ms, top_ms, apogee_ms))


def _seeded(generator: random.Random) -> list:
    """
    The seeded flights, each drawn until no value sits too close to a threshold (see the module notes).

    Args:
        generator - the seeded random source.

    Returns:
        The scenario documents.
    """
    scenarios = []
    for name, note, config, profile in _FLIGHTS:
        config = _shipped_config() if config is None else config
        for _draw in range(_DRAWS):
            steps, top_ms = _flight_steps(generator, profile)
            try:
                scenario = _scenario(name, note, config, steps)
            except _NearThreshold:
                continue
            if profile['apogee']:
                _check_the_top(scenario, top_ms)
            scenarios.append(scenario)
            break
        else:
            raise SystemExit('sequencer_trace: %s found no clean draw in %d' % (name, _DRAWS))
    return scenarios


def build() -> dict:
    """
    The sequencer_trace fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, scenarios}.
    """
    _watch_the_drop_line()
    try:
        scenarios = _scripted()
    except _NearThreshold as error:
        raise SystemExit('sequencer_trace: a scripted scenario sits on a threshold: %s' % error) from error
    scenarios += _seeded(random.Random(_SEED))
    unknown = set(_RATE_GATE) - {scenario['name'] for scenario in scenarios}
    if unknown:
        raise SystemExit('sequencer_trace: _RATE_GATE pins scenarios there are not: %s' % sorted(unknown))
    for scenario in scenarios:
        _pin_rate_gate(scenario)
    return {
        'meta': {'source': 'src/glider/tasks/sequencer.py, src/glider/commons.py, src/glider/controller.py, '
                           'src/glider/drivers/separation.py'},
        'compare': {'kind': 'exact'},
        'scenarios': scenarios,
    }
