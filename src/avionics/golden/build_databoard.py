"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture databoard.json: plan B's latest-value store (src/glider/databoard.py, Parameter and its channels) as
the Rust port (crates/databoard, Parameter and Writer) must answer it, step by step.

Each scenario is ONE parameter, {name, kind, note, compare, steps}. `kind` is the value the parameter carries:
`scalar` (a float: altitude, pressure), `vector` (three floats: accel, rate, attitude) or `position` (two integers,
latitude and longitude in 1e-7 degrees, as the Rust port holds a fix). Every step is {in, out}; `in.now_us` is the
board's uptime when the step runs (the virtual clock: plan B stamps a push with ticks_us(), the Rust port takes the
time as an argument), and `in.action` is one of
  provide       Databoard.provide(source, {name: {priority, timeout_ms[, reconcile]}}) -- out: the shared window
                (window_us) and the sources in rank order, as the parameter keeps them
  push          the provider's channel .push(value) -- out: null
  read          Parameter.read() -- out: {value, source, age_us} of the winner, or null when no source is fresh
  fresh_below   Parameter.fresh_below(rank) -- out: a bool
  rank          Parameter.rank(source) -- out: the source's priority, or null for a source it does not know
  stamp         Parameter.stamp() -- out: the primary's last push time, in microseconds of uptime
  recent        the provider's channel .recent(window_us) -- out: its latest value if pushed within the window, or null
  latest        the provider's channel .value() -- out: its latest value whatever its age, or null before any push
  offsets       Parameter.offsets() -- out: {source: the learned bias}

What the fixture leaves out, because the port changes it on purpose (doc/specs/avionics-port.md §4.1, §6):
  * EXTRAPOLATION. Plan B's read() and value() project the primary forward when nothing is fresh; value() killed a
    flight at apogee (a stale agl extrapolated past the landing threshold). The port has no extrapolating accessor,
    so a read with no fresh source is recorded as null, and plan B's projected number is dropped. No case is built
    to exercise extrapolation, and value() is never called.
  * The BORN-STALE timestamp. Plan B backdates a new channel by its 1 s default window plus 1 us, so an unpushed
    channel is stale under any window up to 1 s; under a LONGER window (a GNSS-only tier at 2000 ms) it reads fresh,
    with no value, for its first second. The port marks a channel that never pushed as having no reading at all.
    The builder refuses any step that would see plan B's unpushed-but-fresh channel (_Scenario._unborn): an explicit
    scenario with a window past 1 s has every source push before its first query, and a sweep skips the query. It
    also refuses a stamp asked of a primary that never pushed (plan B answers the backdated tick there; the port
    answers nothing).
  * WHERE and HOW OFTEN reconcile learns. Plan B learns each backup's bias inside read(), once per new primary reading
    AT A READ, so on the board its rate was its readers' (warmstart: `altitude` every 500 ms on the pad, every 1 s
    aloft). The port learns in the primary's own push (the writer side, §4.1), because readers run on other threads
    and must not write, and takes one LOOK per _LEARN_PERIOD_US (crates/databoard LEARN_PERIOD_US: warmstart's pad
    poll), so the EMA keeps the time constant plan B flew with. The fixture reads plan B at exactly that cadence:
      - a primary push at least _LEARN_PERIOD_US after the last look (or the first) is a LOOK, and is read at the
        same instant before anything else happens (_Scenario._before), so plan B learns from what the port did;
      - any other primary push is SILENT, and no read may follow it while plan B would learn from it -- the primary
        fresh and its reading unlearned, with a pushed backup below its rank fresh beside it (_Scenario._lesson).
    A scenario that reconciles also registers all its sources before the first push (the port's topology is fixed
    before flight, and a re-ranked primary would make plan B learn from a push the port saw as a backup's).

COMPARE, per scenario. A scenario that never reconciles compares EXACTLY: the port passes every value through
untouched, so each float must come back bit for bit. Its values are f32-exact and drawn at real magnitudes with full
mantissas (pressure near 1e5 Pa, altitude 0-3000 m, unit-vector components), so a channel that dropped low mantissa
bits fails here. A reconciling scenario compares within abs 1e-3 (altitude, m) or 1e-2 (pressure, Pa): the EMA
(alpha 0.1, inexact in binary) runs in f32 on the board and in double here, and a corrected pressure near 1e5 Pa
lands on f32's 1/128 Pa grid, up to 0.004 Pa off. Both are far below any barometer's noise (the ICP-10111's is about
1 Pa, 8 cm). Integers -- ages, ranks, windows, stamps, positions -- compare exactly under every kind. The top-level
`compare` is the loosest a scenario uses.

Inputs are test_databoard.py's cases (reordered where its reconcile loop pushes the backup after the primary: the
port would learn from the backup's previous reading; spaced one look period apart), the plan's named cases (rank
handover, born stale, the window equal to the tightest timeout of the lowest-rank tier, fresh_below), a scenario per
rule (the look cadence, a window past one second, full-mantissa values), and seeded sweeps.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random
import struct
import time

import databoard
import shims

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SWEEPS: int = 24  # seeded scenarios
_SWEEP_STEPS: int = 90  # actions per seeded scenario
_TICKS_PERIOD: int = 1 << 30  # ticks_us wraps here: no scenario may reach it, the port's uptime does not wrap
_LEARN_PERIOD_US: int = 500000  # crates/databoard LEARN_PERIOD_US: the port's one look at the backups per period
_LEARN_PERIOD_MS: int = _LEARN_PERIOD_US // 1000

"""How each scenario compares (see the module notes): exactly, or within a reconciled quantity's tolerance."""
_EXACT: dict = {'kind': 'exact'}
_TOLERANCES: dict = {'altitude': 1e-03, 'pressure': 1e-02}
_LOOSEST: dict = {'kind': 'abs', 'tol': max(_TOLERANCES.values())}

"""Device names the sweeps register, at most eight to a parameter (the port's MAXIMUM_SOURCES)."""
_SOURCES: tuple = ('baro_icp10111', 'baro_bmp581', 'baro_bmp280', 'gnss', 'hitl', 'imu_bno055', 'attitude',
                   'laser_agl')

"""
The timeouts the sweeps draw, ms: 0 is the 1 s default. Those past 1 s (a GNSS-only tier, a slow backup) widen the
window past what plan B's born-stale channel covers; a sweep then skips any query until the sources have pushed.
"""
_TIMEOUTS_MS: tuple = (0, 10, 20, 40, 50, 100, 200, 500, 1000, 1500, 2000, 5000)

"""The windows a sweep's recent() draws, ms: up to the 1.5 s of a driver probe (3 x a 500 ms period) and past it."""
_RECENT_MS: tuple = (10, 50, 200, 1000, 1500, 2000, 5000)

"""The board time a sweep lets pass in one step, ms: a sensor period up to a look period and a second."""
_ADVANCES_MS: tuple = (1, 2, 5, 10, 20, 40, 50, 100, 200, 300, 500, 1000)

"""
Each float quantity's random walk, as (start low, start high, step, bias, noise), in its own unit: real magnitudes,
so every value carries a full f32 mantissa.
"""
_WALKS: dict = {
    'altitude': (0.0, 3000.0, 2.0, 15.0, 0.5),  # metres MSL, a field to a high launch site
    'pressure': (95000.0, 103000.0, 20.0, 60.0, 3.0),  # Pa near sea level: f32 steps by 1/128 Pa here
    'vector': (-1.0, 1.0, 0.05, 0.1, 0.01),  # unit-vector components, held within [-1, 1]
}

"""A position's walk in 1e-7 degree units, as (start spread, step, bias, noise): about a degree of start."""
_POSITION_WALK: tuple = (2400 * 4096, 16 * 4096, 160 * 4096, 8 * 4096)


def _f32(value: float) -> float:
    """The float nearest `value` that an f32 holds exactly: what a driver on the board would push."""
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _now() -> int:
    """
    The board's uptime in microseconds: what plan B's ticks_us() reads, since no scenario reaches the wrap.

    Raises:
        ValueError - the virtual clock reached the 2**30 us wrap, where plan B's ticks and the port's uptime part.
    """
    now = time.ticks_us()
    if shims._uptime_us >= _TICKS_PERIOD or now != shims._uptime_us:
        raise ValueError('the virtual clock reached the ticks_us wrap: plan B and the port would part')
    return now


def _as_json(value: object) -> object:
    """A plan B value as the fixture writes it: a tuple becomes a list, anything else is itself."""
    return list(value) if isinstance(value, tuple) else value


def _as_plan_b(kind: str, value: object) -> object:
    """A fixture value as plan B's drivers push it: a vector or a position is a tuple."""
    return value if kind == 'scalar' else tuple(value)


class _Scenario:
    """One parameter under test: plan B's Parameter, its providers' channels, and the steps written so far."""

    def __init__(self, name: str, kind: str = 'scalar', reconcile: bool = False, note: str = '',
                 quantity: str = 'altitude'):
        """
        Args:
            name - the scenario's name, also the parameter's (every scenario gets a parameter of its own).
            kind - 'scalar', 'vector' or 'position'.
            reconcile - whether the scenario turns offset reconciliation on (the learning rules then hold).
            note - what the scenario shows, for the fixture reader.
            quantity - what a reconciling scenario's values are, 'altitude' or 'pressure': its tolerance.
        """
        if databoard.Databoard._params.get(name) is not None:
            raise ValueError('scenario %s: a parameter of that name exists already' % name)
        self.name: str = name
        self.kind: str = kind
        self.reconcile: bool = reconcile
        self.note: str = note
        self.compare: dict = {'kind': 'abs', 'tol': _TOLERANCES[quantity]} if reconcile else _EXACT
        self.parameter = databoard.Databoard.parameter(name)
        self.handles: dict = {}  # source -> plan B's _Channel, the provider's write handle
        self.pushed: set = set()  # the sources that pushed at least once
        self.steps: list = []
        self.pending = None  # now_us of a look (a learning primary push) not yet read
        self.looked_us = None  # now_us of the port's last look
        self.any_push: bool = False

    def _before(self, action: str) -> int:
        """
        The time of a step about to run, after the look rule is checked (see the module notes).

        Raises:
            ValueError - a look is followed by anything but a read at the same instant.
        """
        now = _now()
        if self.pending is not None and not (action == 'read' and self.pending == now):
            raise ValueError('%s: a look must be read at once, got %s' % (self.name, action))
        return now

    def _unborn(self, now: int) -> object:
        """The source of a channel plan B calls fresh at `now` although it never pushed, or None."""
        for channel in self.parameter.channels:
            if channel.source not in self.pushed and channel.fresh(now, self.parameter.window_us):
                return channel.source
        return None

    def _refuse_unborn(self, now: int) -> None:
        """
        Refuse a step that would see a channel plan B calls fresh although it never pushed (see the module notes).

        Raises:
            ValueError - such a channel exists at `now`.
        """
        source = self._unborn(now)
        if source is not None:
            raise ValueError('%s: %s never pushed but reads fresh under a %d us window'
                             % (self.name, source, self.parameter.window_us))

    def _lesson(self, now: int) -> bool:
        """
        Whether plan B's read at `now` would learn from a primary reading the port took no look at: the primary
        fresh and its newest reading unlearned (plan B `_learn`'s own gate), and a pushed backup below its rank
        fresh to learn from. A read that marks the reading learned with nothing to learn changes no offset, and
        neither does the port's silence, so it is allowed.
        """
        parameter = self.parameter
        if not parameter.reconcile or not parameter.channels:
            return False
        primary = parameter.channels[0]
        if primary.t1 == parameter._learned_t1 or not primary.fresh(now, parameter.window_us):
            return False
        return any(channel.rank != primary.rank and channel.source in self.pushed
                   and channel.fresh(now, parameter.window_us) for channel in parameter.channels[1:])

    def askable(self) -> bool:
        """Whether a fresh_below now is one the fixture can take: no unpushed channel reads fresh in plan B."""
        return self._unborn(_now()) is None

    def readable(self) -> bool:
        """Whether a read now is one the fixture can take: askable, and the look's own read or no lesson in it."""
        now = _now()
        return self._unborn(now) is None and (self.pending == now or not self._lesson(now))

    def looks(self, source: str) -> bool:
        """Whether a push from `source` now is a look: the reconciling primary's, a look period after the last."""
        parameter = self.parameter
        if not parameter.reconcile or not parameter.channels or parameter.channels[0].source != source:
            return False
        return self.looked_us is None or _now() - self.looked_us >= _LEARN_PERIOD_US

    def _record(self, now: int, action: str, out: object, **arguments) -> object:
        """Write one step; returns its out."""
        step_in = {'action': action, 'now_us': now}
        step_in.update(arguments)
        self.steps.append({'in': step_in, 'out': out})
        return out

    def advance(self, milliseconds: int) -> None:
        """Let board time pass (not a step: the next step's now_us carries it)."""
        if self.pending is not None and milliseconds:
            raise ValueError('%s: time passed over an unread look' % self.name)
        shims.advance(milliseconds)

    def provide(self, source: str, priority: int, timeout_ms: int, reconcile: bool = False) -> dict:
        """Register (or re-register) a provider, as its setup() does through Databoard.provide."""
        now = self._before('provide')
        if self.reconcile and self.any_push:
            raise ValueError('%s: a reconciling scenario registers every source before the first push' % self.name)
        if reconcile and not self.reconcile:
            raise ValueError('%s: reconcile on a scenario not built to the learning rules' % self.name)
        spec = {'priority': priority, 'timeout_ms': timeout_ms}
        if reconcile:
            spec['reconcile'] = True
        self.handles[source] = databoard.Databoard.provide(source, {self.name: spec}, self.name)
        sources = [channel.source for channel in self.parameter.channels]
        out = {'window_us': self.parameter.window_us, 'sources': sources}
        return self._record(now, 'provide', out, source=source, priority=priority, timeout_ms=timeout_ms,
                            reconcile=reconcile)

    def push(self, source: str, value: object) -> None:
        """One reading from a provider, through its channel handle; a look is read at once (see the notes)."""
        look = self.looks(source)
        now = self._before('push')
        self.handles[source].push(_as_plan_b(self.kind, value))
        self.pushed.add(source)
        self.any_push = True
        if look:
            self.looked_us = now
            self.pending = now
        self._record(now, 'push', None, source=source, value=value)

    def read(self) -> object:
        """
        The fused reading, or null when no source is fresh (plan B's extrapolated number is dropped).

        Raises:
            ValueError - plan B would learn here from a primary reading the port took no look at.
        """
        now = self._before('read')
        self._refuse_unborn(now)
        if self.pending != now and self._lesson(now):
            raise ValueError('%s: plan B would learn from a silent primary push here' % self.name)
        value, source, age_ms = list(self.parameter.read())  # a copy: plan B reuses one buffer
        self.pending = None
        if source is None:
            return self._record(now, 'read', None)
        age_us = time.ticks_diff(now, self.parameter._channel(source).t1)
        if age_ms != age_us // 1000:
            raise ValueError('%s: plan B age_ms %r is not age_us %d // 1000' % (self.name, age_ms, age_us))
        return self._record(now, 'read', {'value': _as_json(value), 'source': source, 'age_us': age_us})

    def fresh_below(self, rank: int) -> bool:
        """Whether a source ranked below `rank` is fresh."""
        now = self._before('fresh_below')
        self._refuse_unborn(now)
        return self._record(now, 'fresh_below', self.parameter.fresh_below(rank), rank=rank)

    def rank(self, source: str) -> object:
        """The priority a source registered at, or null."""
        now = self._before('rank')
        return self._record(now, 'rank', self.parameter.rank(source), source=source)

    def stamp(self) -> int:
        """
        The primary's last push time.

        Raises:
            ValueError - the primary never pushed (plan B answers its backdated birth tick, the port nothing).
        """
        now = self._before('stamp')
        if not self.parameter.channels or self.parameter.channels[0].source not in self.pushed:
            raise ValueError('%s: stamp of a primary that never pushed' % self.name)
        return self._record(now, 'stamp', self.parameter.stamp())

    def recent(self, source: str, window_us: int) -> object:
        """A provider reading its own channel back, if pushed within the window."""
        now = self._before('recent')
        return self._record(now, 'recent', _as_json(self.handles[source].recent(window_us)), source=source,
                            window_us=window_us)

    def latest(self, source: str) -> object:
        """A provider's latest value, whatever its age (null before any push)."""
        now = self._before('latest')
        return self._record(now, 'latest', _as_json(self.handles[source].value()), source=source)

    def offsets(self) -> dict:
        """The learned bias per source."""
        now = self._before('offsets')
        return self._record(now, 'offsets', self.parameter.offsets())

    def document(self) -> dict:
        """
        The scenario as the fixture writes it.

        Raises:
            ValueError - a look was never read, or the parameter reconciles though the scenario was not built to
                the learning rules.
        """
        if self.pending is not None:
            raise ValueError('%s: ends on an unread look' % self.name)
        if self.parameter.reconcile and not self.reconcile:
            raise ValueError('%s: reconciles without the learning rules' % self.name)
        return {'name': self.name, 'kind': self.kind, 'note': self.note, 'compare': self.compare,
                'steps': self.steps}


def _explicit() -> list:
    """
    test_databoard.py's cases and the plan's named rules, one scenario each.

    Returns:
        The built scenarios.
    """
    scenarios = []

    def scenario(name: str, **keywords) -> _Scenario:
        built = _Scenario(name, **keywords)
        scenarios.append(built)
        return built

    handover = scenario('rank_handover', note='test_databoard.py: rank 0 answers while fresh; past its 50 ms window '
                        'the fresh rank 1 takes over; rank() and fresh_below() per source')
    handover.provide('icp', 0, 50)
    handover.provide('bmp', 1, 500)
    handover.read()  # nothing written yet
    handover.push('icp', 100.0)
    handover.push('bmp', 200.0)
    handover.read()
    handover.latest('bmp')
    handover.advance(70)
    handover.push('bmp', 201.0)
    handover.read()
    for source in ('icp', 'bmp', 'gnss'):
        handover.rank(source)
    handover.fresh_below(0)
    handover.fresh_below(1)
    handover.advance(70)
    handover.fresh_below(0)  # a stale source below is no cover
    handover.push('bmp', 201.0)
    handover.read()
    handover.push('icp', 99.5)  # the primary back: it wins again at once
    handover.read()

    vector = scenario('vector_passthrough', kind='vector', note='a single provider of a vector: passed through')
    vector.provide('adxl', 0, 100)
    vector.push('adxl', [1.0, 2.0, 3.0])
    vector.read()
    vector.advance(100)
    vector.read()  # exactly at the window: still fresh
    vector.advance(1)
    vector.read()  # past it: no reading (plan B would answer the vector as it is, with no source)

    shared = scenario('shared_window', note='test_databoard.py: the primary tier timeout governs every channel; the '
                      'backup declared 5 s and is used only while fresh within the primary 20 ms')
    shared.provide('pri', 0, 20)
    shared.provide('sec', 1, 5000)
    shared.push('pri', 10.0)
    shared.advance(5)
    shared.push('pri', 12.0)
    shared.push('sec', 5.0)
    shared.read()
    shared.advance(30)
    shared.push('sec', 6.0)
    shared.read()
    shared.advance(30)
    shared.read()  # nobody fresh in the shared 20 ms window: no reading
    shared.fresh_below(0)

    tightest = scenario('two_primaries_take_the_tighter_window', note='test_databoard.py: two rank-0 sources, the '
                        'window is the min of their timeouts; ON the window is fresh, one ms past is not')
    tightest.provide('a0', 0, 40)
    tightest.provide('b0', 0, 15)
    tightest.push('a0', 1.0)
    tightest.advance(15)
    tightest.read()
    tightest.advance(1)
    tightest.read()
    tightest.advance(9)
    tightest.read()  # 25 ms: inside a0's own 40, outside the shared 15

    window = scenario('window_follows_the_lowest_rank_tier', note='the window is the tightest timeout of the '
                      'lowest-rank tier, tracked as sources register: a lower tier resets it, the same tier takes the '
                      'min, a higher tier leaves it; timeout 0 is the 1 s default')
    window.provide('gnss', 3, 1000)
    window.provide('baro_bmp280', 2, 300)  # a lower tier: the window resets to it
    window.provide('baro_bmp581', 1, 500)
    window.provide('hitl', 1, 200)  # the same tier: the min
    window.provide('laser_agl', 1, 400)  # the same tier, looser: unchanged
    window.provide('imu_bno055', 4, 10)  # a higher tier: unchanged
    window.provide('baro_icp10111', 0, 0)  # a lower tier with the default: 1 s
    window.provide('attitude', 0, 100)  # the same tier: 100 ms
    for source in ('baro_bmp581', 'gnss'):
        window.push(source, 50.0 if source == 'gnss' else 48.5)
    window.read()  # the primary never pushed: the best fresh source, rank 1
    window.advance(100)
    window.read()  # ON the 100 ms window
    window.advance(1)
    window.read()  # the GNSS declared 1 s, but the shared window is 100 ms
    window.fresh_below(0)

    reregistered = scenario('reregistration_keeps_the_tier', note='plan B quirk: re-registering the primary at a '
                            'worse rank re-sorts the channels but keeps the tier rank and its window (a setup retry '
                            're-registers at the same rank, which changes nothing)')
    reregistered.provide('baro_icp10111', 0, 50)
    reregistered.provide('baro_bmp581', 1, 200)
    reregistered.provide('baro_icp10111', 0, 50)  # a setup retry: nothing moves
    reregistered.provide('baro_icp10111', 5, 50)  # re-ranked: now last, the window stays 50
    reregistered.provide('baro_bmp280', 1, 20)  # rank 1 is not the tier (still 0): no min with it
    reregistered.push('baro_icp10111', 10.0)
    reregistered.push('baro_bmp581', 11.0)
    reregistered.read()  # the bmp581 is first in rank order now
    reregistered.rank('baro_icp10111')
    reregistered.stamp()  # the primary is the bmp581 now
    reregistered.advance(50)
    reregistered.read()
    reregistered.advance(1)
    reregistered.read()
    reregistered.provide('baro_bmp280', 0, 30)  # back to the tier rank: min(50, 30)
    reregistered.push('baro_bmp280', 12.0)
    reregistered.read()

    born = scenario('born_stale', note='a channel that never pushed is never fresh and has no value: an unpushed '
                    'primary hands over to a pushed backup, recent() and latest() answer nothing')
    born.provide('laser_agl', 0, 0)  # the 1 s default window: the widest that is born stale in plan B
    born.provide('laser_agl_l1x', 0, 100)
    born.read()
    born.fresh_below(-1)
    born.recent('laser_agl', 1000000)
    born.latest('laser_agl')
    born.rank('laser_agl_l1x')
    born.advance(5)
    born.push('laser_agl_l1x', 3.5)
    born.read()  # the co-primary that pushed answers
    born.fresh_below(-1)
    born.latest('laser_agl')

    long = scenario('a_window_past_one_second', note='a GNSS-only tier at 2000 ms with a 5 s backup below it: the '
                    'shared window is 2 s, all of it fresh, and not a microsecond more; recent() with a driver probe '
                    'window (3 x a 500 ms period). Every source pushes before the first query: plan B is born stale '
                    'only under windows up to 1 s')
    long.provide('gnss', 0, 2000)
    long.provide('baro_bmp280', 1, 5000)
    long.push('gnss', _f32(312.789))
    long.push('baro_bmp280', _f32(310.4321))
    long.read()
    long.advance(1200)
    long.recent('gnss', 1500000)
    long.recent('gnss', 1000000)
    long.fresh_below(0)  # the backup, 1.2 s old in the 2 s window
    long.read()
    long.advance(300)
    long.read()  # 1.5 s
    long.advance(499)
    long.read()  # 1.999 s
    long.advance(1)
    long.read()  # ON the window
    long.advance(1)
    long.read()  # past it: null
    long.fresh_below(0)
    long.push('baro_bmp280', _f32(311.0625))
    long.read()  # the backup
    long.advance(2000)
    long.read()  # ON the shared window
    long.advance(1)
    long.read()  # the backup's own 5 s does not count
    long.latest('gnss')

    below = scenario('fresh_below', note='fresh_below(rank): a source with a LARGER rank number, fresh in the shared '
                     'window; equal ranks and stale ones do not count')
    below.provide('attitude', 1, 40)
    below.provide('imu_bno055', 2, 40)
    below.provide('hitl', 0, 40)
    below.provide('gnss', 2, 1000)
    for rank in (-1, 0, 1, 2):
        below.fresh_below(rank)
    below.push('imu_bno055', 1.5)
    for rank in (-1, 0, 1, 2):
        below.fresh_below(rank)
    below.advance(40)
    below.fresh_below(1)  # ON the window
    below.advance(1)
    below.fresh_below(1)
    below.push('attitude', 2.5)
    below.fresh_below(0)
    below.fresh_below(1)
    below.read()

    stamp = scenario('stamp_follows_the_primary', note='stamp(): the primary push time, for change detection; a '
                     'backup push does not move it, and it holds while the primary is stale')
    stamp.provide('gnss', 0, 500)
    stamp.provide('hitl', 1, 500)
    stamp.push('gnss', 84.375)
    stamp.stamp()
    stamp.advance(200)
    stamp.push('hitl', 90.0)
    stamp.stamp()
    stamp.advance(800)
    stamp.stamp()
    stamp.push('gnss', 85.0)
    stamp.stamp()

    recent = scenario('recent_reads_back_its_own_channel', note='recent(window): a provider vouching for its own '
                      'number (a baro re-zero); its own window, not the parameter one; latest() is any age')
    recent.provide('baro_icp10111', 0, 50)
    recent.provide('baro_bmp581', 1, 200)
    recent.push('baro_bmp581', 120.25)
    recent.advance(150)
    recent.recent('baro_bmp581', 150000)
    recent.recent('baro_bmp581', 149999)
    recent.recent('baro_icp10111', 150000)
    recent.latest('baro_bmp581')
    recent.read()  # the parameter: nothing fresh in its 50 ms window

    position = scenario('position_handover', kind='position', note='a fix as integers (1e-7 degrees): passed '
                        'through exactly, the HITL sim outranking the GNSS while fresh')
    position.provide('hitl', 0, 200)
    position.provide('gnss', 1, 200)
    position.push('gnss', [481234567, 115678901])
    position.push('hitl', [481234000, 115678000])
    position.read()
    position.advance(150)
    position.push('gnss', [481234590, 115678950])
    position.advance(60)
    position.read()

    pressure = scenario('values_pass_through_bit_for_bit', note='pressures at sea level with full f32 mantissas '
                        '(101325.37 Pa is 101325.3671875 in f32): every one comes back bit for bit through read, '
                        'recent and latest; a backup reads raw, since this parameter does not reconcile')
    pressure.provide('baro_icp10111', 0, 100)
    pressure.provide('baro_bmp581', 1, 100)
    pressure.push('baro_icp10111', _f32(101325.37))
    pressure.push('baro_bmp581', _f32(101318.912))
    pressure.read()
    pressure.latest('baro_bmp581')
    pressure.advance(150)
    pressure.push('baro_bmp581', struct.unpack('<f', bytes.fromhex('abe6c547'))[0])  # 0x47C5E6AB: 101325.336
    pressure.read()
    pressure.recent('baro_bmp581', 50000)
    pressure.push('baro_icp10111', _f32(99876.54321))
    pressure.read()
    pressure.latest('baro_icp10111')

    unit = scenario('vectors_pass_through_bit_for_bit', kind='vector', note='unit-vector components with full f32 '
                    'mantissas: every word of every vector comes back bit for bit')
    unit.provide('imu_bmi323', 0, 20)
    unit.provide('imu_bno055', 1, 40)
    unit.push('imu_bmi323', [_f32(math.sqrt(0.5)), _f32(-0.123456789), _f32(0.999999)])
    unit.push('imu_bno055', [_f32(math.pi / 4), _f32(-math.e / 3), _f32(1e-7)])
    unit.read()
    unit.advance(30)
    unit.push('imu_bno055', [_f32(-0.33333333), _f32(0.66666667), _f32(-0.987654321)])
    unit.read()  # the backup, raw
    unit.latest('imu_bmi323')

    learns = scenario('reconcile_learns_and_corrects', reconcile=True, note='test_databoard.py (the backup pushed '
                      'BEFORE the primary, so both sides learn from the same reading; the rounds a look period '
                      'apart): while the primary is fresh the backup bias is learned (EMA 0.1); on handover the '
                      'backup reads bias-corrected')
    learns.provide('icp3', 0, 50, reconcile=True)
    learns.provide('bmp3', 1, 5000)
    for backup in (102.5, 102.25, 103.0, 102.5, 102.75):
        learns.push('bmp3', backup)
        learns.push('icp3', 100.0)
        learns.read()
        learns.advance(_LEARN_PERIOD_MS)
    learns.offsets()
    learns.advance(60)
    learns.push('bmp3', 102.5)
    learns.read()  # corrected, not the raw 102.5
    learns.offsets()

    cadence = scenario('reconcile_looks_once_per_period', reconcile=True, note='the port learns at most once per '
                       'look period (500 ms, warmstart\'s pad poll: plan B\'s flown rate), however often the primary '
                       'pushes: pushes inside the period teach nothing even with the backup far off, the period runs '
                       'from the last look, and a handover corrects by the offset of the looks alone')
    cadence.provide('baro_icp10111', 0, 200, reconcile=True)
    cadence.provide('baro_bmp581', 1, 1000)
    cadence.push('baro_bmp581', _f32(124.4567))
    cadence.push('baro_icp10111', _f32(123.456))  # the first push looks
    cadence.read()
    cadence.offsets()
    for elapsed in (100, 100, 100, 100, 99):  # to 499 ms: every primary push is silent
        cadence.advance(elapsed)
        cadence.push('baro_bmp581', _f32(131.789))  # far off: a look here would move the offset by 0.8 m
        cadence.push('baro_icp10111', _f32(123.4321))
        cadence.offsets()
        cadence.stamp()
    cadence.advance(1)  # ON the period
    cadence.push('baro_bmp581', _f32(126.5432))
    cadence.push('baro_icp10111', _f32(123.75))
    cadence.read()
    cadence.offsets()
    cadence.advance(201)  # the primary stale in its 200 ms window, and the backup too
    cadence.push('baro_bmp581', _f32(127.0123))
    cadence.read()  # corrected by the two looks' offset
    cadence.advance(_LEARN_PERIOD_MS - 202)  # 499 ms after the second look: the next primary push is silent
    cadence.push('baro_bmp581', _f32(140.0))
    cadence.push('baro_icp10111', _f32(123.9))
    cadence.offsets()

    sea_level = scenario('reconcile_at_pressure_magnitude', reconcile=True, quantity='pressure', note='a backup '
                         'barometer reconciled near 101325 Pa: the bias is learned in f32 and a corrected pressure '
                         'lands on f32\'s 1/128 Pa grid (within the 0.01 Pa tolerance)')
    sea_level.provide('baro_icp10111', 0, 200, reconcile=True)
    sea_level.provide('baro_bmp581', 1, 200)
    for primary, backup in ((101325.37, 101348.91), (101325.11, 101349.73), (101324.82, 101348.05),
                            (101324.6, 101349.27)):
        sea_level.push('baro_bmp581', _f32(backup))
        sea_level.push('baro_icp10111', _f32(primary))
        sea_level.read()
        sea_level.offsets()
        sea_level.advance(_LEARN_PERIOD_MS)
    sea_level.push('baro_bmp581', _f32(101350.123))
    sea_level.read()  # the primary stale: the backup, corrected
    sea_level.advance(150)
    sea_level.push('baro_bmp581', _f32(101352.987))
    sea_level.read()

    frozen = scenario('reconcile_freezes_while_the_primary_is_stale', reconcile=True, note='offsets freeze while '
                      'the primary is stale; a stale backup learns nothing; a co-primary is never reconciled; the '
                      'flag is set by a BACKUP provider; a primary back inside the look period is silent')
    frozen.provide('baro_icp10111', 0, 100)
    frozen.provide('hitl', 0, 100)  # a co-primary
    frozen.provide('baro_bmp581', 1, 100, reconcile=True)
    frozen.provide('baro_bmp280', 2, 100)
    frozen.push('hitl', 47.0)
    frozen.push('baro_bmp581', 50.0)
    frozen.push('baro_icp10111', 48.0)  # the bmp280 never pushed: nothing to learn from it
    frozen.read()
    frozen.offsets()
    frozen.advance(_LEARN_PERIOD_MS)
    frozen.push('baro_bmp280', 45.0)
    frozen.push('baro_bmp581', 51.0)
    frozen.push('baro_icp10111', 48.5)
    frozen.read()
    frozen.offsets()
    frozen.advance(120)  # everything stale
    frozen.push('baro_bmp581', 60.0)  # a backup moving alone teaches nothing
    frozen.read()
    frozen.advance(10)
    frozen.push('baro_bmp581', 61.0)
    frozen.read()
    frozen.offsets()
    frozen.advance(150)
    frozen.push('baro_bmp280', 44.0)
    frozen.read()  # the bmp280 corrected by its own offset
    frozen.advance(101)
    frozen.push('baro_icp10111', 49.0)  # the primary back, the backups stale, and inside the period: nothing
    frozen.read()
    frozen.offsets()
    frozen.push('hitl', 47.25)
    frozen.advance(101)
    frozen.push('hitl', 47.5)
    frozen.read()  # the co-primary answers raw: it has no offset

    vectors = scenario('reconcile_skips_a_vector', kind='vector', reconcile=True, note='reconciliation is for '
                       'additive scalars: a vector never learns an offset and its backup reads raw')
    vectors.provide('imu_lsm6dso32', 0, 20, reconcile=True)
    vectors.provide('imu_bno055', 1, 40)
    vectors.push('imu_bno055', [0.0, 0.0, 1.0625])
    vectors.push('imu_lsm6dso32', [0.0, 0.0, 1.0])
    vectors.read()
    vectors.offsets()
    vectors.advance(30)
    vectors.push('imu_bno055', [0.0, 0.0, 1.125])
    vectors.read()
    return scenarios


class _Truth:
    """A sweep's physical quantity: a random walk every source reports with its own fixed bias and noise."""

    def __init__(self, generator: random.Random, shape: str):
        """
        Args:
            generator - the sweep's random source.
            shape - 'altitude' or 'pressure' (a scalar), 'vector' or 'position'.
        """
        self.generator: random.Random = generator
        self.shape: str = shape
        self.count: int = {'altitude': 1, 'pressure': 1, 'vector': 3, 'position': 2}[shape]
        if shape == 'position':
            spread = _POSITION_WALK[0]
            self.state: list = [generator.randint(-spread, spread) for _ in range(self.count)]
        else:
            low, high = _WALKS[shape][:2]
            self.state = [generator.uniform(low, high) for _ in range(self.count)]
        self.bias: dict = {}

    def _displacement(self, which: int) -> list:
        """A random displacement in the value's shape: walk member `which` (1 step, 2 bias, 3 noise) either way."""
        if self.shape == 'position':
            units = _POSITION_WALK[which]
            return [self.generator.randint(-units, units) for _ in range(self.count)]
        scale = _WALKS[self.shape][which + 1]
        return [self.generator.uniform(-scale, scale) for _ in range(self.count)]

    def _held(self, values: list) -> list:
        """The values in their shape: a vector's components within [-1, 1], floats as f32 holds them."""
        if self.shape == 'position':
            return values
        if self.shape == 'vector':
            values = [max(-1.0, min(1.0, value)) for value in values]
        return [_f32(value) for value in values]

    def walk(self) -> None:
        """Move the quantity a little."""
        self.state = [value + delta for value, delta in zip(self.state, self._displacement(1))]
        if self.shape == 'vector':
            self.state = [max(-1.0, min(1.0, value)) for value in self.state]

    def reading(self, source: str) -> object:
        """What `source` reports now: the truth plus its bias plus noise, in the kind's shape."""
        if source not in self.bias:
            self.bias[source] = self._displacement(2)
        noise = self._displacement(3)
        values = self._held([value + bias + jitter
                             for value, bias, jitter in zip(self.state, self.bias[source], noise)])
        return values[0] if self.count == 1 else values


def _sweep(generator: random.Random, name: str) -> _Scenario:
    """
    One seeded scenario: 2-6 providers, random ranks (ties included) and timeouts, pushes, time and every query.

    A reconciling one registers all its providers first and reads each look at once; a read that would let plan B
    learn from a silent push, and any query plan B would answer from an unpushed channel, is skipped. The others
    also register late providers and re-rank existing ones.
    """
    kind = generator.choice(('scalar', 'scalar', 'scalar', 'vector', 'position'))
    quantity = generator.choice(('altitude', 'pressure')) if kind == 'scalar' else kind
    reconcile = kind == 'scalar' and generator.random() < 0.5
    built = _Scenario(name, kind=kind, reconcile=reconcile, note='seeded: %s' % quantity,
                      quantity=quantity if kind == 'scalar' else 'altitude')
    truth = _Truth(generator, quantity)
    pool = list(generator.sample(_SOURCES, generator.randint(2, 6)))
    flagged = generator.choice(pool) if reconcile else None  # the provider whose spec says `reconcile: true`
    registered = []

    def register(source: str) -> None:
        built.provide(source, generator.randint(0, 3), generator.choice(_TIMEOUTS_MS), source == flagged)
        if source not in registered:
            registered.append(source)

    for source in pool[:len(pool) if reconcile else 2]:
        register(source)
    if reconcile:
        # every source's first reading, the primary's (a look, read at once) last, so that read meets no channel
        # plan B would call fresh unpushed (a reconciling sweep registers nobody later)
        primary = built.parameter.channels[0].source
        for source in [source for source in registered if source != primary] + [primary]:
            look = built.looks(source)
            truth.walk()
            built.push(source, truth.reading(source))
            if look:
                built.read()
    for _ in range(_SWEEP_STEPS):
        draw = generator.random()
        if not reconcile and draw < 0.04 and len(registered) < len(_SOURCES):
            unregistered = [source for source in _SOURCES if source not in registered]
            register(generator.choice(unregistered))
        elif not reconcile and draw < 0.07:
            register(generator.choice(registered))  # a setup retry or a re-rank
        elif draw < 0.45:
            source = generator.choice(registered)
            look = built.looks(source)
            truth.walk()
            built.push(source, truth.reading(source))
            if look:
                built.read()
        elif draw < 0.65:
            built.advance(generator.choice(_ADVANCES_MS))
        elif draw < 0.80:
            if built.readable():
                built.read()
        elif draw < 0.85:
            if built.askable():
                built.fresh_below(generator.randint(-1, 3))
        elif draw < 0.88:
            built.rank(generator.choice(_SOURCES))
        elif draw < 0.91:
            if built.parameter.channels[0].source in built.pushed:
                built.stamp()
        elif draw < 0.94:
            built.recent(generator.choice(registered), generator.choice(_RECENT_MS) * 1000)
        elif draw < 0.97:
            built.latest(generator.choice(registered))
        else:
            built.offsets()
    return built


def build() -> dict:
    """
    The databoard fixture document.

    Returns:
        {meta, compare, note, scenarios}.
    """
    scenarios = _explicit()
    generator = random.Random(_SEED)
    scenarios += [_sweep(generator, 'sweep_%02d' % index) for index in range(_SWEEPS)]
    return {
        'meta': {'source': 'src/glider/databoard.py'},
        'compare': _LOOSEST,
        'note': ('one parameter per scenario, each with its own compare; now_us is the uptime each step runs at; a '
                 'read with no fresh source is null (no extrapolation); integers compare exactly'),
        'scenarios': [scenario.document() for scenario in scenarios],
    }
