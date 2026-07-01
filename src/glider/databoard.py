# databoard.py — the shared latest-value store + sensor fusion for hot data (specs/coludo.md "Task
# Data-Flow and Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task
# with a registry of Parameter objects whose fused value is computed on read.
#
# Structure.
#   Databoard   — a registry of Parameter objects. Databoard.parameter(name) gets-or-creates one;
#                  a sensor registers itself as a source via provide() (which returns its channel
#                  handles) and then reports by pushing each channel directly -- the hot write path
#                  is one step, no lookup. value()/read() resolve the winner + primary in one pass.
#   Parameter    — one fused quantity (e.g. 'altitude') for the consumer. Holds a short LIST of
#                  channels KEPT IN RANK ORDER (lowest = primary first; a list, not a dict, is faster
#                  at this size), plus the shared freshness window derived from its primary tier.
#   _Channel     — one source's stream: a static rank (priority; lower = preferred) and TWO slots
#                  (the last two readings) -- two slots because the extrapolation here is LINEAR
#                  (needs 2 points); a degree-N model would keep N+1.
#
# Fusion is a pure read-time function, Parameter.value():
#   1. winner = the lowest-rank channel still fresh. Channels are rank-ordered, so it is just the
#      FIRST fresh one in the list (same-rank channels are equivalent). Freshness uses ONE shared
#      window per parameter: the tightest expiry among the rank-0 tier (min() if two share rank 0),
#      applied to EVERY channel. Return its value.
#   2. if NO channel is fresh, linearly extrapolate the PRIMARY (channels[0]) two slots to now --
#      project the trusted source forward rather than hand out a backup that is itself stale.
#   3. if the primary never pushed (startup), None.
# So "rank 0 answers while fresh; a backup takes over only while itself THIS fresh, else rank 0 is
# extrapolated" is EMERGENT -- every read re-evaluates freshness against the shared window. A channel
# is BORN STALE (t1 a full _DEFAULT_EXPIRE in the past), so an un-pushed channel is never fresh; and
# since every window is <= _DEFAULT_EXPIRE, a FRESH channel always has data -- which is why nothing
# downstream needs a v1-None check (a source that never produces is simply never fresh, and surfaces
# as a missing reading rather than a hidden guard).
#
# The shared window decides WHEN to fall back; offset reconciliation (opt-in, 'reconcile': true on a
# provider) decides WHAT the fallback reports. While the primary is fresh, each backup's BIAS against
# it is learned (EMA, once per new primary reading -- the rate is set by data, not by reads); on
# handover the backup's value is corrected by that offset, so it reads what the primary would --
# closing the bias gap between e.g. ICP-10111 and BMP280 rather than jumping across it. Offsets FREEZE
# while the primary is stale, and reconciliation is for additive SCALARS only (altitude, pressure) --
# never vectors (attitude/accel) or unlike quantities (agl, position). Per-source slots keep
# extrapolation within a single source.
#
# Dependencies. A sensor that consumes another's quantity grabs a read handle with parameter(*names)
# (get-or-create, so setup order does not matter); a provider gets its write-channels from
# provide(source, provides, *want). Both return one handle for one name, a tuple for several.
#
# Telemetry is separate: each sensor writes its own raw SENSOR.csv directly. A global singleton,
# Inspectable as `databoard` (fused value/source/age per parameter).

import time

import inspector

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_DEFAULT_EXPIRE_US: int = const(1000_000)  # default + upper bound on a freshness window (~2x slowest sensor)
_OFFSET_ALPHA: float = 0.1  # EMA weight when learning a backup's bias against the fresh primary


class _Channel:
    """One source's stream within a Parameter: a static rank and two slots (newest = t1/v1, previous
    = t0/v0) for linear extrapolation. Born STALE (t1 a full _DEFAULT_EXPIRE in the past, no data), so
    an un-pushed channel is never fresh and a fresh channel always has data. `offset` is the bias
    learned against the primary (None until learned), added on a reconciled fallback."""

    def __init__(self, source: str, rank: int):
        self.source: str = source
        self.rank: int = rank
        self.t0: int = time.ticks_add(time.ticks_us(), -(_DEFAULT_EXPIRE_US + 1))  # past any window: born stale
        self.v0 = None
        self.t1: int = self.t0
        self.v1 = None
        self.offset = None  # learned bias vs the primary, added when this is the reconciled fallback

    def push(self, value) -> None:
        """Record a reading now (timestamp taken here -- a value is delivered the moment it's known)."""
        ts = time.ticks_us()
        self.t0, self.v0 = self.t1, self.v1
        self.t1, self.v1 = ts, value

    def fresh(self, now: int, window_us: int) -> bool:
        """Fresh iff it pushed within `window_us` of now. Born stale + window <= _DEFAULT_EXPIRE means
        a fresh channel always has data -- no v1-None check needed here or downstream."""
        return time.ticks_diff(now, self.t1) <= window_us

    def value(self):
        """This channel's latest reading (None until first push). The handle a source reads back."""
        return self.v1


def _extrapolate(chan: _Channel, now: int):
    """Linear projection of a channel's two slots to `now`. With only one reading (v0 None), a
    non-scalar value (a stale vector), or a zero span, there is no linear model -- return the latest
    as-is (None if the channel never pushed). Explicit guards, NOT try/except: a caught exception
    allocates its object, and this runs every step for a stale channel with GC off (e.g. accel dropped
    -> airspeed integrator) -- the degraded path must not leak."""
    v1, v0 = chan.v1, chan.v0
    if v0 is None or not isinstance(v1, (int, float)):
        return v1  # one reading, or a vector (no scalar linear model) -> latest as-is, no alloc
    span = time.ticks_diff(chan.t1, chan.t0)
    if span == 0:
        return v1
    return v1 + (v1 - v0) * time.ticks_diff(now, chan.t1) / span  # scalar projection (boxes floats: inherent)


class Parameter:
    """One fused quantity. Holds a rank-ordered channel per source; value() fuses by rank/freshness,
    falling back to extrapolation of the primary when none is fresh."""

    def __init__(self, name: str):
        self.name: str = name
        self.channels: list = []  # _Channel per source, rank-ordered (channels[0] = primary)
        self.window_us: int = _DEFAULT_EXPIRE_US  # shared freshness window = primary tier's tightest expiry
        self._primary_rank: int = 1 << 30  # rank of the current primary tier (for window tracking)
        self.reconcile: bool = False  # offset reconciliation on? (a provider declares it)
        self._learned_t1 = None  # primary.t1 at the last offset update (learn once per new reading)
        self._read_buf: list = [None, None, None]  # reused (value, source, age_ms) -> no per-read() alloc

    def _channel(self, source: str) -> _Channel:
        for channel in self.channels:
            if channel.source == source:
                return channel
        return None

    def add_source(self, source: str, rank: int, expire_us: int, reconcile: bool = False) -> _Channel:
        """Register (or re-register) a source at `rank`; return its channel to push() to directly (no
        per-write lookup). Channels are KEPT IN RANK ORDER (channels[0] = primary). The shared
        freshness window is the tightest expiry of the primary (lowest-rank) tier, tracked here -- the
        only place expiry is consulted; runtime freshness uses window_us alone. `reconcile` (any
        provider) turns on offset reconciliation for this parameter."""
        channel = self._channel(source)
        if channel is None:
            channel = _Channel(source, rank)
            self.channels.append(channel)
        else:
            channel.rank = rank
        self.channels.sort(key=lambda candidate: candidate.rank)  # rank order: channels[0] = primary
        if reconcile:
            self.reconcile = True
        expire_us = expire_us or _DEFAULT_EXPIRE_US
        if rank < self._primary_rank:  # a new, lower-rank primary tier -> the window resets to it
            self._primary_rank = rank
            self.window_us = expire_us
        elif rank == self._primary_rank:  # another primary-tier source -> the tightest wins
            self.window_us = min(self.window_us, expire_us)
        return channel

    def write(self, value, source: str) -> None:
        """Report a source's latest reading by name (convenience; sensors push() their channel). The
        source is created on first write if it was not provided (rank 0, default window)."""
        try:
            self._channel(source).push(value)
        except AttributeError:
            self.add_source(source, 0, _DEFAULT_EXPIRE_US).push(value)

    def _winner(self, now: int):
        """The first fresh channel in rank order (lowest rank wins; same-rank channels are equivalent),
        or None. The primary is always channels[0], so value()/read() read it directly -- no (winner,
        primary) tuple is built on the hot path (a GC-off flight allocates nothing here)."""
        for channel in self.channels:
            if channel.fresh(now, self.window_us):
                return channel
        return None

    def _learn(self, now: int, primary: _Channel) -> None:
        """While the primary (channels[0]) is fresh it is truth; learn each fresh backup's bias
        against it (EMA) -- once per new primary reading, so the rate is set by data, not by reads.
        Co-primaries (same rank) are not reconciled. Offsets FREEZE while the primary is stale; they
        are applied on fallback in value()/read()."""
        if primary.t1 == self._learned_t1 or not primary.fresh(now, self.window_us):
            return
        self._learned_t1 = primary.t1
        reference = primary.v1
        for channel in self.channels[1:]:  # backups (rank-ordered, primary is channels[0])
            if channel.rank == primary.rank or not channel.fresh(now, self.window_us):
                continue  # skip co-primaries and stale backups
            try:
                sample = reference - channel.v1
                channel.offset = sample if channel.offset is None else (
                    channel.offset + _OFFSET_ALPHA * (sample - channel.offset))
            except TypeError:
                pass  # a non-scalar slipped into a reconciled param -> skip, never crash

    def value(self):
        """The fused estimate (offset-reconciled when enabled); None if nothing was ever written.
        Allocation-free on the hot path: returns the fresh winner's STORED value directly (or the
        primary extrapolated to now when nothing is fresh -- that fallback boxes floats, but it is the
        rare degraded case)."""
        if not self.channels:
            return None  # no sources registered yet (a consumer touched the param early)
        now = time.ticks_us()
        primary = self.channels[0]
        if self.reconcile:
            self._learn(now, primary)
        winner = self._winner(now)
        if winner is None:
            return _extrapolate(primary, now)
        if self.reconcile and winner is not primary and winner.offset is not None:
            return winner.v1 + winner.offset  # reconciled scalar -> a boxed float (altitude/pressure only)
        return winner.v1

    def read(self) -> list:
        """[value, source, age_ms] of the fused estimate; `source` is None when extrapolated, else the
        raw provider even for a reconciled (offset-corrected) value. Returns a REUSED 3-slot buffer
        (mutated each call), NOT a fresh tuple -- so a GC-off flight allocates nothing here. Safe because
        every caller unpacks it immediately (`a, b, c = param.read()`); do NOT retain the result across
        another read() of the same parameter (they alias one buffer)."""
        buf = self._read_buf
        if not self.channels:
            buf[0] = buf[1] = buf[2] = None
            return buf
        now = time.ticks_us()
        primary = self.channels[0]
        if self.reconcile:
            self._learn(now, primary)
        winner = self._winner(now)
        if winner is None:
            buf[0] = _extrapolate(primary, now)
            buf[1] = buf[2] = None
            return buf
        buf[0] = winner.v1 + winner.offset if (
            self.reconcile and winner is not primary and winner.offset is not None) else winner.v1
        buf[1] = winner.source
        buf[2] = time.ticks_diff(now, winner.t1) // 1000
        return buf

    def offsets(self) -> dict:
        """Learned bias per source (source -> offset) for diagnostics; empty until reconciled."""
        return {channel.source: channel.offset for channel in self.channels if channel.offset is not None}

    def raw(self, source: str):
        """A specific source's latest value (None if absent / unwritten)."""
        try:
            return self._channel(source).v1
        except AttributeError:
            return None

    def sources(self) -> list:
        return sorted(channel.source for channel in self.channels)


class Databoard:
    name: str = 'databoard'
    kind: str = 'databoard'
    _params: dict = {}  # name -> Parameter

    @classmethod
    def parameter(cls, *names):
        """Get-or-create read handle(s) for `names` -- the dependency accessor: a consumer grabs
        another sensor's Parameter regardless of setup order (created on first touch, reused after;
        registers with the Inspector on the very first one). One name returns the Parameter; several
        return a tuple in order."""
        handles = []
        for name in names:
            param = cls._params.get(name)
            if param is None:
                if not cls._params:
                    inspector.Inspector.register(cls)
                param = cls._params[name] = Parameter(name)
            handles.append(param)
        return handles[0] if len(handles) == 1 else tuple(handles)

    @classmethod
    def provide(cls, source: str, provides: dict, *want):
        """Register `source` for the params it provides ({param: {priority, timeout_ms[, reconcile]}})
        and hand back its write-channel(s), ready to push(): name the ones you `want` -- one name
        returns that channel, several return a tuple in that order, none returns the {param: channel}
        dict. So a driver writes `self._a, self._b = provide(name, provides, 'a', 'b')` in one line."""
        channels = {name: cls.parameter(name).add_source(source, spec.get('priority', 0),
                                                         spec.get('timeout_ms', 0) * 1000,
                                                         spec.get('reconcile', False))
                    for name, spec in provides.items()}
        if not want:
            return channels
        if len(want) == 1:
            return channels[want[0]]
        return tuple(channels[name] for name in want)

    @classmethod
    def write(cls, name: str, value, source: str) -> None:
        cls.parameter(name).write(value, source)

    @classmethod
    def value(cls, name: str):
        param = cls._params.get(name)
        return param.value() if param is not None else None

    @classmethod
    def read(cls, name: str) -> tuple:
        param = cls._params.get(name)
        return param.read() if param is not None else (None, None, None)

    @classmethod
    def raw(cls, name: str, source: str):
        param = cls._params.get(name)
        return param.raw(source) if param is not None else None

    # --- Inspectable (fused value per parameter) ---
    @classmethod
    def inspect(cls) -> dict:
        out = {}
        for name, param in cls._params.items():
            value, source, age_ms = param.read()
            out[name] = {'value': value, 'source': source, 'age_ms': age_ms, 'offsets': param.offsets()}
        return out

    @classmethod
    def stats(cls) -> dict:
        return {name: param.sources() for name, param in cls._params.items()}
