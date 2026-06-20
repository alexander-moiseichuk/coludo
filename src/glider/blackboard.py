# blackboard.py — the shared latest-value store + sensor fusion for hot data (specs/coludo.md "Task
# Data-Flow and Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task
# with a registry of Parameter objects whose fused value is computed on read.
#
# Structure.
#   Blackboard   — a registry of Parameter objects. Blackboard.parameter(name) gets-or-creates one;
#                  a sensor registers itself as a source via provide() (which returns its channel
#                  handles) and then reports by pushing each channel directly -- the hot write path
#                  is one step, no lookup. value()/read() resolve the winner + primary in one pass.
#   Parameter    — one fused quantity (e.g. 'altitude') for the consumer. Holds a short LIST of
#                  channels KEPT IN RANK ORDER (lowest = primary first; a list, not a dict, is faster
#                  at this size), plus the shared freshness window derived from its primary tier.
#   _Channel     — one source's stream: a static rank (priority; lower = preferred), a declared
#                  expiry (the parameter applies one shared window), and TWO slots (the last two
#                  readings) -- two slots because the extrapolation here is LINEAR (needs 2 points).
#
# Fusion is a pure read-time function, Parameter.value():
#   1. winner = the lowest-rank channel still fresh. Channels are rank-ordered, so it is just the
#      FIRST fresh one in the list (same-rank channels are equivalent). Freshness uses ONE shared
#      window per parameter: the tightest expiry among the rank-0 tier (min() if two share rank 0),
#      applied to EVERY channel. Return its value.
#   2. if NO channel is fresh, linearly extrapolate the PRIMARY's two slots to now -- project the
#      trusted source forward rather than hand out a backup that is itself stale (and bias-shifted).
#   3. if nothing was ever written, None.
# So "rank 0 answers while fresh; a backup takes over only while itself THIS fresh, else rank 0 is
# extrapolated" is EMERGENT -- every read re-evaluates freshness against the shared window; there is
# no queue to sort, insert into, or evict. A channel with no data (v1 None) is never fresh.
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
# Inspectable as `blackboard` (fused value/source/age per parameter).

import time

import inspector

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_DEFAULT_EXPIRE_US: int = const(1000_000)  # fallback freshness window when a source declares none
_OFFSET_ALPHA: float = 0.1  # EMA weight when learning a backup's bias against the fresh primary


class _Channel:
    """One source's stream within a Parameter: a static rank, a declared expiry (the parameter
    applies one shared window, min() over the rank-0 tier), and two slots (newest = t1/v1, previous =
    t0/v0) for linear extrapolation. No data (v1 None) reads as not-fresh. `offset` is the bias
    learned against the primary (None until learned), added on a reconciled fallback."""

    def __init__(self, source: str, rank: int, expire_us: int):
        self.source: str = source
        self.rank: int = rank
        self.expire_us: int = expire_us or _DEFAULT_EXPIRE_US  # only the rank-0 tier's min is used
        self.t0: int = time.ticks_add(time.ticks_us(), -self.expire_us)  # in the past: born stale
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
        """Fresh iff it holds a reading pushed within `window_us` of now (no data -> never fresh)."""
        return self.v1 is not None and time.ticks_diff(now, self.t1) <= window_us

    def value(self):
        """This channel's latest reading (None until first push). The handle a source reads back."""
        return self.v1


def _extrapolate(channel: _Channel, now: int):
    """Linear projection of a channel's two slots to `now` -- value is a scalar (int|float). With
    only one reading, or a non-scalar value (a stale vector), return the latest as-is."""
    older, newest = channel.v0, channel.v1
    span = time.ticks_diff(channel.t1, channel.t0)
    if older is None or span <= 0:
        return newest
    try:
        return newest + (newest - older) * time.ticks_diff(now, channel.t1) / span
    except TypeError:  # vector etc. -> latest, no projection
        return newest


class Parameter:
    """One fused quantity. Holds a channel per source; value() fuses by rank/freshness, falling back
    to extrapolation when none is fresh."""

    def __init__(self, name: str):
        self.name: str = name
        self.channels: list = []  # _Channel per source (few; a list beats a dict at this size)
        self.window_us: int = _DEFAULT_EXPIRE_US  # shared freshness window = primary tier's min expiry
        self.reconcile: bool = False  # offset reconciliation on? (a provider declares it)
        self._learned_t1 = None  # primary.t1 at the last offset update (learn once per new reading)

    def _channel(self, source: str) -> _Channel:
        for channel in self.channels:
            if channel.source == source:
                return channel
        return None

    def add_source(self, source: str, rank: int, expire_us: int, reconcile: bool = False) -> _Channel:
        """Register (or re-register) a source with its rank + expiry; return its channel so the sensor
        can push() to it directly (no per-write lookup). Channels are KEPT IN RANK ORDER (channels[0]
        = primary), so resolve is a single forward scan. Recomputes the shared freshness window: the
        tightest expiry among the rank-0 tier. `reconcile` (any provider) turns it on here."""
        channel = self._channel(source)
        if channel is None:
            channel = _Channel(source, rank, expire_us)
            self.channels.append(channel)
        else:
            channel.rank, channel.expire_us = rank, expire_us or _DEFAULT_EXPIRE_US
        if reconcile:
            self.reconcile = True
        self.channels.sort(key=lambda candidate: candidate.rank)  # rank order: channels[0] = primary
        primary_rank = self.channels[0].rank
        self.window_us = min(candidate.expire_us for candidate in self.channels
                             if candidate.rank == primary_rank)
        return channel

    def write(self, value, source: str) -> None:
        """Report a source's latest reading by name (convenience; sensors push() their channel)."""
        channel = self._channel(source)
        if channel is None:
            channel = self.add_source(source, 0, _DEFAULT_EXPIRE_US)
        channel.push(value)

    def _resolve(self, now: int) -> tuple:
        """One forward scan over the rank-ordered channels: (winner, primary). winner = the first
        channel fresh within the shared window (lowest rank wins; same-rank channels are equivalent,
        so the first is taken); primary = the first channel holding data -- extrapolated when nothing
        is fresh."""
        winner = None
        primary = None
        for channel in self.channels:
            if primary is None and channel.v1 is not None:
                primary = channel
            if winner is None and channel.fresh(now, self.window_us):
                winner = channel
            if winner is not None and primary is not None:
                break
        return winner, primary

    def _learn(self, now: int, primary: _Channel) -> None:
        """While the primary (lowest-rank source with data) is fresh, learn each fresh backup's bias
        against it (EMA) -- once per new primary reading, so the rate is set by data, not by reads.
        Co-primaries (same rank) are not reconciled. Offsets FREEZE while the primary is stale; they
        are applied on fallback in _estimate()."""
        if primary is None or primary.t1 == self._learned_t1 or not primary.fresh(now, self.window_us):
            return
        self._learned_t1 = primary.t1
        reference = primary.v1
        for channel in self.channels:
            if channel.rank <= primary.rank or channel.v1 is None or not channel.fresh(now, self.window_us):
                continue  # skip the primary and its co-primaries; only lower-priority backups learn
            try:
                sample = reference - channel.v1
            except TypeError:
                continue  # a non-scalar in a reconciled param -> skip, never crash
            channel.offset = sample if channel.offset is None else (
                channel.offset + _OFFSET_ALPHA * (sample - channel.offset))

    def _estimate(self, now: int) -> tuple:
        """(value, winner) of the fused estimate. winner is None when nothing is fresh (the value is
        then the primary extrapolated, or None). With reconciliation on, a fresh non-primary winner
        is bias-corrected by its learned offset so the handover off the primary is seamless."""
        winner, primary = self._resolve(now)
        if self.reconcile:
            self._learn(now, primary)
        if winner is None:
            return (_extrapolate(primary, now) if primary is not None else None, None)
        if self.reconcile and winner is not primary and winner.offset is not None:
            return (winner.v1 + winner.offset, winner)
        return (winner.v1, winner)

    def value(self):
        """The fused estimate (offset-reconciled when enabled); None if nothing was ever written."""
        return self._estimate(time.ticks_us())[0]

    def read(self) -> tuple:
        """(value, source, age_ms) of the fused estimate; `source` is None when extrapolated. A
        reconciled value is offset-corrected, but `source` still names the raw provider it came from."""
        now = time.ticks_us()
        value, winner = self._estimate(now)
        if winner is None:
            return (value, None, None)
        return (value, winner.source, time.ticks_diff(now, winner.t1) // 1000)

    def offsets(self) -> dict:
        """Learned bias per source (source -> offset) for diagnostics; empty until reconciled."""
        return {channel.source: channel.offset for channel in self.channels if channel.offset is not None}

    def raw(self, source: str):
        """A specific source's latest value (None if absent / unwritten)."""
        channel = self._channel(source)
        return channel.v1 if channel is not None else None

    def sources(self) -> list:
        return sorted(channel.source for channel in self.channels)


class Blackboard:
    name: str = 'blackboard'
    kind: str = 'blackboard'
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
