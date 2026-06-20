# blackboard.py — the shared latest-value store + sensor fusion for hot data (specs/coludo.md "Task
# Data-Flow and Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task
# with a registry of Parameter objects whose fused value is computed on read.
#
# Structure.
#   Blackboard   — a registry of Parameter objects. Blackboard.parameter(name) gets-or-creates one;
#                  a sensor registers itself as a source via provide() (which returns its channel
#                  handles) and then reports by pushing each channel directly -- the hot write path
#                  is one step, no lookup. value()/read() resolve the winner + newest in one pass.
#   Parameter    — one fused quantity (e.g. 'altitude') for the consumer. Holds a short LIST of
#                  channels (one per source -- 1..few; a list, not a dict, is faster at this size).
#   _Channel     — one source's stream: a static rank (priority; lower = preferred), an expiry, and
#                  TWO slots (the last two readings) -- two slots because the extrapolation here is
#                  LINEAR (needs 2 points); a degree-N model would keep N+1.
#
# Fusion is a pure read-time function, Parameter.value():
#   1. winner = the lowest-rank channel still fresh (freshness is per channel: now <= its deadline,
#      where deadline = last-write + expire); a rank tie breaks to the newer reading. Return its value.
#   2. if NO channel is fresh, take the newest channel and linearly extrapolate its two slots to now.
#   3. if nothing was ever written, None.
# So "rank 0 answers while fresh; rank 1 takes over the instant rank 0 expires" is EMERGENT -- every
# read re-evaluates each channel's freshness; there is no queue to sort, insert into, or evict. A
# channel is BORN EXPIRED (t1 = now - expire), so "no data yet" needs no flag -- it is just expired.
#
# Bounded by design. A parameter with M ranked sources holds exactly M channels, each with two
# slots; storage is fixed when sources register, nothing grows per sample. Per-source slots keep
# extrapolation within a single source, never across the bias gap between e.g. ICP-10111 and BMP280.
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


class _Channel:
    """One source's stream within a Parameter: a static rank, an expiry, and two slots (newest =
    t1/v1, previous = t0/v0) for linear extrapolation. Born expired so 'no data yet' is just stale."""

    def __init__(self, source: str, rank: int, expire_us: int):
        self.source: str = source
        self.rank: int = rank
        self.expire_us: int = expire_us or _DEFAULT_EXPIRE_US
        self.t0: int = time.ticks_add(time.ticks_us(), -self.expire_us)  # in the past: born expired
        self.v0 = None
        self.t1: int = self.t0
        self.v1 = None
        self.deadline: int = time.ticks_add(self.t1, self.expire_us)  # fresh while now <= deadline

    def push(self, value) -> None:
        """Record a reading now (timestamp taken here -- a value is delivered the moment it's known)."""
        ts = time.ticks_us()
        self.t0, self.v0 = self.t1, self.v1
        self.t1, self.v1 = ts, value
        self.deadline = time.ticks_add(ts, self.expire_us)

    def fresh(self, now: int) -> bool:
        return time.ticks_diff(self.deadline, now) >= 0


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

    def _channel(self, source: str) -> _Channel:
        for channel in self.channels:
            if channel.source == source:
                return channel
        return None

    def add_source(self, source: str, rank: int, expire_us: int) -> _Channel:
        """Register (or re-register) a source with its rank + expiry; return its channel so the
        sensor can push() to it directly (no per-write lookup)."""
        channel = self._channel(source)
        if channel is None:
            channel = _Channel(source, rank, expire_us)
            self.channels.append(channel)
        else:
            channel.rank, channel.expire_us = rank, expire_us or _DEFAULT_EXPIRE_US
        return channel

    def write(self, value, source: str) -> None:
        """Report a source's latest reading by name (convenience; sensors push() their channel)."""
        channel = self._channel(source)
        if channel is None:
            channel = self.add_source(source, 0, _DEFAULT_EXPIRE_US)
        channel.push(value)

    def _resolve(self, now: int) -> tuple:
        """One pass over the channels: (winner, newest). winner = lowest-rank fresh channel (newest
        reading breaks a rank tie) or None; newest = the channel with the most recent reading."""
        winner = None
        newest = None
        for channel in self.channels:
            if newest is None or time.ticks_diff(channel.t1, newest.t1) > 0:
                newest = channel
            if channel.fresh(now) and (winner is None or channel.rank < winner.rank or (
                    channel.rank == winner.rank and time.ticks_diff(channel.t1, winner.t1) > 0)):
                winner = channel
        return winner, newest

    def value(self):
        """The fused estimate: the preferred fresh source's value; if none is fresh, the newest
        source extrapolated to now; else None."""
        now = time.ticks_us()
        winner, newest = self._resolve(now)
        if winner is not None:
            return winner.v1
        return _extrapolate(newest, now) if newest is not None else None

    def read(self) -> tuple:
        """(value, source, age_ms) of the fused estimate; `source` is None when extrapolated."""
        now = time.ticks_us()
        winner, newest = self._resolve(now)
        if winner is not None:
            return (winner.v1, winner.source, time.ticks_diff(now, winner.t1) // 1000)
        return (_extrapolate(newest, now) if newest is not None else None, None, None)

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
    def parameter(cls, name: str) -> Parameter:
        """Get-or-create the Parameter for `name` (registers with the Inspector on the first one)."""
        param = cls._params.get(name)
        if param is None:
            if not cls._params:
                inspector.Inspector.register(cls)
            param = cls._params[name] = Parameter(name)
        return param

    @classmethod
    def provide(cls, source: str, provides: dict) -> dict:
        """A sensor registers the params it provides ({param: {priority, timeout_ms}}) and gets back
        {param: _Channel} so it can push() its readings directly -- one step, no per-write lookup."""
        return {name: cls.parameter(name).add_source(source, spec.get('priority', 0),
                                                     spec.get('timeout_ms', 0) * 1000)
                for name, spec in provides.items()}

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
            out[name] = {'value': value, 'source': source, 'age_ms': age_ms}
        return out

    @classmethod
    def stats(cls) -> dict:
        return {name: param.sources() for name, param in cls._params.items()}
