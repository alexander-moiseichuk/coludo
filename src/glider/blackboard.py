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
#                  channels (one per source -- 1..few; a list, not a dict, is faster at this size),
#                  plus the shared freshness window derived from its primary tier.
#   _Channel     — one source's stream: a static rank (priority; lower = preferred), a declared
#                  expiry (the parameter applies one shared window), and TWO slots (the last two
#                  readings) -- two slots because the extrapolation here is LINEAR (needs 2 points).
#
# Fusion is a pure read-time function, Parameter.value():
#   1. winner = the lowest-rank channel still fresh. Freshness uses ONE shared window per parameter:
#      the tightest expiry among the primary tier (the lowest-rank sources -- min() if two share
#      rank 0), applied to EVERY channel. A rank tie breaks to the newer reading. Return its value.
#   2. if NO channel is fresh, linearly extrapolate the PRIMARY's two slots to now -- project the
#      trusted source forward rather than hand out a backup that is itself stale (and bias-shifted).
#   3. if nothing was ever written, None.
# So "rank 0 answers while fresh; a backup takes over only while itself THIS fresh, else rank 0 is
# extrapolated" is EMERGENT -- every read re-evaluates freshness against the shared window; there is
# no queue to sort, insert into, or evict. A channel with no data (v1 None) is never fresh.
#
# This shared window is the simple stand-in for offset reconciliation: instead of learning and
# subtracting each backup's bias, we only hand a backup out while it is genuinely current; otherwise
# the primary's own trajectory is projected. Per-source slots keep extrapolation within a single
# source, never across the bias gap between e.g. ICP-10111 and BMP280.
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


class _Channel:
    """One source's stream within a Parameter: a static rank, a declared expiry (the parameter
    applies one shared window, min() over the rank-0 tier), and two slots (newest = t1/v1, previous =
    t0/v0) for linear extrapolation. No data (v1 None) reads as not-fresh."""

    def __init__(self, source: str, rank: int, expire_us: int):
        self.source: str = source
        self.rank: int = rank
        self.expire_us: int = expire_us or _DEFAULT_EXPIRE_US  # only the rank-0 tier's min is used
        self.t0: int = time.ticks_add(time.ticks_us(), -self.expire_us)  # in the past: born stale
        self.v0 = None
        self.t1: int = self.t0
        self.v1 = None

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

    def _channel(self, source: str) -> _Channel:
        for channel in self.channels:
            if channel.source == source:
                return channel
        return None

    def add_source(self, source: str, rank: int, expire_us: int) -> _Channel:
        """Register (or re-register) a source with its rank + expiry; return its channel so the sensor
        can push() to it directly (no per-write lookup). Recomputes the shared freshness window: the
        tightest expiry among the primary tier (lowest-rank sources), used for every channel."""
        channel = self._channel(source)
        if channel is None:
            channel = _Channel(source, rank, expire_us)
            self.channels.append(channel)
        else:
            channel.rank, channel.expire_us = rank, expire_us or _DEFAULT_EXPIRE_US
        primary_rank = min(candidate.rank for candidate in self.channels)
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
        """One pass over the channels: (winner, primary). winner = lowest-rank channel fresh within
        the shared window (a newer reading breaks a rank tie) or None; primary = the lowest-rank
        channel that holds data -- the source extrapolated when nothing is fresh."""
        winner = None
        primary = None
        for channel in self.channels:
            if channel.v1 is not None and (primary is None or channel.rank < primary.rank or (
                    channel.rank == primary.rank and time.ticks_diff(channel.t1, primary.t1) > 0)):
                primary = channel
            if channel.fresh(now, self.window_us) and (winner is None or channel.rank < winner.rank or (
                    channel.rank == winner.rank and time.ticks_diff(channel.t1, winner.t1) > 0)):
                winner = channel
        return winner, primary

    def value(self):
        """The fused estimate: the preferred fresh source's value; if none is fresh, the primary
        source extrapolated to now; else None."""
        now = time.ticks_us()
        winner, primary = self._resolve(now)
        if winner is not None:
            return winner.v1
        return _extrapolate(primary, now) if primary is not None else None

    def read(self) -> tuple:
        """(value, source, age_ms) of the fused estimate; `source` is None when extrapolated."""
        now = time.ticks_us()
        winner, primary = self._resolve(now)
        if winner is not None:
            return (winner.v1, winner.source, time.ticks_diff(now, winner.t1) // 1000)
        return (_extrapolate(primary, now) if primary is not None else None, None, None)

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
        """Register `source` for the params it provides ({param: {priority, timeout_ms}}) and hand
        back its write-channel(s), ready to push(): name the ones you `want` -- one name returns that
        channel, several return a tuple in that order, none returns the {param: channel} dict. So a
        driver writes `self._a, self._b = provide(name, provides, 'a', 'b')` in one line."""
        channels = {name: cls.parameter(name).add_source(source, spec.get('priority', 0),
                                                         spec.get('timeout_ms', 0) * 1000)
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
            out[name] = {'value': value, 'source': source, 'age_ms': age_ms}
        return out

    @classmethod
    def stats(cls) -> dict:
        return {name: param.sources() for name, param in cls._params.items()}
