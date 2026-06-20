# blackboard.py — the shared latest-value store + sensor fusion for hot data (specs/coludo.md "Task
# Data-Flow and Message Propagation"). Replaces a two-layer raw/fused store + a polling fusion task
# with a registry of Parameter objects whose fused value is computed on read.
#
# Structure.
#   Blackboard   — a registry of Parameter objects. Blackboard.parameter(name) gets-or-creates one;
#                  a sensor registers itself as a source via provide() and reports via write().
#   Parameter    — one fused quantity (e.g. 'altitude'). Owns a _Channel per source.
#   _Channel     — one source's stream: a static rank (priority; lower = preferred), an expiry
#                  window, and TWO slots (the last two readings) + a deadline. write() updates the
#                  channel's value + timestamp on the fly; the deadline = ts + expire is recomputed.
#
# Fusion is a pure read-time function, Parameter.value():
#   1. winner = the lowest-rank channel that is still fresh (freshness is evaluated per channel,
#      now <= its deadline); a rank tie breaks to the newer reading. Return its value.
#   2. if NO channel is fresh, take the newest channel and linearly extrapolate its two slots to
#      `now` (component-wise for vector values).
#   3. if nothing was ever written, None.
# So "rank 0 answers while fresh; rank 1 takes over the instant rank 0 expires" is EMERGENT -- every
# read re-evaluates each channel's freshness; there is no queue to sort, insert into, or evict.
#
# Bounded by design. A parameter with M ranked sources holds exactly M channels. Each channel keeps
# TWO slots because the extrapolation here is LINEAR (needs 2 points); the slot depth is a function
# of the model order -- degree-N polynomial would need N+1 readings (linear=2, quadratic=3, ...). A
# future model registry could carry that, e.g. {'linear': (fn, 2), 'quadratic': (fn, 3)}, and size
# the channel from it; for now it is fixed at 2 (linear). Storage is fixed when sources register --
# nothing grows per sample. (A rank may in theory have several sources -- just channels with the
# same rank, tiebroken by time -- though in practice each rank is one device.) Per-source slots keep
# extrapolation within a single source, never across the bias gap between, say, ICP-10111 and BMP280.
#
# Telemetry is separate: each sensor writes its own raw SENSOR.csv directly. A global singleton,
# Inspectable as `blackboard` (fused value/source/age per parameter).

import time

import inspector

_DEFAULT_EXPIRE_US = 1000000  # fallback freshness window for a channel written without provide()


class _Channel:
    """One source's stream within a Parameter: a static rank, an expiry, and the last two readings
    (newest = t1/v1, previous = t0/v0) -- two slots for LINEAR extrapolation (a degree-N model would
    keep N+1)."""

    def __init__(self, source: str, rank: int, expire_us: int):
        self.source: str = source
        self.rank: int = rank
        self.expire_us: int = expire_us
        self.t0: int = 0
        self.v0 = None
        self.t1: int = 0
        self.v1 = None
        self.deadline: int = 0  # t1 + expire_us; fresh while now <= deadline
        self.has_data: bool = False

    def push(self, ts: int, value) -> None:
        self.t0, self.v0 = self.t1, self.v1
        self.t1, self.v1 = ts, value
        self.deadline = time.ticks_add(ts, self.expire_us)
        self.has_data = True

    def fresh(self, now: int) -> bool:
        return self.has_data and time.ticks_diff(self.deadline, now) >= 0


def _extrapolate(channel: _Channel, now: int):
    """Linearly project a channel's two slots to `now` (component-wise for tuples); falls back to
    the latest value if there is only one reading. LINEAR (2-point) model -- a higher-order fit
    (quadratic = 3 points, etc.) would read deeper history and pair with a wider channel; if that is
    ever needed, swap this for a model from a {'linear': (fn, 2), 'quadratic': (fn, 3)} registry."""
    if channel.v0 is None or channel.t1 == channel.t0:
        return channel.v1
    span = time.ticks_diff(channel.t1, channel.t0)
    ahead = time.ticks_diff(now, channel.t1)

    def project(old, new):
        return new + (new - old) * ahead / span

    if isinstance(channel.v1, (tuple, list)):
        return tuple(project(o, n) for o, n in zip(channel.v0, channel.v1))
    return project(channel.v0, channel.v1)


class Parameter:
    """One fused quantity. Holds a channel per source; value() fuses them by rank/freshness, falling
    back to extrapolation when none is fresh."""

    def __init__(self, name: str):
        self.name: str = name
        self.channels: dict = {}  # source -> _Channel

    def add_source(self, source: str, rank: int, expire_us: int) -> None:
        """Register (or re-register) a source for this parameter with its rank + expiry."""
        self.channels[source] = _Channel(source, rank, expire_us)

    def write(self, value, source: str) -> None:
        """Report a source's latest reading (updates its channel's value + timestamp)."""
        channel = self.channels.get(source)
        if channel is None:
            channel = self.channels[source] = _Channel(source, 0, _DEFAULT_EXPIRE_US)
        channel.push(time.ticks_us(), value)

    def _winner(self, now: int) -> _Channel:
        """The lowest-rank fresh channel (newest reading breaks a rank tie), or None."""
        best = None
        for channel in self.channels.values():
            if not channel.fresh(now):
                continue
            if best is None or channel.rank < best.rank or (
                    channel.rank == best.rank and time.ticks_diff(channel.t1, best.t1) > 0):
                best = channel
        return best

    def value(self):
        """The fused estimate: the preferred fresh source's value; if none is fresh, the newest
        source extrapolated to now; else None."""
        now = time.ticks_us()
        winner = self._winner(now)
        if winner is not None:
            return winner.v1
        newest = None
        for channel in self.channels.values():
            if channel.has_data and (newest is None or time.ticks_diff(channel.t1, newest.t1) > 0):
                newest = channel
        return _extrapolate(newest, now) if newest is not None else None

    def read(self) -> tuple:
        """(value, source, age_ms) of the fused estimate; `source` is None when extrapolated."""
        now = time.ticks_us()
        winner = self._winner(now)
        if winner is not None:
            return (winner.v1, winner.source, time.ticks_diff(now, winner.t1) // 1000)
        return (self.value(), None, None)

    def raw(self, source: str):
        """A specific source's latest value (None if absent / unwritten)."""
        channel = self.channels.get(source)
        return channel.v1 if (channel is not None and channel.has_data) else None

    def sources(self) -> list:
        return sorted(self.channels.keys())


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
    def provide(cls, source: str, provides: dict) -> None:
        """A sensor registers the params it provides: {param: {priority, timeout_ms}}."""
        for name, spec in provides.items():
            cls.parameter(name).add_source(source, spec.get('priority', 0), spec.get('timeout_ms', 1000) * 1000)

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
