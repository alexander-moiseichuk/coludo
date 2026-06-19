# blackboard.py — the shared latest-value store for hot sensor data (specs/coludo.md "Task
# Data-Flow"). Two layers:
#   raw   — each sensor's latest reading, kept per (quantity, source) so several providers of the
#           same quantity (e.g. altitude from the ICP-10111 and the BMP280) coexist.
#   fused — the one selected value per quantity that consumers read; the fusion task picks it from
#           the raw providers by priority + freshness (tasks/fusion.py) and publishes it here.
# Sensors call write(); fusion calls providers()/publish(); consumers call value()/read(). Slots are
# updated in place (allocation-free on the hot path). A global singleton, Inspectable as
# `blackboard` (shows the fused values).

import time

import inspector


class _Slot:
    """One reading: the value, when it was written (ticks_us), and its source sensor."""

    def __init__(self):
        self.value = None
        self.timestamp: int = 0
        self.source = None


class Blackboard:
    name: str = 'blackboard'
    kind: str = 'blackboard'
    _raw: dict = {}  # quantity -> { source -> _Slot }
    _fused: dict = {}  # quantity -> _Slot

    @classmethod
    def declare(cls, quantity: str) -> None:
        """Preallocate a quantity's raw map + fused slot (idempotent). Registers on first use."""
        if not cls._raw and not cls._fused:
            inspector.Inspector.register(cls)
        cls._raw.setdefault(quantity, {})
        if quantity not in cls._fused:
            cls._fused[quantity] = _Slot()

    # -- raw layer (sensors write, fusion reads) -------------------------------
    @classmethod
    def write(cls, quantity: str, value, source: str) -> None:
        """A sensor publishes its latest raw reading for `quantity` (latest-wins per source)."""
        sources = cls._raw.get(quantity)
        if sources is None:
            cls.declare(quantity)
            sources = cls._raw[quantity]
        slot = sources.get(source)
        if slot is None:
            slot = sources[source] = _Slot()
        slot.value = value
        slot.timestamp = time.ticks_us()
        slot.source = source

    @classmethod
    def raw(cls, quantity: str, source: str) -> _Slot:
        """A specific sensor's latest raw reading for `quantity` (None if it never wrote)."""
        return cls._raw.get(quantity, {}).get(source)

    @classmethod
    def providers(cls, quantity: str) -> dict:
        """All raw readings for `quantity` as { source -> _Slot } (for the fusion task)."""
        return cls._raw.get(quantity, {})

    # -- fused layer (fusion publishes, consumers read) ------------------------
    @classmethod
    def publish(cls, quantity: str, value, source: str) -> None:
        """Fusion sets the selected value for `quantity` (`source` = which provider won)."""
        slot = cls._fused.get(quantity)
        if slot is None:
            cls.declare(quantity)
            slot = cls._fused[quantity]
        slot.value = value
        slot.timestamp = time.ticks_us()
        slot.source = source

    @classmethod
    def read(cls, quantity: str) -> _Slot:
        """The fused _Slot (value/timestamp/source) for `quantity` — what consumers read."""
        return cls._fused.get(quantity)

    @classmethod
    def value(cls, quantity: str):
        """Just the fused value for `quantity` (None if nothing fused yet)."""
        slot = cls._fused.get(quantity)
        return slot.value if slot is not None else None

    # --- Inspectable (shows the fused values) ---------------------------------
    @classmethod
    def inspect(cls) -> dict:
        now = time.ticks_us()
        return {
            quantity: {'value': slot.value, 'source': slot.source,
                       'age_ms': time.ticks_diff(now, slot.timestamp) // 1000}
            for quantity, slot in cls._fused.items()
        }

    @classmethod
    def stats(cls) -> dict:
        return {quantity: sorted(sources.keys()) for quantity, sources in cls._raw.items()}
