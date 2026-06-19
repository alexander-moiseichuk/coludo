# blackboard.py — the shared latest-value store for hot sensor data (specs/coludo.md "Task
# Data-Flow and Message Propagation"). Per-quantity slots hold the latest value + timestamp +
# source; sensor drivers write, the control loop / fusion read directly. Latest-wins, no fan-out,
# no subscriptions. Slots are declared up front so a steady-state write just updates fields in place
# (the value object itself is the producer's concern -- prefer reusing a buffer to stay allocation-
# free on the hot path). A global singleton like the Recorder, and Inspectable so the operator can
# watch live values (`inspect blackboard`).

import time

import inspector


class _Slot:
    """One quantity's latest reading: the value, when it was written, and by whom."""

    def __init__(self):
        self.value = None
        self.timestamp: int = 0  # time.ticks_us() of the last write
        self.source = None  # name of the sensor that wrote it


class Blackboard:
    name: str = 'blackboard'
    kind: str = 'blackboard'
    _slots: dict = {}  # quantity -> _Slot

    @classmethod
    def declare(cls, quantity: str) -> None:
        """Preallocate a quantity's slot (idempotent). Registers with the Inspector on first use."""
        if not cls._slots:
            inspector.Inspector.register(cls)
        if quantity not in cls._slots:
            cls._slots[quantity] = _Slot()

    @classmethod
    def write(cls, quantity: str, value, source: str = None) -> None:
        """Store the latest value for a quantity (latest-wins), updating the slot fields in place."""
        slot = cls._slots.get(quantity)
        if slot is None:
            cls.declare(quantity)
            slot = cls._slots[quantity]
        slot.value = value
        slot.timestamp = time.ticks_us()
        slot.source = source

    @classmethod
    def read(cls, quantity: str) -> _Slot:
        """The latest _Slot (value / timestamp / source) for a quantity, or None if never written."""
        return cls._slots.get(quantity)

    @classmethod
    def value(cls, quantity: str):
        """Just the latest value for a quantity (None if absent)."""
        slot = cls._slots.get(quantity)
        return slot.value if slot is not None else None

    # --- Inspectable ---
    @classmethod
    def inspect(cls) -> dict:
        now = time.ticks_us()
        return {
            quantity: {'value': slot.value, 'source': slot.source,
                       'age_ms': time.ticks_diff(now, slot.timestamp) // 1000}
            for quantity, slot in cls._slots.items()
        }

    @classmethod
    def stats(cls) -> dict:
        return {'quantities': sorted(cls._slots.keys())}
