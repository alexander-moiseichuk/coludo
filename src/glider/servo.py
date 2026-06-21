# servo.py — shared servo infrastructure, sibling of the bus helpers (i2cbus/spibus). The slew gate
# bounds how many fins slew at once (the boost-rail current transient): a process-wide counting
# semaphore so `servo_concurrency` (board config) caps total simultaneous slews across every servo
# driver. Servo-type-agnostic -- each driver (sg90, future mg90s/mg996r) imports the gate and adds its
# own pulse range + slew timing.

import asyncio

_gate = None  # the process-wide slew gate, created on the first servo setup


class Gate:
    """A tiny FIFO counting semaphore (MicroPython asyncio has no Semaphore, only Lock/Event): at most
    `permits` holders at once, the rest queue and are handed a permit in order on release."""

    def __init__(self, permits: int):
        self._free: int = permits
        self._waiters: list = []

    async def acquire(self) -> None:
        if self._free > 0:
            self._free -= 1
        else:
            event = asyncio.Event()
            self._waiters.append(event)
            await event.wait()  # release() hands this waiter the permit directly; _free unchanged

    def release(self) -> None:
        if self._waiters:
            self._waiters.pop(0).set()  # direct hand-off to the next in line (FIFO)
        else:
            self._free += 1

    async def __aenter__(self):
        await self.acquire()
        return self

    async def __aexit__(self, *exception):
        self.release()


def slew_gate(permits: int) -> Gate:
    """The process-wide slew gate, created once (the first servo's `permits` wins) and shared by every
    servo driver, so `servo_concurrency` bounds simultaneous slews board-wide rather than per driver."""
    global _gate
    if _gate is None:
        _gate = Gate(permits)
    return _gate


def reset_gate() -> None:
    """Drop the shared gate so the next slew_gate() rebuilds it -- for tests (clean permit count) and
    a full reconfigure."""
    global _gate
    _gate = None
