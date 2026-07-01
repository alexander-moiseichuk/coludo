# servo.py — shared servo infrastructure, sibling of the bus helpers (i2cbus/spibus). The slew gate
# bounds how many fins slew at once (the boost-rail current transient): a process-wide counting
# semaphore so `servo_concurrency` (board config) caps total simultaneous slews across every servo
# driver. Servo-type-agnostic -- each driver (sg90, future mg90s/mg996r) imports the gate and adds its
# own pulse range + slew timing.

import asyncio


class Gate:
    """A tiny FIFO counting semaphore (MicroPython asyncio has no Semaphore, only Lock/Event): at most
    `permits` holders at once, the rest queue and are handed a permit in order on release. The
    process-wide shared instance lives on the class itself (Gate.slew()/Gate.reset()) -- no module
    global."""

    _shared: 'Gate' = None  # the process-wide slew gate, created on the first Gate.slew()

    def __init__(self, permits: int):
        self._free: int = permits
        self._waiters: list = []
        self._pool: list = []  # spent Events, reused -> no per-acquire alloc while the gate is contended

    async def acquire(self) -> None:
        if self._free > 0:
            self._free -= 1
            return
        event = self._pool.pop() if self._pool else asyncio.Event()  # reuse a spent Event if we have one
        event.clear()
        self._waiters.append(event)
        try:
            await event.wait()  # release() hands this waiter the permit directly; _free unchanged
        except:  # cancelled: clean up WITHOUT leaking a permit
            if event in self._waiters:
                self._waiters.remove(event)  # still queued -- we never held a permit
            else:
                self.release()  # release() already handed us the permit -- pass it on, do not lose it
            raise
        finally:
            self._pool.append(event)  # return it for reuse (single-threaded: never live in two places)

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

    @classmethod
    def slew(cls, permits: int) -> 'Gate':
        """The process-wide slew gate, created once (the first servo's `permits` wins) and shared by
        every servo driver, so `servo_concurrency` bounds simultaneous slews board-wide rather than
        per driver."""
        if cls._shared is None:
            cls._shared = cls(permits)
        return cls._shared

    @classmethod
    def reset(cls) -> None:
        """Drop the shared gate so the next Gate.slew() rebuilds it -- for tests (clean permit count)
        and a full reconfigure."""
        cls._shared = None
