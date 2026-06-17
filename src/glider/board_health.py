# BoardHealth — periodic device vitals (temperature, free memory, CPU load) pushed to telemetry
# and exposed to the operator via the Inspector (findings.txt #10). CPU load is estimated from a
# low-priority idle task: the fewer times it runs in a period (vs the most it has ever run), the
# busier the board.

import asyncio
import gc
import time

from inspector import Inspectable, Inspector
from recorder import Telemetry

try:
    import esp32
except ImportError:
    esp32 = None


class BoardHealth(Inspectable):
    name = 'health'
    kind = 'health'

    def __init__(self, period_ms: int = 1000):
        self.period_ms: int = period_ms
        self.load: float = 0.0
        self._idle_count: int = 0
        self._max_rate: float = 0.0
        self._tlm = Telemetry('health.csv', ('temp', 'mem_free', 'load'))
        Inspector.register(self)

    def temperature(self):
        if esp32 is not None:
            try:
                return esp32.mcu_temperature()
            except Exception:
                return None
        return None

    def mem_free(self) -> int:
        return gc.mem_free()

    def sample(self) -> dict:
        return {'temp': self.temperature(), 'mem_free': self.mem_free(), 'load': round(self.load, 3)}

    async def _idle_loop(self) -> None:
        while True:
            self._idle_count += 1
            await asyncio.sleep_ms(0)

    async def run(self) -> None:
        """Sample vitals every period_ms, estimate load, and push a telemetry row. Runs forever."""
        asyncio.create_task(self._idle_loop())
        last_count = self._idle_count
        last_ms = time.ticks_ms()
        while True:
            await asyncio.sleep_ms(self.period_ms)
            now_count = self._idle_count
            now_ms = time.ticks_ms()
            elapsed = time.ticks_diff(now_ms, last_ms)
            rate = (now_count - last_count) / elapsed if elapsed else 0.0
            last_count, last_ms = now_count, now_ms
            if rate > self._max_rate:
                self._max_rate = rate
            self.load = max(0.0, 1.0 - rate / self._max_rate) if self._max_rate else 0.0
            vitals = self.sample()
            self._tlm.push((vitals['temp'], vitals['mem_free'], vitals['load']))

    # --- Inspectable ---
    def inspect(self) -> dict:
        return self.sample()

    def stats(self) -> dict:
        return self.sample()
