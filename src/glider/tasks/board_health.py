# tasks/board_health.py — board vitals task: samples temperature, free memory and CPU load every
# period, pushes a telemetry row, and exposes the latest to the operator. CPU load is estimated from
# a low-priority idle task: the fewer times it runs in a period (vs the most it ever has), the busier
# the board. Registered as @task.activity('health') so the Controller creates and supervises it.

import asyncio
import gc
import time

import recorder
import task

try:
    import esp32
except ImportError:
    esp32 = None


@task.activity('health')
class BoardHealth(task.Task):
    """Periodic vitals -> telemetry (health.csv) + `inspect health`."""

    async def setup(self) -> bool:
        self.period_ms: int = self.config.get('period_ms', 1000)
        self.load: float = 0.0
        self._idle_count: int = 0
        self._max_rate: float = 0.0
        self._tlm = recorder.Telemetry('health.csv', ('temp', 'mem_free', 'load'))
        self._ok = True
        return True

    def temperature(self) -> float:
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
