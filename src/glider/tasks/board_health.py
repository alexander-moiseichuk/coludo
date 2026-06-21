# tasks/board_health.py — board vitals task: samples temperature, free memory and CPU load every
# period, pushes a telemetry row (health.csv) and exposes the latest to the operator. Registered as
# @task.activity('health') so the Controller creates and supervises it.
#
# CPU load (an integer percent 0..100) is estimated from a low-priority idle task that increments a
# counter and yields (sleep_ms(0)) in a tight loop. Each period we measure the idle counter's RATE
# (counts/ms); the highest rate ever observed (`_max_rate`) is taken as the fully-idle baseline, and
#   load% = round(100 * (1 - rate / _max_rate)).
# So load is RELATIVE to the busiest-idle moment seen: it self-calibrates as the board gets idle
# time (the baseline only rises), but a board that is never truly idle reads relative to its
# least-busy moment. test_board_health drives a CPU hog and asserts the load rises with real load.

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
        self.load: int = 0  # CPU load as an integer percent 0..100
        self._idle_count: int = 0
        self._max_rate: float = 0.0
        self._telemetry = recorder.Telemetry('health.csv', ('temp', 'mem_free', 'load'))
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
        return {'temp': self.temperature(), 'mem_free': self.mem_free(), 'load': self.load}

    def _row(self) -> None:
        vitals = self.sample()
        self._telemetry.push((vitals['temp'], vitals['mem_free'], vitals['load']))

    async def _idle_loop(self) -> None:
        while True:
            self._idle_count += 1
            await asyncio.sleep_ms(0)

    async def run(self) -> None:
        """Push a vitals row at startup, then every period_ms estimate load and push again. Runs
        forever."""
        asyncio.create_task(self._idle_loop())
        last_count = self._idle_count
        last_ms = time.ticks_ms()
        self._row()  # first row at startup (load 0 until the first interval calibrates it)
        while True:
            await asyncio.sleep_ms(self.period_ms)
            now_count = self._idle_count
            now_ms = time.ticks_ms()
            elapsed = time.ticks_diff(now_ms, last_ms)
            rate = (now_count - last_count) / elapsed if elapsed else 0.0
            last_count, last_ms = now_count, now_ms
            if rate > self._max_rate:
                self._max_rate = rate
            self.load = round(100 * (1.0 - rate / self._max_rate)) if self._max_rate else 0
            self._row()

    async def probe(self) -> str:
        """On-demand self-test: free memory reads positive (a basic board-vitals sanity); the
        temperature reading is logged for the operator (None on a build without esp32.mcu_temperature)."""
        try:
            recorder.Recorder.log(self.name, 'probe: vitals ...')
            mem = self.mem_free()
            if mem <= 0:
                raise ValueError('mem_free %d' % mem)
            recorder.Recorder.log(self.name, 'probe: vitals ok (mem_free %d, temp %s)' % (mem, self.temperature()))
        except Exception as error:
            message = 'vitals: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    # --- Inspectable ---
    def inspect(self) -> dict:
        return self.sample()

    def stats(self) -> dict:
        return self.sample()
