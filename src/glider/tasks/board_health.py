# tasks/board_health.py — board vitals task: samples temperature, free memory and CPU load every
# period, pushes a telemetry row (health.csv) and exposes the latest to the operator. Registered as
# @task.activity('health') so the Controller creates and supervises it.
#
# CPU load (an integer percent 0..100) is estimated WITHOUT busy-spinning: a probe task sleeps a fixed
# period (probe_ms) and measures how LATE it actually wakes. asyncio.sleep_ms only resumes once the
# event loop is free, so time other tasks spend running delays the wake-up -- the overshoot beyond the
# nominal sleep is the time the CPU was busy with other work:
#   load% = round(100 * (elapsed - probe_ms) / elapsed).
# Sleeping rather than spinning on sleep_ms(0) lets the core actually idle between probes (FreeRTOS idle
# / WFI) -- much lower idle power draw (the old spin pinned the CPU at ~100%). No calibration baseline
# is needed (it is absolute). test_board_health drives a CPU hog and asserts the load rises with it.

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
        self._probe_ms: int = self.config.get('probe_ms', 10)  # idle-probe sleep -> CPU relaxes between probes
        self.load: int = 0  # CPU load as an integer percent 0..100 (from probe wake-up lateness)
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

    async def _probe_loop(self) -> None:
        """Estimate CPU load from how late a fixed sleep wakes (other tasks delay the event loop). The
        probe SLEEPS rather than spinning, so the core idles between samples -> low power. load% =
        100 * overshoot / elapsed, clamped 0..100."""
        nominal = self._probe_ms
        while True:
            before = time.ticks_us()
            await asyncio.sleep_ms(nominal)
            elapsed = time.ticks_diff(time.ticks_us(), before) / 1000.0  # ms actually slept
            overshoot = elapsed - nominal
            self.load = min(100, round(100.0 * overshoot / elapsed)) if overshoot > 0 else 0

    async def run(self) -> None:
        """Push a vitals row at startup, then every period_ms. A probe task tracks CPU load. Runs
        forever."""
        asyncio.create_task(self._probe_loop())
        self._row()  # first row at startup
        while True:
            await asyncio.sleep_ms(self.period_ms)
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
