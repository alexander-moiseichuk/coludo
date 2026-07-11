"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Board vitals task: samples temperature, free memory and CPU load every period, pushes a telemetry row
(health.csv) and exposes the latest to the operator. Registered as @task.activity('health') so the
Controller creates and supervises it.

CPU load (an integer percent 0..100) is estimated WITHOUT busy-spinning: a probe task sleeps a fixed
period (probe_ms) and measures how LATE it actually wakes. asyncio.sleep_ms only resumes once the event
loop is free, so time other tasks spend running delays the wake-up -- the overshoot beyond the nominal
sleep is the time the CPU was busy with other work:
  load% = round(100 * (elapsed - probe_ms) / elapsed).
Sleeping rather than spinning on sleep_ms(0) lets the core actually idle between probes (FreeRTOS idle /
WFI) -- much lower idle power draw (the old spin pinned the CPU at ~100%). No calibration baseline is
needed (it is absolute). test_board_health drives a CPU hog and asserts the load rises with it.
"""

import asyncio
import gc
import time

import controller as controller_mod
import databoard
import fixed
import recorder
import task

try:
    import esp32
except ImportError:
    esp32 = None

_RESCUE_PAUSE_MS: int = 200  # a CONSERVATIVE gc.collect() pause estimate (the typical collect is ~67 ms).
                             # The rescue's whole safe-altitude floor is 2x THIS worth of descent (dynamic,
                             # from the live sink rate -- no fixed base): so the doubled, already-generous
                             # pause still leaves the glider clear of the ground after the collect.


@task.activity('health')
class BoardHealth(task.Task):
    """Periodic vitals -> telemetry (health.csv) + `inspect health`."""

    async def setup(self) -> bool:
        self.period_ms: int = self.config.get('period_ms', 1000)
        self._probe_ms: int = self.config.get('probe_ms', 10)  # idle-probe sleep -> CPU relaxes between probes
        # all-integer bookkeeping (fixed-point convention): the leak slope is a plain bytes/s count, the
        # descent slope + elevation are fixnums (m/s, m -- the SCALE-100 centi-representation is internal),
        # predictions are whole seconds; the float baro elevation converts to a fixnum once, at the boundary.
        # The rescue's
        # safe-altitude floor is FULLY DYNAMIC (see _rescue) -- 2x the descent a ~200 ms collect pause
        # costs, so the pause never sinks the glider to the ground. No fixed/base altitude: the stage gate
        # already excludes the LANDING flare, and 2x keeps the glider ~200 ms clear of the ground after.
        self._rescue_on: bool = self.config.get('rescue', True)  # explicit off for memory-behaviour tests
        self._elevation = databoard.Databoard.parameter('elevation')  # rescue safety gate (baro height)
        self.load: int = 0  # CPU load as an integer percent 0..100 (from probe wake-up lateness)
        self.rescues: int = 0  # emergency in-flight collects performed (operator-visible)
        self._leak_bps: int = 0  # EMA of the mem_free decay (bytes/s; 0 = not shrinking)
        self._last_free: int = 0
        self._last_us: int = time.ticks_us()
        self._descent: fixed.fixnum = 0  # EMA of the sink rate (fixnum m/s down; 0 = not descending)
        self._last_elevation = None  # last baro elevation as a fixnum (m), for the sink slope + land_s
        self._last_elevation_us: int = time.ticks_us()
        self._telemetry = recorder.Telemetry('health.csv', ('temp', 'mem_free', 'load', 'oom_s', 'land_s'))
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
        return {'temp': self.temperature(), 'mem_free': self.mem_free(), 'load': self.load,
                'oom_s': self.oom_s(), 'land_s': self.land_s(), 'rescues': self.rescues}

    def oom_s(self):
        """
        Predicted seconds to memory exhaustion from the current decay slope.

        Telemetry + `inspect health`; one side of the rescue decision.

        Returns:
            Seconds to exhaustion at the current leak slope, or None while the heap is not shrinking.
        """
        if self._leak_bps <= 0:
            return None
        return self._last_free // self._leak_bps

    def land_s(self):
        """
        Predicted seconds until the glider sinks to the ground, from the elevation-decay slope.

        The other side of the rescue decision, and the operator's landing countdown.

        Returns:
            Seconds to touchdown at the current sink rate, or None while not descending / no elevation.
        """
        if self._descent <= 0 or self._last_elevation is None:
            return None
        return max(0, self._last_elevation) // self._descent  # seconds to sink to the ground

    def _track(self, free: int, elevation) -> None:
        """
        Update the memory-decay and elevation-decay EMAs behind oom_s()/land_s().

        Both EMAs use alpha 1/4, all-int. Whole-second elapsed keeps every product inside small ints (a
        1 Hz sampler; the EMA absorbs the rounding). Growth resets a slope -- a completed collect / a
        climb invalidates the old trend.

        Args:
            free - the current gc.mem_free() reading (bytes).
            elevation - the current baro elevation (metres, float), or None when there is no reading.

        Returns:
            None; updates the decay-slope state as a side effect.
        """
        now = time.ticks_us()
        elapsed_s = max(1, time.ticks_diff(now, self._last_us) // 1000000)
        if self._last_free:
            slope = (self._last_free - free) // elapsed_s  # bytes/s, positive = shrinking
            self._leak_bps = (self._leak_bps * 3 + slope) // 4 if slope > 0 else 0
        self._last_free = free
        self._last_us = now
        if elevation is not None:
            elevation_fx = fixed.from_float(elevation)  # the float sensor value -> a fixnum (m), boundary-only
            elapsed_s = max(1, time.ticks_diff(now, self._last_elevation_us) // 1000000)
            if self._last_elevation is not None:
                sink = (self._last_elevation - elevation_fx) // elapsed_s  # fixnum m/s down
                self._descent = (self._descent * 3 + sink) // 4 if sink > 0 else 0
            self._last_elevation = elevation_fx
            self._last_elevation_us = now

    def _rescue(self, free: int, elevation) -> None:
        """
        The pre-OOM memory rescue (coludo.md "In-flight reboot"): an emergency in-flight collect.

        With GC off for the airborne phase the leak is GARBAGE, so an emergency collect (the fins hold
        their last write through the pause) reclaims the flight -- vastly cheaper than the OOM chain
        (~1.4 s frozen fins + ~7 s reboot + a warm-start gamble). Called every period: the rescue
        RE-FIRES for as long as the trigger holds, so a persistent leak gets a collect per second while
        the altitude allows. The decision is physics, not a byte threshold: collect when memory dies
        BEFORE the flight is safely over -- predicted oom_s < 2x the time left to sink to the ground
        (land_s) -- and only with proven safe altitude (a known elevation above the DYNAMIC floor = 2x
        the descent a ~200 ms pause costs), in BOOSTING/GLIDING (never LANDING). No descent trend yet ->
        no rescue: the glide always descends, so land_s exists exactly where a rescue is meaningful.

        Args:
            free - the current gc.mem_free() reading (bytes), for the rescue log line.
            elevation - the current baro elevation (metres, float), or None; None -> no rescue (safe
                altitude must be PROVEN, not assumed).

        Returns:
            None; runs an emergency gc.collect() (kicking the watchdog around it) when the trigger holds,
            otherwise returns without pausing.
        """
        if not self._rescue_on:
            return  # the rescue is disabled (config)
        if elevation is None:
            return  # safe altitude must be PROVEN, not assumed
        # DYNAMIC floor, no base: 2x the descent the ~pause would cost. A faster sink needs more headroom,
        # and doubling a pause estimate that is ALREADY conservative (typical collect ~67 ms) leaves the
        # glider ~200 ms clear of the ground after the collect. The stage gate excludes the LANDING flare.
        floor = self._descent * 2 * _RESCUE_PAUSE_MS // 1000  # fixnum m: same SCALE as the elevation below
        if fixed.from_float(elevation) <= floor:  # elevation (m) -> fixnum at the boundary, then compare
            return
        stage = self.controller.stage
        if not (controller_mod.Stage.BOOSTING <= stage < controller_mod.Stage.LANDING):
            return
        oom = self.oom_s()
        if oom is None:
            return
        land = self.land_s()
        if land is None or oom >= 2 * land:
            return  # not descending yet, or we land long before memory dies -- no pause needed
        watchdog = self.controller.active('watchdog')  # None when the watchdog is disabled
        if watchdog is not None:
            watchdog.kick()  # the collect is atomic and unfeedable -- start it on a FULL WDT budget
        started = time.ticks_us()
        gc.collect()
        paused_ms = time.ticks_diff(time.ticks_us(), started) // 1000
        if watchdog is not None:
            watchdog.kick()  # and hand the feed loop a fresh window on the way out
        self.rescues += 1
        self._leak_bps = 0  # the old trend is gone with the garbage
        recorder.Recorder.log(self.name, 'memory rescue: collect %d ms, %d -> %d KB free (oom %ss, land %ss)' % (
            paused_ms, free // 1024, gc.mem_free() // 1024, oom, land))

    def _row(self) -> None:
        vitals = self.sample()
        elevation = self._elevation.value()
        self._track(vitals['mem_free'], elevation)
        self._rescue(vitals['mem_free'], elevation)
        self._telemetry.push((vitals['temp'], vitals['mem_free'], vitals['load'],
                              self.oom_s(), self.land_s()))

    async def _probe_loop(self) -> None:
        """
        Estimate CPU load from how late a fixed sleep wakes (other tasks delay the event loop).

        The probe SLEEPS rather than spinning, so the core idles between samples -> low power.
        load% = 100 * overshoot / elapsed, clamped 0..100.

        Returns:
            None; loops forever, updating self.load once per probe.
        """
        nominal = self._probe_ms
        while True:
            before = time.ticks_us()
            await asyncio.sleep_ms(nominal)
            elapsed = time.ticks_diff(time.ticks_us(), before) / 1000.0  # ms actually slept
            overshoot = elapsed - nominal
            self.load = min(100, round(100.0 * overshoot / elapsed)) if overshoot > 0 else 0

    async def run(self) -> None:
        """
        Push a vitals row at startup, then every period_ms.

        A probe task tracks CPU load alongside the row loop.

        Returns:
            None; loops forever, pushing a vitals row per period.
        """
        asyncio.create_task(self._probe_loop())
        self._row()  # first row at startup
        while True:
            await asyncio.sleep_ms(self.period_ms)
            self._row()

    async def probe(self) -> str:
        """
        On-demand self-test: free memory reads positive (a basic board-vitals sanity).

        The temperature reading is logged for the operator (None on a build without
        esp32.mcu_temperature).

        Returns:
            None when the vitals read fine; an error message string when the free-memory check fails.
        """
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

    """Inspectable: the operator-facing vitals snapshot (inspect/stats)."""

    def inspect(self) -> dict:
        return self.sample()

    def stats(self) -> dict:
        return self.sample()
