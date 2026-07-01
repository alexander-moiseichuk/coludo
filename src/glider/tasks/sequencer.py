# tasks/sequencer.py — Phase 3 flight-stage automation. @task.activity('sequencer'). Watches the
# databoard and drives the guarded, forward-only stage machine that the control loop gates on:
# SETTING -> BOOSTING : |accel| over launch_g sustained launch_ms (motor ignition), OR the baro climbing
#                       past launch_alt_m off the pad (an independent, threshold-robust backup)
# BOOSTING -> GLIDING : the separation switch (drivers/separation.py) is primary; this is the
# burnout-timeout FALLBACK if the switch never fires
# GLIDING -> LANDING : agl below land_agl_m (the laser sees the ground; elevation is the fallback)
# LANDING -> done : |accel| ~1 g (stationary) sustained ground_ms (on the ground)
# Each transition fires once (the stage check + reset-on-change is the guard), logs the reason and a
# sequencer.csv telemetry marker. Thresholds are config; launch_g/launch_ms is exactly what the
# E16/F15 passive flights tune. One control-independent tick, so it runs on the passive flights too
# (stages logged, no actuation -- the flight task stays disabled).

import asyncio
import gc
import math
import time

import commons
import controller as controller_mod
import databoard
import recorder
import task

_STAGE = controller_mod.Stage


def _magnitude_sq(accel):
    """Squared magnitude |accel|^2 in g^2 from (ax, ay, az), or None when there is no reading. Squared so
    the threshold compares skip math.sqrt() -- only the rare transition log takes the root. (At the
    50 Hz sequencer rate, with _magnitude_sq called only in SETTING/LANDING, this is a tidy-up, not a
    hot-path win: it is NOT on the 100 Hz control loop.) `accel is None` is guarded explicitly, not via a
    try/except on the unpack -- raising would allocate a traceback frame exactly under launch/impact
    vibration, the worst moment for GC churn."""
    if accel is None:
        return None
    ax, ay, az = accel
    return commons.magnitude_sq(ax, ay, az)


@task.activity('sequencer')
class Sequencer(task.Task):
    """Drive the flight-stage machine from sensor signals (forward-only, guarded, logged)."""

    async def setup(self) -> bool:
        cfg = self.config
        self._period_ms: int = cfg.get('period_ms', 50)
        self._launch_g: float = cfg.get('launch_g', 2.5)
        self._launch_ms: int = cfg.get('launch_ms', 100)
        self._launch_alt_m: float = cfg.get('launch_alt_m', 10.0)  # OR-trigger: clearly climbed off the pad
        self._boost_timeout_ms: int = cfg.get('boost_timeout_ms', 6000)
        self._land_agl_m: float = cfg.get('land_agl_m', 5.0)
        self._land_ms: int = cfg.get('land_ms', 300)  # AGL must stay below land_agl_m this long (anti-spike)
        self._still_g: float = cfg.get('still_g', 0.3)
        self._ground_ms: int = cfg.get('ground_ms', 3000)
        # compare |accel|^2 against squared thresholds so the detect path skips math.sqrt() (only the
        # rare transition LOG takes the root). The still-band 1 +/- still_g g maps to [lo, hi] in g^2
        # (assumes still_g < 1, which it always is -- it is a tolerance around 1 g).
        self._launch_g_sq: float = self._launch_g * self._launch_g
        self._still_lo_sq: float = (1.0 - self._still_g) ** 2 if self._still_g < 1.0 else 0.0
        self._still_hi_sq: float = (1.0 + self._still_g) ** 2
        # (coludo.md GC policy): compact the heap at launch and DISABLE GC while airborne, so no GC
        # pause (0.3 ms clean .. tens of ms on a full heap) can blow a 100 Hz control slice; re-enable at
        # touchdown. Safe only because the hot paths are near-zero-alloc (mixer, nav cache) and the
        # ~12 MB PSRAM absorbs the rest of the flight -- verified by a HITL heap soak. gc_flight False
        # keeps GC on (ground tests, and the unit test below).
        self._gc_flight: bool = cfg.get('gc_flight', True)
        self._accel = databoard.Databoard.parameter('accel')
        self._agl = databoard.Databoard.parameter('agl')
        self._elevation = databoard.Databoard.parameter('elevation')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('stage', 'reason'))
        self._since = None  # start of the current pending condition (sustained-detect timer)
        self._stage_seen = None  # last stage observed -> reset the timer on any change (incl. separation)
        self._ok = True
        return True

    def _advance(self, to_stage: int, reason: str) -> None:
        self.controller.set_stage(to_stage)  # logs 'controller :: stage -> X'
        recorder.Recorder.log(self.name, 'stage -> %s (%s)' % (_STAGE.STAGES[to_stage], reason))
        self._telemetry.push((_STAGE.STAGES[to_stage], reason))
        self._since = None
        if self._gc_flight:  # clean heap into the flight, GC OFF for the WHOLE airborne phase
            if to_stage == _STAGE.BOOSTING:
                start = time.ticks_us()
                gc.collect()    # compact + free before the flight (a known pause, on the rod)
                took = time.ticks_diff(time.ticks_us(), start)
                recorder.Recorder.log(self.name, 'gc pre-flight collect %d us' % took)  # for post-flight analysis
                gc.disable()
            elif to_stage == _STAGE.DONE:
                # re-enable + collect ONLY once stationary on the ground. The collect after a GC-off
                # flight has accumulated garbage and blocks tens of ms (coludo.md) -- paying that at the
                # LANDING transition would be at <land_agl_m (<5 m) and possibly mid-flare, the worst
                # place for a control-loop stall. Holding GC off through the flare and collecting on the
                # ground means NO GC pause ever happens in the air (it would be wrong to fly the whole
                # descent and then crash on a GC stall at the end). Log the post-flight pause -- it is the
                # actual cost the airborne phase deferred, recorded for analysis.
                gc.enable()
                start = time.ticks_us()
                gc.collect()
                took = time.ticks_diff(time.ticks_us(), start)
                recorder.Recorder.log(self.name, 'gc post-flight collect %d us' % took)  # the deferred cost

    async def finish(self) -> None:
        gc.enable()  # never leave GC disabled if the task stops mid-flight (defensive)

    def _sustained(self, now: int, threshold_ms: int) -> bool:
        """Dwell timer shared by every stage branch: start it on the first call after a reset, then
        return True once `now` is at least `threshold_ms` past the start -- so a single sample never
        triggers a transition. Callers clear self._since (the start) when their condition lapses."""
        if self._since is None:
            self._since = now
        return time.ticks_diff(now, self._since) >= threshold_ms

    def _tick(self, now: int) -> None:
        """One stage-machine step. `now` is ticks_ms. Forward-only: each branch only advances, and the
        sustained-detect timer resets whenever the stage changes (so a separation-driven hop is clean).
        Paused while the operator holds the stage (ground test).

        Missing sensor readings are guarded with explicit `is not None` checks -- NOT try/except. In
        MicroPython raising allocates a traceback frame (GC churn), and the missing-accel case is most
        frequent exactly under the launch/impact vibration that drops the most samples -- the worst
        moment for a GC latency spike (). A dropped sample simply does not advance the timer."""
        if self.controller.manual:  # operator holds the stage -> do not auto-advance
            return
        stage = self.controller.stage
        if stage != self._stage_seen:  # changed (by us or by the separation driver) -> fresh timer
            self._since = None
            self._stage_seen = stage
        # branches ordered by in-flight likelihood -- GLIDING (the long, control-critical phase)
        # first, then BOOSTING, LANDING, and SETTING (on the pad, relaxed) last, so the airborne stages
        # cost the fewest comparisons.
        if stage == _STAGE.GLIDING:
            agl = self._agl.value()
            height = agl if agl is not None else self._elevation.value()
            if height is not None and height < self._land_agl_m:  # below the landing height...
                if self._sustained(now, self._land_ms):  # ...and SUSTAINED (not a single spike)
                    self._advance(_STAGE.LANDING, 'agl %.1fm' % height)
            else:
                self._since = None  # rose back / lost reading -> reset: a single low sample never flares
        elif stage == _STAGE.BOOSTING:
            if self._sustained(now, self._boost_timeout_ms):  # burnout fallback if separation never fires
                self._advance(_STAGE.GLIDING, 'burnout timeout (no separation)')
        elif stage == _STAGE.LANDING:
            g_sq = _magnitude_sq(self._accel.value())
            if g_sq is not None and self._still_lo_sq < g_sq < self._still_hi_sq:  # ~1 g, squared
                if self._sustained(now, self._ground_ms):
                    self._advance(_STAGE.DONE, 'stationary %.1fg' % math.sqrt(g_sq))
            else:
                self._since = None
        elif stage == _STAGE.SETTING:
            # launch = a sustained boost |a| (fast, primary) OR the baro clearly climbing off the pad
            # (slower, but unambiguous and threshold-independent -- a heavy stack that boosts near the
            # launch_g line, or a missed accel window, still trips once it has left the rod).
            elevation = self._elevation.value()
            g_sq = _magnitude_sq(self._accel.value())
            if elevation is not None and elevation > self._launch_alt_m:
                self._advance(_STAGE.BOOSTING, 'launch alt=%.0fm' % elevation)
            elif g_sq is not None and g_sq > self._launch_g_sq:  # |a| over launch_g, squared
                if self._sustained(now, self._launch_ms):
                    self._advance(_STAGE.BOOSTING, 'launch |a|=%.1fg' % math.sqrt(g_sq))
            else:
                self._since = None

    async def run(self) -> None:
        while True:
            await asyncio.sleep_ms(self._period_ms)
            self._tick(time.ticks_ms())
