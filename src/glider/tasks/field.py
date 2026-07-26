"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The CC-less field agent (doc/specs/coludo.md "Field operation without CC"). @task.activity('field'),
DISABLED by default. On the pad (SETTING) it makes at most two decisions:
  1. SITE BY GPS -- on the first fresh fix, the mission adopts the nearest launch.config site within
     max_range_m; none in range -> the synthesized spiral-landing fallback zone offset from the fix
     at the configured clear-sector bearing.
  2. AUTO-ARM (opt-in) -- arm once the board has sat STATIONARY with a live fix for the whole
     auto_arm_dwell_s. The long dwell makes a bench/carry arm unlikely, and the flight loop's
     control-stage gating still holds the fins neutral on the ground either way.
Each decision fires once, then the task idles; the operator/CC can still override everything live.
"""

import asyncio
import time

import commons
import controller as controller_mod
import databoard
import inspector
import recorder
import task

_STAGE = controller_mod.Stage


@task.activity('field')
class Field(task.Task):
    """Site-by-GPS + optional auto-arm, so a board can fly with no Control hub present."""

    async def setup(self) -> bool:
        cfg = self.config
        self._period_ms: int = cfg.get('period_ms', 1000)
        self._site_select: bool = cfg.get('site_select', True)
        self._fallback_bearing: float = cfg.get('fallback_bearing_deg', 0.0)  # the clear sector
        self._fallback_near: float = cfg.get('fallback_near_m', 50.0)    # near edge from the pad (safety)
        self._fallback_width: float = cfg.get('fallback_width_m', 100.0)  # wide side facing the pad (L-R)
        self._fallback_depth: float = cfg.get('fallback_depth_m', 90.0)   # near-far extent
        self._auto_arm: bool = cfg.get('auto_arm', False)  # opt-in: arming is normally a human act
        self._arm_dwell_ms: int = int(cfg.get('auto_arm_dwell_s', 60) * 1000)
        self._still_g: float = cfg.get('still_g', 0.3)  # the stationary band, same as the sequencer
        self._still_lo_sq: float = (1.0 - self._still_g) ** 2
        self._still_hi_sq: float = (1.0 + self._still_g) ** 2
        self._position = databoard.Databoard.parameter('position')
        self._accel = databoard.Databoard.parameter('accel')
        self._mission = inspector.Inspector.get('mission')  # may be None (bench)
        self._site_done: bool = not self._site_select or self._mission is None
        self._still_since = None  # start of the stationary-with-a-fix dwell (auto-arm)
        self._ok = True
        return True

    def _tick(self, now: int) -> None:
        """
        One pad decision step: site-by-GPS, then optionally auto-arm.

        Sync + tick-driven so the test controls time. Only in SETTING -- past ignition or once armed
        there is nothing left to decide.

        Args:
            now - the current time in ticks_ms (the auto-arm dwell is measured against it).

        Returns:
            None; may adopt a site / fallback zone and auto-arm the controller as side effects.
        """
        if self.controller.stage != _STAGE.SETTING:
            return
        fix, source, _age = self._position.read()
        if source is None or fix is None:  # arming requires a LIVE fix through the whole dwell
            self._still_since = None
            return
        if not self._site_done:  # the first fresh fix picks the site (once)
            self._site_done = True
            name = self._mission.select_site(fix)
            if name is not None:
                recorder.Recorder.log(self.name, 'site by GPS: %s' % name)
            else:
                self._mission.fallback_zone(fix, self._fallback_bearing, self._fallback_near,
                                            self._fallback_width, self._fallback_depth)
                recorder.Recorder.log(self.name, 'no site within %.0fm -> fallback zone (bearing %.0f)'
                                      % (self._mission.max_range_m, self._fallback_bearing))
        if not self._auto_arm or self.controller.armed:
            return
        if self._mission is not None and self._mission.zone is None:
            return  # no landing zone -> never auto-arm (a blind flight is a HUMAN decision)
        accel = self._accel.value()
        if accel is None:
            self._still_since = None
            return
        g_sq = commons.magnitude_sq(accel[0], accel[1], accel[2])
        if not (self._still_lo_sq < g_sq < self._still_hi_sq):
            self._still_since = None  # moving (being carried/handled) -> the dwell restarts
            return
        if self._still_since is None:
            self._still_since = now
        elif time.ticks_diff(now, self._still_since) >= self._arm_dwell_ms:
            self.controller.arm()  # logs 'controller :: armed'
            recorder.Recorder.log(self.name, 'auto-armed: stationary with a fix for %d s'
                                  % (self._arm_dwell_ms // 1000))

    async def run(self) -> None:
        while True:
            await asyncio.sleep_ms(self._period_ms)
            self._tick(time.ticks_ms())
