# tasks/gnss_calib.py — GNSS consistent-drift calibration on the pad. @task.activity('gnss_calib').
#
# A STATIONARY GNSS position walks slowly (changing satellite geometry, ionospheric delay, multipath).
# Over the ~60 s a rocket sits on the pad the walk is roughly a constant DRIFT VELOCITY, and the almanac
# barely changes through the ~60 s flight -- so the drift measured on the pad predicts the drift in the
# air. A fixed antenna is NOT carried by the wind, so on the pad the receiver's whole reported ground
# velocity IS its drift: we average it through SETTING and FREEZE it at launch. The flight loop then
# subtracts it from the GNSS ground velocity before the wind triangle -- otherwise the drift folds
# STRAIGHT into the wind estimate (wind = ground_velocity - airspeed*heading), reading as phantom wind.
#
# Position-nav is deliberately NOT corrected: the drift over a 60 s flight is a few metres, inside the
# ~20 m turn-radius landing floor (specs/coludo.md), so it would not move the touchdown -- the win is a
# clean wind estimate. Slow loop (the drift is slow); Inspectable -> the operator sees the frozen drift.

import asyncio
import math

import controller
import databoard
import recorder
import task

_STAGE = controller.Stage


@task.activity('gnss_calib')
class GnssCalib(task.Task):
    """Average the reported ground velocity while stationary on the pad (SETTING) -> the GNSS drift, and
    freeze it at launch (BOOSTING). drift() hands it to the flight loop to de-bias the wind."""

    async def setup(self) -> bool:
        self._speed = databoard.Databoard.parameter('speed')    # reported ground speed (m/s)
        self._course = databoard.Databoard.parameter('course')  # reported ground track (deg)
        self._sum_e: float = 0.0    # accumulated reported ground velocity (ENU) while stationary
        self._sum_n: float = 0.0
        self._count: int = 0
        self._drift_e: float = 0.0  # frozen drift velocity (m/s ENU); (0, 0) until launch
        self._drift_n: float = 0.0
        self._frozen: bool = False
        self._min_samples: int = self.config.get('min_samples', 5)  # too few -> distrust, leave drift 0
        self._period_ms: int = self.config.get('period_ms', 1000)
        self._ok = True
        return True

    def _sample(self) -> None:
        """Fold one on-the-pad reading into the mean drift velocity (stationary -> velocity = drift)."""
        speed = self._speed.value()
        course = self._course.value()
        if speed is None or course is None:
            return
        course_r = math.radians(course)
        self._sum_e += speed * math.sin(course_r)
        self._sum_n += speed * math.cos(course_r)
        self._count += 1

    def _freeze(self) -> None:
        """At launch (SETTING -> BOOSTING): drift = the mean stationary ground velocity. Too few samples
        (a fast arm, or no fix on the pad) -> leave the drift 0 rather than trust a noisy mean."""
        if self._count >= self._min_samples:
            self._drift_e = self._sum_e / self._count
            self._drift_n = self._sum_n / self._count
        self._frozen = True
        recorder.Recorder.log(self.name, 'drift %.3f, %.3f m/s (%d samples)'
                              % (self._drift_e, self._drift_n, self._count))

    async def run(self) -> None:
        while True:
            if not self._frozen:
                stage = self.controller.stage
                if stage == _STAGE.SETTING:
                    self._sample()
                elif stage >= _STAGE.BOOSTING:  # off the pad (incl. a warm start into GLIDING) -> lock it
                    self._freeze()
            await asyncio.sleep_ms(self._period_ms)  # slow -- the drift is slow

    def drift(self) -> tuple:
        """(east, north) m/s frozen drift velocity; (0, 0) until launch freezes it (or too few samples)."""
        return self._drift_e, self._drift_n

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['drift_e'] = round(self._drift_e, 3)
        status['drift_n'] = round(self._drift_n, 3)
        status['samples'] = self._count
        status['frozen'] = self._frozen
        return status
