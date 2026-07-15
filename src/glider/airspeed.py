"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Hybrid airspeed estimate for the dynamic-pressure fin governor (coludo.md "Fin authority"). There is
NO pitot tube, so:
  * accelerometer integration is the BACKBONE (predict) -- primary, and the only usable source during
    boost and right after separation, when GNSS is jittery under high dynamics;
  * a valid, sane GNSS ground speed nudges out the integrator's drift (correct) -- a complementary
    filter, GNSS as the slow truth, accel as the fast signal.
GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading
under separation is a glitch), and only ever BLENDED (never a hard replace) so one bad-but-in-range
sample cannot jump the estimate; repeated good fixes pull the drift out. The estimate is biased to
over-read when uncertain -- a high airspeed tightens the governor cap, which is the safe direction.

CONFIDENCE: a freshly-constructed estimator (cold boot, or a MID-AIR RESET that re-runs setup) reads 0
before anything charges it -- and a 0 airspeed would open the governor's fin cap to full 45deg at high
dynamic pressure (the unsafe direction). `confident()` reports whether a TRUSTED speed exists yet: it
flips True once the accel integrator charges a clearly-airborne speed (boost / a building dive) or the
first sane GNSS fix anchors it (a direct set, not a slow blend). Until then the governor caps
conservatively rather than off the un-charged 0. `confident` is a latch -- once trusted, it stays so.
"""

_CONFIDENT_MS: float = 5.0  # accel-charged speed above which the estimate is 'clearly airborne' -> trusted


class AirspeedEstimator:
    """
    Fuse integrated body acceleration with sanity-gated GNSS ground speed into one airspeed estimate.

    The airspeed estimate (m/s) feeds the fin governor. Stateless of HOW accel-along-path is derived --
    the caller passes it (e.g. |accel| - g during boost), so this stays unit-testable on the host.
    """

    def __init__(self, ceiling_ms: float = 60.0, gnss_gain: float = 0.2):
        self._speed: float = 0.0            # current airspeed estimate (m/s)
        self._ceiling: float = ceiling_ms   # clamp the integral + reject GNSS above this (glitch guard)
        self._gnss_gain: float = gnss_gain  # complementary blend toward an accepted GNSS sample (0..1)
        self._confident: bool = False       # a trusted speed exists (accel-charged / GNSS-anchored); latch

    def value(self) -> float:
        """The current airspeed estimate (m/s)."""
        return self._speed

    def confident(self) -> bool:
        """
        Whether the estimate is trustworthy yet (see module header).

        False on a fresh construct / mid-air reset until the accel integrator charges a clearly-airborne
        speed or a sane GNSS fix anchors it; the governor caps conservatively while False so a spurious
        0 cannot open the fin authority to full 45deg at high dynamic pressure. A latch (once True, stays).

        Args:
            (none)

        Returns:
            True once a trusted speed has been established.
        """
        return self._confident

    def predict(self, accel_along: float, dt: float) -> float:
        """
        Integrate net acceleration ALONG the flight path over `dt` seconds -- the backbone.

        Clamped to [0, ceiling]. Pass the acceleration >= 0 / over-read to stay conservative.

        Args:
            accel_along - net acceleration along the flight path (m/s^2), >= 0 to stay conservative.
            dt - the integration interval (seconds).

        Returns:
            The updated airspeed estimate (m/s), clamped to [0, ceiling].
        """
        self._speed = max(0.0, min(self._speed + accel_along * dt, self._ceiling))
        if self._speed >= _CONFIDENT_MS:  # accel has charged a clearly-airborne speed -> trust it
            self._confident = True
        return self._speed

    def correct(self, gnss_speed: float, has_fix: bool) -> float:
        """
        Blend toward a GNSS ground speed ONLY if trustworthy: a live fix and within the physical ceiling.

        Anything above the ceiling is a separation/dynamics glitch -> ignored. When already confident it
        BLENDS by gnss_gain so one in-range bad sample moves the estimate only slightly, while a run of
        good fixes removes the integrator drift. When NOT yet confident (fresh boot / mid-air reset) the
        first accepted fix SEEDS directly -- a full set, not a slow 20% crawl that would leave the fin cap
        open for seconds. No fix or out-of-range -> the integrated backbone is kept untouched.

        Args:
            gnss_speed - the GNSS ground speed to blend toward (m/s).
            has_fix - whether the GNSS currently has a valid fix.

        Returns:
            The airspeed estimate (m/s): seeded/nudged toward gnss_speed when trusted, else unchanged.
        """
        if has_fix and 0.0 <= gnss_speed <= self._ceiling:
            if self._confident:
                self._speed += self._gnss_gain * (gnss_speed - self._speed)  # complementary blend
            else:
                self._speed = gnss_speed  # first trusted anchor -> seed directly
            self._confident = True
        return self._speed
