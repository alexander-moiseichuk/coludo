# airspeed.py — hybrid airspeed estimate for the dynamic-pressure fin governor (coludo.md "Fin
# authority"). There is NO pitot tube, so:
#   * accelerometer integration is the BACKBONE (predict) — primary, and the only usable source during
#     boost and right after separation, when GNSS is jittery under high dynamics;
#   * a valid, sane GNSS ground speed nudges out the integrator's drift (correct) — a complementary
#     filter, GNSS as the slow truth, accel as the fast signal.
# GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading
# under separation is a glitch), and only ever BLENDED (never a hard replace) so one bad-but-in-range
# sample cannot jump the estimate; repeated good fixes pull the drift out. The estimate is biased to
# over-read when uncertain — a high airspeed tightens the governor cap, which is the safe direction.


class AirspeedEstimator:
    """Fuse integrated body acceleration (predict) with sanity-gated GNSS ground speed (correct) into one
    airspeed estimate (m/s) for the fin governor. Stateless of HOW accel-along-path is derived — the
    caller passes it (e.g. |accel| - g during boost), so this stays unit-testable on the host."""

    def __init__(self, ceiling_ms: float = 60.0, gnss_gain: float = 0.2):
        self._speed: float = 0.0            # current airspeed estimate (m/s)
        self._ceiling: float = ceiling_ms   # clamp the integral + reject GNSS above this (glitch guard)
        self._gnss_gain: float = gnss_gain  # complementary blend toward an accepted GNSS sample (0..1)

    def value(self) -> float:
        """The current airspeed estimate (m/s)."""
        return self._speed

    def predict(self, accel_along: float, dt: float) -> float:
        """Integrate net acceleration ALONG the flight path (m/s^2; pass it >= 0 / over-read to stay
        conservative) over `dt` seconds — the backbone. Clamped to [0, ceiling]."""
        self._speed = max(0.0, min(self._speed + accel_along * dt, self._ceiling))
        return self._speed

    def correct(self, gnss_speed: float, has_fix: bool) -> float:
        """Blend toward a GNSS ground speed ONLY if trustworthy: a live fix and within the physical
        ceiling (anything above is a separation/dynamics glitch -> ignored). Blends by gnss_gain so one
        in-range bad sample moves the estimate only slightly, while a run of good fixes removes the
        integrator drift. No fix or out-of-range -> the integrated backbone is kept untouched."""
        if has_fix and 0.0 <= gnss_speed <= self._ceiling:
            self._speed += self._gnss_gain * (gnss_speed - self._speed)
        return self._speed
