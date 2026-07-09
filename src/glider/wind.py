# wind.py — wind estimation from GNSS (plan #6). The MINIMAL method, proven first: the WIND TRIANGLE.
#
# The GNSS reports the ground velocity (course + ground speed); the attitude gives the heading; the
# governor gives the airspeed. The air mass the glider flies through is moving, so
#   wind = ground_velocity - air_velocity = (ground along course) - (airspeed along heading).
# An EMA smooths the per-fix estimate. The CROSSWIND component is bias-free; the ALONG-heading component
# inherits the governor's airspeed error (an over-read biases head/tailwind) -- acceptable for the rough
# uses (reachability margin, approach crab). If the field data shows that bias hurts, the airspeed-free
# GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).
#
# Float trig, fed once per GNSS fix (off the hot loop), like navigation -- a telemetry + reachability /
# approach input, not a fixnum control quantity.

import math


class WindEstimator:
    """Estimate the wind (east/north m/s) from the GNSS ground velocity vs the air velocity (the wind
    triangle, EMA-smoothed). update() once per GNSS fix; speed()/direction()/components() read it."""

    def __init__(self, alpha: float = 0.05):
        self._we: float = 0.0  # EMA wind (m/s, east / north)
        self._wn: float = 0.0
        self._seen: bool = False
        self._alpha: float = alpha

    def update(self, course: float, ground_speed: float, airspeed: float, heading: float) -> None:
        """One GNSS-fix update. `course`/`heading` deg, `ground_speed`/`airspeed` m/s."""
        course_r = math.radians(course)
        heading_r = math.radians(heading)
        wind_e = ground_speed * math.sin(course_r) - airspeed * math.sin(heading_r)
        wind_n = ground_speed * math.cos(course_r) - airspeed * math.cos(heading_r)
        if self._seen:
            self._we += self._alpha * (wind_e - self._we)
            self._wn += self._alpha * (wind_n - self._wn)
        else:
            self._we, self._wn, self._seen = wind_e, wind_n, True

    def components(self) -> tuple:
        """(east, north) m/s."""
        return self._we, self._wn

    def speed(self) -> float:
        return math.sqrt(self._we * self._we + self._wn * self._wn)

    def direction(self) -> float:
        """Where the wind blows FROM (meteorological convention), degrees. 0 when calm."""
        return math.degrees(math.atan2(-self._we, -self._wn)) % 360.0 if (self._we or self._wn) else 0.0

    def stats(self) -> dict:
        """Diagnostics for the wind soak / telemetry: the method, the estimate, and the raw components."""
        return {
            'method': 'triangle' if self._seen else 'none',
            'speed': round(self.speed(), 2), 'from': round(self.direction()),
            'we': round(self._we, 2), 'wn': round(self._wn, 2),
        }
