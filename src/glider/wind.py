"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Wind estimation from GNSS. The MINIMAL method, proven first: the WIND TRIANGLE.

The GNSS reports the ground velocity (course + ground speed); the attitude gives the heading; the
governor gives the airspeed. The air mass the glider flies through is moving, so
  wind = ground_velocity - air_velocity = (ground along course) - (airspeed along heading).
An EMA smooths the per-fix estimate. The CROSSWIND component is bias-free; the ALONG-heading component
inherits the governor's airspeed error (an over-read biases head/tailwind) -- acceptable for the rough
uses (reachability margin, approach crab). If the field data shows that bias hurts, the airspeed-free
GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).

The estimator OWNS its config subtree (the board `wind` block) and is Inspectable: it publishes and
accepts live tweaks of triangle_alpha + the physical envelope (calm_speed..max_speed) through
inspect()/update(), rather than the caller wiring individual params in. Float trig, fed once per GNSS
fix (off the hot loop) via observe() -- a telemetry + reachability/approach input, not a control fixnum.
"""

import math

import inspector


class WindEstimator(inspector.Inspectable):
    """
    Estimate the wind (east/north m/s) from the GNSS ground velocity vs the air velocity.

    The wind triangle, EMA-smoothed. observe() folds in one GNSS fix; speed()/direction()/components()
    read the estimate; inspect()/update() publish + retune the config subtree.
    """

    name: str = 'wind'
    kind: str = 'wind'
    _writable: tuple = ('triangle_alpha', 'calm_speed', 'max_speed')  # live-tunable via CC update

    def __init__(self, config: dict = None):
        """
        Bind the wind config subtree: the EMA rate and the physical wind envelope.

        Args:
            config - the board's `wind` subsection: `triangle_alpha` (the EMA rate) and the physical
                ENVELOPE `calm_speed`..`max_speed`. None -> the defaults.
        """
        config = config or {}
        self._we: float = 0.0  # EMA wind (m/s, east / north)
        self._wn: float = 0.0
        self._seen: bool = False
        self.triangle_alpha: float = config.get('triangle_alpha', 0.05)
        """
        the physical wind ENVELOPE (launches are ≤ ~10 m/s; the ceiling has spare for a GNSS jump): a
        per-fix estimate beyond max_speed is a GNSS course/speed JUMP, not wind -> reject the sample; a
        result below calm_speed is estimate/GNSS noise -> report calm.
        """
        self.calm_speed: float = config.get('calm_speed', 0.5)
        self.max_speed: float = config.get('max_speed', 15.0)

    def observe(self, course: float, ground_speed: float, airspeed: float, heading: float) -> None:
        """
        Fold in one GNSS fix (the wind triangle for this sample, EMA-blended into the estimate).

        A per-fix estimate beyond the wind envelope (max_speed) is rejected as a GNSS jump rather than
        folded in.

        Args:
            course - the GNSS ground-track course (degrees).
            ground_speed - the GNSS ground speed (m/s).
            airspeed - the governor's airspeed estimate (m/s).
            heading - the glider's heading (degrees).

        Returns:
            None. Updates the EMA wind estimate in place (or leaves it untouched on a rejected sample).
        """
        course_r = math.radians(course)
        heading_r = math.radians(heading)
        wind_e = ground_speed * math.sin(course_r) - airspeed * math.sin(heading_r)
        wind_n = ground_speed * math.cos(course_r) - airspeed * math.cos(heading_r)
        if wind_e * wind_e + wind_n * wind_n > self.max_speed * self.max_speed:
            return  # beyond the physical envelope -> a GNSS jump, not wind: reject (keep the EMA)
        if self._seen:
            self._we += self.triangle_alpha * (wind_e - self._we)
            self._wn += self.triangle_alpha * (wind_n - self._wn)
        else:
            self._we, self._wn, self._seen = wind_e, wind_n, True

    def components(self) -> tuple:
        """(east, north) m/s -- floored to calm (0, 0) below the envelope's calm_speed."""
        if self._we * self._we + self._wn * self._wn < self.calm_speed * self.calm_speed:
            return 0.0, 0.0
        return self._we, self._wn

    def speed(self) -> float:
        east, north = self.components()  # floored to calm below the envelope
        return math.sqrt(east * east + north * north)

    def direction(self) -> float:
        """Where the wind blows FROM (meteorological convention), degrees. 0 when calm."""
        east, north = self.components()  # floored to calm below the envelope
        return math.degrees(math.atan2(-east, -north)) % 360.0 if (east or north) else 0.0

    def inspect(self) -> dict:
        """Operator-facing: the tunable config + the current estimate."""
        return {'triangle_alpha': self.triangle_alpha, 'calm_speed': self.calm_speed,
                'max_speed': self.max_speed, 'speed': round(self.speed(), 2), 'from': round(self.direction())}

    def stats(self) -> dict:
        """Diagnostics for the wind soak / telemetry: the method, the estimate, and the raw components."""
        return {
            'method': 'triangle' if self._seen else 'none',
            'speed': round(self.speed(), 2), 'from': round(self.direction()),
            'we': round(self._we, 2), 'wn': round(self._wn, 2),
        }
