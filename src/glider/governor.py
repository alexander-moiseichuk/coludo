# governor.py — the dynamic-pressure fin governor (specs/coludo.md "Fin authority"), sibling of
# pid.py / mixer.py / airspeed.py. Owns the airspeed ESTIMATE (airspeed.AirspeedEstimator: accel
# backbone + GNSS corrector), the ADAPTIVE THROTTLE that keeps that float path off the GC-off hot
# loop once the glide settles, and the mixer authority cap (commons.fin_deflection_limit ∝ 1/v²,
# × the board's fin_limit_multiplier safety dial). Extracted from tasks/flight.py (doc/plan.md
# structural roadmap #1) so the throttle policy is unit-testable without a Flight task.
#
# Host-runnable by construction (tools/virtual_flight.py drives the REAL governor): the sensor
# dependencies are INJECTED databoard-style handles — `accel.value()` -> (x, y, z) in g or None,
# `gnss_speed.read()` -> (m/s, source, age_ms) — never the databoard itself, and nothing here
# touches time or the machine.
#
# Why the estimator is throttled at all: the update is a FLOAT path (sqrt magnitude, integrate,
# GNSS blend) ~ the biggest GC-off allocator measured (~22 KB/s at 100 Hz). It runs FULL RATE where
# the estimate cannot be trusted to pace itself (pre-glide boost/decel, a fresh dive); everywhere
# else the DISTANCE-CONSTANT law paces it: update at clamp(speed, floor, ceiling) Hz = one update
# per ~1 m of TRAVEL. Probed 1..60 m/s against the previous error-adaptive law (7/04):
#   * consistency — exactly 1.00 m/check across the whole 5..50 m/s envelope (old: 0.04..1.90 m
#     with a 9.5x discontinuity at its 20 m/s full-rate trigger);
#   * safety — the old law's WORST staleness (1.9 m at 19 m/s) sat right below its own trigger;
#     here staleness self-scales, an overspeed shrinks its own next interval, so the absolute-speed
#     trigger is gone entirely;
#   * leak — same class at glide trim (3.1 vs 2.2..5.6 KB/s), 3.3x LESS in a 30 m/s dive (6.7 vs
#     22.4 KB/s: no more 100 Hz above 20 m/s for granularity nothing needs);
#   * simplicity — 4 knobs -> 2 (floor/ceiling Hz) and the adaptation state machine becomes the
#     same precomputed integer-indexed table as commons.fin_deflection_limit.
# The estimator integrates the ACCUMULATED dt, so cadence never changes the integral — only how
# fresh the fin-authority cap is (the cap persists between updates).

import airspeed
import commons
import fixed
from fixed import fixnum

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_SPEED_MAX = const(80)  # m/s -- the update-interval table saturates here (mirror of the fin table)


class GovernorConfig:
    """The governor's knobs, resolved from the flight task's config dict ONCE (typed config: one
    place for defaults + doc-in-code; the keys keep their board.config names)."""

    def __init__(self, config: dict):
        # DISTANCE-CONSTANT throttle: the estimator updates at clamp(speed, floor, ceiling) Hz --
        # one update per ~1 m of TRAVEL at any speed (probed 1..60 m/s: exactly 1.00 m/check across
        # the whole 5..50 band). The floor bounds time-staleness near stall (200 ms where q is
        # harmless); the ceiling caps the float-path leak at speed (20 ms, 1.0-1.2 m/check past 50).
        # Precomputed ONCE as an interval table indexed by integer m/s -- the exact mirror of
        # commons.fin_deflection_limit (no per-update division on the hot path).
        self.floor_hz: int = config.get('airspeed_floor_hz', 5)
        self.ceiling_hz: int = config.get('airspeed_ceiling_hz', 50)
        self._interval_by_speed: tuple = tuple(
            1.0 / min(self.ceiling_hz, max(self.floor_hz, speed)) for speed in range(_SPEED_MAX + 1))
        # PROACTIVE full-rate override: a steep nose-down builds speed FAST and would outrun even the
        # self-scaling interval (it derives from the LAST estimate), so a dive (fresh pitch,
        # centidegree fixnum) forces full rate at once — a LEADING indicator.
        self.dive_pitch: fixnum = fixed.from_float(config.get('airspeed_dive_pitch', -45.0))
        # GNSS reports 2D GROUND speed: near-vertical flight (the boost climb, a steep dive) makes it
        # under-read true airspeed, and blending an under-read LOOSENS the fin cap exactly at high q —
        # the unsafe direction. At/steeper than this |pitch| the corrector is gated OFF (the integrator
        # flies alone, over-read biased = safe). HITL masks this (it publishes 3D total speed); the
        # real ATGM336H does not, so the gate is attitude-truth, not stage-truth.
        self.steep_pitch: fixnum = fixed.from_float(config.get('gnss_steep_pitch', 45.0))

    def update_interval(self, speed: float) -> float:
        """The estimator update interval (s) for `speed` (m/s) — the distance-constant table lookup
        (clamped to the floor/ceiling rates; saturates at _SPEED_MAX)."""
        return self._interval_by_speed[max(0, min(int(speed), _SPEED_MAX))]


class Governor:
    """Cap the mixer's control authority by estimated airspeed (torque ∝ v²): step() each control
    slice decides full-rate vs throttled, updates the estimator over the accumulated dt, and writes
    the deflection cap into mixer.limit."""

    def __init__(self, config: GovernorConfig, mixer, accel, gnss_speed,
                 fin_limit_multiplier: float = 1.0):
        self._config: GovernorConfig = config
        self._mixer = mixer  # the cap lands in mixer.limit (the final actuator clamp, every stage)
        self._accel = accel  # injected handle: value() -> (x, y, z) in g, or None
        self._gnss_speed = gnss_speed  # injected handle: read() -> (m/s, source, age_ms)
        self._multiplier: float = fin_limit_multiplier  # scales the whole 1/v² schedule (safety dial)
        self._estimator = airspeed.AirspeedEstimator()
        self._interval_s: float = config.update_interval(0.0)  # from the LAST estimate (floor at rest)
        self._accum_s: float = 0.0  # wall time since the last estimator update (integration dt)

    def airspeed(self) -> float:
        """The current airspeed estimate (m/s) — the boost rod gate and telemetry read it here."""
        return self._estimator.value()

    def step(self, dt: float, full_rate_override: bool, pitch: fixnum) -> None:
        """One control slice: accumulate `dt` (wall seconds since the last step) and update the
        estimator + fin cap when due. Full rate (every step) on either of:
          - `full_rate_override` — the CALLER forces full rate from its own authority (the flight
            task passes stage < GLIDING: boost + active deceleration; stage stays the task's call,
            the governor only decides by its own attitude rule below);
          - a fresh steep nose-down (`pitch` <= dive_pitch) — a dive builds speed faster than the
            self-scaling interval (which derives from the LAST estimate) could follow, so it forces
            full rate at once — a LEADING indicator.
        Otherwise the DISTANCE-CONSTANT throttle: update when the interval since the last update
        elapsed, then re-derive the interval from the fresh estimate (clamp(speed, floor, ceiling)
        Hz — one update per ~1 m of travel). Staleness self-scales with speed: an overspeed shrinks
        its own interval on the next update, so no absolute-speed trigger is needed."""
        self._accum_s += dt
        full_rate = full_rate_override or pitch <= self._config.dive_pitch
        if not (full_rate or self._accum_s >= self._interval_s):
            return  # throttled: the cap stays warm from the last update
        self._update(self._accum_s, pitch)
        self._accum_s = 0.0
        self._interval_s = self._config.update_interval(self._estimator.value())

    def _update(self, dt: float, pitch: fixnum) -> None:
        """Integrate |accel|-g over `dt` (the backbone) + blend a sane GNSS fix, then cap the mixer
        authority (∝ 1/v², × the safety multiplier). `dt` covers the wall time since the LAST update
        (per-step at full rate, accumulated when throttled) so the integral is cadence-independent.

        The GNSS corrector is gated by ATTITUDE: at |pitch| >= steep_pitch (the boost climb, a steep
        dive) the receiver's 2D ground speed cannot represent airspeed — blending it would drag the
        estimate DOWN and loosen the fin cap exactly at high dynamic pressure. Near-vertical, the
        integrator flies alone (over-read biased, the safe direction); a shallow attitude re-opens
        the blend and repeated good fixes pull the drift out.

        FLOAT PATH — ~22 KB/s GC-off leak at full rate (measured 224 B/call): the sqrt, the integral
        and the GNSS blend all box floats. Kept float BY DESIGN (findings §18: a fixnum rewrite
        captures ~1/3 of the saving and inverts the safety over-read bias — floor rounding under-reads
        speed → a looser fin cap, the unsafe direction); the adaptive throttle in step() amortizes it
        instead (25 Hz moving → 10 Hz settled)."""
        accel = self._accel.value()
        if accel is not None:
            self._estimator.predict(
                (commons.magnitude_sq(accel[0], accel[1], accel[2]) ** 0.5 - 1.0) * 9.81, dt)
        speed, speed_source, _speed_age = self._gnss_speed.read()
        steep = pitch >= self._config.steep_pitch or pitch <= -self._config.steep_pitch
        self._estimator.correct(speed if speed is not None else 0.0,
                                speed_source is not None and not steep)
        self._mixer.limit = max(1, int(
            commons.fin_deflection_limit(self._estimator.value()) * self._multiplier))
