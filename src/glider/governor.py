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
# GNSS blend) ~ the biggest GC-off allocator measured (~22 KB/s at full rate). It matters most while
# the speed is fast-changing (boost accel + active deceleration), so it runs FULL RATE there; once
# settled it updates on an interval that adapts to the airspeed change — snap to the fast floor when
# the estimate moves, grow toward the ceiling as it settles. The estimator integrates the
# ACCUMULATED dt, so cadence never changes the integral — only how fresh the fin-authority cap is
# (the cap persists between updates).

import airspeed
import commons
import fixed
from fixed import fixnum


class GovernorConfig:
    """The governor's knobs, resolved from the flight task's config dict ONCE (typed config: one
    place for defaults + doc-in-code; the keys keep their board.config names)."""

    def __init__(self, config: dict):
        # full rate at/over this estimate (m/s): ANY overspeed (crosswind/gust, not just a dive) is
        # control-critical, so the throttle disengages wherever the speed may be breaking the limit.
        self.full_speed: float = config.get('airspeed_full_speed', 20.0)
        self.floor_s: float = config.get('airspeed_min_ms', 40) / 1000.0  # fast floor (~25 Hz)
        # settled ceiling. It ALSO bounds how stale the absolute-speed trigger can be: the estimate
        # refreshes at least this often, so an overspeed restores full rate within it. Kept low
        # (~10 Hz): the leak cost of a lower ceiling is tiny (~1 KB/s) but the safety gain is not.
        self.ceiling_s: float = config.get('airspeed_max_ms', 100) / 1000.0
        self.settle: float = config.get('airspeed_settle', 0.5)  # m/s change to keep updating fast
        # PROACTIVE full-rate override: a steep nose-down builds speed FAST and would outrun even the
        # bounded estimate, so a dive (fresh pitch, centidegree fixnum) forces full rate at once — a
        # LEADING indicator (the dive precedes the overspeed), complementing the absolute trigger.
        self.dive_pitch: fixnum = fixed.from_float(config.get('airspeed_dive_pitch', -45.0))
        # GNSS reports 2D GROUND speed: near-vertical flight (the boost climb, a steep dive) makes it
        # under-read true airspeed, and blending an under-read LOOSENS the fin cap exactly at high q —
        # the unsafe direction. At/steeper than this |pitch| the corrector is gated OFF (the integrator
        # flies alone, over-read biased = safe). HITL masks this (it publishes 3D total speed); the
        # real ATGM336H does not, so the gate is attitude-truth, not stage-truth.
        self.steep_pitch: fixnum = fixed.from_float(config.get('gnss_steep_pitch', 45.0))


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
        self._interval_s: float = config.floor_s  # current adaptive throttle interval (starts fast)
        self._accum_s: float = 0.0  # wall time since the last estimator update (integration dt)

    def airspeed(self) -> float:
        """The current airspeed estimate (m/s) — the boost rod gate and telemetry read it here."""
        return self._estimator.value()

    def step(self, dt: float, full_rate_override: bool, pitch: fixnum) -> None:
        """One control slice: accumulate `dt` (wall seconds since the last step) and update the
        estimator + fin cap when due. Full rate while the speed is fast-changing — so control is kept
        whenever airspeed may break the limit — on any of:
          - `full_rate_override` — the CALLER forces full rate from its own authority (the flight
            task passes stage < GLIDING: boost + active deceleration; stage stays the task's call,
            the governor only decides by its own speed/attitude rules below);
          - the ABSOLUTE estimate at/over full_speed — covers a crosswind/gust overspeed at any
            attitude; the ceiling keeps this trigger's staleness bounded (reaction within ceiling_s);
          - a fresh steep nose-down (`pitch` <= dive_pitch) — a dive leads the overspeed, so this
            re-arms full rate before the estimate would even show it.
        Otherwise adapt the throttle: update when the interval elapsed, snapping the interval to the
        floor when the estimate moved (>= settle) and growing it toward the ceiling as it settles."""
        self._accum_s += dt
        full_rate = (full_rate_override or self._estimator.value() >= self._config.full_speed
                     or pitch <= self._config.dive_pitch)
        if not (full_rate or self._accum_s >= self._interval_s):
            return  # throttled: the cap stays warm from the last update
        previous = self._estimator.value()
        self._update(self._accum_s, pitch)
        self._accum_s = 0.0
        if not full_rate:  # adapt the interval to the airspeed change since the last update
            moved = abs(self._estimator.value() - previous) >= self._config.settle
            self._interval_s = self._config.floor_s if moved else \
                min(self._config.ceiling_s, self._interval_s + self._config.floor_s)

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
