//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The dynamic-pressure fin governor (src/glider governor.py; doc/specs/coludo.md "Fin authority"), sibling of
//! [`crate::pid`], [`crate::mixer`] and [`crate::airspeed`]. It owns the airspeed ESTIMATE (the pitot when in
//! band, else the accelerometer backbone with the GNSS corrector), the ADAPTIVE THROTTLE that paces it, and the
//! mixer's authority cap: [`crate::commons::fin_deflection_limit`], which goes as 1/v^2, times the board's
//! `fins.limit_multiplier` safety dial.
//!
//! The sensors come in as the caller's fresh [`Readings`] -- plan B injected databoard handles and called
//! `read()`, never the databoard itself -- and nothing here reads a clock: the caller passes the elapsed time.
//!
//! WHY A THROTTLE AT ALL. On MicroPython the estimator's update is a float path (a square root, an integral, a
//! blend), measured at ~22 KB/s of GC-off leak at 100 Hz, so plan B updates it at full rate only where the
//! estimate cannot be trusted to pace itself (the boost and the active deceleration, a fresh dive), and
//! elsewhere by the DISTANCE-CONSTANT law: at clamp(speed, floor, ceiling) Hz, one update per ~1 m of TRAVEL.
//! Probed over 1..60 m/s against the earlier error-adaptive law: exactly 1.00 m per update across 5..50 m/s (the
//! old law ranged 0.04..1.90 m with a 9.5x step at its 20 m/s full-rate trigger), staleness that scales itself (an
//! overspeed shrinks its own next interval, so no absolute-speed trigger), and 3.3x less leak in a 30 m/s dive.
//! Rust has no such leak, but the throttle SHAPES the estimate -- the accelerometer is sampled once per update
//! and held over the whole accumulated interval, and each GNSS or pitot blend is one per update -- so it is ported
//! unchanged, as behaviour. The estimator integrates the ACCUMULATED elapsed time, so the cadence never changes
//! the integral's sum, only how fresh the cap is (the cap persists between updates).

use serde_json::Value;

use crate::airspeed::AirspeedEstimator;
use crate::commons::fin_deflection_limit;
use crate::mixer::Mixer;

/// The speed in whole m/s where the update-interval table saturates (governor._SPEED_MAX), the mirror of the fin
/// table's.
const SPEED_TABLE_TOP: i64 = 80;

/// Standard gravity as plan B writes it, m/s^2: the 1 g a level accelerometer reads, taken off |accel| before
/// integrating (9.81, not 9.80665, so the estimate matches plan B's).
pub const STANDARD_GRAVITY: f32 = 9.81;

/// The governor's knobs, resolved once from the flight component's config (governor.GovernorConfig). The fields
/// keep plan B's meaning; their config keys are named on each.
#[derive(Clone, Debug, PartialEq)]
pub struct GovernorConfig {
    /// `airspeed_floor_hz` (default 5): the slowest update rate, Hz. It bounds the time-staleness near stall
    /// (200 ms, where q is harmless).
    pub floor_hertz: f32,
    /// `airspeed_ceiling_hz` (default 50): the fastest throttled update rate, Hz. It capped the float-path leak
    /// at speed (20 ms, 1.0-1.2 m per update past 50 m/s).
    pub ceiling_hertz: f32,
    /// `airspeed_dive_pitch` (default -45), degrees: a pitch at or below it forces full rate at once. A steep
    /// nose-down builds speed FAST and would outrun even the self-scaling interval, which derives from the LAST
    /// estimate, so the dive is a LEADING indicator.
    pub dive_pitch: f32,
    /// `gnss_steep_pitch` (default 45), degrees: at or beyond +/- this pitch the GNSS corrector is gated OFF. GNSS
    /// reports 2D GROUND speed, so near-vertical flight (the boost climb, a steep dive) under-reads the airspeed,
    /// and blending an under-read LOOSENS the cap exactly at high q: the unsafe direction. The integrator flies
    /// alone there (over-read biased, safe). HITL masks this (it publishes the 3D total speed); the real ATGM336H
    /// does not, so the gate is attitude truth, not stage truth.
    pub steep_pitch: f32,
    /// `wind.max_speed` (default 15), m/s: a GNSS speed further than this from the estimate is a JUMP, not wind,
    /// and is rejected. A real ground speed differs from the airspeed only by the wind, which is physically
    /// bounded, and blending a spurious low speed loosens the cap. Shares the wind estimator's config subtree, so
    /// the envelope is set in one place.
    pub maximum_wind: f32,
    /// `airspeed_unconfident_ms` (default 30), m/s: the speed the cap is taken from until the estimate is
    /// confident. A fresh construct or a MID-AIR RESET reads 0 before anything charges it, and the table's 0 m/s
    /// entry is the full 45 degrees, wide open at high q. 30 m/s gives ~14 degrees: tight for a post-apogee dive
    /// yet past the ~10 degree boost hold. The estimate turns confident within a boost or on the first sane fix.
    pub unconfident_speed: f32,
    /// `pitot_gain` (default 0.5): the blend toward an in-band pitot reading, higher than the GNSS gain because the
    /// pitot measures the air, not a ground-speed proxy.
    pub pitot_gain: f32,
    /// `pitot_max_ms` (default 28), m/s: a pitot reading at or above it is dropped. The +/-500 Pa sensor rails at
    /// ~30 m/s (the boost, a steep dive) and UNDER-reads there, which would loosen the cap.
    pub pitot_maximum: f32,
    /// `pitot_min_ms` (default 3), m/s: a pitot reading below it is dropped, and this is the more dangerous side.
    /// q goes as v^2, so below a few m/s the pressure sinks into the tare and installation error (3 m/s is
    /// ~5.5 Pa, 1 m/s ~0.6 Pa), and a BLOCKED, iced, disconnected or unpressurised tube reads exactly there: a
    /// DEAD SENSOR, not a slow airspeed. Feeding it in drags the estimate to 0 and OPENS the cap to 45 degrees at
    /// high q. Found on the bench: the HITL sim did not simulate `airspeed`, so the real SDP810 kept publishing its
    /// still-air ~0 m/s; the glider flew a simulated 14 m/s while the governor believed 0.00, and whether the boost
    /// transient latched confidence decided the trajectory -- which is what split repeated identical HITL
    /// flights into two families.
    pub pitot_minimum: f32,
}

impl Default for GovernorConfig {
    /// Plan B's defaults, as an empty flight config resolves them.
    fn default() -> GovernorConfig {
        GovernorConfig::new(&Value::Null)
    }
}

impl GovernorConfig {
    /// The knobs a flight component config holds (governor.GovernorConfig(config)); a missing key takes plan B's
    /// default. A value that is not a number also takes the default: validate() refuses such a config before it
    /// gets here, where plan B would fail on it at its first use.
    pub fn new(config: &Value) -> GovernorConfig {
        let number =
            |value: Option<&Value>, default: f32| value.and_then(Value::as_f64).map_or(default, |value| value as f32);
        let key = |name: &str, default: f32| number(config.get(name), default);
        GovernorConfig {
            floor_hertz: key("airspeed_floor_hz", 5.0),
            ceiling_hertz: key("airspeed_ceiling_hz", 50.0),
            dive_pitch: key("airspeed_dive_pitch", -45.0),
            steep_pitch: key("gnss_steep_pitch", 45.0),
            maximum_wind: number(config.get("wind").and_then(|wind| wind.get("max_speed")), 15.0),
            unconfident_speed: key("airspeed_unconfident_ms", 30.0),
            pitot_gain: key("pitot_gain", 0.5),
            pitot_maximum: key("pitot_max_ms", 28.0),
            pitot_minimum: key("pitot_min_ms", 3.0),
        }
    }

    /// The estimator's update interval in seconds at a speed in m/s: the distance-constant law
    /// (governor.GovernorConfig.update_interval).
    ///
    /// `1 / clamp(speed, floor, ceiling)` Hz -- one update per ~1 m of travel -- with the speed truncated to a whole
    /// m/s and saturating at 80, exactly the entries of plan B's precomputed table (a MicroPython device to keep a
    /// division off the hot path; computed here, the value is the same).
    pub fn update_interval(&self, speed: f32) -> f32 {
        let whole = (speed as i64).clamp(0, SPEED_TABLE_TOP) as f32;
        1.0 / whole.max(self.floor_hertz).min(self.ceiling_hertz)
    }
}

/// The sensors one governor step may read, each FRESH or None (plan B's injected handles' `read()`, collapsed: a
/// stale or absent channel is None).
///
/// A STALE accelerometer must be None, not its last value extrapolated. It is the estimator's backbone: every
/// predict compounds into the airspeed the cap comes off, and the checkpoint persists that airspeed across a warm
/// start. A dead IMU extrapolates toward a steady ~1 g, which reads as level flight and drifts the estimate
/// toward the ceiling -- safe in DIRECTION (a high estimate tightens the cap), but it defeats the freshness-paced
/// update and survives a reboot through the crumb. No sample is better than an invented one.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Readings {
    /// The acceleration (x, y, z) in g.
    pub acceleration: Option<[f32; 3]>,
    /// The GNSS ground speed in m/s (a live fix).
    pub gnss_speed: Option<f32>,
    /// The pitot's indicated airspeed in m/s (the SDP810 driver did the square root): the DIRECT source.
    pub pitot: Option<f32>,
}

/// Cap the mixer's control authority by the estimated airspeed: torque goes as v^2 (governor.Governor).
///
/// [`Governor::step`] each control slice decides full rate or throttled, updates the estimator over the accumulated
/// time, and writes the deflection cap into the mixer's limit.
#[derive(Clone, Debug, PartialEq)]
pub struct Governor {
    /// The knobs.
    config: GovernorConfig,
    /// The board's `fins.limit_multiplier` safety dial: scales the whole 1/v^2 schedule.
    multiplier: f32,
    /// The airspeed estimate the cap comes off.
    estimator: AirspeedEstimator,
    /// The current update interval in seconds, from the LAST estimate (the floor's at rest).
    interval: f32,
    /// The seconds since the last estimator update: the integration interval of the next one.
    accumulated: f32,
    /// A stage's own ceiling on the cap, whole degrees (BOOSTING's `boost_limit_deg`, the operator's +/-10 of
    /// 2026-06-25): the cap written is the lower of the two. None: the cap alone, plan B's.
    ceiling: Option<i32>,
    /// The cap the last update derived, before the ceiling; None before the first.
    derived: Option<i32>,
    /// The pitot's airspeed the last update took, m/s, None when that update had none in band.
    measured: Option<f32>,
}

impl Governor {
    /// A governor with these knobs and the board's `fins.limit_multiplier` (default 1.0; a smaller one tightens
    /// the whole schedule), over a fresh, unconfident estimator.
    pub fn new(config: GovernorConfig, fin_limit_multiplier: f32) -> Governor {
        let interval = config.update_interval(0.0);
        Governor {
            config,
            multiplier: fin_limit_multiplier,
            estimator: AirspeedEstimator::default(),
            interval,
            accumulated: 0.0,
            ceiling: None,
            derived: None,
            measured: None,
        }
    }

    /// Set the stage's ceiling on the cap (None lifts it), the mixer re-capped AT ONCE when it changes and a cap was
    /// derived: a glide must not start under the boost's ceiling until the throttled next update. Before the first
    /// update the ceiling waits for it.
    pub fn set_ceiling(&mut self, mixer: &mut Mixer, ceiling: Option<i32>) {
        if ceiling == self.ceiling {
            return;
        }
        self.ceiling = ceiling;
        if let Some(cap) = self.derived {
            mixer.set_limit(self.ceiled(cap));
        }
    }

    /// The cap under the ceiling, if any.
    fn ceiled(&self, cap: i32) -> i32 {
        self.ceiling.map_or(cap, |ceiling| cap.min(ceiling))
    }

    /// Write a derived cap into the mixer, under the ceiling, and keep it for a ceiling change.
    fn write_cap(&mut self, mixer: &mut Mixer, cap: i32) {
        self.derived = Some(cap);
        mixer.set_limit(self.ceiled(cap));
    }

    /// The current airspeed estimate in m/s: the boost rod gate, the telemetry and the warm-start crumb read it.
    pub fn airspeed(&self) -> f32 {
        self.estimator.value()
    }

    /// The pitot's airspeed the last update measured (in its band), m/s; None when it had none. A MEASUREMENT, unlike
    /// [`Governor::airspeed`], which over-reads by design when unsure.
    pub fn measured_airspeed(&self) -> Option<f32> {
        self.measured
    }

    /// The estimator, read-only: its value and whether it is trusted yet.
    pub fn estimator(&self) -> &AirspeedEstimator {
        &self.estimator
    }

    /// The cap in whole degrees for a speed in m/s: the table times the safety dial, truncated as plan B's int()
    /// does, and never below 1 degree (always some authority).
    fn cap(&self, speed: f32) -> i32 {
        ((fin_deflection_limit(speed) as f32 * self.multiplier) as i32).max(1)
    }

    /// Restore the airspeed in m/s the warm-start crumb persisted, and cap the mixer off it AT ONCE
    /// (governor.Governor.seed_airspeed).
    ///
    /// Without the immediate re-cap the mixer would keep the blunt unconfident limit until the next scheduled
    /// update -- up to 1/floor (200 ms) of starved authority on a glider that has just rebooted mid-air, exactly
    /// the window this exists to close.
    ///
    /// A garbage value (negative, past the ceiling) seeds nothing, yet the re-cap still runs, off the unseeded
    /// estimate and whether or not it is trusted -- as in plan B. On a fresh governor that is 0 m/s: the FULL 45
    /// degrees (times the dial) until the next update caps off the unconfident speed, at most 1/floor later. The
    /// unsafe direction, kept because it is plan B's behaviour; the warm-start gate is what keeps garbage out.
    pub fn seed_airspeed(&mut self, mixer: &mut Mixer, airspeed: f32) {
        self.estimator.seed(airspeed);
        self.write_cap(mixer, self.cap(self.estimator.value()));
    }

    /// One control slice (governor.Governor.step): accumulate `elapsed` seconds and, when due, update the estimator
    /// and write the fin cap into the mixer.
    ///
    /// FULL RATE (every step) on either of:
    ///  * `full_rate_override` -- the CALLER forces it from its own authority: the flight task passes "stage before
    ///    GLIDING" (the boost and the active deceleration); the stage stays the task's call;
    ///  * a fresh steep nose-down, `pitch` (degrees, the last measured) at or below the dive pitch: a dive builds
    ///    speed faster than the interval, which derives from the LAST estimate, could follow.
    ///
    /// Otherwise the DISTANCE-CONSTANT throttle: update once the interval since the last update has elapsed, then
    /// re-derive the interval from the fresh estimate.
    pub fn step(&mut self, mixer: &mut Mixer, elapsed: f32, full_rate_override: bool, pitch: f32, readings: &Readings) {
        self.accumulated += elapsed;
        let full_rate = full_rate_override || pitch <= self.config.dive_pitch;
        if !(full_rate || self.accumulated >= self.interval) {
            return; // throttled: the cap stays warm from the last update
        }
        self.update(mixer, self.accumulated, pitch, readings);
        self.accumulated = 0.0;
        self.interval = self.config.update_interval(self.estimator.value());
    }

    /// Integrate |accel| - g over `elapsed` (the backbone), fold in the pitot or a sane GNSS fix, then cap the
    /// mixer (governor.Governor._update).
    ///
    /// PITOT FIRST: a fresh reading inside [pitot_minimum, pitot_maximum) is a direct measurement, with no wind or
    /// steep gate (it reads air, not ground). Both bounds exist for the same reason: outside them the reading
    /// under-reads, and an under-read LOOSENS the cap. Out of band, the GNSS corrector runs, gated by ATTITUDE
    /// (|pitch| at or beyond the steep pitch: the 2D ground speed cannot represent the airspeed) and by the wind
    /// bound (a jump is rejected) -- except while not yet confident, when the first fix is accepted as the seed.
    ///
    /// The cap comes off the estimate ONLY when it is trusted; else off the conservative unconfident speed, never
    /// off an uncharged 0, which would open the fins to the full 45 degrees at high q.
    fn update(&mut self, mixer: &mut Mixer, elapsed: f32, pitch: f32, readings: &Readings) {
        if let Some([x, y, z]) = readings.acceleration {
            let magnitude = (x * x + y * y + z * z).sqrt();
            self.estimator.predict((magnitude - 1.0) * STANDARD_GRAVITY, elapsed);
        }
        self.measured = None;
        match readings.pitot {
            Some(airspeed) if self.config.pitot_minimum <= airspeed && airspeed < self.config.pitot_maximum => {
                self.estimator.measure(airspeed, self.config.pitot_gain);
                self.measured = Some(airspeed);
            }
            _ => {
                let steep = pitch >= self.config.steep_pitch || pitch <= -self.config.steep_pitch;
                let estimate = self.estimator.value();
                let plausible = readings.gnss_speed.is_some_and(|speed| {
                    !self.estimator.confident() || (speed - estimate).abs() <= self.config.maximum_wind
                });
                self.estimator
                    .correct(readings.gnss_speed.unwrap_or(0.0), plausible && !steep);
            }
        }
        let cap_speed = if self.estimator.confident() {
            self.estimator.value()
        } else {
            self.config.unconfident_speed
        };
        self.write_cap(mixer, self.cap(cap_speed));
    }
}

#[cfg(test)]
mod tests {
    use super::{Governor, GovernorConfig, Readings};
    use crate::mixer::Mixer;
    use serde_json::{Value, json};

    /// One g straight up: net acceleration 0, so predict() changes nothing.
    const LEVEL: [f32; 3] = [0.0, 0.0, 1.0];

    /// A governor over this config and multiplier, and a default mixer (limit 45).
    fn governor(config: Value, multiplier: f32) -> (Governor, Mixer) {
        (
            Governor::new(GovernorConfig::new(&config), multiplier),
            Mixer::default(),
        )
    }

    /// Readings of a level accelerometer plus this GNSS speed and pitot reading.
    fn level(gnss_speed: Option<f32>, pitot: Option<f32>) -> Readings {
        Readings {
            acceleration: Some(LEVEL),
            gnss_speed,
            pitot,
        }
    }

    #[test]
    fn the_cap_follows_a_trusted_speed_times_the_dial() {
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        unit.seed_airspeed(&mut mixer, 0.0);
        unit.step(&mut mixer, 0.01, true, 0.0, &level(None, None));
        assert_eq!(mixer.limit(), 45, "0 m/s, genuinely slow: full authority");
        unit.seed_airspeed(&mut mixer, 40.0);
        assert_eq!(mixer.limit(), 8, "the seed re-caps at once");
        let (mut halved, mut halved_mixer) = governor(Value::Null, 0.5);
        halved.seed_airspeed(&mut halved_mixer, 0.0);
        assert_eq!(halved_mixer.limit(), 22, "int(45 x 0.5)");
        let (mut tiny, mut tiny_mixer) = governor(Value::Null, 0.01);
        tiny.seed_airspeed(&mut tiny_mixer, 50.0);
        assert_eq!(tiny_mixer.limit(), 1, "never below one degree");
    }

    #[test]
    fn a_stage_ceiling_caps_the_cap_and_lifts_at_once() {
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        // before any update the ceiling waits: the mixer keeps its config limit
        unit.set_ceiling(&mut mixer, Some(10));
        assert_eq!(mixer.limit(), 45);
        unit.seed_airspeed(&mut mixer, 0.0);
        assert_eq!(mixer.limit(), 10, "0 m/s derives 45; the ceiling writes 10");
        unit.seed_airspeed(&mut mixer, 40.0);
        assert_eq!(mixer.limit(), 8, "a cap under the ceiling stands");
        unit.seed_airspeed(&mut mixer, 0.0);
        unit.set_ceiling(&mut mixer, None);
        assert_eq!(mixer.limit(), 45, "lifted at once, not at the next update");
        // NEGATIVE: no ceiling is plan B's cap, and setting the same ceiling again writes nothing
        let (mut plain, mut plain_mixer) = governor(Value::Null, 1.0);
        plain.seed_airspeed(&mut plain_mixer, 0.0);
        assert_eq!(plain_mixer.limit(), 45);
        plain.set_ceiling(&mut plain_mixer, None);
        assert_eq!(plain_mixer.limit(), 45);
        unit.set_ceiling(&mut mixer, Some(10));
        assert_eq!(mixer.limit(), 10);
        unit.set_ceiling(&mut mixer, Some(10));
        assert_eq!(mixer.limit(), 10);
    }

    #[test]
    fn an_unconfident_estimate_caps_off_the_conservative_speed_not_zero() {
        let (mut unit, mut mixer) = governor(json!({"airspeed_unconfident_ms": 30.0}), 1.0);
        assert!(!unit.estimator().confident());
        unit.step(&mut mixer, 0.01, true, 0.0, &level(None, None));
        assert_eq!(mixer.limit(), 14, "fin_deflection_limit(30), not the 45 of 0 m/s");
        unit.step(&mut mixer, 0.01, true, 0.0, &level(Some(4.0), None));
        assert!(unit.estimator().confident());
        assert_eq!(
            mixer.limit(),
            45,
            "a sane fix anchors 4 m/s: low q, full authority is right"
        );
    }

    #[test]
    fn an_unconfident_estimate_takes_any_fix_as_its_seed() {
        // the safety path after a mid-air reset: the uncharged 0 m/s must not reject a real 25 m/s as a wind jump,
        // or the cap stays on the blunt unconfident speed (14 degrees) instead of the measured one (20)
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        unit.step(&mut mixer, 0.01, true, 0.0, &level(Some(25.0), None));
        assert_eq!(unit.airspeed(), 25.0);
        assert!(unit.estimator().confident());
        assert_eq!(mixer.limit(), 20);
        // NEGATIVE: once confident, the same 25 m/s off is a jump
        unit.step(&mut mixer, 0.01, true, 0.0, &level(Some(50.5), None));
        assert_eq!(unit.airspeed(), 25.0);
    }

    #[test]
    fn every_decision_edge_falls_on_plan_b_side() {
        // the pitot band includes its floor and excludes its ceiling
        let (mut band, mut band_mixer) = governor(Value::Null, 1.0);
        band.seed_airspeed(&mut band_mixer, 20.0);
        band.step(&mut band_mixer, 0.01, true, 0.0, &level(None, Some(28.0)));
        assert_eq!(
            (band.airspeed(), band_mixer.limit()),
            (20.0, 31),
            "exactly pitot_max_ms: dropped"
        );
        band.step(&mut band_mixer, 0.01, true, 0.0, &level(None, Some(3.0)));
        assert_eq!(band.airspeed(), 11.5, "exactly pitot_min_ms: blended");
        // the steep gate includes +/-45 degrees, and a centidegree inside blends
        let (mut gate, mut gate_mixer) = governor(Value::Null, 1.0);
        gate.seed_airspeed(&mut gate_mixer, 14.0);
        for steep in [45.0, -45.0] {
            gate.step(&mut gate_mixer, 0.01, true, steep, &level(Some(0.0), None));
            assert_eq!(gate.airspeed(), 14.0, "{steep} degrees: gated");
        }
        gate.step(&mut gate_mixer, 0.01, true, 44.99, &level(Some(0.0), None));
        assert!((gate.airspeed() - 11.2).abs() < 1e-5, "{}", gate.airspeed());
        // the wind bound includes max_wind, and half a m/s past it rejects
        let (mut bound, mut bound_mixer) = governor(Value::Null, 1.0);
        bound.seed_airspeed(&mut bound_mixer, 20.0);
        bound.step(&mut bound_mixer, 0.01, true, 0.0, &level(Some(35.5), None));
        assert_eq!(bound.airspeed(), 20.0, "15.5 m/s off: a jump");
        bound.step(&mut bound_mixer, 0.01, true, 0.0, &level(Some(35.0), None));
        assert_eq!(
            (bound.airspeed(), bound_mixer.limit()),
            (23.0, 24),
            "exactly 15 m/s off: blended"
        );
    }

    #[test]
    fn a_stale_accelerometer_never_reaches_the_integral() {
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        let boost = Readings {
            acceleration: Some([0.0, 0.0, 6.0]),
            ..Readings::default()
        };
        unit.step(&mut mixer, 0.1, true, 0.0, &boost);
        unit.step(&mut mixer, 0.1, true, 0.0, &boost);
        let charged = unit.airspeed();
        assert!(charged > 5.0 && unit.estimator().confident(), "{charged}");
        unit.step(&mut mixer, 0.1, true, 0.0, &Readings::default());
        assert_eq!(unit.airspeed(), charged, "no reading: nothing moves");
    }

    #[test]
    fn an_in_band_pitot_wins_and_a_railed_one_falls_back_to_gnss() {
        let (mut unit, mut mixer) = governor(json!({"pitot_gain": 0.5, "pitot_max_ms": 28.0}), 1.0);
        unit.step(&mut mixer, 0.1, true, 0.0, &level(None, Some(15.0)));
        assert_eq!(unit.airspeed(), 15.0, "the first in-band pitot seeds");
        unit.step(&mut mixer, 0.1, true, 0.0, &level(Some(40.0), Some(16.0)));
        assert_eq!(
            unit.airspeed(),
            15.5,
            "blended toward the pitot; the GNSS 40 was not consulted"
        );
        assert_eq!(unit.measured_airspeed(), Some(16.0), "the pitot's measurement");
        unit.step(&mut mixer, 0.1, true, 0.0, &level(Some(12.5), Some(30.0)));
        assert!(
            (unit.airspeed() - (15.5 + 0.2 * (12.5 - 15.5))).abs() < 1e-5,
            "railed: the GNSS blend ran"
        );
        // NEGATIVE: a railed pitot is no measurement, though the estimate holds a value
        assert_eq!(unit.measured_airspeed(), None);
    }

    #[test]
    fn a_dead_pitot_must_not_open_the_cap() {
        let (mut unit, mut mixer) = governor(json!({"pitot_min_ms": 3.0}), 1.0);
        let charge = Readings {
            acceleration: Some([0.0, 0.0, 3.0]),
            ..Readings::default()
        };
        for _ in 0..8 {
            unit.step(&mut mixer, 0.1, true, 0.0, &charge);
        }
        let (flying, tight) = (unit.airspeed(), mixer.limit());
        for _ in 0..20 {
            unit.step(&mut mixer, 0.1, true, 0.0, &level(None, Some(0.0)));
        }
        assert_eq!((unit.airspeed(), mixer.limit()), (flying, tight));
        unit.step(&mut mixer, 0.1, true, 0.0, &level(None, Some(2.9)));
        assert_eq!(unit.airspeed(), flying, "just under the floor: refused");
        unit.step(&mut mixer, 0.1, true, 0.0, &level(None, Some(3.1)));
        assert_ne!(unit.airspeed(), flying, "just over it: trusted");
    }

    #[test]
    fn a_gnss_jump_is_rejected_and_a_plausible_fix_blends() {
        let (mut unit, mut mixer) = governor(json!({"wind": {"max_speed": 15.0}}), 1.0);
        unit.seed_airspeed(&mut mixer, 20.0);
        unit.step(&mut mixer, 0.1, true, 0.0, &level(Some(60.0), None));
        assert_eq!(unit.airspeed(), 20.0, "40 m/s off: a jump");
        unit.step(&mut mixer, 0.1, true, 0.0, &level(Some(16.0), None));
        assert!((unit.airspeed() - 19.2).abs() < 1e-5, "{}", unit.airspeed());
    }

    #[test]
    fn steep_attitude_gates_the_gnss_both_ways() {
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        unit.seed_airspeed(&mut mixer, 14.0);
        unit.step(&mut mixer, 0.1, true, 85.0, &level(Some(0.0), None));
        assert_eq!(
            unit.airspeed(),
            14.0,
            "a vertical climb's 2D ground speed is not airspeed"
        );
        unit.step(&mut mixer, 0.1, true, -80.0, &level(Some(0.0), None));
        assert_eq!(unit.airspeed(), 14.0, "nor a steep dive's");
        unit.step(&mut mixer, 0.1, true, -6.0, &level(Some(0.0), None));
        assert!(unit.airspeed() < 14.0, "a shallow glide re-opens the blend");
        let (mut tight, mut tight_mixer) = governor(json!({"gnss_steep_pitch": 10.0}), 1.0);
        tight.seed_airspeed(&mut tight_mixer, 14.0);
        tight.step(&mut tight_mixer, 0.1, true, -15.0, &level(Some(0.0), None));
        assert_eq!(tight.airspeed(), 14.0);
    }

    #[test]
    fn the_throttle_skips_until_due_and_a_dive_or_the_override_forces_full_rate() {
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        unit.seed_airspeed(&mut mixer, 14.0);
        unit.interval = 1.0;
        unit.step(&mut mixer, 0.01, false, -6.0, &level(None, None));
        assert!(unit.accumulated > 0.0, "a settled glide throttles");
        unit.accumulated = 0.5;
        unit.step(&mut mixer, 0.01, false, -60.0, &level(None, None));
        assert_eq!(
            unit.accumulated, 0.0,
            "a dive forces full rate before the estimate shows the overspeed"
        );
        unit.step(&mut mixer, 0.01, true, 0.0, &level(None, None));
        assert_eq!(unit.accumulated, 0.0, "the caller's override never throttles");
        // NEGATIVE: exactly at the dive pitch counts as a dive, just above it does not
        unit.interval = 1.0;
        unit.step(&mut mixer, 0.01, false, -44.99, &level(None, None));
        assert!(unit.accumulated > 0.0);
        unit.step(&mut mixer, 0.01, false, -45.0, &level(None, None));
        assert_eq!(unit.accumulated, 0.0);
    }

    #[test]
    fn the_interval_is_one_metre_of_travel_between_the_floor_and_the_ceiling() {
        let config = GovernorConfig::new(&json!({"airspeed_floor_hz": 5, "airspeed_ceiling_hz": 50}));
        assert_eq!(config.update_interval(0.0), 0.2);
        assert_eq!(config.update_interval(4.0), 0.2);
        assert_eq!(config.update_interval(14.0), 1.0 / 14.0);
        assert_eq!(config.update_interval(14.9), 1.0 / 14.0, "a whole m/s, truncated");
        assert_eq!(config.update_interval(50.0), 0.02);
        assert_eq!(config.update_interval(75.0), 0.02);
        assert_eq!(config.update_interval(-3.0), 0.2);
        for speed in [5.0_f32, 10.0, 20.0, 35.0, 50.0] {
            assert!((speed * config.update_interval(speed) - 1.0).abs() < 1e-6);
        }
        // the interval re-derives from the fresh estimate after an update
        let (mut unit, mut mixer) = governor(Value::Null, 1.0);
        unit.seed_airspeed(&mut mixer, 14.0);
        unit.interval = unit.config.update_interval(14.0);
        unit.accumulated = unit.interval;
        unit.step(&mut mixer, 0.0, false, -6.0, &level(Some(28.0), None));
        assert!(unit.airspeed() > 14.0);
        assert_eq!(unit.interval, unit.config.update_interval(unit.airspeed()));
    }

    #[test]
    fn config_reads_plan_b_keys_and_defaults() {
        assert_eq!(
            GovernorConfig::default(),
            GovernorConfig::new(
                &json!({"airspeed_floor_hz": 5, "airspeed_ceiling_hz": 50, "airspeed_dive_pitch": -45.0,
                "gnss_steep_pitch": 45.0, "wind": {"max_speed": 15.0}, "airspeed_unconfident_ms": 30.0,
                "pitot_gain": 0.5, "pitot_max_ms": 28.0, "pitot_min_ms": 3.0})
            )
        );
        let config = GovernorConfig::new(&json!({"pitot_min_ms": "three", "wind": 4}));
        assert_eq!(
            (config.pitot_minimum, config.maximum_wind),
            (3.0, 15.0),
            "the wrong type takes the default"
        );
    }
}
