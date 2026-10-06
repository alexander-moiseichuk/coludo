//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The hybrid airspeed estimate the dynamic-pressure fin governor caps the fins by (src/glider airspeed.py;
//! doc/specs/coludo.md "Fin authority"). Three sources, in order of trust:
//!  * a PITOT/static differential-pressure reading ([`AirspeedEstimator::measure`]) -- a DIRECT airspeed
//!    measurement, the preferred source WHEN IN-BAND: it measures the air, not the ground, so no wind offset and no
//!    attitude gate. It saturates past its full scale (boost, a steep dive), where the caller falls back to the
//!    accelerometer rather than trust an under-read;
//!  * accelerometer integration is the BACKBONE ([`AirspeedEstimator::predict`]) -- always running, and the only
//!    usable source in the boost and right after separation, when the pitot rails and the GNSS jitters;
//!  * a valid, sane GNSS ground speed nudges out the integrator's drift ([`AirspeedEstimator::correct`]) -- a
//!    complementary filter used when the pitot is absent or stale: GNSS the slow truth, accel the fast signal.
//!
//! GNSS is DISTRUSTED by default: rejected without a fix and above a physical ceiling (a 100+ m/s reading at
//! separation is a glitch), and only ever BLENDED, never a hard replace, so one bad-but-in-range sample cannot jump
//! the estimate; repeated good fixes pull the drift out. The estimate is biased to over-read when uncertain: a
//! high airspeed tightens the governor's cap, the safe direction.
//!
//! CONFIDENCE: a fresh estimator (a cold boot, or a MID-AIR RESET that runs the setup again) reads 0 before
//! anything charges it, and a 0 airspeed would open the fin cap to the full 45 degrees at high dynamic pressure.
//! [`AirspeedEstimator::confident`] says whether a TRUSTED speed exists yet: it latches once the accelerometer
//! charges a clearly airborne speed or the first sane GNSS fix anchors it (a direct set, not a slow blend). Until
//! then the governor caps conservatively rather than off the uncharged 0.
//!
//! WARM START: the conservative cap keeps a recovered glider SAFE but blunt -- pinned at the unconfident 30 m/s it
//! starves fin authority exactly when a glider that just rebooted mid-air needs it. [`AirspeedEstimator::seed`]
//! closes that window from the warm-start crumb, so the recovery order is pitot, then saved, then GNSS: the crumb
//! is there at once (no fix to wait for), the accelerometer integrates on from it, and the pitot overrides it on
//! its first in-band read. GNSS comes last because a fix can take tens of seconds to come back after a reset.
//!
//! f32 m/s throughout, as plan B ran it (a float path even on MicroPython, where its boxing was amortised by the
//! governor's throttle). Plan B's `max(0.0, min(x, ceiling))` is written out in Python's comparison order so a
//! NaN acceleration bounds to 0 as it does there.

/// The speed in m/s above which an accelerometer-charged or pitot-measured estimate counts as clearly airborne,
/// and so trusted (airspeed._CONFIDENT_MS).
pub const CONFIDENT_SPEED: f32 = 5.0;

/// The default glitch ceiling in m/s (airspeed.AirspeedEstimator `ceiling_ms`): the integral is clamped to it and a
/// GNSS speed above it is rejected.
pub const DEFAULT_CEILING: f32 = 60.0;

/// The default GNSS blend weight (airspeed.AirspeedEstimator `gnss_gain`): each accepted fix moves the estimate a
/// fifth of the way.
pub const DEFAULT_GNSS_GAIN: f32 = 0.2;

/// Fuse integrated acceleration with a sanity-gated GNSS ground speed and a direct pitot reading into one airspeed
/// estimate, in m/s (airspeed.AirspeedEstimator).
///
/// It does not know HOW the acceleration along the path is derived: the caller passes it (the governor uses
/// |accel| - g), so it stays testable on the host.
#[derive(Clone, Debug, PartialEq)]
pub struct AirspeedEstimator {
    /// The current estimate, m/s.
    speed: f32,
    /// The glitch ceiling, m/s: the integral is clamped to it and a GNSS speed above it rejected.
    ceiling: f32,
    /// The complementary blend weight toward an accepted GNSS speed, 0..1.
    gnss_gain: f32,
    /// Whether a trusted speed exists (accelerometer-charged, GNSS-anchored, pitot-measured or seeded); a latch.
    confident: bool,
}

impl Default for AirspeedEstimator {
    /// The estimator the governor runs: a 60 m/s ceiling and a 0.2 GNSS gain.
    fn default() -> AirspeedEstimator {
        AirspeedEstimator::new(DEFAULT_CEILING, DEFAULT_GNSS_GAIN)
    }
}

impl AirspeedEstimator {
    /// A fresh, unconfident estimator at 0 m/s with this glitch `ceiling` (m/s) and GNSS blend `gnss_gain`.
    pub fn new(ceiling: f32, gnss_gain: f32) -> AirspeedEstimator {
        AirspeedEstimator {
            speed: 0.0,
            ceiling,
            gnss_gain,
            confident: false,
        }
    }

    /// The current airspeed estimate, m/s.
    pub fn value(&self) -> f32 {
        self.speed
    }

    /// Whether the estimate is trustworthy yet (see the module notes): false on a fresh construct or a mid-air
    /// reset until the accelerometer charges a clearly airborne speed, a sane GNSS fix anchors it, an in-band pitot
    /// reads airborne, or a crumb seeds it. The governor caps conservatively while it is false, so a spurious 0
    /// cannot open the fins to the full 45 degrees at high dynamic pressure. A latch: once true, it stays.
    pub fn confident(&self) -> bool {
        self.confident
    }

    /// Restore a PERSISTED airspeed in m/s, the warm-start crumb's, as the starting estimate (airspeed.seed).
    ///
    /// Unlike [`AirspeedEstimator::measure`] and [`AirspeedEstimator::correct`], the value is not a live reading
    /// but the last one this airframe knew before the reset, so it is accepted whatever its magnitude -- including
    /// one under [`CONFIDENT_SPEED`], which measure() would refuse to trust (a 0 on the pad must never latch
    /// confidence). The CALLER has already established that a flight was in progress (the warm-start gate: a
    /// crumb, the separation latch, the baro above the pad), so a slow airspeed is a real slow airspeed.
    ///
    /// Confidence latches, so the governor caps off this value instead of the blunt unconfident floor. No over-read
    /// margin is added for the reboot gap: its length is not knowable to better than the crumb period, and
    /// inventing one would bias every recovery on a guess. A value below 0 or above the ceiling (garbage) is
    /// ignored. Returns the estimate.
    pub fn seed(&mut self, airspeed: f32) -> f32 {
        if 0.0 <= airspeed && airspeed <= self.ceiling {
            self.speed = airspeed;
            self.confident = true;
        }
        self.speed
    }

    /// Integrate the net acceleration ALONG the flight path, m/s^2, over `elapsed` seconds: the backbone
    /// (airspeed.predict).
    ///
    /// Clamped to [0, ceiling]; pass an acceleration that over-reads to stay conservative. A speed charged to
    /// [`CONFIDENT_SPEED`] or more latches confidence. Returns the estimate.
    pub fn predict(&mut self, acceleration_along: f32, elapsed: f32) -> f32 {
        let charged = self.speed + acceleration_along * elapsed;
        let bounded = if self.ceiling < charged { self.ceiling } else { charged };
        self.speed = if bounded > 0.0 { bounded } else { 0.0 };
        if self.speed >= CONFIDENT_SPEED {
            self.confident = true;
        }
        self.speed
    }

    /// Blend toward a GNSS ground speed in m/s ONLY if trustworthy: a live fix and within [0, ceiling]
    /// (airspeed.correct).
    ///
    /// Anything above the ceiling is a separation or dynamics glitch and is ignored. When already confident it
    /// BLENDS by the GNSS gain, so one in-range bad sample moves the estimate only slightly while a run of good
    /// fixes removes the integrator drift. When NOT yet confident (a fresh boot, a mid-air reset) the first
    /// accepted fix SEEDS it directly: a full set, not a 20 % crawl that would leave the fin cap open for seconds.
    /// No fix, or a speed out of range, keeps the integrated backbone untouched. Returns the estimate.
    pub fn correct(&mut self, gnss_speed: f32, has_fix: bool) -> f32 {
        if has_fix && 0.0 <= gnss_speed && gnss_speed <= self.ceiling {
            if self.confident {
                self.speed += self.gnss_gain * (gnss_speed - self.speed);
            } else {
                self.speed = gnss_speed;
            }
            self.confident = true;
        }
        self.speed
    }

    /// Fold in a DIRECT airspeed measurement in m/s, the pitot's -- the preferred source when available
    /// (airspeed.measure).
    ///
    /// Unlike the GNSS ground speed, the pitot measures the AIR: no wind offset, no attitude gate, so it anchors
    /// strongly. It seeds directly while not yet confident (a real measurement, not the slow GNSS crawl), and
    /// once confident blends by `gain` (set higher than the GNSS gain: it is truth, not a proxy). The CALLER
    /// vets the reading in band first: a saturated pitot under-reads, which would loosen the cap, the unsafe
    /// direction. Confidence latches only once the estimate clears [`CONFIDENT_SPEED`], so a 0 on the pad never
    /// trips the cap off an uncharged low speed. Returns the estimate.
    pub fn measure(&mut self, airspeed: f32, gain: f32) -> f32 {
        if self.confident {
            self.speed += gain * (airspeed - self.speed);
        } else {
            self.speed = airspeed;
        }
        if self.speed >= CONFIDENT_SPEED {
            self.confident = true;
        }
        self.speed
    }
}

#[cfg(test)]
mod tests {
    use super::AirspeedEstimator;

    /// Whether an estimate is within f32 rounding of the expected m/s.
    fn close(answer: f32, expected: f32) -> bool {
        (answer - expected).abs() < 1e-4
    }

    #[test]
    fn the_accelerometer_integrates_and_never_goes_negative() {
        let mut estimator = AirspeedEstimator::default();
        for _ in 0..50 {
            estimator.predict(49.0, 0.01);
        }
        assert!((24.0..25.0).contains(&estimator.value()), "{}", estimator.value());
        estimator.predict(-3000.0, 0.01);
        assert_eq!(estimator.value(), 0.0);
    }

    #[test]
    fn the_integral_clamps_at_the_ceiling() {
        let mut estimator = AirspeedEstimator::new(60.0, 0.2);
        for _ in 0..100 {
            estimator.predict(100.0, 0.1);
        }
        assert_eq!(estimator.value(), 60.0);
    }

    #[test]
    fn a_nan_acceleration_bounds_to_zero_as_in_python() {
        let mut estimator = AirspeedEstimator::default();
        estimator.seed(20.0);
        assert_eq!(estimator.predict(f32::NAN, 0.01), 0.0);
    }

    #[test]
    fn gnss_is_gated_then_blended_then_converges() {
        let mut estimator = AirspeedEstimator::new(60.0, 0.2);
        for _ in 0..40 {
            estimator.predict(50.0, 0.01);
        }
        let backbone = estimator.value();
        assert!((19.5..20.5).contains(&backbone), "{backbone}");
        assert_eq!(estimator.correct(14.0, false), backbone, "no fix: ignored");
        assert_eq!(
            estimator.correct(120.0, true),
            backbone,
            "above the ceiling: a glitch, ignored"
        );
        assert_eq!(estimator.correct(-1.0, true), backbone, "below zero: ignored");
        assert!(close(estimator.correct(10.0, true), backbone + 0.2 * (10.0 - backbone)));
        for _ in 0..60 {
            estimator.correct(10.0, true);
        }
        assert!((estimator.value() - 10.0).abs() < 0.5);
    }

    #[test]
    fn confidence_latches_on_a_charge_or_the_first_fix_which_seeds() {
        let mut estimator = AirspeedEstimator::new(60.0, 0.2);
        assert!(!estimator.confident());
        assert_eq!(estimator.value(), 0.0);
        estimator.predict(1.0, 0.1);
        assert!(!estimator.confident(), "0.1 m/s is not airborne");
        estimator.correct(24.0, true);
        assert!(estimator.confident());
        assert_eq!(estimator.value(), 24.0, "the first fix seeds, not a 20 % crawl to 4.88");
        assert!(close(estimator.correct(20.0, true), 23.2), "then fixes blend");
        let mut charged = AirspeedEstimator::default();
        for _ in 0..20 {
            charged.predict(49.0, 0.01);
        }
        assert!(charged.confident());
    }

    #[test]
    fn a_seed_restores_even_a_slow_speed_and_refuses_garbage() {
        let mut estimator = AirspeedEstimator::default();
        estimator.seed(16.5);
        assert!(estimator.confident());
        assert_eq!(estimator.value(), 16.5);
        let mut slow = AirspeedEstimator::default();
        slow.seed(3.0);
        assert!(slow.confident(), "a slow but real speed earns trust, unlike measure()");
        assert!(
            (slow.predict(9.81, 1.0) - 12.81).abs() < 1e-3,
            "the backbone integrates on from the seed"
        );
        assert!(
            close(estimator.measure(25.0, 1.0), 25.0),
            "the pitot still overrides the seed"
        );
        for garbage in [-1.0, 1e6, f32::NAN] {
            let mut fresh = AirspeedEstimator::new(60.0, 0.2);
            fresh.seed(garbage);
            assert!(!fresh.confident(), "{garbage}");
            assert_eq!(fresh.value(), 0.0);
        }
    }

    #[test]
    fn the_pitot_seeds_then_blends_and_a_pad_zero_earns_no_trust() {
        let mut estimator = AirspeedEstimator::default();
        estimator.measure(0.0, 0.5);
        assert_eq!(estimator.value(), 0.0);
        assert!(!estimator.confident());
        estimator.measure(15.0, 0.5);
        assert_eq!(estimator.value(), 15.0);
        assert!(estimator.confident());
        assert_eq!(estimator.measure(17.0, 0.5), 16.0, "a blend, not a re-seed to 17");
    }

    #[test]
    fn the_inclusive_edges_hold_on_their_exact_values() {
        // the latch is `>=`: a first pitot reading of exactly 5 m/s is trusted, just under it is not
        let mut latch = AirspeedEstimator::default();
        latch.measure(5.0, 0.5);
        assert!(latch.confident());
        let mut under = AirspeedEstimator::default();
        under.measure(4.99, 0.5);
        assert!(!under.confident());
        // a fix exactly at the ceiling is accepted (and seeds); half a m/s past it is a glitch
        let mut fix = AirspeedEstimator::new(60.0, 0.2);
        assert_eq!(fix.correct(60.5, true), 0.0);
        assert!(!fix.confident());
        assert_eq!(fix.correct(60.0, true), 60.0);
        assert!(fix.confident());
        // a seed exactly at the ceiling is restored; half a m/s past it is garbage
        let mut restored = AirspeedEstimator::new(60.0, 0.2);
        assert_eq!(restored.seed(60.5), 0.0);
        assert!(!restored.confident());
        assert_eq!(restored.seed(60.0), 60.0);
        assert!(restored.confident());
    }
}
