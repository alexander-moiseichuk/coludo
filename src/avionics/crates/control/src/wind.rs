//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Wind estimation from GNSS (src/glider wind.py). The MINIMAL method, proven first: the WIND TRIANGLE.
//!
//! The GNSS reports the ground velocity (course and ground speed); the attitude gives the heading; the governor
//! gives the airspeed. The air mass the glider flies through is moving, so
//!   wind = ground velocity - air velocity = (ground speed along the course) - (airspeed along the heading).
//! An EMA smooths the per-fix estimate. The ALONG-heading component inherits the governor's airspeed error (an
//! over-read biases the head/tailwind), which is acceptable for the rough uses (the reachability margin, the approach
//! crab, dead reckoning). The CROSSWIND component needs a heading independent of the track, and the board's attitude
//! filter pulls its heading onto the GNSS track (crates/control attitude `course_shift`): in straight flight its
//! crab is pulled out and the crosswind reads ~0; only the turns, where the track swings faster than the pull, show
//! it. Plan B's design, invisible in HITL while the sim's own heading was mirrored. If the field data shows that bias hurts, the
//! airspeed-free GPS-only min/max-ground-speed method is the next layer (kept out until a corner case earns it).
//!
//! The estimator OWNS its config subtree (the board `wind` block) and is operator-facing, as plan B's
//! Inspectable is: [`WindEstimator::inspect`] publishes `triangle_alpha` and the physical envelope
//! (`calm_speed`..`max_speed`), [`WindEstimator::update`] retunes them live, [`WindEstimator::stats`] reports
//! the estimate. It is fed once per GNSS fix, off the hot loop, through [`WindEstimator::observe`].
//!
//! Deliberate differences from plan B (the same ones crates/mission makes at the JSON boundary):
//!  * an `update` value that is not a number (text, a bool, null) is ignored and not listed as changed. Plan B
//!    stores whatever arrives: a text `max_speed` then raises on the next fix, and `True` flies as 1;
//!  * the changed names come in key order, because serde_json's map is sorted; plan B lists them in the order the
//!    request named them;
//!  * the tunables read back as JSON floats. Plan B echoes the JSON number it was given, so a `max_speed` set as
//!    `12` reads back `12` there and `12.0` here: the same value of another JSON kind.

use serde_json::{Map, Value, json};

/// The default EMA rate (wind.py `triangle_alpha`): each fix moves the estimate 5 % of the way to its own
/// triangle, a time constant of 20 fixes.
const TRIANGLE_ALPHA: f32 = 0.05;

/// The default calm floor in m/s: an estimate below it is estimate or GNSS noise, reported as calm.
const CALM_SPEED: f32 = 0.5;

/// The default envelope ceiling in m/s. Launches happen in at most ~10 m/s of wind, and the ceiling has spare for
/// a GNSS jump: a per-fix triangle beyond it is a course/speed jump, not wind, and is rejected.
const MAX_SPEED: f32 = 15.0;

/// Estimate the wind (east and north, m/s) from the GNSS ground velocity against the air velocity
/// (wind.WindEstimator): the wind triangle, EMA-smoothed.
#[derive(Clone, Debug, PartialEq)]
pub struct WindEstimator {
    /// The EMA wind blowing toward the east, m/s (wind.py `_we`).
    east: f32,
    /// The EMA wind blowing toward the north, m/s (wind.py `_wn`).
    north: f32,
    /// Whether a fix has been folded in: the first one seeds the EMA instead of blending into zero.
    seen: bool,
    /// The EMA rate: the fraction of the way each accepted fix moves the estimate.
    pub triangle_alpha: f32,
    /// The calm floor, m/s: an estimate slower than this reads as calm (0, 0).
    pub calm_speed: f32,
    /// The envelope ceiling, m/s: a per-fix triangle faster than this is a GNSS jump and is rejected.
    pub max_speed: f32,
}

impl WindEstimator {
    /// The operator-facing name `inspect`, `update` and `stats` find it by (wind.py `name`).
    pub const NAME: &'static str = "wind";

    /// Its category in the object list (wind.py `kind`).
    pub const KIND: &'static str = "wind";

    /// The properties `update` may change live (wind.py `_writable`).
    const WRITABLE: [&'static str; 3] = ["triangle_alpha", "calm_speed", "max_speed"];

    /// An estimator bound to the board's `wind` config block: `triangle_alpha`, `calm_speed` and `max_speed`.
    ///
    /// A key that is missing or not a number takes its default, and so does every key of a block that is not an
    /// object (plan B's `config or {}`). The block's `min_speed` is the flight task's (the ground speed below which
    /// a GNSS course is too noisy to feed in), not the estimator's.
    pub fn new(config: &Value) -> WindEstimator {
        let number = |key: &str, default: f32| {
            config
                .get(key)
                .and_then(Value::as_f64)
                .map_or(default, |value| value as f32)
        };
        WindEstimator {
            east: 0.0,
            north: 0.0,
            seen: false,
            triangle_alpha: number("triangle_alpha", TRIANGLE_ALPHA),
            calm_speed: number("calm_speed", CALM_SPEED),
            max_speed: number("max_speed", MAX_SPEED),
        }
    }

    /// Fold in one GNSS fix: this sample's wind triangle, EMA-blended into the estimate (wind.py `observe`).
    ///
    /// `course` is the GNSS ground-track course and `heading` the glider's heading, both in degrees; `ground_speed`
    /// is the GNSS ground speed and `airspeed` the governor's estimate, both in m/s.
    ///
    /// A triangle that is not a number is rejected before the envelope test, because the envelope test cannot see
    /// it: every comparison with a NaN is false, so `> max_speed^2` would admit it, the EMA would be NaN for the
    /// rest of the flight, and dead reckoning would advect a NaN position with no error and no way back. Cheap to
    /// guard and impossible to recover from. An infinite triangle needs no test of its own: its square exceeds the
    /// envelope, so the two tests together are total over the non-finite cases. A triangle beyond the envelope is
    /// a GNSS jump, not wind, and leaves the estimate as it was.
    pub fn observe(&mut self, course: f32, ground_speed: f32, airspeed: f32, heading: f32) {
        let course_radians = course.to_radians();
        let heading_radians = heading.to_radians();
        let wind_east = ground_speed * course_radians.sin() - airspeed * heading_radians.sin();
        let wind_north = ground_speed * course_radians.cos() - airspeed * heading_radians.cos();
        if wind_east.is_nan() || wind_north.is_nan() {
            return;
        }
        if wind_east * wind_east + wind_north * wind_north > self.max_speed * self.max_speed {
            return;
        }
        if self.seen {
            self.east += self.triangle_alpha * (wind_east - self.east);
            self.north += self.triangle_alpha * (wind_north - self.north);
        } else {
            (self.east, self.north, self.seen) = (wind_east, wind_north, true);
        }
    }

    /// The wind as `(east, north)` m/s, the direction it blows TOWARD; `(0, 0)` below the calm floor (wind.py
    /// `components`). What dead reckoning adds to the air velocity, and what reachability projects.
    pub fn components(&self) -> (f32, f32) {
        if self.east * self.east + self.north * self.north < self.calm_speed * self.calm_speed {
            return (0.0, 0.0);
        }
        (self.east, self.north)
    }

    /// The wind speed in m/s, 0 below the calm floor (wind.py `speed`).
    pub fn speed(&self) -> f32 {
        let (east, north) = self.components();
        (east * east + north * north).sqrt()
    }

    /// Where the wind blows FROM, the meteorological convention, in degrees: 0 = from the north, clockwise, in
    /// [0, 360]; 0 when calm (wind.py `direction`).
    pub fn direction(&self) -> f32 {
        let (east, north) = self.components();
        if east != 0.0 || north != 0.0 {
            (-east).atan2(-north).to_degrees().rem_euclid(360.0)
        } else {
            0.0
        }
    }

    /// The operator-facing snapshot (`inspect wind`; wind.py `inspect`): the tunables, the speed to the centimetre
    /// per second and the direction to the degree, as plan B rounds them.
    pub fn inspect(&self) -> Value {
        json!({
            "triangle_alpha": json_number(self.triangle_alpha),
            "calm_speed": json_number(self.calm_speed),
            "max_speed": json_number(self.max_speed),
            "speed": hundredths(self.speed()),
            "from": whole_degrees(self.direction()),
        })
    }

    /// Apply a live retune (`update wind`; plan B's Inspectable `update`): each writable property whose number
    /// differs from the current value is set. Returns the names actually changed (see the module notes for the
    /// order and the values that are ignored).
    pub fn update(&mut self, properties: &Map<String, Value>) -> Vec<&'static str> {
        let mut changed = Vec::new();
        for (key, value) in properties {
            let Some(name) = Self::WRITABLE.into_iter().find(|name| name == key) else {
                continue;
            };
            let Some(number) = value.as_f64().map(|number| number as f32) else {
                continue;
            };
            let field = match name {
                "triangle_alpha" => &mut self.triangle_alpha,
                "calm_speed" => &mut self.calm_speed,
                _ => &mut self.max_speed,
            };
            if *field != number {
                *field = number;
                changed.push(name);
            }
        }
        changed
    }

    /// The diagnostics for the wind soak and telemetry (`stats wind`; wind.py `stats`): the method ('triangle'
    /// once a fix was folded in, else 'none'), the estimate, and the raw EMA components before the calm floor.
    pub fn stats(&self) -> Value {
        json!({
            "method": if self.seen { "triangle" } else { "none" },
            "speed": hundredths(self.speed()),
            "from": whole_degrees(self.direction()),
            "we": hundredths(self.east),
            "wn": hundredths(self.north),
        })
    }
}

/// `value` as a JSON number with its shortest f32 text, so 0.05 reads back 0.05 and not 0.0500000007 (the f64 that
/// f32 holds). JSON numbers are f64 only at this boundary (decision D18).
fn json_number(value: f32) -> Value {
    value
        .to_string()
        .parse::<f64>()
        .ok()
        .and_then(serde_json::Number::from_f64)
        .map_or(Value::Null, Value::Number)
}

/// `value` rounded to hundredths, as plan B's `round(value, 2)` answers: the f64 nearest to k/100, ties to even.
///
/// The rounding is done in f64, the JSON boundary's type, so the answer is exactly the float Python's round gives
/// (k/100 correctly rounded). An f32 hundredth widened afterwards would print as 4.989999771 for 4.99.
fn hundredths(value: f32) -> f64 {
    (f64::from(value) * 100.0).round_ties_even() / 100.0
}

/// `degrees` rounded to a whole degree, ties to even, as plan B's `round(direction)` answers (an integer).
fn whole_degrees(degrees: f32) -> i32 {
    degrees.round_ties_even() as i32
}

#[cfg(test)]
mod tests {
    use super::{WindEstimator, hundredths};
    use serde_json::{Value, json};

    /// The `(course, ground_speed)` a GNSS reads for the air velocity (airspeed along heading) plus the wind
    /// (test_wind `_ground`).
    fn ground(airspeed: f32, heading: f32, wind_east: f32, wind_north: f32) -> (f32, f32) {
        let radians = heading.to_radians();
        let east = airspeed * radians.sin() + wind_east;
        let north = airspeed * radians.cos() + wind_north;
        (
            east.atan2(north).to_degrees().rem_euclid(360.0),
            (east * east + north * north).sqrt(),
        )
    }

    /// An estimator with no EMA lag, so one fix is the estimate.
    fn instant(extra: Value) -> WindEstimator {
        let mut config = json!({"triangle_alpha": 1.0});
        config
            .as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        WindEstimator::new(&config)
    }

    #[test]
    fn still_air_reads_calm() {
        let mut estimator = instant(json!({}));
        let (course, speed) = ground(14.0, 0.0, 0.0, 0.0);
        estimator.observe(course, speed, 14.0, 0.0);
        assert!(estimator.speed() < 0.05);
    }

    #[test]
    fn an_east_wind_is_recovered_from_the_west() {
        let mut estimator = instant(json!({}));
        let (course, speed) = ground(14.0, 0.0, 5.0, 0.0);
        estimator.observe(course, speed, 14.0, 0.0);
        let (east, north) = estimator.components();
        assert!((east - 5.0).abs() < 0.1 && north.abs() < 0.1);
        assert!((estimator.speed() - 5.0).abs() < 0.1);
        assert!((estimator.direction() - 270.0).abs() < 1.0);
        assert_eq!(estimator.stats()["method"], "triangle");
    }

    #[test]
    fn the_crosswind_is_bias_free() {
        // an airspeed over-read of 2 m/s lands wholly in the along-heading (north) component
        let mut estimator = instant(json!({}));
        let (course, speed) = ground(14.0, 0.0, 5.0, 0.0);
        estimator.observe(course, speed, 16.0, 0.0);
        let (east, north) = estimator.components();
        assert!((east - 5.0).abs() < 0.1);
        assert!((north + 2.0).abs() < 0.1);
    }

    #[test]
    fn a_gnss_jump_beyond_the_envelope_is_rejected() {
        let mut estimator = instant(json!({"max_speed": 15.0}));
        let (course, speed) = ground(14.0, 0.0, 5.0, 0.0);
        estimator.observe(course, speed, 14.0, 0.0);
        estimator.observe(200.0, 99.0, 14.0, 0.0);
        assert!((estimator.speed() - 5.0).abs() < 0.1);
    }

    #[test]
    fn non_finite_fixes_never_poison_the_estimate() {
        let mut estimator = instant(json!({}));
        let (course, speed) = ground(14.0, 0.0, 5.0, 0.0);
        estimator.observe(course, speed, 14.0, 0.0);
        estimator.observe(f32::NAN, speed, 14.0, 0.0);
        estimator.observe(course, f32::INFINITY, 14.0, 0.0);
        estimator.observe(course, speed, f32::NEG_INFINITY, 0.0);
        // due north at an infinite speed: inf x sin(0) is a NaN east, inf the north -- one NaN is enough
        estimator.observe(0.0, f32::INFINITY, 14.0, 0.0);
        assert!((estimator.speed() - 5.0).abs() < 0.1, "{:?}", estimator.components());
        // and before any fix, a NaN does not even count as the first one
        let mut fresh = instant(json!({}));
        fresh.observe(f32::NAN, 10.0, 14.0, 0.0);
        assert_eq!(fresh.stats()["method"], "none");
    }

    #[test]
    fn a_wind_below_the_calm_floor_reads_calm() {
        let mut estimator = instant(json!({"calm_speed": 0.5}));
        let (course, speed) = ground(14.0, 0.0, 0.2, 0.0);
        estimator.observe(course, speed, 14.0, 0.0);
        assert_eq!(estimator.components(), (0.0, 0.0));
        assert_eq!((estimator.speed(), estimator.direction()), (0.0, 0.0));
        // the raw EMA still holds it, for the soak
        assert_eq!(estimator.stats()["we"], 0.2);
    }

    #[test]
    fn the_ema_moves_alpha_of_the_way() {
        let mut estimator = WindEstimator::new(&json!({"triangle_alpha": 0.25}));
        let (course, speed) = ground(14.0, 90.0, 0.0, 4.0);
        estimator.observe(course, speed, 14.0, 90.0);
        let (course, speed) = ground(14.0, 90.0, 0.0, 8.0);
        estimator.observe(course, speed, 14.0, 90.0);
        let (_, north) = estimator.components();
        assert!((north - 5.0).abs() < 1e-4, "{north}");
    }

    #[test]
    fn inspect_publishes_the_tunables_and_update_retunes_them() {
        let mut estimator = WindEstimator::new(&json!({"triangle_alpha": 0.05, "max_speed": 15.0}));
        assert_eq!(
            estimator.inspect(),
            json!({"triangle_alpha": 0.05, "calm_speed": 0.5, "max_speed": 15.0, "speed": 0.0, "from": 0})
        );
        let changed = estimator.update(
            json!({"max_speed": 12.0, "calm_speed": 1.0, "triangle_alpha": 0.05, "speed": 3.0})
                .as_object()
                .unwrap(),
        );
        assert_eq!(changed, ["calm_speed", "max_speed"]);
        assert_eq!((estimator.max_speed, estimator.calm_speed), (12.0, 1.0));
    }

    #[test]
    fn update_ignores_what_is_not_a_number() {
        let mut estimator = WindEstimator::new(&Value::Null);
        let changed = estimator.update(
            json!({"max_speed": "12", "calm_speed": true, "triangle_alpha": null})
                .as_object()
                .unwrap(),
        );
        assert!(changed.is_empty());
        assert_eq!(estimator, WindEstimator::new(&json!({})));
    }

    #[test]
    fn a_config_block_that_is_not_an_object_takes_the_defaults() {
        let defaults = WindEstimator::new(&json!({}));
        assert_eq!(
            (defaults.triangle_alpha, defaults.calm_speed, defaults.max_speed),
            (0.05, 0.5, 15.0)
        );
        assert_eq!(WindEstimator::new(&json!([1, 2])), defaults);
        assert_eq!(WindEstimator::new(&json!({"max_speed": "fast"})), defaults);
    }

    #[test]
    fn hundredths_round_as_python_does() {
        assert_eq!(hundredths(4.99), 4.99);
        assert_eq!(hundredths(0.125), 0.12); // an exact tie goes to even, as round(0.125, 2) does
        assert_eq!(hundredths(-2.0), -2.0);
    }
}
