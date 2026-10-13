//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the guidance law, after src/glider/test/test_guidance.py: the control-stage gate, the boost rod
//! gate and hold, the GPS-degrading tiers with the navigation cache and dead reckoning, bank-to-turn against the
//! full-authority land bank, final approach, the loiter orbit and the endgame patterns, the steering filter,
//! the turn physics and reachability. The golden fixture (tests/golden_guidance.rs) holds the numbers to plan B;
//! these hold the rules, each with what must NOT happen.

use super::{
    Fix, Guidance, GuidanceConfig, Heading, Inputs, Landing, Setpoint, SteeringFilter, heading_error, keep_turn,
};
use crate::navigation::{self, Position};
use sequencer::Stage;
use serde_json::{Value, json};

/// The position at these decimal degrees.
fn at(latitude: f64, longitude: f64) -> Position {
    Position::from_degrees(latitude, longitude)
}

/// test_guidance's zone: 0.001 degree tall by 0.010 wide near 48 N, so gates on the left (gate_a, west) and
/// right (gate_b, east) edges. Its aspect is ~6.7, which the Mission resolves 'auto' to two lobes for.
fn strip() -> Landing {
    Landing::new(at(48.001, 11.000), at(48.000, 11.010), Heading::TwoLobes)
}

/// A point east of the strip, past its right gate: steering heads ~270.
fn east_of_strip() -> Position {
    at(48.0005, 11.020)
}

/// A point west of the strip, before its left gate: steering heads ~90.
fn west_of_strip() -> Position {
    at(48.0005, 10.990)
}

/// A guidance law over `config` (the flight task's GNSS window of 1000 ms).
fn law(config: Value) -> Guidance {
    Guidance::new(GuidanceConfig::from_json(&config, 1000))
}

/// Inputs with a zone and a fresh fix at `position`, nothing else.
fn fixed_at(position: Position) -> Inputs {
    Inputs {
        landing: Some(strip()),
        position: Some(Fix { position, age_ms: 0 }),
        ..Inputs::default()
    }
}

/// Whether two headings agree within `tolerance` degrees, the short way around.
fn close(first: f32, second: f32, tolerance: f32) -> bool {
    let around = (first - second).rem_euclid(360.0);
    around.min(360.0 - around) <= tolerance
}

#[test]
fn heading_error_takes_the_short_way() {
    assert_eq!(heading_error(10.0, 350.0), 20);
    assert_eq!(heading_error(350.0, 10.0), -20);
    assert_eq!(heading_error(100.0, 100.0), 0);
    // the half turn keeps its sign either way
    assert_eq!(heading_error(180.0, 0.0), 180);
    assert_eq!(heading_error(0.0, 180.0), -180);
    // truncation toward zero, then the wrap
    assert_eq!(heading_error(10.75, 0.0), 10);
    assert_eq!(heading_error(0.0, 10.75), -10);
    assert_eq!(heading_error(0.0, 359.5), 1);
    assert_eq!(heading_error(-540.5, 0.0), -180);
    // a NaN is no error at all, never a panic
    assert_eq!(heading_error(f32::NAN, 0.0), 0);
}

#[test]
fn patterns_resolve_from_config_names() {
    assert_eq!(Heading::resolve("auto"), Heading::Auto);
    assert_eq!(Heading::resolve("O"), Heading::Circle);
    assert_eq!(Heading::resolve("o-o"), Heading::TwoLobes);
    assert_eq!(Heading::resolve("OV"), Heading::Oval);
    assert_eq!(Heading::TwoLobes.name(), "oo");
    // an unknown name lets the Mission decide
    assert_eq!(Heading::resolve("figure-8"), Heading::Auto);
    assert_eq!(Heading::resolve(""), Heading::Auto);
    assert_eq!(Heading::resolve("ooo"), Heading::Auto);
}

#[test]
fn auto_flies_the_pattern_the_mission_resolved() {
    // the Mission decides 'auto' (crates/mission): the law flies whatever the Landing carries, and a Landing
    // that carries 'auto' itself circles rather than asking again
    let near = |pattern: Heading| Inputs {
        landing: Some(Landing::new(at(48.001, 11.000), at(48.000, 11.010), pattern)),
        elevation: Some(25.0),
        airspeed: 14.0,
        position: Some(Fix {
            position: navigation::advance(strip().target, 45.0, -20.0),
            age_ms: 0,
        }),
        ..Inputs::default()
    };
    let target = |pattern: Heading| {
        let mut guidance = law(json!({"endgame_pattern": "auto"}));
        guidance.enter(0.5, 0.0, 0.0);
        guidance.compute(Stage::Gliding, 0.5, 0, &near(pattern));
        guidance.target_heading().unwrap()
    };
    let configured = |name: &str| {
        let mut guidance = law(json!({ "endgame_pattern": name }));
        guidance.enter(0.5, 0.0, 0.0);
        guidance.compute(Stage::Gliding, 0.5, 0, &near(Heading::TwoLobes));
        guidance.target_heading().unwrap()
    };
    assert_eq!(target(Heading::Circle), configured("o"));
    assert_eq!(target(Heading::Oval), configured("ov"));
    assert_eq!(target(Heading::TwoLobes), configured("oo"));
    assert_eq!(target(Heading::Auto), configured("o"));
    // NEGATIVE: a configured pattern wins over the Mission's
    assert_ne!(configured("o"), configured("oo"));
}

#[test]
fn the_fallbacks_match_what_config_default_ships() {
    // a partial config must fly the law the panel reports: bank_limit 45 (not the 30 that limit-cycled),
    // land_bank_gain 3.0 (not the 1.5 that froze the spiral at ~44 m)
    let empty = GuidanceConfig::from_json(&json!({}), 1000);
    assert_eq!(
        (empty.bank_limit, empty.land_bank_gain, empty.land_bank_limit),
        (45.0, 3.0, 45.0)
    );
    assert_eq!((empty.loiter_radius_m, empty.endgame_altitude_m), (30.0, 50.0));
    assert_eq!((empty.steer_filter_shift, empty.navigation_period_us), (3, 100_000));
    assert_eq!(empty.position_age_max_ms, 1000);
    assert_eq!(empty.endgame_pattern, Heading::Auto);
}

#[test]
fn only_configured_stages_are_control_stages() {
    let empty = law(json!({}));
    assert_eq!(empty.setpoint(Stage::Gliding), Some(Setpoint::default()));
    assert_eq!(empty.setpoint(Stage::Boosting), None);
    assert_eq!(empty.setpoint(Stage::Setting), None);
    let staged = law(json!({"stages": {"gliding": {"pitch": 0}, "landing": {"pitch": -2}, "Cruise": {}}}));
    assert_eq!(
        staged.setpoint(Stage::Landing),
        Some(Setpoint { roll: 0.0, pitch: -2.0 })
    );
    assert_eq!(staged.setpoint(Stage::Done), None);
    // an empty stage table means no control stage at all
    assert_eq!(law(json!({"stages": {}})).setpoint(Stage::Gliding), None);
}

#[test]
fn wrong_config_types_take_their_defaults() {
    let config = GuidanceConfig::from_json(
        &json!({"bank_limit": "steep", "steer_filter_shift": -1, "endgame_pattern": 2, "stages": "gliding"}),
        700,
    );
    assert_eq!(config.bank_limit, 45.0);
    assert_eq!(config.steer_filter_shift, 3);
    assert_eq!(config.endgame_pattern, Heading::Auto);
    assert_eq!(
        Guidance::new(config.clone()).setpoint(Stage::Gliding),
        Some(Setpoint::default())
    );
    assert_eq!(config.position_age_max_ms, 700);
    // a fractional period keeps `elapsed < period` for an integer clock: it rounds UP, even below the half
    // (plan B's `age < 250.2` takes an age of 250, which a period rounded to 250 would refuse)
    let fractional = GuidanceConfig::from_json(&json!({"nav_period_ms": 99.9992, "position_age_max_ms": 250.2}), 1000);
    assert_eq!(
        (fractional.navigation_period_us, fractional.position_age_max_ms),
        (100_000, 251)
    );
}

#[test]
fn a_present_null_or_false_switches_a_feature_off_and_only_an_absent_key_defaults() {
    // plan B stores the value and gates on its truth (`config.final_agl and ...`, `if not shift`): a null or
    // false there switches the feature OFF, never back on to the default
    let every_switch = |value: &Value| {
        GuidanceConfig::from_json(
            &json!({"nav_bank_gain": value, "land_bank_gain": value, "endgame_alt_m": value,
                "final_approach_agl": value, "steer_filter_shift": value}),
            1000,
        )
    };
    let switches = |config: GuidanceConfig| {
        (
            config.bank_gain,
            config.land_bank_gain,
            config.endgame_altitude_m,
            config.final_approach_agl,
            config.steer_filter_shift,
        )
    };
    for off in [
        json!(null),
        json!(false),
        json!(0),
        json!(0.0),
        json!(""),
        json!([]),
        json!({}),
    ] {
        assert_eq!(switches(every_switch(&off)), (0.0, 0.0, 0.0, 0.0, 0), "{off}");
    }
    // true flies as 1, as a Python bool does in the comparison, the product and the shift
    assert_eq!(switches(every_switch(&json!(true))), (1.0, 1.0, 1.0, 1.0, 1));
    // absent: the shipped defaults, which switch every feature on
    assert_eq!(
        switches(GuidanceConfig::from_json(&json!({}), 1000)),
        (1.5, 3.0, 50.0, 8.0, 3)
    );
    // a whole float is a shift (plan B's `>>` would raise on it; 0.0 is the falsy off above)
    let float_shift = GuidanceConfig::from_json(&json!({"steer_filter_shift": 4.0}), 1000);
    assert_eq!(float_shift.steer_filter_shift, 4);
    // NEGATIVE: what plan B cannot fly at all (a non-empty text or list, a fraction or a negative shift) takes
    // the default, never the off switch
    for wrong in [json!("8"), json!([8]), json!({"agl": 8})] {
        assert_eq!(switches(every_switch(&wrong)), (1.5, 3.0, 50.0, 8.0, 3), "{wrong}");
    }
    for wrong in [json!(2.5), json!(-1), json!(-2.0), json!(1e12)] {
        let config = GuidanceConfig::from_json(&json!({ "steer_filter_shift": wrong }), 1000);
        assert_eq!(config.steer_filter_shift, 3, "{wrong}");
    }
}

#[test]
fn the_filter_seeds_smooths_and_follows_a_real_change() {
    let mut filter = SteeringFilter::new(3);
    assert_eq!(filter.filter(40), 40);
    assert_eq!(filter.filter(48), 41); // +8 of jitter moves an eighth of the way
    let mut settled = 0;
    for _ in 0..60 {
        settled = filter.filter(48);
    }
    assert!((settled - 48).abs() <= 1);
    assert_eq!(filter.filter(-120), -120); // a flip over 90 degrees is followed at once
    let mut off = SteeringFilter::new(0);
    assert_eq!((off.filter(40), off.filter(48)), (40, 48));
}

#[test]
fn the_filter_has_no_bias_on_a_symmetric_oscillation() {
    // floor division would leave -0.5 degree of steady output for a +/-8 input whose true mean is zero
    let mut filter = SteeringFilter::new(3);
    let outputs: Vec<i32> = (0..600)
        .map(|step| filter.filter(if step % 2 == 0 { 8 } else { -8 }))
        .collect();
    let settled = &outputs[500..];
    let bias = settled.iter().sum::<i32>() as f32 / settled.len() as f32;
    assert!(bias.abs() < 0.01, "biased by {bias}");
    // and a constant it was seeded with stays exactly itself, in both signs
    for constant in [1, -1, 5, -5, 12, -12, 45, -45] {
        let mut steady = SteeringFilter::new(3);
        let mut last = 0;
        for _ in 0..80 {
            last = steady.filter(constant);
        }
        assert_eq!(last, constant);
    }
}

#[test]
fn a_constant_approached_upward_settles_one_degree_short_as_in_plan_b() {
    // Plan B's behaviour, ported as it is: the arithmetic shift floors, so a positive step under 2^shift
    // sixteenths moves the state by nothing, and it stalls up to 7/16 below a target it approaches from below;
    // the truncation then reads one degree short. A negative step always moves at least one sixteenth, so from
    // above it arrives exactly. (test_guidance's own check allows the degree: |smoothed - 48| <= 1.)
    let settle = |seed: i32, constant: i32| {
        let mut filter = SteeringFilter::new(3);
        filter.filter(seed);
        (0..200).map(|_| filter.filter(constant)).last().unwrap()
    };
    assert_eq!(settle(40, 48), 47);
    assert_eq!(settle(0, 5), 4);
    assert_eq!(settle(0, 1), 0);
    assert_eq!(settle(10, 5), 5);
    assert_eq!(settle(0, -5), -5);
    assert_eq!(settle(-10, -5), -5);
}

#[test]
fn a_jump_of_exactly_ninety_degrees_is_still_smoothed() {
    // the re-seed takes a jump OVER 90 degrees (plan B's `abs(jump) > 90 * 16`): exactly 90 (1440 sixteenths)
    // moves an eighth of the way, 91 is followed at once; the same either way round
    for (error, answer) in [(90, 11), (91, 91), (-90, -11), (-91, -91)] {
        let mut filter = SteeringFilter::new(3);
        assert_eq!(filter.filter(0), 0);
        assert_eq!(filter.filter(error), answer, "0 then {error}");
    }
}

#[test]
fn a_reset_reseeds_and_a_huge_shift_never_panics() {
    let mut filter = SteeringFilter::new(3);
    filter.filter(10);
    filter.reset();
    assert_eq!(filter.filter(-50), -50);
    let mut frozen = SteeringFilter::new(200);
    assert_eq!(frozen.filter(10), 10);
    assert_eq!(frozen.filter(30), 10);
    assert_eq!(frozen.filter(-20), 9); // the shift leaves -1 sixteenth: rounded toward zero, 9
}

#[test]
fn boost_engages_only_past_the_rod_and_holds_the_captured_attitude() {
    let config = json!({"stages": {"boosting": {}, "gliding": {}}, "boost_engage_speed": 15.0});
    let slots = |guidance: &Guidance| {
        (
            guidance.roll_setpoint(),
            guidance.pitch_setpoint(),
            guidance.heading_error(),
        )
    };
    let mut guidance = law(config.clone());
    guidance.enter(0.5, 12.5, 90.0); // the rod holds the airframe 12.5 degrees rolled
    let on_the_rod = Inputs {
        airspeed: 5.0,
        ..Inputs::default()
    };
    assert!(!guidance.compute(Stage::Boosting, 0.5, 0, &on_the_rod));
    assert_eq!(slots(&guidance), (0.0, 0.0, 0), "neutral leaves the slots untouched");
    // a gliding step leaves its own bank and heading error in the slots: the strip's left gate is ~90 degrees
    // off a 0.5 heading, so the bank hits its 45 cap
    let gliding = Inputs {
        airspeed: 30.0,
        ..fixed_at(west_of_strip())
    };
    guidance.compute(Stage::Gliding, 0.5, 10_000, &gliding);
    assert_eq!(slots(&guidance), (45.0, 0.0, 89));
    // past the rod the boost restores the CAPTURED rod attitude and zeroes the heading error, whatever the last
    // law left: no navigation or yaw steering near vertical
    let past_the_rod = Inputs {
        airspeed: 30.0,
        ..Inputs::default()
    };
    assert!(guidance.compute(Stage::Boosting, 123.0, 20_000, &past_the_rod));
    assert_eq!(slots(&guidance), (12.5, 90.0, 0));
    // exactly at the engage speed it engages: plan B holds neutral only BELOW it (`airspeed < boost_engage`)
    let mut edge = law(config);
    edge.enter(0.5, 12.5, 90.0);
    let below = Inputs {
        airspeed: 14.999,
        ..Inputs::default()
    };
    assert!(!edge.compute(Stage::Boosting, 0.5, 0, &below));
    let at_the_speed = Inputs {
        airspeed: 15.0,
        ..Inputs::default()
    };
    assert!(edge.compute(Stage::Boosting, 0.5, 0, &at_the_speed));
    assert_eq!(slots(&edge), (12.5, 90.0, 0));
}

#[test]
fn the_heading_tiers_degrade_with_the_gnss() {
    // tier 4: no fix, no launch point -> the captured heading
    let mut blind = law(json!({}));
    blind.enter(200.0, 0.0, 0.0);
    let zone_only = Inputs {
        landing: Some(strip()),
        ..Inputs::default()
    };
    assert!(blind.compute(Stage::Gliding, 0.5, 0, &zone_only));
    assert_eq!(blind.heading_error(), heading_error(200.0, 0.5));
    assert!(!blind.reckoning());
    // tier 3: never a fix, a launch point west of the zone -> the left gate, ~east
    let mut guidance = law(json!({}));
    guidance.enter(200.0, 0.0, 0.0);
    let launch = Inputs {
        launch: Some(west_of_strip()),
        ..zone_only
    };
    guidance.compute(Stage::Gliding, 0.5, 0, &launch);
    assert!(close(guidance.target_heading().unwrap(), 90.0, 5.0));
    // tier 1: a fresh fix east of the zone overrides -> ~west
    let fix = Inputs {
        launch: Some(west_of_strip()),
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 0.5, 100_000, &fix);
    assert!(close(guidance.target_heading().unwrap(), 270.0, 5.0));
    // a fix past the gate: tier 2 reckons on from the last fix (still ~west), NOT back to the launch point
    let old = Inputs {
        position: Some(Fix {
            position: east_of_strip(),
            age_ms: 1500,
        }),
        ..fix
    };
    guidance.compute(Stage::Gliding, 0.5, 200_000, &old);
    assert!(guidance.reckoning());
    assert!(close(guidance.target_heading().unwrap(), 270.0, 5.0));
    // the gate is strict: a fix one millisecond inside the window steers, one exactly as old as it reckons
    let mut edge = law(json!({}));
    edge.enter(200.0, 0.0, 0.0);
    let aged = |age_ms: u32| Inputs {
        position: Some(Fix {
            position: east_of_strip(),
            age_ms,
        }),
        ..fix
    };
    edge.compute(Stage::Gliding, 0.5, 0, &aged(999));
    assert!(!edge.reckoning());
    edge.compute(Stage::Gliding, 0.5, 100_000, &aged(1000));
    assert!(
        edge.reckoning(),
        "an age of exactly position_age_max_ms is past the gate"
    );
}

#[test]
fn no_zone_holds_the_captured_heading_whatever_the_fix() {
    let mut guidance = law(json!({}));
    guidance.enter(200.0, 0.0, 0.0);
    let no_zone = Inputs {
        landing: None,
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 0.5, 0, &no_zone);
    assert_eq!(guidance.heading_error(), heading_error(200.0, 0.5));
    assert_eq!(guidance.target_heading(), None, "nothing is cached without a zone");
}

#[test]
fn the_target_is_cached_for_the_navigation_period() {
    let mut guidance = law(json!({"nav_period_ms": 100}));
    guidance.enter(200.0, 0.0, 0.0);
    guidance.compute(Stage::Gliding, 0.5, 0, &fixed_at(east_of_strip()));
    let first = guidance.target_heading().unwrap();
    let moved = fixed_at(at(48.0005, 10.980));
    guidance.compute(Stage::Gliding, 0.5, 50_000, &moved);
    assert_eq!(guidance.target_heading(), Some(first), "50 ms is inside the period");
    guidance.compute(Stage::Gliding, 0.5, 150_000, &moved);
    assert_ne!(guidance.target_heading(), Some(first), "past the period it recomputes");
    // a clock that reads earlier stays cached, as plan B's signed tick difference does
    let recomputed = guidance.target_heading();
    guidance.compute(Stage::Gliding, 0.5, 100_000, &fixed_at(east_of_strip()));
    assert_eq!(guidance.target_heading(), recomputed);
    guidance.enter(200.0, 0.0, 0.0);
    assert_eq!(guidance.target_heading(), None, "entering control drops the cache");
}

#[test]
fn gliding_banks_to_turn_and_landing_takes_the_full_authority() {
    let mut guidance = law(json!({"nav_bank_gain": 1.5, "bank_limit": 30, "stages": {"gliding": {}, "landing": {}}}));
    guidance.enter(0.5, 0.0, 0.0);
    guidance.compute(Stage::Gliding, 0.5, 0, &fixed_at(west_of_strip()));
    assert_eq!(guidance.roll_setpoint(), 30.0, "+89 degrees of error hits the 30 cap");
    guidance.compute(Stage::Landing, 0.5, 100_000, &fixed_at(west_of_strip()));
    assert_eq!(guidance.roll_setpoint(), 45.0, "LANDING: the land gain and the 45 cap");
    // gain 0 steers on the rudder alone: the configured roll holds
    let mut flat = law(json!({"nav_bank_gain": 0, "stages": {"gliding": {"roll": 2.0}}}));
    flat.enter(0.5, 0.0, 0.0);
    flat.compute(Stage::Gliding, 0.5, 0, &fixed_at(west_of_strip()));
    assert_eq!(flat.roll_setpoint(), 2.0);
}

#[test]
fn a_turn_near_the_half_turn_keeps_its_direction_across_the_wrap() {
    // both within 30 of 180, on opposite sides: the turn in progress goes on, as +-180
    assert_eq!(keep_turn(179, -178, 30), 180);
    assert_eq!(keep_turn(-170, 165, 30), -180);
    assert_eq!(keep_turn(180, -180, 30), 180);
    // NEGATIVE: one side out of the band, or the same side, is the error itself
    assert_eq!(keep_turn(149, -178, 30), -178, "the last error was no half turn");
    assert_eq!(keep_turn(179, -149, 30), -149, "this one is no half turn");
    assert_eq!(keep_turn(170, 160, 30), 160);
    assert_eq!(keep_turn(0, -180, 30), -180);
    assert_eq!(keep_turn(179, -178, 0), -178, "a band of 0 is plan B's shortest turn");
    assert_eq!(
        GuidanceConfig::from_json(&json!({}), 1000).turn_wrap_band,
        45,
        "on by default"
    );
    // through the law (the steering filter off: what is under test is the latch after it): the zone behind the
    // glider, its heading wobbling across the wrap
    let config = json!({"nav_bank_gain": 1.5, "bank_limit": 30, "steer_filter_shift": 0, "stages": {"gliding": {}}});
    let mut guidance = law(config);
    guidance.enter(0.5, 0.0, 0.0);
    guidance.compute(Stage::Gliding, 0.5, 0, &fixed_at(west_of_strip()));
    let target = guidance.target_heading().unwrap();
    let right = (target + 181.5).rem_euclid(360.0); // 178-179 degrees to the right
    let left = (target + 177.5).rem_euclid(360.0); // 177-178 degrees to the left
    guidance.compute(Stage::Gliding, right, 0, &fixed_at(west_of_strip()));
    assert!(guidance.heading_error() >= 150 && guidance.roll_setpoint() == 30.0);
    guidance.compute(Stage::Gliding, left, 0, &fixed_at(west_of_strip()));
    assert_eq!(
        (guidance.heading_error(), guidance.roll_setpoint()),
        (180, 30.0),
        "the wobble does not flip the bank"
    );
    // NEGATIVE: out of the band the latch is gone, and the same wobble then turns left
    let quarter = (target - 120.0).rem_euclid(360.0);
    guidance.compute(Stage::Gliding, quarter, 0, &fixed_at(west_of_strip()));
    guidance.compute(Stage::Gliding, left, 0, &fixed_at(west_of_strip()));
    assert!(guidance.heading_error() <= -150 && guidance.roll_setpoint() == -30.0);
    // and a control entry carries no turn over
    guidance.enter(0.5, 0.0, 0.0);
    guidance.compute(Stage::Gliding, right, 0, &fixed_at(west_of_strip()));
    assert_eq!(guidance.roll_setpoint(), 30.0);
    // NEGATIVE: turn_wrap_band_deg 0 flies plan B's flip
    let mut plan_b = law(json!({"nav_bank_gain": 1.5, "bank_limit": 30, "steer_filter_shift": 0,
                                "turn_wrap_band_deg": 0, "stages": {"gliding": {}}}));
    plan_b.enter(0.5, 0.0, 0.0);
    plan_b.compute(Stage::Gliding, right, 0, &fixed_at(west_of_strip()));
    plan_b.compute(Stage::Gliding, left, 0, &fixed_at(west_of_strip()));
    assert!(plan_b.heading_error() <= -150 && plan_b.roll_setpoint() == -30.0);
}

#[test]
fn final_approach_tracks_the_centreline_only_on_a_fresh_low_agl() {
    let config = json!({"final_approach_agl": 8, "nav_bank_gain": 1.5, "land_bank_gain": 1.5, "land_bank_limit": 45});
    let mut guidance = law(config);
    guidance.enter(0.5, 0.0, 0.0);
    let high = Inputs {
        agl: Some(20.0),
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 270.5, 0, &high);
    let homing = guidance.target_heading().unwrap();
    let low = Inputs {
        agl: Some(4.0),
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 270.5, 100_000, &low);
    let tracking = guidance.target_heading().unwrap();
    assert_ne!(homing, tracking);
    assert!(heading_error(tracking, 270.5).abs() <= 45, "the intercept is capped");
    // a stale agl (None) never engages it
    guidance.compute(Stage::Gliding, 270.5, 200_000, &fixed_at(east_of_strip()));
    assert_eq!(guidance.target_heading(), Some(homing));
    // exactly at the threshold it still homes: plan B engages only BELOW it (`agl < final_agl`)
    let at_the_threshold = Inputs {
        agl: Some(8.0),
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 270.5, 300_000, &at_the_threshold);
    assert_eq!(guidance.target_heading(), Some(homing));
    // final_approach_agl 0 switches it off, and so does a present null: plan B's gate reads it as false, where
    // taking it for absent would turn the 8 m default ON
    for off in [json!(0), json!(null)] {
        let mut off = law(json!({ "final_approach_agl": off }));
        off.enter(0.5, 0.0, 0.0);
        off.compute(Stage::Gliding, 270.5, 0, &low);
        assert_eq!(off.target_heading(), Some(homing));
    }
}

#[test]
fn a_stale_baro_never_opens_the_endgame() {
    let mut guidance = law(json!({"endgame_alt_m": 50, "land_bank_gain": 3.0}));
    guidance.enter(0.5, 0.0, 0.0);
    guidance.compute(Stage::Gliding, 0.5, 0, &fixed_at(east_of_strip()));
    let stale_bank = guidance.roll_setpoint();
    assert_ne!(stale_bank, 0.0, "a stale baro must leave ordinary steering in charge");
    let fresh_low = Inputs {
        elevation: Some(10.0),
        ..fixed_at(east_of_strip())
    };
    guidance.compute(Stage::Gliding, 0.5, 100_000, &fresh_low);
    // the endgame band's gated bank: no airspeed, too slow to bank, wings level
    assert_eq!(guidance.roll_setpoint(), 0.0);
    // exactly at the band's top it is still ordinary steering: plan B opens the band only BELOW it
    let mut top = law(json!({"endgame_alt_m": 50, "land_bank_gain": 3.0}));
    top.enter(0.5, 0.0, 0.0);
    let at_the_top = Inputs {
        elevation: Some(50.0),
        ..fixed_at(east_of_strip())
    };
    top.compute(Stage::Gliding, 0.5, 0, &at_the_top);
    assert_eq!(top.roll_setpoint(), stale_bank);
}

#[test]
fn the_loiter_orbits_and_the_endgame_spirals_in() {
    let config = json!({"loiter_radius_m": 30, "loiter_capture_m": 120, "loiter_gain": 3.0, "endgame_alt_m": 50,
        "endgame_pattern": "o", "nav_bank_gain": 1.5, "land_bank_gain": 3.0});
    let mut guidance = law(config);
    guidance.enter(0.5, 0.0, 0.0);
    let centre = strip().target;
    let south = |metres: f32| navigation::advance(centre, 0.0, -metres);
    let high = |position: Position| Inputs {
        elevation: Some(200.0),
        ..fixed_at(position)
    };
    // 60 m south, high: the tangent (bearing 0 + 90) with the inward cut 3 x (60 - 30), capped at 60
    guidance.compute(Stage::Gliding, 0.5, 0, &high(south(60.0)));
    assert!(close(guidance.target_heading().unwrap(), 30.0, 1.0));
    // on the circle: the pure tangent
    guidance.compute(Stage::Gliding, 0.5, 100_000, &high(south(30.0)));
    assert!(close(guidance.target_heading().unwrap(), 90.0, 1.0));
    // the endgame at half the band halves the radius: a deeper cut, 3 x (30 - 15)
    let endgame = Inputs {
        elevation: Some(25.0),
        ..fixed_at(south(30.0))
    };
    guidance.compute(Stage::Gliding, 0.5, 200_000, &endgame);
    assert!(close(guidance.target_heading().unwrap(), 45.0, 1.0));
    // far outside the capture radius the gates steer, not the tangent
    let far = navigation::advance(centre, 20.0, -500.0);
    guidance.compute(Stage::Gliding, 0.5, 300_000, &high(far));
    let gate = navigation::steer(far, strip().top_left, strip().bottom_right).bearing;
    assert!(close(guidance.target_heading().unwrap(), gate, 0.01));
}

#[test]
fn the_orbit_never_asks_for_a_turn_tighter_than_the_airframe_holds() {
    // at 30 m/s the 45 degree cruise bank turns no tighter than ~92 m: the 30 m orbit is clamped to it
    let mut guidance = law(json!({"endgame_pattern": "o"}));
    guidance.enter(0.5, 0.0, 0.0);
    let centre = strip().target;
    let fast = Inputs {
        airspeed: 30.0,
        ..fixed_at(navigation::advance(centre, 0.0, -92.0))
    };
    guidance.compute(Stage::Gliding, 0.5, 0, &fast);
    let floor = guidance.min_turn_radius(45.0, 30.0);
    assert!((floor - 91.7).abs() < 0.5, "{floor}");
    assert!(
        close(guidance.target_heading().unwrap(), 90.0, 1.0),
        "92 m is on the clamped circle"
    );
}

#[test]
fn two_lobes_flip_once_per_centre_crossing() {
    let config = json!({"loiter_radius_m": 30, "loiter_capture_m": 120, "loiter_gain": 3.0, "endgame_alt_m": 50,
        "land_bank_gain": 3.0});
    let mut guidance = law(config.clone());
    guidance.enter(0.5, 0.0, 0.0);
    let near = |position: Position| Inputs {
        elevation: Some(25.0),
        airspeed: 14.0,
        ..fixed_at(position)
    };
    let centre = strip().target;
    let beside = navigation::advance(centre, 1.0, 3.0);
    guidance.compute(Stage::Gliding, 0.5, 0, &near(beside));
    let crossed = guidance.target_heading().unwrap();
    guidance.compute(Stage::Gliding, 0.5, 100_000, &near(beside));
    assert_eq!(
        guidance.target_heading(),
        Some(crossed),
        "staying near the centre does not re-flip"
    );
    guidance.compute(
        Stage::Gliding,
        0.5,
        200_000,
        &near(navigation::advance(centre, 60.0, 0.0)),
    );
    guidance.compute(Stage::Gliding, 0.5, 300_000, &near(beside));
    let back = guidance.target_heading().unwrap();
    assert!(
        !close(back, crossed, 10.0),
        "the next crossing flips back to the other lobe"
    );
    // a fresh entry starts on the gate_b lobe again, so the first crossing flips the same way
    guidance.enter(0.5, 0.0, 0.0);
    guidance.compute(Stage::Gliding, 0.5, 400_000, &near(beside));
    assert_eq!(guidance.target_heading(), Some(crossed));
    // ...also right after an odd crossing, when the glider is on the gate_a lobe with the near-latch set: out and
    // back near again, the crossing flips to gate_a as the first one did, NOT back to gate_b
    let mut odd = law(config);
    odd.enter(0.5, 0.0, 0.0);
    odd.compute(Stage::Gliding, 0.5, 0, &near(beside));
    assert_eq!(odd.target_heading(), Some(crossed));
    odd.enter(0.5, 0.0, 0.0);
    odd.compute(
        Stage::Gliding,
        0.5,
        100_000,
        &near(navigation::advance(centre, 60.0, 0.0)),
    );
    odd.compute(Stage::Gliding, 0.5, 200_000, &near(beside));
    assert_eq!(odd.target_heading(), Some(crossed));
    assert_ne!(odd.target_heading(), Some(back));
}

#[test]
fn a_zone_of_one_point_never_steers_to_nan() {
    // both corners on one point: the centre IS gate_b, the long axis has no length, and plan B's `or 1.0` keeps
    // the axis direction at (0, 0) rather than dividing by zero; every pattern still answers a heading
    let point = at(48.0005, 11.005);
    for pattern in ["o", "ov", "oo"] {
        let mut guidance = law(json!({ "endgame_pattern": pattern }));
        guidance.enter(0.5, 0.0, 0.0);
        let inputs = Inputs {
            landing: Some(Landing::new(point, point, Heading::Circle)),
            elevation: Some(25.0),
            airspeed: 14.0,
            position: Some(Fix {
                position: navigation::advance(point, 20.0, 10.0),
                age_ms: 0,
            }),
            ..Inputs::default()
        };
        guidance.compute(Stage::Gliding, 0.5, 0, &inputs);
        let target = guidance.target_heading().unwrap();
        assert!(
            target.is_finite() && (0.0..360.0).contains(&target),
            "{pattern}: {target}"
        );
    }
}

#[test]
fn the_oval_reverses_at_the_leg_ends_never_shorter_than_fifteen_percent() {
    let mut guidance = law(json!({"endgame_pattern": "ov", "endgame_alt_m": 50, "land_bank_gain": 3.0}));
    guidance.enter(90.5, 0.0, 0.0);
    let centre = strip().target;
    let half_length = navigation::distance(centre, strip().gate_b); // ~372 m, toward gate_b (east)
    // low in the band (2 m of 50) the legs collapse to their floor, 15 % of the half-length: ~55.9 m
    let floor = 0.15 * half_length;
    let mut now = 0;
    let mut fly = |guidance: &mut Guidance, along: f32| {
        let inputs = Inputs {
            elevation: Some(2.0),
            airspeed: 14.0,
            ..fixed_at(navigation::advance(centre, along, 0.0))
        };
        guidance.compute(Stage::Gliding, 90.5, now, &inputs);
        now += 100_000;
        guidance.target_heading().unwrap()
    };
    // a fresh entry runs toward gate_b; on the centreline the leg is the axis itself, due east
    assert!(
        close(fly(&mut guidance, floor - 2.0), 90.0, 0.01),
        "short of the leg end: on east"
    );
    assert!(
        close(fly(&mut guidance, floor + 2.0), 270.0, 0.01),
        "past it: reversed, west"
    );
    assert!(
        close(fly(&mut guidance, -(floor - 2.0)), 270.0, 0.01),
        "short of the other end: on west"
    );
    assert!(
        close(fly(&mut guidance, -(floor + 2.0)), 90.0, 0.01),
        "past it: east again"
    );
    // high in the band the leg runs out to the fraction left (40 m of 50: 80 %), well past the floor
    let mut high = law(json!({"endgame_pattern": "ov", "endgame_alt_m": 50}));
    high.enter(90.5, 0.0, 0.0);
    let inputs = Inputs {
        elevation: Some(40.0),
        airspeed: 14.0,
        ..fixed_at(navigation::advance(centre, 0.7 * half_length, 0.0))
    };
    high.compute(Stage::Gliding, 90.5, 0, &inputs);
    assert!(
        close(high.target_heading().unwrap(), 90.0, 0.01),
        "70 % along is short of 80 %"
    );
    // a re-entry after a reversal starts toward gate_b again (the leg shares the lobe's reset)
    assert!(close(fly(&mut guidance, floor + 2.0), 270.0, 0.01));
    guidance.enter(90.5, 0.0, 0.0);
    assert!(
        close(fly(&mut guidance, floor - 2.0), 90.0, 0.01),
        "a fresh leg runs east"
    );
}

#[test]
fn dead_reckoning_moves_the_position_by_air_velocity_and_wind() {
    let launch = at(48.010, 11.0053);
    let mut guidance = law(json!({}));
    guidance.enter(0.0, 0.0, 0.0);
    let mut inputs = Inputs {
        landing: Some(strip()),
        launch: Some(launch),
        airspeed: 20.0,
        ..Inputs::default()
    };
    // no fix ever: the launch point, and nothing to reckon from
    assert!(guidance.compute(Stage::Gliding, 0.0, 0, &inputs));
    assert!(!guidance.reckoning());
    // a fix seeds the reckoner
    inputs.position = Some(Fix {
        position: launch,
        age_ms: 0,
    });
    guidance.compute(Stage::Gliding, 180.0, 1_000_000, &inputs);
    assert!(!guidance.reckoning());
    // the fix goes: 0.2 s at 20 m/s due south is 4 m
    inputs.position = None;
    guidance.compute(Stage::Gliding, 180.0, 1_200_000, &inputs);
    assert!(guidance.reckoning());
    let (east, north) = guidance.reckoned_offset();
    assert!(east.abs() < 0.01 && (north + 4.0).abs() < 0.01, "{east} {north}");
    // the wind adds: 10 m/s east for 0.2 s
    inputs.wind_east = 10.0;
    guidance.compute(Stage::Gliding, 180.0, 1_400_000, &inputs);
    assert!((guidance.reckoned_offset().0 - east - 2.0).abs() < 0.01);
    // NEGATIVE: a 5 s gap, or a clock that reads earlier, advances nothing
    let held = guidance.reckoned_offset();
    guidance.compute(Stage::Gliding, 180.0, 6_400_000, &inputs);
    assert_eq!(guidance.reckoned_offset(), held);
    guidance.compute(Stage::Gliding, 180.0, 6_300_000, &inputs);
    assert_eq!(guidance.reckoned_offset(), held);
    // a fix again re-seeds: the offset restarts at zero
    inputs.position = Some(Fix {
        position: launch,
        age_ms: 0,
    });
    guidance.compute(Stage::Gliding, 180.0, 6_500_000, &inputs);
    assert_eq!(guidance.reckoned_offset(), (0.0, 0.0));
    // the step ceiling is half a second, exclusive: exactly 500 ms after that fix advances nothing, the next
    // 450 ms does (9 m due south at 20 m/s), and a whole second does not
    inputs.position = None;
    inputs.wind_east = 0.0;
    guidance.compute(Stage::Gliding, 180.0, 7_000_000, &inputs);
    assert!(guidance.reckoning());
    assert_eq!(guidance.reckoned_offset(), (0.0, 0.0), "500 ms is a gap, not a step");
    guidance.compute(Stage::Gliding, 180.0, 7_450_000, &inputs);
    let (east, north) = guidance.reckoned_offset();
    assert!(east.abs() < 0.01 && (north + 9.0).abs() < 0.01, "{east} {north}");
    guidance.compute(Stage::Gliding, 180.0, 8_450_000, &inputs);
    assert_eq!(guidance.reckoned_offset(), (east, north), "a second is a gap");
}

#[test]
fn slow_dead_reckoning_steps_are_kept() {
    // 50 steps of 0.2 m: on plan B's board an f32 latitude loses each one whole; the metres must total 10
    let mut guidance = law(json!({}));
    guidance.enter(0.0, 0.0, 0.0);
    let mut inputs = Inputs {
        airspeed: 2.0,
        ..fixed_at(at(48.010, 11.0053))
    };
    guidance.compute(Stage::Gliding, 180.0, 0, &inputs);
    inputs.position = None;
    for step in 1..=50 {
        guidance.compute(Stage::Gliding, 180.0, step * 100_000, &inputs);
    }
    assert!(
        (guidance.reckoned_offset().1 + 10.0).abs() < 0.01,
        "{:?}",
        guidance.reckoned_offset()
    );
}

#[test]
fn the_hold_law_holds_the_captured_heading_and_the_configured_setpoints() {
    let mut guidance = law(json!({"stages": {"setting": {"roll": 1.0, "pitch": -2.0}}}));
    guidance.enter(100.0, 0.0, 0.0);
    assert!(guidance.compute(Stage::Setting, 90.0, 0, &fixed_at(east_of_strip())));
    assert_eq!((guidance.roll_setpoint(), guidance.pitch_setpoint()), (1.0, -2.0));
    assert_eq!(guidance.heading_error(), 10, "against the captured 100, never the zone");
    assert_eq!(guidance.target_heading(), None);
}

#[test]
fn a_stage_without_setpoints_answers_zeros_rather_than_panicking() {
    // plan B raises here; the flight task never computes a stage that is not configured
    let mut guidance = law(json!({"stages": {"gliding": {"pitch": -6}}}));
    guidance.enter(0.0, 0.0, 0.0);
    assert!(guidance.compute(Stage::Done, 0.0, 0, &Inputs::default()));
    assert_eq!((guidance.roll_setpoint(), guidance.pitch_setpoint()), (0.0, 0.0));
}

#[test]
fn the_turn_radius_is_the_physical_floor() {
    let guidance = law(json!({}));
    assert!((guidance.min_turn_radius(45.0, 14.0) - 20.0).abs() < 0.5);
    assert!((guidance.min_turn_radius(30.0, 14.0) - 34.6).abs() < 0.5);
    assert!(guidance.min_turn_radius(45.0, 20.0) > guidance.min_turn_radius(45.0, 14.0));
    // NEGATIVE: no airspeed or no bank, no estimate
    assert_eq!(guidance.min_turn_radius(45.0, 0.0), 0.0);
    assert_eq!(guidance.min_turn_radius(0.0, 14.0), 0.0);
    assert_eq!(guidance.min_turn_radius(-10.0, 14.0), 0.0);
}

#[test]
fn the_endgame_banks_as_steep_as_the_airspeed_allows() {
    let guidance = law(json!({}));
    // trim 14 m/s against the 10.8 floor: acos((10.8 / 14)^2) ~ 53.5, steeper than the fixed 45
    assert!((guidance.endgame_bank(14.0) - 53.5).abs() < 1.0);
    assert_eq!(guidance.endgame_bank(20.0), 60.0, "capped at the structural limit");
    assert_eq!(guidance.endgame_bank(10.0), 0.0, "under the floor: wings level");
    assert_eq!(
        law(json!({"stall_speed_1g": 0})).endgame_bank(14.0),
        45.0,
        "disabled: the fixed limit"
    );
    // the landing radius is quoted at that bank, and is unbounded (None, never 0) when too slow to bank
    let landing = guidance.landing_turn_radius(14.0).unwrap();
    assert!((landing - guidance.min_turn_radius(guidance.endgame_bank(14.0), 14.0)).abs() < 1e-4);
    assert!(landing < 20.0, "{landing}");
    assert_eq!(guidance.landing_turn_radius(5.0), None);
    // exactly AT the stall-margin floor it is too slow (plan B's `airspeed <= floor`): wings level, the radius
    // unbounded; just above it the bank opens
    let config = guidance.config();
    let floor = config.stall_speed_1g * config.stall_margin;
    assert_eq!(guidance.endgame_bank(floor), 0.0);
    assert_eq!(guidance.landing_turn_radius(floor), None);
    assert!(guidance.endgame_bank(floor * 1.01) > 0.0);
}

#[test]
fn reachability_compares_the_wind_adjusted_reach_with_the_distance() {
    let guidance = law(json!({}));
    let mut inputs = Inputs {
        landing: Some(strip()),
        launch: Some(at(48.010, 11.0053)),
        elevation: Some(100.0),
        ..Inputs::default()
    };
    assert_eq!(guidance.reachability(3.0, &inputs), None, "no fix yet");
    inputs.position = Some(Fix {
        position: at(48.00055, 11.0050),
        age_ms: 0,
    });
    let near = guidance.reachability(3.0, &inputs).unwrap();
    assert!(near.reachable && near.margin_m > 0 && near.distance_m >= 0);
    // ~1.7 km north of the zone: 300 m of reach is not enough
    inputs.position = Some(Fix {
        position: at(48.020, 11.0053),
        age_ms: 5000,
    });
    let far = guidance.reachability(3.0, &inputs).unwrap();
    assert!(!far.reachable && far.margin_m < 0);
    assert_eq!(far.headwind_mps, 0.0);
    // flying south to the zone, a wind toward the north opposes (headwind), one toward the south helps
    let windy = |wind_north: f32| Inputs {
        wind_north,
        airspeed: 14.0,
        ..inputs
    };
    let head = guidance.reachability(3.0, &windy(8.0)).unwrap();
    let tail = guidance.reachability(3.0, &windy(-8.0)).unwrap();
    assert!(tail.margin_m > far.margin_m && far.margin_m > head.margin_m);
    assert!(head.headwind_mps > 0.0 && tail.headwind_mps < 0.0);
    assert_eq!(
        head.to_json(),
        json!({"reachable": false, "margin_m": head.margin_m, "distance_m": head.distance_m, "headwind_mps": 8.0})
    );
    // NEGATIVE: no airspeed skips the wind projection altogether, however strong the wind
    let still = Inputs {
        wind_north: 8.0,
        ..inputs
    };
    assert_eq!(guidance.reachability(3.0, &still), Some(far));
    // NEGATIVE: no elevation, no zone or no launch point -> None
    for missing in [
        Inputs {
            elevation: None,
            ..inputs
        },
        Inputs {
            landing: None,
            ..inputs
        },
        Inputs { launch: None, ..inputs },
    ] {
        assert_eq!(guidance.reachability(3.0, &missing), None);
    }
}

#[test]
fn reachability_rounds_half_to_even_as_plan_b_does() {
    // on the zone's centre the distance is exactly 0, and 0.5 m x 5 is a reach of exactly 2.5 m: plan B's
    // round() goes half to EVEN, 2, where half away from zero would say 3; 1.5 m x 5 = 7.5 goes up to 8
    let guidance = law(json!({}));
    let target = strip().target;
    let on_target = Inputs {
        landing: Some(strip()),
        launch: Some(at(48.010, 11.0053)),
        position: Some(Fix {
            position: target,
            age_ms: 0,
        }),
        elevation: Some(0.5),
        ..Inputs::default()
    };
    let reach = guidance.reachability(5.0, &on_target).unwrap();
    assert_eq!((reach.reachable, reach.margin_m, reach.distance_m), (true, 2, 0));
    let higher = Inputs {
        elevation: Some(1.5),
        ..on_target
    };
    assert_eq!(guidance.reachability(5.0, &higher).unwrap().margin_m, 8);
    // the headwind's tenths too: 0.25 m/s straight along the bearing is a tie, 0.2 either way round (not 0.3)
    let north_of_target = Position {
        latitude_e7: target.latitude_e7 + 10_000,
        ..target
    };
    let windy = |wind_north: f32| Inputs {
        position: Some(Fix {
            position: north_of_target,
            age_ms: 0,
        }),
        elevation: Some(100.0),
        wind_north,
        airspeed: 14.0,
        ..on_target
    };
    assert_eq!(guidance.reachability(3.0, &windy(0.25)).unwrap().headwind_mps, 0.2);
    assert_eq!(guidance.reachability(3.0, &windy(-0.25)).unwrap().headwind_mps, -0.2);
    // the distance too. No position is exactly k.5 m away in f32 AND f64 (a distance is a product with the
    // metres-per-unit), so this pins the Rust rule alone: 77659 units due north is exactly 864.5 m in f32 (plan
    // B's f64 says 864.499988, which rounds to 864 as well), and half to even keeps 864, not 865
    let far_north = Position {
        latitude_e7: target.latitude_e7 + 77_659,
        ..target
    };
    assert_eq!(
        navigation::distance(far_north, target),
        864.5,
        "the f32 tie this test stands on"
    );
    let far = Inputs {
        position: Some(Fix {
            position: far_north,
            age_ms: 0,
        }),
        ..on_target
    };
    assert_eq!(guidance.reachability(5.0, &far).unwrap().distance_m, 864);
    // and the comparison is inclusive: no height left ON the centre is a reach of 0 for a distance of 0, which
    // still reaches (plan B's `reach >= distance`)
    let grounded = Inputs {
        elevation: Some(0.0),
        ..on_target
    };
    assert!(guidance.reachability(5.0, &grounded).unwrap().reachable);
    // NEGATIVE: a centimetre off the centre with no height left does not
    let off_centre = Inputs {
        position: Some(Fix {
            position: Position {
                latitude_e7: target.latitude_e7 + 1,
                ..target
            },
            age_ms: 0,
        }),
        ..grounded
    };
    assert!(!guidance.reachability(5.0, &off_centre).unwrap().reachable);
}
