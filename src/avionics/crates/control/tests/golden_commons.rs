//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-math half of commons.py answers as plan B does: src/avionics/golden/commons.json, sections
//! `fin_deflection_limit` and `altitude_m`, which golden/build_commons.py builds from src/glider/commons.py on
//! CPython (its `dwell_step` and `apogee_step` sections are crates/sequencer's). Each section states its
//! tolerance and why: the fin table exact, the altitude to a centimetre (f32 measures 3.4 mm at worst).

use control::commons::{SEA_LEVEL_PA, altitude_m, fin_deflection_limit};
use golden::{Compare, Fixture};
use serde_json::json;

/// plan B's commons.py primitives (four sections; this crate ports two).
const COMMONS: &str = include_str!("../../../golden/commons.json");

/// A section of the fixture as golden fixture text: `{compare, note, cases}`.
fn section(name: &str) -> String {
    Fixture::parse(COMMONS).get(name).to_string()
}

/// A case's input as the f32 the port takes.
fn input(value: &serde_json::Value) -> f32 {
    value.as_f64().expect("a number") as f32
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(COMMONS);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG, "{meta}");
    assert_eq!(meta["generator"], "commons");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/commons.py"));
    let altitude = Fixture::parse(&section("altitude_m"));
    assert_eq!(altitude.compare(), Compare::Abs(0.01), "the stated centimetre");
}

#[test]
fn fin_deflection_limit_matches_plan_b() {
    golden::run(&section("fin_deflection_limit"), |speed| {
        json!(fin_deflection_limit(input(speed)))
    });
}

#[test]
fn altitude_m_matches_plan_b() {
    golden::run(&section("altitude_m"), |pressure| {
        json!(f64::from(altitude_m(input(pressure))))
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn an_altitude_two_centimetres_off_fails() {
    // the tolerance is real: the f32 answer is within 3.4 mm, so 2 cm is a failure
    golden::run(&section("altitude_m"), |pressure| {
        let metres = altitude_m(input(pressure));
        json!(f64::from(if metres == 0.0 { metres } else { metres + 0.02 }))
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn a_fin_limit_off_by_one_degree_fails() {
    golden::run(&section("fin_deflection_limit"), |speed| {
        json!(fin_deflection_limit(input(speed)) + 1)
    });
}

#[test]
fn altitude_refuses_a_dead_read_and_passes_nan_through() {
    assert_eq!(altitude_m(SEA_LEVEL_PA), 0.0);
    assert_eq!(altitude_m(0.0), 0.0, "zero pressure is a dead read, not a height");
    assert_eq!(altitude_m(-5.0), 0.0);
    assert!(
        altitude_m(f32::NAN).is_nan(),
        "plan B's `nan <= 0` is false, so NaN flows through"
    );
    // the shape: lower pressure is higher, and about 8.4 m per hPa near the datum
    assert!(altitude_m(100_000.0) > 0.0 && altitude_m(105_000.0) < 0.0);
    let per_hectopascal = altitude_m(101_225.0) - altitude_m(101_325.0);
    assert!((8.0..9.0).contains(&per_hectopascal), "{per_hectopascal}");
}
