//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The fin deflection table answers as plan B's does: src/avionics/golden/fin_limit.json, which
//! golden/build_fin_limit.py builds from src/glider/commons.py on CPython. Compare exact: whole degrees, every
//! entry of the table and both sides of every step it takes.

use control::commons::fin_deflection_limit;
use golden::Fixture;
use serde_json::json;

/// plan B's fin_deflection_limit over the test and bench speeds, every whole speed, every step and a sweep.
const FIN_LIMIT: &str = include_str!("../../../golden/fin_limit.json");

#[test]
fn the_fixture_says_where_it_came_from() {
    let meta = Fixture::parse(FIN_LIMIT).get("meta").clone();
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "fin_limit");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/commons.py"));
}

#[test]
fn the_table_matches_plan_b() {
    golden::run(FIN_LIMIT, |input| {
        json!(fin_deflection_limit(input.as_f64().expect("a speed") as f32))
    });
}

#[test]
#[should_panic(expected = "golden cases failed")]
fn rounding_the_speed_instead_of_truncating_it_fails() {
    // NEGATIVE: int() truncates; a lookup that rounds the speed lands on the next entry at every step's midpoint
    golden::run(FIN_LIMIT, |input| {
        json!(fin_deflection_limit((input.as_f64().unwrap() as f32).round()))
    });
}
