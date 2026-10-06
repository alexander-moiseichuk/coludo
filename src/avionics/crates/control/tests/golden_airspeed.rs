//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The airspeed estimator answers as plan B's does: src/avionics/golden/airspeed.json, which golden/build_airspeed.py
//! builds from src/glider/airspeed.py on CPython. Each scenario constructs an estimator and runs its operations in
//! order; every answer -- the estimate within 1e-3 m/s, the confidence flag exactly -- is compared with plan B's.

use control::airspeed::AirspeedEstimator;
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// plan B's AirspeedEstimator over test_airspeed.py's cases and seeded flights.
const AIRSPEED: &str = include_str!("../../../golden/airspeed.json");

/// A number of the fixture as f32 (the port's unit); panics naming it when it is not a number.
fn number(value: &Value, name: &str) -> f32 {
    value
        .as_f64()
        .unwrap_or_else(|| panic!("{name} is not a number: {value}")) as f32
}

/// Apply one operation to the estimator and return the port's `{value, confident}`.
fn apply(estimator: &mut AirspeedEstimator, request: &Value) -> Value {
    match request["action"].as_str() {
        Some("predict") => {
            estimator.predict(
                number(&request["acceleration_along"], "acceleration_along"),
                number(&request["elapsed"], "elapsed"),
            );
        }
        Some("correct") => {
            let has_fix = request["has_fix"].as_bool().expect("has_fix is a flag");
            estimator.correct(number(&request["gnss_speed"], "gnss_speed"), has_fix);
        }
        Some("measure") => {
            estimator.measure(
                number(&request["airspeed"], "airspeed"),
                number(&request["gain"], "gain"),
            );
        }
        Some("seed") => {
            estimator.seed(number(&request["airspeed"], "airspeed"));
        }
        other => panic!("unknown action {other:?}"),
    }
    json!({"value": f64::from(estimator.value()), "confident": estimator.confident()})
}

/// Run every scenario: per step, its name, plan B's answer and the port's. `gnss_gain_factor` scales each
/// scenario's GNSS blend weight: 1 is the port as it is, anything else a perturbed port for the negative test.
fn answers(gnss_gain_factor: f32) -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(AIRSPEED);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios").as_array().expect("scenarios is a list") {
        let name = scenario["name"].as_str().expect("a scenario has a name");
        let init = &scenario["init"];
        let mut estimator = AirspeedEstimator::new(
            number(&init["ceiling"], "ceiling"),
            number(&init["gnss_gain"], "gnss_gain") * gnss_gain_factor,
        );
        for (index, step) in scenario["steps"]
            .as_array()
            .expect("steps is a list")
            .iter()
            .enumerate()
        {
            answers.push((
                format!("{name} step {index}"),
                step["out"].clone(),
                apply(&mut estimator, &step["in"]),
            ));
        }
    }
    answers
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(AIRSPEED);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "airspeed");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/airspeed.py"));
    assert_eq!(fixture.compare(), Compare::Abs(1e-3));
}

/// The failures of the port's answers under the fixture's comparison.
fn failures(answers: &[(String, Value, Value)]) -> Vec<String> {
    let compare = Fixture::parse(AIRSPEED).compare();
    answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect()
}

#[test]
fn every_operation_matches_plan_b() {
    let answers = answers(1.0);
    assert!(answers.len() > 400, "only {} operations compared", answers.len());
    let failures = failures(&answers);
    assert!(
        failures.is_empty(),
        "{} of {} airspeed operations differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
    let worst = answers
        .iter()
        .filter_map(|(_, owed, answer)| Some((owed["value"].as_f64()? - answer["value"].as_f64()?).abs()))
        .fold(0.0, f64::max);
    eprintln!(
        "airspeed: worst |port - plan B| {worst:.3e} m/s over {} operations",
        answers.len()
    );
}

#[test]
fn a_port_with_a_one_percent_stronger_gnss_blend_fails() {
    // NEGATIVE: the fixture replayed through a wrong estimator must fail, which also proves this file's wiring
    // (the actions, the arguments and the comparison) reaches the estimate: a GNSS gain 1 % high moves each blend
    // by about 0.1 m/s on a 10 m/s correction, a hundred times the tolerance
    let failures = failures(&answers(1.01));
    assert!(
        failures.len() > 50,
        "only {} operations told the gain apart",
        failures.len()
    );
}

#[test]
fn a_confidence_flip_fails_the_comparison() {
    // NEGATIVE: the flag compares exactly under the m/s tolerance; a port that latched at 4.9 m/s would differ
    let compare = Fixture::parse(AIRSPEED).compare();
    let owed = json!({"value": 4.95, "confident": false});
    assert!(
        compare
            .check("flip", &owed, &json!({"value": 4.95, "confident": true}))
            .is_err()
    );
    assert!(
        compare
            .check("drift", &owed, &json!({"value": 4.952, "confident": false}))
            .is_err()
    );
    assert_eq!(
        compare.check("close", &owed, &json!({"value": 4.9505, "confident": false})),
        Ok(())
    );
}
