//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The wind estimator answers as plan B does: src/avionics/golden/wind.json, which golden/build_wind.py builds
//! from src/glider/wind.py on CPython. Stateful scenarios (test_wind's cases, the non-finite fixes, the update
//! rules, seeded runs with GNSS noise and jumps) replayed step by step; each answer grouped by unit and compared
//! under the tolerance the fixture states (see tests/grouped).

mod grouped;

use control::wind::WindEstimator;
use grouped::Grouped;
use serde_json::{Value, json};

/// plan B's wind estimator over explicit and seeded runs.
const WIND: &str = include_str!("../../../golden/wind.json");

/// The fixture, loaded.
fn fixture() -> Grouped {
    Grouped::parse(WIND)
}

/// An observation as the fixture writes it: a number, or the text 'nan', 'inf' or '-inf'.
fn observed(value: &Value) -> f32 {
    match value {
        Value::String(text) => text
            .parse::<f32>()
            .unwrap_or_else(|_| panic!("not a float text: {text}")),
        number => number.as_f64().expect("a number") as f32,
    }
}

/// The estimator's answers after a step, grouped as the fixture writes them; `changed` only after an update.
fn answers(estimator: &WindEstimator, changed: Option<Vec<&'static str>>) -> Value {
    let mut exact = json!({"inspect": estimator.inspect(), "stats": estimator.stats()});
    if let Some(changed) = changed {
        exact["changed"] = json!(changed);
    }
    let (east, north) = estimator.components();
    json!({
        "exact": exact,
        "speed": {"components": [east, north], "speed": estimator.speed()},
        "degrees": {"direction": estimator.direction()},
    })
}

/// Run one step on the estimator: an observe, or an update with the names it changed.
fn step(estimator: &mut WindEstimator, input: &Value) -> Option<Vec<&'static str>> {
    match input["action"].as_str() {
        Some("observe") => {
            estimator.observe(
                observed(&input["course"]),
                observed(&input["ground_speed"]),
                observed(&input["airspeed"]),
                observed(&input["heading"]),
            );
            None
        }
        Some("update") => Some(estimator.update(input["properties"].as_object().expect("properties"))),
        other => panic!("unknown action {other:?}"),
    }
}

/// Replay one scenario; every mismatch, as "<case> step <n>: ...".
fn replay(fixture: &Grouped, case: &Value, mut corrupt: impl FnMut(&mut Value)) -> Vec<String> {
    let name = case["name"].as_str().unwrap();
    let mut estimator = WindEstimator::new(&case["init"]["config"]);
    let steps = case["steps"].as_array().expect("a scenario has steps");
    assert!(!steps.is_empty(), "{name}: no steps");
    steps
        .iter()
        .enumerate()
        .filter_map(|(index, recorded)| {
            let changed = step(&mut estimator, &recorded["in"]);
            let mut answer = answers(&estimator, changed);
            corrupt(&mut answer);
            fixture
                .check(&format!("{name} step {index}"), &recorded["out"], &answer)
                .err()
        })
        .collect()
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let meta = fixture().get("meta").clone();
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "wind");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/wind.py"));
}

#[test]
fn the_estimator_answers_every_step_as_plan_b() {
    let fixture = fixture();
    let cases = fixture.cases("scenarios");
    let failures: Vec<String> = cases.iter().flat_map(|case| replay(&fixture, case, |_| {})).collect();
    assert!(
        failures.is_empty(),
        "{} golden steps failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
    assert!(cases.len() >= 10, "{} scenarios", cases.len());
}

#[test]
fn a_speed_off_by_a_hundredth_fails() {
    // components and speed hold a millimetre per second: 0.01 m/s off fails every non-calm step
    let fixture = fixture();
    let case = fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == "the_ema_blends_each_fix_in")
        .unwrap();
    let failures = replay(&fixture, case, |answer| {
        let speed = answer["speed"]["speed"].as_f64().unwrap();
        answer["speed"]["speed"] = json!(speed + 0.01);
    });
    assert_eq!(failures.len(), 4, "{failures:?}");
}

#[test]
fn a_rounded_figure_one_step_off_fails() {
    // inspect and stats compare exactly: a centimetre per second or a degree off is a different answer
    let fixture = fixture();
    let case = fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == "an_east_wind_from_the_west")
        .unwrap();
    let failures = replay(&fixture, case, |answer| {
        answer["exact"]["stats"]["from"] = json!(answer["exact"]["stats"]["from"].as_i64().unwrap() + 1);
    });
    assert_eq!(failures.len(), 1);
    let failures = replay(&fixture, case, |answer| {
        answer["exact"]["inspect"]["speed"] = json!(answer["exact"]["inspect"]["speed"].as_f64().unwrap() + 0.01);
    });
    assert_eq!(failures.len(), 1);
}
