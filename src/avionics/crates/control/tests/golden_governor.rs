//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The fin governor answers as plan B's does: src/avionics/golden/governor.json, which golden/build_governor.py
//! builds from src/glider/governor.py on CPython. Two sections: the throttle's update interval (seconds, abs 1e-6),
//! and stateful scenarios -- after every control slice, the airspeed within 1e-3 m/s and the confidence flag and the
//! whole-degree cap in the mixer exactly. Plan B's centidegree pitch is read as pitch_cd / 100 degrees.

use control::governor::{Governor, GovernorConfig, Readings};
use control::mixer::Mixer;
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// plan B's Governor over test_governor.py's cases (through the public API) and seeded flights.
const GOVERNOR: &str = include_str!("../../../golden/governor.json");

/// A section of the fixture as golden fixture text: `{compare, note, cases}`.
fn section(name: &str) -> String {
    Fixture::parse(GOVERNOR).get(name).to_string()
}

/// A number of the fixture as f32 (the port's unit); panics naming it when it is not a number.
fn number(value: &Value, name: &str) -> f32 {
    value
        .as_f64()
        .unwrap_or_else(|| panic!("{name} is not a number: {value}")) as f32
}

/// A reading that may be null (absent or stale).
fn optional(value: &Value, name: &str) -> Option<f32> {
    (!value.is_null()).then(|| number(value, name))
}

/// The readings one step carries.
fn readings(request: &Value) -> Readings {
    let acceleration = (!request["accel"].is_null()).then(|| {
        let axis = |index: usize| number(&request["accel"][index], "accel");
        [axis(0), axis(1), axis(2)]
    });
    Readings {
        acceleration,
        gnss_speed: optional(&request["gnss"], "gnss"),
        pitot: optional(&request["pitot"], "pitot"),
    }
}

/// Apply one step to the governor (and its mixer) and return the port's `{airspeed, confident, cap}`.
fn apply(governor: &mut Governor, mixer: &mut Mixer, request: &Value) -> Value {
    match request["action"].as_str() {
        Some("step") => {
            let full_rate_override = request["full_rate_override"].as_bool().expect("a flag");
            let pitch = number(&request["pitch_cd"], "pitch_cd") / 100.0;
            governor.step(
                mixer,
                number(&request["dt"], "dt"),
                full_rate_override,
                pitch,
                &readings(request),
            );
        }
        Some("seed") => governor.seed_airspeed(mixer, number(&request["airspeed"], "airspeed")),
        other => panic!("unknown action {other:?}"),
    }
    json!({"airspeed": f64::from(governor.airspeed()), "confident": governor.estimator().confident(),
           "cap": mixer.limit()})
}

/// Run every scenario: per step, its name, plan B's answer and the port's. `unthrottled` forces every slice to
/// full rate, a port without the distance-constant throttle, for the negative test.
fn answers(unthrottled: bool) -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(GOVERNOR);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios")["cases"]
        .as_array()
        .expect("scenarios.cases is a list")
    {
        let name = scenario["name"].as_str().expect("a scenario has a name");
        let init = &scenario["init"];
        let mut governor = Governor::new(
            GovernorConfig::new(&init["config"]),
            number(&init["multiplier"], "multiplier"),
        );
        let mut mixer = Mixer::default();
        for (index, step) in scenario["steps"]
            .as_array()
            .expect("steps is a list")
            .iter()
            .enumerate()
        {
            let mut request = step["in"].clone();
            if unthrottled && request["action"] == "step" {
                request["full_rate_override"] = json!(true);
            }
            let answer = apply(&mut governor, &mut mixer, &request);
            answers.push((format!("{name} step {index}"), step["out"].clone(), answer));
        }
    }
    answers
}

/// The failures of the port's answers under the scenarios' comparison.
fn failures(answers: &[(String, Value, Value)]) -> Vec<String> {
    let compare = Compare::from_json(&Fixture::parse(GOVERNOR).get("scenarios")["compare"]);
    answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect()
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(GOVERNOR);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "governor");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/governor.py"));
    assert_eq!(
        Compare::from_json(&fixture.get("scenarios")["compare"]),
        Compare::Abs(1e-3)
    );
}

#[test]
fn the_update_interval_matches_plan_b() {
    golden::run(&section("update_interval"), |input| {
        json!(f64::from(
            GovernorConfig::new(&input["config"]).update_interval(number(&input["speed"], "speed"))
        ))
    });
}

#[test]
fn every_control_slice_matches_plan_b() {
    let answers = answers(false);
    assert!(answers.len() > 400, "only {} slices compared", answers.len());
    let failures = failures(&answers);
    assert!(
        failures.is_empty(),
        "{} of {} governor slices differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
    let worst = answers
        .iter()
        .filter_map(|(_, owed, answer)| Some((owed["airspeed"].as_f64()? - answer["airspeed"].as_f64()?).abs()))
        .fold(0.0, f64::max);
    let caps: std::collections::BTreeSet<i64> =
        answers.iter().filter_map(|(_, owed, _)| owed["cap"].as_i64()).collect();
    eprintln!(
        "governor: worst |port - plan B| {worst:.3e} m/s over {} slices; caps seen {caps:?}",
        answers.len()
    );
    assert!(caps.len() > 10, "the scenarios exercise too few caps: {caps:?}");
}

#[test]
fn a_port_without_the_throttle_fails() {
    // NEGATIVE: the throttle shapes the estimate (the accelerometer held over the accumulated interval, one blend
    // per update), so updating every slice must not pass as plan B
    let failures = failures(&answers(true));
    assert!(
        failures.len() > 50,
        "only {} slices told the throttle apart",
        failures.len()
    );
}

#[test]
fn the_garbage_seed_opens_the_cap_until_the_next_update() {
    // plan B behaviour the port keeps: a refused seed still re-caps, off the unseeded 0 m/s -- the full 45 degrees
    let mut governor = Governor::new(GovernorConfig::default(), 1.0);
    let mut mixer = Mixer::default();
    governor.seed_airspeed(&mut mixer, -1.0);
    assert!(!governor.estimator().confident());
    assert_eq!(mixer.limit(), 45);
    let level = Readings {
        acceleration: Some([0.0, 0.0, 1.0]),
        ..Readings::default()
    };
    governor.step(&mut mixer, 0.01, true, 0.0, &level);
    assert_eq!(mixer.limit(), 14, "the next update caps off the unconfident 30 m/s");
}
