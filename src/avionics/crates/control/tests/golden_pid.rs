//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The PID answers as plan B's does: src/avionics/golden/pid.json, which golden/build_pid.py builds from
//! src/glider/pid.py on CPython. Each scenario constructs a controller and runs its steps in order; every step's
//! answer is compared with plan B's under the fixture's tolerance (abs 0.02 degrees, justified in the builder).
//! Inputs are plan B's integer units, read here exactly as the builder states: error_cd / 100 degrees, dt_ms /
//! 1000 seconds, rate_cd_s / 100 degrees per second.

use control::pid::{Gains, Pid};
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// plan B's Pid over test_pid.py's cases, bench_cases.py's pid_steps, two guard scenarios and seeded sweeps.
const PID: &str = include_str!("../../../golden/pid.json");

/// A number of the fixture as f32 (the port's unit); panics naming it when it is not a number.
fn number(value: &Value, name: &str) -> f32 {
    value
        .as_f64()
        .unwrap_or_else(|| panic!("{name} is not a number: {value}")) as f32
}

/// The controller a scenario's `init` describes.
fn controller(init: &Value) -> Pid {
    let gains = Gains {
        proportional: number(&init["kp"], "kp"),
        integral: number(&init["ki"], "ki"),
        derivative: number(&init["kd"], "kd"),
    };
    let shift = init["anti_windup_shift"]
        .as_u64()
        .and_then(|shift| u32::try_from(shift).ok())
        .expect("anti_windup_shift is a small integer");
    Pid::new(
        gains,
        number(&init["integral_limit"], "integral_limit"),
        number(&init["output_limit"], "output_limit"),
        shift,
    )
}

/// Apply one step's `in` to the controller and return the port's `out`.
fn apply(axis: &mut Pid, request: &Value) -> Value {
    match request["action"].as_str() {
        Some("step") => {
            let error = number(&request["error_cd"], "error_cd") / 100.0;
            let elapsed = number(&request["dt_ms"], "dt_ms") / 1000.0;
            let rate = (!request["rate_cd_s"].is_null()).then(|| number(&request["rate_cd_s"], "rate_cd_s") / 100.0);
            json!(f64::from(axis.step(error, elapsed, rate)))
        }
        Some("set_limit") => {
            let limit = request["limit"].as_i64().and_then(|limit| i32::try_from(limit).ok());
            axis.set_limit(limit.expect("set_limit takes whole degrees"));
            Value::Null
        }
        Some("reset") => {
            axis.reset();
            Value::Null
        }
        other => panic!("unknown action {other:?}"),
    }
}

/// Run every scenario: per step, its name, plan B's answer and the port's.
fn answers(text: &str) -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(text);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios").as_array().expect("scenarios is a list") {
        let name = scenario["name"].as_str().expect("a scenario has a name");
        let mut axis = controller(&scenario["init"]);
        for (index, step) in scenario["steps"]
            .as_array()
            .expect("steps is a list")
            .iter()
            .enumerate()
        {
            answers.push((
                format!("{name} step {index}"),
                step["out"].clone(),
                apply(&mut axis, &step["in"]),
            ));
        }
    }
    answers
}

/// The failures of the port's answers under this comparison.
fn failures(compare: Compare, answers: &[(String, Value, Value)]) -> Vec<String> {
    answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect()
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(PID);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "pid");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/pid.py"));
    assert_eq!(fixture.compare(), Compare::Abs(0.02));
}

#[test]
fn every_step_matches_plan_b() {
    let compare = Fixture::parse(PID).compare();
    let answers = answers(PID);
    assert!(answers.len() > 1000, "only {} steps compared", answers.len());
    let failures = failures(compare, &answers);
    assert!(
        failures.is_empty(),
        "{} of {} PID steps differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
    let worst = answers
        .iter()
        .filter_map(|(_, owed, answer)| Some((owed.as_f64()? - answer.as_f64()?).abs()))
        .fold(0.0, f64::max);
    eprintln!(
        "pid: worst |port - plan B| {worst:.6} degrees over {} steps",
        answers.len()
    );
}

#[test]
fn the_tolerance_is_tight_enough_to_catch_a_wrong_gain() {
    // NEGATIVE: kp 1 % high fails some step -- the 0.02 degree tolerance is not a blanket
    let text = PID.replace("\"kp\": 1.5,", "\"kp\": 1.515,");
    assert_ne!(text, PID, "the fixture has a kp of 1.5 to perturb");
    assert!(!failures(Fixture::parse(PID).compare(), &answers(&text)).is_empty());
}

#[test]
fn the_guard_scenarios_show_the_counter_windup() {
    // the reversed fin the stop-at-zero guard lets through: plan B commands +20.36 degrees for a -1 degree error
    let fixture = Fixture::parse(PID);
    let scenarios = fixture.get("scenarios").as_array().unwrap();
    let reversal = scenarios
        .iter()
        .find(|scenario| scenario["name"] == "an_opposite_integral_winds_on_and_reverses_the_fin")
        .expect("the reversal scenario");
    let last = reversal["steps"].as_array().unwrap().last().unwrap();
    assert_eq!(last["in"]["error_cd"], -100);
    assert!(last["out"].as_f64().unwrap() > 20.0, "{}", last["out"]);
}
