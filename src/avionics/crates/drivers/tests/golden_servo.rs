//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The fin servos answer as plan B's do: src/avionics/golden/servo.json, which golden/build_servo.py builds from
//! src/glider/drivers/sg90.py and mg90s.py on CPython, compared exactly. The clamp (Python's round, a half to even,
//! then the range), and servos set up from a component block and driven step by step: operator updates, the flight
//! loop's compare-and-set, moves with their travel estimate, and travel_ms itself.

use drivers::mg90s::MG90S;
use drivers::sg90::{SG90, Servo, ServoType, Settings};
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// plan B's SG90 and MG90S over the clamp and scenarios sections.
const SERVO: &str = include_str!("../../../golden/servo.json");

/// The servo type a scenario names.
fn servo_type(name: &str) -> &'static ServoType {
    match name {
        "sg90" => &SG90,
        "mg90s" => &MG90S,
        other => panic!("unknown servo type {other}"),
    }
}

/// What the servo holds, as every step reports it.
fn state(fin: &Servo) -> Value {
    json!({"angle": fin.angle(), "pulse_us": fin.pulse_us(), "trim": fin.trim(), "neutral": fin.neutral()})
}

/// Merge members into a state object.
fn with(mut base: Value, members: Value) -> Value {
    for (key, value) in members.as_object().unwrap() {
        base[key] = value.clone();
    }
    base
}

/// Apply one step's `in` and answer its `out`.
fn step(fin: &mut Servo, request: &Value) -> Value {
    let angle = || request["angle"].as_f64().expect("a numeric angle");
    match request["action"].as_str() {
        Some("update") => {
            let (applied, command) = fin.update(&request["properties"]).expect("a numeric update");
            with(state(fin), json!({"applied": applied, "written": command.is_some()}))
        }
        Some("set_angle") => {
            let command = fin.set_angle(angle());
            with(
                state(fin),
                json!({"returned": fin.clamp(angle()), "written": command.is_some()}),
            )
        }
        Some("move") => {
            let moved = fin.begin_move(angle());
            let rows = [moved.command.row(false), moved.command.row(true)];
            with(
                state(fin),
                json!({"target": moved.command.angle, "travel_ms": moved.travel_ms, "rows": rows}),
            )
        }
        Some("travel") => {
            let degrees = i32::try_from(request["degrees"].as_i64().unwrap()).unwrap();
            with(
                state(fin),
                json!({"travel_ms": fin.travel_ms(degrees, request["settle"].as_bool().unwrap())}),
            )
        }
        other => panic!("unknown servo action {other:?}"),
    }
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(SERVO);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "servo");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/drivers/sg90.py"));
}

#[test]
fn the_clamp_rounds_a_half_to_even() {
    let fixture = Fixture::parse(SERVO);
    let section = fixture.get("clamp");
    let compare = Compare::from_json(&section["compare"]);
    let cases = section["cases"].as_array().unwrap();
    for case in cases {
        let input = &case["input"];
        let config = json!({"min_deg": input["min_deg"], "max_deg": input["max_deg"]});
        let fin = Servo::new(&SG90, &Settings::from_config(&config, &SG90));
        compare
            .check(
                case["name"].as_str().unwrap(),
                &case["output"],
                &json!(fin.clamp(input["angle"].as_f64().unwrap())),
            )
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert!(cases.len() > 80);
}

#[test]
fn every_step_matches_plan_b() {
    let fixture = Fixture::parse(SERVO);
    let section = fixture.get("scenarios");
    let compare = Compare::from_json(&section["compare"]);
    let mut steps = 0;
    for scenario in section["cases"].as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let kind = servo_type(scenario["init"]["type"].as_str().unwrap());
        // plan B wrote any change: its fixtures replay at step 1, the default now being the 3-degree grid
        let mut config = scenario["init"]["config"].clone();
        if let Some(fields) = config.as_object_mut() {
            fields.entry("step_deg").or_insert(json!(1));
        }
        let mut fin = Servo::new(kind, &Settings::from_config(&config, kind));
        compare
            .check(&format!("{name} setup"), &scenario["setup"], &state(&fin))
            .unwrap_or_else(|failure| panic!("{failure}"));
        for (index, item) in scenario["steps"].as_array().unwrap().iter().enumerate() {
            compare
                .check(
                    &format!("{name} step {index}"),
                    &item["out"],
                    &step(&mut fin, &item["in"]),
                )
                .unwrap_or_else(|failure| panic!("{failure}"));
            steps += 1;
        }
    }
    assert!(steps > 200, "only {steps} servo steps");
}

#[test]
fn a_rounding_away_from_zero_would_fail_the_fixture() {
    // NEGATIVE: 2.5 rounds to 2 in plan B; f64::round would give 3, and the fixture says so
    let fixture = Fixture::parse(SERVO);
    let tie = fixture.get("clamp")["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|case| case["input"]["angle"] == json!(2.5))
        .expect("the 2.5 tie");
    assert_eq!(tie["output"], 2);
    assert_ne!(2.5_f64.round() as i32, 2);
}
