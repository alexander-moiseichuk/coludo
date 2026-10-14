//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC-less field agent answers as plan B does: src/avionics/golden/field.json, which golden/build_field.py builds
//! from src/glider/tasks/field.py and src/glider/mission.py on CPython. Each case loads a Mission from its
//! launch.config, then replays plan B's _tick() step by step; an arm the agent asks for arms, as plan B's
//! Controller.arm() did (the gates are the firmware's, outside this crate).

use mission::Mission;
use mission::field::{Field, Settings};
use serde_json::{Value, json};

/// plan B's field agent over explicit and seeded pads.
const FIELD: &str = include_str!("../../../golden/field.json");

/// A JSON [lat, lon] as a position, None for null.
fn position(value: &Value) -> Option<mission::Position> {
    let [latitude, longitude] = value.as_array()?.as_slice() else {
        panic!("not a position: {value}");
    };
    Some(mission::Position::from_degrees(
        latitude.as_f64().expect("a latitude"),
        longitude.as_f64().expect("a longitude"),
    ))
}

#[test]
fn the_field_answers_as_plan_b() {
    golden::run(FIELD, |input| {
        let range = input["max_range_m"].as_f64().expect("max_range_m") as f32;
        let mut mission = Mission::load(Some(&input["launch"].to_string()), range);
        let settings = Settings::from_config(input["settings"].as_object().expect("settings")).expect("valid settings");
        let mut field = Field::new(settings);
        let (mut armed, mut arms) = (false, 0);
        let states: Vec<Value> = input["steps"]
            .as_array()
            .expect("steps")
            .iter()
            .map(|step| {
                armed |= step["armed"].as_bool().expect("armed");
                let accel = step["accel"].as_array().map(|axes| {
                    let axis = |index: usize| axes[index].as_f64().expect("an axis") as f32;
                    [axis(0), axis(1), axis(2)]
                });
                let tick = field.tick(
                    step["now_ms"].as_u64().expect("now_ms"),
                    step["stage"] == "setting",
                    armed,
                    position(&step["fix"]),
                    accel,
                    &mut mission,
                );
                if tick.arm {
                    armed = true;
                    arms += 1;
                }
                let zone = mission
                    .zone
                    .map(|zone| json!([zone.top_left.degrees(), zone.bottom_right.degrees()]));
                json!({"site": mission.site, "zone": zone, "armed": armed, "arms": arms})
            })
            .collect();
        Value::from(states)
    });
}

#[test]
fn a_refusal_restarts_the_dwell() {
    let settings = Settings::from_config(
        json!({"auto_arm": true, "auto_arm_dwell_s": 2})
            .as_object()
            .expect("a map"),
    )
    .expect("valid");
    let mut mission = Mission::load(Some(r#"{"zone": [[45.503, -73.502], [45.502, -73.5005]]}"#), 200.0);
    let mut field = Field::new(settings);
    let fix = Some(mission::Position::from_degrees(45.5, -73.5));
    let still = Some([0.0, 0.0, 1.0]);
    let arms: Vec<bool> = (0..8)
        .map(|second| {
            let tick = field.tick(1000 * second, true, false, fix, still, &mut mission);
            if tick.arm {
                field.refused();
            }
            tick.arm
        })
        .collect();
    assert_eq!(
        arms,
        [false, false, true, false, false, true, false, false],
        "a whole dwell after each refusal"
    );
}

#[test]
fn a_knob_plan_b_could_not_fly_refuses_the_entry() {
    let refused = |fields: Value| Settings::from_config(fields.as_object().expect("a map")).expect_err("refused");
    assert!(refused(json!({"auto_arm": "yes"})).starts_with("auto_arm:"));
    assert!(refused(json!({"auto_arm_dwell_s": -1})).starts_with("auto_arm_dwell_s:"));
    assert!(refused(json!({"still_g": 1.5})).starts_with("still_g:"));
    assert!(refused(json!({"period_ms": 0})).starts_with("period_ms:"));
    assert!(refused(json!({"fallback_near_m": -5})).starts_with("fallback_near_m:"));
    let defaults = Settings::from_config(&serde_json::Map::new()).expect("the defaults");
    assert_eq!(
        (defaults.arm_dwell_ms, defaults.auto_arm, defaults.site_select),
        (60_000, false, true)
    );
    assert_eq!(
        Settings::from_config(json!({"auto_arm_dwell_s": 2.5}).as_object().expect("a map")).map(|s| s.arm_dwell_ms),
        Ok(2500)
    );
}
