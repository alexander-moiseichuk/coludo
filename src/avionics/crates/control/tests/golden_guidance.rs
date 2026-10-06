//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The guidance law answers as plan B does: src/avionics/golden/guidance.json, which golden/build_guidance.py
//! builds from src/glider/guidance.py on CPython. Its sections cover GuidanceConfig, heading_error, the Heading
//! patterns, the zone geometry of the `Landing`, the steering filter, the turn physics, reachability, and the law
//! itself over stateful scenarios: test_guidance's, and seeded closed-loop flights replayed open-loop. A scenario
//! hands the law the pattern plan B's Mission resolved for its zone, as the caller does (the Mission's rule itself
//! is the mission crate's, pinned in mission.json). Each answer is grouped by unit and compared under the
//! tolerance the fixture states and justifies (see tests/grouped).

mod grouped;

use control::guidance::{Fix, Guidance, GuidanceConfig, Heading, Inputs, Landing, SteeringFilter, heading_error};
use control::navigation::Position;
use grouped::{Grouped, number};
use sequencer::Stage;
use serde_json::{Map, Value, json};

/// plan B's guidance over explicit cases, seeded sweeps and flights.
const GUIDANCE: &str = include_str!("../../../golden/guidance.json");

/// The fixture, loaded.
fn fixture() -> Grouped {
    Grouped::parse(GUIDANCE)
}

/// A fixture position, `[latitude_e7, longitude_e7]`.
fn position(value: &Value) -> Position {
    let unit = |index: usize| {
        value[index]
            .as_i64()
            .and_then(|units| i32::try_from(units).ok())
            .unwrap_or_else(|| panic!("a position is two i32 units: {value}"))
    };
    Position {
        latitude_e7: unit(0),
        longitude_e7: unit(1),
    }
}

/// A position as the fixture writes one, `[latitude_e7, longitude_e7]`.
fn units(point: Position) -> Value {
    json!([point.latitude_e7, point.longitude_e7])
}

/// A member that is present and not null: the fixture leaves out (or nulls) what no source vouches for.
fn present<'a>(value: &'a Value, member: &str) -> Option<&'a Value> {
    value.get(member).filter(|item| !item.is_null())
}

/// An optional f32 member.
fn optional_number(value: &Value, member: &str) -> Option<f32> {
    present(value, member).map(|item| item.as_f64().expect("a number") as f32)
}

/// The landing geometry of a fixture zone `[[top_left], [bottom_right]]` with the pattern the Mission resolved for
/// it (a plan B Heading id), or None for a null zone. Without a pattern (reachability, which reads none) it is the
/// circle, as a `Landing` that names none flies.
fn landing(zone: &Value, endgame_heading: &Value) -> Option<Landing> {
    let pattern = endgame_heading.as_u64().map_or(Heading::Circle, |id| {
        Heading::ALL
            .into_iter()
            .find(|pattern| *pattern as u64 == id)
            .unwrap_or_else(|| panic!("no pattern has the id {id}"))
    });
    (!zone.is_null()).then(|| Landing::new(position(&zone[0]), position(&zone[1]), pattern))
}

/// A GuidanceConfig from a fixture's `config` and `position_window_ms` (plan B's flight task hands 1000).
fn config(input: &Value) -> GuidanceConfig {
    let window = input["position_window_ms"]
        .as_u64()
        .map_or(1000, |window| window as u32);
    GuidanceConfig::from_json(&input["config"], window)
}

/// What a step's inputs hand the law: its readings, the zone's landing geometry and the launch point.
fn inputs(step: &Value, landing: Option<Landing>) -> Inputs {
    let wind = |index: usize| present(step, "wind").map_or(0.0, |wind| wind[index].as_f64().unwrap() as f32);
    Inputs {
        landing,
        launch: present(step, "launch").map(position),
        position: present(step, "position").map(|fix| Fix {
            position: position(&fix["position"]),
            age_ms: fix["age_ms"].as_u64().expect("an age") as u32,
        }),
        agl: optional_number(step, "agl"),
        elevation: optional_number(step, "elevation"),
        airspeed: optional_number(step, "airspeed").unwrap_or(0.0),
        wind_east: wind(0),
        wind_north: wind(1),
    }
}

/// The law's slots after a step, grouped as the fixture writes them; `active` only after a compute.
fn slots(guidance: &Guidance, active: Option<bool>) -> Value {
    let mut exact = json!({"heading_error": guidance.heading_error(), "reckoning": guidance.reckoning()});
    if let Some(active) = active {
        exact["active"] = json!(active);
    }
    let (east, north) = guidance.reckoned_offset();
    json!({
        "exact": exact,
        "setpoint": {"roll": guidance.roll_setpoint(), "pitch": guidance.pitch_setpoint()},
        "degrees": {"target_heading": guidance.target_heading()},
        "metres": {"reckoned_offset": [east, north]},
    })
}

/// Replay one scenario case step by step; every mismatch, as "<case> step <n>: ...".
fn replay(fixture: &Grouped, case: &Value) -> Vec<String> {
    let name = case["name"].as_str().unwrap();
    let init = &case["init"];
    let zone = landing(&init["zone"], &init["endgame_heading"]);
    let mut guidance = Guidance::new(GuidanceConfig::from_json(&init["config"], 1000));
    let mut failures = Vec::new();
    let steps = case["steps"].as_array().expect("a scenario has steps");
    assert!(!steps.is_empty(), "{name}: no steps");
    for (index, step) in steps.iter().enumerate() {
        let input = &step["in"];
        let answer = match input["action"].as_str() {
            Some("enter") => {
                let centidegrees = |member: &str| input[member].as_i64().unwrap() as f32 / 100.0;
                guidance.enter(
                    number(input, "heading"),
                    centidegrees("roll_centidegrees"),
                    centidegrees("pitch_centidegrees"),
                );
                slots(&guidance, None)
            }
            Some("compute") => {
                let stage = Stage::from_name(input["stage"].as_str().unwrap()).expect("a stage");
                let now_us = input["now_us"].as_u64().expect("a time");
                let active = guidance.compute(stage, number(input, "heading"), now_us, &inputs(input, zone));
                slots(&guidance, Some(active))
            }
            other => panic!("{name}: unknown action {other:?}"),
        };
        if let Err(failure) = fixture.check(&format!("{name} step {index}"), &step["out"], &answer) {
            failures.push(failure);
        }
    }
    failures
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let meta = fixture().get("meta").clone();
    assert_eq!(meta["tag"], "micropython_plan_B_2");
    assert_eq!(meta["generator"], "guidance");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/guidance.py"));
}

#[test]
fn config_matches_plan_b() {
    fixture().run("config", |input| {
        let config = config(input);
        let guidance = Guidance::new(config.clone());
        let stages: Map<String, Value> = Stage::ALL
            .into_iter()
            .filter_map(|stage| {
                guidance
                    .setpoint(stage)
                    .map(|setpoint| (stage.name().to_owned(), json!([setpoint.roll, setpoint.pitch])))
            })
            .collect();
        json!({
            "config": {
                "bank_gain": config.bank_gain, "bank_limit": config.bank_limit, "land_bank_gain": config.land_bank_gain,
                "land_bank_limit": config.land_bank_limit, "endgame_alt_m": config.endgame_altitude_m,
                "stall_speed_1g": config.stall_speed_1g, "stall_margin": config.stall_margin,
                "endgame_max_bank": config.endgame_max_bank, "loiter_radius_m": config.loiter_radius_m,
                "loiter_capture_m": config.loiter_capture_m, "loiter_gain": config.loiter_gain,
                "final_agl": config.final_approach_agl, "final_cross_gain": config.final_cross_gain,
                "final_intercept": config.final_intercept, "boost_engage": config.boost_engage_speed,
            },
            "setpoint": {"stages": stages},
            "exact": {
                "endgame_pattern": config.endgame_pattern as u8,
                "steer_filter_shift": config.steer_filter_shift,
                "nav_period_us": config.navigation_period_us,
                "position_age_max_ms": config.position_age_max_ms,
            },
        })
    });
}

#[test]
fn heading_error_matches_plan_b() {
    fixture().run(
        "heading_error",
        |input| json!({"exact": heading_error(number(input, "target"), number(input, "current"))}),
    );
}

#[test]
fn heading_resolve_matches_plan_b() {
    fixture().run("heading_resolve", |input| {
        let pattern = Heading::resolve(input["name"].as_str().unwrap());
        json!({"exact": {"id": pattern as u8, "name": pattern.name()}})
    });
}

#[test]
fn landing_matches_the_missions_zone_points() {
    fixture().run("landing", |input| {
        let landing = Landing::new(
            position(&input["top_left"]),
            position(&input["bottom_right"]),
            Heading::Circle,
        );
        json!({"exact": {
            "target": units(landing.target),
            "gate_a": units(landing.gate_a),
            "gate_b": units(landing.gate_b),
        }})
    });
}

#[test]
fn filter_matches_plan_b() {
    fixture().run("filter", |input| {
        let mut filter = SteeringFilter::new(input["shift"].as_u64().unwrap() as u32);
        let errors: Vec<i32> = input["errors"]
            .as_array()
            .unwrap()
            .iter()
            .map(|error| filter.filter(error.as_i64().unwrap() as i32))
            .collect();
        json!({"exact": errors})
    });
}

#[test]
fn turn_physics_matches_plan_b() {
    fixture().run("turn", |input| {
        let guidance = Guidance::new(config(input));
        let (airspeed, bank) = (number(input, "airspeed"), number(input, "bank"));
        json!({
            "metres": {
                "min_turn_radius": guidance.min_turn_radius(bank, airspeed),
                "landing_turn_radius": guidance.landing_turn_radius(airspeed),
            },
            "setpoint": {"endgame_bank": guidance.endgame_bank(airspeed)},
        })
    });
}

#[test]
fn reachability_matches_plan_b() {
    fixture().run("reachability", |input| {
        let guidance = Guidance::new(GuidanceConfig::from_json(&json!({}), 1000));
        // plan B takes the wind and the airspeed as arguments; here they ride in the inputs, as in flight
        let inputs = Inputs {
            airspeed: number(input, "reach_airspeed"),
            wind_east: number(input, "wind_east"),
            wind_north: number(input, "wind_north"),
            ..inputs(input, landing(&input["zone"], &Value::Null))
        };
        let reach = guidance.reachability(number(input, "glide_ratio"), &inputs);
        json!({"exact": reach.map(|reach| reach.to_json())})
    });
}

#[test]
fn the_law_flies_every_scenario_as_plan_b() {
    let fixture = fixture();
    let cases = fixture.cases("scenarios");
    let failures: Vec<String> = cases.iter().flat_map(|case| replay(&fixture, case)).collect();
    assert!(
        failures.is_empty(),
        "{} golden steps failed:\n{}",
        failures.len(),
        failures.join("\n")
    );
    // the seeded flights are there and long: a replay that compared nothing must not pass
    let flights = cases
        .iter()
        .filter(|case| case["name"].as_str().unwrap().starts_with("flight_"))
        .count();
    assert!(flights >= 4, "{flights} seeded flights");
}

#[test]
fn a_one_degree_slip_in_the_heading_error_fails_a_scenario() {
    // the heading error is an integer and compares exactly: a law one degree off fails at once
    let fixture = fixture();
    let case = fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == "bank_to_turn")
        .unwrap();
    let step = &case["steps"][1];
    let mut answer = step["out"].clone();
    assert_eq!(fixture.check("as recorded", &step["out"], &answer), Ok(()));
    answer["exact"]["heading_error"] = json!(answer["exact"]["heading_error"].as_i64().unwrap() + 1);
    assert!(fixture.check("one degree off", &step["out"], &answer).is_err());
}

#[test]
fn a_tenth_of_a_degree_in_the_target_fails() {
    let fixture = fixture();
    let case = fixture
        .cases("scenarios")
        .iter()
        .find(|case| case["name"] == "loiter_and_endgame_spiral")
        .unwrap();
    let step = &case["steps"][1];
    let mut answer = step["out"].clone();
    let target = answer["degrees"]["target_heading"].as_f64().unwrap();
    answer["degrees"]["target_heading"] = json!(target + 0.1);
    assert!(fixture.check("target off", &step["out"], &answer).is_err());
    // and a setpoint two centidegrees off fails its 0.011 degree tolerance
    let mut answer = step["out"].clone();
    let roll = answer["setpoint"]["roll"].as_f64().unwrap();
    answer["setpoint"]["roll"] = json!(roll + 0.02);
    assert!(fixture.check("roll off", &step["out"], &answer).is_err());
}

#[test]
fn the_fixture_holds_the_cases_that_pin_the_rules_edges() {
    // these cases sit ON a rule's edge on purpose (the builder's notes say why): a regenerated fixture without
    // them would still pass every comparison, so their presence is checked
    let fixture = fixture();
    let names = |section: &str| -> Vec<String> {
        fixture
            .cases(section)
            .iter()
            .map(|case| case["name"].as_str().unwrap().to_owned())
            .collect()
    };
    let expected = [
        (
            "scenarios",
            &[
                "final_approach_agl_none_is_off",
                "final_approach_agl_false_is_off",
                "final_approach_agl_true_is_one",
                "nav_bank_gain_null_is_rudder_only",
                "land_bank_gain_null_is_off",
                "endgame_alt_m_null_is_off",
                "steer_filter_shift_float_zero_is_off",
                "steer_filter_shift_null_is_off",
                "steer_filter_shift_true_is_one",
            ][..],
        ),
        (
            "reachability",
            &[
                "a_reach_of_2_5_rounds_to_even",
                "a_reach_of_7_5_rounds_to_even",
                "a_headwind_of_0_25_rounds_to_even",
                "a_tailwind_of_0_25_rounds_to_even",
                "no_height_on_the_centre_still_reaches",
            ][..],
        ),
        (
            "filter",
            &["a_jump_of_+90", "a_jump_of_+91", "a_jump_of_-90", "a_jump_of_-91"][..],
        ),
    ];
    for (section, wanted) in expected {
        let present = names(section);
        for name in wanted {
            assert!(present.iter().any(|case| case == name), "{section}: {name} is missing");
        }
    }
}
