//! The mission answers as plan B does: src/avionics/golden/mission.json, which golden/build_mission.py builds
//! from src/glider/mission.py on CPython. One tolerance covers the whole fixture (its `tolerance` member says
//! why), and the deliberate divergences (`sites`, and integers written as floats) are applied where the
//! fixture marks them.

use golden::{Compare, Fixture};
use mission::{DEFAULT_MAX_RANGE_M, Mission, Position, TimeSet, clock_text, epoch_request};
use serde_json::{Map, Value, json};

/// plan B's Mission over load/update/set-config/reboot scenarios, time sets and clock texts.
const MISSION: &str = include_str!("../../../golden/mission.json");

/// How a request's fields are applied: [`Mission::update`] (`update mission`) or [`Mission::merge`]
/// (`set-config launch`).
type Apply = fn(&mut Mission, &Map<String, Value>) -> Vec<&'static str>;

/// A member of the fixture as golden fixture text, under the fixture's own compare block.
fn section(member: &str) -> String {
    let fixture = Fixture::parse(MISSION);
    json!({"compare": fixture.get("compare"), "cases": fixture.get(member)}).to_string()
}

/// A fixture fix, `[latitude_e7, longitude_e7]` or null.
fn fix(value: &Value) -> Option<Position> {
    let units = value.as_array()?;
    let unit = |index: usize| i32::try_from(units[index].as_i64().expect("a unit is an integer")).unwrap();
    Some(Position {
        latitude_e7: unit(0),
        longitude_e7: unit(1),
    })
}

/// A request handled the way the firmware handles one: the fields, then the clock when the request asks for a
/// time set, listed last as plan B lists it.
fn apply(mission: &mut Mission, properties: &Map<String, Value>, fields: Apply) -> Vec<&'static str> {
    let mut changed = fields(mission, properties);
    if epoch_request(properties).is_some() {
        changed.push("epoch");
    }
    changed
}

/// The Rust answer after a step, in the fixture's `out` shape (inspect without the caller's clock and epoch).
fn state(mission: &Mission, fix: Option<Position>) -> Value {
    let mut inspect = mission.inspect(fix, "", 0);
    let snapshot = inspect.as_object_mut().unwrap();
    snapshot.remove("clock");
    snapshot.remove("epoch");
    let persisted = mission.persisted();
    json!({"persisted": persisted, "sites": persisted["sites"], "probe": mission.probe(fix), "inspect": inspect})
}

/// `value` with every integer number made a float: how the Rust port writes metres and degrees, which it holds
/// as f32 and 1e-7 units, where plan B echoes the integer it was given.
fn floats(value: &Value) -> Value {
    match value {
        Value::Number(number) if !number.is_f64() => json!(number.as_f64().expect("a JSON number reads as f64")),
        Value::Array(items) => Value::Array(items.iter().map(floats).collect()),
        Value::Object(members) => {
            Value::Object(members.iter().map(|(key, item)| (key.clone(), floats(item))).collect())
        }
        other => other.clone(),
    }
}

/// The answer the Rust port owes for a step: plan B's, with the step's marked divergences applied (the Rust
/// `sites` and `changed`, and with `floats` every integer in the persisted, inspect and sites answers as a float).
/// Rust's persisted() always carries `sites` (the ones it holds), where plan B's leaves them out.
fn owed(step: &Value) -> Value {
    let mut out = step["out"].clone();
    if let Some(divergence) = step.get("divergence") {
        for member in ["sites", "changed"] {
            if let Some(value) = divergence.get(member) {
                out[member] = value.clone();
            }
        }
        if divergence["floats"] == true {
            for member in ["persisted", "inspect", "sites"] {
                out[member] = floats(&out[member]);
            }
        }
    }
    out["persisted"]["sites"] = out["sites"].clone();
    out
}

/// The fixture's scenario of this name.
fn scenario<'fixture>(fixture: &'fixture Fixture, name: &str) -> &'fixture Value {
    fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .find(|scenario| scenario["name"] == name)
        .unwrap_or_else(|| panic!("no scenario {name}"))
}

/// The (name, owed, Rust answer) of one step, named as `"<scenario> step <index>"`.
fn step_answer(step: &str) -> (String, Value, Value) {
    scenario_answers()
        .into_iter()
        .find(|(name, _, _)| name == step)
        .unwrap_or_else(|| panic!("no {step}"))
}

/// Run every scenario and return, per step, its name, the answer owed and the Rust answer.
fn scenario_answers() -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(MISSION);
    let mut answers = Vec::new();
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let max_range_m = scenario["init"]["max_range_m"].as_f64().unwrap() as f32;
        let mut stored = scenario["init"]["stored"].as_str().map(str::to_owned);
        let mut mission = Mission::load(stored.as_deref(), max_range_m);
        for (index, step) in scenario["steps"].as_array().unwrap().iter().enumerate() {
            let request = &step["in"];
            let properties = request["properties"].as_object();
            let changed = match request["action"].as_str().unwrap() {
                "observe" => None,
                "update" => Some(apply(&mut mission, properties.unwrap(), Mission::update)),
                "set_config" => {
                    let changed = apply(&mut mission, properties.unwrap(), Mission::merge);
                    stored = Some(mission.persisted().to_string());
                    Some(changed)
                }
                "reboot" => {
                    mission = Mission::load(stored.as_deref(), max_range_m);
                    None
                }
                other => panic!("{name}: unknown action {other}"),
            };
            let mut answer = state(&mission, fix(&request["fix"]));
            if let Some(changed) = changed {
                answer["changed"] = json!(changed);
            }
            answers.push((format!("{name} step {index}"), owed(step), answer));
        }
    }
    answers
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(MISSION);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], "micropython_plan_B");
    assert_eq!(meta["generator"], "mission");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/mission.py"));
    assert_eq!(fixture.compare(), Compare::Abs(5.01e-8));
}

#[test]
fn scenarios_match_plan_b() {
    let compare = Fixture::parse(MISSION).compare();
    let answers = scenario_answers();
    assert!(!answers.is_empty());
    let failures: Vec<String> = answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} mission steps differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
}

#[test]
fn the_sites_divergence_is_real_and_marked() {
    // plan B's own answer for a marked step must NOT pass: the Rust port keeps the sites it dropped
    let fixture = Fixture::parse(MISSION);
    let compare = fixture.compare();
    let reboot = &scenario(&fixture, "sites_survive_set_config")["steps"][2];
    assert_eq!(reboot["in"]["action"], "reboot");
    assert_eq!(reboot["out"]["sites"], json!([]), "plan B lost them at the reboot");
    assert_eq!(reboot["divergence"]["sites"].as_array().unwrap().len(), 3);
    let (name, owed, answer) = step_answer("sites_survive_set_config step 2");
    assert_eq!(compare.check(&name, &owed, &answer), Ok(()));
    let mut plan_b = reboot["out"].clone();
    plan_b["persisted"]["sites"] = json!([]);
    assert!(compare.check(&name, &plan_b, &answer).is_err());
}

#[test]
fn the_number_kind_divergence_is_real_and_marked() {
    // plan B echoes the integers it was sent; the Rust port writes the same values as JSON floats, and the
    // comparison holds an integer exact, so plan B's own answer for a marked step must NOT pass
    let fixture = Fixture::parse(MISSION);
    let compare = fixture.compare();
    let update = &scenario(&fixture, "numbers_sent_as_integers")["steps"][0];
    assert_eq!(update["out"]["inspect"]["latitude"], json!(48));
    assert_eq!(update["out"]["inspect"]["altitude"], json!(500));
    assert_eq!(update["out"]["persisted"]["zone"][0], json!([48, 11]));
    assert_eq!(update["divergence"]["floats"], true);
    let (name, owed, answer) = step_answer("numbers_sent_as_integers step 0");
    assert_eq!(owed["inspect"]["latitude"], json!(48.0));
    assert_eq!(compare.check(&name, &owed, &answer), Ok(()));
    let mut plan_b = update["out"].clone();
    plan_b["persisted"]["sites"] = plan_b["sites"].clone();
    assert!(compare.check(&name, &plan_b, &answer).is_err());
    // and a step plan B answers in floats carries no such mark
    assert!(scenario(&fixture, "zone_range")["steps"][0].get("divergence").is_none());
}

#[test]
fn a_degree_one_unit_off_fails_the_comparison() {
    // the tolerance is half a 1e-7 degree unit, the most an integer zone centre may round: one unit must fail
    let compare = Fixture::parse(MISSION).compare();
    let (name, owed, mut answer) = step_answer("hprc_default_updates step 0");
    assert_eq!(compare.check(&name, &owed, &answer), Ok(()));
    let latitude = answer["inspect"]["target"][0].as_f64().unwrap();
    answer["inspect"]["target"][0] = json!(latitude + 1e-7);
    assert!(compare.check(&name, &owed, &answer).is_err());
}

#[test]
fn time_sets_match_plan_b() {
    golden::run(&section("time_sets"), |input| {
        let properties = input.as_object().unwrap();
        let mut mission = Mission::load(None, DEFAULT_MAX_RANGE_M);
        let epoch = epoch_request(properties);
        let changed = apply(&mut mission, properties, Mission::update);
        let time_set = epoch.map(|_| {
            let time_set = TimeSet::from_properties(properties);
            json!({"utc_offset": time_set.utc_offset, "source": time_set.source,
                "cc_position": time_set.cc_position.map(Position::degrees)})
        });
        json!({"changed": changed, "epoch": epoch, "time_set": time_set})
    });
}

#[test]
fn clocks_match_plan_b() {
    golden::run(&section("clocks"), |input| {
        json!(clock_text(input.as_i64().expect("an epoch is an integer")))
    });
}
