//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-readiness gate answers as plan B's: src/avionics/golden/readiness.json, which
//! golden/build_readiness.py builds by running src/glider cc_client.py's `_readiness` over the board-config corpus
//! (the launches board files, the defaults, config_hitl and the flight-ready configs) and over copies with each gate
//! tripped, with plan B's own Mission holding the zone and the sites each case asks for. Every verdict compares
//! EXACTLY. A case names its document as golden/config_corpus.py describes: a committed board file (compiled in
//! below), a base the fixture carries, or the document itself, with edits (plain data moves) on top.

use controller::readiness;
use golden::{Compare, Fixture, PLAN_B_TAG};
use serde_json::{Value, json};

/// plan B's `_readiness` over the corpus and each gate tripped.
const READINESS: &str = include_str!("../../../golden/readiness.json");

/// Every launches/20261003/*/*.config, by its repository path, as the bytes git holds.
const LAUNCH_FILES: [(&str, &str); 5] = [
    (
        "launches/20261003/TMS-7C/tms7c.config",
        include_str!("../../../../../launches/20261003/TMS-7C/tms7c.config"),
    ),
    (
        "launches/20261003/TMS-7D/tms7d.config",
        include_str!("../../../../../launches/20261003/TMS-7D/tms7d.config"),
    ),
    (
        "launches/20261003/TMS-7D/tms7d_control.config",
        include_str!("../../../../../launches/20261003/TMS-7D/tms7d_control.config"),
    ),
    (
        "launches/20261003/TMS-7E/tms7e.config",
        include_str!("../../../../../launches/20261003/TMS-7E/tms7e.config"),
    ),
    (
        "launches/20261003/TMS-7F/tms7f.config",
        include_str!("../../../../../launches/20261003/TMS-7F/tms7f.config"),
    ),
];

/// The member of `value` an edit path step names: an object key or a list index.
fn step<'value>(value: &'value mut Value, key: &Value) -> &'value mut Value {
    let found = match key {
        Value::String(key) => value.get_mut(key.as_str()),
        Value::Number(index) => value.get_mut(index.as_u64().expect("a list index") as usize),
        _ => None,
    };
    found.unwrap_or_else(|| panic!("edit path step {key} names nothing"))
}

/// Apply one fixture edit: set (or append, one past a list's end) or delete the member at its path.
fn apply(document: &mut Value, edit: &Value) {
    let path = edit["path"].as_array().expect("an edit has a path");
    let (last, parents) = path.split_last().expect("an edit path is not empty");
    let parent = parents.iter().fold(document, step);
    let delete = edit["delete"] == true;
    let value = || edit.get("value").expect("a set edit has a value").clone();
    match (parent, last) {
        (Value::Object(members), Value::String(key)) if delete => {
            members.remove(key).expect("a deleted key exists");
        }
        (Value::Object(members), Value::String(key)) => {
            members.insert(key.clone(), value());
        }
        (Value::Array(items), Value::Number(index)) => {
            let index = index.as_u64().expect("a list index") as usize;
            if delete {
                items.remove(index);
            } else if index == items.len() {
                items.push(value());
            } else {
                items[index] = value();
            }
        }
        (parent, last) => panic!("edit {edit}: {last} does not address a member of {parent}"),
    }
}

/// The document a case's `config` names, with its edits applied.
fn document(bases: &Value, specification: &Value) -> Value {
    let forms = ["file", "base", "document"].map(|form| specification.get(form));
    let mut document = match forms {
        [Some(path), None, None] => {
            let path = path.as_str().expect("a path");
            let (_, text) = LAUNCH_FILES
                .iter()
                .find(|(name, _)| *name == path)
                .unwrap_or_else(|| panic!("{path} is not compiled in"));
            serde_json::from_str(text).expect("a board file is JSON")
        }
        [None, Some(base), None] => {
            let name = base.as_str().expect("a base name");
            bases.get(name).unwrap_or_else(|| panic!("no base {name}")).clone()
        }
        [None, None, Some(document)] => document.clone(),
        _ => panic!("{specification} names its document by none or several of file, base, document"),
    };
    for edit in specification
        .get("edits")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        apply(&mut document, edit);
    }
    document
}

/// The port's verdict on one case input `{config, zone, sites}`.
fn port(bases: &Value, input: &Value) -> Value {
    let flag = |member: &str| input[member].as_bool().unwrap_or_else(|| panic!("{member} is a bool"));
    json!(readiness(
        &document(bases, &input["config"]),
        flag("zone"),
        flag("sites")
    ))
}

/// The fixture's cases, each as (name, plan B's verdict, the port's).
fn answers(fixture: &Fixture) -> Vec<(String, Value, Value)> {
    let bases = fixture.get("bases");
    let cases = fixture.get("cases").as_array().expect("cases");
    cases
        .iter()
        .map(|case| {
            let name = case["name"].as_str().expect("a case name").to_string();
            (name, case["output"].clone(), port(bases, &case["input"]))
        })
        .collect()
}

#[test]
fn the_fixture_is_plan_bs_and_compares_exactly() {
    let fixture = Fixture::parse(READINESS);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], PLAN_B_TAG);
    assert_eq!(meta["generator"], "readiness");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/cc_client.py"));
    assert_eq!(fixture.compare(), Compare::Exact);
}

#[test]
fn every_case_answers_as_plan_b() {
    let fixture = Fixture::parse(READINESS);
    let compare = fixture.compare();
    let answers = answers(&fixture);
    let failures: Vec<String> = answers
        .iter()
        .filter_map(|(name, expected, actual)| compare.check(name, expected, actual).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} cases failed:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
    // every gate trips somewhere and passes somewhere: a fixture that lost one would pass with nothing compared
    for key in ["watchdog", "flight", "gains", "fins.limit_multiplier", "zone"] {
        assert!(
            answers.iter().any(|(_, expected, _)| expected.get(key).is_some()),
            "no case trips {key}"
        );
    }
    assert!(
        answers.iter().any(|(_, expected, _)| expected == &json!({})),
        "no case is flight-ready"
    );
}

#[test]
fn the_fixture_holds_the_corpus_and_the_cases_the_port_plan_names() {
    let fixture = Fixture::parse(READINESS);
    let answers = answers(&fixture);
    let verdict = |name: &str| {
        &answers
            .iter()
            .find(|(case, ..)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"))
            .1
    };
    // the shipped board files: flight disabled on all but 7D's control config, whose loop has no gain at all
    for board in ["tms7c", "tms7d", "tms7e", "tms7f"] {
        assert_eq!(
            verdict(board),
            &json!({"flight": "disabled: fins hold neutral -- ballistic"}),
            "{board}"
        );
    }
    assert_eq!(
        verdict("tms7d_control"),
        &json!({"gains": "zero/unset on roll, pitch, yaw -- the loop computes but cannot act"})
    );
    assert_eq!(verdict("flight_ready_tms7f"), &json!({}));
    assert_eq!(verdict("flight_ready_new"), &json!({}));
    assert!(verdict("hitl").get("watchdog").is_some());
    assert_eq!(
        verdict("watchdog_absent")["watchdog"],
        verdict("watchdog_disabled")["watchdog"]
    );
    for axis in ["roll", "pitch", "yaw"] {
        assert_eq!(
            verdict(&format!("gains_{axis}_zero"))["gains"],
            format!("zero/unset on {axis} -- the loop computes but cannot act")
        );
    }
    // a term that is not a number acts by Python's truthiness, and a bool multiplier is Python's int
    assert_eq!(
        verdict("gains_terms_that_are_not_numbers")["gains"],
        "zero/unset on pitch -- the loop computes but cannot act"
    );
    for (case, printed) in [
        ("half", "0.5"),
        ("tiny", "1e-05"),
        ("large_int", "1.23457e+08"),
        ("false", "0"),
    ] {
        assert_eq!(
            verdict(&format!("multiplier_{case}"))["fins.limit_multiplier"],
            format!("{printed}: a bench derating is still applied")
        );
    }
    assert_eq!(verdict("multiplier_true"), &json!({}));
    assert!(verdict("no_zone_no_sites").get("zone").is_some());
    assert_eq!(verdict("no_zone_with_sites"), &json!({}));
}

#[test]
fn a_wrong_word_in_a_verdict_is_caught() {
    // the comparison is not vacuous: one changed word in plan B's verdict fails the case
    let fixture = Fixture::parse(READINESS);
    let (name, mut expected, actual) = answers(&fixture)
        .into_iter()
        .find(|(name, ..)| name == "no_zone_no_sites")
        .expect("the case");
    expected["zone"] = json!("no landing zone set and no CC-less site to select one from");
    assert!(fixture.compare().check(&name, &expected, &actual).is_err());
}
