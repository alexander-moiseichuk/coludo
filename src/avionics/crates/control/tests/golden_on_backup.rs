//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The health flag `attitude-backup` answers as plan B's: src/avionics/golden/on_backup.json, which
//! golden/build_on_backup.py builds by running src/glider cc_client.py's `_on_backup` over the board-config corpus
//! (the launches board files, config_default and config_hitl), over config_default with its attitude providers
//! fitted, disabled, re-ranked and re-shaped, and over documents plan B cannot read as a config. Each case asks every
//! (channel, source) pair of its document and lists, per channel, the sources plan B answered True for; the port's
//! list must be the same, EXACTLY. A case names its document as golden/config_corpus.py describes: a committed board
//! file (compiled in below), a base the fixture carries, or the document itself, with edits (plain data moves) on top.

use control::attitude::on_backup;
use golden::{Compare, Fixture, PLAN_B_TAG};
use serde_json::{Value, json};

/// plan B's `_on_backup` over the corpus and its edits.
const ON_BACKUP: &str = include_str!("../../../golden/on_backup.json");

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

/// One case's answers, plan B's and the port's: `on_backup`, each channel mapped to the sources answered True, in
/// the order of the case's `sources`.
fn answers(bases: &Value, case: &Value) -> (Value, Value) {
    let document = document(bases, &case["config"]);
    let sources = case["sources"].as_array().expect("sources");
    let expected = case["on_backup"].clone();
    let channels = expected.as_object().expect("on_backup maps channels");
    let actual = channels
        .keys()
        .map(|channel| {
            let flying: Vec<Value> = sources
                .iter()
                .filter(|source| on_backup(&document, channel, source.as_str()))
                .cloned()
                .collect();
            (channel.clone(), Value::Array(flying))
        })
        .collect();
    (expected, Value::Object(actual))
}

/// The fixture's cases, each as (name, plan B's answers, the port's).
fn cases(fixture: &Fixture) -> Vec<(String, Value, Value)> {
    let bases = fixture.get("bases");
    let cases = fixture.get("cases").as_array().expect("cases");
    cases
        .iter()
        .map(|case| {
            let name = case["name"].as_str().expect("a case name").to_string();
            let (expected, actual) = answers(bases, case);
            (name, expected, actual)
        })
        .collect()
}

#[test]
fn the_fixture_is_plan_bs_and_compares_exactly() {
    let fixture = Fixture::parse(ON_BACKUP);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], PLAN_B_TAG);
    assert_eq!(meta["generator"], "on_backup");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/cc_client.py"));
    assert_eq!(fixture.compare(), Compare::Exact);
}

#[test]
fn every_answer_is_plan_bs() {
    let fixture = Fixture::parse(ON_BACKUP);
    let compare = fixture.compare();
    let cases = cases(&fixture);
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|(name, expected, actual)| compare.check(name, expected, actual).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
    // both answers occur, so a port answering one way throughout fails
    let pairs: usize = fixture
        .get("cases")
        .as_array()
        .unwrap()
        .iter()
        .map(|case| case["sources"].as_array().unwrap().len() * case["on_backup"].as_object().unwrap().len())
        .sum();
    let flying: usize = cases
        .iter()
        .flat_map(|(_, expected, _)| expected.as_object().unwrap().values())
        .map(|sources| sources.as_array().unwrap().len())
        .sum();
    assert!(pairs > 5000 && flying > 200 && flying < pairs, "{flying} of {pairs}");
}

#[test]
fn the_fixture_holds_the_cases_the_health_flag_exists_for() {
    let fixture = Fixture::parse(ON_BACKUP);
    let cases = cases(&fixture);
    let attitude = |name: &str| {
        cases
            .iter()
            .find(|(case, ..)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"))
            .1["attitude"]
            .clone()
    };
    // the SEN0253's BNO055 standing in for the filter is the backup; the filter flying is nominal
    assert_eq!(attitude("backup_fitted"), json!(["imu_bno055"]));
    assert_eq!(
        attitude("default"),
        json!([]),
        "no BNO055 fitted: nothing below the filter"
    );
    // a board file saved before the filter became the primary ranks the BNO055 above it
    assert_eq!(attitude("tms7c"), json!(["attitude"]));
    assert_eq!(attitude("tms7f"), json!([]), "7F's BNO055 is disabled");
    // NEGATIVE: the filter gone or disabled, the BNO055 is the best there is; a tie is no fallback
    for name in [
        "backup_fitted_filter_disabled",
        "backup_fitted_filter_absent",
        "backup_ties_the_filter",
        "backup_enabled_zero",
        "backup_enabled_empty_text",
    ] {
        assert_eq!(attitude(name), json!([]), "{name}");
    }
    // NEGATIVE: the HITL sim is no fallback, and nothing fresh never is
    assert_eq!(attitude("hitl"), json!([]));
    // a config plan B cannot read hides no fallback: any fresh source is on the backup
    assert_eq!(attitude("config_null"), json!(["hitl"]));
}

#[test]
fn a_wrong_answer_is_caught() {
    // the comparison is not vacuous: one source moved in plan B's answer fails the case
    let fixture = Fixture::parse(ON_BACKUP);
    let (name, mut expected, actual) = cases(&fixture)
        .into_iter()
        .find(|(name, ..)| name == "backup_fitted")
        .expect("the case");
    expected["attitude"] = json!(["attitude"]);
    assert!(fixture.compare().check(&name, &expected, &actual).is_err());
}
