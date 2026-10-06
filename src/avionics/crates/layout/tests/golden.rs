//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Layout answers as plan B's does: src/avionics/golden/layout.json, which golden/build_layout.py builds from
//! src/glider/layout.py on CPython, every answer compared EXACTLY -- the verdicts and their evidence text, the
//! boot-log lines, every change string, every edit to the config, and the config_id (crates/board_config) of the
//! config the board then runs, the M5 vectors of the port plan among them.
//!
//! A case's config is named as golden/config_corpus.py describes: a committed board file (compiled in below) or the
//! fixture's `default` base, then edits (plain data moves). `edits` in an output is the structural difference the
//! call made, computed here exactly as the builder computes it.

use board_config::config_id;
use golden::{Compare, Fixture};
use layout::{Addresses, Resolved, Revision, Scan, apply, detect, fit, fitted, rank, resolve, verdict};
use serde_json::{Map, Value, json};

/// plan B's layout over test_layout.py's scans and cases, the board files, dead devices and random scans.
const LAYOUT: &str = include_str!("../../../golden/layout.json");

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
fn edit(document: &mut Value, edit: &Value) {
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

/// The config a case names (a file or the base, then its edits), as plan B was shown it.
fn config(bases: &Value, specification: &Value) -> Value {
    let mut document: Value = if let Some(path) = specification.get("file") {
        let path = path.as_str().expect("a path");
        let text = LAUNCH_FILES
            .iter()
            .find(|(name, _)| *name == path)
            .unwrap_or_else(|| panic!("{path} is not compiled in"))
            .1;
        serde_json::from_str(text).expect("a board file is JSON")
    } else {
        let base = specification["base"].as_str().expect("a file or a base");
        bases.get(base).unwrap_or_else(|| panic!("no base {base}")).clone()
    };
    for change in specification
        .get("edits")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
    {
        edit(&mut document, change);
    }
    document
}

/// The structural difference between two documents, exactly as the builder's `_diff` writes it: depth first, objects
/// by sorted key (a member on one side only has that side alone), equal-length lists by index, any other difference
/// one {path, before, after}. Values are the same only when their JSON is (an int is never a float or a bool).
fn difference(before: &Value, after: &Value, path: &mut Vec<Value>, edits: &mut Vec<Value>) {
    match (before, after) {
        (Value::Object(left), Value::Object(right)) => {
            let mut keys: Vec<&String> = left.keys().chain(right.keys()).collect();
            keys.sort_unstable();
            keys.dedup();
            for key in keys {
                path.push(json!(key));
                match (left.get(key), right.get(key)) {
                    (Some(old), None) => edits.push(json!({"path": path.clone(), "before": old})),
                    (None, Some(new)) => edits.push(json!({"path": path.clone(), "after": new})),
                    (Some(old), Some(new)) => difference(old, new, path, edits),
                    (None, None) => unreachable!("a key from one side or the other"),
                }
                path.pop();
            }
        }
        (Value::Array(left), Value::Array(right)) if left.len() == right.len() => {
            for (index, (old, new)) in left.iter().zip(right).enumerate() {
                path.push(json!(index));
                difference(old, new, path, edits);
                path.pop();
            }
        }
        _ if before == after => {}
        _ => edits.push(json!({"path": path.clone(), "before": before, "after": after})),
    }
}

/// The edits between two documents (see [`difference`]).
fn edits(before: &Value, after: &Value) -> Value {
    let mut found = Vec::new();
    difference(before, after, &mut Vec::new(), &mut found);
    Value::Array(found)
}

/// The scripted hardware of a case: the closure [`detect`] and [`resolve`] take, standing for plan B's `I2C(...)` and
/// its `scan()`. It answers the case's scan for the bus it is asked, or fails for a bus in `raises`, and records every
/// bus it was asked: the bus ids plan B opened an I2C on, `scanned`.
fn hardware<'scanned>(
    input: &Value,
    scanned: &'scanned mut Vec<u8>,
) -> impl FnMut(u8, &Map<String, Value>) -> Result<Addresses, &'static str> + 'scanned {
    let seen = scan(input);
    let raises: Vec<u8> = input
        .get("raises")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .map(|bus| bus.as_u64().and_then(|bus| u8::try_from(bus).ok()).expect("a bus id"))
        .collect();
    move |bus, _spec| {
        scanned.push(bus);
        if raises.contains(&bus) {
            Err("a shorted bus")
        } else {
            Ok(seen[usize::from(bus)])
        }
    }
}

/// A scan as the fixture writes it, {"0": [...], "1": [...]}.
fn scan(input: &Value) -> Scan {
    let bus = |id: &str| -> Addresses {
        input["scan"][id]
            .as_array()
            .expect("a bus's addresses")
            .iter()
            .map(|address| {
                address
                    .as_u64()
                    .and_then(|address| u8::try_from(address).ok())
                    .expect("an address")
            })
            .collect()
    };
    [bus("0"), bus("1")]
}

/// A revision as the fixture names it; None for null or any other name ('undecided', 'auto').
fn revision(value: &Value) -> Option<Revision> {
    value.as_str().and_then(Revision::from_name)
}

/// A revision's name, or null.
fn named(revision: Option<Revision>) -> Value {
    revision.map_or(Value::Null, |revision| json!(revision.name()))
}

/// The `found` list of names, or None for null.
fn names(value: &Value) -> Option<Vec<&str>> {
    value
        .as_array()
        .map(|items| items.iter().map(|name| name.as_str().expect("a device name")).collect())
}

/// Run one case's call on the port and answer in the fixture's shape.
fn port(bases: &Value, input: &Value) -> Value {
    match input["function"].as_str() {
        Some("verdict") => {
            let verdict = verdict(&scan(input));
            json!({"revision": named(verdict.revision), "detail": verdict.detail, "found": verdict.found})
        }
        Some("detect") => {
            let document = config(bases, &input["config"]);
            let mut scanned = Vec::new();
            let (revision, detail) = detect(&document, hardware(input, &mut scanned));
            json!({"revision": named(revision), "detail": detail, "scanned": scanned})
        }
        Some("resolve") => {
            let before = config(bases, &input["config"]);
            let mut document = before.clone();
            let mut scanned = Vec::new();
            let resolved: Resolved = resolve(&mut document, hardware(input, &mut scanned));
            json!({"revision": named(resolved.revision()), "resolved": resolved.label(),
                   "log": resolved.log_lines(), "scanned": scanned, "edits": edits(&before, &document),
                   "config_id": config_id(&document)})
        }
        Some("apply") => {
            let before = config(bases, &input["config"]);
            let mut document = before.clone();
            let changes: Vec<Vec<String>> = input["applies"]
                .as_array()
                .expect("applies is a list")
                .iter()
                .map(|call| {
                    let revision = revision(&call["revision"]).expect("apply takes a revision");
                    apply(&mut document, revision, names(&call["found"]).as_deref())
                })
                .collect();
            json!({"changes": changes, "edits": edits(&before, &document)})
        }
        Some("rank") => {
            let before = config(bases, &input["config"]);
            let mut document = before.clone();
            let mut changes = Vec::new();
            rank(&mut document, &mut changes, revision(&input["revision"]));
            json!({"changes": changes, "edits": edits(&before, &document)})
        }
        Some("fit") => {
            let before = config(bases, &input["config"]);
            let mut document = before.clone();
            let mut changes = Vec::new();
            let present = input["present"].as_bool().expect("present is a bool");
            fit(
                &mut document,
                input["device"].as_str().expect("a device"),
                present,
                &mut changes,
            );
            json!({"changes": changes, "edits": edits(&before, &document)})
        }
        Some("fitted") => {
            let on = revision(&input["revision"]);
            let answers: Map<String, Value> = input["devices"]
                .as_array()
                .expect("devices is a list")
                .iter()
                .map(|device| {
                    let device = device.as_str().expect("a device name");
                    (device.to_string(), json!(fitted(device, on)))
                })
                .collect();
            Value::Object(answers)
        }
        other => panic!("unknown function {other:?}"),
    }
}

/// Every case's name, plan B's answer and the port's.
fn answers(text: &str) -> Vec<(String, Value, Value)> {
    let fixture = Fixture::parse(text);
    let bases = fixture.get("bases");
    fixture
        .get("cases")
        .as_array()
        .expect("cases is a list")
        .iter()
        .map(|case| {
            let name = case["name"].as_str().expect("a case has a name").to_string();
            (name, case["output"].clone(), port(bases, &case["input"]))
        })
        .collect()
}

/// The failures among the port's answers under this comparison.
fn failures(compare: Compare, answers: &[(String, Value, Value)]) -> Vec<String> {
    answers
        .iter()
        .filter_map(|(name, owed, answer)| compare.check(name, owed, answer).err())
        .collect()
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(LAYOUT);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "layout");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/layout.py"));
    assert_eq!(fixture.compare(), Compare::Exact);
}

#[test]
fn every_case_matches_plan_b_exactly() {
    let answers = answers(LAYOUT);
    assert!(answers.len() > 500, "only {} cases compared", answers.len());
    let failures = failures(Compare::Exact, &answers);
    assert!(
        failures.is_empty(),
        "{} of {} layout cases differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
}

#[test]
fn the_published_m5_config_ids_hold() {
    // avionics-port.md M5: tms7c.config as-is resolves as a declared v0.1 (no key, and the builder adds none);
    // the explicit key hashes differently; tms7f scanned as the taster is v1.1 with no SEN0253
    let answers = answers(LAYOUT);
    let published = [
        (
            "m5_tms7c_as_is_resolves_declared_v0_1",
            "f6a9c35d6372",
            "v0.1 (declared)",
        ),
        (
            "m5_tms7c_with_an_explicit_layout_v0_1",
            "92d537918c7c",
            "v0.1 (declared)",
        ),
        ("m5_tms7f_scanned_on_the_taster", "d18751e70f6c", "v1.1"),
    ];
    for (name, id, label) in published {
        let (_, owed, answer) = answers
            .iter()
            .find(|(case, _, _)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"));
        assert_eq!(answer["config_id"], id, "{name}: the port");
        assert_eq!(owed["config_id"], id, "{name}: plan B");
        assert_eq!(answer["resolved"], label, "{name}");
    }
}

#[test]
fn the_m5_board_proof_scan_is_pinned() {
    // avionics-port.md M5: the taster's live scan, i2c:0 {0x15 0x18 0x36 0x40 0x47 0x69} and i2c:1 {0x25 0x63},
    // detects v1.1; the CC `detect` reply on the board must be this case's detail, word for word
    let answers = answers(LAYOUT);
    let case = |name: &str| {
        answers
            .iter()
            .find(|(case, _, _)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"))
    };
    let (_, owed, answer) = case("m5_detect_taster");
    assert_eq!(owed["revision"], "v1.1");
    let detail = owed["detail"].as_str().unwrap();
    assert!(detail.starts_with("i2c0=['0x15', '0x18', '0x36', '0x40', '0x47', '0x69'] i2c1=['0x25', '0x63']"));
    assert_eq!(answer["detail"], detail);
    for name in [
        "m5_verdict_taster",
        "m5_resolve_tms7f_on_the_live_taster_scan",
        "m5_resolve_default_on_the_live_taster_scan",
    ] {
        let (_, owed, answer) = case(name);
        assert_eq!(owed["revision"], "v1.1", "{name}");
        assert_eq!(answer, owed, "{name}");
    }
}

#[test]
fn the_scan_guard_cases_scan_only_defined_buses() {
    // plan B's own _scan ran in the builder: a bus the config does not define (or gives no pins) was never opened
    let answers = answers(LAYOUT);
    let scanned = |name: &str| -> Value {
        let (_, owed, answer) = answers
            .iter()
            .find(|(case, _, _)| case == name)
            .unwrap_or_else(|| panic!("no case {name}"));
        assert_eq!(answer["scanned"], owed["scanned"], "{name}");
        owed["scanned"].clone()
    };
    assert_eq!(scanned("detect_no_i2c1_spec"), json!([0]));
    assert_eq!(scanned("detect_an_i2c1_spec_without_scl"), json!([0]));
    assert_eq!(scanned("detect_no_i2c0_spec"), json!([1]));
    assert_eq!(scanned("resolve_no_buses"), json!([]));
    // NEGATIVE: a failing bus WAS opened -- it is the answer that is empty, not the attempt
    assert_eq!(scanned("detect_a_failing_i2c1"), json!([0, 1]));
}

#[test]
fn a_wrong_edit_or_a_wrong_word_is_caught() {
    // NEGATIVE: the exact comparison is not a blanket -- one change string or one priority off fails its case
    let text = LAYOUT.replacen("imu_bno055 attitude p0->p2", "imu_bno055 attitude p0->p3", 1);
    assert_ne!(text, LAYOUT, "the fixture logs a BNO055 demotion");
    assert!(!failures(Compare::Exact, &answers(&text)).is_empty());
    let text = LAYOUT.replacen("\"config_id\": \"f6a9c35d6372\"", "\"config_id\": \"f6a9c35d6373\"", 1);
    assert_ne!(text, LAYOUT);
    assert_eq!(failures(Compare::Exact, &answers(&text)).len(), 1);
}
