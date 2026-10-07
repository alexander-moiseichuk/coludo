//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bring-up answers as plan B's: src/avionics/golden/controller.json, which golden/build_controller.py builds by
//! running src/glider controller.py's Controller (and cc_client.py's dispatcher over it) with stub parts on CPython,
//! every answer compared EXACTLY -- the log lines, the retry gaps, each alternatives pass, the failures and
//! alternatives, every verdict line CC prints.
//!
//! A `setup` case is replayed as the firmware brings a board up: board_config's directory of the case's config, one
//! [`set_up`] per entry with each attempt answered by the case's stub for that implementation, a [`Table`] over the
//! outcomes and the lines it ends the bring-up with, then the crashes the case lists, then the readers: every log line
//! and every reader's answer is the crate's own, none is formatted or filtered here. A `rerun` case replays each
//! alternatives pass plan B ran ([`sort_alternatives`]) from the state it started from.

use std::collections::HashMap;

use board_config::Entry;
use controller::{Attempt, Candidate, Table, candidate, set_up, sort_alternatives};
use golden::{Fixture, PLAN_B_TAG};
use serde_json::{Value, json};

/// plan B's Controller over stub parts: retry traces, every alternatives scenario, the board files after layout.
const CONTROLLER: &str = include_str!("../../../golden/controller.json");

/// How many attempts each stub class has seen, by implementation: a `flaky` stub counts across all its entries,
/// as plan B's class-level counter does.
type Counters = HashMap<String, u64>;

/// One attempt by the stub `specification` (None: no class, so no driver) for the entry `name`.
fn attempt(
    specification: Option<&Value>,
    implementation: &str,
    name: &str,
    last: bool,
    counters: &mut Counters,
) -> Attempt {
    let Some(specification) = specification else {
        return Attempt::NoDriver;
    };
    let count = counters.entry(implementation.to_string()).or_default();
    *count += 1;
    let diagnosis = || {
        specification
            .get("diagnose")
            .filter(|_| last)
            .map(|detail| detail.as_str().expect("a diagnosis is text").to_string())
    };
    let up = match specification["setup"].as_str().expect("a stub's setup") {
        "up" => specification
            .get("names")
            .and_then(Value::as_array)
            .is_none_or(|names| names.iter().any(|each| each == name)),
        "fail" => false,
        "flaky" => *count >= specification["up_at"].as_u64().expect("a flaky stub's up_at"),
        "raise" => {
            return Attempt::Raised {
                error: specification["error"]
                    .as_str()
                    .expect("a raising stub's error")
                    .to_string(),
                diagnosis: diagnosis(),
            };
        }
        other => panic!("unknown stub setup {other}"),
    };
    if up {
        Attempt::Up
    } else {
        Attempt::Failed { diagnosis: diagnosis() }
    }
}

/// `pairs` as a JSON list of [name, text] lists.
fn listed<'a>(pairs: impl IntoIterator<Item = (&'a str, String)>) -> Value {
    Value::Array(pairs.into_iter().map(|(name, text)| json!([name, text])).collect())
}

/// `pairs` as a JSON object, {name: text}.
fn mapped<'a, Text: Into<Value>>(pairs: impl IntoIterator<Item = (&'a str, Text)>) -> Value {
    Value::Object(
        pairs
            .into_iter()
            .map(|(name, text)| (name.to_string(), text.into()))
            .collect(),
    )
}

/// One alternatives pass as the fixture traces it: the up entries and the failures it starts from, then the pairs
/// and the failures it leaves, all by name.
fn sort_trace(candidates: &[Candidate], up: &[usize], failures: &[(usize, String)]) -> Value {
    let name = |index: usize| candidates[index].name;
    let mut after = failures.to_vec();
    let alternatives: Vec<Value> = sort_alternatives(candidates, up, &mut after)
        .into_iter()
        .map(|(loser, winner)| json!([name(loser), name(winner)]))
        .collect();
    json!({
        "tasks": up.iter().map(|index| name(*index)).collect::<Vec<_>>(),
        "failures": listed(failures.iter().map(|(index, reason)| (name(*index), reason.clone()))),
        "alternatives": alternatives,
        "after": listed(after.into_iter().map(|(index, reason)| (name(index), reason))),
    })
}

/// A `setup` case: the whole bring-up and every reader.
fn setup(input: &Value) -> Value {
    let document = &input["config"];
    let registry = input["registry"].as_object().expect("a registry");
    let directory: Vec<Entry> = board_config::directory(document);
    let attempts = board_config::setup_retries(document);
    let (mut log, mut sleeps, mut counters) = (Vec::new(), Vec::new(), Counters::new());
    let mut bring_up = Vec::new();
    for entry in &directory {
        let specification = registry.get(entry.implementation);
        let outcome = set_up(
            entry.name,
            entry.implementation,
            attempts,
            |_, last| attempt(specification, entry.implementation, entry.name, last, &mut counters),
            |milliseconds| sleeps.push(milliseconds),
            |line| log.push(line),
        );
        bring_up.push((candidate(document, entry), outcome));
    }

    let candidates: Vec<Candidate> = bring_up.iter().map(|(each, _)| each.clone()).collect();
    let up: Vec<usize> = (0..bring_up.len()).filter(|index| bring_up[*index].1.is_ok()).collect();
    let failures: Vec<(usize, String)> = bring_up
        .iter()
        .enumerate()
        .filter_map(|(index, (_, outcome))| outcome.clone().err().map(|reason| (index, reason)))
        .collect();
    let sorts = json!([sort_trace(&candidates, &up, &failures)]);

    let table = Table::new(bring_up);
    log.extend(table.bring_up_lines());
    for (name, reason) in input.get("crashes").and_then(Value::as_object).into_iter().flatten() {
        let entry = table.find(name).unwrap_or_else(|| panic!("no entry {name} to crash"));
        assert!(
            entry.crash(reason.as_str().expect("a crash reason").to_string()),
            "{name} did not crash"
        );
    }

    let inspection = table.inspect();
    let probed: Vec<Value> = input["probes"]
        .as_array()
        .expect("probe queries")
        .iter()
        .map(|query| json!(table.probed(query[0].as_str().expect("a probed name"), query[1].as_str())))
        .collect();
    let failure = table
        .entries()
        .iter()
        .filter_map(|entry| Some((entry.name, table.failure(entry.name)?)));
    json!({
        "log": log,
        "sleeps": sleeps,
        "sorts": sorts,
        "inspect": {
            "tasks": inspection.tasks,
            "failures": mapped(inspection.failures),
            "alternatives": mapped(inspection.alternatives),
        },
        "unfitted": mapped(table.alternatives()),
        "verify": {
            "devices": mapped(table.devices()),
            "problems": mapped(table.failures()),
            "alternatives": mapped(table.alternatives()),
        },
        "tasks": table.tasks().into_iter().map(|(name, ok)| json!({"name": name, "ok": ok})).collect::<Vec<_>>(),
        "probed": probed,
        "failure": mapped(failure),
    })
}

/// A `rerun` case: each alternatives pass plan B ran, from the state it started from.
fn rerun(input: &Value, expected: &Value) -> Value {
    let document = &input["config"];
    let directory = board_config::directory(document);
    let candidates: Vec<Candidate> = directory.iter().map(|entry| candidate(document, entry)).collect();
    let index = |name: &Value| {
        let name = name.as_str().expect("a device name");
        candidates
            .iter()
            .position(|each| each.name == name)
            .unwrap_or_else(|| panic!("{name} is not in the directory"))
    };
    let sorts: Vec<Value> = expected["sorts"]
        .as_array()
        .expect("traced passes")
        .iter()
        .map(|pass| {
            let up: Vec<usize> = pass["tasks"].as_array().expect("up tasks").iter().map(index).collect();
            let failures: Vec<(usize, String)> = pass["failures"]
                .as_array()
                .expect("failures")
                .iter()
                .map(|failure| (index(&failure[0]), failure[1].as_str().expect("a reason").to_string()))
                .collect();
            sort_trace(&candidates, &up, &failures)
        })
        .collect();
    json!({"sorts": sorts})
}

#[test]
fn the_fixture_is_plan_bs_and_compares_exactly() {
    let fixture = Fixture::parse(CONTROLLER);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], PLAN_B_TAG);
    assert_eq!(meta["generator"], "controller");
    assert!(meta["source"].as_str().unwrap().contains("src/glider/controller.py"));
    assert_eq!(fixture.compare(), golden::Compare::Exact);
}

#[test]
fn every_case_answers_as_plan_b() {
    let fixture = Fixture::parse(CONTROLLER);
    let compare = fixture.compare();
    let cases = fixture.get("cases").as_array().expect("cases");
    let mut functions: HashMap<&str, usize> = HashMap::new();
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| {
            let name = case["name"].as_str().expect("a case name");
            let function = case["input"]["function"].as_str().expect("a function");
            *functions.entry(function).or_default() += 1;
            let answer = match function {
                "setup" => setup(&case["input"]),
                "rerun" => rerun(&case["input"], &case["output"]),
                other => panic!("case {name}: unknown function {other}"),
            };
            compare.check(name, &case["output"], &answer).err()
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
    // every kind of case is there: a fixture that lost one would pass with nothing compared
    assert!(functions.get("setup") >= Some(&40), "{functions:?}");
    assert_eq!(functions.get("rerun"), Some(&1), "{functions:?}");
}

#[test]
fn the_fixture_holds_the_cases_the_port_plan_names() {
    let fixture = Fixture::parse(CONTROLLER);
    let cases = fixture.get("cases").as_array().expect("cases");
    let case = |name: &str| {
        cases
            .iter()
            .find(|case| case["name"] == name)
            .unwrap_or_else(|| panic!("no case {name}"))
    };
    // the taster: both lasers absent, so no winner, and both block arming
    let taster = &case("taster_both_lasers_absent")["output"];
    assert_eq!(taster["inspect"]["alternatives"], json!({}));
    for laser in ["laser_agl", "laser_agl_l1x"] {
        assert!(
            taster["verify"]["problems"].get(laser).is_some(),
            "{laser} must block arming"
        );
    }
    assert_eq!(
        taster["sleeps"],
        json!([200, 200, 200, 200]),
        "three attempts each, two gaps each"
    );
    // 7F: the M5 stage 5 hardware line, word for word
    assert_eq!(
        case("tms7f_the_l1x_fitted")["output"]["verify"]["alternatives"]["laser_agl"],
        "vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:1 0x29"
    );
    // 7C (v0.1): the loser pulses the shared XSHUT, so it is a failure naming the fix
    let tms7c = case("tms7c_both_lasers_on_xshut_gpio5")["output"]["inspect"]["failures"]["laser_agl"]
        .as_str()
        .expect("a failure reason");
    assert!(
        tms7c.ends_with(
            "shares XSHUT GPIO5 with laser_agl_l1x (vl53l1x): its setup resets the fitted laser -- \
                             enable exactly one of laser_agl, laser_agl_l1x on this board"
        ),
        "{tms7c}"
    );
}
