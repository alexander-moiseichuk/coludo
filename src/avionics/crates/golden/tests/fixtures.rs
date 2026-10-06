//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The committed fixtures load and carry what their consumers rely on: provenance, a compare kind, and the
//! members each one promises (src/avionics/golden/build_*.py). cc_dispatch is also replayed end to end
//! through golden::dispatch, so the comparator and the fixture are known to agree before a port meets them.

use golden::dispatch::{self, ArgumentsKind, Case};
use golden::{Compare, Fixture};
use serde_json::{Map, Value, json};

/// plan B's built-in board config, which the firmware embeds until it has a config store.
const CONFIG_DEFAULT: &str = include_str!("../../../golden/config_default.json");
/// The CC dispatcher's error paths, envelope rules and reply shapes.
const CC_DISPATCH: &str = include_str!("../../../golden/cc_dispatch.json");

/// Every fixture says where it came from.
fn assert_provenance(fixture: &Fixture, generator: &str) {
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], generator);
    assert_eq!(meta["python"], "3.12", "the oracle's major.minor, never a patch level");
    for member in ["source", "commit", "glider_tree"] {
        assert!(
            meta[member].as_str().is_some_and(|text| !text.is_empty()),
            "meta.{member}"
        );
    }
}

#[test]
fn config_default_carries_the_document_and_its_id() {
    let fixture = Fixture::parse(CONFIG_DEFAULT);
    assert_provenance(&fixture, "config_default");
    assert_eq!(fixture.compare(), Compare::Exact);
    let config_id = fixture.get("config_id").as_str().unwrap();
    assert!(
        config_id.len() == 12
            && config_id
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    );
    let board = &fixture.get("document")["board"];
    assert_eq!(
        &board["mcu"],
        fixture.get("mcu"),
        "the firmware reads mcu at the top level"
    );
    assert_eq!(board["firmware_version"], "dev", "the frozen pin a consumer restamps");
    assert!(fixture.get("note").as_str().unwrap().contains("restamp"));
}

/// One reply that meets a case's answer: plan B's own for `exact`/`json`, and a stand-in for what the answer
/// leaves free (an `any` text, a `json_keys` value of the first type listed).
fn reply_meeting(case: &Case) -> Option<String> {
    let status = case.status.as_deref()?;
    let arguments: Vec<String> = match case.kind {
        ArgumentsKind::Exact => case
            .expected
            .iter()
            .map(|item| item.as_str().unwrap().to_string())
            .collect(),
        ArgumentsKind::Any => case
            .expected
            .iter()
            .map(|item| item.as_str().unwrap().to_string())
            .chain(["free text".to_string()])
            .collect(),
        ArgumentsKind::Json => case.expected.iter().map(Value::to_string).collect(),
        ArgumentsKind::JsonKeys => case
            .expected
            .iter()
            .map(|item| match item {
                Value::Object(types) => sample(types).to_string(),
                text => text.as_str().unwrap().to_string(),
            })
            .collect(),
    };
    let arguments: Vec<&str> = arguments.iter().map(String::as_str).collect();
    Some(cc_protocol::build(status, &arguments, &[]))
}

/// A JSON object with every key of `types`, each holding a value of the first type its specification lists.
fn sample(types: &Map<String, Value>) -> Value {
    let value = |specification: &Value| match specification.as_str().unwrap().split('|').next().unwrap() {
        "null" => Value::Null,
        "bool" => json!(false),
        "int" => json!(0),
        "float" => json!(0.5),
        "str" => json!(""),
        "list" => json!([]),
        _ => json!({}),
    };
    Value::Object(
        types
            .iter()
            .map(|(key, specification)| (key.clone(), value(specification)))
            .collect(),
    )
}

#[test]
fn cc_dispatch_cases_are_well_formed() {
    let fixture = Fixture::parse(CC_DISPATCH);
    assert_provenance(&fixture, "cc_dispatch");
    assert!(fixture.get("arguments_kinds").is_object());
    let cases = dispatch::cases(CC_DISPATCH);
    let empty = cases.iter().find(|case| case.request.is_empty()).unwrap();
    assert!(empty.status.is_none(), "an empty line has no reply");
    for kind in [
        ArgumentsKind::Exact,
        ArgumentsKind::Any,
        ArgumentsKind::Json,
        ArgumentsKind::JsonKeys,
    ] {
        assert!(cases.iter().any(|case| case.kind == kind), "no {kind:?} case");
    }
    assert!(cases.iter().any(|case| case.owed), "no owed answer");
}

#[test]
fn cc_dispatch_passes_a_port_that_meets_every_answer() {
    let cases = dispatch::cases(CC_DISPATCH);
    let mut replies = cases.iter().map(reply_meeting);
    dispatch::run(CC_DISPATCH, |_request| replies.next().unwrap());
}

#[test]
#[should_panic(expected = "dispatch cases failed")]
fn cc_dispatch_fails_a_port_that_answers_pong_to_everything() {
    dispatch::run(CC_DISPATCH, |_request| Some("pong".to_string()));
}

#[test]
fn cc_dispatch_health_admits_the_rust_board_s_own_keys() {
    let cases = dispatch::cases(CC_DISPATCH);
    let health = cases.iter().find(|case| case.name == "health").unwrap();
    let mut answer = reply_meeting(health).unwrap();
    assert_eq!(health.check(Some(&answer)), Ok(()));
    let Value::Object(mut object) = sample(health.expected[0].as_object().unwrap()) else {
        unreachable!()
    };
    object.insert("internal_free".to_string(), json!(250000));
    object.insert("sd_free_mb".to_string(), Value::Null);
    answer = cc_protocol::build("ok", &[&Value::Object(object.clone()).to_string()], &[]);
    assert_eq!(health.check(Some(&answer)), Ok(()));
    object.insert("psram_free".to_string(), json!(0));
    answer = cc_protocol::build("ok", &[&Value::Object(object).to_string()], &[]);
    assert!(health.check(Some(&answer)).unwrap_err().contains("psram_free"));
}
