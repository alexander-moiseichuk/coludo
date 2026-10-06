//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC dispatcher fixture (`src/avionics/golden/cc_dispatch.json`): request lines sent IN ORDER to one
//! dispatcher, and the reply each must get. Plan B keeps state between commands (the stage, the log window,
//! the clock), so the order is part of the fixture.
//!
//! A case is `{name, request, status, arguments_kind, expected}`: `status` is the reply's first token (null
//! when no reply is due) and `expected` describes its decoded positional arguments by `arguments_kind`
//! ([`ArgumentsKind`]). Optional members:
//!  * `extra_keys` -- `{key: type}` a `json_keys` object may carry beyond the expected keys;
//!  * `owed` -- `{status, arguments_kind, expected}`: the answer the Rust board gives on purpose instead of
//!    plan B's. A port is checked against `owed` when it is present;
//!  * `note` -- prose for the reader.
//!
//! Reply lines are parsed with `cc_protocol`, as the hub parses them, so a base64-carried argument compares by
//! its decoded text.

use std::collections::BTreeSet;

use serde_json::{Map, Value};

use crate::{Compare, Fixture};

/// The JSON type names a `json_keys` expectation uses; `float|null` allows either.
pub const TYPE_NAMES: [&str; 7] = ["null", "bool", "int", "float", "str", "list", "object"];

/// How a case's `expected` list describes a reply's decoded positional arguments.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ArgumentsKind {
    /// Verbatim strings, as many as listed.
    Exact,
    /// The listed leading arguments (the error code) verbatim, then exactly one more: free text, such as a
    /// Python repr no port reproduces.
    Any,
    /// Each argument is JSON equal by value to the listed one (separators and key order are free; an integer
    /// never equals a float).
    Json,
    /// Each listed string matches verbatim; each listed `{key: type}` object wants a JSON object with exactly
    /// those keys (plus the case's `extra_keys`), each value of its type.
    JsonKeys,
}

impl ArgumentsKind {
    /// The kind a case names.
    ///
    /// Panics on an unknown name: a case that cannot say how it compares must fail its test, not pass it.
    pub fn from_name(name: &str) -> ArgumentsKind {
        match name {
            "exact" => ArgumentsKind::Exact,
            "any" => ArgumentsKind::Any,
            "json" => ArgumentsKind::Json,
            "json_keys" => ArgumentsKind::JsonKeys,
            _ => panic!("golden: unknown arguments_kind '{name}'"),
        }
    }

    /// Compare a reply's decoded positional arguments with `expected`.
    ///
    /// `extra_keys` (`{key: type}`) are the keys a `json_keys` object may carry beyond the expected ones.
    /// Returns `Err` naming the first difference.
    pub fn check(
        self,
        expected: &[Value],
        arguments: &[String],
        extra_keys: &Map<String, Value>,
    ) -> Result<(), String> {
        let count = match self {
            ArgumentsKind::Any => expected.len() + 1,
            _ => expected.len(),
        };
        if arguments.len() != count {
            return Err(format!(
                "expected {count} arguments, got {}: {arguments:?}",
                arguments.len()
            ));
        }
        expected
            .iter()
            .zip(arguments)
            .enumerate()
            .try_for_each(|(index, (wanted, argument))| match (self, wanted) {
                (ArgumentsKind::Json, _) => {
                    let answered = json_argument(index, argument)?;
                    Compare::Exact
                        .walk("$", wanted, &answered)
                        .map_err(|difference| format!("argument {index} at {difference}"))
                }
                (ArgumentsKind::JsonKeys, Value::Object(types)) => {
                    keys_and_types(types, &json_argument(index, argument)?, extra_keys)
                        .map_err(|difference| format!("argument {index}: {difference}"))
                }
                _ if wanted.as_str() == Some(argument.as_str()) => Ok(()),
                _ => Err(format!("argument {index}: expected {wanted}, got {argument:?}")),
            })
    }
}

/// One argument parsed as JSON, or why it is not JSON.
fn json_argument(index: usize, argument: &str) -> Result<Value, String> {
    serde_json::from_str(argument).map_err(|error| format!("argument {index} is not JSON ({error}): {argument:?}"))
}

/// The type name `json_keys` gives a value: an integer is `int`, any other number `float`.
fn type_name(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "bool",
        Value::Number(number) if number.is_f64() => "float",
        Value::Number(_) => "int",
        Value::String(_) => "str",
        Value::Array(_) => "list",
        Value::Object(_) => "object",
    }
}

/// Does the type specification (`float` or `float|null`) admit `value`?
fn admits(specification: &Value, value: &Value) -> bool {
    specification
        .as_str()
        .is_some_and(|text| text.split('|').any(|name| name == type_name(value)))
}

/// An answer that should be a JSON object holding exactly `types`' keys (plus any of `extra_keys`), each
/// value of its type; Err naming the missing, unexpected and mistyped keys.
fn keys_and_types(types: &Map<String, Value>, answered: &Value, extra_keys: &Map<String, Value>) -> Result<(), String> {
    let Value::Object(object) = answered else {
        return Err(format!("expected a JSON object, got {answered}"));
    };
    let missing: Vec<&String> = types.keys().filter(|key| !object.contains_key(*key)).collect();
    let unexpected: Vec<&String> = object
        .keys()
        .filter(|key| !types.contains_key(*key) && !extra_keys.contains_key(*key))
        .collect();
    let mistyped: Vec<String> = object
        .iter()
        .filter_map(|(key, value)| {
            let specification = types.get(key).or_else(|| extra_keys.get(key))?;
            (!admits(specification, value)).then(|| format!("{key} is {} not {specification}", type_name(value)))
        })
        .collect();
    if missing.is_empty() && unexpected.is_empty() && mistyped.is_empty() {
        return Ok(());
    }
    Err(format!(
        "missing keys {missing:?}, unexpected keys {unexpected:?}, mistyped {mistyped:?}"
    ))
}

/// One dispatcher case, checked and resolved: the answer a port must give is `owed`'s where the case has one.
#[derive(Debug)]
pub struct Case {
    /// Unique within the fixture.
    pub name: String,
    /// The request line, as the board receives it.
    pub request: String,
    /// The reply's first token; None when no reply is due.
    pub status: Option<String>,
    /// How `expected` describes the reply's arguments.
    pub kind: ArgumentsKind,
    /// The arguments, as `kind` describes them.
    pub expected: Vec<Value>,
    /// `{key: type}` a `json_keys` object may carry beyond the expected keys.
    pub extra_keys: Map<String, Value>,
    /// Whether the answer is the `owed` one, which the Rust board gives on purpose instead of plan B's.
    pub owed: bool,
}

impl Case {
    /// Compare a port's reply line (None for no reply) with this case's answer.
    ///
    /// Returns `Err` naming the case and the first difference.
    pub fn check(&self, reply: Option<&str>) -> Result<(), String> {
        let name = &self.name;
        let answer = if self.owed {
            "the owed answer"
        } else {
            "plan B's answer"
        };
        let (status, line) = match (&self.status, reply) {
            (None, None) => return Ok(()),
            (None, Some(line)) => return Err(format!("case '{name}': expected no reply, got {line:?}")),
            (Some(status), None) => return Err(format!("case '{name}': expected {status} ({answer}), got no reply")),
            (Some(status), Some(line)) => (status, line),
        };
        let message = cc_protocol::parse(line)
            .map_err(|error| format!("case '{name}': the reply does not parse ({error:?}): {line:?}"))?;
        if message.command.as_deref() != Some(status.as_str()) || !message.named.is_empty() {
            return Err(format!(
                "case '{name}': expected {status} with no named parameters ({answer}), got {line:?}"
            ));
        }
        self.kind
            .check(&self.expected, &message.args, &self.extra_keys)
            .map_err(|difference| format!("case '{name}' ({answer}): {difference}; reply {line:?}"))
    }
}

/// Refuse a type specification that names a type outside [`TYPE_NAMES`].
fn check_specification(name: &str, key: &str, specification: &Value) {
    let known = specification
        .as_str()
        .is_some_and(|text| text.split('|').all(|type_name| TYPE_NAMES.contains(&type_name)));
    assert!(
        known,
        "golden: case '{name}': {key} has the unknown type {specification}"
    );
}

/// The answer an answer block (a case, or its `owed`) describes: (status, kind, expected), checked.
fn answer(name: &str, block: &Value) -> (Option<String>, ArgumentsKind, Vec<Value>) {
    let status = match &block["status"] {
        Value::Null => None,
        Value::String(status) => Some(status.clone()),
        other => panic!("golden: case '{name}': status {other} is neither a string nor null"),
    };
    let kind_name = block["arguments_kind"].as_str();
    let kind =
        ArgumentsKind::from_name(kind_name.unwrap_or_else(|| panic!("golden: case '{name}' has no arguments_kind")));
    let expected = block["expected"]
        .as_array()
        .unwrap_or_else(|| panic!("golden: case '{name}': expected is not a list"))
        .clone();
    assert!(
        status.is_some() || expected.is_empty(),
        "golden: case '{name}': no reply, yet arguments expected"
    );
    match kind {
        ArgumentsKind::Exact | ArgumentsKind::Any => {
            let listed = kind == ArgumentsKind::Exact || !expected.is_empty();
            assert!(listed, "golden: case '{name}': an any case lists the code");
            let texts = expected.iter().all(Value::is_string);
            assert!(texts, "golden: case '{name}': {kind:?} arguments are strings");
        }
        ArgumentsKind::Json => {}
        ArgumentsKind::JsonKeys => {
            for item in &expected {
                if let Value::Object(types) = item {
                    types
                        .iter()
                        .for_each(|(key, specification)| check_specification(name, key, specification));
                } else {
                    assert!(
                        item.is_string(),
                        "golden: case '{name}': {item} is neither a string nor {{key: type}}"
                    );
                }
            }
        }
    }
    (status, kind, expected)
}

/// The cases of a dispatcher fixture, checked: known kinds and type names, an `any` case that lists its code,
/// unique names, `extra_keys` only on a `json_keys` answer.
///
/// Panics naming the first broken case, and when there are no cases.
pub fn cases(text: &str) -> Vec<Case> {
    let fixture = Fixture::parse(text);
    let blocks = fixture.get("cases").as_array().expect("golden: 'cases' is a list");
    assert!(!blocks.is_empty(), "golden: the fixture has no cases");
    let mut names = BTreeSet::new();
    blocks
        .iter()
        .map(|block| {
            let name = block["name"]
                .as_str()
                .expect("golden: every case has a name")
                .to_string();
            assert!(names.insert(name.clone()), "golden: case '{name}' is named twice");
            let request = block["request"]
                .as_str()
                .unwrap_or_else(|| panic!("golden: case '{name}' has no request"))
                .to_string();
            let owed = block.get("owed");
            let (status, kind, expected) = answer(&name, owed.unwrap_or(block));
            let extra_keys = match block.get("extra_keys") {
                None => Map::new(),
                Some(Value::Object(extra_keys)) => {
                    assert_eq!(
                        kind,
                        ArgumentsKind::JsonKeys,
                        "golden: case '{name}': extra_keys need json_keys"
                    );
                    extra_keys
                        .iter()
                        .for_each(|(key, specification)| check_specification(&name, key, specification));
                    extra_keys.clone()
                }
                Some(other) => panic!("golden: case '{name}': extra_keys {other} is not {{key: type}}"),
            };
            Case {
                name,
                request,
                status,
                kind,
                expected,
                extra_keys,
                owed: owed.is_some(),
            }
        })
        .collect()
}

/// Send every request of a dispatcher fixture to `port`, in order, and check each reply line (None for no
/// reply) against the case's answer.
///
/// Panics listing every failing case, and when the fixture is broken (see [`cases`]).
pub fn run(text: &str, mut port: impl FnMut(&str) -> Option<String>) {
    let cases = cases(text);
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| case.check(port(&case.request).as_deref()).err())
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} dispatch cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}

#[cfg(test)]
mod tests {
    use super::{ArgumentsKind, Case, cases, run};
    use serde_json::{Map, Value, json};

    /// The arguments as owned strings.
    fn arguments(texts: &[&str]) -> Vec<String> {
        texts.iter().map(|text| text.to_string()).collect()
    }

    /// The kind's verdict on `texts` against `expected`, with no extra keys.
    fn verdict(kind: ArgumentsKind, expected: Value, texts: &[&str]) -> Result<(), String> {
        kind.check(expected.as_array().unwrap(), &arguments(texts), &Map::new())
    }

    #[test]
    fn exact_wants_every_argument_verbatim() {
        let expected = json!(["badargs", "no object nosuch"]);
        assert_eq!(
            verdict(ArgumentsKind::Exact, expected.clone(), &["badargs", "no object nosuch"]),
            Ok(())
        );
        assert!(verdict(ArgumentsKind::Exact, expected.clone(), &["badargs", "no object Nosuch"]).is_err());
        assert!(verdict(ArgumentsKind::Exact, expected.clone(), &["badargs"]).is_err());
        assert!(verdict(ArgumentsKind::Exact, expected, &["badargs", "no object nosuch", "more"]).is_err());
        assert_eq!(verdict(ArgumentsKind::Exact, json!([]), &[]), Ok(()));
    }

    #[test]
    fn any_wants_the_code_and_exactly_one_free_text() {
        assert_eq!(
            verdict(ArgumentsKind::Any, json!(["internal"]), &["internal", "TypeError(...)"]),
            Ok(())
        );
        assert!(verdict(ArgumentsKind::Any, json!(["internal"]), &["badargs", "TypeError(...)"]).is_err());
        assert!(verdict(ArgumentsKind::Any, json!(["internal"]), &["internal"]).is_err());
        assert!(verdict(ArgumentsKind::Any, json!(["internal"]), &["internal", "a", "b"]).is_err());
    }

    #[test]
    fn json_compares_by_value() {
        let expected = json!([{"stage": "setting", "manual": false}]);
        assert_eq!(
            verdict(
                ArgumentsKind::Json,
                expected.clone(),
                &[r#"{"manual":false,"stage":"setting"}"#]
            ),
            Ok(())
        );
        let failure = verdict(
            ArgumentsKind::Json,
            expected.clone(),
            &[r#"{"manual":true,"stage":"setting"}"#],
        );
        assert!(failure.unwrap_err().starts_with("argument 0 at $.manual"));
        assert!(
            verdict(ArgumentsKind::Json, expected, &["not json"])
                .unwrap_err()
                .contains("is not JSON")
        );
        assert!(verdict(ArgumentsKind::Json, json!([{"dropped": 0}]), &[r#"{"dropped":0.0}"#]).is_err());
    }

    #[test]
    fn json_keys_wants_the_keys_and_their_types() {
        let expected = json!(["taster", {"epoch": "int", "temp": "float|null", "stage": "str"}]);
        let answer = |json: &str| verdict(ArgumentsKind::JsonKeys, expected.clone(), &["taster", json]);
        assert_eq!(
            answer(r#"{"epoch": 946684810, "temp": 41.5, "stage": "setting"}"#),
            Ok(())
        );
        assert_eq!(
            answer(r#"{"epoch": 946684810, "temp": null, "stage": "setting"}"#),
            Ok(())
        );
        assert!(
            answer(r#"{"epoch": 946684810.5, "temp": 41.5, "stage": "setting"}"#).is_err(),
            "float epoch"
        );
        assert!(
            answer(r#"{"epoch": 946684810, "temp": "41.5", "stage": "setting"}"#).is_err(),
            "text temp"
        );
        assert!(
            answer(r#"{"epoch": 946684810, "temp": 41.5}"#)
                .unwrap_err()
                .contains(r#"missing keys ["stage"]"#)
        );
        let failure = answer(r#"{"epoch": 1, "temp": 1.5, "stage": "setting", "psram_free": 0}"#).unwrap_err();
        assert!(failure.contains(r#"unexpected keys ["psram_free"]"#), "{failure}");
        assert!(answer("[1, 2]").unwrap_err().contains("expected a JSON object"));
        let failure = verdict(ArgumentsKind::JsonKeys, expected, &["other", r#"{"epoch": 1}"#]).unwrap_err();
        assert!(failure.starts_with("argument 0: expected \"taster\""), "{failure}");
    }

    #[test]
    fn json_keys_admits_the_extra_keys_with_their_types() {
        let expected = json!([{"uptime": "int"}]);
        let extra_keys: Map<String, Value> = json!({"sd_free_mb": "int|null"}).as_object().unwrap().clone();
        let check =
            |json: &str| ArgumentsKind::JsonKeys.check(expected.as_array().unwrap(), &arguments(&[json]), &extra_keys);
        assert_eq!(check(r#"{"uptime": 5}"#), Ok(()));
        assert_eq!(check(r#"{"uptime": 5, "sd_free_mb": 29000}"#), Ok(()));
        assert_eq!(check(r#"{"uptime": 5, "sd_free_mb": null}"#), Ok(()));
        assert!(
            check(r#"{"uptime": 5, "sd_free_mb": "full"}"#)
                .unwrap_err()
                .contains("sd_free_mb is str")
        );
    }

    /// A three-case fixture: no reply, a plan B answer, and an owed one.
    const SAMPLE: &str = r#"{"meta": {}, "cases": [
        {"name": "empty", "request": "", "status": null, "arguments_kind": "exact", "expected": []},
        {"name": "ping", "request": "ping", "status": "pong", "arguments_kind": "exact", "expected": []},
        {"name": "five", "request": "update mission 5", "status": "err", "arguments_kind": "any",
         "expected": ["internal"], "owed": {"status": "err", "arguments_kind": "any", "expected": ["refused"]}}]}"#;

    /// A port that answers SAMPLE the way the Rust board owes it.
    fn rust_board(request: &str) -> Option<String> {
        match request {
            "" => None,
            "ping" => Some("pong".to_string()),
            _ => Some(cc_protocol::build(
                "err",
                &["refused", "the properties must be a JSON object"],
                &[],
            )),
        }
    }

    #[test]
    fn run_passes_a_port_that_answers_every_case() {
        run(SAMPLE, rust_board);
    }

    #[test]
    fn run_sends_the_requests_in_order() {
        let mut sent = Vec::new();
        run(SAMPLE, |request| {
            sent.push(request.to_string());
            rust_board(request)
        });
        assert_eq!(sent, ["", "ping", "update mission 5"]);
    }

    #[test]
    #[should_panic(expected = "3 of 3 dispatch cases failed:\ncase 'empty': expected no reply, got \"pong x\"")]
    fn run_lists_every_failing_case() {
        run(SAMPLE, |_request| Some("pong x".to_string()));
    }

    #[test]
    fn an_owed_answer_replaces_plan_b_s() {
        let five: Case = cases(SAMPLE).pop().unwrap();
        assert!(five.owed);
        assert_eq!(five.check(Some("err refused base64:dGV4dA==")), Ok(()));
        let failure = five.check(Some("err internal TypeError")).unwrap_err();
        assert!(
            failure.starts_with("case 'five' (the owed answer): argument 0"),
            "{failure}"
        );
    }

    #[test]
    fn a_reply_is_checked_for_its_status_and_presence() {
        let ping = cases(SAMPLE).remove(1);
        assert_eq!(
            ping.check(Some("PONG")),
            Ok(()),
            "the command is lowercased, as the hub reads it"
        );
        assert!(ping.check(Some("ok")).is_err());
        assert!(
            ping.check(Some("pong x=1"))
                .unwrap_err()
                .contains("no named parameters")
        );
        assert!(ping.check(None).unwrap_err().contains("got no reply"));
        assert!(
            ping.check(Some("pong base64:@@@"))
                .unwrap_err()
                .contains("does not parse")
        );
    }

    /// A fixture holding just `case` (one case object, or several separated by commas).
    fn with_case(case: &str) -> String {
        format!(r#"{{"meta": {{}}, "cases": [{case}]}}"#)
    }

    #[test]
    #[should_panic(expected = "unknown arguments_kind 'roughly'")]
    fn an_unknown_kind_is_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "x", "status": "ok", "arguments_kind": "roughly", "expected": []}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'a': temp has the unknown type \"decimal\"")]
    fn an_unknown_type_name_is_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "x", "status": "ok", "arguments_kind": "json_keys", "expected": [{"temp": "decimal"}]}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'a': uptime has the unknown type \"long\"")]
    fn an_unknown_extra_key_type_is_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "x", "status": "ok", "arguments_kind": "json_keys", "expected": [{}], "extra_keys": {"uptime": "long"}}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'a': extra_keys need json_keys")]
    fn extra_keys_outside_json_keys_are_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "x", "status": "ok", "arguments_kind": "json", "expected": [{}], "extra_keys": {"uptime": "int"}}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'a': an any case lists the code")]
    fn an_any_case_without_its_code_is_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "x", "status": "err", "arguments_kind": "any", "expected": []}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'a': no reply, yet arguments expected")]
    fn no_reply_with_arguments_is_refused() {
        cases(&with_case(
            r#"{"name": "a", "request": "", "status": null, "arguments_kind": "exact", "expected": ["x"]}"#,
        ));
    }

    #[test]
    #[should_panic(expected = "case 'twice' is named twice")]
    fn a_repeated_name_is_refused() {
        let case =
            r#"{"name": "twice", "request": "ping", "status": "pong", "arguments_kind": "exact", "expected": []}"#;
        cases(&with_case(&format!("{case}, {case}")));
    }

    #[test]
    #[should_panic(expected = "the fixture has no cases")]
    fn a_fixture_without_cases_is_refused() {
        cases(r#"{"cases": []}"#);
    }
}
