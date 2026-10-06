//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Golden vectors: the fixtures `src/avionics/golden/generate.py` writes from plan B (src/glider run on
//! CPython), loaded and compared against the Rust port. Dev-only: a crate takes this as a dev-dependency and
//! its test does `golden::run(include_str!("../../../golden/<fixture>.json"), |input| port(input))`, so the
//! fixture is compiled in and the test reads no file.
//!
//! A fixture says how its values compare (`"compare": {"kind": ..., "tol": ...}`):
//!  * `exact` -- equal JSON values; an integer never equals a float;
//!  * `abs` -- numbers within `tol`;
//!  * `rel` -- numbers within `tol` times the expected magnitude;
//!  * `angle_deg` -- numbers within `tol` degrees the short way around the circle (359.9 and 0.1 are 0.2
//!    apart);
//!  * `bytes_hex` -- strings of hex digit pairs naming the same bytes, either case.
//!
//! Under every kind the structure (keys, lengths) and every other value compare exactly: an expected INTEGER
//! (a count, a flag, a raw reading, an `int()` or `//` result) wants the same integer, never a float near it;
//! the tolerance applies only where plan B answered a float. Expected values and tolerances are f64 because
//! they come from JSON, the boundary decision D18 allows; a port's f32 widens to f64 exactly, so the
//! comparison adds no rounding of its own.
//!
//! The CC dispatcher fixture compares reply lines instead of values: see [`dispatch`].

pub mod dispatch;

use serde_json::Value;

/// How a fixture's expected values compare with a port's answers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Compare {
    /// Equal JSON values.
    Exact,
    /// Numbers within this absolute tolerance.
    Abs(f64),
    /// Numbers within this fraction of the expected magnitude.
    Rel(f64),
    /// Angles in degrees within this tolerance, the short way around.
    AngleDeg(f64),
    /// Strings of hex digit pairs naming the same bytes, either case; any other string compares exactly.
    BytesHex,
}

impl Compare {
    /// The comparison a fixture's `compare` block names, such as `{"kind": "abs", "tol": 0.02}`.
    ///
    /// Panics on an unknown kind, or a numeric kind without a non-negative `tol`: a fixture that cannot say
    /// how it compares must fail the test that loads it, not pass it.
    pub fn from_json(block: &Value) -> Compare {
        let tolerance = || {
            block["tol"]
                .as_f64()
                .filter(|tolerance| *tolerance >= 0.0)
                .unwrap_or_else(|| panic!("golden: compare {block} needs a non-negative tol"))
        };
        match block["kind"].as_str() {
            Some("exact") => Compare::Exact,
            Some("abs") => Compare::Abs(tolerance()),
            Some("rel") => Compare::Rel(tolerance()),
            Some("angle_deg") => Compare::AngleDeg(tolerance()),
            Some("bytes_hex") => Compare::BytesHex,
            _ => panic!("golden: unknown compare kind in {block}"),
        }
    }

    /// Compare one expected value with the port's answer, all the way down.
    ///
    /// Returns `Err` with the case name, the path of the first difference inside the value (`$` is the value
    /// itself) and both values there.
    pub fn check(self, case: &str, expected: &Value, actual: &Value) -> Result<(), String> {
        self.walk("$", expected, actual)
            .map_err(|difference| format!("case '{case}' at {difference}"))
    }

    /// The first difference between `expected` and `actual` at `path`, as "<path>: expected .., got ..".
    fn walk(self, path: &str, expected: &Value, actual: &Value) -> Result<(), String> {
        let differ =
            |detail: &str| -> Result<(), String> { Err(format!("{path}: expected {expected}, got {actual}{detail}")) };
        match (expected, actual) {
            (Value::Object(expected_map), Value::Object(actual_map)) => {
                let missing: Vec<&String> = expected_map
                    .keys()
                    .filter(|key| !actual_map.contains_key(*key))
                    .collect();
                let extra: Vec<&String> = actual_map
                    .keys()
                    .filter(|key| !expected_map.contains_key(*key))
                    .collect();
                if !missing.is_empty() || !extra.is_empty() {
                    return differ(&format!(" (missing keys {missing:?}, unexpected keys {extra:?})"));
                }
                expected_map
                    .iter()
                    .try_for_each(|(key, item)| self.walk(&format!("{path}.{key}"), item, &actual_map[key]))
            }
            (Value::Array(expected_items), Value::Array(actual_items)) => {
                if expected_items.len() != actual_items.len() {
                    return differ(&format!(" (length {} vs {})", expected_items.len(), actual_items.len()));
                }
                expected_items
                    .iter()
                    .zip(actual_items)
                    .enumerate()
                    .try_for_each(|(index, (item, answer))| self.walk(&format!("{path}[{index}]"), item, answer))
            }
            // an integer is exact under every kind: no tolerance turns a wrong count into a right one
            (Value::Number(expected_number), Value::Number(_)) if !expected_number.is_f64() => {
                if expected == actual {
                    Ok(())
                } else {
                    differ(" (an integer compares exactly)")
                }
            }
            (Value::Number(expected_number), Value::Number(actual_number)) => {
                let wanted = expected_number.as_f64().expect("a JSON number reads as f64");
                let answered = actual_number.as_f64().expect("a JSON number reads as f64");
                match self.error_and_limit(wanted, answered) {
                    Some((error, limit)) if error > limit => differ(&format!(" (off by {error}, allowed {limit})")),
                    Some(_) => Ok(()),
                    None if expected == actual => Ok(()),
                    None => differ(""),
                }
            }
            (Value::String(expected_text), Value::String(actual_text))
                if self == Compare::BytesHex && is_hex_bytes(expected_text) && is_hex_bytes(actual_text) =>
            {
                if expected_text.eq_ignore_ascii_case(actual_text) {
                    return Ok(());
                }
                let same = expected_text
                    .as_bytes()
                    .chunks(2)
                    .zip(actual_text.as_bytes().chunks(2))
                    .take_while(|(wanted, answered)| wanted.eq_ignore_ascii_case(answered))
                    .count();
                differ(&format!(" (bytes differ from byte {same})"))
            }
            _ if expected == actual => Ok(()),
            _ => differ(""),
        }
    }

    /// How far a numeric answer is from the expected value, and how far it may be; None for the kinds that
    /// compare numbers exactly.
    fn error_and_limit(self, wanted: f64, answered: f64) -> Option<(f64, f64)> {
        match self {
            Compare::Abs(tolerance) => Some(((answered - wanted).abs(), tolerance)),
            Compare::Rel(tolerance) => Some(((answered - wanted).abs(), tolerance * wanted.abs())),
            Compare::AngleDeg(tolerance) => {
                let around = (answered - wanted).rem_euclid(360.0);
                Some((around.min(360.0 - around), tolerance))
            }
            Compare::Exact | Compare::BytesHex => None,
        }
    }
}

/// Is `text` bytes written as hex: an even count of ASCII hex digits? Only such strings compare case-blind.
fn is_hex_bytes(text: &str) -> bool {
    text.len().is_multiple_of(2) && text.bytes().all(|byte| byte.is_ascii_hexdigit())
}

/// A fixture document, loaded from the text the caller compiled in.
#[derive(Debug)]
pub struct Fixture {
    /// The whole document: `meta`, `compare` and the fixture's own members.
    document: Value,
}

impl Fixture {
    /// Load a fixture's text.
    ///
    /// Panics with the parse error when the text is not a JSON object: a broken fixture fails its test.
    pub fn parse(text: &str) -> Fixture {
        let document: Value = serde_json::from_str(text).unwrap_or_else(|error| panic!("golden: not JSON: {error}"));
        assert!(document.is_object(), "golden: a fixture is a JSON object");
        Fixture { document }
    }

    /// A top-level member, such as `meta`, `cases` or a fixture's own `config_id`.
    ///
    /// Panics naming the member when the fixture lacks it.
    pub fn get(&self, member: &str) -> &Value {
        self.document
            .get(member)
            .unwrap_or_else(|| panic!("golden: fixture has no '{member}'"))
    }

    /// How this fixture's values compare.
    pub fn compare(&self) -> Compare {
        Compare::from_json(self.get("compare"))
    }
}

/// Run every `{name, input, output}` case of a fixture through `port` and compare its answer by the fixture's
/// kind.
///
/// Panics listing every failing case, and also when the fixture has no cases or a case lacks its input or
/// output (a misspelt member would otherwise read as null and pass a port that answers null): a run that
/// compared nothing must not read as a pass.
pub fn run(text: &str, mut port: impl FnMut(&Value) -> Value) {
    let fixture = Fixture::parse(text);
    let compare = fixture.compare();
    let cases = fixture.get("cases").as_array().expect("golden: 'cases' is a list");
    assert!(!cases.is_empty(), "golden: the fixture has no cases");
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| {
            let name = case["name"].as_str().expect("golden: every case has a name");
            let member = |member: &str| {
                case.get(member)
                    .unwrap_or_else(|| panic!("golden: case '{name}' has no '{member}'"))
            };
            compare.check(name, member("output"), &port(member("input"))).err()
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} golden cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
}

/// The lowercase hex of `bytes`: how a port hands bytes to a `bytes_hex` comparison.
pub fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::{Compare, Fixture, hex, run};
    use serde_json::{Value, json};

    /// The comparison's verdict, with the failure text on a miss.
    fn verdict(compare: Compare, expected: Value, actual: Value) -> Result<(), String> {
        compare.check("sample", &expected, &actual)
    }

    #[test]
    fn exact_matches_equal_values_and_names_the_first_difference() {
        let value = json!({"stage": "setting", "ids": [1, 2], "manual": false, "nothing": null});
        assert_eq!(verdict(Compare::Exact, value.clone(), value), Ok(()));
        let failure = verdict(Compare::Exact, json!({"ids": [1, 2]}), json!({"ids": [1, 3]})).unwrap_err();
        assert!(
            failure.starts_with("case 'sample' at $.ids[1]: expected 2, got 3"),
            "{failure}"
        );
    }

    #[test]
    fn exact_tells_integers_from_floats() {
        assert!(verdict(Compare::Exact, json!(2.0), json!(2)).is_err());
        assert!(verdict(Compare::Exact, json!(2), json!(2.0)).is_err());
    }

    #[test]
    fn structure_differences_fail_under_every_kind() {
        let failure = verdict(Compare::Abs(1.0), json!({"a": 1, "b": 2}), json!({"a": 1, "c": 2})).unwrap_err();
        assert!(
            failure.contains(r#"missing keys ["b"], unexpected keys ["c"]"#),
            "{failure}"
        );
        let failure = verdict(Compare::Rel(0.5), json!([1, 2]), json!([1, 2, 3])).unwrap_err();
        assert!(failure.contains("length 2 vs 3"), "{failure}");
        assert!(verdict(Compare::Abs(1.0), json!({"name": "a"}), json!({"name": "b"})).is_err());
        assert!(verdict(Compare::AngleDeg(1.0), json!(1.0), json!("1.0")).is_err());
    }

    #[test]
    fn abs_allows_the_tolerance_and_no_more() {
        assert_eq!(verdict(Compare::Abs(0.02), json!(1.5), json!(1.51)), Ok(()));
        assert_eq!(verdict(Compare::Abs(0.02), json!(-3.0), json!(-3.01)), Ok(()));
        let failure = verdict(Compare::Abs(0.02), json!({"roll": 1.5}), json!({"roll": 1.53})).unwrap_err();
        assert!(
            failure.starts_with("case 'sample' at $.roll: expected 1.5, got 1.53 (off by"),
            "{failure}"
        );
    }

    #[test]
    fn integers_compare_exactly_under_every_kind() {
        for compare in [Compare::Abs(0.02), Compare::Rel(0.001), Compare::AngleDeg(0.25)] {
            assert_eq!(verdict(compare, json!(7), json!(7)), Ok(()), "{compare:?}");
            assert_eq!(verdict(compare, json!(-3), json!(-3)), Ok(()), "{compare:?}");
            let failure = verdict(compare, json!(3), json!(3.01)).unwrap_err();
            assert!(
                failure.ends_with("(an integer compares exactly)"),
                "{compare:?}: {failure}"
            );
            assert!(
                verdict(compare, json!(7), json!(7.0)).is_err(),
                "{compare:?}: int vs float"
            );
            assert!(
                verdict(compare, json!(7), json!(8)).is_err(),
                "{compare:?}: int off by one"
            );
            assert!(
                verdict(compare, json!(-3), json!(-2)).is_err(),
                "{compare:?}: int off by one"
            );
        }
        assert!(verdict(Compare::Rel(0.001), json!(100000), json!(100090)).is_err());
        assert!(verdict(Compare::AngleDeg(1.0), json!(359), json!(0)).is_err());
    }

    #[test]
    fn a_float_expectation_takes_an_integer_answer_within_the_tolerance() {
        assert_eq!(verdict(Compare::Abs(1.0), json!(255147635.0), json!(255147635)), Ok(()));
        assert!(verdict(Compare::Abs(1.0), json!(255147635.0), json!(255147637)).is_err());
    }

    #[test]
    fn abs_accepts_an_f32_answer_against_its_f64_expectation() {
        let answer = 0.1_f32;
        assert_eq!(
            verdict(Compare::Abs(1e-6), json!(0.1), json!(f64::from(answer))),
            Ok(())
        );
        assert!(verdict(Compare::Exact, json!(0.1), json!(f64::from(answer))).is_err());
    }

    #[test]
    fn rel_scales_with_the_expected_magnitude() {
        assert_eq!(verdict(Compare::Rel(0.001), json!(100000.0), json!(100090.0)), Ok(()));
        assert!(verdict(Compare::Rel(0.001), json!(1.0), json!(1.002)).is_err());
        assert_eq!(verdict(Compare::Rel(0.001), json!(0.0), json!(0.0)), Ok(()));
        assert!(verdict(Compare::Rel(0.001), json!(0.0), json!(1e-9)).is_err());
    }

    #[test]
    fn angle_deg_wraps_at_360() {
        assert_eq!(verdict(Compare::AngleDeg(0.25), json!(359.9), json!(0.1)), Ok(()));
        assert_eq!(verdict(Compare::AngleDeg(0.25), json!(0.1), json!(-0.1)), Ok(()));
        assert_eq!(verdict(Compare::AngleDeg(0.25), json!(-180.0), json!(180.0)), Ok(()));
        assert!(verdict(Compare::AngleDeg(1.0), json!(10.0), json!(12.0)).is_err());
        assert!(verdict(Compare::AngleDeg(1.0), json!(0.0), json!(180.0)).is_err());
    }

    #[test]
    fn bytes_hex_compares_other_strings_exactly() {
        assert!(
            verdict(
                Compare::BytesHex,
                json!({"stage": "Setting"}),
                json!({"stage": "setting"})
            )
            .is_err()
        );
        assert!(verdict(Compare::BytesHex, json!("xyz"), json!("XYZ")).is_err());
        assert!(
            verdict(Compare::BytesHex, json!("abc"), json!("ABC")).is_err(),
            "an odd count is no bytes"
        );
        assert_eq!(verdict(Compare::BytesHex, json!("setting"), json!("setting")), Ok(()));
        let failure = verdict(Compare::BytesHex, json!("b56206"), json!("b5620g")).unwrap_err();
        assert!(!failure.contains("bytes differ"), "{failure}");
    }

    #[test]
    fn bytes_hex_ignores_case_and_points_at_the_first_differing_byte() {
        assert_eq!(verdict(Compare::BytesHex, json!("b562aa"), json!("B562AA")), Ok(()));
        assert_eq!(
            verdict(Compare::BytesHex, json!("b56206"), json!(hex(&[0xb5, 0x62, 0x06]))),
            Ok(())
        );
        let failure = verdict(Compare::BytesHex, json!("b56206"), json!("b56207")).unwrap_err();
        assert!(failure.ends_with("(bytes differ from byte 2)"), "{failure}");
        assert!(verdict(Compare::BytesHex, json!("b562"), json!("b56200")).is_err());
        assert!(verdict(Compare::BytesHex, json!(1), json!(2)).is_err());
    }

    #[test]
    fn compare_blocks_parse_by_kind() {
        assert_eq!(Compare::from_json(&json!({"kind": "exact"})), Compare::Exact);
        assert_eq!(
            Compare::from_json(&json!({"kind": "abs", "tol": 0.02})),
            Compare::Abs(0.02)
        );
        assert_eq!(
            Compare::from_json(&json!({"kind": "rel", "tol": 1e-3})),
            Compare::Rel(1e-3)
        );
        assert_eq!(
            Compare::from_json(&json!({"kind": "angle_deg", "tol": 0.1})),
            Compare::AngleDeg(0.1)
        );
        assert_eq!(Compare::from_json(&json!({"kind": "bytes_hex"})), Compare::BytesHex);
    }

    #[test]
    #[should_panic(expected = "unknown compare kind")]
    fn an_unknown_kind_is_refused() {
        Compare::from_json(&json!({"kind": "close_enough"}));
    }

    #[test]
    #[should_panic(expected = "needs a non-negative tol")]
    fn a_numeric_kind_without_a_tolerance_is_refused() {
        Compare::from_json(&json!({"kind": "abs"}));
    }

    #[test]
    #[should_panic(expected = "needs a non-negative tol")]
    fn a_negative_tolerance_is_refused() {
        Compare::from_json(&json!({"kind": "rel", "tol": -0.1}));
    }

    /// A two-case fixture doubling its input.
    const DOUBLING: &str = r#"{"meta": {"generator": "doubling"}, "compare": {"kind": "abs", "tol": 0.01},
        "cases": [{"name": "one", "input": 1.0, "output": 2.0}, {"name": "half", "input": 0.5, "output": 1.0}]}"#;

    #[test]
    fn run_passes_a_port_that_answers_every_case() {
        run(DOUBLING, |input| json!(input.as_f64().unwrap() * 2.0));
    }

    #[test]
    #[should_panic(expected = "2 of 2 golden cases failed:\ncase 'one' at $: expected 2.0, got 3.0")]
    fn run_lists_every_failing_case() {
        run(DOUBLING, |input| json!(input.as_f64().unwrap() * 3.0));
    }

    #[test]
    #[should_panic(expected = "golden: case 'typo' has no 'output'")]
    fn run_refuses_a_case_without_its_output() {
        let text = r#"{"compare": {"kind": "exact"}, "cases": [{"name": "typo", "input": 1, "outputs": 42}]}"#;
        run(text, |_input| Value::Null);
    }

    #[test]
    #[should_panic(expected = "golden: case 'bare' has no 'input'")]
    fn run_refuses_a_case_without_its_input() {
        run(
            r#"{"compare": {"kind": "exact"}, "cases": [{"name": "bare", "output": 1}]}"#,
            |input| input.clone(),
        );
    }

    #[test]
    #[should_panic(expected = "the fixture has no cases")]
    fn run_refuses_a_fixture_without_cases() {
        run(r#"{"compare": {"kind": "exact"}, "cases": []}"#, |input| input.clone());
    }

    #[test]
    fn fixture_members_are_reachable() {
        let fixture = Fixture::parse(DOUBLING);
        assert_eq!(fixture.get("meta")["generator"], "doubling");
        assert_eq!(fixture.compare(), Compare::Abs(0.01));
    }

    #[test]
    #[should_panic(expected = "fixture has no 'config_id'")]
    fn a_missing_member_is_named() {
        Fixture::parse(DOUBLING).get("config_id");
    }

    #[test]
    #[should_panic(expected = "golden: not JSON")]
    fn text_that_is_not_json_is_refused() {
        Fixture::parse("{\"compare\": ");
    }

    #[test]
    #[should_panic(expected = "a fixture is a JSON object")]
    fn a_fixture_is_an_object() {
        Fixture::parse("[1, 2]");
    }
}
