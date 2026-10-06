//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The golden transcripts, replayed: the I2C steps a golden builder recorded from plan B's own driver
//! (src/avionics/golden/driver_bus.py writes them) become the scripted steps of the crate's mock bus, so the Rust
//! driver must make the same transactions, wait the same waits and drive the same pins, in the same order.

#![allow(dead_code)] // each golden test uses the part of this module its fixture needs

use std::collections::BTreeMap;

use drivers::mock::{MockBus, Step};
use golden::{Compare, Fixture};
use serde_json::Value;

/// The bytes a hex string names.
pub fn bytes(text: &str) -> Vec<u8> {
    assert!(text.len().is_multiple_of(2), "odd hex: {text}");
    (0..text.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&text[index..index + 2], 16).expect("hex"))
        .collect()
}

/// A JSON integer as an unsigned byte-sized or word-sized number; panics naming the member when it is not one.
fn unsigned(value: &Value, member: &str) -> u64 {
    value[member]
        .as_u64()
        .unwrap_or_else(|| panic!("transcript: '{member}' is not an unsigned integer in {value}"))
}

/// One transcript step as the mock's step.
fn step(entry: &Value) -> Step {
    let address = || u8::try_from(unsigned(entry, "address")).expect("a 7-bit address");
    let hex = |member: &str| {
        bytes(
            entry[member]
                .as_str()
                .unwrap_or_else(|| panic!("no '{member}' in {entry}")),
        )
    };
    match entry["op"].as_str() {
        Some("write") => Step::Write {
            address: address(),
            bytes: hex("bytes"),
        },
        Some("read") => Step::Read {
            address: address(),
            bytes: hex("bytes"),
        },
        Some("write_read") => Step::WriteRead {
            address: address(),
            write: hex("write"),
            read: hex("read"),
        },
        Some("delay_ms") => Step::Delay {
            nanoseconds: unsigned(entry, "ms") * 1_000_000,
        },
        Some("pin") => Step::Pin {
            high: entry["high"].as_bool().expect("a pin level"),
        },
        other => panic!("transcript: unknown op {other:?}"),
    }
}

/// Append a transcript's steps to the mock's script, the refused ones marked to fail.
pub fn script(mock: &MockBus, transcript: &Value) {
    for entry in transcript.as_array().expect("a transcript is a list") {
        mock.expect(step(entry));
        if entry.get("fails").and_then(Value::as_bool) == Some(true) {
            mock.then_fail();
        }
    }
}

/// A fresh mock bus scripted with a transcript.
pub fn replay(transcript: &Value) -> MockBus {
    let mock = MockBus::new();
    script(&mock, transcript);
    mock
}

/// A section of a sectioned fixture, checked to say where it came from: plan B's tag, the generator and a source.
pub fn section(text: &str, generator: &str, name: &str) -> Value {
    let fixture = Fixture::parse(text);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG, "{generator}: the plan B tag");
    assert_eq!(meta["generator"], generator);
    assert!(
        meta["source"]
            .as_str()
            .is_some_and(|source| source.contains("src/glider/drivers/")),
        "{generator}: {meta}"
    );
    fixture.get(name).clone()
}

/// A section's comparison, member by member: its `compare` block for every value, except the members its optional
/// `fields` block names (`{member: compare}`), which take their own comparison wherever they appear in an output.
///
/// One tolerance per section is right when every number in it is the same quantity. A barometer's reading is not:
/// its pressure carries the polynomial's f32 budget (Pa), its temperature is an exact power-of-two scaling, and its
/// altitude is metres. A pressure budget applied to the other two let a decode that drops a temperature byte, or a
/// pressure quantised before the altitude, pass every test, so each member is held to its own measured margin.
pub struct FieldCompare {
    /// The section's `compare`: every value no field names.
    default: Compare,
    /// The members the section's `fields` names, each with its own comparison.
    fields: BTreeMap<String, Compare>,
}

impl FieldCompare {
    /// The comparison a section names: `compare`, and `fields` when present (absent: `compare` everywhere).
    ///
    /// Panics on a `fields` that is not `{member: compare}`: a fixture that cannot say how it compares must fail.
    pub fn from_section(section: &Value) -> FieldCompare {
        let fields = match section.get("fields") {
            None => BTreeMap::new(),
            Some(Value::Object(members)) => members
                .iter()
                .map(|(member, block)| (member.clone(), Compare::from_json(block)))
                .collect(),
            Some(other) => panic!("transcript: fields {other} is not {{member: compare}}"),
        };
        FieldCompare {
            default: Compare::from_json(&section["compare"]),
            fields,
        }
    }

    /// Compare one expected output with the port's answer; `Err` names the case and the first difference.
    pub fn check(&self, case: &str, expected: &Value, actual: &Value) -> Result<(), String> {
        self.walk(case, "$", expected, actual)
    }

    /// The first difference at `path`: objects and lists are walked here so a named member meets its own
    /// comparison at any depth; everything else is the section's.
    fn walk(&self, case: &str, path: &str, expected: &Value, actual: &Value) -> Result<(), String> {
        match (expected, actual) {
            (Value::Object(expected_map), Value::Object(actual_map))
                if !self.fields.is_empty()
                    && expected_map.len() == actual_map.len()
                    && expected_map.keys().all(|member| actual_map.contains_key(member)) =>
            {
                expected_map.iter().try_for_each(|(member, item)| {
                    let at = format!("{path}.{member}");
                    match self.fields.get(member) {
                        Some(compare) => compare.check(&format!("{case} {at}"), item, &actual_map[member]),
                        None => self.walk(case, &at, item, &actual_map[member]),
                    }
                })
            }
            (Value::Array(expected_items), Value::Array(actual_items))
                if expected_items.len() == actual_items.len() && !self.fields.is_empty() =>
            {
                expected_items
                    .iter()
                    .zip(actual_items)
                    .enumerate()
                    .try_for_each(|(index, (item, answer))| self.walk(case, &format!("{path}[{index}]"), item, answer))
            }
            // a mismatched shape, a leaf, or no fields at all: the section's comparison names the difference
            _ => self.default.check(&format!("{case} {path}"), expected, actual),
        }
    }
}

/// Run a section's `{name, input, output}` cases through `port` under the section's comparison (its `fields`
/// included, see [`FieldCompare`]); panics listing every failing case, and on a section with no cases.
pub fn check_cases(section: &Value, label: &str, mut port: impl FnMut(&Value) -> Value) -> usize {
    let compare = FieldCompare::from_section(section);
    let cases = section["cases"].as_array().expect("cases");
    assert!(!cases.is_empty(), "{label}: no cases");
    let failures: Vec<String> = cases
        .iter()
        .filter_map(|case| {
            let name = format!("{label} {}", case["name"].as_str().expect("a case name"));
            compare.check(&name, &case["output"], &port(&case["input"])).err()
        })
        .collect();
    assert!(
        failures.is_empty(),
        "{} of {} {label} cases failed:\n{}",
        failures.len(),
        cases.len(),
        failures.join("\n")
    );
    cases.len()
}
