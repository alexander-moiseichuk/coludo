//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Grouped golden fixtures: one answer, several units. A navigation or guidance answer mixes metres, degrees and
//! exact values (a leg, a flag, a heading error in whole degrees), and each needs its own tolerance. So these
//! fixtures (navigation.json, guidance.json, wind.json) carry a `compare` MAP, group name -> golden compare
//! block, and every expected output is an object keyed by those group names: `{"degrees": ..., "exact": ...}`.
//! Each group is then compared by its own kind with `golden::Compare`; integers stay exact under every kind.
//!
//! The fixtures' sections are `{note, cases: [{name, input, output}]}` (stateless) or, for the stateful units,
//! `{note, cases: [{name, init, steps: [{in, out}]}]}`, which each test walks itself and checks step by step.

// every test binary that includes this module uses a part of it
#![allow(dead_code)]

use golden::Compare;
use serde_json::Value;

/// A grouped fixture document, loaded from the text the test compiled in.
pub struct Grouped {
    /// The whole document: `meta`, `compare`, `tolerance` and the sections.
    document: Value,
}

impl Grouped {
    /// Load a grouped fixture's text; panics when it is not JSON or has no `compare` map.
    pub fn parse(text: &str) -> Grouped {
        let document: Value = serde_json::from_str(text).unwrap_or_else(|error| panic!("golden: not JSON: {error}"));
        assert!(
            document["compare"].is_object(),
            "golden: a grouped fixture has a compare map"
        );
        Grouped { document }
    }

    /// A top-level member (`meta`, a section); panics naming it when the fixture lacks it.
    pub fn get(&self, member: &str) -> &Value {
        self.document
            .get(member)
            .unwrap_or_else(|| panic!("golden: fixture has no '{member}'"))
    }

    /// A section's cases; panics when there are none, since a run that compared nothing must not pass.
    pub fn cases(&self, section: &str) -> &Vec<Value> {
        let cases = self.get(section)["cases"]
            .as_array()
            .unwrap_or_else(|| panic!("golden: section '{section}' has no case list"));
        assert!(!cases.is_empty(), "golden: section '{section}' has no cases");
        cases
    }

    /// Compare one expected output with the port's answer, group by group.
    ///
    /// Both must be objects with the same group names, and every name must be in the `compare` map. Returns `Err`
    /// naming the case, the group and the first difference.
    pub fn check(&self, case: &str, expected: &Value, actual: &Value) -> Result<(), String> {
        let (Some(expected_groups), Some(actual_groups)) = (expected.as_object(), actual.as_object()) else {
            return Err(format!(
                "case '{case}': an output is an object of groups, got {expected} / {actual}"
            ));
        };
        let mut expected_names: Vec<&String> = expected_groups.keys().collect();
        let mut actual_names: Vec<&String> = actual_groups.keys().collect();
        expected_names.sort();
        actual_names.sort();
        if expected_names != actual_names {
            return Err(format!(
                "case '{case}': groups {expected_names:?} expected, {actual_names:?} answered"
            ));
        }
        for (group, wanted) in expected_groups {
            let block = &self.document["compare"][group.as_str()];
            assert!(block.is_object(), "golden: the compare map has no group '{group}'");
            Compare::from_json(block).check(&format!("{case} / {group}"), wanted, &actual_groups[group])?;
        }
        Ok(())
    }

    /// Run every `{name, input, output}` case of a section through `port`; panics listing every failing case.
    pub fn run(&self, section: &str, mut port: impl FnMut(&Value) -> Value) {
        let cases = self.cases(section);
        let failures: Vec<String> = cases
            .iter()
            .filter_map(|case| {
                let name = case["name"].as_str().expect("golden: every case has a name");
                let member = |member: &str| {
                    case.get(member)
                        .unwrap_or_else(|| panic!("golden: case '{name}' has no '{member}'"))
                };
                self.check(name, member("output"), &port(member("input"))).err()
            })
            .collect();
        assert!(
            failures.is_empty(),
            "{} of {} golden cases failed in '{section}':\n{}",
            failures.len(),
            cases.len(),
            failures.join("\n")
        );
    }
}

/// An f32 the fixture wrote as a JSON number; panics naming the member when it is not one.
pub fn number(value: &Value, member: &str) -> f32 {
    value[member]
        .as_f64()
        .unwrap_or_else(|| panic!("golden: '{member}' is a number in {value}")) as f32
}
