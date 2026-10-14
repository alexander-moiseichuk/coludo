//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The databoard answers as plan B's does: src/avionics/golden/databoard.json, which golden/build_databoard.py
//! builds from src/glider/databoard.py on CPython. Each scenario is one parameter; its steps run in order at the
//! uptime each one names (`now_us`), and every answer is compared with plan B's under the scenario's own `compare`:
//! EXACTLY where the parameter never reconciles (a value passes through untouched, so every float must come back bit
//! for bit), within abs 1e-3 (altitude) or 1e-2 (pressure) where it does (the EMA runs in f32 here, in double in
//! plan B); every integer is exact. The builder leaves out what the port changes on purpose (extrapolation, the
//! backdated birth, read-side learning at every read) and says how; the unit tests pin those.

use databoard::{Channel, Parameter, Sample};
use golden::{Compare, Fixture};
use serde_json::{Value, json};

/// plan B's Parameter over test_databoard.py's cases, the plan's named rules and seeded sweeps.
const DATABOARD: &str = include_str!("../../../golden/databoard.json");

/// A value the fixture writes: read from and written to JSON as plan B's value of that kind.
trait Json: Sample {
    /// The value a step's `value` holds.
    fn from_json(value: &Value) -> Self;
    /// The value as plan B wrote it.
    fn to_json(self) -> Value;
}

/// A float as f32 (the port's unit); panics naming it when it is not a number.
fn float(value: &Value) -> f32 {
    value.as_f64().unwrap_or_else(|| panic!("not a number: {value}")) as f32
}

impl Json for f32 {
    fn from_json(value: &Value) -> Self {
        float(value)
    }

    fn to_json(self) -> Value {
        json!(f64::from(self))
    }
}

impl Json for [f32; 3] {
    fn from_json(value: &Value) -> Self {
        let items = value.as_array().expect("a vector is a list");
        core::array::from_fn(|index| float(&items[index]))
    }

    fn to_json(self) -> Value {
        json!(self.map(f64::from))
    }
}

impl Json for [i32; 2] {
    fn from_json(value: &Value) -> Self {
        let items = value.as_array().expect("a position is a list");
        core::array::from_fn(|index| {
            items[index]
                .as_i64()
                .and_then(|unit| i32::try_from(unit).ok())
                .expect("a position unit is an i32")
        })
    }

    fn to_json(self) -> Value {
        json!(self)
    }
}

/// A text the parameter keeps for good (a source or parameter name): the fixture outlives every scenario, so the
/// leak is bounded by the fixture.
fn name(value: &Value) -> &'static str {
    Box::leak(value.as_str().expect("a name").to_string().into_boxed_str())
}

/// An integer of a step, panicking naming it when it is missing or out of range.
fn integer<Integer: TryFrom<i64>>(step: &Value, member: &str) -> Integer {
    step[member]
        .as_i64()
        .and_then(|value| Integer::try_from(value).ok())
        .unwrap_or_else(|| panic!("{member} is not an integer in range: {step}"))
}

/// A registered source's channel, panicking naming it when the source never registered.
fn channel<'parameter, T: Json>(parameter: &'parameter Parameter<T>, source: &str) -> &'parameter Channel<T> {
    parameter
        .channel(source)
        .unwrap_or_else(|| panic!("{source} never registered"))
}

/// Run one step on the parameter and return the port's answer as plan B's fixture writes it.
fn apply<T: Json>(parameter: &mut Parameter<T>, step: &Value) -> Value {
    let now_us: u64 = integer(step, "now_us");
    let source = || step["source"].as_str().expect("a source");
    match step["action"].as_str() {
        Some("provide") => {
            parameter
                .add_source(
                    name(&step["source"]),
                    integer(step, "priority"),
                    integer(step, "timeout_ms"),
                    step["reconcile"] == true,
                )
                .expect("room for the source");
            json!({"window_us": parameter.window_us(), "sources": parameter.sources().collect::<Vec<_>>()})
        }
        Some("push") => {
            let mut writer = parameter.writer(source()).expect("a free writer");
            writer.push(T::from_json(&step["value"]), now_us);
            Value::Null
        }
        Some("read") => parameter.read(now_us).map_or(
            Value::Null,
            |reading| json!({"value": reading.value.to_json(), "source": reading.source, "age_us": reading.age_us}),
        ),
        Some("fresh_below") => json!(parameter.fresh_below(integer(step, "rank"), now_us)),
        Some("rank") => json!(parameter.rank(source())),
        Some("stamp") => json!(parameter.stamp()),
        Some("recent") => channel(parameter, source())
            .recent(integer(step, "window_us"), now_us)
            .map_or(Value::Null, T::to_json),
        Some("latest") => channel(parameter, source()).latest().map_or(Value::Null, T::to_json),
        Some("offsets") => Value::Object(
            parameter
                .offsets()
                .map(|(source, offset)| (source.to_string(), json!(f64::from(offset))))
                .collect(),
        ),
        other => panic!("unknown action {other:?}"),
    }
}

/// One step's comparison: its label, plan B's answer, the port's, and how they compare.
type Step = (String, Value, Value, Compare);

/// Run a scenario of one value kind: per step, its label, plan B's answer, the port's, and the scenario's compare.
fn scenario<T: Json>(scenario: &Value) -> Vec<Step> {
    let label = scenario["name"].as_str().expect("a scenario has a name");
    let compare = Compare::from_json(&scenario["compare"]);
    let mut parameter: Parameter<T> = Parameter::new(name(&scenario["name"]));
    scenario["steps"]
        .as_array()
        .expect("steps is a list")
        .iter()
        .enumerate()
        .map(|(index, step)| {
            let answer = apply(&mut parameter, &step["in"]);
            (
                format!("{label} step {index} ({})", step["in"]["action"]),
                step["out"].clone(),
                answer,
                compare,
            )
        })
        .collect()
}

/// Every scenario's steps, run by kind.
fn answers(text: &str) -> Vec<Step> {
    let fixture = Fixture::parse(text);
    let scenarios = fixture.get("scenarios").as_array().expect("scenarios is a list");
    scenarios
        .iter()
        .flat_map(|run| match run["kind"].as_str() {
            Some("scalar") => scenario::<f32>(run),
            Some("vector") => scenario::<[f32; 3]>(run),
            Some("position") => scenario::<[i32; 2]>(run),
            other => panic!("unknown kind {other:?}"),
        })
        .collect()
}

/// The failures among the port's answers, each under its scenario's comparison.
fn failures(answers: &[Step]) -> Vec<String> {
    answers
        .iter()
        .filter_map(|(label, owed, answer, compare)| compare.check(label, owed, answer).err())
        .collect()
}

/// Whether a scenario turns reconciliation on: a provide step carries `reconcile: true`.
fn reconciles(scenario: &Value) -> bool {
    scenario["steps"]
        .as_array()
        .expect("steps is a list")
        .iter()
        .any(|step| step["in"]["action"] == "provide" && step["in"]["reconcile"] == true)
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(DATABOARD);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "databoard");
    assert_eq!(meta["source"], "src/glider/databoard.py");
    assert_eq!(fixture.compare(), Compare::Abs(1e-2), "the loosest a scenario uses");
    // a scenario that never reconciles passes its values through untouched, so it must compare exactly
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let compare = Compare::from_json(&scenario["compare"]);
        let name = &scenario["name"];
        if reconciles(scenario) {
            assert!(
                matches!(compare, Compare::Abs(tolerance) if tolerance <= 1e-2),
                "{name}: {compare:?}"
            );
        } else {
            assert_eq!(compare, Compare::Exact, "{name}");
        }
    }
}

#[test]
fn every_step_matches_plan_b() {
    let answers = answers(DATABOARD);
    assert!(answers.len() > 1500, "only {} steps compared", answers.len());
    let failures = failures(&answers);
    assert!(
        failures.is_empty(),
        "{} of {} databoard steps differ:\n{}",
        failures.len(),
        answers.len(),
        failures.join("\n")
    );
}

#[test]
fn the_fixture_exercises_every_rule() {
    // a fixture that drifted into compiling nothing interesting must fail loudly, not pass
    let answers = answers(DATABOARD);
    let count = |predicate: &dyn Fn(&str, &Value) -> bool| {
        answers
            .iter()
            .filter(|(label, owed, _, _)| predicate(label, owed))
            .count()
    };
    let read = |label: &str| label.ends_with("(\"read\")");
    let reads = count(&|label, _| read(label));
    let none_fresh = count(&|label, owed| read(label) && owed.is_null());
    let aged = count(&|label, owed| read(label) && owed["age_us"].as_u64().is_some_and(|age| age > 0));
    // a window past one second, read past one second: plan B's born-stale birth covered only up to 1 s
    let long = count(&|label, owed| read(label) && owed["age_us"].as_u64().is_some_and(|age| age > 1_000_000));
    let learned = count(&|label, owed| {
        label.ends_with("(\"offsets\")") && owed.as_object().is_some_and(|offsets| !offsets.is_empty())
    });
    assert!(
        reads > 300 && none_fresh > 40 && aged > 80 && long >= 4 && learned > 10,
        "{reads} {none_fresh} {aged} {long} {learned}"
    );
}

#[test]
fn the_tolerance_sees_an_eighth_of_a_metre_of_backup_reading() {
    // NEGATIVE: one backup reading 0.125 m off moves the learned bias by 1.25 cm (alpha 0.1), ten times the
    // altitude tolerance: the comparison must notice, or the reconcile cases would prove nothing
    let text = DATABOARD.replace("\"value\": 102.25", "\"value\": 102.375");
    assert_ne!(text, DATABOARD, "the fixture has the 102.25 backup to perturb");
    assert!(!failures(&answers(&text)).is_empty());
}

#[test]
fn one_ulp_off_a_value_passed_through_is_caught() {
    // NEGATIVE: the exact comparison sees the lowest mantissa bit of a full-mantissa pressure; a channel that
    // dropped low bits (or compared within a tolerance) would let this through
    let mut document: Value = serde_json::from_str(DATABOARD).unwrap();
    let scenario = document["scenarios"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|scenario| scenario["name"] == "values_pass_through_bit_for_bit")
        .expect("the full-mantissa scenario");
    let step = scenario["steps"]
        .as_array_mut()
        .unwrap()
        .iter_mut()
        .find(|step| step["in"]["action"] == "read")
        .expect("a read");
    let owed = step["out"]["value"].as_f64().unwrap() as f32;
    assert_ne!(
        owed.to_bits() & 0xFFF,
        0,
        "a full mantissa: the low bits are not all zero"
    );
    step["out"]["value"] = json!(f64::from(f32::from_bits(owed.to_bits() + 1)));
    let text = serde_json::to_string(&document).unwrap();
    assert_eq!(failures(&answers(&text)).len(), 1);
}

#[test]
fn a_wrong_window_rule_is_caught() {
    // NEGATIVE: claim a 51 ms window where plan B's rule gives 50 -- the comparison must notice
    let text = DATABOARD.replacen("\"window_us\": 50000", "\"window_us\": 51000", 1);
    assert_ne!(text, DATABOARD);
    assert!(!failures(&answers(&text)).is_empty());
}
