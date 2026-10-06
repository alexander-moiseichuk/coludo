//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The separation switch answers as plan B's does: src/avionics/golden/separation.json, which
//! golden/build_separation.py builds by running plan B's own run() and _settle() (src/glider/drivers/separation.py)
//! over scripted pin timelines on a virtual clock. Each step is one pin read plan B made -- its time, its level and
//! whether an edge was flagged since the read before -- and the port, fed the same reads, must conclude the same
//! settles on the same reads with the same verdicts, and record the same separation.csv events, exactly.

use drivers::separation::{Debounce, Separation, Settings, Verdict};
use golden::{Compare, Fixture};
use sequencer::Stage;
use serde_json::{Value, json};

/// plan B's separation switch over test_separation.py's timelines, the guards, the debounce corners and seeded ones.
const SEPARATION: &str = include_str!("../../../golden/separation.json");

/// A verdict as the fixture names it.
fn verdict_name(verdict: Verdict) -> &'static str {
    match verdict {
        Verdict::Separated => "separated",
        Verdict::Nested => "nested",
        Verdict::Unsettled => "unsettled",
    }
}

/// Replay one scenario: per step the port's answer, then the final latch and stage.
fn replay(scenario: &Value) -> (Vec<Value>, Value) {
    let init = &scenario["init"];
    let debounce_ms = u32::try_from(init["debounce_ms"].as_u64().unwrap()).unwrap();
    let mut stage = Stage::from_name(init["stage"].as_str().unwrap()).expect("a stage name");
    let mut debounce = Debounce::new(debounce_ms);
    let mut switch = Separation::new(&Settings { debounce_ms }, !init["separated"].as_bool().unwrap());
    let mut answers = Vec::new();
    for step in scenario["steps"].as_array().unwrap() {
        let read = &step["in"];
        let now_ms = u32::try_from(read["now_ms"].as_u64().unwrap()).unwrap();
        let high = read["high"].as_bool().unwrap();
        let edge = read["edge"].as_bool().unwrap();
        let verdict = debounce.step(now_ms, high, edge);
        let event = switch.step(now_ms, high, edge, stage);
        if let Some(event) = event {
            assert!(verdict.is_some(), "an event without a verdict at {now_ms} ms");
            stage = event.stage;
        }
        answers.push(match verdict {
            Some(verdict) => json!({"verdict": verdict_name(verdict), "event": event.map(|event| event.row())}),
            None => Value::Null,
        });
    }
    (answers, json!({"separated": switch.separated(), "stage": stage.name()}))
}

#[test]
fn the_fixture_says_where_it_came_from() {
    let fixture = Fixture::parse(SEPARATION);
    let meta = fixture.get("meta");
    assert_eq!(meta["tag"], golden::PLAN_B_TAG);
    assert_eq!(meta["generator"], "separation");
    assert!(
        meta["source"]
            .as_str()
            .unwrap()
            .contains("src/glider/drivers/separation.py")
    );
    assert_eq!(fixture.compare(), Compare::Exact);
}

#[test]
fn every_read_settles_as_plan_b_s() {
    let fixture = Fixture::parse(SEPARATION);
    let compare = fixture.compare();
    let (mut reads, mut events) = (0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let (answers, last) = replay(scenario);
        for (index, (step, answer)) in scenario["steps"].as_array().unwrap().iter().zip(&answers).enumerate() {
            compare
                .check(&format!("{name} read {index}"), &step["out"], answer)
                .unwrap_or_else(|failure| panic!("{failure}"));
            events += usize::from(!answer["event"].is_null() && !answer.is_null());
        }
        compare
            .check(&format!("{name} final"), &scenario["final"], &last)
            .unwrap_or_else(|failure| panic!("{failure}"));
        reads += answers.len();
    }
    assert!(
        reads > 1000 && events > 30,
        "only {reads} reads and {events} events replayed"
    );
}

#[test]
fn the_test_separation_timelines_end_where_the_test_says() {
    // test_separation.py: the blip and the chatter must not deploy; the held and the bounce-then-hold must
    let fixture = Fixture::parse(SEPARATION);
    let ends: Vec<(String, Value)> = fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .filter(|scenario| scenario["name"].as_str().unwrap().starts_with("test_"))
        .map(|scenario| {
            (
                scenario["name"].as_str().unwrap().to_string(),
                replay(scenario).1["stage"].clone(),
            )
        })
        .collect();
    let owed = [
        ("test_a_7_ms_blip", "boosting"),
        ("test_chatter_that_never_holds_open", "boosting"),
        ("test_a_held_separation", "gliding"),
        ("test_bounce_then_hold", "gliding"),
    ];
    assert_eq!(ends.len(), owed.len());
    for (name, stage) in owed {
        assert!(
            ends.contains(&(name.to_string(), json!(stage))),
            "{name} should end {stage}: {ends:?}"
        );
    }
}

#[test]
fn a_one_read_shift_fails_the_fixture() {
    // NEGATIVE: the comparison bites -- a debounce one millisecond longer concludes a settle on another read
    let fixture = Fixture::parse(SEPARATION);
    let scenario = fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .find(|scenario| scenario["name"] == "irq_a_held_separation")
        .unwrap()
        .clone();
    let mut longer = scenario.clone();
    longer["init"]["debounce_ms"] = json!(26);
    let (answers, _) = replay(&longer);
    let owed: Vec<Value> = scenario["steps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|step| step["out"].clone())
        .collect();
    assert_ne!(answers, owed);
}
