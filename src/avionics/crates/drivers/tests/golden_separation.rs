//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The separation switch against plan B's: src/avionics/golden/separation.json, which golden/build_separation.py
//! builds by running plan B's own run() and _settle() (src/glider/drivers/separation.py) over scripted pin timelines
//! on a virtual clock. Each step is one pin read plan B made -- its time, its level and whether an edge was flagged
//! since the read before -- and the port is fed the same reads.
//!
//! The port settles by two rules of its own (D15, see drivers::separation): a level that disagrees with the latch
//! starts a settle with no edge, and an edge flagged between two reads restarts the run under way. So, on plan B's
//! reads:
//!  * in 36 of the 42 scenarios the port records plan B's separation.csv events on the same reads, exactly, and ends
//!    on plan B's latch and stage;
//!  * it concludes fewer settles: plan B's edge, carried over a settle, started another that ended on the latch's
//!    own level and recorded nothing. Where both conclude a settle on the same read, the verdict is the same;
//!  * the six scenarios in [`DIFFERING`] are pinned with the port's events. On this fixture's reads, each difference
//!    delays a plan B event by at most [`LARGEST_DELAY_MS`], or drops a change plan B read through a bounce together
//!    with the change that undid it. None is earlier than plan B's, and none advances to GLIDING where plan B did
//!    not. (A bound of this fixture, not of the port: drivers::separation states the port's latency.)

use drivers::separation::{Debounce, Event, Separation, Settings, Verdict};
use golden::{Compare, Fixture};
use sequencer::Stage;
use serde_json::{Value, json};

/// plan B's separation switch over test_separation.py's timelines, the guards, the debounce corners and seeded ones.
const SEPARATION: &str = include_str!("../../../golden/separation.json");

/// The most any of the six differing scenarios' events is delayed past plan B's, ms: one 10 ms control tick on this
/// fixture. Not a bound of the port: a settle run out to its deadline delays a deploy further (see
/// drivers::separation, THE LATENCY).
const LARGEST_DELAY_MS: u32 = 10;

/// One pinned event: the read it came on (its index among the scenario's steps), that read's time, and the
/// separation.csv row.
type Pinned = (usize, u32, [&'static str; 2]);

/// The scenarios whose events differ from plan B's, each with the port's events.
const DIFFERING: &[(&str, &[Pinned])] = &[
    // plan B's re-nest at 319 ms was read through the bounce at 315-317 ms; its separation at 339 ms undid it
    ("seeded_01", &[(9, 224, ["separated", "setting"])]),
    // plan B separated at 40 ms through the bounce at 26-29 ms: one tick later
    (
        "seeded_05",
        &[(10, 50, ["separated", "gliding"]), (17, 155, ["nested", "gliding"])],
    ),
    // plan B separated at 48 ms through the bounce at 25-27 ms: one tick later
    (
        "seeded_07",
        &[
            (12, 58, ["separated", "setting"]),
            (40, 201, ["nested", "setting"]),
            (54, 491, ["separated", "setting"]),
            (61, 638, ["nested", "setting"]),
        ],
    ),
    // plan B's separation at 27 ms was read through the bounces at 15-17 and 25-27 ms; its re-nest at 77 ms undid it
    (
        "seeded_09",
        &[(40, 221, ["separated", "setting"]), (45, 296, ["nested", "setting"])],
    ),
    // plan B re-nested at 266 ms through the bounce at 253-255 ms: one tick later
    (
        "seeded_10",
        &[
            (4, 78, ["separated", "gliding"]),
            (31, 276, ["nested", "gliding"]),
            (38, 460, ["separated", "gliding"]),
            (43, 576, ["nested", "gliding"]),
            (59, 859, ["separated", "gliding"]),
        ],
    ),
    // plan B's separation at 89 ms was read through the bounce at 80-84 ms; its re-nest at 144 ms undid it
    (
        "seeded_19",
        &[(40, 330, ["separated", "setting"]), (43, 392, ["nested", "setting"])],
    ),
];

/// A verdict as the fixture names it.
fn verdict_name(verdict: Verdict) -> &'static str {
    match verdict {
        Verdict::Separated => "separated",
        Verdict::Nested => "nested",
        Verdict::Unsettled => "unsettled",
    }
}

/// The port's answer to one read: the settle it concluded, and the event the switch recorded.
struct Answer {
    /// The verdict of the settle this read concluded, None while none did.
    verdict: Option<Verdict>,
    /// The separation.csv event, None for none.
    event: Option<Event>,
}

/// Replay one scenario: per read the port's answer, then the final latch and stage.
///
/// The switch records the events; a twin debounce fed the same reads reports the verdicts, given the level rule as
/// [`Separation::step`] applies it (a settle starts on a level that disagrees with the latch).
fn replay(scenario: &Value) -> (Vec<Answer>, Value) {
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
        let verdict = debounce.step(now_ms, high, edge, high == switch.separated());
        let event = switch.step(now_ms, high, edge, stage);
        if let Some(event) = event {
            let held = if event.separated {
                Verdict::Separated
            } else {
                Verdict::Nested
            };
            assert_eq!(verdict, Some(held), "an event without its verdict at {now_ms} ms");
            stage = event.stage;
        }
        answers.push(Answer { verdict, event });
    }
    (answers, json!({"separated": switch.separated(), "stage": stage.name()}))
}

/// An event as the fixture records it: its separation.csv row, or null.
fn event_value(event: Option<Event>) -> Value {
    event.map_or(Value::Null, |event| json!(event.row()))
}

/// The scenarios, by name.
fn scenario(fixture: &Fixture, name: &str) -> Value {
    fixture
        .get("scenarios")
        .as_array()
        .unwrap()
        .iter()
        .find(|scenario| scenario["name"] == name)
        .unwrap_or_else(|| panic!("no scenario {name}"))
        .clone()
}

/// One of plan B's events: the read it came on, its time, its row, and whether it advanced BOOSTING -> GLIDING.
struct Recorded {
    /// The read's index among the scenario's steps.
    read: usize,
    /// The read's time, ms.
    now_ms: u32,
    /// The separation.csv row.
    row: Value,
    /// Whether the stage was BOOSTING before it and GLIDING after.
    advanced: bool,
}

/// plan B's events in one scenario, in order.
fn plan_b_events(scenario: &Value) -> Vec<Recorded> {
    let mut stage = scenario["init"]["stage"].clone();
    let mut events = Vec::new();
    for (read, step) in scenario["steps"].as_array().unwrap().iter().enumerate() {
        let row = &step["out"]["event"];
        if row.is_null() {
            continue;
        }
        events.push(Recorded {
            read,
            now_ms: u32::try_from(step["in"]["now_ms"].as_u64().unwrap()).unwrap(),
            row: row.clone(),
            advanced: stage == "boosting" && row[1] == "gliding",
        });
        stage = row[1].clone();
    }
    events
}

/// Whether plan B concluded the settle on `read` through a bounce: an edge flagged on a read of the run of agreeing
/// reads it concluded on, after the run's first -- the level did not hold between two of the reads it counted.
fn read_through_a_bounce(scenario: &Value, read: usize) -> bool {
    let steps = scenario["steps"].as_array().unwrap();
    let level = &steps[read]["in"]["high"];
    let mut first = read;
    while first > 0 && steps[first - 1]["in"]["high"] == *level && steps[first - 1]["out"].is_null() {
        first -= 1;
    }
    steps[first + 1..=read].iter().any(|step| step["in"]["edge"] == true)
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
fn every_event_lands_on_plan_b_s_read() {
    let fixture = Fixture::parse(SEPARATION);
    let compare = fixture.compare();
    let (mut scenarios, mut reads, mut events) = (0, 0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        if DIFFERING.iter().any(|(differing, _)| *differing == name) {
            continue;
        }
        let (answers, last) = replay(scenario);
        for (index, (step, answer)) in scenario["steps"].as_array().unwrap().iter().zip(&answers).enumerate() {
            compare
                .check(
                    &format!("{name} read {index}"),
                    &step["out"]["event"],
                    &event_value(answer.event),
                )
                .unwrap_or_else(|failure| panic!("{failure}"));
            events += usize::from(answer.event.is_some());
        }
        compare
            .check(&format!("{name} final"), &scenario["final"], &last)
            .unwrap_or_else(|failure| panic!("{failure}"));
        scenarios += 1;
        reads += answers.len();
    }
    assert_eq!(scenarios, 36, "36 of the 42 scenarios record plan B's events exactly");
    assert!(
        reads > 1000 && events > 30,
        "only {reads} reads and {events} events replayed"
    );
}

#[test]
fn where_both_conclude_a_settle_the_verdict_is_the_same() {
    let fixture = Fixture::parse(SEPARATION);
    let (mut both, mut plan_b_only, mut port_only) = (0, 0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let (answers, _) = replay(scenario);
        for (index, (step, answer)) in scenario["steps"].as_array().unwrap().iter().zip(&answers).enumerate() {
            match (step["out"].is_null(), answer.verdict) {
                (false, Some(verdict)) => {
                    assert_eq!(step["out"]["verdict"], verdict_name(verdict), "{name} read {index}");
                    both += 1;
                }
                (false, None) => plan_b_only += 1,
                (true, Some(_)) => port_only += 1,
                (true, None) => {}
            }
        }
    }
    // pinned: plan B's follow-up settles are most of the 74 the port never concludes (a carried-over edge would bring
    // them back); of the port's own 7, six are plan B's verdict one or two reads (5-10 ms) later, the restart rule
    // having waited out a bounce, and one is seeded_19's chatter running a settle out
    assert_eq!(
        (both, plan_b_only, port_only),
        (120, 74, 7),
        "shared, plan B-only and port-only verdicts"
    );
}

#[test]
fn the_six_that_differ_are_pinned_never_earlier_and_never_a_glide_plan_b_did_not_make() {
    let fixture = Fixture::parse(SEPARATION);
    let (mut delayed, mut dropped) = (0, 0);
    for (name, pinned) in DIFFERING {
        let scenario = scenario(&fixture, name);
        let (answers, last) = replay(&scenario);
        let port: Vec<(usize, u32, Value, bool)> = answers
            .iter()
            .enumerate()
            .filter_map(|(read, answer)| answer.event.map(|event| (read, event)))
            .map(|(read, event)| {
                let now_ms = u32::try_from(scenario["steps"][read]["in"]["now_ms"].as_u64().unwrap()).unwrap();
                (read, now_ms, json!(event.row()), event.advanced)
            })
            .collect();
        let owed: Vec<(usize, u32, Value)> = pinned
            .iter()
            .map(|(read, now_ms, row)| (*read, *now_ms, json!(row)))
            .collect();
        let found: Vec<(usize, u32, Value)> = port
            .iter()
            .map(|(read, now_ms, row, _)| (*read, *now_ms, row.clone()))
            .collect();
        assert_eq!(found, owed, "{name}: the port's events");
        assert_eq!(last, scenario["final"], "{name}: the latch and stage end as plan B's");
        let plan_b = plan_b_events(&scenario);
        let plan_b_rows: Vec<(usize, u32, Value)> = plan_b
            .iter()
            .map(|event| (event.read, event.now_ms, event.row.clone()))
            .collect();
        assert_ne!(found, plan_b_rows, "{name} does differ from plan B");

        // walk plan B's events: each is the port's next one at most LARGEST_DELAY_MS later, or dropped
        let mut next = port.iter().peekable();
        let mut unmatched = Vec::new();
        for event in &plan_b {
            match next.peek() {
                Some((_, now_ms, row, advanced))
                    if *row == event.row && *now_ms >= event.now_ms && *now_ms - event.now_ms <= LARGEST_DELAY_MS =>
                {
                    assert_eq!(*advanced, event.advanced, "{name}: a GLIDING at {now_ms} ms");
                    delayed += usize::from(*now_ms > event.now_ms);
                    next.next();
                }
                _ => unmatched.push(event),
            }
        }
        assert!(
            next.next().is_none(),
            "{name}: an event plan B never made, or an earlier one"
        );
        // a drop is a pair: a change plan B read through a bounce, then the change that undid it; never a deploy
        assert_eq!(unmatched.len() % 2, 0, "{name}: an unpaired drop");
        for pair in unmatched.chunks(2) {
            assert!(
                read_through_a_bounce(&scenario, pair[0].read),
                "{name}: plan B's {} at {} ms held",
                pair[0].row,
                pair[0].now_ms
            );
            assert_ne!(
                pair[0].row[0], pair[1].row[0],
                "{name}: the second change undoes the first"
            );
            assert!(!pair[0].advanced && !pair[1].advanced, "{name}: a dropped deploy");
            dropped += 1;
        }
    }
    assert_eq!(
        (delayed, dropped),
        (3, 3),
        "three events a tick later, three pairs dropped"
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
    // NEGATIVE: the comparison bites -- a debounce 6 ms longer needs one read more than plan B made
    let fixture = Fixture::parse(SEPARATION);
    let scenario = scenario(&fixture, "irq_a_held_separation");
    let mut longer = scenario.clone();
    longer["init"]["debounce_ms"] = json!(26);
    let (answers, _) = replay(&longer);
    let found: Vec<Value> = answers.iter().map(|answer| event_value(answer.event)).collect();
    let owed: Vec<Value> = scenario["steps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|step| step["out"]["event"].clone())
        .collect();
    assert_ne!(found, owed);
}
