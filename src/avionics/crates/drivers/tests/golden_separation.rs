//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The separation switch against plan B's: src/avionics/golden/separation.json, which golden/build_separation.py
//! builds by running plan B's own run() and _settle() (src/glider/drivers/separation.py) over scripted pin timelines
//! on a virtual clock. Each step is one pin read plan B made -- its time, its level and whether an edge was flagged
//! since the read before -- and the port is fed the same reads as its tick: with the interrupt (`irq` every), a
//! flagged read is an edge recorded at the read's own time (plan B's flag carries no time of its own) and every read
//! is observed; test_separation.py's own timelines (`irq` first: its scripted pin raises no interrupt after the one
//! that starts the settle) are a polled pin, each read polled against the previous. Each read then takes the pair with
//! the scenario's `debounce_ms` as the quiet and concludes an episode by its own level.
//!
//! The port decides by its own model (drivers::debounce): an episode concludes on the first read a quiet after its
//! latest edge, by that read's level, and an episode that ends on the latch's own level is a `blip` row plan B never
//! wrote. Plan B counts three agreeing reads spanning the debounce, restarting where a read sees the level change.
//! Plan B's reads are 5 ms apart while it settles and the edge is stamped at its read, so a quiet of at most one read
//! decides sooner than plan B's third read. So, on plan B's reads:
//!  * in 34 of the 42 scenarios every separation and re-nest the port records is plan B's, on the same read, exactly,
//!    and the latch and the stage end as plan B's;
//!  * the seven scenarios in [`DIFFERING`] are pinned with the port's separations and re-nests. On this fixture's
//!    reads, each difference delays a plan B event by at most [`LARGEST_DELAY_MS`], brings one forward by at most
//!    [`PLAN_B_SAMPLE_MS`] (a quiet of at most one of plan B's reads), or drops a change plan B read through a bounce
//!    together with the change that undid it -- a pair the port records as one blip spanning both. None is a
//!    separation, nor a GLIDING, plan B did not make. (A bound of this fixture, not of the port: drivers::separation
//!    states the port's latency.)
//!  * [`PLAN_B_NEVER_SETTLED`] is plan B's own defect: its 0 ms debounce ran its deadline out on its second read,
//!    before the third agreeing read it needed, so it never separated a held open. The port's quiet of 0 separates it
//!    on the edge's own read: the one separation, and the one GLIDING, the port makes and plan B did not;
//!  * every blip concludes on a read of the latch's level, most of them on a read where plan B concluded a settle on
//!    that same level (its no-event verdict);
//!  * every event's moment and its last edge are reads of the episode, after the previous event's read, and the event
//!    comes on a read a quiet or more after the last edge.

use drivers::debounce::StampPair;
use drivers::separation::{Event, Kind, Separation, Settings};
use golden::{Compare, Fixture};
use sequencer::Stage;
use serde_json::{Value, json};

/// plan B's separation switch over test_separation.py's timelines, the guards, the debounce corners and seeded ones.
const SEPARATION: &str = include_str!("../../../golden/separation.json");

/// Microseconds in a millisecond: the fixture's reads are stamped in ms, the stamps' clock is µs.
const MICROSECONDS_PER_MILLISECOND: u32 = 1000;

/// The most any of the differing scenarios' events is delayed past plan B's, ms: one 10 ms read pair on this
/// fixture. Not a bound of the port, whose latency drivers::separation states (THE LATENCY).
const LARGEST_DELAY_MS: u32 = 10;

/// Plan B's read spacing while it settles, ms (separation.py `_SAMPLE_MS`): the most an event of the port's may come
/// before plan B's, where the quiet is timed from the edge's read and plan B waits for a third read -- for a quiet of
/// at most this, the only one the edge's next read meets.
const PLAN_B_SAMPLE_MS: u32 = 5;

/// One pinned event: the read it came on (its index among the scenario's steps), that read's time (ms), and the
/// separation.csv row.
type Pinned = (usize, u32, [&'static str; 2]);

/// The scenarios whose separations and re-nests differ from plan B's, each with the port's (blips aside).
const DIFFERING: &[(&str, &[Pinned])] = &[
    // plan B's 1 ms debounce separated on its third read, at 15 ms; the port's quiet ends on the read after the edge:
    // one plan B read earlier
    ("debounce_1", &[(1, 10, ["separated", "gliding"])]),
    // plan B's re-nest at 319 ms was read through the bounce at 315-317 ms; its separation at 339 ms undid it
    ("seeded_01", &[(9, 224, ["separated", "setting"])]),
    // plan B separated at 40 ms through the bounce at 26-29 ms: one read pair later
    (
        "seeded_05",
        &[(10, 50, ["separated", "gliding"]), (17, 155, ["nested", "gliding"])],
    ),
    // plan B separated at 48 ms through the bounce at 25-27 ms: one read pair later
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
    // plan B re-nested at 266 ms through the bounce at 253-255 ms: one read pair later
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

/// Plan B's defect, with the port's event: its 0 ms debounce never settled the open held from 5 ms (the scenario's
/// name), which the port's quiet of 0 separates on that edge's own read.
const PLAN_B_NEVER_SETTLED: (&str, &[Pinned]) = ("debounce_0_never_settles", &[(0, 5, ["separated", "gliding"])]);

/// The port's answer to one read: the event it concluded there, None for none.
type Answer = Option<Event>;

/// A read's time, µs.
fn read_us(step: &Value) -> u32 {
    u32::try_from(step["in"]["now_ms"].as_u64().unwrap()).unwrap() * MICROSECONDS_PER_MILLISECOND
}

/// A read's time, ms.
fn read_ms(step: &Value) -> u32 {
    u32::try_from(step["in"]["now_ms"].as_u64().unwrap()).unwrap()
}

/// The scenario's quiet, µs.
fn quiet_us(scenario: &Value) -> u32 {
    u32::try_from(scenario["init"]["debounce_ms"].as_u64().unwrap()).unwrap() * MICROSECONDS_PER_MILLISECOND
}

/// Replay one scenario as the firmware's tick: per read the event it concluded, then the final latch and stage.
fn replay(scenario: &Value) -> (Vec<Answer>, Value) {
    let init = &scenario["init"];
    let debounce_ms = u32::try_from(init["debounce_ms"].as_u64().unwrap()).unwrap();
    let mut stage = Stage::from_name(init["stage"].as_str().unwrap()).expect("a stage name");
    let mut switch = Separation::new(&Settings { debounce_ms }, !init["separated"].as_bool().unwrap());
    let polled = init["irq"] == "first";
    let pair = StampPair::new();
    let mut previous = switch.latched();
    let mut answers = Vec::new();
    for step in scenario["steps"].as_array().unwrap() {
        let (now_us, high) = (read_us(step), step["in"]["high"].as_bool().unwrap());
        if polled {
            pair.poll(now_us, high, previous, switch.latched());
        } else {
            if step["in"]["edge"].as_bool().unwrap() {
                pair.record(now_us);
            }
            pair.observe(now_us, high, switch.latched());
        }
        previous = high;
        let event = pair
            .take(now_us, switch.quiet_us())
            .map(|episode| switch.conclude(episode, high, stage));
        if let Some(event) = event {
            stage = event.stage;
        }
        answers.push(event);
    }
    (answers, json!({"separated": switch.separated(), "stage": stage.name()}))
}

/// A latch change as the fixture records it: its separation.csv row, or null for none and for a blip.
fn change_value(answer: Answer) -> Value {
    answer
        .filter(|event| event.kind != Kind::Blip)
        .map_or(Value::Null, |event| json!(event.row()))
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
            now_ms: read_ms(step),
            row: row.clone(),
            advanced: stage == "boosting" && row[1] == "gliding",
        });
        stage = row[1].clone();
    }
    events
}

/// The port's separations and re-nests in one scenario: (read, its time in ms, the row, whether it advanced).
fn port_changes(scenario: &Value, answers: &[Answer]) -> Vec<(usize, u32, Value, bool)> {
    answers
        .iter()
        .enumerate()
        .filter_map(|(read, answer)| {
            answer
                .filter(|event| event.kind != Kind::Blip)
                .map(|event| (read, event))
        })
        .map(|(read, event)| {
            (
                read,
                read_ms(&scenario["steps"][read]),
                json!(event.row()),
                event.advanced,
            )
        })
        .collect()
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
fn every_latch_change_lands_on_plan_b_s_read() {
    let fixture = Fixture::parse(SEPARATION);
    let compare = fixture.compare();
    let (mut scenarios, mut reads, mut changes) = (0, 0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        if name == PLAN_B_NEVER_SETTLED.0 || DIFFERING.iter().any(|(differing, _)| *differing == name) {
            continue;
        }
        let (answers, last) = replay(scenario);
        for (index, (step, answer)) in scenario["steps"].as_array().unwrap().iter().zip(&answers).enumerate() {
            compare
                .check(
                    &format!("{name} read {index}"),
                    &step["out"]["event"],
                    &change_value(*answer),
                )
                .unwrap_or_else(|failure| panic!("{failure}"));
            changes += usize::from(!change_value(*answer).is_null());
        }
        compare
            .check(&format!("{name} final"), &scenario["final"], &last)
            .unwrap_or_else(|failure| panic!("{failure}"));
        scenarios += 1;
        reads += answers.len();
    }
    assert_eq!(
        scenarios, 34,
        "34 of the 42 scenarios record plan B's latch changes exactly"
    );
    assert!(
        reads > 1000 && changes > 30,
        "only {reads} reads and {changes} changes replayed"
    );
}

#[test]
fn never_a_gliding_plan_b_did_not_make() {
    // the hard invariant, over every scenario but plan B's own defect: each GLIDING the port makes is one of plan B's,
    // at most a plan B read earlier or a read pair later, and the port makes as many
    let fixture = Fixture::parse(SEPARATION);
    let mut advances = 0;
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        if name == PLAN_B_NEVER_SETTLED.0 {
            continue;
        }
        let (answers, _) = replay(scenario);
        let port: Vec<u32> = port_changes(scenario, &answers)
            .into_iter()
            .filter(|(_, _, _, advanced)| *advanced)
            .map(|(_, now_ms, _, _)| now_ms)
            .collect();
        let plan_b: Vec<u32> = plan_b_events(scenario)
            .into_iter()
            .filter(|event| event.advanced)
            .map(|event| event.now_ms)
            .collect();
        assert_eq!(
            port.len(),
            plan_b.len(),
            "{name}: GLIDING at {port:?}, plan B at {plan_b:?}"
        );
        for (port_ms, plan_b_ms) in port.iter().zip(&plan_b) {
            assert!(
                port_ms + PLAN_B_SAMPLE_MS >= *plan_b_ms && *port_ms <= plan_b_ms + LARGEST_DELAY_MS,
                "{name}: GLIDING at {port_ms} ms, plan B's at {plan_b_ms} ms"
            );
        }
        advances += port.len();
    }
    assert!(advances > 10, "only {advances} GLIDINGs replayed");
}

#[test]
fn the_differing_ones_are_pinned_never_a_separation_plan_b_did_not_make() {
    let fixture = Fixture::parse(SEPARATION);
    let (mut earlier, mut delayed, mut dropped) = (0, 0, 0);
    for (name, pinned) in DIFFERING {
        let scenario = scenario(&fixture, name);
        let (answers, last) = replay(&scenario);
        let port = port_changes(&scenario, &answers);
        let owed: Vec<(usize, u32, Value)> = pinned
            .iter()
            .map(|(read, now_ms, row)| (*read, *now_ms, json!(row)))
            .collect();
        let found: Vec<(usize, u32, Value)> = port
            .iter()
            .map(|(read, now_ms, row, _)| (*read, *now_ms, row.clone()))
            .collect();
        assert_eq!(found, owed, "{name}: the port's separations and re-nests");
        assert_eq!(last, scenario["final"], "{name}: the latch and stage end as plan B's");
        let plan_b = plan_b_events(&scenario);
        let plan_b_rows: Vec<(usize, u32, Value)> = plan_b
            .iter()
            .map(|event| (event.read, event.now_ms, event.row.clone()))
            .collect();
        assert_ne!(found, plan_b_rows, "{name} does differ from plan B");

        // walk plan B's events: each is the port's next one, at most a plan B read earlier or LARGEST_DELAY_MS later,
        // or dropped
        let mut next = port.iter().peekable();
        let mut unmatched = Vec::new();
        for event in &plan_b {
            match next.peek() {
                Some((_, now_ms, row, advanced))
                    if *row == event.row
                        && *now_ms + PLAN_B_SAMPLE_MS >= event.now_ms
                        && *now_ms <= event.now_ms + LARGEST_DELAY_MS =>
                {
                    assert_eq!(*advanced, event.advanced, "{name}: a GLIDING at {now_ms} ms");
                    earlier += usize::from(*now_ms < event.now_ms);
                    delayed += usize::from(*now_ms > event.now_ms);
                    next.next();
                }
                _ => unmatched.push(event),
            }
        }
        assert!(
            next.next().is_none(),
            "{name}: an event plan B never made, or one more than a read earlier"
        );
        // a drop is a pair: a change plan B read through a bounce, then the change that undid it; never a deploy. The
        // port records the pair as one blip, from before the first change to the read of the second or later
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
            let spanning = answers.iter().enumerate().any(|(read, answer)| {
                answer.is_some_and(|event| {
                    event.kind == Kind::Blip
                        && event.moment_us <= pair[0].now_ms * MICROSECONDS_PER_MILLISECOND
                        && read >= pair[1].read
                })
            });
            assert!(spanning, "{name}: no blip spans plan B's pair at {} ms", pair[0].now_ms);
            dropped += 1;
        }
    }
    assert_eq!(
        (earlier, delayed, dropped),
        (1, 3, 3),
        "one event a plan B read earlier, three a read pair later, three pairs dropped"
    );
}

#[test]
fn plan_b_s_0_ms_debounce_never_settled_and_the_port_s_does() {
    // the one separation plan B did not make: a clean open held from 5 ms (the edge's read) to the end, which plan B's
    // arithmetic gave up on and the port's quiet of 0 separates on that read -- dated by the edge, no duration
    let fixture = Fixture::parse(SEPARATION);
    let (name, pinned) = PLAN_B_NEVER_SETTLED;
    let scenario = scenario(&fixture, name);
    assert_eq!(scenario["init"]["debounce_ms"], 0);
    assert_eq!(scenario["init"]["timeline"], json!([[0, 1], [5, 0]]), "a held open");
    assert!(plan_b_events(&scenario).is_empty(), "plan B never separated it");
    assert_eq!(scenario["final"], json!({"separated": false, "stage": "boosting"}));
    let (answers, last) = replay(&scenario);
    let found: Vec<(usize, u32, Value, (u32, u32))> = answers
        .iter()
        .enumerate()
        .filter_map(|(read, answer)| answer.map(|event| (read, event)))
        .map(|(read, event)| {
            (
                read,
                read_ms(&scenario["steps"][read]),
                json!(event.row()),
                (event.moment_us, event.duration_us),
            )
        })
        .collect();
    let owed: Vec<(usize, u32, Value, (u32, u32))> = pinned
        .iter()
        .map(|(read, now_ms, row)| (*read, *now_ms, json!(row), (5000, 0)))
        .collect();
    assert_eq!(found, owed);
    assert_eq!(last, json!({"separated": true, "stage": "gliding"}));
}

#[test]
fn every_blip_is_the_latch_s_own_level_mostly_plan_b_s_no_event_verdict() {
    // a blip concludes on a read of the level the latch already holds, and leaves the latch and the stage. Pinned: 47
    // land on a read where plan B concluded a settle on that same level and wrote nothing. Of the six elsewhere, four
    // are an episode the port ran on through a bounce and ended a quiet after its last edge, between two of plan B's
    // verdicts (irq_chatter_that_never_holds_open, seeded_00, seeded_06, seeded_12), and two are pairs plan B read
    // through a bounce (seeded_01, seeded_09; seeded_19's lands on a no-event verdict of plan B's)
    let fixture = Fixture::parse(SEPARATION);
    let (mut on_plan_b_s, mut elsewhere) = (0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let (answers, _) = replay(scenario);
        let mut stage = Stage::from_name(scenario["init"]["stage"].as_str().unwrap()).unwrap();
        let mut separated = scenario["init"]["separated"].as_bool().unwrap();
        for (index, (step, answer)) in scenario["steps"].as_array().unwrap().iter().zip(&answers).enumerate() {
            let Some(event) = answer else {
                continue;
            };
            if event.kind == Kind::Blip {
                assert_eq!(step["in"]["high"], !separated, "{name} read {index}: the latch's level");
                assert!(!event.advanced && event.stage == stage, "{name} read {index}");
                let held = if separated { "separated" } else { "nested" };
                if step["out"]["verdict"] == held && step["out"]["event"].is_null() {
                    on_plan_b_s += 1;
                } else {
                    elsewhere += 1;
                }
            }
            separated = event.kind == Kind::Separated || (separated && event.kind == Kind::Blip);
            stage = event.stage;
        }
    }
    assert_eq!(
        (on_plan_b_s, elsewhere),
        (47, 6),
        "blips on plan B's no-event verdict, and elsewhere"
    );
}

#[test]
fn every_moment_and_last_edge_are_reads_of_their_episode() {
    // the moment is the episode's first edge and the duration runs to its last: both are reads that recorded an edge
    // (a flagged read, or one the tick's read or the poll found off the level before), after the previous event's
    // read, and the event comes on a read at least the quiet after the last edge
    let fixture = Fixture::parse(SEPARATION);
    let (mut events, mut wobbled) = (0, 0);
    for scenario in fixture.get("scenarios").as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let quiet_us = quiet_us(scenario);
        let steps = scenario["steps"].as_array().unwrap();
        let reads: Vec<u32> = steps.iter().map(read_us).collect();
        let (answers, _) = replay(scenario);
        let mut previous_us = None;
        for (index, answer) in answers.iter().enumerate() {
            let Some(event) = answer else {
                continue;
            };
            let last_us = event.moment_us + event.duration_us;
            assert!(
                reads.contains(&event.moment_us) && reads.contains(&last_us),
                "{name} read {index}: {event:?} is not dated by reads"
            );
            assert!(
                previous_us.is_none_or(|previous_us| event.moment_us > previous_us),
                "{name} read {index}: a moment before the previous event"
            );
            assert!(
                reads[index] - last_us >= quiet_us,
                "{name} read {index}: concluded before the quiet"
            );
            previous_us = Some(reads[index]);
            events += 1;
            wobbled += usize::from(event.duration_us > 0);
        }
    }
    assert!(
        events > 100 && wobbled > 30,
        "only {events} events, {wobbled} with a wobble"
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
    let found: Vec<Value> = answers.iter().map(|answer| change_value(*answer)).collect();
    let owed: Vec<Value> = scenario["steps"]
        .as_array()
        .unwrap()
        .iter()
        .map(|step| step["out"]["event"].clone())
        .collect();
    assert_ne!(found, owed);
}
