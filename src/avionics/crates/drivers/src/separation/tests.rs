//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the separation switch on the Rust cadence (one read per 10 ms control tick, the edges stamped in µs as
//! the interrupt stamps them), beyond its golden fixture (tests/golden_separation.rs), which replays plan B's 5 ms
//! reads: the operator's physics (a wobble of at most 7 ms as the 2 cm pads part at 3 m/s, dated by its first edge and
//! measured to its last), boost blips that must never deploy, the hold exact from the edge stamps, a held return that
//! ends a blip's episode, chatter and its deadlines, the edge the level read saw first, the clock's wrap, the polled
//! pin, the 10 ms default and its floor, and the latch's own rules. The generic debounce's rules are tested in
//! debounce/tests.rs.

use super::{DEFAULT_DEBOUNCE_MS, Event, Separation, Settings};
use crate::debounce::Edges;
use sequencer::Stage;
use serde_json::json;

/// The control tick, µs: the default 100 Hz schedule.
const TICK_US: u32 = 10_000;

/// The fastest tick a config may ask for, µs: schedule_hz 1000.
const FAST_TICK_US: u32 = 1000;

/// The pads' travel as the operator measures it, µs: 2 cm at 3 m/s is about 7 ms, and any loss and restoring of
/// contact while they part falls inside it.
const WOBBLE_US: u32 = 7000;

/// How the switch learns that the line moved between two reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pin {
    /// The both-edge interrupt stamps every change, however brief: the first and the last since the previous read.
    Interrupt,
    /// No interrupt (an arm that failed): the reads alone (the edge contract).
    Polled,
}

/// What a switch did over a timeline: its events, each with its read's time (µs), the stage it ended in, and the
/// switch itself.
struct Flight {
    /// The events, each with the time of the read that recorded it.
    events: Vec<(u32, Event)>,
    /// The stage after the last event.
    stage: Stage,
    /// The switch after the last read.
    switch: Separation,
}

/// The reads of a `tick_us` tick from `phase_us` to `end_us`, µs.
fn ticks(tick_us: u32, phase_us: u32, end_us: u32) -> impl Iterator<Item = u32> {
    (phase_us..=end_us).step_by(tick_us as usize)
}

/// Read a pin in `stage` with a `debounce_ms` debounce, its latch set up from the timeline's first level: `timeline` is
/// the level (true = HIGH = nested) from each time on, µs, ascending from 0, and `reads` the read times, µs, ascending.
/// On a [`Pin::Interrupt`] each read passes the first and the last change since the previous read, however brief.
fn fly(debounce_ms: u32, stage: Stage, timeline: &[(u32, bool)], pin: Pin, reads: impl Iterator<Item = u32>) -> Flight {
    let mut flight = Flight {
        events: Vec::new(),
        stage,
        switch: Separation::new(&Settings { debounce_ms }, timeline[0].1),
    };
    let mut level = timeline[0].1;
    let mut next = 1;
    for read_us in reads {
        let mut seen: Option<(u32, u32)> = None;
        while next < timeline.len() && timeline[next].0 <= read_us {
            if timeline[next].1 != level {
                let at_us = timeline[next].0;
                seen = Some((seen.map_or(at_us, |(first_us, _)| first_us), at_us));
            }
            level = timeline[next].1;
            next += 1;
        }
        let edges = match (pin, seen) {
            (Pin::Polled, _) => Edges::Polled,
            (Pin::Interrupt, None) => Edges::Quiet,
            (Pin::Interrupt, Some((first_us, last_us))) => Edges::Seen { first_us, last_us },
        };
        if let Some(event) = flight.switch.step(read_us, level, edges, flight.stage) {
            flight.stage = event.stage;
            flight.events.push((read_us, event));
        }
    }
    flight
}

/// [`fly`] in BOOSTING at the default debounce, on the 10 ms tick from `phase_us` to `end_us`.
fn boost(timeline: &[(u32, bool)], pin: Pin, phase_us: u32, end_us: u32) -> Flight {
    fly(
        DEFAULT_DEBOUNCE_MS,
        Stage::Boosting,
        timeline,
        pin,
        ticks(TICK_US, phase_us, end_us),
    )
}

/// The level of `timeline` at `at_us`.
fn level_at(timeline: &[(u32, bool)], at_us: u32) -> bool {
    timeline
        .iter()
        .take_while(|(time_us, _)| *time_us <= at_us)
        .last()
        .expect("the timeline starts at 0")
        .1
}

/// The time of the first tick from `phase_us` that reads `level` on `timeline`, µs.
fn first_read_of(timeline: &[(u32, bool)], level: bool, phase_us: u32) -> u32 {
    (phase_us..)
        .step_by(TICK_US as usize)
        .find(|read_us| level_at(timeline, *read_us) == level)
        .expect("the timeline reaches the level")
}

/// A timeline in ms, as test_separation.py writes it, in µs.
fn in_microseconds(timeline_ms: &[(u32, bool)]) -> Vec<(u32, bool)> {
    timeline_ms.iter().map(|(ms, high)| (ms * 1000, *high)).collect()
}

/// Every tick phase, in 0.5 ms steps: where the 10 ms reads fall against a timeline.
fn phases() -> impl Iterator<Item = u32> {
    (0..TICK_US).step_by(500)
}

/// A step of xorshift32: the seeded timelines, a function of the code alone.
fn next(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

/// A number in `low..=high` from `state`.
fn within(state: &mut u32, low: u32, high: u32) -> u32 {
    low + next(state) % (high - low + 1)
}

/// Append to a nested `timeline` a departure at `departure_us` and a seeded wobble confined to the [`WOBBLE_US`]
/// after it: contact lost and restored 0-4 times, then open for good when `ends_open`, else one restoring more (a
/// blip that returns to nested). Returns the time of the last change.
fn wobble(state: &mut u32, timeline: &mut Vec<(u32, bool)>, departure_us: u32, ends_open: bool) -> u32 {
    let toggles = 2 * within(state, 0, 4) + u32::from(!ends_open);
    timeline.push((departure_us, false));
    let (mut at, mut level) = (departure_us, false);
    for _ in 0..toggles {
        at += within(state, 1, WOBBLE_US / (toggles + 1));
        level = !level;
        timeline.push((at, level));
    }
    assert!(at < departure_us + WOBBLE_US && level != ends_open);
    at
}

/// A seeded bounce on a nested line: it starts within 50 ms, toggles every 0.5-2 ms for 50-200 ms, then the line
/// holds `then_high`. Returns the timeline (µs) and the time of its last change.
fn bounce_then(state: &mut u32, then_high: bool) -> (Vec<(u32, bool)>, u32) {
    let start = within(state, 0, 50_000);
    let end = start + within(state, 50_000, 200_000);
    let mut timeline = vec![(0, true)];
    let (mut at, mut level) = (start, true);
    loop {
        at += within(state, 500, 2000);
        if at >= end {
            break;
        }
        level = !level;
        timeline.push((at, level));
    }
    if level != then_high {
        timeline.push((end, then_high));
    }
    let last = timeline.last().unwrap().0;
    (timeline, last)
}

/// An event's time fields: (moment, duration), µs.
fn dated(event: &Event) -> (u32, u32) {
    (event.moment_us, event.duration_us)
}

#[test]
fn the_operator_s_wobble_separates_once_dated_by_its_first_edge() {
    // the pads part, lose and regain contact any number of times within 7 ms, then stay open: at every phase of the
    // tick, exactly one separation, GLIDING, its moment the first edge and its duration the wobble's span to the last
    // edge, both exact, decided 10-20 ms after the last edge (the first quiet read 10 ms after it)
    let mut state = 20261007;
    for _ in 0..300 {
        let mut timeline = vec![(0, true)];
        let departure_us = within(&mut state, 50_000, 60_000);
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase in phases() {
            let flight = boost(&timeline, Pin::Interrupt, phase, 300_000);
            assert_eq!(
                flight.events.len(),
                1,
                "phase {phase} µs: {:?} on {timeline:?}",
                flight.events
            );
            let (read_us, event) = flight.events[0];
            assert_eq!((event.row(), event.advanced), (["separated", "gliding"], true));
            assert_eq!(
                dated(&event),
                (departure_us, last_us - departure_us),
                "the first edge and the span, on {timeline:?}"
            );
            assert!(
                (10_000..20_000).contains(&(read_us - last_us)),
                "phase {phase} µs: {} µs after the last edge",
                read_us - last_us
            );
            assert_eq!((flight.stage, flight.switch.separated()), (Stage::Gliding, true));
        }
    }
}

#[test]
fn a_polled_wobble_is_dated_by_its_first_open_read() {
    // no interrupt: the level rule settles the wobble on its third open read, and the moment is the first read that
    // saw the pads open; the 7 ms wobble falls between two reads at most once, so it reads as no wobble at all
    let mut state = 7;
    for _ in 0..100 {
        let mut timeline = vec![(0, true)];
        let departure_us = within(&mut state, 50_000, 60_000);
        wobble(&mut state, &mut timeline, departure_us, true);
        for phase in phases() {
            let flight = boost(&timeline, Pin::Polled, phase, 300_000);
            assert_eq!(flight.events.len(), 1, "phase {phase} µs on {timeline:?}");
            let first_open_us = first_read_of(&timeline, false, phase);
            let (read_us, event) = flight.events[0];
            assert_eq!(dated(&event), (first_open_us, 0), "phase {phase} µs on {timeline:?}");
            assert_eq!(read_us, first_open_us + 2 * TICK_US);
            assert_eq!(flight.stage, Stage::Gliding);
        }
    }
}

#[test]
fn a_polled_duration_is_counted_in_whole_ticks() {
    // open at 3 ms, contact again from 12 ms (9 ms, short of the debounce), open for good from 21 ms, read on the tick
    // from 0: the interrupt measures the wobble exactly, 18 ms from the first edge; the reads alone see open, nested,
    // open from 10, 20 and 30 ms, and count 20 ms from the first open read to the first of the final run
    let timeline = in_microseconds(&[(0, true), (3, false), (12, true), (21, false)]);
    let armed = boost(&timeline, Pin::Interrupt, 0, 200_000);
    let found: Vec<(u32, (u32, u32))> = armed
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(40_000, (3000, 18_000))]);
    let polled = boost(&timeline, Pin::Polled, 0, 200_000);
    let found: Vec<(u32, (u32, u32))> = polled
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(50_000, (10_000, 20_000))]);
    assert_eq!(found[0].1.1 % TICK_US, 0, "a polled duration is whole ticks");
}

#[test]
fn a_held_separation_in_boosting_glides() {
    // test_separation.py's timeline: the pads open at 5 ms and stay open, a clean single edge: no duration. With the
    // interrupt, decided 10-20 ms after the edge; polled, two ticks after the first open read, 20-30 ms after it opened
    let timeline = in_microseconds(&[(0, true), (5, false)]);
    for pin in [Pin::Interrupt, Pin::Polled] {
        for phase in phases() {
            let flight = boost(&timeline, pin, phase, 500_000);
            assert_eq!(flight.events.len(), 1, "{pin:?}, phase {phase} µs");
            let (read_us, event) = flight.events[0];
            assert_eq!(event.row(), ["separated", "gliding"]);
            assert!(event.advanced && flight.switch.separated());
            assert_eq!(flight.stage, Stage::Gliding);
            let (moment_us, latency_us) = match pin {
                Pin::Interrupt => (5000, 10_000..20_000),
                Pin::Polled => (first_read_of(&timeline, false, phase), 20_000..30_000),
            };
            assert_eq!(dated(&event), (moment_us, 0), "{pin:?}, phase {phase} µs");
            assert!(
                latency_us.contains(&(read_us - 5000)),
                "{pin:?}, phase {phase} µs: at {read_us} µs"
            );
        }
    }
}

#[test]
fn a_bounce_then_hold_glides_dated_by_its_first_bounce() {
    // test_separation.py's timeline: three bounces, open for good from 15 ms; every read until then sees an edge, so
    // the hold waits out the bounces: the moment is the first at 3 ms, the duration 12 ms to the last
    let timeline = in_microseconds(&[(0, true), (3, false), (6, true), (9, false), (12, true), (15, false)]);
    for phase in phases() {
        let flight = boost(&timeline, Pin::Interrupt, phase, 500_000);
        assert_eq!(flight.events.len(), 1, "phase {phase} µs");
        let (read_us, event) = flight.events[0];
        assert!((25_000..35_000).contains(&read_us), "phase {phase} µs: at {read_us} µs");
        assert_eq!(
            (event.row(), flight.stage, dated(&event)),
            (["separated", "gliding"], Stage::Gliding, (3000, 12_000))
        );
    }
}

#[test]
fn a_boost_blip_up_to_7_ms_never_deploys() {
    // NEGATIVE: vibration opens the pads for up to 7 ms, with any wobble, and they close again for good -- with the
    // interrupt or polled, at every phase: no event, BOOSTING, nested, and the blip's settle concluded nested
    let mut state = 2026;
    for _ in 0..300 {
        let mut timeline = vec![(0, true)];
        let blip_us = within(&mut state, 50_000, 60_000);
        wobble(&mut state, &mut timeline, blip_us, false);
        for pin in [Pin::Interrupt, Pin::Polled] {
            for phase in phases() {
                let flight = boost(&timeline, pin, phase, 300_000);
                assert!(
                    flight.events.is_empty(),
                    "{pin:?}, phase {phase} µs: {:?} on {timeline:?}",
                    flight.events
                );
                assert_eq!((flight.stage, flight.switch.separated()), (Stage::Boosting, false));
                assert!(!flight.switch.settling(), "the blip's settle concluded nested");
            }
        }
    }
}

#[test]
fn the_7_ms_blip_and_the_8_ms_chatter_never_deploy() {
    // NEGATIVE: test_separation.py's two timelines that must not deploy, at every phase of the tick. Polled too: the
    // blip is open for one read at most, and no three reads in a row see the chatter's 8 ms open halves
    let blip = in_microseconds(&[(0, true), (19, false), (26, true)]);
    let mut chatter: Vec<(u32, bool)> = vec![(0, true)];
    chatter.extend((8..200).step_by(8).map(|start| (start * 1000, start / 8 % 2 == 1)));
    chatter.push((200_000, true));
    for pin in [Pin::Interrupt, Pin::Polled] {
        for timeline in [&blip, &chatter] {
            for phase in phases() {
                let flight = boost(timeline, pin, phase, 600_000);
                assert!(
                    flight.events.is_empty(),
                    "{pin:?}, phase {phase} µs: {:?}",
                    flight.events
                );
                assert_eq!((flight.stage, flight.switch.separated()), (Stage::Boosting, false));
            }
        }
    }
}

#[test]
fn the_hold_is_exact_to_the_microsecond_from_the_edge_stamps() {
    // an open of 9 or 10 ms then a re-nest never deploys at a 10 ms debounce; an 11 ms one deploys on any read in its
    // last ms, the only reads both quiet and 10 ms past its edge -- every phase of a 1 kHz tick, one phase in ten of
    // the 10 ms tick (the reads in 0.1 ms phases). The deploy is that read, at least 10 ms after the edge
    let opened_us = 50_000;
    for (tick_us, deploying) in [(FAST_TICK_US, 10), (TICK_US, 10)] {
        let mut deployed = 0;
        for phase in (0..tick_us).step_by(100) {
            for open_us in [9000, 10_000, 11_000] {
                let timeline = [(0, true), (opened_us, false), (opened_us + open_us, true)];
                let flight = fly(
                    DEFAULT_DEBOUNCE_MS,
                    Stage::Boosting,
                    &timeline,
                    Pin::Interrupt,
                    ticks(tick_us, phase, 300_000),
                );
                let deciding = ticks(tick_us, phase, 300_000)
                    .find(|read_us| (opened_us + 10_000..opened_us + open_us).contains(read_us));
                let first = flight
                    .events
                    .first()
                    .map(|(read_us, event)| (*read_us, event.name(), dated(event)));
                assert_eq!(
                    first,
                    deciding.map(|read_us| (read_us, "separated", (opened_us, 0))),
                    "a {open_us} µs open, tick {tick_us} µs, phase {phase} µs"
                );
                deployed += usize::from(first.is_some());
            }
        }
        assert_eq!(deployed, deploying, "tick {tick_us} µs");
    }
}

#[test]
fn a_held_nest_after_a_blip_ends_its_episode() {
    // a boost blip, the pads nested again for the debounce or more, then the real separation: one event, dated and
    // measured by the separation's own wobble, at every phase. A nest of 10-19 ms often has no quiet read 10 ms after
    // its last edge before the separation's first; the read of that edge concludes it (the return), so where the reads
    // fall no longer decides which departure dates the separation
    let mut state = 99;
    for _ in 0..300 {
        let mut timeline = vec![(0, true)];
        let blip_us = within(&mut state, 50_000, 60_000);
        let returned_us = wobble(&mut state, &mut timeline, blip_us, false);
        let departure_us = returned_us + within(&mut state, 10_000, 30_000);
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase in phases() {
            let flight = boost(&timeline, Pin::Interrupt, phase, 400_000);
            assert_eq!(
                flight.events.len(),
                1,
                "phase {phase} µs: {:?} on {timeline:?}",
                flight.events
            );
            assert_eq!(
                dated(&flight.events[0].1),
                (departure_us, last_us - departure_us),
                "phase {phase} µs on {timeline:?}"
            );
            assert_eq!(flight.stage, Stage::Gliding);
        }
    }
}

#[test]
fn a_return_shorter_than_the_debounce_is_the_same_episode() {
    // NEGATIVE, the documented consequence: the pads closed again for under 10 ms before they parted for good, so the
    // nest never held the debounce and the settle never concluded nested -- the separation is dated by the blip's
    // first edge, and its duration runs from there, at any phase
    let mut state = 1234;
    for _ in 0..200 {
        let mut timeline = vec![(0, true)];
        let blip_us = within(&mut state, 50_000, 60_000);
        let returned_us = wobble(&mut state, &mut timeline, blip_us, false);
        let departure_us = returned_us + within(&mut state, 1000, 9999);
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase in phases() {
            let flight = boost(&timeline, Pin::Interrupt, phase, 400_000);
            assert_eq!(
                flight.events.len(),
                1,
                "phase {phase} µs: {:?} on {timeline:?}",
                flight.events
            );
            assert_eq!(
                dated(&flight.events[0].1),
                (blip_us, last_us - blip_us),
                "phase {phase} µs on {timeline:?}"
            );
        }
    }
}

#[test]
fn a_debounce_up_to_the_tick_decides_on_the_same_read() {
    // on the 10 ms tick a quiet read comes a whole tick after the last edge, so every debounce up to 10 ms decides the
    // operator's wobble on the same read, settled at its last edge; polled, the third read decides them all alike.
    // What a debounce no longer than a contact gap inside the wobble changes is the moment: where a read saw the pads
    // closed in that gap, the next edge's read concludes the return (it held the debounce), and the separation is
    // dated by a later departure -- the floor of the 10 ms choice, seen on the default tick
    let mut state = 31;
    let mut moved = 0;
    for _ in 0..100 {
        let mut timeline = vec![(0, true)];
        let departure_us = within(&mut state, 50_000, 60_000);
        wobble(&mut state, &mut timeline, departure_us, true);
        let longest_gap_us = timeline
            .windows(2)
            .skip(1)
            .filter(|pair| pair[0].1)
            .map(|pair| pair[1].0 - pair[0].0)
            .max()
            .unwrap_or(0);
        for pin in [Pin::Interrupt, Pin::Polled] {
            for phase in phases() {
                let decided = |debounce_ms: u32| {
                    fly(
                        debounce_ms,
                        Stage::Boosting,
                        &timeline,
                        pin,
                        ticks(TICK_US, phase, 300_000),
                    )
                    .events
                };
                let default = decided(DEFAULT_DEBOUNCE_MS);
                assert_eq!(default.len(), 1);
                let (read_us, event) = default[0];
                let first_us = match pin {
                    Pin::Interrupt => departure_us,
                    Pin::Polled => first_read_of(&timeline, false, phase),
                };
                assert_eq!(
                    event.moment_us, first_us,
                    "the default dates it by its first edge or open read"
                );
                for debounce_ms in [0, 1, 2, 5, 9] {
                    let shorter = decided(debounce_ms);
                    let context = format!("{pin:?}, debounce {debounce_ms} ms, phase {phase} µs on {timeline:?}");
                    assert_eq!(shorter.len(), 1, "{context}");
                    let (shorter_read_us, shorter) = shorter[0];
                    assert_eq!(
                        (shorter_read_us, shorter.row(), shorter.moment_us + shorter.duration_us),
                        (read_us, event.row(), event.moment_us + event.duration_us),
                        "{context}"
                    );
                    if shorter.moment_us != event.moment_us {
                        assert!(
                            pin == Pin::Interrupt
                                && shorter.moment_us > event.moment_us
                                && debounce_ms * 1000 <= longest_gap_us,
                            "{context}"
                        );
                        moved += 1;
                    }
                }
            }
        }
    }
    assert!(moved > 0, "no shorter debounce moved a moment");
}

#[test]
fn below_the_wobble_s_gap_a_faster_tick_splits_one_separation() {
    // NEGATIVE, the floor of the 10 ms choice: the debounce must outlast the wobble's longest contact gap. Pads open 3
    // ms, in contact 3 ms, then open for good: on a 1 kHz tick a 2 ms debounce records separated, nested, separated
    // (GLIDING stays); open 1 ms, in contact 5 ms, then open: a 4 ms debounce concludes the contact nested, which
    // records nothing but ends the episode, so the moment moves to the later departure. On the 10 ms tick no quiet read
    // falls inside either, so each is one row, but the same debounces move its moment wherever a read saw the contact
    // (the return). 10 ms keeps each one event dated by the first edge, on either tick
    let opened_us = 50_000;
    let three_and_three = [
        (0, true),
        (opened_us, false),
        (opened_us + 3000, true),
        (opened_us + 6000, false),
    ];
    let one_and_five = [
        (0, true),
        (opened_us, false),
        (opened_us + 1000, true),
        (opened_us + 6000, false),
    ];
    for phase in (0..FAST_TICK_US).step_by(100) {
        let rows = |debounce_ms: u32, timeline: &[(u32, bool)]| {
            let flight = fly(
                debounce_ms,
                Stage::Boosting,
                timeline,
                Pin::Interrupt,
                ticks(FAST_TICK_US, phase, 300_000),
            );
            assert_eq!(flight.stage, Stage::Gliding, "the stage never reverses");
            flight
                .events
                .iter()
                .map(|(_, event)| (event.row(), dated(event)))
                .collect::<Vec<_>>()
        };
        assert_eq!(
            rows(2, &three_and_three),
            [
                (["separated", "gliding"], (opened_us, 0)),
                (["nested", "gliding"], (opened_us + 3000, 0)),
                (["separated", "gliding"], (opened_us + 6000, 0)),
            ],
            "phase {phase} µs"
        );
        assert_eq!(
            rows(4, &one_and_five),
            [(["separated", "gliding"], (opened_us + 6000, 0))],
            "phase {phase} µs"
        );
        for timeline in [&three_and_three[..], &one_and_five[..]] {
            assert_eq!(
                rows(DEFAULT_DEBOUNCE_MS, timeline),
                [(["separated", "gliding"], (opened_us, 6000))],
                "phase {phase} µs"
            );
        }
    }
    for phase in phases() {
        // the one read in the wobble's tick, and the contact each timeline holds
        let read_us = opened_us + phase;
        for (debounce_ms, timeline, contact_us) in [
            (2, &three_and_three, opened_us + 3000..opened_us + 6000),
            (4, &one_and_five, opened_us + 1000..opened_us + 6000),
            (DEFAULT_DEBOUNCE_MS, &three_and_three, 0..0),
            (DEFAULT_DEBOUNCE_MS, &one_and_five, 0..0),
        ] {
            let flight = fly(
                debounce_ms,
                Stage::Boosting,
                timeline,
                Pin::Interrupt,
                ticks(TICK_US, phase, 300_000),
            );
            let found: Vec<(u32, u32)> = flight.events.iter().map(|(_, event)| dated(event)).collect();
            let owed = if contact_us.contains(&read_us) {
                (opened_us + 6000, 0)
            } else {
                (opened_us, 6000)
            };
            assert_eq!(found, [owed], "debounce {debounce_ms} ms, phase {phase} µs");
        }
    }
}

#[test]
fn seeded_chatter_never_deploys_and_a_held_open_after_it_does() {
    // toggles of 0.5-2 ms for 50-200 ms. Ending nested, no debounce deploys. Ending open, the switch deploys 10-20 ms
    // after the last bounce, a deadline or not (a deadline carries the run under way into the next settle) -- and never
    // during the bounce. It is settled at the last bounce. Its moment is the first bounce whenever the chatter lasts
    // 90 ms or less (the settle it started concludes before its deadline); after a deadline, the start of the run the
    // last deadline carried over
    let mut state = 20261007;
    let (mut first_bounce, mut runs) = (0, 0);
    for _ in 0..1000 {
        let phase = within(&mut state, 0, TICK_US - 1);
        let (nested, last) = bounce_then(&mut state, true);
        for debounce_ms in [0, 2, 10, 20, 30] {
            let flight = fly(
                debounce_ms,
                Stage::Boosting,
                &nested,
                Pin::Interrupt,
                ticks(TICK_US, phase, last + 500_000),
            );
            assert!(
                flight.events.is_empty(),
                "debounce {debounce_ms} ms, phase {phase} µs: {:?} on {nested:?}",
                flight.events
            );
            assert_eq!(flight.stage, Stage::Boosting);
        }
        let (open, last) = bounce_then(&mut state, false);
        let flight = boost(&open, Pin::Interrupt, phase, last + 500_000);
        assert_eq!(
            flight.events.len(),
            1,
            "phase {phase} µs: {:?} on {open:?}",
            flight.events
        );
        let (read_us, event) = flight.events[0];
        assert_eq!((event.row(), flight.stage), (["separated", "gliding"], Stage::Gliding));
        let after_us = read_us - last;
        assert!(
            (10_000..20_000).contains(&after_us),
            "{after_us} µs after the last bounce, phase {phase} µs"
        );
        let departure_us = open[1].0;
        let (moment_us, duration_us) = dated(&event);
        assert!(
            (departure_us..=read_us).contains(&moment_us),
            "a moment {moment_us} µs outside the episode"
        );
        assert_eq!(moment_us + duration_us, last, "phase {phase} µs on {open:?}");
        if last - departure_us <= 90_000 {
            assert_eq!(moment_us, departure_us, "phase {phase} µs on {open:?}");
        }
        first_bounce += usize::from(moment_us == departure_us);
        runs += 1;
    }
    // a fair share of the chatters, the shorter ones, are dated by their first bounce, and the longer ones by the run
    // their last deadline carried over
    assert!(
        first_bounce > runs / 5 && first_bounce < runs,
        "{first_bounce} of {runs}"
    );
}

#[test]
fn a_bounce_that_runs_out_its_settles_deploys_on_time() {
    // toggles every 5 ms, open for good from 215 ms, read at 6, 16, ... ms: every read sees the pads open and bouncing.
    // The read at 6 ms starts a settle that runs out at 106; the run it was on (open since the bounce at 105 ms) carries
    // into the next settle, departed there, which runs out at 206 and carries on from 205; 216 sees the last bounces
    // (210 and 215 ms) and restarts the run at 215; 226 is quiet and 11 ms past it: decided there, dated 205 ms, with
    // the 10 ms to the sustained open as its duration -- the deadlines delayed nothing
    let mut timeline: Vec<(u32, bool)> = vec![(0, true)];
    timeline.extend((1..=43).map(|bounce| (bounce * 5000, bounce % 2 == 0)));
    assert_eq!(timeline.last(), Some(&(215_000, false)), "open for good from 215 ms");
    let flight = boost(&timeline, Pin::Interrupt, 6000, 500_000);
    let found: Vec<(u32, (u32, u32))> = flight
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(226_000, (205_000, 10_000))], "11 ms after the last bounce");
    assert_eq!(flight.stage, Stage::Gliding);
}

#[test]
fn a_re_nest_on_the_tick_never_reverses_the_stage() {
    // NEGATIVE: open for good from 5 ms, nested again from 300 ms -- recorded, dated when the pads closed, and GLIDING
    // stays
    let timeline = in_microseconds(&[(0, true), (5, false), (300, true)]);
    let flight = boost(&timeline, Pin::Interrupt, 0, 600_000);
    let rows: Vec<([&str; 2], (u32, u32))> = flight
        .events
        .iter()
        .map(|(_, event)| (event.row(), dated(event)))
        .collect();
    assert_eq!(
        rows,
        [
            (["separated", "gliding"], (5000, 0)),
            (["nested", "gliding"], (300_000, 0))
        ]
    );
    assert!(!flight.events[1].1.advanced);
    assert_eq!((flight.stage, flight.switch.separated()), (Stage::Gliding, false));
}

#[test]
fn outside_boosting_the_latch_moves_but_the_stage_does_not() {
    for stage in [Stage::Setting, Stage::Gliding, Stage::Landing, Stage::Done] {
        let mut switch = Separation::new(&Settings { debounce_ms: 10 }, true);
        let event = switch.apply(true, stage, 7000, 300).expect("a change of the latch");
        assert_eq!(
            (event.stage, event.advanced, dated(&event)),
            (stage, false, (7000, 300)),
            "{stage:?}"
        );
    }
}

#[test]
fn a_re_nest_is_recorded_and_never_reverses() {
    let mut switch = Separation::new(&Settings { debounce_ms: 10 }, true);
    assert_eq!(
        switch.apply(true, Stage::Boosting, 1000, 0).unwrap().stage,
        Stage::Gliding
    );
    assert_eq!(
        switch.apply(true, Stage::Gliding, 2000, 0),
        None,
        "the same level records nothing"
    );
    let nested = switch.apply(false, Stage::Gliding, 3000, 1500).unwrap();
    assert_eq!(
        (nested.name(), nested.stage, nested.advanced, dated(&nested)),
        ("nested", Stage::Gliding, false, (3000, 1500))
    );
}

#[test]
fn a_disagreeing_level_with_no_edge_settles() {
    // the blind window: the pads opened between setup and arming the interrupt, or the arm failed -- no edge ever.
    // Armed, a quiet read 10 ms after the first open one decides; polled, the third. Dated by that first read
    for (edges, verdict_us) in [(Edges::Quiet, 10_000), (Edges::Polled, 20_000)] {
        let mut nested = Separation::new(&Settings { debounce_ms: 10 }, true);
        assert_eq!(nested.step(0, false, edges, Stage::Boosting), None);
        assert!(nested.settling(), "the disagreeing level starts a settle");
        for now_us in (TICK_US..verdict_us).step_by(TICK_US as usize) {
            assert_eq!(nested.step(now_us, false, edges, Stage::Boosting), None, "{edges:?}");
        }
        let event = nested
            .step(verdict_us, false, edges, Stage::Boosting)
            .expect("a held separation");
        assert_eq!(
            (event.row(), event.advanced, dated(&event)),
            (["separated", "gliding"], true, (0, 0)),
            "{edges:?}"
        );
        // and the other way: a latch set up separated, nested before arming, dated by its first nested read
        let mut open = Separation::new(&Settings { debounce_ms: 10 }, false);
        let events: Vec<Option<Event>> = (1..=3)
            .map(|tick| open.step(tick * TICK_US, true, edges, Stage::Setting))
            .collect();
        let nested_at = (verdict_us / TICK_US) as usize;
        assert_eq!(
            events[nested_at].map(|event| (event.row(), dated(&event))),
            Some((["nested", "setting"], (TICK_US, 0))),
            "{edges:?}"
        );
        assert!(events[..nested_at].iter().all(Option::is_none), "{edges:?}");
    }
}

#[test]
fn a_level_equal_to_the_latch_starts_nothing() {
    // NEGATIVE: with no edge, a read of the latch's own level is no settle, whichever the latch and the input
    for edges in [Edges::Quiet, Edges::Polled] {
        for high in [true, false] {
            let mut switch = Separation::new(&Settings { debounce_ms: 10 }, high);
            for tick in 0..30 {
                assert_eq!(switch.step(tick * TICK_US, high, edges, Stage::Boosting), None);
                assert!(!switch.settling(), "{edges:?}, latch high={high}, tick {tick}");
            }
        }
    }
}

#[test]
fn an_edge_forced_on_every_read_never_separates() {
    // NEGATIVE, the edge contract pinned at the switch: a caller that passed an edge on every read would restart
    // every run, so the pads held open from the first read never separate; Polled is how a pin without an interrupt
    // reads
    let mut forced = Separation::new(&Settings { debounce_ms: 10 }, true);
    for tick in 0..100 {
        let now_us = tick * TICK_US;
        let edges = Edges::Seen {
            first_us: now_us,
            last_us: now_us,
        };
        assert_eq!(forced.step(now_us, false, edges, Stage::Boosting), None, "tick {tick}");
    }
    assert!(!forced.separated());
}

#[test]
fn an_edge_the_level_read_saw_before_its_stamp_dates_the_separation() {
    // the firmware's tick takes the edges, then reads the level, then its clock: an edge between the first two is seen
    // as a level with no edge, dated at that read, and its stamp, a µs earlier, reaches the next read. The moment is
    // the stamp and the duration 0, never a wrapped 4 294 967 295 µs
    let mut switch = Separation::new(&Settings { debounce_ms: 10 }, true);
    assert_eq!(switch.step(10_000, false, Edges::Quiet, Stage::Boosting), None);
    let edge = Edges::Seen {
        first_us: 9999,
        last_us: 9999,
    };
    assert_eq!(switch.step(20_000, false, edge, Stage::Boosting), None);
    let event = switch
        .step(30_000, false, Edges::Quiet, Stage::Boosting)
        .expect("a held separation");
    assert_eq!((event.row(), dated(&event)), (["separated", "gliding"], (9999, 0)));
}

#[test]
fn a_wobble_across_the_wrap_keeps_its_moment_and_duration() {
    // the pads part 2 ms before the u32 µs clock wraps and wobble 5 ms across it: the moment is the first edge, before
    // the wrap, the duration the 5 ms to the last, after it, and the hold 10 ms from that last edge
    let first_us = u32::MAX - 1999;
    let last_us = first_us.wrapping_add(5000);
    let mut switch = Separation::new(&Settings { debounce_ms: 10 }, true);
    let wobbled = Edges::Seen { first_us, last_us };
    assert_eq!(
        switch.step(first_us.wrapping_add(6000), false, wobbled, Stage::Boosting),
        None
    );
    let quiet_us = first_us.wrapping_add(14_999);
    assert_eq!(switch.step(quiet_us, false, Edges::Quiet, Stage::Boosting), None);
    let event = switch
        .step(first_us.wrapping_add(15_000), false, Edges::Quiet, Stage::Boosting)
        .expect("a held separation");
    assert_eq!(
        (event.row(), dated(&event)),
        (["separated", "gliding"], (first_us, 5000))
    );
}

#[test]
fn a_blip_just_before_the_clock_wraps_never_deploys() {
    // NEGATIVE: a 2 ms open ending 6 ms before the u32 µs clock wraps, read at 1 kHz on through the wrap. The hold's end
    // (the edge + 10 ms) has wrapped to a small number, and a quiet read before the wrap is not past it: the hold is an
    // elapsed time
    let opened_us = u32::MAX - 7999;
    let closed_us = opened_us + 2000;
    let mut switch = Separation::new(&Settings { debounce_ms: 10 }, true);
    for read in 0..40 {
        let now_us = opened_us.wrapping_add(500 + read * FAST_TICK_US);
        let (high, edges) = match read {
            0 => (
                false,
                Edges::Seen {
                    first_us: opened_us,
                    last_us: opened_us,
                },
            ),
            1 => (false, Edges::Quiet),
            2 => (
                true,
                Edges::Seen {
                    first_us: closed_us,
                    last_us: closed_us,
                },
            ),
            _ => (true, Edges::Quiet),
        };
        assert_eq!(switch.step(now_us, high, edges, Stage::Boosting), None, "read {read}");
    }
    assert!(
        !switch.separated() && !switch.settling(),
        "the blip's settle concluded nested"
    );
}

#[test]
fn the_debounce_ms_becomes_microseconds_once() {
    // 25 ms: an edge at 5 ms on a 1 kHz tick is decided on the read at 30 ms exactly, not one before
    let timeline = in_microseconds(&[(0, true), (5, false)]);
    let flight = fly(
        25,
        Stage::Boosting,
        &timeline,
        Pin::Interrupt,
        ticks(FAST_TICK_US, 0, 100_000),
    );
    let found: Vec<(u32, (u32, u32))> = flight
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(30_000, (5000, 0))]);
    // NEGATIVE: a debounce past the µs clock's range saturates and never concludes (the validator keeps it out)
    for debounce_ms in [4_294_968, u32::MAX] {
        let flight = fly(
            debounce_ms,
            Stage::Boosting,
            &timeline,
            Pin::Interrupt,
            ticks(TICK_US, 0, 20_000_000),
        );
        assert!(flight.events.is_empty(), "debounce {debounce_ms} ms");
        assert!(flight.switch.settling());
    }
}

#[test]
fn settings_and_the_setup_read() {
    assert_eq!(DEFAULT_DEBOUNCE_MS, 10, "the operator's 10 ms or less");
    assert_eq!(Settings::from_config(&json!({})), Settings { debounce_ms: 10 });
    assert_eq!(
        Settings::from_config(&json!({"debounce_ms": 20})),
        Settings { debounce_ms: 20 },
        "plan B's 20, written explicitly, is taken"
    );
    // every whole number of ms a u32 holds is taken as written: bounding it is the config validator's
    for debounce_ms in [0, 1, 1000, 1001, u32::MAX] {
        assert_eq!(
            Settings::from_config(&json!({ "debounce_ms": debounce_ms })),
            Settings { debounce_ms }
        );
    }
    // NEGATIVE: a negative, fractional, textual or too large debounce takes the default rather than wrapping
    for malformed in [
        json!(-1),
        json!(20.5),
        json!("5"),
        json!(null),
        json!(true),
        json!(u64::from(u32::MAX) + 1),
    ] {
        assert_eq!(
            Settings::from_config(&json!({ "debounce_ms": malformed })).debounce_ms,
            DEFAULT_DEBOUNCE_MS,
            "{malformed}"
        );
    }
    // LOW at setup (pads open) starts the latch separated; HIGH nested
    assert!(Separation::new(&Settings { debounce_ms: 10 }, false).separated());
    assert!(!Separation::new(&Settings { debounce_ms: 10 }, true).separated());
}
