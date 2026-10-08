//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the separation switch on the Rust cadence (one read per 10 ms control tick, the edges stamped in µs by
//! the interrupt), beyond its golden fixture (tests/golden_separation.rs), which replays plan B's 5 ms reads: the
//! decision table (separated, nested, blip; only a separation in BOOSTING advances), the operator's physics (a wobble
//! of about 7 ms as the strips slide out at 3 m/s, dated by its first edge and measured to its last), boost blips that
//! are recorded and never deploy, the quiet exact from the stamps, a gap past the quiet that splits a wobble, chatter
//! that keeps one episode, the polled pin and the tick's own read (a board powered up unassembled, then assembled),
//! the clock's wrap, and the settings. The generic stamps and their race are tested in debounce/tests.rs.

use super::{DEFAULT_DEBOUNCE_MS, Event, Kind, Separation, Settings};
use crate::debounce::{Episode, StampPair};
use board_config::LONGEST_DEBOUNCE_MS;
use sequencer::Stage;
use serde_json::json;

/// The control tick, µs: the default 100 Hz schedule.
const TICK_US: u32 = 10_000;

/// The fastest tick a config may ask for, µs: schedule_hz 1000.
const FAST_TICK_US: u32 = 1000;

/// The default quiet, µs.
const QUIET_US: u32 = DEFAULT_DEBOUNCE_MS * 1000;

/// The strips' travel as the operator measures it, µs: 2 cm at 3 m/s is about 7 ms, and any loss and restoring of
/// contact while they slide out falls inside it.
const WOBBLE_US: u32 = 7000;

/// How the switch learns that the line moved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pin {
    /// The both-edge interrupt records every change at its own time, however brief, and the tick observes its reads.
    Interrupt,
    /// The interrupt is armed but every edge is lost (or came before it was armed): the tick's own reads alone.
    Lost,
    /// No interrupt (an arm that failed): each read is polled against the previous one.
    Polled,
}

/// What a switch did over a timeline: its events, each with its read's time (µs), the stage it ended in, and the
/// switch itself.
struct Flight {
    /// The events, each with the time of the read that concluded it.
    events: Vec<(u32, Event)>,
    /// The stage after the last event.
    stage: Stage,
    /// The switch after the last read.
    switch: Separation,
}

impl Flight {
    /// The events' kinds, in order.
    fn kinds(&self) -> Vec<Kind> {
        self.events.iter().map(|(_, event)| event.kind).collect()
    }
}

/// The reads of a `tick_us` tick from `phase_us` to `end_us`, µs.
fn ticks(tick_us: u32, phase_us: u32, end_us: u32) -> impl Iterator<Item = u32> {
    (phase_us..=end_us).step_by(tick_us as usize)
}

/// Read a pin in `stage` with a `debounce_ms` quiet, its latch set up from the timeline's first level (bring-up's
/// read): `timeline` is the level (true = HIGH = nested) from each time on, µs, ascending from 0, and `reads` the
/// read times, µs, ascending. Each read is the firmware's tick: the interrupt's edges up to it recorded at their own
/// times ([`Pin::Interrupt`]), then the level, observed or polled, then the take and its conclusion by that level.
fn fly(debounce_ms: u32, stage: Stage, timeline: &[(u32, bool)], pin: Pin, reads: impl Iterator<Item = u32>) -> Flight {
    let pair = StampPair::new();
    let mut flight = Flight {
        events: Vec::new(),
        stage,
        switch: Separation::new(&Settings { debounce_ms }, timeline[0].1),
    };
    let (mut level, mut previous, mut next) = (timeline[0].1, timeline[0].1, 1);
    for read_us in reads {
        while next < timeline.len() && timeline[next].0 <= read_us {
            if timeline[next].1 != level && pin == Pin::Interrupt {
                pair.record(timeline[next].0);
            }
            level = timeline[next].1;
            next += 1;
        }
        match pin {
            Pin::Interrupt | Pin::Lost => pair.observe(read_us, level, flight.switch.latched()),
            Pin::Polled => pair.poll(read_us, level, previous, flight.switch.latched()),
        }
        previous = level;
        if let Some(episode) = pair.take(read_us, flight.switch.quiet_us()) {
            let event = flight.switch.conclude(episode, level, flight.stage);
            flight.stage = event.stage;
            flight.events.push((read_us, event));
        }
    }
    flight
}

/// [`fly`] in `stage` at the default quiet, on the 10 ms tick from `phase_us` for a second.
fn tick_through(timeline: &[(u32, bool)], pin: Pin, stage: Stage, phase_us: u32) -> Flight {
    fly(
        DEFAULT_DEBOUNCE_MS,
        stage,
        timeline,
        pin,
        ticks(TICK_US, phase_us, 1_000_000),
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

/// The first tick from `phase_us` at or after `at_us`, µs.
fn first_tick_from(at_us: u32, phase_us: u32) -> u32 {
    (phase_us..)
        .step_by(TICK_US as usize)
        .find(|tick_us| *tick_us >= at_us)
        .expect("a tick")
}

/// The first tick from `phase_us` that reads `level` on `timeline`, µs.
fn first_read_of(timeline: &[(u32, bool)], level: bool, phase_us: u32) -> u32 {
    (phase_us..)
        .step_by(TICK_US as usize)
        .find(|read_us| level_at(timeline, *read_us) == level)
        .expect("the timeline reaches the level")
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

/// An event's time fields: (moment, duration), µs.
fn dated(event: &Event) -> (u32, u32) {
    (event.moment_us, event.duration_us)
}

/// An episode from `start_us` to `finish_us`.
fn episode(start_us: u32, finish_us: u32) -> Episode {
    Episode { start_us, finish_us }
}

#[test]
fn the_level_read_at_the_conclusion_decides() {
    // the latch and the level read decide the kind; only a separation in BOOSTING moves the stage
    let stages = [
        Stage::Setting,
        Stage::Boosting,
        Stage::Gliding,
        Stage::Landing,
        Stage::Done,
    ];
    let table = [
        (true, false, Kind::Separated),
        (true, true, Kind::Blip),
        (false, true, Kind::Nested),
        (false, false, Kind::Blip),
    ];
    for stage in stages {
        for (setup_high, high, kind) in table {
            let mut switch = Separation::new(&Settings { debounce_ms: 10 }, setup_high);
            let event = switch.conclude(episode(1000, 4500), high, stage);
            let advanced = kind == Kind::Separated && stage == Stage::Boosting;
            assert_eq!(
                event,
                Event {
                    kind,
                    stage: if advanced { Stage::Gliding } else { stage },
                    advanced,
                    moment_us: 1000,
                    duration_us: 3500,
                },
                "{stage:?}, set up {setup_high}, read {high}"
            );
            // the latch follows the level read: a blip leaves it where it was
            assert_eq!(switch.separated(), !high, "{stage:?}: the latch");
            assert_eq!(switch.latched(), high, "{stage:?}: the level it stands for");
        }
    }
}

#[test]
fn every_event_is_named_in_its_row() {
    let named = [
        (Kind::Separated, "separated"),
        (Kind::Nested, "nested"),
        (Kind::Blip, "blip"),
    ];
    for (kind, name) in named {
        let event = Event {
            kind,
            stage: Stage::Boosting,
            advanced: false,
            moment_us: 0,
            duration_us: 0,
        };
        assert_eq!((event.name(), event.row()), (name, [name, "boosting"]));
    }
}

#[test]
fn the_operator_s_wobble_separates_once_dated_by_its_first_edge() {
    // seeded wobbles within the 7 ms the strips take to slide out, at every tick phase: one separation, its moment the
    // first 1 -> 0 and its duration to the last edge, exact to the µs, decided on the first tick a quiet after it
    let mut state = 0x5eed_0001;
    let (mut flights, mut wobbled) = (0, 0);
    for _ in 0..40 {
        let departure_us = within(&mut state, 20_000, 60_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, phase_us);
            let (read_us, event) = flight.events[0];
            assert_eq!(flight.kinds(), [Kind::Separated], "{timeline:?} at phase {phase_us}");
            assert_eq!(dated(&event), (departure_us, last_us - departure_us));
            assert_eq!(read_us, first_tick_from(last_us + QUIET_US, phase_us));
            assert!(event.advanced && flight.stage == Stage::Gliding && flight.switch.separated());
            flights += 1;
        }
        wobbled += usize::from(last_us > departure_us);
    }
    assert!(flights == 800 && wobbled > 20, "{flights} flights, {wobbled} wobbled");
}

#[test]
fn a_boost_blip_is_recorded_and_never_deploys() {
    // NEGATIVE: a wobble that returns to nested within 7 ms, at every phase: one blip, dated like a separation, the
    // latch and BOOSTING untouched -- armed, and polled where a read saw it (else nothing)
    let mut state = 0x5eed_0002;
    let mut polled_blips = 0;
    for _ in 0..40 {
        let departure_us = within(&mut state, 20_000, 60_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, false);
        for phase_us in phases() {
            let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, phase_us);
            assert_eq!(flight.kinds(), [Kind::Blip], "{timeline:?} at phase {phase_us}");
            assert_eq!(dated(&flight.events[0].1), (departure_us, last_us - departure_us));
            assert!(flight.stage == Stage::Boosting && !flight.switch.separated());
            let polled = tick_through(&timeline, Pin::Polled, Stage::Boosting, phase_us);
            assert!(
                polled.kinds().iter().all(|kind| *kind == Kind::Blip),
                "{timeline:?} polled"
            );
            assert!(polled.stage == Stage::Boosting && !polled.switch.separated());
            polled_blips += polled.events.len();
        }
    }
    assert!(polled_blips > 0, "a read never fell inside a blip");
}

#[test]
fn an_open_up_to_the_quiet_is_a_blip_and_one_a_tick_past_it_a_separation() {
    // the quiet is the line: an open of up to 10 ms concludes nested whatever the phase (no tick can find it a quiet
    // old and still open), one held 20 ms is found open by a tick at every phase -- and the re-nest after it is a
    // nested row that leaves GLIDING alone
    for open_us in [1, 500, 7000, 9999, QUIET_US] {
        let timeline = [(0, true), (30_000, false), (30_000 + open_us, true)];
        for phase_us in phases() {
            let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, phase_us);
            assert_eq!(flight.kinds(), [Kind::Blip], "open {open_us} µs at phase {phase_us}");
            assert_eq!(dated(&flight.events[0].1), (30_000, open_us));
            assert_eq!(flight.stage, Stage::Boosting);
        }
    }
    let timeline = [(0, true), (30_000, false), (50_000, true)];
    for phase_us in phases() {
        let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, phase_us);
        assert_eq!(flight.kinds(), [Kind::Separated, Kind::Nested], "phase {phase_us}");
        assert_eq!(dated(&flight.events[0].1), (30_000, 0));
        assert_eq!(dated(&flight.events[1].1), (50_000, 0));
        assert!(flight.events[0].1.advanced && !flight.events[1].1.advanced);
        assert_eq!(flight.stage, Stage::Gliding, "a re-nest never reverses the stage");
    }
}

#[test]
fn the_quiet_is_exact_to_the_microsecond_from_the_stamps() {
    // a single edge at 12.345 ms on a 1 kHz tick: duration 0, decided on the first tick 10 ms after it (23 ms, not 22);
    // a wobble's quiet counts from its last edge
    let flight = fly(
        DEFAULT_DEBOUNCE_MS,
        Stage::Boosting,
        &[(0, true), (12_345, false)],
        Pin::Interrupt,
        ticks(FAST_TICK_US, 0, 100_000),
    );
    let found: Vec<(u32, (u32, u32))> = flight
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(23_000, (12_345, 0))]);
    let flight = fly(
        DEFAULT_DEBOUNCE_MS,
        Stage::Boosting,
        &[(0, true), (12_345, false), (13_001, true), (14_567, false)],
        Pin::Interrupt,
        ticks(FAST_TICK_US, 0, 100_000),
    );
    let found: Vec<(u32, (u32, u32))> = flight
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, dated(event)))
        .collect();
    assert_eq!(found, [(25_000, (12_345, 2222))]);
}

#[test]
fn a_contact_gap_past_the_quiet_splits_a_wobble_and_never_reverses() {
    // LOW at 5 ms, back at 6 ms, a 12 ms gap nested, LOW for good at 18 ms: where a tick reads the gap a quiet after
    // its edge (16-18 ms) the first part concludes a blip and the separation is dated 18 ms; elsewhere one separation
    // from 5 ms. GLIDING either way
    let timeline = [(0, true), (5000, false), (6000, true), (18_000, false)];
    let (mut split, mut whole) = (0, 0);
    for phase_us in phases() {
        let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, phase_us);
        let dates: Vec<(Kind, (u32, u32))> = flight
            .events
            .iter()
            .map(|(_, event)| (event.kind, dated(event)))
            .collect();
        if phase_us % TICK_US >= 6000 && phase_us % TICK_US < 8000 {
            assert_eq!(
                dates,
                [(Kind::Blip, (5000, 1000)), (Kind::Separated, (18_000, 0))],
                "phase {phase_us}"
            );
            split += 1;
        } else {
            assert_eq!(dates, [(Kind::Separated, (5000, 13_000))], "phase {phase_us}");
            whole += 1;
        }
        assert_eq!(flight.stage, Stage::Gliding);
    }
    assert_eq!((split, whole), (4, 16));
}

#[test]
fn chatter_that_never_holds_still_keeps_one_episode() {
    // seeded chatter, toggling every 0.5-2 ms for 50-200 ms, then held: nothing while it chatters, then one event for
    // the whole run, dated by its first edge -- a separation where it ends open, a blip where it ends nested
    let mut state = 0x5eed_0003;
    for run in 0..20 {
        let ends_open = run % 2 == 0;
        let start_us = within(&mut state, 10_000, 50_000);
        let end_us = start_us + within(&mut state, 50_000, 200_000);
        let mut timeline = vec![(0, true)];
        let (mut at_us, mut level) = (start_us, true);
        while at_us < end_us {
            level = !level;
            timeline.push((at_us, level));
            at_us += within(&mut state, 500, 2000);
        }
        if level == ends_open {
            timeline.push((at_us, !ends_open));
        }
        let last_us = timeline.last().unwrap().0;
        let flight = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, 0);
        let kind = if ends_open { Kind::Separated } else { Kind::Blip };
        assert_eq!(flight.kinds(), [kind], "run {run}");
        let (read_us, event) = flight.events[0];
        assert_eq!(dated(&event), (start_us, last_us - start_us));
        assert_eq!(read_us, first_tick_from(last_us + QUIET_US, 0));
        assert_eq!(flight.stage == Stage::Gliding, ends_open);
    }
}

#[test]
fn a_polled_pin_is_dated_by_its_reads() {
    // the operator's wobble on a polled pin: the moment is the first read that saw LOW, the duration 0 (a 7 ms wobble
    // holds at most one 10 ms read), decided a tick later
    let mut state = 0x5eed_0004;
    for _ in 0..40 {
        let departure_us = within(&mut state, 20_000, 60_000);
        let mut timeline = vec![(0, true)];
        wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let flight = tick_through(&timeline, Pin::Polled, Stage::Boosting, phase_us);
            let moment_us = first_read_of(&timeline, false, phase_us);
            let found: Vec<(u32, Kind, (u32, u32))> = flight
                .events
                .iter()
                .map(|(read_us, event)| (*read_us, event.kind, dated(event)))
                .collect();
            assert_eq!(
                found,
                [(moment_us + TICK_US, Kind::Separated, (moment_us, 0))],
                "{timeline:?}"
            );
            assert_eq!(flight.stage, Stage::Gliding);
        }
    }
    // a slower wobble seen by three reads: the duration is whole ticks, where the interrupt has the µs
    let timeline = [(0, true), (3000, false), (14_000, true), (26_000, false)];
    let polled = tick_through(&timeline, Pin::Polled, Stage::Boosting, 0);
    let armed = tick_through(&timeline, Pin::Interrupt, Stage::Boosting, 0);
    assert_eq!(
        (polled.events[0].0, dated(&polled.events[0].1)),
        (40_000, (10_000, 20_000))
    );
    assert_eq!((armed.events[0].0, dated(&armed.events[0].1)), (40_000, (3000, 23_000)));
}

#[test]
fn a_board_powered_up_unassembled_ends_nested_once_assembled() {
    // the operator usually assembles with the power off, but a board powered up unassembled reads LOW at bring-up: its
    // latch starts separated, and assembling it -- the ring sliding in, wobbling for a few ms -- concludes nested in
    // SETTING, with the interrupt, with its edges lost (the tick's own read), and polled: the final level is 1
    let assembly = [(0, false), (500_000, true), (501_500, false), (503_000, true)];
    for pin in [Pin::Interrupt, Pin::Lost, Pin::Polled] {
        for phase_us in phases() {
            let flight = tick_through(&assembly, pin, Stage::Setting, phase_us);
            assert_eq!(flight.kinds(), [Kind::Nested], "{pin:?} at phase {phase_us}");
            let (read_us, event) = flight.events[0];
            if pin == Pin::Interrupt {
                assert_eq!(
                    (read_us, dated(&event)),
                    (first_tick_from(513_000, phase_us), (500_000, 3000))
                );
            } else {
                // the first read of contact, the ring's first touch or the ring in place, decided a tick later
                let moment_us = first_read_of(&assembly, true, phase_us);
                assert_eq!(
                    (read_us, dated(&event)),
                    (moment_us + TICK_US, (moment_us, 0)),
                    "{pin:?}"
                );
            }
            assert_eq!(flight.stage, Stage::Setting);
            assert!(!flight.switch.separated() && flight.switch.latched());
        }
    }
    // NEGATIVE: left unassembled, or assembled from the start, every read agrees with the latch: nothing, ever
    for level in [false, true] {
        for pin in [Pin::Interrupt, Pin::Lost, Pin::Polled] {
            let flight = tick_through(&[(0, level)], pin, Stage::Setting, 0);
            assert!(flight.events.is_empty(), "{pin:?}, level {level}");
            assert_eq!(flight.switch.separated(), !level);
        }
    }
}

#[test]
fn a_change_with_no_edge_is_seen_by_the_tick_s_read() {
    // the strips opened before the interrupt was armed (or their edge was lost): the first read off the latch opens
    // the episode, a quiet later it concludes -- in BOOSTING a separation, dated by that read
    let timeline = [(0, true), (1, false)];
    let flight = tick_through(&timeline, Pin::Lost, Stage::Boosting, 4000);
    let found: Vec<(u32, Kind, (u32, u32))> = flight
        .events
        .iter()
        .map(|(read_us, event)| (*read_us, event.kind, dated(event)))
        .collect();
    assert_eq!(found, [(14_000, Kind::Separated, (4000, 0))]);
    assert_eq!(flight.stage, Stage::Gliding);
}

#[test]
fn a_wobble_across_the_clock_s_wrap_keeps_its_moment_and_duration() {
    // the stamps are esp_timer's µs truncated to u32, wrapping every 71.6 minutes
    let pair = StampPair::new();
    let mut switch = Separation::new(&Settings::from_config(&json!({})), true);
    for edge_us in [u32::MAX - 3000, u32::MAX - 1000, 2000] {
        pair.record(edge_us);
    }
    let mut events = Vec::new();
    for read_us in [u32::MAX - 500, 5000, 11_999, 12_000] {
        pair.observe(read_us, false, switch.latched());
        if let Some(episode) = pair.take(read_us, switch.quiet_us()) {
            events.push((read_us, switch.conclude(episode, false, Stage::Boosting)));
        }
    }
    assert_eq!(events.len(), 1);
    let (read_us, event) = events[0];
    assert_eq!(
        (read_us, event.kind, dated(&event)),
        (12_000, Kind::Separated, (u32::MAX - 3000, 5001))
    );
}

#[test]
fn the_debounce_ms_is_the_quiet_in_microseconds() {
    // 25 ms: an edge at 5 ms on a 1 kHz tick is decided on the read at 30 ms exactly, not one before
    let flight = fly(
        25,
        Stage::Boosting,
        &[(0, true), (5000, false)],
        Pin::Interrupt,
        ticks(FAST_TICK_US, 0, 100_000),
    );
    assert_eq!(
        flight.events.iter().map(|(read_us, _)| *read_us).collect::<Vec<_>>(),
        [30_000]
    );
    // the validator's longest, 1 s, and 0 (decided on the edge's own tick)
    let longest = u32::try_from(LONGEST_DEBOUNCE_MS).expect("the bound fits");
    assert_eq!(
        Separation::new(&Settings { debounce_ms: longest }, true).quiet_us(),
        1_000_000
    );
    let flight = fly(
        0,
        Stage::Boosting,
        &[(0, true), (5000, false)],
        Pin::Interrupt,
        ticks(TICK_US, 0, 100_000),
    );
    assert_eq!(
        flight.events.iter().map(|(read_us, _)| *read_us).collect::<Vec<_>>(),
        [10_000]
    );
    // NEGATIVE: a debounce past the µs clock's range saturates, a quiet that never comes (the validator keeps it out)
    for debounce_ms in [4_294_968, u32::MAX] {
        assert_eq!(Separation::new(&Settings { debounce_ms }, true).quiet_us(), u32::MAX);
        let flight = fly(
            debounce_ms,
            Stage::Boosting,
            &[(0, true), (5000, false)],
            Pin::Interrupt,
            ticks(TICK_US, 0, 20_000_000),
        );
        assert!(
            flight.events.is_empty() && !flight.switch.separated(),
            "debounce {debounce_ms} ms"
        );
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
    // LOW at setup (the contact open) starts the latch separated; HIGH nested
    let open = Separation::new(&Settings { debounce_ms: 10 }, false);
    let nested = Separation::new(&Settings { debounce_ms: 10 }, true);
    assert!(open.separated() && !open.latched());
    assert!(!nested.separated() && nested.latched());
    assert_eq!(nested.quiet_us(), 10_000);
}
