//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the separation switch on the Rust cadence (one read per 10 ms control tick, the edges stamped in µs by
//! the interrupt), beyond its golden fixture (tests/golden_separation.rs), which replays plan B's 5 ms reads: the
//! decision table (separated, nested, blip; only a separation in BOOSTING advances), the operator's physics (a wobble
//! of about 7 ms as the strips slide out at 3 m/s, dated by its first edge and measured to its last), boost blips that
//! are recorded and never deploy, the quiet exact from the stamps, a gap past the quiet that splits a wobble, chatter
//! that keeps one episode, the polled pin and the tick's own read (a board powered up unassembled, then assembled),
//! the clock's wrap, and the settings. Then the thrust gate on its own (seen, then ended, leaky, afresh at each
//! BOOSTING entry, shut for a wrong axis or sign or no IMU) and the switch under it: the ejection after thrust end
//! counts as ungated, pads open before the gate opens count when it does, an early separation opens the gate itself,
//! the backstop, opens and chatter under thrust that never deploy (a wobbly open cleared by its re-nest included), a
//! touch while pending, a pending separation when BOOSTING ends otherwise, and the gate or the stage's end meeting a
//! contact closing again, which waits for its episode. The generic stamps and their race are tested in
//! debounce/tests.rs.

use super::{BACKSTOP_US, DEFAULT_DEBOUNCE_MS, Event, Kind, Separation, Settings, THRUST_END_MS, Thrust, ThrustGate};
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

/// A pin's timeline replayed read by read into a stamp pair, as the firmware's interrupt and tick feed it.
struct Line<'timeline> {
    /// The level (true = HIGH = nested) from each time on, µs, ascending from 0.
    timeline: &'timeline [(u32, bool)],
    /// How the switch learns that the line moved.
    pin: Pin,
    /// The level now.
    level: bool,
    /// The previous read's level: a polled pin's edges are the reads that differ from it.
    previous: bool,
    /// The next change to replay.
    next: usize,
}

impl<'timeline> Line<'timeline> {
    /// `timeline` before its first read, at its first level.
    fn new(timeline: &'timeline [(u32, bool)], pin: Pin) -> Line<'timeline> {
        Line {
            timeline,
            pin,
            level: timeline[0].1,
            previous: timeline[0].1,
            next: 1,
        }
    }

    /// The tick's read at `read_us` (µs, after the previous one): the changes up to it recorded into `pair` at their
    /// own times with the interrupt ([`Pin::Interrupt`]), then the level observed, or polled, against `latched`.
    /// Returns the level read.
    fn read(&mut self, pair: &StampPair, read_us: u32, latched: bool) -> bool {
        while self.next < self.timeline.len() && self.timeline[self.next].0 <= read_us {
            let (change_us, level) = self.timeline[self.next];
            if level != self.level && self.pin == Pin::Interrupt {
                pair.record(change_us);
            }
            self.level = level;
            self.next += 1;
        }
        match self.pin {
            Pin::Interrupt | Pin::Lost => pair.observe(read_us, self.level, latched),
            Pin::Polled => pair.poll(read_us, self.level, self.previous, latched),
        }
        self.previous = self.level;
        self.level
    }
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
    let mut line = Line::new(timeline, pin);
    for read_us in reads {
        let level = line.read(&pair, read_us, flight.switch.latched());
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

/// The boost's a_x on an IMU mounted per the body frame (+x at the nose), g: well above plan B's `launch_g` of 2.5.
const BOOST_G: f32 = 5.0;

/// The coast's a_x once nothing pushes the glider, g: drag decelerates it.
const COAST_G: f32 = -0.4;

/// The burn of the nominal flight, µs: thrust from the first read to 1 s.
const BURNOUT_US: u32 = 1_000_000;

/// The ejection charge's delay after thrust end, µs: the nominal separation, about 4 s.
const EJECTION_DELAY_US: u32 = 4_000_000;

/// The gate's ms clock at a read, from the read's µs: the firmware's uptime ms, which the tests' reads never wrap.
fn milliseconds(read_us: u32) -> u32 {
    read_us / 1000
}

/// A burn until `burnout_us`, then the coast, as an IMU mounted per the body frame reads a_x, g.
fn burn(burnout_us: u32) -> impl Fn(u32) -> Option<f32> {
    move |at_us| Some(if at_us < burnout_us { BOOST_G } else { COAST_G })
}

/// The same burn on an IMU mounted with +x to the tail: the thrust reads negative, the coast positive.
fn reversed(burnout_us: u32) -> impl Fn(u32) -> Option<f32> {
    move |at_us| Some(if at_us < burnout_us { -BOOST_G } else { -COAST_G })
}

/// An a_x profile: the glider's a_x in g at a read's time in µs, None when no source is fresh.
type Profile = Box<dyn Fn(u32) -> Option<f32>>;

/// No IMU, or none fresh: no a_x at any read.
fn no_accel(_: u32) -> Option<f32> {
    None
}

/// Steps `gate` every 10 ms from `from_ms` up to `to_ms` in `stage`, with the same a_x each time.
fn feed(gate: &mut ThrustGate, from_ms: u32, to_ms: u32, stage: Stage, axial_g: Option<f32>) {
    for now_ms in (from_ms..to_ms).step_by(10) {
        gate.step(now_ms, stage, axial_g);
    }
}

/// What a switch under the thrust gate did over a timeline.
struct Gated {
    /// The events, each with the time of the read that recorded it.
    events: Vec<(u32, Event)>,
    /// The stage after the last read.
    stage: Stage,
    /// The switch after the last read.
    switch: Separation,
    /// The gate after the last read.
    gate: ThrustGate,
    /// The first read the gate was open at, µs.
    opened_us: Option<u32>,
    /// The reads after which a separation was pending, µs.
    pending_us: Vec<u32>,
}

impl Gated {
    /// The events' kinds, in order.
    fn kinds(&self) -> Vec<Kind> {
        self.events.iter().map(|(_, event)| event.kind).collect()
    }
}

/// [`fly_gated_until`] at the default quiet, BOOSTING ended only by the switch.
fn fly_gated(timeline: &[(u32, bool)], pin: Pin, phase_us: u32, axial: impl Fn(u32) -> Option<f32>) -> Gated {
    fly_gated_until(timeline, pin, phase_us, axial, DEFAULT_DEBOUNCE_MS, u32::MAX)
}

/// A switch with a `debounce_ms` quiet under the thrust gate on plan B's sequencer defaults, read on the 10 ms tick
/// from `phase_us` until the quiet and 300 ms past the timeline's last change, in BOOSTING from the first read (the
/// launch just detected). Each read is the firmware's tick: the line read ([`Line::read`]), the gate stepped with
/// `axial` at the read, then the switch's tick by the gate's verdict. An advanced event moves the stage to GLIDING,
/// and so does the stage machine at the first read from `boost_end_us` (its apogee or burnout; u32::MAX for never).
fn fly_gated_until(
    timeline: &[(u32, bool)],
    pin: Pin,
    phase_us: u32,
    axial: impl Fn(u32) -> Option<f32>,
    debounce_ms: u32,
    boost_end_us: u32,
) -> Gated {
    let pair = StampPair::new();
    let mut line = Line::new(timeline, pin);
    let mut flight = Gated {
        events: Vec::new(),
        stage: Stage::Boosting,
        switch: Separation::new(&Settings { debounce_ms }, timeline[0].1),
        gate: ThrustGate::new(&sequencer::Settings::default()),
        opened_us: None,
        pending_us: Vec::new(),
    };
    let end_us = timeline.last().expect("a timeline").0 + debounce_ms * 1000 + 300_000;
    for read_us in ticks(TICK_US, phase_us, end_us) {
        if read_us >= boost_end_us && flight.stage == Stage::Boosting {
            flight.stage = Stage::Gliding;
        }
        let high = line.read(&pair, read_us, flight.switch.latched());
        flight.gate.step(milliseconds(read_us), flight.stage, axial(read_us));
        if flight.gate.open() && flight.opened_us.is_none() {
            flight.opened_us = Some(read_us);
        }
        if let Some(event) = flight
            .switch
            .tick(&pair, high, read_us, flight.stage, flight.gate.open())
        {
            flight.stage = event.stage;
            flight.events.push((read_us, event));
        }
        if flight.switch.pending() {
            flight.pending_us.push(read_us);
        }
    }
    flight
}

#[test]
fn the_gate_opens_once_thrust_is_seen_and_then_ended() {
    // plan B's defaults on the 10 ms tick: a_x above 2.5 g for 100 ms of net time, then below 0 g for 20 ms
    let mut gate = ThrustGate::new(&sequencer::Settings::default());
    assert_eq!(
        (gate.thrust(), gate.open()),
        (Thrust::Unseen, false),
        "shut before any BOOSTING"
    );
    let (mut changes, mut thrust) = (Vec::new(), gate.thrust());
    for now_ms in (0..2000).step_by(10) {
        gate.step(now_ms, Stage::Boosting, burn(1_000_000)(now_ms * 1000));
        if gate.thrust() != thrust {
            thrust = gate.thrust();
            changes.push((now_ms, thrust, gate.open()));
        }
    }
    assert_eq!(changes, [(100, Thrust::Seen, false), (1020, Thrust::Ended, true)]);
    // the sequencer entry's own numbers: 3 g for 200 ms
    let settings = sequencer::Settings::from_config(&json!({"launch_g": 3.0, "launch_ms": 200})).expect("valid");
    let mut gate = ThrustGate::new(&settings);
    feed(&mut gate, 0, 1000, Stage::Boosting, Some(2.9));
    assert_eq!(gate.thrust(), Thrust::Unseen, "2.9 g is no thrust at 3 g");
    feed(&mut gate, 1000, 1200, Stage::Boosting, Some(3.1));
    assert_eq!(gate.thrust(), Thrust::Unseen, "190 ms of net time");
    gate.step(1200, Stage::Boosting, Some(3.1));
    assert_eq!(gate.thrust(), Thrust::Seen, "200 ms");
    // NEGATIVE: both levels are strict, and NaN is neither thrust nor its end
    let mut gate = ThrustGate::new(&sequencer::Settings::default());
    feed(&mut gate, 0, 1000, Stage::Boosting, Some(2.5));
    feed(&mut gate, 1000, 2000, Stage::Boosting, Some(f32::NAN));
    assert_eq!(gate.thrust(), Thrust::Unseen, "exactly launch_g, then NaN");
    feed(&mut gate, 2000, 2110, Stage::Boosting, Some(BOOST_G));
    assert_eq!(gate.thrust(), Thrust::Seen);
    feed(&mut gate, 2110, 3000, Stage::Boosting, Some(0.0));
    feed(&mut gate, 3000, 4000, Stage::Boosting, Some(f32::NAN));
    assert_eq!(
        (gate.thrust(), gate.open()),
        (Thrust::Seen, false),
        "exactly 0 g, then NaN"
    );
    // the faintest drag below 0 g ends it, after the same 20 ms
    feed(&mut gate, 4000, 4020, Stage::Boosting, Some(-0.01));
    assert!(!gate.open(), "10 ms of net time");
    gate.step(4020, Stage::Boosting, Some(-0.01));
    assert_eq!((gate.thrust(), gate.open()), (Thrust::Ended, true), "-0.01 g for 20 ms");
}

#[test]
fn the_gate_s_dwells_are_leaky_like_the_launch_s() {
    // a missing a_x drains the thrust's credit for its step rather than restarting it: 60 ms, two misses, then 90 ms
    let mut gate = ThrustGate::new(&sequencer::Settings::default());
    feed(&mut gate, 0, 70, Stage::Boosting, Some(BOOST_G));
    feed(&mut gate, 70, 90, Stage::Boosting, None);
    feed(&mut gate, 90, 140, Stage::Boosting, Some(BOOST_G));
    assert_eq!(gate.thrust(), Thrust::Unseen, "90 ms of net time");
    gate.step(140, Stage::Boosting, Some(BOOST_G));
    assert_eq!(gate.thrust(), Thrust::Seen, "100 ms of net time at 140 ms");
    // the end's dwell alike, on a 1 kHz tick: 15 ms of drag, a 2 ms shudder of push, then drag: open at 24 ms
    for now_ms in 1000..=1015 {
        gate.step(now_ms, Stage::Boosting, Some(COAST_G));
    }
    for now_ms in 1016..=1017 {
        gate.step(now_ms, Stage::Boosting, Some(BOOST_G));
    }
    for now_ms in 1018..1024 {
        gate.step(now_ms, Stage::Boosting, Some(COAST_G));
        assert!(!gate.open(), "{now_ms} ms");
    }
    gate.step(1024, Stage::Boosting, Some(COAST_G));
    assert!(gate.open(), "20 ms of net time at 1024 ms");
    // NEGATIVE: a lone spike drains to nothing and restarts, however often it comes
    let mut spiked = ThrustGate::new(&sequencer::Settings::default());
    for now_ms in (0..2000).step_by(10) {
        let axial_g = if now_ms % 50 == 0 { BOOST_G } else { 1.0 };
        spiked.step(now_ms, Stage::Boosting, Some(axial_g));
    }
    assert_eq!(spiked.thrust(), Thrust::Unseen);
}

#[test]
fn the_gate_starts_afresh_at_each_boosting_entry_and_holds_still_elsewhere() {
    let mut gate = ThrustGate::new(&sequencer::Settings::default());
    // NEGATIVE: on the pad the gate steps nothing, a shake and a drop included
    feed(&mut gate, 0, 500, Stage::Setting, Some(BOOST_G));
    feed(&mut gate, 500, 1000, Stage::Setting, Some(COAST_G));
    assert_eq!(gate.thrust(), Thrust::Unseen);
    // a boost credited 50 ms, a hold out of BOOSTING and back: afresh, the full 100 ms again
    feed(&mut gate, 1000, 1060, Stage::Boosting, Some(BOOST_G));
    feed(&mut gate, 1060, 1100, Stage::Setting, Some(BOOST_G));
    feed(&mut gate, 1100, 1200, Stage::Boosting, Some(BOOST_G));
    assert_eq!(gate.thrust(), Thrust::Unseen, "the credit before the hold is gone");
    gate.step(1200, Stage::Boosting, Some(BOOST_G));
    assert_eq!(gate.thrust(), Thrust::Seen);
    feed(&mut gate, 1210, 1240, Stage::Boosting, Some(COAST_G));
    assert!(gate.open());
    // NEGATIVE: open, it stays open for the rest of this BOOSTING, a fresh push of twice launch_ms included
    feed(&mut gate, 1240, 1500, Stage::Boosting, Some(BOOST_G));
    assert_eq!(
        (gate.thrust(), gate.open()),
        (Thrust::Ended, true),
        "a push after the end"
    );
    // GLIDING and beyond: it keeps what it saw, whatever a_x does
    feed(&mut gate, 1500, 3000, Stage::Gliding, Some(BOOST_G));
    feed(&mut gate, 3000, 4000, Stage::Done, Some(1.0));
    assert_eq!((gate.thrust(), gate.open()), (Thrust::Ended, true));
    // the next BOOSTING entry (an operator hold, or the next flight) shuts it at once, whatever a_x says
    gate.step(4000, Stage::Boosting, Some(COAST_G));
    assert_eq!((gate.thrust(), gate.open()), (Thrust::Unseen, false));
}

#[test]
fn a_wrong_axis_or_sign_or_no_imu_never_opens_the_gate() {
    // NEGATIVE, the fail-safe: +x to the tail (the burn reads negative, the coast positive), a lateral axis, no IMU,
    // and a bench hold nose up or flat never see thrust, so the gate stays shut through a whole boost and coast
    let profiles: [(&str, Profile); 5] = [
        ("+x to the tail", Box::new(reversed(BURNOUT_US))),
        ("a lateral axis", Box::new(|_| Some(0.3))),
        ("no IMU", Box::new(no_accel)),
        ("a bench hold nose up", Box::new(|_| Some(1.0))),
        ("a bench hold flat", Box::new(|_| Some(0.0))),
    ];
    for (name, axial) in &profiles {
        let mut gate = ThrustGate::new(&sequencer::Settings::default());
        for read_us in ticks(TICK_US, 0, 6_000_000) {
            gate.step(milliseconds(read_us), Stage::Boosting, axial(read_us));
            assert_eq!(
                (gate.thrust(), gate.open()),
                (Thrust::Unseen, false),
                "{name} at {read_us} µs"
            );
        }
    }
}

#[test]
fn every_thrust_is_named_for_control() {
    let named = [
        (Thrust::Unseen, "none"),
        (Thrust::Seen, "seen"),
        (Thrust::Ended, "ended"),
    ];
    for (thrust, name) in named {
        assert_eq!(thrust.name(), name);
    }
}

#[test]
fn outside_boosting_or_with_the_gate_open_the_tick_is_the_bare_conclusion() {
    // every stage, latch and level, either gate: the tick's event and switch are the bare conclusion's -- but for a
    // separation in BOOSTING under the shut gate, which is pending: no event, in the latch, not counted
    let table = [
        (true, false, Kind::Separated),
        (true, true, Kind::Blip),
        (false, true, Kind::Nested),
        (false, false, Kind::Blip),
    ];
    for stage in Stage::ALL {
        for gate_open in [false, true] {
            for (setup_high, high, kind) in table {
                let pair = StampPair::new();
                pair.record(1000);
                pair.record(4500);
                let mut gated = Separation::new(&Settings { debounce_ms: 10 }, setup_high);
                let mut bare = gated;
                let found = gated.tick(&pair, high, 14_500, stage, gate_open);
                let owed = bare.conclude(episode(1000, 4500), high, stage);
                let case = format!("{stage:?}, gate open {gate_open}, set up {setup_high}, read {high}");
                if stage == Stage::Boosting && !gate_open && kind == Kind::Separated {
                    assert_eq!(found, None, "{case}");
                    assert!(gated.pending() && !gated.separated() && !gated.latched(), "{case}");
                } else {
                    assert_eq!(found, Some(owed), "{case}");
                    assert_eq!(gated, bare, "{case}");
                    assert!(!gated.pending(), "{case}");
                }
            }
        }
    }
}

#[test]
fn a_pending_separation_is_in_the_latch_but_not_counted() {
    // an edge at 100 ms in BOOSTING under the shut gate: concluded at 110 ms and pending -- the latch follows the pin,
    // so the tick's LOW reads open no episode, but the counted latch waits -- until the backstop at 200 ms
    let pair = StampPair::new();
    let mut switch = Separation::new(&Settings::from_config(&json!({})), true);
    pair.record(100_000);
    for read_us in (110_000..200_000).step_by(10_000) {
        pair.observe(read_us, false, switch.latched());
        assert_eq!(
            switch.tick(&pair, false, read_us, Stage::Boosting, false),
            None,
            "{read_us} µs"
        );
        assert!(!pair.episode_open(), "{read_us} µs: a LOW read agrees with the latch");
        assert!(
            switch.pending() && !switch.separated() && !switch.latched(),
            "{read_us} µs"
        );
    }
    pair.observe(200_000, false, switch.latched());
    assert_eq!(
        switch.tick(&pair, false, 200_000, Stage::Boosting, false),
        Some(Event {
            kind: Kind::Separated,
            stage: Stage::Gliding,
            advanced: true,
            moment_us: 100_000,
            duration_us: 0,
        }),
        "the backstop, 100 ms after the edge"
    );
    assert!(switch.separated() && !switch.pending() && !switch.latched());
    // NEGATIVE: once counted, LOW reads record nothing more
    pair.observe(300_000, false, switch.latched());
    assert_eq!(switch.tick(&pair, false, 300_000, Stage::Gliding, true), None);
}

#[test]
fn the_ejection_after_thrust_end_counts_at_its_conclusion_as_ungated() {
    // the nominal flight: a 1 s burn, the gate open 20 ms after the first coast read, and the ejection charge's seeded
    // wobble 4 s later, at every phase: the bare conclusion's events on the same reads, never a pending separation
    let mut state = 0x5eed_0101;
    for _ in 0..10 {
        let departure_us = BURNOUT_US + EJECTION_DELAY_US + within(&mut state, 0, 50_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(BURNOUT_US));
            let reads = ticks(TICK_US, phase_us, last_us + QUIET_US + 300_000);
            let bare = fly(DEFAULT_DEBOUNCE_MS, Stage::Boosting, &timeline, Pin::Interrupt, reads);
            assert_eq!(gated.events, bare.events, "{timeline:?} at phase {phase_us}");
            assert_eq!(gated.kinds(), [Kind::Separated]);
            let (read_us, event) = gated.events[0];
            assert_eq!(
                (read_us, dated(&event)),
                (
                    first_tick_from(last_us + QUIET_US, phase_us),
                    (departure_us, last_us - departure_us)
                )
            );
            assert!(event.advanced && gated.stage == Stage::Gliding && gated.switch.separated());
            assert_eq!(
                gated.opened_us,
                Some(first_tick_from(BURNOUT_US, phase_us) + THRUST_END_MS * 1000)
            );
            assert!(gated.pending_us.is_empty(), "never pending");
        }
    }
}

#[test]
fn pads_open_before_the_gate_opens_count_when_it_opens_dated_by_the_first_edge() {
    // a seeded wobble 20-60 ms before thrust end, left open: pending from its conclusion, it counts on the read the
    // gate opens at (20 ms after the first coast read, before the backstop's 100 ms), its moment the first edge
    let mut state = 0x5eed_0102;
    for _ in 0..20 {
        let departure_us = BURNOUT_US - within(&mut state, 20_000, 60_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(BURNOUT_US));
            let opened_us = first_tick_from(BURNOUT_US, phase_us) + THRUST_END_MS * 1000;
            assert_eq!(gated.opened_us, Some(opened_us));
            assert_eq!(gated.kinds(), [Kind::Separated], "{timeline:?} at phase {phase_us}");
            let (read_us, event) = gated.events[0];
            assert_eq!(
                (read_us, dated(&event)),
                (opened_us, (departure_us, last_us - departure_us))
            );
            assert!(event.advanced && gated.stage == Stage::Gliding);
            assert_eq!(
                (gated.pending_us.first(), gated.pending_us.last()),
                (
                    Some(&first_tick_from(last_us + QUIET_US, phase_us)),
                    Some(&(opened_us - TICK_US))
                ),
                "pending from the conclusion until the gate opened"
            );
        }
    }
}

#[test]
fn an_early_separation_opens_the_gate_itself_and_counts_within_its_20_ms() {
    // a destabilized stack loses the glider at a seeded moment mid-burn: the pads open with a wobble and the glider's
    // a_x drops to drag at once (the booster burns on, but no longer pushes it). The gate opens 20 ms after the first
    // read of the drop, and the separation counts then, or at its own conclusion if that is later: within 30 ms of
    // its first edge on the 10 ms tick
    let mut state = 0x5eed_0103;
    let (mut by_the_gate, mut by_the_conclusion) = (0, 0);
    for _ in 0..40 {
        let separation_us = within(&mut state, 300_000, 900_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, separation_us, true);
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(separation_us));
            let opened_us = first_tick_from(separation_us, phase_us) + THRUST_END_MS * 1000;
            let concluded_us = first_tick_from(last_us + QUIET_US, phase_us);
            assert_eq!(gated.opened_us, Some(opened_us));
            assert_eq!(gated.kinds(), [Kind::Separated], "{timeline:?} at phase {phase_us}");
            let (read_us, event) = gated.events[0];
            assert_eq!(
                (read_us, dated(&event)),
                (opened_us.max(concluded_us), (separation_us, last_us - separation_us))
            );
            assert!(read_us - separation_us <= THRUST_END_MS * 1000 + TICK_US);
            assert!(event.advanced && gated.stage == Stage::Gliding);
            if opened_us > concluded_us {
                by_the_gate += 1;
            } else {
                by_the_conclusion += 1;
            }
        }
    }
    assert!(
        by_the_gate > 0 && by_the_conclusion > 0,
        "{by_the_gate} by the gate, {by_the_conclusion} by the conclusion"
    );
}

#[test]
fn the_backstop_counts_a_held_open_when_no_thrust_was_ever_seen() {
    // no IMU (or a dead one): a seeded wobble left open is pending from its conclusion until a read finds the pin LOW
    // 100 ms after its last edge, then counts, dated by its first edge
    let mut state = 0x5eed_0104;
    for _ in 0..20 {
        let departure_us = within(&mut state, 200_000, 600_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, no_accel);
            let counted_us = first_tick_from(last_us + BACKSTOP_US, phase_us);
            assert_eq!(gated.kinds(), [Kind::Separated], "{timeline:?} at phase {phase_us}");
            let (read_us, event) = gated.events[0];
            assert_eq!(
                (read_us, dated(&event)),
                (counted_us, (departure_us, last_us - departure_us))
            );
            assert!(event.advanced && gated.stage == Stage::Gliding);
            assert_eq!(
                (gated.pending_us.first(), gated.pending_us.last()),
                (
                    Some(&first_tick_from(last_us + QUIET_US, phase_us)),
                    Some(&(counted_us - TICK_US))
                )
            );
            assert!(gated.opened_us.is_none() && gated.switch.separated() && !gated.switch.pending());
        }
    }
    // the tick's own read and the polled pin alike, dated by the first read of LOW: counted 100 ms after it
    let timeline = [(0, true), (250_300, false)];
    for pin in [Pin::Lost, Pin::Polled] {
        for phase_us in phases() {
            let gated = fly_gated(&timeline, pin, phase_us, no_accel);
            let moment_us = first_read_of(&timeline, false, phase_us);
            let found: Vec<(u32, Kind, (u32, u32))> = gated
                .events
                .iter()
                .map(|(read_us, event)| (*read_us, event.kind, dated(event)))
                .collect();
            assert_eq!(
                found,
                [(moment_us + BACKSTOP_US, Kind::Separated, (moment_us, 0))],
                "{pin:?} at phase {phase_us}"
            );
        }
    }
}

#[test]
fn a_reversed_axis_or_no_accel_leaves_only_the_backstop() {
    // NEGATIVE, the fail-safe in flight: the nominal flight with +x to the tail, and with no accel. The gate never
    // opens, so a 50 ms open mid-burn is one blip that never deploys, and the ejection counts by the backstop, 100 ms
    // after its last edge instead of a quiet after it
    let mut state = 0x5eed_0105;
    for _ in 0..10 {
        let departure_us = BURNOUT_US + EJECTION_DELAY_US + within(&mut state, 0, 50_000);
        let mut timeline = vec![(0, true), (400_000, false), (450_000, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        for phase_us in phases() {
            let flights = [
                (
                    "+x to the tail",
                    fly_gated(&timeline, Pin::Interrupt, phase_us, reversed(BURNOUT_US)),
                ),
                ("no accel", fly_gated(&timeline, Pin::Interrupt, phase_us, no_accel)),
            ];
            for (name, gated) in flights {
                assert!(
                    gated.opened_us.is_none() && gated.gate.thrust() == Thrust::Unseen,
                    "{name}"
                );
                assert_eq!(
                    gated.kinds(),
                    [Kind::Blip, Kind::Separated],
                    "{name} at phase {phase_us}"
                );
                assert_eq!(dated(&gated.events[0].1), (400_000, 50_000), "{name}");
                assert!(!gated.events[0].1.advanced, "{name}");
                let (read_us, event) = gated.events[1];
                assert_eq!(
                    (read_us, dated(&event)),
                    (
                        first_tick_from(last_us + BACKSTOP_US, phase_us),
                        (departure_us, last_us - departure_us)
                    ),
                    "{name}"
                );
                assert!(event.advanced && gated.stage == Stage::Gliding, "{name}");
            }
        }
    }
}

#[test]
fn an_open_under_thrust_up_to_the_backstop_never_deploys() {
    // NEGATIVE: thrust that never ends, and a clean open from 300 ms for 1 µs up to 100 ms, at every phase: one blip
    // from the open to the re-nest, BOOSTING kept, the latch and the count nested. An open longer than the quiet was
    // pending first, and its re-nest cleared it
    for open_us in [1, 500, 7000, QUIET_US, 20_000, 50_000, 99_000, BACKSTOP_US] {
        let timeline = [(0, true), (300_000, false), (300_000 + open_us, true)];
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(u32::MAX));
            let case = format!("open {open_us} µs at phase {phase_us}");
            assert_eq!(gated.kinds(), [Kind::Blip], "{case}");
            assert_eq!(dated(&gated.events[0].1), (300_000, open_us), "{case}");
            assert_eq!(gated.stage, Stage::Boosting, "{case}");
            assert!(
                !gated.switch.separated() && gated.switch.latched() && !gated.switch.pending(),
                "{case}"
            );
            assert_eq!(
                gated.pending_us.is_empty(),
                open_us <= QUIET_US,
                "{case}: pending first"
            );
        }
    }
    // the tick's own read and the polled pin alike: a 50 ms open is one blip
    let timeline = [(0, true), (300_000, false), (350_000, true)];
    for pin in [Pin::Lost, Pin::Polled] {
        for phase_us in phases() {
            let gated = fly_gated(&timeline, pin, phase_us, burn(u32::MAX));
            assert_eq!(gated.kinds(), [Kind::Blip], "{pin:?} at phase {phase_us}");
            assert_eq!(gated.stage, Stage::Boosting);
        }
    }
    // a contact held open a tick past the backstop counts by it whatever the IMU says (the stack came apart and the
    // a_x never showed it), and its re-nest is a nested row that leaves GLIDING alone
    let timeline = [(0, true), (300_000, false), (300_000 + BACKSTOP_US + TICK_US, true)];
    for phase_us in phases() {
        let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(u32::MAX));
        assert_eq!(gated.kinds(), [Kind::Separated, Kind::Nested], "phase {phase_us}");
        assert_eq!(gated.events[0].0, first_tick_from(300_000 + BACKSTOP_US, phase_us));
        assert!(gated.events[0].1.advanced && !gated.events[1].1.advanced);
        assert_eq!(gated.stage, Stage::Gliding);
    }
}

#[test]
fn a_wobbly_open_cleared_by_its_re_nest_is_one_blip_from_its_first_edge() {
    // NEGATIVE: under thrust, a seeded wobble left open past the quiet (pending), then a re-nest before the backstop,
    // at every phase: one blip from the wobble's first edge to the re-nest, BOOSTING kept, the latch nested
    let mut state = 0x5eed_0107;
    let mut wobbled = 0;
    for _ in 0..20 {
        let departure_us = within(&mut state, 200_000, 500_000);
        let mut timeline = vec![(0, true)];
        let last_us = wobble(&mut state, &mut timeline, departure_us, true);
        let re_nest_us = last_us + within(&mut state, 25_000, 95_000);
        timeline.push((re_nest_us, true));
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(u32::MAX));
            let case = format!("{timeline:?} at phase {phase_us}");
            assert_eq!(
                gated.events,
                [(
                    first_tick_from(re_nest_us + QUIET_US, phase_us),
                    Event {
                        kind: Kind::Blip,
                        stage: Stage::Boosting,
                        advanced: false,
                        moment_us: departure_us,
                        duration_us: re_nest_us - departure_us,
                    }
                )],
                "{case}"
            );
            assert!(!gated.pending_us.is_empty(), "{case}: pending first");
            assert!(
                !gated.switch.separated() && gated.switch.latched() && !gated.switch.pending(),
                "{case}"
            );
        }
        wobbled += usize::from(last_us > departure_us);
    }
    assert!(wobbled > 0, "no open wobbled");
}

#[test]
fn chatter_under_thrust_never_deploys_and_left_open_counts_by_the_backstop() {
    // seeded chatter under thrust, toggling every 0.5-2 ms for 50-200 ms: one episode, which never concludes while it
    // chatters. Ending nested it is one blip, never pending; ending open it is pending, and counts by the backstop
    // 100 ms after its last edge, dated by its first
    let mut state = 0x5eed_0106;
    for run in 0..20 {
        let ends_open = run % 2 == 0;
        let start_us = within(&mut state, 200_000, 500_000);
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
        let gated = fly_gated(&timeline, Pin::Interrupt, 0, burn(u32::MAX));
        let (read_us, event) = gated.events[0];
        assert_eq!(dated(&event), (start_us, last_us - start_us), "run {run}");
        if ends_open {
            assert_eq!(gated.kinds(), [Kind::Separated], "run {run}");
            assert_eq!(read_us, first_tick_from(last_us + BACKSTOP_US, 0));
            assert!(event.advanced && gated.stage == Stage::Gliding);
        } else {
            assert_eq!(gated.kinds(), [Kind::Blip], "run {run}");
            assert!(gated.pending_us.is_empty() && gated.stage == Stage::Boosting);
        }
    }
}

#[test]
fn a_touch_while_pending_restarts_the_backstop_from_its_last_edge() {
    // no thrust: the pads open at 300 ms and stay open but for a 3 ms touch, which is part of the pending separation:
    // the backstop runs from the touch's last edge, and the row spans to it. A touch just before the first backstop
    // would come (395 ms) is still an open episode at 400 ms, so it holds the backstop too
    for touch_us in [350_000, 395_000] {
        let timeline = [(0, true), (300_000, false), (touch_us, true), (touch_us + 3000, false)];
        for phase_us in phases() {
            let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, no_accel);
            assert_eq!(
                gated.kinds(),
                [Kind::Separated],
                "touch at {touch_us} µs, phase {phase_us}"
            );
            let (read_us, event) = gated.events[0];
            assert_eq!(
                (read_us, dated(&event)),
                (
                    first_tick_from(touch_us + 3000 + BACKSTOP_US, phase_us),
                    (300_000, touch_us + 3000 - 300_000)
                ),
                "touch at {touch_us} µs, phase {phase_us}"
            );
            assert!(event.advanced);
        }
    }
}

#[test]
fn a_pending_separation_counts_when_boosting_ends_otherwise() {
    // no thrust: the pads open at 300 ms, pending; the stage machine moves BOOSTING to GLIDING at 350 ms (its apogee or
    // burnout), before the backstop: the first read in GLIDING counts it there, dated by its first edge, nothing
    // advanced
    let timeline = [(0, true), (300_000, false)];
    for phase_us in phases() {
        let gated = fly_gated_until(
            &timeline,
            Pin::Interrupt,
            phase_us,
            no_accel,
            DEFAULT_DEBOUNCE_MS,
            350_000,
        );
        assert_eq!(
            gated.events,
            [(
                first_tick_from(350_000, phase_us),
                Event {
                    kind: Kind::Separated,
                    stage: Stage::Gliding,
                    advanced: false,
                    moment_us: 300_000,
                    duration_us: 0,
                }
            )],
            "phase {phase_us}"
        );
        assert!(gated.switch.separated() && !gated.switch.pending());
    }
    // NEGATIVE: re-closed at 345 ms, before BOOSTING ended: the re-nest, concluded in GLIDING, clears it -- one blip
    // from the open to the re-nest, and the counted latch never moved
    let timeline = [(0, true), (300_000, false), (345_000, true)];
    for phase_us in phases() {
        let gated = fly_gated_until(
            &timeline,
            Pin::Interrupt,
            phase_us,
            no_accel,
            DEFAULT_DEBOUNCE_MS,
            350_000,
        );
        assert_eq!(
            gated.events,
            [(
                first_tick_from(355_000, phase_us),
                Event {
                    kind: Kind::Blip,
                    stage: Stage::Gliding,
                    advanced: false,
                    moment_us: 300_000,
                    duration_us: 45_000,
                }
            )],
            "phase {phase_us}"
        );
        assert!(!gated.switch.separated() && gated.switch.latched() && !gated.switch.pending());
    }
    // the contact closing again across the end, with a 3 ms bounce (LOW 343-346 ms): a read of the bounce in GLIDING
    // is no held LOW, so the separation waits for that episode. NEGATIVE: ending closed, its conclusion clears it, one
    // blip from the open to the last edge; ending open (a touch), it folds in and counts at its conclusion, spanning it
    let mut bounces_read = 0;
    for (timeline, ends_open, last_us) in [
        (
            &[
                (0, true),
                (300_000, false),
                (338_000, true),
                (343_000, false),
                (346_000, true),
            ][..],
            false,
            346_000,
        ),
        (
            &[(0, true), (300_000, false), (338_000, true), (343_000, false)][..],
            true,
            343_000,
        ),
    ] {
        for phase_us in phases() {
            let gated = fly_gated_until(
                timeline,
                Pin::Interrupt,
                phase_us,
                no_accel,
                DEFAULT_DEBOUNCE_MS,
                345_000,
            );
            let kind = if ends_open { Kind::Separated } else { Kind::Blip };
            assert_eq!(
                gated.events,
                [(
                    first_tick_from(last_us + QUIET_US, phase_us),
                    Event {
                        kind,
                        stage: Stage::Gliding,
                        advanced: false,
                        moment_us: 300_000,
                        duration_us: last_us - 300_000,
                    }
                )],
                "ends open {ends_open}, phase {phase_us}"
            );
            assert_eq!(gated.switch.separated(), ends_open);
            assert!(!gated.switch.pending());
            bounces_read += usize::from(!ends_open && !level_at(timeline, first_tick_from(345_000, phase_us)));
        }
    }
    assert!(bounces_read > 0, "no read in GLIDING fell on the bounce");
}

#[test]
fn a_quiet_of_the_backstop_or_longer_counts_at_the_conclusion() {
    // a debounce_ms of 100 ms or more: by the conclusion the pin has held LOW for the backstop, so a separation in
    // BOOSTING counts there even under a shut gate, and is never left pending after a tick
    let timeline = [(0, true), (300_000, false)];
    for debounce_ms in [100, 150, 1000] {
        for phase_us in phases() {
            let gated = fly_gated_until(&timeline, Pin::Interrupt, phase_us, no_accel, debounce_ms, u32::MAX);
            let found: Vec<(u32, Kind)> = gated
                .events
                .iter()
                .map(|(read_us, event)| (*read_us, event.kind))
                .collect();
            assert_eq!(
                found,
                [(first_tick_from(300_000 + debounce_ms * 1000, phase_us), Kind::Separated)],
                "debounce {debounce_ms} ms at phase {phase_us}"
            );
            assert!(gated.pending_us.is_empty() && gated.stage == Stage::Gliding);
        }
    }
}

#[test]
fn the_gate_opening_on_a_closed_contact_counts_nothing_and_the_re_nest_clears_it() {
    // NEGATIVE: the pads open 50 ms before thrust end and close again at 1015 ms, just before the gate opens: the gate
    // finds the pin HIGH and counts nothing, and the re-nest makes the open one blip. BOOSTING goes on, and the
    // ejection 4 s later counts at its conclusion under the open gate
    let ejection_us = BURNOUT_US + EJECTION_DELAY_US;
    let timeline = [(0, true), (950_000, false), (1_015_000, true), (ejection_us, false)];
    for phase_us in phases() {
        let gated = fly_gated(&timeline, Pin::Interrupt, phase_us, burn(BURNOUT_US));
        let found: Vec<(u32, Kind, (u32, u32), bool)> = gated
            .events
            .iter()
            .map(|(read_us, event)| (*read_us, event.kind, dated(event), event.advanced))
            .collect();
        assert_eq!(
            found,
            [
                (
                    first_tick_from(1_025_000, phase_us),
                    Kind::Blip,
                    (950_000, 65_000),
                    false
                ),
                (
                    first_tick_from(ejection_us + QUIET_US, phase_us),
                    Kind::Separated,
                    (ejection_us, 0),
                    true
                ),
            ],
            "phase {phase_us}"
        );
        assert!(gated.opened_us.is_some_and(|opened_us| opened_us < ejection_us));
    }
}

#[test]
fn a_gate_opening_inside_a_closing_contact_s_episode_waits_for_its_conclusion() {
    // pending from the 950 ms open, the contact closes again at 1012 ms with a 3 ms bounce (LOW from 1015 ms) across
    // the gate's opening (1020 ms at phase 0): a read of the bounce is no held LOW, so the separation waits for that
    // episode. NEGATIVE: re-closed at 1022 ms, its conclusion clears it, one blip from the open to the last edge,
    // BOOSTING kept; left open (a touch), it folds in and counts at its conclusion, or at the gate's opening if that
    // is later, spanning it
    let mut bounces_read = 0;
    for (timeline, ends_open, last_us) in [
        (
            &[
                (0, true),
                (950_000, false),
                (1_012_000, true),
                (1_015_000, false),
                (1_022_000, true),
            ][..],
            false,
            1_022_000,
        ),
        (
            &[(0, true), (950_000, false), (1_012_000, true), (1_015_000, false)][..],
            true,
            1_015_000,
        ),
    ] {
        for phase_us in phases() {
            let gated = fly_gated(timeline, Pin::Interrupt, phase_us, burn(BURNOUT_US));
            let opened_us = first_tick_from(BURNOUT_US, phase_us) + THRUST_END_MS * 1000;
            assert_eq!(gated.opened_us, Some(opened_us));
            let (kind, stage) = if ends_open {
                (Kind::Separated, Stage::Gliding)
            } else {
                (Kind::Blip, Stage::Boosting)
            };
            assert_eq!(
                gated.events,
                [(
                    first_tick_from(last_us + QUIET_US, phase_us).max(opened_us),
                    Event {
                        kind,
                        stage,
                        advanced: ends_open,
                        moment_us: 950_000,
                        duration_us: last_us - 950_000,
                    }
                )],
                "ends open {ends_open}, phase {phase_us}"
            );
            assert_eq!((gated.stage, gated.switch.separated()), (stage, ends_open));
            assert!(!gated.switch.pending());
            bounces_read += usize::from(!ends_open && !level_at(timeline, opened_us));
        }
    }
    assert!(bounces_read > 0, "the gate never opened on the bounce");
}
