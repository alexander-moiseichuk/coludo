//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the separation debounce on the Rust cadence (one read per 10 ms control tick), beyond its golden
//! fixture (tests/golden_separation.rs), which replays plan B's 5 ms reads.

use super::{Debounce, Event, Separation, Settings, Verdict};
use sequencer::Stage;
use serde_json::json;

/// The control tick, µs.
const TICK_US: u32 = 10_000;

/// A switch with the default 20 ms debounce whose pin read `high` at setup.
fn switch(high: bool) -> Separation {
    Separation::new(&Settings { debounce_ms: 20 }, high)
}

/// How the switch learns that the line moved between two reads.
#[derive(Clone, Copy, Debug)]
enum Pin {
    /// The both-edge interrupt flags every change, however brief.
    Interrupt,
    /// No interrupt (an arm that failed): the reads alone, the edge never flagged (the edge contract).
    Polled,
}

/// Read a pin through 10 ms ticks from `phase_us` to `end_us`: `timeline` is the level (true = HIGH = nested) from
/// each time on, in µs, ascending from 0. Each tick reads the level; on a [`Pin::Interrupt`] it flags an edge when
/// the line changed since the previous read, however briefly. Returns the events with their read's time in µs; the
/// stage follows them.
fn read_on_ticks(
    switch: &mut Separation,
    stage: &mut Stage,
    timeline: &[(u32, bool)],
    pin: Pin,
    phase_us: u32,
    end_us: u32,
) -> Vec<(u32, Event)> {
    let mut level = timeline[0].1;
    let mut next = 1;
    let mut events = Vec::new();
    for read_us in (phase_us..=end_us).step_by(TICK_US as usize) {
        let mut edge = false;
        while next < timeline.len() && timeline[next].0 <= read_us {
            edge |= timeline[next].1 != level;
            level = timeline[next].1;
            next += 1;
        }
        let edge = edge && matches!(pin, Pin::Interrupt);
        if let Some(event) = switch.step(read_us / 1000, level, edge, *stage) {
            *stage = event.stage;
            events.push((read_us, event));
        }
    }
    events
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

#[test]
fn a_held_separation_in_boosting_glides() {
    // test_separation.py's timeline: the pads open at 5 ms and stay open. Polled too: the level rule alone settles it
    let timeline = in_microseconds(&[(0, true), (5, false)]);
    for pin in [Pin::Interrupt, Pin::Polled] {
        for phase in phases() {
            let mut switch = switch(true);
            let mut stage = Stage::Boosting;
            let events = read_on_ticks(&mut switch, &mut stage, &timeline, pin, phase, 500_000);
            assert_eq!(events.len(), 1, "{pin:?}, phase {phase} µs");
            let (read_us, event) = events[0];
            // the first open read, then 10 and 20 ms on: three agreeing reads spanning 20 ms, 20-30 ms after it opened
            assert!(
                (25_000..=35_000).contains(&read_us),
                "{pin:?}, phase {phase} µs: at {read_us} µs"
            );
            assert_eq!(event.row(), ["separated", "gliding"]);
            assert!(event.advanced && switch.separated());
            assert_eq!(stage, Stage::Gliding);
        }
    }
}

#[test]
fn a_bounce_then_hold_glides() {
    // test_separation.py's timeline: three bounces, open for good from 15 ms; the restart rule waits out the bounces
    let timeline = in_microseconds(&[(0, true), (3, false), (6, true), (9, false), (12, true), (15, false)]);
    for phase in phases() {
        let mut switch = switch(true);
        let mut stage = Stage::Boosting;
        let events = read_on_ticks(&mut switch, &mut stage, &timeline, Pin::Interrupt, phase, 500_000);
        assert_eq!(events.len(), 1, "phase {phase} µs");
        let (read_us, event) = events[0];
        assert!(
            (35_000..=45_000).contains(&read_us),
            "phase {phase} µs: at {read_us} µs"
        );
        assert_eq!((event.row(), stage), (["separated", "gliding"], Stage::Gliding));
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
                let mut switch = switch(true);
                let mut stage = Stage::Boosting;
                let events = read_on_ticks(&mut switch, &mut stage, timeline, pin, phase, 600_000);
                assert!(events.is_empty(), "{pin:?}, phase {phase} µs: {events:?}");
                assert_eq!((stage, switch.separated()), (Stage::Boosting, false));
            }
        }
    }
}

#[test]
fn an_edge_forced_on_every_read_never_settles() {
    // NEGATIVE, the edge contract pinned: a polled caller that passed edge=true on every read would restart every
    // run; held open from the first read, each settle runs out to its deadline, Unsettled, and the switch never
    // separates -- the reason a polled pin passes false
    let mut debounce = Debounce::new(20);
    let verdicts: Vec<(u32, Verdict)> = (0..=85)
        .filter_map(|tick| {
            debounce
                .step(tick * 10, false, true, true)
                .map(|verdict| (tick * 10, verdict))
        })
        .collect();
    assert_eq!(
        verdicts,
        [
            (200, Verdict::Unsettled),
            (410, Verdict::Unsettled),
            (620, Verdict::Unsettled),
            (830, Verdict::Unsettled)
        ]
    );
    let mut forced = switch(true);
    for tick in 0..100 {
        assert_eq!(
            forced.step(tick * 10, false, true, Stage::Boosting),
            None,
            "tick {tick}"
        );
    }
    assert!(!forced.separated());
}

#[test]
fn a_disagreeing_level_with_no_edge_settles() {
    // the blind window: the pads opened between setup and arming the interrupt, or the arm failed -- no edge ever
    let mut nested = switch(true);
    assert_eq!(nested.step(0, false, false, Stage::Boosting), None);
    assert!(nested.settling(), "the disagreeing level starts a settle");
    assert_eq!(nested.step(10, false, false, Stage::Boosting), None);
    let event = nested
        .step(20, false, false, Stage::Boosting)
        .expect("a held separation");
    assert_eq!((event.row(), event.advanced), (["separated", "gliding"], true));
    // and the other way: a latch set up separated, nested before arming
    let mut open = switch(false);
    let events: Vec<Option<Event>> = (0..3)
        .map(|tick| open.step(tick * 10, true, false, Stage::Setting))
        .collect();
    assert_eq!(events[..2], [None, None]);
    assert_eq!(events[2].map(|event| event.row()), Some(["nested", "setting"]));
}

#[test]
fn a_level_equal_to_the_latch_starts_nothing() {
    // NEGATIVE: with no edge flagged, a read of the latch's own level is no settle, whichever the latch
    for high in [true, false] {
        let mut switch = switch(high);
        for tick in 0..30 {
            assert_eq!(switch.step(tick * 10, high, false, Stage::Boosting), None);
            assert!(!switch.settling(), "latch high={high}, tick {tick}");
        }
    }
    // the bare debounce: neither an edge nor a disagreeing level, whatever the level read
    let mut debounce = Debounce::new(20);
    for tick in 0..10 {
        assert_eq!(debounce.step(tick * 10, tick % 2 == 0, false, false), None);
        assert!(!debounce.settling());
    }
}

#[test]
fn an_edge_between_two_agreeing_reads_gives_no_verdict_on_the_third() {
    // NEGATIVE: three reads of the open level spanning 20 ms, but the line moved between the second and the third
    let mut debounce = Debounce::new(20);
    assert_eq!(debounce.step(0, false, true, true), None);
    assert_eq!(debounce.step(10, false, false, true), None);
    assert_eq!(debounce.step(20, false, true, true), None, "the restart rule");
    assert!(debounce.settling());
    // the run restarted on that read: two more agreeing reads conclude it
    assert_eq!(debounce.step(30, false, false, true), None);
    assert_eq!(debounce.step(40, false, false, true), Some(Verdict::Separated));
}

#[test]
fn a_verdict_needs_three_reads_and_the_span() {
    let mut debounce = Debounce::new(20);
    assert_eq!(debounce.step(0, false, true, true), None); // the edge: one agreeing read
    assert_eq!(debounce.step(5, false, false, true), None); // two
    assert_eq!(debounce.step(10, false, false, true), None); // three, but 10 ms only
    assert_eq!(debounce.step(20, false, false, true), Some(Verdict::Separated));
    // NEGATIVE: the span alone is not enough either -- two reads 30 ms apart are two reads
    let mut sparse = Debounce::new(20);
    assert_eq!(sparse.step(0, true, true, true), None);
    assert_eq!(sparse.step(30, true, false, true), None);
    assert_eq!(sparse.step(40, true, false, true), Some(Verdict::Nested));
}

#[test]
fn chatter_runs_out_at_ten_windows_and_the_level_rule_takes_it_up() {
    let mut debounce = Debounce::new(20);
    let mut level = true;
    let mut verdicts = Vec::new();
    for tick in 0..=20 {
        level = !level;
        verdicts.extend(
            debounce
                .step(tick * 10, level, true, true)
                .map(|verdict| (tick, verdict)),
        );
    }
    assert_eq!(
        verdicts,
        [(20, Verdict::Unsettled)],
        "the deadline is 200 ms after the first read"
    );
    // NEGATIVE: the edges seen during it are not carried over: the latch's own level starts nothing
    assert_eq!(debounce.step(210, level, false, false), None);
    assert!(!debounce.settling());
    // a level that disagrees with the latch does
    assert_eq!(debounce.step(220, level, false, true), None);
    assert!(debounce.settling());
}

#[test]
fn seeded_chatter_never_deploys_and_a_held_open_after_it_does() {
    // D15's property: toggles of 0.5-2 ms for 50-200 ms. Ending nested, no debounce the 10 ms tick takes deploys.
    // Ending open, the switch deploys 20-30 ms after the last bounce, or up to two ticks more when the bounce ran a
    // settle out to its deadline in the ticks just before (that tick concludes nothing; the level rule starts the
    // next) -- and never during the bounce.
    let mut state = 20261007;
    let (mut late, mut runs) = (0, 0);
    for _ in 0..1000 {
        let phase = within(&mut state, 0, TICK_US - 1);
        let (nested, last) = bounce_then(&mut state, true);
        for debounce_ms in [2, 10, 20, 30] {
            let mut switch = Separation::new(&Settings { debounce_ms }, true);
            let mut stage = Stage::Boosting;
            let events = read_on_ticks(&mut switch, &mut stage, &nested, Pin::Interrupt, phase, last + 500_000);
            assert!(
                events.is_empty(),
                "debounce {debounce_ms} ms, phase {phase} µs: {events:?} on {nested:?}"
            );
            assert_eq!(stage, Stage::Boosting);
        }
        let (open, last) = bounce_then(&mut state, false);
        let mut switch = switch(true);
        let mut stage = Stage::Boosting;
        let events = read_on_ticks(&mut switch, &mut stage, &open, Pin::Interrupt, phase, last + 500_000);
        assert_eq!(events.len(), 1, "phase {phase} µs: {events:?} on {open:?}");
        let (read_us, event) = events[0];
        assert_eq!((event.row(), stage), (["separated", "gliding"], Stage::Gliding));
        let after_us = read_us - last;
        assert!(
            (20_000..=50_000).contains(&after_us),
            "{after_us} µs after the last bounce, phase {phase} µs"
        );
        late += usize::from(after_us > 30_000);
        runs += 1;
    }
    // the late case is real, and rare
    assert!(
        late > 0 && late < runs / 4,
        "{late} of {runs} deployed over 30 ms after the last bounce"
    );
}

#[test]
fn a_bounce_that_runs_out_its_settle_deploys_two_ticks_later() {
    // the late case pinned: toggles every 5 ms, open for good from 195 ms, read at 6, 16, ... ms. The read at 6 ms
    // starts the settle (deadline 206 ms); 196 restarts its run on the last bounce; 206 is the deadline and concludes
    // nothing; 216 starts the next settle on the level rule; 236 concludes it -- not 216, two ticks later
    let mut timeline: Vec<(u32, bool)> = vec![(0, true)];
    timeline.extend((1..=39).map(|bounce| (bounce * 5000, bounce % 2 == 0)));
    assert_eq!(timeline.last(), Some(&(195_000, false)), "open for good from 195 ms");
    let mut switch = switch(true);
    let mut stage = Stage::Boosting;
    let events = read_on_ticks(&mut switch, &mut stage, &timeline, Pin::Interrupt, 6000, 500_000);
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].0, 236_000, "41 ms after the last bounce");
    assert_eq!(stage, Stage::Gliding);
}

#[test]
fn a_re_nest_on_the_tick_never_reverses_the_stage() {
    // NEGATIVE: open for good from 5 ms, nested again from 300 ms -- recorded, and GLIDING stays
    let timeline = in_microseconds(&[(0, true), (5, false), (300, true)]);
    let mut switch = switch(true);
    let mut stage = Stage::Boosting;
    let events = read_on_ticks(&mut switch, &mut stage, &timeline, Pin::Interrupt, 0, 600_000);
    let rows: Vec<[&str; 2]> = events.iter().map(|(_, event)| event.row()).collect();
    assert_eq!(rows, [["separated", "gliding"], ["nested", "gliding"]]);
    assert!(!events[1].1.advanced);
    assert_eq!((stage, switch.separated()), (Stage::Gliding, false));
}

#[test]
fn outside_boosting_the_latch_moves_but_the_stage_does_not() {
    for stage in [Stage::Setting, Stage::Gliding, Stage::Landing, Stage::Done] {
        let mut switch = switch(true);
        let event = switch.apply(true, stage).expect("a change of the latch");
        assert_eq!((event.stage, event.advanced), (stage, false), "{stage:?}");
    }
}

#[test]
fn a_re_nest_is_recorded_and_never_reverses() {
    let mut switch = switch(true);
    assert_eq!(switch.apply(true, Stage::Boosting).unwrap().stage, Stage::Gliding);
    assert_eq!(
        switch.apply(true, Stage::Gliding),
        None,
        "the same level records nothing"
    );
    let nested = switch.apply(false, Stage::Gliding).unwrap();
    assert_eq!(
        (nested.name(), nested.stage, nested.advanced),
        ("nested", Stage::Gliding, false)
    );
}

#[test]
fn a_debounce_of_0_never_settles() {
    // plan B's arithmetic, kept: the deadline (0 x 10 ms) passes before three reads can agree
    let mut debounce = Debounce::new(0);
    assert_eq!(debounce.step(0, false, true, true), None);
    assert_eq!(debounce.step(10, false, false, true), Some(Verdict::Unsettled));
}

#[test]
fn the_clock_may_wrap() {
    let mut debounce = Debounce::new(20);
    let start = u32::MAX - 15;
    assert_eq!(debounce.step(start, false, true, true), None);
    assert_eq!(debounce.step(start.wrapping_add(10), false, false, true), None);
    assert_eq!(
        debounce.step(start.wrapping_add(20), false, false, true),
        Some(Verdict::Separated)
    );
}

#[test]
fn settings_and_the_setup_read() {
    assert_eq!(Settings::from_config(&json!({})), Settings { debounce_ms: 20 });
    assert_eq!(Settings::for_tick(&json!({}), 10), Ok(Settings { debounce_ms: 20 }));
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 35}), 10),
        Ok(Settings { debounce_ms: 35 })
    );
    // NEGATIVE: a negative or non-integer debounce takes the default rather than wrapping into a huge one
    assert_eq!(Settings::from_config(&json!({"debounce_ms": -1})).debounce_ms, 20);
    assert_eq!(Settings::from_config(&json!({"debounce_ms": "5"})).debounce_ms, 20);
    // from_config checks no bound; for_tick does
    assert_eq!(Settings::from_config(&json!({"debounce_ms": 0})).debounce_ms, 0);
    // LOW at setup (pads open) starts the latch separated; HIGH nested
    assert!(switch(false).separated());
    assert!(!switch(true).separated());
}

#[test]
fn a_debounce_that_cannot_hold_three_tick_reads_is_refused() {
    // NEGATIVE: at the 10 ms tick, 0 and 1 ms windows (0 and 10 ms settles) cannot hold three nominal reads
    for debounce_ms in [0, 1] {
        assert_eq!(
            Settings::for_tick(&json!({ "debounce_ms": debounce_ms }), 10),
            Err(format!("debounce_ms {debounce_ms} cannot settle on the 10 ms tick"))
        );
    }
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 0}), 0),
        Err("debounce_ms 0 cannot settle on the 0 ms tick".to_string()),
        "0 never settles, on any tick"
    );
    // 2 ms is the floor at 10 ms; plan B's 5 ms reads settled 1 ms
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 2}), 10),
        Ok(Settings { debounce_ms: 2 })
    );
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 1}), 5),
        Ok(Settings { debounce_ms: 1 })
    );
    // held open on EXACT 10 ms ticks, 2 deploys and 1 never does
    let timeline = in_microseconds(&[(0, true), (5, false)]);
    for (debounce_ms, deploys) in [(2, true), (1, false)] {
        let mut switch = Separation::new(&Settings { debounce_ms }, true);
        let mut stage = Stage::Boosting;
        let events = read_on_ticks(&mut switch, &mut stage, &timeline, Pin::Interrupt, 0, 500_000);
        assert_eq!(!events.is_empty(), deploys, "debounce {debounce_ms} ms");
    }
}

#[test]
fn the_floor_is_conservative() {
    // a settle concludes once its second read comes before the deadline: some refused values settle by luck. A ms
    // stamp of the µs clock puts two 10 ms reads 9 ms apart, and 1 ms (deadline 11) then settles on the third read
    let mut jittered = Debounce::new(1);
    assert_eq!(jittered.step(1, false, true, true), None);
    assert_eq!(jittered.step(10, false, false, true), None);
    assert_eq!(jittered.step(20, false, false, true), Some(Verdict::Separated));
    // and on exact 20 ms ticks, 3 ms (window 30 < span 40) settles on the read at 40, yet is refused
    let mut slow = Debounce::new(3);
    assert_eq!(slow.step(0, false, true, true), None);
    assert_eq!(slow.step(20, false, false, true), None);
    assert_eq!(slow.step(40, false, false, true), Some(Verdict::Separated));
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 3}), 20),
        Err("debounce_ms 3 cannot settle on the 20 ms tick".to_string())
    );
}

#[test]
fn a_debounce_too_long_for_the_ms_clock_is_refused() {
    // NEGATIVE: a window past i32::MAX ms can read its deadline as passed on the second read: 300 000 000 ms (a
    // window of 3e9 ms) ends every settle Unsettled, and the switch would never separate
    let mut absurd = Debounce::new(300_000_000);
    assert_eq!(absurd.step(0, false, true, true), None);
    assert_eq!(absurd.step(10, false, false, true), Some(Verdict::Unsettled));
    for debounce_ms in [214_748_365_u32, 300_000_000, u32::MAX] {
        assert_eq!(
            Settings::for_tick(&json!({ "debounce_ms": debounce_ms }), 10),
            Err(format!("debounce_ms {debounce_ms} is too long for the ms clock"))
        );
    }
    // the longest taken: its window, 2 147 483 640 ms, is within i32::MAX
    assert_eq!(
        Settings::for_tick(&json!({"debounce_ms": 214_748_364}), 10),
        Ok(Settings {
            debounce_ms: 214_748_364
        })
    );
}
