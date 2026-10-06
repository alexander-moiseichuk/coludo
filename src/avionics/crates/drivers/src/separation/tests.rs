//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the separation debounce on the Rust cadence (one read per 10 ms control tick), beyond its golden
//! fixture (tests/golden_separation.rs), which replays plan B's 5 ms reads.

use super::{Debounce, Event, Separation, Settings, Verdict};
use sequencer::Stage;
use serde_json::json;

/// Step a switch through 10 ms ticks over `levels` (true = HIGH = nested), the edge flagged on each change of
/// level; returns the events with the tick they came on.
fn ticks(switch: &mut Separation, stage: &mut Stage, start_high: bool, levels: &[bool]) -> Vec<(usize, Event)> {
    let mut previous = start_high;
    let mut events = Vec::new();
    for (tick, high) in levels.iter().copied().enumerate() {
        let edge = high != previous;
        previous = high;
        if let Some(event) = switch.step(tick as u32 * 10, high, edge, *stage) {
            *stage = event.stage;
            events.push((tick, event));
        }
    }
    events
}

#[test]
fn a_held_separation_in_boosting_glides() {
    let mut switch = Separation::new(&Settings { debounce_ms: 20 }, true);
    let mut stage = Stage::Boosting;
    let levels = [true, false, false, false, false];
    let events = ticks(&mut switch, &mut stage, true, &levels);
    // the edge's read, then 10 and 20 ms on: three agreeing reads spanning 20 ms
    assert_eq!(events.len(), 1);
    let (tick, event) = events[0];
    assert_eq!(tick, 3);
    assert_eq!(event.row(), ["separated", "gliding"]);
    assert!(event.advanced && switch.separated());
    assert_eq!(stage, Stage::Gliding);
}

#[test]
fn a_one_tick_blip_never_deploys() {
    // NEGATIVE: the 7 ms blip that once forced GLIDING in boost, seen by one tick
    let mut switch = Separation::new(&Settings { debounce_ms: 20 }, true);
    let mut stage = Stage::Boosting;
    let levels = [true, false, true, true, true, true, true];
    assert!(ticks(&mut switch, &mut stage, true, &levels).is_empty());
    assert_eq!((stage, switch.separated()), (Stage::Boosting, false));
}

#[test]
fn outside_boosting_the_latch_moves_but_the_stage_does_not() {
    for stage in [Stage::Setting, Stage::Gliding, Stage::Landing, Stage::Done] {
        let mut switch = Separation::new(&Settings { debounce_ms: 20 }, true);
        let event = switch.apply(true, stage).expect("a change of the latch");
        assert_eq!((event.stage, event.advanced), (stage, false), "{stage:?}");
    }
}

#[test]
fn a_re_nest_is_recorded_and_never_reverses() {
    let mut switch = Separation::new(&Settings { debounce_ms: 20 }, true);
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
fn an_idle_line_reads_nothing_into_a_settle() {
    // NEGATIVE: with no edge flagged, the control tick's reads start nothing, whatever the level
    let mut debounce = Debounce::new(20);
    for tick in 0..10 {
        assert_eq!(debounce.step(tick * 10, tick % 2 == 0, false), None);
        assert!(!debounce.settling());
    }
}

#[test]
fn a_verdict_needs_three_reads_and_the_span() {
    let mut debounce = Debounce::new(20);
    assert_eq!(debounce.step(0, false, true), None); // the edge: one agreeing read
    assert_eq!(debounce.step(5, false, false), None); // two
    assert_eq!(debounce.step(10, false, false), None); // three, but 10 ms only
    assert_eq!(debounce.step(20, false, false), Some(Verdict::Separated));
    // NEGATIVE: the span alone is not enough either -- two reads 30 ms apart are two reads
    let mut sparse = Debounce::new(20);
    assert_eq!(sparse.step(0, true, true), None);
    assert_eq!(sparse.step(30, true, false), None);
    assert_eq!(sparse.step(40, true, false), Some(Verdict::Nested));
}

#[test]
fn chatter_runs_out_at_ten_windows_and_its_own_edges_settle_again() {
    let mut debounce = Debounce::new(20);
    let mut level = true;
    let mut verdicts = Vec::new();
    for tick in 0..=20 {
        level = !level;
        verdicts.extend(debounce.step(tick * 10, level, true).map(|verdict| (tick, verdict)));
    }
    assert_eq!(
        verdicts,
        [(20, Verdict::Unsettled)],
        "the deadline is 200 ms after the first read"
    );
    // the edges flagged during it start the next settle on the next read
    assert_eq!(debounce.step(210, level, false), None);
    assert!(debounce.settling());
}

#[test]
fn a_debounce_of_0_never_settles() {
    // plan B's arithmetic, kept: the deadline (0 x 10 ms) passes before three reads can agree
    let mut debounce = Debounce::new(0);
    assert_eq!(debounce.step(0, false, true), None);
    assert_eq!(debounce.step(10, false, false), Some(Verdict::Unsettled));
}

#[test]
fn the_clock_may_wrap() {
    let mut debounce = Debounce::new(20);
    let start = u32::MAX - 15;
    assert_eq!(debounce.step(start, false, true), None);
    assert_eq!(debounce.step(start.wrapping_add(10), false, false), None);
    assert_eq!(
        debounce.step(start.wrapping_add(20), false, false),
        Some(Verdict::Separated)
    );
}

#[test]
fn settings_and_the_setup_read() {
    assert_eq!(Settings::from_config(&json!({})).debounce_ms, 20);
    assert_eq!(Settings::from_config(&json!({"debounce_ms": 35})).debounce_ms, 35);
    // NEGATIVE: a negative debounce takes the default rather than wrapping into a huge one
    assert_eq!(Settings::from_config(&json!({"debounce_ms": -1})).debounce_ms, 20);
    // LOW at setup (pads open) starts the latch separated; HIGH nested
    assert!(Separation::new(&Settings { debounce_ms: 20 }, false).separated());
    assert!(!Separation::new(&Settings { debounce_ms: 20 }, true).separated());
}
