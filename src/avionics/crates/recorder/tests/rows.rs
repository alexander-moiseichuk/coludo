//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A run's rows over a real Recorder (recorder::Rows): accepted rows pass quietly, the first refusal of a run is the
//! one reported, the rest stay quiet, and a stream the board does not record takes nothing and reports nothing.

mod common;

use common::{SETTINGS, recorder};
use recorder::{Cell, Registry, Rows, Telemetry};

#[test]
fn the_first_refusal_of_a_run_is_the_one_reported() {
    let mut registry = Registry::new();
    let stream = registry.add("probe.csv", &["value"], -1, false).unwrap();
    // one small ring and no writer draining it: it fills, then every row is refused
    let parts = recorder(registry, 1, 256, SETTINGS);
    let mut rows = Rows::new("probe", "telemetry", Telemetry::new(parts.recorder, stream));
    assert!(rows.recording());
    let mut reported = 0;
    let mut accepted = 0;
    for index in 0..200 {
        if rows.push(1_000 + index, &[Cell::I32(index as i32)]) {
            reported += 1;
        } else if reported == 0 {
            accepted += 1;
        }
    }
    assert!(accepted > 0, "the ring took rows before it filled");
    assert_eq!(reported, 1, "one report for the whole run of refusals");
}

#[test]
fn a_stream_that_does_not_record_takes_nothing() {
    // NEGATIVE: no telemetry: every push is a quiet no-op
    let mut rows = Rows::new("probe", "telemetry", None);
    assert!(!rows.recording());
    assert!((0..10).all(|index| !rows.push(index, &[Cell::I32(1)])));
}
