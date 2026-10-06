//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Telemetry decimates as plan B does: src/avionics/golden/recorder.json, which golden/build_recorder.py builds from
//! src/glider/recorder.py's `Telemetry.due` and `push` on CPython. For each push, whether a row was due before it
//! and whether it emitted one; the rows are read back from the flash the writer filled, so "emitted" means on
//! flash. Every value compares exactly.

mod common;

use common::*;

use flash_ring::Geometry;
use recorder::{CUT_AGE_US, Cell, MERGE_HORIZON_US, Registry, Settings, Telemetry, Writer};
use serde_json::{Value, json};

/// The fixture, compiled in.
const FIXTURE: &str = include_str!("../../../golden/recorder.json");

/// One case: the stream's own setting, the global window, and the push uptimes; plan B's answer per push.
fn decimation(input: &Value) -> Value {
    let decimate_us = i32::try_from(input["decimate_us"].as_i64().unwrap()).unwrap();
    let telemetry_ms = u32::try_from(input["telemetry_ms"].as_u64().unwrap()).unwrap();
    let pushes: Vec<u64> = input["pushes_us"]
        .as_array()
        .unwrap()
        .iter()
        .map(|push| push.as_u64().unwrap())
        .collect();
    let mut registry = Registry::new(SESSION).unwrap();
    let stream = registry.add("imu.csv", &["index"], decimate_us, false).unwrap();
    let settings = Settings {
        telemetry_ms,
        stats_ms: 1000,
    };
    let (recorder, clock) = recorder(registry, 1, 1 << 16, settings);
    let mut writer = Writer::new(recorder, ring(recorder, Geometry::new(8, 16).unwrap(), OPTIONS)).unwrap();
    let _attached = recorder.attach(0).unwrap();
    let mut telemetry = Telemetry::new(recorder, stream).unwrap();
    let mut due = Vec::new();
    for (index, &now) in pushes.iter().enumerate() {
        due.push(telemetry.due(now));
        telemetry.push(now, &[Cell::I32(index as i32)]).unwrap();
    }
    let end = *pushes.last().unwrap();
    for now in [end, end + MERGE_HORIZON_US, end + MERGE_HORIZON_US + CUT_AGE_US] {
        clock.set(now);
        writer.step(now, EPOCH);
    }
    let rows = read_back(writer.ring())
        .remove(&Some(format!("{SESSION}_imu.csv")))
        .unwrap();
    assert_eq!(rows[0], "uptime;index", "the header first");
    let emitted: Vec<usize> = rows[1..]
        .iter()
        .map(|row| row.split(';').nth(1).unwrap().parse().unwrap())
        .collect();
    let answers = due
        .iter()
        .enumerate()
        .map(|(index, &due)| json!({"due": due, "emitted": emitted.contains(&index)}))
        .collect();
    Value::Array(answers)
}

#[test]
fn telemetry_decimates_as_plan_b_does() {
    golden::run(FIXTURE, decimation);
}

#[test]
fn the_fixture_covers_every_setting() {
    let document: Value = serde_json::from_str(FIXTURE).unwrap();
    let settings: Vec<i64> = document["cases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|case| case["input"]["decimate_us"].as_i64().unwrap().signum())
        .collect();
    for setting in [-1, 0, 1] {
        assert!(
            settings.contains(&setting),
            "no case with a decimate_us of sign {setting}"
        );
    }
    // negative: the comparison does fail a wrong answer
    let wrong = std::panic::catch_unwind(|| {
        golden::run(FIXTURE, |input| {
            let mut answer = decimation(input);
            answer[0]["emitted"] = json!(false);
            answer
        })
    });
    assert!(wrong.is_err(), "a first push that emits nothing must fail the fixture");
}
