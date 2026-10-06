//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMM350 answers as plan B's does: src/avionics/golden/drivers_bmm350.json, which golden/build_drivers_bmm350.py
//! builds from src/glider/drivers/bmm350.py on CPython. Everything compares EXACTLY (counts, and plan B's integer
//! calibration): the burst decode, the operator calibration scenario by scenario -- per sample the published axes and
//! the octants covered, then the verdict, the calibration and the corrected axes -- the NVS blob both ways, and plan
//! B's I2C transcripts (setup with the OTP power-off, the re-arm, the probe, the diagnosis) through the mock bus.

mod transcript;

use drivers::bmm350::{Bmm350, Calibration, Field, ProbeFailure};
use drivers::device::{Device, Fault};
use serde_json::{Value, json};
use transcript::{bytes, check_cases, replay, section};

/// plan B's BMM350 driver over the decode, calibration, nvs, setup, rearm, probe and diagnose sections.
const BMM350: &str = include_str!("../../../golden/drivers_bmm350.json");

/// The default address plan B's transcripts were taken at.
const ADDRESS: u8 = 0x15;

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(BMM350, "drivers_bmm350", name)
}

/// An i32 from a fixture integer.
fn integer(value: &Value) -> i32 {
    i32::try_from(value.as_i64().expect("an integer")).expect("an i32")
}

/// A raw field from a fixture `[x, y, z]`.
fn field(sample: &Value) -> Field {
    Field {
        x: integer(&sample[0]),
        y: integer(&sample[1]),
        z: integer(&sample[2]),
    }
}

/// A calibration from a fixture `[centre x, centre y, radius x, radius y]`.
fn calibration(values: &Value) -> Calibration {
    Calibration {
        centre_x: integer(&values[0]),
        centre_y: integer(&values[1]),
        radius_x: integer(&values[2]),
        radius_y: integer(&values[3]),
    }
}

/// A calibration as the fixture writes it.
fn listed(calibration: Option<Calibration>) -> Value {
    calibration.map_or(Value::Null, |calibration| {
        json!([
            calibration.centre_x,
            calibration.centre_y,
            calibration.radius_x,
            calibration.radius_y
        ])
    })
}

#[test]
fn the_burst_decodes_to_raw_counts() {
    let count = check_cases(&part("decode"), "decode", |input| {
        let mock = replay(&input["script"]);
        let mut magnetometer = Bmm350::new(ADDRESS);
        let raw = magnetometer.read(&mut mock.clone()).expect("a scripted burst");
        mock.done();
        json!(magnetometer.observe(&raw))
    });
    assert!(count > 40, "{count} decode cases");
}

#[test]
fn the_calibration_follows_plan_b_sample_by_sample() {
    let count = check_cases(&part("calibration"), "calibration", |input| {
        let mut magnetometer = Bmm350::new(ADDRESS);
        if !input["restored"].is_null() {
            magnetometer.set_calibration(calibration(&input["restored"]));
        }
        let mut published = Vec::new();
        let mut covered = Vec::new();
        for sample in input["samples"].as_array().expect("samples") {
            published.push(magnetometer.observe(&field(sample)));
            covered.push(magnetometer.covered());
        }
        let before = magnetometer.instruction();
        let answer = match magnetometer.calibrate() {
            Ok(captured) => {
                // plan B persists (no NVS here) and then puts it in force; the port leaves the order to the firmware
                magnetometer.set_calibration(captured);
                Value::Null
            }
            Err(refusal) => json!(refusal),
        };
        let after = magnetometer.instruction();
        let corrected: Vec<[i32; 3]> = input["then"]
            .as_array()
            .expect("then")
            .iter()
            .map(|sample| magnetometer.observe(&field(sample)))
            .collect();
        let span = magnetometer
            .span()
            .map(|span| json!([span.x_minimum, span.x_maximum, span.y_minimum, span.y_maximum]));
        json!({"published": published, "covered": covered, "seen": magnetometer.seen(), "span": span,
               "instruction_before": before, "answer": answer, "calibration": listed(magnetometer.calibration()),
               "instruction_after": after, "corrected": corrected})
    });
    assert!(count >= 20, "{count} calibration scenarios");
}

#[test]
fn the_nvs_blob_is_plan_b_s() {
    check_cases(&part("nvs"), "nvs", |input| match input.get("calibration") {
        Some(values) => json!({"blob": golden::hex(&calibration(values).to_bytes())}),
        None => {
            let blob = bytes(input["blob"].as_str().expect("a blob"));
            json!({"calibration": listed(Calibration::from_bytes(&blob))})
        }
    });
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmm350::new(ADDRESS).setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
}

#[test]
fn rearm_runs_the_table_again() {
    check_cases(&part("rearm"), "rearm", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmm350::new(ADDRESS).rearm(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
}

#[test]
fn the_probe_refuses_a_part_that_is_not_measuring() {
    check_cases(&part("probe"), "probe", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmm350::new(ADDRESS).probe(&mut mock.clone());
        mock.done();
        let (failed, reason) = match outcome {
            Ok(_) => (Value::Null, Value::Null),
            Err(ProbeFailure::NotMeasuring(reason)) => (json!("read"), json!(reason)),
            Err(ProbeFailure::Fault(Fault::Identity { .. })) => (json!("chip id"), Value::Null),
            Err(ProbeFailure::Fault(Fault::Bus(_))) if mock.performed() == 1 => (json!("chip id"), Value::Null),
            Err(ProbeFailure::Fault(Fault::Bus(_))) => (json!("read"), Value::Null),
            Err(other) => panic!("unexpected failure {other:?}"),
        };
        json!({"failed": failed, "reason": reason})
    });
}

#[test]
fn diagnose_names_the_fault_in_plan_b_s_words() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let verdict = Bmm350::new(ADDRESS).diagnose(&mut mock.clone(), 0);
        mock.done();
        json!(verdict.split(" -- ").next().unwrap())
    });
}
