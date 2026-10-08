//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581 answers as plan B's does: src/avionics/golden/drivers_bmp581.json, which golden/build_drivers_bmp581.py
//! builds from src/glider/drivers/bmp581.py on CPython. The burst conversion member by member (temperature and
//! pressure EXACT, power-of-two scalings in both precisions; altitude within 5 mm, the f32 of commons.altitude_m),
//! plan B's I2C transcripts replayed through the mock bus -- setup with its ground zero, the re-arm out of standby,
//! the probe and the diagnosis -- and the operator's ground updates (one divergence: a word for a ground is refused
//! in the port's words, where plan B's were the interpreter's). Setup cases that read ground samples owe their
//! `divergence`: each burst read twice by the garbage guard, plan B's answer and ground unchanged.

mod transcript;

use drivers::bmp581::Bmp581;
use drivers::device::{Device, Fault};
use golden::Compare;
use serde_json::{Value, json};
use transcript::{FieldCompare, check_cases, replay, section};

/// plan B's BMP581 driver over the read, setup, rearm, update, probe and diagnose sections.
const BMP581: &str = include_str!("../../../golden/drivers_bmp581.json");

/// The default address plan B's transcripts were taken at.
const ADDRESS: u8 = 0x47;

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(BMP581, "drivers_bmp581", name)
}

#[test]
fn the_burst_converts_as_plan_b() {
    let count = check_cases(&part("read"), "read", |input| {
        let mock = replay(&input["script"]);
        let reading = Bmp581::new(ADDRESS).read(&mut mock.clone()).expect("a scripted burst");
        mock.done();
        json!({"altitude": f64::from(reading.altitude), "temperature": f64::from(reading.temperature),
               "pressure": f64::from(reading.pressure)})
    });
    assert!(count > 40, "{count} read cases");
}

#[test]
fn setup_makes_plan_b_s_transactions_and_ground_zero() {
    // a case that reads a ground sample owes its divergence's script (each burst read twice, the garbage guard), and
    // plan B's answer and ground all the same
    let section = part("setup");
    let compare = FieldCompare::from_section(&section);
    let mut divergences = 0;
    for case in section["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let script = match case.get("divergence") {
            Some(divergence) => {
                divergences += 1;
                &divergence["script"]
            }
            None => &case["input"]["script"],
        };
        let mock = replay(script);
        let mut barometer = Bmp581::new(ADDRESS);
        let ok = barometer.setup(&mut mock.clone(), &mut mock.delay()).is_ok();
        mock.done();
        let answer = json!({"ok": ok, "ground": ok.then(|| f64::from(barometer.ground()))});
        compare
            .check(name, &case["output"], &answer)
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert_eq!(divergences, 2, "the healthy and the drifting pad read ground samples");
}

#[test]
fn rearm_writes_the_mode_again() {
    check_cases(&part("rearm"), "rearm", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmp581::new(ADDRESS).rearm(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
}

#[test]
fn a_reading_is_checked_member_by_member() {
    // NEGATIVE: the XLSB a dropped low byte loses (1/65536 degrees C, 1/64 Pa) fails exactly, and an altitude 6 mm
    // off fails its own 5 mm, where one abs 0.01 for all three passed both
    let section = part("read");
    let compare = FieldCompare::from_section(&section);
    let expected = &section["cases"][0]["output"];
    let nudged = |member: &str, by: f64| {
        let mut answer = expected.clone();
        answer[member] = json!(expected[member].as_f64().unwrap() + by);
        compare.check("nudged", expected, &answer)
    };
    assert!(nudged("temperature", 1.0 / 65_536.0).is_err());
    assert!(nudged("pressure", 1.0 / 64.0).is_err());
    assert!(nudged("altitude", 0.006).is_err());
    assert!(nudged("altitude", 0.004).is_ok());
}

#[test]
fn the_ground_updates_as_plan_b() {
    let section = part("update");
    let compare = Compare::from_json(&section["compare"]);
    let mut divergences = 0;
    for case in section["cases"].as_array().unwrap() {
        let input = &case["input"];
        let mut barometer = Bmp581::new(ADDRESS);
        barometer
            .update(&json!({"ground": 7.0}), None)
            .expect("the starting ground");
        let fresh = input["fresh_altitude"].as_f64().map(|altitude| altitude as f32);
        let answer = match barometer.update(&input["properties"], fresh) {
            Ok(changed) => json!({"changed": changed, "ground": f64::from(barometer.ground())}),
            Err(refusal) => json!({"refused": refusal.to_string(), "ground": f64::from(barometer.ground())}),
        };
        let owed = match case.get("divergence") {
            Some(divergence) => {
                divergences += 1;
                &divergence["output"]
            }
            None => &case["output"],
        };
        compare
            .check(case["name"].as_str().unwrap(), owed, &answer)
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert_eq!(
        divergences, 1,
        "a word for a ground, refused in the port's words, is the one divergence"
    );
}

#[test]
fn the_probe_fails_at_plan_b_s_step() {
    check_cases(&part("probe"), "probe", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmp581::new(ADDRESS).probe(&mut mock.clone());
        mock.done();
        let failed = match outcome {
            Ok(_) => Value::Null,
            Err(Fault::Identity { .. }) => json!("chip id"),
            Err(Fault::Bus(_)) if mock.performed() == 1 => json!("chip id"),
            Err(Fault::Bus(_)) => json!("read"),
            Err(other) => panic!("unexpected fault {other:?}"),
        };
        json!({"failed": failed})
    });
}

#[test]
fn diagnose_names_the_wire_fault() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let verdict = Bmp581::new(ADDRESS).diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}
