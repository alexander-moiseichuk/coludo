//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMI323 answers as plan B's does: src/avionics/golden/drivers_bmi323.json, which golden/build_drivers_bmi323.py
//! builds from src/glider/drivers/bmi323.py on CPython. The burst decode past the dummy bytes (exact), the gyro in
//! degrees per second (within plan B's 0.01 floor), then plan B's I2C transcripts replayed through the mock bus --
//! setup with its start-up wait, the re-arm (two cases owe the fixture's `divergence`: the INT1 route restored, and a
//! gyro that never restarts reported), the probe and the diagnosis -- and the run loop's INT1 verdict per wake.

mod transcript;

use drivers::bmi323::Bmi323;
use drivers::device::{Device, Fault};
use golden::Compare;
use serde_json::{Value, json};
use transcript::{check_cases, replay, section};

/// plan B's BMI323 driver over the decode, rate, setup, rearm, wake, probe and diagnose sections.
const BMI323: &str = include_str!("../../../golden/drivers_bmi323.json");

/// The default address plan B's transcripts were taken at.
const ADDRESS: u8 = 0x69;

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(BMI323, "drivers_bmi323", name)
}

/// Whether a case's INT1 is wired.
fn wired(input: &Value) -> bool {
    input["interrupt_wired"].as_bool().expect("interrupt_wired")
}

#[test]
fn the_burst_decodes_past_the_dummy_bytes() {
    let count = check_cases(&part("decode"), "decode", |input| {
        let mock = replay(&input["script"]);
        let sample = Bmi323::new(ADDRESS, false)
            .read(&mut mock.clone())
            .expect("a scripted burst");
        mock.done();
        json!({"accel_g": sample.accel_g().map(f64::from), "rate_cd_s": sample.rate_centidegrees_per_second()})
    });
    assert!(count > 40, "{count} decode cases");
}

#[test]
fn the_rate_is_within_plan_b_s_floor() {
    check_cases(&part("rate"), "rate", |input| {
        let mock = replay(&input["script"]);
        let sample = Bmi323::new(ADDRESS, false)
            .read(&mut mock.clone())
            .expect("a scripted burst");
        mock.done();
        json!(sample.rate_degrees_per_second().map(f64::from))
    });
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmi323::new(ADDRESS, wired(input)).setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
}

#[test]
fn rearm_restores_what_the_general_call_cleared() {
    let section = part("rearm");
    let compare = Compare::from_json(&section["compare"]);
    let mut divergences = 0;
    for case in section["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let input = &case["input"];
        let mut imu = Bmi323::new(ADDRESS, wired(input));
        let setup = replay(&input["setup"]);
        imu.setup(&mut setup.clone(), &mut setup.delay())
            .expect("the setup before the re-arm");
        setup.done();
        let (owed_script, owed) = match case.get("divergence") {
            Some(divergence) => {
                divergences += 1;
                (&divergence["script"], json!({"ok": divergence["ok"]}))
            }
            None => (&input["script"], case["output"].clone()),
        };
        let mock = replay(owed_script);
        let outcome = imu.rearm(&mut mock.clone(), &mut mock.delay());
        mock.done();
        compare
            .check(name, &owed, &json!({"ok": outcome.is_ok()}))
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert_eq!(
        divergences, 2,
        "the INT1 route and the start-up verdict are the two divergences"
    );
}

#[test]
fn the_int1_verdict_follows_plan_b_s_run_loop() {
    check_cases(&part("wake"), "wake", |input| {
        let mut imu = Bmi323::new(ADDRESS, wired(input));
        let verdicts: Vec<Value> = input["irq_runs"]
            .as_array()
            .expect("irq_runs")
            .iter()
            .map(|runs| {
                let noted = imu.wake(u32::try_from(runs.as_u64().expect("a count")).expect("a small count"));
                json!({"noted": noted, "silent": imu.interrupt_silent()})
            })
            .collect();
        json!(verdicts)
    });
}

#[test]
fn the_probe_fails_at_plan_b_s_step() {
    check_cases(&part("probe"), "probe", |input| {
        let mock = replay(&input["script"]);
        let outcome = Bmi323::new(ADDRESS, false).probe(&mut mock.clone());
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
fn diagnose_names_the_fault_in_plan_b_s_words() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let verdict = Bmi323::new(ADDRESS, false).diagnose(&mut mock.clone(), 0);
        mock.done();
        // the error's repr after ' -- ' is the runtime's own, on both sides
        json!(verdict.split(" -- ").next().unwrap())
    });
}
