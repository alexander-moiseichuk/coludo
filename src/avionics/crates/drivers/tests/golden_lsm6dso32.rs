//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The LSM6DSO32 answers as plan B's does: src/avionics/golden/drivers_lsm6dso32.json, which
//! golden/build_drivers_lsm6dso32.py builds from src/glider/drivers/lsm6dso32.py on CPython. The burst decode and
//! its scaling (rel 1e-6 for the f32 conversion, integers exact), then plan B's I2C transcripts replayed through the
//! mock bus -- the write-then-verify setup, the probe, the diagnosis -- and the run loop's INT1 verdict per wake,
//! with INT1 wired and without (plan B counts both on this part, and the port keeps it). The decode frames are
//! replayed once more through the port's status burst (plan B has none: it read the 12 bytes alone), prefixed with
//! a status and the bytes the burst skips, to prove the burst decodes them bit for bit as the bare read does.

mod transcript;

use drivers::device::{Device, Fault};
use drivers::lsm6dso32::{Fresh, Lsm6dso32, OverI2c, STATUS_REG};
use serde_json::{Value, json};
use transcript::{check_cases, replay, section};

/// plan B's LSM6DSO32 driver over the decode, setup, probe, diagnose and wake sections.
const LSM6DSO32: &str = include_str!("../../../golden/drivers_lsm6dso32.json");

/// The address plan B's i2c path used.
const ADDRESS: u8 = 0x6A;

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(LSM6DSO32, "drivers_lsm6dso32", name)
}

/// A driver on the I2C window plan B's transcripts were taken through.
fn driver(interrupt_wired: bool) -> Lsm6dso32<OverI2c> {
    Lsm6dso32::new(OverI2c { address: ADDRESS }, interrupt_wired)
}

/// Three f32 values as the fixture writes floats.
fn floats(values: [f32; 3]) -> Value {
    json!(values.map(f64::from))
}

#[test]
fn the_burst_decodes_and_scales_as_plan_b() {
    let count = check_cases(&part("decode"), "decode", |input| {
        let mock = replay(&input["script"]);
        let sample = driver(false).read(&mut mock.clone()).expect("a scripted burst");
        mock.done();
        json!({"accel_g": floats(sample.accel_g()), "rate_cd_s": sample.rate_centidegrees_per_second(),
               "rate_dps": floats(sample.rate_degrees_per_second())})
    });
    assert!(count > 40, "{count} decode cases");
}

#[test]
fn the_status_burst_decodes_plan_b_s_frames_bit_for_bit() {
    // each frame as the 16-byte burst from STATUS_REG: status 0x03 (XLDA and GDA), the reserved 0x1F and OUT_TEMP
    // (not zero, so a decode that took them for output would show), then plan B's 12 bytes
    let count = check_cases(&part("decode"), "decode", |input| {
        let steps = input["script"].as_array().expect("a transcript");
        assert_eq!(steps.len(), 1, "a decode case is one burst: {input}");
        let burst = &steps[0];
        assert_eq!(
            burst["write"],
            json!("22"),
            "the bare burst starts at OUTX_L_G: {burst}"
        );
        let mut status_burst = burst.clone();
        status_burst["write"] = json!(format!("{STATUS_REG:02x}"));
        status_burst["read"] = json!(format!(
            "03a55ac3{}",
            burst["read"].as_str().expect("the frame's bytes")
        ));
        let bare = replay(&input["script"]);
        let expected = driver(true).read(&mut bare.clone()).expect("a scripted burst");
        bare.done();
        let mock = replay(&json!([status_burst]));
        let (fresh, sample) = driver(true)
            .read_fresh(&mut mock.clone())
            .expect("a scripted status burst");
        mock.done();
        assert_eq!(
            fresh,
            Fresh {
                accel: true,
                rate: true
            }
        );
        assert_eq!(
            sample, expected,
            "the status burst's sample is the bare burst's, bit for bit: {burst}"
        );
        json!({"accel_g": floats(sample.accel_g()), "rate_cd_s": sample.rate_centidegrees_per_second(),
               "rate_dps": floats(sample.rate_degrees_per_second())})
    });
    assert!(count > 40, "{count} decode cases");
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    let count = check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let outcome = driver(input["interrupt_wired"].as_bool().unwrap()).setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok()})
    });
    assert!(count >= 10);
}

#[test]
fn the_probe_fails_at_plan_b_s_step() {
    check_cases(&part("probe"), "probe", |input| {
        let mock = replay(&input["script"]);
        let outcome = driver(false).probe(&mut mock.clone());
        mock.done();
        let failed = match outcome {
            Ok(_) => Value::Null,
            Err(Fault::Identity { .. }) => json!("who_am_i"),
            // the identity read is the first transaction: a bus fault there is the identity step's
            Err(Fault::Bus(_)) if mock.performed() == 1 => json!("who_am_i"),
            Err(Fault::Bus(_)) => json!("sample"),
            Err(other) => panic!("unexpected fault {other:?}"),
        };
        json!({"failed": failed})
    });
}

#[test]
fn diagnose_names_the_wire_fault() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let verdict = driver(false).diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}

#[test]
fn the_int1_verdict_follows_plan_b_s_run_loop() {
    let section = part("wake");
    let unwired = section["cases"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|case| case["input"]["interrupt_wired"] == json!(false))
        .count();
    assert!(unwired >= 2, "the unwired wakes pin plan B's count without a wire");
    let count = check_cases(&section, "wake", |input| {
        let mut imu = driver(input["interrupt_wired"].as_bool().expect("interrupt_wired"));
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
    assert!(count > 10);
}
