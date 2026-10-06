//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The INA226 power monitor answers as plan B's does: src/avionics/golden/drivers_ina226.json, which
//! golden/build_drivers_ina226.py builds from src/glider/drivers/ina226.py on CPython. Every section compares
//! exactly: the driver is integer milli-units end to end. The calibration and alert-limit math, then the I2C
//! transcripts replayed through the mock bus -- read (three separate 2-byte reads), setup, rearm, probe and
//! diagnose -- where the Rust driver must make plan B's transactions in plan B's order. The one deliberate
//! divergence, the re-arm of an armed alert (healthy, or with one of its two extra writes refused), is the
//! fixture's own `divergence` member, and the port owes that.

mod transcript;

use drivers::device::{Device, Fault};
use drivers::ina226::{self, Ina226, Settings};
use golden::Compare;
use serde_json::{Value, json};
use transcript::{check_cases, replay, section};

/// plan B's INA226 driver over the calibration, alert, read, setup, rearm, probe and diagnose sections.
const INA226: &str = include_str!("../../../golden/drivers_ina226.json");

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(INA226, "drivers_ina226", name)
}

/// An i64 member.
fn integer(value: &Value, member: &str) -> i64 {
    value[member]
        .as_i64()
        .unwrap_or_else(|| panic!("'{member}' is not an integer in {value}"))
}

/// A monitor as setup leaves it, from a component block (the alert as the fixture says it is wired).
fn monitor(config: &Value, alert_wired: bool) -> Ina226 {
    Ina226::new(&Settings::from_config(config, alert_wired))
}

#[test]
fn the_calibration_register() {
    let count = check_cases(&part("calibration"), "calibration", |input| {
        json!(ina226::calibration(
            integer(input, "max_current_ma"),
            integer(input, "shunt_mohms")
        ))
    });
    assert!(count > 40);
    // test_ina226.py: 167772160 // (5000 x 10) = 3355
    assert_eq!(ina226::calibration(5000, 10), Some(3355));
}

#[test]
fn the_alert_limit() {
    check_cases(&part("alert"), "alert", |input| {
        json!(ina226::alert_limit(
            integer(input, "alert_ma"),
            integer(input, "shunt_mohms")
        ))
    });
}

#[test]
fn three_separate_reads_convert_to_milli_units() {
    check_cases(&part("read"), "read", |input| {
        let mock = replay(&input["script"]);
        let block = json!({"max_current_ma": integer(input, "max_current_ma")});
        let measurement = monitor(&block, false).read(&mut mock.clone()).expect("a scripted read");
        mock.done();
        json!({"voltage_mv": measurement.voltage_mv, "current_ma": measurement.current_ma,
               "power_mw": measurement.power_mw})
    });
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let mut power = monitor(&input["config"], input["alert_wired"].as_bool().unwrap());
        let outcome = power.setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok(), "calibration": power.calibration(), "alert_ma": power.alert_ma()})
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
        let mut power = monitor(&input["config"], input["alert_wired"].as_bool().unwrap());
        let setup = replay(&input["setup"]);
        power
            .setup(&mut setup.clone(), &mut setup.delay())
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
        let outcome = power.rearm(&mut mock.clone(), &mut mock.delay());
        mock.done();
        compare
            .check(name, &owed, &json!({"ok": outcome.is_ok()}))
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    // the armed-alert re-arm is the one divergence: healthy, its ALERT LIMIT refused, its MASK/ENABLE refused
    assert_eq!(divergences, 3, "the armed-alert re-arms are the divergences");
}

#[test]
fn the_probe_fails_at_plan_b_s_step() {
    check_cases(&part("probe"), "probe", |input| {
        let mock = replay(&input["script"]);
        let outcome = monitor(&json!({}), false).probe(&mut mock.clone());
        mock.done();
        let failed = match outcome {
            Ok(_) => Value::Null,
            Err(Fault::Identity { .. }) => json!("die id"),
            // the die id read itself is a bus fault on the first step; a bus fault later is the read's
            Err(Fault::Bus(_)) if input["script"].as_array().unwrap().len() == 1 => json!("die id"),
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
        let verdict = monitor(&json!({}), false).diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}
