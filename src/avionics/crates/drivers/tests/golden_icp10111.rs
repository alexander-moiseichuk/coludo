//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ICP-10111 answers as plan B's does: src/avionics/golden/drivers_icp10111.json, which
//! golden/build_drivers_icp10111.py builds from src/glider/drivers/icp10111.py on CPython.
//!
//! THE POLYNOMIAL, decision Q8 (avionics-port.md): the port's f32 is checked twice against plan B, within 0.05 Pa of
//! CPython's f64 and EXACTLY equal to plan B's expression evaluated in single precision, as MicroPython computes it on
//! the P4. Both hold, so the polynomial stays f32 and no f64 is needed. Then the frame CRCs and conversion, plan B's
//! setup transcripts (its one divergence: the port reports every general call), and the run loop's TIMELINE: plan
//! B's run() on a virtual clock, against the port's two-phase poll ticked every millisecond, transaction for
//! transaction at the same millisecond, with the same readings and the same peer re-arms.
//!
//! A reading is checked member by member (the sections' `fields`): pressure within the polynomial's 0.05 Pa,
//! temperature EXACTLY (an exact scaling in both precisions) and altitude within 5 mm (3.2 mm measured), so a
//! decode that drops a temperature bit or an altitude from a rounded pressure cannot hide under a Pa budget.

mod transcript;

use drivers::device::Device;
use drivers::icp10111::{self, Icp10111, Otp, Poll, Reading};
use drivers::mock::MockBus;
use golden::Compare;
use serde_json::{Value, json};
use transcript::{FieldCompare, bytes, check_cases, replay, script, section};

/// plan B's ICP-10111 driver over the compensate, compensate_f32, measure, setup, run, update and diagnose sections.
const ICP10111: &str = include_str!("../../../golden/drivers_icp10111.json");

/// The address plan B's transcripts were taken at.
const ADDRESS: u8 = 0x63;

/// plan B's default period, which its run loop measured at.
const PERIOD_MS: u32 = 100;

/// Microseconds per millisecond: the fixture's clock is in milliseconds, the poll's in microseconds.
const MICROSECONDS_PER_MILLISECOND: u64 = 1000;

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(ICP10111, "drivers_icp10111", name)
}

/// The OTP constants of a case.
fn otp(input: &Value) -> Otp {
    let constants: Vec<i16> = input["otp"]
        .as_array()
        .expect("otp")
        .iter()
        .map(|constant| i16::try_from(constant.as_i64().expect("an integer")).expect("an i16"))
        .collect();
    constants.try_into().expect("four constants")
}

/// The raw words of a case.
fn raw(input: &Value) -> (u32, u16) {
    let pressure = u32::try_from(input["p_raw"].as_u64().expect("p_raw")).expect("24 bits");
    let temperature = u16::try_from(input["t_raw"].as_u64().expect("t_raw")).expect("16 bits");
    (pressure, temperature)
}

/// A reading as the fixture writes one.
fn reading(reading: &Reading) -> Value {
    json!({"altitude": f64::from(reading.altitude), "temperature": f64::from(reading.temperature),
           "pressure": f64::from(reading.pressure)})
}

#[test]
fn the_fixture_measured_f32_inside_the_budget() {
    let measured = section(ICP10111, "drivers_icp10111", "measured");
    let worst = measured["worst_f32_against_f64_pa"].as_f64().unwrap();
    assert!(worst < 0.05, "{measured}");
    // the cache the port does not carry moves plan B's answer by more than f32 does: dropping it is the bigger change
    assert!(
        measured["worst_cache_effect_pa"].as_f64().unwrap() > worst,
        "{measured}"
    );
}

#[test]
fn the_polynomial_is_within_the_budget_of_plan_b_in_f64() {
    let count = check_cases(&part("compensate"), "compensate", |input| {
        let (pressure_raw, temperature_raw) = raw(input);
        json!(f64::from(icp10111::compensate(
            &otp(input),
            pressure_raw,
            temperature_raw
        )))
    });
    assert!(count > 100, "{count} cases");
    // the plan's own case: _compensate(11441968, 27034) with OTP [211, 371, 553, 3833]
    let pressure = icp10111::compensate(&[211, 371, 553, 3833], 11_441_968, 27_034);
    assert!((f64::from(pressure) - 101_797.238_025_787_5).abs() < 0.01, "{pressure}");
}

#[test]
fn the_polynomial_is_the_board_s_to_the_bit() {
    check_cases(&part("compensate_f32"), "compensate_f32", |input| {
        let (pressure_raw, temperature_raw) = raw(input);
        json!(f64::from(icp10111::compensate(
            &otp(input),
            pressure_raw,
            temperature_raw
        )))
    });
}

#[test]
fn the_tolerance_catches_a_wrong_constant() {
    // NEGATIVE: one OTP constant off by one moves the answer past the budget on some case
    let section = part("compensate");
    let compare = Compare::from_json(&section["compare"]);
    let failures = section["cases"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|case| {
            let input = &case["input"];
            let mut constants = otp(input);
            constants[3] += 1;
            let (pressure_raw, temperature_raw) = raw(input);
            let answer = json!(f64::from(icp10111::compensate(
                &constants,
                pressure_raw,
                temperature_raw
            )));
            compare.check("perturbed", &case["output"], &answer).is_err()
        })
        .count();
    assert!(failures > 0);
}

#[test]
fn the_frame_is_checked_and_converted_as_plan_b() {
    let otp: Otp = [211, 371, 553, 3833];
    check_cases(&part("measure"), "measure", |input| {
        let steps = input["script"].as_array().expect("script");
        assert_eq!(steps[0]["bytes"], "48a3", "the measure command first");
        assert_eq!(
            steps[1],
            json!({"op": "delay_ms", "ms": 12}),
            "then the 12 ms conversion"
        );
        let frame: [u8; 9] = bytes(steps[2]["bytes"].as_str().unwrap())
            .try_into()
            .expect("a 9-byte frame");
        match icp10111::decode(&frame) {
            Some((pressure_raw, temperature_raw)) => reading(&Reading::convert(&otp, pressure_raw, temperature_raw)),
            None => json!({"refused": "icp10111 CRC"}),
        }
    });
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    let section = part("setup");
    let compare = Compare::from_json(&section["compare"]);
    let mut divergences = 0;
    for case in section["cases"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let mock = replay(&case["input"]["script"]);
        let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS);
        let ok = barometer.setup(&mut mock.clone(), &mut mock.delay()).is_ok();
        mock.done();
        let answer = json!({"ok": ok, "ground": barometer.ground().filter(|_| ok).map(f64::from),
                            "peers_rearmed": barometer.take_general_call()});
        let owed = match case.get("divergence") {
            Some(divergence) => {
                divergences += 1;
                &divergence["output"]
            }
            None => &case["output"],
        };
        compare
            .check(name, owed, &answer)
            .unwrap_or_else(|failure| panic!("{failure}"));
    }
    assert_eq!(divergences, 1, "the dead part's general call is the one divergence");
}

#[test]
fn the_run_loop_keeps_plan_b_s_timeline() {
    let count = check_cases(&part("run"), "run", |input| {
        let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS);
        let setup = replay(&input["setup"]);
        barometer
            .setup(&mut setup.clone(), &mut setup.delay())
            .expect("the setup before the run");
        setup.done();
        let all = input["script"].as_array().expect("script");
        let transactions: Vec<Value> = all.iter().filter(|step| step["op"] != "delay_ms").cloned().collect();
        let mock = MockBus::new();
        script(&mock, &Value::Array(transactions.clone()));
        let mut bus = mock.clone();
        // plan B stopped in the wait that began at the last step's millisecond; nothing after it is compared
        let end = all.last().expect("a step")["at_ms"].as_u64().expect("at_ms");
        let (mut times, mut readings, mut rearms) = (Vec::new(), Vec::new(), Vec::new());
        for now in 0..=end {
            match barometer.poll(&mut bus, now * MICROSECONDS_PER_MILLISECOND) {
                Poll::Waiting => {}
                Poll::Rearm => rearms.push(now),
                Poll::Reading(collected) => {
                    times.push(now);
                    let mut row = reading(&collected);
                    row["at_ms"] = json!(now);
                    readings.push(row);
                }
                Poll::Issued | Poll::Failed { .. } | Poll::GeneralCall => times.push(now),
            }
        }
        mock.done();
        let expected: Vec<u64> = transactions
            .iter()
            .map(|step| step["at_ms"].as_u64().unwrap())
            .collect();
        assert_eq!(times, expected, "the transactions' milliseconds");
        json!({"readings": readings, "rearms": rearms})
    });
    assert!(count >= 7);
}

#[test]
fn a_reading_is_checked_member_by_member() {
    // NEGATIVE: the two errors a pressure budget hid -- a temperature one count low, an altitude 6 mm off -- fail
    // their own members, while a pressure inside its 0.05 Pa still passes
    let section = part("measure");
    let compare = FieldCompare::from_section(&section);
    let case = &section["cases"][0];
    let expected = &case["output"];
    let nudged = |member: &str, by: f64| {
        let mut answer = expected.clone();
        answer[member] = json!(expected[member].as_f64().unwrap() + by);
        compare.check("nudged", expected, &answer)
    };
    assert!(nudged("temperature", -175.0 / 65_536.0).is_err());
    assert!(nudged("altitude", 0.006).is_err());
    assert!(nudged("pressure", 0.04).is_ok());
    assert!(nudged("pressure", 0.06).is_err());
}

#[test]
fn the_ground_updates_as_plan_b() {
    let section = part("update");
    let compare = Compare::from_json(&section["compare"]);
    let mut divergences = 0;
    for case in section["cases"].as_array().unwrap() {
        let input = &case["input"];
        let mut barometer = Icp10111::new(ADDRESS, PERIOD_MS);
        barometer
            .update(&json!({"ground": 7.0}), None)
            .expect("the starting ground");
        let fresh = input["fresh_altitude"].as_f64().map(|altitude| altitude as f32);
        let ground = |barometer: &Icp10111| f64::from(barometer.ground().expect("a ground"));
        let answer = match barometer.update(&input["properties"], fresh) {
            Ok(changed) => json!({"changed": changed, "ground": ground(&barometer)}),
            Err(refusal) => json!({"refused": refusal.to_string(), "ground": ground(&barometer)}),
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
fn diagnose_names_the_wire_fault() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let verdict = Icp10111::new(ADDRESS, PERIOD_MS).diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}
