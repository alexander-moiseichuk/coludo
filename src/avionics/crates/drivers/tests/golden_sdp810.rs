//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The SDP810 pitot answers as plan B's does: src/avionics/golden/drivers_sdp810.json, which
//! golden/build_drivers_sdp810.py builds from src/glider/drivers/sdp810.py on CPython. Eight sections, each under
//! its own comparison: the CRC, the frame check, the pressure (Pa x 100 exactly, the signed channel in Pa within the
//! builder's measured tolerance), the airspeed (m/s within that tolerance, cm/s exactly), the tare ladder step by
//! step, and three I2C transcripts replayed through the mock bus -- setup, the run loop's restart ladder (each good
//! pass's row exactly and its two channel values within the tolerance), and diagnose -- where the Rust driver must
//! make plan B's transactions in plan B's order.

mod transcript;

use drivers::device::{Device, Fault};
use drivers::sdp810::{self, Sdp810, Settings};
use golden::Compare;
use serde_json::{Value, json};
use transcript::{bytes, check_cases, replay, script, section};

/// plan B's SDP810 driver over the crc, frames, pressure, airspeed, tare, setup, run and diagnose sections.
const SDP810: &str = include_str!("../../../golden/drivers_sdp810.json");

/// A fixture section, checked for provenance.
fn part(name: &str) -> Value {
    section(SDP810, "drivers_sdp810", name)
}

/// An integer member, narrowed.
fn integer<T: TryFrom<i64>>(value: &Value, member: &str) -> T {
    value[member]
        .as_i64()
        .and_then(|number| T::try_from(number).ok())
        .unwrap_or_else(|| panic!("'{member}' is not an integer in range in {value}"))
}

/// A driver with plan B's scale latched and its zero (Pa x 100) and density set, as the fixture's state names them.
fn driver(scale: u16, zero: i32, density: f32) -> Sdp810 {
    let mut pitot = Sdp810::new(&Settings {
        address: sdp810::ADDRESS,
        period_ms: sdp810::DEFAULT_PERIOD_MS,
        air_density: density,
        zero_offset: zero,
    });
    pitot.latch_scale(scale);
    pitot
}

#[test]
fn the_crc() {
    let count = check_cases(&part("crc"), "crc", |input| {
        json!(drivers::commons::sensirion_crc8(
            integer(input, "first"),
            integer(input, "second")
        ))
    });
    assert!(count > 40);
}

#[test]
fn the_three_crc_frame_check() {
    check_cases(&part("frames"), "frames", |input| {
        let frame = bytes(input["frame"].as_str().unwrap());
        let mut answer = json!({"ok": sdp810::frame_ok(&frame)});
        if frame.len() == sdp810::FRAME_LENGTH {
            answer["pressure"] = json!(sdp810::signed16(frame[0], frame[1]));
            answer["temperature"] = json!(sdp810::signed16(frame[3], frame[4]));
            answer["scale"] = json!(u16::from_be_bytes([frame[6], frame[7]]));
        }
        answer
    });
}

#[test]
fn the_pressure_in_hundredths_of_a_pascal_and_on_the_channel() {
    let section = part("pressure");
    assert_eq!(Compare::from_json(&section["compare"]), Compare::Abs(0.01));
    let count = check_cases(&section, "pressure", |input| {
        let mut pitot = driver(integer(input, "scale"), integer(input, "zero"), sdp810::AIR_DENSITY);
        let raw = integer(input, "raw");
        json!({"hundredths": pitot.pressure_hundredths(raw), "dynamic_pressure": f64::from(pitot.pressure(raw))})
    });
    assert!(count > 100);
}

#[test]
fn the_airspeed_band() {
    let section = part("airspeed");
    assert_eq!(Compare::from_json(&section["compare"]), Compare::Abs(0.01));
    check_cases(&section, "airspeed", |input| {
        let mut pitot = driver(
            integer(input, "scale"),
            integer(input, "zero"),
            input["density"].as_f64().unwrap() as f32,
        );
        let pressure = pitot.pressure(integer(input, "raw"));
        let airspeed = pitot.airspeed(pressure);
        json!({"airspeed": f64::from(airspeed), "airspeed_cms": sdp810::airspeed_cms(airspeed),
               "dynamic_pressure": f64::from(pressure)})
    });
}

#[test]
fn the_airspeed_band_of_test_sdp810() {
    // test_sdp810.py: 135 Pa at 1.225 kg/m^3 is 14.5..15.5 m/s; a negative and a zero q are exactly 0
    let mut pitot = driver(60, 0, 1.225);
    for (raw, low, high) in [(8100, 14.5, 15.5), (-600, 0.0, 0.0), (0, 0.0, 0.0)] {
        let pressure = pitot.pressure(raw);
        let airspeed = pitot.airspeed(pressure);
        assert!(low <= airspeed && airspeed <= high, "raw {raw}: {airspeed}");
    }
}

/// The driver state every tare step reports.
fn state(pitot: &Sdp810) -> Value {
    json!({"zero": pitot.zero_offset(), "tared": pitot.tared(), "density": f64::from(pitot.air_density())})
}

/// Apply one tare step's `in` and answer its `out`.
fn tare_step(pitot: &mut Sdp810, request: &Value) -> Value {
    match request["action"].as_str() {
        Some("pressure") => {
            let raw = integer(request, "raw");
            pitot.pressure(raw);
            let mut out = state(pitot);
            out["hundredths"] = json!(pitot.pressure_hundredths(raw));
            out
        }
        Some("update") => match pitot.update(&request["properties"]) {
            Ok(update) => {
                let mut out = state(pitot);
                out["changed"] = json!(update.changed);
                out["persist"] = json!(update.persist);
                out
            }
            Err(refusal) => {
                let mut out = state(pitot);
                out["refused"] = json!(refusal.to_string());
                out
            }
        },
        Some("calibration") => {
            let channel = request["channel"].as_i64().map(|hundredths| hundredths as f32 / 100.0);
            let mut out = state(pitot);
            out["prompt"] = json!(pitot.calibration(channel).unwrap_or_default());
            out
        }
        Some("calibrate") => {
            let answer = pitot.calibrate();
            let mut out = state(pitot);
            out["message"] = json!(answer.err());
            out["persist"] = json!(answer.ok());
            out
        }
        other => panic!("unknown tare action {other:?}"),
    }
}

#[test]
fn the_tare_ladder_step_by_step() {
    let section = part("tare");
    let compare = Compare::from_json(&section["compare"]);
    let mut steps = 0;
    for scenario in section["scenarios"].as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let init = &scenario["init"];
        assert_eq!(init["tared"], false, "{name}: every scenario starts untared");
        let mut pitot = driver(
            integer(init, "scale"),
            integer(init, "zero"),
            init["density"].as_f64().unwrap() as f32,
        );
        for (index, step) in scenario["steps"].as_array().unwrap().iter().enumerate() {
            let answer = tare_step(&mut pitot, &step["in"]);
            compare
                .check(&format!("{name} step {index}"), &step["out"], &answer)
                .unwrap_or_else(|failure| panic!("{failure}"));
            steps += 1;
        }
    }
    assert!(steps > 20, "only {steps} tare steps");
}

#[test]
fn setup_makes_plan_b_s_transactions() {
    check_cases(&part("setup"), "setup", |input| {
        let mock = replay(&input["script"]);
        let mut pitot = Sdp810::new(&Settings::from_config(&input["config"]));
        if let Some(stored) = input["stored_zero"].as_i64() {
            pitot.restore_zero(i32::try_from(stored).unwrap());
        }
        let outcome = pitot.setup(&mut mock.clone(), &mut mock.delay());
        mock.done();
        json!({"ok": outcome.is_ok(), "scale": pitot.scale(), "zero": pitot.zero_offset(), "tared": pitot.tared()})
    });
}

#[test]
fn the_run_loop_and_its_restart_ladder() {
    let section = part("run");
    let compare = Compare::from_json(&section["compare"]);
    let mut restarts = 0;
    for scenario in section["scenarios"].as_array().unwrap() {
        let name = scenario["name"].as_str().unwrap();
        let mut pitot = driver(60, 0, 1.18);
        let mock = drivers::mock::MockBus::new();
        for (index, step) in scenario["steps"].as_array().unwrap().iter().enumerate() {
            script(&mock, &step["in"]["script"]);
            let answer = match pitot.read(&mut mock.clone(), &mut mock.delay()) {
                Ok(sample) => json!({"row": sample.row, "dynamic_pressure": f64::from(sample.dynamic_pressure),
                                     "airspeed": f64::from(sample.airspeed)}),
                Err(failure) => {
                    let fault = match failure.fault {
                        Fault::Bus(_) => "bus",
                        Fault::Checksum => "checksum",
                        other => panic!("{name} pass {index}: unexpected fault {other:?}"),
                    };
                    restarts += usize::from(failure.restart.is_some());
                    json!({"fault": fault, "restart": failure.restart.map(|outcome| outcome.is_ok())})
                }
            };
            mock.done();
            compare
                .check(&format!("{name} pass {index}"), &step["out"], &answer)
                .unwrap_or_else(|failure| panic!("{failure}"));
        }
    }
    assert!(restarts >= 6, "only {restarts} restarts replayed");
}

#[test]
fn diagnose_names_the_wire_fault() {
    check_cases(&part("diagnose"), "diagnose", |input| {
        let mock = replay(&input["script"]);
        let pitot = Sdp810::new(&Settings::from_config(&json!({})));
        let verdict = pitot.diagnose(&mut mock.clone());
        mock.done();
        json!(verdict)
    });
}

#[test]
fn a_wrong_answer_fails_the_fixture() {
    // NEGATIVE: the comparisons bite -- a pressure one hundredth off, a reverse flow clamped to 0 Pa on the
    // channel, and a transcript missing the 3 ms settle
    let section = part("pressure");
    let compare = Compare::from_json(&section["compare"]);
    let case = &section["cases"][0];
    let mut wrong = case["output"].clone();
    wrong["hundredths"] = json!(case["output"]["hundredths"].as_i64().unwrap() + 1);
    assert!(compare.check("pressure", &case["output"], &wrong).is_err());
    let reverse = section["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|case| case["input"] == json!({"raw": -600, "scale": 60, "zero": 0}))
        .unwrap();
    assert_eq!(reverse["output"]["dynamic_pressure"], -10.0);
    let mut clamped = reverse["output"].clone();
    clamped["dynamic_pressure"] = json!(0.0);
    assert!(compare.check("pressure", &reverse["output"], &clamped).is_err());
    let setup = part("setup");
    let mut script = setup["cases"][0]["input"]["script"].clone();
    script.as_array_mut().unwrap().retain(|step| step["ms"] != 3);
    let outcome = std::panic::catch_unwind(|| {
        let mock = replay(&script);
        let mut pitot = Sdp810::new(&Settings::from_config(&json!({})));
        let _ = pitot.setup(&mut mock.clone(), &mut mock.delay());
    });
    assert!(outcome.is_err(), "a stop straight into a start must not replay");
}

#[test]
fn the_plan_s_anchors_are_in_the_fixture() {
    // avionics-port.md M10: CRC 0xBEEF -> 0x92, _pressure(8100) = 13500, tare 200 -> 333
    let crc = part("crc");
    let beef = crc["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|case| case["name"] == "crc_beef")
        .unwrap();
    assert_eq!(beef["output"], 0x92);
    let pressure = part("pressure");
    let anchor = pressure["cases"]
        .as_array()
        .unwrap()
        .iter()
        .find(|case| case["input"] == json!({"raw": 8100, "scale": 60, "zero": 0}))
        .unwrap();
    assert_eq!(anchor["output"]["hundredths"], 13500);
    let tare = part("tare");
    let ladder = &tare["scenarios"][0];
    assert_eq!(ladder["name"], "test_sdp810_ladder");
    let captured = ladder["steps"]
        .as_array()
        .unwrap()
        .iter()
        .find(|step| step["out"]["persist"].is_i64())
        .unwrap();
    assert_eq!(
        (captured["out"]["persist"].clone(), captured["out"]["zero"].clone()),
        (json!(333), json!(333))
    );
}
