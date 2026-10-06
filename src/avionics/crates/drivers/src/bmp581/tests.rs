//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581's unit tests: the conversion in both temperature signs (test_bmp581.py's frames), the bring-up with
//! its ground zero and the re-arm over the scripted bus, and the operator's ground updates, re-zero fresh-only.

use control::commons::altitude_m;
use serde_json::json;

use super::{
    Bmp581, CHIP_ID, CONFIGURATION, FIRST_CONVERSION_MS, GROUND_SAMPLE_MS, GROUND_SAMPLES, IDENTITY, Reading, Refusal,
    TEMP_DATA_XLSB,
};
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's address in these tests.
const ADDRESS: u8 = 0x47;

/// test_bmp581.py's positive frame: 25.0 degrees C (1638400 counts) and 101336 Pa (6485504 counts).
const WARM: [u8; 6] = [0x00, 0x00, 0x19, 0x00, 0xF6, 0x62];

/// test_bmp581.py's negative frame: -10.0 degrees C (0xF60000 as 24-bit two's complement), the same pressure.
const COLD: [u8; 6] = [0x00, 0x00, 0xF6, 0x00, 0xF6, 0x62];

/// A frame whose XLSB bytes are NOT zero: 25 + 1/65536 degrees C (0x190001) and 101336 + 1/64 Pa (0x62F601). Both
/// of test_bmp581.py's frames carry 0x00 there, so a decode that drops the low byte would pass them.
const FINE: [u8; 6] = [0x01, 0x00, 0x19, 0x01, 0xF6, 0x62];

/// Script the configuration writes.
fn script_configure(mock: &MockBus) {
    for (register, value) in CONFIGURATION {
        mock.expect_write(ADDRESS, &[register, value]);
    }
}

#[test]
fn the_conversion_is_exact_and_the_altitude_is_the_shared_one() {
    let reading = Reading::decode(&WARM);
    assert_eq!(reading.temperature, 25.0);
    assert_eq!(reading.pressure, 101_336.0);
    assert_eq!(
        reading.altitude,
        altitude_m(101_336.0),
        "altitude must come from the shared helper"
    );
}

#[test]
fn the_low_byte_counts() {
    // NEGATIVE against the classic decode bug: (raw >> 8) / 256 drops the XLSB, a 1/65536 degree (and 1/64 Pa) step
    let reading = Reading::decode(&FINE);
    assert_eq!(reading.temperature, 25.0 + 1.0 / 65_536.0);
    assert_ne!(reading.temperature, 25.0);
    assert_eq!(reading.pressure, 101_336.0 + 1.0 / 64.0);
    assert_eq!(reading.altitude, altitude_m(reading.pressure));
    // the XLSB of a sub-zero count: 0xF5FFFF is -10 - 1/65536 degrees C
    assert_eq!(
        Reading::decode(&[0xFF, 0xFF, 0xF5, 0x00, 0xF6, 0x62]).temperature,
        -10.0 - 1.0 / 65_536.0
    );
}

#[test]
fn a_sub_zero_temperature_sign_extends() {
    // NEGATIVE: read as unsigned, -10 degrees C would come back as ~+246 degrees C, plausible on a chart
    let reading = Reading::decode(&COLD);
    assert_eq!(reading.temperature, -10.0);
    assert_eq!(
        reading.pressure, 101_336.0,
        "pressure is unsigned: no sign extension there"
    );
    let top = Reading::decode(&[0, 0, 0, 0xFF, 0xFF, 0xFF]);
    assert_eq!(
        top.pressure,
        16_777_215.0 / 64.0,
        "the top pressure count stays positive"
    );
}

#[test]
fn a_dead_read_is_no_altitude() {
    // zero pressure is a dead read, not sea level: the shared helper answers 0
    assert_eq!(Reading::decode(&[0; 6]).altitude, 0.0);
}

#[test]
fn setup_configures_waits_and_averages_the_ground() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY]);
    script_configure(&mock);
    mock.expect_delay_ms(FIRST_CONVERSION_MS);
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &WARM)
            .expect_delay_ms(GROUND_SAMPLE_MS);
    }
    let mut barometer = Bmp581::new(ADDRESS);
    barometer.setup(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert!(
        (barometer.ground() - altitude_m(101_336.0)).abs() < 1e-3,
        "{}",
        barometer.ground()
    );
    assert!(barometer.elevation(&Reading::decode(&WARM)).abs() < 1e-3);
}

#[test]
fn another_part_or_a_failed_read_fails_setup() {
    // NEGATIVE: another identity configures nothing; a NAK while averaging fails setup and keeps the old ground
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0x58]);
    let mut barometer = Bmp581::new(ADDRESS);
    assert_eq!(
        barometer.setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Identity {
            expected: 0x50,
            read: 0x58
        })
    );
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY]);
    script_configure(&mock);
    mock.expect_delay_ms(FIRST_CONVERSION_MS)
        .expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &WARM)
        .expect_delay_ms(GROUND_SAMPLE_MS)
        .expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &WARM)
        .then_fail();
    assert!(matches!(
        barometer.setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Bus(_))
    ));
    assert_eq!(barometer.ground(), 0.0);
    mock.done();
}

#[test]
fn the_rearm_writes_the_mode_again_and_keeps_the_ground() {
    let mock = MockBus::new();
    script_configure(&mock);
    let mut barometer = Bmp581::new(ADDRESS);
    barometer.update(&json!({"ground": 12.5}), None).unwrap();
    barometer.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert_eq!(barometer.ground(), 12.5);
}

#[test]
fn the_ground_is_set_or_re_zeroed_from_a_fresh_altitude_only() {
    let mut barometer = Bmp581::new(ADDRESS);
    barometer.update(&json!({"ground": 7.0}), None).unwrap();
    // NEGATIVE: no fresh altitude (never pushed, or silent for over three periods) refuses, and leaves the ground
    assert_eq!(
        barometer.update(&json!({"rezero": true}), None),
        Err(Refusal::NoFreshAltitude)
    );
    assert_eq!(barometer.ground(), 7.0);
    assert_eq!(
        barometer.update(&json!({"rezero": true}), Some(123.5)),
        Ok(vec!["ground"])
    );
    assert_eq!(barometer.ground(), 123.5);
    // a falsy re-zero, or nothing known, changes nothing
    assert_eq!(barometer.update(&json!({"rezero": false}), Some(1.0)), Ok(vec![]));
    assert_eq!(barometer.update(&json!({"rezero": 0}), Some(1.0)), Ok(vec![]));
    assert_eq!(barometer.update(&json!({"other": 1}), Some(1.0)), Ok(vec![]));
    assert_eq!(barometer.ground(), 123.5);
    // a set wins over a re-zero in one update; a non-number is refused
    assert_eq!(
        barometer.update(&json!({"ground": 3.0, "rezero": true}), Some(9.0)),
        Ok(vec!["ground"])
    );
    assert_eq!(barometer.ground(), 3.0);
    assert_eq!(
        barometer.update(&json!({"ground": "high"}), None),
        Err(Refusal::NotANumber)
    );
    assert_eq!(barometer.ground(), 3.0, "a refused ground leaves the ground");
    // plan B's float() takes a numeral string and a bool as well as a number
    assert_eq!(barometer.update(&json!({"ground": "12.5"}), None), Ok(vec!["ground"]));
    assert_eq!(barometer.ground(), 12.5);
    assert_eq!(barometer.update(&json!({"ground": false}), None), Ok(vec!["ground"]));
    assert_eq!(barometer.ground(), 0.0);
    assert_eq!(
        barometer.update(&json!({"ground": null}), None),
        Err(Refusal::NotANumber)
    );
    assert_eq!(Refusal::NotANumber.to_string(), "ground must be a number");
    assert_eq!(
        Refusal::NoFreshAltitude.to_string(),
        "no fresh altitude to re-zero from"
    );
}

#[test]
fn the_probe_and_the_diagnosis() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &COLD)
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0xFF])
        .expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .then_fail();
    let barometer = Bmp581::new(ADDRESS);
    assert_eq!(barometer.probe(&mut mock.clone()).unwrap().temperature, -10.0);
    assert!(barometer.diagnose(&mut mock.clone()).contains("idle-high"));
    assert!(barometer.diagnose(&mut mock.clone()).starts_with("no bus response"));
    mock.done();
}
