//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581's unit tests: the conversion in both temperature signs (test_bmp581.py's frames), the bring-up with
//! its ground zero and the re-arm over the scripted bus, the operator's ground updates, re-zero fresh-only, the
//! calibrate's re-captured ground (M9 D7), and freshness by changed bytes with a standby part going stale (D3).

use control::commons::altitude_m;
use serde_json::json;

use super::{
    Bmp581, CHIP_ID, CONFIGURATION, CONVERSION_PERIOD_US, Change, FIRST_CONVERSION_MS, GROUND_SAMPLE_MS,
    GROUND_SAMPLES, IDENTITY, ODR_CONFIG, REZERO_WINDOW_US, Reading, Refusal, TEMP_DATA_XLSB,
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

/// 101 236 Pa, 100 Pa below [`WARM`]'s, at the same 25 degrees C (6479104 counts): about 8.4 m higher.
const HIGHER: [u8; 6] = [0x00, 0x00, 0x19, 0x00, 0xDD, 0x62];

#[test]
fn the_configuration_already_runs_at_50_hz_in_normal_mode() {
    // ODR_CONFIG bits 6:2 = 0x0F, the 50 Hz entry of Bosch's table (240 Hz at 0); bits 1:0 = 01, normal mode; bit 7
    // keeps deep standby off. D2's 50 Hz needs no register change, so the setup transcript stays plan B's.
    let (register, value) = CONFIGURATION[1];
    assert_eq!(register, ODR_CONFIG);
    assert_eq!((value >> 2) & 0x1F, 0x0F);
    assert_eq!(value & 0x03, 0x01);
    assert_eq!(value & 0x80, 0x80);
    assert_eq!(CONVERSION_PERIOD_US, 1_000_000 / 50);
    assert_eq!(REZERO_WINDOW_US, 60_000, "three conversion periods");
}

#[test]
fn a_calibrate_re_captures_the_ground_from_fresh_readings() {
    // M9 D7: the port's calibrate is capture_ground again, after setup, on the live part
    let mock = MockBus::new();
    let mut barometer = Bmp581::new(ADDRESS);
    barometer.update(&json!({"ground": 12.5}), None).unwrap();
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &HIGHER)
            .expect_delay_ms(GROUND_SAMPLE_MS);
    }
    let ground = barometer.capture_ground(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert_eq!(ground, barometer.ground());
    assert!((ground - altitude_m(101_236.0)).abs() < 1e-3, "{ground}");
    assert!(barometer.elevation(&Reading::decode(&HIGHER)).abs() < 1e-3);
    // NEGATIVE: a calibrate whose read fails keeps the ground it had
    mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &WARM).then_fail();
    assert!(matches!(
        barometer.capture_ground(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Bus(_))
    ));
    assert_eq!(barometer.ground(), ground);
    mock.done();
}

/// Read the part on a 10 ms tick from `start_us`, answering each read with the next of `frames`; the verdicts.
fn poll(barometer: &mut Bmp581, start_us: u64, frames: &[[u8; 6]]) -> Vec<Change> {
    let mock = MockBus::new();
    for frame in frames {
        mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], frame);
    }
    let verdicts = (0u64..)
        .zip(frames)
        .map(|(tick, _)| {
            let (change, _) = barometer
                .read_fresh(&mut mock.clone(), start_us + tick * 10_000)
                .unwrap();
            change
        })
        .collect();
    mock.done();
    verdicts
}

#[test]
fn each_conversion_is_fresh_once_on_a_tick_twice_as_fast() {
    use Change::{Fresh, Unchanged};
    // polled every 10 ms, a 50 Hz part shows each conversion on two reads: published on the first only
    let mut barometer = Bmp581::new(ADDRESS);
    let verdicts = poll(&mut barometer, 0, &[WARM, WARM, HIGHER, HIGHER, FINE, FINE]);
    assert_eq!(verdicts, [Fresh, Unchanged, Fresh, Unchanged, Fresh, Unchanged]);
    // the reading comes back decoded either way: freshness is the bytes', not the decode's
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &COLD);
    let (change, reading) = barometer.read_fresh(&mut mock.clone(), 60_000).unwrap();
    mock.done();
    assert_eq!((change, reading.temperature), (Fresh, -10.0));
}

#[test]
fn a_standby_part_goes_stale_after_three_conversion_periods() {
    // NEGATIVE: a part a reset left in standby returns its last sample forever; the firmware calls it unhealthy at
    // Stale, three conversion periods (60 ms) after the bytes last changed
    let mut barometer = Bmp581::new(ADDRESS);
    let verdicts = poll(&mut barometer, 1_000_000, &[WARM; 9]);
    let first_stale = verdicts.iter().position(|change| *change == Change::Stale).unwrap();
    assert_eq!(first_stale, 6, "{verdicts:?}");
    assert!(verdicts[1..6].iter().all(|change| *change == Change::Unchanged));
    assert!(verdicts[6..].iter().all(|change| *change == Change::Stale));
    // the re-arm writes the mode again and the first new conversion is fresh at once
    let mock = MockBus::new();
    script_configure(&mock);
    barometer.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert_eq!(
        poll(&mut barometer, 1_100_000, &[WARM, HIGHER]),
        [Change::Stale, Change::Fresh]
    );
}

#[test]
fn a_failed_read_is_a_bus_fault_and_judges_nothing() {
    // NEGATIVE: a NAK is the transport's fault, never an unchanged or a stale sample
    let mut barometer = Bmp581::new(ADDRESS);
    poll(&mut barometer, 0, &[WARM]);
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &HIGHER).then_fail();
    assert!(matches!(
        barometer.read_fresh(&mut mock.clone(), 10_000),
        Err(Fault::Bus(_))
    ));
    mock.done();
    assert_eq!(poll(&mut barometer, 20_000, &[WARM]), [Change::Unchanged]);
}
