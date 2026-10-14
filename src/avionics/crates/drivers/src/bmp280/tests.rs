//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP280's unit tests: the compensation against plan B's `_compensate` (the datasheet's worked example and two
//! more frames, the golden numbers computed by plan B's own code on CPython), the calibration's byte order and
//! signs, the bring-up with its calibration read and ground zero over the scripted bus, the re-arm, and the BMP581's
//! guards kept: a pair that disagrees is torn, one out of the part's range implausible, a ground capture that runs out
//! of tries keeps the old ground.

use super::{
    Bmp280, CALIBRATION, CALIBRATION_LENGTH, CHIP_ID, CONFIGURATION, Calibration, DATA, FIRST_CONVERSION_MS,
    GROUND_SAMPLE_MS, GROUND_SAMPLES, GROUND_TRIES, IDENTITY, Outcome, Reading, floor_div,
};
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's address in these tests.
const ADDRESS: u8 = 0x76;

/// The datasheet's example calibration (dig_T1 27504, T2 26435, T3 -1000, P1 36477, P2 -10685, P3 3024, P4 2855,
/// P5 140, P6 -7, P7 15500, P8 -14600, P9 6000) as the part's 24 little-endian bytes.
const DATASHEET: [u8; CALIBRATION_LENGTH] = [
    0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, 0x7D, 0x8E, 0x43, 0xD6, 0xD0, 0x0B, 0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF, 0x8C,
    0x3C, 0xF8, 0xC6, 0x70, 0x17,
];

/// The datasheet's example frame: adc_P 415148, adc_T 519888 -> 100653.254 Pa (plan B), 25.08 degrees C.
const EXAMPLE: [u8; 6] = [0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00];

/// adc_T 400000, adc_P 300000 -> 113634.82 Pa at -12.64 degrees C: above the part's 1100 hPa, so implausible.
const ABOVE_RANGE: [u8; 6] = [0x49, 0x3E, 0x00, 0x61, 0xA8, 0x00];

/// adc_T 600000, adc_P 500000 -> 89314.27 Pa at 50.11 degrees C.
const WARM: [u8; 6] = [0x7A, 0x12, 0x00, 0x92, 0x7C, 0x00];

#[test]
fn the_compensation_is_plan_b_s_to_the_bit() {
    let calibration = Calibration::decode(&DATASHEET);
    let reading = Reading::decode(&EXAMPLE, &calibration);
    // plan B's numbers, as its single-precision floats hold them: the integer to f32, then the exact division
    assert_eq!(reading.pressure, 25_767_233_f32 / 256.0, "{}", reading.pressure);
    assert!((reading.pressure - 100_653.25).abs() < 0.01, "{}", reading.pressure);
    assert_eq!(reading.temperature, 2508_f32 / 100.0);
    assert_eq!(reading.altitude, control::commons::altitude_m(reading.pressure));
    let warm = Reading::decode(&WARM, &calibration);
    assert!(
        (warm.pressure - 89_314.27).abs() < 0.01 && warm.temperature == 5011_f32 / 100.0,
        "{warm:?}"
    );
    let cold = Reading::decode(&ABOVE_RANGE, &calibration);
    assert!(
        (cold.pressure - 113_634.82).abs() < 0.01 && cold.temperature == -1264_f32 / 100.0,
        "{cold:?}"
    );
}

#[test]
fn the_calibration_reads_unsigned_and_signed_words_little_endian() {
    let calibration = Calibration::decode(&DATASHEET);
    assert_eq!(calibration.temperature, [27_504, 26_435, -1_000]);
    assert_eq!(
        calibration.pressure,
        [36_477, -10_685, 3_024, 2_855, 140, -7, 15_500, -14_600, 6_000]
    );
    // NEGATIVE: dig_T1 and dig_P1 are unsigned: 0xFFFF reads 65535, never -1
    let mut high = DATASHEET;
    high[0] = 0xFF;
    high[1] = 0xFF;
    assert_eq!(Calibration::decode(&high).temperature[0], 65_535);
}

#[test]
fn a_degenerate_calibration_gives_zero_pascal_as_plan_b() {
    // NEGATIVE: dig_P1 0 makes the divisor 0: plan B returns 0.0 Pa, never a division by zero
    let mut degenerate = DATASHEET;
    degenerate[6] = 0;
    degenerate[7] = 0;
    let (pressure, temperature) = Calibration::decode(&degenerate).compensate(519_888, 415_148);
    assert_eq!(pressure, 0.0);
    assert_eq!(temperature, 25.08);
}

#[test]
fn floor_division_is_python_s() {
    assert_eq!(floor_div(7, 2), 3);
    assert_eq!(floor_div(-7, 2), -4, "floored, not truncated toward zero");
    assert_eq!(floor_div(7, -2), -4);
    assert_eq!(floor_div(-7, -2), 3);
    assert_eq!(floor_div(-8, 2), -4, "exact: no correction");
}

/// Script the setup's reads up to the ground capture: the identity, the calibration, the configuration, the wait.
fn script_setup(mock: &MockBus) {
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write_read(ADDRESS, &[CALIBRATION], &DATASHEET);
    for (register, value) in CONFIGURATION {
        mock.expect_write(ADDRESS, &[register, value]);
    }
    mock.expect_delay_ms(FIRST_CONVERSION_MS);
}

#[test]
fn setup_reads_the_calibration_and_averages_the_ground() {
    let mock = MockBus::new();
    script_setup(&mock);
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
            .expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
            .expect_delay_ms(GROUND_SAMPLE_MS);
    }
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bmp280::new(ADDRESS);
    part.setup(&mut bus, &mut delay).unwrap();
    mock.done();
    let expected = Reading::decode(&EXAMPLE, &Calibration::decode(&DATASHEET)).altitude;
    assert!(
        (part.ground() - expected).abs() < 1e-3,
        "{} vs {expected}",
        part.ground()
    );
}

#[test]
fn a_wrong_identity_fails_setup_before_any_write() {
    // NEGATIVE: a BME280 (0x60) or another part answers: setup refuses, nothing is configured
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0x60]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let outcome = Bmp280::new(ADDRESS).setup(&mut bus, &mut delay);
    mock.done();
    assert!(
        matches!(
            outcome,
            Err(Fault::Identity {
                expected: 0x58,
                read: 0x60
            })
        ),
        "{outcome:?}"
    );
}

#[test]
fn a_garbage_ground_capture_keeps_the_old_ground() {
    // NEGATIVE: every pair of the first sample disagrees: the capture fails with Garbage and the ground stays
    let mock = MockBus::new();
    script_setup(&mock);
    for _ in 0..GROUND_TRIES {
        mock.expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
            .expect_write_read(ADDRESS, &[DATA], &WARM);
    }
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bmp280::new(ADDRESS);
    let outcome = part.setup(&mut bus, &mut delay);
    mock.done();
    assert!(matches!(outcome, Err(Fault::Garbage)), "{outcome:?}");
    assert_eq!(part.ground(), 0.0);
}

/// A set-up part (the ground at the example's altitude) over a fresh mock.
fn set_up() -> (Bmp280, MockBus) {
    let mock = MockBus::new();
    script_setup(&mock);
    for _ in 0..GROUND_SAMPLES {
        mock.expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
            .expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
            .expect_delay_ms(GROUND_SAMPLE_MS);
    }
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bmp280::new(ADDRESS);
    part.setup(&mut bus, &mut delay).unwrap();
    mock.done();
    (part, MockBus::new())
}

#[test]
fn read_fresh_publishes_new_conversions_and_refuses_torn_and_implausible_pairs() {
    let (mut part, mock) = set_up();
    mock.expect_write_read(ADDRESS, &[DATA], &WARM)
        .expect_write_read(ADDRESS, &[DATA], &WARM)
        .expect_write_read(ADDRESS, &[DATA], &WARM)
        .expect_write_read(ADDRESS, &[DATA], &WARM)
        .expect_write_read(ADDRESS, &[DATA], &WARM)
        .expect_write_read(ADDRESS, &[DATA], &EXAMPLE)
        .expect_write_read(ADDRESS, &[DATA], &ABOVE_RANGE)
        .expect_write_read(ADDRESS, &[DATA], &ABOVE_RANGE);
    let mut bus = mock.clone();
    assert!(matches!(part.read_fresh(&mut bus, 1_000_000), Ok(Outcome::Fresh(_))));
    assert_eq!(
        part.read_fresh(&mut bus, 1_010_000),
        Ok(Outcome::Unchanged),
        "the same bytes: no new conversion"
    );
    // NEGATIVE: a pair that disagrees is torn, judged nothing
    assert_eq!(part.read_fresh(&mut bus, 1_020_000), Ok(Outcome::Torn));
    // NEGATIVE: a pair that agrees above 1100 hPa is garbage that repeated
    assert!(matches!(
        part.read_fresh(&mut bus, 1_030_000),
        Ok(Outcome::Implausible(_))
    ));
    mock.done();
}

#[test]
fn rearm_writes_the_mode_again_and_keeps_the_calibration() {
    let (mut part, mock) = set_up();
    for (register, value) in CONFIGURATION {
        mock.expect_write(ADDRESS, &[register, value]);
    }
    mock.expect_write_read(ADDRESS, &[DATA], &EXAMPLE);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    part.rearm(&mut bus, &mut delay).unwrap();
    let reading = part.read(&mut bus).unwrap();
    mock.done();
    assert_eq!(reading.temperature, 25.08, "the calibration survived the re-arm");
}
