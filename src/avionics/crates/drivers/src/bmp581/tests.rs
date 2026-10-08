//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMP581's unit tests: the conversion in both temperature signs (test_bmp581.py's frames), the bring-up with
//! its ground zero and the re-arm over the scripted bus, the operator's ground updates, re-zero fresh-only, the
//! calibrate's re-captured ground (M9 D7), freshness by changed bytes with a standby part going stale (D3), and the
//! bus-garbage guard: a burst read twice and believed only when both agree and lie in the part's operating range.

use control::commons::altitude_m;
use serde_json::json;

use super::{
    Bmp581, CHIP_ID, CONFIGURATION, CONVERSION_PERIOD_US, FIRST_CONVERSION_MS, GROUND_SAMPLE_MS, GROUND_SAMPLES,
    IDENTITY, ODR_CONFIG, Outcome, REZERO_WINDOW_US, Reading, Refusal, TEMP_DATA_XLSB,
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

/// One double read's two bursts.
type Pair = ([u8; 6], [u8; 6]);

/// A pair of bursts that agree.
const fn twice(frame: [u8; 6]) -> Pair {
    (frame, frame)
}

/// Read the part on a 10 ms tick from `start_us`, answering each read's two bursts with the next of `pairs`; the
/// outcomes.
fn poll(barometer: &mut Bmp581, start_us: u64, pairs: &[Pair]) -> Vec<Outcome> {
    let mock = MockBus::new();
    for (first, second) in pairs {
        mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], first)
            .expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], second);
    }
    let outcomes = (0u64..)
        .zip(pairs)
        .map(|(tick, _)| {
            barometer
                .read_fresh(&mut mock.clone(), start_us + tick * 10_000)
                .unwrap()
        })
        .collect();
    mock.done();
    outcomes
}

/// Whether an outcome is a fresh reading.
fn fresh(outcome: &Outcome) -> bool {
    matches!(outcome, Outcome::Fresh(_))
}

#[test]
fn each_conversion_is_fresh_once_on_a_tick_twice_as_fast() {
    // polled every 10 ms, a 50 Hz part shows each conversion on two reads: published on the first only
    let mut barometer = Bmp581::new(ADDRESS);
    let outcomes = poll(
        &mut barometer,
        0,
        &[
            twice(WARM),
            twice(WARM),
            twice(HIGHER),
            twice(HIGHER),
            twice(FINE),
            twice(FINE),
        ],
    );
    assert_eq!(
        outcomes,
        [
            Outcome::Fresh(Reading::decode(&WARM)),
            Outcome::Unchanged,
            Outcome::Fresh(Reading::decode(&HIGHER)),
            Outcome::Unchanged,
            Outcome::Fresh(Reading::decode(&FINE)),
            Outcome::Unchanged
        ]
    );
    // a sub-zero temperature is in range, decoded as it always was
    let cold = poll(&mut barometer, 60_000, &[twice(COLD)]);
    assert!(
        matches!(cold[..], [Outcome::Fresh(reading)] if reading.temperature == -10.0),
        "{cold:?}"
    );
}

#[test]
fn a_standby_part_goes_stale_after_three_conversion_periods() {
    // NEGATIVE: a part a reset left in standby returns its last sample forever; the firmware calls it unhealthy at
    // Stale, three conversion periods (60 ms) after the bytes last changed
    let mut barometer = Bmp581::new(ADDRESS);
    let outcomes = poll(&mut barometer, 1_000_000, &[twice(WARM); 9]);
    let first_stale = outcomes.iter().position(|outcome| *outcome == Outcome::Stale).unwrap();
    assert_eq!(first_stale, 6, "{outcomes:?}");
    assert!(outcomes[1..6].iter().all(|outcome| *outcome == Outcome::Unchanged));
    assert!(outcomes[6..].iter().all(|outcome| *outcome == Outcome::Stale));
    // the re-arm writes the mode again and the first new conversion is fresh at once
    let mock = MockBus::new();
    script_configure(&mock);
    barometer.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    let outcomes = poll(&mut barometer, 1_100_000, &[twice(WARM), twice(HIGHER)]);
    assert_eq!(outcomes[0], Outcome::Stale);
    assert!(fresh(&outcomes[1]));
}

#[test]
fn a_torn_pair_between_unchanged_reads_neither_resets_nor_advances_the_stale_clock() {
    // the bytes last changed at 0; the torn pair at 20 ms carries other bytes, which are not stored: the next read
    // of the part's own bytes is still Unchanged (not Fresh against the garbage), and Stale comes at 60 ms exactly as
    // with no torn pair at all
    let mut barometer = Bmp581::new(ADDRESS);
    let outcomes = poll(
        &mut barometer,
        0,
        &[
            twice(WARM),
            twice(WARM),
            (WARM, HIGHER),
            twice(WARM),
            twice(WARM),
            twice(WARM),
            twice(WARM),
        ],
    );
    assert!(fresh(&outcomes[0]));
    assert_eq!(
        outcomes[1..],
        [
            Outcome::Unchanged,
            Outcome::Torn,
            Outcome::Unchanged,
            Outcome::Unchanged,
            Outcome::Unchanged,
            Outcome::Stale
        ]
    );
    // torn pairs all the way: the clock still runs from the last judged change, so the first agreeing pair of the
    // part's old bytes after them is Stale at once, and a new conversion is Fresh at once
    let mut barometer = Bmp581::new(ADDRESS);
    let outcomes = poll(
        &mut barometer,
        0,
        &[twice(WARM), (WARM, HIGHER), (HIGHER, WARM), (FINE, WARM), (WARM, FINE)],
    );
    assert!(
        outcomes[1..].iter().all(|outcome| *outcome == Outcome::Torn),
        "{outcomes:?}"
    );
    assert_eq!(poll(&mut barometer, 60_000, &[twice(WARM)]), [Outcome::Stale]);
    assert!(fresh(&poll(&mut barometer, 70_000, &[twice(HIGHER)])[0]));
}

#[test]
fn a_conversion_between_the_two_bursts_is_read_on_the_next_tick() {
    // the pair straddles a conversion (old bytes, then new): torn, and the next tick reads the new one twice
    let mut barometer = Bmp581::new(ADDRESS);
    let outcomes = poll(&mut barometer, 0, &[twice(WARM), (WARM, HIGHER), twice(HIGHER)]);
    assert_eq!(outcomes[1], Outcome::Torn);
    assert_eq!(outcomes[2], Outcome::Fresh(Reading::decode(&HIGHER)));
}

/// 0xFFFFFF in both counts: 262 143.98 Pa at -1/65536 degrees C (the measured all-ones transfer).
const ALL_ONES: [u8; 6] = [0xFF; 6];

/// One byte repeated in the pressure: 0x1B1B1B, 27 756.4 Pa, at 25 degrees C.
const REPEATED: [u8; 6] = [0x00, 0x00, 0x19, 0x1B, 0x1B, 0x1B];

/// A plausible-looking tear: 94 577 Pa at 27.7 degrees C, in range, so only the second burst can catch it.
const TEAR: [u8; 6] = [0x33, 0xB3, 0x1B, 0x40, 0x5C, 0x5C];

/// A tear at -44 degrees C and 192 100 Pa.
const COLD_AND_HIGH: [u8; 6] = [0x00, 0x00, 0xD4, 0x00, 0x99, 0xBB];

/// A tear at 101 degrees C and 103 829 Pa.
const HOT: [u8; 6] = [0x00, 0x00, 0x65, 0x40, 0x65, 0x65];

/// 29 999 Pa at 25 degrees C: one pascal below the operating range.
const BELOW_RANGE: [u8; 6] = [0x00, 0x00, 0x19, 0xC0, 0x4B, 0x1D];

#[test]
fn bursts_that_differ_are_torn() {
    // NEGATIVE: one byte apart, the measured all-ones transfer, a plausible tear, either way round
    for pair in [
        (WARM, [0x00, 0x00, 0x19, 0x00, 0xF6, 0x63]),
        (WARM, ALL_ONES),
        (ALL_ONES, WARM),
        (TEAR, WARM),
        (WARM, TEAR),
        (WARM, [0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF]),
    ] {
        let mut barometer = Bmp581::new(ADDRESS);
        assert_eq!(poll(&mut barometer, 0, &[pair]), [Outcome::Torn], "{pair:02x?}");
        // nothing was judged: the first agreeing pair is the first read the judge sees, fresh
        assert!(fresh(&poll(&mut barometer, 10_000, &[twice(WARM)])[0]));
    }
}

#[test]
fn garbage_that_repeats_is_implausible() {
    // NEGATIVE: both bursts alike, outside 30 000..=125 000 Pa or -40..=85 degrees C
    for frame in [ALL_ONES, REPEATED, COLD_AND_HIGH, HOT, BELOW_RANGE] {
        let mut barometer = Bmp581::new(ADDRESS);
        poll(&mut barometer, 0, &[twice(WARM)]);
        assert_eq!(
            poll(&mut barometer, 10_000, &[twice(frame)]),
            [Outcome::Implausible(Reading::decode(&frame))],
            "{frame:02x?}"
        );
        // never judged, never stored: the part's own bytes next are Unchanged, not Fresh against the garbage
        assert_eq!(poll(&mut barometer, 20_000, &[twice(WARM)]), [Outcome::Unchanged]);
    }
    assert_eq!(Reading::decode(&ALL_ONES).pressure, 262_143.98);
    assert_eq!(Reading::decode(&REPEATED).pressure, 27_756.422);
    assert_eq!(Reading::decode(&BELOW_RANGE).pressure, 29_999.0);
    // a plausible tear that repeats is not caught by the range: the double read is what catches it
    assert!(fresh(&poll(&mut Bmp581::new(ADDRESS), 0, &[twice(TEAR)])[0]));
}

#[test]
fn the_operating_range_is_inclusive_at_both_ends() {
    let edges = [
        ([0x00, 0x00, 0x19, 0x00, 0x4C, 0x1D], true),  // 30 000 Pa
        ([0x00, 0x00, 0x19, 0x00, 0x12, 0x7A], true),  // 125 000 Pa
        ([0x00, 0x00, 0x19, 0x01, 0x12, 0x7A], false), // 125 000 + 1/64 Pa
        ([0x00, 0x00, 0xD8, 0x00, 0xF6, 0x62], true),  // -40 degrees C
        ([0x00, 0x00, 0x55, 0x00, 0xF6, 0x62], true),  // 85 degrees C
        ([0x01, 0x00, 0x55, 0x00, 0xF6, 0x62], false), // 85 + 1/65536 degrees C
        ([0xFF, 0xFF, 0xD7, 0x00, 0xF6, 0x62], false), // -40 - 1/65536 degrees C
    ];
    for (frame, plausible) in edges {
        let outcome = poll(&mut Bmp581::new(ADDRESS), 0, &[twice(frame)]);
        assert_eq!(fresh(&outcome[0]), plausible, "{frame:02x?}: {outcome:?}");
    }
}

#[test]
fn a_failed_burst_is_a_bus_fault_and_judges_nothing() {
    // NEGATIVE: a NAK on either burst is the transport's fault, never a torn, unchanged or stale sample
    let mut barometer = Bmp581::new(ADDRESS);
    poll(&mut barometer, 0, &[twice(WARM)]);
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &HIGHER).then_fail();
    assert!(matches!(
        barometer.read_fresh(&mut mock.clone(), 10_000),
        Err(Fault::Bus(_))
    ));
    mock.expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &HIGHER)
        .expect_write_read(ADDRESS, &[TEMP_DATA_XLSB], &HIGHER)
        .then_fail();
    assert!(matches!(
        barometer.read_fresh(&mut mock.clone(), 20_000),
        Err(Fault::Bus(_))
    ));
    mock.done();
    assert_eq!(poll(&mut barometer, 30_000, &[twice(WARM)]), [Outcome::Unchanged]);
}
