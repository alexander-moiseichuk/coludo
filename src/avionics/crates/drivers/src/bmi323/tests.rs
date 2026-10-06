//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMI323's unit tests: the dummy-byte offset, exact scaling in both signs, the 0x8000 start-up marker, the
//! bring-up and the re-arm over the scripted bus, and the INT1 silent verdict (counted only with a wire). Plan B's
//! test_bmi323.py frames are in the golden fixture drivers_bmi323.json as well.

use super::{
    ACC_DATA_X, Bmi323, CHIP_ID, CONFIGURATION, IDENTITY, INTERRUPT_SILENT_LIMIT, NO_SAMPLE, ROUTE_INTERRUPT,
    SOFT_RESET, SOFT_RESET_SETTLE_MS, START_UP_POLL_MS, START_UP_POLLS, Sample,
};
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's address in these tests.
const ADDRESS: u8 = 0x69;

/// A burst as the part sends it: two dummy bytes (deliberately not zero: reading from offset 0 must corrupt the
/// result), then the six words little-endian.
fn frame(words: [i16; 6]) -> [u8; 14] {
    let mut frame = [0u8; 14];
    frame[0] = 0xA5;
    frame[1] = 0x5A;
    for (index, word) in words.iter().enumerate() {
        frame[2 + 2 * index..4 + 2 * index].copy_from_slice(&word.to_le_bytes());
    }
    frame
}

/// The bytes one 16-bit register write sends: the register, then the value least significant byte first.
fn written((register, value): (u8, u16)) -> [u8; 3] {
    let [low, high] = value.to_le_bytes();
    [register, low, high]
}

/// Script the reset and the configuration.
fn script_configure(mock: &MockBus) {
    mock.expect_write(ADDRESS, &written(SOFT_RESET))
        .expect_delay_ms(SOFT_RESET_SETTLE_MS);
    for write in CONFIGURATION {
        mock.expect_write(ADDRESS, &written(write));
    }
}

/// Script one start-up look answered with this burst.
fn script_look(mock: &MockBus, burst: [u8; 14]) {
    mock.expect_delay_ms(START_UP_POLL_MS)
        .expect_write_read(ADDRESS, &[ACC_DATA_X], &burst);
}

#[test]
fn the_payload_starts_past_the_dummy_bytes() {
    let sample = Sample::decode(&frame([0, 0, 2048, 1000, -1000, 0]));
    assert_eq!(sample.accel, [0, 0, 2048]);
    assert_eq!(sample.gyro, [1000, -1000, 0]);
    assert_eq!(sample.accel_g(), [0.0, 0.0, 1.0], "2048 counts are exactly 1 g");
}

#[test]
fn the_gyro_keeps_its_sign_through_the_integer_conversion() {
    let sample = Sample::decode(&frame([0, 0, 2048, 1000, -1000, 0]));
    // raw x 2000 x 100 // 32768, floored: 1000 -> 6103, -1000 -> -6104 (Python's floor, not truncation)
    assert_eq!(sample.rate_centidegrees_per_second(), [6103, -6104, 0]);
    assert_eq!(sample.rate_degrees_per_second(), [61.035_156, -61.035_156, 0.0]);
    let one = Sample {
        accel: [0; 3],
        gyro: [1, -1, 0],
    };
    assert_eq!(one.rate_centidegrees_per_second(), [6, -7, 0]);
}

#[test]
fn the_rails_both_ways() {
    // -32768 is a legitimate -16 g rail on the accel, not a marker
    let sample = Sample::decode(&frame([32767, -32768, 0, 32767, -32768, 0]));
    let accel = sample.accel_g();
    assert!(15.9 < accel[0] && accel[0] < 16.0, "{accel:?}");
    assert_eq!(accel[1], -16.0);
    assert_eq!(sample.rate_centidegrees_per_second()[..2], [199_993, -200_000]);
    assert_eq!(sample.rate_degrees_per_second()[1], -2000.0);
}

#[test]
fn the_start_up_marker_is_any_gyro_axis_at_0x8000() {
    assert!(!Sample::decode(&frame([0, 0, 2048, NO_SAMPLE, NO_SAMPLE, NO_SAMPLE])).gyro_started());
    assert!(!Sample::decode(&frame([0, 0, 2048, 5, NO_SAMPLE, 5])).gyro_started());
    assert!(Sample::decode(&frame([0, 0, 2048, 5, -5, 5])).gyro_started());
    // NEGATIVE: the accel's -32768 rail does not hold the start-up wait
    assert!(Sample::decode(&frame([NO_SAMPLE, 0, 0, 5, -5, 5])).gyro_started());
}

#[test]
fn a_present_part_is_reset_configured_and_waited_for() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0xA5, 0x5A, IDENTITY]);
    script_configure(&mock);
    script_look(&mock, frame([0, 0, 2048, NO_SAMPLE, NO_SAMPLE, NO_SAMPLE]));
    script_look(&mock, frame([0, 0, 2048, 3, -2, 1]));
    Bmi323::new(ADDRESS, false)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap();
    mock.done();
}

#[test]
fn the_identity_is_the_third_byte_not_the_first() {
    // NEGATIVE: the dummy byte at offset 0 happens to be 0x43; the real identity behind it is 0x00
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY, 0x00, 0x00]);
    let fault = Bmi323::new(ADDRESS, false)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert_eq!(
        fault,
        Fault::Identity {
            expected: 0x43,
            read: 0x00
        }
    );
    mock.done();
}

#[test]
fn a_gyro_stuck_at_the_marker_fails_setup() {
    // NEGATIVE: a part stuck at 0x8000 fails setup instead of publishing a phantom -2000 dps rotation
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY]);
    script_configure(&mock);
    for _ in 0..START_UP_POLLS {
        script_look(&mock, frame([0, 0, 2048, NO_SAMPLE, NO_SAMPLE, NO_SAMPLE]));
    }
    let fault = Bmi323::new(ADDRESS, true)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert_eq!(fault, Fault::Timeout);
    mock.done();
}

#[test]
fn a_start_up_look_that_naks_fails_setup_at_once() {
    // NEGATIVE: plan B's try turns the NAK into False; the wait must not ride over it to the next look
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY]);
    script_configure(&mock);
    script_look(&mock, frame([0, 0, 2048, NO_SAMPLE, NO_SAMPLE, NO_SAMPLE]));
    script_look(&mock, frame([0; 6]));
    mock.then_fail();
    let fault = Bmi323::new(ADDRESS, true)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn a_clearing_read_that_naks_fails_setup() {
    // NEGATIVE: the route went in but the clearing read NAKed: plan B's setup answers False, and so does the port's
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY]);
    script_configure(&mock);
    script_look(&mock, frame([0, 0, 2048, 0, 0, 0]));
    for write in ROUTE_INTERRUPT {
        mock.expect_write(ADDRESS, &written(write));
    }
    mock.expect_write_read(ADDRESS, &[ACC_DATA_X], &frame([0; 6]))
        .then_fail();
    let fault = Bmi323::new(ADDRESS, true)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn a_wired_int1_is_routed_then_cleared() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY]);
    script_configure(&mock);
    script_look(&mock, frame([0, 0, 2048, 0, 0, 0]));
    for write in ROUTE_INTERRUPT {
        mock.expect_write(ADDRESS, &written(write));
    }
    mock.expect_write_read(ADDRESS, &[ACC_DATA_X], &frame([0; 6]));
    Bmi323::new(ADDRESS, true)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap();
    mock.done();
}

#[test]
fn the_rearm_restores_the_configuration_and_the_int1_route() {
    // no identity read (as plan B), the reset and configuration, the start-up wait, and the INT1 route plan B left
    let mock = MockBus::new();
    script_configure(&mock);
    script_look(&mock, frame([0, 0, 2048, 0, 0, 0]));
    for write in ROUTE_INTERRUPT {
        mock.expect_write(ADDRESS, &written(write));
    }
    mock.expect_write_read(ADDRESS, &[ACC_DATA_X], &frame([0; 6]));
    Bmi323::new(ADDRESS, true)
        .rearm(&mut mock.clone(), &mut mock.delay())
        .unwrap();
    mock.done();
}

#[test]
fn a_failed_rearm_is_reported() {
    // NEGATIVE: the part is gone -- the first write NAKs and nothing else is tried
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &written(SOFT_RESET)).then_fail();
    let fault = Bmi323::new(ADDRESS, false)
        .rearm(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)));
    mock.done();
}

#[test]
fn a_rearm_whose_start_up_look_naks_is_reported() {
    // NEGATIVE: the configuration went in, the first start-up look NAKed: reported at once (plan B logged it)
    let mock = MockBus::new();
    script_configure(&mock);
    script_look(&mock, frame([0; 6]));
    mock.then_fail();
    let fault = Bmi323::new(ADDRESS, false)
        .rearm(&mut mock.clone(), &mut mock.delay())
        .unwrap_err();
    assert!(matches!(fault, Fault::Bus(_)), "{fault:?}");
    mock.done();
}

#[test]
fn a_declared_line_that_goes_quiet_is_called_silent_once() {
    let mut imu = Bmi323::new(ADDRESS, true);
    assert_eq!(imu.interrupt_silent(), Some(false));
    let notes: Vec<bool> = (0..INTERRUPT_SILENT_LIMIT + 5).map(|_| imu.wake(0)).collect();
    assert_eq!(notes.iter().filter(|note| **note).count(), 1);
    assert!(notes[INTERRUPT_SILENT_LIMIT as usize - 1]);
    assert_eq!(imu.interrupt_silent(), Some(true));
    assert!(!imu.wake(1));
    assert_eq!(imu.interrupt_silent(), Some(false), "one edge resumes interrupt mode");
}

#[test]
fn a_part_with_no_wire_is_polled_by_design_and_never_silent() {
    // NEGATIVE: no int_pin -> no count, no note, and the operator view is None rather than true
    let mut imu = Bmi323::new(ADDRESS, false);
    assert!((0..100).all(|_| !imu.wake(0)));
    assert_eq!(imu.interrupt_silent(), None);
}

#[test]
fn the_probe_and_the_diagnosis() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY])
        .expect_write_read(ADDRESS, &[ACC_DATA_X], &frame([0, 0, 2048, 0, 0, 0]))
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, 0x50])
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, 0x50])
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY])
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, 0])
        .then_fail();
    let imu = Bmi323::new(ADDRESS, false);
    assert_eq!(imu.probe(&mut mock.clone()).unwrap().accel_g(), [0.0, 0.0, 1.0]);
    assert_eq!(
        imu.probe(&mut mock.clone()).unwrap_err(),
        Fault::Identity {
            expected: 0x43,
            read: 0x50
        }
    );
    assert_eq!(
        imu.diagnose(&mut mock.clone(), 0),
        "wrong device at i2c:0 0x69: id 0x50, expected 0x43"
    );
    assert_eq!(
        imu.diagnose(&mut mock.clone(), 0),
        "present and answering, but setup did not complete"
    );
    assert!(
        imu.diagnose(&mut mock.clone(), 0)
            .starts_with("no ack at i2c:0 0x69 -- bus")
    );
    mock.done();
}
