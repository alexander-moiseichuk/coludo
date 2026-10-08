//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMI323's unit tests: the dummy-byte offset, exact scaling in both signs, the 0x8000 start-up marker, the
//! bring-up and the re-arm over the scripted bus, the INT1 silent verdict (counted only with a wire), the STATUS
//! burst's freshness (the flag decides, the bytes correct it, a 0x8000 frame never fresh) and the `axes` map over
//! the push and the row alike. Plan B's test_bmi323.py frames are in the golden fixture drivers_bmi323.json as well.

use serde_json::json;

use super::{
    ACC_DATA_X, Bmi323, CHIP_ID, CONFIGURATION, Fresh, IDENTITY, INTERRUPT_SILENT_LIMIT, NO_SAMPLE, ROUTE_INTERRUPT,
    SOFT_RESET, SOFT_RESET_SETTLE_MS, START_UP_POLL_MS, START_UP_POLLS, STATUS, STATUS_FRAME_LENGTH, Sample,
};
use crate::axes::{Axes, AxesError};
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

/// A STATUS burst as the part sends it: two dummy bytes (not zero), the STATUS word little-endian, then the six data
/// words (accel x, y, z, gyro x, y, z).
fn status_burst(status: u16, words: [i16; 6]) -> [u8; STATUS_FRAME_LENGTH] {
    let mut burst = [0u8; STATUS_FRAME_LENGTH];
    burst[..2].copy_from_slice(&[0xA5, 0x5A]);
    burst[2..4].copy_from_slice(&status.to_le_bytes());
    burst[4..].copy_from_slice(&frame(words)[2..]);
    burst
}

/// STATUS with drdy_acc (bit 7) and drdy_gyr (bit 6) set, as measured on every burst whose data changed.
const BOTH_READY: u16 = 0x00C0;

/// STATUS with drdy_acc alone.
const ACCEL_READY: u16 = 0x0080;

/// STATUS with drdy_gyr alone.
const GYRO_READY: u16 = 0x0040;

/// A part at rest: 1 g on z, a small gyro bias.
const REST: [i16; 6] = [5, -7, 2048, 3, -2, 1];

/// [`REST`] one accel conversion later.
const ACCEL_MOVED: [i16; 6] = [6, -7, 2048, 3, -2, 1];

/// [`REST`] one gyro conversion later.
const GYRO_MOVED: [i16; 6] = [5, -7, 2048, 3, -2, 2];

/// Both halves new by their flags, nothing corrected.
const BOTH: Fresh = Fresh {
    accel: true,
    rate: true,
    accel_raced: false,
    rate_raced: false,
    accel_identical: false,
    rate_identical: false,
};

/// Read STATUS bursts in turn on one driver, each `(status, words)`, and return what each one judged.
fn read_in_turn(imu: &mut Bmi323, bursts: &[(u16, [i16; 6])]) -> Vec<Fresh> {
    let mock = MockBus::new();
    for (status, words) in bursts {
        mock.expect_write_read(ADDRESS, &[STATUS], &status_burst(*status, *words));
    }
    let verdicts = bursts
        .iter()
        .map(|_| imu.read_fresh(&mut mock.clone()).expect("a scripted burst").0)
        .collect();
    mock.done();
    verdicts
}

#[test]
fn the_status_burst_is_one_read_from_status_decoded_as_the_bare_one() {
    // the register 0x02, a repeated start, then 16 bytes: the mock refuses any other shape or length
    let mock = MockBus::new();
    mock.expect_write_read(
        ADDRESS,
        &[STATUS],
        &status_burst(BOTH_READY, [0, 0, 2048, 1000, -1000, 0]),
    );
    let (fresh, sample) = Bmi323::new(ADDRESS, true).read_fresh(&mut mock.clone()).unwrap();
    mock.done();
    assert_eq!(fresh, BOTH);
    // the dummy bytes and the STATUS word are skipped: the sample is the bare burst's, bit for bit
    assert_eq!(sample, Sample::decode(&frame([0, 0, 2048, 1000, -1000, 0])));
}

#[test]
fn each_flag_speaks_for_its_own_half_and_the_others_are_ignored() {
    // drdy_acc (bit 7) is the accel's, drdy_gyr (bit 6) the gyro's; drdy_temp (bit 5) and por_detected (bit 0) and
    // the high byte belong to neither
    let cases = [
        (BOTH_READY, true, true),
        (ACCEL_READY, true, false),
        (GYRO_READY, false, true),
        (0x00E1, true, true),
        // NEGATIVE: no data-ready, the temperature's and the power-on flag alone
        (0x0000, false, false),
        (0x0021, false, false),
        (0xFF00, false, false),
    ];
    for (status, accel, rate) in cases {
        let verdict = read_in_turn(&mut Bmi323::new(ADDRESS, false), &[(status, REST)]);
        assert_eq!(
            verdict,
            [Fresh {
                accel,
                rate,
                ..Fresh::default()
            }],
            "status 0x{status:04x}"
        );
    }
}

#[test]
fn a_raced_conversion_is_pushed_once() {
    // M9 stage 0: reading STATUS clears the flags, so a conversion landing between the STATUS word and the data
    // reads with its flag clear and its bytes new, and leaves its flag set for the next burst over the same bytes
    let mut imu = Bmi323::new(ADDRESS, true);
    let verdicts = read_in_turn(
        &mut imu,
        &[
            (BOTH_READY, REST),
            (0, ACCEL_MOVED),
            (BOTH_READY, ACCEL_MOVED),
            (0, ACCEL_MOVED),
        ],
    );
    assert_eq!(verdicts[0], BOTH);
    assert_eq!(
        verdicts[1],
        Fresh {
            accel: true,
            accel_raced: true,
            ..Fresh::default()
        }
    );
    // the late flag over the raced bytes is the duplicate (skipped); the gyro's flag over ITS same bytes is a
    // conversion that repeated them (pushed, counted identical)
    assert_eq!(
        verdicts[2],
        Fresh {
            rate: true,
            accel_identical: true,
            rate_identical: true,
            ..Fresh::default()
        }
    );
    assert_eq!(verdicts[3], Fresh::default());
}

#[test]
fn a_flag_with_changed_bytes_and_a_flag_alone_are_fresh() {
    let mut imu = Bmi323::new(ADDRESS, true);
    let verdicts = read_in_turn(
        &mut imu,
        &[(BOTH_READY, REST), (BOTH_READY, GYRO_MOVED), (BOTH_READY, GYRO_MOVED)],
    );
    assert_eq!(
        verdicts[1],
        Fresh {
            accel_identical: true,
            ..BOTH
        }
    );
    // NEGATIVE of the correction: identical bytes under a flag after a FLAGGED push are a new conversion
    assert_eq!(
        verdicts[2],
        Fresh {
            accel_identical: true,
            rate_identical: true,
            ..BOTH
        }
    );
}

#[test]
fn a_stale_part_is_not_fresh() {
    // NEGATIVE: no flag, the same bytes: a read between two conversions, or a part that stopped (the firmware's
    // STALE_LIMIT run turns it unhealthy)
    let mut imu = Bmi323::new(ADDRESS, true);
    let verdicts = read_in_turn(&mut imu, &[(BOTH_READY, REST), (0, REST), (0, REST), (0, REST)]);
    assert_eq!(verdicts[1..], [Fresh::default(); 3]);
}

#[test]
fn a_0x8000_frame_is_never_fresh_and_never_stored() {
    // NEGATIVE: a reset part's frame, any gyro axis at the marker, all three accel axes at it -- under flags, and
    // with bytes that changed
    let frames = [
        [NO_SAMPLE; 6],
        [5, -7, 2048, 3, NO_SAMPLE, 1],
        [NO_SAMPLE, NO_SAMPLE, NO_SAMPLE, 3, -2, 1],
    ];
    for words in frames {
        let mut imu = Bmi323::new(ADDRESS, true);
        let verdicts = read_in_turn(
            &mut imu,
            &[(BOTH_READY, REST), (BOTH_READY, words), (0, words), (0, REST)],
        );
        assert_eq!(verdicts[1..], [Fresh::default(); 3], "{words:?}");
    }
}

#[test]
fn one_accel_axis_at_0x8000_is_the_rail_and_fresh() {
    let mut imu = Bmi323::new(ADDRESS, true);
    let rail = [NO_SAMPLE, -7, 2048, 3, -2, 1];
    let verdicts = read_in_turn(
        &mut imu,
        &[(BOTH_READY, rail), (0, [NO_SAMPLE, NO_SAMPLE, 2048, 3, -2, 1])],
    );
    assert_eq!(verdicts[0], BOTH);
    // two axes on the rail and the third not: still a sample, its change under a clear flag a raced conversion
    assert!(verdicts[1].accel && verdicts[1].accel_raced);
}

#[test]
fn after_forget_and_after_a_rearm_a_burst_is_judged_by_its_flag() {
    let mut imu = Bmi323::new(ADDRESS, false);
    read_in_turn(&mut imu, &[(BOTH_READY, REST)]);
    imu.forget();
    // NEGATIVE: changed bytes under a clear flag are no raced conversion with nothing stored
    assert_eq!(read_in_turn(&mut imu, &[(0, ACCEL_MOVED)]), [Fresh::default()]);
    // the re-arm forgets as well
    let mock = MockBus::new();
    script_configure(&mock);
    script_look(&mock, frame(REST));
    imu.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    mock.done();
    assert_eq!(read_in_turn(&mut imu, &[(0, GYRO_MOVED)]), [Fresh::default()]);
    assert!(read_in_turn(&mut imu, &[(GYRO_READY, GYRO_MOVED)])[0].rate);
}

#[test]
fn the_axes_map_the_push_and_the_row_alike() {
    // the firmware maps the converted values (D10): the f32 push and the i32 centidegree row through one map. The
    // gyro counts are multiples of 512, whose centidegrees are exact, so a negated column equals the column of the
    // negated count (elsewhere the floor makes them differ by one centidegree, which is why counts are never mapped)
    let sample = Sample::decode(&frame([100, -200, 2048, 512, -1024, 1536]));
    for (text, accel, rate) in [
        ("+x+y+z", [100, -200, 2048], [512, -1024, 1536]),
        ("-y+x+z", [200, 100, 2048], [1024, 512, 1536]),
        ("+z+y-x", [2048, -200, -100], [1536, -1024, -512]),
    ] {
        let axes = Axes::from_config(&json!({"axes": text})).unwrap();
        let mapped = Sample { accel, gyro: rate };
        assert_eq!(axes.apply(sample.accel_g()), mapped.accel_g(), "{text}");
        assert_eq!(
            axes.apply(sample.rate_degrees_per_second()),
            mapped.rate_degrees_per_second(),
            "{text}"
        );
        assert_eq!(
            axes.apply(sample.rate_centidegrees_per_second()),
            mapped.rate_centidegrees_per_second(),
            "{text}"
        );
    }
    // no key is the identity
    assert_eq!(Axes::from_config(&json!({"driver": "bmi323"})), Ok(Axes::IDENTITY));
    // the flat bench copy's map: a board lying flat (1 g on the sensor's z) reads +1 g on the airframe's x
    assert_eq!(Axes::parse("+z+y-x").unwrap().apply(sample.accel_g())[0], 1.0);
}

#[test]
fn a_mirror_axes_is_refused_naming_the_key() {
    // NEGATIVE: two axes swapped without a sign is no mounting; the refusal the firmware's setup failure carries
    let refusal = Axes::from_config(&json!({"axes": "+y+x+z"})).unwrap_err();
    assert!(matches!(refusal, AxesError::Mirror(_)));
    assert!(refusal.to_string().starts_with("axes '+y+x+z'"), "{refusal}");
}
