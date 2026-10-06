//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMM350's unit tests: the 24-bit decode in both signs, the OTP power-off table over the scripted bus, and the
//! OPERATOR CALIBRATION that CC drives, scenario by scenario as test_bmm350.py runs them: a partial turn, a still
//! board and a dead part are REFUSED; an ellipse and a full circle are ACCEPTED, and the corrected axes give the
//! heading back. The same scenarios run through plan B in the golden fixture drivers_bmm350.json.

use super::{
    Bmm350, CHIP_ID, CONFIGURATION, Calibration, Field, IDENTITY, MAG_X_XLSB, NEEDED_SECTORS, ProbeFailure, Step,
    not_measuring,
};
use crate::axes::{Axes, AxesError};
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's address in these tests.
const ADDRESS: u8 = 0x15;

/// A burst as the part sends it: two dummy bytes, then x, y, z and a temperature, 24-bit little-endian.
fn frame(x: i32, y: i32, z: i32) -> [u8; 14] {
    let mut frame = [0u8; 14];
    frame[0] = 0xA5;
    frame[1] = 0x5A;
    for (index, value) in [x, y, z, 0].iter().enumerate() {
        let bytes = value.to_le_bytes();
        frame[2 + 3 * index..5 + 3 * index].copy_from_slice(&bytes[..3]);
    }
    frame
}

/// Feed the driver a sweep of `degrees` of a circle about a hard-iron centre (test_bmm350._turn).
fn turn(magnetometer: &mut Bmm350, degrees: i32, centre: (f64, f64), radius: (f64, f64), step: usize) {
    for angle in (0..degrees).step_by(step) {
        let radians = f64::from(angle).to_radians();
        let field = Field {
            x: (centre.0 + radius.0 * radians.cos()) as i32,
            y: (centre.1 + radius.1 * radians.sin()) as i32,
            z: 700,
        };
        magnetometer.observe(&field);
    }
}

/// test_bmm350.py's hard iron: a circle of radius 4500 about (7500, -4500).
const CENTRE: (f64, f64) = (7500.0, -4500.0);

/// test_bmm350.py's circle radius.
const ROUND: (f64, f64) = (4500.0, 4500.0);

#[test]
fn the_axes_are_24_bit_twos_complement_both_signs() {
    // NEGATIVE half: a magnetometer reads negative on half of every turn; an unsigned decode would be wrong there
    let field = Field::decode(&frame(-12345, 6789, 700));
    assert_eq!(
        field,
        Field {
            x: -12345,
            y: 6789,
            z: 700
        }
    );
    let rails = Field::decode(&frame(-0x80_0000, 0x7F_FFFF, -1));
    assert_eq!(
        rails,
        Field {
            x: -0x80_0000,
            y: 0x7F_FFFF,
            z: -1
        }
    );
}

#[test]
fn setup_powers_the_otp_off_before_asking_for_samples() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY]);
    for step in CONFIGURATION {
        match step {
            Step::Write { register, value } => mock.expect_write(ADDRESS, &[register, value]),
            Step::Wait(milliseconds) => mock.expect_delay_ms(milliseconds),
        };
    }
    Bmm350::new(ADDRESS)
        .setup(&mut mock.clone(), &mut mock.delay())
        .unwrap();
    mock.done();
    // the OTP power-off (OTP_CMD_REG = 0x80) is in the table, after the reset, before normal mode
    let writes: Vec<(u8, u8)> = CONFIGURATION
        .iter()
        .filter_map(|step| match step {
            Step::Write { register, value } => Some((*register, *value)),
            Step::Wait(_) => None,
        })
        .collect();
    assert_eq!(
        writes,
        [(0x7E, 0xB6), (0x50, 0x80), (0x04, 0x24), (0x06, 0x02), (0x06, 0x01)]
    );
}

#[test]
fn another_part_is_refused_and_the_rearm_is_the_table_alone() {
    // NEGATIVE: the dummy byte is not the identity
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY, 0, 0x43]);
    assert_eq!(
        Bmm350::new(ADDRESS).setup(&mut mock.clone(), &mut mock.delay()),
        Err(Fault::Identity {
            expected: 0x33,
            read: 0x43
        })
    );
    // the re-arm runs the table with no identity read, and keeps the calibration
    for step in CONFIGURATION {
        match step {
            Step::Write { register, value } => mock.expect_write(ADDRESS, &[register, value]),
            Step::Wait(milliseconds) => mock.expect_delay_ms(milliseconds),
        };
    }
    let mut magnetometer = Bmm350::new(ADDRESS);
    let calibration = Calibration {
        centre_x: 1,
        centre_y: 2,
        radius_x: 3,
        radius_y: 4,
    };
    magnetometer.set_calibration(calibration);
    magnetometer.rearm(&mut mock.clone(), &mut mock.delay()).unwrap();
    assert_eq!(magnetometer.calibration(), Some(calibration));
    mock.done();
}

#[test]
fn nothing_seen_asks_for_the_turn() {
    let fresh = Bmm350::new(ADDRESS);
    assert_eq!(
        fresh.instruction(),
        "turn the airframe through a full circle, LEVEL, then calibrate (no field seen yet)"
    );
    assert_eq!(
        fresh.calibrate(),
        Err(fresh.instruction()),
        "no samples must not calibrate"
    );
    assert!(!fresh.calibrated());
}

#[test]
fn a_quarter_turn_is_refused_on_coverage_however_round_it_looks() {
    // swept back and forth past the sample floor, so the COVERAGE gate is what refuses it
    let mut quarter = Bmm350::new(ADDRESS);
    for _ in 0..3 {
        turn(&mut quarter, 90, CENTRE, ROUND, 1);
    }
    assert!(quarter.seen() >= 100);
    let span = quarter.span().unwrap();
    assert!(
        (span.x_span() - span.y_span()).abs() < 150,
        "the arc must look ROUND by its spans"
    );
    let refusal = quarter.calibrate().unwrap_err();
    assert!(refusal.contains("sectors"), "{refusal}");
    assert!(quarter.covered() < NEEDED_SECTORS);
}

#[test]
fn a_still_board_covers_nothing() {
    // NEGATIVE: +/-100 counts of noise must not scatter into octants (it once reached 8 of 8)
    let mut still = Bmm350::new(ADDRESS);
    for index in 0..400 {
        still.observe(&Field {
            x: 13700 + (index * 37) % 201 - 100,
            y: 16780 + (index * 53) % 201 - 100,
            z: 700,
        });
    }
    assert_eq!(still.covered(), 0);
    assert!(still.calibrate().is_err());
}

#[test]
fn a_dead_part_is_caught_by_its_span() {
    let mut dead = Bmm350::new(ADDRESS);
    for _ in 0..200 {
        dead.observe(&Field { x: 0, y: 0, z: 700 });
    }
    let refusal = dead.calibrate().unwrap_err();
    assert_eq!(
        refusal,
        "no field change seen (x span 0, y span 0) -- is the magnetometer alive?"
    );
}

#[test]
fn an_elliptical_field_is_corrected_not_refused() {
    let mut squashed = Bmm350::new(ADDRESS);
    turn(&mut squashed, 360, CENTRE, (6000.0, 3000.0), 2);
    turn(&mut squashed, 360, CENTRE, (6000.0, 3000.0), 2);
    assert!(
        squashed.calibrate().is_ok(),
        "a 2:1 soft iron is what the calibration exists to correct"
    );
}

#[test]
fn a_full_circle_calibrates_and_the_heading_comes_back() {
    let mut good = Bmm350::new(ADDRESS);
    turn(&mut good, 360, CENTRE, ROUND, 2);
    turn(&mut good, 360, CENTRE, ROUND, 2);
    assert_eq!(good.covered(), 8);
    let calibration = good.calibrate().unwrap();
    assert!((calibration.centre_x - 7500).abs() <= 100 && (calibration.centre_y + 4500).abs() <= 100);
    assert!((calibration.radius_x - 4500).abs() <= 100 && (calibration.radius_y - 4500).abs() <= 100);
    assert!(
        !good.calibrated(),
        "calibrate computes; the firmware persists, then puts it in force"
    );
    good.set_calibration(calibration);
    assert!(good.calibrated() && good.instruction().is_empty());
    for degrees in [0.0_f64, 37.0, 90.0, 154.0, 180.0, 271.0, 330.0] {
        let radians = degrees.to_radians();
        let corrected = good.observe(&Field {
            x: (7500.0 + 4500.0 * radians.cos()) as i32,
            y: (-4500.0 + 4500.0 * radians.sin()) as i32,
            z: 700,
        });
        let recovered = f64::from(corrected[1]).atan2(f64::from(corrected[0])).to_degrees();
        let error = ((recovered - degrees + 180.0).rem_euclid(360.0) - 180.0).abs();
        assert!(error < 2.0, "heading {degrees} recovered as {recovered}");
        assert_eq!(corrected[2], 700, "z passes raw");
    }
}

#[test]
fn a_turned_part_calibrates_in_the_sensor_frame_and_publishes_in_the_airframe() {
    // the same level turn, read by a part mounted per its marks and by one turned 90 degrees about z ('-y+x+z')
    let turned = Axes::parse("-y+x+z").unwrap();
    let mut marks = Bmm350::new(ADDRESS);
    let mut mounted = Bmm350::new(ADDRESS).mounted(turned).unwrap();
    assert_eq!(mounted.axes(), turned);
    for magnetometer in [&mut marks, &mut mounted] {
        turn(magnetometer, 360, CENTRE, ROUND, 2);
        turn(magnetometer, 360, CENTRE, ROUND, 2);
    }
    // the evidence and the calibration are the SENSOR's: the same blob either way, so plan B's (which has no axes)
    // stays valid after a reflash in both directions
    assert_eq!(mounted.covered(), 8);
    let calibration = mounted.calibrate().unwrap();
    assert_eq!(calibration, marks.calibrate().unwrap());
    assert_eq!(Calibration::from_bytes(&calibration.to_bytes()), Some(calibration));
    // only the published vector is turned, raw and corrected alike, and it is turned AFTER the correction
    let field = Field {
        x: 9000,
        y: -2000,
        z: 700,
    };
    assert_eq!(mounted.observe(&field), turned.apply(marks.observe(&field)));
    marks.set_calibration(calibration);
    mounted.set_calibration(calibration);
    let corrected = marks.observe(&field);
    assert_eq!(mounted.observe(&field), [-corrected[1], corrected[0], corrected[2]]);
}

#[test]
fn an_upside_down_part_keeps_its_level_plane() {
    // flipped over about x ('+x-y-z'): the sensor's x/y plane is still level, so the turn covers it as usual
    let flipped = Axes::parse("+x-y-z").unwrap();
    let mut magnetometer = Bmm350::new(ADDRESS).mounted(flipped).unwrap();
    // two laps: the first settles the centre estimate, whose moves restart the coverage
    turn(&mut magnetometer, 360, CENTRE, ROUND, 2);
    turn(&mut magnetometer, 360, CENTRE, ROUND, 2);
    assert_eq!(magnetometer.covered(), 8);
    assert_eq!(magnetometer.observe(&Field { x: 1, y: 2, z: 3 }), [1, -2, -3]);
}

#[test]
fn a_part_mounted_on_its_side_is_refused_because_its_turn_never_covers() {
    // NEGATIVE: '+x+z-y' puts the sensor's z along the airframe's y -- refused at construction, naming the map
    let on_its_side = Axes::parse("+x+z-y").unwrap();
    let refusal = Bmm350::new(ADDRESS).mounted(on_its_side).unwrap_err();
    assert_eq!(refusal, AxesError::Tilted("+x+z-y".to_string()));
    assert!(refusal.to_string().contains("tilts the sensor's z off the vertical"));
    assert!(Bmm350::new(ADDRESS).mounted(Axes::parse("+z+y-x").unwrap()).is_err());
    // why: a level turn of that part sweeps the sensor's x/z plane, its y barely moves, and the coverage check would
    // refuse every calibration the operator ever tried
    let mut magnetometer = Bmm350::new(ADDRESS);
    for angle in (0..720).step_by(2) {
        let radians = f64::from(angle).to_radians();
        magnetometer.observe(&Field {
            x: (CENTRE.0 + ROUND.0 * radians.cos()) as i32,
            y: 700,
            z: (CENTRE.1 + ROUND.1 * radians.sin()) as i32,
        });
    }
    assert!(magnetometer.covered() < NEEDED_SECTORS);
    assert!(magnetometer.calibrate().unwrap_err().starts_with("refused"));
}

#[test]
fn an_uncorrected_hard_iron_is_a_heading_dependent_error() {
    // why a learned track offset cannot absorb it: the error varies with heading
    let mut raw = Bmm350::new(ADDRESS);
    let errors: Vec<f64> = [0.0_f64, 90.0, 180.0, 270.0]
        .iter()
        .map(|degrees| {
            let radians = degrees.to_radians();
            let published = raw.observe(&Field {
                x: (7500.0 + 4500.0 * radians.cos()) as i32,
                y: (-4500.0 + 4500.0 * radians.sin()) as i32,
                z: 700,
            });
            (f64::from(published[1]).atan2(f64::from(published[0])).to_degrees() - degrees + 180.0).rem_euclid(360.0)
                - 180.0
        })
        .collect();
    let spread = errors.iter().cloned().fold(f64::MIN, f64::max) - errors.iter().cloned().fold(f64::MAX, f64::min);
    assert!(spread > 20.0, "{errors:?}");
}

#[test]
fn a_refused_recalibration_keeps_the_old_one_and_says_refused() {
    let mut recalibrating = Bmm350::new(ADDRESS);
    let restored = Calibration {
        centre_x: 0,
        centre_y: 0,
        radius_x: 100,
        radius_y: 100,
    };
    recalibrating.set_calibration(restored);
    // a real swing on x and y, but only three octants: 0, 2 and 6
    for (x, y) in [
        (-4000, -4000),
        (4000, -4000),
        (4000, 4000),
        (-4000, 4000),
        (4000, 3000),
        (-3900, 3990),
    ] {
        recalibrating.observe(&Field { x, y, z: 0 });
    }
    for _ in 0..200 {
        recalibrating.observe(&Field { x: 4000, y: 3000, z: 0 });
    }
    let answer = recalibrating.calibrate().unwrap_err();
    assert!(answer.starts_with("refused -- only"), "{answer}");
    assert_eq!(recalibrating.calibration(), Some(restored));
}

#[test]
fn too_few_samples_on_a_restored_calibration_answers_an_empty_string() {
    // plan B's answer to the letter: the instruction, which is '' once a calibration is in force
    let mut restored = Bmm350::new(ADDRESS);
    restored.set_calibration(Calibration {
        centre_x: 0,
        centre_y: 0,
        radius_x: 100,
        radius_y: 100,
    });
    restored.observe(&Field { x: 1, y: 1, z: 1 });
    assert_eq!(restored.calibrate(), Err(String::new()));
}

#[test]
fn the_nvs_blob_is_four_little_endian_words() {
    let calibration = Calibration {
        centre_x: 7500,
        centre_y: -4500,
        radius_x: 4500,
        radius_y: 1,
    };
    let blob = calibration.to_bytes();
    assert_eq!(&blob[..8], &[0x4C, 0x1D, 0, 0, 0x6C, 0xEE, 0xFF, 0xFF]);
    assert_eq!(Calibration::from_bytes(&blob), Some(calibration));
    // NEGATIVE: a short or long blob is not a calibration
    assert_eq!(Calibration::from_bytes(&blob[..15]), None);
    assert_eq!(Calibration::from_bytes(&[0; 17]), None);
}

#[test]
fn the_correction_floors_and_a_zero_radius_divides_by_one() {
    let calibration = Calibration {
        centre_x: 10,
        centre_y: 0,
        radius_x: 3,
        radius_y: 0,
    };
    // (9 - 10) x 1000 // 3 = -334 (floored, not -333); y over a zero radius divides by 1
    assert_eq!(calibration.correct(&Field { x: 9, y: 2, z: 5 }), [-334, 2000, 5]);
    // a corrupt negative radius floors the Python way: -1000 // -3 = 333
    let corrupt = Calibration {
        radius_x: -3,
        ..calibration
    };
    assert_eq!(corrupt.correct(&Field { x: 9, y: 0, z: 0 })[0], 333);
    assert_eq!(corrupt.correct(&Field { x: 11, y: 0, z: 0 })[0], -334);
}

#[test]
fn the_probe_refuses_a_part_that_is_not_measuring() {
    assert_eq!(
        not_measuring(&Field { x: 0, y: 0, z: 0 }, [0, 0, 0]),
        Some("all axes zero -- the part is not measuring")
    );
    let otp_powered = Field {
        x: 0x7F_7F7F,
        y: 0x7F_7F7F,
        z: 0x7F_7F7F,
    };
    assert!(
        not_measuring(&otp_powered, [1, 1, 1])
            .unwrap()
            .contains("OTP still powered")
    );
    assert_eq!(not_measuring(&Field { x: 5, y: 0, z: 0 }, [5, 0, 0]), None);
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY])
        .expect_write_read(ADDRESS, &[MAG_X_XLSB], &frame(0x7F_7F7F, 0x7F_7F7F, 0x7F_7F7F))
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY])
        .expect_write_read(ADDRESS, &[MAG_X_XLSB], &frame(1200, -300, 700))
        .expect_write_read(ADDRESS, &[CHIP_ID], &[0, 0, IDENTITY])
        .then_fail();
    let mut magnetometer = Bmm350::new(ADDRESS);
    assert!(matches!(
        magnetometer.probe(&mut mock.clone()),
        Err(ProbeFailure::NotMeasuring(_))
    ));
    assert_eq!(magnetometer.probe(&mut mock.clone()), Ok([1200, -300, 700]));
    assert_eq!(
        magnetometer.seen(),
        2,
        "the probe's reads feed the evidence, as plan B's _read does"
    );
    assert!(matches!(
        magnetometer.probe(&mut mock.clone()),
        Err(ProbeFailure::Fault(Fault::Bus(_)))
    ));
    mock.done();
}
