//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BMM350's unit tests: the 24-bit decode in both signs, the OTP power-off table over the scripted bus, and the
//! OPERATOR CALIBRATION that CC drives, scenario by scenario as test_bmm350.py runs them: a partial turn, a still
//! board and a dead part are REFUSED; an ellipse and a full circle are ACCEPTED, and the corrected axes give the
//! heading back. The same scenarios run through plan B in the golden fixture drivers_bmm350.json. And the port's own
//! (avionics-port.md M9): the 12.5 Hz averaging-8 rate byte (D2), freshness by changed axis bytes with a suspended
//! part going stale (D3), and the calibration computed without being applied, applied from its value, and restored
//! from its NVS blob (D9), its text formatted from the values inspect carries.

use super::{
    Bmm350, CALIBRATION_BYTES, CHIP_ID, CONFIGURATION, CONVERSION_PERIOD_US, Calibration, Change, Field, IDENTITY,
    MAG_X_XLSB, NEEDED_SECTORS, NO_DATA, PMU_CMD_AGGR_SET, POLL_PERIOD_MS, ProbeFailure, SECTORS, Step,
    format_instruction, not_measuring,
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
    // the OTP power-off (OTP_CMD_REG = 0x80) is in the table, after the reset, before normal mode; the rate is
    // M9 D2's 12.5 Hz averaging 8 (0x37), plan B's 0x24 the one byte that differs
    let writes: Vec<(u8, u8)> = CONFIGURATION
        .iter()
        .filter_map(|step| match step {
            Step::Write { register, value } => Some((*register, *value)),
            Step::Wait(_) => None,
        })
        .collect();
    assert_eq!(
        writes,
        [(0x7E, 0xB6), (0x50, 0x80), (0x04, 0x37), (0x06, 0x02), (0x06, 0x01)]
    );
}

#[test]
fn the_rate_byte_is_12_5_hz_averaging_8() {
    // bmm350_defs.h: the ODR in bits 3:0 (BMM350_DATA_RATE_12_5HZ = 7; 100 Hz = 4), the averaging in bits 5:4
    // (BMM350_AVERAGING_8 = 3; 4 is 2): 0x37, where plan B wrote 0x24
    let rate = CONFIGURATION
        .iter()
        .find_map(|step| match step {
            Step::Write {
                register: PMU_CMD_AGGR_SET,
                value,
            } => Some(*value),
            _ => None,
        })
        .unwrap();
    assert_eq!(rate & 0x0F, 0x07, "12.5 Hz");
    assert_eq!((rate >> 4) & 0x03, 0x03, "averaging 8");
    assert_eq!(rate & 0xC0, 0, "the reserved bits stay clear");
    assert_eq!(CONVERSION_PERIOD_US, 80_000);
    assert_eq!(
        u64::from(POLL_PERIOD_MS) * 2_000,
        CONVERSION_PERIOD_US,
        "two reads a conversion"
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
    // the probe's real sample feeds the evidence, as plan B's _read does; its 0x7F7F7F one does not (plan B's did)
    assert_eq!(magnetometer.seen(), 1);
    assert_eq!(
        magnetometer.span().map(|span| (span.x_maximum, span.y_maximum)),
        Some((1200, -300))
    );
    assert!(matches!(
        magnetometer.probe(&mut mock.clone()),
        Err(ProbeFailure::Fault(Fault::Bus(_)))
    ));
    mock.done();
}

/// A magnetometer that has seen a full level circle twice: ready to calibrate.
fn turned() -> Bmm350 {
    let mut magnetometer = Bmm350::new(ADDRESS);
    turn(&mut magnetometer, 360, CENTRE, ROUND, 2);
    turn(&mut magnetometer, 360, CENTRE, ROUND, 2);
    magnetometer
}

#[test]
fn a_calibration_is_computed_without_being_applied_then_applied_from_its_value() {
    // D9: the owner answers the computed calibration; only after the executor persisted it is it put in force
    let mut magnetometer = turned();
    let computed = magnetometer.calibrate().unwrap();
    assert_eq!(magnetometer.calibration(), None, "computing applies nothing");
    assert!(!magnetometer.calibrated());
    let raw = magnetometer.observe(&Field {
        x: 12_000,
        y: -4500,
        z: 700,
    });
    assert_eq!(raw, [12_000, -4500, 700], "the axes stay raw until it is applied");
    // the value travels (to NVS and back) as its 16 bytes, and is applied from them
    let persisted = Calibration::from_bytes(&computed.to_bytes()).unwrap();
    magnetometer.set_calibration(persisted);
    assert_eq!(magnetometer.calibration(), Some(computed));
    assert_eq!(
        magnetometer.observe(&Field {
            x: 12_000,
            y: -4500,
            z: 700
        }),
        [1000, 0, 700]
    );
    assert_eq!(magnetometer.instruction(), "");
}

#[test]
fn a_refused_calibration_has_nothing_to_persist() {
    // NEGATIVE: a still board -- the answer is the refusal text, no Calibration value exists to write to NVS, and the
    // calibration in force (here none) is untouched
    let mut still = Bmm350::new(ADDRESS);
    for index in 0..400 {
        still.observe(&Field {
            x: 13_700 + (index * 37) % 201 - 100,
            y: 16_780 + (index * 53) % 201 - 100,
            z: 700,
        });
    }
    let refusal = still.calibrate().unwrap_err();
    assert!(refusal.starts_with("no field change seen"), "{refusal}");
    assert_eq!(still.calibration(), None);
}

#[test]
fn a_16_byte_blob_restores_and_a_short_one_leaves_the_axes_raw() {
    let blob = Calibration {
        centre_x: 7500,
        centre_y: -4500,
        radius_x: 4500,
        radius_y: 4500,
    }
    .to_bytes();
    let mut magnetometer = Bmm350::new(ADDRESS);
    let restored = magnetometer.restore(&blob).unwrap();
    assert_eq!(magnetometer.calibration(), Some(restored));
    assert_eq!(restored.centre_x, 7500);
    assert_eq!(
        magnetometer.observe(&Field {
            x: 12_000,
            y: -4500,
            z: 700
        }),
        [1000, 0, 700]
    );
    // NEGATIVE: a short blob (and a long one) is no calibration: nothing restored, the axes stay raw
    for length in [0, 4, CALIBRATION_BYTES - 1, CALIBRATION_BYTES + 1] {
        let mut raw = Bmm350::new(ADDRESS);
        let mut other = blob.to_vec();
        other.resize(length, 0);
        assert_eq!(raw.restore(&other), None, "{length} bytes");
        assert!(!raw.calibrated());
        assert_eq!(
            raw.observe(&Field {
                x: 12_000,
                y: -4500,
                z: 700
            }),
            [12_000, -4500, 700]
        );
    }
}

#[test]
fn the_instruction_formatter_is_the_live_part_s_instruction() {
    // the executor formats from the atomics (calibrated, sectors, samples): the same text as the part's own, in each
    // of plan B's three states and as the turn progresses
    assert_eq!(
        format_instruction(false, 0, 0),
        "turn the airframe through a full circle, LEVEL, then calibrate (no field seen yet)"
    );
    assert_eq!(
        format_instruction(false, 3, 120),
        format!(
            "turn the airframe slowly through a FULL CIRCLE, keeping it level -- 3 of {SECTORS} sectors covered, 120 \
             samples"
        )
    );
    assert_eq!(format_instruction(true, 8, 500), "");
    let mut magnetometer = Bmm350::new(ADDRESS);
    let pure = |magnetometer: &Bmm350| {
        format_instruction(magnetometer.calibrated(), magnetometer.covered(), magnetometer.seen())
    };
    assert_eq!(pure(&magnetometer), magnetometer.instruction());
    for degrees in [10, 90, 200, 360] {
        turn(&mut magnetometer, degrees, CENTRE, ROUND, 5);
        assert_eq!(pure(&magnetometer), magnetometer.instruction(), "{degrees} degrees");
    }
    magnetometer.set_calibration(magnetometer.calibrate().unwrap_or(Calibration {
        centre_x: 0,
        centre_y: 0,
        radius_x: 1,
        radius_y: 1,
    }));
    assert_eq!(pure(&magnetometer), "");
    assert_eq!(pure(&magnetometer), magnetometer.instruction());
}

/// Read the part every [`POLL_PERIOD_MS`] from `start_us`, answering each read with the next of `fields`; the
/// verdicts, each fresh field observed as the firmware observes it.
fn poll(magnetometer: &mut Bmm350, start_us: u64, fields: &[(i32, i32, i32)]) -> Vec<Change> {
    let mock = MockBus::new();
    for (x, y, z) in fields {
        mock.expect_write_read(ADDRESS, &[MAG_X_XLSB], &frame(*x, *y, *z));
    }
    let verdicts = (0u64..)
        .zip(fields)
        .map(|(tick, _)| {
            let now_us = start_us + tick * u64::from(POLL_PERIOD_MS) * 1000;
            let (change, field) = magnetometer.read_fresh(&mut mock.clone(), now_us).unwrap();
            if change == Change::Fresh {
                magnetometer.observe(&field);
            }
            change
        })
        .collect();
    mock.done();
    verdicts
}

#[test]
fn each_conversion_is_fresh_once_and_observed_once() {
    use Change::{Fresh, Unchanged};
    // polled every 40 ms, a 12.5 Hz part shows each conversion on two reads
    let mut magnetometer = Bmm350::new(ADDRESS);
    let fields = [
        (1200, -300, 700),
        (1200, -300, 700),
        (1210, -300, 700),
        (1210, -300, 700),
        (1210, -300, 701),
    ];
    assert_eq!(
        poll(&mut magnetometer, 0, &fields),
        [Fresh, Unchanged, Fresh, Unchanged, Fresh]
    );
    assert_eq!(
        magnetometer.seen(),
        3,
        "the evidence counts conversions, never a read twice"
    );
}

#[test]
fn only_the_axis_bytes_decide() {
    // NEGATIVE: the dummy bytes and the temperature may change while the axes do not -- no new field
    let mut magnetometer = Bmm350::new(ADDRESS);
    let mock = MockBus::new();
    let first = frame(1200, -300, 700);
    let mut second = first;
    second[0] = 0x00;
    second[11..].copy_from_slice(&[0x11, 0x22, 0x33]);
    mock.expect_write_read(ADDRESS, &[MAG_X_XLSB], &first)
        .expect_write_read(ADDRESS, &[MAG_X_XLSB], &second);
    assert_eq!(magnetometer.read_fresh(&mut mock.clone(), 0).unwrap().0, Change::Fresh);
    assert_eq!(
        magnetometer.read_fresh(&mut mock.clone(), 40_000).unwrap().0,
        Change::Unchanged
    );
    mock.done();
}

#[test]
fn a_suspended_part_goes_stale_after_three_conversion_periods() {
    // NEGATIVE: a reset leaves the part SUSPENDED, answering its last sample forever; the firmware calls it unhealthy
    // at Stale, three conversion periods (240 ms) after the axes last changed
    let mut magnetometer = Bmm350::new(ADDRESS);
    let verdicts = poll(&mut magnetometer, 5_000_000, &[(1200, -300, 700); 9]);
    let first_stale = verdicts.iter().position(|change| *change == Change::Stale).unwrap();
    let periods = u64::try_from(first_stale).unwrap() * u64::from(POLL_PERIOD_MS) * 1000 / CONVERSION_PERIOD_US;
    assert_eq!(periods, 3, "{verdicts:?}");
    assert!(
        verdicts[1..first_stale]
            .iter()
            .all(|change| *change == Change::Unchanged)
    );
    assert!(verdicts[first_stale..].iter().all(|change| *change == Change::Stale));
    assert_eq!(magnetometer.seen(), 1, "a stale field is never observed");
    // the re-arm brings it back: the first new conversion is fresh at once
    assert_eq!(
        poll(&mut magnetometer, 6_000_000, &[(1200, -300, 700), (1201, -300, 700)]),
        [Change::Stale, Change::Fresh]
    );
}

/// The burst a re-armed part answers until its first conversion lands: every axis at the "no sample" pattern.
fn no_data() -> (i32, i32, i32) {
    (NO_DATA, NO_DATA, NO_DATA)
}

#[test]
fn the_no_data_pattern_after_a_rearm_is_neither_fresh_nor_observed() {
    use Change::{Fresh, Unchanged};
    // MEASURED on the board: after a re-arm the axes read 0x7F7F7F until the first conversion. Changed bytes alone
    // called it fresh, and its observe put 8 355 711 into the span for the rest of the boot
    let mut magnetometer = Bmm350::new(ADDRESS);
    let outcomes = poll(
        &mut magnetometer,
        0,
        &[
            (1200, -300, 700),
            no_data(),
            no_data(),
            (1210, -300, 700),
            (1210, -300, 700),
        ],
    );
    assert_eq!(outcomes, [Fresh, Unchanged, Unchanged, Fresh, Unchanged]);
    assert_eq!(magnetometer.seen(), 2, "only the two real conversions are evidence");
    let span = magnetometer.span().unwrap();
    assert_eq!((span.x_maximum, span.y_maximum), (1210, -300));
    // NEGATIVE: the pattern was not stored, so the real sample from before it, read again, is no change
    let mut magnetometer = Bmm350::new(ADDRESS);
    let outcomes = poll(&mut magnetometer, 0, &[(1200, -300, 700), no_data(), (1200, -300, 700)]);
    assert_eq!(outcomes, [Fresh, Unchanged, Unchanged]);
}

#[test]
fn a_field_that_is_no_sample_is_never_evidence() {
    // NEGATIVE: observe() itself keeps them out, whoever calls it (the probe after a re-arm): x and y at the pattern,
    // or all three axes zero; the published vector still comes back, for not_measuring's verdict
    let mut magnetometer = Bmm350::new(ADDRESS);
    for field in [
        Field {
            x: NO_DATA,
            y: NO_DATA,
            z: NO_DATA,
        },
        Field {
            x: NO_DATA,
            y: NO_DATA,
            z: 700,
        },
        Field::default(),
    ] {
        assert!(!field.measured(), "{field:?}");
        assert_eq!(magnetometer.observe(&field), [field.x, field.y, field.z]);
    }
    assert_eq!(magnetometer.seen(), 0);
    assert_eq!(magnetometer.span(), None);
    assert_eq!(
        magnetometer.instruction(),
        "turn the airframe through a full circle, LEVEL, then calibrate (no field seen yet)"
    );
    // one axis at the pattern, or two axes zero, is a field: the rails of a strong field, a level board's x and y
    for field in [Field { x: NO_DATA, y: 5, z: 0 }, Field { x: 0, y: 0, z: 700 }] {
        assert!(field.measured(), "{field:?}");
    }
}

#[test]
fn a_part_stuck_at_no_data_goes_stale() {
    // NEGATIVE: a re-arm that never brings the conversions back: Unchanged, then Stale three conversion periods
    // (240 ms) after the last real change, where the firmware calls the part unhealthy; nothing is observed
    let mut magnetometer = Bmm350::new(ADDRESS);
    let mut fields = vec![(1200, -300, 700)];
    fields.extend([no_data(); 8]);
    let outcomes = poll(&mut magnetometer, 0, &fields);
    let first_stale = outcomes.iter().position(|change| *change == Change::Stale).unwrap();
    assert_eq!(
        u64::try_from(first_stale).unwrap() * u64::from(POLL_PERIOD_MS) * 1000,
        3 * CONVERSION_PERIOD_US
    );
    assert!(
        outcomes[1..first_stale]
            .iter()
            .all(|change| *change == Change::Unchanged)
    );
    assert!(outcomes[first_stale..].iter().all(|change| *change == Change::Stale));
    assert_eq!(magnetometer.seen(), 1);
    // the first real conversion after it is fresh at once
    assert_eq!(
        poll(&mut magnetometer, 1_000_000, &[(1201, -300, 700)]),
        [Change::Fresh]
    );
    // a part that never produced a sample goes stale on the clock of its first read
    let mut never = Bmm350::new(ADDRESS);
    let outcomes = poll(&mut never, 0, &[no_data(); 7]);
    assert_eq!(outcomes[..6], [Change::Unchanged; 6]);
    assert_eq!(outcomes[6], Change::Stale);
}
