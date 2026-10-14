//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The BNO055's unit tests: the block's scaling, the stall judge (still parts repeat freely; a rotating part with a
//! frozen Euler latches after STALL_SAMPLES, holds, and clears when the Euler moves), the calibration latch that
//! survives the register regressing, what is owed, the bring-up sequence and a wrong identity, the profile read that
//! always returns to NDOF and the restore that sets the latch; and the refused fusion: a switch into NDOF read back, a
//! profile out of range neither restored nor saved, one the chip refuses undone by a reset.

use super::{
    BLOCK_LENGTH, Bno055, CALIB_STAT, CHIP_ID, CalibrationState, DATA, Fusion, IDENTITY, MODE_CONFIG, MODE_NDOF,
    MODE_SETTLE_MS, OPR_MODE, POWER_NORMAL, PROFILE, PROFILE_BYTES, PWR_MODE, RESET, RESET_MS, STALL_SAMPLES, SYS_ERR,
    SYS_TRIGGER, Sample, UNIT_SEL, UNITS, aerospace, owed, profile_valid,
};
use crate::axes::Axes;
use crate::device::{Device, Fault};
use crate::mock::MockBus;

/// The part's address in these tests.
const ADDRESS: u8 = 0x28;

/// A profile of `fill` bytes with the radii set: the accelerometer's `accel_radius`, the magnetometer's `mag_radius`.
fn profile(fill: u8, accel_radius: i16, mag_radius: i16) -> [u8; PROFILE_BYTES] {
    let mut profile: [u8; PROFILE_BYTES] = core::array::from_fn(|index| fill + index as u8);
    profile[18..20].copy_from_slice(&accel_radius.to_le_bytes());
    profile[20..22].copy_from_slice(&mag_radius.to_le_bytes());
    profile
}

/// A block: accel (x, y, z) counts, gyro (x, y, z) counts, Euler (heading, roll, pitch) counts.
fn block(accel: [i16; 3], gyro: [i16; 3], euler: [i16; 3]) -> [u8; BLOCK_LENGTH] {
    let mut bytes = [0u8; BLOCK_LENGTH];
    for (index, value) in accel.iter().enumerate() {
        bytes[2 * index..2 * index + 2].copy_from_slice(&value.to_le_bytes());
    }
    for (index, value) in gyro.iter().enumerate() {
        bytes[12 + 2 * index..14 + 2 * index].copy_from_slice(&value.to_le_bytes());
    }
    for (index, value) in euler.iter().enumerate() {
        bytes[18 + 2 * index..20 + 2 * index].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

#[test]
fn the_block_decodes_to_degrees_and_g() {
    let sample = Sample::decode(&block([0, 0, 981], [16, -32, 0], [1440, -160, 80]));
    assert_eq!(sample.euler, [90.0, -10.0, 5.0], "16 LSB a degree");
    assert!(
        (sample.accel[2] - 981.0 / 980.665).abs() < 1e-6,
        "100 LSB a m/s^2: ~1 g on z"
    );
    assert_eq!(sample.gyro_raw, [16, -32, 0]);
    assert_eq!(sample.euler_raw, [1440, -160, 80]);
}

#[test]
fn a_still_part_repeats_its_euler_and_never_stalls() {
    let mut part = Bno055::new(ADDRESS);
    let still = Sample::decode(&block([0, 0, 981], [1, -2, 1], [1440, 0, 0]));
    for _ in 0..(3 * STALL_SAMPLES) {
        assert_eq!(part.judge(&still), Fusion::Live);
    }
    assert!(!part.stalled(), "a still airframe is no proof of a dead fusion");
}

#[test]
fn a_rotating_part_with_a_frozen_euler_stalls_and_clears_when_it_moves() {
    let mut part = Bno055::new(ADDRESS);
    let frozen = Sample::decode(&block([0, 0, 981], [200, 0, 0], [1440, 0, 0]));
    assert_eq!(part.judge(&frozen), Fusion::Live, "the first sample sets the reference");
    for _ in 1..STALL_SAMPLES {
        assert_eq!(part.judge(&frozen), Fusion::Live, "not yet proven");
    }
    assert_eq!(
        part.judge(&frozen),
        Fusion::Stalled,
        "STALL_SAMPLES rotating repeats latch it"
    );
    assert_eq!(part.judge(&frozen), Fusion::StillStalled);
    // NEGATIVE: once stalled it stays, even when the rotation stops, until the Euler moves
    let still = Sample::decode(&block([0, 0, 981], [0, 0, 0], [1440, 0, 0]));
    assert_eq!(part.judge(&still), Fusion::StillStalled);
    let moved = Sample::decode(&block([0, 0, 981], [200, 0, 0], [1441, 0, 0]));
    assert_eq!(part.judge(&moved), Fusion::Live);
    assert!(!part.stalled());
}

#[test]
fn a_pause_in_the_rotation_restarts_the_count() {
    // NEGATIVE: a rotating repeat, then a still one, then rotating again: the strikes start over
    let mut part = Bno055::new(ADDRESS);
    let rotating = Sample::decode(&block([0, 0, 981], [0, 0, 300], [100, 0, 0]));
    let still = Sample::decode(&block([0, 0, 981], [0, 0, 0], [100, 0, 0]));
    part.judge(&rotating);
    for _ in 1..STALL_SAMPLES {
        part.judge(&rotating);
    }
    assert_eq!(part.judge(&still), Fusion::Live, "the strikes reset");
    for _ in 1..STALL_SAMPLES {
        assert_eq!(part.judge(&rotating), Fusion::Live);
    }
}

#[test]
fn the_calibration_latch_survives_the_register_regressing() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CALIB_STAT], &[0b1111_1110])
        .expect_write_read(ADDRESS, &[CALIB_STAT], &[0b1111_1111])
        .expect_write_read(ADDRESS, &[CALIB_STAT], &[0b1111_1110]);
    let mut bus = mock.clone();
    let mut part = Bno055::new(ADDRESS);
    assert_eq!(part.poll_calibration(&mut bus), Ok(false), "mag 2: not yet");
    assert!(part.owed().unwrap().contains("mag 2"), "{:?}", part.owed());
    assert_eq!(part.poll_calibration(&mut bus), Ok(true), "mag 3 sets the latch, once");
    assert_eq!(
        part.poll_calibration(&mut bus),
        Ok(false),
        "mag back to 2: no new latch"
    );
    mock.done();
    assert!(part.calibrated(), "the latch, not the live register");
    assert_eq!(
        part.calibration_state(),
        Some(CalibrationState {
            system: 3,
            gyro: 3,
            accel: 3,
            mag: 2
        })
    );
    assert!(
        part.owed().unwrap().contains("calibrate imu_bno055"),
        "converged, the save owed"
    );
    assert!(!part.saved());
    part.mark_saved();
    assert!(part.saved());
    assert_eq!(part.owed(), None);
}

#[test]
fn calib_stat_round_trips_through_its_fields() {
    for raw in [0x00, 0b1110_0100, 0xFF, 0b0001_1011] {
        assert_eq!(CalibrationState::decode(raw).encode(), raw);
    }
    assert_eq!(
        CalibrationState::decode(0b1110_0100),
        CalibrationState {
            system: 3,
            gyro: 2,
            accel: 1,
            mag: 0
        }
    );
}

#[test]
fn the_owed_text_follows_the_published_state() {
    let short = Some(CalibrationState {
        system: 0,
        gyro: 3,
        accel: 1,
        mag: 1,
    });
    assert!(
        owed(short, false, false)
            .unwrap()
            .ends_with("(now sys 0 gyr 3 acc 1 mag 1)")
    );
    assert!(
        owed(short, true, false).unwrap().starts_with("converged"),
        "the latch, not the live mag"
    );
    assert_eq!(owed(short, true, true), None);
    // NEGATIVE: a saved flag without the latch owes the figure-8 still
    assert!(owed(None, false, true).unwrap().contains("not read yet"));
}

#[test]
fn nothing_read_yet_owes_the_figure_eight() {
    assert!(Bno055::new(ADDRESS).owed().unwrap().contains("not read yet"));
}

#[test]
fn setup_enters_ndof_through_config_with_normal_power() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &[PWR_MODE, POWER_NORMAL])
        .expect_write(ADDRESS, &[UNIT_SEL, UNITS])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    Bno055::new(ADDRESS).setup(&mut bus, &mut delay).unwrap();
    mock.done();
}

#[test]
fn a_wrong_identity_fails_setup_before_any_write() {
    // NEGATIVE: another part at 0x28 answers: nothing is written
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[0x33]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let outcome = Bno055::new(ADDRESS).setup(&mut bus, &mut delay);
    mock.done();
    assert!(
        matches!(
            outcome,
            Err(Fault::Identity {
                expected: 0xA0,
                read: 0x33
            })
        ),
        "{outcome:?}"
    );
}

#[test]
fn the_profile_read_returns_to_ndof_even_when_it_fails() {
    let profile = profile(0, 1000, 640);
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[PROFILE], &profile)
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bno055::new(ADDRESS);
    assert_eq!(part.read_profile(&mut bus, &mut delay), Ok(profile));
    mock.done();
    // NEGATIVE: the read fails: NDOF is written all the same, the fault reported
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[PROFILE], &profile)
        .then_fail()
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    assert!(matches!(part.read_profile(&mut bus, &mut delay), Err(Fault::Bus(_))));
    mock.done();
}

#[test]
fn a_restored_profile_sets_the_latch_and_owes_nothing() {
    let profile = profile(0x40, 1000, 640);
    let mut write = vec![PROFILE];
    write.extend_from_slice(&profile);
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &write)
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bno055::new(ADDRESS);
    part.restore_profile(&mut bus, &mut delay, &profile).unwrap();
    mock.done();
    assert!(part.calibrated() && part.restored() && part.saved());
    assert_eq!(part.owed(), None);
}

#[test]
fn the_sample_read_is_the_block() {
    let bytes = block([10, 20, 30], [0, 0, 0], [16, 32, 48]);
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[DATA], &bytes);
    let mut bus = mock.clone();
    let sample = Bno055::new(ADDRESS).read(&mut bus).unwrap();
    mock.done();
    assert_eq!(sample.euler, [1.0, 2.0, 3.0]);
}

#[test]
fn a_restarted_fusion_s_zero_euler_is_withheld_until_the_first_real_one() {
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &[PWR_MODE, POWER_NORMAL])
        .expect_write(ADDRESS, &[UNIT_SEL, UNITS])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bno055::new(ADDRESS);
    part.setup(&mut bus, &mut delay).unwrap();
    mock.done();
    let zero = Sample::decode(&block([0, 0, 981], [0, 0, 0], [0, 0, 0]));
    let level = Sample::decode(&block([0, 0, 981], [0, 0, 0], [0, -37, -124]));
    assert_eq!(part.judge(&zero), Fusion::Starting);
    assert_eq!(part.judge(&zero), Fusion::Starting, "until the first real Euler");
    assert_eq!(part.judge(&level), Fusion::Live);
    // NEGATIVE: once started, a zero Euler is an attitude like any other (a level part pointing north)
    assert_eq!(part.judge(&zero), Fusion::Live);
    // a fresh driver never restarted the fusion: nothing to withhold
    assert_eq!(Bno055::new(ADDRESS).judge(&zero), Fusion::Live);
}

#[test]
fn a_refused_ndof_is_a_configuration_fault_with_its_sys_err() {
    // the taster, 2026-10-14: OPR_MODE reads back CONFIG, SYS_ERR 9 (the fusion's configuration)
    let mock = MockBus::new();
    mock.expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &[PWR_MODE, POWER_NORMAL])
        .expect_write(ADDRESS, &[UNIT_SEL, UNITS])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_CONFIG])
        .expect_write_read(ADDRESS, &[SYS_ERR], &[9]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bno055::new(ADDRESS);
    assert!(matches!(part.setup(&mut bus, &mut delay), Err(Fault::Configuration)));
    mock.done();
    assert_eq!(part.refused(), Some(9));
    // NEGATIVE: a switch that takes clears it (the re-arm: CONFIG, the units, NDOF)
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &[UNIT_SEL, UNITS])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    part.rearm(&mut bus, &mut delay).unwrap();
    mock.done();
    assert_eq!(part.refused(), None);
}

#[test]
fn a_profile_with_a_radius_out_of_range_is_never_restored() {
    // the taster's saved profile: a magnetometer radius of 0, which the chip refuses NDOF with
    assert!(profile_valid(&profile(0, 1000, 640)));
    for (accel, mag) in [(1000, 0), (0, 640), (1001, 640), (1000, 961), (-5, 640)] {
        assert!(!profile_valid(&profile(0, accel, mag)), "{accel} {mag}");
        // NEGATIVE: the chip is never touched
        let mock = MockBus::new();
        let mut bus = mock.clone();
        let mut delay = mock.delay();
        let mut part = Bno055::new(ADDRESS);
        let outcome = part.restore_profile(&mut bus, &mut delay, &profile(0, accel, mag));
        mock.done();
        assert!(matches!(outcome, Err(Fault::Configuration)));
        assert!(!part.calibrated() && !part.saved());
    }
}

#[test]
fn a_profile_the_chip_refuses_is_undone_by_a_reset() {
    let profile = profile(0x40, 1000, 640);
    let mut write = vec![PROFILE];
    write.extend_from_slice(&profile);
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &write)
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_CONFIG])
        .expect_write_read(ADDRESS, &[SYS_ERR], &[9])
        // reset, then the bring-up without the profile
        .expect_write(ADDRESS, &[SYS_TRIGGER, RESET])
        .expect_delay_ms(RESET_MS)
        .expect_write_read(ADDRESS, &[CHIP_ID], &[IDENTITY])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write(ADDRESS, &[PWR_MODE, POWER_NORMAL])
        .expect_write(ADDRESS, &[UNIT_SEL, UNITS])
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    let mut part = Bno055::new(ADDRESS);
    assert!(matches!(
        part.restore_profile(&mut bus, &mut delay, &profile),
        Err(Fault::Configuration)
    ));
    mock.done();
    // the part runs, the refusal kept for the log, and the figure-8 owed: no latch, nothing saved
    assert_eq!(part.refused(), Some(9));
    assert!(!part.calibrated() && !part.restored() && !part.saved());
}

#[test]
fn a_profile_with_nothing_learned_is_never_saved() {
    // NEGATIVE: CONFIG for the read, NDOF again, and the radius-0 profile refused: saved, it would blind the next boot
    let learned_nothing = profile(0, 1000, 0);
    let mock = MockBus::new();
    mock.expect_write(ADDRESS, &[OPR_MODE, MODE_CONFIG])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[PROFILE], &learned_nothing)
        .expect_write(ADDRESS, &[OPR_MODE, MODE_NDOF])
        .expect_delay_ms(MODE_SETTLE_MS)
        .expect_write_read(ADDRESS, &[OPR_MODE], &[MODE_NDOF]);
    let mut bus = mock.clone();
    let mut delay = mock.delay();
    assert!(matches!(
        Bno055::new(ADDRESS).read_profile(&mut bus, &mut delay),
        Err(Fault::Configuration)
    ));
    mock.done();
}

/// The filter's (roll, pitch) in degrees of a still accelerometer's reading in the marks frame: the gravity vector is
/// the reading with x negated (crates/control attitude `gravity_from_marks`), roll atan2(gy, gz), pitch
/// atan2(-gx, |(gy, gz)|).
fn tilt_of(marks: [f32; 3]) -> [f32; 2] {
    let [x, y, z] = marks;
    let gravity = [-x, y, z];
    [
        gravity[1].atan2(gravity[2]).to_degrees(),
        (-gravity[0]).atan2(gravity[1].hypot(gravity[2])).to_degrees(),
    ]
}

/// Whether two angles agree within `tolerance` degrees, the short way around.
fn close(first: f32, second: f32, tolerance: f32) -> bool {
    ((first - second + 180.0).rem_euclid(360.0) - 180.0).abs() <= tolerance
}

#[test]
fn the_fused_euler_reads_as_the_filter_s_frame() {
    // measured: the chip's own Euler against its own accelerometer (m/s^2 / 9.81), three poses
    let poses = [
        // the taster, Windows units (2026-10-14)
        ([22.8125, 13.8125, -23.5625], [2.26, -3.64, 8.35]),
        // TMS-7C and 7D on the rod (2026-10-03), recorded in the other orientation: pitch negated into Windows
        ([334.0, 75.56, -171.0], [9.30, -0.32, -2.43]),
        ([342.3, 75.8, 173.2], [9.45, 0.27, -2.37]),
    ];
    for (euler, accel) in poses {
        let [_, roll, pitch] = aerospace(euler, &Axes::IDENTITY);
        let [owed_roll, owed_pitch] = tilt_of(accel);
        assert!(
            close(roll, owed_roll, 1.5) && close(pitch, owed_pitch, 1.5),
            "{euler:?}: {roll} {pitch}"
        );
    }
    // NEGATIVE: the chip's own angles are not the filter's: on the rod the chip's pitch reads 171 and its roll 75,
    // the airframe's pitch is 75 (nose up) and its roll about 180 (the dorsal side down)
    let [_, roll, pitch] = aerospace([334.0, 75.56, -171.0], &Axes::IDENTITY);
    assert!(pitch > 70.0 && roll.abs() > 165.0, "{roll} {pitch}");
}

#[test]
fn the_tilt_agrees_with_gravity_for_any_mounting() {
    // every pose and every mounting: the chip's gravity under the convention, through the axes, reads the tilt the
    // conversion gives
    let gravity_in_chip = |[_, roll, pitch]: [f32; 3]| {
        let (roll, pitch) = (roll.to_radians(), pitch.to_radians());
        // Rx(-pitch) Ry(roll) on the world's up (0, 0, 1)
        let (x, z) = (roll.sin(), roll.cos());
        let (sine, cosine) = (-pitch).sin_cos();
        [x, -z * sine, z * cosine]
    };
    for axes in ["+x+y+z", "-y+x+z", "+y-x+z", "-x-y+z", "+z+x+y"] {
        let axes = Axes::parse(axes).unwrap();
        for euler in [
            [0.0, 10.0, -20.0],
            [123.0, -35.0, 50.0],
            [300.0, 60.0, -140.0],
            [45.0, -80.0, 100.0],
        ] {
            let [_, roll, pitch] = aerospace(euler, &axes);
            let [owed_roll, owed_pitch] = tilt_of(axes.apply(gravity_in_chip(euler)));
            assert!(
                close(roll, owed_roll, 0.01) && close(pitch, owed_pitch, 0.01),
                "{axes} {euler:?}: {roll} {pitch} against {owed_roll} {owed_pitch}"
            );
        }
    }
}

#[test]
fn the_heading_names_the_chip_s_y_axis() {
    // level, mounted per its marks: Bosch's heading is the chip's +y, the marks' left wing, so the nose (+x, its
    // right) points 90 degrees clockwise of it
    for heading in [0.0, 30.0, 200.0, 350.0] {
        let [nose, roll, pitch] = aerospace([heading, 0.0, 0.0], &Axes::IDENTITY);
        assert!(
            close(nose, heading + 90.0, 1e-3) && roll.abs() < 1e-3 && pitch.abs() < 1e-3,
            "{nose}"
        );
    }
    // the chip's +y on the nose (the marks' x the chip's +y, y its -x): its heading is the nose's
    let nose_y = Axes::parse("+y-x+z").unwrap();
    let [nose, ..] = aerospace([30.0, 0.0, 0.0], &nose_y);
    assert!(close(nose, 30.0, 1e-3), "{nose}");
}
