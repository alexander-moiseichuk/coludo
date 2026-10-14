//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the attitude filter, after src/glider/test/test_attitude.py and test_magnetic_yaw.py: the mirror
//! (outranking sources only), the free run (gyro, gravity, the turn gate, the track, the magnetometer), the blind
//! window and the withhold, the gap rule, the boundary units, and what the port does where plan B could not (a
//! non-finite reading, a config of the wrong type). The golden fixtures (tests/golden_attitude.rs,
//! tests/golden_magnetic_yaw.rs) hold the trajectories to plan B; these hold the rules, each with what must NOT happen.

use super::{
    Attitude, BLIND_CYCLES, CENTRIPETAL_SPEED, FIELDS, Fused, GYRO_BACK, GYRO_LOST, MIRROR_AGE_US, MagneticYaw,
    Orientation, PROBE_FAILURE, Readings, STILL_G, STILL_RATE, STREAM, Settings, decimal_text, heading_centidegrees,
    pull, rate_from_marks, signed_centidegrees, wrap_heading, wrap_signed,
};
use serde_json::json;

/// The tick period of the tests, microseconds (the filter's 50 Hz).
const TICK_US: u64 = 20_000;

/// test_attitude's component: a fast blend (corr_shift 2, course_shift 2), a pull on every tick (accel_period_ms 0).
fn test_settings() -> Settings {
    Settings::from_json(&json!({
        "period_ms": 20, "accel_period_ms": 0, "corr_shift": 2, "grav_low_g": 0.7, "grav_high_g": 1.3,
        "turn_gate_deg_s": 4, "course_gate_mps": 5.0, "course_shift": 2, "mag_shift": 2, "mag_offset_shift": 1,
        "provides": {"attitude": {"priority": 1, "timeout_ms": 40}}
    }))
}

/// Body-frame gravity in g at a roll and pitch in degrees: what the accelerometer reads at rest (test_attitude
/// `_accel_for`).
fn gravity_at(roll: f32, pitch: f32) -> [f32; 3] {
    let (roll, pitch) = (roll.to_radians(), pitch.to_radians());
    [-pitch.sin(), pitch.cos() * roll.sin(), pitch.cos() * roll.cos()]
}

/// What a part mounted per its MARKS (x forward, y left, z up) reads at rest at a roll and pitch in degrees: the
/// specific force, +1 g straight up. Nose up tilts x up (+x); right wing down lifts the left wing, y (+y).
fn marks_at(roll: f32, pitch: f32) -> [f32; 3] {
    let (roll, pitch) = (roll.to_radians(), pitch.to_radians());
    [pitch.sin(), pitch.cos() * roll.sin(), pitch.cos() * roll.cos()]
}

/// The horizontal field a level part mounted per its marks reads with the nose at `heading` degrees clockwise from
/// magnetic north: north is then `heading` to the LEFT of the nose, +y in the marks frame.
fn marks_field(heading: f32) -> [f32; 3] {
    let heading = heading.to_radians();
    [1000.0 * heading.cos(), 1000.0 * heading.sin(), -400.0]
}

/// A fresh winner from the priority-0 source (the HITL sim) at this attitude.
fn primary(heading: f32, roll: f32, pitch: f32) -> Option<Fused> {
    Some(Fused {
        orientation: Orientation { heading, roll, pitch },
        rank: 0,
        age_us: 0,
    })
}

/// A filter driven tick by tick on a 20 ms clock.
struct Bench {
    /// The filter.
    filter: Attitude,
    /// The clock, microseconds.
    now_us: u64,
}

impl Bench {
    /// A filter set up at 0 on these settings.
    fn new(settings: Settings) -> Bench {
        Bench {
            filter: Attitude::new(settings, 0),
            now_us: 0,
        }
    }

    /// A test_settings filter seeded at this attitude by one mirrored tick, as a state is set in flight.
    fn seeded(heading: f32, roll: f32, pitch: f32) -> Bench {
        let mut bench = Bench::new(test_settings());
        let readings = Readings {
            rate: Some([0.0; 3]),
            fused: primary(heading, roll, pitch),
            ..Readings::default()
        };
        bench.tick(&readings);
        bench
    }

    /// One tick, 20 ms after the last.
    fn tick(&mut self, readings: &Readings) -> super::Tick {
        self.now_us += TICK_US;
        self.filter.tick(self.now_us, readings)
    }

    /// `count` ticks of the same readings; the last tick's answer.
    fn ticks(&mut self, count: usize, readings: &Readings) -> super::Tick {
        let mut last = self.tick(readings);
        for _ in 1..count {
            last = self.tick(readings);
        }
        last
    }
}

/// Whether two angles agree within `tolerance` degrees, the short way around.
fn close(first: f32, second: f32, tolerance: f32) -> bool {
    wrap_signed(first - second).abs() <= tolerance
}

#[test]
fn the_defaults_are_plan_b_setup_defaults() {
    let defaults = Settings::from_json(&json!({}));
    assert_eq!(defaults, Settings::default());
    assert_eq!(
        (
            defaults.period_ms,
            defaults.acceleration_period_ms,
            defaults.correction_shift
        ),
        (20, 50, 4)
    );
    assert_eq!(
        (
            defaults.course_shift,
            defaults.magnetic_shift,
            defaults.magnetic_offset_shift
        ),
        (5, 6, 5)
    );
    assert_eq!(
        (defaults.turn_gate, defaults.course_gate, defaults.magnetic_level),
        (4.0, 5.0, 15.0)
    );
    assert_eq!(
        (defaults.gravity_low, defaults.gravity_high, defaults.telemetry_ms),
        (0.7, 1.3, 100)
    );
    assert_eq!(defaults.rank, 0, "plan B registers a missing priority at 0");
    assert_eq!(test_settings().rank, 1);
    // the port's own two, on; plan B's behaviour back with 0 and false
    assert_eq!((defaults.gyro_bias, defaults.centripetal), (10.0, true));
    let plan_b = Settings::from_json(&json!({"gyro_bias_s": 0, "centripetal": false}));
    assert_eq!((plan_b.gyro_bias, plan_b.centripetal), (0.0, false));
}

#[test]
fn a_config_of_the_wrong_type_takes_the_defaults() {
    // NEGATIVE: plan B stores whatever arrives and raises where it is used; the port keeps its defaults
    let odd = Settings::from_json(&json!({
        "period_ms": "20", "corr_shift": 2.5, "course_shift": -1, "turn_gate_deg_s": null, "grav_low_g": true,
        "provides": {"attitude": {"priority": 1.5}}
    }));
    assert_eq!(odd, Settings::default());
    assert_eq!(Settings::from_json(&json!([1, 2])), Settings::default());
    assert_eq!(Settings::from_json(&json!({"provides": 3})).rank, 0);
}

#[test]
fn the_rank_is_the_one_the_databoard_registers() {
    // the filter compares a winner's registered rank with its own, so its own is read by the registration's rule
    // (board_config::provides): Python's int, a bool 0 or 1, saturating at i32, anything else 0
    let rank = |priority: Option<serde_json::Value>| {
        let mut block =
            json!({"name": "attitude", "activity": "attitude", "provides": {"attitude": {"timeout_ms": 40}}});
        if let Some(priority) = priority {
            block["provides"]["attitude"]["priority"] = priority;
        }
        let document = json!({"components": [block.clone()]});
        let registered = board_config::provides(&board_config::entries(&document)[0])[0].priority;
        let read = Settings::from_json(&block).rank;
        assert_eq!(read, registered, "{block}");
        read
    };
    for (priority, owed) in [
        (json!(1), 1),
        (json!(300), 300),
        (json!(-1), -1),
        (json!(true), 1),
        (json!(false), 0),
        (json!(2_147_483_648_i64), i32::MAX),
        (json!(i64::MIN), i32::MIN),
        (json!(u64::MAX), i32::MAX),
    ] {
        assert_eq!(rank(Some(priority.clone())), owed, "{priority}");
    }
    // NEGATIVE: what is no int registers at 0, and so does a missing priority
    for priority in [
        json!(1.0),
        json!(1.5),
        json!("1"),
        json!(null),
        json!([1]),
        json!({"priority": 1}),
    ] {
        assert_eq!(rank(Some(priority.clone())), 0, "{priority}");
    }
    assert_eq!(rank(None), 0);
}

#[test]
fn a_pull_is_two_to_the_minus_shift() {
    assert_eq!(pull(0), 1.0);
    assert_eq!(pull(2), 0.25);
    assert_eq!(pull(6), 1.0 / 64.0);
    assert_eq!(pull(u32::MAX), pull(126), "a huge shift neither overflows nor pulls");
    assert!(pull(u32::MAX) < 1e-37);
}

#[test]
fn the_wraps_are_plan_b_ranges() {
    assert_eq!(wrap_heading(360.0), 0.0);
    assert_eq!(wrap_heading(-90.0), 270.0);
    assert_eq!(
        wrap_heading(-1e-9),
        0.0,
        "f32 rem_euclid answers 360 here: the wrap must not"
    );
    assert_eq!(
        wrap_signed(180.0),
        -180.0,
        "the half turn is -180, as plan B's integer wrap"
    );
    assert_eq!(wrap_signed(-180.0), -180.0);
    assert_eq!(wrap_signed(359.0), -1.0);
    assert!(wrap_heading(f32::NAN).is_nan());
}

#[test]
fn mirror_an_outranking_source() {
    let mut bench = Bench::new(test_settings());
    let tick = bench.tick(&Readings {
        rate: Some([0.0; 3]),
        fused: primary(123.0, 45.0, -6.0),
        ..Readings::default()
    });
    assert_eq!(
        tick.published,
        Some(Orientation {
            heading: 123.0,
            roll: 45.0,
            pitch: -6.0
        })
    );
    assert!(bench.filter.seeded() && !bench.filter.free());
    // the source goes quiet: the seeded filter free-runs from the copy, with no accelerometer at all
    let tick = bench.tick(&Readings {
        rate: Some([0.0; 3]),
        ..Readings::default()
    });
    assert_eq!(tick.published.map(|orientation| orientation.roll), Some(45.0));
    assert!(bench.filter.free());
}

#[test]
fn never_mirror_a_backup_a_peer_or_itself() {
    // NEGATIVE: a source at or below the filter's rank is never copied (the BNO055's heading is another frame)
    for rank in [1, 2] {
        let mut bench = Bench::new(test_settings());
        let readings = Readings {
            rate: Some([0.0; 3]),
            acceleration: Some(gravity_at(0.0, 0.0)),
            fused: Some(Fused {
                orientation: Orientation {
                    heading: 77.0,
                    roll: 25.0,
                    pitch: 15.0,
                },
                rank,
                age_us: 0,
            }),
            ..Readings::default()
        };
        let tick = bench.ticks(10, &readings);
        let published = tick.published.expect("the filter stays the source");
        assert!(
            close(published.roll, 0.0, 0.01) && close(published.heading, 0.0, 0.01),
            "{published:?}"
        );
        assert!(bench.filter.free() && !bench.filter.seeded());
    }
}

#[test]
fn never_seeded_and_no_accel_publishes_and_integrates_nothing() {
    let mut bench = Bench::new(test_settings());
    let gyro_only = Readings {
        rate: Some([10.0, 0.0, 0.0]),
        ..Readings::default()
    };
    // NEGATIVE: nothing trustworthy yet -- no publish, and the tick's gyro is lost, not deferred
    assert_eq!(bench.tick(&gyro_only).published, None);
    assert_eq!(bench.filter.orientation(), Orientation::default());
    assert!(!bench.filter.free());
    let with_accel = Readings {
        acceleration: Some([0.0; 3]),
        ..gyro_only
    };
    let tick = bench.tick(&with_accel);
    assert!((tick.published.unwrap().roll - 0.2).abs() < 1e-5, "{tick:?}");
    // the accelerometer goes again: nothing published, and `free` keeps its last value -- nothing clears it
    assert_eq!(bench.tick(&gyro_only).published, None);
    assert!(bench.filter.free());
}

#[test]
fn free_run_integrates_the_gyro() {
    // each body axis alone from level, a second: the angle it names moves by the rate
    let turned = |rate: [f32; 3]| {
        let mut bench = Bench::seeded(0.0, 0.0, 0.0);
        let readings = Readings {
            rate: Some(rate),
            acceleration: Some([0.0; 3]), // present but outside the band: no correction
            ..Readings::default()
        };
        bench.ticks(50, &readings).published.unwrap()
    };
    let rolled = turned([10.0, 0.0, 0.0]);
    assert!(
        (rolled.roll - 10.0).abs() < 1e-3 && rolled.pitch.abs() < 1e-3,
        "{rolled:?}"
    );
    let pitched = turned([0.0, -5.0, 0.0]);
    assert!(
        (pitched.pitch + 5.0).abs() < 1e-3 && pitched.roll.abs() < 1e-3,
        "{pitched:?}"
    );
    let yawed = turned([0.0, 0.0, 20.0]);
    assert!(
        close(yawed.heading, 20.0, 1e-3) && yawed.pitch.abs() < 1e-3,
        "{yawed:?}"
    );
    // NEGATIVE: the rates together are one rotation, not plan B's sum of angles (roll 10, pitch -5, heading 20): the
    // body rolling while it yaws turns part of the yaw into pitch
    let together = turned([10.0, -5.0, 20.0]);
    assert!((together.pitch + 5.0).abs() > 0.5, "not plan B's -5: {together:?}");
}

#[test]
fn a_long_free_run_is_exact_and_a_gyro_off_by_a_fraction_of_a_percent_shows() {
    // 20 s yawing level at 30 deg/s: 600 degrees, f32 over 1000 ticks within 0.01 (plan B's own long run, one axis)
    let run = |factor: f32| {
        let mut bench = Bench::seeded(0.0, 0.0, 0.0);
        let yawing = Readings {
            rate: Some([0.0, 0.0, 30.0 * factor]),
            acceleration: Some([0.0; 3]),
            ..Readings::default()
        };
        bench.ticks(1000, &yawing).published.unwrap().heading
    };
    assert!(close(run(1.0), 240.0, 0.01), "{}", run(1.0));
    // NEGATIVE: a gyro 0.05 % high ends 0.3 degree off: the 0.1 resolves it
    assert!(!close(run(1.0005), 240.0, 0.1), "{}", run(1.0005));
}

#[test]
fn a_boost_pulls_nothing_from_the_accelerometer() {
    // pitched over to 60 in the burn's tail, its 1 g along the nose: an accelerometer that reads nose straight up
    let burn = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 90.0)),
        ballistic: true,
        ..Readings::default()
    };
    let mut bench = Bench::seeded(0.0, 0.0, 60.0);
    let pitch = bench.ticks(20, &burn).published.unwrap().pitch;
    assert!((pitch - 60.0).abs() < 1e-3, "{pitch}");
    // NEGATIVE: the same reading out of BOOSTING is gravity, and pulls
    let mut glide = Bench::seeded(0.0, 0.0, 60.0);
    let pitch = glide
        .ticks(
            20,
            &Readings {
                ballistic: false,
                ..burn
            },
        )
        .published
        .unwrap()
        .pitch;
    assert!(pitch > 85.0, "{pitch}");
}

#[test]
fn a_still_gyro_s_bias_is_learned_at_rest_and_taken_off() {
    let settings = Settings {
        gyro_bias: 2.0,
        ..test_settings()
    };
    let biased = |resting: bool, rate: [f32; 3]| Readings {
        rate: Some(rate),
        acceleration: Some(gravity_at(0.0, 0.0)),
        resting,
        ..Readings::default()
    };
    let bias = [0.5, -0.3, 0.2];
    let mut bench = Bench::new(settings);
    bench.ticks(1000, &biased(true, bias)); // 20 s, ten time constants
    let learned = bench.filter.bias();
    assert!(
        learned
            .iter()
            .zip(bias)
            .all(|(learned, bias)| (learned - bias).abs() < 1e-3),
        "{learned:?}"
    );
    // in flight the bias is taken off: 20 s of the same reading holds the heading (plan B's drifted 4 degrees)
    let heading = bench.filter.orientation().heading;
    bench.ticks(1000, &biased(false, bias));
    assert!(
        close(bench.filter.orientation().heading, heading, 0.01),
        "{:?}",
        bench.filter.orientation()
    );
    // and it is held once out of SETTING: a rotation in flight teaches it nothing
    bench.ticks(100, &biased(false, [1.0, 1.0, 1.0]));
    assert_eq!(bench.filter.bias(), learned);
    // NEGATIVE: a moving board (an axis at the still threshold), a board not resting, or the key off learn nothing
    for (settings, readings) in [
        (settings, biased(true, [STILL_RATE, 0.0, 0.0])),
        (settings, biased(false, bias)),
        (
            Settings {
                gyro_bias: 0.0,
                ..settings
            },
            biased(true, bias),
        ),
    ] {
        let mut bench = Bench::new(settings);
        bench.ticks(500, &readings);
        assert_eq!(bench.filter.bias(), [0.0; 3], "{readings:?}");
    }
    // NEGATIVE: nor a board held off 1 g (a hand moving it)
    let mut bench = Bench::new(settings);
    bench.ticks(
        500,
        &Readings {
            acceleration: Some([0.0, 0.0, 1.0 + 2.0 * STILL_G]),
            ..biased(true, bias)
        },
    );
    assert_eq!(bench.filter.bias(), [0.0; 3]);
}

#[test]
fn each_gyro_keeps_its_own_bias() {
    // the LSM6DSO32 (rank 0) and the BMI323 (rank 1) read different biases; on the pad they alternate
    let settings = Settings {
        gyro_bias: 2.0,
        ..test_settings()
    };
    let at_rest = |rank: i32, rate: [f32; 3]| Readings {
        rate: Some(rate),
        acceleration: Some(gravity_at(0.0, 0.0)),
        resting: true,
        rate_rank: Some(rank),
        ..Readings::default()
    };
    let mut bench = Bench::new(settings);
    for _ in 0..1000 {
        // each its own 20 s of steps: ten time constants
        bench.tick(&at_rest(0, [0.5, 0.0, 0.2]));
        bench.tick(&at_rest(1, [-0.3, 0.1, 0.0]));
    }
    bench.tick(&at_rest(0, [0.5, 0.0, 0.2]));
    let lsm = bench.filter.bias();
    bench.tick(&at_rest(1, [-0.3, 0.1, 0.0]));
    let bmi = bench.filter.bias();
    assert!((lsm[0] - 0.5).abs() < 1e-3 && (lsm[2] - 0.2).abs() < 1e-3, "{lsm:?}");
    assert!((bmi[0] + 0.3).abs() < 1e-3 && (bmi[1] - 0.1).abs() < 1e-3, "{bmi:?}");
    // NEGATIVE: in flight the BMI323 carrying `rate` has its own bias taken off: the heading holds
    let heading = bench.filter.orientation().heading;
    bench.ticks(
        500,
        &Readings {
            resting: false,
            ..at_rest(1, [-0.3, 0.1, 0.0])
        },
    );
    assert!(close(bench.filter.orientation().heading, heading, 0.01));
}

#[test]
fn a_boost_never_anchored_publishes_nothing() {
    // a warm start into BOOSTING: the filter is built LEVEL mid-climb, and the accelerometer reads no gravity
    let boost = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 90.0)),
        ballistic: true,
        ..Readings::default()
    };
    let mut bench = Bench::new(test_settings());
    assert_eq!(
        bench.ticks(10, &boost).published,
        None,
        "the law flies neutral, not on a guess"
    );
    // the glide's first pull anchors it, and it publishes from there
    let glide = Readings {
        acceleration: Some(gravity_at(0.0, -6.0)),
        ballistic: false,
        ..boost
    };
    assert!(bench.tick(&glide).published.is_some());
    // NEGATIVE: a filter anchored on the pad publishes through the boost
    let mut cold = Bench::new(test_settings());
    cold.tick(&Readings {
        ballistic: false,
        ..boost
    });
    assert!(cold.ticks(10, &boost).published.is_some());
}

#[test]
fn the_centripetal_correction_holds_a_bank_the_accelerometer_reads_level() {
    // a coordinated turn banked 30 at 15 m/s: the accelerometer reads its load down the body axis, level; its body
    // rates (q, r) carry the turn. The turn gate opened (1000 deg/s), so every tick pulls
    let (bank, speed) = (30.0_f32, 15.0_f32);
    let turn = (9.806_65 * bank.to_radians().tan() / speed).to_degrees();
    let (sine, cosine) = bank.to_radians().sin_cos();
    let orbit = Readings {
        rate: Some([0.0, turn * sine, turn * cosine]),
        acceleration: Some([0.0, 0.0, 1.0 / cosine]),
        airspeed: Some(speed),
        ..Readings::default()
    };
    let roll = |centripetal: bool| {
        let mut bench = Bench::new(Settings {
            centripetal,
            turn_gate: 1000.0,
            ..test_settings()
        });
        bench.tick(&Readings {
            fused: primary(0.0, bank, 0.0),
            ..Readings::default()
        });
        bench.ticks(50, &orbit).published.unwrap().roll
    };
    assert!((roll(true) - bank).abs() < 0.1, "{}", roll(true));
    // NEGATIVE: uncorrected, the pulls flatten the bank (plan B's reason for the turn gate)
    assert!(roll(false) < 5.0, "{}", roll(false));
    // NEGATIVE: under CENTRIPETAL_SPEED the estimate is no airspeed: no correction
    let mut slow = Bench::new(Settings {
        centripetal: true,
        turn_gate: 1000.0,
        ..test_settings()
    });
    slow.tick(&Readings {
        fused: primary(0.0, bank, 0.0),
        ..Readings::default()
    });
    let slow = slow
        .ticks(
            50,
            &Readings {
                airspeed: Some(CENTRIPETAL_SPEED - 0.1),
                ..orbit
            },
        )
        .published
        .unwrap()
        .roll;
    assert!(slow < 5.0, "{slow}");
}

#[test]
fn a_held_sample_is_not_mirrored() {
    // the HITL sim stopped publishing: its last sample stays in its 1 s window, ever older, while the gyro rolls
    let mut bench = Bench::seeded(0.0, 10.0, 0.0);
    let held = |age_us: u32| Readings {
        rate: Some([20.0, 0.0, 0.0]),
        fused: Some(Fused {
            orientation: Orientation {
                heading: 0.0,
                roll: 10.0,
                pitch: 0.0,
            },
            rank: 0,
            age_us,
        }),
        ..Readings::default()
    };
    // a current sample is mirrored, the gyro ignored
    bench.tick(&held(MIRROR_AGE_US - 1));
    assert!((bench.filter.orientation().roll - 10.0).abs() < 1e-4 && !bench.filter.free());
    // NEGATIVE: a held one is not: the filter free-runs on the gyro, 0.4 degree a tick
    bench.tick(&held(MIRROR_AGE_US));
    assert!(bench.filter.free());
    assert!(
        (bench.filter.orientation().roll - 10.4).abs() < 1e-3,
        "{:?}",
        bench.filter.orientation()
    );
}

#[test]
fn a_banked_turn_keeps_its_pitch() {
    // a coordinated turn's body rates (banked 20, nose down 6, 10 deg/s): the turn gate holds the accelerometer off,
    // so the gyro alone carries the attitude round a 20 s orbit
    let (bank, pitch, turn) = (20.0_f32, -6.0_f32, 10.0_f32);
    let rate = [
        -turn * pitch.to_radians().sin(),
        turn * pitch.to_radians().cos() * bank.to_radians().sin(),
        turn * pitch.to_radians().cos() * bank.to_radians().cos(),
    ];
    let mut bench = Bench::seeded(40.0, bank, pitch);
    let orbit = Readings {
        rate: Some(rate),
        acceleration: Some(gravity_at(0.0, 0.0)), // the load along z: in the turn the accelerometer reads level
        ..Readings::default()
    };
    let published = bench.ticks(1000, &orbit).published.unwrap();
    assert!(close(published.heading, 240.0, 0.1), "{published:?}");
    // NEGATIVE: plan B's sum of angles climbed the pitch by turn * sin(bank) * cos(pitch), 3.4 deg/s
    assert!(
        (published.roll - bank).abs() < 0.1 && (published.pitch - pitch).abs() < 0.1,
        "{published:?}"
    );
}

#[test]
fn the_accel_pulls_toward_gravity_and_a_high_g_reading_is_rejected() {
    let mut bench = Bench::seeded(0.0, 0.0, 0.0);
    let readings = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(30.0, 0.0)),
        ..Readings::default()
    };
    let published = bench.ticks(40, &readings).published.unwrap();
    // the real-number filter: no centi-g truncation, no CORDIC -- converged to 30 well within a hundredth
    assert!(
        (published.roll - 30.0).abs() < 0.01 && published.pitch.abs() < 0.01,
        "{published:?}"
    );
    let pitched = Bench::seeded(0.0, 0.0, 0.0).ticks(
        40,
        &Readings {
            acceleration: Some(gravity_at(10.0, 25.0)),
            ..readings
        },
    );
    let pitched = pitched.published.unwrap();
    assert!(
        (pitched.roll - 10.0).abs() < 0.01 && (pitched.pitch - 25.0).abs() < 0.01,
        "{pitched:?}"
    );
    // NEGATIVE: ~4.2 g (boost, a manoeuvre) is not a gravity vector: the estimate holds
    let boost = Readings {
        acceleration: Some([0.0, 3.0, 3.0]),
        ..readings
    };
    let before = bench.filter.orientation();
    bench.ticks(5, &boost);
    assert_eq!(bench.filter.orientation(), before);
    // and the band's edges: 0.72 g pulls, 0.68 g does not
    let mut edge = Bench::seeded(0.0, 10.0, 0.0);
    edge.tick(&Readings {
        acceleration: Some([0.0, 0.0, 0.68]),
        ..readings
    });
    assert_eq!(edge.filter.orientation().roll, 10.0);
    edge.tick(&Readings {
        acceleration: Some([0.0, 0.0, 0.72]),
        ..readings
    });
    assert!((edge.filter.orientation().roll - 7.5).abs() < 1e-5);
}

#[test]
fn the_turn_gate_holds_the_bank() {
    // a coordinated turn: banked 40 with the accelerometer reading level, its body rates at 30 degree/s
    let mut bench = Bench::seeded(0.0, 40.0, 0.0);
    let (sine, cosine) = 40.0_f32.to_radians().sin_cos();
    let turning = Readings {
        rate: Some([0.0, 30.0 * sine, 30.0 * cosine]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        ..Readings::default()
    };
    bench.ticks(3, &turning);
    assert!(
        (bench.filter.orientation().roll - 40.0).abs() < 1e-3,
        "NEGATIVE: the turn must not roll the estimate flat"
    );
    // NEGATIVE: a gate too loose (30 deg/s, over the turn's 23 about z) pulls the turn flat
    let mut loose = Bench::new(Settings {
        turn_gate: 30.0,
        ..test_settings()
    });
    loose.tick(&Readings {
        fused: primary(0.0, 40.0, 0.0),
        ..Readings::default()
    });
    loose.ticks(3, &turning);
    assert!(
        loose.filter.orientation().roll < 30.0,
        "{:?}",
        loose.filter.orientation()
    );
    // exactly the gate is not a turn (plan B's `>`), and straight-ish flight re-anchors to level
    let straight = Readings {
        rate: Some([0.0, 0.0, 4.0]),
        ..turning
    };
    bench.ticks(40, &straight);
    assert!(bench.filter.orientation().roll.abs() < 0.01);
    // NEGATIVE: with no gyro at all the gate is closed too -- roll and pitch hold
    let mut blind = Bench::seeded(0.0, 40.0, 0.0);
    blind.ticks(3, &Readings { rate: None, ..turning });
    assert_eq!(blind.filter.orientation().roll, 40.0);
}

#[test]
fn the_accel_correction_is_throttled() {
    let settings = Settings {
        acceleration_period_ms: 50,
        ..test_settings()
    };
    let mut bench = Bench::new(settings);
    bench.tick(&Readings {
        fused: primary(0.0, 20.0, 0.0),
        ..Readings::default()
    });
    let level = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        ..Readings::default()
    };
    // set up at 0, mirrored at 20 ms: the ticks at 40 ms hold, 60 ms pulls (60 >= 50), then 80 and 100 hold
    let rolls: Vec<f32> = (0..4)
        .map(|_| {
            bench.tick(&level);
            bench.filter.orientation().roll
        })
        .collect();
    assert_eq!(rolls, [20.0, 15.0, 15.0, 15.0]);
    bench.tick(&level);
    assert_eq!(bench.filter.orientation().roll, 11.25, "120 ms: 60 after the last pull");
}

#[test]
fn yaw_follows_the_ground_track_and_wraps_through_north() {
    let mut bench = Bench::seeded(0.0, 0.0, 0.0);
    let moving = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        course: Some(90.0),
        ground_speed: Some(14.0),
        ..Readings::default()
    };
    bench.ticks(40, &moving);
    assert!(close(bench.filter.orientation().heading, 90.0, 0.01));
    // NEGATIVE: below the course gate the track is ignored
    let mut slow = Bench::seeded(45.0, 0.0, 0.0);
    slow.ticks(
        5,
        &Readings {
            ground_speed: Some(1.0),
            ..moving
        },
    );
    assert!(close(slow.filter.orientation().heading, 45.0, 1e-3));
    // a track just past north pulls 358 UP through 360
    let mut north = Bench::seeded(358.0, 0.0, 0.0);
    north.tick(&Readings {
        course: Some(2.0),
        ..moving
    });
    assert!(
        close(north.filter.orientation().heading, 359.0, 1e-3),
        "{:?}",
        north.filter.orientation()
    );
}

#[test]
fn blind_after_five_gyro_less_ticks_with_no_backup_keeps_publishing() {
    let mut bench = Bench::new(test_settings());
    let sighted = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        ..Readings::default()
    };
    let gone = Readings { rate: None, ..sighted };
    bench.tick(&sighted);
    for cycle in 1..BLIND_CYCLES {
        let tick = bench.tick(&gone);
        assert!(
            !bench.filter.blind() && tick.log.is_none(),
            "blind after only {cycle} gyro-less ticks"
        );
    }
    let tick = bench.tick(&gone);
    assert!(bench.filter.blind());
    assert_eq!(tick.log, Some(GYRO_LOST));
    let tick = bench.tick(&gone);
    assert_eq!(tick.log, None, "logged once per transition");
    assert!(tick.published.is_some(), "no backup: the blind filter keeps publishing");
    let tick = bench.tick(&sighted);
    assert!(!bench.filter.blind());
    assert_eq!(tick.log, Some(GYRO_BACK));
}

#[test]
fn blind_with_a_fresh_backup_withholds_and_a_stale_one_gets_the_publish_back() {
    let mut bench = Bench::new(test_settings());
    let sighted = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        backup_fresh: true,
        ..Readings::default()
    };
    let gone = Readings { rate: None, ..sighted };
    bench.tick(&sighted);
    for _ in 1..BLIND_CYCLES {
        assert!(bench.tick(&gone).published.is_some(), "NEGATIVE: no false withhold");
    }
    assert_eq!(bench.tick(&gone).published, None);
    assert_eq!(bench.tick(&gone).published, None);
    let quiet = Readings {
        backup_fresh: false,
        ..gone
    };
    assert!(
        bench.tick(&quiet).published.is_some(),
        "nothing fresh below: going quiet would leave no attitude"
    );
    assert!(bench.tick(&sighted).published.is_some() && !bench.filter.blind());
}

#[test]
fn blind_is_counted_while_mirroring() {
    let mut bench = Bench::new(test_settings());
    let mirrored = Readings {
        fused: primary(10.0, 5.0, 0.0),
        ..Readings::default()
    };
    bench.ticks(BLIND_CYCLES as usize, &mirrored);
    assert!(bench.filter.blind() && !bench.filter.free());
}

#[test]
fn a_long_gap_integrates_one_nominal_period() {
    let mut bench = Bench::seeded(0.0, 0.0, 0.0);
    let rolling = Readings {
        rate: Some([10.0, 0.0, 0.0]),
        acceleration: Some([0.0; 3]),
        ..Readings::default()
    };
    let roll_after = |bench: &mut Bench, elapsed_us: u64| {
        bench.now_us += elapsed_us;
        bench.filter.tick(bench.now_us, &rolling);
        bench.filter.orientation().roll
    };
    assert!(
        (roll_after(&mut bench, 600_000) - 0.2).abs() < 1e-5,
        "600 ms integrates the 20 ms period"
    );
    assert!(
        (roll_after(&mut bench, 0) - 0.2).abs() < 1e-5,
        "a repeated time integrates nothing"
    );
    assert!(
        (roll_after(&mut bench, 500_000) - 5.2).abs() < 1e-4,
        "exactly 500 ms is integrated whole"
    );
    assert!(
        (roll_after(&mut bench, 500_999) - 10.2099).abs() < 1e-3,
        "500.999 ms is 500 whole ms: whole"
    );
    assert!(
        (roll_after(&mut bench, 501_000) - 10.4099).abs() < 1e-3,
        "501 ms takes the nominal period"
    );
    // the port integrates the microseconds plan B floors away: 20.5 ms is 20.5 ms
    assert!((roll_after(&mut bench, 20_500) - 10.6149).abs() < 1e-3);
    // NEGATIVE: a clock that went backwards integrates the nominal period, not a huge or negative step
    bench.now_us -= 5_000;
    bench.filter.tick(bench.now_us, &rolling);
    assert!(
        (bench.filter.orientation().roll - 10.8149).abs() < 1e-3,
        "{:?}",
        bench.filter.orientation()
    );
}

#[test]
fn a_non_finite_reading_is_no_reading() {
    // NEGATIVE: one NaN in the state would poison the attitude for good
    let mut bench = Bench::seeded(10.0, 5.0, 0.0);
    let poisoned = Readings {
        rate: Some([f32::NAN, 0.0, 0.0]),
        acceleration: Some([0.0, f32::INFINITY, 1.0]),
        course: Some(f32::NAN),
        ground_speed: Some(14.0),
        magnetic: Some([f32::NAN, 0.0, 0.0]),
        fused: primary(f32::NAN, 0.0, 0.0),
        backup_fresh: false,
        ballistic: false,
        resting: false,
        airspeed: None,
        rate_rank: None,
    };
    let tick = bench.ticks(BLIND_CYCLES as usize, &poisoned);
    let published = tick.published.expect("still seeded: it publishes");
    assert_eq!(
        published,
        Orientation {
            heading: 10.0,
            roll: 5.0,
            pitch: 0.0
        }
    );
    assert!(bench.filter.blind(), "a NaN rate is no gyro: it counts toward blind");
}

#[test]
fn the_magnetic_heading_comes_from_the_horizontal_axes() {
    let magnetic = MagneticYaw::new(2, 1, 15.0);
    let heading = |field: [f32; 3], roll: f32, pitch: f32| magnetic.heading(Some(field), roll, pitch);
    assert!(close(heading([1000.0, 0.0, 0.0], 0.0, 0.0).unwrap(), 0.0, 1e-4));
    assert!(
        close(heading([0.0, 1000.0, 0.0], 0.0, 0.0).unwrap(), 270.0, 1e-4),
        "along +y is -90"
    );
    assert!(close(heading([0.0, -1000.0, -400.0], 14.0, -14.0).unwrap(), 90.0, 1e-4));
    assert!(
        (0.0..360.0).contains(&heading([1000.0, 1e-6, 0.0], 0.0, 0.0).unwrap()),
        "a hair west of north"
    );
    // NEGATIVE: past the level gate a flat atan2 is not a heading -- nothing rather than a plausible number
    assert_eq!(magnetic.heading(Some([1000.0, 0.0, 0.0]), 30.0, 0.0), None);
    assert_eq!(magnetic.heading(Some([1000.0, 0.0, 0.0]), 0.0, -40.0), None);
    assert!(
        heading([1000.0, 0.0, 0.0], 375.0, 0.0).is_some(),
        "the gate wraps: 375 is 15"
    );
    assert_eq!(magnetic.heading(None, 0.0, 0.0), None);
}

#[test]
fn the_offset_is_learned_from_the_track_then_carries_the_yaw() {
    let mut magnetic = MagneticYaw::new(2, 1, 15.0);
    let field = Some([1000.0, 0.0, 0.0]); // magnetic heading 0
    let mut yaw = 0.0;
    for _ in 0..40 {
        yaw = magnetic.step(field, 0.0, 0.0, Some(90.0), yaw);
    }
    assert!(magnetic.known() && close(magnetic.offset(), 90.0, 1e-3));
    assert_eq!(yaw, 0.0, "NEGATIVE: while tracking the magnetometer steers nothing");
    for _ in 0..40 {
        yaw = magnetic.step(field, 0.0, 0.0, None, yaw);
    }
    assert!(
        close(yaw, 90.0, 0.01),
        "the GNSS is gone: the field plus the offset carries the yaw ({yaw})"
    );
}

#[test]
fn an_unlearned_offset_never_steers() {
    let mut magnetic = MagneticYaw::new(2, 1, 15.0);
    let mut yaw = 123.45;
    for _ in 0..20 {
        yaw = magnetic.step(Some([0.0, 1000.0, 0.0]), 0.0, 0.0, None, yaw);
    }
    assert_eq!(yaw, 123.45);
    assert!(!magnetic.known());
    // NEGATIVE: a banked airframe learns nothing from the track either
    magnetic.step(Some([1000.0, 0.0, 0.0]), 30.0, 0.0, Some(90.0), yaw);
    assert!(!magnetic.known());
    // ...and one level learning step is enough to call it known, however little it has learned yet
    magnetic.step(Some([1000.0, 0.0, 0.0]), 0.0, 0.0, Some(90.0), yaw);
    assert!(magnetic.known() && close(magnetic.offset(), 45.0, 1e-3));
}

#[test]
fn the_magnetometer_carries_the_filter_through_a_gnss_dropout() {
    let mut bench = Bench::new(test_settings());
    let tracked = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        course: Some(90.0),
        ground_speed: Some(14.0),
        magnetic: Some([1000.0, 0.0, 0.0]),
        ..Readings::default()
    };
    bench.ticks(40, &tracked);
    assert!(bench.filter.magnetic().known());
    bench.tick(&Readings {
        fused: primary(0.0, 0.0, 0.0),
        ..tracked
    });
    let dropout = Readings {
        course: None,
        ground_speed: None,
        ..tracked
    };
    bench.ticks(40, &dropout);
    assert!(
        close(bench.filter.orientation().heading, 90.0, 0.01),
        "{:?}",
        bench.filter.orientation()
    );
}

#[test]
fn the_row_and_inspect_are_centidegree_integers_and_text() {
    let mut bench = Bench::seeded(359.996, -0.004, 12.346);
    assert_eq!(
        bench.filter.row(),
        [0, 0, 1235, 0, 0, 0],
        "359.996 rounds to 36000, which is heading 0"
    );
    bench.tick(&Readings::default());
    assert_eq!(bench.filter.row()[3], 1, "free-running");
    assert_eq!(
        bench.filter.inspect(),
        json!({"free_running": true, "seeded": true, "blind": false, "roll": "0.00", "pitch": "12.35",
               "heading": "0.00", "mag_known": false, "mag_offset": "0.00"})
    );
    assert_eq!(bench.filter.stats(), bench.filter.inspect());
    assert_eq!(decimal_text(-5), "-0.05");
    assert_eq!(decimal_text(-12345), "-123.45");
    assert_eq!(decimal_text(100), "1.00");
    assert_eq!(heading_centidegrees(-0.01), 35_999);
    assert_eq!(STREAM, "attitude.csv");
    assert_eq!(FIELDS[..3], ["heading_cd", "roll_cd", "pitch_cd"]);
}

#[test]
fn a_roll_or_pitch_rounding_to_the_half_turn_is_minus_180() {
    // a wrapped 179.996 rounds to 18000, which plan B's integer roll_cd can never hold: it is -18000, "-180.00"
    let bench = Bench::seeded(10.0, 179.996, 179.994);
    assert_eq!(bench.filter.row()[1..3], [-18_000, 17_999]);
    let inspected = bench.filter.inspect();
    assert_eq!(
        (&inspected["roll"], &inspected["pitch"]),
        (&json!("-180.00"), &json!("179.99"))
    );
    let bench = Bench::seeded(10.0, -179.996, -0.004);
    assert_eq!(
        bench.filter.row()[1..3],
        [-18_000, 0],
        "NEGATIVE: never 18000, and no -0"
    );
    assert_eq!(signed_centidegrees(179.994), 17_999);
    assert_eq!(signed_centidegrees(-180.0), -18_000);
    assert_eq!(signed_centidegrees(359.0), -100, "wrapped as plan B's integer wrap");
}

#[test]
fn the_probe_needs_the_gyro() {
    assert_eq!(Attitude::probe(true), Ok(()));
    assert_eq!(Attitude::probe(false), Err(PROBE_FAILURE));
    assert!(PROBE_FAILURE.contains("blind"));
}

#[test]
fn the_marks_map_is_a_half_turn_about_x() {
    // the D term's (roll, pitch, yaw) rate: about y left the nose goes DOWN, about z up the nose goes LEFT
    assert_eq!(rate_from_marks([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
    let readings = Readings::from_marks(Some([1.0, 2.0, 3.0]), Some([0.1, 0.2, 0.9]), Some([4.0, 5.0, 6.0]));
    assert_eq!(readings.rate, Some([1.0, -2.0, -3.0]));
    assert_eq!(
        readings.acceleration,
        Some([-0.1, 0.2, 0.9]),
        "the half turn, negated into gravity"
    );
    assert_eq!(readings.magnetic, Some([4.0, -5.0, -6.0]));
    // NEGATIVE: absent stays absent, and nothing else is invented
    assert_eq!(Readings::from_marks(None, None, None), Readings::default());
    assert_eq!(
        Readings {
            rate: None,
            acceleration: None,
            magnetic: None,
            ..readings
        },
        Readings::default()
    );
}

#[test]
fn a_part_mounted_per_its_marks_reads_nose_up_as_pitch_up() {
    let marks = |acceleration: [f32; 3]| Readings::from_marks(Some([0.0; 3]), Some(acceleration), None);
    let nose_up = Bench::seeded(0.0, 0.0, 0.0).ticks(40, &marks(marks_at(0.0, 10.0)));
    let nose_up = nose_up.published.unwrap();
    assert!(
        (nose_up.pitch - 10.0).abs() < 0.01 && nose_up.roll.abs() < 0.01,
        "{nose_up:?}"
    );
    let banked = Bench::seeded(0.0, 0.0, 0.0).ticks(40, &marks(marks_at(20.0, 0.0)));
    let banked = banked.published.unwrap();
    assert!(
        (banked.roll - 20.0).abs() < 0.01 && banked.pitch.abs() < 0.01,
        "right wing down: {banked:?}"
    );
    // the gyro agrees with gravity: nose up about y left is a NEGATIVE rate, right wing down about x a positive one
    let gyro = |rate: [f32; 3]| {
        let readings = Readings::from_marks(Some(rate), Some([0.0; 3]), None);
        Bench::seeded(0.0, 0.0, 0.0).ticks(50, &readings).published.unwrap()
    };
    let nosed_up = gyro([0.0, -10.0, 0.0]);
    assert!(
        (nosed_up.pitch - 10.0).abs() < 1e-3 && nosed_up.roll.abs() < 1e-3,
        "{nosed_up:?}"
    );
    let rolled = gyro([10.0, 0.0, 0.0]);
    assert!(
        (rolled.roll - 10.0).abs() < 1e-3 && rolled.pitch.abs() < 1e-3,
        "{rolled:?}"
    );
    // NEGATIVE: the same part fed unmapped reads the nose-up pitch as nose DOWN, with nothing to flag it
    let unmapped = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(marks_at(0.0, 10.0)),
        ..Readings::default()
    };
    let unmapped = Bench::seeded(0.0, 0.0, 0.0).ticks(40, &unmapped).published.unwrap();
    assert!((unmapped.pitch + 10.0).abs() < 0.01, "{unmapped:?}");
}

#[test]
fn a_right_turn_raises_the_heading_with_the_track() {
    // turning right at 20 degrees per second is a NEGATIVE rate about z up; the GNSS track follows the true heading
    let mut mapped = Bench::seeded(90.0, 0.0, 0.0);
    let mut unmapped = Bench::seeded(90.0, 0.0, 0.0);
    for tick in 1..=50 {
        let truth = 90.0 + 20.0 * tick as f32 * 0.02;
        let track = Readings {
            course: Some(truth),
            ground_speed: Some(14.0),
            ..Readings::from_marks(Some([0.0, 0.0, -20.0]), None, None)
        };
        mapped.tick(&track);
        unmapped.tick(&Readings {
            rate: Some([0.0, 0.0, -20.0]),
            ..track
        });
    }
    assert!(
        close(mapped.filter.orientation().heading, 110.0, 0.01),
        "the gyro and the track agree: {:?}",
        mapped.filter.orientation()
    );
    // NEGATIVE: unmapped, the gyro yaw turns left against the track, 0.8 degree a tick apart, and at a 1/4 pull the
    // heading settles 2.4 degrees behind it for good (e = (e + 0.8) * 3/4)
    assert!(
        close(unmapped.filter.orientation().heading, 107.6, 0.01),
        "{:?}",
        unmapped.filter.orientation()
    );
    let free = Bench::seeded(90.0, 0.0, 0.0)
        .ticks(50, &Readings::from_marks(Some([0.0, 0.0, -20.0]), None, None))
        .published
        .unwrap();
    assert!(
        close(free.heading, 110.0, 1e-3),
        "no track: the gyro alone turns right, {free:?}"
    );
}

#[test]
fn a_field_at_the_heading_learns_one_constant_offset() {
    // the nose at heading psi, flown along a track of psi: the offset is the same constant for every psi
    let learned = |mapped: bool, heading: f32| {
        let mut bench = Bench::new(test_settings());
        let level = Readings::from_marks(Some([0.0; 3]), Some(marks_at(0.0, 0.0)), Some(marks_field(heading)));
        let field = if mapped {
            level.magnetic
        } else {
            Some(marks_field(heading))
        };
        let readings = Readings {
            course: Some(heading),
            ground_speed: Some(14.0),
            magnetic: field,
            ..level
        };
        bench.ticks(40, &readings);
        assert!(bench.filter.magnetic().known());
        bench.filter.magnetic().offset()
    };
    for heading in [30.0, 120.0, 250.0, 340.0] {
        assert!(
            close(learned(true, heading), 0.0, 0.01),
            "{heading}: {}",
            learned(true, heading)
        );
    }
    // NEGATIVE: unmapped, the magnetic heading runs backwards and the "constant" learned is twice the heading
    for heading in [30.0, 120.0, 250.0] {
        assert!(
            close(learned(false, heading), 2.0 * heading, 0.01),
            "{heading}: {}",
            learned(false, heading)
        );
    }
}

#[test]
fn the_accel_pull_takes_the_short_way_round() {
    // estimate roll 170, gravity at roll -170 (inverted, inside the 1 g band): 20 degrees apart the short way. Plan B's
    // blend_cd took the unwrapped -340 and swept through wings-level (85, 21, -27, ...); the tilt steps 5 at a time
    let mut bench = Bench::seeded(0.0, 170.0, 0.0);
    let inverted = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(-170.0, 0.0)),
        ..Readings::default()
    };
    let rolls: Vec<f32> = (0..6).map(|_| bench.tick(&inverted).published.unwrap().roll).collect();
    for (roll, owed) in rolls.iter().zip([175.0, 178.75, -178.44, -176.33, -174.75, -173.56]) {
        assert!(close(*roll, owed, 0.01), "{rolls:?}");
    }
    // NEGATIVE: never through wings-level
    assert!(rolls.iter().all(|roll| roll.abs() > 170.0), "{rolls:?}");
}

#[test]
fn the_level_gate_reads_the_roll_before_the_gyro_step() {
    // roll 14.5 under the 15 degree gate, rolling at 50 degrees per second: plan B gates on the 14.5 it had BEFORE
    // this tick's 1 degree, so it learns, although the roll after the tick is 15.5
    let mut bench = Bench::seeded(0.0, 14.5, 0.0);
    let rolling = Readings {
        rate: Some([50.0, 0.0, 0.0]),
        course: Some(90.0),
        ground_speed: Some(14.0),
        magnetic: Some([1000.0, 0.0, 0.0]),
        ..Readings::default()
    };
    bench.tick(&rolling);
    assert!(bench.filter.magnetic().known(), "gated on 14.5: learned");
    assert!(close(bench.filter.magnetic().offset(), 45.0, 1e-3));
    assert!((bench.filter.orientation().roll - 15.5).abs() < 1e-4);
    // NEGATIVE: the next tick starts at 15.5, past the gate: nothing learned, although it ends at 16.5
    bench.tick(&rolling);
    assert!(close(bench.filter.magnetic().offset(), 45.0, 1e-3));
    // and rolling back, 16.5 -> 15.5 refuses, 15.5 -> 14.5 refuses (its start is past the gate), 14.5 learns
    let back = Readings {
        rate: Some([-50.0, 0.0, 0.0]),
        ..rolling
    };
    bench.ticks(2, &back);
    assert!(close(bench.filter.magnetic().offset(), 45.0, 1e-3));
    bench.tick(&back);
    assert!(close(bench.filter.magnetic().offset(), 67.5, 1e-3));
}

#[test]
fn the_track_needs_the_course_and_the_speed() {
    // NEGATIVE: either alone is no track (plan B drops both): no pull, and nothing learned against it
    let mut bench = Bench::seeded(45.0, 0.0, 0.0);
    let level = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        magnetic: Some([1000.0, 0.0, 0.0]),
        ..Readings::default()
    };
    bench.ticks(
        5,
        &Readings {
            course: Some(90.0),
            ..level
        },
    );
    bench.ticks(
        5,
        &Readings {
            ground_speed: Some(14.0),
            ..level
        },
    );
    assert!(close(bench.filter.orientation().heading, 45.0, 1e-3));
    assert!(!bench.filter.magnetic().known());
    // both: the track pulls and the offset is learned
    bench.tick(&Readings {
        course: Some(90.0),
        ground_speed: Some(14.0),
        ..level
    });
    assert!(close(bench.filter.orientation().heading, 56.25, 1e-3));
    assert!(bench.filter.magnetic().known());
}

#[test]
fn the_course_gate_is_strict() {
    // plan B's `speed > gate`: exactly 5 m/s at a 5 m/s gate is not moving
    let moving = |speed: f32| {
        let mut bench = Bench::seeded(45.0, 0.0, 0.0);
        bench.tick(&Readings {
            rate: Some([0.0; 3]),
            course: Some(90.0),
            ground_speed: Some(speed),
            ..Readings::default()
        });
        bench.filter.orientation().heading
    };
    assert!(close(moving(5.0), 45.0, 1e-3), "NEGATIVE: on the gate, no pull");
    assert!(close(moving(5.0_f32.next_up()), 56.25, 1e-3), "one f32 step over pulls");
}

#[test]
fn the_gravity_band_is_inclusive() {
    // plan B compares its centi-g integers `low^2 <= |a|^2 <= high^2`, so 0.7 g and 1.3 g pull. An axis-aligned
    // vector's f32 square is the band edge's own expression, so the edge is met exactly; a vector on the edge in two
    // axes is decided by an f32 rounding (the golden fixture keeps its readings off the edges for that reason)
    let edge = Bench::new(test_settings()).filter;
    assert_eq!(0.0_f32 * 0.0 + (0.0_f32 * 0.0 + 0.7_f32 * 0.7), 0.7_f32 * 0.7);
    for inside in [0.7, 1.3] {
        assert!(edge.in_band([0.0, 0.0, inside]), "{inside} g pulls");
        let mut bench = Bench::seeded(0.0, 10.0, 0.0);
        bench.tick(&Readings {
            rate: Some([0.0; 3]),
            acceleration: Some([0.0, 0.0, inside]),
            ..Readings::default()
        });
        assert!((bench.filter.orientation().roll - 7.5).abs() < 1e-5);
    }
    for outside in [0.69, 1.31] {
        assert!(!edge.in_band([0.0, 0.0, outside]), "NEGATIVE: {outside} g does not");
    }
}

#[test]
fn a_clock_a_fraction_of_a_millisecond_back_takes_the_nominal_period() {
    // plan B floors the elapsed microseconds: -500 us is -1 ms, so the nominal period, not a tiny backward step
    let mut bench = Bench::seeded(0.0, 0.0, 0.0);
    let rolling = Readings {
        rate: Some([10.0, 0.0, 0.0]),
        acceleration: Some([0.0; 3]),
        ..Readings::default()
    };
    bench.tick(&rolling);
    bench.now_us -= 500;
    bench.filter.tick(bench.now_us, &rolling);
    assert!(
        (bench.filter.orientation().roll - 0.4).abs() < 1e-5,
        "{:?}",
        bench.filter.orientation()
    );
    // NEGATIVE: the next tick, a whole period on, integrates its own 20 ms only
    bench.tick(&rolling);
    assert!((bench.filter.orientation().roll - 0.6).abs() < 1e-5);
}
