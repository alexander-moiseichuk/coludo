//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the attitude filter, after src/glider/test/test_attitude.py and test_magnetic_yaw.py: the mirror
//! (outranking sources only), the free run (gyro, gravity, the turn gate, the track, the magnetometer), the blind
//! window and the withhold, the gap rule, the boundary units, and what the port does where plan B could not (a
//! non-finite reading, a config of the wrong type). The golden fixtures (tests/golden_attitude.rs,
//! tests/golden_magnetic_yaw.rs) hold the trajectories to plan B; these hold the rules, each with what must NOT happen.

use super::{
    Attitude, BLIND_CYCLES, FIELDS, Fused, GYRO_BACK, GYRO_LOST, MagneticYaw, Orientation, PROBE_FAILURE, Readings,
    STREAM, Settings, decimal_text, heading_centidegrees, pull, rate_from_marks, signed_centidegrees, wrap_heading,
    wrap_signed,
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
}

#[test]
fn a_config_of_the_wrong_type_takes_the_defaults() {
    // NEGATIVE: plan B stores whatever arrives and raises where it is used; the port keeps its defaults
    let odd = Settings::from_json(&json!({
        "period_ms": "20", "corr_shift": 2.5, "course_shift": -1, "turn_gate_deg_s": null, "grav_low_g": true,
        "provides": {"attitude": {"priority": 300}}
    }));
    assert_eq!(odd, Settings::default());
    assert_eq!(Settings::from_json(&json!([1, 2])), Settings::default());
    assert_eq!(Settings::from_json(&json!({"provides": 3})).rank, 0);
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
    let mut bench = Bench::seeded(0.0, 0.0, 0.0);
    let readings = Readings {
        rate: Some([10.0, -5.0, 20.0]),
        acceleration: Some([0.0; 3]), // present but outside the band: no correction
        ..Readings::default()
    };
    let published = bench.ticks(50, &readings).published.unwrap();
    assert!((published.roll - 10.0).abs() < 1e-3, "{published:?}");
    assert!((published.pitch + 5.0).abs() < 1e-3);
    assert!((published.heading - 20.0).abs() < 1e-3);
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
    // a coordinated turn: banked 40 with the accelerometer reading level and a 30 degree/s yaw rate
    let mut bench = Bench::seeded(0.0, 40.0, 0.0);
    let turning = Readings {
        rate: Some([0.0, 0.0, 30.0]),
        acceleration: Some(gravity_at(0.0, 0.0)),
        ..Readings::default()
    };
    bench.ticks(3, &turning);
    assert_eq!(
        bench.filter.orientation().roll,
        40.0,
        "NEGATIVE: the turn must not roll the estimate flat"
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
    assert_eq!(slow.filter.orientation().heading, 45.0);
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
    let gyro = Readings::from_marks(Some([10.0, -10.0, 0.0]), Some([0.0; 3]), None);
    let turned = Bench::seeded(0.0, 0.0, 0.0).ticks(50, &gyro).published.unwrap();
    assert!(
        (turned.pitch - 10.0).abs() < 1e-3 && (turned.roll - 10.0).abs() < 1e-3,
        "{turned:?}"
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
fn the_accel_pull_takes_the_long_way_round_as_plan_b() {
    // a plan B hazard kept on purpose: estimate roll 170, gravity at roll -170 (inverted, inside the 1 g band). The
    // short way is 20 degrees; blend_cd's unwrapped error is -340, so the estimate sweeps through wings-level
    let mut bench = Bench::seeded(0.0, 170.0, 0.0);
    let inverted = Readings {
        rate: Some([0.0; 3]),
        acceleration: Some(gravity_at(-170.0, 0.0)),
        ..Readings::default()
    };
    let rolls: Vec<f32> = (0..6).map(|_| bench.tick(&inverted).published.unwrap().roll).collect();
    for (roll, owed) in rolls.iter().zip([85.0, 21.25, -26.5625, -62.42, -89.32, -109.49]) {
        assert!((roll - owed).abs() < 0.01, "{rolls:?}");
    }
    // NEGATIVE: not the wrapped error's short step to 175
    assert!(!close(rolls[0], 175.0, 1.0));
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
    assert_eq!(bench.filter.orientation().heading, 45.0);
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
    assert_eq!(moving(5.0), 45.0, "NEGATIVE: on the gate, no pull");
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
        assert!(edge.gravity([0.0, 0.0, inside]).is_some(), "{inside} g pulls");
        let mut bench = Bench::seeded(0.0, 10.0, 0.0);
        bench.tick(&Readings {
            rate: Some([0.0; 3]),
            acceleration: Some([0.0, 0.0, inside]),
            ..Readings::default()
        });
        assert!((bench.filter.orientation().roll - 7.5).abs() < 1e-5);
    }
    for outside in [0.69, 1.31] {
        assert_eq!(
            edge.gravity([0.0, 0.0, outside]),
            None,
            "NEGATIVE: {outside} g does not"
        );
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
