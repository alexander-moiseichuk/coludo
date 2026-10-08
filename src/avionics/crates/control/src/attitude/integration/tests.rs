//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Unit tests of the filter on the board, over a real databoard: what a step reads ([`assemble`]) and the marks map
//! it applies, the backup IMU carrying the inputs, the winner the filter mirrors or never mirrors, the backup it
//! withholds for; the pacing on the control tick ([`Pace`]); and the health's `attitude-backup` ([`on_backup`]),
//! whose plan B answers the golden fixture pins (tests/golden_on_backup.rs). Each rule with what must NOT happen.

use databoard::Parameter;
use serde_json::{Value, json};

use super::{Pace, Quantities, assemble, on_backup};
use crate::attitude::{Attitude, BLIND_CYCLES, GYRO_LOST, Orientation, Readings, Settings, Tick};

/// The filter's step period in the tests, microseconds (its 50 Hz).
const STEP_US: u64 = 20_000;

/// The control tick in the tests, microseconds (100 Hz).
const TICK_US: u64 = 10_000;

/// A v1.x board's quantities as the topology registers them, at config_default's ranks and timeouts: `rate` and
/// `accel` from the LSM6DSO32 (p0) and the BMI323 (p1), 20 ms; `mag` from the BMM350, 500 ms; `attitude` from the
/// filter (p1) and the SEN0253's BNO055 (p2), 40 ms.
struct Board {
    /// `rate`, degrees per second in the marks frame.
    rate: Parameter<[f32; 3]>,
    /// `accel`, g in the marks frame.
    acceleration: Parameter<[f32; 3]>,
    /// `mag`, in the marks frame.
    magnetic: Parameter<[f32; 3]>,
    /// `attitude`, (heading, roll, pitch) degrees.
    attitude: Parameter<[f32; 3]>,
}

impl Board {
    /// The board, nothing pushed yet.
    fn new() -> Board {
        let mut board = Board {
            rate: Parameter::new("rate"),
            acceleration: Parameter::new("accel"),
            magnetic: Parameter::new("mag"),
            attitude: Parameter::new("attitude"),
        };
        for parameter in [&mut board.rate, &mut board.acceleration] {
            parameter.add_source("imu_lsm6dso32", 0, 20, false).unwrap();
            parameter.add_source("imu_bmi323", 1, 20, false).unwrap();
        }
        board.magnetic.add_source("mag_bmm350", 0, 500, false).unwrap();
        board.attitude.add_source("attitude", 1, 40, false).unwrap();
        board.attitude.add_source("imu_bno055", 2, 40, false).unwrap();
        board
    }

    /// The quantities the filter reads, every one present.
    fn quantities(&self) -> Quantities<'_> {
        Quantities {
            rate: Some(&self.rate),
            acceleration: Some(&self.acceleration),
            magnetic: Some(&self.magnetic),
            attitude: &self.attitude,
        }
    }
}

/// Pushes `value` from `source` into `parameter` at `now_us`, as the part's writer does.
fn push(parameter: &Parameter<[f32; 3]>, source: &str, value: [f32; 3], now_us: u64) {
    parameter
        .writer(source)
        .expect("a registered source, unclaimed")
        .push(value, now_us);
}

/// The LSM6DSO32 at `now_us` reading the gyro `rate` and the specific force `acceleration`, in the marks frame.
fn primary_imu(board: &Board, rate: [f32; 3], acceleration: [f32; 3], now_us: u64) {
    push(&board.rate, "imu_lsm6dso32", rate, now_us);
    push(&board.acceleration, "imu_lsm6dso32", acceleration, now_us);
}

/// The filter's settings in the tests: config_default's rank, a fast blend (corr_shift 2) pulling on every step
/// (accel_period_ms 0), and a fast-learned magnetic offset (mag_offset_shift 1).
fn settings() -> Settings {
    Settings::from_json(&json!({
        "accel_period_ms": 0, "corr_shift": 2, "mag_offset_shift": 1,
        "provides": {"attitude": {"priority": 1, "timeout_ms": 40}}
    }))
}

/// Whether two angles agree within `tolerance` degrees, the short way around.
fn close(first: f32, second: f32, tolerance: f32) -> bool {
    ((first - second + 180.0).rem_euclid(360.0) - 180.0).abs() <= tolerance
}

/// The horizontal field a level part mounted per its marks reads with the nose at `heading` degrees clockwise from
/// magnetic north: north is then `heading` to the LEFT of the nose, +y in the marks frame, and the field dips down.
fn marks_field(heading: f32) -> [f32; 3] {
    let heading = heading.to_radians();
    [1000.0 * heading.cos(), 1000.0 * heading.sin(), -400.0]
}

/// The control thread in miniature: each filter step the parts push, then the filter steps on what [`assemble`]
/// reads at that moment and pushes what it publishes, as the firmware integration does.
struct Thread {
    /// The board.
    board: Board,
    /// The filter, publishing as `attitude`.
    filter: Attitude,
    /// The clock, microseconds.
    now_us: u64,
}

impl Thread {
    /// A filter set up at 0 on `board`.
    fn new(board: Board) -> Thread {
        Thread {
            board,
            filter: Attitude::new(settings(), 0),
            now_us: 0,
        }
    }

    /// One step 20 ms on: `parts` pushes the parts' readings at the step's moment, then the filter reads, steps and
    /// pushes what it publishes; its answer.
    fn step(&mut self, parts: &impl Fn(&Board, u64)) -> Tick {
        self.now_us += STEP_US;
        parts(&self.board, self.now_us);
        let readings = assemble(&self.board.quantities(), self.filter.settings().rank, self.now_us);
        let tick = self.filter.tick(self.now_us, &readings);
        if let Some(orientation) = tick.published {
            push(&self.board.attitude, "attitude", orientation.to_sample(), self.now_us);
        }
        tick
    }

    /// `count` steps of the same parts; the last step's answer.
    fn steps(&mut self, count: usize, parts: &impl Fn(&Board, u64)) -> Tick {
        let mut last = self.step(parts);
        for _ in 1..count {
            last = self.step(parts);
        }
        last
    }

    /// What the filter would read now, the step's clock unmoved.
    fn readings(&self) -> Readings {
        assemble(&self.board.quantities(), self.filter.settings().rank, self.now_us)
    }
}

/// The LSM6DSO32 still and level, per its marks.
fn level(board: &Board, now_us: u64) {
    primary_imu(board, [0.0; 3], [0.0, 0.0, 1.0], now_us);
}

#[test]
fn a_step_reads_the_fused_inputs_mapped_from_the_marks() {
    let board = Board::new();
    primary_imu(&board, [1.0, 2.0, 3.0], [0.1, 0.2, 0.9], 1_000);
    push(&board.magnetic, "mag_bmm350", [4.0, 5.0, 6.0], 1_000);
    let readings = assemble(&board.quantities(), 1, 5_000);
    assert_eq!(readings.rate, Some([1.0, -2.0, -3.0]));
    assert_eq!(
        readings.acceleration,
        Some([-0.1, 0.2, 0.9]),
        "the half turn, negated into gravity"
    );
    assert_eq!(readings.magnetic, Some([4.0, -5.0, -6.0]));
    assert_eq!(
        (readings.course, readings.ground_speed),
        (None, None),
        "no GNSS until it is ported"
    );
    assert_eq!((readings.fused, readings.backup_fresh), (None, false));
    // NEGATIVE: past the IMUs' 20 ms window the gyro and the accelerometer are absent, never extrapolated, while the
    // magnetometer's 500 ms window still holds
    let later = assemble(&board.quantities(), 1, 21_001);
    assert_eq!((later.rate, later.acceleration), (None, None));
    assert_eq!(later.magnetic, Some([4.0, -5.0, -6.0]));
    // NEGATIVE: a push stamped after the read is not fresh, so the read comes after the tick's pushes
    assert_eq!(assemble(&board.quantities(), 1, 999), Readings::default());
    // NEGATIVE: a quantity no provider of came up is absent, the others unaffected
    let no_gyro = Quantities {
        rate: None,
        ..board.quantities()
    };
    let readings = assemble(&no_gyro, 1, 5_000);
    assert_eq!(readings.rate, None);
    assert_eq!(readings.acceleration, Some([-0.1, 0.2, 0.9]));
}

#[test]
fn the_bmi323_carries_the_inputs_while_the_lsm6dso32_is_stale() {
    let board = Board::new();
    primary_imu(&board, [1.0; 3], [0.0, 0.0, 1.0], 0);
    push(&board.rate, "imu_bmi323", [2.0; 3], 10_000);
    push(&board.acceleration, "imu_bmi323", [0.0, 0.0, 0.5], 10_000);
    // NEGATIVE: both fresh, the primary wins although the backup pushed later
    let both = assemble(&board.quantities(), 1, 15_000);
    assert_eq!(both.rate, Some([1.0, -1.0, -1.0]));
    assert_eq!(both.acceleration, Some([0.0, 0.0, 1.0]));
    // the LSM6DSO32 past its 20 ms window: the BMI323 carries the gyro and the accelerometer
    let failover = assemble(&board.quantities(), 1, 25_000);
    assert_eq!(failover.rate, Some([2.0, -2.0, -2.0]));
    assert_eq!(failover.acceleration, Some([0.0, 0.0, 0.5]));
    // and hands them back the moment the primary pushes again
    primary_imu(&board, [3.0; 3], [0.0, 0.0, 1.0], 26_000);
    assert_eq!(assemble(&board.quantities(), 1, 27_000).rate, Some([3.0, -3.0, -3.0]));
    // NEGATIVE: both stale, no gyro at all
    assert_eq!(assemble(&board.quantities(), 1, 50_000).rate, None);
}

#[test]
fn a_nose_up_part_reads_pitch_up_ninety() {
    // nose straight up, the part reads +1 g along x, its marks' forward: the filter's pitch is +90
    let pitch = |specific_force: [f32; 3]| {
        let mut thread = Thread::new(Board::new());
        let parts = |board: &Board, now_us: u64| primary_imu(board, [0.0; 3], specific_force, now_us);
        let published = thread.steps(40, &parts).published.expect("published");
        assert!(published.roll.abs() < 0.01, "{published:?}");
        published.pitch
    };
    assert!((pitch([1.0, 0.0, 0.0]) - 90.0).abs() < 0.01);
    // NEGATIVE: nose straight down reads -90, never up
    assert!((pitch([-1.0, 0.0, 0.0]) + 90.0).abs() < 0.01);
}

#[test]
fn a_positive_rate_about_z_up_turns_the_heading_left() {
    // about z up the right-hand rule turns the nose LEFT: 20 degrees per second for a second, the heading falls 20
    let heading = |yaw_rate: f32| {
        let mut thread = Thread::new(Board::new());
        let parts = |board: &Board, now_us: u64| primary_imu(board, [0.0, 0.0, yaw_rate], [0.0, 0.0, 1.0], now_us);
        thread.steps(50, &parts).published.expect("published").heading
    };
    assert!(close(heading(20.0), 340.0, 1e-3), "{}", heading(20.0));
    // NEGATIVE: a negative rate turns right
    assert!(close(heading(-20.0), 20.0, 1e-3), "{}", heading(-20.0));
}

#[test]
fn the_field_maps_to_the_magnetic_heading() {
    let reference = Attitude::new(settings(), 0);
    for heading in [30.0, 120.0, 250.0, 340.0] {
        let board = Board::new();
        push(&board.magnetic, "mag_bmm350", marks_field(heading), 0);
        let readings = assemble(&board.quantities(), 1, 1_000);
        let magnetic = reference.magnetic().heading(readings.magnetic, 0.0, 0.0);
        assert!(
            magnetic.is_some_and(|magnetic| close(magnetic, heading, 1e-3)),
            "{heading}: {magnetic:?}"
        );
        // NEGATIVE: the marks field unmapped runs the heading backwards
        let unmapped = reference.magnetic().heading(Some(marks_field(heading)), 0.0, 0.0);
        assert!(
            unmapped.is_some_and(|unmapped| close(unmapped, 360.0 - heading, 1e-3)),
            "{heading}: {unmapped:?}"
        );
    }
}

#[test]
fn a_nan_magnetometer_is_ignored() {
    // the GNSS track, which the step does not read until the GNSS is ported, added by hand as it will come
    let learn = |field: Option<[f32; 3]>| {
        let board = Board::new();
        let mut filter = Attitude::new(settings(), 0);
        for step in 1..=20 {
            let now_us = step * STEP_US;
            level(&board, now_us);
            if let Some(field) = field {
                push(&board.magnetic, "mag_bmm350", field, now_us);
            }
            let readings = Readings {
                course: Some(90.0),
                ground_speed: Some(14.0),
                ..assemble(&board.quantities(), 1, now_us)
            };
            filter.tick(now_us, &readings);
        }
        filter
    };
    // a finite field at heading 30 along a track of 90 learns the 60 between them
    let finite = learn(Some(marks_field(30.0)));
    assert!(finite.magnetic().known());
    assert!(
        close(finite.magnetic().offset(), 60.0, 0.01),
        "{}",
        finite.magnetic().offset()
    );
    // NEGATIVE: a NaN or an infinite field is no field: nothing learned, and the attitude exactly as with none
    let without = learn(None);
    for field in [[f32::NAN, 0.0, 0.0], [0.0, f32::INFINITY, 0.0]] {
        let filter = learn(Some(field));
        assert!(!filter.magnetic().known());
        assert_eq!(filter.orientation(), without.orientation(), "{field:?}");
    }
}

#[test]
fn a_p0_winner_is_mirrored() {
    let mut board = Board::new();
    board.attitude.add_source("hitl", 0, 40, false).unwrap();
    let mut thread = Thread::new(board);
    let simulated = |board: &Board, now_us: u64| {
        push(&board.rate, "imu_lsm6dso32", [0.0; 3], now_us);
        push(&board.attitude, "hitl", [123.0, 45.0, -6.0], now_us);
    };
    let tick = thread.step(&simulated);
    assert_eq!(
        tick.published,
        Some(Orientation {
            heading: 123.0,
            roll: 45.0,
            pitch: -6.0
        })
    );
    assert!(thread.filter.seeded() && !thread.filter.free());
    // the sim goes quiet: past its window the filter's own push wins, at its own rank, and it free-runs from the copy
    // with no accelerometer at all
    let quiet = |board: &Board, now_us: u64| push(&board.rate, "imu_lsm6dso32", [0.0; 3], now_us);
    let tick = thread.steps(3, &quiet);
    assert!(thread.filter.free());
    assert_eq!(tick.published.map(|orientation| orientation.roll), Some(45.0));
    let own = thread.readings().fused.expect("the filter's own push");
    assert_eq!(own.rank, 1);
    // what the filter pushes reads back as the attitude it published: (heading, roll, pitch), plan B's order
    assert_eq!(Some(own.orientation), tick.published);
}

#[test]
fn a_p2_winner_is_never_mirrored() {
    let mut thread = Thread::new(Board::new());
    let backup = |board: &Board, now_us: u64| {
        push(&board.rate, "imu_lsm6dso32", [0.0; 3], now_us);
        push(&board.attitude, "imu_bno055", [77.0, 25.0, 15.0], now_us);
    };
    // NEGATIVE: the BNO055 wins the read, ranked below the filter: no seed from it, and no accelerometer, so nothing
    let tick = thread.step(&backup);
    assert_eq!(thread.readings().fused.map(|fused| fused.rank), Some(2));
    assert_eq!(tick.published, None);
    assert!(!thread.filter.seeded());
    // with the accelerometer the filter runs from its own zero and becomes the winner
    let running = |board: &Board, now_us: u64| {
        backup(board, now_us);
        level(board, now_us);
    };
    let tick = thread.steps(5, &running);
    let published = tick.published.expect("published");
    assert!(
        close(published.heading, 0.0, 0.01) && close(published.roll, 0.0, 0.01),
        "{published:?}"
    );
    assert_eq!(thread.readings().fused.map(|fused| fused.rank), Some(1));
    // NEGATIVE: a stall makes the filter late, the backup wins that step's read, and it is still never copied
    thread.now_us += 3 * STEP_US;
    let tick = thread.step(&running);
    let published = tick.published.expect("published");
    assert!(
        close(published.heading, 0.0, 0.01) && close(published.roll, 0.0, 0.01),
        "{published:?}"
    );
    assert!(!thread.filter.seeded() && thread.filter.free());
}

#[test]
fn a_blind_filter_withholds_only_while_a_backup_below_is_fresh() {
    let blind = |with_backup: bool| {
        let mut thread = Thread::new(Board::new());
        let parts = move |board: &Board, now_us: u64, gyro: bool| {
            if gyro {
                push(&board.rate, "imu_lsm6dso32", [0.0; 3], now_us);
            }
            push(&board.acceleration, "imu_lsm6dso32", [0.0, 0.0, 1.0], now_us);
            if with_backup {
                push(&board.attitude, "imu_bno055", [10.0, 0.0, 0.0], now_us);
            }
        };
        thread.steps(3, &|board: &Board, now_us: u64| parts(board, now_us, true));
        let gyro_lost = |board: &Board, now_us: u64| parts(board, now_us, false);
        // the gyro's last push still reads fresh one step on, on the edge of its 20 ms window: then BLIND_CYCLES - 1
        let before = thread.steps(BLIND_CYCLES as usize, &gyro_lost);
        assert!(before.published.is_some() && !thread.filter.blind());
        let lost = thread.step(&gyro_lost);
        assert!(thread.filter.blind());
        assert_eq!(lost.log, Some(GYRO_LOST));
        let after = thread.steps(3, &gyro_lost);
        (lost, after, thread.readings())
    };
    // a fresh BNO055 below: withheld, so the filter's own channel goes stale and the backup carries the attitude
    let (lost, after, readings) = blind(true);
    assert_eq!((lost.published, after.published), (None, None));
    assert!(readings.backup_fresh);
    assert_eq!(readings.fused.map(|fused| fused.rank), Some(2));
    // NEGATIVE: nothing fresh below, it keeps publishing, its own channel the winner
    let (lost, after, readings) = blind(false);
    assert!(lost.published.is_some() && after.published.is_some());
    assert!(!readings.backup_fresh);
    assert_eq!(readings.fused.map(|fused| fused.rank), Some(1));
}

/// The control ticks among `wakes` (microseconds) on which a pace of `period_ms` steps the filter.
fn stepped(period_ms: u32, wakes: impl IntoIterator<Item = u64>) -> Vec<u64> {
    let mut pace = Pace::new(period_ms, TICK_US);
    wakes.into_iter().filter(|wake| pace.due(*wake)).collect()
}

/// The first wakes the pacing tests start from, microseconds of uptime: within half a tick of 0, and past it (the
/// board's first tick comes seconds into its uptime).
const STARTS_US: [u64; 3] = [1_000, 7_000, 3_000_007];

/// A second of 10 ms ticks from `start_us`, each waking up to 400 us early or late.
fn jittered_second(start_us: u64) -> impl Iterator<Item = u64> {
    let jitter: [i64; 5] = [0, 400, -400, 250, -250];
    (0..100_u64).map(move |tick| (start_us + tick * TICK_US).saturating_add_signed(jitter[tick as usize % 5]))
}

#[test]
fn twenty_milliseconds_is_every_second_tick_through_the_jitter() {
    for start_us in STARTS_US {
        let wakes: Vec<u64> = jittered_second(start_us).collect();
        let steps = stepped(20, wakes.iter().copied());
        let even: Vec<u64> = wakes.iter().copied().step_by(2).collect();
        // NEGATIVE: the grid's edges never sit on a wake, where the jitter would pick the tick
        assert_eq!(steps, even, "from {start_us}: never a third tick, never two in a row");
        // a period that is no multiple of the tick averages out to its own rate, 66.7 a second
        let fifteen = stepped(15, wakes.iter().copied()).len();
        assert!((66..=67).contains(&fifteen), "from {start_us}: {fifteen}");
    }
}

#[test]
fn a_period_below_the_tick_steps_every_tick() {
    // NEGATIVE: the tick is the fastest the filter runs; a shorter period neither skips a tick nor asks for more
    for start_us in STARTS_US {
        for period_ms in [0, 5, 10] {
            let steps = stepped(period_ms, jittered_second(start_us)).len();
            assert_eq!(steps, 100, "{period_ms} ms from {start_us}");
        }
    }
}

#[test]
fn the_edges_of_half_a_tick_are_on_the_grid() {
    // a wake exactly half a tick early steps: 15 ms on exact 10 ms wakes steps at 0, 10 (due 15), 30, 40 (due 45)
    let exact = (0..7).map(|tick| tick * TICK_US);
    assert_eq!(stepped(15, exact), [0, 10_000, 30_000, 40_000, 60_000]);
    // a wake exactly half a tick late keeps the grid: 25 ms is due at 20, so the next is due at 40, and 35 steps
    let late = [0, 25_000, 35_000, 45_000, 55_000, 65_000];
    assert_eq!(stepped(20, late), [0, 25_000, 35_000, 55_000]);
}

#[test]
fn a_stall_starts_a_new_grid_without_a_burst() {
    // ticks at 0, 10, 20 ms, then nothing until a late wake, then on the 10 ms grid again: the late wake between the
    // timer's edges (75 ms) and on one (70 ms)
    let between = [0, 10_000, 20_000, 75_000, 80_000, 90_000, 100_000, 110_000];
    assert_eq!(stepped(20, between), [0, 20_000, 75_000, 90_000, 110_000]);
    let on_edge = [0, 10_000, 20_000, 70_000, 80_000, 90_000, 100_000, 110_000, 120_000];
    // NEGATIVE: no catch-up step at 80 for the ones the stall missed; the new grid is a period on from the late wake
    assert_eq!(stepped(20, on_edge), [0, 20_000, 70_000, 90_000, 110_000]);
}

/// A config with the filter (p1) and the SEN0253's BNO055 (p2) providing `attitude`, the BNO055 `enabled` as
/// given, and the LSM6DSO32 providing `accel` and `rate`.
fn config(fitted: Value) -> Value {
    json!({
        "sensors": [
            {"name": "imu_lsm6dso32", "driver": "lsm6dso32", "enabled": true,
             "provides": {"accel": {"priority": 0, "timeout_ms": 20}, "rate": {"priority": 0, "timeout_ms": 20}}},
            {"name": "imu_bno055", "driver": "bno055", "enabled": fitted,
             "provides": {"attitude": {"priority": 2, "timeout_ms": 40}}},
        ],
        "components": [
            {"name": "attitude", "activity": "attitude", "enabled": true,
             "provides": {"attitude": {"priority": 1, "timeout_ms": 40}}},
        ],
    })
}

#[test]
fn the_bno055_standing_in_for_the_filter_is_a_backup() {
    let fitted = config(json!(true));
    assert!(on_backup(&fitted, "attitude", Some("imu_bno055")));
    // NEGATIVE: the filter itself is the primary, and so is the only accel provider
    assert!(!on_backup(&fitted, "attitude", Some("attitude")));
    assert!(!on_backup(&fitted, "accel", Some("imu_lsm6dso32")));
    // NEGATIVE: a channel nobody provides has no backup
    assert!(!on_backup(&fitted, "agl", Some("imu_bno055")));
}

#[test]
fn no_better_enabled_provider_is_no_backup() {
    // NEGATIVE: the filter disabled, the BNO055 is the best there is
    let mut no_filter = config(json!(true));
    no_filter["components"][0]["enabled"] = json!(false);
    assert!(!on_backup(&no_filter, "attitude", Some("imu_bno055")));
    // NEGATIVE: a disabled source never flies as anything, and the source the config does not declare (the HITL sim)
    // is no fallback
    assert!(!on_backup(&config(json!(false)), "attitude", Some("imu_bno055")));
    assert!(!on_backup(&config(json!(true)), "attitude", Some("hitl")));
    // NEGATIVE: nothing fresh is a different problem, and a tie is no fallback
    assert!(!on_backup(&config(json!(true)), "attitude", None));
    let mut tie = config(json!(true));
    tie["sensors"][1]["provides"]["attitude"]["priority"] = json!(1);
    assert!(!on_backup(&tie, "attitude", Some("imu_bno055")));
}

#[test]
fn an_unreadable_config_hides_no_fallback() {
    for unreadable in [json!(null), json!([1, 2]), json!("config")] {
        assert!(on_backup(&unreadable, "attitude", Some("imu_bno055")), "{unreadable}");
        // NEGATIVE: nothing fresh is still no fallback
        assert!(!on_backup(&unreadable, "attitude", None), "{unreadable}");
    }
}

#[test]
fn the_providers_are_the_ones_the_bring_up_registers() {
    // a provider with no name never comes up, so it outranks nothing (plan B's walk would count it; validate()
    // refuses one)
    let mut unnamed = config(json!(true));
    unnamed["sensors"]
        .as_array_mut()
        .unwrap()
        .push(json!({"driver": "bno055", "provides": {"attitude": {"priority": 0, "timeout_ms": 40}}}));
    assert!(!on_backup(&unnamed, "attitude", Some("attitude")));
    // a priority reads as the registration reads it: a bool is Python's int, a float is 0
    let mut ranked = config(json!(true));
    ranked["components"][0]["provides"]["attitude"]["priority"] = json!(true);
    assert!(on_backup(&ranked, "attitude", Some("imu_bno055")), "true is 1, above 2");
    ranked["components"][0]["provides"]["attitude"]["priority"] = json!(2.5);
    assert!(on_backup(&ranked, "attitude", Some("imu_bno055")), "2.5 reads 0");
    ranked["sensors"][1]["provides"]["attitude"]["priority"] = json!(false);
    assert!(!on_backup(&ranked, "attitude", Some("imu_bno055")), "false is 0, a tie");
}
