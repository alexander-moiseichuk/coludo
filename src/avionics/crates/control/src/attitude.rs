//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The PRIMARY attitude filter, from src/glider tasks/attitude.py (`Attitude`): a complementary filter over the
//! SEN0697 and the LSM6DSO32 (doc/specs/coludo.md "Sensors Fusion/Backup"). It derives (heading, roll, pitch) from
//! the gyro `rate` (the LSM6DSO32, backed up by the BMI323) and the accelerometer's gravity vector, with the BMM350
//! magnetometer and the GNSS ground track as its heading references, and publishes it on the `attitude` channel at
//! PRIORITY 1. It began as the backup to the BNO055; since 2026-10-05 it is the primary on every board, and the
//! SEN0253's BNO055, where one is still fitted, is the BACKUP below it at priority 2. Priority 0 is the HITL sim's.
//!
//! Two regimes, chosen each tick by the fused attitude's winner ([`Attitude::tick`]):
//!  * a source that OUTRANKS the filter is winning (the HITL sim): MIRROR it, so the filter stays warm and the
//!    handover is seamless the moment that source goes quiet. A source ranked BELOW it (the BNO055) is NEVER
//!    mirrored, even when it wins a tick because the filter was late: copying it would overwrite the primary's
//!    estimate with the backup's, and its heading is in another frame (magnetic, not the learned track);
//!  * otherwise FREE-RUN: integrate the gyro, and while the accelerometer reads about 1 g (a trustworthy gravity
//!    vector) and the airframe is not turning, pull roll and pitch toward the gravity angles; pull the yaw toward
//!    the GNSS track when moving, and toward the magnetometer once its offset to the track has been learned.
//!    Roll and pitch stay gravity-referenced, so the glider holds wings-level and its pitch.
//!
//! BLIND: with no fresh gyro for [`BLIND_CYCLES`] ticks in a row the filter is blind, on every board and in either
//! regime; [`Attitude::blind`] feeds the health flag `attitude-blind`, which until it existed was the only thing that
//! could have said the filter had lost its gyro. What the filter then publishes depends on what is below it:
//!  * a FRESH source ranked below it (the SEN0253's BNO055): WITHHOLD, so the channel goes stale and the databoard
//!    hands the attitude to that backup, instead of masking it with an estimate frozen at its last value (roll and
//!    pitch hold: the accelerometer pull is gated off without a yaw rate);
//!  * nothing fresh below (every board without a SEN0253): publish as before the backup existed. Withholding there
//!    would hand the flight law no attitude at all -- neutral fins -- which no flight or HITL run has validated.
//!
//! Both end on the first fresh gyro sample.
//!
//! FRESH INPUTS ONLY. Plan B was migrated from `value()` to `read()` for every input: `value()` extrapolates without
//! bound, so the filter would blend an invented gyro rate or gravity vector and report an attitude with the same
//! confidence as a real one. A filter written to keep an attitude when another part had failed -- the case where
//! its own inputs are most likely stale too -- cannot tell fresh from invented. Here every input of [`Readings`] is
//! present only when a fresh source vouches for it (crates/databoard has no extrapolation at all).
//!
//! TWO FRAMES. The filter computes in plan B's frame, the one its formulas and the HITL sim speak: the aerospace
//! body frame, x forward, y RIGHT, z DOWN, rates by the right-hand rule (roll right wing down, pitch nose up, yaw
//! clockwise seen from above, all positive), and the accelerometer read as the GRAVITY vector (the specific force
//! it measures, negated). Only there do plan B's roll = atan2(ay, az), pitch = atan2(-ax, |(ay, az)|) and magnetic
//! heading = atan2(-my, mx) hold; plan B's hitl.py publishes `rate` as (roll, pitch, yaw) rate and the field "in
//! the consumer's convention" for that reason, which is all the HITL validation covers.
//!
//! The IMUs, though, are mounted per their MARKS, x forward, y LEFT, z UP (the operator, 2026-10-06), and the
//! drivers publish in that frame (a part mounted otherwise is mapped into it by its board config's `axes` key,
//! crates/drivers `axes`). The two frames are a half turn apart about x, so the gyro reads (x, -y, -z) in the
//! filter's frame, the field (x, -y, -z), and the accelerometer, the half turn negated into gravity, (-x, y, z).
//! [`Readings::from_marks`] is that map, the ONE place the mounting enters the filter, and [`rate_from_marks`] hands
//! the flight law's D term the same (roll, pitch, yaw) rate. Fed the marks frame unmapped, the filter would read
//! the pitch nose-DOWN positive with the gyro and gravity agreeing on the wrong sign (nothing would flag it), turn
//! the gyro yaw against the GNSS track pull, and run the magnetic heading backwards, so the "constant" offset it
//! learns against the track would be twice the heading. Plan B had no such map: its drivers publish the raw axes
//! (its notes say gx -> roll, gy -> pitch, gz -> yaw, and "field calibration flips a sign").
//!
//! Units: f32 degrees, seconds and g (decision D18). Plan B holds every angle as a centidegree fixnum, takes its
//! atan2 from an integer CORDIC and its square root from an integer routine, a MicroPython workaround for the
//! boxed float (doc/specs/avionics-port.md "What is deliberately not ported, or changed"). This port computes the
//! real-number filter those integers stand for. Integer centidegrees survive only at the telemetry and inspect
//! boundary ([`Attitude::row`], [`Attitude::inspect`]).
//!
//! The databoard, the log and attitude.csv are the caller's, as in crates/sequencer: [`Attitude::tick`] reads no
//! channel and no clock, and answers what to publish and what to log in a [`Tick`]. The caller pushes
//! `published`, logs `log`, and writes [`Attitude::row`] to attitude.csv on every tick that published, undecimated
//! (plan B records only what it publishes). What the caller reads and when are pure helpers here too:
//! [`assemble`] builds a tick's [`Readings`] from the databoard, [`Pace`] says which control ticks step the filter,
//! and [`on_backup`] answers the health flag `attitude-backup`.
//!
//! Deliberate differences from plan B, besides the mounting map above:
//!  * attitude.csv takes a row on every publish: `telemetry_ms` is read but thins nothing. Plan B decimated the
//!    stream to it (10 rows a second at its default), a MicroPython-era economy; here every measured sample is reported
//!    (doc/specs/avionics-port.md, "Measured -> reported");
//!  * plan B's integer error is gone: its pulls are right shifts, which floor, so a pull approaching its target from
//!    below stalls up to 2^shift - 1 centidegrees short for good (0.15 degree at corr_shift 4, 0.31 at course_shift
//!    and mag_offset_shift 5, 0.63 at mag_shift 6); its gyro delta floors up to a centidegree a tick and never
//!    recovers it; its CORDIC is good to ~0.05 degree; its from_float() truncates the accelerometer to the centi-g
//!    (up to ~0.6 degree) and its integer square root the pitch's denominator (up to ~0.3 degree). Here a pull of
//!    shift s moves exactly 2^-s of the error and the angles are f32 atan2;
//!  * the step integrates the elapsed MICROSECONDS. Plan B floors them to whole milliseconds, under-integrating every
//!    tick by its sub-millisecond part (up to 5 % of a 20 ms tick). The gap rule is plan B's, on whole milliseconds:
//!    more than 500 or negative takes the nominal period;
//!  * a reading that is not finite (NaN, an infinity) is treated as absent: one NaN in the state would poison the
//!    attitude for good, since no pull can bring a NaN back. Plan B's integers cannot be NaN, and its from_float()
//!    raises on one, which crashes its task;
//!  * a config key of the wrong type takes its default (a whole-number key needs a JSON integer); plan B stores
//!    what arrives and raises where it is used. Its truncation of the band edges and gates to the centi-unit
//!    (int(0.29 * 100) is 28) is gone too: the config number is used as written.
//!
//! Kept although it is a hazard: the accelerometer pull is NOT wrapped, as plan B's fixed.blend_cd is not. Near
//! inverted, an estimate of roll 170 and a gravity vector at roll -170 are 20 degrees apart the short way, but the
//! pull takes the error the long way, -340: at 1/4 per pull the published roll goes 85, 21, -27, -62, -89, -109,
//! sweeping through wings-level instead of moving 20 degrees. It is pinned by the golden fixture and a unit test, and
//! raised with the operator as a plan B defect; wrapping it would be a deliberate, documented deviation.

mod integration;

use serde_json::{Value, json};

pub use integration::{Pace, Quantities, assemble, on_backup};

/// Consecutive ticks with no fresh gyro before the filter is BLIND: 100 ms at the 20 ms period. One or two gyro-less
/// ticks follow every GC pause or stall, and holding the last attitude through those is right; five in a row means
/// the gyro is gone. Plan B's `_BLIND_CYCLES`.
pub const BLIND_CYCLES: u32 = 5;

/// The longest gap, in whole milliseconds, integrated as it is (plan B `_MAX_DT_MS`). A longer one (a bus stall)
/// integrates one nominal period instead: one huge gyro step would put an error into the yaw that nothing but the
/// track or the magnetometer could ever take out again.
const MAXIMUM_GAP_MS: i64 = 500;

/// The attitude.csv stream the caller records the filter into (plan B's `Telemetry('attitude.csv', ...)`).
///
/// RECORDED because this filter is the primary attitude, the only one on boards with no BNO055, and a passive
/// profile has no flight.csv: it once created no stream at all, so the flight meant to validate it as the sole
/// source returned none of its output.
pub const STREAM: &str = "attitude.csv";

/// The attitude.csv columns, the telemetry contract ([`Attitude::row`] fills them): centidegree integers and 0/1
/// flags.
pub const FIELDS: [&str; 6] = [
    "heading_cd",
    "roll_cd",
    "pitch_cd",
    "free",
    "mag_known",
    "mag_offset_cd",
];

/// The line logged when the gyro has been gone for [`BLIND_CYCLES`] ticks: once per transition, not per tick.
pub const GYRO_LOST: &str = "gyro lost -- attitude filter blind";

/// The line logged on the first fresh gyro sample after the filter went blind.
pub const GYRO_BACK: &str = "gyro back -- attitude filter sighted";

/// The probe's failure (plan B `probe`): a dead gyro leaves the filter blind, and the board would fly on the BNO055
/// backup if one is fitted and on a frozen roll and pitch if not, so it is surfaced before the flight.
pub const PROBE_FAILURE: &str = "attitude: no gyro rate -- attitude filter blind";

/// The filter's knobs: the attitude component's board-config keys (named in each field's note), with plan B's
/// `Attitude.setup()` defaults for a missing one ([`Settings::default`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// `period_ms`: the tick period in ms, 20 (50 Hz). The control thread paces the tick (every second tick at
    /// 100 Hz); the filter uses the period only as the nominal step for a gap it refuses to integrate.
    pub period_ms: u32,
    /// `accel_period_ms`: the shortest time between two accelerometer pulls, in ms, 50. Drift correction is slow by
    /// design, so it need not run on every tick.
    pub acceleration_period_ms: u32,
    /// `corr_shift`: each accelerometer pull moves roll and pitch 2^-shift of the way to the gravity angles, 4
    /// (1/16).
    pub correction_shift: u32,
    /// `turn_gate_deg_s`: the yaw rate in degrees per second above which the airframe is TURNING and the
    /// accelerometer pull is gated off, 4. In a coordinated turn the accelerometer reads gravity plus the
    /// centripetal acceleration DOWN THE BODY AXIS, so it looks level however hard the glider banks; correcting to
    /// that would roll the estimate flat. The gyro carries the true bank through the turn.
    pub turn_gate: f32,
    /// `course_gate_mps`: the ground speed in m/s above which the GNSS course is meaningful, 5.
    pub course_gate: f32,
    /// `course_shift`: each tick pulls the yaw 2^-shift of the way to the GNSS track, 5 (1/32): weak, so a crosswind
    /// crab averages out. The TRACK is what the navigation steers by anyway.
    pub course_shift: u32,
    /// `mag_shift`: once the offset is learned and the track is gone, each tick pulls the yaw 2^-shift of the way to
    /// the magnetic heading, 6 (1/64).
    pub magnetic_shift: u32,
    /// `mag_offset_shift`: how fast the magnetic-to-track offset is learned while the track is good, 5 (1/32).
    pub magnetic_offset_shift: u32,
    /// `mag_level_deg`: the LEVEL GATE in degrees, 15: past it in roll or pitch the magnetometer gives no heading.
    /// A flat atan2 of the horizontal axes is a heading only near level; at this latitude's field inclination a
    /// 20 degree bank puts more than 10 degrees into an uncompensated heading. Gating is honest where compensating
    /// without the tilt would be quietly wrong.
    pub magnetic_level: f32,
    /// `grav_low_g`: the bottom of the gravity band in g, 0.7. Outside 0.7..1.3 g (thrust, a manoeuvre) the
    /// accelerometer is not a gravity vector and pulls nothing.
    pub gravity_low: f32,
    /// `grav_high_g`: the top of the gravity band in g, 1.3.
    pub gravity_high: f32,
    /// `telemetry_ms`: plan B's attitude.csv decimation in ms, 100. Read as plan B reads it, but it decimates
    /// nothing: the caller writes a row on every publish (see the module notes).
    pub telemetry_ms: u32,
    /// `provides.attitude.priority`: the rank the filter publishes at, 1 in config_default. A fused winner ranked
    /// LOWER (a smaller number) outranks the filter and is mirrored. Read by the rule the databoard registration
    /// reads it with (crates/board_config `provides`), so it is the rank the filter's own channel holds, which is what
    /// plan B compares (`self._attitude.rank`): Python's int, a bool 0 or 1, saturating at i32; anything else, a float
    /// included, and a missing one, 0. A component with no `attitude` entry at all fails plan B's setup, and reads 0
    /// here.
    pub rank: i32,
}

impl Default for Settings {
    /// Plan B's `Attitude.setup()` defaults, every key missing.
    fn default() -> Settings {
        Settings {
            period_ms: 20,
            acceleration_period_ms: 50,
            correction_shift: 4,
            turn_gate: 4.0,
            course_gate: 5.0,
            course_shift: 5,
            magnetic_shift: 6,
            magnetic_offset_shift: 5,
            magnetic_level: 15.0,
            gravity_low: 0.7,
            gravity_high: 1.3,
            telemetry_ms: 100,
            rank: 0,
        }
    }
}

impl Settings {
    /// The settings an attitude component's config block holds; a key that is missing or of the wrong type takes
    /// its default, and so does every key of a block that is not an object.
    pub fn from_json(config: &Value) -> Settings {
        let defaults = Settings::default();
        let whole = |key: &str, default: u32| {
            config
                .get(key)
                .and_then(Value::as_u64)
                .and_then(|value| u32::try_from(value).ok())
                .unwrap_or(default)
        };
        let number = |key: &str, default: f32| {
            config
                .get(key)
                .and_then(Value::as_f64)
                .map_or(default, |value| value as f32)
        };
        let rank = match config.pointer("/provides/attitude/priority") {
            Some(Value::Bool(flag)) => i32::from(*flag),
            // an integer past i64 is past i32 too
            Some(Value::Number(number)) if !number.is_f64() => number.as_i64().map_or(i32::MAX, |priority| {
                priority.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
            }),
            _ => defaults.rank,
        };
        Settings {
            period_ms: whole("period_ms", defaults.period_ms),
            acceleration_period_ms: whole("accel_period_ms", defaults.acceleration_period_ms),
            correction_shift: whole("corr_shift", defaults.correction_shift),
            turn_gate: number("turn_gate_deg_s", defaults.turn_gate),
            course_gate: number("course_gate_mps", defaults.course_gate),
            course_shift: whole("course_shift", defaults.course_shift),
            magnetic_shift: whole("mag_shift", defaults.magnetic_shift),
            magnetic_offset_shift: whole("mag_offset_shift", defaults.magnetic_offset_shift),
            magnetic_level: number("mag_level_deg", defaults.magnetic_level),
            gravity_low: number("grav_low_g", defaults.gravity_low),
            gravity_high: number("grav_high_g", defaults.gravity_high),
            telemetry_ms: whole("telemetry_ms", defaults.telemetry_ms),
            rank,
        }
    }
}

/// An attitude, in degrees: the `attitude` channel's value. Plan B's channel carries (heading float degrees, roll
/// centidegrees, pitch centidegrees); here all three are f32 degrees.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Orientation {
    /// The heading, clockwise from the reference, in [0, 360) once published.
    pub heading: f32,
    /// The roll, right wing down positive, in [-180, 180) once published.
    pub roll: f32,
    /// The pitch, nose up positive, in [-180, 180) once published.
    pub pitch: f32,
}

impl Orientation {
    /// The attitude a sample of the `attitude` channel carries: (heading, roll, pitch), plan B's order.
    pub fn from_sample([heading, roll, pitch]: [f32; 3]) -> Orientation {
        Orientation { heading, roll, pitch }
    }

    /// This attitude as a sample of the `attitude` channel, for the push: (heading, roll, pitch), plan B's order.
    pub fn to_sample(self) -> [f32; 3] {
        [self.heading, self.roll, self.pitch]
    }
}

/// The fused `attitude` parameter's FRESH winner this tick, as the caller's databoard read answers it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fused {
    /// The winner's attitude.
    pub orientation: Orientation,
    /// The rank its source registered at (crates/databoard `Parameter::rank`): lower is preferred, and only a winner
    /// ranked below the filter's own number (the HITL sim at 0) is mirrored.
    pub rank: i32,
}

/// What one tick may read, each present only when a FRESH source vouches for it (see the module notes).
///
/// The vectors are in the FILTER's frame (x forward, y right, z down; see the module notes): a caller holding the
/// drivers' channels, which speak the marks frame, builds them with [`Readings::from_marks`]; only a source already
/// in the filter's frame (plan B's HITL sim, the golden fixtures) fills them directly.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Readings {
    /// The gyro (roll, pitch, yaw) rate in degrees per second: roll right wing down, pitch nose up, yaw clockwise.
    pub rate: Option<[f32; 3]>,
    /// The gravity vector (x, y, z) in g, in the filter's frame: the accelerometer's reading negated.
    pub acceleration: Option<[f32; 3]>,
    /// The GNSS ground-track course in degrees: an ABSOLUTE yaw reference that bounds the gyro drift.
    pub course: Option<f32>,
    /// The GNSS ground speed in m/s: the course is used only above the course gate.
    pub ground_speed: Option<f32>,
    /// The magnetometer (x, y, z) in the filter's frame, in any unit (only the ratio of x and y sets the heading),
    /// calibrated by its driver for hard and soft iron.
    pub magnetic: Option<[f32; 3]>,
    /// The fused attitude parameter's fresh winner, or None when nothing is fresh. When the filter itself is
    /// winning, as it is in flight, this is its own last push, at its own rank: never mirrored.
    pub fused: Option<Fused>,
    /// Whether a source ranked BELOW the filter is fresh (the BNO055 backup): a blind filter then withholds.
    pub backup_fresh: bool,
}

impl Readings {
    /// The gyro rate (degrees per second), the accelerometer (g) and the magnetometer as the drivers publish them,
    /// in the MARKS frame (x forward, y left, z up), mapped into the filter's frame; every other reading absent, for
    /// the caller to add (`Readings { course, ..Readings::from_marks(rate, acceleration, magnetic) }`). The one
    /// place the mounting enters the filter (see the module notes).
    pub fn from_marks(rate: Option<[f32; 3]>, acceleration: Option<[f32; 3]>, magnetic: Option<[f32; 3]>) -> Readings {
        Readings {
            rate: rate.map(rate_from_marks),
            acceleration: acceleration.map(gravity_from_marks),
            magnetic: magnetic.map(field_from_marks),
            ..Readings::default()
        }
    }

    /// These readings with every non-finite one taken as absent (see the module notes).
    fn finite(&self) -> Readings {
        let vector = |reading: Option<[f32; 3]>| reading.filter(|axes| axes.iter().all(|axis| axis.is_finite()));
        let scalar = |reading: Option<f32>| reading.filter(|value| value.is_finite());
        let fused = self.fused.filter(|fused| {
            let orientation = fused.orientation;
            [orientation.heading, orientation.roll, orientation.pitch]
                .iter()
                .all(|angle| angle.is_finite())
        });
        Readings {
            rate: vector(self.rate),
            acceleration: vector(self.acceleration),
            course: scalar(self.course),
            ground_speed: scalar(self.ground_speed),
            magnetic: vector(self.magnetic),
            fused,
            backup_fresh: self.backup_fresh,
        }
    }
}

/// One tick's answer: what to push and what to log. Plan B pushed and logged on the spot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tick {
    /// The attitude to push on the `attitude` channel, or None: never seeded and no accelerometer yet (nothing
    /// trustworthy to publish), or blind with a fresh backup below (withheld, so the backup takes over).
    pub published: Option<Orientation>,
    /// The line to log for a blind transition ([`GYRO_LOST`], [`GYRO_BACK`]), or None.
    pub log: Option<&'static str>,
}

/// The gyro rate in the marks frame (degrees per second about x forward, y left and z up, by the right-hand rule) as
/// the (roll, pitch, yaw) rate the filter and the flight law's D term take: roll right wing down, pitch nose up, yaw
/// clockwise. About y left the right-hand rule turns the nose DOWN and about z up it turns LEFT, hence two signs.
pub fn rate_from_marks([x, y, z]: [f32; 3]) -> [f32; 3] {
    [x, -y, -z]
}

/// The accelerometer in the marks frame (g; +1 on z at rest, level) as the filter's gravity vector: the half turn
/// about x into the filter's frame, (x, -y, -z), then negated from the specific force the part measures to gravity.
fn gravity_from_marks([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, z]
}

/// The magnetometer in the marks frame as the filter's: the half turn about x. North to the LEFT of the nose reads
/// +y in the marks frame and -y in the filter's, so atan2(-y, x) is then the heading, clockwise.
fn field_from_marks([x, y, z]: [f32; 3]) -> [f32; 3] {
    [x, -y, -z]
}

/// The pull a shift stands for: plan B moves `error >> shift`, the real-number filter 2^-shift of the error. Past
/// 2^-126 the pull leaves f32's normal range, so a larger shift is the same as 126: no pull for any purpose.
fn pull(shift: u32) -> f32 {
    0.5_f32.powi(shift.min(126) as i32)
}

/// A heading wrapped to [0, 360), as plan B's `% 36000` keeps one. f32's `rem_euclid` answers 360 itself for a tiny
/// negative angle (the sum rounds up), which is 0.
fn wrap_heading(degrees: f32) -> f32 {
    let wrapped = degrees.rem_euclid(360.0);
    if wrapped >= 360.0 { 0.0 } else { wrapped }
}

/// A signed angle wrapped to [-180, 180), as plan B's `((x + 18000) % 36000) - 18000` keeps one: the short way to
/// a target, for the pulls, and the published roll and pitch.
fn wrap_signed(degrees: f32) -> f32 {
    wrap_heading(degrees + 180.0) - 180.0
}

/// An angle in whole centidegrees, rounded: the integer unit of the telemetry and inspect boundary.
fn centidegrees(degrees: f32) -> i32 {
    (degrees * 100.0).round() as i32
}

/// A heading in whole centidegrees in [0, 36000) (plan B's heading_cd and mag_offset_cd columns).
fn heading_centidegrees(degrees: f32) -> i32 {
    centidegrees(degrees).rem_euclid(36_000)
}

/// A roll or pitch in whole centidegrees in [-18000, 18000) (plan B's roll_cd and pitch_cd columns). The wrap comes
/// AFTER the rounding: a wrapped 179.996 rounds to 18000, which plan B's integer state can never hold, so it is
/// -18000, the half turn as plan B's wrap writes it.
fn signed_centidegrees(degrees: f32) -> i32 {
    (centidegrees(degrees) + 18_000).rem_euclid(36_000) - 18_000
}

/// The operator-facing snapshot ([`Attitude::inspect`]) from a published [`Attitude::row`] and the two flags the row
/// does not carry: the firmware's executor makes it from what the control thread publishes as integers, so the
/// flight core formats no text.
pub fn inspection(
    [heading, roll, pitch, free, magnetic_known, magnetic_offset]: [i32; 6],
    seeded: bool,
    blind: bool,
) -> Value {
    json!({
        "free_running": free != 0,
        "seeded": seeded,
        "blind": blind,
        "roll": decimal_text(roll),
        "pitch": decimal_text(pitch),
        "heading": decimal_text(heading),
        "mag_known": magnetic_known != 0,
        "mag_offset": decimal_text(magnetic_offset),
    })
}

/// A centidegree value as plan B's `fixed.to_str` prints it: '12.34', '-0.05', the sign kept and no float rounding.
fn decimal_text(centidegrees: i32) -> String {
    let sign = if centidegrees < 0 { "-" } else { "" };
    let magnitude = centidegrees.unsigned_abs();
    format!("{sign}{}.{:02}", magnitude / 100, magnitude % 100)
}

/// The MAGNETIC yaw (plan B `_magnetic_heading` and `_magnetic_yaw`): learn the magnetic-to-track offset while the
/// GNSS track is good, and use it as an absolute heading when the track is not.
///
/// The magnetometer is the only absolute yaw reference that does not need a GNSS fix, and boost is exactly when a
/// consumer receiver loses lock: without it, the heading runs free through the phase that makes the most drift.
///
/// The order matters. While a track is available the magnetometer steers nothing: it is MEASURED against the
/// track, and the difference -- declination plus the mounting, as one lump -- is low-passed into the offset. The
/// moment the track goes (a boost dropout, a phase below the course gate), that offset turns the magnetometer into a
/// standalone reference and the yaw is pulled toward it, as it was toward the track. Two references pulling at once
/// would fight; one at a time does not.
///
/// It learns ONE CONSTANT, which is sound only because the driver removes what is not constant: hard and soft iron
/// shift the heading by an amount that depends on where the nose points (up to 214 degrees on the bench fixture), so
/// no single offset can absorb them. bmm350.calibrate() does, from a level circle the operator turns; until then the
/// device sits on the not-ready list. On an uncalibrated magnetometer this still runs and is still worth having --
/// a reference that is wrong but stationary bounds the pull better than a free-running gyro -- just much less.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagneticYaw {
    /// The learned magnetic-to-track offset in degrees, [0, 360): added to the magnetic heading it gives the track.
    offset: f32,
    /// Whether the offset has been learned. Until then the magnetometer steers nothing: an unlearned offset is a
    /// zero that would assert magnetic north IS the ground track, and a free-running gyro beats a confidently wrong
    /// pull. Every flight starts here. It goes up on the FIRST learning step, however little was learned yet.
    known: bool,
    /// The pull toward the magnetic heading, 2^-mag_shift.
    pull: f32,
    /// The learning rate of the offset, 2^-mag_offset_shift.
    offset_pull: f32,
    /// The level gate in degrees: past it in roll or pitch there is no heading.
    level: f32,
}

impl MagneticYaw {
    /// A magnetic yaw with nothing learned: the pull shift, the offset's learning shift and the level gate in
    /// degrees (the settings' `magnetic_shift`, `magnetic_offset_shift` and `magnetic_level`).
    pub fn new(shift: u32, offset_shift: u32, level: f32) -> MagneticYaw {
        MagneticYaw {
            offset: 0.0,
            known: false,
            pull: pull(shift),
            offset_pull: pull(offset_shift),
            level,
        }
    }

    /// Whether the offset has been learned (attitude.csv `mag_known`, inspect `mag_known`).
    pub fn known(&self) -> bool {
        self.known
    }

    /// The learned offset in degrees, [0, 360) (attitude.csv `mag_offset_cd`, inspect `mag_offset`).
    pub fn offset(&self) -> f32 {
        self.offset
    }

    /// The heading from the magnetometer in degrees, [0, 360), or None when it cannot be trusted this tick: no
    /// fresh field, or the airframe past the level gate in `roll` or `pitch` (degrees).
    ///
    /// A FLAT atan2 of the horizontal axes, which is why the level gate exists (see [`Settings::magnetic_level`]).
    /// The result is in the SENSOR's magnetic frame: it carries the declination and the mounting angle, which the
    /// learned offset absorbs, so no absolute reference is needed here.
    pub fn heading(&self, magnetic: Option<[f32; 3]>, roll: f32, pitch: f32) -> Option<f32> {
        let [x, y, _] = magnetic?;
        if wrap_signed(roll).abs() > self.level || wrap_signed(pitch).abs() > self.level {
            return None;
        }
        Some(wrap_heading((-y).atan2(x).to_degrees()))
    }

    /// One tick of the magnetic yaw: `tracking` is the GNSS course in degrees when the track is usable this tick
    /// (it takes precedence), `yaw` the filter's yaw in degrees. Returns the yaw, pulled toward the magnetic
    /// reference only when the track is gone and the offset is known; while tracking it only learns the offset.
    pub fn step(&mut self, magnetic: Option<[f32; 3]>, roll: f32, pitch: f32, tracking: Option<f32>, yaw: f32) -> f32 {
        let Some(heading) = self.heading(magnetic, roll, pitch) else {
            return yaw;
        };
        if let Some(course) = tracking {
            let target = wrap_signed(course - heading);
            let error = wrap_signed(target - self.offset);
            self.offset = wrap_heading(self.offset + error * self.offset_pull);
            self.known = true;
            return yaw;
        }
        if !self.known {
            return yaw;
        }
        let error = wrap_signed(heading + self.offset - yaw);
        wrap_heading(yaw + error * self.pull)
    }
}

/// The complementary-filter attitude (plan B `Attitude`): the primary, published at priority 1.
#[derive(Clone, Debug, PartialEq)]
pub struct Attitude {
    /// The knobs.
    settings: Settings,
    /// The pull of one accelerometer correction, 2^-correction_shift.
    correction_pull: f32,
    /// The pull toward the GNSS track per tick, 2^-course_shift.
    course_pull: f32,
    /// The roll in degrees: right wing down positive. Wrapped to [-180, 180) at the end of a tick that publishes
    /// (and a tick that withholds is blind, so it moves neither roll nor pitch); within a tick the gyro step and the
    /// accelerometer pull act on it unwrapped, as plan B's do.
    roll: f32,
    /// The pitch in degrees, nose up positive; wrapped like the roll.
    pitch: f32,
    /// The yaw (the heading) in degrees, [0, 360) after every change.
    yaw: f32,
    /// The magnetic reference and its learned offset.
    magnetic: MagneticYaw,
    /// Whether a source that outranks the filter has ever been mirrored. Until then the filter runs only on ticks
    /// with a fresh accelerometer: free-running from a zero nothing vouched for publishes nothing.
    seeded: bool,
    /// Whether the last tick that ran free-ran rather than mirrored (attitude.csv `free`). A tick that ran neither
    /// way leaves it as it was.
    free: bool,
    /// No fresh gyro for [`BLIND_CYCLES`] ticks (see the module notes).
    blind: bool,
    /// The run of consecutive gyro-less ticks (plan B's `Task.strike` counter).
    strikes: u32,
    /// The time of the last tick, in microseconds (a fresh filter: its setup).
    last_us: u64,
    /// The time of the last accelerometer pull check, in microseconds: the throttle's clock.
    acceleration_us: u64,
}

impl Attitude {
    /// A filter with these knobs, set up at `now_us` (the first tick integrates from it, and the first accelerometer
    /// pull comes an `acceleration_period_ms` after it): level, heading north, nothing learned, not seeded.
    pub fn new(settings: Settings, now_us: u64) -> Attitude {
        Attitude {
            settings,
            correction_pull: pull(settings.correction_shift),
            course_pull: pull(settings.course_shift),
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            magnetic: MagneticYaw::new(
                settings.magnetic_shift,
                settings.magnetic_offset_shift,
                settings.magnetic_level,
            ),
            seeded: false,
            free: false,
            blind: false,
            strikes: 0,
            last_us: now_us,
            acceleration_us: now_us,
        }
    }

    /// The knobs.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// The filter's attitude as it stands after the last tick, whether that tick published it or withheld it.
    pub fn orientation(&self) -> Orientation {
        Orientation {
            heading: self.yaw,
            roll: self.roll,
            pitch: self.pitch,
        }
    }

    /// Whether the filter is BLIND: no fresh gyro for [`BLIND_CYCLES`] ticks. The health flag `attitude-blind`.
    pub fn blind(&self) -> bool {
        self.blind
    }

    /// Whether the filter is free-running rather than mirroring a source that outranks it.
    pub fn free(&self) -> bool {
        self.free
    }

    /// Whether a source that outranks the filter has ever been mirrored.
    pub fn seeded(&self) -> bool {
        self.seeded
    }

    /// The magnetic reference: whether its offset is learned, and the offset.
    pub fn magnetic(&self) -> &MagneticYaw {
        &self.magnetic
    }

    /// One tick at `now_us` (plan B `_tick`): watch the gyro, then mirror a source that outranks the filter or
    /// free-run, and answer what to publish and log.
    ///
    /// Publishes every tick except before anything trustworthy exists (never seeded and no fresh accelerometer: the
    /// tick then integrates nothing either, and its gyro is lost, not deferred) and while blind with a fresh source
    /// below the filter to take over. A published roll and pitch are wrapped to [-180, 180), the heading to
    /// [0, 360).
    pub fn tick(&mut self, now_us: u64, readings: &Readings) -> Tick {
        let readings = readings.finite();
        let elapsed = self.step_seconds(now_us);
        let log = self.watch_gyro(readings.rate.is_some());
        match readings.fused {
            Some(fused) if fused.rank < self.settings.rank => {
                self.mirror(fused.orientation);
                self.free = false;
            }
            _ if self.seeded || readings.acceleration.is_some() => {
                self.integrate(elapsed, now_us, &readings);
                self.free = true;
                if self.blind && readings.backup_fresh {
                    return Tick { published: None, log }; // withhold: a frozen attitude would mask the live backup
                }
            }
            _ => return Tick { published: None, log },
        }
        self.roll = wrap_signed(self.roll);
        self.pitch = wrap_signed(self.pitch);
        self.yaw = wrap_heading(self.yaw);
        Tick {
            published: Some(self.orientation()),
            log,
        }
    }

    /// The seconds to integrate this tick, and the tick's time kept for the next one.
    ///
    /// A gap over [`MAXIMUM_GAP_MS`] whole milliseconds, or a clock that went backwards, integrates the nominal
    /// period instead (plan B's rule); anything else integrates exactly, to the microsecond.
    fn step_seconds(&mut self, now_us: u64) -> f32 {
        let elapsed_us = now_us.wrapping_sub(self.last_us) as i64;
        self.last_us = now_us;
        if !(0..=MAXIMUM_GAP_MS).contains(&elapsed_us.div_euclid(1000)) {
            return self.settings.period_ms as f32 / 1000.0;
        }
        elapsed_us as f32 / 1_000_000.0
    }

    /// Count the gyro-less ticks and move BLIND (plan B `_watch_gyro`); answer the transition's log line.
    ///
    /// Counted every tick and in both regimes, so `blind` (and the health flag built on it) tells the truth while
    /// mirroring too: the filter that would take over then has nothing to integrate. Goes up exactly once, on the
    /// tick the run reaches [`BLIND_CYCLES`], and down on the first fresh sample.
    fn watch_gyro(&mut self, sighted: bool) -> Option<&'static str> {
        if sighted {
            self.strikes = 0;
            if self.blind {
                self.blind = false;
                return Some(GYRO_BACK);
            }
            return None;
        }
        self.strikes = self.strikes.saturating_add(1);
        if self.strikes == BLIND_CYCLES {
            self.blind = true;
            return Some(GYRO_LOST);
        }
        None
    }

    /// Copy a source that outranks the filter (plan B `_mirror`), so the filter stays warm and the handover is
    /// seamless the moment that source stops. In practice the HITL sim's attitude.
    fn mirror(&mut self, orientation: Orientation) {
        self.roll = orientation.roll;
        self.pitch = orientation.pitch;
        self.yaw = orientation.heading;
        self.seeded = true;
    }

    /// Free-run one tick of `elapsed` seconds (plan B `_integrate`): gyro-integrate roll, pitch and yaw, pull the
    /// yaw toward the track and the magnetometer, then re-anchor roll and pitch to the gravity vector.
    ///
    /// The accelerometer pull fires ONLY when the gravity vector is trustworthy: inside the gravity band, NOT
    /// turning (the yaw rate at or under the turn gate; with no gyro the gate is closed, so roll and pitch hold),
    /// and at least `acceleration_period_ms` since the last check. A turn neither checks nor restarts that clock.
    ///
    /// Three orders are plan B's and each is pinned by the golden fixture: the track needs the course AND the ground
    /// speed fresh (either alone is no track: plan B drops both); the magnetic level gate reads the roll and pitch as
    /// they were BEFORE this tick's gyro step (plan B's `_magnetic_heading` runs before `blend_cd`); and the
    /// accelerometer pull takes its error unwrapped, the long way round across +/-180 (see the module notes).
    fn integrate(&mut self, elapsed: f32, now_us: u64, readings: &Readings) {
        let mut deltas = [0.0_f32; 3];
        let mut turning = true;
        if let Some(rate) = readings.rate {
            deltas = rate.map(|axis| axis * elapsed);
            turning = rate[2].abs() > self.settings.turn_gate;
        }
        self.yaw = wrap_heading(self.yaw + deltas[2]);
        let tracking = match (readings.course, readings.ground_speed) {
            (Some(course), Some(speed)) if speed > self.settings.course_gate => Some(course),
            _ => None,
        };
        if let Some(course) = tracking {
            self.yaw = wrap_heading(self.yaw + wrap_signed(course - self.yaw) * self.course_pull);
        }
        self.yaw = self
            .magnetic
            .step(readings.magnetic, self.roll, self.pitch, tracking, self.yaw);
        let mut target = None;
        let since_us = now_us.wrapping_sub(self.acceleration_us) as i64;
        if !turning && since_us >= i64::from(self.settings.acceleration_period_ms) * 1000 {
            self.acceleration_us = now_us;
            target = readings
                .acceleration
                .and_then(|acceleration| self.gravity(acceleration));
        }
        self.roll += deltas[0];
        self.pitch += deltas[1];
        if let Some((roll, pitch)) = target {
            // unwrapped, as plan B's blend_cd: a plan B hazard kept on purpose (see the module notes)
            self.roll += (roll - self.roll) * self.correction_pull;
            self.pitch += (pitch - self.pitch) * self.correction_pull;
        }
    }

    /// The (roll, pitch) the gravity vector reads, in degrees, or None outside the gravity band.
    ///
    /// The band is compared squared, so the common case takes no square root: roll = atan2(ay, az), pitch =
    /// atan2(-ax, |(ay, az)|) (see the module notes for the frame).
    fn gravity(&self, [x, y, z]: [f32; 3]) -> Option<(f32, f32)> {
        let planar = y * y + z * z;
        let magnitude = x * x + planar;
        let (low, high) = (self.settings.gravity_low, self.settings.gravity_high);
        if !(low * low <= magnitude && magnitude <= high * high) {
            return None;
        }
        Some((y.atan2(z).to_degrees(), (-x).atan2(planar.sqrt()).to_degrees()))
    }

    /// The attitude.csv row ([`FIELDS`]): heading, roll and pitch in whole centidegrees, the `free` and `mag_known`
    /// flags as 0/1, and the magnetic offset in centidegrees. Integers only, as plan B records them.
    pub fn row(&self) -> [i32; 6] {
        [
            heading_centidegrees(self.yaw),
            signed_centidegrees(self.roll),
            signed_centidegrees(self.pitch),
            i32::from(self.free),
            i32::from(self.magnetic.known),
            heading_centidegrees(self.magnetic.offset),
        ]
    }

    /// The operator-facing snapshot (`inspect attitude`; plan B `inspect`, its attitude keys -- the firmware's
    /// Inspectable adds the name and health): the flags, the angles as text to the hundredth, and the magnetic
    /// reference, which is otherwise invisible -- whether the offset has been LEARNED (the magnetometer steers
    /// nothing until it has) and what it converged to.
    pub fn inspect(&self) -> Value {
        inspection(self.row(), self.seeded, self.blind)
    }

    /// The diagnostics (`stats attitude`): the same snapshot as [`Attitude::inspect`], as in plan B.
    pub fn stats(&self) -> Value {
        self.inspect()
    }

    /// The on-demand self-test (plan B `probe`): the gyro `rate` is present, `gyro_fresh`, the filter's core input.
    /// Err carries the operator text, [`PROBE_FAILURE`].
    pub fn probe(gyro_fresh: bool) -> Result<(), &'static str> {
        if gyro_fresh { Ok(()) } else { Err(PROBE_FAILURE) }
    }
}

#[cfg(test)]
mod tests;
