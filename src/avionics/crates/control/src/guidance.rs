//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage-dependent guidance law (src/glider guidance.py), sibling of pid, mixer and navigation. It turns
//! (stage, heading) into the attitude setpoints and the heading error the PIDs chase: the boost rod-vertical
//! hold, bank-to-turn toward the landing zone, the GPS-degrading heading tiers (fix, dead reckoning, launch
//! point, blind), the loiter orbit and the endgame holding patterns, and the low final-approach centreline
//! tracker. The per-stage law is a `match` on the stage, plan B's dispatch table: a new stage is one arm and one
//! method.
//!
//! Host-testable by construction: nothing here reads a sensor, a channel or a clock. Plan B injects the Mission,
//! the governor, the wind estimator and databoard handles, each read with `read()` -> (value, source, age); here
//! the caller hands their answers in as [`Inputs`], and the time as `now_us`. NOT an extrapolated `value()`: the
//! laser agl is out of range for most of a flight, and an extrapolation answers a stale channel with confident
//! fiction -- the failure that once ended a flight at apogee. So an input is present only when a FRESH source
//! vouches for it (`crates/databoard` has no extrapolating accessor at all).
//!
//! The results land in slots the caller reads ([`Guidance::roll_setpoint`], [`Guidance::pitch_setpoint`],
//! [`Guidance::heading_error`]) rather than a returned tuple, as in plan B. That is behaviour, not only a
//! MicroPython allocation trick: when the boost law holds the fins neutral the slots keep their last values, and
//! the flight row records them.
//!
//! Units: the setpoints are f32 degrees. Plan B holds them as centidegree fixnums (fixed.py), truncating toward
//! zero, a MicroPython workaround (doc/specs/avionics-port.md "What is deliberately not ported, or changed");
//! here they keep their fraction, at most 0.01 degree from plan B's. The heading error stays INTEGER degrees, as
//! plan B's: sub-degree precision is
//! irrelevant to a servo, it is an integer column of the telemetry contract (flight.csv `heading_error`, the
//! health vitals), and its smoothing filter's rounding rule is part of the law (see [`SteeringFilter`]).
//!
//! Deliberate differences from plan B (none changes a flight with a valid config):
//!  * `compute` takes no setpoint argument: plan B passes the stage's setpoint dict through to every law, and no
//!    law reads it.
//!  * a law that needs the stage's configured setpoints and finds none answers with zeros; plan B raises
//!    (KeyError). The flight task only computes a configured control stage, so neither happens in flight.
//!  * the five knobs whose 0 switches a feature off (`nav_bank_gain`, `land_bank_gain`, `endgame_alt_m`,
//!    `final_approach_agl`, `steer_filter_shift`) are read by their TRUTH, as plan B's gates read them: a
//!    present null, false or empty value switches the feature OFF, true flies as 1, and only an absent key
//!    takes the default (see [`GuidanceConfig::from_json`]). That is plan B's behaviour, not a difference. The
//!    difference is in what plan B cannot fly: any other knob of the wrong type, and a non-empty text in one of
//!    those five, takes its default here, where plan B stores it and raises wherever it is used (or flies a
//!    bool as its 0 or 1); a whole float shift (3.0) is that shift here, where plan B's `>>` raises on a float.
//!    Plan B's config validate() refuses a non-number for eight of the knobs before a flight.
//!  * before the first [`Guidance::enter`] the captured heading is north (0); plan B's is None and raises where
//!    it is used. The flight task enters before it computes.

use crate::commons::between;
use crate::navigation::{self, Position};
use sequencer::Stage;
use serde_json::{Value, json};

/// Standard gravity in m/s^2: the coordinated-turn radius is R = v^2 / (g * tan(bank)).
const GRAVITY: f32 = 9.81;

/// The dead-reckoning integration step ceiling in microseconds (0.5 s). A stage hop, a long gap or a first call
/// after one would otherwise integrate one huge step from a heading sampled at its far end; skipping the advance
/// loses a little travel and never invents any.
const RECKON_MAXIMUM_STEP_US: i64 = 500_000;

/// The loiter and endgame laws' cap on the inward/outward cut, in degrees: never steer more than 60 degrees off
/// the circle's tangent, however far from the circle the glider is.
const CUT_LIMIT_DEGREES: f32 = 60.0;

/// The 'ov' endgame never shrinks its legs below this fraction of the half-length: a small end-to-end
/// oscillation persists down to touchdown rather than a dead stop on the centreline.
const OVAL_REACH_FLOOR: f32 = 0.15;

/// Microseconds per second, for the dead-reckoning step.
const MICROSECONDS_PER_SECOND: f32 = 1_000_000.0;

/// The endgame HOLDING pattern (guidance.Heading), named as the config names it.
///
/// The discriminants are plan B's ids. [`Heading::resolve`] turns a config name into one. `Auto` defers to the
/// Mission (the mission crate's `Mission::endgame_heading`, plan B's mission.endgame_heading), which picks the
/// pattern from the zone's long/short aspect against [`Heading::OVAL_ASPECT`] and
/// [`Heading::TWO_LOBES_ASPECT`]: the Mission owns the zone, so it owns that decision, and hands its answer in
/// with the [`Landing`].
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Heading {
    /// 'auto': the Mission decides from the zone shape (plan B `AUTO`).
    Auto = 0,
    /// 'o': a single circle around the centre (plan B `FIG_O`), the reliable in-zone default.
    Circle = 1,
    /// 'oo' (or 'o-o'): two lobes along the long axis, for a very elongated strip (plan B `FIG_OO`).
    TwoLobes = 2,
    /// 'ov': a converging centreline oval, a racetrack, for a moderately elongated strip (plan B `FIG_OVAL`).
    Oval = 3,
}

impl Heading {
    /// A zone whose long/short aspect reaches this flies the oval rather than a single circle (plan B
    /// `OVAL_ASPECT`).
    pub const OVAL_ASPECT: f32 = 2.0;

    /// ...and from this aspect on the two lobes (plan B `OO_ASPECT`): the converging oval wins up to here.
    pub const TWO_LOBES_ASPECT: f32 = 6.0;

    /// Every pattern, in id order.
    pub const ALL: [Heading; 4] = [Heading::Auto, Heading::Circle, Heading::TwoLobes, Heading::Oval];

    /// The config name (plan B `Heading.PATTERNS`).
    pub fn name(self) -> &'static str {
        match self {
            Heading::Auto => "auto",
            Heading::Circle => "o",
            Heading::TwoLobes => "oo",
            Heading::Oval => "ov",
        }
    }

    /// The pattern a config name names (plan B `Heading.resolve`): case-insensitive, and every '-' dropped, so
    /// 'o-o' aliases 'oo'. An unknown name falls back to `Auto`, which lets the Mission decide.
    pub fn resolve(name: &str) -> Heading {
        let plain = name.to_lowercase().replace('-', "");
        Heading::ALL
            .into_iter()
            .find(|pattern| pattern.name() == plain)
            .unwrap_or(Heading::Auto)
    }
}

/// A control stage's configured attitude setpoint in degrees (a `stages` entry of the flight config).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Setpoint {
    /// The roll to hold, degrees, right wing down positive; 0 when the entry names none.
    pub roll: f32,
    /// The pitch to hold, degrees, nose up positive; 0 when the entry names none.
    pub pitch: f32,
}

/// The guidance knobs, resolved from the flight component's config ONCE (guidance.GuidanceConfig).
///
/// One place for the defaults and their reasons. A field keeps its board.config key's meaning; where the field
/// name differs from the key, the doc names the key. Every default equals what config_default ships (plan B's
/// test_fallbacks_match_shipped): a partial config must fly the law the panel reports.
#[derive(Clone, Debug, PartialEq)]
pub struct GuidanceConfig {
    /// The CONTROL stages and their setpoints (`stages`, default `{'gliding': {}}`), indexed by stage id - 1. A
    /// stage that is not listed holds the fins neutral: by default no actuation under thrust or on the ground.
    pub stages: [Option<Setpoint>; 5],
    /// Bank-to-turn gain, roll degrees per degree of heading error (`nav_bank_gain`, 1.5): in GLIDING the roll
    /// setpoint comes from the heading error, so the glider banks into the turn (tight, ~v^2/(g tan(bank)))
    /// instead of skidding flat on the rudder, which over-ranges a small zone. 0 steers on the rudder alone.
    pub bank_gain: f32,
    /// The cruise bank cap, degrees (`bank_limit`, 45). Not the 30 it once was: at 30 degrees the turn floor
    /// R_min is ~40 m at 15.6 m/s while the loiter law commands 30 m, so the heading controller saturates into a
    /// limit cycle (roll setpoint pinned at +/-30, heading error 112-157 degrees). A partial config that omitted
    /// the key flew that while `inspect` showed the shipped 45.
    pub bank_limit: f32,
    /// The endgame / final / landing bank gain (`land_bank_gain`, 3.0): the FULL fin authority, to spiral tight
    /// and crab the crosswind out. At the old 1.5 the endgame P-loop saturates near 25 degrees and the spiral
    /// freezes at that bank's ~44 m radius, against the ~20 m R_min an in-zone landing needs. 0 disables it.
    pub land_bank_gain: f32,
    /// The bank cap on final and in LANDING, degrees (`land_bank_limit`, 45): a straight crabbed approach.
    pub land_bank_limit: f32,
    /// The ENDGAME band, metres of elevation above the pad (`endgame_alt_m`, 50; doc/specs/coludo.md "Gliding",
    /// the fly-long objectives). Below it the full land-bank authority opens, the turn radius halves and the last
    /// seconds spiral tightly around the zone instead of racetracking past it. Up high the gentler `bank_limit`
    /// keeps the fly-long objective, since a tight bank costs sink (~load^1.5). 0 disables the band.
    pub endgame_altitude_m: f32,
    /// The 1-g stall speed, m/s (`stall_speed_1g`, 9.0). The ENDGAME banks as steep as the LIVE airspeed safely
    /// allows (doc/specs/coludo.md "Turn-radius limit"): a coordinated turn at bank phi pulls load 1/cos(phi),
    /// raising the stall speed by sqrt(load). 0 disables the gate (back to `land_bank_limit`).
    pub stall_speed_1g: f32,
    /// The airspeed cushion above that stall speed (`stall_margin`, 1.2).
    pub stall_margin: f32,
    /// The structural bank ceiling in the endgame, degrees (`endgame_max_bank`, 60).
    pub endgame_max_bank: f32,
    /// The LOITER orbit radius, metres (`loiter_radius_m`, 30): the "orbit the target to bleed altitude" the docs
    /// always intended. Within `loiter_capture_m` of the centre the heading command becomes the circle tangent
    /// plus an inward correction, so the glider CAPTURES a constant-radius orbit instead of bang-banging between
    /// overfly and U-turn (measured: the point-steer law swung 184 m racetrack legs and landed on phase luck). R
    /// = 40 m at 14 m/s needs only ~26 degrees of bank; R below the cruise-bank minimum (~34 m at 30 degrees)
    /// destabilizes the pre-endgame orbit: do not shrink it.
    pub loiter_radius_m: f32,
    /// The distance from the centre, metres, within which the orbit takes over from gate steering
    /// (`loiter_capture_m`, 120).
    pub loiter_capture_m: f32,
    /// Degrees of inward cut per metre off the circle (`loiter_gain`, 3.0).
    pub loiter_gain: f32,
    /// The ENDGAME holding pattern (`endgame_pattern`, 'auto': the Mission decides from the zone shape).
    pub endgame_pattern: Heading,
    /// Below this fresh AGL, metres, the target switches to the strip centreline (`final_approach_agl`, 8); 0
    /// disables it. NOTE what the source gate implies: a VL53L1X measured on the bench reaches 2.07 m
    /// (doc/hardware.md), the L4CX is longer and still short of 8 m, and there is no baro fallback here (unlike
    /// the landing trigger), so final approach engages when the laser acquires, not at 8 m; at the 45 degree
    /// bank the slant range is AGL/cos(45), so a 2.07 m sensor covers ~1.46 m of real AGL. Plan B left it so on
    /// purpose: lowering the threshold, adding the elevation fallback or accepting a late start is a flight
    /// decision, not a code one.
    pub final_approach_agl: f32,
    /// Final approach: intercept degrees per metre off the centreline (`final_cross_gain`, 3.0).
    pub final_cross_gain: f32,
    /// Final approach: the cap on the intercept angle, degrees (`final_intercept_deg`, 45).
    pub final_intercept: f32,
    /// BOOSTING engages only past the rod, above this airspeed in m/s (`boost_engage_speed`, 15.0): below it the
    /// fins have no dynamic pressure to bite and the heading is ill-defined near vertical.
    pub boost_engage_speed: f32,
    /// The steering noise filter's EMA divisor power (`steer_filter_shift`, 3: alpha 1/8, tau ~80 ms at 100 Hz).
    /// It kills the bank flapping that wobbled the endgame spiral at >= 25 % sensor noise; 0 disables it.
    pub steer_filter_shift: u32,
    /// The target heading is recomputed at most this often, microseconds (`nav_period_ms`, 100 ms): the GNSS
    /// fixes at ~10 Hz, so recomputing every 100 Hz tick buys nothing. Signed, as plan B's tick difference is.
    pub navigation_period_us: i64,
    /// The tier-1 freshness gate, milliseconds (`position_age_max_ms`, default the GNSS channels' own databoard
    /// window, so it tracks the GNSS rate rather than a magic number). Config may set it TIGHTER to distrust the
    /// GNSS sooner; looser is a no-op, since past the window no source vouches for the fix at all.
    pub position_age_max_ms: u32,
}

impl GuidanceConfig {
    /// The guidance knobs from the flight component's config (`flight` in board.config) and the GNSS channels'
    /// databoard window in milliseconds, the default tier-1 freshness gate.
    ///
    /// `stages` keeps only the names that are stages, each entry's `roll` and `pitch` defaulting to 0. A
    /// fractional `nav_period_ms` or `position_age_max_ms` is rounded up to the whole microsecond or
    /// millisecond, which keeps plan B's `elapsed < period` exactly for the integer clock it is compared with.
    ///
    /// The four knobs whose 0 switches a feature off (`nav_bank_gain`, `land_bank_gain`, `endgame_alt_m`,
    /// `final_approach_agl`) and `steer_filter_shift` are read by their truth, as plan B's gates read them: a
    /// present null, false or empty value is OFF, not missing, and true is 1; only an absent key takes the
    /// default. A whole non-negative float is a shift. Every other knob takes its default for a value that is not
    /// a number.
    pub fn from_json(config: &Value, position_window_ms: u32) -> GuidanceConfig {
        let number = |key: &str, default: f32| {
            config
                .get(key)
                .and_then(Value::as_f64)
                .map_or(default, |value| value as f32)
        };
        GuidanceConfig {
            stages: stages(config.get("stages")),
            bank_gain: switch(config.get("nav_bank_gain"), 1.5),
            bank_limit: number("bank_limit", 45.0),
            land_bank_gain: switch(config.get("land_bank_gain"), 3.0),
            land_bank_limit: number("land_bank_limit", 45.0),
            endgame_altitude_m: switch(config.get("endgame_alt_m"), 50.0),
            stall_speed_1g: number("stall_speed_1g", 9.0),
            stall_margin: number("stall_margin", 1.2),
            endgame_max_bank: number("endgame_max_bank", 60.0),
            loiter_radius_m: number("loiter_radius_m", 30.0),
            loiter_capture_m: number("loiter_capture_m", 120.0),
            loiter_gain: number("loiter_gain", 3.0),
            endgame_pattern: config
                .get("endgame_pattern")
                .and_then(Value::as_str)
                .map_or(Heading::Auto, Heading::resolve),
            final_approach_agl: switch(config.get("final_approach_agl"), 8.0),
            final_cross_gain: number("final_cross_gain", 3.0),
            final_intercept: number("final_intercept_deg", 45.0),
            boost_engage_speed: number("boost_engage_speed", 15.0),
            steer_filter_shift: filter_shift(config.get("steer_filter_shift")),
            navigation_period_us: config
                .get("nav_period_ms")
                .and_then(Value::as_f64)
                .map_or(100_000, |milliseconds| (milliseconds * 1000.0).ceil() as i64),
            position_age_max_ms: config
                .get("position_age_max_ms")
                .and_then(Value::as_f64)
                .map_or(position_window_ms, |milliseconds| milliseconds.ceil() as u32),
        }
    }

    /// The stage's configured setpoint, None when it is not a control stage ([`Guidance::setpoint`] answers it).
    fn stage_setpoint(&self, stage: Stage) -> Option<Setpoint> {
        self.stages[stage as usize - 1]
    }
}

/// Whether Python reads this JSON value as false: null, false, a zero, and the empty text, list and object.
///
/// Plan B's guidance gates its optional features on the knob's truth (`if config.final_agl and ...`,
/// `if not shift`), so this is the rule that decides whether a stored value switches a feature off.
fn falsy(value: &Value) -> bool {
    match value {
        Value::Null => true,
        Value::Bool(on) => !on,
        Value::Number(number) => number.as_f64() == Some(0.0),
        Value::String(text) => text.is_empty(),
        Value::Array(items) => items.is_empty(),
        Value::Object(members) => members.is_empty(),
    }
}

/// A knob whose 0 switches its feature off (`nav_bank_gain`, `land_bank_gain`, `endgame_alt_m`,
/// `final_approach_agl`), read the way plan B's gate reads it.
///
/// Plan B stores whatever the key holds (`config.get(key, default)`) and tests its TRUTH before using it, so a
/// key that is present but null, false or empty switches the feature OFF; it is NOT the missing key that takes
/// the default and switches it on. Treating a null `final_approach_agl` as absent would fly the centreline
/// tracker below 8 m where plan B homes. True flies as 1 (a Python bool is the integer 1 in the comparison and
/// the product). A number is itself. Any other value (a non-empty text, list or object) takes the default here,
/// where plan B raises at its first comparison or product with it.
fn switch(value: Option<&Value>, default: f32) -> f32 {
    match value {
        None => default,
        Some(value) if falsy(value) => 0.0,
        Some(Value::Bool(_)) => 1.0, // true: false is falsy, above
        Some(value) => value.as_f64().map_or(default, |number| number as f32),
    }
}

/// `steer_filter_shift` as plan B's filter reads it (`if not shift: return error`, then `>> shift`).
///
/// Absent, the default 3. Any falsy value (null, false, 0, 0.0, empty) switches the filter OFF, as plan B's
/// `not shift` does; true is a shift of 1, as `>> True` is. A whole non-negative number is that shift, a float
/// such as 3.0 included, where plan B's `>>` raises on a float (0.0 alone works there, being falsy). Anything
/// else -- a fraction, a negative number (plan B's `>>` raises on both), a shift past u32 or a non-empty text --
/// takes the default.
fn filter_shift(value: Option<&Value>) -> u32 {
    /// The shipped shift: alpha 1/8, tau ~80 ms at 100 Hz.
    const DEFAULT: u32 = 3;
    match value {
        None => DEFAULT,
        Some(value) if falsy(value) => 0,
        Some(Value::Bool(_)) => 1, // true: false is falsy, above
        Some(value) => value
            .as_f64()
            .filter(|shift| *shift >= 0.0 && shift.fract() == 0.0 && *shift <= f64::from(u32::MAX))
            .map_or(DEFAULT, |shift| shift as u32),
    }
}

/// The control stages of a `stages` config value: the default (GLIDING only, no setpoint) when it is absent or
/// not an object.
fn stages(value: Option<&Value>) -> [Option<Setpoint>; 5] {
    let mut stages = [None; 5];
    let Some(entries) = value.and_then(Value::as_object) else {
        stages[Stage::Gliding as usize - 1] = Some(Setpoint::default());
        return stages;
    };
    for (name, entry) in entries {
        let Some(stage) = Stage::from_name(name) else {
            continue;
        };
        let angle = |key: &str| {
            entry
                .get(key)
                .and_then(Value::as_f64)
                .map_or(0.0, |degrees| degrees as f32)
        };
        stages[stage as usize - 1] = Some(Setpoint {
            roll: angle("roll"),
            pitch: angle("pitch"),
        });
    }
    stages
}

/// What guidance reads of the Mission's landing zone, resolved ONCE per zone (plan B's memoized
/// `mission.zone_points()`, and the Mission's `endgame_heading()` answer).
///
/// The zone never moves in flight, so the caller (the Mission's owner) builds this when the zone is set and hands
/// it in with every tick's [`Inputs`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Landing {
    /// One corner of the zone, as the operator set it (launch.config's top-left).
    pub top_left: Position,
    /// The diagonally opposite corner.
    pub bottom_right: Position,
    /// The zone's centre, where the glider lands.
    pub target: Position,
    /// The first short-side entrance; the centreline runs from it to `gate_b`.
    pub gate_a: Position,
    /// The second short-side entrance; it also sets the long-axis direction of the endgame patterns.
    pub gate_b: Position,
    /// The pattern `Auto` resolves to for this zone's shape: the Mission's answer. A `Landing` built with `Auto`
    /// here flies the circle, the reliable in-zone default.
    pub endgame_heading: Heading,
}

impl Landing {
    /// The landing geometry of the zone with these two diagonally opposite corners, and `endgame_heading`, the
    /// pattern the Mission resolves `Auto` to for this zone (the mission crate's `Mission::endgame_heading`).
    /// The Mission owns that rule, as it owns the zone; guidance only asks when its config says 'auto'.
    pub fn new(top_left: Position, bottom_right: Position, endgame_heading: Heading) -> Landing {
        let (target, gate_a, gate_b) = navigation::zone(top_left, bottom_right);
        Landing {
            top_left,
            bottom_right,
            target,
            gate_a,
            gate_b,
            endgame_heading,
        }
    }
}

/// A GNSS position some source vouches for, with its age.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Fix {
    /// Where the glider is.
    pub position: Position,
    /// How long ago the receiver reported it, milliseconds.
    pub age_ms: u32,
}

/// What plan B's injected handles answer this tick: the caller reads its channels and the Mission and hands the
/// answers in (see the module notes).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Inputs {
    /// The landing zone (the Mission's), or None when no zone is set: then the law holds the captured heading.
    pub landing: Option<Landing>,
    /// The launch origin (mission.launch_point: the operator-set pad, else a fresh GNSS fix), or None.
    pub launch: Option<Position>,
    /// The GNSS position, None when no fresh source vouches for it (the databoard window passed).
    pub position: Option<Fix>,
    /// The height above the ground in metres from a FRESH source (the laser), None when stale or out of range.
    pub agl: Option<f32>,
    /// The baro height above the pad in metres from a FRESH source, None when stale or not fitted.
    pub elevation: Option<f32>,
    /// The governor's airspeed estimate, m/s.
    pub airspeed: f32,
    /// The wind blowing toward the east, m/s (the wind estimator's components(); 0 with none).
    pub wind_east: f32,
    /// The wind blowing toward the north, m/s.
    pub wind_north: f32,
}

/// Whether the zone is still within gliding reach (guidance.reachability), as the flight panel shows it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reach {
    /// Whether the wind-adjusted reach covers the distance to the zone's centre.
    pub reachable: bool,
    /// Reach minus distance, metres, rounded half to even (negative: short by that much).
    pub margin_m: i32,
    /// The distance to the zone's centre, metres, rounded half to even.
    pub distance_m: i32,
    /// The wind component AGAINST the direction to the zone, m/s to 0.1 (negative: a tailwind).
    pub headwind_mps: f32,
}

impl Reach {
    /// The health vitals' `reach` object, as plan B's dict reads: `{reachable, margin_m, distance_m,
    /// headwind_mps}`.
    pub fn to_json(self) -> Value {
        json!({
            "reachable": self.reachable,
            "margin_m": self.margin_m,
            "distance_m": self.distance_m,
            "headwind_mps": json_number(self.headwind_mps),
        })
    }
}

/// The steering noise filter (guidance `_filter_error`): an all-integer EMA on the heading error.
///
/// The measured heading jitters at the control rate under sensor noise (the navigation target is already
/// cached at the GNSS rate). The state is in sixteenths of a degree; alpha is 1/2^shift. A GENUINE target change
/// (a jump over 90 degrees: an overfly flip, a law handover) re-seeds the state, so steering follows at once;
/// only per-step jitter is smoothed. Shift 0 disables it.
///
/// The answer rounds the sixteenths toward ZERO, not toward negative infinity. Floor division rounds a negative
/// state away from zero (-15 // 16 == -1) and a positive one toward it, which on a symmetric oscillation leaves a
/// one-sided bias in the STEERING command: measured -0.5 degree of steady output for a +/-8 degree input whose
/// true mean is zero, whichever way the oscillation starts. Small, but systematic -- it always pushes the same
/// way, so it does not average out over a flight. Only the final division needed it: a symmetric shift was
/// measured and changes nothing, and a CONSTANT error the filter was seeded with stays exact. Rust's integer
/// division truncates toward zero, which is that rule; the shift stays arithmetic (flooring), as plan B's `>>`
/// is. Same class as the PID integral ratchet: every control path that averages a signed quantity has to say
/// which way it rounds.
///
/// One asymmetry remains, and is ported as it is: the flooring shift moves the state by nothing for a positive
/// step under 2^shift sixteenths but by at least one sixteenth for a negative one, so a constant approached from
/// below stalls up to 7/16 short and reads one degree low (seeded at 40, a steady 48 settles at 47), while one
/// approached from above arrives exactly. Plan B's own test allows that degree.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SteeringFilter {
    /// The EMA divisor power; 0 disables the filter.
    shift: u32,
    /// The state in sixteenths of a degree; None until the next error seeds it. i64, so no error an i32 can
    /// carry overflows the scaling.
    state_sixteenths: Option<i64>,
}

impl SteeringFilter {
    /// The state's fixed-point scale: sixteenths of a degree.
    const SCALE: i64 = 16;

    /// A jump in the error larger than this, degrees, is a genuine target change, followed at once.
    const JUMP_DEGREES: i64 = 90;

    /// A filter with this EMA divisor power (alpha = 1/2^shift), unseeded.
    pub fn new(shift: u32) -> SteeringFilter {
        SteeringFilter {
            shift,
            state_sixteenths: None,
        }
    }

    /// Forget the state: the next error seeds it (a fresh control entry).
    pub fn reset(&mut self) {
        self.state_sixteenths = None;
    }

    /// The filtered heading error in whole degrees for this step's raw `error`: the raw error itself when the
    /// filter is off, when it seeds, or on a genuine target change.
    pub fn filter(&mut self, error: i32) -> i32 {
        if self.shift == 0 {
            return error;
        }
        let error_sixteenths = i64::from(error) * Self::SCALE;
        let state = match self.state_sixteenths {
            Some(state) if (error_sixteenths - state).abs() <= Self::JUMP_DEGREES * Self::SCALE => {
                // a shift of 63 or more leaves 0 or -1 in i64, as any larger one does in Python's integers
                state + ((error_sixteenths - state) >> self.shift.min(63))
            }
            _ => {
                self.state_sixteenths = Some(error_sixteenths);
                return error;
            }
        };
        self.state_sixteenths = Some(state);
        (state / Self::SCALE) as i32
    }
}

/// Which lobe of the 'oo' endgame the glider orbits, and which way along the axis the 'ov' leg runs: a SIGN
/// along the long axis (plan B `_LOBE_B` = 1, `_LOBE_A` = -1), flipped by negation at each crossing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Lobe {
    /// The gate_a side (-1).
    GateA,
    /// The gate_b side (+1), where a fresh entry starts.
    GateB,
}

impl Lobe {
    /// The sign the lobe centre (or the leg direction) takes along the centre -> gate_b unit vector.
    fn sign(self) -> f32 {
        match self {
            Lobe::GateA => -1.0,
            Lobe::GateB => 1.0,
        }
    }

    /// The other lobe.
    fn other(self) -> Lobe {
        match self {
            Lobe::GateA => Lobe::GateB,
            Lobe::GateB => Lobe::GateA,
        }
    }
}

/// The shortest signed heading error `target - current` in INTEGER degrees, wrapped into [-180, 180], so 350 ->
/// 10 is +20 and not -340 (guidance.heading_error).
///
/// The difference is truncated toward zero first (plan B's `int()`), then wrapped (commons.wrap180, which keeps
/// both +180 and -180 as they are). A NaN difference is no error at all here, where plan B's `int()` raises.
pub fn heading_error(target: f32, current: f32) -> i32 {
    let degrees = i64::from((target - current) as i32);
    let wrapped = if (-180..=180).contains(&degrees) {
        degrees
    } else {
        (degrees + 180).rem_euclid(360) - 180
    };
    wrapped as i32
}

/// The roll to hold for a heading error (commons.bank_demand): proportional, with a symmetric hard clamp. A
/// banked turn is tight (~v^2/(g tan(bank))) where a flat rudder skid is wide and weak, so the glider does not
/// over-range a small zone and the overshoot loop becomes an altitude-bleeding orbit.
fn bank_demand(heading_error: i32, gain: f32, limit: f32) -> f32 {
    between(-limit, gain * heading_error as f32, limit)
}

/// The signed time from `earlier_us` to `now_us`, microseconds: negative when the clock reads earlier, as plan
/// B's tick difference is (Rust's uptime does not wrap, so no 2^30 wrap is reproduced).
fn elapsed_us(now_us: u64, earlier_us: u64) -> i64 {
    now_us.wrapping_sub(earlier_us) as i64
}

/// `value` as a JSON number with its shortest f32 text, so 2.5 reads back 2.5 (JSON numbers are f64 only at this
/// boundary, decision D18).
fn json_number(value: f32) -> Value {
    value
        .to_string()
        .parse::<f64>()
        .ok()
        .and_then(serde_json::Number::from_f64)
        .map_or(Value::Null, Value::Number)
}

/// The per-stage control law (guidance.Guidance).
///
/// [`Guidance::setpoint`] gates the control stages; [`Guidance::enter`] captures the holds on entering control;
/// [`Guidance::compute`] runs the stage's law and fills the setpoint slots.
#[derive(Clone, Debug, PartialEq)]
pub struct Guidance {
    /// The knobs.
    config: GuidanceConfig,
    /// The roll setpoint, degrees: what the roll PID chases.
    roll_setpoint: f32,
    /// The pitch setpoint, degrees.
    pitch_setpoint: f32,
    /// The (filtered) heading error, whole degrees: what the yaw PID nulls.
    heading_error: i32,
    /// Whether the last navigation pass flew on dead reckoning (telemetry reads it).
    reckoning: bool,
    /// The heading captured on entering control: the blind fallback and the hold law's heading.
    heading_hold: f32,
    /// The rod-vertical roll captured on entering control, held through the boost climb, degrees.
    roll_hold: f32,
    /// The rod-vertical pitch captured on entering control, degrees.
    pitch_hold: f32,
    /// The cached target heading, degrees; None until the next navigation pass computes one.
    target_heading: Option<f32>,
    /// When the cached target heading was computed (the caller's clock), microseconds.
    target_updated_us: u64,
    /// The steering noise filter on the heading error.
    filter: SteeringFilter,
    /// The 'oo' lobe or the 'ov' leg direction.
    lobe: Lobe,
    /// 'oo' hysteresis: the glider was within a lobe radius of the centre on the last pass.
    was_near: bool,
    /// The last real fix: the dead-reckoning SEED, never moved by the reckoning itself.
    reckoned: Option<Position>,
    /// Metres travelled east of that seed since the fix was lost.
    reckon_east: f32,
    /// Metres travelled north of it. Metres, not degrees: see [`Guidance::compute`]'s dead reckoning.
    reckon_north: f32,
    /// When the reckoned offset was last advanced (the caller's clock), microseconds.
    reckoned_us: u64,
}

impl Guidance {
    /// A guidance law with these knobs, before any control entry: level setpoints, no heading error.
    pub fn new(config: GuidanceConfig) -> Guidance {
        let filter = SteeringFilter::new(config.steer_filter_shift);
        Guidance {
            config,
            roll_setpoint: 0.0,
            pitch_setpoint: 0.0,
            heading_error: 0,
            reckoning: false,
            heading_hold: 0.0,
            roll_hold: 0.0,
            pitch_hold: 0.0,
            target_heading: None,
            target_updated_us: 0,
            filter,
            lobe: Lobe::GateB,
            was_near: false,
            reckoned: None,
            reckon_east: 0.0,
            reckon_north: 0.0,
            reckoned_us: 0,
        }
    }

    /// The knobs.
    pub fn config(&self) -> &GuidanceConfig {
        &self.config
    }

    /// The stage's configured attitude setpoint, or None when it is not a CONTROL stage (guidance.setpoint):
    /// SETTING, BOOSTING and DONE are not, by default -- no actuation under thrust or on the ground. The flight
    /// task holds the fins neutral on None and computes only a stage that has one.
    pub fn setpoint(&self, stage: Stage) -> Option<Setpoint> {
        self.config.stage_setpoint(stage)
    }

    /// The roll setpoint in degrees the last law left (it keeps its value while the boost law holds neutral).
    pub fn roll_setpoint(&self) -> f32 {
        self.roll_setpoint
    }

    /// The pitch setpoint in degrees the last law left.
    pub fn pitch_setpoint(&self) -> f32 {
        self.pitch_setpoint
    }

    /// The heading error in whole degrees the last law left.
    pub fn heading_error(&self) -> i32 {
        self.heading_error
    }

    /// Whether the last navigation pass navigated on dead reckoning, with no usable GNSS fix (flight.csv
    /// `reckoning`).
    pub fn reckoning(&self) -> bool {
        self.reckoning
    }

    /// The heading the steering law aims at, degrees, as last computed (and cached for the navigation period);
    /// None before the first navigation pass after a control entry.
    pub fn target_heading(&self) -> Option<f32> {
        self.target_heading
    }

    /// The dead-reckoned `(east, north)` metres travelled since the last real fix (zero while a fix is fresh).
    pub fn reckoned_offset(&self) -> (f32, f32) {
        (self.reckon_east, self.reckon_north)
    }

    /// The tightest coordinated turn, metres, the airframe can HOLD at `bank_degrees` and `airspeed` m/s
    /// (guidance.min_turn_radius): R = v^2 / (g tan(bank)); 0 when the airspeed or the bank is unusable (<= 0).
    ///
    /// A physical restriction, not a tuning knob: the loiter and endgame laws clamp their commanded radius to it,
    /// so they never ask for a turn the airframe cannot fly. At the 45 degree land-bank limit and ~14 m/s trim
    /// that floor is ~20 m, and it BOUNDS landing accuracy (doc/specs/coludo.md "Turn-radius limit").
    pub fn min_turn_radius(&self, bank_degrees: f32, airspeed: f32) -> f32 {
        let tan_bank = bank_degrees.to_radians().tan();
        if airspeed <= 0.0 || tan_bank <= 0.0 {
            return 0.0;
        }
        airspeed * airspeed / (GRAVITY * tan_bank)
    }

    /// The steepest bank, degrees, the ENDGAME spiral may hold at `airspeed` m/s, stall-margin bounded
    /// (guidance.endgame_bank).
    ///
    /// A coordinated turn at bank phi pulls load n = 1/cos(phi), raising the stall speed to stall_speed_1g x
    /// sqrt(n); requiring airspeed >= stall_margin x stall_speed_1g x sqrt(n) bounds phi to acos((stall_margin x
    /// stall_speed_1g / airspeed)^2). Steeper than the fixed `land_bank_limit` when the airspeed allows (a tighter
    /// R_min, a closer touchdown), collapsing toward wings level as speed decays toward the stall, and capped at
    /// the structural `endgame_max_bank`. 0 when too slow to bank at all (hold the wings level until the speed
    /// recovers); `land_bank_limit` when the gate is disabled.
    pub fn endgame_bank(&self, airspeed: f32) -> f32 {
        let floor = self.config.stall_speed_1g * self.config.stall_margin;
        if floor <= 0.0 {
            return self.config.land_bank_limit;
        }
        if airspeed <= floor {
            return 0.0;
        }
        let ratio = floor / airspeed;
        self.config.endgame_max_bank.min((ratio * ratio).acos().to_degrees())
    }

    /// The endgame turn-radius floor, metres, at the bank the endgame will ACTUALLY hold at `airspeed`
    /// (guidance.landing_turn_radius): the precision bound the flight panel reports for a land-short-vs-stretch
    /// decision.
    ///
    /// Off [`Guidance::endgame_bank`], not the fixed `land_bank_limit`: the spiral banks as steep as the live
    /// airspeed safely allows, up to 60 degrees, so quoting the 45 degree figure UNDER-states what the glider can
    /// do (20.0 m against 11.5 m at 14 m/s). None when the glider is too slow to bank at all, where the radius is
    /// UNBOUNDED: it used to read 0.0, which a panel shows as perfect precision, the exact inverse of the truth.
    pub fn landing_turn_radius(&self, airspeed: f32) -> Option<f32> {
        let bank = self.endgame_bank(airspeed);
        (bank > 0.0).then(|| self.min_turn_radius(bank, airspeed))
    }

    /// Can the glider still glide to the zone from here (guidance.reachability)?
    ///
    /// The still-air reach is elevation x `glide_ratio` (how far the current height carries at a nominal L/D).
    /// The wind component ALONG the bearing to the target scales the GROUND range, reach x (1 + along /
    /// airspeed): a tailwind toward the zone extends it, a headwind shrinks it; an airspeed of 0 skips the
    /// projection. Compared with the distance from the live fix to the zone's centre. The operator's EARLY
    /// warning of an unreachable zone, and the groundwork for a deliberate land-short decision instead of a
    /// doomed stretch.
    ///
    /// The wind and the airspeed are the tick's own [`Inputs`]: plan B passes them as arguments, and its only
    /// caller (the flight panel's vitals) passes exactly the estimator's components and the governor's airspeed,
    /// the values the inputs already hold. One source, so the panel can never quote a reach from another wind
    /// than the one the law steers with.
    ///
    /// None without a zone, a fresh elevation, a vouched-for fix (of any age) or a launch point: plan B reads the
    /// target through the Mission's geometry, which needs the launch point although the reach does not use it.
    pub fn reachability(&self, glide_ratio: f32, inputs: &Inputs) -> Option<Reach> {
        let landing = inputs.landing?;
        let elevation = inputs.elevation?;
        let fix = inputs.position?;
        inputs.launch?;
        let (distance, target_bearing) = navigation::range_bearing(fix.position, landing.target);
        let mut reach = elevation * glide_ratio;
        let mut headwind = 0.0;
        if inputs.airspeed > 0.0 {
            let radians = target_bearing.to_radians();
            let along = inputs.wind_east * radians.sin() + inputs.wind_north * radians.cos();
            reach *= (1.0 + along / inputs.airspeed).max(0.0);
            headwind = -along;
        }
        Some(Reach {
            reachable: reach >= distance,
            margin_m: (reach - distance).round_ties_even() as i32,
            distance_m: distance.round_ties_even() as i32,
            headwind_mps: (headwind * 10.0).round_ties_even() / 10.0,
        })
    }

    /// Entering a control stage from a non-control one (guidance.enter): capture the heading to hold blind and
    /// the rod-vertical attitude (`roll`, `pitch`, degrees) for the boost hold, and reset the navigation cache,
    /// the steering filter and the endgame lobe, so the first controlled step steers fresh. The dead-reckoning
    /// seed survives: a re-entry does not forget where the glider last was.
    pub fn enter(&mut self, heading: f32, roll: f32, pitch: f32) {
        self.heading_hold = heading;
        self.roll_hold = roll;
        self.pitch_hold = pitch;
        self.target_heading = None;
        self.filter.reset();
        self.lobe = Lobe::GateB; // the 'oo' endgame starts fresh: a stale near-latch would drop a crossing
        self.was_near = false;
    }

    /// Run `stage`'s law at the glider's `heading` (degrees) and the caller's `now_us`: fill the setpoint slots
    /// and say whether the fins may actuate (guidance.compute).
    ///
    /// BOOSTING holds the captured rod-vertical attitude; GLIDING and LANDING steer to the zone; any other
    /// configured control stage (ground tests) holds its configured setpoints and the captured heading. False
    /// only while the boost is still on the rod: the caller then holds the fins neutral.
    pub fn compute(&mut self, stage: Stage, heading: f32, now_us: u64, inputs: &Inputs) -> bool {
        match stage {
            Stage::Boosting => self.boost(inputs.airspeed),
            Stage::Gliding | Stage::Landing => self.steer(stage, heading, now_us, inputs),
            Stage::Setting | Stage::Done => self.hold(stage, heading),
        }
    }

    /// BOOSTING: hold the rod-vertical attitude captured at entry, only PAST the rod (airspeed at least
    /// `boost_engage_speed`); no navigation or yaw steering near vertical, where the heading is ill-defined.
    fn boost(&mut self, airspeed: f32) -> bool {
        if airspeed < self.config.boost_engage_speed {
            return false;
        }
        self.roll_setpoint = self.roll_hold;
        self.pitch_setpoint = self.pitch_hold;
        self.heading_error = 0;
        true
    }

    /// GLIDING / LANDING: steer to the landing zone, banking into the turn.
    ///
    /// FRESH agl only (the input is None when stale): out of range an extrapolation would project stale samples
    /// without bound, and a bogus low reading would hold FINAL APPROACH on for the whole glide. FRESH elevation
    /// only: it opens the FULL land-bank authority, so an extrapolated one would unlock the endgame bank at the
    /// wrong height, or never. In the endgame band the remaining-altitude fraction shrinks the loiter radius, so
    /// the orbit SPIRALS IN onto the centre as the energy runs out.
    ///
    /// Endgame, final approach and LANDING take the FULL fin authority: spiral tight, crab the crosswind out (the
    /// residual at strong wind is airframe-bound, not a control gap). In the endgame the ceiling is the
    /// airspeed-gated bank; final and LANDING keep the fixed `land_bank_limit` (a straight crabbed approach, not a
    /// spiral). Otherwise GLIDING banks to turn toward the zone, and with no bank gain the configured roll holds.
    fn steer(&mut self, stage: Stage, heading: f32, now_us: u64, inputs: &Inputs) -> bool {
        let config = &self.config;
        let final_approach =
            config.final_approach_agl != 0.0 && inputs.agl.is_some_and(|agl| agl < config.final_approach_agl);
        let endgame = inputs
            .elevation
            .filter(|elevation| config.endgame_altitude_m != 0.0 && *elevation < config.endgame_altitude_m)
            .map(|elevation| (elevation / config.endgame_altitude_m).max(0.0));
        let target = self.target(heading, final_approach, now_us, endgame, inputs);
        self.heading_error = self.filter.filter(heading_error(target, heading));
        let config = &self.config;
        let setpoint = config.stage_setpoint(stage).unwrap_or_default();
        self.pitch_setpoint = setpoint.pitch;
        self.roll_setpoint =
            if config.land_bank_gain != 0.0 && (final_approach || endgame.is_some() || stage == Stage::Landing) {
                let limit = if endgame.is_some() {
                    self.endgame_bank(inputs.airspeed)
                } else {
                    config.land_bank_limit
                };
                bank_demand(self.heading_error, config.land_bank_gain, limit)
            } else if config.bank_gain != 0.0 && stage == Stage::Gliding {
                bank_demand(self.heading_error, config.bank_gain, config.bank_limit)
            } else {
                setpoint.roll
            };
        true
    }

    /// Any other configured control stage (ground-test configs): hold the configured setpoints and the heading
    /// captured at entry, with no navigation and no filter.
    fn hold(&mut self, stage: Stage, heading: f32) -> bool {
        self.heading_error = heading_error(self.heading_hold, heading);
        let setpoint = self.config.stage_setpoint(stage).unwrap_or_default();
        (self.roll_setpoint, self.pitch_setpoint) = (setpoint.roll, setpoint.pitch);
        true
    }

    /// The heading to steer in GLIDING / LANDING (guidance `_target_heading`).
    ///
    /// High on the glide it homes to the zone (gate, then centre); within `loiter_capture_m` of the centre, or
    /// anywhere in the ENDGAME band, it flies the holding pattern around the CENTRE unconditionally -- the gates
    /// are doorways for a glider with altitude to spend, but the last seconds must spiral down ON the target,
    /// not around the nearest doorway (the zone's long axis puts a gate ~90 m from the centre); low on FINAL
    /// approach it TRACKS the strip centreline. The tiers when homing:
    ///  1. a FRESH fix (younger than `position_age_max_ms`): steer from the current position (closed loop);
    ///  2. no fix, but one was seen: DEAD RECKONING from it;
    ///  3. never a fix, but a launch point: steer from the launch point (open loop);
    ///  4. neither: the captured heading (blind).
    ///
    /// The answer is cached for `navigation_period_us`: the position only moves at the GNSS rate. The cache key is
    /// time alone ON PURPOSE: a stage change cannot serve a stale heading, because [`Guidance::enter`] (called on
    /// every control-stage entry) clears it. Do not add a stage token. With no zone at all the captured heading is
    /// answered and nothing is cached.
    fn target(
        &mut self,
        heading: f32,
        final_approach: bool,
        now_us: u64,
        endgame: Option<f32>,
        inputs: &Inputs,
    ) -> f32 {
        let Some(landing) = inputs.landing else {
            return self.heading_hold;
        };
        if let Some(cached) = self.target_heading
            && elapsed_us(now_us, self.target_updated_us) < self.config.navigation_period_us
        {
            return cached;
        }
        self.target_updated_us = now_us;
        let fresh = inputs
            .position
            .filter(|fix| fix.age_ms < self.config.position_age_max_ms);
        let target = match fresh {
            Some(fix) => {
                // keep the dead-reckoning seed fresh, so losing the fix starts from the last real one
                (self.reckoned, self.reckoned_us, self.reckoning) = (Some(fix.position), now_us, false);
                (self.reckon_east, self.reckon_north) = (0.0, 0.0);
                self.homing(
                    fix.position,
                    heading,
                    final_approach,
                    endgame,
                    &landing,
                    inputs.airspeed,
                )
            }
            None => {
                // No usable fix. GNSS is EXPECTED to be missing for much of a flight -- the boost is a high-g,
                // high-vibration, antenna-shadowed few seconds, and a receiver that loses lock there can take
                // tens of seconds to reacquire, which on a <60 s flight can mean it never returns. A design case,
                // not an error path.
                self.reckoning = self.reckoned.is_some();
                self.reckon(now_us, heading, &landing, inputs)
            }
        };
        self.target_heading = Some(target);
        target
    }

    /// Tier 1, a fresh fix at `position`: the centreline on final, the holding pattern near the centre or in the
    /// endgame, else the gate steering.
    fn homing(
        &mut self,
        position: Position,
        heading: f32,
        final_approach: bool,
        endgame: Option<f32>,
        landing: &Landing,
        airspeed: f32,
    ) -> f32 {
        let config = &self.config;
        if final_approach {
            return navigation::approach_to(
                position,
                landing.target,
                landing.gate_a,
                landing.gate_b,
                heading,
                config.final_cross_gain,
                config.final_intercept,
            );
        }
        let (east, north) = navigation::offset(position, landing.target);
        let span = (east * east + north * north).sqrt();
        if endgame.is_none() && span > config.loiter_capture_m {
            // far out: travel to the zone through the nearer gate, as always
            return steer_to(position, landing);
        }
        let pattern = match config.endgame_pattern {
            Heading::Auto => landing.endgame_heading,
            configured => configured,
        };
        match pattern {
            Heading::TwoLobes => self.two_lobes_heading(position, landing, endgame, airspeed),
            Heading::Oval => self.oval_heading(position, landing, endgame),
            Heading::Auto | Heading::Circle => self.circle_heading(east, north, span, endgame, airspeed),
        }
    }

    /// Tiers 2 to 4, no usable fix (guidance `_reckon`): dead-reckon from the last fix, else steer from the
    /// launch point, else hold the captured heading.
    ///
    /// Dead reckoning advances the last known position by the velocity the glider can still measure without
    /// GNSS: airspeed along the current heading, plus the last wind estimate. It degrades gradually (heading error
    /// and stale wind integrate) where the old fallback did not degrade at all: it steered the pad -> target
    /// bearing forever, ignoring that the glider had moved, so it flew over the target and kept going. Every input
    /// survives a dead GNSS: the airspeed is the pitot/governor, the heading the fused compass. The wind estimate
    /// itself needs GNSS, so it freezes at its last value: the dominant error, bounded and known -- a frozen 5 m/s
    /// estimate wrong by 2 m/s costs ~80 m over 40 s, against the ~720 m a frozen position costs.
    ///
    /// The displacement accumulates in METRES from the seed fix and becomes a position once per pass. Plan B
    /// measured why on its board: MicroPython there is single precision, so a latitude near 48 degrees has a
    /// ~0.4 m ulp, and adding a 0.2 m step (2 m/s over a 100 ms tick) to it changes NOTHING -- per-step
    /// integration in degrees would quantize away precisely when the glider is slow, which is exactly when dead
    /// reckoning has to work, and would look healthy while holding position. Here positions are integers, but the
    /// metres stay the state, so the rounding to a unit happens once per pass on the whole offset, never per step.
    /// A step longer than half a second (or a clock that reads earlier) advances nothing.
    fn reckon(&mut self, now_us: u64, heading: f32, landing: &Landing, inputs: &Inputs) -> f32 {
        let Some(seed) = self.reckoned else {
            return match inputs.launch {
                Some(launch) => steer_to(launch, landing),
                None => self.heading_hold,
            };
        };
        let step_us = elapsed_us(now_us, self.reckoned_us);
        self.reckoned_us = now_us;
        if 0 < step_us && step_us < RECKON_MAXIMUM_STEP_US {
            let step_s = step_us as f32 / MICROSECONDS_PER_SECOND;
            let radians = heading.to_radians();
            let east = inputs.airspeed * radians.sin() + inputs.wind_east;
            let north = inputs.airspeed * radians.cos() + inputs.wind_north;
            self.reckon_east += east * step_s;
            self.reckon_north += north * step_s;
        }
        steer_to(navigation::advance(seed, self.reckon_east, self.reckon_north), landing)
    }

    /// The radius the endgame patterns orbit at, metres: the loiter radius, shrunk in the endgame band by the
    /// remaining-altitude fraction, but never tighter than the turn the airframe can hold at the phase's bank
    /// (the gated endgame bank, or the cruise `bank_limit` above the band).
    fn orbit_radius(&self, endgame: Option<f32>, airspeed: f32) -> f32 {
        let bank = match endgame {
            Some(_) => self.endgame_bank(airspeed),
            None => self.config.bank_limit,
        };
        (self.config.loiter_radius_m * endgame.unwrap_or(1.0)).max(self.min_turn_radius(bank, airspeed))
    }

    /// 'o': a single constant-radius orbit around the centre (guidance `_circle_heading`), given the `(east,
    /// north)` metres from the glider to the centre and their length `span`.
    ///
    /// The tangent heading (bearing + 90) corrected inward or outward by the radius error. One fixed orbit
    /// direction, so noise never flips the turn into S-hunting; in the endgame the radius shrinks with the
    /// altitude (a spiral collapsing toward the centre), clamped to the physical minimum turn radius.
    fn circle_heading(&self, east: f32, north: f32, span: f32, endgame: Option<f32>, airspeed: f32) -> f32 {
        let radius = self.orbit_radius(endgame, airspeed);
        let correction = between(
            -CUT_LIMIT_DEGREES,
            self.config.loiter_gain * (span - radius),
            CUT_LIMIT_DEGREES,
        );
        (navigation::compass(east, north) + 90.0 - correction).rem_euclid(360.0)
    }

    /// The unit vector along the zone's long axis, from the centre toward gate_b, and the half-length it spans in
    /// metres (1 for a degenerate zone, where the direction is then 0 and nothing divides by zero).
    fn long_axis(landing: &Landing) -> ((f32, f32), f32) {
        let (east, north) = navigation::offset(landing.target, landing.gate_b);
        let length = (east * east + north * north).sqrt();
        let half_length = if length == 0.0 { 1.0 } else { length };
        ((east / half_length, north / half_length), half_length)
    }

    /// 'oo': two lobes offset along the long axis, covering an elongated strip (guidance `_oo_heading`).
    ///
    /// Each lobe is a circle of radius r offset r along the axis, so it passes through the centre. The lobe FLIPS
    /// at each centre crossing (entering within r of the centre), but the turn SENSE stays fixed (+90): both loops
    /// turn the same way, so the transition is smooth (the sharp sense reversal of a true figure-8 diverged). r
    /// stays bounded (~loiter radius) and shrinks with the endgame altitude, so the pair collapses toward the
    /// centre. KNOWN LIMITATION: on a strip narrower than ~2r the lobes reach past the short edges; it lands out
    /// of the zone in HITL on the ~40 m HPRC strip, and fitting r to the width needs the steep bank whose sink then
    /// drops it out anyway. 'o' stays the reliable in-zone default; 'oo' needs more geometry work.
    fn two_lobes_heading(&mut self, position: Position, landing: &Landing, endgame: Option<f32>, airspeed: f32) -> f32 {
        let ((unit_east, unit_north), _) = Self::long_axis(landing);
        let radius = self.orbit_radius(endgame, airspeed);
        let (glider_east, glider_north) = navigation::offset(landing.target, position);
        if glider_east * glider_east + glider_north * glider_north < radius * radius {
            if !self.was_near {
                self.lobe = self.lobe.other(); // a crossing: the other lobe, the turn sense unchanged
            }
            self.was_near = true;
        } else {
            self.was_near = false;
        }
        // the tangent to the current lobe's circle (the FIXED +90 sense) plus a cut to hold the radius
        let sign = self.lobe.sign();
        let delta_east = sign * radius * unit_east - glider_east;
        let delta_north = sign * radius * unit_north - glider_north;
        let length = (delta_east * delta_east + delta_north * delta_north).sqrt();
        let to_centre = if length == 0.0 { 1.0 } else { length };
        let cut = between(
            -CUT_LIMIT_DEGREES,
            self.config.loiter_gain * (to_centre - radius),
            CUT_LIMIT_DEGREES,
        );
        (navigation::compass(delta_east, delta_north) + 90.0 - cut).rem_euclid(360.0)
    }

    /// 'ov': a CONVERGING centreline oval, a racetrack, for a moderately elongated strip (guidance
    /// `_oval_heading`).
    ///
    /// The 'oo' failure is its round lobes bulging +/-r across the SHORT axis; a straight leg has no
    /// perpendicular extent, so this flies ALONG the long axis holding the centreline (the final approach's
    /// intercept: proportional to the cross-track offset and capped, so it tracks without the S-hunt limit cycle
    /// of a raw pursuit) and reverses at each leg end. The KEY is convergence: the reversal point shrinks with the
    /// endgame altitude, so the racetrack COLLAPSES onto the centre like the 'o' spiral and lands ON the midline
    /// near the centre, in the zone; it is floored at 15 % of the half-length, so a small end-to-end oscillation
    /// persists down to touchdown rather than a dead stop on the line.
    fn oval_heading(&mut self, position: Position, landing: &Landing, endgame: Option<f32>) -> f32 {
        let ((unit_east, unit_north), half_length) = Self::long_axis(landing);
        let (glider_east, glider_north) = navigation::offset(landing.target, position);
        let along = glider_east * unit_east + glider_north * unit_north; // 0 at the centre
        // plan B's max(0.15, min(1.0, fraction)); the fraction is never NaN (a NaN elevation opens no endgame)
        let reach = half_length * between(OVAL_REACH_FLOOR, endgame.unwrap_or(1.0), 1.0);
        match self.lobe {
            Lobe::GateB if along >= reach => self.lobe = Lobe::GateA,
            Lobe::GateA if along <= -reach => self.lobe = Lobe::GateB,
            _ => {}
        }
        let sign = self.lobe.sign();
        let leg_bearing = navigation::compass(sign * unit_east, sign * unit_north);
        let intercept = between(
            -self.config.final_intercept,
            -self.config.final_cross_gain * navigation::cross_track(position, landing.target, leg_bearing),
            self.config.final_intercept,
        );
        (leg_bearing + intercept).rem_euclid(360.0)
    }
}

/// The gate-steering bearing from `position` through the zone of `landing` (navigation.steer_to with the
/// resolved geometry).
fn steer_to(position: Position, landing: &Landing) -> f32 {
    navigation::steer_to(
        position,
        landing.top_left,
        landing.bottom_right,
        landing.target,
        landing.gate_a,
        landing.gate_b,
    )
    .bearing
}

#[cfg(test)]
mod tests;
