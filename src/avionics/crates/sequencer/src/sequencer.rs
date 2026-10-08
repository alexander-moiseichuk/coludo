//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-stage machine, from src/glider tasks/sequencer.py (`Sequencer._tick` and its detectors): the
//! guarded, forward-only stage automation the control loop gates on.
//!
//! - SETTING -> BOOSTING: |accel| over `launch_g` for `launch_ms` of net time (motor ignition), OR the baro
//!   climbing past `launch_alt_m` for as long (an independent, threshold-robust backup), having risen
//!   `launch_alt_m` within `launch_alt_window_ms` above a level held for `launch_ms` (the rate gate, not plan B's:
//!   weather drift is no launch, nor one low read). Never the GNSS elevation.
//! - BOOSTING -> GLIDING: the separation switch is primary, and it moves the stage from OUTSIDE (the firmware's
//!   separation debounce, plan B drivers/separation.py) -- the machine sees it as an external move. Else the baro
//!   APOGEE (`apogee_drop_m` below the smoothed peak, blind for `apogee_arm_ms` after BOOSTING entry); the
//!   burnout timeout last.
//! - GLIDING -> LANDING: the height below `land_agl_m` for `land_ms`: the laser when it is fresh, else the
//!   elevation.
//! - LANDING -> DONE: |accel| inside 1 +/- `still_g` for `ground_ms` (stopped on the ground), or the landing
//!   timeout.
//! - Any airborne stage -> DONE: the RSO flight timeout, `flight_timeout_ms` after BOOSTING entry.
//!
//! [`StageMachine::step`] is `_tick`, pure: the caller passes the time, the stage and the hold the flight word
//! holds, and the fresh readings, and gets back what to record and where to go. It reads no clock and no channel,
//! allocates nothing, and the caller commits the advance through the flight word's writer
//! ([`crate::FlightWriter::advance`]) before the next step (plan B's `_advance` set the Controller's stage on the
//! spot).
//!
//! [`Settings::from_config`] reads the sequencer's board-config block at bring-up and refuses a value it cannot run
//! as written (text, a fractional or negative ms, or one longer than its clock reaches): the board then fails the
//! entry, so `arm` refuses. Plan B crashed a detector in flight on text, and flew a fractional or negative ms.
//!
//! **Freshness is the whole point.** Plan B's `Databoard.value()` answered a stale channel by extrapolating its
//! last two samples without bound, and production consumers used it: the laser only reaches about 4 m, so for
//! all but the last seconds of a flight the agl channel is stale, and its projection across a whole boost is a
//! noise-signed number -- `landing; agl -9.6m` fired 0.38 s after `gliding; apogee 274m`, ending the flight at
//! apogee. Every detector here takes a [`Reading`] only when a source vouches for it (plan B's `read()` with a
//! non-None source); the Rust databoard has no extrapolation at all, so a stale channel is simply None.
//!
//! Plan B's GC policy (compact and disable at BOOSTING, re-enable at DONE) is not ported: Rust has no
//! collector. Neither is its sequencer.csv push, which is the caller's: [`Step`] carries the rows.

use core::fmt;

use serde_json::Value;

use crate::commons::{self, Apogee, Dwell};
use crate::controller::Stage;

/// The sequencer's board-config keys, each field named after its key (the contract): the step period and the
/// stage machine's thresholds, read by [`Settings::from_config`] with plan B's `Sequencer.setup()` defaults for a
/// missing one ([`Settings::default`]), and the altitude launch's rate gate, which plan B does not have.
/// `disable_gc_flight` is not ported.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The step period in ms (plan B's loop sleep): the control thread steps the machine on a `period_ms` grid of
    /// its uptime clock, on every tick when the tick is slower. The machine itself never reads it: it only ever
    /// sees the time it is given.
    pub period_ms: u32,
    /// The boost |accel| in g that starts the launch dwell. The measured v2 F15 stack (517 g) boosts at only
    /// ~2.84 g, so 3.0 would MISS the launch of the real airframe; config_default ships 2.5.
    pub launch_g: f32,
    /// The launch dwell in ms of net time, for the accel and the baro trigger alike -- and, as in plan B, the
    /// apogee dwell too (§7.2 Q14: ported faithfully, to stay comparable with the HITL baselines). The baro trigger's
    /// rate gate also takes it as how long a level must hold to be the pad a climb is measured from.
    pub launch_ms: u32,
    /// The baro launch backup in metres above the pad: clearly climbed off the rod.
    pub launch_alt_m: f32,
    /// The baro launch backup's rate gate in ms: the elevation must also have risen `launch_alt_m` above the lowest
    /// level it held for `launch_ms` within this window (the hold ending in it), through the whole `launch_ms` dwell.
    /// Not plan B's (operator, 2026-10-08): TMS-7F launched unarmed on the bench overnight when weather drift carried
    /// its elevation +10 m in 2.17 h past `launch_alt_m`. A rocket gains 10 m within about a second of ignition even
    /// on the slowest v2 stack (2.84 g), drift takes hours. A window shorter than `launch_ms` holds no level that
    /// long, so the altitude launch never fires. At most [`LONGEST_LAUNCH_ALTITUDE_WINDOW_MS`].
    pub launch_alt_window_ms: u32,
    /// The burnout fallback in ms after BOOSTING entry, for a flat or absent baro. Plan B's setup() default is
    /// 6000 while config_default ships 12000 (past the slowest v2 apogee, ~11 s), so it never pre-empts the
    /// apogee.
    pub boost_timeout_ms: u32,
    /// How far the smoothed baro must fall below its peak to be apogee, in metres.
    pub apogee_drop_m: f32,
    /// How long after BOOSTING entry the apogee detector stays blind, peak tracking included: the motor's
    /// exhaust pressure wave can spike or dip the in-airframe baro during the burn, and an unarmed detector
    /// could either deploy GLIDING under thrust (wings folded, fins steering the stack) or latch a peak the real
    /// apogee never reads 5 m below. 4000 covers the longest burn (F15, 3.45 s) plus a margin.
    pub apogee_arm_ms: u32,
    /// The RSO backstop in ms after BOOSTING entry: with every landing sensor dead, a blind glider must not
    /// circle in GLIDING until the battery dies. 5 min is long past any physically possible TMS flight.
    pub flight_timeout_ms: u32,
    /// The landing height in metres. The VL53L1X reports at most ~4 m, so a board fitted with it lands on the
    /// baro fallback at the default 5.0 unless this comes nearer 3.0 (doc/hardware.md).
    pub land_agl_m: f32,
    /// How long the height must stay below `land_agl_m`, in ms: one low sample never flares.
    pub land_ms: u32,
    /// The stillness tolerance around 1 g, in g (always under 1: a tolerance, not a level).
    pub still_g: f32,
    /// How long |accel| must stay still to be on the ground, in ms.
    pub ground_ms: u32,
    /// The LANDING -> DONE backstop in ms after LANDING entry, for a touchdown that never goes still (carried
    /// off, rocking in wind, hung in a tree). Plan B's reason was memory: LANDING ran with the GC off and no
    /// rescue, and HITL ran the heap out ~54 s after such a touchdown. Rust keeps the bound because the flight
    /// must still end: a normal LANDING lasts 3.3-4.5 s (48 HITL flights), the descent from `land_agl_m` ~5 s.
    pub land_timeout_ms: u32,
}

impl Default for Settings {
    /// Plan B's `Sequencer.setup()` defaults, and the operator's 1 s rate gate: what a board config without the key
    /// flies.
    fn default() -> Settings {
        Settings {
            period_ms: 50,
            launch_g: 2.5,
            launch_ms: 100,
            launch_alt_m: 10.0,
            launch_alt_window_ms: 1000,
            boost_timeout_ms: 6000,
            apogee_drop_m: 5.0,
            apogee_arm_ms: 4000,
            flight_timeout_ms: 300_000,
            land_agl_m: 5.0,
            land_ms: 300,
            still_g: 0.3,
            ground_ms: 3000,
            land_timeout_ms: 10_000,
        }
    }
}

impl Settings {
    /// Read the sequencer's board-config block: each key it carries over its [`Settings::default`].
    ///
    /// It takes every value plan B flies as written and refuses the rest. A g or metre key takes any JSON number, as
    /// f32. A `*_ms` key takes a whole number of ms from 0 to 2147483647 (about 24.8 days, the longest the machine's
    /// signed elapsed time reaches), written with a fraction of zero too (`300.0`, which plan B flies as 300). The rate
    /// gate's `launch_alt_window_ms`, which plan B has none of, takes the same, up to
    /// [`LONGEST_LAUNCH_ALTITUDE_WINDOW_MS`], the window the machine's history holds. Anything else is refused,
    /// naming the key, so the board fails the entry at bring-up and `arm` refuses. Plan B
    /// with the same block: a text `launch_g` or `still_g` failed its setup; a text `period_ms` or `launch_alt_m`
    /// crashed the task on the pad; a text `launch_ms` passed arm and crashed at ignition; the other keys passed arm
    /// and crashed their stage's detector in flight; and a fractional or negative `*_ms` flew. Keys it does not know
    /// (the entry's own, `disable_gc_flight`) are ignored, as plan B ignores them.
    ///
    /// Args: `config` the sequencer's entry.
    ///
    /// Returns: the settings, or the first key refused and why.
    pub fn from_config(config: &Value) -> Result<Settings, SettingsError> {
        let defaults = Settings::default();
        let duration = |key, default| milliseconds(config, key, default, LONGEST_MS);
        Ok(Settings {
            period_ms: duration("period_ms", defaults.period_ms)?,
            launch_g: real(config, "launch_g", defaults.launch_g)?,
            launch_ms: duration("launch_ms", defaults.launch_ms)?,
            launch_alt_m: real(config, "launch_alt_m", defaults.launch_alt_m)?,
            launch_alt_window_ms: milliseconds(
                config,
                "launch_alt_window_ms",
                defaults.launch_alt_window_ms,
                LONGEST_LAUNCH_ALTITUDE_WINDOW_MS,
            )?,
            boost_timeout_ms: duration("boost_timeout_ms", defaults.boost_timeout_ms)?,
            apogee_drop_m: real(config, "apogee_drop_m", defaults.apogee_drop_m)?,
            apogee_arm_ms: duration("apogee_arm_ms", defaults.apogee_arm_ms)?,
            flight_timeout_ms: duration("flight_timeout_ms", defaults.flight_timeout_ms)?,
            land_agl_m: real(config, "land_agl_m", defaults.land_agl_m)?,
            land_ms: duration("land_ms", defaults.land_ms)?,
            still_g: real(config, "still_g", defaults.still_g)?,
            ground_ms: duration("ground_ms", defaults.ground_ms)?,
            land_timeout_ms: duration("land_timeout_ms", defaults.land_timeout_ms)?,
        })
    }
}

/// A g or metre key of `config`: any JSON number, as f32; `default` when the key is absent.
fn real(config: &Value, key: &'static str, default: f32) -> Result<f32, SettingsError> {
    let Some(value) = config.get(key) else {
        return Ok(default);
    };
    value
        .as_f64()
        .map(|number| number as f32)
        .ok_or_else(|| SettingsError::new(key, value, SettingsProblem::NotANumber))
}

/// The longest `*_ms` setting, `i32::MAX` (about 24.8 days): the machine compares a signed 32-bit elapsed time
/// ([`commons::reached`]), which never reaches a longer threshold, so one would silently switch its detector or
/// backstop off. Every `*_ms` key plan B has takes the same bound, `period_ms` included.
const LONGEST_MS: u32 = i32::MAX as u32;

/// The longest `launch_alt_window_ms`, twice the operator's 1 s: the machine keeps the elevation history for it in a
/// fixed array of 81 slots, 25 ms apart, 12 bytes each (972 B, in the machine on the control thread's stack), and
/// allocates nothing. A sluggish 1.5 g boost is 10 m up 2.0 s after ignition: a 2 s window
/// passes it then, the default 1 s about 0.5 s later.
pub const LONGEST_LAUNCH_ALTITUDE_WINDOW_MS: u32 = 2000;

/// A `*_ms` key of `config`: a whole number of ms up to `longest`, an integer or a float with no fraction;
/// `default` when the key is absent.
fn milliseconds(config: &Value, key: &'static str, default: u32, longest: u32) -> Result<u32, SettingsError> {
    let Some(value) = config.get(key) else {
        return Ok(default);
    };
    let refused = |problem| Err(SettingsError::new(key, value, problem));
    let too_long = SettingsProblem::TooLong { longest_ms: longest };
    let Value::Number(number) = value else {
        return refused(SettingsProblem::NotANumber);
    };
    if let Some(whole) = number.as_u64() {
        return match u32::try_from(whole) {
            Ok(whole) if whole <= longest => Ok(whole),
            _ => refused(too_long),
        };
    }
    // a negative integer, or a float
    let Some(float) = number.as_f64() else {
        return refused(SettingsProblem::NotANumber);
    };
    if float < 0.0 {
        return refused(SettingsProblem::Negative);
    }
    if float.fract() != 0.0 {
        return refused(SettingsProblem::Fractional);
    }
    if float > f64::from(longest) {
        return refused(too_long);
    }
    Ok(float as u32)
}

/// A sequencer key [`Settings::from_config`] refused. [`fmt::Display`] writes `<key>: <value> <why>`, the text
/// the board fails the entry with.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettingsError {
    /// The key.
    pub key: &'static str,
    /// Its value as the config writes it (JSON text).
    pub value: String,
    /// What is wrong with it.
    pub problem: SettingsProblem,
}

impl SettingsError {
    /// The refusal of `key`, whose value is `value`.
    fn new(key: &'static str, value: &Value, problem: SettingsProblem) -> SettingsError {
        SettingsError {
            key,
            value: value.to_string(),
            problem,
        }
    }
}

/// What is wrong with a refused sequencer key.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SettingsProblem {
    /// Not a JSON number: text, true or false, null, a list or an object.
    NotANumber,
    /// A `*_ms` below zero.
    Negative,
    /// A `*_ms` with a fraction: plan B flew one, a clock in whole ms cannot.
    Fractional,
    /// A `*_ms` past the longest its key takes: 2147483647 (`i32::MAX`, about 24.8 days), which the machine's signed
    /// elapsed time never passes, or for `launch_alt_window_ms` [`LONGEST_LAUNCH_ALTITUDE_WINDOW_MS`].
    TooLong {
        /// The longest the key takes, in ms.
        longest_ms: u32,
    },
}

impl fmt::Display for SettingsError {
    /// `<key>: <value> <why>`.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (key, value) = (self.key, &self.value);
        match self.problem {
            SettingsProblem::NotANumber => write!(formatter, "{key}: {value} is not a number"),
            SettingsProblem::Negative => write!(formatter, "{key}: {value} is negative"),
            SettingsProblem::Fractional => write!(formatter, "{key}: {value} is not a whole number of ms"),
            SettingsProblem::TooLong { longest_ms } => {
                write!(formatter, "{key}: {value} is past the longest, {longest_ms} ms")
            }
        }
    }
}

/// Who vouches for a fresh reading. Plan B's `read()` names the device; the stage machine asks one question of
/// it: is this the GNSS?
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// The GNSS. It publishes `elevation` too (at a lower priority than the baros), zeroed at its first fix and
    /// wandering metres on noise. The databoard falls through to it when both baros are briefly stale -- the
    /// ICP-10111 general-call recovery can take them both -- and on the board one 15 m GNSS sample with the
    /// baros stale tripped BOOSTING on the pad. So it never starts a flight; the apogee tracker and the landing
    /// fallback do take it, as in plan B.
    Gnss,
    /// Any other device: a barometer, the IMU, the laser.
    Other,
}

/// One fresh channel value, as plan B's `read()` answers it with a non-None source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading<T> {
    /// The value.
    pub value: T,
    /// The device that published it.
    pub source: Source,
}

/// The channels the stage machine reads, each None when no source is fresh.
///
/// None is not a low reading and not a still one: a detector without its reading does not advance (and the
/// height dwell resets). In plan B a fresh source always carries a value (a channel is born stale), so
/// "fresh" and "has a value" are one fact here.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Readings {
    /// The `accel` channel: (x, y, z) specific force in g.
    pub acceleration: Option<Reading<[f32; 3]>>,
    /// The `elevation` channel: metres above the pad (the baros, else the GNSS).
    pub elevation: Option<Reading<f32>>,
    /// The `agl` channel: the laser's height above the ground in metres.
    pub height_above_ground: Option<Reading<f32>>,
}

/// Why a stage changed: the reason column of sequencer.csv. [`fmt::Display`] writes plan B's text exactly,
/// since post-flight tools read it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reason {
    /// Moved from outside the stage machine (the separation switch, an operator `stage`, a warm-start
    /// restore). Recorded so post-flight tooling keeps ONE stage-event source instead of cross-referencing
    /// separation.csv, which a field capture pull may not even fetch. `external`.
    External,
    /// The baro launch backup: `launch alt=<elevation, 0 decimals>m dwell=<credit>ms`.
    LaunchAltitude {
        /// The elevation that completed the dwell.
        elevation_m: f32,
        /// The dwell's credit, capped at `launch_ms`.
        dwell_ms: u32,
    },
    /// The accel launch: `launch |a|=<magnitude, 1 decimal>g dwell=<credit>ms`. The magnitude is 0.0 when the
    /// sample that completed the dwell had no fresh accel (an inactive sample can complete it when the clock
    /// steps backwards; plan B prints 0.0 rather than taking the root of nothing).
    LaunchAcceleration {
        /// |accel| of the sample that completed the dwell, in g.
        magnitude_g: f32,
        /// The dwell's credit, capped at `launch_ms`.
        dwell_ms: u32,
    },
    /// Apogee: `apogee <smoothed peak, 0 decimals>m`.
    Apogee {
        /// The smoothed peak elevation.
        peak_m: f32,
    },
    /// The burnout fallback: `burnout timeout (no separation)`.
    BurnoutTimeout,
    /// Low enough to land: `agl <height, 1 decimal>m` -- the laser's, or the elevation it fell back to.
    HeightAboveGround {
        /// The height that completed the dwell.
        height_m: f32,
    },
    /// The LANDING backstop: `landing timeout <timeout_ms // 1000> s` (whole seconds, truncated as plan B
    /// truncates them: 2500 ms reads `2 s`).
    LandingTimeout {
        /// `land_timeout_ms`.
        timeout_ms: u32,
    },
    /// Still on the ground: `stationary <|accel|, 1 decimal>g`.
    Stationary {
        /// |accel| of the sample that completed the dwell, in g.
        magnitude_g: f32,
    },
    /// The RSO backstop: `flight timeout`.
    FlightTimeout,
}

impl fmt::Display for Reason {
    /// Plan B's reason text. Rust's `{:.N}` rounds the exact binary value half to even and keeps the sign of a
    /// negative that rounds to zero, as Python's `%.Nf` does.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Reason::External => formatter.write_str("external"),
            Reason::LaunchAltitude { elevation_m, dwell_ms } => {
                write!(formatter, "launch alt={elevation_m:.0}m dwell={dwell_ms}ms")
            }
            Reason::LaunchAcceleration { magnitude_g, dwell_ms } => {
                write!(formatter, "launch |a|={magnitude_g:.1}g dwell={dwell_ms}ms")
            }
            Reason::Apogee { peak_m } => write!(formatter, "apogee {peak_m:.0}m"),
            Reason::BurnoutTimeout => formatter.write_str("burnout timeout (no separation)"),
            Reason::HeightAboveGround { height_m } => write!(formatter, "agl {height_m:.1}m"),
            Reason::LandingTimeout { timeout_ms } => write!(formatter, "landing timeout {} s", timeout_ms / 1000),
            Reason::Stationary { magnitude_g } => write!(formatter, "stationary {magnitude_g:.1}g"),
            Reason::FlightTimeout => formatter.write_str("flight timeout"),
        }
    }
}

/// One stage change and why: one sequencer.csv row (`stage`, `reason`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transition {
    /// The stage entered.
    pub to: Stage,
    /// Why.
    pub reason: Reason,
}

impl Transition {
    /// Whether the move came from outside the stage machine (the separation switch, an operator, a warm start).
    pub fn external(&self) -> bool {
        self.reason == Reason::External
    }
}

/// What one step reports: up to two sequencer.csv rows, in the order plan B pushes them.
///
/// Two, because `_tick` first notices a move made from outside since the last tick and only then runs the
/// detectors, and both can record on one tick: an operator releases a hold that outlasted the flight timeout,
/// and the backstop fires on the very tick the held stage is first seen.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Step {
    /// A stage someone else moved to since the last step, recorded with [`Reason::External`].
    pub external: Option<Transition>,
    /// The stage this step advances to. The caller commits it ([`crate::FlightWriter::advance`]) before the next
    /// step.
    pub advance: Option<Transition>,
}

impl Step {
    /// The rows to record, in order.
    pub fn transitions(&self) -> impl Iterator<Item = Transition> {
        self.external.into_iter().chain(self.advance)
    }
}

/// The clock span of one slot of the climb history, in ms: the shortest step period it keeps apart. Half the shipped
/// `period_ms` (50), so the shipped steps, 50 ms apart give or take a control tick, each keep a slot of their own and
/// the window is exact. Faster steps share a slot, dated by its first reading: the window then loses up to a slot at
/// its far end, and never grows.
const CLIMB_SLOT_MS: u32 = 25;

/// The slots the climb history holds: as many as can begin within the longest window, one slot apart.
const CLIMB_SLOTS: usize = (LONGEST_LAUNCH_ALTITUDE_WINDOW_MS / CLIMB_SLOT_MS) as usize + 1;

/// One slot of the climb history.
#[derive(Clone, Copy, Debug, Default)]
struct Slot {
    /// The clock of the first reading in it.
    start_ms: u32,
    /// The highest elevation read in it, in metres: what a later reading's hold looks back at.
    highest_m: f32,
    /// The lowest level, in metres, that the elevation had held for `launch_ms` at one of its readings (the highest
    /// it read over the `launch_ms` before that reading); infinite while none of them had a history reaching back
    /// that far.
    held_m: f32,
}

/// The fresh barometer elevations of the last `launch_alt_window_ms`, for the altitude launch's rate gate: how far
/// the elevation has risen above the lowest level it held for `launch_ms` within the window. A level counts only
/// once held as long as the launch itself must be, so one low or glitched read is no pad to climb from, as one high
/// read is no climb. A fixed ring of [`CLIMB_SLOTS`] slots, so stepping it allocates nothing.
#[derive(Clone, Debug)]
struct Climb {
    /// The slots, a ring: `count` of them from `oldest` on, each starting at least [`CLIMB_SLOT_MS`] after the one
    /// before it.
    slots: [Slot; CLIMB_SLOTS],
    /// The index of the oldest slot.
    oldest: usize,
    /// How many slots hold readings.
    count: usize,
}

impl Climb {
    /// A history with no readings.
    fn new() -> Climb {
        Climb {
            slots: [Slot::default(); CLIMB_SLOTS],
            oldest: 0,
            count: 0,
        }
    }

    /// Forget every reading.
    fn clear(&mut self) {
        self.count = 0;
    }

    /// Record a fresh elevation and say how far it lies above the lowest level held for `held_ms` within the window.
    ///
    /// From the oldest on, a slot is dropped, its readings with it, while it began more than `window_ms` ago, or
    /// reads as begun after `now_ms` (the clock stepped back, or ran on 24.8 days or more, past what its signed
    /// elapsed time tells apart): nothing older than the window counts, nor a reading the clock no longer compares
    /// with. A reading less than [`CLIMB_SLOT_MS`] after the newest slot began (or before it: a clock stepping back)
    /// joins that slot; any other opens a new one. A window past [`LONGEST_LAUNCH_ALTITUDE_WINDOW_MS`], which only a
    /// hand-built `Settings` carries, acts as the longest. So a new slot finds the others begun between one slot and
    /// one window ago, a slot apart at least, and the ring never holds more than [`CLIMB_SLOTS`].
    ///
    /// The level this reading confirms is the highest read from the slot in effect `held_ms` ago (the newest begun
    /// that long ago) on, this reading included: the elevation held at or below it all that time, a gap in the
    /// readings holding the last one. With no slot begun that long ago, none is confirmed: right after the history
    /// starts, after a gap longer than the window, and always for a `held_ms` longer than the window.
    ///
    /// Args: `now_ms` the step clock; `elevation_m` the fresh elevation; `window_ms` `launch_alt_window_ms`;
    /// `held_ms` `launch_ms`.
    ///
    /// Returns: the rise in metres: the elevation minus the lowest level confirmed within the window, or 0.0 when
    /// none lies below it (NaN for a NaN elevation, which no comparison passes).
    fn rise(&mut self, now_ms: u32, elevation_m: f32, window_ms: u32, held_ms: u32) -> f32 {
        let window_ms = window_ms.min(LONGEST_LAUNCH_ALTITUDE_WINDOW_MS);
        let within = |start_ms: u32| {
            u32::try_from(commons::elapsed_ms(now_ms, start_ms)).is_ok_and(|age_ms| age_ms <= window_ms)
        };
        while self.count > 0 && !within(self.slot(0).start_ms) {
            self.oldest = (self.oldest + 1) % CLIMB_SLOTS;
            self.count -= 1;
        }
        if self.count > 0 && !commons::reached(now_ms, self.slot(self.count - 1).start_ms, CLIMB_SLOT_MS) {
            let newest = self.slot_mut(self.count - 1);
            newest.highest_m = newest.highest_m.max(elevation_m);
        } else {
            self.count += 1;
            *self.slot_mut(self.count - 1) = Slot {
                start_ms: now_ms,
                highest_m: elevation_m,
                held_m: f32::INFINITY,
            };
        }
        let mut highest_m = elevation_m;
        let confirmed = (0..self.count).rev().find_map(|offset| {
            let slot = self.slot(offset);
            highest_m = highest_m.max(slot.highest_m);
            commons::reached(now_ms, slot.start_ms, held_ms).then_some(highest_m)
        });
        if let Some(level_m) = confirmed {
            let newest = self.slot_mut(self.count - 1);
            newest.held_m = newest.held_m.min(level_m);
        }
        let lowest_m = (0..self.count)
            .map(|offset| self.slot(offset).held_m)
            .fold(elevation_m, f32::min);
        elevation_m - lowest_m
    }

    /// The slot `offset` places after the oldest.
    fn slot(&self, offset: usize) -> Slot {
        self.slots[(self.oldest + offset) % CLIMB_SLOTS]
    }

    /// The slot `offset` places after the oldest, to change.
    fn slot_mut(&mut self, offset: usize) -> &mut Slot {
        &mut self.slots[(self.oldest + offset) % CLIMB_SLOTS]
    }
}

/// The stage machine: plan B's `Sequencer` state, stepped by [`StageMachine::step`].
#[derive(Clone, Debug)]
pub struct StageMachine {
    /// The thresholds.
    settings: Settings,
    /// `launch_g` squared, so the accel compares without a root.
    launch_g_squared: f32,
    /// The still band's lower edge squared: (1 - `still_g`)^2, or 0 for a tolerance of 1 g or more.
    still_low_squared: f32,
    /// The still band's upper edge squared: (1 + `still_g`)^2.
    still_high_squared: f32,
    /// The start of the current sustained condition (the burnout, landing and stillness dwells), or None.
    since_ms: Option<u32>,
    /// The stage the last step saw, None before the first: any change resets the dwells, so a separation-driven
    /// hop starts clean.
    stage_seen: Option<Stage>,
    /// The stage this machine itself advanced to, until the next step sees it: tells its own moves from
    /// external ones.
    advanced_to: Option<Stage>,
    /// The apogee tracker, reset at every BOOSTING entry.
    apogee: Apogee,
    /// The leaky launch dwell on |accel|.
    launch_dwell: Dwell,
    /// The leaky launch dwell on the baro elevation.
    altitude_dwell: Dwell,
    /// The baro elevations of the rate gate's window, recorded on the pad.
    climb: Climb,
    /// The step clock of BOOSTING entry: arms the apogee detector and bases the flight timeout. Re-based on the
    /// first step of an airborne stage entered without BOOSTING (a warm start, an operator move).
    boost_entry_ms: Option<u32>,
    /// The step clock of LANDING entry: bases the landing timeout.
    landing_entry_ms: Option<u32>,
}

impl StageMachine {
    /// A machine that has seen nothing yet: its first step records the stage it finds as external (plan B starts
    /// with no stage seen, so even a cold boot's SETTING opens sequencer.csv as `setting,external`).
    pub fn new(settings: Settings) -> StageMachine {
        // the still band 1 +/- still_g compares squared; still_g is a tolerance around 1 g, so always under 1
        let still_low = 1.0 - settings.still_g;
        StageMachine {
            settings,
            launch_g_squared: settings.launch_g * settings.launch_g,
            still_low_squared: if settings.still_g < 1.0 {
                still_low * still_low
            } else {
                0.0
            },
            still_high_squared: (1.0 + settings.still_g) * (1.0 + settings.still_g),
            since_ms: None,
            stage_seen: None,
            advanced_to: None,
            apogee: Apogee::default(),
            launch_dwell: Dwell::default(),
            altitude_dwell: Dwell::default(),
            climb: Climb::new(),
            boost_entry_ms: None,
            landing_entry_ms: None,
        }
    }

    /// The thresholds this machine runs.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// One stage-machine step: plan B's `Sequencer._tick`, exactly.
    ///
    /// An operator hold (`manual`) suspends everything, the change detection included, so a held stage is first
    /// seen -- and recorded as external -- on the step after `stage auto`. Otherwise a stage different from the
    /// last one seen restarts the dwells and records an external move unless this machine made it; entering
    /// SETTING forgets the last flight's timers (else the backstop fired from SETTING on the pad one flight
    /// timeout after an operator's `stage gliding` / `stage setting`), entering LANDING starts the landing
    /// timeout, entering BOOSTING resets the apogee tracker and bases the arming window and the flight timeout,
    /// and entering GLIDING or LANDING without ever having seen BOOSTING re-bases the flight timeout there: the
    /// backstop re-bases, it never disappears. Then the RSO backstop, then the stage's own detector; each
    /// advances at most once.
    ///
    /// Only a stage that differs from the last one SEEN is a change, and a hold sees nothing. So a hold that
    /// ends where the machine last looked -- `stage gliding` for a fin check on the pad, then `stage setting` --
    /// is no change at all: nothing is recorded, nothing is reset, and a launch dwell credited before the hold
    /// survives it. The first boosting sample after it then spans the whole hold and completes the dwell alone:
    /// a launch on ONE sample, and in Rust that detected launch also ends the link's reconnects.
    /// Plan B behaves so; ported as is and pinned by sequencer_trace `a_hold_hides_the_return_to_setting`. The
    /// same holds for a move back to the seen stage between an advance and the next step: it goes unrecorded.
    ///
    /// Args: `now_ms` the caller's monotonic clock; `stage` the stage the flight word holds now (with the
    /// previous step's advance applied); `readings` the fresh channels; `manual` the operator hold.
    pub fn step(&mut self, now_ms: u32, stage: Stage, readings: &Readings, manual: bool) -> Step {
        let mut step = Step::default();
        if manual {
            return step; // the operator holds the stage: no auto-advance, nothing observed
        }
        if self.stage_seen != Some(stage) {
            self.since_ms = None;
            self.stage_seen = Some(stage);
            if self.advanced_to != Some(stage) {
                step.external = Some(Transition {
                    to: stage,
                    reason: Reason::External,
                });
            }
            self.advanced_to = None;
            self.enter(now_ms, stage);
        }
        if stage.airborne()
            && let Some(entry_ms) = self.boost_entry_ms
            && commons::reached(now_ms, entry_ms, self.settings.flight_timeout_ms)
        {
            step.advance = Some(self.advance(Stage::Done, Reason::FlightTimeout));
            return step;
        }
        step.advance = match stage {
            Stage::Setting => self.detect_launch(now_ms, readings),
            Stage::Boosting => self.detect_apogee(now_ms, readings),
            Stage::Gliding => self.detect_landing(now_ms, readings),
            Stage::Landing => self.detect_stationary(now_ms, readings),
            Stage::Done => None,
        };
        step
    }

    /// The bookkeeping of a newly seen stage (the second half of `_tick`'s change prologue).
    fn enter(&mut self, now_ms: u32, stage: Stage) {
        if stage == Stage::Setting {
            // back on the ground: forget the last flight's timers
            self.boost_entry_ms = None;
            self.landing_entry_ms = None;
            self.launch_dwell = Dwell::default();
            self.altitude_dwell = Dwell::default();
            self.climb.clear(); // the pad's elevation is measured afresh (after a re-zero, from the new ground)
        }
        if stage == Stage::Landing {
            self.landing_entry_ms = Some(now_ms); // starts the landing timeout (a warm start into LANDING too)
        }
        if stage == Stage::Boosting {
            self.apogee = Apogee::default(); // peak tracking starts fresh for this flight
            self.boost_entry_ms = Some(now_ms); // arms the apogee detector, starts the flight timeout
        } else if self.boost_entry_ms.is_none() && matches!(stage, Stage::Gliding | Stage::Landing) {
            self.boost_entry_ms = Some(now_ms); // airborne without BOOSTING seen: re-base the backstop here
        }
    }

    /// Advance to `to` (plan B `_advance`, minus the Controller write the caller makes): the dwell restarts, and
    /// the move is marked as this machine's own, so the next step does not record it again as external.
    fn advance(&mut self, to: Stage, reason: Reason) -> Transition {
        self.since_ms = None;
        self.advanced_to = Some(to);
        Transition { to, reason }
    }

    /// The contiguous dwell shared by the burnout, landing and stillness detectors (plan B `_sustained`): starts
    /// on the first call after a reset, then reports whether `threshold_ms` has passed. Callers clear
    /// `since_ms` when their condition lapses.
    fn sustained(&mut self, now_ms: u32, threshold_ms: u32) -> bool {
        let since_ms = *self.since_ms.get_or_insert(now_ms);
        commons::reached(now_ms, since_ms, threshold_ms)
    }

    /// SETTING -> BOOSTING: the baro backup first, then the accel (plan B `_detect_launch`).
    ///
    /// Both inputs are FRESH or nothing. The accel is the trigger that STARTS the flight, and with the baro
    /// gated an extrapolated accel would be the only input left: a false launch on the pad from a channel no
    /// sensor feeds. Plan B paid for one with the GC off and the stage timeline running on the ground; in Rust it
    /// also ends the link's reconnects (they key on the detected launch). A stale baro extrapolates past
    /// `launch_alt_m` the same way (the ICP-10111 read timeouts on the bench), and it must hold `launch_ms`
    /// through the same leaky dwell as the accel: one sample over the line was enough once, which a single spike
    /// or a glitched read could satisfy. And the GNSS elevation is no baro (see [`Source::Gnss`]).
    ///
    /// The baro sample counts for the dwell only when the elevation has also RISEN more than `launch_alt_m` within
    /// `launch_alt_window_ms` (the rate gate, not plan B's): a rocket gains it within a second, weather drift over
    /// hours, and a pad drifted past `launch_alt_m` launched TMS-7F unarmed overnight under plan B's rule alone. The
    /// rise is measured from a level the elevation HELD for `launch_ms`, as the climb must hold: one low or glitched
    /// read on a drifted pad would otherwise stand in for the pad for the whole window. So an elevation already over
    /// the line on the pad's first readings (no rise seen) is no launch either.
    fn detect_launch(&mut self, now_ms: u32, readings: &Readings) -> Option<Transition> {
        let elevation = readings
            .elevation
            .filter(|reading| reading.source != Source::Gnss)
            .map(|reading| reading.value);
        let magnitude_squared = readings
            .acceleration
            .map(|reading| commons::magnitude_squared(reading.value));
        let Settings {
            launch_alt_m: line_m,
            launch_alt_window_ms: window_ms,
            launch_ms,
            ..
        } = self.settings;
        let climbed_now = elevation.is_some_and(|metres| {
            let rise_m = self.climb.rise(now_ms, metres, window_ms, launch_ms);
            metres > line_m && rise_m > line_m
        });
        let (altitude_dwell, climbed) =
            commons::dwell_step(climbed_now, now_ms, self.altitude_dwell, self.settings.launch_ms);
        self.altitude_dwell = altitude_dwell;
        if climbed {
            let reason = Reason::LaunchAltitude {
                elevation_m: elevation.unwrap_or(0.0),
                dwell_ms: altitude_dwell.credit_ms,
            };
            return Some(self.advance(Stage::Boosting, reason));
        }
        let boosting_now = magnitude_squared.is_some_and(|squared| squared > self.launch_g_squared);
        let (launch_dwell, fired) =
            commons::dwell_step(boosting_now, now_ms, self.launch_dwell, self.settings.launch_ms);
        self.launch_dwell = launch_dwell;
        if !fired {
            return None;
        }
        let reason = Reason::LaunchAcceleration {
            magnitude_g: magnitude_squared.map_or(0.0, f32::sqrt),
            dwell_ms: launch_dwell.credit_ms,
        };
        Some(self.advance(Stage::Boosting, reason))
    }

    /// BOOSTING -> GLIDING: apogee, else the burnout timeout (plan B `_detect_apogee`).
    ///
    /// The apogee tracker sees the fresh elevation (the GNSS's included) only once armed, `apogee_arm_ms` past
    /// BOOSTING entry; an extrapolated sample would feed the PEAK, and a peak is latched. The dwell is
    /// `launch_ms`, plan B's reuse. The burnout timeout runs from BOOSTING entry regardless of arming.
    fn detect_apogee(&mut self, now_ms: u32, readings: &Readings) -> Option<Transition> {
        let armed = self
            .boost_entry_ms
            .is_some_and(|entry_ms| commons::reached(now_ms, entry_ms, self.settings.apogee_arm_ms));
        let elevation = readings.elevation.filter(|_| armed).map(|reading| reading.value);
        let (apogee, fired) = commons::apogee_step(
            elevation,
            now_ms,
            self.apogee,
            self.settings.apogee_drop_m,
            self.settings.launch_ms,
        );
        self.apogee = apogee;
        if fired && let Some(peak_m) = apogee.peak {
            return Some(self.advance(Stage::Gliding, Reason::Apogee { peak_m }));
        }
        if self.sustained(now_ms, self.settings.boost_timeout_ms) {
            return Some(self.advance(Stage::Gliding, Reason::BurnoutTimeout));
        }
        None
    }

    /// GLIDING -> LANDING: the height below `land_agl_m`, sustained `land_ms` (plan B `_detect_landing`).
    ///
    /// The fresh laser is primary; with no fresh laser the fresh elevation is the fallback. BOTH are
    /// source-gated: gating only the laser just moved where the invented number came from (a dead baro behind an
    /// out-of-range laser handed the landing trigger an extrapolated height). With neither fresh there is no
    /// height and the dwell resets: no reading is not a low reading.
    fn detect_landing(&mut self, now_ms: u32, readings: &Readings) -> Option<Transition> {
        let height = readings
            .height_above_ground
            .or(readings.elevation)
            .map(|reading| reading.value);
        if let Some(height_m) = height
            && height_m < self.settings.land_agl_m
        {
            if self.sustained(now_ms, self.settings.land_ms) {
                return Some(self.advance(Stage::Landing, Reason::HeightAboveGround { height_m }));
            }
            return None;
        }
        self.since_ms = None; // rose back, or lost the reading: a single low sample never flares
        None
    }

    /// LANDING -> DONE: the landing timeout, else stillness sustained `ground_ms` (plan B
    /// `_detect_stationary`).
    ///
    /// The accel is gated for the mirror reason of the launch: an extrapolated accel settles toward a plausible
    /// ~1 g, exactly the still band, so a dead accel would read as landed while still flying.
    fn detect_stationary(&mut self, now_ms: u32, readings: &Readings) -> Option<Transition> {
        if let Some(entry_ms) = self.landing_entry_ms
            && commons::reached(now_ms, entry_ms, self.settings.land_timeout_ms)
        {
            let reason = Reason::LandingTimeout {
                timeout_ms: self.settings.land_timeout_ms,
            };
            return Some(self.advance(Stage::Done, reason));
        }
        let magnitude_squared = readings
            .acceleration
            .map(|reading| commons::magnitude_squared(reading.value));
        if let Some(squared) = magnitude_squared
            && self.still_low_squared < squared
            && squared < self.still_high_squared
        {
            if self.sustained(now_ms, self.settings.ground_ms) {
                let reason = Reason::Stationary {
                    magnitude_g: squared.sqrt(),
                };
                return Some(self.advance(Stage::Done, reason));
            }
            return None;
        }
        self.since_ms = None;
        None
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{
        CLIMB_SLOTS, LONGEST_LAUNCH_ALTITUDE_WINDOW_MS, Reading, Readings, Reason, Settings, SettingsProblem, Source,
        Stage, StageMachine, Step, Transition,
    };

    /// test_sequencer.py's SPEC: small thresholds for fast cases.
    fn spec() -> Settings {
        Settings {
            launch_g: 3.0,
            boost_timeout_ms: 500,
            apogee_arm_ms: 50,
            land_ms: 100,
            ground_ms: 300,
            ..Settings::default()
        }
    }

    /// A fresh accel reading.
    fn acceleration(z: f32) -> Readings {
        Readings {
            acceleration: Some(Reading {
                value: [0.0, 0.0, z],
                source: Source::Other,
            }),
            ..Readings::default()
        }
    }

    /// A fresh elevation reading from `source`.
    fn elevation(metres: f32, source: Source) -> Readings {
        Readings {
            elevation: Some(Reading { value: metres, source }),
            ..Readings::default()
        }
    }

    /// The rows a step records, as `(stage, reason text)`.
    fn rows(step: Step) -> Vec<(Stage, String)> {
        step.transitions()
            .map(|transition| (transition.to, transition.reason.to_string()))
            .collect()
    }

    /// Step a machine through `ticks`, applying each advance as the caller does; returns the stage at the end.
    fn run(machine: &mut StageMachine, mut stage: Stage, ticks: &[(u32, Readings)]) -> Stage {
        for (now_ms, readings) in ticks {
            if let Some(advance) = machine.step(*now_ms, stage, readings, false).advance {
                stage = advance.to;
            }
        }
        stage
    }

    #[test]
    fn reasons_read_as_plan_b_writes_them() {
        let texts = [
            (Reason::External, "external"),
            (
                Reason::LaunchAltitude {
                    elevation_m: 15.0,
                    dwell_ms: 100,
                },
                "launch alt=15m dwell=100ms",
            ),
            (
                Reason::LaunchAltitude {
                    elevation_m: 12.5,
                    dwell_ms: 100,
                },
                "launch alt=12m dwell=100ms",
            ),
            (
                Reason::LaunchAcceleration {
                    magnitude_g: 8.0,
                    dwell_ms: 100,
                },
                "launch |a|=8.0g dwell=100ms",
            ),
            (Reason::Apogee { peak_m: 240.25 }, "apogee 240m"),
            (Reason::BurnoutTimeout, "burnout timeout (no separation)"),
            (Reason::HeightAboveGround { height_m: -1.5 }, "agl -1.5m"),
            (Reason::HeightAboveGround { height_m: -0.04 }, "agl -0.0m"),
            (Reason::LandingTimeout { timeout_ms: 2500 }, "landing timeout 2 s"),
            (Reason::Stationary { magnitude_g: 1.07355 }, "stationary 1.1g"),
            (Reason::FlightTimeout, "flight timeout"),
        ];
        for (reason, text) in texts {
            assert_eq!(reason.to_string(), text);
        }
    }

    #[test]
    fn only_the_external_reason_is_external() {
        let external = Transition {
            to: Stage::Gliding,
            reason: Reason::External,
        };
        let own = Transition {
            to: Stage::Gliding,
            reason: Reason::BurnoutTimeout,
        };
        assert!(external.external());
        assert!(!own.external());
    }

    #[test]
    fn the_first_step_records_the_stage_it_finds() {
        let mut machine = StageMachine::new(spec());
        let step = machine.step(0, Stage::Setting, &acceleration(1.0), false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(
            machine.step(50, Stage::Setting, &acceleration(1.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_sustained_boost_launches_and_is_not_recorded_twice() {
        let mut machine = StageMachine::new(spec());
        machine.step(1000, Stage::Setting, &acceleration(8.0), false);
        assert_eq!(
            machine.step(1050, Stage::Setting, &acceleration(8.0), false).advance,
            None
        );
        let step = machine.step(1100, Stage::Setting, &acceleration(8.0), false);
        assert_eq!(
            rows(step),
            [(Stage::Boosting, "launch |a|=8.0g dwell=100ms".to_string())]
        );
        assert_eq!(
            machine.step(1110, Stage::Boosting, &acceleration(8.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_stale_or_gnss_input_never_launches() {
        let mut machine = StageMachine::new(spec());
        let gnss = elevation(15.0, Source::Gnss);
        let nothing = Readings::default();
        let ticks: Vec<(u32, Readings)> = (0..20)
            .map(|tick| (tick * 50, if tick % 2 == 0 { gnss } else { nothing }))
            .collect();
        assert_eq!(run(&mut machine, Stage::Setting, &ticks), Stage::Setting);
        // the same 15 m from a barometer, risen from the pad it held, is a launch
        let (pad, baro) = (elevation(0.0, Source::Other), elevation(15.0, Source::Other));
        let ticks = [(1850, pad), (1900, pad), (1950, pad), (2000, baro), (2100, baro)];
        assert_eq!(run(&mut machine, Stage::Setting, &ticks), Stage::Boosting);
    }

    #[test]
    fn the_gnss_elevation_is_no_pad_to_climb_from() {
        // the baro drifted to 10.5 m; both baros go stale for 200 ms and the databoard falls through to the GNSS,
        // zeroed at its first fix and reading -1 m; then the baro is back at 10.5 m, which never rose from the GNSS
        let flown = |low: Source| {
            let ticks: Vec<(u32, Readings)> = (0..=3000)
                .step_by(50)
                .map(|now_ms| match now_ms {
                    1000..1200 => (now_ms, elevation(-1.0, low)),
                    _ => (now_ms, elevation(10.5, Source::Other)),
                })
                .collect();
            run(&mut gated(1000), Stage::Setting, &ticks)
        };
        assert_eq!(flown(Source::Gnss), Stage::Setting);
        // the same -1 m from a barometer is a pad the 10.5 m rose 11.5 m from
        assert_eq!(flown(Source::Other), Stage::Boosting);
    }

    #[test]
    fn an_operator_hold_suspends_everything() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, &acceleration(1.0), false);
        for now_ms in [50, 100, 500] {
            assert_eq!(
                machine.step(now_ms, Stage::Gliding, &acceleration(8.0), true),
                Step::default()
            );
        }
        // released: the held stage is seen now, as external
        let step = machine.step(550, Stage::Gliding, &Readings::default(), false);
        assert_eq!(rows(step), [(Stage::Gliding, "external".to_string())]);
    }

    #[test]
    fn a_long_hold_ends_in_two_rows() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 1000,
            ..spec()
        });
        let stage = run(
            &mut machine,
            Stage::Setting,
            &[
                (0, acceleration(8.0)),
                (50, acceleration(8.0)),
                (100, acceleration(8.0)),
            ],
        );
        assert_eq!(stage, Stage::Boosting);
        machine.step(150, Stage::Boosting, &acceleration(8.0), false);
        machine.step(200, Stage::Gliding, &acceleration(1.0), true);
        let step = machine.step(1550, Stage::Gliding, &acceleration(1.0), false);
        assert_eq!(
            rows(step),
            [
                (Stage::Gliding, "external".to_string()),
                (Stage::Done, "flight timeout".to_string())
            ]
        );
    }

    #[test]
    fn a_warm_start_rebases_the_backstop_on_its_first_step() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 2000,
            ..spec()
        });
        let high = elevation(80.0, Source::Other);
        assert_eq!(
            run(&mut machine, Stage::Gliding, &[(10_000, high), (11_999, high)]),
            Stage::Gliding
        );
        let step = machine.step(12_000, Stage::Gliding, &high, false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
    }

    #[test]
    fn setting_forgets_the_backstop() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 1000,
            ..spec()
        });
        machine.step(0, Stage::Gliding, &Readings::default(), false);
        machine.step(100, Stage::Setting, &acceleration(1.0), false);
        assert_eq!(
            machine.step(5000, Stage::Setting, &acceleration(1.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_stale_laser_never_lands_the_glider_at_altitude() {
        let mut machine = StageMachine::new(spec());
        // the laser is out of range (None), the baro says 274 m: no landing however long
        let apogee = elevation(274.0, Source::Other);
        assert_eq!(
            run(
                &mut machine,
                Stage::Gliding,
                &[(0, apogee), (500, apogee), (30_000, apogee)]
            ),
            Stage::Gliding
        );
        // a fresh laser low reading lands it, and wins over the baro
        let low = Readings {
            height_above_ground: Some(Reading {
                value: 2.0,
                source: Source::Other,
            }),
            ..apogee
        };
        machine.step(30_050, Stage::Gliding, &low, false);
        let step = machine.step(30_150, Stage::Gliding, &low, false);
        assert_eq!(rows(step), [(Stage::Landing, "agl 2.0m".to_string())]);
    }

    #[test]
    fn a_lost_height_resets_the_landing_dwell() {
        let mut machine = StageMachine::new(spec());
        let low = elevation(3.0, Source::Other);
        machine.step(0, Stage::Gliding, &low, false);
        machine.step(90, Stage::Gliding, &Readings::default(), false);
        assert_eq!(
            machine.step(100, Stage::Gliding, &low, false).advance,
            None,
            "the dwell restarted at 100"
        );
        assert!(machine.step(200, Stage::Gliding, &low, false).advance.is_some());
    }

    #[test]
    fn a_dead_accel_never_reads_as_still() {
        let mut machine = StageMachine::new(Settings {
            land_timeout_ms: 1000,
            ..spec()
        });
        let nothing = Readings::default();
        assert_eq!(
            run(
                &mut machine,
                Stage::Landing,
                &[(0, nothing), (500, nothing), (999, nothing)]
            ),
            Stage::Landing
        );
        let step = machine.step(1000, Stage::Landing, &nothing, false);
        assert_eq!(rows(step), [(Stage::Done, "landing timeout 1 s".to_string())]);
    }

    #[test]
    fn stillness_lands_and_the_band_is_open() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Landing, &acceleration(0.69), false);
        machine.step(100, Stage::Landing, &acceleration(1.31), false);
        machine.step(200, Stage::Landing, &acceleration(0.71), false);
        assert_eq!(
            machine.step(499, Stage::Landing, &acceleration(1.29), false).advance,
            None
        );
        let step = machine.step(500, Stage::Landing, &acceleration(1.0), false);
        assert_eq!(rows(step), [(Stage::Done, "stationary 1.0g".to_string())]);
    }

    /// A fresh laser reading.
    fn laser(metres: f32) -> Readings {
        Readings {
            height_above_ground: Some(Reading {
                value: metres,
                source: Source::Other,
            }),
            ..Readings::default()
        }
    }

    /// Step `stage` with the same `readings` every 50 ms from `from_ms` through `to_ms`, without applying an
    /// advance (the cases end on their advance), and return the last advance.
    fn repeat(
        machine: &mut StageMachine,
        stage: Stage,
        readings: Readings,
        from_ms: u32,
        to_ms: u32,
    ) -> Option<Transition> {
        (from_ms..=to_ms)
            .step_by(50)
            .filter_map(|now_ms| machine.step(now_ms, stage, &readings, false).advance)
            .last()
    }

    /// A machine on `spec()` that has read a pad of `pad_m`, from 0 through 100 ms: a level held `launch_ms`.
    fn on_a_pad(pad_m: f32) -> StageMachine {
        let mut machine = StageMachine::new(spec());
        assert_eq!(
            repeat(&mut machine, Stage::Setting, elevation(pad_m, Source::Other), 0, 100),
            None
        );
        machine
    }

    #[test]
    fn the_launch_lines_are_strict() {
        // exactly launch_g (3.0 g) and exactly launch_alt_m (10.0 m) up from a 0 m pad, held: no launch; f32 holds
        // both exactly
        let mut machine = on_a_pad(0.0);
        assert_eq!(repeat(&mut machine, Stage::Setting, acceleration(3.0), 150, 450), None);
        assert_eq!(
            repeat(&mut machine, Stage::Setting, elevation(10.0, Source::Other), 500, 800),
            None
        );
        // clearly over either line, it launches
        let launch = repeat(&mut machine, Stage::Setting, acceleration(3.5), 850, 950);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch |a|=3.5g dwell=100ms".to_string())
        );
        // from a pad of pad_m to metres, held
        let climb = |pad_m: f32, metres: f32| {
            let high = elevation(metres, Source::Other);
            repeat(&mut on_a_pad(pad_m), Stage::Setting, high, 150, 1050).map(|step| step.to)
        };
        assert_eq!(climb(0.0, 10.5), Some(Stage::Boosting));
        // over the line, but risen exactly launch_alt_m (10.5 up from a 0.5 m pad): the rise is strict too
        assert_eq!(climb(0.5, 10.5), None);
        // risen 11 m from a -1 m pad, but exactly on launch_alt_m: plan B's line stays strict under the gate
        assert_eq!(climb(-1.0, 10.0), None);
    }

    #[test]
    fn the_landing_height_is_strict() {
        let mut machine = StageMachine::new(spec());
        assert_eq!(
            repeat(&mut machine, Stage::Gliding, elevation(5.0, Source::Other), 0, 200),
            None
        );
        assert_eq!(repeat(&mut machine, Stage::Gliding, laser(5.0), 250, 450), None);
        let landing = repeat(&mut machine, Stage::Gliding, laser(4.5), 500, 600);
        assert_eq!(
            landing.map(|step| step.reason.to_string()),
            Some("agl 4.5m".to_string())
        );
    }

    #[test]
    fn the_still_band_is_open_at_its_exact_edges() {
        // still_g 0.5: the band is (0.25, 2.25) in g^2, and 0.5 g and 1.5 g sit exactly on its edges
        let settings = Settings {
            still_g: 0.5,
            land_timeout_ms: 3000,
            ..spec()
        };
        let mut machine = StageMachine::new(settings);
        assert_eq!(repeat(&mut machine, Stage::Landing, acceleration(0.5), 0, 400), None);
        assert_eq!(repeat(&mut machine, Stage::Landing, acceleration(1.5), 450, 800), None);
        let done = repeat(&mut machine, Stage::Landing, acceleration(1.0), 850, 1150);
        assert_eq!(
            done.map(|step| step.reason.to_string()),
            Some("stationary 1.0g".to_string())
        );
        // a tolerance of 1 g or more has no lower edge: near free fall reads as still
        let mut machine = StageMachine::new(Settings { still_g: 1.5, ..spec() });
        assert!(repeat(&mut machine, Stage::Landing, acceleration(0.2), 0, 300).is_some());
    }

    #[test]
    fn the_apogee_detector_arms_exactly_apogee_arm_ms_after_entry() {
        let mut machine = StageMachine::new(Settings {
            apogee_arm_ms: 100,
            boost_timeout_ms: 5000,
            ..spec()
        });
        let ticks = [
            (0, Readings::default()),
            (99, elevation(300.0, Source::Other)), // a millisecond early: ignored, or the peak would be 300 m
            (100, elevation(210.0, Source::Other)), // armed on the dot: seeds the peak
            (150, elevation(200.0, Source::Other)),
            (200, elevation(200.0, Source::Other)),
            (250, elevation(170.0, Source::Other)),
            (300, elevation(170.0, Source::Other)),
        ];
        assert_eq!(run(&mut machine, Stage::Boosting, &ticks), Stage::Boosting);
        let step = machine.step(350, Stage::Boosting, &elevation(170.0, Source::Other), false);
        assert_eq!(rows(step), [(Stage::Gliding, "apogee 210m".to_string())]);
    }

    #[test]
    fn entering_setting_restarts_the_launch_dwells() {
        let (still, boost) = (acceleration(1.0), acceleration(8.0));
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, &still, false);
        machine.step(50, Stage::Setting, &boost, false);
        machine.step(100, Stage::Setting, &boost, false);
        machine.step(110, Stage::Gliding, &boost, false);
        // back on the pad 30 ms later: a kept half credit would only drain to 20 and launch at 230; restarted, the
        // dwell needs launch_ms afresh
        let step = machine.step(130, Stage::Setting, &still, false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(machine.step(180, Stage::Setting, &boost, false).advance, None);
        assert_eq!(machine.step(230, Stage::Setting, &boost, false).advance, None);
        assert!(machine.step(280, Stage::Setting, &boost, false).advance.is_some());
        // the altitude dwell: the pad must hold launch_ms afresh first, which drains any credit kept, so only a clock
        // stepping back shows one. It feeds an inactive dwell: a kept half credit would complete on the pad's reading
        let (pad, high) = (elevation(0.0, Source::Other), elevation(15.0, Source::Other));
        let mut machine = on_a_pad(0.0);
        machine.step(150, Stage::Setting, &high, false);
        machine.step(200, Stage::Setting, &high, false);
        machine.step(210, Stage::Gliding, &high, false);
        let step = machine.step(110, Stage::Setting, &pad, false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        // restarted, the pad holds from 110 to 210, and launch_ms of the climb after it launches
        assert_eq!(repeat(&mut machine, Stage::Setting, pad, 160, 210), None);
        let launch = repeat(&mut machine, Stage::Setting, high, 260, 360);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch alt=15m dwell=100ms".to_string())
        );
    }

    #[test]
    fn entering_boosting_restarts_the_apogee_tracker() {
        let mut machine = StageMachine::new(Settings {
            apogee_arm_ms: 0,
            boost_timeout_ms: 5000,
            ..spec()
        });
        let ticks = [
            (0, elevation(240.0, Source::Other)),
            (50, elevation(240.0, Source::Other)),
        ];
        assert_eq!(run(&mut machine, Stage::Boosting, &ticks), Stage::Boosting);
        machine.step(100, Stage::Setting, &elevation(0.5, Source::Other), false);
        // a second flight at 100 m: a kept 240 m peak would call apogee within launch_ms
        let low = elevation(100.0, Source::Other);
        assert_eq!(repeat(&mut machine, Stage::Boosting, low, 200, 400), None);
    }

    #[test]
    fn a_warm_landing_rebases_the_backstop() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 500,
            land_timeout_ms: 1000,
            ..spec()
        });
        let carried = acceleration(1.6);
        assert_eq!(
            run(&mut machine, Stage::Landing, &[(0, carried), (499, carried)]),
            Stage::Landing
        );
        let step = machine.step(500, Stage::Landing, &carried, false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
    }

    #[test]
    fn a_hold_ending_where_the_machine_looked_is_no_change() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, &acceleration(8.0), false);
        machine.step(50, Stage::Setting, &acceleration(8.0), false);
        machine.step(60, Stage::Gliding, &acceleration(8.0), true);
        // `stage setting` ends the hold on the stage last seen: no external row, and the kept half credit plus
        // the 4950 ms gap launch on this one sample
        let step = machine.step(5000, Stage::Setting, &acceleration(8.0), false);
        assert_eq!(
            rows(step),
            [(Stage::Boosting, "launch |a|=8.0g dwell=100ms".to_string())]
        );
    }

    #[test]
    fn the_launch_reasons_plan_b_prints() {
        // both dwells complete on one tick, the elevation risen from the pad: the baro backup is checked first
        let both = Readings {
            elevation: Some(Reading {
                value: 15.0,
                source: Source::Other,
            }),
            ..acceleration(8.0)
        };
        let launch = repeat(&mut on_a_pad(0.0), Stage::Setting, both, 150, 250);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch alt=15m dwell=100ms".to_string())
        );
        // a clock stepping back onto a sample with no reading feeds the dwell to full: plan B prints 0
        for (pad, readings, text) in [
            (acceleration(1.0), acceleration(8.0), "launch |a|=0.0g dwell=100ms"),
            (
                elevation(0.0, Source::Other),
                elevation(15.0, Source::Other),
                "launch alt=0m dwell=100ms",
            ),
        ] {
            let mut machine = StageMachine::new(spec());
            assert_eq!(repeat(&mut machine, Stage::Setting, pad, 850, 950), None);
            machine.step(1000, Stage::Setting, &readings, false);
            machine.step(1080, Stage::Setting, &readings, false);
            let step = machine.step(1060, Stage::Setting, &Readings::default(), false);
            assert_eq!(rows(step), [(Stage::Boosting, text.to_string())]);
        }
    }

    #[test]
    fn done_has_no_detector_and_no_backstop() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 100,
            ..spec()
        });
        machine.step(0, Stage::Boosting, &Readings::default(), false);
        machine.step(10, Stage::Done, &acceleration(1.0), false);
        assert_eq!(
            machine.step(10_000, Stage::Done, &acceleration(8.0), false),
            Step::default()
        );
    }

    #[test]
    fn the_clock_may_wrap_mid_flight() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 1000,
            ..spec()
        });
        let start = u32::MAX - 400;
        machine.step(start, Stage::Gliding, &Readings::default(), false);
        assert_eq!(
            machine.step(start.wrapping_add(999), Stage::Gliding, &Readings::default(), false),
            Step::default()
        );
        let step = machine.step(start.wrapping_add(1000), Stage::Gliding, &Readings::default(), false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
    }

    #[test]
    fn the_defaults_are_plan_b_setup_defaults() {
        let settings = Settings::default();
        assert_eq!(settings.period_ms, 50);
        assert_eq!(
            (settings.launch_g, settings.launch_ms, settings.launch_alt_m),
            (2.5, 100, 10.0)
        );
        assert_eq!(
            (
                settings.boost_timeout_ms,
                settings.apogee_drop_m,
                settings.apogee_arm_ms
            ),
            (6000, 5.0, 4000)
        );
        assert_eq!(
            (settings.flight_timeout_ms, settings.land_agl_m, settings.land_ms),
            (300_000, 5.0, 300)
        );
        assert_eq!(
            (settings.still_g, settings.ground_ms, settings.land_timeout_ms),
            (0.3, 3000, 10_000)
        );
        // the rate gate is not plan B's: the operator's 10 m within 1 s
        assert_eq!(settings.launch_alt_window_ms, 1000);
        assert_eq!(StageMachine::new(settings).settings(), &settings);
    }

    /// Every key [`Settings::from_config`] reads.
    const KEYS: [&str; 14] = [
        "period_ms",
        "launch_g",
        "launch_ms",
        "launch_alt_m",
        "launch_alt_window_ms",
        "boost_timeout_ms",
        "apogee_drop_m",
        "apogee_arm_ms",
        "flight_timeout_ms",
        "land_agl_m",
        "land_ms",
        "still_g",
        "ground_ms",
        "land_timeout_ms",
    ];

    #[test]
    fn an_empty_block_takes_plan_b_setup_defaults() {
        assert_eq!(Settings::from_config(&json!({})), Ok(Settings::default()));
        // the entry's own keys and the GC switch (not ported) are not settings, whatever they hold
        let entry = json!({"name": "sequencer", "activity": "sequencer", "enabled": true, "disable_gc_flight": "on"});
        assert_eq!(Settings::from_config(&entry), Ok(Settings::default()));
    }

    #[test]
    fn the_shipped_block_reads_as_written() {
        // src/glider config_default's sequencer entry
        let entry = json!({
            "name": "sequencer", "activity": "sequencer", "enabled": true, "period_ms": 50,
            "launch_g": 2.5, "launch_ms": 100, "launch_alt_m": 10.0,
            "apogee_drop_m": 5.0, "apogee_arm_ms": 4000, "boost_timeout_ms": 12000,
            "land_agl_m": 5.0, "land_ms": 300, "still_g": 0.3, "ground_ms": 3000,
            "land_timeout_ms": 10000, "flight_timeout_ms": 300000, "disable_gc_flight": true,
        });
        let expected = Settings {
            boost_timeout_ms: 12_000,
            ..Settings::default()
        };
        assert_eq!(Settings::from_config(&entry), Ok(expected));
        // each key lands in its own field
        for (index, key) in KEYS.into_iter().enumerate() {
            let mut config = json!({});
            config[key] = json!(index + 1);
            let settings = Settings::from_config(&config).unwrap();
            assert_ne!(settings, Settings::default(), "{key} was read");
        }
    }

    #[test]
    fn a_whole_number_of_ms_may_be_written_as_a_float() {
        for (value, expected) in [
            (json!(300.0), 300),
            (json!(300), 300),
            (json!(0), 0),
            (json!(-0.0), 0),
            (json!(1e3), 1000),
            (json!(2_147_483_647), 2_147_483_647),
            (json!(2_147_483_647.0), 2_147_483_647),
        ] {
            let settings = Settings::from_config(&json!({ "land_ms": value }));
            assert_eq!(settings.map(|settings| settings.land_ms), Ok(expected), "{value}");
        }
        // a g or metre key takes any number, an integer included
        let settings = Settings::from_config(&json!({"launch_g": 3, "land_agl_m": -2.5, "still_g": 1e-3})).unwrap();
        assert_eq!(
            (settings.launch_g, settings.land_agl_m, settings.still_g),
            (3.0, -2.5, 0.001)
        );
    }

    #[test]
    fn the_rate_gate_window_reads_as_the_other_ms_keys_up_to_its_history() {
        for (value, expected) in [
            (json!(500), 500),
            (json!(250.0), 250),
            (json!(0), 0),
            (json!(-0.0), 0),
            (json!(2000), 2000),
            (json!(2e3), 2000),
        ] {
            let settings = Settings::from_config(&json!({ "launch_alt_window_ms": value }));
            assert_eq!(
                settings.map(|settings| settings.launch_alt_window_ms),
                Ok(expected),
                "{value}"
            );
        }
        let cases = [
            (json!(2001), "2001 is past the longest, 2000 ms"),
            (json!(2000.5), "2000.5 is not a whole number of ms"),
            (json!(2001.0), "2001.0 is past the longest, 2000 ms"),
            (json!(3_600_000), "3600000 is past the longest, 2000 ms"),
            (json!(-1), "-1 is negative"),
            (json!(500.5), "500.5 is not a whole number of ms"),
            (json!("1000"), "\"1000\" is not a number"),
            (json!(true), "true is not a number"),
            (json!(null), "null is not a number"),
        ];
        for (value, why) in cases {
            let error = Settings::from_config(&json!({ "launch_alt_window_ms": value })).unwrap_err();
            assert_eq!(error.key, "launch_alt_window_ms");
            assert_eq!(error.to_string(), format!("launch_alt_window_ms: {why}"));
        }
        assert_eq!(
            Settings::from_config(&json!({"launch_alt_window_ms": 2001}))
                .unwrap_err()
                .problem,
            SettingsProblem::TooLong { longest_ms: 2000 }
        );
    }

    #[test]
    fn the_longest_ms_it_takes_is_one_the_machine_reaches() {
        let settings = Settings::from_config(&json!({"flight_timeout_ms": 2_147_483_647})).unwrap();
        let mut machine = StageMachine::new(settings);
        machine.step(0, Stage::Gliding, &Readings::default(), false);
        assert_eq!(
            machine.step(2_147_483_646, Stage::Gliding, &Readings::default(), false),
            Step::default()
        );
        let step = machine.step(2_147_483_647, Stage::Gliding, &Readings::default(), false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
        // one ms longer, which it refuses: the signed elapsed time turns negative first, so the backstop never fires
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 2_147_483_648,
            ..settings
        });
        machine.step(0, Stage::Gliding, &Readings::default(), false);
        for now_ms in [2_147_483_647, 2_147_483_648, u32::MAX] {
            assert_eq!(
                machine.step(now_ms, Stage::Gliding, &Readings::default(), false),
                Step::default(),
                "{now_ms}"
            );
        }
    }

    #[test]
    fn a_value_it_cannot_run_as_written_is_refused_naming_the_key() {
        let cases = [
            (json!({"launch_g": "3"}), "launch_g: \"3\" is not a number"),
            (json!({"still_g": null}), "still_g: null is not a number"),
            (json!({"launch_alt_m": true}), "launch_alt_m: true is not a number"),
            (json!({"land_ms": "300"}), "land_ms: \"300\" is not a number"),
            (json!({"land_ms": -5}), "land_ms: -5 is negative"),
            (json!({"land_ms": -0.5}), "land_ms: -0.5 is negative"),
            (json!({"land_ms": 300.5}), "land_ms: 300.5 is not a whole number of ms"),
            (json!({"land_ms": true}), "land_ms: true is not a number"),
            (json!({"land_ms": null}), "land_ms: null is not a number"),
            (json!({"land_ms": [300]}), "land_ms: [300] is not a number"),
            (
                json!({"flight_timeout_ms": 2_147_483_648_u64}),
                "flight_timeout_ms: 2147483648 is past the longest, 2147483647 ms",
            ),
            (
                json!({"land_timeout_ms": 2_147_483_648.0}),
                "land_timeout_ms: 2147483648.0 is past the longest, 2147483647 ms",
            ),
            (
                json!({"apogee_arm_ms": 4_294_967_295_u64}),
                "apogee_arm_ms: 4294967295 is past the longest, 2147483647 ms",
            ),
            (
                json!({"period_ms": 4_294_967_296_u64}),
                "period_ms: 4294967296 is past the longest, 2147483647 ms",
            ),
            (
                json!({"flight_timeout_ms": 1e10}),
                "flight_timeout_ms: 10000000000.0 is past the longest, 2147483647 ms",
            ),
        ];
        for (config, text) in cases {
            let error = Settings::from_config(&config).unwrap_err();
            assert_eq!(error.to_string(), text);
        }
        // every key is checked, and the refusal names it
        for key in KEYS {
            let mut config = json!({"launch_g": 3.0, "land_ms": 300});
            config[key] = json!({"value": 1});
            let error = Settings::from_config(&config).unwrap_err();
            assert_eq!((error.key, error.problem), (key, SettingsProblem::NotANumber));
        }
    }

    /// The shipped thresholds (`launch_alt_m` 10, `launch_ms` 100) with this rate gate window.
    fn gated(window_ms: u32) -> StageMachine {
        StageMachine::new(Settings {
            launch_alt_window_ms: window_ms,
            ..Settings::default()
        })
    }

    /// Step SETTING every `period_ms` from `from_ms` through `to_ms` on a fresh barometer reading of
    /// `elevation_m(now_ms)`, and return the first launch: its clock and its reason, None when there is none.
    fn first_launch(
        machine: &mut StageMachine,
        from_ms: u32,
        to_ms: u32,
        period_ms: usize,
        elevation_m: impl Fn(u32) -> f32,
    ) -> Option<(u32, String)> {
        (from_ms..=to_ms).step_by(period_ms).find_map(|now_ms| {
            let readings = elevation(elevation_m(now_ms), Source::Other);
            let advance = machine.step(now_ms, Stage::Setting, &readings, false).advance;
            advance.map(|advance| (now_ms, advance.reason.to_string()))
        })
    }

    #[test]
    fn ten_metres_risen_in_half_a_second_launch() {
        // a second on the pad, then 0 -> 12 m in 500 ms (24 m/s), held: over both lines from 1450 (10.8 m), and
        // launch_ms later it launches
        let climb = |now_ms: u32| (now_ms.saturating_sub(1000) as f32 * 0.024).min(12.0);
        assert_eq!(
            first_launch(&mut gated(1000), 0, 5000, 50, climb),
            Some((1550, "launch alt=12m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn ten_metres_risen_in_one_and_a_half_seconds_do_not() {
        // 0 -> 12 m in 1.5 s (8 m/s), held: over the line from 2300 on, which plan B's rule alone launches on, but
        // never 10 m above the lowest level it held within the last second
        let climb = |now_ms: u32| (now_ms.saturating_sub(1010) as f32 * 0.008).min(12.0);
        assert_eq!(first_launch(&mut gated(1000), 0, 10_000, 50, climb), None);
        // a 2 s window takes the same climb: 10.32 m up at 2300, all of it within the window
        assert_eq!(
            first_launch(&mut gated(2000), 0, 10_000, 50, climb),
            Some((2400, "launch alt=11m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn two_hours_of_drift_never_launch() {
        // TMS-7F's night, a little steeper: the pad's elevation creeps +12 m in 2 h (it launched on +10 m in
        // 2.17 h), with 0.3 m of sawtooth noise, read every 50 ms
        let drift = |now_ms: u32| now_ms as f32 * 12.0 / 7_200_000.0 + (now_ms / 50 % 7) as f32 * 0.05;
        assert_eq!(first_launch(&mut gated(1000), 0, 7_200_000, 50, drift), None);
        assert!(drift(6_100_000) > 10.0, "over the line for the last 18 minutes");
    }

    #[test]
    fn a_step_launches_and_a_spike_shorter_than_launch_ms_does_not() {
        // two 15 m readings 50 ms apart, twice: over both lines, but never for launch_ms
        let spikes = |now_ms: u32| {
            if matches!(now_ms, 1000 | 1050 | 2000 | 2050) {
                15.0
            } else {
                0.0
            }
        };
        assert_eq!(first_launch(&mut gated(1000), 0, 5000, 50, spikes), None);
        // the same step held, as the bench makes a launch (`update` lowers the ground 15 m in SETTING)
        let step = |now_ms: u32| if now_ms >= 1000 { 15.0 } else { 0.0 };
        assert_eq!(
            first_launch(&mut gated(1000), 0, 5000, 50, step),
            Some((1100, "launch alt=15m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn the_rise_must_still_end_over_the_line() {
        // a pad re-zeroed 5 m high (it reads -5 m), then 8 m: risen 13 m, but not launch_alt_m above the ground
        let shape = |now_ms: u32| if now_ms < 1000 { -5.0 } else { 8.0 };
        assert_eq!(first_launch(&mut gated(1000), 0, 3000, 50, shape), None);
        // on to 12 m within the window, it launches
        let higher = |now_ms: u32| if now_ms < 1500 { shape(now_ms) } else { 12.0 };
        assert_eq!(
            first_launch(&mut gated(1000), 0, 3000, 50, higher),
            Some((1600, "launch alt=12m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn a_low_read_on_a_drifted_pad_is_no_pad_to_climb_from() {
        // TMS-7F's bench at dawn: the pad drifted to 10.5 m, over launch_alt_m. One glitched read of 0 m would stand
        // for the pad through the whole window, 10.5 m below; a level is one held launch_ms, as a climb is
        let dip = |last_ms: u32| move |now_ms: u32| if (5000..=last_ms).contains(&now_ms) { 0.0 } else { 10.5 };
        assert_eq!(first_launch(&mut gated(1000), 0, 10_000, 50, dip(5000)), None);
        // two low reads in a row neither; three, 100 ms of 0 m, are a level the 10.5 m rose from
        assert_eq!(first_launch(&mut gated(1000), 0, 10_000, 50, dip(5050)), None);
        assert_eq!(
            first_launch(&mut gated(1000), 0, 10_000, 50, dip(5100)),
            Some((5250, "launch alt=10m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn the_window_ends_exactly_launch_alt_window_ms_back() {
        // the pad's 0 m from 0 to 100, a level held launch_ms at 100; then 5 m, then 11 m for 100 ms: risen 11 m above
        // the pad's level only. On time, the dwell completes with that level confirmed exactly 1000 ms before, still
        // in the window; a millisecond later it is out
        for (late_ms, launched) in [(0, true), (1, false)] {
            let ticks: Vec<(u32, Readings)> = (0..=1100)
                .step_by(50)
                .map(|now_ms| match now_ms {
                    0..=100 => (now_ms, elevation(0.0, Source::Other)),
                    150..1000 => (now_ms, elevation(5.0, Source::Other)),
                    _ => (now_ms + late_ms, elevation(11.0, Source::Other)),
                })
                .collect();
            let stage = run(&mut gated(1000), Stage::Setting, &ticks);
            assert_eq!(stage == Stage::Boosting, launched, "the climb {late_ms} ms late");
        }
    }

    #[test]
    fn a_window_shorter_than_launch_ms_never_launches_by_altitude() {
        // 11 m more on every reading, 50 ms apart: a 100 ms window still holds the level confirmed launch_ms (100)
        // after the reading it looks back to; a shorter one has dropped that reading first
        let climb = |now_ms: u32| now_ms.saturating_sub(950) as f32 * 0.22;
        for (window_ms, launch_ms) in [(100, Some(1100)), (99, None), (0, None)] {
            let launch = first_launch(&mut gated(window_ms), 0, 3000, 50, climb);
            assert_eq!(launch.map(|(now_ms, _)| now_ms), launch_ms, "{window_ms} ms");
        }
        // the accel launch is not gated
        let launch = repeat(&mut gated(0), Stage::Setting, acceleration(8.0), 0, 100);
        assert_eq!(launch.map(|launch| launch.to), Some(Stage::Boosting));
    }

    #[test]
    fn the_longest_window_fits_the_history() {
        // a reading every 25 ms, a slot each: the pad's 0 m from 0 to 100, its level confirmed at 100; 5 m, then 11 m
        // from 2000. The dwell completes at 2100 with that level exactly 2000 ms old, in the oldest of 81 slots: the
        // history is full
        assert_eq!(CLIMB_SLOTS, 81);
        let shape = |now_ms: u32| match now_ms {
            0..=100 => 0.0,
            101..2000 => 5.0,
            _ => 11.0,
        };
        let mut machine = gated(LONGEST_LAUNCH_ALTITUDE_WINDOW_MS);
        assert_eq!(
            first_launch(&mut machine, 0, 3000, 25, shape),
            Some((2100, "launch alt=11m dwell=100ms".to_string()))
        );
        assert_eq!(machine.climb.count, CLIMB_SLOTS);
    }

    #[test]
    fn a_reading_one_slot_after_the_newest_opens_its_own() {
        // a reading every 25 ms, exactly a slot apart: the pad's 0 m from 0 to 125, then 5 m, then 11 m from 1025. The
        // level confirmed at 125 has a slot of its own, dated 125, so it counts until exactly 1125, when the dwell
        // completes; sharing the slot begun at 100, it would be gone 25 ms before
        let shape = |now_ms: u32| match now_ms {
            0..=125 => 0.0,
            126..1025 => 5.0,
            _ => 11.0,
        };
        assert_eq!(
            first_launch(&mut gated(1000), 0, 3000, 25, shape),
            Some((1125, "launch alt=11m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn steps_just_under_a_slot_apart_share_one_and_fit_the_longest_window() {
        // a reading every 24 ms, two to a slot: 42 slots span the 2000 ms window, where one each would take 84, past
        // the 81 the history holds. The pad's 0 m from 0 to 120, its level confirmed at 120 in the slot begun at 96;
        // 5 m, then 11 m from 1968: the dwell completes at 2088, the pad's slot 1992 ms old and still counted
        let shape = |now_ms: u32| match now_ms {
            0..=120 => 0.0,
            121..1968 => 5.0,
            _ => 11.0,
        };
        let mut machine = gated(LONGEST_LAUNCH_ALTITUDE_WINDOW_MS);
        assert_eq!(
            first_launch(&mut machine, 0, 3000, 24, shape),
            Some((2088, "launch alt=11m dwell=100ms".to_string()))
        );
        assert_eq!(machine.climb.count, 42);
    }

    #[test]
    fn a_window_past_the_longest_acts_as_the_longest() {
        // only a hand-built Settings carries one. 0 -> 12 m at 4 m/s, read every 50 ms: never 10 m up within 2 s,
        // though it is within 10 s
        let climb = |now_ms: u32| (now_ms.saturating_sub(1010) as f32 * 0.004).min(12.0);
        assert_eq!(first_launch(&mut gated(10_000), 0, 20_000, 50, climb), None);
    }

    #[test]
    fn a_reading_the_clock_no_longer_compares_with_is_forgotten() {
        // a second on the pad at 0 m, then no fresh baro for 2^31 ms (24.8 days): the signed elapsed time reads the
        // pad's readings as being from the future, so they are dropped, and 11 m is no rise
        let resumed_ms = 1000 + (1_u32 << 31);
        let mut ticks: Vec<(u32, Readings)> = (0..=1000)
            .step_by(50)
            .map(|now_ms| (now_ms, elevation(0.0, Source::Other)))
            .collect();
        ticks.extend((0..20).map(|tick| (resumed_ms + tick * 50, elevation(11.0, Source::Other))));
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Setting);
        // the same climb across the clock's wrap, read every 50 ms, launches
        let start = u32::MAX - 1000;
        let ticks: Vec<(u32, Readings)> = (0..40)
            .map(|tick: u32| {
                let metres = if tick < 25 { 0.0 } else { 11.0 };
                (start.wrapping_add(tick * 50), elevation(metres, Source::Other))
            })
            .collect();
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Boosting);
    }

    #[test]
    fn a_level_confirmed_before_the_clock_stepped_back_is_forgotten() {
        // the pad's 0 m from 1000 to 1500, then 11 m: launch_ms into the climb, it launches
        let pad = |now_ms: u32| (now_ms, elevation(0.0, Source::Other));
        let high = |now_ms: u32| (now_ms, elevation(11.0, Source::Other));
        let mut ticks: Vec<(u32, Readings)> = (1000..=1500).step_by(50).map(pad).collect();
        ticks.extend((1550..=2000).step_by(50).map(high));
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Boosting);
        // the same 11 m once the clock has stepped back to 600: the pad's level reads as confirmed in the future, and
        // no climb is measured from it
        let mut ticks: Vec<(u32, Readings)> = (1000..=1500).step_by(50).map(pad).collect();
        ticks.extend((600..=2000).step_by(50).map(high));
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Setting);
    }

    #[test]
    fn faster_steps_share_a_slot_dated_by_its_first_reading() {
        // a reading every 10 ms, three to a slot: the pad's 0 m to pad_end_ms, then 5 m, then 11 m from climb_ms
        let launch = |pad_end_ms: u32, climb_ms: u32| {
            let shape = |now_ms: u32| match now_ms {
                _ if now_ms <= pad_end_ms => 0.0,
                _ if now_ms < climb_ms => 5.0,
                _ => 11.0,
            };
            first_launch(&mut gated(1000), 0, 3000, 10, shape).map(|(now_ms, _)| now_ms)
        };
        // the pad's level, last confirmed at 200 in the slot begun at 180, counts until 1180: a climb from 1080
        // launches then
        assert_eq!(launch(200, 1080), Some(1180));
        // 10 ms later the slot is gone at 1190, though the reading that confirmed the level is 990 ms old: a shared
        // slot shortens the window at its far end, and never lengthens it
        assert_eq!(launch(200, 1090), None);
        // the pad ending on the slot's first reading, its 5 m reads after confirm 5 m: the slot keeps the lowest level
        assert_eq!(launch(180, 1080), Some(1180));
    }

    #[test]
    fn a_level_counts_once_held_exactly_launch_ms() {
        // the drifted pad's 10.5 m, then 0 m read at 5000 and again held_ms later, nothing between (a gap holds the
        // last reading), then 10.5 m: held exactly launch_ms, the 0 m is a level the 10.5 m rose from; a millisecond
        // short, it is not
        for (held_ms, launched) in [(100, true), (99, false)] {
            let mut ticks: Vec<(u32, Readings)> = (0..5000)
                .step_by(50)
                .map(|now_ms| (now_ms, elevation(10.5, Source::Other)))
                .collect();
            ticks.extend([5000, 5000 + held_ms].map(|now_ms| (now_ms, elevation(0.0, Source::Other))));
            ticks.extend(
                (5150..=6000)
                    .step_by(50)
                    .map(|now_ms| (now_ms, elevation(10.5, Source::Other))),
            );
            let stage = run(&mut gated(1000), Stage::Setting, &ticks);
            assert_eq!(stage == Stage::Boosting, launched, "held {held_ms} ms");
        }
    }

    #[test]
    fn a_shared_slot_keeps_its_highest_read_for_the_holds_after_it() {
        // a reading every 10 ms, three to a slot, the pad at 0 m but for one 6 m read every 90 ms, each the middle
        // reading of its slot: the elevation never holds 0 m for launch_ms, so 11 m is only 5 m above a held level
        let pad = |spikes: bool| {
            move |now_ms: u32| match now_ms {
                1000.. => 11.0,
                _ if spikes && now_ms % 90 == 10 => 6.0,
                _ => 0.0,
            }
        };
        assert_eq!(first_launch(&mut gated(1000), 0, 3000, 10, pad(true)), None);
        // without the 6 m reads, the same climb launches
        assert_eq!(
            first_launch(&mut gated(1000), 0, 3000, 10, pad(false)),
            Some((1100, "launch alt=11m dwell=100ms".to_string()))
        );
    }

    #[test]
    fn a_return_to_setting_measures_the_pad_afresh() {
        let high = elevation(15.0, Source::Other);
        let mut machine = on_a_pad(0.0);
        machine.step(150, Stage::Setting, &high, false);
        machine.step(160, Stage::Gliding, &high, false);
        // back on the pad 10 ms later, still 15 m up: the pad's level before the excursion is forgotten, so no rise is
        // seen
        let step = machine.step(170, Stage::Setting, &high, false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(repeat(&mut machine, Stage::Setting, high, 220, 2000), None);
    }

    #[test]
    fn a_hold_records_no_level() {
        // the pad at 5 m; an operator hold (`stage gliding`, a fin check) reads 0 m; `stage setting` ends it where the
        // machine last looked, so nothing restarts, and 11 m is risen only 6 m above the level the machine saw
        let (low, high) = (elevation(0.0, Source::Other), elevation(11.0, Source::Other));
        let mut machine = on_a_pad(5.0);
        for now_ms in (550..=800).step_by(50) {
            assert_eq!(machine.step(now_ms, Stage::Gliding, &low, true), Step::default());
        }
        assert_eq!(repeat(&mut machine, Stage::Setting, high, 850, 2000), None);
        // the same 0 m read on the pad is a level the 11 m rose from
        let mut machine = on_a_pad(5.0);
        assert_eq!(repeat(&mut machine, Stage::Setting, low, 550, 800), None);
        assert!(repeat(&mut machine, Stage::Setting, high, 850, 2000).is_some());
    }

    #[test]
    fn a_warm_entry_is_unaffected_by_the_gate() {
        // a warm start into each airborne stage flies the same whatever the window: the gate looks only in SETTING
        let ticks: Vec<(u32, Readings)> = (0..80)
            .map(|tick| {
                let now_ms = tick * 50;
                let metres = 200.0 - (now_ms as f32 - 600.0).abs() * 0.1;
                let readings = Readings {
                    elevation: Some(Reading {
                        value: metres.max(0.0),
                        source: Source::Other,
                    }),
                    ..acceleration(1.0)
                };
                (now_ms, readings)
            })
            .collect();
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing] {
            let flown = |window_ms: u32| {
                let mut machine = StageMachine::new(Settings {
                    launch_alt_window_ms: window_ms,
                    apogee_arm_ms: 0,
                    land_agl_m: 150.0,
                    ..spec()
                });
                let mut current = stage;
                let mut recorded = Vec::new();
                for (now_ms, readings) in &ticks {
                    let step = machine.step(*now_ms, current, readings, false);
                    current = step.advance.map_or(current, |advance| advance.to);
                    recorded.extend(rows(step));
                }
                (current, recorded)
            };
            let (end, recorded) = flown(1000);
            assert_eq!(end, Stage::Done, "{stage:?} flies to the ground");
            assert_eq!(flown(0), (end, recorded.clone()), "{stage:?}");
            assert_eq!(flown(LONGEST_LAUNCH_ALTITUDE_WINDOW_MS), (end, recorded), "{stage:?}");
        }
    }
}
