//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-stage machine, from src/glider tasks/sequencer.py (`Sequencer._tick` and its detectors): the
//! guarded, forward-only stage automation the control loop gates on.
//!
//! - SETTING -> BOOSTING: |accel| over `launch_g` for `launch_ms` of net time (motor ignition), OR the baro
//!   climbing past `launch_alt_m` for as long (an independent, threshold-robust backup). Never the GNSS
//!   elevation.
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
/// missing one ([`Settings::default`]). `disable_gc_flight` is not ported.
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
    /// apogee dwell too (§7.2 Q14: ported faithfully, to stay comparable with the HITL baselines).
    pub launch_ms: u32,
    /// The baro launch backup in metres above the pad: clearly climbed off the rod.
    pub launch_alt_m: f32,
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
    /// Plan B's `Sequencer.setup()` defaults: what a board config without the key flies.
    fn default() -> Settings {
        Settings {
            period_ms: 50,
            launch_g: 2.5,
            launch_ms: 100,
            launch_alt_m: 10.0,
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
    /// signed elapsed time reaches), written with a fraction of zero too (`300.0`, which plan B flies as 300).
    /// Anything else is refused, naming the key, so the board fails the entry at bring-up and `arm` refuses. Plan B
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
        Ok(Settings {
            period_ms: milliseconds(config, "period_ms", defaults.period_ms)?,
            launch_g: real(config, "launch_g", defaults.launch_g)?,
            launch_ms: milliseconds(config, "launch_ms", defaults.launch_ms)?,
            launch_alt_m: real(config, "launch_alt_m", defaults.launch_alt_m)?,
            boost_timeout_ms: milliseconds(config, "boost_timeout_ms", defaults.boost_timeout_ms)?,
            apogee_drop_m: real(config, "apogee_drop_m", defaults.apogee_drop_m)?,
            apogee_arm_ms: milliseconds(config, "apogee_arm_ms", defaults.apogee_arm_ms)?,
            flight_timeout_ms: milliseconds(config, "flight_timeout_ms", defaults.flight_timeout_ms)?,
            land_agl_m: real(config, "land_agl_m", defaults.land_agl_m)?,
            land_ms: milliseconds(config, "land_ms", defaults.land_ms)?,
            still_g: real(config, "still_g", defaults.still_g)?,
            ground_ms: milliseconds(config, "ground_ms", defaults.ground_ms)?,
            land_timeout_ms: milliseconds(config, "land_timeout_ms", defaults.land_timeout_ms)?,
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
/// backstop off. Every `*_ms` key takes the same bound, `period_ms` included.
const LONGEST_MS: u32 = i32::MAX as u32;

/// A `*_ms` key of `config`: a whole number of ms up to [`LONGEST_MS`], an integer or a float with no fraction;
/// `default` when the key is absent.
fn milliseconds(config: &Value, key: &'static str, default: u32) -> Result<u32, SettingsError> {
    let Some(value) = config.get(key) else {
        return Ok(default);
    };
    let refused = |problem| Err(SettingsError::new(key, value, problem));
    let Value::Number(number) = value else {
        return refused(SettingsProblem::NotANumber);
    };
    if let Some(whole) = number.as_u64() {
        return match u32::try_from(whole) {
            Ok(whole) if whole <= LONGEST_MS => Ok(whole),
            _ => refused(SettingsProblem::TooLong),
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
    if float > f64::from(LONGEST_MS) {
        return refused(SettingsProblem::TooLong);
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
    /// A `*_ms` past 2147483647 (`i32::MAX`, about 24.8 days): the machine's signed elapsed time never reaches it.
    TooLong,
}

impl fmt::Display for SettingsError {
    /// `<key>: <value> <why>`.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (key, value) = (self.key, &self.value);
        match self.problem {
            SettingsProblem::NotANumber => write!(formatter, "{key}: {value} is not a number"),
            SettingsProblem::Negative => write!(formatter, "{key}: {value} is negative"),
            SettingsProblem::Fractional => write!(formatter, "{key}: {value} is not a whole number of ms"),
            SettingsProblem::TooLong => write!(formatter, "{key}: {value} is past the longest, {LONGEST_MS} ms"),
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
    fn detect_launch(&mut self, now_ms: u32, readings: &Readings) -> Option<Transition> {
        let elevation = readings
            .elevation
            .filter(|reading| reading.source != Source::Gnss)
            .map(|reading| reading.value);
        let magnitude_squared = readings
            .acceleration
            .map(|reading| commons::magnitude_squared(reading.value));
        let climbed_now = elevation.is_some_and(|metres| metres > self.settings.launch_alt_m);
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

    use super::{Reading, Readings, Reason, Settings, SettingsProblem, Source, Stage, StageMachine, Step, Transition};

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
        // the same 15 m from a barometer is a launch
        let baro = elevation(15.0, Source::Other);
        assert_eq!(
            run(&mut machine, Stage::Setting, &[(2000, baro), (2100, baro)]),
            Stage::Boosting
        );
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

    #[test]
    fn the_launch_lines_are_strict() {
        // exactly launch_g (3.0 g) and exactly launch_alt_m (10.0 m), held: no launch; f32 holds both exactly
        let mut machine = StageMachine::new(spec());
        assert_eq!(repeat(&mut machine, Stage::Setting, acceleration(3.0), 0, 300), None);
        assert_eq!(
            repeat(&mut machine, Stage::Setting, elevation(10.0, Source::Other), 350, 650),
            None
        );
        // clearly over either line, it launches
        let launch = repeat(&mut machine, Stage::Setting, acceleration(3.5), 700, 800);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch |a|=3.5g dwell=100ms".to_string())
        );
        let mut machine = StageMachine::new(spec());
        let launch = repeat(&mut machine, Stage::Setting, elevation(10.5, Source::Other), 0, 100);
        assert_eq!(launch.map(|step| step.to), Some(Stage::Boosting));
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
        for readings in [acceleration(8.0), elevation(15.0, Source::Other)] {
            let mut machine = StageMachine::new(spec());
            machine.step(0, Stage::Setting, &readings, false);
            machine.step(50, Stage::Setting, &readings, false);
            machine.step(60, Stage::Gliding, &readings, false);
            // back on the pad 4.9 s later: the half credit is gone, so one sample is no launch
            let step = machine.step(5000, Stage::Setting, &readings, false);
            assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
            assert_eq!(machine.step(5050, Stage::Setting, &readings, false).advance, None);
            assert!(machine.step(5100, Stage::Setting, &readings, false).advance.is_some());
        }
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
        // both dwells complete on one tick: the baro backup is checked first
        let both = Readings {
            elevation: Some(Reading {
                value: 15.0,
                source: Source::Other,
            }),
            ..acceleration(8.0)
        };
        let mut machine = StageMachine::new(spec());
        let launch = repeat(&mut machine, Stage::Setting, both, 0, 100);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch alt=15m dwell=100ms".to_string())
        );
        // a clock stepping back onto a sample with no reading feeds the dwell to full: plan B prints 0
        for (readings, text) in [
            (acceleration(8.0), "launch |a|=0.0g dwell=100ms"),
            (elevation(15.0, Source::Other), "launch alt=0m dwell=100ms"),
        ] {
            let mut machine = StageMachine::new(spec());
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
        assert_eq!(StageMachine::new(settings).settings(), &settings);
    }

    /// Every key [`Settings::from_config`] reads.
    const KEYS: [&str; 13] = [
        "period_ms",
        "launch_g",
        "launch_ms",
        "launch_alt_m",
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
}
