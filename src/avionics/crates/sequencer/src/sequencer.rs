//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight-stage machine, from src/glider tasks/sequencer.py (`Sequencer._tick` and its detectors): the
//! guarded, forward-only stage automation the control loop gates on, deciding from every source of each quantity.
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
//! **Every source decides in its turn (the operator's, not plan B's).** Plan B decided on one fused reading per
//! quantity. Here every source the databoard registers for a quantity holds a fixed slot, bound by rank at bring-up
//! ([`Binding`]): four accelerometers, two barometers plus the GNSS (by name, never a voter), one laser. A slot keeps
//! only the history that compares its source with itself (a barometer's climb and apogee tracker, an accelerometer's
//! thrust gate and its run at rest), so the offset between two barometers never reads as a rise or a drop, and no
//! reconcile offset is ever applied. A source is LIVE at a step when it is fresh in its quantity's shared window and
//! its value is finite: plan B's `read()`, which never consults health.
//!
//! Each detector keeps ONE dwell, plan B's own, stepped every step with the condition of its DECIDER: the
//! highest-ranked live source of the class (for the launch; for the apogee the top live barometer, else the GNSS; for
//! the landing the laser, else the top live barometer, else the GNSS). A lower source never decides while a higher one
//! is live, and after a handover the dwell carries on only while the new decider's own condition holds too. With no
//! decider the dwell sees no reading, as in plan B. One exception, for the apogee: a source back from a gap the flight
//! went on through without it JOINS afresh ([`Tracker`]): its smoothing restarts, it sits out `launch_ms`, and it
//! decides only while no barometer with an unbroken record is live, since its own record may lack the top.
//!
//! With one source per class this is plan B step for step (golden sequencer_trace.json) but for three things: the
//! physics vetoes below (pinned in the fixture where they change a plan B event); a NaN reading, which is no reading
//! here and poisoned plan B's apogee tracker; and a GNSS standing in for the barometer, which plan B's one apogee
//! tracker smoothed into the barometer's series and the port keeps apart, the GNSS joining on its return (no fixture
//! event moves).
//!
//! On a step where the decider's condition holds, a VETO makes it an inactive sample (a leaky dwell drains, a
//! contiguous one restarts): a held event must earn its whole dwell again unvetoed, and a peer that misses one step
//! cannot release it.
//! - The PEER veto: at least half of the live voting sources, the decider among them, are contrary ([`Detector`] says
//!   when a peer is). A missing reading is never contrary; the GNSS never counts and never votes.
//! - The PHYSICS vetoes, across classes, each on EVERY live accelerometer (none live, no veto), so one faulty but live
//!   part can never block a launch path alone: the altitude launch is held while every live accelerometer has read
//!   inside the still band for the whole `launch_alt_window_ms` (a boost leaves the band within a step, a coast passes
//!   through it in about 0.4 s; a step without a reading keeps an accelerometer's run, so a dropout never releases the
//!   veto); the apogee is held while every live accelerometer's thrust gate ([`crate::thrust`], on its own +x) still
//!   sees the boost.
//!
//! The backstops (burnout, flight and landing timeouts) run on machine clocks from stage entry and are never vetoed.
//! [`Step`] reports where each veto episode begins and ends, for vetoes.csv.
//!
//! [`StageMachine::step`] is `_tick`, pure: the caller passes the time, the stage and the hold the flight word
//! holds, and every source's fresh reading, and gets back what to record and where to go. It reads no clock and no
//! channel, allocates nothing, and the caller commits the advance through the flight word's writer
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
//! apogee. Every detector here takes a reading only when its source vouches for it (plan B's `read()` with a
//! non-None source); the Rust databoard has no extrapolation at all, so a stale channel is simply None.
//!
//! Plan B's GC policy (compact and disable at BOOSTING, re-enable at DONE) is not ported: Rust has no
//! collector. Neither is its sequencer.csv push, which is the caller's: [`Step`] carries the rows.

use core::fmt;

use serde_json::Value;

use crate::commons::{self, Apogee, Dwell};
use crate::controller::Stage;
use crate::thrust::{Thrust, ThrustGate};

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
    /// long, so the altitude launch never fires. At most [`LONGEST_LAUNCH_ALTITUDE_WINDOW_MS`]. It is also how long
    /// every live accelerometer must have read at rest to hold the altitude launch (see the module notes).
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
    /// The stillness tolerance around 1 g, in g (always under 1: a tolerance, not a level). Every shipped config
    /// keeps `launch_g` above 1 + `still_g`, so a boost is never at rest and never a contrary peer of a boost.
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

/// The longest `launch_alt_window_ms`, twice the operator's 1 s: the machine keeps each barometer slot's elevation
/// history for it in a fixed array of 81 slots, 25 ms apart, 12 bytes each (972 B, two of them, 1944 B), boxed when the
/// machine is built so they live on the heap and not on the control thread's stack; stepping allocates nothing. A
/// sluggish 1.5 g boost is 10 m up 2.0 s after ignition: a 2 s window passes it then, the default 1 s about 0.5 s later.
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

/// The device name the GNSS publishes `elevation` under (plan B tasks/sequencer.py `_GNSS_SOURCE`): [`Binding`] gives
/// it the elevation class's GNSS slot, which never starts a flight and never votes.
pub const GNSS_SOURCE: &str = "gnss";

/// The accel class's voting slots: the LSM6DSO32 and the BMI323 on a SEN0697 board, and room for the ADXL375 and the
/// BNO055 once they are ported.
pub const ACCELEROMETER_SLOTS: usize = 4;

/// The elevation class's voting slots: the barometers (the ICP-10111 and the BMP581; the BMP280 on TMS-7C). The GNSS
/// has a slot of its own besides them, by name.
pub const BAROMETER_SLOTS: usize = 2;

/// The `agl` class's slots: the laser. The pitot's `airspeed` class comes with the pitot.
const LASER_SLOTS: usize = 1;

/// Who vouches for a fresh fused reading. Plan B's `read()` names the device; the one-source form asks one question of
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

/// One fresh fused channel value, as plan B's `read()` answers it with a non-None source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading<T> {
    /// The value.
    pub value: T,
    /// The device that published it.
    pub source: Source,
}

/// The fused read: one reading per quantity, each None when no source is fresh -- the one-source form the control
/// thread steps the machine with until it assembles [`SourceReadings`] from its [`Binding`]. It converts as plan B's
/// fused read would sit in the slots: the accel in the first accelerometer slot, the elevation in the first barometer
/// slot (the GNSS slot when the GNSS vouches for it), the agl in the laser slot. Stepped so, the machine is plan B's
/// but for the fusion rule's own (see the module notes): the physics vetoes read the fused accel, so a board lying
/// still holds an altitude launch, and only a bench copy with no accelerometer entry launches by altitude hands-free.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Readings {
    /// The `accel` channel: (x, y, z) specific force in g.
    pub acceleration: Option<Reading<[f32; 3]>>,
    /// The `elevation` channel: metres above the pad (the baros, else the GNSS).
    pub elevation: Option<Reading<f32>>,
    /// The `agl` channel: the laser's height above the ground in metres.
    pub height_above_ground: Option<Reading<f32>>,
}

/// Every bound source's fresh reading at one step, by its slot in the [`Binding`]: None where the source is not fresh
/// in its quantity's shared window, or no source holds the slot.
///
/// None is not a low reading and not a still one: a detector whose decider has no reading does not advance (and the
/// contiguous dwells restart). A value that is not finite (a NaN, an infinity, in any axis of a vector) is no reading
/// either: the machine checks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SourceReadings {
    /// The `accel` sources in rank order: (x, y, z) specific force in g, on the body frame (+x the nose).
    pub accelerations: [Option<[f32; 3]>; ACCELEROMETER_SLOTS],
    /// The barometers' `elevation` in rank order: metres above the pad each zeroed its own.
    pub elevations: [Option<f32>; BAROMETER_SLOTS],
    /// The GNSS's `elevation`: metres above its first fix.
    pub gnss_elevation: Option<f32>,
    /// The laser's `agl`: the height above the ground in metres.
    pub height_above_ground: Option<f32>,
}

impl SourceReadings {
    /// The live readings: every value that is not finite taken out.
    fn live(self) -> SourceReadings {
        SourceReadings {
            accelerations: self
                .accelerations
                .map(|reading| reading.filter(|vector| vector.iter().all(|axis| axis.is_finite()))),
            elevations: self
                .elevations
                .map(|reading| reading.filter(|metres| metres.is_finite())),
            gnss_elevation: self.gnss_elevation.filter(|metres| metres.is_finite()),
            height_above_ground: self.height_above_ground.filter(|metres| metres.is_finite()),
        }
    }
}

impl From<&Readings> for SourceReadings {
    /// Plan B's fused read in each class's first slot, the GNSS's elevation in the GNSS slot.
    fn from(readings: &Readings) -> SourceReadings {
        let mut sources = SourceReadings::default();
        sources.accelerations[0] = readings.acceleration.map(|reading| reading.value);
        match readings.elevation {
            Some(Reading {
                value,
                source: Source::Gnss,
            }) => sources.gnss_elevation = Some(value),
            Some(Reading {
                value,
                source: Source::Other,
            }) => sources.elevations[0] = Some(value),
            None => {}
        }
        sources.height_above_ground = readings.height_above_ground.map(|reading| reading.value);
        sources
    }
}

/// Which slots the registered sources hold: bound once at bring-up, in rank order per quantity (the databoard's
/// `sources()`), so a slot always belongs to the same source and one source's history never passes to another.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Binding {
    /// The `accel` sources, by slot.
    accelerometers: [Option<&'static str>; ACCELEROMETER_SLOTS],
    /// The `elevation` sources but the GNSS, by slot.
    barometers: [Option<&'static str>; BAROMETER_SLOTS],
    /// The GNSS, when it publishes `elevation`.
    gnss: Option<&'static str>,
    /// The `agl` sources, by slot.
    lasers: [Option<&'static str>; LASER_SLOTS],
    /// The sources past their class's slots, in the order they came.
    ignored: Vec<Ignored>,
}

/// A source past its class's slots: the detectors never read it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ignored {
    /// The quantity it publishes.
    pub quantity: &'static str,
    /// Its device name.
    pub source: &'static str,
    /// How many sources the class reads.
    pub slots: usize,
}

impl fmt::Display for Ignored {
    /// The bring-up log line's text: `<source> ignored: the stage machine's <quantity> slots are full (<slots>)`.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{} ignored: the stage machine's {} slots are full ({})",
            self.source, self.quantity, self.slots
        )
    }
}

impl Binding {
    /// Bind each quantity's registered sources, given in rank order, to its class's slots: the first `accel` sources
    /// to the accelerometers, the first `elevation` sources but [`GNSS_SOURCE`] to the barometers and the GNSS to its
    /// own slot, the first `agl` source to the laser. The rest are [`Binding::ignored`].
    pub fn new(
        accelerations: impl IntoIterator<Item = &'static str>,
        elevations: impl IntoIterator<Item = &'static str>,
        heights_above_ground: impl IntoIterator<Item = &'static str>,
    ) -> Binding {
        let mut binding = Binding::default();
        let (gnss, barometers): (Vec<&'static str>, Vec<&'static str>) =
            elevations.into_iter().partition(|source| *source == GNSS_SOURCE);
        binding.gnss = gnss.first().copied();
        binding.accelerometers = bind("accel", accelerations, &mut binding.ignored);
        binding.barometers = bind("elevation", barometers, &mut binding.ignored);
        binding.lasers = bind("agl", heights_above_ground, &mut binding.ignored);
        binding
    }

    /// The sources past their class's slots, for the bring-up log and `inspect`.
    pub fn ignored(&self) -> &[Ignored] {
        &self.ignored
    }

    /// The source holding `decider`'s slot; None for an empty slot.
    pub fn source(&self, decider: Decider) -> Option<&'static str> {
        match decider {
            Decider::Accelerometer(slot) => self.accelerometers.get(slot).copied().flatten(),
            Decider::Barometer(slot) => self.barometers.get(slot).copied().flatten(),
            Decider::Gnss => self.gnss,
        }
    }

    /// One step's readings, by slot: each bound source's reading through the reader of its quantity, called with its
    /// device name -- the firmware reads that source's channel, fresh in the quantity's shared window or None. An
    /// ignored source is never read.
    pub fn readings(
        &self,
        acceleration: impl Fn(&'static str) -> Option<[f32; 3]>,
        elevation: impl Fn(&'static str) -> Option<f32>,
        height_above_ground: impl Fn(&'static str) -> Option<f32>,
    ) -> SourceReadings {
        let [laser] = self.lasers;
        SourceReadings {
            accelerations: self.accelerometers.map(|source| source.and_then(&acceleration)),
            elevations: self.barometers.map(|source| source.and_then(&elevation)),
            gnss_elevation: self.gnss.and_then(&elevation),
            height_above_ground: laser.and_then(height_above_ground),
        }
    }
}

/// `sources` of `quantity`, in rank order, into `SLOTS` slots; the rest pushed to `ignored`.
fn bind<const SLOTS: usize>(
    quantity: &'static str,
    sources: impl IntoIterator<Item = &'static str>,
    ignored: &mut Vec<Ignored>,
) -> [Option<&'static str>; SLOTS] {
    let mut slots = [None; SLOTS];
    for (rank, source) in sources.into_iter().enumerate() {
        match slots.get_mut(rank) {
            Some(slot) => *slot = Some(source),
            None => ignored.push(Ignored {
                quantity,
                source,
                slots: SLOTS,
            }),
        }
    }
    slots
}

/// The slot a detector's decider holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decider {
    /// An accelerometer, by slot.
    Accelerometer(usize),
    /// A barometer, by slot.
    Barometer(usize),
    /// The GNSS.
    Gnss,
}

/// How many detectors a veto can hold: [`Detector`]'s variants.
const DETECTORS: usize = 4;

/// A detector a veto can hold, and when a live peer of its decider is contrary. The landing has no veto: the drift
/// between two barometers is unmeasured, and the pitot's vetoes come with the pitot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Detector {
    /// The altitude launch: a peer barometer that has confirmed a level and risen less than `launch_alt_m`/2 from it is
    /// contrary. Its physics veto: every live accelerometer at rest.
    LaunchAltitude,
    /// The accel launch: a peer accelerometer reading inside the still band is contrary.
    LaunchAcceleration,
    /// The apogee: a peer barometer whose own tracker raised its smoothed peak on this step is contrary (one joining
    /// the flight afresh does not vote yet). Its physics veto: every live accelerometer still under thrust.
    Apogee,
    /// The stillness on the ground: a peer accelerometer reading outside the still band is contrary.
    Stillness,
}

impl Detector {
    /// The detector's name, vetoes.csv's `detector` cell.
    pub fn name(self) -> &'static str {
        match self {
            Detector::LaunchAltitude => "launch_altitude",
            Detector::LaunchAcceleration => "launch_acceleration",
            Detector::Apogee => "apogee",
            Detector::Stillness => "stillness",
        }
    }
}

/// The physics veto that held a detector (see the module notes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Physics {
    /// Every live accelerometer at rest for the rate gate's window: the altitude launch is held.
    AtRest,
    /// Every live accelerometer's thrust gate still sees the boost: the apogee is held.
    UnderThrust,
}

/// One vetoed step of a detector: who decided, which vetoes held it, the vote, and the decider's value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Veto {
    /// The detector held.
    pub detector: Detector,
    /// Its decider's slot ([`Binding::source`] names it).
    pub decider: Decider,
    /// Whether the peers' quorum held it.
    pub peer: bool,
    /// The physics veto that held it, if one did.
    pub physics: Option<Physics>,
    /// How many of the decider's live peers were contrary.
    pub contrary: u8,
    /// How many voting sources were live, the decider included.
    pub live: u8,
    /// The decider's value: |accel| in g for the launch's accel and the stillness, the elevation in metres for the
    /// altitude launch and the apogee.
    pub value: f32,
}

impl Veto {
    /// The vetoes that held it, vetoes.csv's `kinds` cell: `peer`, `at_rest` or `thrust`, two joined by `+`.
    pub fn kinds(&self) -> &'static str {
        match (self.peer, self.physics) {
            (true, None) => "peer",
            (false, Some(Physics::AtRest)) => "at_rest",
            (false, Some(Physics::UnderThrust)) => "thrust",
            (true, Some(Physics::AtRest)) => "peer+at_rest",
            (true, Some(Physics::UnderThrust)) => "peer+thrust",
            (false, None) => "none", // never a veto: the machine makes none without a kind
        }
    }
}

/// Where a veto episode -- a detector vetoed on consecutive steps -- begins and ends: the two rows vetoes.csv records.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VetoEdge {
    /// The episode's first vetoed step.
    Begin(Veto),
    /// The first step its detector is not vetoed (it ran unvetoed, its condition failed, or its stage ended), carrying
    /// the episode's last vetoed step.
    End(Veto),
}

impl VetoEdge {
    /// `begin` or `end`: vetoes.csv's `edge` cell.
    pub fn name(&self) -> &'static str {
        match self {
            VetoEdge::Begin(_) => "begin",
            VetoEdge::End(_) => "end",
        }
    }

    /// The vetoed step the edge carries.
    pub fn veto(&self) -> &Veto {
        match self {
            VetoEdge::Begin(veto) | VetoEdge::End(veto) => veto,
        }
    }
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
        /// The deciding barometer's elevation on the step that completed the dwell (0.0 with none live).
        elevation_m: f32,
        /// The dwell's credit, capped at `launch_ms`.
        dwell_ms: u32,
    },
    /// The accel launch: `launch |a|=<magnitude, 1 decimal>g dwell=<credit>ms`. The magnitude is 0.0 when the
    /// sample that completed the dwell had no live accel (an inactive sample can complete it when the clock
    /// steps backwards; plan B prints 0.0 rather than taking the root of nothing).
    LaunchAcceleration {
        /// The deciding accelerometer's |accel| on the step that completed the dwell, in g.
        magnitude_g: f32,
        /// The dwell's credit, capped at `launch_ms`.
        dwell_ms: u32,
    },
    /// Apogee: `apogee <smoothed peak, 0 decimals>m`.
    Apogee {
        /// The deciding source's smoothed peak elevation.
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
        /// The deciding accelerometer's |accel| on the step that completed the dwell, in g.
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

/// What one step reports: up to two sequencer.csv rows, in the order plan B pushes them, and the veto episodes that
/// begin or end on it.
///
/// Two rows, because `_tick` first notices a move made from outside since the last tick and only then runs the
/// detectors, and both can record on one tick: an operator releases a hold that outlasted the flight timeout,
/// and the backstop fires on the very tick the held stage is first seen.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Step {
    /// A stage someone else moved to since the last step, recorded with [`Reason::External`].
    pub external: Option<Transition>,
    /// The stage this step advances to. The caller commits it ([`crate::FlightWriter::advance`]) before the next
    /// step.
    pub advance: Option<Transition>,
    /// Each detector's episode edge on this step, if one begins or ends, by [`Detector`] (both launch detectors
    /// can hold episodes at once).
    vetoes: [Option<VetoEdge>; DETECTORS],
}

impl Step {
    /// The rows to record, in order.
    pub fn transitions(&self) -> impl Iterator<Item = Transition> {
        self.external.into_iter().chain(self.advance)
    }

    /// The veto episodes that begin or end on this step, in [`Detector`] order: vetoes.csv's rows.
    pub fn vetoes(&self) -> impl Iterator<Item = VetoEdge> + '_ {
        self.vetoes.iter().flatten().copied()
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

/// One barometer's fresh elevations of the last `launch_alt_window_ms`, for the altitude launch's rate gate: how far
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
    /// Args: `now_ms` the step clock; `elevation_m` the fresh elevation (finite); `window_ms` `launch_alt_window_ms`;
    /// `held_ms` `launch_ms`.
    ///
    /// Returns: the rise in metres: the elevation minus the lowest level confirmed within the window (negative when
    /// all lie above it); None while no level within the window is confirmed, so a source back from a gap longer than
    /// the window, or new on the pad, confirms nothing.
    fn rise(&mut self, now_ms: u32, elevation_m: f32, window_ms: u32, held_ms: u32) -> Option<f32> {
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
            .fold(f32::INFINITY, f32::min);
        lowest_m.is_finite().then_some(elevation_m - lowest_m)
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

/// One accelerometer slot's history: what the machine compares that source with, and only it.
#[derive(Clone, Copy, Debug)]
struct Accelerometer {
    /// Its thrust gate on its own +x: what it has seen of the boost since BOOSTING entry.
    thrust: ThrustGate,
    /// The first step of its current run of readings inside the still band: None before its first reading inside the
    /// band and after one outside it. A step without a reading keeps the run, so a dropout never releases the at-rest
    /// veto; the source then simply is not live, and the veto asks only the live ones.
    at_rest_since_ms: Option<u32>,
}

/// One elevation source's apogee history (a barometer's or the GNSS's), reset at every BOOSTING entry: its tracker,
/// and how current it is, for the return rule.
///
/// A tracker holds through a missing reading, as plan B's does, while nothing fresher exists. But a source back from
/// more than `launch_ms` without a reading (or first read that long after arming), while another elevation source was
/// read after it, comes back to a flight that moved on: its smoothing would crawl from where it left off, raising its
/// peak step after step, so as the decider it would hold the apogee off (or never call it) and as a peer it would vote
/// against it. So it JOINS:
/// - its smoothing restarts on the fresh reading (its peak stays its own);
/// - for `launch_ms` it neither decides nor votes;
/// - for the rest of the flight it decides only while no barometer with an unbroken record is live: its own record
///   may lack the top, and a peak without the top calls the apogee late.
#[derive(Clone, Copy, Debug, Default)]
struct Tracker {
    /// Its apogee tracker. Only its smoothed elevation and peak count: the apogee dwell is the machine's own.
    apogee: Apogee,
    /// The step clock its history is current to: the first armed step, then each reading's. None before arming.
    current_ms: Option<u32>,
    /// The step clock it last joined on: it decides and votes again `launch_ms` later. None while its record is
    /// unbroken.
    joined_ms: Option<u32>,
}

impl Tracker {
    /// Take a fresh elevation, armed: join first when the source comes back to a flight that moved on (see the type
    /// notes).
    ///
    /// Args: `now_ms` the step clock; `metres` the fresh elevation (finite); `current` every elevation source's
    /// `current_ms` before this step's readings, this one's included; `drop_m` `apogee_drop_m`; `launch_ms` the apogee
    /// dwell, how long a source may miss before it joins, and how long it then sits out.
    ///
    /// Returns: whether the reading raised the smoothed peak (a seed raises nothing: a peak that did not exist was not
    /// raised).
    fn feed(&mut self, now_ms: u32, metres: f32, current: &[Option<u32>], drop_m: f32, launch_ms: u32) -> bool {
        if let Some(own_ms) = self.current_ms {
            let overtaken = current
                .iter()
                .flatten()
                .any(|&peer_ms| commons::elapsed_ms(peer_ms, own_ms) > 0);
            if overtaken && i64::from(commons::elapsed_ms(now_ms, own_ms)) > i64::from(launch_ms) {
                self.apogee.smooth = None; // the smoothing restarts on this reading
                self.joined_ms = Some(now_ms);
            }
        }
        let peak = self.apogee.peak;
        (self.apogee, _) = commons::apogee_step(Some(metres), now_ms, self.apogee, drop_m, launch_ms);
        self.current_ms = Some(now_ms);
        peak.zip(self.apogee.peak).is_some_and(|(before, after)| after > before)
    }

    /// Whether it sits out at `now_ms`: less than `launch_ms` since it joined.
    fn joining(&self, now_ms: u32, launch_ms: u32) -> bool {
        self.joined_ms
            .is_some_and(|joined_ms| !commons::reached(now_ms, joined_ms, launch_ms))
    }
}

/// One barometer slot's history: what the machine compares that source with, and only it.
#[derive(Clone, Debug)]
struct Barometer {
    /// Its elevations on the pad, for the altitude launch's rate gate: boxed, so the 972 B of history live on the heap
    /// and the machine on the control thread's stack stays small.
    climb: Box<Climb>,
    /// Its apogee history.
    tracker: Tracker,
}

/// The stage machine: plan B's `Sequencer` state, one dwell per detector, and each slot's own history, stepped by
/// [`StageMachine::step`].
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
    /// The start of the current sustained condition, or None: the burnout dwell in BOOSTING, the landing dwell in
    /// GLIDING, the stillness dwell in LANDING (plan B's `_since`, each detector's in its own stage).
    since_ms: Option<u32>,
    /// The stage the last step saw, None before the first: any change resets the dwells, so a separation-driven
    /// hop starts clean.
    stage_seen: Option<Stage>,
    /// The stage this machine itself advanced to, until the next step sees it: tells its own moves from
    /// external ones.
    advanced_to: Option<Stage>,
    /// The apogee dwell: when the decider's smoothed elevation fell below its own peak by `apogee_drop_m`, or None
    /// while it is not below (plan B's tracker `since`), reset at every BOOSTING entry.
    apogee_since_ms: Option<u32>,
    /// The leaky launch dwell on |accel|.
    launch_dwell: Dwell,
    /// The leaky launch dwell on the baro elevation.
    altitude_dwell: Dwell,
    /// The step clock of BOOSTING entry: arms the apogee detector and bases the flight timeout. Re-based on the
    /// first step of an airborne stage entered without BOOSTING (a warm start, an operator move).
    boost_entry_ms: Option<u32>,
    /// The step clock of LANDING entry: bases the landing timeout.
    landing_entry_ms: Option<u32>,
    /// The accelerometer slots' histories.
    accelerometers: [Accelerometer; ACCELEROMETER_SLOTS],
    /// The barometer slots' histories.
    barometers: [Barometer; BAROMETER_SLOTS],
    /// The GNSS slot's apogee history.
    gnss: Tracker,
    /// Each detector's open veto episode, by [`Detector`]: its latest vetoed step.
    episodes: [Option<Veto>; DETECTORS],
}

/// The decider of a class: the first live slot, with its value.
fn decider<T: Copy>(readings: &[Option<T>]) -> Option<(usize, T)> {
    readings
        .iter()
        .enumerate()
        .find_map(|(slot, reading)| reading.map(|value| (slot, value)))
}

/// The peer vote on a step where the decider in `decider_slot` holds its condition: how many of its live peers in
/// `readings` are `contrary`, and how many voting sources are live, the decider included.
fn vote<T: Copy>(readings: &[Option<T>], decider_slot: usize, contrary: impl Fn(usize, T) -> bool) -> (u8, u8) {
    readings
        .iter()
        .enumerate()
        .filter_map(|(slot, reading)| reading.map(|value| (slot, value)))
        .fold((0, 0), |(against, live), (slot, value)| {
            (
                against + u8::from(slot != decider_slot && contrary(slot, value)),
                live + 1,
            )
        })
}

/// Whether the peers' vote holds the decider: at least half of the live voting sources contrary, one at least.
fn quorum(contrary: u8, live: u8) -> bool {
    contrary > 0 && 2 * u16::from(contrary) >= u16::from(live)
}

impl StageMachine {
    /// A machine that has seen nothing yet: its first step records the stage it finds as external (plan B starts
    /// with no stage seen, so even a cold boot's SETTING opens sequencer.csv as `setting,external`). The one
    /// allocation it ever makes is here: the barometers' climb histories, boxed.
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
            apogee_since_ms: None,
            launch_dwell: Dwell::default(),
            altitude_dwell: Dwell::default(),
            boost_entry_ms: None,
            landing_entry_ms: None,
            accelerometers: [Accelerometer {
                thrust: ThrustGate::new(&settings),
                at_rest_since_ms: None,
            }; ACCELEROMETER_SLOTS],
            barometers: core::array::from_fn(|_| Barometer {
                climb: Box::new(Climb::new()),
                tracker: Tracker::default(),
            }),
            gnss: Tracker::default(),
            episodes: [None; DETECTORS],
        }
    }

    /// The thresholds this machine runs.
    pub fn settings(&self) -> &Settings {
        &self.settings
    }

    /// One stage-machine step: plan B's `Sequencer._tick`, on every source.
    ///
    /// An operator hold (`manual`) suspends everything, the change detection and every slot's history included, so
    /// a held stage is first seen -- and recorded as external -- on the step after `stage auto`. Otherwise a stage
    /// different from the last one seen restarts the dwells and records an external move unless this machine made it;
    /// entering SETTING forgets the last flight's timers and the pad's climbs (else the backstop fired from SETTING on
    /// the pad one flight timeout after an operator's `stage gliding` / `stage setting`), entering LANDING starts the
    /// landing timeout, entering BOOSTING resets the apogee trackers and bases the arming window and the flight
    /// timeout, and entering GLIDING or LANDING without ever having seen BOOSTING re-bases the flight timeout there:
    /// the backstop re-bases, it never disappears. Then every accelerometer's thrust gate and run at rest take its
    /// reading, then the RSO backstop, then the stage's own detector; each advances at most once.
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
    /// previous step's advance applied); `readings` every bound source's fresh reading ([`SourceReadings`], or the
    /// fused [`Readings`] in the first slots); `manual` the operator hold.
    ///
    /// Returns: the rows to record and the veto episodes' edges.
    pub fn step(&mut self, now_ms: u32, stage: Stage, readings: impl Into<SourceReadings>, manual: bool) -> Step {
        let mut step = Step::default();
        if manual {
            return step; // the operator holds the stage: no auto-advance, nothing observed
        }
        let readings = readings.into().live();
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
        self.observe(now_ms, stage, &readings);
        let mut vetoes = [None; DETECTORS];
        step.advance = self.detect(now_ms, stage, &readings, &mut vetoes);
        step.vetoes = self.edges(vetoes);
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
            // the pad's elevation is measured afresh (after a re-zero, from the new ground)
            for barometer in &mut self.barometers {
                barometer.climb.clear();
            }
        }
        if stage == Stage::Landing {
            self.landing_entry_ms = Some(now_ms); // starts the landing timeout (a warm start into LANDING too)
        }
        if stage == Stage::Boosting {
            // peak tracking starts fresh for this flight
            for barometer in &mut self.barometers {
                barometer.tracker = Tracker::default();
            }
            self.gnss = Tracker::default();
            self.apogee_since_ms = None;
            self.boost_entry_ms = Some(now_ms); // arms the apogee detector, starts the flight timeout
        } else if self.boost_entry_ms.is_none() && matches!(stage, Stage::Gliding | Stage::Landing) {
            self.boost_entry_ms = Some(now_ms); // airborne without BOOSTING seen: re-base the backstop here
        }
    }

    /// Every accelerometer's own history takes its reading: its thrust gate on its +x (which starts afresh at a
    /// BOOSTING entry, and which a missing reading drains), and its run at rest, which only a reading outside the still
    /// band ends.
    fn observe(&mut self, now_ms: u32, stage: Stage, readings: &SourceReadings) {
        let still = readings
            .accelerations
            .map(|reading| reading.map(|vector| self.still(commons::magnitude_squared(vector))));
        for ((accelerometer, reading), still) in self.accelerometers.iter_mut().zip(readings.accelerations).zip(still) {
            accelerometer
                .thrust
                .step(now_ms, stage, reading.map(|[axial_g, _, _]| axial_g));
            match still {
                Some(true) => _ = accelerometer.at_rest_since_ms.get_or_insert(now_ms),
                Some(false) => accelerometer.at_rest_since_ms = None,
                None => {} // no reading: the run goes on
            }
        }
    }

    /// The RSO backstop, then the stage's own detector, each vetoed step written to `vetoes` by its detector.
    fn detect(
        &mut self,
        now_ms: u32,
        stage: Stage,
        readings: &SourceReadings,
        vetoes: &mut [Option<Veto>; DETECTORS],
    ) -> Option<Transition> {
        if stage.airborne()
            && let Some(entry_ms) = self.boost_entry_ms
            && commons::reached(now_ms, entry_ms, self.settings.flight_timeout_ms)
        {
            return Some(self.advance(Stage::Done, Reason::FlightTimeout));
        }
        match stage {
            Stage::Setting => self.detect_launch(now_ms, readings, vetoes),
            Stage::Boosting => self.detect_apogee(now_ms, readings, vetoes),
            Stage::Gliding => self.detect_landing(now_ms, readings),
            Stage::Landing => self.detect_stationary(now_ms, readings, vetoes),
            Stage::Done => None,
        }
    }

    /// Each detector's episode edge from this step's `vetoes`: a begin where an episode opens, an end (with its last
    /// vetoed step) where one closes; the open episodes move on.
    fn edges(&mut self, vetoes: [Option<Veto>; DETECTORS]) -> [Option<VetoEdge>; DETECTORS] {
        let mut edges = [None; DETECTORS];
        for ((edge, episode), veto) in edges.iter_mut().zip(&mut self.episodes).zip(vetoes) {
            *edge = match (*episode, veto) {
                (None, Some(veto)) => Some(VetoEdge::Begin(veto)),
                (Some(last), None) => Some(VetoEdge::End(last)),
                (Some(_), Some(_)) | (None, None) => None,
            };
            *episode = veto;
        }
        edges
    }

    /// Advance to `to` (plan B `_advance`, minus the Controller write the caller makes): the dwell restarts, and
    /// the move is marked as this machine's own, so the next step does not record it again as external.
    fn advance(&mut self, to: Stage, reason: Reason) -> Transition {
        self.since_ms = None;
        self.advanced_to = Some(to);
        Transition { to, reason }
    }

    /// The contiguous dwell of the burnout, landing and stillness detectors (plan B `_sustained`): starts on the first
    /// call after a reset, then reports whether `threshold_ms` has passed. Callers clear `since_ms` when their
    /// condition lapses or is vetoed.
    fn sustained(&mut self, now_ms: u32, threshold_ms: u32) -> bool {
        let since_ms = *self.since_ms.get_or_insert(now_ms);
        commons::reached(now_ms, since_ms, threshold_ms)
    }

    /// Whether |accel| squared lies inside the still band 1 +/- `still_g` (open at both edges).
    fn still(&self, squared: f32) -> bool {
        self.still_low_squared < squared && squared < self.still_high_squared
    }

    /// Whether `holds` for every live accelerometer, one at least: a physics veto's quorum, so one faulty but live part
    /// never holds a detector alone, and with none live nothing is held.
    fn every_live_accelerometer(&self, readings: &SourceReadings, holds: impl Fn(&Accelerometer) -> bool) -> bool {
        let mut live = self
            .accelerometers
            .iter()
            .zip(readings.accelerations)
            .filter_map(|(accelerometer, reading)| reading.map(|_| accelerometer))
            .peekable();
        live.peek().is_some() && live.all(holds)
    }

    /// The altitude launch's physics veto: every live accelerometer has read inside the still band for the whole
    /// `launch_alt_window_ms`.
    fn at_rest(&self, now_ms: u32, readings: &SourceReadings) -> bool {
        let window_ms = self.settings.launch_alt_window_ms;
        self.every_live_accelerometer(readings, |accelerometer| {
            accelerometer
                .at_rest_since_ms
                .is_some_and(|since_ms| commons::reached(now_ms, since_ms, window_ms))
        })
    }

    /// The apogee's physics veto: every live accelerometer's thrust gate still sees the boost.
    fn under_thrust(&self, readings: &SourceReadings) -> bool {
        self.every_live_accelerometer(readings, |accelerometer| accelerometer.thrust.thrust() == Thrust::Seen)
    }

    /// SETTING -> BOOSTING: the baro backup first, then the accel (plan B `_detect_launch`).
    ///
    /// Both inputs are FRESH or nothing. The accel is the trigger that STARTS the flight, and with the baro
    /// gated an extrapolated accel would be the only input left: a false launch on the pad from a channel no
    /// sensor feeds. Plan B paid for one with the GC off and the stage timeline running on the ground; in Rust it
    /// also ends the link's reconnects (they key on the detected launch). A stale baro extrapolates past
    /// `launch_alt_m` the same way (the ICP-10111 read timeouts on the bench), and it must hold `launch_ms`
    /// through the same leaky dwell as the accel: one sample over the line was enough once, which a single spike
    /// or a glitched read could satisfy. And the GNSS elevation is no baro (see [`Source::Gnss`]): it has its own
    /// slot, which no launch reads.
    ///
    /// Every live barometer records its own climb. The deciding barometer's sample counts for the dwell only when its
    /// elevation is over `launch_alt_m` and has also RISEN more than `launch_alt_m` within `launch_alt_window_ms` (the
    /// rate gate, not plan B's): a rocket gains it within a second, weather drift over hours, and a pad drifted past
    /// `launch_alt_m` launched TMS-7F unarmed overnight under plan B's rule alone. The rise is measured from a level
    /// its own elevation HELD for `launch_ms`, as the climb must hold: one low or glitched read on a drifted pad would
    /// otherwise stand in for the pad for the whole window. So an elevation already over the line on a barometer's
    /// first readings (no rise seen) is no launch either. A held altitude launch counts as not fired, so the accel
    /// dwell steps on, as in plan B's else-branch.
    fn detect_launch(
        &mut self,
        now_ms: u32,
        readings: &SourceReadings,
        vetoes: &mut [Option<Veto>; DETECTORS],
    ) -> Option<Transition> {
        let Settings {
            launch_alt_m: line_m,
            launch_alt_window_ms: window_ms,
            launch_ms,
            ..
        } = self.settings;
        let mut rises = [None; BAROMETER_SLOTS];
        for ((barometer, reading), rise) in self.barometers.iter_mut().zip(readings.elevations).zip(&mut rises) {
            if let Some(metres) = reading {
                *rise = barometer.climb.rise(now_ms, metres, window_ms, launch_ms);
            }
        }
        let barometer = decider(&readings.elevations);
        let climbed_now = barometer
            .is_some_and(|(slot, metres)| metres > line_m && rises[slot].is_some_and(|rise_m| rise_m > line_m));
        let altitude_veto = barometer.filter(|_| climbed_now).and_then(|(slot, metres)| {
            let (contrary, live) = vote(&readings.elevations, slot, |peer, _| {
                rises[peer].is_some_and(|rise_m| rise_m < line_m / 2.0)
            });
            let peer = quorum(contrary, live);
            let physics = self.at_rest(now_ms, readings).then_some(Physics::AtRest);
            (peer || physics.is_some()).then_some(Veto {
                detector: Detector::LaunchAltitude,
                decider: Decider::Barometer(slot),
                peer,
                physics,
                contrary,
                live,
                value: metres,
            })
        });
        vetoes[Detector::LaunchAltitude as usize] = altitude_veto;
        let (altitude_dwell, climbed) = commons::dwell_step(
            climbed_now && altitude_veto.is_none(),
            now_ms,
            self.altitude_dwell,
            launch_ms,
        );
        self.altitude_dwell = altitude_dwell;
        if climbed {
            let reason = Reason::LaunchAltitude {
                elevation_m: barometer.map_or(0.0, |(_, metres)| metres),
                dwell_ms: altitude_dwell.credit_ms,
            };
            return Some(self.advance(Stage::Boosting, reason));
        }
        let accelerometer =
            decider(&readings.accelerations).map(|(slot, vector)| (slot, commons::magnitude_squared(vector)));
        let boosting_now = accelerometer.is_some_and(|(_, squared)| squared > self.launch_g_squared);
        let acceleration_veto = accelerometer.filter(|_| boosting_now).and_then(|(slot, squared)| {
            let (contrary, live) = vote(&readings.accelerations, slot, |_, vector| {
                self.still(commons::magnitude_squared(vector))
            });
            quorum(contrary, live).then_some(Veto {
                detector: Detector::LaunchAcceleration,
                decider: Decider::Accelerometer(slot),
                peer: true,
                physics: None,
                contrary,
                live,
                value: squared.sqrt(),
            })
        });
        vetoes[Detector::LaunchAcceleration as usize] = acceleration_veto;
        let (launch_dwell, fired) = commons::dwell_step(
            boosting_now && acceleration_veto.is_none(),
            now_ms,
            self.launch_dwell,
            launch_ms,
        );
        self.launch_dwell = launch_dwell;
        if !fired {
            return None;
        }
        let reason = Reason::LaunchAcceleration {
            magnitude_g: accelerometer.map_or(0.0, |(_, squared)| squared.sqrt()),
            dwell_ms: launch_dwell.credit_ms,
        };
        Some(self.advance(Stage::Boosting, reason))
    }

    /// BOOSTING -> GLIDING: apogee, else the burnout timeout (plan B `_detect_apogee`).
    ///
    /// Each barometer's tracker and the GNSS's see their own fresh elevation only once armed, `apogee_arm_ms` past
    /// BOOSTING entry; an extrapolated sample would feed the PEAK, and a peak is latched. A tracker holds through a
    /// missing reading, as plan B's does, but a source back to a flight that moved on joins ([`Tracker`]). The decider
    /// is the top live barometer with an unbroken record, else the top live one past its joining, else the GNSS; its
    /// condition is its own smoothed elevation below its own peak by `apogee_drop_m`, and the dwell on it is
    /// `launch_ms`, plan B's reuse. With no decider (blind, or none live) the dwell holds, as plan B's tracker does
    /// through None. The burnout timeout runs from BOOSTING entry regardless of arming, and no veto holds it.
    fn detect_apogee(
        &mut self,
        now_ms: u32,
        readings: &SourceReadings,
        vetoes: &mut [Option<Veto>; DETECTORS],
    ) -> Option<Transition> {
        let armed = self
            .boost_entry_ms
            .is_some_and(|entry_ms| commons::reached(now_ms, entry_ms, self.settings.apogee_arm_ms));
        let Settings {
            apogee_drop_m: drop_m,
            launch_ms,
            ..
        } = self.settings;
        // the barometers that vote (live, armed and not joining), those of them with an unbroken record, whether each
        // raised its peak, and the GNSS's elevation when it may decide
        let mut voting = [None; BAROMETER_SLOTS];
        let mut unbroken = [None; BAROMETER_SLOTS];
        let mut raised = [false; BAROMETER_SLOTS];
        let mut gnss = None;
        if armed {
            let trackers = self.barometers.iter_mut().map(|barometer| &mut barometer.tracker);
            for tracker in trackers.chain([&mut self.gnss]) {
                tracker.current_ms.get_or_insert(now_ms); // current from the first armed step on
            }
            let current: [Option<u32>; BAROMETER_SLOTS + 1] =
                core::array::from_fn(|slot| match self.barometers.get(slot) {
                    Some(barometer) => barometer.tracker.current_ms,
                    None => self.gnss.current_ms,
                });
            let slots = self.barometers.iter_mut().zip(readings.elevations);
            let marks = voting.iter_mut().zip(&mut unbroken).zip(&mut raised);
            for ((barometer, reading), ((voting, unbroken), raised)) in slots.zip(marks) {
                if let Some(metres) = reading {
                    let tracker = &mut barometer.tracker;
                    *raised = tracker.feed(now_ms, metres, &current, drop_m, launch_ms);
                    *voting = (!tracker.joining(now_ms, launch_ms)).then_some(metres);
                    *unbroken = voting.filter(|_| tracker.joined_ms.is_none());
                }
            }
            if let Some(metres) = readings.gnss_elevation {
                self.gnss.feed(now_ms, metres, &current, drop_m, launch_ms);
                gnss = (!self.gnss.joining(now_ms, launch_ms)).then_some(metres);
            }
        }
        // the decider (its barometer slot, None for the GNSS) and its tracker, just stepped: below its own line when it
        // holds a dwell start
        let decided = decider(&unbroken)
            .or_else(|| decider(&voting))
            .map(|(slot, metres)| (Some(slot), metres, self.barometers[slot].tracker.apogee))
            .or_else(|| gnss.map(|metres| (None, metres, self.gnss.apogee)));
        if let Some((slot, metres, apogee)) = decided {
            let below = apogee.since_ms.is_some();
            let veto = below
                .then(|| {
                    // the GNSS has no peers: no barometer votes when it decides
                    let (contrary, live) = slot.map_or((0, 0), |slot| vote(&voting, slot, |peer, _| raised[peer]));
                    let peer = quorum(contrary, live);
                    let physics = self.under_thrust(readings).then_some(Physics::UnderThrust);
                    (peer || physics.is_some()).then_some(Veto {
                        detector: Detector::Apogee,
                        decider: slot.map_or(Decider::Gnss, Decider::Barometer),
                        peer,
                        physics,
                        contrary,
                        live,
                        value: metres,
                    })
                })
                .flatten();
            vetoes[Detector::Apogee as usize] = veto;
            if below && veto.is_none() {
                let since_ms = *self.apogee_since_ms.get_or_insert(now_ms);
                if commons::reached(now_ms, since_ms, launch_ms)
                    && let Some(peak_m) = apogee.peak
                {
                    return Some(self.advance(Stage::Gliding, Reason::Apogee { peak_m }));
                }
            } else {
                self.apogee_since_ms = None; // back inside the band, or held: the fall must hold afresh
            }
        }
        if self.sustained(now_ms, self.settings.boost_timeout_ms) {
            return Some(self.advance(Stage::Gliding, Reason::BurnoutTimeout));
        }
        None
    }

    /// GLIDING -> LANDING: the height below `land_agl_m`, sustained `land_ms` (plan B `_detect_landing`).
    ///
    /// The fresh laser decides; with no fresh laser the top live barometer's elevation, else the GNSS's. ALL are
    /// source-gated: gating only the laser just moved where the invented number came from (a dead baro behind an
    /// out-of-range laser handed the landing trigger an extrapolated height). With none fresh there is no height
    /// and the dwell resets: no reading is not a low reading. A handover carries the dwell on while the new
    /// decider is low too (the laser coming into range mid-dwell). No veto holds the landing.
    fn detect_landing(&mut self, now_ms: u32, readings: &SourceReadings) -> Option<Transition> {
        let height = readings
            .height_above_ground
            .or_else(|| decider(&readings.elevations).map(|(_, metres)| metres))
            .or(readings.gnss_elevation);
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
    /// ~1 g, exactly the still band, so a dead accel would read as landed while still flying. The top live
    /// accelerometer decides, and the landing timeout fires whatever the vote.
    fn detect_stationary(
        &mut self,
        now_ms: u32,
        readings: &SourceReadings,
        vetoes: &mut [Option<Veto>; DETECTORS],
    ) -> Option<Transition> {
        if let Some(entry_ms) = self.landing_entry_ms
            && commons::reached(now_ms, entry_ms, self.settings.land_timeout_ms)
        {
            let reason = Reason::LandingTimeout {
                timeout_ms: self.settings.land_timeout_ms,
            };
            return Some(self.advance(Stage::Done, reason));
        }
        let accelerometer = decider(&readings.accelerations)
            .map(|(slot, vector)| (slot, commons::magnitude_squared(vector)))
            .filter(|(_, squared)| self.still(*squared));
        let veto = accelerometer.and_then(|(slot, squared)| {
            let (contrary, live) = vote(&readings.accelerations, slot, |_, vector| {
                !self.still(commons::magnitude_squared(vector))
            });
            quorum(contrary, live).then_some(Veto {
                detector: Detector::Stillness,
                decider: Decider::Accelerometer(slot),
                peer: true,
                physics: None,
                contrary,
                live,
                value: squared.sqrt(),
            })
        });
        vetoes[Detector::Stillness as usize] = veto;
        if let Some((_, squared)) = accelerometer
            && veto.is_none()
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
        Apogee, BAROMETER_SLOTS, CLIMB_SLOTS, Climb, Decider, Detector, LONGEST_LAUNCH_ALTITUDE_WINDOW_MS, Physics,
        Reading, Readings, Reason, Settings, SettingsProblem, Source, SourceReadings, Stage, StageMachine, Step,
        Thrust, Tracker, Transition, Veto, VetoEdge,
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

    /// A fresh accel reading from the first accelerometer.
    fn acceleration(z: f32) -> SourceReadings {
        let mut readings = SourceReadings::default();
        readings.accelerations[0] = Some([0.0, 0.0, z]);
        readings
    }

    /// A fresh elevation from the first barometer.
    fn barometer(metres: f32) -> SourceReadings {
        let mut readings = SourceReadings::default();
        readings.elevations[0] = Some(metres);
        readings
    }

    /// A fresh elevation from the GNSS.
    fn gnss(metres: f32) -> SourceReadings {
        SourceReadings {
            gnss_elevation: Some(metres),
            ..SourceReadings::default()
        }
    }

    /// The rows a step records, as `(stage, reason text)`.
    fn rows(step: Step) -> Vec<(Stage, String)> {
        step.transitions()
            .map(|transition| (transition.to, transition.reason.to_string()))
            .collect()
    }

    /// Step a machine through `ticks`, applying each advance as the caller does; returns the stage at the end.
    fn run(machine: &mut StageMachine, mut stage: Stage, ticks: &[(u32, SourceReadings)]) -> Stage {
        for (now_ms, readings) in ticks {
            if let Some(advance) = machine.step(*now_ms, stage, *readings, false).advance {
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
        let step = machine.step(0, Stage::Setting, acceleration(1.0), false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(
            machine.step(50, Stage::Setting, acceleration(1.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_sustained_boost_launches_and_is_not_recorded_twice() {
        let mut machine = StageMachine::new(spec());
        machine.step(1000, Stage::Setting, acceleration(8.0), false);
        assert_eq!(
            machine.step(1050, Stage::Setting, acceleration(8.0), false).advance,
            None
        );
        let step = machine.step(1100, Stage::Setting, acceleration(8.0), false);
        assert_eq!(
            rows(step),
            [(Stage::Boosting, "launch |a|=8.0g dwell=100ms".to_string())]
        );
        assert_eq!(
            machine.step(1110, Stage::Boosting, acceleration(8.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_stale_or_gnss_input_never_launches() {
        let mut machine = StageMachine::new(spec());
        let fifteen = gnss(15.0);
        let nothing = SourceReadings::default();
        let ticks: Vec<(u32, SourceReadings)> = (0..20)
            .map(|tick| (tick * 50, if tick % 2 == 0 { fifteen } else { nothing }))
            .collect();
        assert_eq!(run(&mut machine, Stage::Setting, &ticks), Stage::Setting);
        // the same 15 m from a barometer, risen from the pad it held, is a launch
        let (pad, baro) = (barometer(0.0), barometer(15.0));
        let ticks = [(1850, pad), (1900, pad), (1950, pad), (2000, baro), (2100, baro)];
        assert_eq!(run(&mut machine, Stage::Setting, &ticks), Stage::Boosting);
    }

    #[test]
    fn the_gnss_elevation_is_no_pad_to_climb_from() {
        // the baro drifted to 10.5 m; both baros go stale for 200 ms and the databoard falls through to the GNSS,
        // zeroed at its first fix and reading -1 m; then the baro is back at 10.5 m, which never rose from the GNSS
        let flown = |low: fn(f32) -> SourceReadings| {
            let ticks: Vec<(u32, SourceReadings)> = (0..=3000)
                .step_by(50)
                .map(|now_ms| match now_ms {
                    1000..1200 => (now_ms, low(-1.0)),
                    _ => (now_ms, barometer(10.5)),
                })
                .collect();
            run(&mut gated(1000), Stage::Setting, &ticks)
        };
        assert_eq!(flown(gnss), Stage::Setting);
        // the same -1 m from a barometer is a pad the 10.5 m rose 11.5 m from
        assert_eq!(flown(barometer), Stage::Boosting);
    }

    #[test]
    fn an_operator_hold_suspends_everything() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, acceleration(1.0), false);
        for now_ms in [50, 100, 500] {
            assert_eq!(
                machine.step(now_ms, Stage::Gliding, acceleration(8.0), true),
                Step::default()
            );
        }
        // released: the held stage is seen now, as external
        let step = machine.step(550, Stage::Gliding, SourceReadings::default(), false);
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
        machine.step(150, Stage::Boosting, acceleration(8.0), false);
        machine.step(200, Stage::Gliding, acceleration(1.0), true);
        let step = machine.step(1550, Stage::Gliding, acceleration(1.0), false);
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
        let high = barometer(80.0);
        assert_eq!(
            run(&mut machine, Stage::Gliding, &[(10_000, high), (11_999, high)]),
            Stage::Gliding
        );
        let step = machine.step(12_000, Stage::Gliding, high, false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
    }

    #[test]
    fn setting_forgets_the_backstop() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 1000,
            ..spec()
        });
        machine.step(0, Stage::Gliding, SourceReadings::default(), false);
        machine.step(100, Stage::Setting, acceleration(1.0), false);
        assert_eq!(
            machine.step(5000, Stage::Setting, acceleration(1.0), false),
            Step::default()
        );
    }

    #[test]
    fn a_stale_laser_never_lands_the_glider_at_altitude() {
        let mut machine = StageMachine::new(spec());
        // the laser is out of range (None), the baro says 274 m: no landing however long
        let apogee = barometer(274.0);
        assert_eq!(
            run(
                &mut machine,
                Stage::Gliding,
                &[(0, apogee), (500, apogee), (30_000, apogee)]
            ),
            Stage::Gliding
        );
        // a fresh laser low reading lands it, and wins over the baro
        let low = SourceReadings {
            height_above_ground: Some(2.0),
            ..apogee
        };
        machine.step(30_050, Stage::Gliding, low, false);
        let step = machine.step(30_150, Stage::Gliding, low, false);
        assert_eq!(rows(step), [(Stage::Landing, "agl 2.0m".to_string())]);
    }

    #[test]
    fn a_lost_height_resets_the_landing_dwell() {
        let mut machine = StageMachine::new(spec());
        let low = barometer(3.0);
        machine.step(0, Stage::Gliding, low, false);
        machine.step(90, Stage::Gliding, SourceReadings::default(), false);
        assert_eq!(
            machine.step(100, Stage::Gliding, low, false).advance,
            None,
            "the dwell restarted at 100"
        );
        assert!(machine.step(200, Stage::Gliding, low, false).advance.is_some());
    }

    #[test]
    fn a_dead_accel_never_reads_as_still() {
        let mut machine = StageMachine::new(Settings {
            land_timeout_ms: 1000,
            ..spec()
        });
        let nothing = SourceReadings::default();
        assert_eq!(
            run(
                &mut machine,
                Stage::Landing,
                &[(0, nothing), (500, nothing), (999, nothing)]
            ),
            Stage::Landing
        );
        let step = machine.step(1000, Stage::Landing, nothing, false);
        assert_eq!(rows(step), [(Stage::Done, "landing timeout 1 s".to_string())]);
    }

    #[test]
    fn stillness_lands_and_the_band_is_open() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Landing, acceleration(0.69), false);
        machine.step(100, Stage::Landing, acceleration(1.31), false);
        machine.step(200, Stage::Landing, acceleration(0.71), false);
        assert_eq!(
            machine.step(499, Stage::Landing, acceleration(1.29), false).advance,
            None
        );
        let step = machine.step(500, Stage::Landing, acceleration(1.0), false);
        assert_eq!(rows(step), [(Stage::Done, "stationary 1.0g".to_string())]);
    }

    /// A fresh laser reading.
    fn laser(metres: f32) -> SourceReadings {
        SourceReadings {
            height_above_ground: Some(metres),
            ..SourceReadings::default()
        }
    }

    /// Step `stage` with the same `readings` every 50 ms from `from_ms` through `to_ms`, without applying an
    /// advance (the cases end on their advance), and return the last advance.
    fn repeat(
        machine: &mut StageMachine,
        stage: Stage,
        readings: SourceReadings,
        from_ms: u32,
        to_ms: u32,
    ) -> Option<Transition> {
        (from_ms..=to_ms)
            .step_by(50)
            .filter_map(|now_ms| machine.step(now_ms, stage, readings, false).advance)
            .last()
    }

    /// A machine on `spec()` that has read a pad of `pad_m`, from 0 through 100 ms: a level held `launch_ms`.
    fn on_a_pad(pad_m: f32) -> StageMachine {
        let mut machine = StageMachine::new(spec());
        assert_eq!(repeat(&mut machine, Stage::Setting, barometer(pad_m), 0, 100), None);
        machine
    }

    #[test]
    fn the_launch_lines_are_strict() {
        // exactly launch_g (3.0 g) and exactly launch_alt_m (10.0 m) up from a 0 m pad, held: no launch; f32 holds
        // both exactly
        let mut machine = on_a_pad(0.0);
        assert_eq!(repeat(&mut machine, Stage::Setting, acceleration(3.0), 150, 450), None);
        assert_eq!(repeat(&mut machine, Stage::Setting, barometer(10.0), 500, 800), None);
        // clearly over either line, it launches
        let launch = repeat(&mut machine, Stage::Setting, acceleration(3.5), 850, 950);
        assert_eq!(
            launch.map(|step| step.reason.to_string()),
            Some("launch |a|=3.5g dwell=100ms".to_string())
        );
        // from a pad of pad_m to metres, held
        let climb = |pad_m: f32, metres: f32| {
            let high = barometer(metres);
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
        assert_eq!(repeat(&mut machine, Stage::Gliding, barometer(5.0), 0, 200), None);
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
        // a barometer's tracker and the GNSS's alike
        for source in [barometer, gnss] {
            let mut machine = StageMachine::new(Settings {
                apogee_arm_ms: 100,
                boost_timeout_ms: 5000,
                ..spec()
            });
            let ticks = [
                (0, SourceReadings::default()),
                (99, source(300.0)),  // a millisecond early: ignored, or the peak would be 300 m
                (100, source(210.0)), // armed on the dot: seeds the peak
                (150, source(200.0)),
                (200, source(200.0)),
                (250, source(170.0)),
                (300, source(170.0)),
            ];
            assert_eq!(run(&mut machine, Stage::Boosting, &ticks), Stage::Boosting);
            let step = machine.step(350, Stage::Boosting, source(170.0), false);
            assert_eq!(rows(step), [(Stage::Gliding, "apogee 210m".to_string())]);
        }
    }

    #[test]
    fn entering_setting_restarts_the_launch_dwells() {
        let (still, boost) = (acceleration(1.0), acceleration(8.0));
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, still, false);
        machine.step(50, Stage::Setting, boost, false);
        machine.step(100, Stage::Setting, boost, false);
        machine.step(110, Stage::Gliding, boost, false);
        // back on the pad 30 ms later: a kept half credit would only drain to 20 and launch at 230; restarted, the
        // dwell needs launch_ms afresh
        let step = machine.step(130, Stage::Setting, still, false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(machine.step(180, Stage::Setting, boost, false).advance, None);
        assert_eq!(machine.step(230, Stage::Setting, boost, false).advance, None);
        assert!(machine.step(280, Stage::Setting, boost, false).advance.is_some());
        // the altitude dwell: the pad must hold launch_ms afresh first, which drains any credit kept, so only a clock
        // stepping back shows one. It feeds an inactive dwell: a kept half credit would complete on the pad's reading
        let (pad, high) = (barometer(0.0), barometer(15.0));
        let mut machine = on_a_pad(0.0);
        machine.step(150, Stage::Setting, high, false);
        machine.step(200, Stage::Setting, high, false);
        machine.step(210, Stage::Gliding, high, false);
        let step = machine.step(110, Stage::Setting, pad, false);
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
        // a barometer's tracker and the GNSS's alike
        for source in [barometer, gnss] {
            let mut machine = StageMachine::new(Settings {
                apogee_arm_ms: 0,
                boost_timeout_ms: 5000,
                ..spec()
            });
            let ticks = [(0, source(240.0)), (50, source(240.0))];
            assert_eq!(run(&mut machine, Stage::Boosting, &ticks), Stage::Boosting);
            machine.step(100, Stage::Setting, barometer(0.5), false);
            // a second flight at 100 m: a kept 240 m peak would call apogee within launch_ms
            let low = source(100.0);
            assert_eq!(repeat(&mut machine, Stage::Boosting, low, 200, 400), None);
        }
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
        let step = machine.step(500, Stage::Landing, carried, false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
    }

    #[test]
    fn a_hold_ending_where_the_machine_looked_is_no_change() {
        let mut machine = StageMachine::new(spec());
        machine.step(0, Stage::Setting, acceleration(8.0), false);
        machine.step(50, Stage::Setting, acceleration(8.0), false);
        machine.step(60, Stage::Gliding, acceleration(8.0), true);
        // `stage setting` ends the hold on the stage last seen: no external row, and the kept half credit plus
        // the 4950 ms gap launch on this one sample
        let step = machine.step(5000, Stage::Setting, acceleration(8.0), false);
        assert_eq!(
            rows(step),
            [(Stage::Boosting, "launch |a|=8.0g dwell=100ms".to_string())]
        );
    }

    #[test]
    fn the_launch_reasons_plan_b_prints() {
        // both dwells complete on one tick, the elevation risen from the pad: the baro backup is checked first
        let both = SourceReadings {
            elevations: [Some(15.0), None],
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
            (barometer(0.0), barometer(15.0), "launch alt=0m dwell=100ms"),
        ] {
            let mut machine = StageMachine::new(spec());
            assert_eq!(repeat(&mut machine, Stage::Setting, pad, 850, 950), None);
            machine.step(1000, Stage::Setting, readings, false);
            machine.step(1080, Stage::Setting, readings, false);
            let step = machine.step(1060, Stage::Setting, SourceReadings::default(), false);
            assert_eq!(rows(step), [(Stage::Boosting, text.to_string())]);
        }
    }

    #[test]
    fn done_has_no_detector_and_no_backstop() {
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 100,
            ..spec()
        });
        machine.step(0, Stage::Boosting, SourceReadings::default(), false);
        machine.step(10, Stage::Done, acceleration(1.0), false);
        assert_eq!(
            machine.step(10_000, Stage::Done, acceleration(8.0), false),
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
        machine.step(start, Stage::Gliding, SourceReadings::default(), false);
        assert_eq!(
            machine.step(
                start.wrapping_add(999),
                Stage::Gliding,
                SourceReadings::default(),
                false
            ),
            Step::default()
        );
        let step = machine.step(
            start.wrapping_add(1000),
            Stage::Gliding,
            SourceReadings::default(),
            false,
        );
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
        machine.step(0, Stage::Gliding, SourceReadings::default(), false);
        assert_eq!(
            machine.step(2_147_483_646, Stage::Gliding, SourceReadings::default(), false),
            Step::default()
        );
        let step = machine.step(2_147_483_647, Stage::Gliding, SourceReadings::default(), false);
        assert_eq!(rows(step), [(Stage::Done, "flight timeout".to_string())]);
        // one ms longer, which it refuses: the signed elapsed time turns negative first, so the backstop never fires
        let mut machine = StageMachine::new(Settings {
            flight_timeout_ms: 2_147_483_648,
            ..settings
        });
        machine.step(0, Stage::Gliding, SourceReadings::default(), false);
        for now_ms in [2_147_483_647, 2_147_483_648, u32::MAX] {
            assert_eq!(
                machine.step(now_ms, Stage::Gliding, SourceReadings::default(), false),
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
            let readings = barometer(elevation_m(now_ms));
            let advance = machine.step(now_ms, Stage::Setting, readings, false).advance;
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
            let ticks: Vec<(u32, SourceReadings)> = (0..=1100)
                .step_by(50)
                .map(|now_ms| match now_ms {
                    0..=100 => (now_ms, barometer(0.0)),
                    150..1000 => (now_ms, barometer(5.0)),
                    _ => (now_ms + late_ms, barometer(11.0)),
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
        assert_eq!(machine.barometers[0].climb.count, CLIMB_SLOTS);
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
        assert_eq!(machine.barometers[0].climb.count, 42);
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
        let mut ticks: Vec<(u32, SourceReadings)> =
            (0..=1000).step_by(50).map(|now_ms| (now_ms, barometer(0.0))).collect();
        ticks.extend((0..20).map(|tick| (resumed_ms + tick * 50, barometer(11.0))));
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Setting);
        // the same climb across the clock's wrap, read every 50 ms, launches
        let start = u32::MAX - 1000;
        let ticks: Vec<(u32, SourceReadings)> = (0..40)
            .map(|tick: u32| {
                let metres = if tick < 25 { 0.0 } else { 11.0 };
                (start.wrapping_add(tick * 50), barometer(metres))
            })
            .collect();
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Boosting);
    }

    #[test]
    fn a_level_confirmed_before_the_clock_stepped_back_is_forgotten() {
        // the pad's 0 m from 1000 to 1500, then 11 m: launch_ms into the climb, it launches
        let pad = |now_ms: u32| (now_ms, barometer(0.0));
        let high = |now_ms: u32| (now_ms, barometer(11.0));
        let mut ticks: Vec<(u32, SourceReadings)> = (1000..=1500).step_by(50).map(pad).collect();
        ticks.extend((1550..=2000).step_by(50).map(high));
        assert_eq!(run(&mut gated(1000), Stage::Setting, &ticks), Stage::Boosting);
        // the same 11 m once the clock has stepped back to 600: the pad's level reads as confirmed in the future, and
        // no climb is measured from it
        let mut ticks: Vec<(u32, SourceReadings)> = (1000..=1500).step_by(50).map(pad).collect();
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
            let mut ticks: Vec<(u32, SourceReadings)> =
                (0..5000).step_by(50).map(|now_ms| (now_ms, barometer(10.5))).collect();
            ticks.extend([5000, 5000 + held_ms].map(|now_ms| (now_ms, barometer(0.0))));
            ticks.extend((5150..=6000).step_by(50).map(|now_ms| (now_ms, barometer(10.5))));
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
        let high = barometer(15.0);
        let mut machine = on_a_pad(0.0);
        machine.step(150, Stage::Setting, high, false);
        machine.step(160, Stage::Gliding, high, false);
        // back on the pad 10 ms later, still 15 m up: the pad's level before the excursion is forgotten, so no rise is
        // seen
        let step = machine.step(170, Stage::Setting, high, false);
        assert_eq!(rows(step), [(Stage::Setting, "external".to_string())]);
        assert_eq!(repeat(&mut machine, Stage::Setting, high, 220, 2000), None);
    }

    #[test]
    fn a_hold_records_no_level() {
        // the pad at 5 m; an operator hold (`stage gliding`, a fin check) reads 0 m; `stage setting` ends it where the
        // machine last looked, so nothing restarts, and 11 m is risen only 6 m above the level the machine saw
        let (low, high) = (barometer(0.0), barometer(11.0));
        let mut machine = on_a_pad(5.0);
        for now_ms in (550..=800).step_by(50) {
            assert_eq!(machine.step(now_ms, Stage::Gliding, low, true), Step::default());
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
        let ticks: Vec<(u32, SourceReadings)> = (0..80)
            .map(|tick| {
                let now_ms = tick * 50;
                let metres = 200.0 - (now_ms as f32 - 600.0).abs() * 0.1;
                let readings = SourceReadings {
                    elevations: [Some(metres.max(0.0)), None],
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
                    let step = machine.step(*now_ms, current, *readings, false);
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

    #[test]
    fn the_fused_read_takes_the_first_slots_and_the_gnss_its_own() {
        // the one-source form the control thread still steps with: plan B's fused read, slot by slot
        let fused = Readings {
            acceleration: Some(Reading {
                value: [0.0, 0.0, 1.0],
                source: Source::Other,
            }),
            elevation: Some(Reading {
                value: 12.0,
                source: Source::Other,
            }),
            height_above_ground: Some(Reading {
                value: 2.0,
                source: Source::Other,
            }),
        };
        let slots = SourceReadings::from(&fused);
        assert_eq!(slots.accelerations, [Some([0.0, 0.0, 1.0]), None, None, None]);
        assert_eq!(
            (slots.elevations, slots.gnss_elevation, slots.height_above_ground),
            ([Some(12.0), None], None, Some(2.0))
        );
        // NEGATIVE: the GNSS's elevation never sits in a barometer's slot, nor does nothing anywhere
        let gnss = Readings {
            elevation: Some(Reading {
                value: 15.0,
                source: Source::Gnss,
            }),
            ..Readings::default()
        };
        let slots = SourceReadings::from(&gnss);
        assert_eq!((slots.elevations, slots.gnss_elevation), ([None, None], Some(15.0)));
        assert_eq!(SourceReadings::from(&Readings::default()), SourceReadings::default());
        // the control thread's call, as it is written today: a fused read, by reference, steps the machine as its slots
        let mut by_reference = StageMachine::new(spec());
        let mut by_slots = StageMachine::new(spec());
        for now_ms in [0, 50, 100] {
            assert_eq!(
                by_reference.step(now_ms, Stage::Setting, &gnss, false),
                by_slots.step(now_ms, Stage::Setting, SourceReadings::from(&gnss), false)
            );
        }
    }

    #[test]
    fn a_reading_that_is_not_finite_is_no_reading() {
        let readings = SourceReadings {
            accelerations: [
                Some([0.0, f32::NAN, 1.0]),
                Some([0.0, 0.0, f32::NEG_INFINITY]),
                Some([0.0, 0.0, 1.0]),
                None,
            ],
            elevations: [Some(f32::INFINITY), Some(-3.5)],
            gnss_elevation: Some(f32::NAN),
            height_above_ground: Some(f32::NAN),
        };
        let live = readings.live();
        assert_eq!(live.accelerations, [None, None, Some([0.0, 0.0, 1.0]), None]);
        assert_eq!(
            (live.elevations, live.gnss_elevation, live.height_above_ground),
            ([None, Some(-3.5)], None, None)
        );
        // NEGATIVE: finite readings pass as they are, zero and negative ones included
        let finite = SourceReadings {
            elevations: [Some(0.0), Some(-0.0)],
            height_above_ground: Some(0.0),
            ..readings
        };
        assert_eq!(finite.live().elevations, [Some(0.0), Some(-0.0)]);
    }

    #[test]
    fn a_climb_confirms_no_level_until_one_is_held() {
        // the first readings of a barometer, and its first readings back after a gap past the window, confirm no level:
        // no rise, not even 0
        let mut climb = Climb::new();
        assert_eq!(climb.rise(0, 0.0, 1000, 100), None);
        assert_eq!(climb.rise(50, 0.0, 1000, 100), None);
        assert_eq!(climb.rise(100, 0.0, 1000, 100), Some(0.0), "held launch_ms");
        assert_eq!(climb.rise(150, 15.0, 1000, 100), Some(15.0));
        // a level above the reading: the rise is negative, not clamped
        assert_eq!(climb.rise(200, -2.0, 1000, 100), Some(-2.0));
        assert_eq!(climb.rise(1500, 3.0, 1000, 100), None, "the levels are past the window");
        assert_eq!(climb.rise(1600, 3.0, 1000, 100), Some(0.0));
    }

    #[test]
    fn vetoes_read_as_vetoes_csv_writes_them() {
        let veto = |peer: bool, physics: Option<Physics>| Veto {
            detector: Detector::Apogee,
            decider: Decider::Gnss,
            peer,
            physics,
            contrary: 0,
            live: 0,
            value: 0.0,
        };
        let kinds = [
            (veto(true, None), "peer"),
            (veto(false, Some(Physics::AtRest)), "at_rest"),
            (veto(false, Some(Physics::UnderThrust)), "thrust"),
            (veto(true, Some(Physics::AtRest)), "peer+at_rest"),
            (veto(true, Some(Physics::UnderThrust)), "peer+thrust"),
            (veto(false, None), "none"),
        ];
        for (veto, text) in kinds {
            assert_eq!(veto.kinds(), text);
        }
        let names = [
            Detector::LaunchAltitude,
            Detector::LaunchAcceleration,
            Detector::Apogee,
            Detector::Stillness,
        ]
        .map(Detector::name);
        assert_eq!(names, ["launch_altitude", "launch_acceleration", "apogee", "stillness"]);
        let held = veto(true, None);
        assert_eq!(
            (VetoEdge::Begin(held).name(), VetoEdge::End(held).name()),
            ("begin", "end")
        );
        assert_eq!(VetoEdge::End(held).veto(), &held);
    }

    #[test]
    fn a_decider_never_votes_against_itself() {
        // a launch line inside the still band (no shipped config has one): 1.25 g is over launch_g 1.2 and still. Its
        // own reading counts toward the live voters, never as contrary, so alone it launches
        let mut machine = StageMachine::new(Settings {
            launch_g: 1.2,
            ..spec()
        });
        let wobble = acceleration(1.25);
        let launch = repeat(&mut machine, Stage::Setting, wobble, 0, 100);
        assert_eq!(launch.map(|launch| launch.to), Some(Stage::Boosting));
        // NEGATIVE: a peer reading the same still 1.25 g is contrary, and holds it
        let mut machine = StageMachine::new(Settings {
            launch_g: 1.2,
            ..spec()
        });
        let mut pair = wobble;
        pair.accelerations[1] = pair.accelerations[0];
        assert_eq!(repeat(&mut machine, Stage::Setting, pair, 0, 300), None);
    }

    #[test]
    fn entering_boosting_restarts_the_apogee_dwell() {
        // the fall below the line opens the dwell at 100; a separation moves the stage to GLIDING at 150 (external),
        // and an operator's `stage boosting` brings it back at 1000. The new flight's tracker seeds afresh at 1050 (300
        // m, its own peak), and its fall opens a dwell of its own at 1100 that holds launch_ms
        let mut machine = StageMachine::new(Settings {
            boost_timeout_ms: 5000,
            ..spec()
        });
        let ticks = [(0, barometer(200.0)), (50, barometer(200.0)), (100, barometer(150.0))];
        assert_eq!(run(&mut machine, Stage::Boosting, &ticks), Stage::Boosting);
        machine.step(150, Stage::Gliding, barometer(150.0), false);
        for (now_ms, metres) in [(1000, 300.0), (1050, 300.0)] {
            assert_eq!(
                machine.step(now_ms, Stage::Boosting, barometer(metres), false).advance,
                None
            );
        }
        assert_eq!(
            machine.step(1100, Stage::Boosting, barometer(250.0), false).advance,
            None
        );
        let step = machine.step(1200, Stage::Boosting, barometer(250.0), false);
        assert_eq!(rows(step), [(Stage::Gliding, "apogee 300m".to_string())]);
    }

    #[test]
    fn a_clock_stepping_back_into_the_arming_window_blinds_the_apogee_again() {
        // the fall below the line at 100 is held under thrust (8 g on +x, seen at 100): an apogee episode opens. The
        // clock then steps back to 40, inside apogee_arm_ms: blind, the detector has no decider, and the episode ends
        let mut machine = StageMachine::new(Settings {
            boost_timeout_ms: 5000,
            ..spec()
        });
        let thrust = |metres: f32| SourceReadings {
            accelerations: [Some([8.0, 0.0, 0.0]), None, None, None],
            ..barometer(metres)
        };
        machine.step(0, Stage::Boosting, thrust(200.0), false);
        machine.step(50, Stage::Boosting, thrust(200.0), false);
        let held = machine.step(100, Stage::Boosting, thrust(150.0), false);
        assert!(matches!(held.vetoes().next(), Some(VetoEdge::Begin(veto)) if veto.detector == Detector::Apogee));
        let blind = machine.step(40, Stage::Boosting, thrust(150.0), false);
        assert!(matches!(blind.vetoes().next(), Some(VetoEdge::End(veto)) if veto.detector == Detector::Apogee));
        // NEGATIVE: armed again, still under thrust, the episode opens again
        let again = machine.step(150, Stage::Boosting, thrust(150.0), false);
        assert!(matches!(again.vetoes().next(), Some(VetoEdge::Begin(_))));
    }

    #[test]
    fn a_source_joins_only_when_away_past_launch_ms_with_a_peer_read_since() {
        // the tracker read at 100, its peak 240 m; a peer read at 150
        let read = |current: [Option<u32>; 2], now_ms: u32| {
            let mut tracker = Tracker {
                apogee: Apogee {
                    peak: Some(240.0),
                    since_ms: None,
                    smooth: Some(238.0),
                },
                current_ms: current[0],
                joined_ms: None,
            };
            tracker.feed(now_ms, 230.0, &current, 5.0, 100);
            tracker
        };
        // back exactly launch_ms on, one missed step: it holds through, smoothing on from 238
        let held = read([Some(100), Some(150)], 200);
        assert_eq!((held.apogee.smooth, held.joined_ms), (Some(236.0), None));
        // a millisecond later it joins: the smoothing restarts on the reading, the peak stays its own
        let joined = read([Some(100), Some(150)], 201);
        assert_eq!(
            (joined.apogee.smooth, joined.apogee.peak, joined.joined_ms),
            (Some(230.0), Some(240.0), Some(201))
        );
        // it sits out launch_ms exactly
        assert!(joined.joining(300, 100) && !joined.joining(301, 100));
        // NEGATIVE: with no peer read after it, it holds through any gap, as plan B's one tracker does
        let alone = read([Some(100), Some(100)], 60_000);
        assert_eq!((alone.apogee.smooth, alone.joined_ms), (Some(236.0), None));
        assert!(!alone.joining(60_000, 100));
    }

    #[test]
    fn a_flickering_accelerometer_never_sees_thrust() {
        // NEGATIVE: 8 g on +x every other step, nothing between: each missing reading drains the slot's thrust dwell
        let flicker = |now_ms: u32| SourceReadings {
            accelerations: [now_ms.is_multiple_of(100).then_some([8.0, 0.0, 0.0]), None, None, None],
            ..SourceReadings::default()
        };
        let mut machine = StageMachine::new(spec());
        for now_ms in (0..=1000).step_by(50) {
            machine.step(now_ms, Stage::Boosting, flicker(now_ms), false);
        }
        assert_eq!(machine.accelerometers[0].thrust.thrust(), Thrust::Unseen);
        // on every step it is seen, launch_ms on
        let mut machine = StageMachine::new(spec());
        for now_ms in (0..=100).step_by(50) {
            machine.step(now_ms, Stage::Boosting, flicker(0), false);
        }
        assert_eq!(machine.accelerometers[0].thrust.thrust(), Thrust::Seen);
    }

    #[test]
    fn the_machine_and_its_histories_fit_their_budget() {
        // the machine on the control thread's stack, and the climbs' histories it boxes on the heap
        let stack = size_of::<StageMachine>();
        let heap = BAROMETER_SLOTS * size_of::<Climb>();
        assert!(
            stack <= 1024 && heap <= 2048 && stack + heap <= 3 * 1024,
            "{stack} B + {heap} B"
        );
    }
}
