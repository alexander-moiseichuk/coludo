//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The SG90 micro fin servo (src/glider/drivers/sg90.py), and the servo logic the MG90S shares ([`crate::mg90s`]):
//! one instance per fin (yaw, left and right elevon), each on a 50 Hz PWM pin. The command unit is INTEGER DEGREES,
//! linearly mapped to a pulse width (min_us..max_us over min_deg..max_deg, integer math) and CLAMPED to the range,
//! so a bad command can never drive the horn past the linkage.
//!
//! OPEN-LOOP -- NO POSITION FEEDBACK. A 3-wire SG90 only RECEIVES a PWM command; there is no wire back, so the board
//! cannot read where the horn is. Everything reported ([`Servo::angle`], [`Servo::pulse_us`]) is the LAST COMMANDED
//! value, what was asked for, not a measurement: a stalled or jammed surface still reads its target. The one
//! closed loop available is the INA226: [`Servo::probe_verdict`] judges a probe sweep by the power it drew.
//!
//! Each fin has a `trim` (whole degrees), its MECHANICAL zero offset: a horn on a ~18-degree spline plus linkage slop
//! is never installed exactly, so each engine is zeroed individually. The trim is added to every command in the
//! degree domain before the pulse map, so boot, failsafe and control all land on the fin's true centre; the pulse is
//! then bounded to [min_us, max_us] so a large trim can never over-drive the horn.
//!
//! Python's arithmetic, reproduced (avionics-port.md §5.1): `round()` rounds a half to EVEN (sg90.py:251: 90.5 ->
//! 90, 91.5 -> 92), and the pulse map's `//` floors toward minus infinity, below min_deg included.
//!
//! The flight loop drives a servo through the mixer: [`Servo`] is a `control::mixer::Fin`, so `Mixer::actuate` sends
//! each fin its whole-degree angle and the compare-and-set decides whether anything is written.
//!
//! Not ported (avionics-port.md §6): the command coalescing (built, off by default, and no measured benefit in five
//! flown policies), and plan B's duty_u16 (the firmware's LEDC has its own resolution and takes the pulse). Not here
//! (firmware glue): the PWM itself, the shared slew gate (`fins.concurrency`), the telemetry push, the probe's sweep
//! timing and its INA reads, and the inspect JSON.

use control::mixer::Fin;
use serde_json::Value;

/// The servo frame, us: 50 Hz.
pub const PERIOD_US: i32 = 20_000;

/// The default pulse at min_deg, us (an SG90 spans ~500..2500 us).
pub const DEFAULT_MIN_US: i32 = 500;

/// The default pulse at max_deg, us.
pub const DEFAULT_MAX_US: i32 = 2_500;

/// The default command range's low end, degrees.
pub const DEFAULT_MIN_DEGREES: i32 = 0;

/// The default command range's high end, degrees.
pub const DEFAULT_MAX_DEGREES: i32 = 180;

/// The default step of the flight loop's command, whole degrees (`step_deg`; the operator, 2026-10-13): a 3-degree
/// grid. Plan B wrote any change, which is `step_deg: 1`.
pub const DEFAULT_STEP_DEGREES: i32 = 3;

/// The probe's settle before it takes the resting power baseline, ms (only with a fresh INA226).
pub const PROBE_SETTLE_MS: u32 = 300;

/// The INA226 samples the probe takes at each sweep target, one every [`PROBE_SAMPLE_MS`]: the travel window.
pub const PROBE_SAMPLES: u32 = 24;

/// The probe's sampling period at each sweep target, ms.
pub const PROBE_SAMPLE_MS: u32 = 20;

/// The defaults that belong to the servo TYPE, not the instance: a component's config still overrides the probe
/// window per fin.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ServoType {
    /// The driver name a board config selects it by (`sg90`, `mg90s`).
    pub name: &'static str,
    /// The rated slew, ms per 60 degrees: the open-loop travel estimate.
    pub slew_ms_per_60: i32,
    /// Added to the slew estimate so a move returns after the horn has settled, ms.
    pub settle_margin_ms: i32,
    /// The least power rise over the resting baseline a working, wired, powered servo shows on a probe sweep, mW.
    pub engine_min_mw: i32,
    /// The power rise above which a probe flags HIGH draw (stall or binding), mW: a flag, not a failure.
    pub engine_max_mw: i32,
    /// The pulses the part drives without stalling, us (low, high): every pulse is held inside them after the angle
    /// map, whatever the fin's range and trim ask. The SG90 holds its whole 500..2500.
    pub stall_free_us: (i32, i32),
}

/// The SG90: ~0.15 s per 60 degrees (plastic gear), probe window 500..3500 mW.
///
/// The probe thresholds are RISE OVER BASELINE, never absolute draw, and that is load bearing: the aft power bus is
/// to be rewired to serve the main MCU and the servos together, putting a steady 0.5 W or more under every reading.
/// Because the probe subtracts the settled baseline first, the rewiring shifts the baseline and leaves these numbers
/// valid. Do not "correct" them for the higher rail total.
pub const SG90: ServoType = ServoType {
    name: "sg90",
    slew_ms_per_60: 150,
    settle_margin_ms: 60,
    engine_min_mw: 500,
    engine_max_mw: 3_500,
    stall_free_us: (500, 2500),
};

/// Python's `round()` of a whole-degree command: a half to even (2.5 -> 2, -1.5 -> -2).
///
/// f64 on purpose: an operator's angle or trim arrives as a JSON number (an f64) and plan B rounds that f64; in f32
/// a value a hair above a half could land ON the tie and round the other way. The mixer's whole degrees pass through
/// unchanged.
pub fn round_half_even(degrees: f64) -> i32 {
    degrees.round_ties_even() as i32
}

/// plan B's clamp_int: `low if value < low else (high if value > high else value)`. Never panics, unlike
/// `i32::clamp`, on a range configured upside down; it answers what plan B answers.
fn clamp_int(low: i32, value: i32, high: i32) -> i32 {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}

/// Python's floor division `numerator // denominator` (toward minus infinity, either sign); the denominator is not 0.
fn floor_divide(numerator: i64, denominator: i64) -> i64 {
    let quotient = numerator / denominator;
    if numerator % denominator != 0 && ((numerator < 0) != (denominator < 0)) {
        quotient - 1
    } else {
        quotient
    }
}

/// A servo's settings, from its component block (the keys plan B's setup() reads, with its and the type's defaults).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    /// The pulse at min_deg, us (`min_us`).
    pub min_us: i32,
    /// The pulse at max_deg, us (`max_us`).
    pub max_us: i32,
    /// The command range's low end, degrees (`min_deg`).
    pub min_degrees: i32,
    /// The command range's high end, degrees (`max_deg`).
    pub max_degrees: i32,
    /// The mechanical zero offset, whole degrees (`trim`, rounded half to even).
    pub trim: i32,
    /// The angle commanded at setup, degrees (`angle`), None for the range's neutral.
    pub angle: Option<f64>,
    /// The probe's least power rise, mW (`engine_min_mw`, default the type's).
    pub engine_min_mw: i32,
    /// The probe's HIGH-draw flag, mW (`engine_max_mw`, default the type's).
    pub engine_max_mw: i32,
    /// The flight loop's grid, whole degrees (`step_deg`, [`DEFAULT_STEP_DEGREES`]; operator 2026-10-13): over 1,
    /// a fin stands on the neutral plus whole steps and leaves its point only once its command is more than half a
    /// step and a degree away ([`Servo::set_angle`]). 1 writes any change, 0 every command.
    pub step_degrees: i32,
}

impl Settings {
    /// Read a component block for a servo of `servo_type`; a missing or non-integer key takes the default.
    pub fn from_config(config: &Value, servo_type: &ServoType) -> Settings {
        let integer = |key: &str, default: i32| {
            config
                .get(key)
                .and_then(Value::as_i64)
                .and_then(|value| i32::try_from(value).ok())
                .unwrap_or(default)
        };
        let number = |key: &str| config.get(key).and_then(Value::as_f64);
        Settings {
            min_us: integer("min_us", DEFAULT_MIN_US),
            max_us: integer("max_us", DEFAULT_MAX_US),
            min_degrees: integer("min_deg", DEFAULT_MIN_DEGREES),
            max_degrees: integer("max_deg", DEFAULT_MAX_DEGREES),
            trim: round_half_even(number("trim").unwrap_or(0.0)),
            angle: number("angle"),
            engine_min_mw: integer("engine_min_mw", servo_type.engine_min_mw),
            engine_max_mw: integer("engine_max_mw", servo_type.engine_max_mw),
            step_degrees: integer("step_deg", DEFAULT_STEP_DEGREES).max(0),
        }
    }
}

/// One command written to the PWM.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Command {
    /// The commanded angle, whole degrees, before trim.
    pub angle: i32,
    /// The pulse width driven, us, trim included and bounded to [min_us, max_us].
    pub pulse_us: i32,
}

impl Command {
    /// The `<name>.csv` row: `angle`, `pulse_us`, `done` (0 when a command is issued, 1 when a move has, by its
    /// estimate, completed).
    pub fn row(&self, done: bool) -> [i32; 3] {
        [self.angle, self.pulse_us, i32::from(done)]
    }
}

/// A move under way: the command issued and how long its travel is estimated to take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Move {
    /// The command issued (row `done` 0); the same command completes it (row `done` 1) after the travel.
    pub command: Command,
    /// The estimated travel plus settle, ms ([`Servo::travel_ms`]).
    pub travel_ms: i32,
}

/// Why an operator `update` was refused; nothing has changed when it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A property that must be a number is not one.
    NotANumber(&'static str),
}

impl core::fmt::Display for Refusal {
    /// The refusal text CC carries.
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::NotANumber(key) => write!(formatter, "{key} must be a number"),
        }
    }
}

/// What a probe sweep showed, judged by the power it drew (sg90.probe's closing verdict).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProbeVerdict {
    /// No fresh INA226 power reading at the start (or it went stale while settling): an open-loop sweep, no
    /// verdict, a pass.
    OpenLoop,
    /// The INA226 went stale during the sweep: no closed-loop verdict, a pass (no reading is not a dead servo).
    Stale,
    /// The servo drew at least its type's floor: a pass; `high` flags a rise past the ceiling (stall or binding).
    Drew {
        /// The peak rise over the resting baseline, mW.
        peak_rise_mw: i32,
        /// Whether the rise passed the HIGH-draw ceiling.
        high: bool,
    },
    /// The servo drew less than the floor: dead, its PWM pin lost, or its rail unpowered -- the 'no pins lost'
    /// check. A failure.
    NoDraw {
        /// The peak rise over the resting baseline, mW.
        peak_rise_mw: i32,
    },
}

impl ProbeVerdict {
    /// The probe's failure text (plan B's, word for word), None when the verdict passes.
    pub fn failure(&self) -> Option<String> {
        match self {
            ProbeVerdict::NoDraw { peak_rise_mw } => Some(format!(
                "no current draw ({peak_rise_mw} mW rise) -- servo dead / PWM pin lost / rail unpowered?"
            )),
            _ => None,
        }
    }
}

/// One fin servo: its range, its trim, and the last command (the only position there is).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Servo {
    /// The type's defaults: slew, settle and the probe window's source.
    servo_type: ServoType,
    /// The pulse at min_deg, us.
    min_us: i32,
    /// The pulse at max_deg, us.
    max_us: i32,
    /// The command range's low end, degrees.
    min_degrees: i32,
    /// The command range's high end, degrees.
    max_degrees: i32,
    /// The range's centre, `(min_deg + max_deg) // 2`: boot, failsafe and the probe's last stop.
    neutral: i32,
    /// The mechanical zero offset, whole degrees.
    trim: i32,
    /// The probe's least power rise, mW.
    engine_min_mw: i32,
    /// The probe's HIGH-draw flag, mW.
    engine_max_mw: i32,
    /// The flight loop's grid, whole degrees (1: any change, 0: every command).
    step_degrees: i32,
    /// The last command written.
    command: Command,
}

impl Servo {
    /// A servo of `servo_type` with these settings, commanded to the setup angle (the neutral by default): plan B's
    /// setup() drives the PWM there before anything else may move it. The returned servo's [`Servo::command`] is
    /// that first command.
    pub fn new(servo_type: &ServoType, settings: &Settings) -> Servo {
        let neutral = floor_divide(i64::from(settings.min_degrees) + i64::from(settings.max_degrees), 2) as i32;
        let mut servo = Servo {
            servo_type: *servo_type,
            min_us: settings.min_us,
            max_us: settings.max_us,
            min_degrees: settings.min_degrees,
            max_degrees: settings.max_degrees,
            neutral,
            trim: settings.trim,
            engine_min_mw: settings.engine_min_mw,
            engine_max_mw: settings.engine_max_mw,
            step_degrees: settings.step_degrees,
            command: Command {
                angle: neutral,
                pulse_us: settings.min_us,
            },
        };
        servo.apply(settings.angle.unwrap_or(f64::from(neutral)));
        servo
    }

    /// The servo type.
    pub fn servo_type(&self) -> &ServoType {
        &self.servo_type
    }

    /// The last commanded angle, whole degrees (COMMANDED, never measured).
    pub fn angle(&self) -> i32 {
        self.command.angle
    }

    /// The last pulse driven, us.
    pub fn pulse_us(&self) -> i32 {
        self.command.pulse_us
    }

    /// The last command written.
    pub fn command(&self) -> Command {
        self.command
    }

    /// The mechanical zero offset, whole degrees.
    pub fn trim(&self) -> i32 {
        self.trim
    }

    /// The range's centre, degrees.
    pub fn neutral(&self) -> i32 {
        self.neutral
    }

    /// The probe's least power rise, mW.
    pub fn engine_min_mw(&self) -> i32 {
        self.engine_min_mw
    }

    /// The probe's HIGH-draw flag, mW.
    pub fn engine_max_mw(&self) -> i32 {
        self.engine_max_mw
    }

    /// A command in degrees, rounded half to even and clamped to [min_deg, max_deg] (sg90._clamp).
    pub fn clamp(&self, degrees: f64) -> i32 {
        clamp_int(self.min_degrees, round_half_even(degrees), self.max_degrees)
    }

    /// The pulse, us, for an already-clamped angle (sg90._write's map): `min_us + (angle + trim - min_deg) x
    /// (max_us - min_us) // span`, or min_us for a zero span, bounded to [min_us, max_us] and then to the type's
    /// stall-free pulses.
    pub fn pulse(&self, angle: i32) -> i32 {
        let span = i64::from(self.max_degrees) - i64::from(self.min_degrees);
        let pulse = if span != 0 {
            i64::from(self.min_us)
                + floor_divide(
                    (i64::from(angle) + i64::from(self.trim) - i64::from(self.min_degrees))
                        * (i64::from(self.max_us) - i64::from(self.min_us)),
                    span,
                )
        } else {
            i64::from(self.min_us)
        };
        let bounded = pulse.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
        let (low, high) = self.servo_type.stall_free_us;
        clamp_int(low, clamp_int(self.min_us, bounded, self.max_us), high)
    }

    /// Write an already-clamped angle (sg90._write): map it, drive it, remember it. Returns the command, whose row
    /// the caller pushes with `done` 0.
    pub fn write(&mut self, angle: i32) -> Command {
        self.command = Command {
            angle,
            pulse_us: self.pulse(angle),
        };
        self.command
    }

    /// Clamp then write (sg90._apply): the operator and move() path.
    pub fn apply(&mut self, degrees: f64) -> Command {
        let angle = self.clamp(degrees);
        self.write(angle)
    }

    /// The 100 Hz flight-loop command, compare-and-set (sg90.set_angle): clamp, then write ONLY a change. A step
    /// (`step_deg`) over 1 puts the fin on a grid of that many degrees each way from the neutral (5 over 45-135: 19
    /// positions) and moves it off its grid point only once the command is more than half a step and a degree away,
    /// so noise at a grid boundary does not toggle it. A small command then rests at the exact neutral and a held fin
    /// is never more than about half a step off its command (doc/sims/TMS-7-servo_step: on the taster's SG90s a
    /// 3-degree grid writes half the moves of step 1 and draws ~20 % less servo energy, and over the SITL it lands
    /// no worse). A step of 1 is any change, plan B's; 0 writes every command, unchanged ones too. None when nothing
    /// was written.
    pub fn set_angle(&mut self, degrees: f64) -> Option<Command> {
        let mut angle = self.clamp(degrees);
        if self.step_degrees > 1 {
            if 2 * (angle - self.command.angle).saturating_abs() < self.step_degrees + 2 {
                return None;
            }
            angle = self.on_grid(angle);
        }
        (self.step_degrees == 0 || angle != self.command.angle).then(|| self.write(angle))
    }

    /// `angle`'s nearest point of the step's grid about the neutral (a half step rounds away from it), within the
    /// range.
    fn on_grid(&self, angle: i32) -> i32 {
        let step = self.step_degrees;
        let offset = angle - self.neutral;
        let steps = (offset.saturating_abs() + step / 2) / step;
        let grid = self.neutral + offset.signum() * steps * step;
        clamp_int(self.min_degrees, grid, self.max_degrees)
    }

    /// How long a move of `degrees` occupies the horn, ms (sg90.travel_ms): the type's rated slew, plus the settle
    /// margin when `settle`. An ESTIMATE -- there is no feedback -- which is why an SG90 and an MG90S answer the same
    /// call differently.
    pub fn travel_ms(&self, degrees: i32, settle: bool) -> i32 {
        degrees.saturating_abs().saturating_mul(self.servo_type.slew_ms_per_60) / 60
            + if settle { self.servo_type.settle_margin_ms } else { 0 }
    }

    /// Start a gated move (sg90.move): the travel is estimated from the angle held BEFORE the move, then the
    /// clamped target is written. The caller holds the slew gate for the travel and then pushes the same command
    /// with `done` 1: the completion record is this snapshot, never a later angle.
    pub fn begin_move(&mut self, degrees: f64) -> Move {
        let target = self.clamp(degrees);
        let travel_ms = self.travel_ms(target - self.command.angle, true);
        Move {
            command: self.write(target),
            travel_ms,
        }
    }

    /// Apply an operator override, immediate and ungated (sg90.update): `trim` (whole degrees, rounded half to even)
    /// then `angle` (clamped). A trim alone re-drives the current angle, so CC can zero each engine live and watch
    /// the fin settle on its true centre. Returns the keys applied, in that order, and the command written, if any.
    ///
    /// A non-numeric value refuses the whole update before anything changes (plan B raised from `round()` after
    /// applying what came before it).
    pub fn update(&mut self, properties: &Value) -> Result<(Vec<&'static str>, Option<Command>), Refusal> {
        let number = |key: &'static str| -> Result<Option<f64>, Refusal> {
            properties
                .get(key)
                .map(|value| match value {
                    Value::Bool(flag) => Ok(f64::from(u8::from(*flag))),
                    _ => value.as_f64().ok_or(Refusal::NotANumber(key)),
                })
                .transpose()
        };
        let trim = number("trim")?;
        let angle = number("angle")?;
        let mut applied = Vec::new();
        if let Some(trim) = trim {
            self.trim = round_half_even(trim);
            applied.push("trim");
        }
        let command = if let Some(angle) = angle {
            applied.push("angle");
            Some(self.apply(angle))
        } else if !applied.is_empty() {
            Some(self.write(self.command.angle))
        } else {
            None
        };
        Ok((applied, command))
    }

    /// The probe sweep's stops, in order: (`min`, ...), (`max`, ...), (`neutral`, neutral). With no `limit`, plan
    /// B's full range, min_deg and max_deg. With `limit` (whole degrees, the board config's `fins.mixer.limit_deg`),
    /// neutral -/+ limit inside min_deg..max_deg: the port's own, at the operator's word (2026-10-11) -- a sweep
    /// to the servo's own ends drives the fins past the linkage's deflection range, so the wings had to be opened
    /// for every probe; the configured range is the one the fins fly.
    pub fn probe_targets(&self, limit: Option<i32>) -> [(&'static str, i32); 3] {
        let (low, high) = limit.map_or((self.min_degrees, self.max_degrees), |limit| {
            let limit = limit.max(0);
            (
                (self.neutral - limit).max(self.min_degrees),
                (self.neutral + limit).min(self.max_degrees),
            )
        });
        [("min", low), ("max", high), ("neutral", self.neutral)]
    }

    /// Judge a probe sweep (sg90.probe's close): `watching` says a fresh INA226 power baseline was taken, `sampled`
    /// how many fresh readings the sweep got, and `peak_rise_mw` the largest reading minus the baseline (the probe
    /// starts it at 0, so a fall never counts).
    pub fn probe_verdict(&self, watching: bool, sampled: u32, peak_rise_mw: i32) -> ProbeVerdict {
        if !watching {
            ProbeVerdict::OpenLoop
        } else if sampled == 0 {
            ProbeVerdict::Stale
        } else if peak_rise_mw < self.engine_min_mw {
            ProbeVerdict::NoDraw { peak_rise_mw }
        } else {
            ProbeVerdict::Drew {
                peak_rise_mw,
                high: peak_rise_mw > self.engine_max_mw,
            }
        }
    }
}

impl Fin for Servo {
    /// The mixer's whole-degree command (mixer.actuate -> sg90.set_angle): the compare-and-set
    /// [`Servo::set_angle`]. A write shows to the caller as a change of [`Servo::command`], whose row it pushes; an
    /// unchanged angle writes nothing, so a held fin costs nothing.
    fn set_angle(&mut self, angle: i32) {
        Servo::set_angle(self, f64::from(angle));
    }

    /// The angle last written, after the clamp and before the trim: always one, since construction drives the
    /// setup angle (plan B's setup() did the same before anything else could move the fin).
    fn angle(&self) -> Option<i32> {
        Some(self.command.angle)
    }
}

#[cfg(test)]
mod tests;
