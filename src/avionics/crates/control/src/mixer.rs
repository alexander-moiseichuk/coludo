//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The control-surface mixer (src/glider mixer.py), sibling of [`crate::pid`] and [`crate::governor`]. It maps the
//! control axes -- roll, pitch and yaw, each a deflection command in degrees -- to a servo angle per fin, for the
//! airframe's mixing: ELEVONS (the two elerons move together for pitch and differentially for roll) and a RUDDER
//! (the yaw fin), with a hard +/- limit on the control deflection.
//!
//! Signs are config (the `surfaces` gains), set during bench alignment: a surface that deflects the wrong way gets
//! its gain sign flipped. MECHANICAL NEUTRAL (each fin's true zero) is NOT here: it is the servo driver's `trim`,
//! per fin, so boot, failsafe and control all land on it alike. The mixer commands one COMMON neutral and the
//! driver offsets each fin to its centre.
//!
//! The fins take WHOLE degrees, as plan B's sg90 driver does (its PWM path is integer, and its compare-and-set
//! skips a fin whose angle did not change). Plan B mixed integer commands -- the flight task floored each PID
//! output to whole degrees first (`// fixed.SCALE`) -- and sg90's `_clamp` rounded whatever reached it with
//! Python's `round()`, half to even. This mixer takes the f32 commands and rounds each fin's angle the same way,
//! half to even, where the sum is formed: an integer command mixes exactly as in plan B, and a fractional one
//! lands on the angle plan B's driver would have set. Whether the flight loop keeps plan B's floor (it biases
//! every command half a degree low on average, and sends -0.01 degrees as -1) is the flight port's to decide.
//!
//! Plan B's zero-allocation shapes (the pre-resolved surface tuples, the shared output dict, `actuate` fused into
//! the mixing loop) exist for MicroPython's GC; here [`Mixer::mix`] rewrites one buffer and allocates nothing.

use serde_json::Value;

use crate::commons::{SERVO_NEUTRAL_DEGREES, between};

/// The default control-deflection limit in degrees (mixer.py `limit_deg`): the fins' mechanical throw.
const DEFAULT_LIMIT_DEGREES: i32 = 45;

/// One control surface: its fin's name and how much of each axis command it takes (mixer.py `surfaces`).
#[derive(Clone, Debug, PartialEq)]
pub struct Surface {
    /// The fin's component name in the board config, such as `servo_eleron_left`.
    pub name: String,
    /// Degrees of this fin per degree of roll command.
    pub roll: f32,
    /// Degrees of this fin per degree of pitch command.
    pub pitch: f32,
    /// Degrees of this fin per degree of yaw command.
    pub yaw: f32,
}

impl Surface {
    /// A surface with these gains.
    fn new(name: &str, roll: f32, pitch: f32, yaw: f32) -> Surface {
        Surface {
            name: name.to_string(),
            roll,
            pitch,
            yaw,
        }
    }

    /// The surface a config entry `{name: {roll, pitch, yaw}}` describes; a missing or non-numeric gain is 0, as
    /// plan B's `gains.get(axis, 0)` reads a missing one.
    fn from_config(name: &str, gains: &Value) -> Surface {
        let gain = |axis: &str| gains.get(axis).and_then(Value::as_f64).unwrap_or(0.0) as f32;
        Surface::new(name, gain("roll"), gain("pitch"), gain("yaw"))
    }
}

/// A fin servo the mixer drives: plan B's "object with set_angle(angle)", the sg90 driver in flight.
///
/// NOT YET HERE: plan B's actuate() also called each fin's `settle()` after `set_angle()`, so sg90's slew
/// coalescing writes a command it folded mid-slew once the horn arrives. That is the servo driver's behaviour, and
/// it joins this trait with the sg90 port.
pub trait Fin {
    /// Command this whole-degree angle. The driver adds its mechanical trim and clamps to its own physical range.
    fn set_angle(&mut self, angle: i32);

    /// The whole-degree angle the driver last wrote to its servo -- after its own range clamp, before its trim --
    /// or None before the first: plan B's sg90 `angle` attribute, which [`Mixer::angles`] reads for the operator.
    fn angle(&self) -> Option<i32>;
}

/// An axis command the mix can use: a finite one as it is, a non-finite one (NaN or an infinity) as 0, which leaves
/// that axis at neutral (see [`Mixer::mix`]).
fn finite_or_neutral(command: f32) -> f32 {
    if command.is_finite() { command } else { 0.0 }
}

/// Mix (roll, pitch, yaw) deflection commands into a whole-degree angle per fin (mixer.Mixer).
///
/// `angle = round_half_even(neutral + between(-limit, roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw,
/// limit))`. Each fin's mechanical zero is its driver's trim, applied downstream.
#[derive(Clone, Debug, PartialEq)]
pub struct Mixer {
    /// The common neutral angle in whole degrees: plan B's `int(neutral_deg)`, truncated toward zero.
    neutral: i32,
    /// The largest control deflection from neutral, in whole degrees, per surface. OWNERSHIP: the config only
    /// seeds the pre-flight value; in flight the governor OWNS it (rewrites it every update, as 1/v^2 of the
    /// airspeed) and the mixing reads it. Nothing else may write it -- a "configurable limit" would silently
    /// fight the dynamic-pressure governor -- so its setter is crate-private.
    limit: i32,
    /// The surfaces in mixing order: plan B's default order, or the config's key order (sorted, as serde_json's
    /// map keeps it; plan B keeps the order written).
    surfaces: Vec<Surface>,
    /// The last mixed angle per surface, parallel to `surfaces`: the buffer [`Mixer::mix`] rewrites and lends.
    angles: Vec<i32>,
}

impl Default for Mixer {
    /// The default mixing: neutral 90, limit 45, elevons and a rudder.
    fn default() -> Mixer {
        Mixer::new(&Value::Null)
    }
}

impl Mixer {
    /// The mixer a board's `fins.mixer` config block describes (mixer.Mixer(config)); null or a missing key takes
    /// plan B's default.
    ///
    /// Keys: `neutral_deg` (default 90, truncated to whole degrees), `limit_deg` (default 45; whole degrees, as the
    /// governor writes it -- plan B kept a fractional config value until the governor's first update), and
    /// `surfaces`, `{name: {roll, pitch, yaw}}` (default: `servo_yaw` takes yaw, `servo_eleron_left` takes pitch
    /// and roll, `servo_eleron_right` takes pitch and minus roll). A value that is not a number takes the default;
    /// validate() refuses such a config before it gets here.
    pub fn new(config: &Value) -> Mixer {
        let whole_degrees = |key: &str, default: i32| {
            config
                .get(key)
                .and_then(Value::as_f64)
                .map_or(default, |degrees| degrees as i32)
        };
        let surfaces: Vec<Surface> = match config.get("surfaces").and_then(Value::as_object) {
            Some(surfaces) => surfaces
                .iter()
                .map(|(name, gains)| Surface::from_config(name, gains))
                .collect(),
            None => vec![
                Surface::new("servo_yaw", 0.0, 0.0, 1.0),
                Surface::new("servo_eleron_left", 1.0, 1.0, 0.0),
                Surface::new("servo_eleron_right", -1.0, 1.0, 0.0),
            ],
        };
        let neutral = whole_degrees("neutral_deg", SERVO_NEUTRAL_DEGREES);
        Mixer {
            neutral,
            limit: whole_degrees("limit_deg", DEFAULT_LIMIT_DEGREES),
            angles: vec![neutral; surfaces.len()],
            surfaces,
        }
    }

    /// The common neutral angle in whole degrees.
    pub fn neutral(&self) -> i32 {
        self.neutral
    }

    /// The control-deflection limit in whole degrees: the config's until the governor's first update, the
    /// dynamic-pressure cap after (governor.Governor.cap reads it here too).
    pub fn limit(&self) -> i32 {
        self.limit
    }

    /// Rewrite the limit: the governor's alone (see the `limit` field).
    pub(crate) fn set_limit(&mut self, limit: i32) {
        self.limit = limit;
    }

    /// The surfaces, in the order [`Mixer::mix`] answers and [`Mixer::actuate`] expects the fins.
    pub fn surfaces(&self) -> &[Surface] {
        &self.surfaces
    }

    /// The whole-degree angle per surface for these axis commands in degrees (mixer.Mixer.mix, then sg90's
    /// round()).
    ///
    /// Only the CONTROL AUTHORITY is clamped here, to +/- limit, and a combined demand is clamped as a whole (the
    /// total surface deflection is bounded). The absolute PHYSICAL endpoint is each fin driver's own range: the
    /// safe travel is per linkage, not one global bound (a horn that binds at 135 degrees gets max_deg=135).
    /// Returns the angles in [`Mixer::surfaces`] order, lent from a buffer the next call rewrites.
    ///
    /// NON-FINITE COMMANDS, which plan B never mixed: its commands were whole-degree integers. In f32 the
    /// authority clamp does not hold for NaN (`between` passes it through, and `NaN as i32` saturates to 0), so a
    /// NaN roll would command EVERY fin to 0 degrees, 90 off neutral and twice the 45 degree limit; and an
    /// infinite one, multiplied by the zero gain other fins hold for that axis (0 x inf is NaN), would throw the
    /// rudder hard over on a roll command. So a non-finite axis command counts as 0 -- that axis neutral, plan B's
    /// degraded rule for a stale or absent attitude -- while the finite axes keep their authority. With every axis
    /// finite a zero gain contributes exactly 0, so no term needs skipping; the one NaN left, finite products
    /// overflowing to opposite infinities, is a neutral deflection too (a single infinity clamps to the limit, as
    /// Python's float overflow did).
    pub fn mix(&mut self, roll: f32, pitch: f32, yaw: f32) -> &[i32] {
        let limit = self.limit as f32;
        let (roll, pitch, yaw) = (
            finite_or_neutral(roll),
            finite_or_neutral(pitch),
            finite_or_neutral(yaw),
        );
        for (surface, angle) in self.surfaces.iter().zip(self.angles.iter_mut()) {
            let demand = surface.roll * roll + surface.pitch * pitch + surface.yaw * yaw;
            let deflection = if demand.is_nan() {
                0.0
            } else {
                between(-limit, demand, limit)
            };
            *angle = (self.neutral as f32 + deflection).round_ties_even() as i32;
        }
        &self.angles
    }

    /// The neutral angle of every fin (mixer.Mixer.neutralise): the safe, control-disabled output.
    pub fn neutralise(&mut self) -> &[i32] {
        self.mix(0.0, 0.0, 0.0)
    }

    /// Mix and command each fin in one pass (mixer.Mixer.actuate).
    ///
    /// `fins` is parallel to [`Mixer::surfaces`]: the driver resolved for each surface, or None where none was.
    /// A surface without a fin is skipped, as plan B's bind() tolerated, and the bound ones still drive -- which
    /// is exactly why [`Mixer::missing`] exists.
    pub fn actuate<F: Fin>(&mut self, fins: &mut [Option<F>], roll: f32, pitch: f32, yaw: f32) {
        let angles = self.mix(roll, pitch, yaw);
        for (fin, angle) in fins.iter_mut().zip(angles) {
            if let Some(fin) = fin {
                fin.set_angle(*angle);
            }
        }
    }

    /// The surfaces with no fin to drive in `fins` (mixer.Mixer.missing), in [`Mixer::surfaces`] order.
    ///
    /// SAY WHICH SURFACES DID NOT BIND: skipping them silently is what makes a missing fin dangerous. Such a
    /// surface holds whatever angle it was last left at while the others fly, so the airframe loses a third of
    /// its control authority, and nothing reports it: inspect showed the commanded angles, verify was clean,
    /// the capture looked normal. Not hypothetical: servo_eleron_right died on the bench and TWO full
    /// 12-scenario matrix runs completed before anything noticed, and only because the INA226 rail current
    /// happened to be plotted. An airframe without a power monitor (7C) gives no signal at all. So verify and
    /// inspect fail on this list instead of the operator inferring it from a current trace.
    pub fn missing<'mixer, F>(&'mixer self, fins: &[Option<F>]) -> Vec<&'mixer str> {
        self.surfaces
            .iter()
            .enumerate()
            .filter(|(index, _)| !matches!(fins.get(*index), Some(Some(_))))
            .map(|(_, surface)| surface.name.as_str())
            .collect()
    }

    /// The angle each fin in `fins` was last commanded, by surface name, in [`Mixer::surfaces`] order
    /// (mixer.Mixer.angles): the health vitals' `fins`.
    ///
    /// For the OPERATOR view, called at the vitals rate (~0.5 Hz in plan B), never from the control step: it
    /// allocates. It reads each DRIVER's stored command ([`Fin::angle`]) rather than this mixer's last mix, so
    /// what the panel shows is literally what the servo was told -- a command the driver clamped to its linkage
    /// shows clamped, and a fin never commanded shows None. Only the surfaces with a fin are listed, as plan B
    /// lists only the bound ones; [`Mixer::missing`] names the rest.
    pub fn angles<'mixer, F: Fin>(&'mixer self, fins: &[Option<F>]) -> Vec<(&'mixer str, Option<i32>)> {
        self.surfaces
            .iter()
            .zip(fins)
            .filter_map(|(surface, fin)| Some((surface.name.as_str(), fin.as_ref()?.angle())))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::{Fin, Mixer};
    use serde_json::json;

    /// A fin that remembers its last command (plan B's test `_Fin`).
    #[derive(Debug, Default)]
    struct RecordingFin {
        /// The last commanded angle; None before the first.
        angle: Option<i32>,
    }

    impl Fin for RecordingFin {
        fn set_angle(&mut self, angle: i32) {
            self.angle = Some(angle);
        }

        fn angle(&self) -> Option<i32> {
            self.angle
        }
    }

    /// A fin whose linkage binds at [80, 100] degrees: it stores the clamped command, as sg90 does.
    #[derive(Debug, Default)]
    struct BindingFin {
        /// The last angle written, after the clamp; None before the first.
        angle: Option<i32>,
    }

    impl Fin for BindingFin {
        fn set_angle(&mut self, angle: i32) {
            self.angle = Some(angle.clamp(80, 100));
        }

        fn angle(&self) -> Option<i32> {
            self.angle
        }
    }

    /// The default mixer's (yaw, left, right) angles for these commands.
    fn mixed(roll: f32, pitch: f32, yaw: f32) -> Vec<i32> {
        Mixer::default().mix(roll, pitch, yaw).to_vec()
    }

    #[test]
    fn the_default_mixes_elevons_and_a_rudder() {
        let mut mixer = Mixer::default();
        let names: Vec<&str> = mixer.surfaces().iter().map(|surface| surface.name.as_str()).collect();
        assert_eq!(names, ["servo_yaw", "servo_eleron_left", "servo_eleron_right"]);
        assert_eq!(mixer.neutralise(), [90, 90, 90]);
        assert_eq!(
            mixed(0.0, 10.0, 0.0),
            [90, 100, 100],
            "pitch moves both elevons together"
        );
        assert_eq!(mixed(10.0, 0.0, 0.0), [90, 100, 80], "roll moves them differentially");
        assert_eq!(mixed(0.0, 0.0, 15.0), [105, 90, 90], "yaw moves the rudder only");
        assert_eq!(mixed(5.0, 10.0, 0.0), [90, 105, 95]);
    }

    #[test]
    fn the_control_deflection_is_clamped_as_a_whole() {
        assert_eq!(mixed(0.0, 100.0, 0.0)[1], 135);
        assert_eq!(mixed(0.0, 0.0, -100.0)[0], 45);
        assert_eq!(mixed(40.0, 40.0, 0.0)[1], 135, "80 degrees combined clamps to 45");
        let mut tight = Mixer::new(&json!({"neutral_deg": 90, "limit_deg": 30, "surfaces": {"servo_yaw": {"yaw": 1}}}));
        assert_eq!(tight.mix(0.0, 0.0, 0.0), [90]);
        assert_eq!(tight.mix(0.0, 0.0, 100.0), [120]);
        assert_eq!(tight.mix(0.0, 0.0, -100.0), [60]);
    }

    #[test]
    fn a_fractional_angle_rounds_half_to_even_like_python() {
        // the parity is the angle's, not the deflection's: 90.5 -> 90 but 91.5 -> 92
        assert_eq!(mixed(0.0, 0.5, 0.0), [90, 90, 90]);
        assert_eq!(mixed(0.0, 1.5, 0.0), [90, 92, 92]);
        assert_eq!(mixed(0.0, -0.5, 0.0), [90, 90, 90]);
        assert_eq!(mixed(0.0, 0.0, 2.25), [92, 90, 90]);
        let mut odd = Mixer::new(&json!({"neutral_deg": 91}));
        assert_eq!(odd.mix(0.0, 0.5, 0.0), [91, 92, 92]);
    }

    #[test]
    fn the_governor_alone_moves_the_limit() {
        let mut mixer = Mixer::default();
        assert_eq!(mixer.limit(), 45);
        mixer.set_limit(8);
        assert_eq!(mixer.mix(0.0, 100.0, 0.0), [90, 98, 98]);
    }

    #[test]
    fn config_values_read_as_plan_b_reads_them() {
        let mixer = Mixer::new(&json!({"neutral_deg": 90.7, "limit_deg": 30.0, "surfaces": {"a": {"roll": 2}}}));
        assert_eq!((mixer.neutral(), mixer.limit()), (90, 30), "whole degrees, truncated");
        assert_eq!(
            (mixer.surfaces()[0].roll, mixer.surfaces()[0].pitch),
            (2.0, 0.0),
            "a missing gain is 0"
        );
        // NEGATIVE: a key of the wrong type falls back to the default rather than poisoning the mix
        let mixer = Mixer::new(&json!({"neutral_deg": "ninety", "limit_deg": null}));
        assert_eq!((mixer.neutral(), mixer.limit()), (90, 45));
        assert_eq!(mixer.surfaces().len(), 3);
    }

    #[test]
    fn actuate_drives_the_bound_fins_and_skips_the_missing() {
        let mut mixer = Mixer::default();
        let mut fins: Vec<Option<RecordingFin>> =
            vec![Some(RecordingFin::default()), Some(RecordingFin::default()), None];
        mixer.actuate(&mut fins, 0.0, 10.0, 0.0);
        assert_eq!(fins[0].as_ref().unwrap().angle, Some(90));
        assert_eq!(fins[1].as_ref().unwrap().angle, Some(100));
        assert_eq!(mixer.missing(&fins), ["servo_eleron_right"]);
        // a fins slice shorter than the surfaces leaves the tail undriven, and says so
        let short: Vec<Option<RecordingFin>> = vec![Some(RecordingFin::default())];
        assert_eq!(mixer.missing(&short), ["servo_eleron_left", "servo_eleron_right"]);
        let all: Vec<Option<RecordingFin>> = (0..3).map(|_| Some(RecordingFin::default())).collect();
        assert!(mixer.missing(&all).is_empty());
    }

    #[test]
    fn a_non_finite_command_leaves_its_axis_neutral_and_the_others_flying() {
        // NEGATIVE: NaN once commanded every fin to 0 degrees (NaN as i32), and an infinite roll threw the rudder
        // over (0 x inf); the clamp must hold, the axis must go neutral and the finite axes must keep flying
        for garbage in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
            assert_eq!(mixed(garbage, 0.0, 0.0), [90, 90, 90], "roll {garbage}");
            assert_eq!(mixed(0.0, garbage, 0.0), [90, 90, 90], "pitch {garbage}");
            assert_eq!(mixed(0.0, 0.0, garbage), [90, 90, 90], "yaw {garbage}");
            assert_eq!(
                mixed(garbage, 10.0, 15.0),
                [105, 100, 100],
                "the finite axes, with roll {garbage}"
            );
            for angle in mixed(garbage, garbage, garbage) {
                assert_eq!(angle, 90, "{garbage} on every axis");
            }
        }
        // POSITIVE: a huge but finite command still saturates at the limit, as plan B's float overflow did
        assert_eq!(mixed(f32::MAX, 0.0, 0.0), [90, 135, 45]);
        // and two finite products that overflow to opposite infinities are no demand, not a NaN angle
        let mut opposed = Mixer::new(&json!({"surfaces": {"servo_yaw": {"roll": 2, "pitch": -2}}}));
        assert_eq!(opposed.mix(f32::MAX, f32::MAX, 0.0), [90]);
    }

    #[test]
    fn angles_read_what_each_driver_was_told() {
        let mut mixer = Mixer::default();
        let mut fins: Vec<Option<BindingFin>> = vec![Some(BindingFin::default()), None, Some(BindingFin::default())];
        assert_eq!(
            mixer.angles(&fins),
            [("servo_yaw", None), ("servo_eleron_right", None)],
            "bound but never commanded: None; the missing surface is not listed"
        );
        mixer.actuate(&mut fins, 0.0, 30.0, 4.0);
        // the mix asked 94 of the rudder and 120 of the right elevon; the right elevon's linkage stopped it at 100,
        // and the operator sees 100, not the re-derived mix
        assert_eq!(mixer.mix(0.0, 30.0, 4.0)[2], 120);
        assert_eq!(
            mixer.angles(&fins),
            [("servo_yaw", Some(94)), ("servo_eleron_right", Some(100))]
        );
        // NEGATIVE: no fins at all lists nothing
        let none: Vec<Option<BindingFin>> = Vec::new();
        assert!(mixer.angles(&none).is_empty());
    }
}
