//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The optional per-sensor `axes` key of the board config: how a sensor's axes map onto the airframe's. NEW in the
//! port, with no plan B module behind it (avionics-port.md §7.2 Q7, the operator's decision of 2026-10-06).
//!
//! The IMUs are mounted per their marks: the sensor's x forward, y left and z up are the airframe's, the frame every
//! `accel`, `rate` and `mag` sample is published in. Its consumers map it into their own in one place, crates/control
//! `attitude::Readings::from_marks`: the attitude filter computes in the aerospace frame (x forward, y right, z down),
//! so the (roll, pitch, yaw) rate the filter and the PID D term take is (gx, -gy, -gz), not plan B's gx -> roll,
//! gy -> pitch, gz -> yaw, which reads a part mounted per its marks nose-down-positive. Plan B had no way to state an
//! exception, and two parts publish into ONE channel: the LSM6DSO32 and the BMI323 both feed `accel` and `rate`, so a
//! part glued in turned would make a failover flip the airframe's axes in flight. The key states the exception, per
//! sensor, and the firmware maps each sample through it before publishing, so every source of a channel speaks the
//! airframe's axes.
//!
//! The value is a signed permutation, three `<sign><axis>` pairs naming, for the airframe's x, y and z in turn, the
//! sensor axis that reads along it: `"-y+x+z"` means airframe x = -sensor y, airframe y = +sensor x, airframe z =
//! +sensor z (the part turned 90 degrees about z). No key is the identity `"+x+y+z"`, so every existing board file
//! keeps its meaning.
//!
//! A MIRROR is refused: a part's axes are right-handed, a mounting is a rotation, and a rotation keeps handedness, so a
//! permutation with determinant -1 (two axes swapped without a sign, or one sign flipped alone) cannot describe any
//! mounting and is a typo. Plan B's config validation ignores keys it does not know, and the Rust validation keeps its
//! accept set, so a malformed `axes` is not a config error: the device refuses it at setup, naming the key.
//!
//! The map is applied to CONVERTED values (f32 g and degrees per second, i32 centidegree columns), never to a raw i16
//! count, whose -32768 has no positive counterpart.
//!
//! THE MAGNETOMETER IS THE EXCEPTION to "the firmware maps each sample": the BMM350 driver applies its own map,
//! AFTER its calibration ([`crate::bmm350::Bmm350::mounted`]), and the firmware must not map `mag` again. The order
//! matters there and nowhere else. The calibration is learnt from a LEVEL turn in the SENSOR's own x/y plane and
//! persisted in NVS as a 16-byte blob shared with plan B, which has no axes and so speaks sensor x/y; observing in the
//! sensor frame keeps that blob valid across a reflash both ways, and only the corrected vector is turned onto the
//! airframe's axes. And a mounting that tilts the sensor's z off the vertical ([`Axes::keeps_vertical`] false) is
//! refused for the magnetometer: its x/y plane would not be level, a level turn would barely move one of its axes,
//! and the coverage check would refuse every calibration.

use core::fmt;
use core::ops::Neg;

use serde_json::Value;

/// The config key.
pub const KEY: &str = "axes";

/// Why an `axes` value was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AxesError {
    /// The value is not a string.
    NotAString,
    /// The text is not three `<sign><axis>` pairs (`+` or `-`, then `x`, `y` or `z`).
    Malformed(String),
    /// A sensor axis is named twice, so another is never read.
    Repeated(String),
    /// The permutation is a mirror (determinant -1): no mounting produces it.
    Mirror(String),
    /// A rotation that moves the sensor's z off the airframe's vertical, on a part that calibrates in its own level
    /// x/y plane (the magnetometer): the turn would never cover its circle.
    Tilted(String),
}

impl fmt::Display for AxesError {
    /// The refusal text the device's setup failure carries.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AxesError::NotAString => write!(formatter, "axes must be a string such as '-y+x+z'"),
            AxesError::Malformed(text) => {
                write!(formatter, "axes '{text}' is not three signed axes such as '-y+x+z'")
            }
            AxesError::Repeated(text) => write!(formatter, "axes '{text}' names a sensor axis twice"),
            AxesError::Mirror(text) => write!(
                formatter,
                "axes '{text}' is a mirror, which no mounting produces (swap two axes AND flip one sign)"
            ),
            AxesError::Tilted(text) => write!(
                formatter,
                "axes '{text}' tilts the sensor's z off the vertical: a magnetometer calibrates in its own LEVEL x/y \
                 plane, so mount it flat (turned about z, or upside down)"
            ),
        }
    }
}

/// A sensor-to-airframe axis map: a signed permutation that is a rotation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Axes {
    /// For the airframe's x, y and z in turn: which sensor axis (0, 1, 2 for x, y, z) reads along it.
    source: [usize; 3],
    /// For the airframe's x, y and z in turn: whether that sensor axis reads against it.
    negated: [bool; 3],
}

impl Default for Axes {
    /// The identity: the part mounted per its marks.
    fn default() -> Axes {
        Axes::IDENTITY
    }
}

impl Axes {
    /// The part mounted per its marks: x forward, y left, z up.
    pub const IDENTITY: Axes = Axes {
        source: [0, 1, 2],
        negated: [false; 3],
    };

    /// Parse a `<sign><axis>` triple such as `"-y+x+z"`; spaces are not allowed, nor a missing sign.
    pub fn parse(text: &str) -> Result<Axes, AxesError> {
        let malformed = || AxesError::Malformed(text.to_string());
        let characters: Vec<char> = text.chars().collect();
        if characters.len() != 6 {
            return Err(malformed());
        }
        let mut source = [0usize; 3];
        let mut negated = [false; 3];
        for (index, pair) in characters.chunks(2).enumerate() {
            negated[index] = match pair[0] {
                '+' => false,
                '-' => true,
                _ => return Err(malformed()),
            };
            source[index] = match pair[1] {
                'x' => 0,
                'y' => 1,
                'z' => 2,
                _ => return Err(malformed()),
            };
        }
        if source[0] == source[1] || source[1] == source[2] || source[0] == source[2] {
            return Err(AxesError::Repeated(text.to_string()));
        }
        let axes = Axes { source, negated };
        if axes.determinant() < 0 {
            return Err(AxesError::Mirror(text.to_string()));
        }
        Ok(axes)
    }

    /// The map a sensor's config block states: its `axes` key, or the identity when there is none (or it is null).
    pub fn from_config(config: &Value) -> Result<Axes, AxesError> {
        match config.get(KEY) {
            None | Some(Value::Null) => Ok(Axes::IDENTITY),
            Some(Value::String(text)) => Axes::parse(text),
            Some(_) => Err(AxesError::NotAString),
        }
    }

    /// The determinant of the signed permutation: +1 for a rotation, -1 for a mirror.
    fn determinant(&self) -> i32 {
        // a permutation's sign is its count of inversions, and each negated axis flips it once more
        let inversions = (0..3)
            .flat_map(|first| (first + 1..3).map(move |second| (first, second)))
            .filter(|&(first, second)| self.source[first] > self.source[second])
            .count();
        let flips = self.negated.iter().filter(|negated| **negated).count();
        if (inversions + flips).is_multiple_of(2) { 1 } else { -1 }
    }

    /// Whether the airframe's z reads the sensor's z, either way up: the part is turned about z or flipped over, so
    /// its x/y plane stays level. A magnetometer needs it ([`AxesError::Tilted`]).
    pub fn keeps_vertical(&self) -> bool {
        self.source[2] == 2
    }

    /// Map one sensor vector onto the airframe's axes.
    pub fn apply<T: Copy + Neg<Output = T>>(&self, vector: [T; 3]) -> [T; 3] {
        [0, 1, 2].map(|axis| {
            let value = vector[self.source[axis]];
            if self.negated[axis] { -value } else { value }
        })
    }
}

impl fmt::Display for Axes {
    /// The config text this map parses from (`+x+y+z` for the identity), so a refusal can name it.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for axis in 0..3 {
            let sign = if self.negated[axis] { '-' } else { '+' };
            let name = ['x', 'y', 'z'][self.source[axis]];
            write!(formatter, "{sign}{name}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::{Axes, AxesError};
    use serde_json::json;

    #[test]
    fn no_key_is_the_identity() {
        assert_eq!(Axes::from_config(&json!({"name": "imu"})), Ok(Axes::IDENTITY));
        assert_eq!(Axes::from_config(&json!({"axes": null})), Ok(Axes::IDENTITY));
        assert_eq!(Axes::parse("+x+y+z"), Ok(Axes::IDENTITY));
        assert_eq!(Axes::IDENTITY.apply([1.0_f32, 2.0, 3.0]), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn a_part_turned_about_z() {
        // airframe x = -sensor y, airframe y = +sensor x, airframe z = +sensor z
        let axes = Axes::from_config(&json!({"axes": "-y+x+z"})).unwrap();
        assert_eq!(axes.apply([1, 2, 3]), [-2, 1, 3]);
        assert_eq!(axes.apply([0.5_f32, -0.25, 1.0]), [0.25, 0.5, 1.0]);
    }

    #[test]
    fn a_part_upside_down() {
        // turned 180 degrees about x: y and z both reversed (two flips keep the handedness)
        let axes = Axes::parse("+x-y-z").unwrap();
        assert_eq!(axes.apply([1, 2, 3]), [1, -2, -3]);
        assert_eq!(
            Axes::parse("+y+z+x").unwrap().apply([1, 2, 3]),
            [2, 3, 1],
            "a cyclic turn is a rotation"
        );
    }

    #[test]
    fn every_rotation_is_accepted_and_every_mirror_refused() {
        // 6 permutations x 8 sign patterns: exactly half (the 24 rotations of a cube) are mountings
        let mut rotations = 0;
        for order in ["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"] {
            for signs in 0..8 {
                let text: String = order
                    .chars()
                    .enumerate()
                    .flat_map(|(index, axis)| [if signs & (1 << index) != 0 { '-' } else { '+' }, axis])
                    .collect();
                match Axes::parse(&text) {
                    Ok(_) => rotations += 1,
                    Err(AxesError::Mirror(_)) => {}
                    Err(other) => panic!("{text}: {other}"),
                }
            }
        }
        assert_eq!(rotations, 24);
    }

    #[test]
    fn a_map_prints_as_the_text_it_parses_from() {
        for text in ["+x+y+z", "-y+x+z", "+x-y-z", "+y+z+x", "+x+z-y"] {
            assert_eq!(Axes::parse(text).unwrap().to_string(), text);
        }
    }

    #[test]
    fn a_flat_mounting_keeps_the_vertical_and_a_tilted_one_does_not() {
        // turned about z, or upside down: the x/y plane is still level
        for flat in ["+x+y+z", "-y+x+z", "-x-y+z", "+x-y-z", "+y+x-z"] {
            assert!(Axes::parse(flat).unwrap().keeps_vertical(), "{flat}");
        }
        // NEGATIVE: on its side or on its end, the sensor's z lies along the airframe's x or y
        for tilted in ["+x+z-y", "+z+y-x", "+y+z+x", "-z+x-y"] {
            assert!(!Axes::parse(tilted).unwrap().keeps_vertical(), "{tilted}");
        }
    }

    #[test]
    fn typos_are_refused_by_name() {
        // NEGATIVE: the most likely slips -- a swap without a sign, a missing sign, a repeated axis, a non-string
        assert!(matches!(Axes::parse("+y+x+z"), Err(AxesError::Mirror(_))));
        assert!(matches!(Axes::parse("-x+y+z"), Err(AxesError::Mirror(_))));
        assert!(matches!(Axes::parse("y+x+z"), Err(AxesError::Malformed(_))));
        assert!(matches!(Axes::parse("+x+x+z"), Err(AxesError::Repeated(_))));
        assert!(matches!(Axes::parse("+x +y+z"), Err(AxesError::Malformed(_))));
        assert!(matches!(Axes::parse("+X+Y+Z"), Err(AxesError::Malformed(_))));
        assert!(matches!(Axes::parse("+x+y+w"), Err(AxesError::Malformed(_))));
        assert!(matches!(Axes::parse(""), Err(AxesError::Malformed(_))));
        assert!(matches!(Axes::parse("+x+y+z+"), Err(AxesError::Malformed(_))));
        assert_eq!(Axes::from_config(&json!({"axes": 3})), Err(AxesError::NotAString));
        assert_eq!(
            AxesError::Mirror("+y+x+z".to_string()).to_string(),
            "axes '+y+x+z' is a mirror, which no mounting produces (swap two axes AND flip one sign)"
        );
    }
}
