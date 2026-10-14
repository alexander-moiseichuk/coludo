//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The filter's orientation as a unit quaternion, body to earth (the aerospace frames: the body x forward, y right,
//! z down; the earth north, east, down), and what the filter does to it:
//!  * [`Rotation::turned`] -- the gyro's BODY rates integrated as a rotation of the body, singularity-free: through a
//!    vertical nose and a banked turn alike;
//!  * [`Rotation::yawed`] -- a turn about the earth's vertical, which moves the heading alone (the track and the
//!    magnetometer pulls);
//!  * [`Rotation::tilted`] -- a turn toward the measured gravity, the shortest way, which moves roll and pitch alone
//!    (the accelerometer pull);
//!  * [`Rotation::from_euler`] and [`Rotation::euler`] -- the aerospace heading, pitch and roll (Z-Y-X), what the
//!    databoard carries.
//!
//! Plan B integrated the body rates into the Euler angles directly (roll += p dt, pitch += q dt, yaw += r dt), which
//! holds only about a level body: in a banked turn its pitch drifts by the turn rate times the sine of the bank, and
//! at a vertical nose a spin scrambles all three. Its HITL sim published Euler rates for a gyro, which hid it.

/// A unit quaternion: the body-to-earth rotation (w, x, y, z).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Rotation {
    /// The scalar part.
    w: f32,
    /// The vector part.
    x: f32,
    y: f32,
    z: f32,
}

impl Rotation {
    /// Level, heading north.
    pub(super) const LEVEL: Rotation = Rotation {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// The orientation of the aerospace `heading` (about the earth's down axis), then `pitch` (about the turned right
    /// axis), then `roll` (about the nose), degrees.
    pub(super) fn from_euler(heading: f32, roll: f32, pitch: f32) -> Rotation {
        let (sine_yaw, cosine_yaw) = (heading.to_radians() / 2.0).sin_cos();
        let (sine_pitch, cosine_pitch) = (pitch.to_radians() / 2.0).sin_cos();
        let (sine_roll, cosine_roll) = (roll.to_radians() / 2.0).sin_cos();
        Rotation {
            w: cosine_roll * cosine_pitch * cosine_yaw + sine_roll * sine_pitch * sine_yaw,
            x: sine_roll * cosine_pitch * cosine_yaw - cosine_roll * sine_pitch * sine_yaw,
            y: cosine_roll * sine_pitch * cosine_yaw + sine_roll * cosine_pitch * sine_yaw,
            z: cosine_roll * cosine_pitch * sine_yaw - sine_roll * sine_pitch * cosine_yaw,
        }
    }

    /// The aerospace (heading, roll, pitch) in degrees: heading (-180, 180], roll (-180, 180], pitch [-90, 90]. At a
    /// vertical nose the heading and the roll share one angle; the split is the formula's, and continuous elsewhere.
    pub(super) fn euler(self) -> (f32, f32, f32) {
        let Rotation { w, x, y, z } = self;
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let pitch = (2.0 * (w * y - z * x)).clamp(-1.0, 1.0).asin();
        let heading = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        (heading.to_degrees(), roll.to_degrees(), pitch.to_degrees())
    }

    /// The earth's down axis in the body frame (unit): the gravity vector a still accelerometer reads, negated.
    pub(super) fn down(self) -> [f32; 3] {
        let Rotation { w, x, y, z } = self;
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    }

    /// This orientation after the body turned at `rate` (roll, pitch, yaw: the body rates p, q, r, degrees per
    /// second) for `elapsed` seconds, the rate held over the step.
    pub(super) fn turned(self, rate: [f32; 3], elapsed: f32) -> Rotation {
        let turn = rate.map(|axis| axis.to_radians() * elapsed);
        let angle = length(turn);
        if angle <= f32::EPSILON {
            return self;
        }
        self.then(Rotation::about(turn.map(|axis| axis / angle), angle))
    }

    /// This orientation turned about the earth's down axis by `degrees`, clockwise seen from above: the heading moves
    /// by it, the roll and the pitch (and so the body's own view of the vertical) do not.
    pub(super) fn yawed(self, degrees: f32) -> Rotation {
        Rotation::about([0.0, 0.0, 1.0], degrees.to_radians()).then(self)
    }

    /// This orientation turned `fraction` of the way that brings its down axis onto the measured `gravity` (body
    /// frame, any length), the shortest way: about an axis square to both, so the heading stays (to the second order
    /// off level). Plan B blended the roll and the pitch angles instead, unwrapped, so an estimate near inverted swept
    /// through wings-level; the shortest way never does. Exactly opposite (180 degrees, no axis), it rolls.
    pub(super) fn tilted(self, gravity: [f32; 3], fraction: f32) -> Rotation {
        let magnitude = length(gravity);
        if magnitude <= f32::EPSILON {
            return self;
        }
        let measured = gravity.map(|axis| axis / magnitude);
        let predicted = self.down();
        let axis = cross(predicted, measured);
        let sine = length(axis);
        let cosine = dot(predicted, measured);
        let angle = sine.atan2(cosine);
        let axis = if sine > 1e-6 {
            axis.map(|component| component / sine)
        } else if cosine > 0.0 {
            return self;
        } else {
            [1.0, 0.0, 0.0]
        };
        // the body turns by the opposite of the turn that moves its view of the vertical
        self.then(Rotation::about(axis, -fraction * angle))
    }

    /// The rotation of `angle` radians about the unit `axis`.
    fn about([x, y, z]: [f32; 3], angle: f32) -> Rotation {
        let (sine, cosine) = (angle / 2.0).sin_cos();
        Rotation {
            w: cosine,
            x: x * sine,
            y: y * sine,
            z: z * sine,
        }
    }

    /// `self` followed by `next` in the body frame (the Hamilton product self * next), normalised against f32's drift.
    fn then(self, next: Rotation) -> Rotation {
        let (a, b) = (self, next);
        let product = Rotation {
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
        let norm =
            (product.w * product.w + product.x * product.x + product.y * product.y + product.z * product.z).sqrt();
        Rotation {
            w: product.w / norm,
            x: product.x / norm,
            y: product.y / norm,
            z: product.z / norm,
        }
    }
}

/// The length of `vector`.
fn length(vector: [f32; 3]) -> f32 {
    dot(vector, vector).sqrt()
}

/// The dot product.
fn dot(first: [f32; 3], second: [f32; 3]) -> f32 {
    first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
}

/// The cross product, first x second.
fn cross(first: [f32; 3], second: [f32; 3]) -> [f32; 3] {
    [
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::Rotation;

    /// Whether two angles agree within `tolerance` degrees, the short way around.
    fn close(first: f32, second: f32, tolerance: f32) -> bool {
        ((first - second + 180.0).rem_euclid(360.0) - 180.0).abs() <= tolerance
    }

    #[test]
    fn euler_angles_round_trip() {
        for (heading, roll, pitch) in [
            (0.0, 0.0, 0.0),
            (30.0, 20.0, -10.0),
            (250.0, -60.0, 45.0),
            (90.0, 170.0, 5.0),
        ] {
            let (h, r, p) = Rotation::from_euler(heading, roll, pitch).euler();
            assert!(
                close(h, heading, 1e-3) && close(r, roll, 1e-3) && close(p, pitch, 1e-3),
                "{h} {r} {p}"
            );
        }
        // NEGATIVE: a pitch past vertical comes back as the same orientation, the other way round
        let (h, r, p) = Rotation::from_euler(30.0, 0.0, 167.5).euler();
        assert!(
            close(h, 210.0, 1e-2) && close(r, 180.0, 1e-2) && close(p, 12.5, 1e-2),
            "{h} {r} {p}"
        );
    }

    #[test]
    fn the_down_axis_is_the_filter_s_gravity() {
        let [x, y, z] = Rotation::from_euler(0.0, 0.0, 30.0).down();
        assert!(
            (x + 0.5).abs() < 1e-6 && y.abs() < 1e-6 && (z - 0.75_f32.sqrt()).abs() < 1e-6,
            "nose up 30"
        );
        let [x, y, z] = Rotation::from_euler(77.0, 30.0, 0.0).down();
        assert!(
            x.abs() < 1e-6 && (y - 0.5).abs() < 1e-6 && (z - 0.75_f32.sqrt()).abs() < 1e-6,
            "right wing down 30"
        );
    }

    #[test]
    fn a_coordinated_turn_keeps_its_bank_and_pitch() {
        // banked 30, nose down 6, turning right at 10 deg/s: the body rates of that turn, for 20 s
        let (bank, pitch, turn) = (30.0_f32, -6.0_f32, 10.0_f32);
        let rate = [
            -turn * pitch.to_radians().sin(),
            turn * pitch.to_radians().cos() * bank.to_radians().sin(),
            turn * pitch.to_radians().cos() * bank.to_radians().cos(),
        ];
        let mut rotation = Rotation::from_euler(40.0, bank, pitch);
        for _ in 0..1000 {
            rotation = rotation.turned(rate, 0.02);
        }
        let (h, r, p) = rotation.euler();
        assert!(
            close(h, 40.0 + 200.0, 0.05) && close(r, bank, 0.05) && close(p, pitch, 0.05),
            "{h} {r} {p}"
        );
    }

    #[test]
    fn a_spin_about_a_vertical_nose_keeps_it_vertical() {
        // a degree off a vertical nose, spinning at 300 deg/s about it (the boost's roll) for 2 s
        let mut rotation = Rotation::from_euler(30.0, 0.0, 89.0);
        for _ in 0..200 {
            rotation = rotation.turned([300.0, 0.0, 0.0], 0.01);
        }
        let [x, ..] = rotation.down();
        assert!(x < -0.999, "the nose still points up: {x}");
    }

    #[test]
    fn yawing_moves_the_heading_alone() {
        let (h, r, p) = Rotation::from_euler(350.0, 25.0, -8.0).yawed(20.0).euler();
        assert!(
            close(h, 10.0, 1e-3) && close(r, 25.0, 1e-3) && close(p, -8.0, 1e-3),
            "{h} {r} {p}"
        );
        // NEGATIVE: the body's own view of the vertical does not move
        let before = Rotation::from_euler(350.0, 25.0, -8.0).down();
        let after = Rotation::from_euler(350.0, 25.0, -8.0).yawed(20.0).down();
        assert!(before.iter().zip(after).all(|(a, b)| (a - b).abs() < 1e-6));
    }

    #[test]
    fn tilting_takes_the_short_way_and_keeps_the_heading() {
        // the estimate rolled 170, gravity says -170: 20 degrees apart the short way, never through wings-level
        let gravity = Rotation::from_euler(0.0, -170.0, 0.0).down();
        let mut rotation = Rotation::from_euler(0.0, 170.0, 0.0);
        for _ in 0..40 {
            rotation = rotation.tilted(gravity, 0.25);
            let (_, roll, _) = rotation.euler();
            assert!(roll.abs() > 165.0, "through the inverted side only: {roll}");
        }
        let (_, roll, _) = rotation.euler();
        assert!(close(roll, -170.0, 0.01), "{roll}");
        // a quarter of a 10-degree pitch error, the heading kept
        let (h, r, p) = Rotation::from_euler(120.0, 0.0, 0.0)
            .tilted(Rotation::from_euler(120.0, 0.0, 10.0).down(), 0.25)
            .euler();
        assert!(
            close(h, 120.0, 1e-3) && close(r, 0.0, 1e-3) && close(p, 2.5, 1e-3),
            "{h} {r} {p}"
        );
        // NEGATIVE: aligned, nothing moves
        let level = Rotation::LEVEL;
        assert_eq!(level.tilted([0.0, 0.0, 1.0], 0.25), level);
    }
}
