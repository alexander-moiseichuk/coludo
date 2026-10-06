//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Landing-zone navigation geometry (src/glider navigation.py). For now the part the Mission reads: [`distance`],
//! [`zone`] (the target and the two short-side gates) and [`zone_aspect`].
//!
//! Equirectangular (flat-earth) math, as in plan B: "not exact but about", which is plenty at zone scale
//! (under a kilometre or so).
//!
//! Positions are integers. A [`Position`] counts 1e-7 degree units, 1.1 cm along a meridian. A float degree
//! cannot hold that: plan B is single precision on the board, where an f32 ulp at latitude 48 is 3.8e-6 degree
//! (0.42 m), so a 0.2 m step added to it is exactly zero. The geometry therefore never forms an f32 degree of
//! latitude or longitude. It subtracts the integers (exact), and only the difference becomes f32 metres; at
//! zone scale that is a few thousand units, exact in f32 up to 2^24 units (186 km). The one absolute angle
//! left is the mean latitude a cosine is taken of. Rounding it to f32 moves the cosine by up to about
//! tan(latitude) x 2e-7 of itself, and never less than the cosine's own rounding (3e-8): the worst measured
//! is 1.5e-7 between 45 and 50 degrees, 7e-7 between 70 and 75, and 2e-6 between 80 and 85, growing without
//! bound toward a pole as the cosine itself goes to zero. Only the east-west part of a distance takes it. Over
//! 5 km a distance stays within about 1 mm of plan B's CPython f64 answer at mid latitudes (at most 2.6e-7 of
//! it up to 50 degrees) and within 5 mm up to 80 degrees, with no f64 at all (decision D18), closer than plan
//! B's own board gets with its f32 degrees.

/// Metres per degree of latitude, and per degree of longitude times cos(latitude) (src/glider
/// `commons.M_PER_DEG`).
pub const METRES_PER_DEGREE: f32 = 111_320.0;

/// Position units per degree: a [`Position`] counts 1e-7 degrees. f64, because it is only used to turn a
/// JSON coordinate (an f64) into units and back.
const UNITS_PER_DEGREE: f64 = 1.0e7;

/// Metres per position unit along a meridian (1.1132 cm): what an integer difference of latitudes is scaled by.
const METRES_PER_UNIT: f32 = METRES_PER_DEGREE / 1.0e7;

/// 180 degrees in position units: where a longitude difference wraps.
const HALF_TURN_UNITS: i64 = 1_800_000_000;

/// A point on the globe in 1e-7 degree units. Every position takes this integer form, so it keeps its
/// centimetre at any latitude (see the module notes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Position {
    /// Latitude in 1e-7 degree, north positive: -900_000_000..=900_000_000 on the globe.
    pub latitude_e7: i32,
    /// Longitude in 1e-7 degree, east positive: -1_800_000_000..=1_800_000_000 on the globe.
    pub longitude_e7: i32,
}

impl Position {
    /// The position nearest these decimal degrees, as JSON carries them (f64, the boundary decision D18
    /// allows).
    ///
    /// The caller range-checks first. A coordinate off the globe means nothing here, and one past the i32
    /// range saturates.
    pub fn from_degrees(latitude: f64, longitude: f64) -> Position {
        Position {
            latitude_e7: e7_from_degrees(latitude),
            longitude_e7: e7_from_degrees(longitude),
        }
    }

    /// `[latitude, longitude]` in decimal degrees, for a JSON answer (f64, the boundary decision D18 allows).
    pub fn degrees(self) -> [f64; 2] {
        [degrees_from_e7(self.latitude_e7), degrees_from_e7(self.longitude_e7)]
    }
}

/// The 1e-7 degree units nearest `degrees`: how a JSON coordinate (an f64) enters. Rounding to the nearest
/// unit gives back exactly the seven decimals a launch.config carries.
pub fn e7_from_degrees(degrees: f64) -> i32 {
    (degrees * UNITS_PER_DEGREE).round() as i32
}

/// Decimal degrees of `e7` units, for a JSON answer.
///
/// Divides by 1e7, which f64 holds exactly, rather than multiplying by 1e-7, which it does not. The quotient is
/// then correctly rounded, so 255_143_790 prints as 25.514379, the text it came from.
pub fn degrees_from_e7(e7: i32) -> f64 {
    f64::from(e7) / UNITS_PER_DEGREE
}

/// The mean of two latitudes in radians: where a degree of longitude is measured. Rounding it to f32 costs the
/// cosine about tan(latitude) x 2e-7 of itself, a millimetre over 5 km at mid latitudes (see the module notes).
fn mean_latitude_radians(first: i32, second: i32) -> f32 {
    ((i64::from(first) + i64::from(second)) as f32 / 2.0e7).to_radians()
}

/// The metre (east, north) offset from `from` to `to` (navigation.offset), the one geographic primitive.
///
/// The longitude difference wraps into [-180, 180) degrees as plan B's `(dlon + 180) % 360 - 180` does (Python's
/// `%` floors, as `rem_euclid` does), so a span across the anti-meridian takes the short way.
fn offset(from: Position, to: Position) -> (f32, f32) {
    let north_units = i64::from(to.latitude_e7) - i64::from(from.latitude_e7);
    let east_units = (i64::from(to.longitude_e7) - i64::from(from.longitude_e7) + HALF_TURN_UNITS)
        .rem_euclid(2 * HALF_TURN_UNITS)
        - HALF_TURN_UNITS;
    let scale = mean_latitude_radians(from.latitude_e7, to.latitude_e7).cos();
    (
        east_units as f32 * METRES_PER_UNIT * scale,
        north_units as f32 * METRES_PER_UNIT,
    )
}

/// The straight-line distance in metres from `from` to `to` (navigation.distance), equirectangular.
pub fn distance(from: Position, to: Position) -> f32 {
    let (east, north) = offset(from, to);
    (east * east + north * north).sqrt()
}

/// The zone rectangle's (north-south, east-west) sides in metres, both >= 0: what [`zone`] orients by and
/// [`zone_aspect`] divides.
///
/// No anti-meridian wrap here, as in plan B: both functions take `abs(lon_r - lon_l)` as given. The east-west
/// side is the absolute value of the whole product. At a pole the f32 cosine of 90 degrees is -4.4e-8 where
/// f64's is +6.1e-17, and a negative side would turn a polar line vertical; everywhere else the cosine is
/// positive and the abs() changes nothing.
fn spans(top_left: Position, bottom_right: Position) -> (f32, f32) {
    let latitude_units = (i64::from(top_left.latitude_e7) - i64::from(bottom_right.latitude_e7)).abs();
    let longitude_units = (i64::from(bottom_right.longitude_e7) - i64::from(top_left.longitude_e7)).abs();
    let scale = mean_latitude_radians(top_left.latitude_e7, bottom_right.latitude_e7).cos();
    (
        latitude_units as f32 * METRES_PER_UNIT,
        (longitude_units as f32 * METRES_PER_UNIT * scale).abs(),
    )
}

/// Resolve a zone rectangle to `(target, gate_a, gate_b)` (navigation.zone): the centre, and the midpoints of
/// the two shorter sides, so the glider enters along the long axis.
///
/// A zone wider (east-west) than tall gates on its left and right edges, a taller one on its top and bottom. An
/// exact square counts as wide, as plan B's `lon_span >= lat_span` does. Corner order does not matter: the
/// centre is the mean, the sides are absolute, and the gates sit on the corners' own coordinates, so either
/// diagonal pair gives the same two side midpoints. The centre is the integer mean of the corners, rounded
/// toward zero, which is at most half a unit (5.6 mm) from plan B's exact average.
///
/// SAFETY: the gates are fixed to the short sides, and nothing here knows what lies beyond them. Orienting the
/// zone so its entrances face clear corridors is the operator's decision (doc/specs/coludo.md "Zone
/// orientation").
pub fn zone(top_left: Position, bottom_right: Position) -> (Position, Position, Position) {
    let target = Position {
        latitude_e7: top_left.latitude_e7.midpoint(bottom_right.latitude_e7),
        longitude_e7: top_left.longitude_e7.midpoint(bottom_right.longitude_e7),
    };
    let (latitude_span, longitude_span) = spans(top_left, bottom_right);
    if longitude_span >= latitude_span {
        let left = Position {
            latitude_e7: target.latitude_e7,
            longitude_e7: top_left.longitude_e7,
        };
        let right = Position {
            latitude_e7: target.latitude_e7,
            longitude_e7: bottom_right.longitude_e7,
        };
        return (target, left, right);
    }
    let top = Position {
        latitude_e7: top_left.latitude_e7,
        longitude_e7: target.longitude_e7,
    };
    let bottom = Position {
        latitude_e7: bottom_right.latitude_e7,
        longitude_e7: target.longitude_e7,
    };
    (target, top, bottom)
}

/// The zone rectangle's long/short side ratio (navigation.zone_aspect), >= 1; 1.0 when a side is degenerate.
///
/// The endgame picks its holding pattern from it: a squarish zone orbits one circle, an elongated strip flies
/// along its long axis.
pub fn zone_aspect(top_left: Position, bottom_right: Position) -> f32 {
    let (latitude_span, longitude_span) = spans(top_left, bottom_right);
    let (long_side, short_side) = (latitude_span.max(longitude_span), latitude_span.min(longitude_span));
    if short_side > 0.0 { long_side / short_side } else { 1.0 }
}

#[cfg(test)]
mod tests {
    use super::{Position, degrees_from_e7, distance, e7_from_degrees, zone, zone_aspect};

    /// The position at these decimal degrees.
    fn at(latitude: f64, longitude: f64) -> Position {
        Position::from_degrees(latitude, longitude)
    }

    /// The position at these 1e-7 degree units.
    fn units(latitude_e7: i32, longitude_e7: i32) -> Position {
        Position {
            latitude_e7,
            longitude_e7,
        }
    }

    #[test]
    fn degrees_convert_to_units_and_back_exactly() {
        assert_eq!(at(25.514379, -80.391795), units(255_143_790, -803_917_950));
        assert_eq!(at(25.514379, -80.391795).degrees(), [25.514379, -80.391795]);
        assert_eq!(degrees_from_e7(-900_000_000), -90.0);
        // an eighth decimal rounds to the nearest unit, either way
        assert_eq!(e7_from_degrees(48.00000004), 480_000_000);
        assert_eq!(e7_from_degrees(48.00000006), 480_000_001);
        assert_eq!(e7_from_degrees(-48.00000006), -480_000_001);
    }

    #[test]
    fn distance_follows_the_meridian_and_shrinks_east_by_the_cosine() {
        let origin = at(48.0, 11.0);
        assert!((distance(origin, at(48.001, 11.0)) - 111.32).abs() < 0.01);
        let east = distance(origin, at(48.0, 11.001));
        assert!((east - 111.32 * 48.0005_f32.to_radians().cos()).abs() < 0.01, "{east}");
        assert_eq!(distance(origin, origin), 0.0);
        assert_eq!(distance(origin, at(48.001, 11.0)), distance(at(48.001, 11.0), origin));
    }

    #[test]
    fn a_step_an_f32_degree_loses_is_kept() {
        // the trap the integer positions exist for: 0.2 m north of latitude 48 is no step at all in f32 degrees
        let step_degrees = 0.2 / f64::from(super::METRES_PER_DEGREE);
        assert_eq!(48.0_f32 + step_degrees as f32, 48.0_f32);
        let moved = at(48.0 + step_degrees, 11.0);
        assert_eq!(moved.latitude_e7, 480_000_018);
        assert!((distance(at(48.0, 11.0), moved) - 0.2).abs() < 0.006);
    }

    #[test]
    fn distance_takes_the_short_way_across_the_anti_meridian() {
        let span = distance(at(10.0, 179.9995), at(10.0, -179.9995));
        assert!((span - 111.32 * 10.0_f32.to_radians().cos()).abs() < 0.01, "{span}");
        // and the long way never: the same pair 359.999 degrees apart would be ~39 000 km
        assert!(span < 200.0);
    }

    #[test]
    fn a_wide_zone_gates_on_its_left_and_right_edges() {
        let (target, gate_a, gate_b) = zone(at(48.001, 11.0), at(48.0, 11.01));
        assert_eq!(target, at(48.0005, 11.005));
        assert_eq!((gate_a, gate_b), (at(48.0005, 11.0), at(48.0005, 11.01)));
        assert!((zone_aspect(at(48.001, 11.0), at(48.0, 11.01)) - 6.69).abs() < 0.01);
    }

    #[test]
    fn a_tall_zone_gates_on_its_top_and_bottom_edges() {
        let (target, gate_a, gate_b) = zone(at(48.01, 11.0), at(48.0, 11.001));
        assert_eq!(target, at(48.005, 11.0005));
        assert_eq!((gate_a, gate_b), (at(48.01, 11.0005), at(48.0, 11.0005)));
    }

    #[test]
    fn corner_order_does_not_move_the_gates() {
        let (target, gate_a, gate_b) = zone(at(48.001, 11.0), at(48.0, 11.01));
        let (other_target, other_a, other_b) = zone(at(48.0, 11.0), at(48.001, 11.01));
        assert_eq!(target, other_target);
        assert_eq!([gate_a, gate_b], [other_a, other_b]);
        let (_, swapped_a, swapped_b) = zone(at(48.0, 11.01), at(48.001, 11.0));
        assert_eq!([gate_a, gate_b], [swapped_b, swapped_a]);
    }

    #[test]
    fn an_exact_square_counts_as_wide() {
        // centred on the equator, so the cosine is exactly 1 and the two sides are bit-identical
        let (target, gate_a, gate_b) = zone(at(0.0005, -0.0005), at(-0.0005, 0.0005));
        assert_eq!(target, at(0.0, 0.0));
        assert_eq!((gate_a, gate_b), (at(0.0, -0.0005), at(0.0, 0.0005)));
        assert_eq!(zone_aspect(at(0.0005, -0.0005), at(-0.0005, 0.0005)), 1.0);
    }

    #[test]
    fn a_degenerate_zone_has_aspect_one() {
        assert_eq!(zone_aspect(at(48.0, 11.0), at(48.0, 11.002)), 1.0);
        assert_eq!(zone_aspect(at(48.0, 11.0), at(48.0, 11.0)), 1.0);
        // a line along a meridian is tall; a point is "wide" (both sides 0)
        let (_, gate_a, gate_b) = zone(at(48.001, 11.0), at(48.0, 11.0));
        assert_eq!((gate_a, gate_b), (at(48.001, 11.0), at(48.0, 11.0)));
        let (_, gate_a, gate_b) = zone(at(48.0, 11.0), at(48.0, 11.0));
        assert_eq!((gate_a, gate_b), (at(48.0, 11.0), at(48.0, 11.0)));
    }

    #[test]
    fn a_line_at_the_pole_stays_wide() {
        // f32's cosine of 90 degrees is negative; plan B's f64 one is positive, and the line gates left/right
        let (_, gate_a, gate_b) = zone(at(90.0, 0.0), at(90.0, 0.001));
        assert_eq!((gate_a, gate_b), (at(90.0, 0.0), at(90.0, 0.001)));
        assert_eq!(zone_aspect(at(90.0, 0.0), at(90.0, 0.001)), 1.0);
    }

    #[test]
    fn the_centre_of_an_odd_span_rounds_toward_zero() {
        let (north, _, _) = zone(units(3, 0), units(0, 4));
        assert_eq!(north, units(1, 2));
        let (south, _, _) = zone(units(-3, 0), units(0, -4));
        assert_eq!(south, units(-1, -2));
    }
}
