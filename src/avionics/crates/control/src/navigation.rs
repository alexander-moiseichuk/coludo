//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Landing-zone navigation geometry, 'heading-to-home' (src/glider navigation.py), sibling of the guidance law
//! that flies it. The mission's landing zone is a latitude/longitude rectangle given by two diagonally opposite
//! corners (doc/specs/coludo.md). Its TARGET is the centre; its two GATES are the midpoints of the two SHORTER
//! sides, so the glider enters along the long axis (the documented "vector to the shortest boundary entrance").
//! [`steer`] heads for the nearer gate until the glider is inside the zone, then for the centre; [`approach`]
//! tracks the long-axis centreline low on final.
//!
//! Equirectangular (flat-earth) math, as in plan B: "not exact but about", which is plenty at zone scale (under a
//! kilometre or so). [`offset`] is the one geographic primitive; [`distance`], [`bearing`] and [`range_bearing`]
//! derive from it, and [`advance`] is its inverse.
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
//!
//! SAFETY: the gates are FIXED to the short sides, and [`steer`] always vectors to one (and turns ~180 degrees
//! back through it on an overshoot) with NO knowledge of what lies beyond any side (trees, the launch pad,
//! people). So the operator must ORIENT the zone: choose the corners in launch.config so the two short-side
//! entrances face hazard-free approach corridors and the long sides border the hazards. Aerodynamics (a long
//! run-in, lower crosswind) and safety (clear corridors) only align when it is laid out that way, and the
//! firmware cannot verify it (doc/specs/coludo.md "Zone orientation -- an operator safety decision").
//!
//! Plan B's third leg id, `UNKNOWN` ("not steering", a sentinel steer() never produces), is not ported: no
//! caller reads it.

use crate::commons::between;

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

/// The smallest cos(latitude) [`advance`] divides by. Closer to a pole a metre east is no longer a usable
/// number of degrees, and plan B moves the longitude by nothing there (its `scale > 1e-9` guard).
const POLE_SCALE: f32 = 1e-9;

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

/// Which waypoint the heading [`steer`] returns aims at this tick (navigation.TARGET / navigation.GATE).
///
/// The discriminants are plan B's ids, so a log line or a row that carries the leg reads the same on both
/// firmwares.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Leg {
    /// Inside the zone: heading for the centre, the landing target.
    Target = 1,
    /// Outside the zone: heading for the nearer short-side entrance.
    Gate = 2,
}

/// What [`steer`] answers: the heading to fly, the waypoint it aims at, and which kind of waypoint that is.
///
/// Plan B returns the same three as a tuple `(bearing_degrees, waypoint, leg)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Steering {
    /// The compass bearing to the waypoint, degrees: 0 = north, 90 = east, clockwise, in [0, 360].
    pub bearing: f32,
    /// The gate or the target the bearing points at.
    pub waypoint: Position,
    /// Whether the waypoint is a gate (outside the zone) or the target (inside it).
    pub leg: Leg,
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

/// The metre `(east, north)` offset from `from` to `to` (navigation.offset), the one geographic primitive.
///
/// The longitude difference wraps into [-180, 180) degrees as plan B's `(dlon + 180) % 360 - 180` does (Python's
/// `%` floors, as `rem_euclid` does), so a span across the anti-meridian takes the short way. Coludo flies nowhere
/// near 180 degrees, but the guard is free. A caller that needs both range and bearing to one point takes
/// [`range_bearing`], which computes this once instead of twice.
pub fn offset(from: Position, to: Position) -> (f32, f32) {
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

/// `position` moved by a metre `(east, north)` offset (navigation.advance): the inverse of [`offset`], with the
/// same flat-earth model.
///
/// It exists for DEAD RECKONING: with no GNSS fix the glider propagates its last known position by the air
/// velocity it can still measure (airspeed along the heading) plus the last wind estimate, so navigation keeps a
/// moving position instead of a frozen one. Equirectangular like [`offset`], which is exact enough over the
/// hundreds of metres a flight covers. The longitude is measured at the mean of the old and new latitudes, as
/// [`offset`] measures it, and closer to a pole than a cosine of 1e-9 it does not move at all (plan B's guard).
///
/// The answer is rounded to the nearest unit (5.6 mm at most along each axis). That is the point of the integer
/// position: plan B on the board adds the offset to an f32 degree, whose 0.42 m ulp swallows a slow glider's
/// steps whole, which is why its dead reckoning accumulates metres and converts once; here the conversion keeps
/// its centimetre wherever it is done. Plan B does not wrap the longitude either; a result past the i32 range
/// (some 35 degrees beyond the anti-meridian) saturates.
pub fn advance(position: Position, east: f32, north: f32) -> Position {
    let latitude_e7 = saturating_units(i64::from(position.latitude_e7) + rounded_units(north / METRES_PER_UNIT));
    let scale = mean_latitude_radians(position.latitude_e7, latitude_e7).cos();
    let east_units = if scale > POLE_SCALE {
        rounded_units(east / (METRES_PER_UNIT * scale))
    } else {
        0
    };
    Position {
        latitude_e7,
        longitude_e7: saturating_units(i64::from(position.longitude_e7) + east_units),
    }
}

/// The whole number of units nearest `units` (a NaN is none, and a value past i64 saturates, as `as` does).
fn rounded_units(units: f32) -> i64 {
    units.round() as i64
}

/// A unit count held to the i32 a [`Position`] stores.
fn saturating_units(units: i64) -> i32 {
    units.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
}

/// The compass bearing of a metre `(east, north)` offset (navigation.compass): degrees, 0 = north, 90 = east,
/// clockwise, wrapped into [0, 360).
///
/// A tiny negative angle wraps to 360 itself in f32 (as it can in plan B's f64): a caller comparing headings
/// takes the difference the short way, never the raw value.
pub fn compass(east: f32, north: f32) -> f32 {
    east.atan2(north).to_degrees().rem_euclid(360.0)
}

/// The compass bearing from `from` to `to` in degrees (navigation.bearing).
pub fn bearing(from: Position, to: Position) -> f32 {
    let (east, north) = offset(from, to);
    compass(east, north)
}

/// The straight-line distance in metres from `from` to `to` (navigation.distance), equirectangular.
pub fn distance(from: Position, to: Position) -> f32 {
    let (east, north) = offset(from, to);
    (east * east + north * north).sqrt()
}

/// The distance in metres AND the bearing in degrees from `from` to `to`, in one pass (navigation.range_bearing).
///
/// The fused primitive for a caller that needs both (the loiter: orbit radius and tangent heading;
/// reachability: range and the wind's projection): one [`offset`] instead of the [`distance`] and [`bearing`]
/// pair that would compute it twice.
pub fn range_bearing(from: Position, to: Position) -> (f32, f32) {
    let (east, north) = offset(from, to);
    ((east * east + north * north).sqrt(), compass(east, north))
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
/// zone so its entrances face clear corridors is the operator's decision (see the module notes).
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
/// along its long axis (the mission crate's `Mission::endgame_heading`, which hands guidance its answer).
pub fn zone_aspect(top_left: Position, bottom_right: Position) -> f32 {
    let (latitude_span, longitude_span) = spans(top_left, bottom_right);
    let (long_side, short_side) = (latitude_span.max(longitude_span), latitude_span.min(longitude_span));
    if short_side > 0.0 { long_side / short_side } else { 1.0 }
}

/// Whether `position` lies within the zone rectangle, its edges included (navigation.inside); corner order does
/// not matter.
///
/// An integer comparison, so it answers exactly as plan B's comparison of the same positions in degrees does
/// (dividing by 1e7 keeps the order).
pub fn inside(position: Position, top_left: Position, bottom_right: Position) -> bool {
    let latitudes =
        top_left.latitude_e7.min(bottom_right.latitude_e7)..=top_left.latitude_e7.max(bottom_right.latitude_e7);
    let longitudes =
        top_left.longitude_e7.min(bottom_right.longitude_e7)..=top_left.longitude_e7.max(bottom_right.longitude_e7);
    latitudes.contains(&position.latitude_e7) && longitudes.contains(&position.longitude_e7)
}

/// The heading toward the landing target through the nearer gate (navigation.steer).
///
/// Head for the closer short-side entrance until inside the zone, then for the centre. Stateless and
/// re-evaluated each tick, so the overshoot loop is emergent: a glider that crosses the zone and leaves by the
/// far side still high finds the gate it just crossed is now the nearer one, turns back (~180 degrees) and
/// re-approaches through it. No waypoint memory: the spec's "recalculate to the nearest alternative entry and
/// loop" just happens.
pub fn steer(position: Position, top_left: Position, bottom_right: Position) -> Steering {
    let (target, gate_a, gate_b) = zone(top_left, bottom_right);
    steer_to(position, top_left, bottom_right, target, gate_a, gate_b)
}

/// [`steer`] with the zone geometry (target and the two gates) ALREADY resolved (navigation.steer_to).
///
/// A caller that steers every navigation tick resolves the per-flight-constant [`zone`] once (the guidance
/// law's `Landing`) instead of paying for it on each call. [`inside`] still takes the corners: a cheap integer
/// comparison. Equidistant gates go to `gate_a`, as plan B's `<=` sends them.
pub fn steer_to(
    position: Position,
    top_left: Position,
    bottom_right: Position,
    target: Position,
    gate_a: Position,
    gate_b: Position,
) -> Steering {
    let (waypoint, leg) = if inside(position, top_left, bottom_right) {
        (target, Leg::Target)
    } else if distance(position, gate_a) <= distance(position, gate_b) {
        (gate_a, Leg::Gate)
    } else {
        (gate_b, Leg::Gate)
    };
    Steering {
        bearing: bearing(position, waypoint),
        waypoint,
        leg,
    }
}

/// The signed perpendicular distance in metres from `position` to the line through `point` along compass
/// `heading` degrees (navigation.cross_track): positive to the RIGHT of the line, looking along `heading`.
pub fn cross_track(position: Position, point: Position, heading: f32) -> f32 {
    let (east, north) = offset(point, position);
    let radians = heading.to_radians();
    east * radians.cos() - north * radians.sin()
}

/// The final-approach heading that TRACKS the zone's long-axis centreline, the strip (navigation.approach).
///
/// Used low on final instead of homing to the centre POINT: the glider intercepts the line at up to
/// `intercept_max` degrees (`cross_gain` degrees per metre off it), then flies down it, so a crosswind is crabbed
/// out and the touchdown holds the narrow strip. (A banked crab, not a wing-low slip: that would need a
/// sideslip-capable airframe model, and the residual at strong wind is airframe-bound, not a control gap.)
/// `heading`, the glider's current one, picks which way along the strip to fly.
pub fn approach(
    position: Position,
    top_left: Position,
    bottom_right: Position,
    heading: f32,
    cross_gain: f32,
    intercept_max: f32,
) -> f32 {
    let (target, gate_a, gate_b) = zone(top_left, bottom_right);
    approach_to(position, target, gate_a, gate_b, heading, cross_gain, intercept_max)
}

/// [`approach`] with the zone geometry ALREADY resolved (navigation.approach_to; see [`steer_to`]).
///
/// The centreline runs from `gate_a` to `gate_b`; when that direction is more than 90 degrees off `heading` the
/// glider flies it the other way, the along-strip direction it is already going. Off to the right of the line it
/// aims left to intercept (and vice versa), capped at `intercept_max`.
pub fn approach_to(
    position: Position,
    target: Position,
    gate_a: Position,
    gate_b: Position,
    heading: f32,
    cross_gain: f32,
    intercept_max: f32,
) -> f32 {
    let mut centreline = bearing(gate_a, gate_b);
    if ((centreline - heading + 180.0).rem_euclid(360.0) - 180.0).abs() > 90.0 {
        centreline = (centreline + 180.0).rem_euclid(360.0);
    }
    let intercept = between(
        -intercept_max,
        -cross_gain * cross_track(position, target, centreline),
        intercept_max,
    );
    (centreline + intercept).rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::{
        Leg, Position, advance, approach, bearing, compass, cross_track, degrees_from_e7, distance, e7_from_degrees,
        inside, offset, range_bearing, steer, steer_to, zone, zone_aspect,
    };

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

    /// Whether two headings agree within `tolerance` degrees, the short way around.
    fn close_heading(first: f32, second: f32, tolerance: f32) -> bool {
        let around = (first - second).rem_euclid(360.0);
        around.min(360.0 - around) <= tolerance
    }

    /// test_navigation's wide zone near 48 N: 0.001 degree tall by 0.010 wide, gates on the left and right edges.
    const WIDE: (f64, f64, f64, f64) = (48.001, 11.000, 48.000, 11.010);

    /// The wide zone's two corners.
    fn wide() -> (Position, Position) {
        (at(WIDE.0, WIDE.1), at(WIDE.2, WIDE.3))
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

    #[test]
    fn bearings_point_the_four_ways() {
        // test_navigation.test_bearing_distance
        let origin = at(48.0, 11.0);
        assert!(close_heading(bearing(origin, at(48.001, 11.0)), 0.0, 1.0));
        assert!(close_heading(bearing(origin, at(48.0, 11.001)), 90.0, 1.0));
        assert!(close_heading(bearing(origin, at(47.999, 11.0)), 180.0, 1.0));
        assert!(close_heading(bearing(origin, at(48.0, 10.999)), 270.0, 1.0));
        let (range, heading) = range_bearing(origin, at(48.001, 11.001));
        assert_eq!(range, distance(origin, at(48.001, 11.001)));
        assert_eq!(heading, bearing(origin, at(48.001, 11.001)));
    }

    #[test]
    fn compass_wraps_into_a_full_turn() {
        assert_eq!(compass(0.0, 1.0), 0.0);
        assert_eq!(compass(1.0, 0.0), 90.0);
        assert_eq!(compass(0.0, -1.0), 180.0);
        assert_eq!(compass(-1.0, 0.0), 270.0);
        // the negative zero of a due-south offset still reads south, never -180
        assert_eq!(compass(-0.0, -1.0), 180.0);
        // no offset at all is north, as plan B's atan2(0, 0) answers
        assert_eq!(compass(0.0, 0.0), 0.0);
        assert!((0.0..=360.0).contains(&compass(-1e-6, 1.0)));
    }

    #[test]
    fn advance_inverts_offset() {
        let start = at(48.0, 11.0);
        for (east, north) in [
            (100.0_f32, 0.0_f32),
            (0.0, -250.0),
            (-731.5, 412.25),
            (0.2, 0.0),
            (0.0, 0.2),
        ] {
            let moved = advance(start, east, north);
            let (back_east, back_north) = offset(start, moved);
            assert!((back_east - east).abs() < 0.01, "{east} -> {back_east}");
            assert!((back_north - north).abs() < 0.01, "{north} -> {back_north}");
        }
        assert_eq!(advance(start, 0.0, 0.0), start);
    }

    #[test]
    fn advance_does_not_move_east_at_a_pole() {
        let pole = units(900_000_000, 0);
        assert_eq!(advance(pole, 500.0, 0.0), pole);
        // and a position past the i32 range saturates rather than wrapping to the other hemisphere
        let far_east = advance(units(0, 2_147_000_000), 1.0e6, 0.0);
        assert_eq!(far_east.longitude_e7, i32::MAX);
    }

    #[test]
    fn inside_includes_the_edges_in_either_corner_order() {
        let (top_left, bottom_right) = wide();
        assert!(inside(at(48.0005, 11.005), top_left, bottom_right));
        assert!(inside(top_left, top_left, bottom_right));
        assert!(inside(bottom_right, bottom_right, top_left));
        assert!(!inside(at(48.0005, 10.990), top_left, bottom_right));
        assert!(!inside(units(480_010_001, 110_050_000), top_left, bottom_right));
    }

    #[test]
    fn steer_heads_for_the_nearer_gate_then_the_target() {
        // test_navigation.test_inside_and_steer and test_overshoot_loop
        let (top_left, bottom_right) = wide();
        let west = steer(at(48.0005, 10.990), top_left, bottom_right);
        assert_eq!((west.leg, west.waypoint), (Leg::Gate, at(48.0005, 11.000)));
        assert!(close_heading(west.bearing, 90.0, 5.0));
        let east = steer(at(48.0005, 11.020), top_left, bottom_right);
        assert_eq!((east.leg, east.waypoint), (Leg::Gate, at(48.0005, 11.010)));
        assert!(close_heading(east.bearing, 270.0, 5.0));
        let within = steer(at(48.0005, 11.002), top_left, bottom_right);
        assert_eq!((within.leg, within.waypoint), (Leg::Target, at(48.0005, 11.005)));
        assert!(close_heading(within.bearing, 90.0, 5.0));
        // just past the east edge after crossing eastbound: the gate just crossed is nearest -> turn back west
        let overshoot = steer(at(48.0005, 11.011), top_left, bottom_right);
        assert_eq!((overshoot.leg, overshoot.waypoint), (Leg::Gate, at(48.0005, 11.010)));
        assert!(close_heading(overshoot.bearing, 270.0, 5.0));
        let (target, gate_a, gate_b) = zone(top_left, bottom_right);
        assert_eq!(
            steer_to(at(48.0005, 10.990), top_left, bottom_right, target, gate_a, gate_b),
            west
        );
    }

    #[test]
    fn equidistant_gates_go_to_gate_a() {
        // north of the centre of the wide zone: both gates are equally far
        let (top_left, bottom_right) = wide();
        let (_, gate_a, _) = zone(top_left, bottom_right);
        assert_eq!(steer(at(48.002, 11.005), top_left, bottom_right).waypoint, gate_a);
    }

    #[test]
    fn cross_track_is_positive_to_the_right() {
        // test_navigation.test_cross_track_and_approach: a line north through (48, 11)
        let point = at(48.0, 11.0);
        assert!(cross_track(at(48.0, 11.001), point, 0.0) > 50.0);
        assert!(cross_track(at(48.0, 10.999), point, 0.0) < -50.0);
        assert!(cross_track(at(48.001, 11.0), point, 0.0).abs() < 1.0);
    }

    #[test]
    fn approach_tracks_the_centreline_the_way_the_glider_flies() {
        let (top_left, bottom_right) = wide();
        let centre = at(48.0005, 11.005);
        assert!(close_heading(
            approach(centre, top_left, bottom_right, 90.0, 3.0, 45.0),
            90.0,
            1.0
        ));
        assert!(close_heading(
            approach(centre, top_left, bottom_right, 260.0, 3.0, 45.0),
            270.0,
            1.0
        ));
        // ~22 m south of the line flying east: aim left (north) of 90, the intercept capped at 45
        let aimed = approach(at(48.0003, 11.005), top_left, bottom_right, 90.0, 3.0, 45.0);
        assert!((45.0..90.0).contains(&aimed), "{aimed}");
        // and with no gain the strip direction is flown as is, however far off the line
        assert!(close_heading(
            approach(at(48.0003, 11.005), top_left, bottom_right, 90.0, 0.0, 45.0),
            90.0,
            0.01
        ));
    }
}
