"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Landing-zone navigation geometry ('heading-to-home'), sibling of mixer.py/pid.py. The mission's
landing zone is a lat/lon rectangle, top-left (TL) + bottom-right (BR) corners (doc/specs/coludo.md).
The TARGET is the zone centre; the two GATES are the midpoints of the two SHORTER sides, so the
glider enters along the long axis (the documented "vector to the shortest boundary entrance").
steer() picks the nearer gate, heads for it until inside the zone, then for the centre.
Equirectangular (flat-earth) math -- "not exact but about", which is plenty at zone scale (<~1 km).

Perf: steer()/bearing()/distance() use float trig and allocate a few small tuples per call --
measured GC-off (n=4000): distance ~170 B, bearing ~256, steer ~518, approach ~982. The primary
throttle is on the CALLER side -- guidance._target_heading() caches the result at GPS cadence
(~10 Hz), so the rate drops ~10x -- but at that rate the nav path is still ~5 KB/s of the ~15 KB/s
glide leak, so "no measurable gain" (the old note) was wrong. The lever is REDUNDANT float work,
not fixnum (lat/lon need ~1e-5 deg; a SCALE-100 fixnum degree is ~1 km -- far too coarse): a caller
that needs both range AND bearing to a point now calls range_bearing() (one offset(), not two).
navigation stays pure float; the geometry is computed once. (Next lever, if needed: cache the
per-flight-constant zone() geometry.)

SAFETY: the gates are FIXED to the short sides, and steer() will always vector to one (and turn ~180
back through it on an overshoot) with NO knowledge of what lies beyond any side (trees / launch pad /
people). So the operator must ORIENT the zone -- choose the TL/BR corners in launch.config so the two
short-side entrances point at hazard-free approach corridors and the long sides border the hazards.
Aerodynamics (long run-in, lower crosswind) and safety (clear corridors) only align if it is laid out
that way; the firmware cannot verify it. See doc/specs/coludo.md "Zone orientation -- an operator safety
decision".
"""

import math

from commons import M_PER_DEG, between

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const



# steer() leg -- which waypoint the returned heading aims at this tick. Named integer codes rather
# than bare strings (cheaper to compare, no typo-prone literals at the call sites).
UNKNOWN: int = const(0)  # not steering (no zone) -- never produced by steer(), a sentinel for callers
TARGET: int = const(1)   # inside the zone -> heading for the centre (the landing target)
GATE: int = const(2)     # outside the zone -> heading for the nearer short-side entrance


def offset(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple:
    """
    Metre (east, north) offset from point 1 to point 2 (equirectangular).

    The longitude delta is wrapped to [-180, 180] so a span crossing the anti-meridian (+/-180 deg)
    does not flip the vector -- the same wrap the heading-error math uses (Coludo flies nowhere near
    +/-180, but it is a free correctness guard). This is the ONE geographic primitive: distance() /
    bearing() / range_bearing() all derive from it, so a caller needing BOTH range and bearing to the
    same point computes the float-trig ONCE (via range_bearing) instead of twice (see the header note).

    Args:
        lat1, lon1 - the FROM point (decimal degrees).
        lat2, lon2 - the TO point (decimal degrees).

    Returns:
        (east, north) offset in metres.
    """
    lat_mid = math.radians((lat1 + lat2) / 2.0)
    dlon = (lon2 - lon1 + 180.0) % 360.0 - 180.0  # anti-meridian-safe longitude delta
    east = dlon * M_PER_DEG * math.cos(lat_mid)
    north = (lat2 - lat1) * M_PER_DEG
    return east, north


def advance(position: tuple, east: float, north: float) -> tuple:
    """
    Move a position by a metre (east, north) offset -- the inverse of offset(), same flat-earth model.

    Exists for DEAD RECKONING: with no GNSS fix the glider propagates its last known position by the
    air velocity it can still measure (airspeed x heading) plus the last wind estimate, so navigation
    keeps a moving position instead of a frozen one. Equirectangular like offset(), which is exact
    enough over the hundreds of metres a flight covers.

    Args:
        position - the (latitude, longitude) to move from (decimal degrees).
        east, north - the offset to apply (metres).

    Returns:
        The new (latitude, longitude) in decimal degrees.
    """
    latitude = position[0] + north / M_PER_DEG
    scale = math.cos(math.radians((position[0] + latitude) / 2.0))  # same mid-latitude as offset()
    longitude = position[1] + (east / (M_PER_DEG * scale) if scale > 1e-9 else 0.0)  # 0 at the poles
    return latitude, longitude


def compass(east: float, north: float) -> float:
    """
    Convert an (east, north) metre offset to a compass bearing.

    Args:
        east - eastward component (metres).
        north - northward component (metres).

    Returns:
        Bearing in degrees: 0 = north, 90 = east, clockwise, wrapped to [0, 360).
    """
    return math.degrees(math.atan2(east, north)) % 360.0


def bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Compass bearing from point 1 to point 2.

    Args:
        lat1, lon1 - the FROM point (decimal degrees).
        lat2, lon2 - the TO point (decimal degrees).

    Returns:
        Bearing in degrees (0 = north, 90 = east, clockwise).
    """
    return compass(*offset(lat1, lon1, lat2, lon2))


def distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Straight-line distance from point 1 to point 2 (equirectangular).

    Args:
        lat1, lon1 - the FROM point (decimal degrees).
        lat2, lon2 - the TO point (decimal degrees).

    Returns:
        Distance in metres.
    """
    east, north = offset(lat1, lon1, lat2, lon2)
    return math.sqrt(east * east + north * north)


def range_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple:
    """
    Distance AND bearing from point 1 to point 2 in one pass.

    The fused primitive for a caller that needs both (loiter: orbit radius + tangent heading;
    reachability: range + wind projection) -- one offset()/float-trig instead of the distance() +
    bearing() pair that recomputed it twice.

    Args:
        lat1, lon1 - the FROM point (decimal degrees).
        lat2, lon2 - the TO point (decimal degrees).

    Returns:
        (distance_metres, bearing_degrees).
    """
    east, north = offset(lat1, lon1, lat2, lon2)
    return math.sqrt(east * east + north * north), compass(east, north)


def zone(corner_tl: tuple, corner_br: tuple) -> tuple:
    """
    Resolve a zone rectangle to its target centre + the two short-side gates.

    A horizontally (longitude) stretched zone gates on its left/right edges; a vertically (latitude)
    stretched one on top/bottom. Corner ORDER does not matter: the centre is the average, the spans use
    abs(), and the gates are the coordinate EXTREMES -- so whichever diagonal pair is passed (TL/BR or
    BL/TR) the two returned gates are the same side-midpoints (steer() then picks the nearer). No
    normalisation needed.

    Args:
        corner_tl - one corner (lat, lon), decimal degrees.
        corner_br - the diagonally-opposite corner (lat, lon).

    Returns:
        (target, gate_a, gate_b): the centre (lat, lon) and the two shorter-side midpoints.
    """
    lat_t, lon_l = corner_tl
    lat_b, lon_r = corner_br
    lat_c = (lat_t + lat_b) / 2.0
    lon_c = (lon_l + lon_r) / 2.0
    target = (lat_c, lon_c)
    lat_span = abs(lat_t - lat_b) * M_PER_DEG
    lon_span = abs(lon_r - lon_l) * M_PER_DEG * math.cos(math.radians(lat_c))
    if lon_span >= lat_span:  # wider than tall -> short sides are the left/right edges
        return target, (lat_c, lon_l), (lat_c, lon_r)
    return target, (lat_t, lon_c), (lat_b, lon_c)  # taller than wide -> top/bottom edges


def zone_aspect(corner_tl: tuple, corner_br: tuple) -> float:
    """
    The zone rectangle's long/short side ratio.

    The endgame picks its pattern from it: a SQUARISH zone (ratio <= the config threshold) orbits a
    single circle ('o'); an ELONGATED strip flies two lobes along the long axis ('oo').

    Args:
        corner_tl - one corner (lat, lon), decimal degrees.
        corner_br - the diagonally-opposite corner (lat, lon).

    Returns:
        The ratio (>= 1); 1.0 when a side is degenerate.
    """
    lat_t, lon_l = corner_tl
    lat_b, lon_r = corner_br
    lat_span = abs(lat_t - lat_b) * M_PER_DEG
    lon_span = abs(lon_r - lon_l) * M_PER_DEG * math.cos(math.radians((lat_t + lat_b) / 2.0))
    long_side, short_side = max(lat_span, lon_span), min(lat_span, lon_span)
    return long_side / short_side if short_side else 1.0


def inside(position: tuple, corner_tl: tuple, corner_br: tuple) -> bool:
    """
    Whether a position is within the zone rectangle (corner order-agnostic).

    Args:
        position - the point (lat, lon), decimal degrees.
        corner_tl, corner_br - the two diagonally-opposite corners (lat, lon).

    Returns:
        True if position is inside the rectangle, else False.
    """
    lat, lon = position
    lat_t, lon_l = corner_tl
    lat_b, lon_r = corner_br
    return (min(lat_t, lat_b) <= lat <= max(lat_t, lat_b) and
            min(lon_l, lon_r) <= lon <= max(lon_l, lon_r))




def steer(position: tuple, corner_tl: tuple, corner_br: tuple) -> tuple:
    """
    The heading to fly toward the landing target via the nearer gate.

    Head for the closer short-side entrance until inside the zone, then for the centre. Stateless +
    re-evaluated each tick, so the overshoot loop is emergent: if the glider crosses the zone and exits
    the far side still high, the gate it just crossed is now the nearest -> it turns back (~180deg) and
    re-approaches through it. No waypoint memory -- the spec's "recalculate to the nearest alternative
    entry and loop" just happens.

    Args:
        position - the glider (lat, lon), decimal degrees.
        corner_tl, corner_br - the zone's diagonally-opposite corners (lat, lon).

    Returns:
        (bearing_degrees, waypoint, leg), with leg GATE (outside the zone) or TARGET (inside).
    """
    return steer_to(position, corner_tl, corner_br, *zone(corner_tl, corner_br))


def steer_to(position: tuple, corner_tl: tuple, corner_br: tuple,
             target: tuple, gate_a: tuple, gate_b: tuple) -> tuple:
    """
    steer() with the zone geometry (target + the two gates) ALREADY resolved.

    Lets a caller that steers every nav tick resolve the per-flight-constant zone() ONCE
    (mission.zone_points) instead of paying it here each call. inside() still takes the corners -- a
    cheap min/max, no trig/alloc.

    Args:
        position - the glider (lat, lon), decimal degrees.
        corner_tl, corner_br - the zone corners (for the cheap inside() check).
        target, gate_a, gate_b - the pre-resolved zone geometry (from zone()).

    Returns:
        (bearing_degrees, waypoint, leg), with leg GATE or TARGET.
    """
    lat, lon = position
    if inside(position, corner_tl, corner_br):
        waypoint = target
        leg = TARGET
    else:
        waypoint = gate_a if distance(lat, lon, *gate_a) <= distance(lat, lon, *gate_b) else gate_b
        leg = GATE
    return bearing(lat, lon, waypoint[0], waypoint[1]), waypoint, leg


def cross_track(position: tuple, point: tuple, heading: float) -> float:
    """
    Signed perpendicular distance from a position to a line.

    Args:
        position - the point to measure (lat, lon), decimal degrees.
        point - a point ON the line (lat, lon).
        heading - the line's compass direction (degrees).

    Returns:
        Perpendicular distance in metres; positive = to the RIGHT of the line, looking along heading.
    """
    east, north = offset(point[0], point[1], position[0], position[1])
    radians = math.radians(heading)
    return east * math.cos(radians) - north * math.sin(radians)


def approach(position: tuple, corner_tl: tuple, corner_br: tuple, heading: float,
             cross_gain: float, intercept_max: float) -> float:
    """
    Final-approach heading that TRACKS the zone's long-axis centreline (the strip).

    Used low on final instead of homing to the centre POINT: the glider intercepts the line at up to
    intercept_max deg (cross_gain deg per metre off it), then flies down it, so a crosswind is crabbed
    out and the touchdown holds the narrow strip. (A banked crab, not a wing-low slip -- that would need
    a sideslip-capable airframe model; the residual at strong wind is airframe-bound, not a control gap.)

    Args:
        position - the glider (lat, lon), decimal degrees.
        corner_tl, corner_br - the zone's diagonally-opposite corners (lat, lon).
        heading - the glider's current heading (degrees), used to pick the along-strip direction.
        cross_gain - intercept degrees commanded per metre off the centreline.
        intercept_max - the cap on the intercept angle (degrees).

    Returns:
        The heading to fly (degrees).
    """
    return approach_to(position, *zone(corner_tl, corner_br),
                       heading, cross_gain, intercept_max)


def approach_to(position: tuple, target: tuple, gate_a: tuple, gate_b: tuple,
                heading: float, cross_gain: float, intercept_max: float) -> float:
    """
    approach() with the zone geometry ALREADY resolved (see steer_to()).

    The caller reuses one mission.zone_points() resolve across the whole nav tick instead of each nav
    call recomputing zone().

    Args:
        position - the glider (lat, lon), decimal degrees.
        target, gate_a, gate_b - the pre-resolved zone geometry (from zone()).
        heading - the glider's current heading (degrees).
        cross_gain - intercept degrees commanded per metre off the centreline.
        intercept_max - the cap on the intercept angle (degrees).

    Returns:
        The heading to fly (degrees).
    """
    centreline = bearing(gate_a[0], gate_a[1], gate_b[0], gate_b[1])
    if abs(((centreline - heading + 180.0) % 360.0) - 180.0) > 90.0:  # fly the along-strip way we are going
        centreline = (centreline + 180.0) % 360.0
    # off to the RIGHT -> aim left of the centreline to intercept (and vice versa), capped
    intercept = between(-intercept_max, -cross_gain * cross_track(position, target, centreline), intercept_max)
    return (centreline + intercept) % 360.0
