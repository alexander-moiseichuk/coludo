# nav.py — landing-zone navigation geometry (Phase 4 'heading-to-home'), sibling of mixer.py/pid.py.
# The mission's landing zone is a lat/lon rectangle, top-left (TL) + bottom-right (BR) corners
# (specs/coludo.md). The TARGET is the zone centre; the two GATES are the midpoints of the two SHORTER
# sides, so the glider enters along the long axis (the documented "vector to the shortest boundary
# entrance"). steer() picks the nearer gate, heads for it until inside the zone, then for the centre.
# Equirectangular (flat-earth) math -- "not exact but about", which is plenty at zone scale (<~1 km).
#
# SAFETY: the gates are FIXED to the short sides, and steer() will always vector to one (and turn ~180
# back through it on an overshoot) with NO knowledge of what lies beyond any side (trees / launch pad /
# people). So the operator must ORIENT the zone -- choose the TL/BR corners in launch.config so the two
# short-side entrances point at hazard-free approach corridors and the long sides border the hazards.
# Aerodynamics (long run-in, lower crosswind) and safety (clear corridors) only align if it is laid out
# that way; the firmware cannot verify it. See specs/coludo.md "Zone orientation -- an operator safety
# decision".

import math

_M_PER_DEG: float = 111320.0  # metres per degree of latitude (and per degree longitude * cos(lat))


def _offset_m(lat1, lon1, lat2, lon2):
    """(east, north) offset in metres from point 1 to point 2 (equirectangular)."""
    lat_mid = math.radians((lat1 + lat2) / 2.0)
    east = (lon2 - lon1) * _M_PER_DEG * math.cos(lat_mid)
    north = (lat2 - lat1) * _M_PER_DEG
    return east, north


def bearing(lat1, lon1, lat2, lon2):
    """Compass bearing in degrees (0 = north, 90 = east, clockwise) from point 1 to point 2."""
    east, north = _offset_m(lat1, lon1, lat2, lon2)
    return math.degrees(math.atan2(east, north)) % 360.0


def distance(lat1, lon1, lat2, lon2):
    """Distance in metres from point 1 to point 2 (equirectangular)."""
    east, north = _offset_m(lat1, lon1, lat2, lon2)
    return math.sqrt(east * east + north * north)


def zone(corner_tl, corner_br):
    """Resolve the rectangle (top-left, bottom-right corners, each (lat, lon)) -> (target, gate_a,
    gate_b): the centre and the midpoints of the two SHORTER sides. A horizontally (longitude)
    stretched zone gates on its left/right edges; a vertically (latitude) stretched one on top/bottom."""
    lat_t, lon_l = corner_tl
    lat_b, lon_r = corner_br
    lat_c = (lat_t + lat_b) / 2.0
    lon_c = (lon_l + lon_r) / 2.0
    target = (lat_c, lon_c)
    lat_span = abs(lat_t - lat_b) * _M_PER_DEG
    lon_span = abs(lon_r - lon_l) * _M_PER_DEG * math.cos(math.radians(lat_c))
    if lon_span >= lat_span:  # wider than tall -> short sides are the left/right edges
        return target, (lat_c, lon_l), (lat_c, lon_r)
    return target, (lat_t, lon_c), (lat_b, lon_c)  # taller than wide -> top/bottom edges


def inside(position, corner_tl, corner_br):
    """True if position (lat, lon) is within the zone rectangle (corner order-agnostic)."""
    lat, lon = position
    lat_t, lon_l = corner_tl
    lat_b, lon_r = corner_br
    return (min(lat_t, lat_b) <= lat <= max(lat_t, lat_b) and
            min(lon_l, lon_r) <= lon <= max(lon_l, lon_r))


def steer(position, corner_tl, corner_br):
    """The heading to fly toward the landing target via the nearer gate: head for the closer short-side
    entrance until inside the zone, then for the centre. Returns (bearing_deg, waypoint, leg) with leg
    'gate' or 'target'. position = (lat, lon).

    Stateless + re-evaluated each tick, so the overshoot loop is emergent: if the glider crosses the
    zone and exits the far side without landing (still high), the gate it just crossed is now the
    nearest one -> it turns back (~180deg) and re-approaches through it. No waypoint memory -- the
    spec's 'recalculate to the nearest alternative entry and loop' just happens."""
    lat, lon = position
    target, gate_a, gate_b = zone(corner_tl, corner_br)
    if inside(position, corner_tl, corner_br):
        waypoint = target
        leg = 'target'
    else:
        waypoint = gate_a if distance(lat, lon, *gate_a) <= distance(lat, lon, *gate_b) else gate_b
        leg = 'gate'
    return bearing(lat, lon, waypoint[0], waypoint[1]), waypoint, leg
