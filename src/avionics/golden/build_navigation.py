"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture navigation.json: the rest of plan B's landing-zone navigation (src/glider/navigation.py) --
offset, advance, compass, bearing, distance and range_bearing, inside, steer/steer_to, cross_track and
approach/approach_to. navigation_zone.json keeps the part the Mission reads (distance, zone, zone_aspect).

The Rust port (crates/control, module navigation) keeps a position as integer 1e-7 degree units, so every
position here is written in those units and plan B is called with units / 1e7, the float the same position is
in plan B. Both sides start from the same point, and the Rust test never re-derives an input.

Every case's output is an object keyed by how its values compare; `compare` names each group's kind:
  * metres -- abs 0.5 m, the tolerance doc/specs/avionics-port.md states for navigation. The Rust offsets are
    f32 on integer differences: the worst measured over these cases is 0.4 mm on an offset and 1 mm on a cross
    track (3 km reaches, within 60 degrees of latitude), so the plan's half metre is a wide margin, as it means
    to be: it bounds what the guidance law can feel, a fortieth of the ~20 m turn floor.
  * degrees -- angle_deg 0.05, the plan's: bearings and headings, compared the short way around, so 359.99 and
    0.01 agree. f32 trig on identical integer positions measured 2e-5 degree at worst.
  * units -- abs 0.6 units, for advance(), whose answer is a position. The Rust port rounds it to the nearest
    unit (half a unit at most); the rest is f32 (0.007 unit the worst measured over these cases, 3 km offsets
    within 60 degrees of latitude). Far inside the plan's half metre (45 units along a meridian); a one-unit
    slip fails.
  * exact -- inside()'s verdict, steer()'s waypoint (in units) and leg. Integers compare exactly under every kind.

Cases stay off the decision thresholds, where one ulp could flip an answer: steer() picks the nearer gate by
comparing two distances, and approach() turns the centreline round when it lies more than 90 degrees off the
heading. Every steer and approach case is re-run by plan B with its position moved two units (2.2 cm) each way
and its heading 0.01 degree each way, far more than the f32 answer can differ; a case whose waypoint, leg or
answer moves under that is skipped from the sweep (an explicit case raises). Zones have even corner sums, so
plan B's centre is a whole unit and the Rust integer centre is the same point.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random

import commons
import navigation

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_SWEEP: int = 120  # seeded cases per section
_UNITS_PER_DEGREE: float = 1e7  # a Rust Position counts 1e-7 degrees
_HALF_TURN_E7: int = 1_800_000_000  # 180 degrees in units
_SWEEP_LATITUDE_E7: int = 600_000_000  # sweep latitudes stay within +/-60 degrees
_SWEEP_REACH_M: float = 3000.0  # sweep offsets reach this far
_ZONE_SIDE_M: tuple = (20.0, 600.0)  # sweep zone side lengths, metres
_JITTER_UNITS: int = 2  # the robustness re-runs move a position this far along each axis
_JITTER_DEGREES: float = 0.01  # ...and a heading this far
_APPROACH_STABLE_DEGREES: float = 1.0  # an approach answer that moves more under the jitter sits on the flip
_EXACT_SLACK: float = 1e-6  # a plan B waypoint this close to a whole unit is that unit (float slack only)

_COMPARE: dict = {
    'metres': {'kind': 'abs', 'tol': 0.5},
    'degrees': {'kind': 'angle_deg', 'tol': 0.05},
    'units': {'kind': 'abs', 'tol': 0.6},
    'exact': {'kind': 'exact'},
}

"""
Explicit positions, (latitude, longitude) in units: the HPRC pad and zone (mission._HPRC_DEFAULT, whose corner
sums happen to be even), the plan B test points near 48 N (test_navigation.py), and the hemispheres and edges a
sweep would rarely hit.
"""
_HPRC_PAD: tuple = (255_143_790, -803_917_950)
_HPRC_ZONE: tuple = ((255_149_440, -803_929_720), (255_145_830, -803_911_110))
_ORIGIN_48: tuple = (480_000_000, 110_000_000)
_WIDE_48: tuple = ((480_010_000, 110_000_000), (480_000_000, 110_100_000))  # test_navigation's zone
_TALL_48: tuple = ((480_100_000, 110_000_000), (480_000_000, 110_010_000))

_PAIRS: tuple = (
    ('hprc_pad_to_zone_corner_west', _HPRC_PAD, _HPRC_ZONE[0]),
    ('hprc_pad_to_zone_corner_east', _HPRC_PAD, _HPRC_ZONE[1]),
    ('due_north_at_48', _ORIGIN_48, (480_010_000, 110_000_000)),
    ('due_east_at_48', _ORIGIN_48, (480_000_000, 110_010_000)),
    ('due_south_at_48', _ORIGIN_48, (479_990_000, 110_000_000)),
    ('due_west_at_48', _ORIGIN_48, (480_000_000, 109_990_000)),
    ('a_0p2_metre_step_at_48', _ORIGIN_48, (480_000_018, 110_000_000)),
    ('same_point', _ORIGIN_48, _ORIGIN_48),
    ('south_west_hemispheres', (-346_037_000, -583_816_000), (-346_047_000, -583_800_000)),
    ('south_east_hemispheres', (-338_688_000, 1_512_093_000), (-338_700_000, 1_512_110_000)),
    ('across_the_anti_meridian_east', (100_000_000, 1_799_995_000), (100_000_000, -1_799_995_000)),
    ('across_the_anti_meridian_west', (100_000_000, -1_799_995_000), (100_003_000, 1_799_990_000)),
    ('across_the_equator', (5_000, 300_000_000), (-5_000, 300_000_000)),
    ('five_kilometres_north_east', _ORIGIN_48, (480_317_600, 110_474_800)),
)

_ADVANCES: tuple = (
    ('east_100_at_48', _ORIGIN_48, 100.0, 0.0),
    ('south_250_at_48', _ORIGIN_48, 0.0, -250.0),
    ('north_west_at_48', _ORIGIN_48, -731.5, 412.25),
    ('a_0p2_metre_step_north', _ORIGIN_48, 0.0, 0.2),
    ('a_0p2_metre_step_east', _ORIGIN_48, 0.2, 0.0),
    ('no_offset', _ORIGIN_48, 0.0, 0.0),
    ('dead_reckoning_from_hprc', _HPRC_PAD, -38.25, 61.5),
    ('south_west_hemispheres', (-346_037_000, -583_816_000), 1200.0, -800.0),
    ('south_east_hemispheres', (-338_688_000, 1_512_093_000), -2500.0, 1750.0),
    ('across_the_equator', (5_000, 300_000_000), 10.0, -12.0),
    ('at_the_north_pole', (900_000_000, 0), 500.0, 0.0),
)

_COMPASS: tuple = (
    ('north', 0.0, 1.0), ('east', 1.0, 0.0), ('south', 0.0, -1.0), ('west', -1.0, 0.0),
    ('south_negative_zero', -0.0, -1.0), ('no_offset', 0.0, 0.0), ('three_four', 3.0, 4.0),
    ('minus_three_four', -3.0, -4.0), ('just_east_of_north', 1e-3, 1e3), ('just_west_of_north', -1e-3, 1e3),
    ('large', 4.0e3, -2.5e3), ('small', -2.5e-3, 1.0e-3),
)

_INSIDES: tuple = (
    ('centre', (480_005_000, 110_050_000), _WIDE_48),
    ('west_of_the_zone', (480_005_000, 109_900_000), _WIDE_48),
    ('the_top_left_corner', (480_010_000, 110_000_000), _WIDE_48),
    ('the_bottom_right_corner', (480_000_000, 110_100_000), _WIDE_48),
    ('one_unit_north', (480_010_001, 110_050_000), _WIDE_48),
    ('one_unit_south', (479_999_999, 110_050_000), _WIDE_48),
    ('one_unit_west', (480_005_000, 109_999_999), _WIDE_48),
    ('one_unit_east', (480_005_000, 110_100_001), _WIDE_48),
    ('corners_swapped', (480_005_000, 110_050_000), (_WIDE_48[1], _WIDE_48[0])),
    ('other_diagonal', (480_005_000, 110_050_000), ((480_000_000, 110_000_000), (480_010_000, 110_100_000))),
)

_STEERS: tuple = (
    ('west_of_the_zone_takes_the_left_gate', (480_005_000, 109_900_000), _WIDE_48),
    ('east_of_the_zone_takes_the_right_gate', (480_005_000, 110_200_000), _WIDE_48),
    ('inside_heads_for_the_centre', (480_005_000, 110_020_000), _WIDE_48),
    ('overshoot_turns_back_through_the_gate', (480_005_000, 110_110_000), _WIDE_48),
    ('north_west_of_a_tall_zone', (480_150_000, 109_950_000), _TALL_48),
    ('south_of_a_tall_zone', (479_900_000, 110_004_000), _TALL_48),
    ('hprc_pad', _HPRC_PAD, _HPRC_ZONE),
)

_CROSS_TRACKS: tuple = (
    ('east_of_a_north_line', (480_000_000, 110_010_000), _ORIGIN_48, 0.0),
    ('west_of_a_north_line', (480_000_000, 109_990_000), _ORIGIN_48, 0.0),
    ('ahead_on_the_line', (480_010_000, 110_000_000), _ORIGIN_48, 0.0),
    ('north_of_an_east_line', (480_010_000, 110_000_000), _ORIGIN_48, 90.0),
    ('behind_a_diagonal_line', (479_990_000, 109_990_000), _ORIGIN_48, 45.0),
)

_APPROACHES: tuple = (
    ('on_the_line_heading_east', (480_005_000, 110_050_000), _WIDE_48, 90.0, 3.0, 45.0),
    ('south_of_the_line_heading_east', (480_003_000, 110_050_000), _WIDE_48, 90.0, 3.0, 45.0),
    ('on_the_line_heading_west', (480_005_000, 110_050_000), _WIDE_48, 260.0, 3.0, 45.0),
    ('north_of_the_line_heading_west', (480_006_000, 110_020_000), _WIDE_48, 280.0, 1.5, 30.0),
    ('far_off_the_line_is_capped', (480_100_000, 110_050_000), _WIDE_48, 100.0, 3.0, 45.0),
    ('no_gain_flies_the_strip', (480_003_000, 110_050_000), _WIDE_48, 90.0, 0.0, 45.0),
    ('a_tall_strip_heading_south', (480_050_000, 110_007_000), _TALL_48, 170.0, 3.0, 45.0),
)


def _degrees(position: tuple) -> tuple:
    """Plan B's float (latitude, longitude) for a position in units."""
    return position[0] / _UNITS_PER_DEGREE, position[1] / _UNITS_PER_DEGREE


def _units(point: tuple) -> list:
    """A plan B (latitude, longitude) in units, unrounded: what a Rust position is compared with."""
    return [point[0] * _UNITS_PER_DEGREE, point[1] * _UNITS_PER_DEGREE]


def _whole_units(point: tuple) -> list:
    """
    A plan B point that IS a whole position (a corner, a gate, an even-sum centre), in integer units.

    Raises:
        ValueError - the point is no whole position, so it cannot compare exactly.
    """
    units = _units(point)
    whole = [round(value) for value in units]
    if any(abs(value - integer) > _EXACT_SLACK for value, integer in zip(units, whole)):
        raise ValueError('%r is not a whole position' % (point,))
    return whole


def _wrap_longitude(longitude: int) -> int:
    """A longitude in units, wrapped into [-180, 180) degrees."""
    return (longitude + _HALF_TURN_E7) % (2 * _HALF_TURN_E7) - _HALF_TURN_E7


def _moved(position: tuple, east_m: float, north_m: float) -> tuple:
    """A position moved by a metre offset, in whole units (the sweeps' generator), its longitude wrapped."""
    latitude = position[0] + round(north_m / commons.M_PER_DEG * _UNITS_PER_DEGREE)
    scale = commons.M_PER_DEG * math.cos(math.radians(position[0] / _UNITS_PER_DEGREE))
    return latitude, _wrap_longitude(position[1] + round(east_m / scale * _UNITS_PER_DEGREE))


def _random_position(generator: random.Random) -> tuple:
    """A position anywhere within the sweep latitudes, clear of the anti-meridian by ten degrees."""
    return (generator.randint(-_SWEEP_LATITUDE_E7, _SWEEP_LATITUDE_E7),
            generator.randint(-_HALF_TURN_E7 + 100_000_000, _HALF_TURN_E7 - 100_000_000))


def _random_zone(generator: random.Random) -> tuple:
    """
    A zone around a random centre, sides of 20 to 600 m, corners in a random diagonal order, both corner sums even.

    Returns:
        (first corner, second corner), positions in units.
    """
    centre = _random_position(generator)
    height = generator.uniform(*_ZONE_SIDE_M)
    width = generator.uniform(*_ZONE_SIDE_M)
    north_east = _moved(centre, width / 2.0, height / 2.0)
    south_west = _moved(centre, -width / 2.0, -height / 2.0)
    # one unit on the north-east corner makes both sums even: the centre is then a whole position on both sides
    north_east = (north_east[0] + (north_east[0] + south_west[0]) % 2,
                  north_east[1] + (north_east[1] + south_west[1]) % 2)
    corners = [((north_east[0], south_west[1]), (south_west[0], north_east[1])),
               ((south_west[0], north_east[1]), (north_east[0], south_west[1])),
               (south_west, north_east),
               (north_east, south_west)][generator.randrange(4)]
    return corners


def _near(generator: random.Random, centre: tuple, reach_m: float) -> tuple:
    """A position within `reach_m` metres of `centre`, on a random bearing."""
    reach = generator.uniform(0.0, reach_m)
    bearing = math.radians(generator.uniform(0.0, 360.0))
    return _moved(centre, reach * math.sin(bearing), reach * math.cos(bearing))


def _jittered(position: tuple) -> list:
    """The position moved _JITTER_UNITS along each axis, every way: the robustness re-runs' positions."""
    return [(position[0] + north, position[1] + east)
            for north in (-_JITTER_UNITS, 0, _JITTER_UNITS) for east in (-_JITTER_UNITS, 0, _JITTER_UNITS)]


def _steer(position: tuple, zone: tuple) -> tuple:
    """Plan B's steer() for positions in units: (bearing, waypoint, leg)."""
    return navigation.steer(_degrees(position), _degrees(zone[0]), _degrees(zone[1]))


def _steer_robust(position: tuple, zone: tuple) -> bool:
    """Whether steer() keeps its waypoint and leg when the position moves two units any way."""
    _bearing, waypoint, leg = _steer(position, zone)
    return all(_steer(moved, zone)[1:] == (waypoint, leg) for moved in _jittered(position))


def _approach(position: tuple, zone: tuple, heading: float, cross_gain: float, intercept_max: float) -> float:
    """Plan B's approach() for positions in units."""
    return navigation.approach(_degrees(position), _degrees(zone[0]), _degrees(zone[1]), heading, cross_gain,
                               intercept_max)


def _turn(first: float, second: float) -> float:
    """How far apart two headings are, the short way around, in degrees."""
    around = (first - second) % 360.0
    return min(around, 360.0 - around)


def _approach_robust(position: tuple, zone: tuple, heading: float, cross_gain: float, intercept_max: float) -> bool:
    """Whether approach() moves less than a degree when the position and the heading are jittered."""
    answer = _approach(position, zone, heading, cross_gain, intercept_max)
    return all(
        _turn(_approach(moved, zone, heading + turn, cross_gain, intercept_max), answer) < _APPROACH_STABLE_DEGREES
        for moved in _jittered(position) for turn in (-_JITTER_DEGREES, 0.0, _JITTER_DEGREES))


def _offset_case(name: str, start: tuple, end: tuple) -> dict:
    """One offset case: the two positions in units, plan B's (east, north) metres."""
    east, north = navigation.offset(*_degrees(start), *_degrees(end))
    return {'name': name, 'input': {'from': list(start), 'to': list(end)}, 'output': {'metres': [east, north]}}


def _range_bearing_case(name: str, start: tuple, end: tuple) -> dict:
    """One range_bearing case: plan B's distance in metres and bearing in degrees."""
    distance, bearing = navigation.range_bearing(*_degrees(start), *_degrees(end))
    return {'name': name, 'input': {'from': list(start), 'to': list(end)},
            'output': {'metres': distance, 'degrees': bearing}}


def _advance_case(name: str, position: tuple, east: float, north: float) -> dict:
    """One advance case: the start in units and the metre offset, plan B's moved position in (unrounded) units."""
    return {'name': name, 'input': {'position': list(position), 'east': east, 'north': north},
            'output': {'units': _units(navigation.advance(_degrees(position), east, north))}}


def _compass_case(name: str, east: float, north: float) -> dict:
    """One compass case: the metre offset, plan B's bearing."""
    return {'name': name, 'input': {'east': east, 'north': north},
            'output': {'degrees': navigation.compass(east, north)}}


def _inside_case(name: str, position: tuple, zone: tuple) -> dict:
    """One inside case: the position and the corners in units, plan B's verdict."""
    return {'name': name, 'input': {'position': list(position), 'top_left': list(zone[0]),
                                    'bottom_right': list(zone[1])},
            'output': {'exact': navigation.inside(_degrees(position), _degrees(zone[0]), _degrees(zone[1]))}}


def _steer_case(name: str, position: tuple, zone: tuple) -> dict:
    """
    One steer case: plan B's bearing, waypoint (whole units) and leg.

    Raises:
        ValueError - the case sits on the nearer-gate tie (see the module notes).
    """
    if not _steer_robust(position, zone):
        raise ValueError('steer %s: a two-unit move changes the waypoint' % name)
    bearing, waypoint, leg = _steer(position, zone)
    return {'name': name, 'input': {'position': list(position), 'top_left': list(zone[0]),
                                    'bottom_right': list(zone[1])},
            'output': {'degrees': bearing, 'exact': {'waypoint': _whole_units(waypoint), 'leg': leg}}}


def _cross_track_case(name: str, position: tuple, point: tuple, heading: float) -> dict:
    """One cross_track case: plan B's signed metres, right of the line positive."""
    return {'name': name, 'input': {'position': list(position), 'point': list(point), 'heading': heading},
            'output': {'metres': navigation.cross_track(_degrees(position), _degrees(point), heading)}}


def _approach_case(name: str, position: tuple, zone: tuple, heading: float, cross_gain: float,
                   intercept_max: float) -> dict:
    """
    One approach case: plan B's heading.

    Raises:
        ValueError - the case sits on the centreline flip (see the module notes).
    """
    if not _approach_robust(position, zone, heading, cross_gain, intercept_max):
        raise ValueError('approach %s: the jitter flips the centreline' % name)
    return {'name': name,
            'input': {'position': list(position), 'top_left': list(zone[0]), 'bottom_right': list(zone[1]),
                      'heading': heading, 'cross_gain': cross_gain, 'intercept_max': intercept_max},
            'output': {'degrees': _approach(position, zone, heading, cross_gain, intercept_max)}}


def _sweep_pairs(generator: random.Random) -> list:
    """Seeded (name, start, end) pairs: a start within +/-60 degrees of latitude, an end up to 3 km away."""
    pairs = []
    for index in range(_SWEEP):
        start = _random_position(generator)
        pairs.append(('sweep_%03d' % index, start, _near(generator, start, _SWEEP_REACH_M)))
    return pairs


def _sweep_advances(generator: random.Random) -> list:
    """Seeded advance cases: a start within +/-60 degrees of latitude, an offset of up to 3 km either way."""
    return [_advance_case('sweep_%03d' % index, _random_position(generator),
                          generator.uniform(-_SWEEP_REACH_M, _SWEEP_REACH_M),
                          generator.uniform(-_SWEEP_REACH_M, _SWEEP_REACH_M))
            for index in range(_SWEEP)]


def _sweep_compass(generator: random.Random) -> list:
    """Seeded compass cases: offsets of up to 3 km either way."""
    return [_compass_case('sweep_%03d' % index, generator.uniform(-_SWEEP_REACH_M, _SWEEP_REACH_M),
                          generator.uniform(-_SWEEP_REACH_M, _SWEEP_REACH_M))
            for index in range(_SWEEP)]


def _sweep_insides(generator: random.Random) -> list:
    """Seeded inside cases: a random zone and a position within 1.5 zone sizes of its centre."""
    cases = []
    for index in range(_SWEEP):
        zone = _random_zone(generator)
        centre = ((zone[0][0] + zone[1][0]) // 2, (zone[0][1] + zone[1][1]) // 2)
        cases.append(_inside_case('sweep_%03d' % index, _near(generator, centre, 1.5 * _ZONE_SIDE_M[1]), zone))
    return cases


def _sweep_steers(generator: random.Random) -> list:
    """Seeded steer cases: a random zone and a position up to 3 km from its centre, off the nearer-gate tie."""
    cases = []
    while len(cases) < _SWEEP:
        zone = _random_zone(generator)
        centre = ((zone[0][0] + zone[1][0]) // 2, (zone[0][1] + zone[1][1]) // 2)
        reach = _ZONE_SIDE_M[1] if generator.random() < 0.3 else _SWEEP_REACH_M
        position = _near(generator, centre, reach)
        if _steer_robust(position, zone):
            cases.append(_steer_case('sweep_%03d' % len(cases), position, zone))
    return cases


def _sweep_cross_tracks(generator: random.Random) -> list:
    """Seeded cross_track cases: a line through a random point on a random heading, a position up to 3 km off."""
    cases = []
    for index in range(_SWEEP):
        point = _random_position(generator)
        cases.append(_cross_track_case('sweep_%03d' % index, _near(generator, point, _SWEEP_REACH_M), point,
                                       generator.uniform(0.0, 360.0)))
    return cases


def _sweep_approaches(generator: random.Random) -> list:
    """Seeded approach cases: a random zone, a position near it, any heading and gains, off the centreline flip."""
    cases = []
    while len(cases) < _SWEEP:
        zone = _random_zone(generator)
        centre = ((zone[0][0] + zone[1][0]) // 2, (zone[0][1] + zone[1][1]) // 2)
        arguments = (_near(generator, centre, 300.0), zone, generator.uniform(0.0, 360.0),
                     generator.uniform(0.0, 5.0), generator.uniform(10.0, 60.0))
        if _approach_robust(*arguments):
            cases.append(_approach_case('sweep_%03d' % len(cases), *arguments))
    return cases


def build() -> dict:
    """
    The navigation fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, tolerance, offset, range_bearing, advance, compass, inside, steer, cross_track, approach}:
        each section {note, cases}, every case's output grouped by `compare`.
    """
    generator = random.Random(_SEED)
    pairs = list(_PAIRS) + _sweep_pairs(generator)
    return {
        'meta': {'source': 'src/glider/navigation.py, src/glider/commons.py'},
        'compare': _COMPARE,
        'tolerance': ('metres abs 0.5 and degrees angle_deg 0.05, the plan\'s navigation tolerances; advance() '
                      'positions abs 0.6 unit (the Rust answer is rounded to a whole unit); inside, the steer '
                      'waypoint and leg exactly. No case lies within two units or 0.01 degree of a decision.'),
        'offset': {'note': 'metres (east, north) from `from` to `to`',
                   'cases': [_offset_case(*pair) for pair in pairs]},
        'range_bearing': {'note': 'distance (metres) and bearing (degrees); distance() and bearing() answer the same',
                          'cases': [_range_bearing_case(*pair) for pair in pairs]},
        'advance': {'note': 'the position moved by (east, north) metres, in units',
                    'cases': [_advance_case(*case) for case in _ADVANCES] + _sweep_advances(generator)},
        'compass': {'note': 'the bearing of an (east, north) offset, degrees',
                    'cases': [_compass_case(*case) for case in _COMPASS] + _sweep_compass(generator)},
        'inside': {'note': 'whether the position is within the zone, edges included',
                   'cases': [_inside_case(*case) for case in _INSIDES] + _sweep_insides(generator)},
        'steer': {'note': 'bearing, waypoint and leg (1 target, 2 gate); steer_to answers the same',
                  'cases': [_steer_case(*case) for case in _STEERS] + _sweep_steers(generator)},
        'cross_track': {'note': 'signed metres off the line, right positive',
                        'cases': [_cross_track_case(*case) for case in _CROSS_TRACKS] + _sweep_cross_tracks(generator)},
        'approach': {'note': 'the final-approach heading along the strip centreline',
                     'cases': [_approach_case(*case) for case in _APPROACHES] + _sweep_approaches(generator)},
    }
