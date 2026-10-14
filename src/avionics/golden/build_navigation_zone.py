"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture navigation_zone.json: plan B's landing-zone geometry, the part of src/glider/navigation.py the
Mission reads -- navigation.distance, navigation.zone (target + the two short-side gates) and
navigation.zone_aspect.

The Rust port (crates/control, module navigation) keeps a position as integer 1e-7 degree units, so every input
here is written in those units and plan B is called with units / 1e7, the float the same position is in plan B.
Both sides start from the same point, and the Rust test never re-derives an input.

The three functions answer in three units, so the fixture has three sections, each a golden fixture of its own
({compare, cases}):
  * distance -- metres, abs 0.01 m. The Rust distance subtracts integer positions and scales the difference in
    f32: under 3e-7 of the distance up to 50 degrees of latitude (about 1 mm at 5 km), growing with
    tan(latitude) to 6e-7 at the sweep's 70 degrees (3 mm), so a centimetre is still a threefold margin. The
    cases stay within 5 km, the scale a glider flies; at 100 km an f32 metre is itself a centimetre. The one
    case past 70 degrees, near the pole, is under 10 m long.
  * zone -- the points in 1e-7 degree units, abs 0.501 unit. The corners pass through exactly; the centre is
    the integer mean, rounded toward zero, at most half a unit from plan B's exact average (0.50000006 measured,
    the rest is plan B's own float rounding). A one-unit slip -- truncation for rounding, an off-by-one -- fails.
  * zone_aspect -- the long/short ratio, rel 1e-5. f32 spans hold it to about 3e-7; the endgame compares it
    with thresholds of 2 and 6.

Cases stay off the decision threshold: zone() orients by comparing its two sides, and a sweep zone whose sides
are within 0.1 % of each other is skipped, since one ulp could flip it. The exact ties kept on purpose are
squares centred on the equator, whose two sides are bit-identical in both languages.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import math
import random

import commons
import navigation

_SEED: int = 20261006  # the sweep's random.Random seed: the fixture is a function of the code alone
_SWEEP_DISTANCES: int = 200
_SWEEP_ZONES: int = 120
_MAXIMUM_DISTANCE_M: float = 5000.0  # the sweep's reach (see the module notes)
_ZONE_SIDE_M: tuple = (1.0, 2000.0)  # the sweep's zone side lengths, metres
_SWEEP_LATITUDE_E7: int = 700_000_000  # sweep latitudes stay within +/-70 degrees
_NEAR_TIE: float = 1.001  # a sweep zone with an aspect in (1, this) is skipped: its orientation is a coin toss
_UNITS_PER_DEGREE: float = 1e7  # a Rust Position counts 1e-7 degrees
_HALF_TURN_E7: int = 1_800_000_000  # 180 degrees in units
_ZONE_TOLERANCE_UNITS: float = 0.501  # half a unit (the Rust centre rounds toward zero) plus plan B's float slack

"""
Explicit positions, as (latitude, longitude) in units: the HPRC pad and zone (mission._HPRC_DEFAULT), the
plan B test points near 48 N (test_navigation.py), and the hemispheres and edges a sweep would rarely hit.
"""
_HPRC_PAD: tuple = (255_143_790, -803_917_950)
_HPRC_ZONE: tuple = ((255_149_440, -803_929_720), (255_145_830, -803_911_110))

_DISTANCES: tuple = (
    ('hprc_pad_to_zone_corner_west', _HPRC_PAD, _HPRC_ZONE[0]),
    ('hprc_pad_to_zone_corner_east', _HPRC_PAD, _HPRC_ZONE[1]),
    ('north_0001_degree_at_48', (480_000_000, 110_000_000), (480_010_000, 110_000_000)),
    ('east_0001_degree_at_48', (480_000_000, 110_000_000), (480_000_000, 110_010_000)),
    ('a_0p2_metre_step_at_48', (480_000_000, 110_000_000), (480_000_018, 110_000_000)),
    ('same_point', (480_000_000, 110_000_000), (480_000_000, 110_000_000)),
    ('south_west_hemispheres', (-346_037_000, -583_816_000), (-346_047_000, -583_800_000)),
    ('south_east_hemispheres', (-338_688_000, 1_512_093_000), (-338_700_000, 1_512_110_000)),
    ('across_the_anti_meridian', (100_000_000, 1_799_995_000), (100_000_000, -1_799_995_000)),
    ('across_the_equator', (5_000, 300_000_000), (-5_000, 300_000_000)),
    ('near_the_north_pole', (895_000_000, 0), (895_000_000, 100_000)),
    ('five_kilometres', (480_000_000, 110_000_000), (480_449_157, 110_000_000)),
)

_ZONES: tuple = (
    ('hprc', _HPRC_ZONE[0], _HPRC_ZONE[1]),
    ('wide_at_48', (480_010_000, 110_000_000), (480_000_000, 110_100_000)),
    ('tall_at_48', (480_100_000, 110_000_000), (480_000_000, 110_010_000)),
    ('wide_at_48_other_diagonal', (480_000_000, 110_000_000), (480_010_000, 110_100_000)),
    ('wide_at_48_corners_swapped', (480_000_000, 110_100_000), (480_010_000, 110_000_000)),
    ('south_west_hemispheres', (-346_030_000, -583_830_000), (-346_040_000, -583_810_000)),
    ('south_east_tall', (-338_680_000, 1_512_090_000), (-338_720_000, 1_512_100_000)),
    ('square_at_null_island', (5_000, -5_000), (-5_000, 5_000)),
    ('square_half_degree_on_the_equator', (2_500_000, 300_000_000), (-2_500_000, 305_000_000)),
    ('line_east_west', (480_000_000, 110_000_000), (480_000_000, 110_020_000)),
    ('line_north_south', (480_010_000, 110_000_000), (480_000_000, 110_000_000)),
    ('a_point', (480_000_000, 110_000_000), (480_000_000, 110_000_000)),
    ('line_at_the_north_pole', (900_000_000, 0), (900_000_000, 10_000)),
    ('odd_centre_rounds', (480_000_003, 110_000_001), (480_000_000, 110_020_000)),
    ('across_the_anti_meridian', (100_005_000, 1_799_995_000), (99_995_000, -1_799_995_000)),
    ('very_elongated', (480_000_100, 110_000_000), (480_000_000, 110_200_000)),
)


def _degrees(position: tuple) -> tuple:
    """Plan B's float (latitude, longitude) for a position in units."""
    return position[0] / _UNITS_PER_DEGREE, position[1] / _UNITS_PER_DEGREE


def _units(position: tuple) -> list:
    """A plan B (latitude, longitude) in units, unrounded: what the Rust integers are compared with."""
    return [position[0] * _UNITS_PER_DEGREE, position[1] * _UNITS_PER_DEGREE]


def _wrap_longitude(longitude: int) -> int:
    """A longitude in units, wrapped into [-180, 180) degrees."""
    return (longitude + _HALF_TURN_E7) % (2 * _HALF_TURN_E7) - _HALF_TURN_E7


def _moved(position: tuple, east_m: float, north_m: float) -> tuple:
    """
    A position moved by a metre offset, in whole units (the sweep's destination generator).

    Args:
        position - the start, (latitude, longitude) in units.
        east_m, north_m - the offset in metres.

    Returns:
        The moved position in units, its longitude wrapped onto the globe.
    """
    latitude = position[0] + round(north_m / commons.M_PER_DEG * _UNITS_PER_DEGREE)
    scale = commons.M_PER_DEG * math.cos(math.radians(position[0] / _UNITS_PER_DEGREE))
    return latitude, _wrap_longitude(position[1] + round(east_m / scale * _UNITS_PER_DEGREE))


def _distance_case(name: str, start: tuple, end: tuple) -> dict:
    """One distance case: the two positions in units, plan B's metres."""
    return {'name': name, 'input': {'from': list(start), 'to': list(end)},
            'output': navigation.distance(*_degrees(start), *_degrees(end))}


def _zone_case(name: str, top_left: tuple, bottom_right: tuple) -> dict:
    """One zone case: the two corners in units, plan B's target and gates in (unrounded) units."""
    target, gate_a, gate_b = navigation.zone(_degrees(top_left), _degrees(bottom_right))
    return {'name': name, 'input': {'top_left': list(top_left), 'bottom_right': list(bottom_right)},
            'output': {'target': _units(target), 'gate_a': _units(gate_a), 'gate_b': _units(gate_b)}}


def _aspect_case(name: str, top_left: tuple, bottom_right: tuple) -> dict:
    """One zone_aspect case: the two corners in units, plan B's ratio."""
    return {'name': name, 'input': {'top_left': list(top_left), 'bottom_right': list(bottom_right)},
            'output': navigation.zone_aspect(_degrees(top_left), _degrees(bottom_right))}


def _sweep_distances(generator: random.Random) -> list:
    """
    Seeded distance pairs: a start anywhere within +/-70 degrees of latitude, an end up to 5 km away on any bearing.

    Args:
        generator - the seeded random source.

    Returns:
        The (name, start, end) triples, positions in units.
    """
    pairs = []
    for index in range(_SWEEP_DISTANCES):
        start = (generator.randint(-_SWEEP_LATITUDE_E7, _SWEEP_LATITUDE_E7),
                 generator.randint(-_HALF_TURN_E7, _HALF_TURN_E7 - 1))
        reach = generator.uniform(0.0, _MAXIMUM_DISTANCE_M)
        bearing = math.radians(generator.uniform(0.0, 360.0))
        pairs.append(('sweep_%03d' % index, start, _moved(start, reach * math.sin(bearing), reach * math.cos(bearing))))
    return pairs


def _sweep_zones(generator: random.Random) -> list:
    """
    Seeded zones: a centre within +/-70 degrees of latitude, sides of 1 m to 2 km, corners in any diagonal order.

    A zone whose sides are within 0.1 % of each other is skipped (see the module notes).

    Args:
        generator - the seeded random source.

    Returns:
        The (name, first corner, second corner) triples, positions in units.
    """
    zones = []
    while len(zones) < _SWEEP_ZONES:
        centre = (generator.randint(-_SWEEP_LATITUDE_E7, _SWEEP_LATITUDE_E7),
                  generator.randint(-_HALF_TURN_E7 + 10_000_000, _HALF_TURN_E7 - 10_000_000))
        height = generator.uniform(*_ZONE_SIDE_M)
        width = generator.uniform(*_ZONE_SIDE_M)
        north_east = _moved(centre, width / 2.0, height / 2.0)
        south_west = _moved(centre, -width / 2.0, -height / 2.0)
        corners = [((north_east[0], south_west[1]), (south_west[0], north_east[1])),  # top-left, bottom-right
                   ((south_west[0], north_east[1]), (north_east[0], south_west[1])),  # bottom-right, top-left
                   (south_west, north_east),                                          # bottom-left, top-right
                   (north_east, south_west)][generator.randrange(4)]                 # top-right, bottom-left
        aspect = navigation.zone_aspect(_degrees(corners[0]), _degrees(corners[1]))
        if 1.0 < aspect < _NEAR_TIE:
            continue
        zones.append(('sweep_%03d' % len(zones), corners[0], corners[1]))
    return zones


def build() -> dict:
    """
    The navigation_zone fixture document.

    Args:
        (none)

    Returns:
        {meta, distance, zone, zone_aspect}: each section a {compare, note, cases} golden fixture of its own.
    """
    generator = random.Random(_SEED)
    distances = list(_DISTANCES) + _sweep_distances(generator)
    zones = list(_ZONES) + _sweep_zones(generator)
    return {
        'meta': {'source': 'src/glider/navigation.py, src/glider/commons.py'},
        'distance': {
            'compare': {'kind': 'abs', 'tol': 0.01},
            'note': ('metres; f32 on integer differences is under 3e-7 of the distance up to 50 degrees of latitude, '
                     '6e-7 at 70; the cases reach 5 km'),
            'cases': [_distance_case(*case) for case in distances],
        },
        'zone': {
            'compare': {'kind': 'abs', 'tol': _ZONE_TOLERANCE_UNITS},
            'note': ('1e-7 degree units; the Rust centre is the integer mean, at most half a unit off, so a '
                     'one-unit slip fails'),
            'cases': [_zone_case(*case) for case in zones],
        },
        'zone_aspect': {
            'compare': {'kind': 'rel', 'tol': 1e-05},
            'note': 'the long/short ratio; f32 sides hold it to ~3e-7',
            'cases': [_aspect_case(*case) for case in zones],
        },
    }
