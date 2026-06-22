# On-board test for the landing-zone navigation geometry (navigation.py): bearing/distance, zone -> target +
# short-side gates (both orientations), inside(), and steer() (nearer gate -> target). Pure math, no
# hardware. Run by `make test`.

import navigation

# a reference near 48 N (lon degrees ~ 0.669 of a lat degree here)
_LAT: float = 48.0
_LON: float = 11.0


def _close(a, b, tol=1.0):
    return abs(a - b) <= tol


def test_bearing_distance():
    assert _close(navigation.bearing(_LAT, _LON, _LAT + 0.001, _LON), 0.0)      # due north
    assert _close(navigation.bearing(_LAT, _LON, _LAT, _LON + 0.001), 90.0)     # due east
    assert _close(navigation.bearing(_LAT, _LON, _LAT - 0.001, _LON), 180.0)    # due south
    assert _close(navigation.bearing(_LAT, _LON, _LAT, _LON - 0.001), 270.0)    # due west
    assert _close(navigation.distance(_LAT, _LON, _LAT + 0.001, _LON), 111.32, 0.5)  # 0.001 deg lat
    # longitude metres shrink by cos(lat) (~0.669 here), so the same deg east is shorter than north
    assert navigation.distance(_LAT, _LON, _LAT, _LON + 0.001) < navigation.distance(_LAT, _LON, _LAT + 0.001, _LON)


def test_zone_orientation():
    # horizontally (longitude) stretched: 0.001 deg tall x 0.010 deg wide -> gates on left/right edges
    target, gate_a, gate_b = navigation.zone((48.001, 11.000), (48.000, 11.010))
    assert _close(target[0], 48.0005, 1e-4) and _close(target[1], 11.005, 1e-4)
    assert _close(gate_a[1], 11.000, 1e-4) and _close(gate_b[1], 11.010, 1e-4)  # left + right edge
    assert _close(gate_a[0], 48.0005, 1e-4) and _close(gate_b[0], 48.0005, 1e-4)  # at vertical middle

    # vertically (latitude) stretched: 0.010 deg tall x 0.001 deg wide -> gates on top/bottom edges
    target, gate_a, gate_b = navigation.zone((48.010, 11.000), (48.000, 11.001))
    assert _close(gate_a[0], 48.010, 1e-4) and _close(gate_b[0], 48.000, 1e-4)  # top + bottom edge
    assert _close(gate_a[1], 11.0005, 1e-4) and _close(gate_b[1], 11.0005, 1e-4)  # at horizontal middle


def test_inside_and_steer():
    tl, br = (48.001, 11.000), (48.000, 11.010)  # horizontally stretched zone (gates left/right)
    assert navigation.inside((48.0005, 11.005), tl, br) is True  # the centre is inside
    assert navigation.inside((48.0005, 10.990), tl, br) is False  # west of the zone

    # west of the zone -> head for the LEFT gate (the nearer entrance), leg 'gate', bearing ~east
    heading, waypoint, leg = navigation.steer((48.0005, 10.990), tl, br)
    assert leg == 'gate' and _close(waypoint[1], 11.000, 1e-4) and _close(heading, 90.0, 5.0)

    # east of the zone -> head for the RIGHT gate
    heading, waypoint, leg = navigation.steer((48.0005, 11.020), tl, br)
    assert leg == 'gate' and _close(waypoint[1], 11.010, 1e-4) and _close(heading, 270.0, 5.0)

    # inside the zone -> track to the centre (target), not a gate
    heading, waypoint, leg = navigation.steer((48.0005, 11.002), tl, br)
    assert leg == 'target' and _close(waypoint[1], 11.005, 1e-4) and _close(heading, 90.0, 5.0)


def test_overshoot_loop():
    # the loop: entered via the left gate (eastbound), crossed the zone, exited the far (east) side
    # without landing -> the gate it just crossed is now nearest -> steer turns back (~180) and
    # re-approaches through the RIGHT gate. Stateless: no waypoint memory, just per-tick nearest-gate.
    tl, br = (48.001, 11.000), (48.000, 11.010)
    heading, waypoint, leg = navigation.steer((48.0005, 11.011), tl, br)  # just past the east edge
    assert leg == 'gate' and _close(waypoint[1], 11.010, 1e-4)  # the right gate (the one just crossed)
    assert _close(heading, 270.0, 5.0)  # ~180 from the eastbound entry -> turn back west through it


test_bearing_distance()
test_zone_orientation()
test_inside_and_steer()
test_overshoot_loop()
print('ok: navigation -- bearing/distance, zone target + gates, inside, steer (gate->target), overshoot loop')
