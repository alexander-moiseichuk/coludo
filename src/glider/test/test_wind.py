# On-board test for the wind estimator (wind.py): the triangle recovers a known wind from the ground
# velocity vs the air velocity, and the crosswind is bias-free under an airspeed over-read (only the
# along-heading component absorbs the airspeed error). Pure math. Run by `make test`.

import math

import wind


def _ground(airspeed, heading_deg, wind_e, wind_n):
    """The (course_deg, ground_speed) a GNSS reads = air velocity (airspeed along heading) + wind."""
    h = math.radians(heading_deg)
    ge = airspeed * math.sin(h) + wind_e
    gn = airspeed * math.cos(h) + wind_n
    return math.degrees(math.atan2(ge, gn)) % 360.0, math.sqrt(ge * ge + gn * gn)


def test():
    # no wind, flying north: the ground track equals the heading -> wind ~0
    est = wind.WindEstimator(alpha=1.0)  # alpha 1 -> instantaneous, no EMA lag for the check
    crs, gs = _ground(14.0, 0.0, 0.0, 0.0)
    est.update(crs, gs, 14.0, 0.0)
    assert est.speed() < 0.05

    # a 5 m/s wind blowing EAST (from the west): recovered as +5 east / ~0 north; FROM = 270 deg
    est = wind.WindEstimator(alpha=1.0)
    crs, gs = _ground(14.0, 0.0, 5.0, 0.0)  # heading north, wind pushing the track east
    est.update(crs, gs, 14.0, 0.0)
    east, north = est.components()
    assert abs(east - 5.0) < 0.1 and abs(north) < 0.1
    assert abs(est.speed() - 5.0) < 0.1 and abs(est.direction() - 270.0) < 1.0
    assert est.stats()['method'] == 'triangle'

    # crosswind is BIAS-FREE: an airspeed over-read leaves the crosswind (east) clean; only the
    # along-heading (north) component absorbs the error
    est = wind.WindEstimator(alpha=1.0)
    est.update(crs, gs, 16.0, 0.0)  # airspeed over-read by 2 m/s
    east, north = est.components()
    assert abs(east - 5.0) < 0.1        # crosswind untouched by the airspeed bias
    assert abs(north + 2.0) < 0.1       # the north component absorbs the 2 m/s over-read (biased)

    print('ok: wind -- triangle recovers wind, crosswind bias-free under an airspeed over-read')


test()
