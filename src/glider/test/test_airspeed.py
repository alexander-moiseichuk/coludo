"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for airspeed.AirspeedEstimator (the fin-governor airspeed fusion): accel-integration
backbone, ceiling clamp, and the sanity-gated GNSS complementary blend (no-fix and out-of-range
rejected, in-range blended, repeated good fixes converge). Pure math, no hardware. Run by `make test`.
"""

from airspeed import AirspeedEstimator


def test_predict_integration():
    estimator = AirspeedEstimator()
    for _ in range(50):  # 5g net (~49 m/s^2) for 0.5 s -> ~24.5 m/s
        estimator.predict(49.0, 0.01)
    assert 24.0 < estimator.value() < 25.0
    estimator.predict(-3000.0, 0.01)  # -30 m/s step cannot drive speed negative -> floors at 0
    assert estimator.value() == 0.0


def test_ceiling_clamp():
    estimator = AirspeedEstimator(ceiling_ms=60.0)
    for _ in range(100):
        estimator.predict(100.0, 0.1)  # absurd accel -> clamps at the ceiling
    assert estimator.value() == 60.0


def test_gnss_correct_gated():
    estimator = AirspeedEstimator(ceiling_ms=60.0, gnss_gain=0.2)
    for _ in range(40):  # integrate to ~20 m/s
        estimator.predict(50.0, 0.01)
    backbone = estimator.value()
    assert 19.5 < backbone < 20.5

    estimator.correct(14.0, has_fix=False)            # no fix -> ignored
    assert estimator.value() == backbone
    estimator.correct(120.0, has_fix=True)            # above ceiling -> glitch, ignored
    assert estimator.value() == backbone
    estimator.correct(10.0, has_fix=True)             # valid -> blend by gain 0.2
    assert abs(estimator.value() - (backbone + 0.2 * (10.0 - backbone))) < 1e-6

    for _ in range(60):                               # a run of good fixes converges toward GNSS
        estimator.correct(10.0, has_fix=True)
    assert abs(estimator.value() - 10.0) < 0.5


def test_confidence_and_seed():
    # a fresh estimator (cold boot / mid-air reset) is NOT confident -- it reads 0 before anything charges
    estimator = AirspeedEstimator(gnss_gain=0.2)
    assert not estimator.confident() and estimator.value() == 0.0

    # a tiny accel charge (below the clearly-airborne threshold) does not yet earn trust
    estimator.predict(1.0, 0.1)  # -> 0.1 m/s, well under 5
    assert not estimator.confident()

    # the FIRST accepted GNSS fix while un-confident SEEDS directly (full set, not a 20% crawl to 4.88)
    estimator.correct(24.0, has_fix=True)
    assert estimator.confident() and abs(estimator.value() - 24.0) < 1e-6

    # once confident, further fixes BLEND (not re-seed)
    estimator.correct(20.0, has_fix=True)
    assert abs(estimator.value() - (24.0 + 0.2 * (20.0 - 24.0))) < 1e-6  # 23.2

    # accel alone can earn trust too: charged past the clearly-airborne threshold (boost)
    charged = AirspeedEstimator()
    for _ in range(20):
        charged.predict(49.0, 0.01)  # -> ~9.8 m/s, past 5
    assert charged.confident()


def test_pitot_measure():
    """
    The DIRECT pitot measurement: seed while un-confident, blend once confident, latch confidence only
    past the airborne threshold (so a 0 on the pad never trips the cap off an un-charged low speed).
    """
    estimator = AirspeedEstimator()
    # a pad reading (0 m/s) updates the speed but does NOT earn trust (would open the cap off a low speed)
    estimator.measure(0.0, 0.5)
    assert estimator.value() == 0.0 and not estimator.confident()
    # the first airborne reading SEEDS directly (a measurement, not a crawl) and latches confidence
    estimator.measure(15.0, 0.5)
    assert abs(estimator.value() - 15.0) < 1e-6 and estimator.confident()
    # once confident, further readings BLEND by gain (higher than the GNSS gain -- it is truth)
    estimator.measure(17.0, 0.5)
    assert abs(estimator.value() - (15.0 + 0.5 * (17.0 - 15.0))) < 1e-6  # 16.0, not a re-seed to 17


test_predict_integration()
test_ceiling_clamp()
test_gnss_correct_gated()
test_confidence_and_seed()
test_pitot_measure()
print('ok: airspeed -- accel backbone, ceiling clamp, GNSS blend + convergence, confidence + first-fix seed, '
      'pitot direct-measure')
