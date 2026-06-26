# On-board test for airspeed.AirspeedEstimator (the fin-governor airspeed fusion): accel-integration
# backbone, ceiling clamp, and the sanity-gated GNSS complementary blend (no-fix and out-of-range
# rejected, in-range blended, repeated good fixes converge). Pure math, no hardware. Run by `make test`.

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


test_predict_integration()
test_ceiling_clamp()
test_gnss_correct_gated()
print('ok: airspeed -- accel-integrate backbone, ceiling clamp, sanity-gated GNSS blend + convergence')
