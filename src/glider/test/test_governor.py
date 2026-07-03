# On-board test for the dynamic-pressure fin governor (governor.py): the mixer authority cap
# (1/v² schedule × safety multiplier), the accel-backbone + GNSS-corrector estimator wiring, and the
# adaptive throttle with its full-rate overrides (pre-glide / absolute overspeed / dive). Pure logic
# with injected stub handles -- no Flight task, no databoard. Run by `make test`.

import fixed
import governor


class _ValueHandle:
    """value() stand-in for a databoard Parameter (the accel handle)."""

    def __init__(self, value=None):
        self.value_now = value

    def value(self):
        return self.value_now


class _ReadHandle:
    """read() stand-in for a databoard Parameter (the GNSS speed handle)."""

    def __init__(self):
        self.reading = (None, None, None)

    def read(self):
        return self.reading


class _StubMixer:
    def __init__(self):
        self.limit = 45


def _build(config=None, multiplier=1.0):
    """A governor over fresh stubs; returns (governor, mixer, accel, gnss_speed)."""
    mix = _StubMixer()
    accel = _ValueHandle((0.0, 0.0, 1.0))  # exactly 1 g -> net accel 0 -> predict() is a no-op
    gnss_speed = _ReadHandle()
    unit = governor.Governor(governor.GovernorConfig(config or {}), mix, accel, gnss_speed, multiplier)
    return unit, mix, accel, gnss_speed


def test_authority_cap():
    """The 1/v² deflection schedule lands in mixer.limit, scaled by the safety multiplier."""
    unit, mix, _accel, _speed = _build()
    unit._estimator._speed = 0.0
    unit.step(0.01, True, 0)  # pre-glide -> full rate -> the cap updates every step
    assert mix.limit == 45  # 0 m/s -> full 45 deg authority
    unit._estimator._speed = 40.0
    unit.step(0.01, True, 0)
    assert mix.limit == 8  # fin_deflection_limit(40) -> 8 deg
    capped, capped_mix, _a, _s = _build(multiplier=0.5)  # the safety dial halves the whole schedule
    capped._estimator._speed = 0.0
    capped.step(0.01, True, 0)
    assert capped_mix.limit == 22  # int(45 * 0.5)
    assert capped_mix.limit >= 1  # the cap never reaches 0 (always-some authority)


def test_estimator_wiring():
    """predict() integrates the accel backbone; correct() blends a sane GNSS fix; missing readings
    degrade gracefully (backbone kept, no crash)."""
    unit, _mix, accel, gnss_speed = _build()
    accel.value_now = (0.0, 0.0, 6.0)  # 6 g -> ~49 m/s^2 net along the path
    unit.step(0.1, True, 0)
    integrated = unit.airspeed()
    assert integrated > 0.0  # integrated off zero
    gnss_speed.reading = (10.0, 'gnss', 0)  # a live fix pulls the estimate toward GNSS
    accel.value_now = (0.0, 0.0, 1.0)  # net 0 -> only the corrector moves it
    unit.step(0.1, True, 0)
    assert abs(unit.airspeed() - integrated) < abs(10.0 - integrated)  # blended, not replaced
    # negative: accel absent -> predict skipped; GNSS absent -> correct(0, False) keeps the backbone
    accel.value_now = None
    gnss_speed.reading = (None, None, None)
    before = unit.airspeed()
    unit.step(0.1, True, 0)
    assert unit.airspeed() == before  # nothing moved, nothing raised


def test_adaptive_throttle_and_overrides():
    """Settled glide skips the float update on the adaptive interval; the pre-glide / absolute
    overspeed / dive triggers each restore full rate at once. Observable: _accum_s == 0 iff the
    update ran (it resets on update)."""
    unit, _mix, _accel, _speed = _build({'airspeed_full_speed': 20.0, 'airspeed_dive_pitch': -45.0})
    unit._interval_s = 1.0  # force a long throttle interval so only an override can fire the update
    # (a) settled: low speed + normal pitch + accum under the interval -> THROTTLED (update skipped)
    unit._estimator._speed = 14.0
    unit._accum_s = 0.0
    unit.step(0.01, False, fixed.from_float(-6.0))
    assert unit._accum_s > 0.0, 'settled glide should throttle (skip the update)'
    # (b) ABSOLUTE speed over the limit at a normal attitude (crosswind/gust) -> full rate restored
    unit._estimator._speed = 30.0
    unit._accum_s = 0.5
    unit.step(0.01, False, fixed.from_float(-6.0))
    assert unit._accum_s == 0.0, 'overspeed must re-arm full rate regardless of attitude'
    # (c) a steep nose-down while the (throttled) estimate is still low -> dive override fires first
    unit._estimator._speed = 14.0
    unit._accum_s = 0.5
    unit.step(0.01, False, fixed.from_float(-60.0))
    assert unit._accum_s == 0.0, 'a dive must re-arm full rate before the estimate shows the overspeed'
    # (d) the caller's full_rate_override (the flight task's pre-glide) always runs full rate
    unit._estimator._speed = 0.0
    unit._accum_s = 0.0
    unit.step(0.01, True, 0)
    assert unit._accum_s == 0.0, 'the full-rate override (pre-glide) must never throttle'


def test_interval_adaptation():
    """The throttle interval snaps to the floor when the estimate moves and grows toward the ceiling
    as it settles; the accumulated dt reaches the integrator either way (cadence-independent)."""
    unit, _mix, accel, gnss_speed = _build(
        {'airspeed_min_ms': 40, 'airspeed_max_ms': 100, 'airspeed_settle': 0.5})
    gnss_speed.reading = (None, None, None)
    # settled: net-zero accel, due interval -> the interval GROWS toward the ceiling
    unit._estimator._speed = 5.0
    unit._interval_s = unit._config.floor_s
    unit._accum_s = unit._config.floor_s  # due now
    unit.step(0.0, False, 0)
    assert unit._interval_s == min(unit._config.ceiling_s, 2 * unit._config.floor_s)
    grown = unit._interval_s
    # moving: a hard acceleration between updates -> the interval SNAPS back to the floor
    accel.value_now = (0.0, 0.0, 6.0)  # ~49 m/s^2 -> the estimate moves >> settle over 0.1 s
    unit._accum_s = grown  # due now
    unit.step(0.0, False, 0)
    assert unit._interval_s == unit._config.floor_s
    # ceiling clamp: repeated settled updates never grow past the ceiling
    accel.value_now = (0.0, 0.0, 1.0)
    for _ in range(10):
        unit._accum_s = unit._interval_s
        unit.step(0.0, False, 0)
    assert unit._interval_s == unit._config.ceiling_s


test_authority_cap()
test_estimator_wiring()
test_adaptive_throttle_and_overrides()
test_interval_adaptation()
print('ok: governor -- 1/v2 authority cap, estimator wiring, adaptive throttle, full-rate overrides')
