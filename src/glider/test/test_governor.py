"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the dynamic-pressure fin governor (governor.py): the mixer authority cap
(1/v² schedule × safety multiplier), the accel-backbone + GNSS-corrector estimator wiring, and the
adaptive throttle with its full-rate overrides (pre-glide / absolute overspeed / dive). Pure logic
with injected stub handles -- no Flight task, no databoard. Run by `make test`.
"""

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
    """The 1/v² deflection schedule lands in mixer.limit (when the estimate is TRUSTED), × the multiplier."""
    unit, mix, _accel, _speed = _build()
    unit._estimator._confident = True  # a trusted estimate -> the cap tracks the live speed
    unit._estimator._speed = 0.0
    unit.step(0.01, True, 0)  # pre-glide -> full rate -> the cap updates every step
    assert mix.limit == 45  # 0 m/s (genuinely slow, low q) -> full 45 deg authority
    unit._estimator._speed = 40.0
    unit.step(0.01, True, 0)
    assert mix.limit == 8  # fin_deflection_limit(40) -> 8 deg
    capped, capped_mix, _a, _s = _build(multiplier=0.5)  # the safety dial halves the whole schedule
    capped._estimator._confident = True
    capped._estimator._speed = 0.0
    capped.step(0.01, True, 0)
    assert capped_mix.limit == 22  # int(45 * 0.5)
    assert capped_mix.limit >= 1  # the cap never reaches 0 (always-some authority)


def test_unconfident_cap_floor():
    """
    Until the estimate is TRUSTED (a fresh construct / mid-air reset reads 0 before anything charges
    it), the cap comes from the conservative unconfident_ms, NOT the un-charged 0 -- which would open
    authority to the full 45deg at high q, the unsafe direction (finding 23.4). It turns confident on
    the first accel-charge or the first sane GNSS fix.
    """
    unit, mix, _accel, gnss_speed = _build({'airspeed_unconfident_ms': 30.0})
    assert not unit._estimator.confident()  # fresh: no trusted speed yet
    unit.step(0.01, True, 0)  # airborne with a 0 estimate -> the conservative floor, NOT 45
    assert mix.limit == 14  # fin_deflection_limit(30) -> 14, not fin_deflection_limit(0) == 45
    # a sane GNSS fix anchors it -> confident -> the cap tracks the real (here genuinely slow) speed
    gnss_speed.reading = (4.0, 'gnss', 0)
    unit.step(0.01, True, 0)
    assert unit._estimator.confident() and mix.limit == 45  # 4 m/s, low q -> full authority is now correct


def test_estimator_wiring():
    """
    predict() integrates the accel backbone; correct() blends a sane GNSS fix.

    Missing readings degrade gracefully (backbone kept, no crash).
    """
    unit, _mix, accel, gnss_speed = _build()
    accel.value_now = (0.0, 0.0, 6.0)  # 6 g -> ~49 m/s^2 net along the path
    unit.step(0.1, True, 0)
    unit.step(0.1, True, 0)  # charge past the clearly-airborne threshold (~9.8 m/s) -> confident -> BLEND
    integrated = unit.airspeed()
    assert integrated > 5.0 and unit._estimator.confident()  # a trusted, clearly-airborne backbone
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


def test_gnss_jump_rejected():
    """
    A GNSS ground-speed JUMP (more than one max_wind off the airspeed estimate) is rejected, not
    blended.

    A spurious low speed would loosen the fin cap (the unsafe direction). A fix WITHIN the max-wind
    band still blends.
    """
    unit, _mix, accel, gnss_speed = _build({'wind': {'max_speed': 15.0}})
    accel.value_now = (0.0, 0.0, 3.0)  # integrate the accel backbone up to a real airspeed
    for _ in range(8):
        unit.step(0.1, True, 0)
    accel.value_now = (0.0, 0.0, 1.0)  # net ~0 -> only the corrector could move the estimate now
    settled = unit.airspeed()
    assert settled > 5.0
    gnss_speed.reading = (settled + 60.0, 'gnss', 0)  # a jump 60 m/s off -> beyond max_wind -> rejected
    unit.step(0.1, True, 0)
    assert unit.airspeed() == settled  # the jump did not move the estimate
    gnss_speed.reading = (settled - 4.0, 'gnss', 0)  # within max_wind -> a plausible fix DOES blend
    unit.step(0.1, True, 0)
    assert unit.airspeed() != settled


def test_throttle_and_overrides():
    """
    The distance-constant throttle skips the float update between intervals; the pre-glide and dive
    overrides force full rate at once.

    Observable: _accum_s == 0 iff the update ran.
    """
    unit, _mix, _accel, _speed = _build({'airspeed_dive_pitch': -45.0})
    unit._interval_s = 1.0  # force a long throttle interval so only an override can fire the update
    # (a) glide: normal pitch + accum under the interval -> THROTTLED (update skipped)
    unit._estimator._speed = 14.0
    unit._accum_s = 0.0
    unit.step(0.01, False, fixed.from_float(-6.0))
    assert unit._accum_s > 0.0, 'settled glide should throttle (skip the update)'
    # (b) a steep nose-down while the (throttled) estimate is still low -> dive override fires first
    unit._accum_s = 0.5
    unit.step(0.01, False, fixed.from_float(-60.0))
    assert unit._accum_s == 0.0, 'a dive must force full rate before the estimate shows the overspeed'
    # (c) the caller's full_rate_override (the flight task's pre-glide) always runs full rate
    unit._estimator._speed = 0.0
    unit._accum_s = 0.0
    unit.step(0.01, True, 0)
    assert unit._accum_s == 0.0, 'the full-rate override (pre-glide) must never throttle'


def test_distance_constant_interval():
    """
    The update interval follows clamp(speed, floor, ceiling) Hz -- one update per ~1 m of travel.

    The table maps floor below 5 m/s, speed-proportional across 5..50, ceiling past 50; and after
    every update the interval re-derives from the FRESH estimate (staleness self-scales).
    """
    unit, _mix, _accel, gnss_speed = _build({'airspeed_floor_hz': 5, 'airspeed_ceiling_hz': 50})
    config = unit._config
    assert config.update_interval(0.0) == 0.2  # at rest: the 5 Hz floor bounds time-staleness
    assert config.update_interval(4.0) == 0.2  # below the floor band -> still 5 Hz
    assert config.update_interval(14.0) == 1.0 / 14  # glide trim: 14 Hz = 1 m per check
    assert config.update_interval(30.0) == 1.0 / 30  # dive: 30 Hz, still 1 m per check
    assert config.update_interval(50.0) == 0.02  # ceiling
    assert config.update_interval(75.0) == 0.02  # past the ceiling -> clamped (never faster)
    # one update per ~1 m of travel across the proportional band
    for speed in (5, 10, 20, 35, 50):
        assert abs(speed * config.update_interval(float(speed)) - 1.0) < 1e-9
    # the interval RE-DERIVES from the fresh estimate after an update (self-scaling staleness)
    gnss_speed.reading = (28.0, 'gnss', 0)  # a live fix (within max_wind of 14) pulls the estimate up
    unit._estimator._speed = 14.0
    unit._interval_s = config.update_interval(14.0)
    unit._accum_s = unit._interval_s  # due now
    unit.step(0.0, False, fixed.from_float(-6.0))
    assert unit._estimator.value() > 14.0  # the blend moved the estimate up...
    assert unit._interval_s == config.update_interval(unit._estimator.value())  # ...and the interval followed


def test_gnss_steep_pitch_gate():
    """
    Near-vertical flight gates the GNSS corrector OFF.

    The receiver's 2D ground speed reads ~0 in a vertical boost climb (and under-reads in a steep
    dive), and blending that would drag the estimate down -> a LOOSER fin cap at high q, the unsafe
    direction. Shallow attitude re-opens it.
    """
    unit, _mix, _accel, gnss_speed = _build()
    gnss_speed.reading = (0.0, 'gnss', 0)  # live fix, 2D ground speed ~0 (what a vertical climb reads)
    unit._estimator._speed = 14.0  # within max_wind of 0 (a 14 m/s headwind) -> only the STEEP gate can reject
    unit.step(0.1, True, fixed.from_float(85.0))  # near-vertical boost climb
    assert unit.airspeed() == 14.0, 'a bogus 2D ground speed must not drag the estimate in a climb'
    unit.step(0.1, True, fixed.from_float(-80.0))  # steep dive: same physics, same gate
    assert unit.airspeed() == 14.0, 'a steep dive must keep the integrator alone (over-read = safe)'
    unit.step(0.1, True, fixed.from_float(-6.0))  # shallow glide -> ground speed is representative again
    assert unit.airspeed() < 14.0, 'a shallow attitude must re-open the GNSS blend'
    # the gate threshold is config: a tighter gnss_steep_pitch gates earlier
    tight, _mix, _accel, tight_speed = _build({'gnss_steep_pitch': 10.0})
    tight_speed.reading = (0.0, 'gnss', 0)
    tight._estimator._speed = 14.0
    tight.step(0.1, True, fixed.from_float(-15.0))  # steeper than 10 deg -> gated even at a mild dive
    assert tight.airspeed() == 14.0


test_authority_cap()
test_unconfident_cap_floor()
test_estimator_wiring()
test_throttle_and_overrides()
test_distance_constant_interval()
test_gnss_steep_pitch_gate()
test_gnss_jump_rejected()
print('ok: governor -- 1/v2 authority cap, unconfident cap-floor, estimator wiring, distance-constant '
      'throttle, full-rate overrides, steep-pitch GNSS gate, GNSS jump rejection')
