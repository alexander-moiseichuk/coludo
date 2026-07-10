# On-board test for the stage-dependent guidance law (guidance.py): control-stage gating, the boost
# rod gate + vertical hold, the three GPS-degrading heading tiers with the nav cache, bank-to-turn
# vs the full-authority landing/final-approach bank, and the strip-centreline final approach. Pure
# logic with injected stubs -- no Flight task, no databoard. Run by `make test`.

import fixed
import guidance
from controller import Stage

_ZONE = ((48.001, 11.000), (48.000, 11.010))  # longitude-stretched -> gates on the left/right edges


class _PositionHandle:
    """read() stand-in for the databoard position Parameter."""

    def __init__(self):
        self.reading = (None, None, None)  # (position, source, age_ms)

    def read(self):
        return self.reading


class _AglHandle:
    """value() stand-in for the databoard agl Parameter."""

    def __init__(self, value=None):
        self.value_now = value

    def value(self):
        return self.value_now


class _StubGovernor:
    def __init__(self, speed=0.0):
        self.speed = speed

    def airspeed(self):
        return self.speed


class _StubMission:
    def __init__(self, zone=None, launch=None):
        self.zone = zone
        self._launch = launch

    def launch_point(self):
        return self._launch

    def geometry(self):
        if self.zone is None:
            return None
        import navigation
        target, _gate_a, _gate_b = navigation.zone(self.zone[0], self.zone[1])
        return {'target': target}


def _build(config=None, zone=_ZONE, launch=None, airspeed=0.0):
    """A guidance unit over fresh stubs; returns (guidance, position, agl, governor)."""
    position = _PositionHandle()
    agl = _AglHandle()
    gov = _StubGovernor(airspeed)
    unit = guidance.Guidance(guidance.GuidanceConfig(config or {}, 1000),
                             _StubMission(zone, launch), gov, position, agl)
    return unit, position, agl, gov


def test_heading_error():
    """Shortest signed wrap: 350 -> 10 is +20, not -340; the +/-180 boundary keeps its sign."""
    assert guidance.heading_error(10.0, 350.0) == 20
    assert guidance.heading_error(350.0, 10.0) == -20
    assert guidance.heading_error(100.0, 100.0) == 0
    assert guidance.heading_error(180.0, 0.0) == 180


def test_control_stage_gate():
    """setpoint() names the CONTROL stages from config (default: gliding only); everything else is
    None -> the caller holds neutral."""
    unit, _p, _a, _g = _build()
    assert unit.setpoint(Stage.GLIDING) == {}
    assert unit.setpoint(Stage.BOOSTING) is None and unit.setpoint(Stage.SETTING) is None
    staged, _p, _a, _g = _build({'stages': {'gliding': {'pitch': 0}, 'landing': {'pitch': -2}}})
    assert staged.setpoint(Stage.LANDING) == {'pitch': -2}
    assert staged.setpoint(Stage.DONE) is None


def test_boost_hold():
    """BOOSTING engages only PAST the rod (airspeed > boost_engage) and then holds the attitude
    captured at enter(); no yaw steering near vertical."""
    unit, _p, _a, gov = _build({'stages': {'boosting': {}, 'gliding': {}}, 'boost_engage_speed': 15.0})
    unit.enter(0.0, 0, fixed.from_float(90.0))  # rod-vertical capture (heading, roll, pitch)
    gov.speed = 5.0  # still on the rod
    assert unit.compute(Stage.BOOSTING, {}, 0.0, 0) is False  # caller neutrals
    gov.speed = 30.0  # past the rod
    assert unit.compute(Stage.BOOSTING, {}, 0.0, 0) is True
    assert unit.pitch_setpoint == fixed.from_float(90.0) and unit.roll_setpoint == 0
    assert unit.heading_error == 0  # heading ill-defined near vertical -> no nav/yaw steering


def test_heading_tiers():
    """The three GPS-degrading tiers of the glide target heading, and the freshness gate."""
    # tier 3: no fix, no launch point -> hold the heading captured at enter() (blind)
    unit, position, _a, _g = _build()
    unit.enter(200.0, 0, 0)
    assert unit.compute(Stage.GLIDING, {}, 0.0, 0) is True
    assert unit.heading_error == guidance.heading_error(200.0, 0.0)
    # tier 2: no fix, CC-set launch point (west of the zone) -> launch->left-gate bearing (~east, 90)
    unit, position, _a, _g = _build(launch=(48.0005, 10.990))
    unit.enter(200.0, 0, 0)
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    assert abs(unit.heading_error - 90) < 5
    # tier 1: a fresh fix overrides -> steer from the CURRENT position (east of the zone -> ~270)
    position.reading = ((48.0005, 11.020), 'gnss', 0)
    unit._nav_heading = None  # invalidate the cache (inputs changed faster than nav_period on purpose)
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    assert abs(unit.heading_error - (-90)) < 5  # ~270 -> wrapped -90 from a 0 heading
    # tier-1 freshness gate: an age past position_age_max_ms skips tier 1 even on a live fix
    unit._config.position_age_max_ms = 0
    unit._nav_heading = None
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    assert abs(unit.heading_error - 90) < 5  # gated off tier 1 -> tier 2 launch bearing
    # no zone at all -> blind hold regardless of the fix
    blind, position, _a, _g = _build(zone=None)
    position.reading = ((48.0005, 11.020), 'gnss', 0)
    blind.enter(200.0, 0, 0)
    blind.compute(Stage.GLIDING, {}, 0.0, 0)
    assert blind.heading_error == guidance.heading_error(200.0, 0.0)


def test_nav_cache():
    """The steer() trig is cached for nav_period_us: a second compute within the period returns the
    cached heading even if the fix moves; enter() invalidates."""
    unit, position, _a, _g = _build({'nav_period_ms': 100})
    unit.enter(200.0, 0, 0)
    position.reading = ((48.0005, 11.020), 'gnss', 0)  # east of the zone -> ~270
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    first = unit._nav_heading
    position.reading = ((48.0005, 10.980), 'gnss', 0)  # move the fix west; uncached this -> ~90
    unit.compute(Stage.GLIDING, {}, 0.0, 50000)  # 50 ms < nav_period -> cached
    assert unit._nav_heading == first
    unit.compute(Stage.GLIDING, {}, 0.0, 150000)  # past the period -> recomputed from the new fix
    assert unit._nav_heading != first
    unit.enter(200.0, 0, 0)  # entering control invalidates the cache outright
    assert unit._nav_heading is None


def test_bank_to_turn():
    """GLIDING banks into the turn (roll setpoint = nav_bank_gain * heading error, capped at
    bank_limit); LANDING and low FINAL use the land gains with the FULL fin authority."""
    unit, position, agl, _g = _build({'nav_bank_gain': 1.5, 'bank_limit': 30,
                                      'stages': {'gliding': {}, 'landing': {}}})
    unit.enter(0.0, 0, 0)
    position.reading = ((48.0005, 10.990), 'gnss', 0)  # west of the zone -> steer ~east (+90 error)
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    assert unit.roll_setpoint == fixed.from_float(30.0)  # bank_demand(+90, 1.5, 30) hit the cap
    # LANDING: the land bank limit (45) opens the full authority for the crosswind crab
    unit._nav_heading = None
    unit.compute(Stage.LANDING, {}, 0.0, 0)
    assert unit.roll_setpoint == fixed.from_float(45.0)  # bank_demand(+90, 1.5, 45)
    # gain 0 -> rudder-only steering: the roll setpoint stays the configured one
    flat, position, _a, _g = _build({'nav_bank_gain': 0, 'stages': {'gliding': {'roll': 2.0}}})
    flat.enter(0.0, 0, 0)
    position.reading = ((48.0005, 10.990), 'gnss', 0)
    flat.compute(Stage.GLIDING, flat.setpoint(Stage.GLIDING), 0.0, 0)
    assert flat.roll_setpoint == fixed.from_float(2.0)


def test_final_approach():
    """Below final_approach_agl the target switches from homing on the centre to TRACKING the strip
    centreline (navigation.approach) with the land bank gains -- even while still in GLIDING."""
    unit, position, agl, _g = _build({'final_approach_agl': 8, 'nav_bank_gain': 1.5,
                                      'land_bank_gain': 1.5, 'land_bank_limit': 45})
    unit.enter(0.0, 0, 0)
    position.reading = ((48.0005, 11.020), 'gnss', 0)  # east of the zone, past the right gate
    agl.value_now = 20.0  # high -> homing (steer)
    unit.compute(Stage.GLIDING, {}, 270.0, 0)
    homing = unit._nav_heading
    agl.value_now = 4.0  # low on final -> centreline tracking (approach)
    unit._nav_heading = None
    unit.compute(Stage.GLIDING, {}, 270.0, 0)
    tracking = unit._nav_heading
    assert homing != tracking  # approach() tracks the long-axis line, not the centre point
    assert abs(guidance.heading_error(tracking, 270.0)) <= 45  # intercept capped at final_intercept
    # final_approach_agl 0 disables the final-approach switch (and an absent agl never triggers it)
    off, position, agl, _g = _build({'final_approach_agl': 0})
    off.enter(0.0, 0, 0)
    agl.value_now = 1.0
    position.reading = ((48.0005, 11.020), 'gnss', 0)
    off.compute(Stage.GLIDING, {}, 270.0, 0)
    assert off._nav_heading is not None  # steered (homing path), no crash with the feature off


def test_loiter_and_endgame_spiral():
    """The loiter orbit + endgame spiral (the fly-long glide law): within loiter_capture_m the
    heading command is the circle TANGENT corrected inward/outward by the radius error; in the
    endgame band the radius scales with the remaining-altitude fraction (spiral-in); far outside
    the capture radius the gate steering stays in charge."""
    import navigation

    class _Elevation:
        def __init__(self):
            self.value_now = None

        def value(self):
            return self.value_now

    elevation = _Elevation()
    unit, position, _agl, _gov = _build({'loiter_radius_m': 30, 'loiter_capture_m': 120,
                                         'loiter_gain': 3.0, 'endgame_alt_m': 50,
                                         'nav_bank_gain': 1.5, 'land_bank_gain': 3.0})
    unit._elevation = elevation
    unit.enter(0.0, 0, 0)
    centre = ((48.001 + 48.000) / 2, (11.000 + 11.010) / 2)  # the _ZONE centre
    # ~60 m due south of the centre, HIGH (no endgame): tangent + inward cut (d=60 > R=30)
    south = (centre[0] - 60 / 111320.0, centre[1])
    position.reading = (south, 'gnss', 0)
    elevation.value_now = 200.0
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    bearing_centre = navigation.bearing(south[0], south[1], centre[0], centre[1])  # ~0 (north)
    expected = (bearing_centre + 90.0 - 60.0) % 360.0  # tangent +90, inward cut 3*(60-30) CAPPED at 60
    assert abs(guidance.heading_error(expected, unit._nav_heading)) <= 1
    # ON the circle (d == R): pure tangent, no correction
    on_circle = (centre[0] - 30 / 111320.0, centre[1])
    position.reading = (on_circle, 'gnss', 0)
    unit._nav_heading = None
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    tangent = (navigation.bearing(on_circle[0], on_circle[1], centre[0], centre[1]) + 90.0) % 360.0
    assert abs(guidance.heading_error(tangent, unit._nav_heading)) <= 1
    # ENDGAME at half the band: the commanded radius halves -> a deeper inward cut from d=30
    elevation.value_now = 25.0  # 25/50 -> radius 15
    unit._nav_heading = None
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    spiral = (navigation.bearing(on_circle[0], on_circle[1], centre[0], centre[1])
              + 90.0 - 3.0 * (30 - 15)) % 360.0
    assert abs(guidance.heading_error(spiral, unit._nav_heading)) <= 1
    # far OUTSIDE the capture radius: the gate steering stays in charge (not the tangent law)
    far = (centre[0] - 500 / 111320.0, centre[1])
    position.reading = (far, 'gnss', 0)
    elevation.value_now = 200.0
    unit._nav_heading = None
    unit.compute(Stage.GLIDING, {}, 0.0, 0)
    gate_heading = navigation.steer(far, (48.001, 11.000), (48.000, 11.010))[0]
    assert abs(guidance.heading_error(gate_heading, unit._nav_heading)) <= 1


def test_steering_filter():
    """The heading-error EMA: seeds on the first sample (raw through), smooths jitter with
    alpha 1/2^shift, follows a genuine >90-deg change at once, and shift 0 disables."""
    unit, _p, _a, _g = _build({'steer_filter_shift': 3})
    assert unit._filter_error(40) == 40  # first sample seeds -> raw
    smoothed = unit._filter_error(48)  # +8 jitter -> moves 1/8 of the way
    assert smoothed == 41
    for _ in range(60):  # converges onto a steady value
        smoothed = unit._filter_error(48)
    assert abs(smoothed - 48) <= 1
    assert unit._filter_error(-120) == -120  # a >90 deg flip (overfly) resets -> follow at once
    off, _p, _a, _g = _build({'steer_filter_shift': 0})
    assert off._filter_error(40) == 40 and off._filter_error(48) == 48  # disabled -> passthrough


def test_hold_law():
    """A configured control stage with no specific law (ground tests, e.g. 'setting') holds the
    configured setpoints and the heading captured at enter() -- no navigation."""
    unit, position, _a, _g = _build({'stages': {'setting': {'roll': 1.0, 'pitch': -2.0}}})
    position.reading = ((48.0005, 11.020), 'gnss', 0)  # a live fix must NOT steer this law
    unit.enter(100.0, 0, 0)
    assert unit.compute(Stage.SETTING, {'roll': 1.0, 'pitch': -2.0}, 90.0, 0) is True
    assert unit.roll_setpoint == fixed.from_float(1.0)
    assert unit.pitch_setpoint == fixed.from_float(-2.0)
    assert unit.heading_error == 10  # vs the captured 100, not the zone


def test_reachability():
    """reach = glide_ratio × elevation vs distance-to-zone -> reachable y/n + margin; None when a fix,
    zone, or elevation is missing (the flight panel's zone-reachable signal)."""
    position = _PositionHandle()
    elevation = _AglHandle(100.0)  # 100 m above the pad
    unit = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(_ZONE),
                             _StubGovernor(), position, _AglHandle(), elevation)
    assert unit.reachability(3.0) is None  # no fix yet -> None
    position.reading = ((48.0005, 11.005), 'gnss', 0)  # near the zone centre
    good = unit.reachability(3.0)  # 100 m × 3 = 300 m reach, near the zone -> reachable with margin
    assert good['reachable'] is True and good['margin_m'] > 0 and good['distance_m'] >= 0
    position.reading = ((48.020, 11.005), 'gnss', 0)  # ~1.7 km north of the zone -> target is SOUTH
    far = unit.reachability(3.0)  # 300 m reach << ~1700 m distance -> not reachable, negative margin
    assert far['reachable'] is False and far['margin_m'] < 0
    assert far['headwind_mps'] == 0.0  # no airspeed passed -> no wind adjustment
    # headwind adjustment: flying south to the zone, a NORTH-blowing wind opposes (headwind, shrinks the
    # reach); a SOUTH-blowing wind helps (tailwind, extends it) -- still air brackets the two
    head = unit.reachability(3.0, 0.0, 8.0, 14.0)   # wind toward the north = away from target -> headwind
    tail = unit.reachability(3.0, 0.0, -8.0, 14.0)  # wind toward the south = toward target -> tailwind
    assert tail['margin_m'] > far['margin_m'] > head['margin_m']
    assert head['headwind_mps'] > 0.0 > tail['headwind_mps']  # headwind positive, tailwind negative
    # missing elevation -> None (can't estimate reach)
    blind = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(_ZONE),
                              _StubGovernor(), position, _AglHandle(), _AglHandle(None))
    assert blind.reachability(3.0) is None
    # no zone -> None
    nozone = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(None),
                               _StubGovernor(), position, _AglHandle(), elevation)
    assert nozone.reachability(3.0) is None


def test_min_turn_radius():
    """R = v²/(g·tan bank): the physical turn-radius floor guidance clamps the endgame spiral to (5.1)."""
    slow = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(_ZONE),
                             _StubGovernor(14.0), _PositionHandle(), _AglHandle(), _AglHandle(100.0))
    assert abs(slow.min_turn_radius(45.0) - 20.0) < 0.5   # 14²/(9.81·tan45) ≈ 20 m -- the endgame floor
    assert abs(slow.min_turn_radius(30.0) - 34.6) < 0.5   # a gentler bank -> a wider turn
    assert abs(slow.landing_turn_radius() - 20.0) < 0.5   # defaults to the 45° land-bank limit
    fast = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(_ZONE),
                             _StubGovernor(20.0), _PositionHandle(), _AglHandle(), _AglHandle(100.0))
    assert fast.min_turn_radius(45.0) > slow.min_turn_radius(45.0)  # R ∝ v² -> faster flight widens it
    stalled = guidance.Guidance(guidance.GuidanceConfig({}, 1000), _StubMission(_ZONE),
                                _StubGovernor(0.0), _PositionHandle(), _AglHandle(), _AglHandle(100.0))
    assert stalled.min_turn_radius(45.0) == 0.0  # no airspeed -> no estimate (guarded)


test_heading_error()
test_control_stage_gate()
test_boost_hold()
test_heading_tiers()
test_nav_cache()
test_bank_to_turn()
test_final_approach()
test_loiter_and_endgame_spiral()
test_steering_filter()
test_reachability()
test_min_turn_radius()
test_hold_law()
print('ok: guidance -- stage gate, boost hold, GPS tiers + nav cache, bank-to-turn, loiter orbit + '
      'endgame spiral, final approach, reachability, min turn radius, hold law')
