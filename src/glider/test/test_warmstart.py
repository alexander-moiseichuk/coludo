"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for warmstart.py (in-flight reboot recovery). Covers the pure gate should_restore() --
its reset / age / per-stage separation defenses and their accept/refuse boundaries, plus a TORN crumb
(missing key) refusing cleanly instead of crashing the boot -- and _apply_restore(), which restores
the mission, rebases the baros, and sets the saved (armed GLIDING) stage (stubbed controller/mission).
Run by `make test`.
"""

import controller
import inspector
import warmstart

_CRUMB = {'launch': [25.5, -80.4], 'zone': [[25.514, -80.393], [25.514, -80.391]],
          'pad_altitude': 100.0, 'stamp': 1000, 'stage': controller.Stage.GLIDING, 'armed': True}


def test_should_restore():
    # happy path: a GLIDING crumb, separated, WDT reset, 30 s old
    ok, reason = warmstart.should_restore(_CRUMB, True, True, 1030)
    assert ok, reason
    # no crumb -> cold boot
    assert not warmstart.should_restore(None, True, True, 1030)[0]
    # TORN crumb -- a missing stage or stamp REFUSES with no KeyError
    assert not warmstart.should_restore({'stamp': 1000}, True, True, 1030)[0]
    assert not warmstart.should_restore({'stage': controller.Stage.GLIDING}, True, True, 1030)[0]
    # a GLIDING (passive) crumb with the switch nested (not separated) -> refuse
    assert not warmstart.should_restore(_CRUMB, False, True, 1030)[0]
    # power-on boot (recovery crew's hands / a brownout), not a reset -> refuse
    assert not warmstart.should_restore(_CRUMB, True, False, 1030)[0]
    # age negative (clock rewound / power-cycled RTC) -> refuse
    assert not warmstart.should_restore(_CRUMB, True, True, 990)[0]
    # age over the 600 s max -> refuse; exactly 600 s -> pass (boundary)
    assert not warmstart.should_restore(_CRUMB, True, True, 1000 + 601)[0]
    assert warmstart.should_restore(_CRUMB, True, True, 1000 + 600)[0]
    # BOOSTING is the ACTIVE boost (pre-separation) -> recovers with the switch NESTED, on reset + age
    boost = {'stage': controller.Stage.BOOSTING, 'stamp': 1000}
    assert warmstart.should_restore(boost, False, True, 1030)[0]
    # LANDING is a PASSIVE glide stage -> the separation latch is required (a landed-nested glider cannot)
    landing = {'stage': controller.Stage.LANDING, 'stamp': 1000}
    assert warmstart.should_restore(landing, True, True, 1030)[0]        # separated -> recover LANDING
    assert not warmstart.should_restore(landing, False, True, 1030)[0]   # nested -> refuse
    # SETTING/DONE are on the ground -- no separation gate, just reset + age
    setting = {'stage': controller.Stage.SETTING, 'stamp': 1000}
    assert warmstart.should_restore(setting, False, True, 1030)[0]       # armed on the pad, WDT reset
    done = {'stage': controller.Stage.DONE, 'stamp': 1000}
    assert warmstart.should_restore(done, False, True, 1030)[0]          # landed, WDT reset -> stay DONE
    assert not warmstart.should_restore(done, False, False, 1030)[0]     # ... but a power-on stays cold


class _StubBaro:
    def __init__(self):
        self.ground = None

    def update(self, patch):
        self.ground = patch.get('ground')


class _StubMission:
    name = 'mission'

    def __init__(self):
        self.updated = None

    def update(self, patch):
        self.updated = patch


class _StubPitot:
    """The SDP810 as warmstart sees it: a tare to freeze on the pad and restore after a reboot."""

    def __init__(self, zero=0.0):
        self.zero = zero

    def inspect(self):
        return {'zero_offset_pa': self.zero}

    def update(self, props):
        self.zero = props['zero_offset_pa']
        return ['zero_offset_pa']


class _StubFlightTask:
    """The flight task as _apply_restore sees it: somewhere to hand the crumb's airspeed back."""

    def __init__(self):
        self.seeded = None

    def seed_airspeed(self, airspeed):
        self.seeded = airspeed


class _StubFlight:
    def __init__(self, baros, flight_task=None, pitot=None):
        self._baros = baros
        self._flight_task = flight_task
        self.pitot = pitot
        self.stage = None
        self.armed = False
        self.warm_started = False

    def find(self, names):
        # name-aware: _apply_restore asks for 'flight' (the airspeed seed), the pitot (its tare) and
        # the baro names
        if list(names) == ['flight']:
            return [self._flight_task]
        if list(names) == ['airspeed_sdp810']:
            return [self.pitot]
        return self._baros

    def set_stage(self, stage):
        self.stage = stage

    def arm(self):
        self.armed = True


def test_apply_restore():
    mission = _StubMission()
    inspector.Inspector.register(mission)
    try:
        baro = _StubBaro()
        flight = _StubFlight([baro])
        cfg = {'sensors': [{'name': 'baro_icp10111', 'driver': 'icp10111'}]}
        warmstart._apply_restore(flight, _CRUMB, cfg)
        # mission identity + zone restored from the crumb
        assert mission.updated['latitude'] == 25.5 and mission.updated['longitude'] == -80.4
        assert mission.updated['zone'] == [[25.514, -80.393], [25.514, -80.391]]
        # baros rebased to the crumb's pad altitude (else elevation reads ~0 mid-air -> instant flare)
        assert baro.ground == 100.0
        # restored to an armed GLIDING + the degraded-mode flag
        assert flight.stage == controller.Stage.GLIDING
        assert flight.armed and flight.warm_started
    finally:
        inspector.Inspector.unregister('mission')


def test_apply_restore_seeds_airspeed():
    """
    findings §23.4: the crumb's fused airspeed is handed back BEFORE the loop runs, so the fin cap comes
    off a real speed instead of the blunt unconfident floor (starved authority right after a mid-air
    reset). An older crumb without the field must simply not seed -- never crash the recovery boot.
    """
    mission = _StubMission()
    inspector.Inspector.register(mission)
    try:
        cfg = {'sensors': [{'name': 'baro_icp10111', 'driver': 'icp10111'}]}
        task = _StubFlightTask()
        crumb = dict(_CRUMB, airspeed=16.5)
        warmstart._apply_restore(_StubFlight([_StubBaro()], task), crumb, cfg)
        assert task.seeded == 16.5

        # NEGATIVE: a crumb predating the field -> no seed, recovery still completes
        old = _StubFlightTask()
        flight = _StubFlight([_StubBaro()], old)
        warmstart._apply_restore(flight, _CRUMB, cfg)
        assert old.seeded is None and flight.stage == controller.Stage.GLIDING

        # NEGATIVE: no flight task registered at all (flight disabled) -> no crash
        warmstart._apply_restore(_StubFlight([_StubBaro()], None), crumb, cfg)
    finally:
        inspector.Inspector.unregister('mission')


def test_apply_restore_pitot_tare():
    """
    The PITOT TARE must survive a mid-air reboot. It lives in RAM, so without this a warm-started
    board flies on dynamic pressure that still carries the interior-static bias -- and that is the
    airspeed the fin governor caps off. Same class as the baro rebase, which exists for this reason.
    """
    mission = _StubMission()
    inspector.Inspector.register(mission)
    try:
        cfg = {'sensors': [{'name': 'baro_icp10111', 'driver': 'icp10111'}]}
        pitot = _StubPitot(zero=0.0)
        flight = _StubFlight([_StubBaro()], _StubFlightTask(), pitot)
        warmstart._apply_restore(flight, dict(_CRUMB, pitot_zero=-1.75), cfg)
        assert pitot.zero == -1.75, pitot.zero

        # NEGATIVE: a crumb without the field leaves the driver's own tare alone, never zeroes it
        untouched = _StubPitot(zero=-0.5)
        warmstart._apply_restore(_StubFlight([_StubBaro()], _StubFlightTask(), untouched), _CRUMB, cfg)
        assert untouched.zero == -0.5
    finally:
        inspector.Inspector.unregister('mission')


def test_apply_restore_torn_crumb():
    # a crumb with pad_altitude+stamp (gate passed) but launch/zone dropped -> mission NOT updated and
    # no crash; the baro rebase + armed GLIDING still happen
    mission = _StubMission()
    inspector.Inspector.register(mission)
    try:
        baro = _StubBaro()
        flight = _StubFlight([baro])
        warmstart._apply_restore(flight, {'pad_altitude': 100.0, 'stamp': 1000,
                                          'stage': controller.Stage.GLIDING},
                                 {'sensors': [{'name': 'b', 'driver': 'bmp280'}]})
        assert mission.updated is None       # no launch/zone -> mission left as-is, no crash
        assert baro.ground == 100.0          # rebase still applied
        assert flight.stage == controller.Stage.GLIDING and flight.warm_started
    finally:
        inspector.Inspector.unregister('mission')


test_should_restore()
test_apply_restore()
test_apply_restore_seeds_airspeed()
test_apply_restore_pitot_tare()
test_apply_restore_torn_crumb()
print('ok: warmstart -- gate (5 defenses + boundaries + torn-crumb refuse), apply restores '
      'mission/baros/armed-GLIDING')
