# On-board test for warmstart.py (in-flight reboot recovery). Covers the pure gate should_restore() --
# all five defenses, their accept/refuse boundaries, and a TORN crumb (missing key) refusing cleanly
# instead of crashing the boot (findings §21.1) -- and _apply_restore(), which restores the mission,
# rebases the baros, and sets an armed GLIDING (stubbed controller/mission). Run by `make test`.

import controller
import inspector
import warmstart

_CRUMB = {'launch': [25.5, -80.4], 'zone': [[25.514, -80.393], [25.514, -80.391]],
          'pad_altitude': 100.0, 'stamp': 1000}


def test_should_restore():
    # happy path: separated, 50 m above the pad, WDT reset, crumb 30 s old
    ok, reason = warmstart.should_restore(_CRUMB, True, 150.0, True, 1030)
    assert ok, reason
    # 1. no breadcrumb -> cold boot
    assert not warmstart.should_restore(None, True, 150.0, True, 1030)[0]
    # 1. TORN crumb -- a missing pad_altitude or stamp REFUSES with no KeyError (findings §21.1)
    assert not warmstart.should_restore({'stamp': 1000}, True, 150.0, True, 1030)[0]
    assert not warmstart.should_restore({'pad_altitude': 100.0}, True, 150.0, True, 1030)[0]
    # 2. separation switch nested (not separated) -> refuse
    assert not warmstart.should_restore(_CRUMB, False, 150.0, True, 1030)[0]
    # 3. no altitude reading (baro not up in time) -> refuse
    assert not warmstart.should_restore(_CRUMB, True, None, True, 1030)[0]
    # 3. only 5 m above the pad (< 15) -> refuse; exactly 15 m -> pass (boundary)
    assert not warmstart.should_restore(_CRUMB, True, 105.0, True, 1030)[0]
    assert warmstart.should_restore(_CRUMB, True, 115.0, True, 1030)[0]
    # 4. power-on boot (recovery crew's hands / a brownout), not a reset -> refuse
    assert not warmstart.should_restore(_CRUMB, True, 150.0, False, 1030)[0]
    # 5. age negative (clock rewound / power-cycled RTC) -> refuse
    assert not warmstart.should_restore(_CRUMB, True, 150.0, True, 990)[0]
    # 5. age over the 600 s max -> refuse; exactly 600 s -> pass (boundary)
    assert not warmstart.should_restore(_CRUMB, True, 150.0, True, 1000 + 601)[0]
    assert warmstart.should_restore(_CRUMB, True, 150.0, True, 1000 + 600)[0]


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


class _StubFlight:
    def __init__(self, baros):
        self._baros = baros
        self.stage = None
        self.armed = False
        self.warm_started = False

    def find(self, names):
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


def test_apply_restore_torn_crumb():
    # a crumb with pad_altitude+stamp (gate passed) but launch/zone dropped -> mission NOT updated and
    # no crash; the baro rebase + armed GLIDING still happen
    mission = _StubMission()
    inspector.Inspector.register(mission)
    try:
        baro = _StubBaro()
        flight = _StubFlight([baro])
        warmstart._apply_restore(flight, {'pad_altitude': 100.0, 'stamp': 1000},
                                 {'sensors': [{'name': 'b', 'driver': 'bmp280'}]})
        assert mission.updated is None       # no launch/zone -> mission left as-is, no crash
        assert baro.ground == 100.0          # rebase still applied
        assert flight.stage == controller.Stage.GLIDING and flight.warm_started
    finally:
        inspector.Inspector.unregister('mission')


test_should_restore()
test_apply_restore()
test_apply_restore_torn_crumb()
print('ok: warmstart -- gate (5 defenses + boundaries + torn-crumb refuse), apply restores '
      'mission/baros/armed-GLIDING')
