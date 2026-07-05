# On-board test for the CC-less field agent (tasks/field.py): site-by-GPS on the first fix (known
# site vs the synthesized fallback zone), and the auto-arm dwell — stationary with a live fix for
# the whole window, restarted by motion or a lost fix, gated by a missing zone. Tick-driven with a
# controlled `now`. Run by `make test`.

import asyncio

import config_default
import databoard
import task
from controller import Stage
from tasks import field


class _StubController:
    def __init__(self):
        self.config = config_default.default()
        self.stage = Stage.SETTING
        self.armed = False

    def arm(self):
        self.armed = True


class _StubMission:
    max_range_m = 200

    def __init__(self, site_hit: bool):
        self._site_hit = site_hit
        self.zone = None
        self.selected = None
        self.fallback = None

    def select_site(self, fix):
        if not self._site_hit:
            return None
        self.selected = fix
        self.zone = ((48.001, 11.0), (48.0, 11.001))
        return 'hit'

    def fallback_zone(self, fix, bearing_deg, offset_m):
        self.fallback = (fix, bearing_deg, offset_m)
        self.zone = ((48.001, 11.0), (48.0, 11.001))
        return self.zone


async def amain():
    assert task.ACTIVITIES.get('field') is field.Field  # registered activity
    position = databoard.Databoard.provide('gnss_f', {'position': {'priority': 0, 'timeout_ms': 1000}},
                                           'position')
    accel = databoard.Databoard.provide('imu_f', {'accel': {'priority': 0, 'timeout_ms': 1000}}, 'accel')

    # site-by-GPS: the first fresh fix selects the known site, exactly once
    ctrl = _StubController()
    agent = field.Field('field', {'auto_arm': False}, ctrl)
    assert await agent.setup() is True
    agent._mission = _StubMission(site_hit=True)
    agent._site_done = False
    agent._tick(0)  # no fix yet -> nothing decided
    assert agent._mission.selected is None
    position.push((47.9995, 11.0))
    agent._tick(1000)
    assert agent._mission.selected == (47.9995, 11.0)  # the live fix reached select_site
    agent._mission.selected = None
    agent._tick(2000)  # a second fix must NOT re-select (one decision per boot)
    assert agent._mission.selected is None

    # no site in range -> the fallback zone is synthesized from the fix + configured sector
    miss_ctrl = _StubController()
    miss = field.Field('field', {'fallback_bearing_deg': 90.0, 'fallback_offset_m': 60.0}, miss_ctrl)
    assert await miss.setup() is True
    miss._mission = _StubMission(site_hit=False)
    miss._site_done = False
    position.push((48.0, 11.0))
    miss._tick(0)
    assert miss._mission.fallback == ((48.0, 11.0), 90.0, 60.0)

    # auto-arm: stationary + live fix for the WHOLE dwell; motion or a lost fix restarts it; a
    # missing zone blocks it outright (a blind flight is a human decision)
    arm_ctrl = _StubController()
    armer = field.Field('field', {'auto_arm': True, 'auto_arm_dwell_s': 1, 'site_select': False},
                        arm_ctrl)
    assert await armer.setup() is True
    armer._mission = _StubMission(site_hit=True)
    armer._mission.zone = None
    position.push((48.0, 11.0))
    accel.push((0.0, 0.0, 1.0))  # stationary 1 g
    armer._tick(0)
    armer._tick(1500)
    assert arm_ctrl.armed is False  # zone unset -> never auto-arm
    armer._mission.zone = ((48.001, 11.0), (48.0, 11.001))
    armer._tick(2000)  # the dwell starts here
    assert arm_ctrl.armed is False
    accel.push((0.0, 0.0, 3.0))  # picked up / carried -> the dwell restarts
    armer._tick(2500)
    accel.push((0.0, 0.0, 1.0))
    armer._tick(3000)  # stationary again -> a fresh dwell from 3000
    armer._tick(3900)
    assert arm_ctrl.armed is False  # 900 ms < 1 s dwell
    armer._tick(4100)
    assert arm_ctrl.armed is True  # 1.1 s stationary with a fix -> armed
    armer._tick(5000)  # armed -> the agent idles (no crash, no re-arm churn)

    # past SETTING nothing is decided (the pad agent stands down at ignition)
    late_ctrl = _StubController()
    late_ctrl.stage = Stage.BOOSTING
    late = field.Field('field', {'auto_arm': True}, late_ctrl)
    assert await late.setup() is True
    late._mission = _StubMission(site_hit=True)
    late._site_done = False
    late._tick(0)
    assert late._mission.selected is None and late_ctrl.armed is False

    print('ok: field -- site-by-GPS once, fallback zone, auto-arm dwell/restart/zone-gate, '
          'stands down past SETTING')


asyncio.run(amain())
