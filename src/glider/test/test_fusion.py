# On-board test for the sensor-fusion task (tasks/fusion.py): @task.activity('fusion') registration,
# priority selection, freshness fallback (preferred provider goes stale), and single-provider
# passthrough. Run by `make test`.

import asyncio

import task
from blackboard import Blackboard
from tasks import fusion


class _StubController:
    # a small self-contained config: altitude from icp (prio 0, 50 ms) + bmp (prio 1, 200 ms), a
    # single-provider accel, and an unfed position -- so the test is fast and timeout-independent.
    config = {
        'sensors': [
            {'name': 'baro_icp10111', 'enabled': True,
             'provides': {'altitude': {'priority': 0, 'timeout_ms': 50}}},
            {'name': 'baro_bmp280', 'enabled': True,
             'provides': {'altitude': {'priority': 1, 'timeout_ms': 200}}},
            {'name': 'accel_adxl375', 'enabled': True,
             'provides': {'accel': {'priority': 0, 'timeout_ms': 100}}},
            {'name': 'gnss', 'enabled': True,
             'provides': {'position': {'priority': 0, 'timeout_ms': 150}}},
        ],
        'components': [],
    }


async def amain():
    assert task.ACTIVITIES.get('fusion') is fusion.Fusion  # registered

    fuse = fusion.Fusion('fusion', {'period_ms': 5}, _StubController())
    assert await fuse.setup() is True and fuse.validate()

    # both providers fresh -> the priority-0 (ICP) wins
    Blackboard.write('altitude', 100.0, 'baro_icp10111')
    Blackboard.write('altitude', 200.0, 'baro_bmp280')
    fuse.fuse_once()
    assert Blackboard.value('altitude') == 100.0
    assert fuse._selected['altitude'] == 'baro_icp10111'

    # the preferred provider goes stale (older than its 50 ms timeout) -> fall back to the BMP280
    await asyncio.sleep_ms(70)
    Blackboard.write('altitude', 201.0, 'baro_bmp280')  # backup is fresh
    fuse.fuse_once()
    assert Blackboard.value('altitude') == 201.0
    assert fuse._selected['altitude'] == 'baro_bmp280'

    # single-provider quantity passes straight through
    Blackboard.write('accel', (1.0, 2.0, 3.0), 'accel_adxl375')
    fuse.fuse_once()
    assert Blackboard.value('accel') == (1.0, 2.0, 3.0)
    assert fuse._selected['accel'] == 'accel_adxl375'

    # a quantity with no fresh provider -> not selected
    assert fuse._selected.get('position') is None

    print('ok: fusion priority select, stale-fallback, single-provider passthrough')


asyncio.run(amain())
