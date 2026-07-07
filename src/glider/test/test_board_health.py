# On-board test for the board vitals task (tasks/board_health.py): @task.activity('health')
# registration, vitals sampling, telemetry (first row at startup), int load, and that the load
# estimate tracks real CPU load. Run by `make test`.

import asyncio

import config_default
import recorder
import task
from tasks import board_health


class _FakeWriter:
    def __init__(self):
        self.items = []

    def write(self, data):
        self.items.append(bytes(data))

    async def drain(self):
        pass


async def test_basics():
    # registered as the 'health' activity the Controller builds from config
    assert task.ACTIVITIES.get('health') is board_health.BoardHealth

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth('health', {'period_ms': 20}, None)
    assert await health.setup() is True and health.validate()

    # sample()/inspect() report the vitals; load is an int percent 0..100; the flight-safety
    # forecasts (oom_s / land_s) and the rescue counter ride along
    vitals = health.sample()
    assert set(vitals.keys()) == {'temp', 'mem_free', 'load', 'oom_s', 'land_s', 'rescues'}
    assert isinstance(vitals['mem_free'], int) and vitals['mem_free'] > 0
    assert isinstance(vitals['load'], int) and 0 <= vitals['load'] <= 100
    assert vitals['rescues'] == 0 and vitals['oom_s'] is None and vitals['land_s'] is None
    assert set(health.inspect().keys()) == {'temp', 'mem_free', 'load', 'oom_s', 'land_s', 'rescues'}

    # the FIRST telemetry row lands at startup -- not one period late
    runner = asyncio.create_task(health.run())
    await asyncio.sleep_ms(5)  # << period (20 ms): only the startup row so far
    await recorder.Recorder.drain()
    rows = [bytes(i) for i in recorder.Recorder._uart.items]
    runner.cancel()
    try:
        await runner
    except asyncio.CancelledError:
        pass
    assert any(b'uptime;temp;mem_free;load' in r for r in rows)  # header emitted
    assert sum(1 for r in rows if b'_health.csv@' in r) >= 2  # header + the startup data row


async def test_load_tracking():
    # load tracks real CPU load: idle -> low (the probe sleep barely overshoots), a CPU hog -> higher
    # (the hog delays the probe's wake-up). No calibration baseline -- the overshoot is absolute.
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    health = board_health.BoardHealth('health', {'period_ms': 100, 'probe_ms': 10}, None)
    await health.setup()
    runner = asyncio.create_task(health.run())
    await asyncio.sleep_ms(500)  # a few idle probes -> load near 0
    idle_load = health.load

    async def hog():  # burn cycles between minimal yields so the idle task runs far less
        while True:
            total = 0
            for _ in range(30000):
                total += 1
            await asyncio.sleep_ms(0)

    hogger = asyncio.create_task(hog())
    await asyncio.sleep_ms(500)  # board now busy
    busy_load = health.load

    hogger.cancel()
    runner.cancel()
    for stopping in (hogger, runner):
        try:
            await stopping
        except asyncio.CancelledError:
            pass

    assert 0 <= idle_load <= 100 and 0 <= busy_load <= 100
    assert busy_load > idle_load, (idle_load, busy_load)  # load rises with real CPU load
    print('  load: idle=%d%% busy=%d%%' % (idle_load, busy_load))


async def test_memory_rescue():
    # the physics-based pre-OOM rescue: collect when the predicted time-to-OOM < 2x the time
    # left to sink to the rescue floor (land_s), with a PROVEN safe altitude
    # (elevation > rescue_agl_m), in BOOSTING..GLIDING only; no descent trend -> no rescue.
    import controller as controller_mod

    class _StubController:
        stage = controller_mod.Stage.GLIDING

        def active(self, name=None):
            return None  # no watchdog in the rig -> the rescue's WDT kick is skipped

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    rig = _StubController()
    health = board_health.BoardHealth('health', {'rescue_agl_m': 10}, rig)
    assert await health.setup() is True

    # build the two trends (the tracker clamps elapsed to whole seconds -> deltas ARE the
    # per-second slopes here): memory dying in ~1 s vs sinking to the floor in ~6 min
    health._track(1_000_000, 100.0)  # seeds both slopes
    await asyncio.sleep_ms(20)
    health._track(200_000, 99.0)  # 800 KB/s leak (EMA 200k), 1 m/s sink (EMA 25 cm/s)
    assert health.oom_s() is not None and health.land_s() is not None
    assert health.oom_s() < 2 * health.land_s()
    health._rescue(health.mem_free(), 99.0)  # dying before landing + safe altitude -> collect
    assert health.rescues == 1
    assert health._leak_bps == 0  # the trend resets with the garbage

    # memory outliving the flight -> no pause: a trickle leak, oom_s >> 2x land_s
    await asyncio.sleep_ms(20)
    health._track(199_500, 98.5)  # 500 B/s (EMA 125) -> oom_s ~27 min, land ~4.7 min
    assert health.oom_s() > 2 * health.land_s()
    health._rescue(health.mem_free(), 98.5)
    assert health.rescues == 1

    # not descending yet (boost/climb): NO rescue even under a catastrophic burn -- the glide
    # always descends, so the rescue waits for a land_s it can weigh the pause against
    health._descent_cm_s = 0
    health._last_free = 0
    health._track(1_000_000, None)
    await asyncio.sleep_ms(20)
    health._track(0, None)  # catastrophic burn -> oom_s ~0, but no descent trend
    assert health.land_s() is None and health.oom_s() is not None
    rig.stage = controller_mod.Stage.BOOSTING
    health._rescue(health.mem_free(), 50.0)
    assert health.rescues == 1

    # the gates: LANDING / at-or-below the floor / UNKNOWN elevation / rescue_agl_m 0 -> never
    health._descent_cm_s = 25  # descending again (land_s finite) -- isolate each gate
    health._last_elevation_cm = 10000
    rig.stage = controller_mod.Stage.LANDING
    health._rescue(health.mem_free(), 100.0)
    rig.stage = controller_mod.Stage.GLIDING
    health._rescue(health.mem_free(), 10.0)  # at the floor: not proven safe
    health._rescue(health.mem_free(), None)  # unknown elevation: not proven safe
    assert health.rescues == 1
    health._rescue(health.mem_free(), 100.0)  # control: the same trends DO rescue past the gates
    assert health.rescues == 2
    off = board_health.BoardHealth('health', {'rescue_agl_m': 0}, rig)
    assert await off.setup() is True
    off._rescue(0, 100.0)
    assert off.rescues == 0

    # oom_s trend mechanics: growth (a collect happened) invalidates the old slope
    health._last_free = 0
    health._leak_bps = 0.0
    health._track(1_000_000, None)
    await asyncio.sleep_ms(20)
    health._track(900_000, None)
    assert health.oom_s() is not None
    await asyncio.sleep_ms(20)
    health._track(950_000, None)
    assert health.oom_s() is None


async def amain():
    await test_basics()
    await test_load_tracking()
    await test_memory_rescue()
    print('ok: board_health registered, sample/inspect, first-row-at-startup, int load tracks CPU, '
          'physics rescue + oom_s/land_s')


asyncio.run(amain())
