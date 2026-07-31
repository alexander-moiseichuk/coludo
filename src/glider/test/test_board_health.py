"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the board vitals task (tasks/board_health.py): @task.activity('health')
registration, vitals sampling, telemetry (first row at startup), int load, and that the load
estimate tracks real CPU load. Run by `make test`.
"""

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
    assert set(vitals.keys()) == {'temp', 'mem_free', 'load', 'oom_s', 'land_s', 'rescues',
                                 'rescue_ms'}
    assert isinstance(vitals['mem_free'], int) and vitals['mem_free'] > 0
    assert isinstance(vitals['load'], int) and 0 <= vitals['load'] <= 100
    assert vitals['rescues'] == 0 and vitals['rescue_ms'] == 0  # no rescue yet -> no pause recorded
    assert vitals['oom_s'] is None and vitals['land_s'] is None
    assert set(health.inspect().keys()) == {'name', 'ok', 'healthy',  # the common Task.inspect base
                                            'temp', 'mem_free', 'load', 'oom_s', 'land_s', 'rescues',
                                            'rescue_ms'}  # the MEASURED pause, not an assumed one

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
    """
    the physics-based pre-OOM rescue: collect when the predicted time-to-OOM is at or under the time
    left to sink to the ground (land_s) -- i.e. memory dies BEFORE landing -- with a PROVEN safe
    altitude (elevation above the dynamic floor = 2x the descent a ~200 ms pause costs), in
    BOOSTING..GLIDING only; no descent -> no rescue.
    """
    import controller as controller_mod

    class _StubController:
        stage = controller_mod.Stage.GLIDING

        def active(self, name=None):
            return None  # no watchdog in the rig -> the rescue's WDT kick is skipped

    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    rig = _StubController()
    health = board_health.BoardHealth('health', {}, rig)
    assert await health.setup() is True

    # Build the two trends. The leak trend is CUMULATIVE and needs _LEAK_MIN_SAMPLES *intervals*
    # before it reports anything -- the first _track only seeds _last_kb, so that is N+1 readings.
    # An early guess is worse than none when the number arms a mid-glide control-loop pause.
    free, sink = 4_000_000, 800_000  # ~780 KB/s leak against a 1 m/s sink
    for step in range(board_health._LEAK_MIN_SAMPLES):
        health._track(free - step * sink, 100.0 - step)
        assert health.oom_s() is None, 'reported a trend after only %d intervals' % step
    health._track(free - board_health._LEAK_MIN_SAMPLES * sink,
                  100.0 - board_health._LEAK_MIN_SAMPLES)
    assert health.oom_s() is not None and health.land_s() is not None
    assert health.oom_s() <= health.land_s()  # memory dies before touchdown -> rescue
    health._rescue(health.mem_free(), 99.0)  # dying before landing + safe altitude -> collect
    assert health.rescues == 1
    # the trend is CUMULATIVE SINCE THE LAST COLLECT, so the next _track sees the collect's jump and
    # starts a new total -- the old one described garbage that no longer exists.
    health._track(health.mem_free(), 99.0)
    assert health._leak_kb == 0 and health._leak_ticks == 0

    # memory outliving the flight -> no pause: a trickle leak, oom_s >> 2x land_s
    health._leak_kb = health._leak_ticks = health._leak_kbps = 0  # a fresh trend
    health._last_kb = 0
    for step in range(board_health._LEAK_MIN_SAMPLES):
        health._track(200_000 - step * 500, 98.5 - step * 0.5)  # 500 B/s -> under 1 KB/s
    assert health.oom_s() is None or health.oom_s() > health.land_s()  # outlives the flight
    health._rescue(health.mem_free(), 98.5)
    assert health.rescues == 1

    # not descending yet (boost/climb): NO rescue even under a catastrophic burn -- the glide
    # always descends, so the rescue waits for a land_s it can weigh the pause against
    health._descent = 0
    health._last_free = 0
    health._leak_kb = health._leak_ticks = health._leak_kbps = 0
    health._last_kb = 0
    for step in range(board_health._LEAK_MIN_SAMPLES + 1):
        health._track(4_000_000 - step * 1_000_000, None)  # catastrophic burn, no descent trend
    assert health.land_s() is None and health.oom_s() is not None
    rig.stage = controller_mod.Stage.BOOSTING
    health._rescue(health.mem_free(), 50.0)
    assert health.rescues == 1

    # the gates: LANDING stage / UNKNOWN elevation / `rescue: false` -> never
    health._descent = 25  # slow sink -> a ~0.1 m floor, so altitude never gates here -- isolate stage
    health._last_elevation = 10000
    rig.stage = controller_mod.Stage.LANDING
    health._rescue(health.mem_free(), 100.0)  # LANDING -> never (no pause into the flare)
    rig.stage = controller_mod.Stage.GLIDING
    health._rescue(health.mem_free(), None)  # unknown elevation -> not proven safe
    assert health.rescues == 1
    health._rescue(health.mem_free(), 100.0)  # control: the same trends DO rescue past the gates
    assert health.rescues == 2
    # the DYNAMIC floor = 2x the descent a ~200 ms pause costs (NO base): a fast sink raises it, gating an
    # altitude a slow sink clears. Pin oom_s tiny + land_s large so only the floor varies.
    health._leak_bps = 1_000_000
    health._last_free = 1_000_000       # oom_s ~1 s (dying fast)
    health._last_elevation = 100_000  # 1000 m -> land_s huge, so oom < 2*land at any sink here
    health._descent = 2500  # 25 m/s -> floor = 2 * 0.2 s * 25 = 10 m
    health._rescue(health.mem_free(), 8.0)   # 8 m < 10 m -> gated by the pause margin
    assert health.rescues == 2
    health._descent = 25    # 0.25 m/s -> floor ~0.1 m -> 8 m clears it easily
    health._rescue(health.mem_free(), 8.0)
    assert health.rescues == 3   # the SAME altitude passes when sinking slowly
    off = board_health.BoardHealth('health', {'rescue': False}, rig)
    assert await off.setup() is True
    off._rescue(0, 100.0)
    assert off.rescues == 0

    # oom_s trend mechanics. A SMALL rise is sampler noise and must NOT blank the forecast -- that
    # was the defect: on a real flight one such sample zeroed the estimate and oom_s went None right
    # when the glider needed it. Only a COLLECT-sized jump starts a new trend.
    health._last_free = 0
    health._last_kb = 0
    health._leak_kb = health._leak_ticks = health._leak_kbps = 0
    history = 20  # a real flight accumulates tens of samples; a blip should move a 1/n average little
    for step in range(history + 1):
        health._track(4_000_000 - step * 100_000, None)  # ~97 KB/s, steady
    assert health.oom_s() is not None
    steady = health.oom_s()
    health._track(4_000_000 - history * 100_000 + 64 * 1024, None)  # a +64 KB sampler blip
    assert health.oom_s() is not None, 'a sampler blip blanked the forecast'
    assert abs(health.oom_s() - steady) <= max(4, steady // 4), 'a blip swung the forecast'
    health._track(30_000_000, None)  # a collect: a different heap -> the old total is meaningless
    assert health._leak_kb == 0 and health._leak_ticks == 0 and health.oom_s() is None


async def test_leak_forecast_is_steady():
    """
    The OOM forecast must not swing sample to sample -- it ARMS a mid-glide control-loop pause.

    Measured on a real board flight before this was fixed (doc/sims/TMS-7-phase5_refactor), successive
    oom_s readings were 271, 155, 119, 109, None, 362, None, 206: a single sample where the heap grew
    zeroed the whole estimate. Feed a steady leak with realistic sampler noise and a collect in the
    middle, and assert the forecast stays within a tolerance band and never goes blank after the
    collect.
    """
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())

    class _Rig:
        stage = 0

        def active(self, name=None):
            return None

    health = board_health.BoardHealth('health', {'rescue': False}, _Rig())
    assert await health.setup() is True

    free = 30_000_000        # a 30 MB PSRAM heap, as on the board
    leak = 330 * 1024        # the 331 KB/s measured in flight
    jitter = (0, 40_000, -25_000, 15_000, -35_000, 5_000)  # sampler noise, deterministic
    collect_at = 14
    quiet_until = collect_at + board_health._LEAK_MIN_SAMPLES  # the trend legitimately rebuilds here
    forecasts = []
    for step in range(34):
        if step == collect_at:
            free += 8_000_000  # a collect mid-run: the old estimator went blank right here
        free -= leak
        health._track(free + jitter[step % len(jitter)], 200.0 - step)
        if board_health._LEAK_MIN_SAMPLES + 2 <= step and not (collect_at <= step <= quiet_until):
            got = health.oom_s()
            assert got is not None, 'forecast went blank at step %d' % step
            forecasts.append((step, got))

    # Within a segment (between collects) a steady leak gives a smooth countdown, not a random walk.
    # ACROSS a collect it legitimately jumps up -- 8 MB was just reclaimed -- so only consecutive
    # samples are compared, and the collect breaks the chain.
    for (step_a, before), (step_b, after) in zip(forecasts, forecasts[1:]):
        if step_b != step_a + 1:
            continue  # the collect's rebuild gap: a step change here is the collect, not jitter
        assert abs(after - before) <= max(4, before // 4), \
            'forecast jumped %d -> %d s between steps %d and %d' % (before, after, step_a, step_b)
    values = [value for _step, value in forecasts]
    print('ok: leak forecast steady over %d samples (%d..%d s), survived a mid-run collect'
          % (len(values), min(values), max(values)))


async def amain():
    await test_basics()
    await test_load_tracking()
    await test_memory_rescue()
    await test_leak_forecast_is_steady()
    print('ok: board_health registered, sample/inspect, first-row-at-startup, int load tracks CPU, '
          'physics rescue + oom_s/land_s')


asyncio.run(amain())
