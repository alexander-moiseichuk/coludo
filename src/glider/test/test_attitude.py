"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the attitude filter (tasks/attitude.py), the PRIMARY attitude: it MIRRORS a source
that outranks it (a stand-in for the HITL sim at priority 0) while that is winning, NEVER mirrors one
ranked below it (a stand-in for the SEN0253's BNO055 backup at priority 2), FREE-RUNS a complementary
filter (gyro integrate + accel gravity correction) otherwise, and goes BLIND when the gyro is gone --
WITHHOLDING its output only while a fresh backup below can take over, publishing as before where none
can. Uses the real databoard. Run by `make test`.
"""

import asyncio
import math

import config_default
import databoard
from tasks import attitude


class _Stub:
    config = config_default.default()


def _accel_for(roll_d, pitch_d):
    """Body-frame gravity (g) at a roll/pitch -- what the accelerometer reads at rest (yaw irrelevant)."""
    r, p = math.radians(roll_d), math.radians(pitch_d)
    return (-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))


async def amain():
    assert __import__('task').ACTIVITIES.get('attitude') is attitude.Attitude  # registered

    # a priority-0 stand-in for the HITL sim's attitude, plus the gyro `rate` + `accel` channels
    primary = databoard.Databoard.provide('primary', {'attitude': {'priority': 0, 'timeout_ms': 40}},
                                          'attitude')
    accel_ch, rate_ch = databoard.Databoard.provide(
        'imu', {'accel': {'priority': 0, 'timeout_ms': 100000}, 'rate': {'priority': 0, 'timeout_ms': 100000}},
        'accel', 'rate')

    comp = {'name': 'attitude', 'activity': 'attitude', 'period_ms': 20, 'accel_period_ms': 0,
            'corr_shift': 2, 'grav_low_g': 0.7, 'grav_high_g': 1.3, 'turn_gate_deg_s': 4,
            'course_gate_mps': 5.0, 'course_shift': 2,  # fast blend for the test (default 5 is weaker)
            'provides': {'attitude': {'priority': 1, 'timeout_ms': 40}}}
    unit = attitude.Attitude('attitude', comp, _Stub())
    assert await unit.setup() is True

    # MIRROR: while the priority-0 source is fresh, the filter copies it (roll/pitch already fixnum cd,
    # heading float deg -> cd) and does NOT free-run
    primary.push((123.0, 4500, -600))  # heading 123 deg, roll 45.00, pitch -6.00 (cd)
    value, source, _age = unit._attitude_param.read()
    assert source == 'primary'
    unit._mirror(value)
    assert unit._roll_cd == 4500 and unit._pitch_cd == -600 and unit._yaw_cd == 12300 and unit._seeded

    # FREE-RUN gyro integration: with no accel, roll/pitch/yaw integrate the rate (centideg/s) over dt
    accel_ch.push((0.0, 0.0, 0.0))  # present but zero -> outside the 1g band -> no accel correction
    rate_ch.push((1000, -500, 2000))  # gx=+10, gy=-5, gz=+20 deg/s
    unit._roll_cd = unit._pitch_cd = unit._yaw_cd = 0
    unit._integrate(1000)  # dt = 1000 ms = 1 s -> +1000, -500, +2000 cd
    assert unit._roll_cd == 1000 and unit._pitch_cd == -500 and unit._yaw_cd == 2000

    # ACCEL correction: at rest at roll 30 / pitch 0, the gravity vector pulls roll/pitch toward truth;
    # the complementary filter converges over repeated steps (no gyro motion here)
    ax, ay, az = _accel_for(30, 0)
    accel_ch.push((ax, ay, az))
    rate_ch.push((0, 0, 0))
    unit._roll_cd = unit._pitch_cd = 0
    for _ in range(40):
        unit._integrate(20)
    # tolerance 100 cd (1 deg): the accel->angle runs at the control's centi-fixnum scale (from_float),
    # ~0.5 deg typical / <2 deg worst over the glide envelope (coludo.md) -- fine for the backup
    assert abs(unit._roll_cd - 3000) <= 100 and abs(unit._pitch_cd) <= 100, (unit._roll_cd, unit._pitch_cd)

    # a HIGH-g reading (boost/manoeuvre) is REJECTED -- gravity vector untrustworthy, no correction
    accel_ch.push((0.0, 3.0, 3.0))  # |a| ~ 4.2 g, well outside the band
    before = unit._roll_cd
    unit._integrate(20)
    assert unit._roll_cd == before  # unchanged (rate is 0, accel rejected)

    # TURN GATE: with the accel showing 'level' (as a coordinated turn would) but a high yaw rate, the
    # accel correction is suppressed -- the gyro carries the bank instead of the estimate rolling flat
    accel_ch.push(_accel_for(0, 0))  # accel reads wings-level (the coordinated-turn illusion)
    rate_ch.push((0, 0, 3000))       # yaw rate 30 deg/s -> in a turn
    unit._roll_cd = 4000             # we are actually banked 40 deg
    unit._integrate(20)
    assert unit._roll_cd == 4000     # gyro roll rate 0 here + accel gated off -> bank held, not flattened
    rate_ch.push((0, 0, 100))        # yaw rate 1 deg/s -> straight-ish -> accel re-anchors toward level
    for _ in range(40):
        unit._integrate(20)
    assert abs(unit._roll_cd) <= 100  # gravity vector now pulls roll back to ~0 (centi-scale tolerance)

    # YAW from the GNSS course: moving, the gyro-integrated yaw pulls toward the ground track (an
    # absolute reference -- no magnetometer); below the speed gate the course is ignored (gyro-only)
    course_ch, speed_ch = databoard.Databoard.provide(
        'gps', {'course': {'priority': 0, 'timeout_ms': 100000}, 'speed': {'priority': 0, 'timeout_ms': 100000}},
        'course', 'speed')
    accel_ch.push(_accel_for(0, 0))  # level -> roll/pitch settle at 0, isolate the yaw behaviour
    rate_ch.push((0, 0, 0))
    course_ch.push(90.0)
    speed_ch.push(14.0)               # moving -> course is meaningful
    unit._yaw_cd = 0
    for _ in range(40):
        unit._integrate(20)
    assert abs(((unit._yaw_cd - 9000 + 18000) % 36000) - 18000) <= 100  # yaw pulled to the ~90 deg track
    # below the speed gate the course is ignored -> pure gyro (here 0) holds
    speed_ch.push(1.0)
    unit._yaw_cd = 4500
    unit._integrate(20)
    assert unit._yaw_cd == 4500  # no gyro motion + course gated off -> yaw unchanged
    # the wrap is handled: track just past north pulls a near-360 yaw forward, not backward
    speed_ch.push(14.0)
    course_ch.push(2.0)
    unit._yaw_cd = 35800  # 358 deg -> the +4 deg error must nudge it UP through 360, not down
    unit._integrate(20)
    assert unit._yaw_cd > 35800 or unit._yaw_cd < 200  # moved toward 360/0, not toward 358-... backward

    # publish format matches the BNO055 slot: (heading FLOAT deg, roll cd, pitch cd). Let the
    # priority-0 stand-in (timeout 40 ms) go stale so the filter's priority-1 push is the fused winner.
    await asyncio.sleep_ms(50)
    unit._yaw_cd, unit._roll_cd, unit._pitch_cd = 9000, 4500, -600
    unit._attitude.push((unit._yaw_cd / 100.0, unit._roll_cd, unit._pitch_cd))
    heading, roll_cd, pitch_cd = databoard.Databoard.parameter('attitude').value()
    assert isinstance(heading, float) and heading == 90.0 and roll_cd == 4500 and pitch_cd == -600

    """
    MIRROR ONLY A FRESH PRIMARY. With nothing fresh, read() returns the primary's EXTRAPOLATED old value
    with source None, and mirroring that pinned a dead part's last attitude after every >40 ms gap while
    throwing away the gyro integration. The stand-in pushes once and goes silent: the filter must
    free-run on the gyro (30 deg/s of yaw here), not hold the dead heading of 10 deg.
    """
    accel_ch.push((0.0, 0.0, 0.0))  # outside the 1 g band: no accel correction to muddy the yaw
    rate_ch.push((0, 0, 3000))
    primary.push((10.0, 500, 0))
    runner = asyncio.create_task(unit.run())
    await asyncio.sleep_ms(400)  # the primary is stale 40 ms in; the rest must be integrated
    runner.cancel()
    await asyncio.sleep_ms(0)
    # the old code mirrored the extrapolated value every cycle: _free stayed False and yaw sat at exactly
    # the dead part's 1000 cd. (Where yaw goes instead depends on the GNSS course pull set up above.)
    assert unit._free, 'a silent primary must hand over to the gyro'
    assert unit._yaw_cd != 1000, 'yaw pinned at the dead primary\'s last heading'

    """
    BLIND WITHOUT A BACKUP: exactly the behaviour before the backup existed. Nothing ranks below the
    filter yet, so with the gyro gone it goes on publishing its held estimate and stays the fused
    source -- withholding would hand flight.py no attitude at all. `blind` still goes up, on this board
    as on every other: the health flag is the only thing that says the filter lost its gyro.
    Driven tick by tick, so the count is exact. 5 = the module's _BLIND_CYCLES (a const: not readable
    from here on the board).
    """
    rate_handle = unit._rate
    accel_ch.push(_accel_for(0, 0))
    rate_ch.push((0, 0, 0))
    unit._tick()
    assert not unit.blind and unit._attitude_param.read()[1] == 'attitude'
    unit._rate = _Blind()
    for cycle in range(4):  # 1..4 gyro-less cycles are a GC pause or a stall, not a lost gyro
        unit._tick()
        assert not unit.blind, 'blind after only %d gyro-less cycles' % (cycle + 1)
    unit._tick()
    assert unit.blind, 'five gyro-less cycles in a row must raise blind'
    unit._roll_cd = 1234
    unit._tick()
    value, source, _age = unit._attitude_param.read()
    assert source == 'attitude' and value[1] == 1234, 'with no backup a blind filter must keep publishing'
    unit._rate = rate_handle
    unit._tick()
    assert not unit.blind, 'one fresh gyro sample ends blind'

    """
    ...and through a REAL databoard channel, not the stub: a stale parameter's read() still returns a
    value -- the last one, extrapolated -- with source None, so `blind` must be judged on the SOURCE. The
    stub returns None for both and could not tell a value check from a source check.
    """
    stale_ch = databoard.Databoard.provide('stale_gyro', {'rate_blind_test': {'priority': 0, 'timeout_ms': 20}},
                                           'rate_blind_test')
    stale_ch.push((0, 0, 0))
    unit._rate = databoard.Databoard.parameter('rate_blind_test')
    unit._tick()
    assert not unit.blind
    await asyncio.sleep_ms(40)  # past the 20 ms window: stale
    value, source, _age = unit._rate.read()
    assert value is not None and source is None, 'a stale channel must read (value, None): %r' % ((value, source),)
    for _ in range(5):
        unit._tick()
    assert unit.blind, 'a stale real channel (value present, source None) must make the filter blind'
    unit._rate = rate_handle
    unit._tick()
    assert not unit.blind

    # ...and BLIND WHILE MIRRORING: counted in either regime, so the flag is true before a handover
    unit._rate = _Blind()
    for _ in range(5):
        primary.push((10.0, 500, 0))
        unit._tick()
    assert unit.blind and not unit._free, 'blind must be tracked while mirroring too'
    unit._rate = rate_handle
    unit._tick()
    assert not unit.blind
    await asyncio.sleep_ms(50)  # the p0 stand-in goes stale again before the backup cases

    """
    NEVER MIRROR A BACKUP. The SEN0253's BNO055 sits BELOW the filter (p2). When it wins a cycle -- the
    filter was late, or blind -- copying it would overwrite the primary's estimate with the backup's,
    heading frame and all. With the gyro alive the filter keeps its own state and stays the fused source.
    A source at the filter's OWN rank does not outrank it either.
    """
    backup = databoard.Databoard.provide('backup', {'attitude': {'priority': 2, 'timeout_ms': 40}}, 'attitude')
    databoard.Databoard.provide('peer', {'attitude': {'priority': 1, 'timeout_ms': 40}}, 'attitude')  # never pushed
    assert unit._outranked('primary') and not unit._outranked('backup') and not unit._outranked('nobody')
    assert not unit._outranked('peer'), 'an equal rank is not an outranking source'
    feeder = asyncio.create_task(_feed(backup, (77.0, 2500, 1500)))  # the backup, kept FRESH throughout
    rate_ch.push((0, 0, 0))
    unit._roll_cd = unit._pitch_cd = 0
    runner = asyncio.create_task(unit.run())
    await asyncio.sleep_ms(200)
    assert unit._attitude_param.read()[1] == 'attitude', 'the filter must stay the source with its gyro'
    assert unit._roll_cd != 2500 and unit._pitch_cd != 1500, 'the filter copied the BNO055 backup'
    assert unit._free and not unit.blind

    """
    BLIND -> WITHHOLD -> the backup takes over, end to end through run(). A filter with no gyro used to
    publish its last attitude as FRESH forever, which masked a live backup below it. Lose the gyro:
    within the blind limit + one freshness window the fused source is the BNO055 backup. Give it back:
    the filter resumes and is the source again.
    """
    unit._rate = _Blind()
    await asyncio.sleep_ms(300)
    assert unit.blind, 'no gyro for 300 ms must make the filter blind'
    assert unit._attitude_param.read()[1] == 'backup', 'a blind filter must hand the attitude to the backup'
    unit._rate = rate_handle
    await asyncio.sleep_ms(100)
    assert not unit.blind and unit._attitude_param.read()[1] == 'attitude', 'the gyro is back: filter resumes'
    runner.cancel()
    feeder.cancel()
    await asyncio.sleep_ms(0)

    """
    The same WITH a backup, tick by tick. NO FALSE WITHHOLD: 1..4 gyro-less cycles leave the filter
    publishing. The fifth withholds, and once its last push ages out the backup is the source. And the
    backup must be FRESH to count as cover -- a backup that went quiet too gets the no-backup behaviour
    back (publish), rather than the filter falling silent with nothing under it.
    """
    for cycle in range(4):
        backup.push((77.0, 2500, 1500))
        unit._rate = _Blind()
        unit._tick()
        assert not unit.blind and unit._attitude_param.read()[1] == 'attitude', (
            'withheld after only %d gyro-less cycles' % (cycle + 1))
    backup.push((77.0, 2500, 1500))
    unit._tick()
    assert unit.blind
    await asyncio.sleep_ms(50)  # the filter's last push ages past the 40 ms window; the backup's is renewed
    backup.push((77.0, 2500, 1500))
    unit._tick()
    assert unit._attitude_param.read()[1] == 'backup', 'blind with a fresh backup must withhold'
    await asyncio.sleep_ms(50)  # now the backup is stale as well
    unit._tick()
    assert unit.blind and unit._attitude_param.read()[1] == 'attitude', (
        'blind with a STALE backup must publish -- going quiet would leave no attitude at all')
    unit._rate = rate_handle
    unit._tick()
    assert not unit.blind and unit._attitude_param.read()[1] == 'attitude'

    # RECORDED: this filter is the primary attitude, and it once created no stream at all
    assert unit._telemetry.filename == 'attitude.csv' and unit._telemetry.fields[:3] == (
        'heading_cd', 'roll_cd', 'pitch_cd'), unit._telemetry.fields

    # probe: healthy with a gyro rate present, fails without
    assert await unit.probe() is None
    unit._rate = _Blind()
    assert 'blind' in await unit.probe()

    print('ok: attitude filter -- mirror (fresh, outranking only), never mirrors a backup or a peer, blind '
          'after 5 cycles not 1..4, blind with no backup keeps publishing, blind while mirroring, blind '
          'with a fresh backup withholds (stale backup: publishes), resumes, gyro integrate, accel gravity '
          'correct, high-g reject, publish format, probe')


async def _feed(channel, value: tuple) -> None:
    """Keep a stand-in source FRESH: push `value` every 10 ms, well inside its 40 ms window."""
    while True:
        channel.push(value)
        await asyncio.sleep_ms(10)


class _Blind:
    """
    A channel with NOTHING behind it -- no value and no source.

    Carries read() as well as value() because the task was migrated to read(): a stub that implements
    only the old accessor makes the new code path untestable, and here it made probe() silently take
    the healthy branch. A stand-in has to expose the same interface as the thing it stands in for, or
    the test measures the stub instead of the code.
    """

    def value(self):
        return None

    def read(self):
        return None, None, 0    # (value, source, age) -- source None means "nothing fresh"


asyncio.run(amain())
