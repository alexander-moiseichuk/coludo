# On-board test for the attitude-redundancy backup (tasks/attitude.py): it MIRRORS the higher-priority
# fused source while that is winning, FREE-RUNS a complementary filter (gyro integrate + accel gravity
# correction) once the primary is lost, and its integer-CORDIC roll/pitch converge to the true attitude.
# Uses the real databoard with a stand-in priority-0 source. Run by `make test`.

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

    # a priority-0 stand-in for the BNO055/sim attitude, plus the gyro `rate` + `accel` channels
    primary = databoard.Databoard.provide('primary', {'attitude': {'priority': 0, 'timeout_ms': 40}},
                                          'attitude')
    accel_ch, rate_ch = databoard.Databoard.provide(
        'imu', {'accel': {'priority': 0, 'timeout_ms': 100000}, 'rate': {'priority': 0, 'timeout_ms': 100000}},
        'accel', 'rate')

    comp = {'name': 'attitude', 'activity': 'attitude', 'period_ms': 20, 'accel_period_ms': 0,
            'corr_shift': 2, 'grav_low_g': 0.7, 'grav_high_g': 1.3, 'turn_gate_deg_s': 4,
            'provides': {'attitude': {'priority': 1, 'timeout_ms': 40}}}
    unit = attitude.Attitude('attitude', comp, _Stub())
    assert await unit.setup() is True

    # MIRROR: while the priority-0 source is fresh, the backup copies it (roll/pitch already fixnum cd,
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
    assert abs(unit._roll_cd - 3000) <= 30 and abs(unit._pitch_cd) <= 30, (unit._roll_cd, unit._pitch_cd)

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
    assert abs(unit._roll_cd) <= 30  # gravity vector now pulls roll back to ~0

    # publish format matches the BNO055 slot: (heading FLOAT deg, roll cd, pitch cd). Let the
    # priority-0 primary (timeout 40 ms) go stale so the backup's priority-1 push is the fused winner.
    await asyncio.sleep_ms(50)
    unit._yaw_cd, unit._roll_cd, unit._pitch_cd = 9000, 4500, -600
    unit._attitude.push((unit._yaw_cd / 100.0, unit._roll_cd, unit._pitch_cd))
    heading, roll_cd, pitch_cd = databoard.Databoard.parameter('attitude').value()
    assert isinstance(heading, float) and heading == 90.0 and roll_cd == 4500 and pitch_cd == -600

    # probe: healthy with a gyro rate present, fails without
    assert await unit.probe() is None
    unit._rate = _Blind()
    assert 'blind' in await unit.probe()

    print('ok: attitude backup -- mirror, gyro integrate, accel gravity correct, high-g reject, '
          'publish format, probe')


class _Blind:
    def value(self):
        return None


asyncio.run(amain())
