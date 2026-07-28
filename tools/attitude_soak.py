"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Attitude-REDUNDANCY validation (MicroPython, runs ON the board). Flies a HITL glide, then mid-glide
flips the sim's `drop_attitude` to simulate a BNO055 death: the sim stops publishing `attitude` while
accel + rate keep flowing, so the priority-1 complementary-filter backup (tasks/attitude.py) must take
over the fused attitude slot and keep the glider controllable to DONE.

It reports: the backup-vs-truth attitude error just before the drop (was it mirroring / warm?), the
fused-source handover (imu/sim -> attitude backup), the backup-vs-truth error through the backup-flown
descent, and whether the flight still reaches DONE (control stayed stable on the backup attitude).

Deploy first (tools/deploy.sh), then:
  printf 'import attitude_soak\nattitude_soak.soak("F15", 6.0)\n' > /tmp/launch.py
  python3 tools/board_reboot.py PORT && mpremote connect PORT run /tmp/launch.py
"""

import asyncio
import time

import config_hitl
import controller
import databoard
import drivers
import mission
import tasks


def _err(backup, roll_true, pitch_true) -> str:
    """|backup - truth| for roll/pitch (backup roll/pitch are centidegrees, truth is float deg)."""
    if backup is None:
        return 'no attitude'
    _heading, roll_cd, pitch_cd = backup
    return 'roll %+.1f/%.1f pitch %+.1f/%.1f (err %.1f/%.1f)' % (
        roll_cd / 100.0, roll_true, pitch_cd / 100.0, pitch_true,
        abs(roll_cd / 100.0 - roll_true), abs(pitch_cd / 100.0 - pitch_true))


async def _go(motor: str, drop_after_s: float) -> None:
    drivers.load()
    tasks.load()
    launch = mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, 0.05, False, 0.0, 0.0, glider_g=285, inject_hz=25)
    flight = controller.Controller(cfg, log=lambda message: None)
    await flight.setup()
    await flight.start()
    flight.arm()
    hitl = flight.active('hitl')
    body = hitl._body
    attitude = databoard.Databoard.parameter('attitude')
    print('SESSION', motor, 'drop_after', drop_after_s, 's | zone', launch.zone is not None)
    stages = controller.Stage
    started = time.ticks_ms()
    last = -1
    drop_at_ms = None
    dropped = False
    sampled = 0
    while True:
        stage = flight.stage
        if stage != last:
            print('STAGE', stage, stages.STAGES.get(stage))
            if stage == stages.GLIDING and drop_at_ms is None:
                drop_at_ms = time.ticks_add(time.ticks_ms(), int(drop_after_s * 1000))
            last = stage
        now = time.ticks_ms()
        value, source, _age = attitude.read()
        if drop_at_ms is not None and not dropped and time.ticks_diff(now, drop_at_ms) >= 0 \
                and stage == stages.GLIDING:
            dropped = True
            print('PRE-DROP  source=%s | %s' % (source, _err(value, body.roll, body.pitch)))
            hitl.drop_attitude = True
            print('>>> BNO055 DROPPED (sim attitude off; accel+rate still live)')
        if dropped and stage == stages.GLIDING and sampled < 6 and time.ticks_diff(now, drop_at_ms) > 400:
            print('POST-DROP source=%s | %s' % (source, _err(value, body.roll, body.pitch)))
            sampled += 1
            await asyncio.sleep_ms(600)
            continue
        if stage == stages.DONE:
            print('DONE  final source=%s' % source)
            break
        if time.ticks_diff(now, started) > 150000:
            print('TIMEOUT', stage)
            break
        await asyncio.sleep_ms(200)
    await asyncio.sleep_ms(800)
    await flight.finish()
    print('RUN_END')


def soak(motor: str = 'F15', drop_after_s: float = 6.0) -> None:
    asyncio.run(_go(motor, drop_after_s))
