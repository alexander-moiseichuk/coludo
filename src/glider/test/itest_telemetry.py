# -- telemetry-collection mode validation: bring up the FULL stack with the flight control loop
# ENABLED, with NO engines/servos/booster physically plugged, and prove it (a) boots every enabled task
# (hardware-absent ones skip gracefully, by design), (b) runs with no unhandled error, and (c) PRODUCES
# fin commands once gliding -- the control path a telemetry launch exercises.
#
# Integration test (itest_, not auto-run by run_tests.sh): enables flight + runs the real task loops for
# a few seconds. Bring-up is inlined (mirrors main.bringup) so main.py need not be on the board (no kiosk).
# Run on-board: mpremote connect PORT run test/itest_telemetry.py

import asyncio

import config_default
import databoard
import drivers
import mission
import tasks
from controller import Controller, Stage

_logs = []


def _log(message):
    _logs.append(str(message))


async def _bringup(cfg):
    """Mirror main.bringup: register drivers/tasks, create the Mission, build + start enabled tasks."""
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=cfg.get('max_range_m', 200))
    flight_controller = Controller(cfg, log=_log)
    await flight_controller.setup()
    await flight_controller.start()
    return flight_controller


async def amain():
    cfg = config_default.default()
    for component in cfg['components']:
        if component['name'] == 'flight':
            component['enabled'] = True  # telemetry mode: run the control loop, emit commands

    ctrl = await _bringup(cfg)
    built = sorted(ctrl.tasks.keys())
    print('telemetry: built tasks =', built)
    assert 'flight' in built, 'flight task not built with engines unplugged'
    assert any(name.startswith('servo_') for name in built), 'no fin servos built'

    # 1) run idle on the bench (stage SETTING -> loop gated, holds neutral): must not error
    await asyncio.sleep_ms(2500)
    steps_idle = ctrl.find(['flight'])[0].progress()[1]

    # 2) inject a GLIDING scenario (no real launch needed) -> the loop engages + emits fin commands
    attitude = databoard.Databoard.provide('imu', {'attitude': {'priority': 0, 'timeout_ms': 2000}}, 'attitude')
    position = databoard.Databoard.provide('gnss', {'position': {'priority': 0, 'timeout_ms': 2000}}, 'position')
    ctrl.arm()
    ctrl.set_stage(Stage.GLIDING)
    for _ in range(20):
        attitude.push((100.0, 8.0, -3.0))  # heading, roll, pitch -> a roll error to correct
        position.push((48.0005, 11.004))
        await asyncio.sleep_ms(100)
    flight = ctrl.find(['flight'])[0]
    steps_glide = flight.progress()[1]

    # read the commanded fin angles straight off the servo tasks (what telemetry/actuators would get)
    fins = {name: ctrl.find([name])[0] for name in ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')}
    angles = {name: getattr(unit, 'inspect', lambda: {})().get('angle') for name, unit in fins.items() if unit}
    print('telemetry: steps idle=%d -> gliding=%d (delta=%d), fin angles=%s'
          % (steps_idle, steps_glide, steps_glide - steps_idle, angles))

    await ctrl.finish()  # clean shutdown: cancel tasks, deinit the flight timer
    assert steps_glide > steps_idle, 'flight loop produced no commands in GLIDING (engines-off path broken)'
    print('ok: telemetry -- full stack + flight enabled boots & runs, commands flow, no engines plugged')


asyncio.run(amain())
