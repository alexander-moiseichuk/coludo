"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Board bring-up, run on boot. Loads the driver/task packages (so every @task.activity / @task.driver
registers), creates the Mission (launch identity), and hands the config to the Controller, which
builds + supervises the *enabled* tasks. Connectivity (Wi-Fi + the CC link) is just two of those
tasks, so a board with no Wi-Fi (e.g. FireBeetle 2) boots and runs everything else without CC --
nothing here is hardcoded. Adding a task is dropping a file in drivers/ or tasks/ and enabling it in
the board config.

Telemetry-first: the task loops (recording included) start immediately and keep running; the Wi-Fi/CC
tasks connect in the background when they can. Time sync + live tweaks arrive from Control over the
link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.
"""

import asyncio

import config
import controller
import drivers
import mission
import recorder
import tasks
import warmstart


async def bringup(cfg: dict, log=print) -> controller.Controller:
    """
    Register every driver/task, create the Mission, and start the enabled tasks from the config.

    Network-free itself -- any Wi-Fi/CC work happens inside the tasks the Controller starts.

    Args:
        cfg - the validated board config the Controller builds its tasks from.
        log - line logger for bring-up progress (defaults to print).

    Returns:
        The Controller, with each enabled component's task created and its run loop launched.
    """
    drivers.load()  # HAL drivers (LED, sensors, ...) -> task.ACTIVITIES
    tasks.load()  # subsystem tasks (Recorder, BoardHealth, Wi-Fi, CC link, ...) -> task.ACTIVITIES
    # max_range_m lives on the field component (its site-select uses it too); Mission reads it from there
    field_cfg = config.device(cfg, name='field') or {}
    mission.Mission(max_range_m=field_cfg.get('max_range_m', 200))  # launch identity + clock + zone range gate
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled component's task; skip the ones without a driver / hardware
    await flight.start()  # launch the task run loops
    return flight


async def main() -> None:
    cfg, source, errors = config.load()
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))
    flight = await bringup(cfg)
    """
    PROVENANCE (findings §27.3): stamp the build + config identity into the CAPTURE, not just the
    console. A recording that cannot be attributed to the firmware and config that produced it is not
    comparable across a flight campaign -- which is the whole point of the passive-telemetry flights.
    Logged after bringup so the Recorder exists to carry it; log() is best-effort by policy, so a board
    with recording disabled just skips it.
    """
    board = cfg.get('board', {})
    recorder.Recorder.log('main', 'boot: board %s | firmware %s | config %s %s' % (
        board.get('id', '?'), board.get('firmware_version', '?'), config.config_id(cfg), source))
    await warmstart.restore(flight, cfg)  # warm start after a mid-air reset (no-op on a cold boot)
    while True:  # the supervised tasks do the work; keep the event loop alive
        await asyncio.sleep_ms(10000)


if __name__ == '__main__':
    asyncio.run(main())
