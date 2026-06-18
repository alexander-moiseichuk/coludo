# main.py — board bring-up, run on boot. Loads the driver/task packages (so every @task.activity /
# @task.driver registers), creates the Mission (launch identity), and hands the config to the Controller,
# which builds + supervises the *enabled* tasks. Connectivity (Wi-Fi + the CC link) is just two of
# those tasks, so a board with no Wi-Fi (e.g. FireBeetle 2) boots and runs everything else without
# CC -- nothing here is hardcoded. Adding a task is dropping a file in drivers/ or tasks/ and
# enabling it in the board config.
#
# Telemetry-first: the task loops (recording included) start immediately and keep running; the
# Wi-Fi/CC tasks connect in the background when they can. Time sync + live tweaks arrive from Control
# over the link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.

import asyncio

import config
import controller
import drivers
import mission
import tasks


async def bringup(cfg: dict, log=print) -> controller.Controller:
    """Register every driver/task, create the Mission, and have the Controller build + start the
    enabled tasks from the config. Returns the Controller. Network-free itself -- any Wi-Fi/CC work
    happens inside the tasks the Controller starts."""
    drivers.load()  # HAL drivers (LED, sensors, ...) -> task.ACTIVITIES
    tasks.load()  # subsystem tasks (Recorder, BoardHealth, Wi-Fi, CC link, ...) -> task.ACTIVITIES
    mission.Mission()  # launch identity + clock; not a task, self-registers for `inspect mission`
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled component's task; skip the ones without a driver / hardware
    await flight.start()  # launch the task run loops
    return flight


async def main() -> None:
    cfg, source, errors = config.load()
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))
    await bringup(cfg)
    while True:  # the supervised tasks do the work; keep the event loop alive
        await asyncio.sleep_ms(10000)


if __name__ == '__main__':
    asyncio.run(main())
