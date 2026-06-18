# main.py — board bring-up, run on boot. Loads the driver/task packages (so every @task.driver
# registers), creates the Mission (launch identity), and hands the board config to the Controller,
# which builds + supervises the *enabled* tasks (Recorder, LED, BoardHealth, and later the sensors).
# Then it joins Wi-Fi, dials Control and serves. No driver is named here by hand — adding a task is
# dropping a file in drivers/ or tasks/ and enabling it in the board config.
#
# Telemetry-first: the task loops (recording included) start BEFORE the network, so the board
# records even if Wi-Fi never comes up. Time sync and live tweaks arrive from Control over the
# running link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.

import asyncio

import cc_client
import config
import controller
import drivers
import mission
import tasks
import wifi


async def bringup(cfg, log=print):
    """Register every driver/task, create the Mission, and have the Controller build + start the
    enabled tasks from the config. Network-free, so it is testable on-board. Returns the Controller."""
    drivers.load()  # HAL drivers (LED, sensors, ...) -> task.DRIVERS
    tasks.load()  # higher-level tasks (Recorder adapter, BoardHealth, ...) -> task.DRIVERS
    mission.Mission()  # launch identity + clock; not a task, self-registers for `inspect mission`
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled component's task; skip the ones without a driver
    await flight.start()  # launch the task run loops
    return flight


async def main():
    cfg, source, errors = config.load()
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))

    radio = wifi.Wifi(cfg, log=print)
    flight = await bringup(cfg)

    if await radio.connect():
        print('main :: wifi %s up, ip=%s' % (radio.ssid, radio.ip()))
    else:
        print('main :: wifi join failed (tasks keep running; Control link retries with backoff)')

    dispatcher = cc_client.create_dispatcher(cfg, controller=flight)
    await cc_client.Client(cfg, dispatcher, log=print).run()  # dial Control + serve forever


if __name__ == '__main__':
    asyncio.run(main())
