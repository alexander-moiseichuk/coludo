# main.py — board bring-up, run on boot. Loads the config, creates the operator-facing objects
# (Mission, Wifi, BoardHealth) and the Controller (which creates the configured tasks, including
# the Recorder virtual driver), starts the task loops, joins Wi-Fi, then dials Control and serves.
#
# Telemetry-first: the task loops (recording included) start BEFORE the network, so the board
# records even if Wi-Fi never comes up. Time sync and live tweaks arrive from Control over the
# running link (e.g. `update mission {epoch}` sets the RTC); the board itself never asks.

import asyncio

import board_health
import cc_client
import config
import controller
import mission
import wifi


async def bringup(cfg, log=print):
    """Create the inspectable objects + Controller and start the configured task loops. This is
    network-free, so it is testable on-board. Returns (controller, board_health) for the caller to
    wire to Control and to start the health telemetry."""
    mission.Mission()  # launch identity + clock; self-registers for `inspect mission`
    health = board_health.BoardHealth()  # vitals -> telemetry + `inspect health`
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled task (incl. the Recorder virtual driver); skip the driverless
    await flight.start()  # launch the task run loops (incl. the Recorder drain)
    return flight, health


async def main():
    cfg, source, errors = config.load()
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))

    radio = wifi.Wifi(cfg, log=print)
    flight, health = await bringup(cfg)
    asyncio.create_task(health.run())

    if await radio.connect():
        print('main :: wifi %s up, ip=%s' % (radio.ssid, radio.ip()))
    else:
        print('main :: wifi join failed (tasks keep running; Control link retries with backoff)')

    dispatcher = cc_client.create_dispatcher(cfg, controller=flight)
    await cc_client.Client(cfg, dispatcher, log=print).run()  # dial Control + serve forever


if __name__ == '__main__':
    asyncio.run(main())
