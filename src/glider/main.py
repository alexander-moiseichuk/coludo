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
    mission.Mission(max_range_m=cfg.get('max_range_m', 200))  # launch identity + clock + zone range gate
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled component's task; skip the ones without a driver / hardware
    await flight.start()  # launch the task run loops
    return flight


async def _restore_flight(flight: controller.Controller, cfg: dict, log=print) -> bool:
    """Warm start (specs/coludo.md "In-flight reboot & warm start"): a mid-air reset must not turn
    the glider ballistic. Restore GLIDING when the NVS breadcrumb AND two physical signals agree —
    the separation switch's latch (post-separation it stays LOW) and the baro ABSOLUTE altitude
    clearly above the breadcrumb's pad. Any doubt -> the breadcrumb is cleared and this is a normal
    cold boot. Cold-path imports stay inside the function (nothing here on a plain boot)."""
    import warmstart
    crumb = warmstart.load()
    if crumb is None:
        return False  # no flight was in progress: the normal boot, zero extra work
    if not cfg.get('warm_start', True):
        warmstart.clear()
        log('main :: warm start disabled by config')
        return False
    import time

    import databoard
    import inspector
    import machine
    gpio = cfg.get('pins', {}).get('separation_switch')
    separated = gpio is not None and machine.Pin(gpio, machine.Pin.IN, machine.Pin.PULL_DOWN).value() == 0
    altitude_parameter = databoard.Databoard.parameter('altitude')
    deadline = time.ticks_add(time.ticks_ms(), 5000)  # give the baro task time for a first reading
    altitude = altitude_parameter.value()
    while altitude is None and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        await asyncio.sleep_ms(100)
        altitude = altitude_parameter.value()
    cause = machine.reset_cause()
    cause_is_reset = cause in (machine.WDT_RESET, machine.SOFT_RESET, machine.HARD_RESET)
    restore, reason = warmstart.should_restore(crumb, separated, altitude, cause_is_reset,
                                               time.time() - crumb['stamp'])
    log('main :: warm-start gate: %s (reset_cause %d)' % (reason, cause))
    if not restore:
        warmstart.clear()  # rejected -> make the NEXT boot unambiguously cold
        return False
    launch = crumb['launch']
    zone = crumb['zone']
    mission_obj = inspector.Inspector.get('mission')
    if mission_obj is not None:
        mission_obj.update({'latitude': launch[0], 'longitude': launch[1],
                            'zone': [list(zone[0]), list(zone[1])]})
    # rebase the rebooted baros to the breadcrumb's pad altitude: their setup re-zeroed mid-air, so
    # without this `elevation` reads ~0 up there and the landing detect would flare immediately.
    baro_names = [sensor['name'] for sensor in cfg.get('sensors', [])
                  if sensor.get('driver') in ('icp10111', 'bmp280')]
    for baro in flight.find(baro_names):
        if baro is not None:
            baro.update({'ground': crumb['pad_altitude']})
    flight.set_stage(controller.Stage.GLIDING)
    flight.arm()  # we were armed when the breadcrumb dropped (the sequencer only saves armed flights)
    import gc
    gc.collect()  # the sequencer's BOOSTING GC hook was skipped: compact + GC OFF for the descent
    gc.disable()
    log('main :: WARM START -> gliding, armed (%.0fm above pad)' % (altitude - crumb['pad_altitude']))
    return True


async def main() -> None:
    cfg, source, errors = config.load()
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))
    flight = await bringup(cfg)
    await _restore_flight(flight, cfg)
    while True:  # the supervised tasks do the work; keep the event loop alive
        await asyncio.sleep_ms(10000)


if __name__ == '__main__':
    asyncio.run(main())
