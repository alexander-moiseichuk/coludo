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
link (CC sends `update mission {epoch, ...}` on connect when `whoami` shows an unset clock); the board
itself never asks. Before anything records, the boot is counted in NVS: the count names its capture
files (doc/specs/recorder-wire.md).
"""

import asyncio

import config
import controller
import drivers
import esp32
import layout
import machine
import mission
import recorder
import tasks
import warmstart

try:
    from micropython import const
except ImportError:  # CPython (lint / host checks)
    def const(value: int) -> int:
        return value

_NVS_NOT_FOUND: int = const(-0x1102)  # ESP_ERR_NVS_NOT_FOUND, as esp32.NVS raises it: the key was never written


def _next_boot(store: esp32.NVS) -> int:
    """
    Count this boot: NVS `coludo`/`boot` + 1, committed before anything records.

    The count names the boot's capture files ('%06u', recorder.Recorder.session()), so it needs no
    clock -- the RTC reads 2000-01-01 until CC sets it. NVS survives deploys (they wipe the filesystem
    only); an erase_flash restarts it. Ids are unique, not consecutive: every reset counts.

    Only a key that was never written starts the count at 1. Any other read error says nothing about
    how far the count got, and restarting it would reuse ids -- appending this boot into an earlier
    boot's files, the failure the id exists to prevent.

    Args:
        store - the `coludo` NVS namespace (esp32.NVS); the tests pass a stand-in, never the real one.

    Returns:
        This boot's id, 1 on a board that never counted. None when NVS cannot be read (other than a
        missing key) or will not take the new count: the Recorder then falls back to its legacy date +
        random prefix, and the boot goes on.
    """
    try:
        boot = store.get_i32('boot')
    except OSError as error:
        if not error.args or error.args[0] != _NVS_NOT_FOUND:  # unreadable, not absent
            return None
        boot = 0  # never counted on this NVS
    boot += 1
    try:
        store.set_i32('boot', boot)
        store.commit()
    except OSError:
        return None
    return boot


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
    """
    Decide the layout BEFORE the Controller reads the config: the bus map (which bus four devices hang
    off) and the attitude parts (the SEN0697 expected, the SEN0253 backup only if found), and a driver
    binds its bus in setup(), so the config has to be right by then. Scanning first also means the raw
    scan buses are gone before i2cbus caches the real ones -- otherwise the 100 kHz scan clock would
    outlive detection on whichever bus it touched.
    """
    layout.resolve(cfg, log=log)
    # max_range_m lives on the field component (its site-select uses it too); Mission reads it from there
    field_cfg = config.device(cfg, name='field') or {}
    mission.Mission(max_range_m=field_cfg.get('max_range_m', 200))  # launch identity + clock + zone range gate
    flight = controller.Controller(cfg, log=log)
    await flight.setup()  # create each enabled component's task; skip the ones without a driver / hardware
    await flight.start()  # launch the task run loops
    return flight


async def main() -> None:
    cfg, source, errors = config.load()
    config.BOOT_SOURCE = source  # CC reads it: a fallback boot must be visible, not just printed
    print('main :: config %s%s' % (source, '' if not errors else ' ERRORS=%s' % errors))
    recorder.Recorder.boot_id = _next_boot(esp32.NVS('coludo'))  # BEFORE bringup: it names the boot's files
    flight = await bringup(cfg)
    """
    PROVENANCE: stamp the build + config identity into the CAPTURE, not just the
    console. A recording that cannot be attributed to the firmware and config that produced it is not
    comparable across a flight campaign -- which is the whole point of the passive-telemetry flights.
    Logged after bringup so the Recorder exists to carry it; log() is best-effort by policy, so a board
    with recording disabled just skips it.
    """
    board = cfg.get('board', {})
    recorder.Recorder.log('main', 'boot: board %s | firmware %s | config %s %s | reset_cause %d | boot %s | '
                          'session %s' % (board.get('id', '?'), board.get('firmware_version', '?'),
                                          config.config_id(cfg), source, machine.reset_cause(),
                                          recorder.Recorder.boot_id, recorder.Recorder.session()))

    def _boot_log(message: str) -> None:
        """The warm-start verdict goes to the CAPTURE too: a mid-air reboot must be provable afterwards."""
        print(message)
        recorder.Recorder.log('main', message)

    await warmstart.restore(flight, cfg, log=_boot_log)  # warm start after a mid-air reset (no-op on a cold boot)
    while True:  # the supervised tasks do the work; keep the event loop alive
        await asyncio.sleep_ms(10000)


if __name__ == '__main__':
    asyncio.run(main())
