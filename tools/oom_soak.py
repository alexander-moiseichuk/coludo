# tools/oom_soak.py -- in-flight OOM soak (MicroPython, runs ON the board; wishes 7/06 #2).
# The GC-off control-path leak is ~15-18 KB/s at 100 Hz -> a natural OOM sits ~36 min out, far
# past any real flight. This soak BALLASTS the heap down to `target_kb` before ignition so the
# SAME leak reaches OOM mid-glide, then lets the production failure chain run for real:
#   MemoryError in the flight slice -> crash->neutral (flight.py's finally) -> the step counter
#   stops -> watchdog stall (stall_ms 500) -> hard reset -> main.py boots and the warm-start
#   five-signal gate decides (on the bench it must REFUSE -- the separation latch reads nested /
#   the baro reads pad level -- and clear the crumb: the negative gate under a REAL reset cause).
# The run therefore ENDS with the board resetting out from under mpremote -- that connection
# drop IS the observable. Afterwards check: machine.reset_cause(), the NVS crumb flag (cleared),
# and the recorder session's watchdog 'control loop stalled' line (Luckfox).
#
# Fly it like hitl_run (deploy first: cd src/glider && ./deploy.sh; then):
#   printf 'import oom_soak\noom_soak.soak("F15", 600)\n' > /tmp/launch.py
#   python3 tools/board_reboot.py PORT && mpremote connect PORT run /tmp/launch.py

import asyncio
import gc
import time

import config_hitl
import controller
import drivers
import mission
import recorder
import tasks

_BIG: int = 1024 * 1024  # coarse ballast chunk (PSRAM is tens of MB; coarse then fine)
_FINE: int = 64 * 1024


async def _ballast(target_kb: int) -> list:
    """Eat the heap down to ~target_kb free (refs held by the caller). Called at GLIDING entry:
    every long-lived structure (sim body, rings, tasks) is already placed and GC is already off,
    so the ballast carves the REMAINING free pool without starving the bring-up -- the first
    attempt ballasted before start() and the fragmented heap killed the sim before ignition.
    Yields per chunk: zeroing ~25 MB of PSRAM blocks for seconds, and the enabled watchdog
    (wdt_timeout 1000 ms) must keep feeding while we dig."""
    hold = []
    try:
        while gc.mem_free() > target_kb * 1024 + 2 * _BIG:
            hold.append(bytearray(_BIG))
            await asyncio.sleep_ms(0)
        while gc.mem_free() > target_kb * 1024:
            hold.append(bytearray(_FINE))
            await asyncio.sleep_ms(0)
    except MemoryError:  # overshoot on a fragmented tail -- close enough, keep what we hold
        pass
    return hold


async def _go(motor: str, target_kb: int) -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, 0.05, False, 0.0, 0.0, glider_g=285, inject_hz=25)
    by_name = {component['name']: component for component in cfg['components']}
    by_name['watchdog']['enabled'] = True  # the reset half of the chain under test (HITL default: off)
    flight = controller.Controller(cfg, log=lambda message: None)
    await flight.setup()
    await flight.start()
    flight.arm()
    print('SESSION', recorder.Recorder.session(), motor, 'target_kb', target_kb)
    stages = controller.Stage
    started = time.ticks_ms()
    last = -1
    tick = started
    ballast = None  # dropped at GLIDING entry (see _ballast)
    while True:
        stage = flight.stage
        if stage != last:
            print('STAGE', stage, stages.STAGES.get(stage))
            last = stage
            if stage == stages.GLIDING and ballast is None:
                ballast = await _ballast(target_kb)
                print('BALLAST', len(ballast), 'chunks held, free', gc.mem_free())
        now = time.ticks_ms()
        if time.ticks_diff(now, tick) >= 2000:  # the decay trace: seconds-since-start, free bytes
            tick = now
            print('MEM', time.ticks_diff(now, started) // 1000, gc.mem_free())
        if stage == stages.DONE:
            print('DONE without OOM -- lower target_kb')  # the soak wants the crash, not a landing
            break
        if time.ticks_diff(now, started) > 150000:
            print('TIMEOUT', stage)
            break
        await asyncio.sleep_ms(200)
    await asyncio.sleep_ms(1200)
    await flight.finish()
    print('RUN_END')  # reaching here means NO reset happened -- the soak failed to fire


def soak(motor: str = 'F15', target_kb: int = 600) -> None:
    asyncio.run(_go(motor, target_kb))
