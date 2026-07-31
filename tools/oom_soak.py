"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

In-flight OOM soak (MicroPython, runs ON the board). MEASURED leak, by varying the sim publish rate
over three board flights and fitting: leak = 187 + 3.32 * inject_hz KB/s -- so ~187 KB/s of PRODUCTION
leak (telemetry + recorder + control path) and ~160 s to exhaustion on a 30 MB heap, with the HITL sim
itself accounting for the rest. The older "~15-18 KB/s" figure quoted here was the control path ALONE.
A natural OOM therefore sits past any real flight, so this soak BALLASTS the heap down to
`target_kb` before ignition so the SAME leak reaches OOM mid-glide, then lets the production failure
chain run for real:
    MemoryError in the flight slice -> crash->neutral (flight.py's finally) -> the step counter stops ->
    watchdog stall (stall_ms 500) -> hard reset -> main.py boots and the warm-start five-signal gate
    decides (on the bench it must REFUSE -- the separation latch reads nested / the baro reads pad level
    -- and clear the crumb: the negative gate under a REAL reset cause).
The run therefore ENDS with the board resetting out from under mpremote -- that connection drop IS the
observable. Afterwards check the NVS crumb flag (cleared) and the recorder session's watchdog
'control loop stalled' line (Luckfox).

reset_cause() CANNOT be checked by reconnecting: mpremote soft-resets on connect, so the value read
back is always SOFT_RESET and the evidence is gone. Use a connection that does not reset --
`mpremote --no-soft-reset connect PORT exec "import machine; print(machine.reset_cause())"`.

Fly it like hitl_run (deploy first: tools/deploy.sh; then):
    printf 'import oom_soak\\noom_soak.soak("F15", 600)\\n' > /tmp/launch.py
    python3 tools/board_reboot.py PORT && mpremote connect PORT run /tmp/launch.py
"""

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
    """
    Eat the heap down to ~target_kb free (refs held by the caller).

    Called at GLIDING entry: every long-lived structure (sim body, rings, tasks) is already placed and
    GC is already off, so the ballast carves the REMAINING free pool without starving the bring-up --
    the first attempt ballasted before start() and the fragmented heap killed the sim before ignition.
    Yields per chunk: zeroing ~25 MB of PSRAM blocks for seconds, and the enabled watchdog (wdt_timeout
    1000 ms) must keep feeding while we dig.

    Args:
        target_kb - leave roughly this many KB of heap free.

    Returns:
        The list of ballast bytearrays; the caller must hold the reference to keep the heap pinned.
    """
    hold = []
    """
    Two things learned the hard way here.

    ONE try PER LOOP, not one around both: with a single try the coarse loop's MemoryError skipped the
    fine loop entirely, and it stopped at 5.03 MB free against a 600 KB target -- 8x short, so the soak
    could never reach OOM and timed out still gliding. A coarse chunk failing is how the coarse stage
    ENDS on a fragmented tail; it means "switch to fine", not "stop ballasting".

    And COUNT the chunks from one reading per stage rather than re-measuring per chunk. gc.mem_free()
    walks the whole GC block table -- millions of blocks on this ~30 MB PSRAM heap -- so calling it per
    chunk made ballasting O(n^2), measured at 80 s of wall time for 99 chunks, which starved the very
    flight being soaked (still GLIDING at the 150 s cap instead of landing at ~57 s).
    """
    def _fill(size: int, count: int):
        for _ in range(count):
            hold.append(bytearray(size))

    try:
        coarse = (gc.mem_free() - target_kb * 1024 - 2 * _BIG) // _BIG
        while coarse > 0:
            batch = min(coarse, 8)  # a batch, then a real slot for the WDT feeder
            _fill(_BIG, batch)
            coarse -= batch
            await asyncio.sleep_ms(10)
    except MemoryError:
        pass  # coarse chunks no longer fit -- the fine loop below carves the rest
    try:
        fine = (gc.mem_free() - target_kb * 1024) // _FINE
        while fine > 0:
            batch = min(fine, 16)
            _fill(_FINE, batch)
            fine -= batch
            await asyncio.sleep_ms(10)
    except MemoryError:  # overshoot on a fragmented tail -- close enough, keep what we hold
        pass
    return hold


async def _go(motor: str, target_kb: int, watchdog: bool) -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, 0.05, False, 0.0, 0.0, glider_g=285, inject_hz=25)
    by_name = {component['name']: component for component in cfg['components']}
    """
    watchdog True = the OOM RESET chain under test (rescue collects on the ballast-full heap
    take ~3.4 s and starve any WDT -- expect the panic/reboot). watchdog False = the memory
    RESCUE under test: the physics trigger collects and the flight must complete to DONE.
    """
    by_name['watchdog']['enabled'] = watchdog
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


def soak(motor: str = 'F15', target_kb: int = 2000, watchdog: bool = True) -> None:
    """
    Fly the OOM soak. `target_kb` must clear board_health's forecast RESERVE by a useful margin.

    The old default of 600 KB was set before oom_s started counting down to a 512 KB reserve. That
    left ~88 KB of countdown, so oom_s read ~0 from the first sample, the rescue fired every second,
    and each collect on a ballast-full heap starved the loop -- measured: the flight sat flat at
    ~1.05 MB free for 130 s and never landed, where this is supposed to reach OOM or complete. 2000 KB
    leaves a real countdown. Pass `rescue=False` in the config instead if you want the pure OOM chain
    with no rescue at all.

    Args:
        motor - 'E16' or 'F15'.
        target_kb - free heap to ballast down to; keep it well above board_health's 512 KB reserve.
        watchdog - True tests the OOM RESET chain (expect the board to reset out from under
            mpremote); False tests the memory RESCUE and the flight should complete.

    Returns:
        None; prints the SESSION/STAGE/BALLAST/MEM trace.
    """
    asyncio.run(_go(motor, target_kb, watchdog))
