# tools/hitl_run.py -- board-side HITL flight runner (MicroPython, runs ON the board). Deploy it with
# rshell (`rshell -p PORT cp tools/hitl_run.py /pyboard/`) and fly a scenario via boardrun + a one-line
# launcher, e.g.:
#   printf 'import hitl_run\nhitl_run.fly("F15", 0.10, 12.0, 210.0, False)\n' > launch.py
#   rshell -p PORT cp launch.py /pyboard/ && tools/boardrun.py PORT runfile launch.py 125
# It brings up config_hitl (real sensors off; the hitl sim feeds the REAL sequencer/flight/pid/mixer/nav),
# ARMS the controller (else the flight loop holds the fins neutral -> no bank -> no descent), and flies to
# DONE (or a 95 s cap). The Recorder streams every stream to the Luckfox (/userdata/recordings/<session>_*
# .csv); pull with adb and assemble with tools/assemble_capture.py. See doc memory `board-data-workflow`.

import asyncio
import time

import config_hitl
import controller
import drivers
import mission
import recorder
import tasks


async def _go(motor: str, noise: float, wind: float, wind_dir: float, spike: bool) -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, noise, spike, wind, wind_dir)
    flight = controller.Controller(cfg, log=lambda message: None)
    await flight.setup()
    await flight.start()
    flight.arm()  # enable actuation -- without it flight.py holds the fins neutral
    print('SESSION', recorder.Recorder.session(), motor, 'noise', noise, 'wind', wind)
    stages = controller.Stage
    started = time.ticks_ms()
    last = -1
    while True:
        stage = flight.stage
        if stage != last:
            print('STAGE', stage, stages.STAGES.get(stage))
            last = stage
        if stage == stages.DONE:
            print('DONE')
            break
        if time.ticks_diff(time.ticks_ms(), started) > 95000:
            print('TIMEOUT', stage)
            break
        await asyncio.sleep_ms(200)
    await asyncio.sleep_ms(1200)  # let the recorder flush the tail to the Luckfox
    await flight.finish()
    print('RUN_END')


def fly(motor: str = 'F15', noise: float = 0.10, wind: float = 0.0,
        wind_dir: float = 210.0, spike: bool = False) -> None:
    """Fly one HITL scenario to completion (or a 95 s cap), recording every stream to the Luckfox."""
    asyncio.run(_go(motor, noise, wind, wind_dir, spike))
