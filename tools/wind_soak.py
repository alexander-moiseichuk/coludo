# tools/wind_soak.py -- wind-estimation validation (MicroPython, runs ON the board). Flies a HITL glide
# with a KNOWN sim wind and prints the estimated wind (flight.vitals) vs the sim truth through the glide,
# so the triangle + loiter orbit-mean can be checked against ground truth. Deploy first
# (cd src/glider && ./deploy.sh), then:
#   printf 'import wind_soak\nwind_soak.soak("F15", 6.0, 270.0)\n' > /tmp/launch.py
#   python3 tools/board_reboot.py PORT && mpremote connect PORT run /tmp/launch.py

import asyncio
import math
import time

import config_hitl
import controller
import drivers
import mission
import navigation
import tasks


async def _go(motor: str, wind_mps: float, wind_dir: float) -> None:
    drivers.load()
    tasks.load()
    mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, 0.05, False, wind_mps, wind_dir, glider_g=285, inject_hz=25)
    flight = controller.Controller(cfg, log=lambda message: None)
    await flight.setup()
    await flight.start()
    flight.arm()
    ft = flight.active('flight')
    body = flight.active('hitl')._body
    print('SESSION', motor, 'wind', wind_mps, 'toward', wind_dir)
    stages = controller.Stage
    started = time.ticks_ms()
    last = -1
    target = None          # zone-centre (lat, lon), fetched once the guidance has a fix/zone
    last_miss = None       # TRUE distance from the body to the target (removes GNSS noise from the metric)
    min_miss = None        # closest approach through the glide
    while True:
        stage = flight.stage
        if stage != last:
            print('STAGE', stages.STAGES.get(stage))
            last = stage
        if target is None and ft._guidance._mission is not None:
            geo = ft._guidance._mission.geometry()
            target = geo['target'] if geo else None
        if target is not None:  # track the touchdown miss every iteration (crab-need measurement)
            pos = body.position()
            last_miss = navigation.distance(pos[0], pos[1], target[0], target[1])
            min_miss = last_miss if min_miss is None else min(min_miss, last_miss)
        if stage == stages.GLIDING:
            st = ft._wind.stats()  # method + estimate + the raw triangle components (see wind.stats())
            true_spd = math.sqrt(body.wind_e * body.wind_e + body.wind_n * body.wind_n)
            true_from = math.degrees(math.atan2(-body.wind_e, -body.wind_n)) % 360.0 if true_spd else 0.0
            print('WIND est %.1f from %3d (%-8s) | true %.1f from %3d | air %.1f hdg %3d gs %.1f '
                  '| we %.1f wn %.1f'
                  % (st['speed'], st['from'], st['method'], true_spd, true_from,
                     body.speed, int(body.heading), body.ground_speed(),
                     st['we'], st['wn']))
            await asyncio.sleep_ms(2000)
            continue
        if stage == stages.DONE:
            print('MISS final=%.1f min=%.1f m | wind %.1f toward %.0f'
                  % (last_miss or -1, min_miss or -1, wind_mps, wind_dir))
            print('DONE')
            break
        if time.ticks_diff(time.ticks_ms(), started) > 150000:
            print('TIMEOUT')
            break
        await asyncio.sleep_ms(200)
    await asyncio.sleep_ms(800)
    await flight.finish()
    print('RUN_END')


def soak(motor: str = 'F15', wind_mps: float = 6.0, wind_dir: float = 270.0) -> None:
    asyncio.run(_go(motor, wind_mps, wind_dir))
