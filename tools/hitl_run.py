# tools/hitl_run.py -- board-side HITL flight runner (MicroPython, runs ON the board). Deploy it with
# `mpremote cp tools/hitl_run.py :` and fly a scenario via `mpremote run` + a one-line launcher, e.g.:
#   printf 'import hitl_run\nhitl_run.fly("F15", 0.10, 12.0, 210.0, False)\n' > /tmp/launch.py
#   tools/board_reboot.py PORT && mpremote connect PORT run /tmp/launch.py
# (hitl_collect.sh wraps this.) boardrun is retired.
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


async def _go(motor: str, noise: float, wind: float, wind_dir: float, spike: bool,
              glider_g: int, inject_hz: int, reboot_s: float, no_cc: bool,
              attitude_drop_s: float = 0.0, gnss_drop_s: float = 0.0) -> None:
    drivers.load()
    tasks.load()
    launch = mission.Mission(max_range_m=200)
    cfg = config_hitl.default(motor, noise, spike, wind, wind_dir, glider_g=glider_g, inject_hz=inject_hz)
    if no_cc:
        # the CC-less scenario: an UNKNOWN field -- no operator zone, no launch point, no known
        # sites. The REAL field agent must synthesize the spiral-landing fallback zone from the sim
        # GNSS fix; the fast tick gives it a decision inside the short pre-ignition SETTING.
        launch.zone = None
        launch.sites = []
        launch.latitude = launch.longitude = None
        launch.site = ''
        by_name = {component['name']: component for component in cfg['components']}
        by_name['field'].update({'enabled': True, 'period_ms': 50})
    flight = controller.Controller(cfg, log=lambda message: None)
    await flight.setup()
    await flight.start()
    if no_cc:  # give the field agent its site decision (fix -> fallback zone) before arming
        for _ in range(60):
            if launch.zone is not None:
                break
            await asyncio.sleep_ms(50)
        print('FIELD zone', launch.site, launch.zone)
    flight.arm()  # enable actuation -- without it flight.py holds the fins neutral
    print('SESSION', recorder.Recorder.session(), motor, 'noise', noise, 'wind', wind)
    stages = controller.Stage
    started = time.ticks_ms()
    last_trace = started
    last = -1
    reboot_at_ms = None  # picked at GLIDING entry when reboot_s > 0
    rebooted = False
    drop_at_ms = None  # picked at GLIDING entry when attitude_drop_s > 0
    dropped = False
    gnss_drop_at_ms = None  # picked at GLIDING entry when gnss_drop_s > 0
    gnss_recover_at_ms = None
    while True:
        stage = flight.stage
        if stage != last:
            print('STAGE', stage, stages.STAGES.get(stage))
            if stage == stages.GLIDING and gnss_drop_s > 0 and gnss_drop_at_ms is None:
                import random
                gnss_drop_at_ms = time.ticks_add(time.ticks_ms(), int((2.0 + random.random() * 4.0) * 1000))
            if stage == stages.GLIDING and reboot_s > 0 and not rebooted:
                # a RANDOM early-to-mid-glide moment (never LANDING: too little altitude to prove
                # anything but luck). A boost-phase reset is NON-restorable by design -- the
                # separation latch reads nested -- so the restorable window starts here.
                import random
                reboot_at_ms = time.ticks_add(time.ticks_ms(), int((1.0 + random.random() * 5.0) * 1000))
            if stage == stages.GLIDING and attitude_drop_s > 0 and not dropped:
                drop_at_ms = time.ticks_add(time.ticks_ms(), int(attitude_drop_s * 1000))
            last = stage
        if reboot_at_ms is not None and not rebooted \
                and time.ticks_diff(time.ticks_ms(), reboot_at_ms) >= 0 and stage == stages.GLIDING:
            rebooted = True
            await _simulated_reboot(flight, reboot_s)
        if drop_at_ms is not None and not dropped \
                and time.ticks_diff(time.ticks_ms(), drop_at_ms) >= 0 and stage == stages.GLIDING:
            dropped = True
            flight.active('hitl').drop_attitude = True  # simulated BNO055 death -> priority-1 backup flies
            print('ATTITUDE DROP: BNO055 off, attitude backup carries the glide')
        if gnss_drop_at_ms is not None and gnss_recover_at_ms is None \
                and time.ticks_diff(time.ticks_ms(), gnss_drop_at_ms) >= 0:
            gnss_recover_at_ms = time.ticks_add(time.ticks_ms(), int(gnss_drop_s * 1000))
            flight.active('hitl').drop_gnss = True  # GNSS out -> open-loop heading tiers + wind feed stalls
            print('GNSS DROP: position/speed/course out for %.1fs' % gnss_drop_s)
        if gnss_recover_at_ms is not None and flight.active('hitl').drop_gnss \
                and time.ticks_diff(time.ticks_ms(), gnss_recover_at_ms) >= 0:
            flight.active('hitl').drop_gnss = False  # fix reacquired
            print('GNSS RECOVER: fix reacquired')
        if stage == stages.DONE:
            print('DONE')
            break
        if time.ticks_diff(time.ticks_ms(), started) > 150000:  # trim-glide flights run ~2x longer
            print('TIMEOUT', stage)
            break
        if stage == stages.GLIDING and time.ticks_diff(time.ticks_ms(), last_trace) >= 5000:
            last_trace = time.ticks_ms()
            body = flight.active('hitl')._body
            print('  t=%ds alt=%.0f roll=%.0f pitch=%.0f hdg=%.0f dropgnss=%s'
                  % (time.ticks_diff(time.ticks_ms(), started) // 1000, body.alt, body.roll,
                     body.pitch, body.heading, flight.active('hitl').drop_gnss))
        await asyncio.sleep_ms(200)
    await asyncio.sleep_ms(1200)  # let the recorder flush the tail to the Luckfox
    await flight.finish()
    print('RUN_END')


async def _simulated_reboot(flight, boot_s: float) -> None:
    """A mid-glide reboot with the REAL warm-start code: the outage (disarm + SETTING under a manual
    hold) lasts `boot_s` like a real boot, then the real breadcrumb is loaded and the real
    five-signal gate decides; a pass restores GLIDING + arm exactly as main._restore_flight does.
    Only the physical inputs are simulated: separated=True (post-separation by construction here),
    the sim's absolute baro altitude, cause=reset. The fins stay FROZEN at their last commanded
    deflection through the outage -- the OOM soak measured that a dying runtime never reaches the
    crash->neutral path, and a rebooting MCU drives no PWM (the servos hold mechanically) -- so the
    flight task's _neutral is stubbed out for the outage (disarmed -> it is the only writer)."""
    import databoard
    import warmstart
    print('REBOOT: outage %.1fs (disarmed, FROZEN fins, stage SETTING)' % boot_s)
    flight_task = flight.active('flight')
    real_neutral = flight_task._neutral if flight_task is not None else None
    if flight_task is not None:
        flight_task._neutral = lambda: None  # outage: no writes at all -> the sim reads frozen fins
    flight.disarm()
    flight.manual = True  # a real reboot has no sequencer either
    flight.set_stage(controller.Stage.SETTING)
    await asyncio.sleep_ms(int(boot_s * 1000))
    if flight_task is not None:
        flight_task._neutral = real_neutral  # boot done: the real fail-safe is back
    crumb = warmstart.load()
    altitude = databoard.Databoard.value('altitude')
    restore, reason = warmstart.should_restore(crumb, True, altitude, True, time.time())
    print('WARM GATE:', restore, reason)
    if restore:
        flight.manual = False
        flight.set_stage(controller.Stage.GLIDING)
        flight.arm()
        print('WARM START -> gliding, armed')
    else:  # by design: any doubt stays a cold boot (the capture will show the uncontrolled descent)
        flight.manual = False


def fly(motor: str = 'F15', noise: float = 0.10, wind: float = 0.0, wind_dir: float = 210.0,
        spike: bool = False, glider_g: int = 285, inject_hz: int = 0,
        reboot_s: float = 0.0, no_cc: bool = False, attitude_drop_s: float = 0.0,
        gnss_drop_s: float = 0.0) -> None:
    """Fly one HITL scenario to completion (or a 95 s cap), recording every stream to the Luckfox.
    `glider_g` is the glider (glide) mass in grams (TMS-7 v3: 285 full, 235 light); the booster adds
    to it for boost then ejects, so a lighter glider glides longer -- the memory-leak stress case.
    `inject_hz` > 0 slims the sim's sensor publish rate (physics still integrate at sim_hz) so the
    on-board HITL leak reflects real flight -- pass e.g. 10 for a memory-measurement run.
    `reboot_s` > 0 simulates a mid-glide reboot: a boot-long outage (neutral fins) at a RANDOM
    early-glide moment, then the real warm-start gate + restore. `no_cc` flies the CC-less scenario:
    no zone/sites -- the field agent synthesizes the spiral-landing fallback from the GNSS fix.
    `attitude_drop_s` > 0 kills the sim `attitude` this many seconds into GLIDING (a BNO055 death):
    the priority-1 complementary-filter backup must carry the glide to a controlled landing.
    `gnss_drop_s` > 0 drops position/speed/course for that many seconds at a random glide moment (a
    tunnel / antenna knock): the guidance falls to its open-loop heading tiers, then recovers. All the
    degradations COMBINE -- pass several at once for the interaction stress (combined-degradation)."""
    asyncio.run(_go(motor, noise, wind, wind_dir, spike, glider_g, inject_hz, reboot_s, no_cc,
                    attitude_drop_s, gnss_drop_s))
