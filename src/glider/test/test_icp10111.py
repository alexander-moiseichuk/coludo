"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the ICP-10111 driver (drivers/icp10111.py): @task.driver('icp10111') registration,
graceful setup when absent, and that the TDK polynomial conversion is sane. Deterministic whether or
not an ICP-10111 is wired. Run by `make test`.
"""

import asyncio

import config_default
import task
from drivers import icp10111


class _Sink:
    """push()/write() stand-in: the recovery test drives the run loop without a databoard or recorder."""

    def push(self, *args):
        pass


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('icp10111') is icp10111.Icp10111  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = icp10111.Icp10111('baro', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    """
    a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it).
    This also walks the soft-reset preamble's never-acks exit (4 tries over ~90 ms -- the busy
    window a mid-conversion sensor NAKs through after an unclean reboot, OSError 19).
    """
    absent = icp10111.Icp10111('baro', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    # conversion against real OTP + raw values captured live from the wired sensor -> ~101797 Pa
    probe = icp10111.Icp10111('baro', {}, _StubController())
    probe._otp = [211, 371, 553, 3833]
    pa = probe._compensate(11441968, 27034)
    assert 100000.0 < pa < 103000.0, pa  # matches the live ~1017.97 hPa reading

    """
    MID-FLIGHT RECOVERY. setup() hardens the BOOT path against a latched digital core, but the run
    loop had none: a read that started failing logged once (deduped) and then failed forever, so a
    mid-air latch-up cost the PRIMARY baro for the rest of the flight -- and `elevation` drives the
    endgame band, the landing fallback and the launch backup. Escalation must fire ONCE at the
    threshold (not every tick while the part stays dead) and a good read must rearm it.
    """
    loop_unit = icp10111.Icp10111('baro', {}, _StubController())
    loop_unit._failures = 0
    recoveries = []

    async def fake_recover():
        recoveries.append(loop_unit._failures)

    loop_unit._recover = fake_recover
    fail = [True]

    async def fake_read():
        if fail[0]:
            raise OSError(19)
        return (100.0, 21.0, 101300.0)

    loop_unit._read = fake_read
    loop_unit._altitude = loop_unit._temperature = loop_unit._pressure = _Sink()
    loop_unit._elevation = _Sink()
    loop_unit._telemetry = _Sink()
    loop_unit._ground = 0.0
    loop_unit._period_ms = 1

    task_handle = asyncio.create_task(loop_unit.run())
    await asyncio.sleep_ms(120)                     # plenty of ticks to pass the threshold
    assert len(recoveries) == 1, 'recovery must fire ONCE, not every tick: %s' % recoveries
    assert recoveries[0] == 3, recoveries          # at _RECOVER_AFTER consecutive failures
    fail[0] = False                                 # the part comes back
    await asyncio.sleep_ms(30)
    assert loop_unit._failures == 0, 'a good read must rearm the counter'
    fail[0] = True                                  # ...and a LATER wedge escalates again
    await asyncio.sleep_ms(120)
    assert len(recoveries) == 2, recoveries
    task_handle.cancel()

    print('ok: icp10111 driver registered; graceful-absent; conversion ~%d Pa; run-loop recovery' % pa)


asyncio.run(amain())
