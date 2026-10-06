"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the ICP-10111 driver (drivers/icp10111.py): @task.driver('icp10111') registration,
graceful setup when absent, and that the TDK polynomial conversion is sane. Deterministic whether or
not an ICP-10111 is wired. Run by `make test`.
"""

import asyncio
import time

import commons
import config_default
import databoard
import inspector
import task
from drivers import icp10111


class _Sink:
    """push()/write() stand-in: the recovery test drives the run loop without a databoard or recorder."""

    def push(self, *args):
        pass


class _StubController:
    config = config_default.default()


class _Bus:
    """A bus that answers every write: the general call 'lands', so the re-arm that follows can be watched."""

    async def writeto(self, address, data):
        pass


class _Peer:
    """A registered device on some bus: counts its rearm() calls (or has none, when `can_rearm` is False)."""

    def __init__(self, name: str, bus, can_rearm: bool = True):
        self.name = name
        self._bus = bus
        self.rearms = 0
        if can_rearm:
            self.rearm = self._rearm

    async def _rearm(self) -> None:
        self.rearms += 1


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

    """
    FRAME CRCs -- positive and negative.

    The sensor appends a CRC to each of its three words and the driver used to read them and throw them
    away. That is the expensive kind of silence: a corrupted I2C word does not raise, it yields a
    plausible altitude, and elevation drives the endgame band, the landing trigger and the launch
    backup. This part also has a documented latch-up habit on this board, which is exactly the state
    that puts bad bytes on the wire.

    POSITIVE: a frame captured live from the wired sensor validates. NEGATIVE: flipping ONE bit in a
    data byte -- leaving its CRC untouched, which is what a corrupted read looks like -- must be
    refused. Both matter: a validator that accepts everything is worthless, and one that rejects
    everything (a wrong polynomial) would silently take out the primary baro.
    """
    live = bytearray(b'\xae\x54\xa9\x3d\x00\x73\x6e\x7a\x9a')  # captured from the real part
    assert commons.sensirion_crc8(live[0], live[1]) == live[2], 'live pressure word failed its own CRC'
    assert commons.sensirion_crc8(live[3], live[4]) == live[5], 'live second word failed its own CRC'
    assert commons.sensirion_crc8(live[6], live[7]) == live[8], 'live temperature word failed its own CRC'
    corrupt = bytearray(live)
    corrupt[0] ^= 0x01                                    # one flipped bit, CRC byte left alone
    assert commons.sensirion_crc8(corrupt[0], corrupt[1]) != corrupt[2], 'a corrupted word passed CRC'

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
    loop_unit._strikes = 0
    recoveries = []

    async def fake_recover():
        recoveries.append(loop_unit._strikes)

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
    assert loop_unit._strikes == 0, 'a good read must rearm the counter'
    fail[0] = True                                  # ...and a LATER wedge escalates again
    await asyncio.sleep_ms(120)
    assert len(recoveries) == 2, recoveries
    task_handle.cancel()


    """
    GENERAL CALL -> RE-ARM EVERY PEER ON THIS BUS, found by membership (the operator's rule, 2026-10-06).

    The general call resets every part on the bus that honours it. The recovery used to re-arm a fixed
    list (bmp280, ina226), so on 7C the SEN0697 parts (BMI323, BMM350, BMP581, added later) stayed in their
    power-on state after a mid-flight recovery of the primary baro. POSITIVE: every peer sharing this Bus
    object is re-armed exactly once, whatever its name. NEGATIVE: a peer on ANOTHER bus is not touched (the
    general call never reached it), the ICP does not re-arm itself, and a peer without rearm() is skipped
    without breaking the walk.
    """
    shared = _Bus()
    recovering = icp10111.Icp10111('baro_icp', {}, _StubController())
    recovering._bus, recovering._period_ms = shared, 0
    peers = [_Peer('imu_bmi323', shared), _Peer('magnet_bmm350', shared), _Peer('baro_bmp581', shared),
             _Peer('laser_agl', shared, can_rearm=False), _Peer('pitot_sdp810', _Bus())]
    for peer in peers:
        inspector.Inspector.register(peer)
    inspector.Inspector.register(recovering)
    try:
        await recovering._recover()
        on_bus = [peer.rearms for peer in peers[:3]]
        assert on_bus == [1, 1, 1], 'every peer on the bus must be re-armed once: %s' % on_bus
        assert peers[4].rearms == 0, 'a peer on another bus was re-armed: the general call never reached it'
        await recovering._rearm_peers()                  # a second recovery re-arms them again, once each
        assert [peer.rearms for peer in peers[:3]] == [2, 2, 2]
    finally:
        for peer in peers:
            inspector.Inspector.unregister(peer.name)
        inspector.Inspector.unregister(recovering.name)

    """
    `update {"rezero": true}` re-zeroes from the driver's OWN channel, fresh within 3 periods. It called
    read() -- the Parameter's method, which a channel does not have -- so every re-zero raised
    AttributeError and the documented pre-launch re-zero answered `err internal`.
    """
    unit = icp10111.Icp10111('baro', {}, _StubController())
    unit._period_ms = 100
    unit._altitude = databoard._Channel('baro', 0)  # born stale: never pushed
    unit._ground = 7.0
    for label in ('never pushed', 'silent for 4 periods'):
        try:
            unit.update({'rezero': True})
            raise AssertionError('re-zero accepted from a channel that is %s' % label)
        except ValueError as error:
            assert 'no fresh altitude' in str(error), error
        assert unit._ground == 7.0, 'a refused re-zero must leave the ground untouched'
        unit._altitude.push(123.5)
        assert unit.update({'rezero': True}) == ['ground'] and unit._ground == 123.5
        unit._ground = 7.0
        unit._altitude.t1 = time.ticks_add(time.ticks_us(), -400000)  # the part goes quiet

    """
    probe() vouches only for pressure produced DURING the probe. It checked value() is not None, which
    holds forever after the first push, so a latched-up ICP (its documented habit) passed verify and arm.
    """
    unit._pressure = databoard._Channel('baro', 0)
    unit._pressure.push(101300.0)                         # a reading from long ago...
    unit._pressure.t1 = time.ticks_add(time.ticks_us(), -2000000)
    unit._addr, unit._period_ms = 0x63, 20
    assert 'no fresh pressure' in await unit.probe(), 'a stopped part must fail its probe'

    async def producing():
        while True:
            unit._pressure.push(101300.0)
            await asyncio.sleep_ms(20)

    feeder = asyncio.create_task(producing())
    verdict = await unit.probe()
    feeder.cancel()
    assert verdict is None, verdict

    print('ok: icp10111 driver registered; graceful-absent; conversion ~%d Pa; frame CRC +/-; run-loop recovery; '
          'general call re-arms every peer on its bus (not others); re-zero fresh-only; probe needs live data' % pa)


asyncio.run(amain())
