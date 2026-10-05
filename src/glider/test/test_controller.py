"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board (MicroPython) test for the Task base + Controller skeleton. Run by `make test`. Raises
(-> runner reports FAIL) on any failed assertion.
"""

import asyncio

import controller
import i2cbus
import inspector
import task


class FakeSensor(task.Task):
    async def setup(self):
        self.ran = 0
        self._ok = True
        return True

    async def run(self):
        for _ in range(3):
            self.ran += 1
            await asyncio.sleep_ms(1)

    def inspect(self):
        status = task.Task.inspect(self)
        status['ran'] = self.ran
        return status


class FailSensor(task.Task):
    async def setup(self):
        return False


class MessySensor(task.Task):
    """
    setup() raises mid-init AND finish() raises on the half-set-up device.

    The controller must still record the failure and bring up the rest of the board, not abort boot.
    """

    async def setup(self):
        raise RuntimeError('setup boom')

    async def finish(self):
        raise RuntimeError('cleanup boom')


class FlakySensor(task.Task):
    """
    Fails its first setup, succeeds on a retry.

    A fresh instance per attempt, so the attempt count is class-level -- models a breadboard contact
    that makes on the second try.
    """

    attempts: int = 0

    async def setup(self):
        FlakySensor.attempts += 1
        self._ok = FlakySensor.attempts >= 2
        return self._ok

    async def run(self):
        await asyncio.sleep_ms(1)


class DiagnosingSensor(task.Task):
    """
    Fails setup, but offers diagnose().

    The controller must fold the deeper wire-level analysis into the recorded failure reason (the bus
    drivers' adxl375/lsm6dso32/... pattern, surfaced to verify/probe).
    """

    async def setup(self):
        return False

    async def diagnose(self):
        return 'id reads 0x00 -- chip-select not asserting'


class LaserL1xOnly(task.Task):
    """One driver declared twice on one socket: only the entry named laser_agl_l1x finds its part."""

    async def setup(self) -> bool:
        return self.name == 'laser_agl_l1x'


class WrongPart(task.Task):
    """A laser entry whose part is not soldered: its model-id check reads the other part."""

    async def setup(self) -> bool:
        return False

    async def diagnose(self) -> str:
        return 'id reads 0xEA, expected 0xEB -- wrong device on this bus/select (crosswired)'


_AGL: dict = {'agl': {'priority': 0, 'timeout_ms': 100}}  # what both laser entries feed
_UNFITTED: str = 'vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:0 0x29'


class RetunedBus:
    """Stands in for i2cbus.get()'s shared Bus: bustune() only retunes it, so no part is touched."""

    def __init__(self):
        self.freqs: list = []  # every frequency bustune() asked for, in call order

    async def retune(self, freq: int) -> None:
        self.freqs.append(freq)


def laser_config(l1x_addr: int = 0x29, l1x_first: bool = False, extra: tuple = ()) -> dict:
    """
    The two laser entries config_default declares for ONE socket (i2c:0 0x29), plus a UART pair.

    Args:
        l1x_addr - the VL53L1X entry's address; 0x29 is the shared socket.
        l1x_first - declare (and so set up) the VL53L1X entry BEFORE the VL53L4CX one.
        extra - further sensor entries, declared after the lasers.

    Returns:
        A board config for a Controller with the test registry.
    """
    lasers = [{'name': 'laser_agl', 'driver': 'vl53l4cx', 'bus': 'i2c', 'id': 0, 'addr': 0x29, 'enabled': True,
               'provides': _AGL},
              {'name': 'laser_agl_l1x', 'driver': 'vl53l1x', 'bus': 'i2c', 'id': 0, 'addr': l1x_addr,
               'enabled': True, 'provides': _AGL}]
    if l1x_first:
        lasers.reverse()
    position = {'position': {'priority': 0, 'timeout_ms': 1500}}
    return {'board': {'id': 'l', 'mcu': 'esp32p4'}, 'sensors': lasers + list(extra),
            'buses': {'i2c': {'0': {'scl': 8, 'sda': 7, 'freq': 400000}}},
            'components': [{'name': 'gnss', 'driver': 'atgm336h', 'bus': 'uart', 'id': 2, 'addr': None,
                            'enabled': True, 'provides': position},
                           {'name': 'gnss_spare', 'driver': 'neo6mv2', 'bus': 'uart', 'id': 2, 'enabled': True,
                            'provides': position},
                           {'name': 'flight', 'activity': 'flight', 'enabled': True}]}


async def alternatives() -> None:
    """
    A failed entry that an UP device stands in for is an unfitted ALTERNATIVE, not a failure.

    config_default declares laser_agl (vl53l4cx) and laser_agl_l1x (vl53l1x) both on i2c:0 0x29 so the
    part that is soldered wins. The loser's failure used to land in failures, where `arm` refused every
    board with an L1X fitted. Decided after the whole setup pass, so the loser may come first. Only an
    I2C socket counts (an SPI entry's addr is an i2c fallback, its chip-select is the socket), and only
    a winner providing the SAME data: anything else stays a failure, so it still blocks arming.
    """
    fitted = {'vl53l4cx': WrongPart, 'vl53l1x': FakeSensor, 'atgm336h': FakeSensor, 'neo6mv2': FailSensor,
              'flight': FakeSensor}
    for l1x_first in (False, True):  # the loser set up before its winner, and after it
        logs = []
        board = controller.Controller(laser_config(l1x_first=l1x_first), registry=fitted, log=logs.append)
        assert await board.setup() is True
        assert board.alternatives == {'laser_agl': 'laser_agl_l1x'}, board.alternatives
        # NEGATIVE: the UART pair has no address, so the dead one stays a failure beside the winner
        assert list(board.failures) == ['gnss_spare'], board.failures
        assert board.unfitted('laser_agl') == _UNFITTED
        assert board.inspect()['alternatives'] == {'laser_agl': 'laser_agl_l1x'}
        assert 'controller :: laser_agl: ' + _UNFITTED in logs, logs
        assert any('1 device(s) not up: gnss_spare' in line for line in logs), logs
        # the driver a verdict names: `driver`, else `activity`; None for a name no entry declares
        assert board.driver('laser_agl') == 'vl53l4cx' and board.driver('flight') == 'flight'
        assert board.driver('mission') is None
        await board.finish()

    # the bus id compares as TEXT: a hand-edited JSON config may carry '0' where config_default has 0
    text_id = laser_config()
    text_id['sensors'][1]['id'] = '0'
    text_board = controller.Controller(text_id, registry=fitted, log=lambda m: None)
    await text_board.setup()
    assert text_board.alternatives == {'laser_agl': 'laser_agl_l1x'}, text_board.alternatives
    assert list(text_board.failures) == ['gnss_spare'], text_board.failures
    await text_board.finish()

    # NEGATIVE: the part that answers sits on ANOTHER bus -> the dead entry is still a failure
    other_bus = laser_config()
    other_bus['sensors'][1]['id'] = 1
    two_buses = controller.Controller(other_bus, registry=fitted, log=lambda m: None)
    await two_buses.setup()
    assert sorted(two_buses.failures) == ['gnss_spare', 'laser_agl'], two_buses.failures
    assert two_buses.alternatives == {}, two_buses.alternatives
    await two_buses.finish()

    # NEGATIVE: neither part answers -> both stay failures, nothing is an alternative
    dead = dict(fitted, vl53l1x=WrongPart)
    dead_board = controller.Controller(laser_config(), registry=dead, log=lambda m: None)
    await dead_board.setup()
    assert sorted(dead_board.failures) == ['gnss_spare', 'laser_agl', 'laser_agl_l1x'], dead_board.failures
    assert dead_board.alternatives == {}, dead_board.alternatives
    assert 'expected 0xEB' in dead_board.failures['laser_agl'], dead_board.failures
    await dead_board.finish()

    # NEGATIVE: the part that answers sits on ANOTHER address -> the dead entry is still a failure
    two_addresses = controller.Controller(laser_config(l1x_addr=0x30), registry=fitted, log=lambda m: None)
    await two_addresses.setup()
    assert sorted(two_addresses.failures) == ['gnss_spare', 'laser_agl'], two_addresses.failures
    assert two_addresses.alternatives == {}, two_addresses.alternatives
    await two_addresses.finish()

    # NEGATIVE: a baro declared at the laser's 0x29 by mistake feeds OTHER data -> a failure, not "not
    # fitted"; and two entries that provide NOTHING (two displays on 0x3c) have no data to stand in for
    # each other -> the dead one is a failure as well, even beside an up twin with the same (empty) set
    baro = {'name': 'baro_icp10111', 'driver': 'icp10111', 'bus': 'i2c', 'id': 0, 'addr': 0x29, 'enabled': True,
            'provides': {'altitude': {'priority': 0, 'timeout_ms': 200}}}
    display = {'name': 'display', 'driver': 'ssd1306', 'bus': 'i2c', 'id': 0, 'addr': 0x3C, 'enabled': True}
    spare = {'name': 'display_spare', 'driver': 'sh1106', 'bus': 'i2c', 'id': 0, 'addr': 0x3C, 'enabled': True}
    mixed = controller.Controller(laser_config(extra=(baro, display, spare)), log=lambda m: None,
                                  registry=dict(fitted, icp10111=FailSensor, ssd1306=FakeSensor, sh1106=FailSensor))
    await mixed.setup()
    assert mixed.alternatives == {'laser_agl': 'laser_agl_l1x'}, mixed.alternatives
    assert sorted(mixed.failures) == ['baro_icp10111', 'display_spare', 'gnss_spare'], mixed.failures
    await mixed.finish()

    # NEGATIVE: two SPI parts on spi:1 share the fallback addr 0x53 but not a chip-select -> two parts,
    # so the dead one stays a failure even though both feed `accel`
    accel = {'accel': {'priority': 0, 'timeout_ms': 20}}
    spi = {'board': {'id': 'l', 'mcu': 'esp32p4'}, 'sensors': [
        {'name': 'accel_adxl375', 'driver': 'adxl375', 'bus': 'spi', 'id': 1, 'addr': 0x53, 'cs_pin': 'adxl_cs',
         'enabled': True, 'provides': accel},
        {'name': 'accel_spare', 'driver': 'adxl375_spare', 'bus': 'spi', 'id': 1, 'addr': 0x53,
         'cs_pin': 'spare_cs', 'enabled': True, 'provides': accel}]}
    spi_board = controller.Controller(spi, registry={'adxl375': FakeSensor, 'adxl375_spare': FailSensor},
                                      log=lambda m: None)
    await spi_board.setup()
    assert list(spi_board.failures) == ['accel_spare'], spi_board.failures
    assert spi_board.alternatives == {}, spi_board.alternatives
    await spi_board.finish()

    # NEGATIVE: the SAME set of data, not an overlap and not a subset -- a dead {agl} entry beside an up
    # {agl, altitude} one, and the reverse, are two different parts, so the dead one stays a failure
    for loser_feeds, winner_feeds in (({'agl'}, {'agl', 'altitude'}), ({'agl', 'altitude'}, {'agl'})):
        uneven = laser_config()
        for entry, feeds in zip(uneven['sensors'], (loser_feeds, winner_feeds)):
            entry['provides'] = {quantity: {'priority': 0, 'timeout_ms': 100} for quantity in feeds}
        uneven_board = controller.Controller(uneven, registry=fitted, log=lambda m: None)
        await uneven_board.setup()
        assert sorted(uneven_board.failures) == ['gnss_spare', 'laser_agl'], (loser_feeds, uneven_board.failures)
        assert uneven_board.alternatives == {}, (loser_feeds, uneven_board.alternatives)
        await uneven_board.finish()

    # NEGATIVE: a loser whose setup pulses XSHUT is NEVER an alternative. On v0.1 both lasers route XSHUT
    # to GPIO5 and each setup pulses it, so the loser's setup reboots the fitted laser, which then never
    # ranges -- the board armed on it. Its reason names the shared pin, the winner and the fix.
    crosswired = ('setup failed (absent / miswired?) -- id reads 0xEA, expected 0xEB -- wrong device on this '
                  'bus/select (crosswired)')
    shared_reset = ('; shares XSHUT GPIO5 with laser_agl_l1x (vl53l1x): its setup resets the fitted laser -- '
                    'enable exactly one of laser_agl, laser_agl_l1x on this board')
    for l1x_first in (False, True):
        shared = laser_config(l1x_first=l1x_first)
        shared['pins'] = {'laser_xshut': 5}
        for entry in shared['sensors']:
            entry['xshut_pin'] = 'laser_xshut'
        shared_board = controller.Controller(shared, registry=fitted, log=lambda m: None)
        await shared_board.setup()
        assert shared_board.alternatives == {}, shared_board.alternatives
        assert shared_board.failures['laser_agl'] == crosswired + shared_reset, shared_board.failures
        assert sorted(shared_board.failures) == ['gnss_spare', 'laser_agl'], shared_board.failures
        await shared_board.finish()
    # ... GPIO 0 is a routed pin like any other (only None / -1 mean "not routed") ...
    zero = laser_config()
    zero['pins'] = {'laser_xshut': 0}
    zero['sensors'][0]['xshut_pin'] = 'laser_xshut'
    zero_board = controller.Controller(zero, registry=fitted, log=lambda m: None)
    await zero_board.setup()
    assert zero_board.alternatives == {} and 'shares XSHUT GPIO0' in zero_board.failures['laser_agl']
    await zero_board.finish()
    # NEGATIVE: one DRIVER twice on one socket is a config mistake, never an alternative -- the duplicate's
    # setup re-initialises the winner's own part, so a failure mid-way would leave it stopped
    duplicate = laser_config()
    duplicate['sensors'][0]['driver'] = 'vl53l1x'
    duplicate_board = controller.Controller(duplicate, registry=dict(fitted, vl53l1x=LaserL1xOnly),
                                            log=lambda m: None)
    await duplicate_board.setup()
    assert duplicate_board.alternatives == {}, duplicate_board.alternatives
    assert sorted(duplicate_board.failures) == ['gnss_spare', 'laser_agl'], duplicate_board.failures
    await duplicate_board.finish()
    # ... and that holds when only the LOSER routes it: its pulse lands on the socket the winner sits in
    for entry in shared['sensors']:
        if entry['name'] == 'laser_agl_l1x':
            entry['xshut_pin'] = None  # the winner routes none
    loser_pulses = controller.Controller(shared, registry=fitted, log=lambda m: None)
    await loser_pulses.setup()
    assert loser_pulses.alternatives == {} and loser_pulses.failures['laser_agl'] == crosswired + shared_reset
    await loser_pulses.finish()
    # POSITIVE: no pulse from the loser -> paired: XSHUT nulled in the pins map, dropped from both entries
    # (layout on v1.0/v1.1), or routed on the winner alone (only the winner's own setup pulses it). A
    # shared INT never counts: it is an input, wired only after the model id matched, which the loser
    # never reaches -- so every case below routes both entries' int_pin to one GPIO.
    for xshut_gpio, loser_pin, winner_pin in ((None, 'laser_xshut', 'laser_xshut'), (-1, 'laser_xshut', None),
                                              (5, None, None), (5, None, 'laser_xshut')):
        unrouted = laser_config()
        unrouted['pins'] = {'laser_xshut': xshut_gpio, 'laser_int': 3}
        unrouted['sensors'][0]['xshut_pin'], unrouted['sensors'][1]['xshut_pin'] = loser_pin, winner_pin
        for entry in unrouted['sensors'][:2]:
            entry['int_pin'] = 'laser_int'
        unrouted_board = controller.Controller(unrouted, registry=fitted, log=lambda m: None)
        await unrouted_board.setup()
        assert unrouted_board.alternatives == {'laser_agl': 'laser_agl_l1x'}, (xshut_gpio, loser_pin, winner_pin)
        assert list(unrouted_board.failures) == ['gnss_spare'], unrouted_board.failures
        await unrouted_board.finish()

    # a setup re-run RECOMPUTES the alternatives: with the winner still up the loser stays one; after
    # the L1X is gone and the L4CX answers, the roles swap -- a stale entry would leave laser_agl both
    # up and "not fitted"; and with neither answering both are failures again
    rerun = controller.Controller(laser_config(), registry=dict(fitted), log=lambda m: None)
    await rerun.setup()
    await rerun.setup()
    assert rerun.alternatives == {'laser_agl': 'laser_agl_l1x'} and list(rerun.failures) == ['gnss_spare']
    await rerun.close('laser_agl_l1x')
    rerun.registry['vl53l4cx'], rerun.registry['vl53l1x'] = FakeSensor, WrongPart
    await rerun.setup()
    assert rerun.alternatives == {'laser_agl_l1x': 'laser_agl'}, rerun.alternatives
    assert list(rerun.failures) == ['gnss_spare'], rerun.failures
    await rerun.close('laser_agl')
    rerun.registry['vl53l4cx'] = WrongPart
    await rerun.setup()
    assert rerun.alternatives == {} and sorted(rerun.failures) == ['gnss_spare', 'laser_agl', 'laser_agl_l1x']
    await rerun.finish()

    # bustune leaves an alternative OUT (it is not on the bus: counting it 'down' failed every rung of
    # the frequency sweep) and names the driver of any device that is not 'ok'
    original = i2cbus.get
    bus = RetunedBus()
    i2cbus.get = lambda bus_id, spec: bus
    try:
        board = controller.Controller(laser_config(), registry=fitted, log=lambda m: None)
        await board.setup()
        report = await board.bustune('i2c', '0', 1000000)
        assert report == {'kind': 'i2c', 'id': '0', 'freq': 1000000, 'devices': {'laser_agl_l1x': 'ok'},
                          'all_ok': True}, report
        await board.finish()
        # NEGATIVE: with neither laser answering both are down on the bus, each naming its driver
        dead_board = controller.Controller(laser_config(), registry=dead, log=lambda m: None)
        await dead_board.setup()
        report = await dead_board.bustune('i2c', 0, 400000)
        reason = ('setup failed (absent / miswired?) -- id reads 0xEA, expected 0xEB -- wrong device on this '
                  'bus/select (crosswired)')
        assert report['devices'] == {'laser_agl': 'vl53l4cx -- down: ' + reason,
                                     'laser_agl_l1x': 'vl53l1x -- down: ' + reason}, report
        assert report['all_ok'] is False
        await dead_board.finish()
    finally:
        i2cbus.get = original
    assert bus.freqs == [1000000, 400000], bus.freqs


def make_config():
    return {
        'board': {'id': 't', 'mcu': 'esp32p4'},
        'buses': {},
        'pins': {},
        'components': [
            {'name': 's1', 'driver': 'fake', 'enabled': True},
            {'name': 's2', 'driver': 'fake', 'enabled': True},
            {'name': 'off', 'driver': 'fake', 'enabled': False},
            {'name': 'bad', 'driver': 'nodriver', 'enabled': True},
            {'name': 'failing', 'driver': 'fail', 'enabled': True},
        ],
    }


async def amain():
    logs = []
    reg = {'fake': FakeSensor, 'fail': FailSensor}
    c = controller.Controller(make_config(), registry=reg, log=lambda m: logs.append(m))

    # directory() excludes disabled, keeps config order
    assert c.directory() == ['s1', 's2', 'bad', 'failing'], c.directory()

    assert await c.setup() is True
    # s1/s2 created; 'off' disabled; 'bad' has no driver; 'failing' setup() -> False
    assert set(c.tasks.keys()) == set(['s1', 's2']), c.tasks.keys()

    # failures collects every enabled device that did not come up (not the disabled 'off')
    assert set(c.failures.keys()) == set(['bad', 'failing']), c.failures
    assert c.failures['bad'] == 'no driver/activity' and 'setup failed' in c.failures['failing']
    assert c.inspect()['failures'] == c.failures  # exposed for the operator (probe / inspect)
    assert any('2 device(s) not up' in m for m in logs), logs

    # setup_retries: a device that fails its first setup comes up on a retry (breadboard contacts)
    FlakySensor.attempts = 0
    retry_cfg = {'board': {'id': 'r', 'mcu': 'esp32p4', 'setup_retries': 2},
                 'components': [{'name': 'flaky', 'driver': 'flaky', 'enabled': True}]}
    rc = controller.Controller(retry_cfg, registry={'flaky': FlakySensor}, log=lambda m: None)
    assert await rc.setup() is True
    assert 'flaky' in rc.tasks and rc.failures == {} and FlakySensor.attempts == 2  # up on the 2nd try
    await rc.finish()

    # a device whose setup AND cleanup both raise must not abort boot -- it is recorded and the
    # rest of the board still comes up.
    messy_cfg = {'board': {'id': 'm', 'mcu': 'esp32p4'},
                 'components': [{'name': 'messy', 'driver': 'messy', 'enabled': True},
                                {'name': 'ok', 'driver': 'fake', 'enabled': True}]}
    mc = controller.Controller(messy_cfg, registry={'messy': MessySensor, 'fake': FakeSensor},
                               log=lambda m: None)
    assert await mc.setup() is True  # boot completes despite the messy cleanup raise
    assert 'ok' in mc.tasks and 'messy' in mc.failures  # good task up, messy one recorded (not crashed)
    await mc.finish()

    # diagnose(): a failed device that offers diagnose() gets the deeper wire-level reason folded into
    # its failure -- the operator sees 'why' (CS dead / wrong device / ...), not just 'absent / miswired?'
    diag_cfg = {'board': {'id': 'd', 'mcu': 'esp32p4'},
                'components': [{'name': 'diag', 'driver': 'diag', 'enabled': True}]}
    dc = controller.Controller(diag_cfg, registry={'diag': DiagnosingSensor}, log=lambda m: None)
    assert await dc.setup() is True
    assert 'setup failed' in dc.failures['diag']  # the generic reason ...
    assert 'chip-select not asserting' in dc.failures['diag']  # ... plus the folded-in diagnose()
    await dc.finish()

    # active()
    assert c.active('s1') is c.tasks['s1']
    assert c.active('missing') is None
    assert len(c.active()) == 2

    # find(): non-blocking dependency lookup (None for any not up); Task.find delegates
    assert c.find(['s1', 's2']) == [c.tasks['s1'], c.tasks['s2']]
    assert c.find(['s1', 'missing']) == [c.tasks['s1'], None]
    assert c.tasks['s1'].find(['s2']) == [c.tasks['s2']]

    # query(waiting=False) == find; query(waiting=True) returns once all are present
    assert await c.query(['s1'], waiting=False) == [c.tasks['s1']]
    assert await c.tasks['s1'].query(['s2']) == [c.tasks['s2']]

    # Task._pin_gpio: resolve a component's pin field -> board GPIO number (None if absent). One lookup
    # shared by adxl375 / lsm6dso32 / vl53l4cx / separation / sg90 for cs_pin / int_pin / xshut_pin / pin.
    class _PinCtl:
        config = {'pins': {'adxl_cs': 49, 'sep_sw': 33}}
    pc = _PinCtl()
    assert task.Task('p', {'cs_pin': 'adxl_cs'}, pc)._pin_gpio('cs_pin') == 49     # field -> name -> gpio
    assert task.Task('p', {'cs_pin': 'adxl_cs'}, pc)._pin_gpio('int_pin') is None  # component omits the field
    assert task.Task('p', {'cs_pin': 'nope'}, pc)._pin_gpio('cs_pin') is None      # name not in the pins map
    assert task.Task('p', {}, pc)._pin_gpio('pin', 'sep_sw') == 33                 # default pin name when omitted
    assert task.Task('p', {}, pc)._pin_gpio('pin') is None                         # no field and no default

    # query(waiting=True) parks until a not-yet-present task appears
    late = FakeSensor('late', {}, c)

    async def appear():
        await asyncio.sleep_ms(20)
        c.tasks['late'] = late

    asyncio.create_task(appear())
    assert await c.query(['s1', 'late'], waiting=True) == [c.tasks['s1'], late]
    c.tasks.pop('late')  # drop the never-set-up fixture so it doesn't skew validate()/stats()

    assert c.validate() is True

    # run the task loops, then check stats()
    await c.start()
    await asyncio.sleep_ms(50)
    rep = c.stats()
    assert rep['stage'] == 'setting'  # the operator-facing stage name
    assert rep['tasks']['s1']['ran'] >= 1, rep

    # tasks are individually inspectable through the Inspector
    assert inspector.Inspector.inspect('s1')['ran'] >= 1

    # notify/emit
    seen = []
    c.tasks['s1'].notify(lambda emitter, ev: seen.append(ev))
    c.tasks['s1'].emit('hello')
    assert seen == ['hello']

    # stage machine: int ids internally, name on the wire
    c.set_stage(controller.Stage.BOOSTING)
    assert c.stage == controller.Stage.BOOSTING and c.stage_name() == 'boosting'
    raised = False
    try:
        c.set_stage(99)  # not a defined stage
    except ValueError:
        raised = True
    assert raised

    # arming + manual hold (the actuation safety gate + ground-test override); arm() also pins the
    # launch point through a registered mission's freeze_launch (bare stubs are tolerated)
    class _FreezableMission:
        name = 'mission'
        frozen = False

        def freeze_launch(self):
            self.frozen = True

    field_mission = _FreezableMission()
    inspector.Inspector.register(field_mission)
    assert c.armed is False and c.manual is False  # disarmed / auto by default
    c.arm()
    assert c.armed is True and c.inspect()['armed'] is True
    assert field_mission.frozen is True  # arm pinned the launch point
    inspector.Inspector.unregister('mission')
    c.disarm()
    assert c.armed is False
    assert c.hold('gliding') is True and c.stage_name() == 'gliding' and c.manual is True
    assert c.hold('nope') is False  # unknown stage name
    c.resume()
    assert c.manual is False and c.inspect()['manual'] is False
    # `stage setting` is a RETURN TO THE GROUND, not a hold: holding SETTING suppressed every launch
    # detector, and a board released that way flew its whole flight recorded as SETTING
    assert c.hold('gliding') is True and c.manual is True
    assert c.hold('setting') is True and c.stage_name() == 'setting' and c.manual is False

    # close one, then finish all
    await c.close('s1')
    assert 's1' not in c.tasks
    await c.finish()
    assert c.tasks == {}
    assert c.stage == controller.Stage.DONE

    """
    A task whose RUN LOOP crashes must be reported as down, not left healthy.

    It used to be logged to the console and nowhere else: the task stayed in tasks, stayed out of
    failures, and kept validate() True -- so a sensor whose loop had died still passed the readiness
    gate `cc arm` consults, and its data merely stopped appearing, which reads like a quiet sensor
    rather than a dead one. Only a failed SETUP was covered before; a crash after a successful setup
    was not.
    """
    class Exploding(task.Task):
        async def setup(self) -> bool:
            self._ok = True
            return True

        async def run(self) -> None:
            raise RuntimeError('loop died')

    crash_cfg = {'board': {'id': 'c', 'mcu': 'esp32p4'},
                 'components': [{'name': 'boom', 'driver': 'boom', 'enabled': True}]}
    cc_ctl = controller.Controller(crash_cfg, registry={'boom': Exploding}, log=lambda m: None)
    assert await cc_ctl.setup() is True
    assert cc_ctl.active('boom').validate() is True and cc_ctl.failures == {}  # healthy until it runs
    await cc_ctl.start()
    await asyncio.sleep_ms(50)                       # let the loop run and die
    assert cc_ctl.active('boom').validate() is False, 'a crashed loop must not still report healthy'
    assert 'boom' in cc_ctl.failures, cc_ctl.failures
    await cc_ctl.finish()

    await alternatives()

    print('ok: controller directory/create/setup/run/active/inspect/stats/validate/close/finish + pin_gpio '
          '+ crashed run loop reported down + unfitted alternatives on a shared socket')


asyncio.run(amain())
