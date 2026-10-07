"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture controller.json: plan B's device bring-up (src/glider/controller.py) and the operator lines CC builds
from its outcome (src/glider/cc_client.py), as the Rust port (crates/controller) must answer them. Everything compares
EXACTLY: every log line, every retry gap, every failure reason, every verdict line.

Plan B's own Controller runs over STUB task classes, the pattern of src/glider/test/test_controller.py:15-60: no stub
touches hardware, it answers what its spec says. A case names its stubs in `input.registry`, {implementation: spec},
each spec a JSON object:
  setup     'up' (setup() returns True), 'fail' (False), 'raise' (setup() raises) or 'flaky' (True from the class's
            `up_at`-th attempt on, counted across every entry of that class, as test_controller.py FlakySensor);
  up_at     'flaky' only;
  names     with 'up' only: just the entries with these names come up, the others fail (test_controller.py
            LaserL1xOnly: one driver declared twice on one socket);
  error     'raise' only: the repr of the exception setup() raises, as plan B logs and records it;
  diagnose  the text the stub's diagnose() answers; absent, the class has no diagnose();
  crash     true: the stub's run() raises once the loops start (see input.crashes).
An implementation the registry lacks has no class, so create() finds no driver. A stub's finish() never raises:
plan B's `cleanup raised` line has no Rust counterpart (a part is dropped, and a drop cannot fail).

Each case is {name, input, output}, and `input.function` says what ran:
  setup   one Controller.setup() over input.config (keys sorted). When input.crashes is present, the run loops are
          then started, so each `crash` stub dies; input.crashes is {name: reason}, the reason plan B's supervisor
          recorded for it (controller.py:518-529), which the port's supervisor (M6+) hands the table as data. out:
            log         every line setup() logged, in order;
            sleeps      every asyncio.sleep_ms(ms) setup() awaited, in order: the retry gaps;
            sorts       one {tasks, failures, alternatives, after} per _sort_alternatives() call: the up tasks in their
                        order and the failures [name, reason] it started from, then the alternatives [loser, winner]
                        and the failures it left, each in plan B's dict order;
            inspect     controller.inspect()'s tasks, failures and alternatives, after the crashes;
            unfitted    {name: unfitted(name)} for each alternative, after the crashes;
            verify      the `verify` reply's devices, problems and alternatives (cc_client.py:807-816), answered by
                        plan B's real dispatcher;
            tasks       the `health` reply's tasks (cc_client.py:513), the same dispatcher;
            probed      cc_client._probed(controller, name, result) for each [name, result] of input.probes: every
                        directory name and 'mission' (an inspectable that is no device), each with result null (a
                        healthy probe) and with _PROBE_FAILURE;
            failure     {name: cc_client._failure(controller, name)} for every name in controller.failures.
  rerun   Controller.setup() run once per input.passes entry over one config (test_controller.py:309-325): before
          its pass, an entry closes the tasks it names and swaps in its registry. out: sorts, as above. The firmware
          brings a board up once per boot, so only the alternatives pass is replayed for these.

Inputs: every _sort_alternatives scenario test_controller.py:145-325 runs (both setup orders, a text bus id, two
buses, two addresses, a baro and two displays beside the lasers, SPI parts sharing a fallback address, uneven
`provides`, XSHUT on GPIO 5 and GPIO 0, the same driver twice, a pulse from the loser alone, null and -1 pins, the
re-run), the retry traces (a flaky part, a failing part with and without diagnose(), an empty diagnosis, no driver, a
raising part, the counts setup_retries reads), the winner's crash, the sockets plan B compares as text (an id-less
entry is 'None'), and three board files after layout: the taster with both lasers absent (no winner: both stay
failures, so a missing laser still blocks arming, controller.py:213-214), 7F with its VL53L1X fitted beside an enabled
VL53L4CX (the alternative M5 stage 5 shows on hardware) and 7C (v0.1, both lasers on XSHUT GPIO 5).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import copy
import json
import os
import tempfile

import cc_client
import cc_protocol as cc
import commons
import config_corpus
import controller as controller_module
import layout
import shims
import task

_AGL: dict = {'agl': {'priority': 0, 'timeout_ms': 100}}  # what both laser entries feed (test_controller.py:103)

"""The VL53L1X's model-id high byte, which a VL53L4CX socket answers as 0xEB (vl53l1x.py diagnose())."""
_MODEL_ID_HIGH: int = 0xEA
_OTHER_PART: int = 0xEB

"""What a laser's diagnose() answers, by plan B's own classifier: the other part soldered, and nothing at all."""
_WRONG_PART: str = commons.id_classify(_MODEL_ID_HIGH, _OTHER_PART)
_ABSENT: str = commons.id_classify(None, _MODEL_ID_HIGH)

_PROBE_FAILURE: str = 'probe FAILED: no ack'  # a failing probe() result, for the _probed rows
_NO_DEVICE: str = 'mission'  # an inspectable that is no configured device: _named passes its result through

_LOOP_DIED: str = 'loop died'  # what a `crash` stub's run() raises
_CONFIG_KEYS: tuple = ('board', 'buses', 'pins', 'sensors', 'components')  # what a board file case keeps

_TMS7C_FILE: str = 'launches/20261003/TMS-7C/tms7c.config'


def _laser_config(l1x_addr: int = 0x29, l1x_first: bool = False, extra: tuple = ()) -> dict:
    """
    test_controller.py:117-142 laser_config(): the two laser entries config_default declares for ONE socket (i2c:0
    0x29), then `extra` sensors, plus a UART pair with no address and the flight activity.
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


def _laser(config: dict, name: str) -> dict:
    """A laser entry of a _laser_config() document, by name."""
    return next(entry for entry in config['sensors'] if entry['name'] == name)


"""The stub specs test_controller.py's classes stand for."""
_UP: dict = {'setup': 'up'}  # FakeSensor
_FAIL: dict = {'setup': 'fail'}  # FailSensor
_WRONG: dict = {'setup': 'fail', 'diagnose': _WRONG_PART}  # WrongPart
_GONE: dict = {'setup': 'fail', 'diagnose': _ABSENT}  # a laser with nothing in its socket

"""test_controller.py:155 `fitted`: the VL53L1X answers, the VL53L4CX entry reads the other part."""
_FITTED: dict = {'vl53l4cx': _WRONG, 'vl53l1x': _UP, 'atgm336h': _UP, 'neo6mv2': _FAIL, 'flight': _UP}


def _raising(message: str, **spec) -> dict:
    """A stub spec whose setup() raises RuntimeError(message); `error` is the repr plan B logs and records."""
    return dict(spec, setup='raise', error=repr(RuntimeError(message)), message=message)


def _stub(spec: dict) -> type:
    """
    The Task class a stub spec describes (see the module docstring).

    Args:
        spec - the spec, possibly with the builder-only `message` of a raising stub.

    Returns:
        A task.Task subclass with its own attempt counter.
    """
    counter = {'attempts': 0}  # class-level, as FlakySensor.attempts: every entry of the class counts

    class Stub(task.Task):
        async def setup(self) -> bool:
            counter['attempts'] += 1
            kind = spec['setup']
            if kind == 'raise':
                raise RuntimeError(spec['message'])
            if kind == 'flaky':
                self._ok = counter['attempts'] >= spec['up_at']
            else:
                self._ok = kind == 'up' and self.name in spec.get('names', [self.name])
            return self._ok

        async def run(self) -> None:
            if spec.get('crash'):
                raise RuntimeError(_LOOP_DIED)

    if 'diagnose' in spec:
        async def diagnose(self) -> str:
            return spec['diagnose']

        Stub.diagnose = diagnose
    return Stub


def _registry(registry: dict) -> dict:
    """The Controller's class registry for a {implementation: spec} registry."""
    return {implementation: _stub(spec) for implementation, spec in registry.items()}


def _published(registry: dict) -> dict:
    """A registry as the fixture writes it: the builder-only `message` of a raising stub left out."""
    return {implementation: {key: value for key, value in spec.items() if key != 'message'}
            for implementation, spec in registry.items()}


class _Sleeps:
    """
    Records every asyncio.sleep_ms(ms) awaited inside the `with`, and moves the virtual clock by as much without
    waiting for real: the retry gaps are the only sleeps a setup pass awaits, and nothing else is running.
    """

    def __enter__(self) -> '_Sleeps':
        self.durations: list = []
        self.saved = asyncio.sleep_ms

        async def sleep_ms(milliseconds: int) -> None:
            self.durations.append(milliseconds)
            await asyncio.sleep(0)
            shims.advance(milliseconds)

        asyncio.sleep_ms = sleep_ms
        return self

    def __exit__(self, *exception) -> bool:
        asyncio.sleep_ms = self.saved
        return False


def _traced(board: controller_module.Controller) -> list:
    """
    Record every _sort_alternatives() call of `board`: the state it starts from and the state it leaves.

    Returns:
        The list the traces are appended to.
    """
    sorts = []
    original = board._sort_alternatives

    def sort_alternatives() -> None:
        trace = {'tasks': list(board.tasks), 'failures': [[name, reason] for name, reason in board.failures.items()]}
        original()
        trace['alternatives'] = [[loser, winner] for loser, winner in board.alternatives.items()]
        trace['after'] = [[name, reason] for name, reason in board.failures.items()]
        sorts.append(trace)

    board._sort_alternatives = sort_alternatives
    return sorts


async def _reply(dispatcher: cc_client.Dispatcher, request: str) -> dict:
    """The JSON object a dispatcher answers `request` with (its first argument); the reply must be ok."""
    message = cc.parse(await dispatcher.handle(request))
    if message.command != 'ok':
        raise ValueError('%s answered %r' % (request, message.line))
    return json.loads(message.args[0])


async def _setup_case(name: str, config: dict, registry: dict) -> dict:
    """
    One setup case: plan B's Controller over `config` with stubs, then what CC reads from it.

    Returns:
        The case.
    """
    config = config_corpus.sorted_keys(config)
    lines = []
    board = controller_module.Controller(copy.deepcopy(config), registry=_registry(registry), log=lines.append)
    sorts = _traced(board)
    with _Sleeps() as sleeps:
        await board.setup()
    log = list(lines)
    crashing = [entry for entry, spec in registry.items() if spec.get('crash')]
    crashes = {}
    if crashing:
        await board.start()
        for _ in range(10):  # let every loop run once (and the crashing ones die)
            await asyncio.sleep(0)
        crashes = {task_name: reason for task_name, reason in board.failures.items() if task_name in board.tasks}
        if not crashes:
            raise ValueError('%s: a crash stub did not crash' % name)
    with tempfile.TemporaryDirectory() as scratch:
        dispatcher = cc_client.create_dispatcher(board.config, controller=board,
                                                 config_path=os.path.join(scratch, 'board.config'))
        verify = await _reply(dispatcher, 'verify')
        health = await _reply(dispatcher, 'health')
    inspected = board.inspect()
    probes = [[device, result] for device in board.directory() + [_NO_DEVICE] for result in (None, _PROBE_FAILURE)]
    output = {
        'log': log,
        'sleeps': sleeps.durations,
        'sorts': sorts,
        'inspect': {key: inspected[key] for key in ('tasks', 'failures', 'alternatives')},
        'unfitted': {loser: board.unfitted(loser) for loser in board.alternatives},
        'verify': {key: verify[key] for key in ('devices', 'problems', 'alternatives')},
        'tasks': health['tasks'],
        'probed': [cc_client._probed(board, device, result) for device, result in probes],
        'failure': {device: cc_client._failure(board, device) for device in board.failures},
    }
    await board.finish()  # unregisters the tasks, so the next case's probe sweep sees only its own
    case_input = {'function': 'setup', 'config': config, 'registry': _published(registry), 'probes': probes}
    if crashes:
        case_input['crashes'] = crashes
    return {'name': name, 'input': case_input, 'output': output}


async def _rerun_case(name: str, config: dict, passes: list) -> dict:
    """
    One rerun case: setup() once per pass, each after closing its `close` tasks and swapping in its registry.

    Returns:
        The case.
    """
    config = config_corpus.sorted_keys(config)
    board = controller_module.Controller(copy.deepcopy(config), registry={}, log=lambda line: None)
    sorts = _traced(board)
    for each in passes:
        for closed in each.get('close', []):
            await board.close(closed)
        board.registry = _registry(each['registry'])
        await board.setup()
    await board.finish()
    published = [dict(each, registry=_published(each['registry'])) for each in passes]
    return {'name': name, 'input': {'function': 'rerun', 'config': config, 'passes': published},
            'output': {'sorts': sorts}}


def _alternatives_scenarios() -> list:
    """test_controller.py:145-307 alternatives(), case by case, plus the sockets plan B compares as text."""
    scenarios = []
    for l1x_first in (False, True):
        order = 'l1x_first' if l1x_first else 'l4cx_first'
        scenarios.append(('alternative_%s' % order, _laser_config(l1x_first=l1x_first), _FITTED))

    text_id = _laser_config()
    _laser(text_id, 'laser_agl_l1x')['id'] = '0'
    scenarios.append(('alternative_a_text_bus_id_is_the_same_socket', text_id, _FITTED))

    other_bus = _laser_config()
    _laser(other_bus, 'laser_agl_l1x')['id'] = 1
    scenarios.append(('no_alternative_on_another_bus', other_bus, _FITTED))

    scenarios.append(('no_alternative_when_neither_answers', _laser_config(), dict(_FITTED, vl53l1x=_WRONG)))
    scenarios.append(('no_alternative_at_another_address', _laser_config(l1x_addr=0x30), _FITTED))

    baro = {'name': 'baro_icp10111', 'driver': 'icp10111', 'bus': 'i2c', 'id': 0, 'addr': 0x29, 'enabled': True,
            'provides': {'altitude': {'priority': 0, 'timeout_ms': 200}}}
    display = {'name': 'display', 'driver': 'ssd1306', 'bus': 'i2c', 'id': 0, 'addr': 0x3C, 'enabled': True}
    spare = {'name': 'display_spare', 'driver': 'sh1106', 'bus': 'i2c', 'id': 0, 'addr': 0x3C, 'enabled': True}
    scenarios.append(('no_alternative_for_other_data_or_none', _laser_config(extra=(baro, display, spare)),
                      dict(_FITTED, icp10111=_FAIL, ssd1306=_UP, sh1106=_FAIL)))

    accel = {'accel': {'priority': 0, 'timeout_ms': 20}}
    spi = {'board': {'id': 'l', 'mcu': 'esp32p4'}, 'sensors': [
        {'name': 'accel_adxl375', 'driver': 'adxl375', 'bus': 'spi', 'id': 1, 'addr': 0x53, 'cs_pin': 'adxl_cs',
         'enabled': True, 'provides': accel},
        {'name': 'accel_spare', 'driver': 'adxl375_spare', 'bus': 'spi', 'id': 1, 'addr': 0x53,
         'cs_pin': 'spare_cs', 'enabled': True, 'provides': accel}]}
    scenarios.append(('no_alternative_on_spi', spi, {'adxl375': _UP, 'adxl375_spare': _FAIL}))

    for label, loser_feeds, winner_feeds in (('a_subset', ('agl',), ('agl', 'altitude')),
                                             ('a_superset', ('agl', 'altitude'), ('agl',))):
        uneven = _laser_config()
        for entry, feeds in zip(uneven['sensors'], (loser_feeds, winner_feeds)):
            entry['provides'] = {quantity: {'priority': 0, 'timeout_ms': 100} for quantity in feeds}
        scenarios.append(('no_alternative_for_%s_of_the_data' % label, uneven, _FITTED))

    for l1x_first in (False, True):
        shared = _laser_config(l1x_first=l1x_first)
        shared['pins'] = {'laser_xshut': 5}
        for entry in shared['sensors']:
            entry['xshut_pin'] = 'laser_xshut'
        order = 'l1x_first' if l1x_first else 'l4cx_first'
        scenarios.append(('no_alternative_sharing_xshut_gpio5_%s' % order, shared, _FITTED))

    zero = _laser_config()
    zero['pins'] = {'laser_xshut': 0}
    _laser(zero, 'laser_agl')['xshut_pin'] = 'laser_xshut'
    scenarios.append(('no_alternative_sharing_xshut_gpio0', zero, _FITTED))

    duplicate = _laser_config()
    _laser(duplicate, 'laser_agl')['driver'] = 'vl53l1x'
    scenarios.append(('no_alternative_for_the_same_driver_twice', duplicate,
                      dict(_FITTED, vl53l1x=dict(_UP, names=['laser_agl_l1x']))))

    loser_pulses = _laser_config()
    loser_pulses['pins'] = {'laser_xshut': 5}
    _laser(loser_pulses, 'laser_agl')['xshut_pin'] = 'laser_xshut'
    _laser(loser_pulses, 'laser_agl_l1x')['xshut_pin'] = None
    scenarios.append(('no_alternative_when_only_the_loser_pulses_xshut', loser_pulses, _FITTED))

    for label, xshut_gpio, loser_pin, winner_pin in (('a_null_xshut', None, 'laser_xshut', 'laser_xshut'),
                                                     ('a_negative_xshut', -1, 'laser_xshut', None),
                                                     ('no_xshut_entries', 5, None, None),
                                                     ('xshut_on_the_winner_alone', 5, None, 'laser_xshut')):
        unrouted = _laser_config()
        unrouted['pins'] = {'laser_xshut': xshut_gpio, 'laser_int': 3}
        _laser(unrouted, 'laser_agl')['xshut_pin'] = loser_pin
        _laser(unrouted, 'laser_agl_l1x')['xshut_pin'] = winner_pin
        for entry in unrouted['sensors'][:2]:
            entry['int_pin'] = 'laser_int'
        scenarios.append(('alternative_with_%s' % label, unrouted, _FITTED))

    # plan B compares str(id): an id-less entry's socket is 'None', which matches another id-less one only
    idless = _laser_config()
    for entry in idless['sensors']:
        del entry['id']
    scenarios.append(('alternative_on_two_idless_sockets', idless, _FITTED))
    half_idless = _laser_config()
    _laser(half_idless, 'laser_agl')['id'] = None
    scenarios.append(('no_alternative_between_a_null_id_and_id_0', half_idless, _FITTED))

    # rule 1 again: no `bus` key is no socket, whatever the driver defaults to; rule 2: an empty `provides`
    busless = _laser_config()
    del _laser(busless, 'laser_agl')['bus']
    scenarios.append(('no_alternative_without_a_bus_key', busless, _FITTED))
    empty = _laser_config()
    _laser(empty, 'laser_agl')['provides'] = {}
    _laser(empty, 'laser_agl_l1x')['provides'] = {}
    scenarios.append(('no_alternative_when_both_provide_nothing', empty, _FITTED))
    return scenarios


def _retry_scenarios() -> list:
    """Retry traces: controller.py:385-445 _bring_up and _diagnose, over board.setup_retries."""
    def single(retries: object) -> dict:
        """One component, 'device', running 'part', with this board.setup_retries (None: no key)."""
        board = {'id': 'r', 'mcu': 'esp32p4'}
        if retries is not None:
            board['setup_retries'] = retries
        return {'board': board, 'components': [{'name': 'device', 'driver': 'part', 'enabled': True}]}

    flaky = {'setup': 'flaky', 'up_at': 2}
    scenarios = [
        ('retry_flaky_up_on_the_second_attempt', single(3), {'part': flaky}),
        ('retry_flaky_out_of_attempts', single(1), {'part': flaky}),
        ('retry_failing_diagnosed_on_the_last_attempt', single(3), {'part': _GONE}),
        ('retry_failing_without_diagnose', single(2), {'part': _FAIL}),
        ('retry_an_empty_diagnosis_keeps_the_reason', single(2), {'part': dict(_FAIL, diagnose='')}),
        ('retry_no_driver_is_never_retried', single(3), {'other': _UP}),
        ('retry_a_raising_part_diagnosed', single(2), {'part': _raising('setup boom', diagnose=_ABSENT)}),
        ('retry_a_raising_part_once', single(None), {'part': _raising('setup boom')}),
        ('retry_zero_is_one_attempt', single(0), {'part': _GONE}),
        ('retry_a_negative_count_is_one_attempt', single(-2), {'part': _GONE}),
        ('retry_true_is_one_attempt', single(True), {'part': _GONE}),
        ('retry_up_at_once', single(3), {'part': _UP}),
    ]
    # test_controller.py:354-385 make_config(): two up, one disabled, one with no driver, one failing
    make_config = {'board': {'id': 't', 'mcu': 'esp32p4'}, 'buses': {}, 'pins': {}, 'components': [
        {'name': 's1', 'driver': 'fake', 'enabled': True}, {'name': 's2', 'driver': 'fake', 'enabled': True},
        {'name': 'off', 'driver': 'fake', 'enabled': False}, {'name': 'bad', 'driver': 'nodriver', 'enabled': True},
        {'name': 'failing', 'driver': 'fail', 'enabled': True}]}
    scenarios.append(('directory_up_disabled_no_driver_failing', make_config, {'fake': _UP, 'fail': _FAIL}))
    # test_controller.py:396-405: a raising part is recorded and the rest of the board still comes up
    messy = {'board': {'id': 'm', 'mcu': 'esp32p4'}, 'components': [
        {'name': 'messy', 'driver': 'messy', 'enabled': True}, {'name': 'ok', 'driver': 'fake', 'enabled': True}]}
    scenarios.append(('a_raising_part_does_not_stop_the_rest', messy, {'messy': _raising('setup boom'),
                                                                        'fake': _UP}))
    return scenarios


def _crash_scenarios() -> list:
    """A winner whose run loop dies after setup: the loser stays not fitted, the line says the winner is down."""
    return [('winner_crashed_after_setup', _laser_config(), dict(_FITTED, vl53l1x=dict(_UP, crash=True))),
            ('a_lone_part_crashed_after_setup', _laser_config(), dict(_FITTED, atgm336h=dict(_UP, crash=True)))]


def _board_file(path: str, declared: str = None, enabled: tuple = (), added: tuple = ()) -> dict:
    """
    A committed board file as plan B's Controller reads it: through layout.resolve (declared, so no scan runs), with
    only the sections the Controller reads kept.

    Args:
        path - the board file (config_corpus.LAUNCH_FILES).
        declared - the board.layout to declare, or None to keep the file's own.
        enabled - device names to enable first.
        added - (template device, new name, new driver): a copy of a device entry, appended after it.

    Returns:
        The post-layout document.
    """
    document = config_corpus.read_file(path)
    if declared is not None:
        document['board']['layout'] = declared
    for template, name, driver in added:
        index = config_corpus.index_of(document, 'sensors', template)
        twin = dict(copy.deepcopy(document['sensors'][index]), name=name, driver=driver)
        document['sensors'].insert(index + 1, twin)
    for name in enabled:
        config_corpus.index_of(document, 'sensors', name)  # refuses an unknown name
        for entry in document['sensors']:
            if entry.get('name') == name:
                entry['enabled'] = True
    layout.resolve(document, log=lambda line: None)
    return {key: document[key] for key in _CONFIG_KEYS if key in document}


def _board_registry(document: dict, lasers: dict, missing: tuple = ()) -> dict:
    """Every implementation a board file runs as an up stub, the lasers as given, `missing` left unregistered."""
    implementations = {entry.get('driver') or entry.get('activity')
                       for entry in document['sensors'] + document['components']}
    registry = {implementation: _UP for implementation in sorted(implementations) if implementation not in missing}
    registry.update(lasers)
    return registry


def _board_scenarios() -> list:
    """The board files the M5 proofs run, after layout."""
    taster = _board_file(config_corpus.TMS7F_FILE, declared='v1.1', enabled=('laser_agl',))
    tms7c = _board_file(_TMS7C_FILE, added=(('laser_agl', 'laser_agl_l1x', 'vl53l1x'),))
    return [
        ('taster_both_lasers_absent', taster,
         _board_registry(taster, {'vl53l4cx': _GONE, 'vl53l1x': _GONE}, missing=('bluetooth',))),
        ('tms7f_the_l1x_fitted', taster, _board_registry(taster, {'vl53l4cx': _WRONG, 'vl53l1x': _UP})),
        ('tms7c_both_lasers_on_xshut_gpio5', tms7c, _board_registry(tms7c, {'vl53l4cx': _WRONG, 'vl53l1x': _UP})),
    ]


def _rerun_passes() -> list:
    """test_controller.py:309-325: the same winner twice, then the roles swapped, then neither answering."""
    swapped = dict(_FITTED, vl53l4cx=_UP, vl53l1x=_WRONG)
    return [{'registry': _FITTED}, {'registry': _FITTED},
            {'close': ['laser_agl_l1x'], 'registry': swapped},
            {'close': ['laser_agl'], 'registry': dict(swapped, vl53l4cx=_WRONG)}]


async def _cases() -> list:
    """Every case, in fixture order."""
    cases = []
    for name, config, registry in (_alternatives_scenarios() + _retry_scenarios() + _crash_scenarios()
                                   + _board_scenarios()):
        cases.append(await _setup_case(name, config, registry))
    cases.append(await _rerun_case('rerun_recomputes_the_alternatives', _laser_config(), _rerun_passes()))
    names = [case['name'] for case in cases]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError('duplicate cases: %s' % ', '.join(duplicates))
    return cases


def build() -> dict:
    """
    The controller fixture document.

    Returns:
        {meta, compare, cases}.
    """
    return {
        'meta': {'source': 'src/glider/controller.py, src/glider/cc_client.py, src/glider/commons.py, '
                           'src/glider/layout.py, launches/20261003/TMS-7C/tms7c.config, '
                           'launches/20261003/TMS-7F/tms7f.config'},
        'compare': {'kind': 'exact'},
        'cases': asyncio.run(_cases()),
    }
