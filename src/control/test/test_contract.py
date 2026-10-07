"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test of the CC contract runner (src/control/contract, tools/cc_contract.py).

Positive: plan B's own dispatcher (the `fake` transport) passes the whole contract -- every case of every tier,
with every allow flag, claiming every milestone -- and its round trips put the launch config and the board
config back. Only the board-only cases skip: the hub tier (the fake has no hub) and the log tee (plan B on
CPython tees nothing).

Negative: the runner catches a board that breaks the contract. A transport that answers through plan B but
replaces one reply on purpose -- a wrong status, a missing JSON key, a reply to an empty line, two replies to
one line, an unknown error code, a value an invariant rejects -- must fail exactly that case. Nothing active (a
fin sweep, a config write, a stage hold, a reboot) is ever sent without its flag, nor to a board that reads
airborne or armed, nor -- with a flag -- to one whose ground state is unreadable; a hold never follows an arm
that was not undone. An `ok` to reboot passes only when a new boot answers. A run that stops on an exception
undoes its writes. A not-yet answer FAILs once the claimed milestone owes the case.

The hub and serial transports are tested without a hub or a board: the line a hub request must carry, the
reply it rebuilds, a stub hub's HTTP errors and its dashboard rows (the hub tier), the console's log and ROM
banner filter and its one-line-in-one-line-out watch, and a console open that cannot reset the board.
"""

import contextlib
import http.server
import io
import json
import os
import sys
import threading
import time
import types
import typing

_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'control'))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import cc_contract  # noqa: E402
import cc_protocol as cc  # noqa: E402
from contract import cases, normalize, transports  # noqa: E402

_PLAN_B: transports.Fake = transports.Fake()  # one plan B for the whole file (tools' own runs build theirs)
_EPOCH_2020: int = 1577836800  # Unix seconds at 2020-01-01: any real host clock reads later
# the cases plan B on CPython cannot run: no tee, no hub, and no device set up (the M6 parts' cases)
_BOARD_ONLY: set = {'log_tee_trigger', 'hub', 'inspect_imu', 'inspect_barometer', 'inspect_separation',
                    'update_barometer_ground', 'update_barometer_text_ground', 'update_barometer_rezero',
                    'calibrate_barometer', 'calibrate_imu', 'probe_devices'}
_PROBE_LAUNCH: str = cc.build('set-config', ['launch', json.dumps({'launch_id': 'contract-probe'})])


class _Tampered:
    """
    A board that answers through plan B, except for one request whose reply lines are replaced.

    It records every line sent, so a test can also prove what was NOT sent.
    """

    name: str = 'tampered plan B'
    tees: bool = False  # plan B's: it tees nothing

    def __init__(self, request: str | None = None, replace: typing.Callable | None = None,
                 occurrence: int | None = None, forward: bool = True) -> None:
        """
        Answer through plan B, replacing the reply to `request`.

        Args:
            request - the request line whose reply is replaced (None: no reply is).
            replace - (plan B's reply lines) -> the reply lines to answer instead.
            occurrence - replace only the n-th time the request is sent (1-based); None for every time.
            forward - whether the tampered request still reaches plan B; when not, `replace` gets [].
        """
        self.sent: list = []
        self._request: str | None = request
        self._replace: typing.Callable | None = replace
        self._occurrence: int | None = occurrence
        self._forward: bool = forward

    def exchange(self, line: str) -> list:
        """Record the line, ask plan B (unless held back), and replace the reply when it is the tampered one."""
        self.sent.append(line)
        tampered = line == self._request and self._occurrence in (None, self.sent.count(line))
        if tampered and not self._forward:
            return self._replace([])
        lines = _PLAN_B.exchange(line)
        return self._replace(lines) if tampered else lines

    def await_reboot(self) -> None:
        """Plan B's own wait for its intercepted reset."""
        _PLAN_B.await_reboot()

    def close(self) -> None:
        """The shared plan B stays open for the next test."""


def _fitted_answers() -> dict:
    """The healthy M6 parts' answers, by request line: the snapshots, the ICP's update round trip, calibrate, probe."""
    ok = lambda payload: [cc.build('ok', [json.dumps(payload)])]  # noqa: E731
    update = lambda properties: cc.build('update', ['baro_icp10111', json.dumps(properties)])  # noqa: E731
    return {
        'inspect imu_lsm6dso32': ok({'name': 'imu_lsm6dso32', 'ok': True, 'healthy': True, 'interrupt': True,
                                     'interrupt_silent': False, 'accel_g': [0.01, -0.02, 1.01],
                                     'rate_dps': [0.1, 0.0, -0.1]}),
        'inspect baro_icp10111': ok({'name': 'baro_icp10111', 'ok': True, 'healthy': True, 'altitude_m': 42.1,
                                     'temperature_c': None, 'pressure_pa': 100810.5, 'elevation_m': 0.2}),
        'inspect separation': ok({'name': 'separation', 'ok': True, 'healthy': True, 'separated': False}),
        update({'ground': 100}): ok({'changed': ['ground']}),
        update({'ground': 'high'}): [cc.build('err', ['refused', "could not convert string to float: 'high'"])],
        update({'rezero': True}): ok({'changed': ['ground']}),
        'calibrate baro_icp10111': ok({'baro_icp10111': None}),
        'calibrate imu_lsm6dso32': ok({'imu_lsm6dso32': None}),
        'probe imu_lsm6dso32': ok({'imu_lsm6dso32': None}),
        'probe baro_icp10111': ok({'baro_icp10111': None}),
        'probe separation': ok({'separation': None}),
    }


class _Fitted(_Tampered):
    """
    Plan B with the M6 parts fitted, which plan B on CPython never sets up: `objects` lists the default config's
    imu_lsm6dso32, baro_icp10111 and separation, and their requests get `answers` (request line -> reply lines, or
    an exception to raise) instead of reaching plan B.
    """

    def __init__(self, answers: dict) -> None:
        """Answer through plan B, the M6 parts by `answers`."""
        super().__init__()
        self._answers: dict = answers

    def exchange(self, line: str) -> list:
        """Record the line; a part's request gets its answer, `objects` lists the parts, the rest is plan B's."""
        self.sent.append(line)
        if line in self._answers:
            answer = self._answers[line]
            if isinstance(answer, BaseException):
                raise answer
            return answer
        lines = _PLAN_B.exchange(line)
        if line == 'objects':
            names = json.loads(cc.parse(lines[0]).args[0])
            return [cc.build('ok', [json.dumps(sorted(names + ['baro_icp10111', 'imu_lsm6dso32', 'separation']))])]
        return lines


class _Settling(_Tampered):
    """A board behind a real transport: its await_reboot only waits, so the runner's whoami proof does the rest."""

    def await_reboot(self) -> None:
        """No settle at all: the first whoami goes out while plan B's reset is still 200 ms away."""


class _Teeing(_Tampered):
    """A board that tees its log: the first `log 0` after `probe mission` drains `logged`."""

    tees: bool = True

    def __init__(self, logged: list) -> None:
        """
        Answer through plan B, with a log tee.

        Args:
            logged - the lines the probe logged ([] for a stub bridge that tees nothing).
        """
        super().__init__()
        self._logged: list = logged
        self._drained: bool = False

    def exchange(self, line: str) -> list:
        """Plan B's answer, except the drain right after the probe, which carries `logged`."""
        lines = super().exchange(line)
        if line == 'log 0' and 'probe mission' in self.sent and not self._drained:
            self._drained = True
            return [cc.build('ok', [json.dumps({'lines': self._logged, 'dropped': 0})])]
        return lines


def _payload_edit(edit: typing.Callable) -> typing.Callable:
    """A reply replacement that edits the reply's last (JSON) argument and re-encodes it."""

    def replace(lines: list) -> list:
        """The reply with its payload edited."""
        message = cc.parse(lines[0])
        payload = json.loads(message.args[-1])
        edit(payload)
        return [cc.build(message.command, message.args[:-1] + [json.dumps(payload)])]

    return replace


def _answer(status: str, payload: object) -> typing.Callable:
    """A reply replacement answering `status <payload as JSON>`, whatever plan B said."""
    return lambda lines: [cc.build(status, [json.dumps(payload)])]


def _config_writes(sent: list) -> list:
    """The set-config lines that carry a document: the error-path probes carry none, or text that is no JSON."""
    return [line for line in sent if line.startswith('set-config ') and len(line.split()) == 3
            and line.split()[2].startswith('base64:')]


def _failed(results: list) -> set:
    """The names of the FAIL results."""
    return {result.name for result in results if result.outcome == cases.FAIL}


@contextlib.contextmanager
def _patched(module: object, **values: object) -> typing.Iterator:
    """Set module attributes for the duration of a block, then put them back."""
    saved = {name: getattr(module, name) for name in values}
    for name, value in values.items():
        setattr(module, name, value)
    try:
        yield
    finally:
        for name, value in saved.items():
            setattr(module, name, value)


def test_plan_b_keeps_the_contract() -> None:
    """Every tier, every flag, every milestone: every case passes -- plan B IS the reference -- and it rebooted."""
    rebooted = _PLAN_B.reboots
    results = cases.run(_PLAN_B, cases.TIERS, cases.ALLOW_FLAGS, None, milestone=15)
    others = [(result.name, result.outcome, result.reason) for result in results if result.outcome != cases.PASS]
    assert {name for name, outcome, _reason in others if outcome == cases.SKIP} == _BOARD_ONLY, others
    assert len(others) == len(_BOARD_ONLY), others
    assert {result.tier for result in results} == set(cases.TIERS)
    assert len(results) > len(cases.CASES), 'inspect_every_object runs once per listed object'
    assert _PLAN_B.reboots == rebooted + 1, 'the reboot case must make the intercepted reset fire'
    assert 'inspect_every_object[inspect mission]' in {result.name for result in results}
    assert 'stage gliding' in {result.request for result in results}, 'the hold went out: disarmed after the arm'


def test_plan_b_stays_in_its_own_process() -> None:
    """The shims plan B needs never reach this process (its clock stays real), and a dead plan B is no answer."""
    assert 'micropython' not in sys.modules and 'shims' not in sys.modules
    assert time.time() > _EPOCH_2020, 'the runner process clock was moved to 2000'
    assert time.strftime('%Y', time.gmtime()) >= '2020'
    dying = transports.Fake()
    assert dying.exchange('ping') == ['pong']
    dying._child.kill()
    dying._child.wait()
    try:
        dying.exchange('ping')
    except transports.NoAnswer as error:
        assert 'plan B exited' in str(error), error
    else:
        raise AssertionError('a dead plan B answered')
    dying.close()


def test_round_trips_put_the_board_back() -> None:
    """The launch config reads as before the run; a probe write that answers badly is still undone."""
    before = _PLAN_B.exchange('get-config launch')
    cases.run(_PLAN_B, ('config',), cases.ALLOW_FLAGS, None)
    assert _PLAN_B.exchange('get-config launch') == before
    broken = _Tampered(_PROBE_LAUNCH, lambda lines: [cc.build('err', ['internal', 'boom'])])
    results = {result.name: result for result in cases.run(broken, ('config',), ('launch_write',), None)}
    assert results['set_config_launch_probe'].outcome == cases.FAIL
    assert results['set_config_launch_restore'].outcome == cases.PASS, results['set_config_launch_restore']
    assert _PLAN_B.exchange('get-config launch') == before, 'the restore must run whatever the write answered'
    restored = json.loads(cc.parse(before[0]).args[0])
    assert broken.sent.count(cc.build('set-config', ['launch', json.dumps({'launch_id': restored['launch_id']})])) == 1


def test_nothing_active_without_its_flag() -> None:
    """No flag: no fin sweep, config write, stage hold or reboot leaves the runner; each flag opens only its own."""
    rebooted = _PLAN_B.reboots
    recording = _Tampered()
    results = cases.run(recording, cases.TIERS, (), None)
    assert not _failed(results)
    for line in recording.sent:
        tokens = line.split()
        command = tokens[0] if tokens else ''
        assert command not in ('reboot', 'arm', 'probe', 'verify', 'detect', 'bustune', 'reset-config'), line
        assert not (command in ('calibrate', 'stage') and len(tokens) > 1), line
    assert _config_writes(recording.sent) == [], recording.sent
    assert _PLAN_B.reboots == rebooted
    reasons = {result.name: result.reason for result in results}
    assert reasons['reboot'] == 'needs --allow-reboot' and reasons['probe_all'] == 'needs --allow-active'
    assert reasons['stage_hold'] == 'needs --allow-stage'
    assert reasons['set_config_board_resave'] == 'needs --allow-config-write'
    assert reasons['set_config_launch_probe'] == 'needs --allow-launch-write'
    teeing = _Teeing(['1 Mission :: probe: launch position ...'])
    cases.run(teeing, cases.TIERS, (), None)
    assert [line for line in teeing.sent if line.startswith('probe')] == ['probe mission'], 'a data check only'
    active = _Tampered()
    cases.run(active, cases.TIERS, ('active',), None)
    assert 'probe' in active.sent and 'arm' in active.sent and 'verify' in active.sent
    assert 'reboot' not in active.sent and 'stage gliding' not in active.sent
    assert _config_writes(active.sent) == [], active.sent
    launch = _Tampered()
    cases.run(launch, cases.TIERS, ('launch_write',), None)
    assert _PROBE_LAUNCH in launch.sent
    assert all(line.startswith('set-config launch ') for line in _config_writes(launch.sent)), launch.sent
    assert not any(line.startswith('stage ') for line in launch.sent)


def test_violations_are_caught() -> None:
    """Each broken core reply fails exactly its own case(s), with a reason naming what broke."""

    def drop_epoch(payload: dict) -> None:
        """whoami without its epoch."""
        del payload['epoch']

    def epoch_in_1999(payload: dict) -> None:
        """whoami with a clock before any RTC's floor."""
        payload['epoch'] = 915148800

    def config_id_not_hex(payload: dict) -> None:
        """whoami with a config id that is no SHA-256 prefix."""
        payload['config_id'] = 'board-7c'

    def mem_free_as_text(payload: dict) -> None:
        """health with a vital as text."""
        payload['mem_free'] = '8 MB'

    def no_armed(payload: dict) -> None:
        """health without armed, which every board with a Controller sends."""
        del payload['armed']

    def agl_as_integer(payload: dict) -> None:
        """health with an integer agl: the laser reports metres as a float."""
        payload['agl'] = 3

    def degraded_not_text(payload: dict) -> None:
        """health with a degraded item that is not text."""
        payload['degraded'] = [5]

    def layout_unknown(payload: dict) -> None:
        """health with a layout verdict no resolver gives."""
        payload['layout'] = 'v9'

    def task_without_ok(payload: dict) -> None:
        """health with a task the dashboard cannot render."""
        payload['tasks'] = [{'name': 'flight'}]

    def calibration_unflagged(payload: dict) -> None:
        """health with calibration outstanding but no needs-calibration flag."""
        payload['calibration'] = {'imu_bno055': 'move it in a figure 8'}

    scenarios = (
        ('ping', lambda lines: ['ok'], {'ping', 'link_survives_errors'}, 'expected pong'),
        ('ping', lambda lines: ['pong extra'], {'ping', 'link_survives_errors'}, 'expected 0 argument(s)'),
        ('ping', lambda lines: [], {'ping', 'link_survives_errors'}, 'no reply'),
        ('ping', lambda lines: [cc.build('err', ['unsupported', 'x'])], {'ping', 'link_survives_errors'},
         'expected pong'),  # the core tier has no not-yet answer
        ('whoami', _payload_edit(drop_epoch), {'whoami'}, "lacks key 'epoch'"),
        ('whoami', _payload_edit(epoch_in_1999), {'whoami'}, 'before 2000-01-01'),
        ('whoami', _payload_edit(config_id_not_hex), {'whoami'}, 'is not 12 (or 8) hex digits'),
        ('health', _payload_edit(mem_free_as_text), {'health'}, "key 'mem_free' is str"),
        ('health', _payload_edit(no_armed), {'health'}, "lacks key 'armed'"),
        ('health', _payload_edit(agl_as_integer), {'health'}, "key 'agl' is int, expected float|null"),
        ('health', _payload_edit(degraded_not_text), {'health'}, 'degraded holds a non-text item'),
        ('health', _payload_edit(layout_unknown), {'health'}, "layout 'v9'"),
        ('health', _payload_edit(task_without_ok), {'health'}, 'a tasks item is not'),
        ('health', _payload_edit(calibration_unflagged), {'health'}, 'lacks needs-calibration'),
        ('', lambda lines: ['pong'], {'empty_line'}, 'a reply to an empty line'),
        ('PING', lambda lines: ['pong', 'pong'], {'command_is_lowercased'}, '2 replies to one line'),
        ('frobnicate', lambda lines: ['err nosuchcode frobnicate'], {'unknown_command'}, 'error code'),
        ('frobnicate', lambda lines: ['err badcmd'], {'unknown_command'}, 'a code and a message'),
        ('FROB', lambda lines: ['err badcmd FROB'], {'unknown_command_is_lowercased'}, "'FROB'"),
        ('inspect base64:QQ', lambda lines: ['ok base64:e30='], {'parse_error_bad_padding'}, 'expected err badargs'),
        ('ping', lambda lines: ['pong reason=x'], {'ping', 'link_survives_errors'}, 'key=value'),
        ('ping', lambda lines: ['base64:@@ pong'], {'ping', 'link_survives_errors'}, 'none of ok/err/pong/iam'),
    )
    for request, replace, names, reason in scenarios:
        results = cases.run(_Tampered(request, replace), ('core',), (), None)
        failed = [result for result in results if result.outcome == cases.FAIL]
        assert {result.name for result in failed} == names, (request, [(item.name, item.reason) for item in failed])
        assert all(reason in result.reason for result in failed), (request, [result.reason for result in failed])
        assert cc_contract.violations(results) == len(names)
        assert cc_contract.summary(results)['core']['verdict'] == cases.FAIL


def test_invariants_beyond_the_core_are_checked() -> None:
    """Each invariant of the other tiers fires on its own broken answer, and fails exactly its own case."""

    def site_moved(payload: dict) -> None:
        """A launch config whose site is not the one read before the round trip."""
        payload['site'] = 'elsewhere'

    def board_renamed(payload: dict) -> None:
        """A running config naming another board."""
        payload['board']['id'] = 'TMS-7Z'

    def hold_not_flagged(payload: dict) -> None:
        """health while held, without STAGE HELD."""
        payload['degraded'] = [item for item in payload['degraded'] if item != 'STAGE HELD']

    def still_manual(payload: dict) -> None:
        """`stage auto` that leaves the hold on."""
        payload['manual'] = True

    def passes_with_problems(payload: dict) -> None:
        """verify passing while it lists a problem."""
        payload['pass'] = True
        payload['problems'] = {'servo_yaw': 'sg90 -- down: no pulse'}

    def ready_with_readiness(payload: dict) -> None:
        """verify ready while it lists a readiness item."""
        payload['ready'] = True

    launch = ('config',), ('launch_write',)
    scenarios = (
        (launch, 'get-config launch', _payload_edit(site_moved), 3, {'set_config_launch_restored'},
         "differs from the launch config before the round trip in ['site']"),
        (launch, 'get-config launch', _payload_edit(site_moved), 2, {'set_config_launch_merges'},
         "fields other than launch_id moved: ['site']"),
        ((('config',), ()), 'get-config board', _payload_edit(board_renamed), None, {'get_config_board'},
         "board.id is 'TMS-7Z'"),
        ((('config',), ()), 'get-config running', _payload_edit(board_renamed), None, {'get_config_running'},
         "differs from get-config board in ['board']"),
        ((('control',), ('stage',)), 'health', _payload_edit(hold_not_flagged), 2, {'health_while_held'},
         'degraded lacks STAGE HELD'),
        ((('control',), ('stage',)), 'stage auto', _payload_edit(still_manual), None, {'stage_resume'},
         "stage reads ('setting', True)"),
        ((('control',), ()), 'disarm', _answer('ok', {'armed': True}), None, {'disarm'}, 'armed reads True'),
        ((('diagnostics',), ('active',)), 'verify', _payload_edit(passes_with_problems), None, {'verify'},
         'pass is True with 1 problem(s)'),
        ((('diagnostics',), ('active',)), 'verify', _payload_edit(ready_with_readiness), None, {'verify'},
         'ready is True with 2 readiness item(s)'),
        ((('stream',), ()), 'log 0', _answer('ok', {'lines': ['1 x :: y'], 'dropped': 0}), 2,
         {'log_closed_window_collects_nothing'}, 'a closed window collected 1 lines'),
        ((('stream',), ()), 'tlm 0', _answer('ok', {'samples': [], 'dropped': -1}), 1, {'tlm_stop'},
         'dropped -1 is negative'),
        ((('inspect',), ()), 'objects', _answer('ok', ['controller']), None, {'objects'}, 'objects lacks mission'),
        ((('inspect',), ()), cc.build('update', ['mission', '{}']), _answer('ok', {'changed': ['epoch']}), None,
         {'update_mission_nothing'}, "an empty update changed ['epoch']"),
    )
    for (tiers, allowed), request, replace, occurrence, names, reason in scenarios:
        results = cases.run(_Tampered(request, replace, occurrence), tiers, allowed, None)
        failed = [result for result in results if result.outcome == cases.FAIL]
        assert {result.name for result in failed} == names, (request, [(item.name, item.reason) for item in failed])
        assert all(reason in result.reason for result in failed), (request, [result.reason for result in failed])
    assert json.loads(cc.parse(_PLAN_B.exchange('stage')[0]).args[0]) == {'stage': 'setting', 'manual': False}


def test_the_m6_parts_are_held_to_plan_b_shapes() -> None:
    """
    The M6 parts' cases pass on healthy answers, and each broken answer fails exactly its own case: a snapshot short
    of a key or naming another part, a ground write that changed nothing, a calibrate or a probe that failed. A
    board without the parts skips them; a run that stops after the ground write re-zeroes on its way out.
    """
    tiers, allowed = ('config', 'inspect', 'diagnostics'), ('active',)
    results = cases.run(_Fitted(_fitted_answers()), tiers, allowed, None, milestone=6)
    m6 = {result.name: result.outcome for result in results if result.name.split('[')[0] in _BOARD_ONLY}
    assert set(m6.values()) == {cases.PASS}, m6
    assert {name.split('[')[0] for name in m6} == _BOARD_ONLY - {'log_tee_trigger', 'hub'}, m6
    assert len([name for name in m6 if name.startswith('probe_devices[')]) == 3, m6

    def broken(request: str, payload: object, status: str = 'ok') -> dict:
        """The healthy answers with one replaced."""
        answers = _fitted_answers()
        answers[request] = [cc.build(status, [json.dumps(payload)])]
        return answers

    update = lambda properties: cc.build('update', ['baro_icp10111', json.dumps(properties)])  # noqa: E731
    scenarios = (
        (broken('inspect separation', {'name': 'separation', 'ok': True, 'healthy': True}),
         {'inspect_separation'}, "lacks key 'separated'"),
        (broken('inspect baro_icp10111', {'name': 'baro', 'ok': True, 'healthy': True, 'altitude_m': None,
                                          'temperature_c': None, 'pressure_pa': None, 'elevation_m': None}),
         {'inspect_barometer'}, "inspect baro_icp10111 names 'baro'"),
        (broken(update({'ground': 100}), {'changed': []}), {'update_barometer_ground'}, 'expected [ground]'),
        (broken(update({'ground': 'high'}), {'changed': ['ground']}), {'update_barometer_text_ground'}, ''),
        (broken('calibrate baro_icp10111', {'baro_icp10111': 'ground zero: no fresh altitude'}),
         {'calibrate_barometer'}, 'expected {\'baro_icp10111\': null}'),
        (broken('probe separation', {'separation': 'separation -- pin 33 reads nothing'}),
         {'probe_devices[probe separation]'}, 'expected {\'separation\': null}'),
    )
    for answers, names, reason in scenarios:
        results = cases.run(_Fitted(answers), tiers, allowed, None)
        failed = [result for result in results if result.outcome == cases.FAIL]
        assert {result.name for result in failed} == names, [(result.name, result.reason) for result in failed]
        assert all(reason in result.reason for result in failed), [result.reason for result in failed]

    bare = {result.name: result for result in cases.run(_PLAN_B, tiers, allowed, None)}
    assert bare['inspect_barometer'].outcome == cases.SKIP and 'no icp10111' in bare['inspect_barometer'].reason

    answers = _fitted_answers()
    answers[update({'ground': 'high'})] = RuntimeError('USB gone')  # right after the ground write
    unplugged = _Fitted(answers)
    try:
        cases.run(unplugged, tiers, allowed, None)
    except cases.Aborted as error:
        assert 'barometer re-zero: ok' in str(error), error
    else:
        raise AssertionError('a transport error did not abort the run')
    assert unplugged.sent[-1] == update({'rezero': True}), unplugged.sent[-3:]


def test_the_hold_needs_a_disarmed_board() -> None:
    """An arm that was not undone, or a disarm that did not take, blocks the hold: an armed board is never held."""
    stuck = _Tampered('disarm', lambda lines: [], occurrence=2)  # the disarm after the arm gets no answer
    results = {result.name: result for result in cases.run(stuck, ('control',), ('active', 'stage'), None)}
    assert results['arm'].outcome == cases.PASS and results['disarm_after_arm'].outcome == cases.FAIL
    assert results['stage_hold'].outcome == cases.SKIP and 'not seen disarmed' in results['stage_hold'].reason
    assert 'stage gliding' not in stuck.sent and 'stage setting' not in stuck.sent
    refused = _Tampered('disarm', _answer('ok', {'armed': True}))
    results = {result.name: result for result in cases.run(refused, ('control',), ('stage',), None)}
    assert results['disarm'].outcome == cases.FAIL and results['stage_hold'].outcome == cases.SKIP
    assert 'stage gliding' not in refused.sent
    assert json.loads(cc.parse(_PLAN_B.exchange('health')[0]).args[0])['armed'] is False


def test_not_yet_is_reported_not_failed() -> None:
    """unsupported outside the core is NOT YET; badcmd is NOT YET for push-* only; a not-yet read blocks its write."""
    unsupported = _Tampered('get-config launch', lambda lines: [cc.build('err', ['unsupported', 'no mission'])])
    results = {result.name: result for result in cases.run(unsupported, ('config',), ('launch_write',), None)}
    assert results['get_config_launch'].outcome == cases.NOT_YET, results['get_config_launch']
    assert results['set_config_launch_probe'].outcome == cases.SKIP, 'nothing to restore to: the write must not go'
    assert _PROBE_LAUNCH not in unsupported.sent
    assert cc_contract.violations(list(results.values())) == 0
    push = cases.run(_Tampered('push-status', lambda lines: ['err badcmd push-status']), ('ota',), (), None)
    assert [result.outcome for result in push] == [cases.NOT_YET]
    objects = cases.run(_Tampered('objects', lambda lines: ['err badcmd objects']), ('inspect',), (), None)
    assert _failed(objects) == {'objects'}


def test_a_milestone_makes_its_cases_mandatory() -> None:
    """--milestone: a not-yet answer to a case owed by the claimed milestone FAILs; one owed later stays NOT YET."""
    unsupported = [cc.build('err', ['unsupported', 'no tee yet'])]
    log_cases = {'log_stop', 'log_closed_window_collects_nothing', 'log_drain_and_stop'}
    results = cases.run(_Tampered('log 0', lambda lines: unsupported), ('stream',), (), None)
    assert {result.name for result in results if result.outcome == cases.NOT_YET} == log_cases and not _failed(results)
    results = cases.run(_Tampered('log 0', lambda lines: unsupported), ('stream',), (), None, milestone=1)
    assert _failed(results) == log_cases
    assert all('owed from M1' in result.reason for result in results if result.outcome == cases.FAIL)
    for milestone, outcome in ((1, cases.NOT_YET), (2, cases.NOT_YET), (3, cases.FAIL)):
        results = cases.run(_Tampered('tlm 0', lambda lines: unsupported), ('stream',), (), None, milestone=milestone)
        assert {result.outcome for result in results if result.name == 'tlm_stop'} == {outcome}, milestone
    assert cc_contract._milestone('M1') == 1 and cc_contract._milestone('m13') == 13
    for text in ('1', 'M', 'milestone 1', 'M1b'):
        try:
            cc_contract._milestone(text)
        except Exception as error:
            assert 'M<number>' in str(error), error
        else:
            raise AssertionError('%r read as a milestone' % text)


def test_preflight_refuses_what_it_cannot_see_on_the_ground() -> None:
    """Airborne or armed: whoami and health and nothing else. With a flag, an unreadable stage or arm state too."""

    def airborne(payload: dict) -> None:
        """whoami from a gliding board."""
        payload['stage'] = 'gliding'

    def armed(payload: dict) -> None:
        """health from an armed board."""
        payload['armed'] = True

    def no_armed(payload: dict) -> None:
        """health that does not say whether the board is armed."""
        del payload['armed']

    for request, edit, allowed, words in (('whoami', airborne, cases.ALLOW_FLAGS, 'gliding'),
                                          ('health', armed, cases.ALLOW_FLAGS, 'ARMED'),
                                          ('health', armed, (), 'ARMED'),
                                          ('health', no_armed, ('reboot',), 'unknown: armed')):
        refused = _Tampered(request, _payload_edit(edit))
        try:
            cases.run(refused, cases.TIERS, allowed, None)
        except cases.Refused as error:
            assert words in str(error), error
        else:
            raise AssertionError('the runner went on with a board whose %s read %s' % (request, edit.__name__))
        assert refused.sent == ['whoami', 'health'], refused.sent

    class _Garbled(_Tampered):
        """A board whose whoami and health both answer something that is no payload."""

        def exchange(self, line: str) -> list:
            """`ok` with nothing for whoami and health, plan B for the rest."""
            if line in ('whoami', 'health'):
                self.sent.append(line)
                return ['ok']
            return super().exchange(line)

    garbled = _Garbled()
    try:
        cases.run(garbled, cases.TIERS, ('stage',), None)
    except cases.Refused as error:
        assert 'unknown: stage, armed' in str(error), error
    else:
        raise AssertionError('a flag went out to a board whose stage nobody read')
    assert garbled.sent == ['whoami', 'health'], garbled.sent
    results = cases.run(_Garbled(), ('core',), (), None)  # read-only: the run goes on and reports the damage
    assert _failed(results) == {'whoami', 'health'}


def test_identity_is_checked_against_the_addressed_board() -> None:
    """Through the hub or the console the iam id must be the board the operator named."""
    results = {result.name: result for result in cases.run(_Tampered(), ('core',), (), 'TMS-7C')}
    assert results['whoami'].outcome == cases.FAIL and 'went to board TMS-7C' in results['whoami'].reason
    results = {result.name: result for result in cases.run(_Tampered(), ('core',), (), 'taster')}
    assert results['whoami'].outcome == cases.PASS


def test_config_source_gates_the_board_writes() -> None:
    """Only a board that booted its own file (active) or none (default) is re-saved; only `default` is reset."""

    def source(value: str) -> typing.Callable:
        """A whoami replacement reporting config_source `value`."""

        def edit(payload: dict) -> None:
            """whoami with the config source replaced."""
            payload['config_source'] = value

        return _payload_edit(edit)

    for value, resaved, reset in (('default', True, True), ('active', True, False),
                                  ('default(fallback: invalid board.config)', False, False),
                                  ('active(config 1, firmware 2 -- re-save to adopt)', False, False)):
        board = _Tampered('whoami', source(value))
        results = {result.name: result for result in cases.run(board, ('config',), ('config_write',), None)}
        assert any(line.startswith('set-config board ') and line != cc.build('set-config', ['board', '{}'])
                   for line in board.sent) is resaved, (value, board.sent)
        assert ('reset-config' in board.sent) is reset, (value, board.sent)
        if not resaved:
            assert 'config_source is %r' % value in results['set_config_board_resave'].reason
    _PLAN_B.exchange('reset-config')  # the `active` pretence left plan B a board.config: back to none


def test_reboot_needs_a_new_boot() -> None:
    """An `ok` to reboot passes only once a whoami from a new boot answers -- not on the old boot's last words."""
    rebooted = _PLAN_B.reboots
    racing = _Settling()  # no settle: the first whoami lands before plan B's reset fires
    with _patched(cases, _REBOOT_POLL_S=0.05):
        results = {result.name: result for result in cases.run(racing, ('system',), ('reboot',), None)}
    assert results['reboot'].outcome == cases.PASS, results['reboot']
    assert _PLAN_B.reboots == rebooted + 1
    after = racing.sent[racing.sent.index('reboot') + 1:]
    assert after.count('whoami') >= 2, 'the first whoami still reached the old boot, and must not have passed'

    never = _Settling('reboot', lambda lines: ['ok'], forward=False)  # answers ok, never resets
    with _patched(cases, _REBOOT_TIMEOUT_S=0.3, _REBOOT_POLL_S=0.05):
        results = {result.name: result for result in cases.run(never, ('system',), ('reboot',), None)}
    assert results['reboot'].outcome == cases.FAIL, results['reboot']
    assert 'never reset' in results['reboot'].reason and 'boot_id still' in results['reboot'].reason
    assert _PLAN_B.reboots == rebooted + 1

    def no_count(board: _Tampered) -> typing.Callable:
        """whoami with no boot count (an NVS failure), its uptime restarted once `reboot` went to `board`."""

        def edit(payload: dict) -> None:
            """boot_id null; uptime 0 after the reboot."""
            payload['boot_id'] = None
            if 'reboot' in board.sent:
                payload['uptime'] = 0

        return _payload_edit(edit)

    uncounted = _Settling()
    uncounted._request, uncounted._replace = 'whoami', no_count(uncounted)
    with _patched(cases, _REBOOT_TIMEOUT_S=5.0, _REBOOT_POLL_S=0.05):
        results = {result.name: result for result in cases.run(uncounted, ('system',), ('reboot',), None)}
    assert results['reboot'].outcome == cases.PASS, 'a restarted uptime proves the reset where no boot was counted'

    unreadable = _Tampered('whoami', lambda lines: ['iam'])
    results = {result.name: result for result in cases.run(unreadable, ('system',), ('reboot',), None)}
    assert results['reboot'].outcome == cases.SKIP and 'reboot' not in unreadable.sent, results['reboot']


def test_tee_proves_a_log_bridge() -> None:
    """A board that tees must drain a line from a window open across `probe mission`; a stub bridge fails."""
    results = {result.name: result for result in cases.run(_Teeing(['1 Mission :: probe: ...']), ('stream',), (),
                                                           None, milestone=1)}
    assert results['log_tee_trigger'].outcome == cases.PASS and results['log_drain_and_stop'].outcome == cases.PASS
    stub = cases.run(_Teeing([]), ('stream',), (), None)
    assert _failed(stub) == {'log_drain_and_stop'}
    assert all('drained no line' in result.reason for result in stub if result.outcome == cases.FAIL)
    unsupported = _Teeing([])
    unsupported._request, unsupported._replace = 'log 3000', lambda lines: [cc.build('err', ['unsupported', 'x'])]
    results = {result.name: result for result in cases.run(unsupported, ('stream',), (), None)}
    assert results['log_tee_trigger'].outcome == cases.SKIP and 'probe mission' not in unsupported.sent


def test_an_aborted_run_undoes_its_writes() -> None:
    """A transport error or Ctrl-C mid-run sends the undo of every write that went out, then reports Aborted."""
    before = _PLAN_B.exchange('get-config launch')

    def unplugged(lines: list) -> list:
        """The port vanishing under the runner."""
        raise RuntimeError('USB gone')

    def interrupted(lines: list) -> list:
        """The operator pressing Ctrl-C."""
        raise KeyboardInterrupt

    probe_written = _Tampered('get-config launch', unplugged, occurrence=2)  # right after the probe write
    try:
        cases.run(probe_written, ('config',), ('launch_write',), None)
    except cases.Aborted as error:
        assert 'RuntimeError' in str(error) and 'launch_id restore: ok' in str(error), error
    else:
        raise AssertionError('a transport error did not abort the run')
    assert _PLAN_B.exchange('get-config launch') == before, 'the probe launch id was left on the board'

    held = _Tampered('health', interrupted, occurrence=2)  # health_while_held: the board is held at gliding
    try:
        cases.run(held, ('control',), ('active', 'stage'), None)
    except cases.Aborted as error:
        assert 'KeyboardInterrupt' in str(error) and 'stage setting: ok' in str(error), error
    else:
        raise AssertionError('Ctrl-C did not abort the run')
    assert held.sent[-2:] == ['disarm', 'stage setting'], held.sent[-3:]
    assert json.loads(cc.parse(_PLAN_B.exchange('stage')[0]).args[0]) == {'stage': 'setting', 'manual': False}

    def port_failed(lines: list) -> list:
        """The serial port failing (pyserial's SerialException is an OSError)."""
        raise OSError(5, 'Input/output error')

    results = cases.run(_Tampered('ping', port_failed), ('core',), (), None)  # no abort: no answer, and go on
    assert _failed(results) == {'ping', 'link_survives_errors'}
    assert all('no answer' in result.reason for result in results if result.outcome == cases.FAIL)

    transport = cc_contract._transport
    cc_contract._transport = lambda board, serial_port, hub_url: _Tampered('get-config launch', unplugged, 2)
    errors = io.StringIO()
    try:
        with contextlib.redirect_stderr(errors), contextlib.redirect_stdout(io.StringIO()):
            status = cc_contract.main(['taster', '--tier', 'config', '--allow-launch-write'])
    finally:
        cc_contract._transport = transport
    assert status == 2 and 'the run stopped' in errors.getvalue(), (status, errors.getvalue())
    assert _PLAN_B.exchange('get-config launch') == before


def test_diff_masks_only_what_cannot_matter() -> None:
    """--diff: vitals and Rust-only keys never differ, KNOWN fields are listed apart, a real difference shows."""

    def rust_whoami(payload: dict) -> None:
        """A Rust board's whoami: its own firmware version and boot."""
        payload['firmware_version'] = 'avionics-1'
        payload['uptime'] = 5
        payload['boot_id'] = 9

    def rust_health(payload: dict) -> None:
        """A Rust board's health: the PSRAM, internal heap and SD figures plan B lacks."""
        payload.update(psram_free=1 << 20, internal_free=1 << 16, sd_free_mb=None, temp=38.0)

    def other_config(payload: dict) -> None:
        """A board that runs another config."""
        payload['config_id'] = '0123456789ab'

    reference = cases.run(_PLAN_B, ('core',), (), None)
    board = cases.run(_Tampered('whoami', _payload_edit(rust_whoami)), ('core',), (), None)
    assert normalize.diff(board, reference) == [], normalize.diff(board, reference)
    assert normalize.known(board, reference) == ['core/whoami firmware_version: board "avionics-1" / fake "dev"']
    board = cases.run(_Tampered('health', _payload_edit(rust_health)), ('core',), (), None)
    assert normalize.diff(board, reference) == [], 'a Rust-only health key is no difference'
    board = cases.run(_Tampered('whoami', _payload_edit(other_config)), ('core',), (), None)
    found = dict(normalize.diff(board, reference))
    assert list(found) == ['core/whoami'] and any('config_id' in line for line in found['core/whoami']), found
    assert normalize.diff(reference, cases.run(_PLAN_B, ('core',), (), None)) == []
    teeing = cases.run(_Teeing(['1 Mission :: probe: ...']), ('stream',), (), None)
    assert normalize.diff(teeing, cases.run(_PLAN_B, ('stream',), (), None)) == [], 'what a window carried'
    assert normalize.differences(1.764, 1.7639999) == [], 'f32 text is the same value'
    assert normalize.differences(1, 1.0), 'an int and a float are different JSON'
    assert normalize.differences(['b', 'a'], ['a', 'b']) == [], 'a list of names is an inventory'
    assert normalize.mask({'health': {'uptime': 1, 'stage': 'setting', 'firmware_version': 'x'}}) == {
        'health': {'stage': 'setting'}}
    assert normalize.mask({'lines': ['a'], 'dropped': 0}) == {'lines': '<list of text>', 'dropped': 0}
    assert normalize.mask({'lines': [1]}) != normalize.mask({'lines': []}), 'a non-text line is a different shape'
    error = normalize.reply(cc.build('err', ['badargs', "Error('Incorrect padding')"]), (1,))
    assert error == {'status': 'err', 'arguments': ['badargs', normalize.FREE]}
    outcome = cases.run(_Tampered('ping', lambda lines: [cc.build('err', ['unsupported', 'x'])]), ('core',), (), None)
    assert any(line.startswith('outcome: board FAIL / fake pass') for line in dict(normalize.diff(outcome, reference))[
        'core/ping'])


def test_hub_request_carries_the_line_verbatim() -> None:
    """/api/cmd rebuilds the line from {command, params}: only a line that survives that goes through the hub."""
    assert transports.hub_request('ping') == ('ping', [])
    assert transports.hub_request('PING') == ('PING', [])
    launch = cc.build('set-config', ['launch', '{"launch_id": "x"}'])
    assert transports.hub_request(launch) == ('set-config', ['launch', '{"launch_id": "x"}'])
    for line in ('', '   ', 'inspect base64:QQ', 'inspect base64:/w==', 'inspect  mission'):
        try:
            transports.hub_request(line)
        except transports.Unreachable:
            continue
        raise AssertionError('the hub cannot carry %r verbatim' % line)
    iam = transports.hub_reply({'board': 'taster', 'status': 'iam', 'args': ['taster', '{"stage": "setting"}']})
    message = cc.parse(iam)
    assert message.command == 'iam' and message.args == ['taster', '{"stage": "setting"}'], iam
    assert transports.hub_reply({'board': 'taster', 'status': 'pong', 'args': []}) == 'pong'


class _HubStub:
    """
    A hub web bridge on localhost (src/control/web.py's two routes the runner uses), answering what a test sets.

    GET /api/boards serves `rows` one list per request (the last one repeats); POST /api/cmd answers
    `answers[command]` as (HTTP status, JSON body or text), or the hub's 404 for a board it does not know.
    """

    def __init__(self) -> None:
        """Serve on a free port, in a daemon thread."""
        self.rows: list = []
        self.answers: dict = {}
        stub = self

        class _Handler(http.server.BaseHTTPRequestHandler):
            """One request: the scripted answer."""

            def do_GET(self) -> None:
                """/api/boards: the next scripted row list."""
                rows = stub.rows.pop(0) if len(stub.rows) > 1 else stub.rows[0]
                self._send(200, rows)

            def do_POST(self) -> None:
                """/api/cmd: the answer scripted for the command."""
                request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                self._send(*stub.answers.get(request['command'], (404, {'error': 'no online board'})))

            def _send(self, status: int, body: object) -> None:
                """Write one HTTP answer: JSON unless the body is text."""
                data = (body if isinstance(body, str) else json.dumps(body)).encode()
                self.send_response(status)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *arguments: object) -> None:
                """Quiet: the test output is the runner's."""

        self._server: http.server.ThreadingHTTPServer = http.server.ThreadingHTTPServer(('127.0.0.1', 0), _Handler)
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.url: str = 'http://127.0.0.1:%d' % self._server.server_address[1]

    def close(self) -> None:
        """Stop serving and free the port."""
        self._server.shutdown()
        self._server.server_close()


def _row(uptime: int, **fields: object) -> dict:
    """A dashboard row (server.Server.board_rows) of an online, healthy board in setting with a set clock."""
    row = {'id': 'taster', 'online': True, 'health_age': 0.5, 'stale': False, 'stage': 'setting',
           'layout': 'undecided', 'version': 'avionics-1', 'config_id': '95e1af1d7b1d', 'uptime': uptime,
           'clock': '2026-10-06T12:00:00', 'position': None, 'launchpad': None, 'launchpad_set': False,
           'site': None, 'armed': False, 'agl': None, 'flight': None, 'degraded': [], 'imu_calibration': None,
           'calibration': None, 'temp': 38.0, 'mem_free': 1 << 20}
    row.update(fields)
    return row


def test_hub_transport_over_http() -> None:
    """The hub's HTTP errors are no answer, an unknown or offline board is unavailable, a reply is rebuilt."""
    stub = _HubStub()
    try:
        stub.rows = [[_row(1000)]]
        hub = transports.Hub('taster', stub.url)
        stub.answers['ping'] = (200, {'board': 'taster', 'status': 'pong', 'args': []})
        assert hub.exchange('ping') == ['pong']
        for status, body, reason in ((404, {'error': 'no online board taster'}, 'hub HTTP 404: no online board'),
                                     (502, {'board': 'taster', 'error': 'offline'}, 'hub HTTP 502: offline'),
                                     (500, 'Internal Server Error', 'hub HTTP 500: Internal Server Error')):
            stub.answers['ping'] = (status, body)
            try:
                hub.exchange('ping')
            except transports.NoAnswer as error:
                assert reason in str(error), error
            else:
                raise AssertionError('HTTP %d read as an answer' % status)
        for board_id, rows in (('ghost', [_row(1000)]), ('taster', [_row(1000, online=False)])):
            stub.rows = [rows]
            try:
                transports.Hub(board_id, stub.url)
            except transports.Unavailable as error:
                assert 'no online board %s' % board_id in str(error), error
            else:
                raise AssertionError('the hub transport took board %s' % board_id)
    finally:
        stub.close()
    try:
        transports.Hub('taster', stub.url)  # the port is closed now
    except transports.Unavailable as error:
        assert 'cannot reach the hub' in str(error), error
    else:
        raise AssertionError('a hub that is gone was reached')


def test_hub_tier_reads_the_dashboard_row() -> None:
    """The hub tier: the row filled, online and fresh across the beats, a set clock -- and each failure named."""
    whoami = {'mcu': 'esp32p4', 'firmware_version': 'avionics-1', 'config_id': '95e1af1d7b1d',
              'config_source': 'default', 'stage': 'setting', 'uptime': 1000, 'boot_id': 3, 'session': '000003',
              'epoch': 946684810}
    health = json.loads(cc.parse(_PLAN_B.exchange('health')[0]).args[0])
    stub = _HubStub()
    stub.answers = {'whoami': (200, {'board': 'taster', 'status': 'iam', 'args': ['taster', json.dumps(whoami)]}),
                    'health': (200, {'board': 'taster', 'status': 'ok', 'args': [json.dumps(health)]})}
    scenarios = (
        ([_row(1000), _row(3000), _row(5000)], set(), ''),
        ([_row(1000), _row(3000, online=False)], {'hub_online_across_beats'}, 'offline at beat 2/3'),
        ([_row(1000), _row(3000, stale=True, health_age=7.0)], {'hub_online_across_beats'}, 'stale at beat 2/3'),
        ([_row(1000), _row(1000), _row(1000)], {'hub_online_across_beats'}, 'the uptime went 1000 -> 1000'),
        ([_row(1000, version=None), _row(3000, version=None), _row(5000, version=None)], {'hub_row_filled'},
         "no ['version']"),
        ([_row(1000, clock='2000-01-01T00:00:10'), _row(3000, clock='2000-01-01T00:00:12'),
          _row(5000, clock='2000-01-01T00:00:14')], {'hub_clock_synced'}, 'its whoami looked syncable'),
    )
    try:
        for rows, names, reason in scenarios:
            stub.rows = [[row] for row in [rows[0]] + rows]  # the first read is the transport's own check
            with _patched(cases, _HUB_BEAT_S=0.0):
                results = cases.run(transports.Hub('taster', stub.url), ('hub',), (), 'taster')
            assert [result.tier for result in results] == ['hub'] * 3, results
            failed = [result for result in results if result.outcome == cases.FAIL]
            assert {result.name for result in failed} == names, [(result.name, result.reason) for result in failed]
            assert all(reason in result.reason for result in failed), [result.reason for result in failed]
        whoami['epoch'] = 946684810.0  # a float epoch: the hub never syncs it
        stub.answers['whoami'] = (200, {'board': 'taster', 'status': 'iam', 'args': ['taster', json.dumps(whoami)]})
        stub.rows = [[_row(1000, clock='2000-01-01T00:00:10')]]
        with _patched(cases, _HUB_BEAT_S=0.0):
            results = {result.name: result for result in cases.run(transports.Hub('taster', stub.url), ('hub',), (),
                                                                   'taster', beats=1)}
        assert 'whoami epoch is float' in results['hub_clock_synced'].reason, results['hub_clock_synced']
        stub.rows = [[_row(1000, stage='done', clock='2000-01-01T00:00:10')]]
        with _patched(cases, _HUB_BEAT_S=0.0):
            results = {result.name: result for result in cases.run(transports.Hub('taster', stub.url), ('hub',), (),
                                                                   'taster', beats=1)}
        assert results['hub_clock_synced'].outcome == cases.SKIP, 'the hub syncs in setting only'
    finally:
        stub.close()
    console = _console([])
    assert [result.outcome for result in cases._hub_checks(console, {}, 3)] == [cases.SKIP], 'no hub, no hub tier'


class _ScriptedLink:
    """A console port that plays back what a board printed, chunk by chunk, and records what was written."""

    def __init__(self, chunks: list) -> None:
        """
        Play `chunks` back, one per read.

        Args:
            chunks - the bytes the board printed, one read's worth each.
        """
        self.chunks: list = list(chunks)
        self.written: list = []
        self.flushed: int = 0

    def reset_input_buffer(self) -> None:
        """Count the discards of pending input (one per request)."""
        self.flushed += 1

    def write(self, data: bytes) -> None:
        """Record what the runner sent."""
        self.written.append(data)

    def read(self, size: int) -> bytes:
        """The next scripted chunk, or nothing after the read timeout."""
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.01)  # the real port blocks for its read timeout
        return b''


def _console(chunks: list) -> transports.Serial:
    """A Serial transport over a scripted port (the real constructor opens a device)."""
    console = transports.Serial.__new__(transports.Serial)
    console.name = 'serial (scripted)'
    console._link = _ScriptedLink(chunks)
    return console


def test_serial_console_filters_logs_and_counts_replies() -> None:
    """ESP-IDF log and ROM boot lines are not replies; a second reply is caught; an empty line gets nothing."""
    assert transports.is_log_line('I (1234) wifi: connected') and transports.is_log_line('E (5) sd: gone')
    assert transports.is_log_line('\x1b[0;33mW (77) link: slow\x1b[0m')
    for line in ('ESP-ROM:esp32p4-eco2-20240710', 'Build:Jul 10 2024', 'Core0 Saved PC:0x4ff0b2c4',
                 'rst:0xc (SW_CPU_RESET),boot:0x30f (SPI_FAST_FLASH_BOOT)', 'SPI mode:DIO, clock div:1',
                 'load:0x4ff33ce0,len:0x164c', 'entry 0x4ff29ae0'):
        assert transports.is_log_line(line), line
    assert not transports.is_log_line('iam taster base64:e30=') and not transports.is_log_line('err badargs x')
    console = _console([b'I (12) wifi: up\r\n\x1b[0;32mI (13) x: y\x1b[0m\r\n', b'po', b'ng\r\n'])
    assert console.exchange('ping') == ['pong']
    assert console._link.written == [b'ping\n'] and console._link.flushed == 1
    banner = b'ESP-ROM:esp32p4-eco2-20240710\r\nrst:0xc (SW_CPU_RESET),boot:0x30f (SPI_FAST_FLASH_BOOT)\r\n'
    assert _console([banner, b'entry 0x4ff29ae0\r\nI (25) boot: ESP-IDF\r\n', b'pong\r\n']).exchange('ping') == ['pong']
    assert _console([b'pong\r\npong\r\n']).exchange('ping') == ['pong', 'pong']
    assert _console([]).exchange('') == []
    assert _console([b'W (1) a: b\r\n']).exchange('') == [], 'a log line is no reply to an empty line'
    with _patched(transports.board, timeout_for=lambda line: 0.2):
        try:
            _console([b'I (1) only: logs\r\n']).exchange('ping')
        except transports.NoAnswer:
            pass
        else:
            raise AssertionError('a console that prints only logs did not answer')


def test_console_opens_without_resetting_the_board() -> None:
    """DTR and RTS are both asserted BEFORE the port opens: releasing them after passes through a reset."""
    events = []

    class _Port:
        """pyserial's Serial, recording every setting and the open in order."""

        def __setattr__(self, name: str, value: object) -> None:
            """Record the setting, then make it."""
            events.append((name, value))
            object.__setattr__(self, name, value)

        def open(self) -> None:
            """Record the open."""
            events.append(('open', None))

    stub = types.ModuleType('serial')
    stub.Serial = _Port
    stub.SerialException = OSError
    real = sys.modules.get('serial')
    sys.modules['serial'] = stub
    try:
        transports.open_console('/dev/scripted')
    finally:
        if real is not None:
            sys.modules['serial'] = real
        else:
            del sys.modules['serial']
    opened = events.index(('open', None))
    assert ('dtr', True) in events[:opened] and ('rts', True) in events[:opened], events
    assert not any(name in ('dtr', 'rts') and value is False for name, value in events), events


def test_command_line() -> None:
    """tools/cc_contract.py fake: the printed report, --json, --milestone, --diff fake against itself, exit status."""
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        status = cc_contract.main(['fake', '--tier', 'core', '--tier', 'stream', '--milestone', 'M1', '--json'])
    assert status == 0, output.getvalue()
    report = json.loads(output.getvalue())
    assert report['violations'] == 0 and list(report['tiers']) == ['core', 'stream'], report['tiers']
    assert report['milestone'] == 1
    assert {case['name'] for case in report['cases'] if case['outcome'] != cases.PASS} == {'log_tee_trigger'}
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        status = cc_contract.main(['fake', '--tier', 'control', '--tier', 'diagnostics', '--diff', 'fake'])
    text = output.getvalue()
    assert status == 0 and 'control     PASS' in text and '0 case(s) differ' in text, text
    assert 'diagnostics PASS' in text and 'known differences' in text, text
    assert 'needs --allow-stage' in text, text
    errors = io.StringIO()
    with contextlib.redirect_stderr(errors):
        try:
            cc_contract.main(['fake', '--milestone', 'one'])
        except SystemExit as error:
            assert error.code == 2, error.code
        else:
            raise AssertionError('a milestone that is not M<number> was taken')
    assert 'M<number>' in errors.getvalue(), errors.getvalue()
    with contextlib.redirect_stderr(io.StringIO()):
        try:
            cc_contract.main(['fake', '--beats', '0'])
        except SystemExit as error:
            assert error.code == 2, error.code
        else:
            raise AssertionError('a hub tier of no heartbeat was taken')
    failing = [cases.Result('core', 'ping', 'ping', cases.FAIL, 'expected pong')]
    assert cc_contract.violations(failing) == 1 and cc_contract.summary(failing)['core']['verdict'] == cases.FAIL


test_plan_b_keeps_the_contract()
test_plan_b_stays_in_its_own_process()
test_round_trips_put_the_board_back()
test_nothing_active_without_its_flag()
test_violations_are_caught()
test_invariants_beyond_the_core_are_checked()
test_the_m6_parts_are_held_to_plan_b_shapes()
test_the_hold_needs_a_disarmed_board()
test_not_yet_is_reported_not_failed()
test_a_milestone_makes_its_cases_mandatory()
test_preflight_refuses_what_it_cannot_see_on_the_ground()
test_identity_is_checked_against_the_addressed_board()
test_config_source_gates_the_board_writes()
test_reboot_needs_a_new_boot()
test_tee_proves_a_log_bridge()
test_an_aborted_run_undoes_its_writes()
test_diff_masks_only_what_cannot_matter()
test_hub_request_carries_the_line_verbatim()
test_hub_transport_over_http()
test_hub_tier_reads_the_dashboard_row()
test_serial_console_filters_logs_and_counts_replies()
test_console_opens_without_resetting_the_board()
test_command_line()
_PLAN_B.close()
print('ok: cc contract -- plan B passes every case of every tier in its own process (board-only cases skip), '
      'round trips restore, nothing active without its flag, violations caught (status, keys, types, empty line, '
      'two replies, codes, every invariant), the M6 parts, the hold needs a disarmed board, preflight refusal, iam id, '
      'config_source gating, the reboot proof, the tee, milestones, aborted runs undo, --diff masking + known '
      'differences, hub line fidelity + HTTP errors + the hub tier, console log/ROM filter + no-reset open, '
      'the command line')
