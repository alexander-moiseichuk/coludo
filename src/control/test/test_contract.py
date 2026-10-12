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
undoes its writes. A not-yet answer FAILs once the claimed milestone owes the case. The M7 gate: an arm while
held is refused for the hold (and disarmed if it was not), verify's readiness names only plan B's checks -- the
set read from plan B's own source -- and the bluetooth stub inspects in plan B's shape. The M9 parts: the SEN0697
snapshots, the attitude filter, the sequencer and the databoard in their shapes, the BMP581's ground round trip,
the bus diagnostics refused armed and held, and the armed bustune only back to the configured clock -- each broken
answer failing its own case, an arm that is refused skipping the armed checks.

The hub and serial transports are tested without a hub or a board: the line a hub request must carry, the
reply it rebuilds, a stub hub's HTTP errors and its dashboard rows (the hub tier), the console's log and ROM
banner filter and its one-line-in-one-line-out watch, and a console open that cannot reset the board.
"""

import ast
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
# the cases plan B on CPython cannot run: no tee, no hub, and no device set up (the M6 parts' and bluetooth's cases)
_M6_BOARD_ONLY: set = {'log_tee_trigger', 'hub', 'inspect_imu', 'inspect_barometer', 'inspect_separation',
                       'inspect_bluetooth', 'update_barometer_ground', 'update_barometer_text_ground',
                       'update_barometer_rezero', 'calibrate_barometer', 'calibrate_imu', 'probe_devices',
                       'bustune_imu_at_its_clock'}
# the M9 cases plan B on CPython runs: health, and the databoard it lists once a health read registered it
_M9_ON_PLAN_B: set = {'health_names_its_disagreements', 'stats_databoard', 'inspect_databoard',
                      'update_databoard_nothing', 'update_databoard_read_only'}
# the M9 cases it cannot: no SEN0697 part, attitude filter or sequencer, and no part on a bus for the (Rust-only) bus
# diagnostics
_M9_BOARD_ONLY: set = {case.name for case in cases.CASES if case.milestone == 9} - _M9_ON_PLAN_B
# the M10-M21 cases: every part's (none set up), and the GCL commands, which plan B has none of (it answers help
# badcmd); health's imu_calibration runs (absent there)
_M10_ON_PLAN_B: set = {'health_imu_calibration'}
_GCL_BOARD_ONLY: set = {'entry_show', 'df', 'ls', 'ring', 'video_stats', 'ota_status'}
_M10_BOARD_ONLY: set = {case.name for case in cases.CASES if 10 <= case.milestone <= 21} - _M10_ON_PLAN_B
_BOARD_ONLY: set = _M6_BOARD_ONLY | _M9_BOARD_ONLY | _M10_BOARD_ONLY | _GCL_BOARD_ONLY
_M6_FITTED: tuple = ('baro_icp10111', 'bluetooth', 'imu_lsm6dso32', 'separation')  # the M6 parts and the BLE stub
_M9_FITTED: tuple = ('attitude', 'baro_bmp581', 'databoard', 'imu_bmi323', 'mag_bmm350', 'sequencer')
_BMP581_ALTITUDE: float = 22.1234  # the fitted BMP581's altitude, m AMSL
_BMP581_GROUND: float = 22.1  # and the ground it boots with: elevation 0.0234 m
_IMU_CLOCK: int = 5000000  # the default config's SPI 1 baud: the IMU bus's configured clock
# the bus diagnostics' answers to the bare line while disarmed in SETTING
_BUS_USAGE: dict = {line: usage for line, _part, usage in cases._BUS_DIAGNOSTICS}
# the bluetooth stub's snapshot in plan B's no-BLE shape (drivers/bluetooth.py inspect): radio off, no BLE
_BLUETOOTH_STUB: dict = {'name': 'bluetooth', 'ok': True, 'healthy': True, 'radio': False, 'active': None}
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
    """
    The healthy fitted parts' answers, by request line: the M6 parts' snapshots, the ICP's update round trip,
    calibrate and probe, and the bluetooth stub's snapshot.
    """
    ok = lambda payload: [cc.build('ok', [json.dumps(payload)])]  # noqa: E731
    update = lambda properties: cc.build('update', ['baro_icp10111', json.dumps(properties)])  # noqa: E731
    return {
        'inspect imu_lsm6dso32': ok({'name': 'imu_lsm6dso32', 'ok': True, 'healthy': True, 'interrupt': True,
                                     'interrupt_silent': False, 'accel_g': [0.01, -0.02, 1.01],
                                     'rate_dps': [0.1, 0.0, -0.1]}),
        'inspect baro_icp10111': ok({'name': 'baro_icp10111', 'ok': True, 'healthy': True, 'altitude_m': 42.1,
                                     'temperature_c': None, 'pressure_pa': 100810.5, 'elevation_m': 0.2,
                                     'change_m_s': 0.004}),
        'inspect separation': ok({'name': 'separation', 'ok': True, 'healthy': True, 'separated': False}),
        'inspect bluetooth': ok(_BLUETOOTH_STUB),
        update({'ground': 100}): ok({'changed': ['ground']}),
        update({'ground': 'high'}): [cc.build('err', ['refused', "could not convert string to float: 'high'"])],
        update({'rezero': True}): ok({'changed': ['ground']}),
        'calibrate baro_icp10111': ok({'baro_icp10111': None}),
        'calibrate imu_lsm6dso32': ok({'imu_lsm6dso32': None}),
        'probe imu_lsm6dso32': ok({'imu_lsm6dso32': None}),
        'probe baro_icp10111': ok({'baro_icp10111': None}),
        'probe separation': ok({'separation': None}),
        'bustune spi 1 5000000': ok({'kind': 'spi', 'id': '1', 'freq': 5000000, 'devices': {'imu_lsm6dso32': 'ok'},
                                     'all_ok': True}),
    }


class _Fitted(_Tampered):
    """
    Plan B with the parts fitted, which plan B on CPython never sets up: `objects` lists `fitted` (by default the
    default config's imu_lsm6dso32, baro_icp10111, separation and bluetooth), and the requests in `answers` get
    their answer instead of plan B's: reply lines, an exception to raise, or a replacement of plan B's reply lines.
    """

    def __init__(self, answers: dict, fitted: tuple = _M6_FITTED) -> None:
        """Answer through plan B, the fitted parts by `answers`."""
        super().__init__()
        self._answers: dict = answers
        self._fitted: tuple = fitted

    def exchange(self, line: str) -> list:
        """Record the line; a part's request gets its answer, `objects` lists the parts, the rest is plan B's."""
        self.sent.append(line)
        if line in self._answers:
            answer = self._answers[line]
            if isinstance(answer, BaseException):
                raise answer
            return answer(_PLAN_B.exchange(line)) if callable(answer) else answer
        lines = _PLAN_B.exchange(line)
        if line == 'objects':
            names = json.loads(cc.parse(lines[0]).args[0])
            return [cc.build('ok', [json.dumps(sorted(set(names) | set(self._fitted)))])]  # plan B lists databoard
        return lines


def _ok(payload: object) -> list:
    """The reply lines `ok <payload as JSON>`."""
    return [cc.build('ok', [json.dumps(payload)])]


def _refused(code: str, message: str) -> list:
    """The reply lines `err <code> <message>`."""
    return [cc.build('err', [code, message])]


def _sen0697_answers() -> dict:
    """
    The healthy M6 and M9 parts' answers, by request line: the SEN0697's BMI323 and BMM350, the attitude filter, the
    sequencer and the databoard as the Rust board answers them (the taster's, 2026-10-08), probes and calibrations
    that pass, and a health that reports the taster's two IMUs apart. The BMP581 and the bus diagnostics answer by
    state (_Sen0697).
    """

    def disagree(payload: dict) -> None:
        """health with the taster's breadboard IMUs 94 degrees apart."""
        payload['degraded'].append('sources-disagree: accel imu_lsm6dso32/imu_bmi323 94.0 deg')

    answers = _fitted_answers()
    answers.update({
        'inspect imu_bmi323': _ok({'name': 'imu_bmi323', 'ok': True, 'healthy': True, 'addr': '0x69',
                                   'interrupt': True, 'interrupt_silent': True, 'accel_g': [-0.71, 0.008, 0.69],
                                   'rate_dps': [0.061, 0.061, -0.061], 'irq_runs': 0, 'odr_hz': 99.68,
                                   'lapses': {'accel': 0, 'rate': 0}, 'raced': [0, 0], 'identical': [45, 266]}),
        'inspect mag_bmm350': _ok({'name': 'mag_bmm350', 'ok': True, 'healthy': True, 'addr': '0x15',
                                   'mag_counts': [15557.0, 3368.0, 8174.0], 'calibrated': False,
                                   'span': [15441, 15608, 3277, 3436], 'samples': 3825, 'sectors': 0}),
        'inspect attitude': _ok({'name': 'attitude', 'ok': True, 'healthy': True, 'blind': False,
                                 'free_running': True, 'heading': '150.71', 'mag_known': False, 'mag_offset': '-6.49',
                                 'pitch': '30.62', 'roll': '-59.44', 'seeded': False}),
        'stats databoard': _ok({'accel': ['imu_bmi323', 'imu_lsm6dso32'], 'agl': ['laser_agl_l1x'],
                                'altitude': ['baro_bmp581', 'baro_icp10111'], 'attitude': ['attitude']}),
        'inspect databoard': _ok({
            'accel': {'value': [0.52, -0.75, 0.45], 'source': 'imu_lsm6dso32', 'age_ms': 5, 'offsets': {}},
            'agl': {'value': None, 'source': None, 'age_ms': None, 'offsets': {}},
            'altitude': {'value': 32.41, 'source': 'baro_icp10111', 'age_ms': 114, 'offsets': {'baro_bmp581': 10.16}},
            'attitude': {'value': [150.7, -59.4, 30.6], 'source': 'attitude', 'age_ms': 15, 'offsets': {}}}),
        cc.build('update', ['databoard', '{}']): _ok({'changed': []}),
        cc.build('update', ['databoard', json.dumps({'altitude': 0})]): _ok({'changed': []}),
        'inspect sequencer': _ok({'name': 'sequencer', 'ok': True, 'healthy': True, 'ignored': [], 'veto_edges': 0,
                                  'sources': 'accel [imu_lsm6dso32, imu_bmi323], barometers [baro_icp10111, '
                                             'baro_bmp581], gnss []'}),
        'health': _payload_edit(disagree),
        'probe attitude': _ok({'attitude': None}),
        'probe imu_bmi323': _ok({'imu_bmi323': None}),
        'probe baro_bmp581': _ok({'baro_bmp581': None}),
        'probe mag_bmm350': _ok({'mag_bmm350': None}),
        'calibrate baro_bmp581': _ok({'baro_bmp581': None}),
        'calibrate mag_bmm350': _ok({'mag_bmm350': 'no field change seen (x span 188, y span 178) -- is the '
                                                   'magnetometer alive?'}),
    })
    return answers


class _Sen0697(_Fitted):
    """
    Plan B with the M6 and the M9 parts fitted, answering as the Rust board does. Two answer by state: the BMP581
    reads against the ground its updates set, and a bus diagnostic (and a rung off the IMU bus's clock) is refused
    while held or armed, and wants its argument otherwise. An entry in `answers` wins over both.
    """

    def __init__(self, answers: dict) -> None:
        """Answer through plan B, the fitted parts by `answers`, the BMP581 on its boot ground, disarmed, unheld."""
        super().__init__(answers, _M6_FITTED + _M9_FITTED)
        self.ground: float = _BMP581_GROUND
        self._armed: bool = False
        self._held: bool = False

    def exchange(self, line: str) -> list:
        """The answer (`answers`, else by state, else plan B's); the arm and the hold kept as they were answered."""
        lines = None if line in self._answers else self._by_state(line)
        if lines is None:
            lines = super().exchange(line)
        else:
            self.sent.append(line)
        if line == 'arm':
            self._armed = lines[0].startswith('ok')
        elif line == 'disarm':
            self._armed = False
        elif line in ('stage gliding', 'stage setting'):
            self._held = line == 'stage gliding'
        return lines

    def _by_state(self, line: str) -> list | None:
        """The BMP581's, the bus diagnostics' and the off-clock rung's answers; None for any other line."""
        message = cc.parse(line)
        if line == 'inspect baro_bmp581':
            return _ok({'name': 'baro_bmp581', 'ok': True, 'healthy': True, 'addr': '0x47',
                        'altitude_m': _BMP581_ALTITUDE, 'temperature_c': 27.65, 'pressure_pa': 101059.56,
                        'elevation_m': _BMP581_ALTITUDE - self.ground, 'change_m_s': 0.083, 'torn': 490,
                        'implausible': 0})
        if message.command == 'update' and message.args[:1] == ['baro_bmp581']:
            properties = json.loads(message.args[1])
            if 'ground' in properties:
                if type(properties['ground']) not in (int, float):
                    return _refused('refused', 'ground must be a number')
                self.ground = properties['ground']
                return _ok({'changed': ['ground']})
            if properties.get('rezero'):
                self.ground = _BMP581_ALTITUDE
                return _ok({'changed': ['ground']})
            return _ok({'changed': []})
        if line in _BUS_USAGE:
            if self._held:
                return _refused('unsafe', cases._REFUSED_OUTSIDE_SETTING)
            return _refused('unsafe', cases._REFUSED_ARMED) if self._armed else _refused('badargs', _BUS_USAGE[line])
        if line == 'bustune spi 1 %d' % (_IMU_CLOCK // 2):
            if self._armed:
                return _refused('unsafe', cases._BUSTUNE_ONLY_BACK % _IMU_CLOCK)
            return _ok({'kind': 'spi', 'id': '1', 'freq': _IMU_CLOCK // 2, 'devices': {'imu_lsm6dso32': 'ok'},
                        'all_ok': True})
        return None


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
    The fitted parts' cases (the M6 parts', and the bluetooth stub's) pass on healthy answers, and each broken M6
    answer fails exactly its own case: a snapshot short of a key or naming another part, a ground write that changed
    nothing, a calibrate or a probe that failed. A board without the parts skips them; a run that stops after the
    ground write re-zeroes on its way out.
    """
    tiers, allowed = ('config', 'inspect', 'diagnostics'), ('active',)
    results = cases.run(_Fitted(_fitted_answers()), tiers, allowed, None, milestone=7)
    assert not _failed(results), [(result.name, result.reason) for result in results if result.outcome == cases.FAIL]
    fitted = {result.name: result.outcome for result in results if result.name.split('[')[0] in _M6_BOARD_ONLY}
    assert set(fitted.values()) == {cases.PASS}, fitted
    assert {name.split('[')[0] for name in fitted} == _M6_BOARD_ONLY - {'log_tee_trigger', 'hub'}, fitted
    assert len([name for name in fitted if name.startswith('probe_devices[')]) == 3, fitted

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
                                          'temperature_c': None, 'pressure_pa': None, 'elevation_m': None,
                                          'change_m_s': None}),
         {'inspect_barometer', 'inspect_barometer_change'}, "inspect baro_icp10111 names 'baro'"),
        (broken(update({'ground': 100}), {'changed': []}), {'update_barometer_ground'}, 'expected [ground]'),
        (broken(update({'ground': 'high'}), {'changed': ['ground']}), {'update_barometer_text_ground'}, ''),
        (broken('calibrate baro_icp10111', {'baro_icp10111': 'ground zero: no fresh altitude'}),
         {'calibrate_barometer'}, 'expected {\'baro_icp10111\': null}'),
        (broken('probe separation', {'separation': 'separation -- pin 33 reads nothing'}),
         {'probe_devices[probe separation]'}, 'expected {\'separation\': null}'),
        (broken('bustune spi 1 5000000', {'kind': 'spi', 'id': '1', 'freq': 5000000,
                                          'devices': {'imu_lsm6dso32': 'lsm6dso32 -- no WHO_AM_I'}, 'all_ok': False}),
         {'bustune_imu_at_its_clock'}, 'the configured clock failed'),
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


def _plan_b_readiness_checks() -> set:
    """
    Every readiness key plan B can report, read from its own source (src/glider/cc_client.py): the `problems[<key>]`
    that `_readiness` sets, and the `readiness[<key>]` that verify adds.
    """
    with open(os.path.join(_ROOT, 'src', 'glider', 'cc_client.py')) as source:
        tree = ast.parse(source.read())
    owners = {'_readiness': 'problems', 'verify': 'readiness'}  # function -> the map it fills with readiness
    keys = set()
    for function in ast.walk(tree):
        if not isinstance(function, (ast.FunctionDef, ast.AsyncFunctionDef)) or function.name not in owners:
            continue
        for node in ast.walk(function):
            if (isinstance(node, ast.Subscript) and isinstance(node.ctx, ast.Store) and isinstance(node.value, ast.Name)
                    and node.value.id == owners[function.name] and isinstance(node.slice, ast.Constant)):
                keys.add(node.slice.value)
    return keys


def test_the_m7_gate_is_held_to_plan_b() -> None:
    """
    The M7 cases pass on plan B and fail on each broken answer, exactly their own case: an arm while held that arms,
    lacks `stage`, gives another stage reason or a non-text problem; a readiness key plan B never reports; a
    bluetooth snapshot with a mistyped or missing radio state. An arm while held needs both flags, and whatever it
    answers a disarm follows before the board returns to the ground -- in the undo too, when the run stops.
    """
    assert _plan_b_readiness_checks() == set(cases._READINESS_CHECKS), _plan_b_readiness_checks()
    held, allowed = _Tampered(), ('stage', 'active')
    results = {result.name: result for result in cases.run(held, ('control',), allowed, None, milestone=7)}
    assert not _failed(results.values()), [(result.name, result.reason) for result in results.values()]
    assert results['arm_while_held'].outcome == cases.PASS and results['disarm_after_held_arm'].outcome == cases.PASS
    after_hold = held.sent[held.sent.index('stage gliding'):]
    assert after_hold.index('arm') + 1 == after_hold.index('disarm') < after_hold.index('stage setting'), after_hold

    for flags, missing, arms in ((('stage',), '--allow-active', 0), (('active',), '--allow-stage', 1)):
        board = _Tampered()
        results = {result.name: result for result in cases.run(board, ('control',), flags, None)}
        assert results['arm_while_held'].reason == 'needs ' + missing, results['arm_while_held']
        assert board.sent.count('arm') == arms, board.sent

    def refusal(problems: dict) -> typing.Callable:
        """A reply replacement refusing the arm with `problems`."""
        return lambda lines: [cc.build('err', ['unsafe', json.dumps(problems)])]

    lost_hold = 'stage is gliding, not setting -- `stage setting`, then power-cycle before flight'
    scenarios = (
        (_answer('ok', {'armed': True}), 'expected err unsafe {stage}, got ok'),
        (refusal({'flight': 'flight -- not ported yet'}), "lacks key 'stage'"),
        (refusal({'stage': lost_hold}), 'stage reads %r' % lost_hold),
        (refusal({'stage': cases._HELD_STAGE_PROBLEM, 'imu_lsm6dso32': 5}), "problems['imu_lsm6dso32'] is int"),
    )
    for replace, reason in scenarios:
        board = _Tampered('arm', replace, occurrence=2)  # the first arm is the arm case's, on the ground
        results = {result.name: result for result in cases.run(board, ('control',), allowed, None)}
        assert _failed(results.values()) == {'arm_while_held'}, [(item.name, item.reason) for item in results.values()]
        assert reason in results['arm_while_held'].reason, results['arm_while_held'].reason
        after_hold = board.sent[board.sent.index('stage gliding'):]
        assert after_hold[after_hold.index('arm') + 1] == 'disarm' and 'stage setting' in after_hold, after_hold
    unsupported = lambda lines: [cc.build('err', ['unsupported', 'no arm gate yet'])]  # noqa: E731
    for milestone, outcome in ((None, cases.NOT_YET), (6, cases.NOT_YET), (7, cases.FAIL)):
        results = {result.name: result for result in cases.run(_Tampered('arm', unsupported, occurrence=2),
                                                               ('control',), allowed, None, milestone=milestone)}
        assert results['arm_while_held'].outcome == outcome, (milestone, results['arm_while_held'])

    def interrupted(lines: list) -> list:
        """The operator pressing Ctrl-C while the disarm after the held arm is out."""
        raise KeyboardInterrupt

    stopped = _Tampered('disarm', interrupted, occurrence=3)  # disarm, disarm_after_arm, then this one
    try:
        cases.run(stopped, ('control',), allowed, None)
    except cases.Aborted as error:
        assert 'disarm: ok' in str(error) and 'stage setting: ok' in str(error), error
    else:
        raise AssertionError('Ctrl-C did not abort the run')
    assert stopped.sent[-2:] == ['disarm', 'stage setting'], stopped.sent[-4:]

    def foreign_readiness(payload: dict) -> None:
        """verify with a readiness item of the board's own."""
        payload['readiness']['not-flight-capable'] = 'flight -- not ported yet'

    def every_check(payload: dict) -> None:
        """verify tripping every check plan B makes."""
        payload['readiness'] = {key: 'tripped' for key in cases._READINESS_CHECKS}

    foreign = "readiness names ['not-flight-capable'], which plan B never reports"
    for edit, failed in ((every_check, set()), (foreign_readiness, {'verify'})):
        results = cases.run(_Tampered('verify', _payload_edit(edit)), ('diagnostics',), ('active',), None)
        assert _failed(results) == failed, [(result.name, result.reason) for result in results]
        assert all(foreign in result.reason for result in results if result.outcome == cases.FAIL), results

    without_active = {key: value for key, value in _BLUETOOTH_STUB.items() if key != 'active'}
    for snapshot, reason in (({**_BLUETOOTH_STUB, 'radio': True, 'active': True}, None),  # a board with BLE, on
                             ({**_BLUETOOTH_STUB, 'radio': 'off'}, "key 'radio' is str, expected bool"),
                             ({**_BLUETOOTH_STUB, 'active': 'on'}, "key 'active' is str, expected bool|null"),
                             (without_active, "lacks key 'active'"),
                             ({**_BLUETOOTH_STUB, 'name': 'ble'}, "inspect bluetooth names 'ble'")):
        answers = _fitted_answers()
        answers['inspect bluetooth'] = [cc.build('ok', [json.dumps(snapshot)])]
        results = {result.name: result for result in cases.run(_Fitted(answers), ('config', 'inspect'), (), None)}
        assert results['inspect_bluetooth'].outcome == (cases.PASS if reason is None else cases.FAIL), snapshot
        assert _failed(results.values()) == ({'inspect_bluetooth'} if reason else set()), snapshot
        assert reason is None or reason in results['inspect_bluetooth'].reason, results['inspect_bluetooth'].reason
    bare = {result.name: result for result in cases.run(_PLAN_B, ('config', 'inspect'), (), None)}
    assert bare['inspect_bluetooth'].outcome == cases.SKIP and 'no bluetooth' in bare['inspect_bluetooth'].reason
    assert json.loads(cc.parse(_PLAN_B.exchange('stage')[0]).args[0]) == {'stage': 'setting', 'manual': False}
    assert json.loads(cc.parse(_PLAN_B.exchange('health')[0]).args[0])['armed'] is False


_M10_FITTED: tuple = ('airspeed_sdp810', 'power_ina226', 'laser_agl', 'gnss', 'gnss_calib', 'servo_eleron_left',
                      'imu_bno055', 'baro_bmp280')


def _m10_answers() -> dict:
    """
    The healthy answers of the M10-M21 parts and the Rust board's GCL reads, by request line: the pitot, the power
    monitor, the laser, the GNSS and its pad drift, a servo, the SEN0253's BNO055 and BMP280 (inspect, probe,
    calibrate), and help (a GCL board), df, ls, ring, video stats, ota status and entry show.
    """
    barometer = {'name': 'baro_bmp280', 'ok': True, 'healthy': True, 'addr': '0x76', 'altitude_m': -30.4,
                 'temperature_c': 27.9, 'pressure_pa': 101691.2, 'elevation_m': 0.15, 'change_m_s': 0.09, 'torn': 47,
                 'implausible': 0}
    return {
        'inspect airspeed_sdp810': _ok({'name': 'airspeed_sdp810', 'ok': True, 'healthy': True,
                                        'dynamic_pressure_pa': 0.4, 'airspeed_ms': 0.8, 'temperature_c': 27.5,
                                        'scale': 60, 'zero_offset_pa': -1.25, 'tared': True, 'air_density': 1.17}),
        'inspect power_ina226': _ok({'name': 'power_ina226', 'ok': True, 'healthy': True, 'voltage_mv': 5012,
                                     'current_ma': 412, 'power_mw': 2065, 'alerts': 0}),
        'inspect laser_agl': _ok({'name': 'laser_agl', 'ok': True, 'healthy': True, 'interrupt': False,
                                  'agl_m': 0.52}),
        'inspect gnss': _ok({'name': 'gnss', 'ok': True, 'healthy': True, 'fix': True, 'satellites': 9, 'hdop': 1.1,
                             'fix_quality': 1, 'position': [48.1, 11.5], 'altitude_m': 520.0, 'elevation_m': 1.0,
                             'speed_ms': 0.1, 'in_view': 12, 'used': 9, 'mode': 3, 'cn0': [41, 39, 38, 36],
                             'cn0_mean': 38.5, 'antenna': 'ok', 'lines': 9216}),
        'inspect gnss_calib': _ok({'name': 'gnss_calib', 'ok': True, 'healthy': True, 'drift_e': 0.02,
                                   'drift_n': -0.01, 'samples': 120, 'frozen': False}),
        'inspect servo_eleron_left': _ok({'name': 'servo_eleron_left', 'ok': True, 'healthy': True, 'angle': 90,
                                          'pulse_us': 1500, 'trim': 0, 'feedback': None}),
        'inspect imu_bno055': _ok({'name': 'imu_bno055', 'ok': True, 'healthy': True, 'addr': '0x28',
                                   'attitude_deg': [0.0, -2.3125, -7.75], 'accel_g': [-0.04, -0.13, 0.94],
                                   'fusion_stalled': False, 'calibration': [0, 3, 3, 3], 'calibrated': True,
                                   'calibration_restored': True}),
        'inspect baro_bmp280': _ok(barometer),
        'probe imu_bno055': _ok({'imu_bno055': None}),
        'probe baro_bmp280': _ok({'baro_bmp280': None}),
        'calibrate imu_bno055': _ok({'imu_bno055': None}),
        'calibrate baro_bmp280': _ok({'baro_bmp280': None}),
        'help': _ok({'cc': ['whoami'], 'commands': [], 'grammar': ''}),
        'entry show': _ok({'lines': []}),
        'df': ['ok df layout=one_partition recordings=13767/14900 bounced=10'],
        'ls': _ok({'entries': [['000794/', None], ['sessions.csv', 110562]]}),
        'ring': ['ok ring erased=664 saved=0 unsaved=13 lost=8 bad=0 reserve=51 writer=66992/1712 saver=66979/3752 '
                 'flash_errors=0 saver_card_errors=0'],
        'video stats': ['ok video idle captured=0 missed=0 encoded=0 dropped=0 write_errors=0 segments=0 preview=0'],
        'ota status': ['ok ota running=ota_0 boot=ota_0 next=ota_1 build=30a03b1135850e7a transfer=none '
                       'received=none'],
    }


def test_the_m10_to_m21_parts_and_gcl_reads_hold_their_shapes() -> None:
    """
    The M10-M21 cases pass on the Rust board's healthy answers -- the pitot, power monitor, laser, GNSS, pad drift,
    servo, the SEN0253 (inspect, probe, calibrate), health's imu_calibration, and the GCL reads with their name=value
    figures -- and each broken answer fails exactly its own case. A named figure on a CC reply breaks the envelope.
    """
    tiers, allowed = ('config', 'inspect', 'diagnostics', 'ota'), ('active',)
    names = {'inspect_sdp810', 'inspect_ina226', 'inspect_laser', 'inspect_gnss', 'inspect_gnss_calib',
             'inspect_servo', 'inspect_bno055', 'inspect_bmp280', 'health_imu_calibration', 'probe_sen0253',
             'calibrate_bmp280', 'calibrate_bno055', 'entry_show', 'df', 'ls', 'ring', 'video_stats', 'ota_status'}

    def run(answers: dict) -> dict:
        """The named cases' results on a board answering `answers`."""
        board = _Fitted(answers, _M10_FITTED)
        return {result.name.split('[')[0]: result for result in cases.run(board, tiers, allowed, None, milestone=21)
                if result.name.split('[')[0] in names}

    healthy = run(_m10_answers())
    assert {name: result.outcome for name, result in healthy.items()} == dict.fromkeys(names, cases.PASS), \
        [(name, result.reason) for name, result in healthy.items() if result.outcome != cases.PASS]

    def edited(request: str, edit: typing.Callable) -> dict:
        """The healthy answers with one JSON payload edited."""
        answers = _m10_answers()
        payload = json.loads(cc.parse(answers[request][0]).args[-1])
        edit(payload)
        answers[request] = _ok(payload)
        return answers

    def replaced(request: str, lines: list) -> dict:
        """The healthy answers with one reply's lines replaced."""
        answers = _m10_answers()
        answers[request] = lines
        return answers

    def health_with(calibration: object) -> dict:
        """The healthy answers, health carrying this imu_calibration on plan B's own health."""
        answers = _m10_answers()
        answers['health'] = _payload_edit(lambda payload: payload.update(imu_calibration=calibration))
        return answers

    for answers, failing, words in (
            (edited('inspect imu_bno055', lambda payload: payload.update(calibration=[0, 3, 3, 4])), 'inspect_bno055',
             'in 0..3'),
            (edited('inspect imu_bno055', lambda payload: payload.update(calibrated=False)), 'inspect_bno055',
             'restored profile sets the latch'),
            (health_with([1, 2, 3]), 'health_imu_calibration', 'in 0..3'),
            (replaced('calibrate imu_bno055', _ok({'imu_bno055': 'profile read failed: no ack'})), 'calibrate_bno055',
             'neither saved'),
            (edited('inspect gnss', lambda payload: payload.update(position=None)), 'inspect_gnss', 'no position'),
            (edited('inspect gnss', lambda payload: payload.update(used=13)), 'inspect_gnss', 'in view'),
            (replaced('df', ['ok df layout=one_partition recordings=13767/14900']), 'df', 'no bounced='),
            (replaced('df', ['ok df layout=one_partition recordings=lots bounced=0']), 'df', 'recordings='),
            (replaced('ota status', ['ok ota running=ota_1 boot=ota_1 next=ota_1 build=30a0 transfer=none '
                                     'received=none']), 'ota_status', 'running slot'),
            (replaced('inspect imu_bno055', ['ok {"name":"imu_bno055"} extra=1']), 'inspect_bno055',
             'key=value params'),
            (replaced('entry show', ['err badcmd entry']), 'entry_show', 'owed from M21'),
    ):
        results = run(answers)
        failed = {name for name, result in results.items() if result.outcome == cases.FAIL}
        assert failed == {failing}, (failing, [(name, result.reason) for name, result in results.items()
                                               if result.outcome != cases.PASS])
        assert words in results[failing].reason, results[failing].reason
    assert cases._named(cases._DF, {'layout': 'two_partitions', 'recordings': '1/2', 'videos': '3/4',
                                    'bounced': '0'}) is None
    assert 'no bounced=' in cases._named(cases._DF, {'layout': 'one_partition'})


def test_the_m9_parts_are_held_to_their_shapes() -> None:
    """
    The M9 cases pass on the Rust board's healthy answers -- the SEN0697 parts, the attitude filter, the sequencer and
    the databoard; the BMP581's ground round trip; the bus diagnostics refused armed and held, bare while disarmed;
    the armed rung off the IMU bus's clock refused and the one back run -- and each broken answer fails exactly its
    own case. Nothing changes without its flag, an arm that is refused skips the armed checks (and sends no rung off
    the clock), a Rust-only diagnostic plan B lacks is not yet until M9, and a run that stops undoes the BMP581
    ground and the off-clock rung.
    """
    tiers, allowed = ('config', 'inspect', 'control', 'diagnostics'), ('active', 'stage')
    ground_line = lambda ground: cc.build('update', ['baro_bmp581', json.dumps({'ground': ground})])  # noqa: E731
    stepped, original = ground_line(round(_BMP581_GROUND + cases._GROUND_STEP_M, 3)), ground_line(_BMP581_GROUND)
    text_ground = ground_line('high')
    off_clock, back = 'bustune spi 1 %d' % (_IMU_CLOCK // 2), 'bustune spi 1 %d' % _IMU_CLOCK
    with _patched(cases, _CONVERSION_SETTLE_S=0.0):
        board = _Sen0697(_sen0697_answers())
        results = cases.run(board, tiers, allowed, None, milestone=9)
        assert not _failed(results), [(result.name, result.reason) for result in results if result.outcome != 'pass']
        outcomes = {result.name.split('[')[0]: result.outcome for result in results
                    if result.name.split('[')[0] in _M9_BOARD_ONLY | _M9_ON_PLAN_B}
        assert set(outcomes.values()) == {cases.PASS}, outcomes
        assert set(outcomes) == _M9_BOARD_ONLY | _M9_ON_PLAN_B, outcomes
        assert len([result for result in results if result.name.startswith('probe_sen0697[')]) == 3, results
        armed = board.sent[board.sent.index('arm'):board.sent.index('disarm', board.sent.index('arm'))]
        assert armed[1:] == list(_BUS_USAGE) + [off_clock, back], armed
        assert board.sent.index(stepped) < board.sent.index(text_ground) < board.sent.index(original), board.sent
        assert board.ground == _BMP581_GROUND, 'the round trip put the ground read back'

        def broken(request: str, answer: object) -> list:
            """A run on the healthy answers with one replaced (a payload: answered ok; lines: as they are)."""
            answers = _sen0697_answers()
            answers[request] = answer if isinstance(answer, list) or callable(answer) else _ok(answer)
            return cases.run(_Sen0697(answers), tiers, allowed, None)

        def edited(request: str, edit: typing.Callable) -> list:
            """A run whose answer to `request` is the healthy one, edited."""
            healthy = _sen0697_answers()[request]
            payload = json.loads(cc.parse(healthy[0]).args[0])
            edit(payload)
            return broken(request, payload)

        bmi323, bmm350, attitude = 'inspect imu_bmi323', 'inspect mag_bmm350', 'inspect attitude'
        unchanged = _ok({'name': 'baro_bmp581', 'ok': True, 'healthy': True, 'addr': '0x47',
                         'altitude_m': _BMP581_ALTITUDE, 'temperature_c': 27.65, 'pressure_pa': 101059.56,
                         'elevation_m': _BMP581_ALTITUDE - _BMP581_GROUND, 'change_m_s': 0.083, 'torn': 490,
                         'implausible': 0})
        undone = _refused('badargs', _BUS_USAGE['imu mute'])  # an armed board with no gate: the disarmed answer
        scenarios = (
            (edited(bmi323, lambda payload: payload.update(interrupt_silent=None)), {'inspect_bmi323'},
             'silent is null exactly when no INT1'),
            (edited(bmi323, lambda payload: payload.update(lapses={'accel': 0})), {'inspect_bmi323'}, 'lapses is'),
            (edited(bmi323, lambda payload: payload.update(addr='0x68')), {'inspect_bmi323'},
             "addr is '0x68', but the config puts imu_bmi323 at 0x69"),
            (edited(bmi323, lambda payload: payload.update(rate_dps=[0.1, 0.2])), {'inspect_bmi323'}, 'rate_dps is'),
            (edited(bmi323, lambda payload: payload.update(rate_cds=payload.pop('rate_dps'))), {'inspect_bmi323'},
             "lacks key 'rate_dps'"),
            (edited('inspect baro_icp10111', lambda payload: payload.pop('change_m_s')), {'inspect_barometer_change'},
             "lacks key 'change_m_s'"),
            (edited(bmm350, lambda payload: payload.update(span=[10, 5, 0, 1])), {'inspect_bmm350'}, 'span is'),
            (edited(bmm350, lambda payload: payload.update(sectors=9)), {'inspect_bmm350'}, 'outside 0..8'),
            (edited(attitude, lambda payload: payload.update(roll=-59.44)), {'inspect_attitude'},
             "key 'roll' is float, expected str"),
            (edited(attitude, lambda payload: payload.update(roll='-59.4')), {'inspect_attitude'}, 'not a decimal'),
            (edited('inspect databoard', lambda payload: payload['accel'].update(source='imu_ghost')),
             {'inspect_databoard'}, 'which stats databoard does not list'),
            (edited('inspect databoard', lambda payload: payload['agl'].update(value=3.0)), {'inspect_databoard'},
             'no source'),
            (edited('stats databoard', lambda payload: payload.update(accel=['imu_lsm6dso32', 'imu_bmi323'])),
             {'stats_databoard'}, 'not sorted'),
            (broken(cc.build('update', ['databoard', json.dumps({'altitude': 0})]), {'changed': ['altitude']}),
             {'update_databoard_read_only'}, 'the databoard took'),
            (edited('inspect sequencer', lambda payload: payload.update(
                ignored=["imu_ghost ignored: the stage machine's accel slots are full (4)"])),
             {'inspect_sequencer'}, "names ['imu_ghost'], which the board does not list"),
            (edited('inspect sequencer', lambda payload: payload.update(ignored=['baro_icp10111'])),
             {'inspect_sequencer'}, "is not `<source> ignored"),
            (broken('health', _payload_edit(lambda payload: payload['degraded'].append(
                'sources-disagree: accel imu_lsm6dso32/imu_bmi323 5.0 deg'))),
             {'health_names_its_disagreements'}, 'within 10 degrees agree'),
            (broken('health', _payload_edit(lambda payload: payload['degraded'].append(
                'sources-disagree: elevation baro_icp10111/baro_ghost 6.1 m'))),
             {'health_names_its_disagreements'}, 'names a source the databoard does not list'),
            (broken('inspect baro_bmp581', unchanged), {'bmp581_ground_moved'}, 'the ground reads 22.100 m'),
            (broken(text_ground, {'changed': ['ground']}), {'update_bmp581_text_ground'}, 'expected err refused'),
            (broken('calibrate mag_bmm350', {'mag_bmm350': None}), {'calibrate_bmm350_still'},
             'a calibration from no turn'),
            (broken('imu mute', undone), {'imu_mute_refused_armed'}, "expected 'unsafe'"),
            (broken(off_clock, {'kind': 'spi', 'id': '1', 'freq': _IMU_CLOCK // 2, 'devices': {}, 'all_ok': True}),
             {'bustune_off_clock_refused_armed'}, 'expected err unsafe <text>'),
            (broken(off_clock, _refused('unsafe', cases._BUSTUNE_ONLY_BACK % 400000)),
             {'bustune_off_clock_refused_armed'}, 'the refusal reads'),
            (broken('probe attitude', {'attitude': 'attitude: no gyro rate -- attitude filter blind'}),
             {'probe_attitude'}, "expected {'attitude': null}"),
            (broken('calibrate baro_bmp581', {'baro_bmp581': 'ground zero: no fresh altitude'}),
             {'calibrate_bmp581'}, "expected {'baro_bmp581': null}"),
        )
        for run, names, reason in scenarios:
            failed = [result for result in run if result.outcome == cases.FAIL]
            assert {result.name for result in failed} == names, [(result.name, result.reason) for result in failed]
            assert all(reason in result.reason for result in failed), [result.reason for result in failed]

        board = _Sen0697(_sen0697_answers())  # a gate that lets the rung run: the way back still follows it
        board._answers[off_clock] = _ok({'kind': 'spi', 'id': '1', 'freq': 1, 'devices': {}, 'all_ok': True})
        cases.run(board, tiers, allowed, None)
        assert board.sent[board.sent.index(off_clock) + 1] == back, board.sent

        quiet = _Sen0697(_sen0697_answers())  # no flag: reads only
        results = cases.run(quiet, tiers, (), None)
        assert not _failed(results), [(result.name, result.reason) for result in results if result.outcome == 'FAIL']
        for line in quiet.sent:
            assert not line.startswith(('update baro_', 'bustune', 'i2c', 'imu', 'arm', 'probe', 'calibrate ')), line
        assert cc.build('update', ['databoard', '{}']) in quiet.sent and 'inspect databoard' in quiet.sent

        refused = _Sen0697(_sen0697_answers())  # a board whose own config does not arm (no hold: no held arm)
        refused._answers['arm'] = _refused('unsafe', json.dumps({'flight': 'flight -- not ported yet'}))
        results = {result.name: result for result in cases.run(refused, tiers, ('active',), None, milestone=9)}
        assert not _failed(results.values()), [(item.name, item.reason) for item in results.values()]
        for name in ('i2c_read_refused_armed', 'imu_mute_refused_armed', 'bustune_off_clock_refused_armed'):
            assert results[name].outcome == cases.SKIP and 'did not arm (flight)' in results[name].reason, results[name]
        assert results['bustune_back_while_armed'].outcome == cases.SKIP and off_clock not in refused.sent
        assert refused.sent[refused.sent.index('arm') + 1] == 'disarm', refused.sent

        plan_b_answer = _refused('badcmd', 'i2c')  # plan B on a board: no such command
        for milestone, outcome in ((None, cases.NOT_YET), (8, cases.NOT_YET), (9, cases.FAIL)):
            answers = _sen0697_answers()
            answers['i2c read'] = plan_b_answer
            results = {result.name: result for result in cases.run(_Sen0697(answers), tiers, allowed, None,
                                                                   milestone=milestone)}
            for name in ('i2c_read_refused_armed', 'bus_diagnostic_while_held', 'i2c_read_without_arguments'):
                assert results[name].outcome == outcome, (milestone, results[name])

        answers = _sen0697_answers()
        answers[text_ground] = RuntimeError('USB gone')  # right after the stepped ground went out
        unplugged = _Sen0697(answers)
        try:
            cases.run(unplugged, tiers, allowed, None)
        except cases.Aborted as error:
            assert 'BMP581 ground restore: ok' in str(error), error
        else:
            raise AssertionError('a transport error did not abort the run')
        assert unplugged.sent[-1] == original and unplugged.ground == _BMP581_GROUND, unplugged.sent[-3:]

        answers = _sen0697_answers()
        answers[off_clock] = KeyboardInterrupt()  # the operator's Ctrl-C while the off-clock rung is out
        stopped = _Sen0697(answers)
        try:
            cases.run(stopped, tiers, allowed, None)
        except cases.Aborted as error:
            assert 'disarm: ok' in str(error) and 'IMU bus back to its clock: ok' in str(error), error
        else:
            raise AssertionError('Ctrl-C did not abort the run')
        after = stopped.sent[stopped.sent.index(off_clock) + 1:]
        assert after[0] == 'disarm' and after[-1] == back, after
    assert json.loads(cc.parse(_PLAN_B.exchange('health')[0]).args[0])['armed'] is False


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
    assert [result.outcome for result in push if result.name == 'push_status'] == [cases.NOT_YET]
    assert [result.outcome for result in push if result.name == 'ota_status'] == [cases.SKIP], 'plan B has no GCL'
    owed = cases.run(_Tampered('push-status', lambda lines: ['err badcmd push-status']), ('ota',), (), None,
                     milestone=21)
    assert [result.outcome for result in owed if result.name == 'push_status'] == [cases.NOT_YET], \
        'replaced by `ota` (M15): no milestone owes it'
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
    """
    /api/cmd rebuilds the line from {command, params, named}: only a line that survives that goes through the hub,
    the GCL commands' `key=value` params included.
    """
    assert transports.hub_request('ping') == ('ping', [], {})
    assert transports.hub_request('PING') == ('PING', [], {})
    launch = cc.build('set-config', ['launch', '{"launch_id": "x"}'])
    assert transports.hub_request(launch) == ('set-config', ['launch', '{"launch_id": "x"}'], {})
    assert transports.hub_request('ls path=/') == ('ls', [], {'path': '/'})
    rm = cc.build('rm', ['confirm'], {'path': '000374'})
    assert transports.hub_request(rm) == ('rm', ['confirm'], {'path': '000374'}), rm
    spaced = cc.build('video', ['start'], {'note': 'a b'})  # a named value that needs base64 survives
    assert transports.hub_request(spaced) == ('video', ['start'], {'note': 'a b'})
    for line in ('', '   ', 'inspect base64:QQ', 'inspect base64:/w==', 'inspect  mission',
                 'rm path=000374 confirm'):  # a positional word after a named param: rebuilt in another order
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
test_the_m7_gate_is_held_to_plan_b()
test_the_m9_parts_are_held_to_their_shapes()
test_the_m10_to_m21_parts_and_gcl_reads_hold_their_shapes()
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
      'two replies, codes, every invariant), the M6 parts, the M7 gate (held arm, readiness keys, bluetooth), the '
      'M9 parts (SEN0697, attitude, sequencer, databoard, BMP581 ground, armed bus diagnostics), the M10-M21 '
      'parts and the GCL reads (named figures, the help gate), the '
      'hold needs a disarmed board, preflight refusal, iam id, '
      'config_source gating, the reboot proof, the tee, milestones, aborted runs undo, --diff masking + known '
      'differences, hub line fidelity + HTTP errors + the hub tier, console log/ROM filter + no-reset open, '
      'the command line')
