"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The CC contract: what a board must answer, request by request, for the hub and the dashboard to treat it as a
board (doc/specs/cc-protocol.md). The Rust port (src/avionics) is held to it, and so is plan B
(src/glider/cc_client.py) itself, run on CPython by the `fake` transport -- so the two compare case by case.

Where the cases come from, in order of authority: the plan B dispatcher (the truth where the spec and the code
disagree; each such case carries a `note`), the spec's command catalog, and the wire asserts of
src/glider/test/test_cc_client.py.

A case is one request line and what may come back:
  * `replies` -- the accepted answers, each a status followed by its arguments. A literal string must match
    verbatim, TEXT is any text, a dict is a JSON object {key: type} (a key ending in '?' is optional; keys the
    case does not name are allowed, so a board may report more), ListOf(type) is a JSON list. The argument
    count is part of the answer. NO_REPLY (no alternatives) is the answer to an empty line: nothing at all.
    A type is one of str, int, float, number (int or float), bool, null, list, object, any -- or several
    joined by '|'.
  * `invariants` -- checks on the values beyond their types (a stage that exists, pass == no problems, the
    launch config restored to the byte). Each returns the problem, or None.
  * `not_yet` -- a documented answer from a board that has not grown the feature: reported as NOT YET, never
    failed. `err unsupported` is one everywhere but the core tier, whose cases must pass from the first
    milestone on.
  * `milestone` -- the port-plan milestone (doc/specs/avionics-port.md) that owes the case. Run with a
    milestone (`--milestone M1`), a not-yet answer to a case owed by then is a FAIL: how a milestone's proof
    makes its tiers mandatory.
  * `allow` -- the operator flags the case needs. Nothing that moves a fin, writes a config (the board's or the
    launch's), holds the stage or reboots the board is ever sent without its flag.

Every reply also passes the envelope first: exactly one reply line per request (none to an empty one), a
status from the spec, and an `err` carrying exactly an error code from the spec and a message.

Cases run in TIERS order, system last, so a reboot cannot disturb the others. Before the first case a
preflight reads whoami and health: it fills the context the cases share and refuses a board that is airborne
or armed -- or, when any flag allows a change, one whose stage or arm state it could not read. Writes are
round trips: the launch config and the board config are read first, changed, and put back by a case that runs
whatever the change answered. A run that stops on an exception never reaches those cases, so run() then sends
the undo of every write that went out itself, best effort, before it reports the abort.

The hub tier is not request lines: it reads the hub's own view of the board (GET /api/boards) over several
heartbeats, and runs through the hub transport only.
"""

import dataclasses
import json
import re
import time
import typing

import cc_protocol as cc

from contract import transports

TIERS: tuple = ('core', 'stream', 'config', 'inspect', 'control', 'diagnostics', 'ota', 'hub', 'system')
ALLOW_FLAGS: tuple = ('reboot', 'config_write', 'launch_write', 'stage', 'active')  # each is a --allow-<flag>

PASS: str = 'pass'
NOT_YET: str = 'not yet'
FAIL: str = 'FAIL'
SKIP: str = 'skip'

STAGES: tuple = ('setting', 'boosting', 'gliding', 'landing', 'done')  # controller.Stage.STAGES, in order
_GROUND: tuple = ('setting', 'done')  # cc_client._GROUND: the stages where nothing flies
_STATUSES: tuple = ('ok', 'err', 'pong', 'iam')
_CODES: tuple = ('badcmd', 'badargs', 'invalid', 'unsafe', 'refused', 'unsupported', 'internal')

_EPOCH_2000: int = 946684800  # Unix seconds at 2000-01-01: no board clock reads earlier (MicroPython's RTC floor)
_BARE_TOKEN: re.Pattern = re.compile(r'[A-Za-z0-9._\-/+:]+')  # cc_protocol's safe set: an id travels bare
_CONFIG_ID: re.Pattern = re.compile(r'[0-9a-f]{12}|[0-9a-f]{8}')  # SHA-256 prefix, or the FNV-1a fallback
_LAYOUT: re.Pattern = re.compile(r'v\d+\.\d+( \(declared\))?|undecided')  # health.layout, as layout.RESOLVED
_HOLD_STAGE: str = 'gliding'  # the stage the gating checks hold: airborne, and quiet on a disarmed bench board
_REFUSED_IN_FLIGHT: str = 'refused in flight -- stage is %s (ground-only: setting/done)' % _HOLD_STAGE
# The Rust board's gate (doc/specs/avionics-port.md §6): every change runs only in SETTING, DONE included, and the
# text says so. Each board answers its own true text, so the gating cases accept either.
_REFUSED_OUTSIDE_SETTING: str = 'refused -- stage is %s (changes only in setting)' % _HOLD_STAGE
_PROBE_LAUNCH_ID: str = 'contract-probe'  # the launch id the launch round trip writes, then puts back
_SHOWN: int = 100  # characters of one reply argument a report line shows

_REBOOT_TIMEOUT_S: float = 90.0  # a board boots and re-dials the hub in ~10-20 s; this is the give-up point
_REBOOT_POLL_S: float = 1.0  # between the whoami reads that ask whether a new boot answers

HUB_BEATS: int = 3  # the heartbeats the hub tier watches by default (the M1 proof asks for 30)
_HUB_BEAT_S: float = 2.0  # one /api/boards read per hub heartbeat (server.HEARTBEAT_S, the hub's default)
_UNSET_CLOCK_YEAR: int = 2000  # a board clock still in 2000 is unset (server: an epoch before 2001-01-01)
_CC_CLOCK_VALID_FROM: int = 1577836800  # 2020-01-01: the hub syncs no board while its own clock reads earlier
"""
The dashboard row fields (server.Server.board_rows) every board fills from whoami and health. temp, position,
agl, launchpad and site are legitimately null, and flight and the calibration fields come and go with parts.
"""
_HUB_ROW: tuple = ('stage', 'layout', 'version', 'config_id', 'uptime', 'clock', 'launchpad_set', 'armed',
                   'degraded', 'mem_free')


TEXT: object = object()  # the argument spec for any text
NO_REPLY: tuple = ()  # the replies of a case whose request must get no answer at all


@dataclasses.dataclass(frozen=True)
class ListOf:
    """A JSON list argument whose items are all of one type (a type expression, as in a dict spec)."""

    item: str


class Blocked(Exception):
    """A case cannot run: what it needs from an earlier case is missing, or running it would change the board."""


class Refused(Exception):
    """The preflight refuses the board: airborne, armed, or (when a flag allows a change) neither readable."""


class Aborted(Exception):
    """The run stopped on an exception (a transport error, Ctrl-C); the message says what was undone."""


@dataclasses.dataclass(frozen=True)
class _NotYet:
    """
    A documented answer from a board that does not have the feature yet.

    Matched only after every accepted reply failed: a board that answers properly passes.
    """

    status: str  # the reply status it matches
    code: str  # the error code (first argument) it matches
    reason: str  # why this answer is legitimate for now


_UNSUPPORTED: _NotYet = _NotYet('err', 'unsupported', 'the capability is absent on this board')


@dataclasses.dataclass(frozen=True)
class Case:
    """One request and what may come back (the module docstring has the vocabulary)."""

    tier: str
    name: str
    request: object  # the line; or (context) -> line | [line, ...], raising Blocked or KeyError when it cannot run
    replies: tuple  # the accepted answers: (status, argument spec, ...) each; NO_REPLY for none
    invariants: tuple = ()  # (reply, context) -> problem | None, run on an accepted answer
    not_yet: tuple = (_UNSUPPORTED,)  # _NotYet answers reported instead of failed (core cases have none)
    allow: tuple = ()  # the ALLOW_FLAGS this case needs
    remember: typing.Callable | None = None  # (reply, context) -> None, run when the case passes
    reboots: bool = False  # a passing answer reboots the board: prove a new boot answers
    milestone: int = 1  # the port-plan milestone that owes it: M1 (CC parity) owes everything plan B answers
    note: str = ''  # where plan B, the spec and the Rust plan disagree


@dataclasses.dataclass(frozen=True)
class Result:
    """What one request got, judged."""

    tier: str
    name: str
    request: str
    outcome: str  # PASS, NOT_YET, FAIL or SKIP
    reason: str  # why it is not a pass ('' for a pass)
    replies: tuple = ()  # the raw reply lines
    free: tuple = ()  # the argument positions the matched answer leaves free (masked by --diff)
    note: str = ''


@dataclasses.dataclass(frozen=True)
class _Reply:
    """A parsed reply as the invariants see it: its arguments as their specs decode them."""

    request: str
    status: str
    values: list  # per argument: the decoded JSON for a dict/ListOf spec, else the text

    @property
    def payload(self) -> object:
        """The last argument's value: the JSON every structured answer carries last."""
        return self.values[-1] if self.values else None


"""Type checks for the JSON values a spec names."""


def _is_type(value: object, expression: str) -> bool:
    """
    Whether a JSON value is of a type expression.

    Args:
        value - a value json.loads produced.
        expression - type names joined by '|' (str, int, float, number, bool, null, list, object, any).

    Returns:
        True when the value is of any of the named types. A bool is never an int or a number.
    """
    for name in expression.split('|'):
        if name == 'any':
            return True
        if name == 'null' and value is None:
            return True
        if name == 'bool' and isinstance(value, bool):
            return True
        if isinstance(value, bool):
            continue
        if name == 'int' and isinstance(value, int):
            return True
        if name == 'float' and isinstance(value, float):
            return True
        if name == 'number' and isinstance(value, (int, float)):
            return True
        if name == 'str' and isinstance(value, str):
            return True
        if name == 'list' and isinstance(value, list):
            return True
        if name == 'object' and isinstance(value, dict):
            return True
    return False


def _type_name(value: object) -> str:
    """The JSON type name of a value, for a problem message."""
    if value is None:
        return 'null'
    if isinstance(value, bool):
        return 'bool'
    return {int: 'int', float: 'float', str: 'str', list: 'list', dict: 'object'}.get(type(value), type(value).__name__)


def _match(alternative: tuple, arguments: list) -> tuple:
    """
    Check a reply's arguments against one accepted answer.

    Args:
        alternative - (status, argument spec, ...).
        arguments - the reply's decoded arguments (text).

    Returns:
        (problem, values): problem None and the decoded values when they match; else the first problem.
    """
    specs = alternative[1:]
    if len(arguments) != len(specs):
        return 'expected %d argument(s) after %s, got %d' % (len(specs), alternative[0], len(arguments)), None
    values = []
    for position, (spec, argument) in enumerate(zip(specs, arguments), start=1):
        if spec is TEXT:
            values.append(argument)
            continue
        if isinstance(spec, str):
            if argument != spec:
                return 'argument %d is %r, expected %r' % (position, _cut(argument), spec), None
            values.append(argument)
            continue
        try:
            value = json.loads(argument)
        except ValueError:
            return 'argument %d is not JSON: %r' % (position, _cut(argument)), None
        problem = _shape(spec, value, position)
        if problem is not None:
            return problem, None
        values.append(value)
    return None, values


def _shape(spec: object, value: object, position: int) -> str | None:
    """
    Check one decoded JSON argument against its dict or ListOf spec.

    Args:
        spec - a {key: type} dict or a ListOf.
        value - the decoded argument.
        position - the argument's place, for the message.

    Returns:
        The problem, or None when the value fits.
    """
    if isinstance(spec, ListOf):
        if not isinstance(value, list):
            return 'argument %d is %s, expected a list' % (position, _type_name(value))
        for item in value:
            if not _is_type(item, spec.item):
                return 'argument %d holds a %s, expected %s items' % (position, _type_name(item), spec.item)
        return None
    if not isinstance(value, dict):
        return 'argument %d is %s, expected an object' % (position, _type_name(value))
    for key, expression in spec.items():
        name = key.rstrip('?')
        if name not in value:
            if not key.endswith('?'):
                return 'argument %d lacks key %r' % (position, name)
            continue
        if not _is_type(value[name], expression):
            return 'argument %d key %r is %s, expected %s' % (position, name, _type_name(value[name]), expression)
    return None


def _cut(text: str) -> str:
    """Text shortened to _SHOWN characters for a report line."""
    return text if len(text) <= _SHOWN else text[: _SHOWN - 3] + '...'


def _render(line: str) -> str:
    """A reply line as a report shows it: the status and each argument decoded, shortened."""
    try:
        message = cc.parse(line)
    except Exception:
        return _cut(line)
    return ' '.join([message.command or ''] + [_cut(argument) for argument in message.args])


def _describe(alternative: tuple) -> str:
    """An accepted answer as a report shows it: `err badargs <text>`, `ok {stage, manual}`, `pong`."""
    parts = [alternative[0]]
    for spec in alternative[1:]:
        if isinstance(spec, dict):
            parts.append('{%s}' % ', '.join(key for key in spec if not key.endswith('?')))
        elif isinstance(spec, ListOf):
            parts.append('[%s, ...]' % spec.item)
        else:
            parts.append('<text>' if spec is TEXT else spec)
    return ' '.join(parts)


"""The envelope: the rules every reply keeps, whatever the request."""


def _envelope(request: str, lines: list) -> str | None:
    """
    Check the reply lines of one request against the framing rules.

    Args:
        request - the request line sent.
        lines - the reply lines that came back.

    Returns:
        The problem, or None: one reply per request (none to an empty line), parsable, a spec status, no
        key=value params, and an err carrying a spec code and a message.
    """
    if not request.strip():
        return 'a reply to an empty line: %s' % ' | '.join(_render(line) for line in lines) if lines else None
    if not lines:
        return 'no reply'
    if len(lines) > 1:
        return '%d replies to one line: %s' % (len(lines), ' | '.join(_render(line) for line in lines))
    try:
        message = cc.parse(lines[0])
    except Exception as error:
        return 'unparsable reply %r (%r)' % (_cut(lines[0]), error)
    if message.command not in _STATUSES:
        return 'status %r is none of %s' % (message.command, '/'.join(_STATUSES))
    if message.named:
        return 'a reply carries key=value params %s' % sorted(message.named)
    if message.command == 'err':
        if len(message.args) != 2:
            return 'an err carries a code and a message, got %d argument(s): %s' % (len(message.args),
                                                                                    _render(lines[0]))
        if message.args[0] not in _CODES:
            return 'error code %r is none of %s' % (message.args[0], '/'.join(_CODES))
    return None


"""Invariants: what a well-typed answer must also say."""


def _known_stage(reply: _Reply, context: dict) -> str | None:
    """The payload's `stage` is one of STAGES."""
    stage = reply.payload.get('stage')
    return None if stage in STAGES else 'stage %r is none of %s' % (stage, '/'.join(STAGES))


def _epoch_after_2000(reply: _Reply, context: dict) -> str | None:
    """The payload's `epoch`, where present, is no earlier than 2000-01-01 (the RTC's floor; CC's sync window)."""
    epoch = reply.payload.get('epoch')
    if epoch is None or epoch >= _EPOCH_2000:
        return None
    return 'epoch %d is before 2000-01-01 (%d)' % (epoch, _EPOCH_2000)


def _uptime_counts(reply: _Reply, context: dict) -> str | None:
    """The payload's `uptime` is not negative."""
    return None if reply.payload['uptime'] >= 0 else 'uptime %d is negative' % reply.payload['uptime']


def _identity(reply: _Reply, context: dict) -> str | None:
    """The iam id travels as a bare token, and it is the board the request was addressed to."""
    identity = reply.values[0]
    if not _BARE_TOKEN.fullmatch(identity):
        return 'iam id %r is not a bare token' % identity
    expected = context.get('board_id')
    if expected is not None and identity != expected:
        return 'iam %s, but the request went to board %s' % (identity, expected)
    return None


def _config_id_format(reply: _Reply, context: dict) -> str | None:
    """The payload's `config_id` is the 12-hex SHA-256 prefix (or the 8-hex fallback) config.config_id() gives."""
    config_id = reply.payload['config_id']
    return None if _CONFIG_ID.fullmatch(config_id) else 'config_id %r is not 12 (or 8) hex digits' % config_id


def _health_lists(reply: _Reply, context: dict) -> str | None:
    """
    health's lists hold what the dashboard renders: degraded texts, tasks {name, ok}, a known layout verdict.

    And outstanding calibration raises the needs-calibration flag: the dashboard's not-ready row reads both.
    """
    health = reply.payload
    if not all(isinstance(item, str) for item in health['degraded']):
        return 'degraded holds a non-text item: %s' % health['degraded']
    if not _LAYOUT.fullmatch(health['layout']):
        return 'layout %r is neither a revision (v1.1, v0.1 (declared)) nor undecided' % health['layout']
    for task in health['tasks']:
        if not isinstance(task, dict) or not isinstance(task.get('name'), str) or not isinstance(task.get('ok'), bool):
            return 'a tasks item is not {name: str, ok: bool}: %r' % (task,)
    if health.get('calibration') and 'needs-calibration' not in health['degraded']:
        return 'calibration is outstanding but degraded lacks needs-calibration'
    return None


def _lines_are_text(key: str) -> typing.Callable:
    """An invariant: the payload's `key` list (log lines, telemetry samples) holds text only."""

    def check(reply: _Reply, context: dict) -> str | None:
        """Every item of the payload's `key` list is text."""
        items = reply.payload[key]
        return None if all(isinstance(item, str) for item in items) else '%s holds a non-text item' % key

    return check


def _dropped_counts(reply: _Reply, context: dict) -> str | None:
    """`dropped` counts records the tee ring discarded: never negative."""
    return None if reply.payload['dropped'] >= 0 else 'dropped %d is negative' % reply.payload['dropped']


def _collected_nothing(key: str) -> typing.Callable:
    """
    An invariant: a closed window collected nothing (`key` is empty).

    The board tees log lines and telemetry for CC only while a window is open; after a `log 0` it collects
    nothing, so a second `log 0` drains an empty ring (cc-protocol.md "Log / telemetry retrieval").
    """

    def check(reply: _Reply, context: dict) -> str | None:
        """The payload's `key` list is empty."""
        if not reply.payload[key]:
            return None
        return ('a closed window collected %d %s -- is the hub streaming this board (stop it with `log off`)?'
                % (len(reply.payload[key]), key))

    return check


def _tee_carried(reply: _Reply, context: dict) -> str | None:
    """
    The log tee carried a line: a window open across `probe mission` drains at least one.

    A stub that answers `ok {lines: [], dropped: 0}` passes every other stream case, so this is the one that
    proves a log bridge exists. Plan B logs the mission probe through its Recorder (mission.py), and the Rust
    board owes the same from M1 (doc/specs/avionics-port.md). Judged only where the probe went out.
    """
    if not context.get('tee_triggered') or reply.payload['lines']:
        return None
    return ('a log window open across `probe mission` drained no line -- is the log bridge a stub, or is the hub '
            'streaming this board (stop it with `log off`)?')


def _names_this_board(reply: _Reply, context: dict) -> str | None:
    """The running config's board.id is the id this board announces in iam."""
    identity = context.get('id')
    configured = reply.payload['board'].get('id')
    if identity is None or configured == identity:
        return None
    return 'board.id is %r, but the board announces itself as %r' % (configured, identity)


def _same_as(key: str, what: str) -> typing.Callable:
    """An invariant: the payload equals the document an earlier case remembered as context[key]."""

    def check(reply: _Reply, context: dict) -> str | None:
        """The payload equals context[key], or names the fields that moved."""
        if key not in context or reply.payload == context[key]:
            return None
        changed = sorted(name for name in set(reply.payload) | set(context[key])
                         if reply.payload.get(name) != context[key].get(name))
        return 'differs from %s in %s' % (what, changed)

    return check


def _launch_id_is(expected: typing.Callable) -> typing.Callable:
    """An invariant: the payload's launch_id is expected(context)."""

    def check(reply: _Reply, context: dict) -> str | None:
        """The payload's launch_id is the one `expected` names."""
        wanted = expected(context)
        actual = reply.payload['launch_id']
        return None if actual == wanted else 'launch_id is %r, expected %r' % (actual, wanted)

    return check


def _merged_not_wiped(reply: _Reply, context: dict) -> str | None:
    """set-config launch MERGES: the probe changed launch_id and every other field kept its value."""
    original = context['launch']
    kept = {key: value for key, value in reply.payload.items() if key != 'launch_id'}
    before = {key: value for key, value in original.items() if key != 'launch_id'}
    if kept == before:
        return None
    return 'fields other than launch_id moved: %s' % sorted(key for key in set(kept) | set(before)
                                                            if kept.get(key) != before.get(key))


def _lists_mission(reply: _Reply, context: dict) -> str | None:
    """objects names the mission (every board has one) and names nothing twice."""
    names = reply.payload
    if 'mission' not in names:
        return 'objects lacks mission'
    if len(set(names)) != len(names):
        return 'objects names something twice: %s' % names
    return None


def _servo_trim(reply: _Reply, context: dict) -> str | None:
    """A servo's inspect carries its numeric `trim` (the dashboard's fin-zero editor reads it)."""
    name = cc.parse(reply.request).args[0]
    if not name.startswith('servo_'):
        return None
    trim = reply.payload.get('trim')
    if _is_type(trim, 'number'):
        return None
    return '%s has no numeric trim (%s)' % (name, _type_name(trim))


def _nothing_changed(reply: _Reply, context: dict) -> str | None:
    """An update with nothing in it changes nothing."""
    return None if reply.payload['changed'] == [] else 'an empty update changed %s' % reply.payload['changed']


def _stage_is(stage: str, manual: bool) -> typing.Callable:
    """An invariant: the stage answer reads {stage, manual} exactly."""

    def check(reply: _Reply, context: dict) -> str | None:
        """The payload reads exactly (stage, manual)."""
        actual = (reply.payload['stage'], reply.payload['manual'])
        return None if actual == (stage, manual) else 'stage reads %s, expected %s' % (actual, (stage, manual))

    return check


def _health_shows_hold(reply: _Reply, context: dict) -> str | None:
    """While held, health reports the held stage and flags STAGE HELD: an operator hold suppresses every detector."""
    if reply.payload['stage'] != _HOLD_STAGE:
        return 'health stage is %r while held at %s' % (reply.payload['stage'], _HOLD_STAGE)
    if 'STAGE HELD' not in reply.payload['degraded']:
        return 'degraded lacks STAGE HELD while the stage is held'
    return None


def _armed_is(armed: bool) -> typing.Callable:
    """An invariant: an ok {armed} answer reads `armed`."""

    def check(reply: _Reply, context: dict) -> str | None:
        """The payload's armed is `armed`."""
        return None if reply.payload['armed'] is armed else 'armed reads %r, expected %r' % (reply.payload['armed'],
                                                                                             armed)

    return check


def _arm_answer(reply: _Reply, context: dict) -> str | None:
    """arm either arms (armed true) or refuses with its problems: {name: '<driver> -- <why>'}, at least one."""
    if reply.status == 'ok':
        return _armed_is(True)(reply, context)
    problems = reply.payload
    if not problems:
        return 'arm refused with no problems'
    return _texts(problems, 'problems')


def _texts(verdicts: dict, what: str) -> str | None:
    """Every value of a verdict map is text, or the problem naming the first that is not."""
    for name, verdict in verdicts.items():
        if not isinstance(verdict, str):
            return '%s[%r] is %s, expected text' % (what, name, _type_name(verdict))
    return None


def _instructions(reply: _Reply, context: dict) -> str | None:
    """The calibrate sweep maps each device to an instruction for the operator: text."""
    return _texts(reply.payload, 'calibrate')


def _probe_maps(reply: _Reply, context: dict) -> str | None:
    """probe's first map holds null (healthy) or a verdict; the optional second, the not-fitted alternatives."""
    for name, verdict in reply.values[0].items():
        if verdict is not None and not isinstance(verdict, str):
            return 'probe[%r] is %s, expected null or text' % (name, _type_name(verdict))
    return _texts(reply.values[1], 'alternatives') if len(reply.values) > 1 else None


def _names_its_target(reply: _Reply, context: dict) -> str | None:
    """`probe <name>` answers a first map with exactly that name in it."""
    target = cc.parse(reply.request).args[0]
    keys = sorted(reply.values[0])
    return None if keys == [target] else 'probe %s answered for %s' % (target, keys)


def _verify_consistent(reply: _Reply, context: dict) -> str | None:
    """verify's verdicts agree with their maps: pass == no problems, ready == no readiness item, texts throughout."""
    report = reply.payload
    if report['pass'] != (not report['problems']):
        return 'pass is %r with %d problem(s)' % (report['pass'], len(report['problems']))
    if report['ready'] != (not report['readiness']):
        return 'ready is %r with %d readiness item(s)' % (report['ready'], len(report['readiness']))
    for key in ('devices', 'problems', 'alternatives', 'readiness'):
        problem = _texts(report[key], key)
        if problem is not None:
            return problem
    return None


"""Not-yet answers: legitimate for a board that has not grown the feature."""


_PUSH_NOT_PORTED: _NotYet = _NotYet('err', 'badcmd', 'push-* is not ported: the Rust board answers badcmd until '
                                                     'the firmware OTA is redesigned')
_SAVED_NOT_KNOWN: _NotYet = _NotYet('err', 'badargs', 'a firmware older than `get-config saved`')

"""Request builders: lines that depend on an earlier answer, and the markers that make a round trip restore."""


def _tee_trigger(context: dict) -> str:
    """
    `probe mission` inside an open log window, on a board that tees; marks it so the drain must carry a line.

    A data check that moves nothing and writes nothing (the launch position and the zone range), so it needs no
    flag. Plan B on CPython never sets its Recorder up, so it tees nothing and the case cannot run there.
    """
    if not context.get('tees'):
        raise Blocked('this board tees no log (plan B on CPython never sets its Recorder up)')
    if not context.get('log_window'):
        raise Blocked('the log window did not open')
    context['tee_triggered'] = True
    return 'probe mission'


def _launch_probe(context: dict) -> str:
    """set-config launch {launch_id: contract-probe}, once the original launch config is known; marks the write."""
    if 'launch' not in context:
        raise Blocked('the launch config was not read: there would be nothing to restore it to')
    context['launch_written'] = True
    return cc.build('set-config', ['launch', json.dumps({'launch_id': _PROBE_LAUNCH_ID})])


def _after_launch_write(context: dict) -> str:
    """get-config launch, once the probe write went out."""
    if not context.get('launch_written'):
        raise Blocked('the launch probe write was not sent')
    return 'get-config launch'


def _launch_restore(context: dict) -> str:
    """set-config launch {launch_id: <original>}: sent whenever the probe write went out, whatever it answered."""
    if not context.get('launch_written'):
        raise Blocked('nothing to restore: the launch probe write was not sent')
    return cc.build('set-config', ['launch', json.dumps({'launch_id': context['launch']['launch_id']})])


def _board_resave(context: dict) -> str:
    """
    set-config board <the saved config, unchanged>, where that leaves the board as it was; marks the write.

    Only on a board that booted its saved file (`active`) or had none (`default`, put back by reset-config). A
    fallback or a schema mismatch means the file on the board is not the document `saved` returns, and
    re-saving would replace it.
    """
    saved = context['saved']
    source = context.get('config_source')
    if source not in ('active', 'default'):
        raise Blocked('config_source is %r: a re-save would replace a board.config this boot did not load' % source)
    context['board_written'] = True
    return cc.build('set-config', ['board', json.dumps(saved)])


def _after_board_write(context: dict) -> str:
    """get-config saved, once the re-save went out."""
    if not context.get('board_written'):
        raise Blocked('the board config re-save was not sent')
    return 'get-config saved'


def _board_restore(context: dict) -> str:
    """reset-config, where the board had no board.config before the re-save wrote one."""
    if not context.get('board_written'):
        raise Blocked('nothing to restore: no board config was written')
    if context.get('config_source') != 'default':
        raise Blocked('nothing to restore: the board had its own board.config, and the re-save wrote it back')
    return 'reset-config'


def _hold(context: dict) -> str:
    """
    `stage gliding`, from setting only (the round trip ends with `stage setting`); marks the hold.

    Only on a board last seen disarmed, with no arm sent since: held at gliding, an armed board flies its
    control loop on the fins (and plan B commits a warm-start crumb) -- the hold is safe only disarmed.
    """
    if context.get('stage') != 'setting':
        raise Blocked('the hold returns the board to setting, and it is in %s' % context.get('stage'))
    if not context.get('disarmed'):
        raise Blocked('the board was not seen disarmed after the last arm: held at gliding it would fly the fins')
    context['hold_sent'] = True
    return 'stage ' + _HOLD_STAGE


def _while_held(line: str) -> typing.Callable:
    """A request sent only while the hold took."""

    def request(context: dict) -> str:
        """`line`, once the hold took."""
        if not context.get('held'):
            raise Blocked('the stage hold did not take')
        return line

    return request


def _return_to_ground(context: dict) -> str:
    """`stage setting` (back on the ground, sequencer free): sent whenever the hold went out."""
    if not context.get('hold_sent'):
        raise Blocked('nothing to undo: the hold was not sent')
    return 'stage setting'


def _arm(context: dict) -> str:
    """`arm`; marks it, so the disarm after it runs whatever it answered, and the board counts as armed till then."""
    context['arm_sent'] = True
    context['disarmed'] = False
    return 'arm'


def _disarm_after_arm(context: dict) -> str:
    """`disarm`, whenever an arm went out."""
    if not context.get('arm_sent'):
        raise Blocked('no arm was sent')
    return 'disarm'


def _reboot(context: dict) -> str:
    """`reboot`, once whoami told which boot it ends: the boot id a new boot must differ from."""
    if 'boot_id' not in context:
        raise Blocked('whoami was unreadable: nothing could prove the board really reset')
    return 'reboot'


def _inspect_each(context: dict) -> list:
    """`inspect <name>` for every object the board lists."""
    return [cc.build('inspect', [name]) for name in context['objects']]


def _remember(key: str) -> typing.Callable:
    """A remember hook storing the passing answer's payload as context[key]."""

    def store(reply: _Reply, context: dict) -> None:
        """context[key] = the payload."""
        context[key] = reply.payload

    return store


def _mark(key: str) -> typing.Callable:
    """A remember hook setting context[key] once the case passed."""

    def store(reply: _Reply, context: dict) -> None:
        """context[key] = True."""
        context[key] = True

    return store


"""The JSON shapes, as plan B sends them (the golden cc_dispatch fixture records the core ones)."""

_WHOAMI: dict = {'mcu': 'str', 'firmware_version': 'str', 'config_id': 'str', 'config_source': 'str',
                 'stage': 'str', 'uptime': 'int', 'boot_id': 'int|null', 'session': 'str', 'epoch': 'int'}
"""
health: every board runs a Controller and a Mission (main.py), so plan B always sends armed and tasks, and
clock, epoch, launchpad, launchpad_set and site; position and agl are always there, null until fresh. Only
calibration, imu_calibration and flight come and go with parts. temp is number|null where the golden
cc_dispatch fixture says float|null: MicroPython's esp32.mcu_temperature() returns an int (modesp32.c), and
the fixture only ever saw the host's null.
"""
_HEALTH: dict = {'temp': 'number|null', 'mem_free': 'int', 'uptime': 'int', 'stage': 'str', 'layout': 'str',
                 'degraded': 'list', 'position': 'list|null', 'clock': 'str', 'epoch': 'int',
                 'launchpad': 'list|null', 'launchpad_set': 'bool', 'site': 'str|null', 'agl': 'float|null',
                 'armed': 'bool', 'tasks': 'list', 'calibration?': 'object', 'imu_calibration?': 'list|null',
                 'flight?': 'object'}
_LOG: dict = {'lines': 'list', 'dropped': 'int'}
_TELEMETRY: dict = {'samples': 'list', 'dropped': 'int'}
_BOARD_CONFIG: dict = {'board': 'object', 'buses': 'object', 'pins': 'object', 'version?': 'str',
                       'components?': 'list', 'sensors?': 'list', 'fins?': 'object', 'recorder?': 'object',
                       'wifi?': 'object'}
_LAUNCH: dict = {'launch_id': 'str', 'site': 'str', 'latitude': 'number|null', 'longitude': 'number|null',
                 'altitude': 'number|null', 'zone': 'list|null', 'sites?': 'list'}
_MISSION: dict = {'launch_id': 'str', 'site': 'str', 'latitude': 'number|null', 'longitude': 'number|null',
                  'altitude': 'number|null', 'launch_point': 'list|null', 'launch_source': 'str|null',
                  'zone': 'list|null', 'clock': 'str', 'epoch': 'int', 'target?': 'list', 'gates?': 'list',
                  'distances_m?': 'object', 'in_range?': 'bool'}
_STAGE: dict = {'stage': 'str', 'manual': 'bool'}
_ARMED: dict = {'armed': 'bool'}
_REPORT: dict = {'stage': 'str', 'tasks': 'object'}
_VERIFY: dict = {'pass': 'bool', 'devices': 'object', 'problems': 'object', 'alternatives': 'object',
                 'ready': 'bool', 'readiness': 'object'}
_DETECT: dict = {'detected': 'str|null', 'applied': 'str|null', 'detail': 'any'}
_PUSH_STATUS: dict = {'uploading': 'str|null', 'orphans': 'list', 'received?': 'int', 'size?': 'int',
                      'chunks?': 'int'}
_ANY_OBJECT: dict = {}


def _bad_arguments(message: object = TEXT) -> tuple:
    """The accepted answer `err badargs <message>` (any message by default)."""
    return ('err', 'badargs', message)


def _core(name: str, request: str, replies: tuple, invariants: tuple = (), note: str = '') -> Case:
    """A core-tier case: it has no not-yet answer, because the core must pass from the first milestone on."""
    return Case('core', name, request, replies, invariants, not_yet=(), note=note)


"""The cases, tier by tier."""

_CORE: tuple = (
    _core('whoami', 'whoami', (('iam', TEXT, _WHOAMI),),
         (_identity, _known_stage, _config_id_format, _epoch_after_2000, _uptime_counts),
         note='the spec example omits boot_id, session and epoch; its catalog row and plan B carry them'),
    _core('ping', 'ping', (('pong',),)),
    _core('command_is_lowercased', 'PING', (('pong',),)),
    _core('health', 'health', (('ok', _HEALTH),),
         (_known_stage, _health_lists, _epoch_after_2000, _uptime_counts),
         note='the spec calls position, agl, armed and tasks "when the part exists"; every plan B board has a '
              'Controller and a Mission, so it always sends them, and the port plan owes them from M1'),
    _core('unknown_command', 'frobnicate', (('err', 'badcmd', 'frobnicate'),)),
    _core('unknown_command_is_lowercased', 'FROB', (('err', 'badcmd', 'frob'),)),
    _core('empty_line', '', NO_REPLY),
    _core('blank_line', '   ', NO_REPLY),
    _core('parse_error_bad_padding', 'inspect base64:QQ', (_bad_arguments(),)),
    _core('parse_error_not_utf8', 'inspect base64:/w==', (_bad_arguments(),)),
    _core('link_survives_errors', 'ping', (('pong',),)),
)

_STREAM: tuple = (
    Case('stream', 'log_stop', 'log 0', (('ok', _LOG),), (_lines_are_text('lines'), _dropped_counts)),
    Case('stream', 'log_closed_window_collects_nothing', 'log 0', (('ok', _LOG),),
         (_collected_nothing('lines'), _dropped_counts)),
    Case('stream', 'log_open_window', 'log 3000', (('ok', _LOG),), (_lines_are_text('lines'), _dropped_counts),
         remember=_mark('log_window')),
    Case('stream', 'log_tee_trigger', _tee_trigger, (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target),
         note='a board only: plan B on CPython tees nothing. The drain after it must carry a line'),
    Case('stream', 'log_drain_and_stop', 'log 0', (('ok', _LOG),),
         (_lines_are_text('lines'), _dropped_counts, _tee_carried)),
    Case('stream', 'log_duration_not_integer', 'log abc', (_bad_arguments('log <duration_ms>'),)),
    Case('stream', 'tlm_stop', 'tlm 0', (('ok', _TELEMETRY),), (_lines_are_text('samples'), _dropped_counts),
         milestone=3),
    Case('stream', 'tlm_closed_window_collects_nothing', 'tlm 0', (('ok', _TELEMETRY),),
         (_collected_nothing('samples'), _dropped_counts), milestone=3),
    Case('stream', 'tlm_open_window', 'tlm 1000', (('ok', _TELEMETRY),),
         (_lines_are_text('samples'), _dropped_counts), milestone=3),
    Case('stream', 'tlm_drain_and_stop', 'tlm 0', (('ok', _TELEMETRY),),
         (_lines_are_text('samples'), _dropped_counts), milestone=3),
    Case('stream', 'tlm_duration_not_integer', 'tlm x', (_bad_arguments('tlm <duration_ms>'),), milestone=3),
)

_CONFIG: tuple = (
    Case('config', 'get_config_defaults_to_board', 'get-config', (('ok', _BOARD_CONFIG),), (_names_this_board,)),
    Case('config', 'get_config_board', 'get-config board', (('ok', _BOARD_CONFIG),), (_names_this_board,),
         remember=_remember('running')),
    Case('config', 'get_config_running', 'get-config running', (('ok', _BOARD_CONFIG),),
         (_same_as('running', 'get-config board'),)),
    Case('config', 'get_config_saved', 'get-config saved', (('ok', _BOARD_CONFIG),), (),
         (_UNSUPPORTED, _SAVED_NOT_KNOWN), remember=_remember('saved')),
    Case('config', 'get_config_default', 'get-config default', (('ok', _BOARD_CONFIG),), ()),
    Case('config', 'get_config_launch', 'get-config launch', (('ok', _LAUNCH),), (),
         remember=_remember('launch')),
    Case('config', 'get_config_unknown_name', 'get-config nosuch', (_bad_arguments('unknown config nosuch'),)),
    Case('config', 'set_config_without_arguments', 'set-config', (_bad_arguments('set-config <name> <json>'),)),
    Case('config', 'set_config_without_json', 'set-config launch', (_bad_arguments('set-config <name> <json>'),)),
    Case('config', 'set_config_bad_json', 'set-config launch nojson', (_bad_arguments('bad json'),)),
    Case('config', 'set_config_launch_probe', _launch_probe, (('ok', _LAUNCH),),
         (_launch_id_is(lambda context: _PROBE_LAUNCH_ID),), allow=('launch_write',),
         note='the spec lists set-config -> ok {config_id}; for launch plan B answers the persisted mission. '
              'On plan B the save rewrites launch.config from Mission.persisted(), which drops a `sites` list '
              'and stores any field an unsaved `update mission` set -- and get-config launch shows neither, so '
              'the restored check cannot see it. Never pass --allow-launch-write to a plan B board with sites'),
    Case('config', 'set_config_launch_merges', _after_launch_write, (('ok', _LAUNCH),),
         (_launch_id_is(lambda context: _PROBE_LAUNCH_ID), _merged_not_wiped), allow=('launch_write',)),
    Case('config', 'set_config_launch_restore', _launch_restore, (('ok', _LAUNCH),),
         (_launch_id_is(lambda context: context['launch']['launch_id']),), allow=('launch_write',)),
    Case('config', 'set_config_launch_restored', _after_launch_write, (('ok', _LAUNCH),),
         (_same_as('launch', 'the launch config before the round trip'),), allow=('launch_write',)),
    Case('config', 'set_config_board_invalid', cc.build('set-config', ['board', '{}']), (('err', 'invalid', TEXT),),
         allow=('config_write',), milestone=2),
    Case('config', 'set_config_board_resave', _board_resave, (('ok', {'config_id': 'str'}),), (_config_id_format,),
         allow=('config_write',), milestone=2),
    Case('config', 'set_config_board_saved', _after_board_write, (('ok', _BOARD_CONFIG),),
         (_same_as('saved', 'the saved config before the re-save'),), allow=('config_write',), milestone=2),
    Case('config', 'reset_config_restore', _board_restore, (('ok',),), allow=('config_write',), milestone=2),
)

_INSPECT: tuple = (
    Case('inspect', 'objects', 'objects', (('ok', ListOf('str')),), (_lists_mission,),
         remember=_remember('objects'),
         note='plan B lists the databoard once a health read registered it, which every board does at once'),
    Case('inspect', 'inspect_without_object', 'inspect', (_bad_arguments('inspect <object>'),)),
    Case('inspect', 'inspect_unknown_object', 'inspect nosuch', (_bad_arguments('no object nosuch'),)),
    Case('inspect', 'inspect_mission', 'inspect mission', (('ok', _MISSION),), (_epoch_after_2000,)),
    Case('inspect', 'inspect_every_object', _inspect_each, (('ok', _ANY_OBJECT),), (_servo_trim,)),
    Case('inspect', 'update_without_arguments', 'update', (_bad_arguments('update <object> <json>'),)),
    Case('inspect', 'update_without_json', 'update mission', (_bad_arguments('update <object> <json>'),)),
    Case('inspect', 'update_unknown_object', cc.build('update', ['nosuch', '{}']),
         (_bad_arguments('no object nosuch'),)),
    Case('inspect', 'update_mission_nothing', cc.build('update', ['mission', '{}']), (('ok', {'changed': 'list'}),),
         (_nothing_changed,)),
    Case('inspect', 'update_mission_not_an_object', 'update mission 5',
         (('err', 'internal', TEXT), ('err', 'refused', TEXT)),
         note='plan B raises inside Mission.update (it iterates the number) and answers err internal, which pins '
              'the envelope rule that a raising handler keeps the link; the Rust board owes err refused '
              '(golden cc_dispatch.json)'),
    Case('inspect', 'stats_without_object', 'stats', (_bad_arguments('stats <object>'),)),
    Case('inspect', 'stats_unknown_object', 'stats nosuch', (_bad_arguments('no object nosuch'),)),
    Case('inspect', 'stats_mission', 'stats mission', (('ok', _ANY_OBJECT),), ()),
)

_CONTROL: tuple = (
    Case('control', 'stage_read', 'stage', (('ok', _STAGE),), (_known_stage,)),
    Case('control', 'report', 'report', (('ok', _REPORT),), (_known_stage,)),
    Case('control', 'disarm', 'disarm', (('ok', _ARMED),), (_armed_is(False),), remember=_mark('disarmed')),
    Case('control', 'arm', _arm, (('ok', _ARMED), ('err', 'unsafe', _ANY_OBJECT)), (_arm_answer,),
         allow=('active',),
         note='arm probes every device -- each servo sweeps its full travel -- before it answers'),
    Case('control', 'disarm_after_arm', _disarm_after_arm, (('ok', _ARMED),), (_armed_is(False),),
         allow=('active',), remember=_mark('disarmed')),
    Case('control', 'stage_unknown_name', 'stage nosuch', (_bad_arguments('no stage nosuch'),), allow=('stage',),
         note='plan B ignores the name only without a Controller, which no board runs without'),
    Case('control', 'stage_hold', _hold, (('ok', _STAGE),), (_stage_is(_HOLD_STAGE, True),), allow=('stage',),
         remember=_mark('held')),
    Case('control', 'health_while_held', _while_held('health'), (('ok', _HEALTH),), (_health_shows_hold,),
         allow=('stage',)),
    Case('control', 'ground_only_refused_in_flight', _while_held(cc.build('set-config', ['board', '{}'])),
         (('err', 'unsafe', _REFUSED_IN_FLIGHT), ('err', 'unsafe', _REFUSED_OUTSIDE_SETTING)), allow=('stage',),
         note='an active command outside the ground answers err unsafe with this exact text: plan B\'s (outside '
              'setting/done) or the Rust board\'s (outside setting). An empty board config fails validation before '
              'any write (config.save), so a broken gate answers err invalid and writes nothing'),
    Case('control', 'update_while_held', _while_held(cc.build('update', ['mission', '{}'])),
         (('ok', {'changed': 'list'}), ('err', 'unsafe', _REFUSED_OUTSIDE_SETTING)), allow=('stage',),
         note='plan B leaves update open in flight (an empty update changes nothing); the Rust board refuses every '
              'change outside SETTING, the operator\'s rule'),
    Case('control', 'stage_return_to_ground', _return_to_ground, (('ok', _STAGE),), (_stage_is('setting', False),),
         allow=('stage',),
         note='stage setting is a return to the ground, not a hold: manual goes false (Controller.hold)'),
    Case('control', 'stage_resume', 'stage auto', (('ok', _STAGE),), (_stage_is('setting', False),),
         allow=('stage',)),
    Case('control', 'stage_read_after_hold', 'stage', (('ok', _STAGE),), (_stage_is('setting', False),),
         allow=('stage',)),
)

_DIAGNOSTICS: tuple = (
    Case('diagnostics', 'calibrate_sweep', 'calibrate', (('ok', _ANY_OBJECT),), (_instructions,),
         note='no argument: a read of the outstanding calibrations, open in every stage'),
    Case('diagnostics', 'calibrate_unknown_device', 'calibrate nosuch',
         (_bad_arguments('no calibrate for nosuch'),), allow=('active',)),
    Case('diagnostics', 'probe_all', 'probe', (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps,), allow=('active',)),
    Case('diagnostics', 'probe_mission', 'probe mission', (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target), allow=('active',)),
    Case('diagnostics', 'probe_unknown_object', 'probe nosuch', (_bad_arguments('no probe for nosuch'),),
         allow=('active',)),
    Case('diagnostics', 'verify', 'verify', (('ok', _VERIFY),), (_verify_consistent,),
         allow=('active',)),
    Case('diagnostics', 'detect', 'detect', (('ok', _DETECT),), (), allow=('active',), milestone=5,
         note='read-only, but the spec counts it active: it re-scans both I2C buses'),
    Case('diagnostics', 'bustune_without_arguments', 'bustune', (_bad_arguments('bustune <kind> <id> <freq>'),),
         allow=('active',), milestone=5),
)

_OTA: tuple = (
    Case('ota', 'push_status', 'push-status', (('ok', _PUSH_STATUS),), (), (_UNSUPPORTED, _PUSH_NOT_PORTED),
         milestone=15,
         note='the spec lists {uploading, received, size, chunks, orphans}; with no upload open plan B answers '
              '{uploading: null, orphans}'),
)

_SYSTEM: tuple = (
    Case('system', 'reboot', _reboot, (('ok',),), allow=('reboot',), reboots=True,
         note='an ok proves nothing until a whoami from a new boot answers: a changed boot_id, or an uptime '
              'shorter than the time since the ok'),
    Case('system', 'ping_after_reboot', 'ping', (('pong',),), not_yet=(), allow=('reboot',)),
)

CASES: tuple = _CORE + _STREAM + _CONFIG + _INSPECT + _CONTROL + _DIAGNOSTICS + _OTA + _SYSTEM

"""Running the cases."""


def _preflight(transport: transports.Transport, board_id: str | None, allowed: tuple) -> dict:
    """
    Read whoami and health once, before any case: the context every case shares, and the safety refusal.

    A failing read leaves its keys out (the cases that need them are then blocked) -- the whoami and health
    cases themselves report the failure. That is enough for a read-only run, but not for one a flag lets change
    the board: the runner never touches a board it has not seen on the ground and disarmed.

    Args:
        transport - the transport to the board.
        board_id - the id the operator addressed (checked against iam), or None for the fake.
        allowed - the ALLOW_FLAGS the operator gave.

    Returns:
        The context: board_id and tees, and from the board id/stage/config_source/boot_id/epoch (whoami),
        armed (health).

    Raises:
        Refused - the board is outside setting/done or armed; or a flag is given and its stage or arm state
            could not be read.
    """
    context = {'board_id': board_id, 'tees': transport.tees}
    for request in ('whoami', 'health'):
        try:
            lines = transport.exchange(request)
            message = cc.parse(lines[0]) if len(lines) == 1 else None
            payload = json.loads(message.args[-1]) if message is not None and message.args else None
        except (transports.NoAnswer, transports.Unreachable, OSError, ValueError, IndexError):
            continue
        if not isinstance(payload, dict):
            continue
        if request == 'whoami' and message.command == 'iam':
            context.update(id=message.args[0], stage=payload.get('stage'), config_source=payload.get('config_source'),
                           boot_id=payload.get('boot_id'), epoch=payload.get('epoch'))
        elif request == 'health' and message.command == 'ok':
            if isinstance(payload.get('armed'), bool):
                context['armed'] = payload['armed']
            if context.get('stage') is None:
                context['stage'] = payload.get('stage')
    if context.get('stage') is not None and context['stage'] not in _GROUND:
        raise Refused('the board is in stage %s -- the contract runs on the ground only (%s)'
                      % (context['stage'], '/'.join(_GROUND)))
    if context.get('armed'):
        raise Refused('the board is ARMED -- disarm it first; the runner never touches an armed board')
    unknown = [what for what in ('stage', 'armed') if context.get(what) is None]
    if allowed and unknown:
        raise Refused('unknown: %s -- whoami/health unreadable; with --allow-* the runner sends nothing to a board '
                      'it has not seen on the ground and disarmed' % ', '.join(unknown))
    return context


def run(transport: transports.Transport, tiers: tuple, allowed: tuple, board_id: str | None,
        milestone: int | None = None, beats: int = HUB_BEATS) -> list:
    """
    Run the cases of `tiers` against one board.

    Args:
        transport - the transport to the board.
        tiers - the tiers to run (any order; they run in TIERS order).
        allowed - the ALLOW_FLAGS the operator gave.
        board_id - the id the operator addressed (iam must match), or None for the fake.
        milestone - the port-plan milestone the board claims: a not-yet answer to a case owed by then FAILs;
            None reports every not-yet answer as NOT YET.
        beats - the heartbeats the hub tier watches.

    Returns:
        A Result per request (per check in the hub tier), in run order.

    Raises:
        Refused - the preflight refused the board; nothing else was sent.
        Aborted - the run stopped on an exception (a transport error, Ctrl-C); the undo lines of every write
            that went out were sent first, best effort, and the message says how each went.
    """
    context = _preflight(transport, board_id, allowed)
    results = []
    try:
        for tier in TIERS:
            if tier not in tiers:
                continue
            if tier == 'hub':
                results.extend(_hub_checks(transport, context, beats))
                continue
            for case in CASES:
                if case.tier == tier:
                    results.extend(_run_case(case, transport, allowed, context, milestone))
    except (Exception, KeyboardInterrupt) as error:
        undone = _undo(transport, context)
        raise Aborted('the run stopped after %d result(s) on %s: %s -- undo: %s'
                      % (len(results), type(error).__name__, error, '; '.join(undone) or 'nothing to undo')) from error
    return results


def _undo(transport: transports.Transport, context: dict) -> list:
    """
    Put the board back after a run that stopped half way: the undo of every write that went out, best effort.

    The cases undo their own writes whatever the write answered, but only if the run reaches them; an
    exception (a USB unplug, Ctrl-C during a 30 s probe) skips them, and could leave the board held at gliding
    or carrying the probe launch id. Every undo is idempotent, so one a case already sent is merely repeated.
    The disarm goes first: an armed board is the one that can move a fin.

    Args:
        transport - the transport to the board.
        context - the shared context, its markers saying which writes went out.

    Returns:
        One line per undo sent: '<what>: <the reply, or why there was none>'.
    """
    undo = []
    if context.get('arm_sent'):
        undo.append(('disarm', 'disarm'))
    if context.get('hold_sent'):
        undo.append(('stage setting', 'stage setting'))
    if context.get('launch_written'):
        undo.append(('launch_id restore', _launch_restore(context)))
    if context.get('board_written') and context.get('config_source') == 'default':
        undo.append(('reset-config', 'reset-config'))
    report = []
    for what, line in undo:
        try:
            replies = transport.exchange(line)
        except Exception as error:
            report.append('%s: %s' % (what, error))
            continue
        report.append('%s: %s' % (what, _render(replies[0]) if replies else 'no reply'))
    return report


def _run_case(case: Case, transport: transports.Transport, allowed: tuple, context: dict,
              milestone: int | None) -> list:
    """
    Run one case: its gate, its request (one, or one per object), and the judgement of each answer.

    Args:
        case - the case.
        transport - the transport to the board.
        allowed - the ALLOW_FLAGS the operator gave.
        context - the shared context.
        milestone - the milestone the board claims (run()), or None.

    Returns:
        The case's Results (one per request; one SKIP when the gate or the context stops it).
    """
    missing = ['--allow-' + flag.replace('_', '-') for flag in case.allow if flag not in allowed]
    if missing:
        return [Result(case.tier, case.name, '', SKIP, 'needs ' + ' '.join(missing), note=case.note)]
    try:
        requests = case.request(context) if callable(case.request) else case.request
    except Blocked as error:
        return [Result(case.tier, case.name, '', SKIP, str(error), note=case.note)]
    except KeyError as error:
        return [Result(case.tier, case.name, '', SKIP, 'blocked: no %s from an earlier case' % error, note=case.note)]
    if isinstance(requests, str):
        return [_judge(case, case.name, requests, transport, context, milestone)]
    return [_judge(case, '%s[%s]' % (case.name, request), request, transport, context, milestone)
            for request in requests]


def _invariant(check: typing.Callable, reply: _Reply, context: dict) -> str | None:
    """
    Run one invariant; one that raises reports the exception as the problem instead of stopping the run.

    Returns:
        The invariant's problem, or None.
    """
    try:
        return check(reply, context)
    except Exception as error:
        return 'invariant %s raised %r' % (getattr(check, '__qualname__', check), error)


def _judge(case: Case, name: str, request: str, transport: transports.Transport, context: dict,
           milestone: int | None) -> Result:
    """
    Send one request and judge the answer: envelope, accepted answers with their invariants, then not-yet.

    Args:
        case - the case.
        name - the result's name (the case's, or with the request for a per-object case).
        request - the request line.
        transport - the transport to the board.
        context - the shared context; a passing case's remember hook writes to it.
        milestone - the milestone the board claims: a not-yet answer to a case it owes is a FAIL.

    Returns:
        The Result.
    """

    def result(outcome: str, reason: str = '', free: tuple = ()) -> Result:
        """This request's Result, carrying the reply lines received so far."""
        return Result(case.tier, name, request, outcome, reason, tuple(lines), free, case.note)

    lines = []
    try:
        lines = transport.exchange(request)
    except transports.Unreachable as error:
        return result(SKIP, 'unreachable on %s: %s' % (transport.name, error))
    except (transports.NoAnswer, OSError) as error:  # OSError: the port or socket itself failed (a USB unplug)
        return result(FAIL, 'no answer: %s' % error)
    answered = time.monotonic()
    problem = _envelope(request, lines)
    if problem is not None:
        return result(FAIL, problem)
    if not lines:
        return result(PASS)
    message = cc.parse(lines[0])
    problems = []
    for alternative in case.replies:
        if alternative[0] != message.command:
            continue
        free = tuple(position for position, spec in enumerate(alternative[1:]) if spec is TEXT)
        problem, values = _match(alternative, message.args)
        if problem is None:
            reply = _Reply(request, message.command, values)
            problem = next(filter(None, (_invariant(check, reply, context) for check in case.invariants)), None)
            if problem is None:
                if case.remember is not None:
                    case.remember(reply, context)
                if case.reboots:
                    problem = _await_new_boot(transport, context, answered)
                    if problem is not None:
                        return result(FAIL, problem, free)
                return result(PASS, '', free)
        problems.append(problem)
    for answer in case.not_yet:
        if answer.status != message.command:
            continue
        if message.args[:1] != [answer.code]:
            continue
        if milestone is not None and case.milestone <= milestone:
            return result(FAIL, 'owed from M%d, and the board claims M%d: %s' % (case.milestone, milestone,
                                                                                 _render(lines[0])))
        return result(NOT_YET, '%s: %s' % (answer.reason, _render(lines[0])))
    if problems:
        return result(FAIL, '%s (got %s)' % ('; '.join(problems), _render(lines[0])))
    expected = ' | '.join(_describe(alternative) for alternative in case.replies) or 'no reply'
    return result(FAIL, 'expected %s, got %s' % (expected, _render(lines[0])))


def _boot_of(lines: list) -> tuple | None:
    """
    The (boot_id, uptime) a whoami answer reports.

    Args:
        lines - the reply lines to a whoami.

    Returns:
        (boot_id, uptime ms), or None when the answer is no single `iam` carrying an integer uptime.
    """
    try:
        message = cc.parse(lines[0]) if len(lines) == 1 else None
        payload = json.loads(message.args[-1]) if message is not None and message.command == 'iam' else None
    except (ValueError, IndexError):
        return None
    if not isinstance(payload, dict) or type(payload.get('uptime')) is not int:
        return None
    return payload.get('boot_id'), payload['uptime']


def _await_new_boot(transport: transports.Transport, context: dict, answered: float) -> str | None:
    """
    Prove that an `ok` to reboot really reset the board: a whoami from a new boot answers.

    An answer alone proves nothing: a board answers until its reset fires (200 ms after the `ok`), and one that
    never resets answers forever. A new boot is a boot_id other than the preflight's, or an uptime shorter than
    the time since the `ok` -- the clock a reset restarts, and the proof where NVS counted no boot (a null
    boot_id both times).

    Args:
        transport - the transport to the board.
        context - the shared context (the preflight's boot_id).
        answered - time.monotonic() when the `ok` arrived.

    Returns:
        None once a new boot answered; else the problem.
    """
    try:
        transport.await_reboot()
    except (transports.NoAnswer, OSError) as error:
        return 'rebooted but never came back: %s' % error
    deadline = time.monotonic() + _REBOOT_TIMEOUT_S
    seen = 'no whoami answered'
    while True:
        try:
            boot = _boot_of(transport.exchange('whoami'))
        except (transports.NoAnswer, OSError):
            boot = None
        if boot is not None:
            boot_id, uptime = boot
            since_ms = (time.monotonic() - answered) * 1000
            if boot_id != context['boot_id'] or uptime < since_ms:
                return None
            seen = 'boot_id still %r, uptime %d ms against %.0f ms since the ok' % (boot_id, uptime, since_ms)
        if time.monotonic() >= deadline:
            return 'answered ok to reboot but never reset within %.0f s (%s)' % (_REBOOT_TIMEOUT_S, seen)
        time.sleep(_REBOOT_POLL_S)


def _hub_checks(transport: transports.Transport, context: dict, beats: int) -> list:
    """
    The hub's own view of the board (GET /api/boards): its dashboard row, online across heartbeats, a set clock.

    What the port plan's M1 proof asks of the hub that a request line cannot show. Not automated, and checked
    by hand: offline after 3 missed beats (the board must go dark), the dashboard's sync buttons, and the
    hub's `clock set` log line at a cold board's registration. The registration itself is the Hub transport's
    precondition: it refuses a board the hub does not list online.

    Args:
        transport - the transport; only the Hub has a row().
        context - the shared context (the whoami epoch, for the clock check's diagnosis).
        beats - the heartbeats to watch.

    Returns:
        Results of the tier `hub`: the row, the beats, the clock; one SKIP off the hub.
    """
    row = getattr(transport, 'row', None)
    if row is None:
        return [Result('hub', 'hub', '', SKIP, 'needs the hub transport (%s has no hub)' % transport.name)]
    rows = []
    problem = None
    for beat in range(1, beats + 1):
        try:
            current = row()
        except transports.NoAnswer as error:
            problem = 'the hub stopped answering at beat %d/%d: %s' % (beat, beats, error)
            break
        if current is None or not current.get('online'):
            problem = 'the board was offline at beat %d/%d' % (beat, beats)
            break
        if current.get('stale'):
            problem = 'the health was stale at beat %d/%d (health_age %s s)' % (beat, beats, current.get('health_age'))
            break
        rows.append(current)
        if beat < beats:
            time.sleep(_HUB_BEAT_S)
    if problem is None and beats > 1:
        first, last = rows[0].get('uptime'), rows[-1].get('uptime')
        if not (type(first) is int and type(last) is int and last > first):
            problem = 'the uptime went %r -> %r across %d beats: the heartbeat is not refreshing health' % (
                first, last, beats)
    online = Result('hub', 'hub_online_across_beats', 'GET /api/boards', FAIL if problem else PASS, problem or '')
    if not rows:
        blocked = 'no row was read'
        return [online, Result('hub', 'hub_row_filled', '', SKIP, blocked),
                Result('hub', 'hub_clock_synced', '', SKIP, blocked)]
    return [online, _row_filled(rows[-1]), _clock_synced(rows[-1], context)]


def _row_filled(row: dict) -> Result:
    """hub_row_filled: every _HUB_ROW field of the dashboard row carries a value (whoami and health fed it)."""
    empty = [field for field in _HUB_ROW if row.get(field) is None]
    if empty:
        return Result('hub', 'hub_row_filled', 'GET /api/boards', FAIL, 'the dashboard row has no %s' % empty)
    return Result('hub', 'hub_row_filled', 'GET /api/boards', PASS, '')


def _clock_synced(row: dict, context: dict) -> Result:
    """
    hub_clock_synced: a board online in setting does not keep a clock in 2000 while CC's own clock is set.

    The hub sets a cold board's clock at registration only when the whoami epoch is a JSON integer within 2000
    and the stage is setting (server.Server sync on connect). A board left in 2000 means one of those failed.
    """
    request = 'GET /api/boards'
    if time.time() < _CC_CLOCK_VALID_FROM:
        return Result('hub', 'hub_clock_synced', request, SKIP, 'the CC clock reads before 2020: the hub syncs nothing')
    if row.get('stage') != 'setting':
        return Result('hub', 'hub_clock_synced', request, SKIP, 'the hub syncs in setting only')
    clock = row.get('clock')
    year = clock[:4] if isinstance(clock, str) else ''
    if not year.isdigit():
        return Result('hub', 'hub_clock_synced', request, FAIL, 'clock %r is not an ISO date' % (clock,))
    if int(year) > _UNSET_CLOCK_YEAR:
        return Result('hub', 'hub_clock_synced', request, PASS, '')
    epoch = context.get('epoch')
    why = ('its whoami epoch is %s, not a JSON integer' % _type_name(epoch) if type(epoch) is not int
           else 'its whoami looked syncable -- see the hub log for the update mission reply')
    return Result('hub', 'hub_clock_synced', request, FAIL, 'the board still reads %s: %s' % (clock, why))
