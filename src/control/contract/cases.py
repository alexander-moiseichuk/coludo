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
  * `named` -- the `name=value` figures a GCL reply carries (doc/specs/cc-protocol.md "The Glider Command Language"),
    {name: regular expression} a value must match whole (a name ending in '?' is optional; names the case does not
    give are allowed). Only a case that gives `named` accepts a reply with named params: a CC reply carries none.
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
# The Rust board's gate (doc/specs/avionics-port.md §6): every change runs only in SETTING, refused at DONE too but
# for a barometer's re-zero, and the text says so. Each board answers its own true text, so the gating cases accept
# either.
_REFUSED_OUTSIDE_SETTING: str = 'refused -- stage is %s (changes only in setting)' % _HOLD_STAGE
# An arm while held is refused under `stage` with plan B's stage problem (cc_client._stage_problem), which the Rust
# board carries verbatim
_HELD_STAGE_PROBLEM: str = ('stage held at %s by an operator command -- `stage setting` to return to the ground'
                            % _HOLD_STAGE)
# The Rust board's bus diagnostics (M9; plan B gates them on the stage only) run only in SETTING while disarmed: an
# armed board refuses each with this text before it reads the arguments, and `bustune` but back to the bus's own clock
_REFUSED_ARMED: str = 'refused -- armed (bus diagnostics only while disarmed)'
_BUSTUNE_ONLY_BACK: str = 'refused -- armed (bustune only back to the configured %d Hz)'
"""
The BMP581 round trip's ground step. Well under half the sequencer's launch_alt_m (10 m by default), so neither the
step nor the re-zero's way back reads as a launch's rise or raises `sources-disagree: elevation`, and far above the
part's noise (sd 0.07 m): a ground that moved reads apart from one that did not by over 1 m.
"""
_GROUND_STEP_M: float = 2.0
_GROUND_TOLERANCE_M: float = 0.5  # a ground read (altitude - elevation, two channels) against the one written
_CONVERSION_SETTLE_S: float = 0.1  # five BMP581 conversions (50 Hz): its elevation reads a ground written before it
_DECIMAL: re.Pattern = re.compile(r'-?\d+\.\d\d')  # plan B fixed.to_str: an angle in centidegrees as text, '-6.49'
_ACCEL_DISAGREE_DEGREES: float = 10.0  # health's sources-disagree for two accelerometers at rest: past this
# health's `sources-disagree: accel <a>/<b> <x.y> deg` and `... elevation <a>/<b> <x.y> m`
_DISAGREEMENT: re.Pattern = re.compile(r'sources-disagree: (accel|elevation) (\S+)/(\S+) (\d+\.\d) (deg|m)')
_DISAGREEMENT_UNIT: dict = {'accel': 'deg', 'elevation': 'm'}
_BOUND: re.Pattern = re.compile(r'\[([^\]]*)\]')  # a slot's sources in `inspect sequencer`'s binding text
# an ignored source in `inspect sequencer`: its bring-up line, the source's name first
_IGNORED: re.Pattern = re.compile(r"([A-Za-z0-9_]+) ignored: the stage machine's [a-z]+ slots are full \(\d+\)")
"""
`calibrate <BMM350>` on a board lying still: no field change seen (the span under the part's minimum), too few
octants of a level turn covered, or -- fewer samples than the calibration needs -- the outstanding instruction.
"""
_STILL_REFUSAL: re.Pattern = re.compile(r'(no field change seen|refused -- only \d of 8 sectors covered|'
                                        r'turn the airframe).*')
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
    named: dict | None = None  # a GCL reply's figures, {name: regex}; None: no named params allowed


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
    named: dict = dataclasses.field(default_factory=dict)  # a GCL reply's name=value figures, as text

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


def _named(spec: dict, named: dict) -> str | None:
    """
    Check a GCL reply's name=value figures against a case's `named` spec.

    Args:
        spec - {name: regular expression}; a name ending in '?' is optional.
        named - the reply's figures, as text.

    Returns:
        The problem, or None: every required name present, every value given its spec matching whole.
    """
    for key, pattern in spec.items():
        name = key.rstrip('?')
        if name not in named:
            if key.endswith('?'):
                continue
            return 'no %s= figure' % name
        if re.fullmatch(pattern, named[name]) is None:
            return '%s=%r does not match %r' % (name, _cut(named[name]), pattern)
    return None


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


def _envelope(request: str, lines: list, named: bool = False) -> str | None:
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
    if message.named and not named:
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


def _refused_for_the_hold(reply: _Reply, context: dict) -> str | None:
    """
    An arm while held is refused for the hold: `stage` carries plan B's held-stage problem, every problem is text.

    The other refusal text, `stage is gliding, not setting`, would mean the board lost the hold's manual flag.
    """
    stage = reply.payload['stage']
    if stage != _HELD_STAGE_PROBLEM:
        return 'stage reads %r, expected %r' % (stage, _HELD_STAGE_PROBLEM)
    return _texts(reply.payload, 'problems')


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


def _readiness_from_plan_b(reply: _Reply, context: dict) -> str | None:
    """verify's readiness names only checks plan B makes (_READINESS_CHECKS): a board adds no readiness of its own."""
    unknown = sorted(set(reply.payload['readiness']) - set(_READINESS_CHECKS))
    if not unknown:
        return None
    return 'readiness names %s, which plan B never reports (%s)' % (unknown, ', '.join(_READINESS_CHECKS))


def _names_itself(reply: _Reply, context: dict) -> str | None:
    """A device's snapshot carries its own name (plan B Task.inspect)."""
    target = cc.parse(reply.request).args[0]
    return None if reply.payload['name'] == target else 'inspect %s names %r' % (target, reply.payload['name'])


def _changed_ground(reply: _Reply, context: dict) -> str | None:
    """A ground set or a re-zero changes exactly the ground."""
    changed = reply.payload['changed']
    return None if changed == ['ground'] else 'changed %s, expected [ground]' % changed


def _succeeded(reply: _Reply, context: dict) -> str | None:
    """`calibrate <device>` / `probe <device>` answer that one device with null: done, or healthy."""
    target = cc.parse(reply.request).args[0]
    answer = reply.values[0]
    return None if answer == {target: None} else '%s answered %s, expected {%r: null}' % (
        cc.parse(reply.request).command, answer, target)


def _rung_passed(reply: _Reply, context: dict) -> str | None:
    """A rung at the configured clock passes: the IMU answers 'ok' and all_ok is true."""
    devices, all_ok = reply.payload['devices'], reply.payload['all_ok']
    if devices.get(_device(context, 'lsm6dso32')) != 'ok' or all_ok is not True:
        return 'the configured clock failed: devices %s, all_ok %r' % (devices, all_ok)
    return None


def _counts(value: object, length: int) -> bool:
    """Whether a value is a list of `length` counts: integers from 0."""
    return (isinstance(value, list) and len(value) == length
            and all(_is_type(item, 'int') and item >= 0 for item in value))


def _configured_address(reply: _Reply, context: dict) -> str | None:
    """
    A part's `addr` is its config entry's address as plan B prints it (hex(addr): '0x69'); an entry with none leaves
    it to the driver's datasheet default, which the contract does not know.
    """
    name = reply.payload['name']
    entries = context['running'].get('sensors', []) + context['running'].get('components', [])
    entry = next((entry for entry in entries if entry.get('name') == name), {})
    if not isinstance(entry.get('addr'), int):
        return None
    expected = hex(entry['addr'])
    return None if reply.payload['addr'] == expected else 'addr is %r, but the config puts %s at %s' % (
        reply.payload['addr'], name, expected)


def _triples(*keys: str) -> typing.Callable:
    """An invariant: each of the payload's `keys` is null (stale) or three numbers (x, y, z)."""

    def check(reply: _Reply, context: dict) -> str | None:
        """Every key named is null or [x, y, z] numbers."""
        for key in keys:
            value = reply.payload[key]
            if value is not None and not (len(value) == 3 and all(_is_type(item, 'number') for item in value)):
                return '%s is %r, expected null or [x, y, z] numbers' % (key, value)
        return None

    return check


def _calib_stat(value: object) -> bool:
    """Whether a value is the BNO055's CALIB_STAT as plan B lists it: [sys, gyr, acc, mag], each 0..3."""
    return isinstance(value, list) and len(value) == 4 and all(
        _is_type(field, 'int') and 0 <= field <= 3 for field in value)


def _bno055_state(reply: _Reply, context: dict) -> str | None:
    """
    The BNO055's calibration adds up (plan B bno055.inspect): the live CALIB_STAT null before its first read, else four
    levels 0..3; a profile restored from NVS sets the convergence latch.
    """
    part = reply.payload
    if part['calibration'] is not None and not _calib_stat(part['calibration']):
        return 'calibration is %r, expected null or [sys, gyr, acc, mag] in 0..3' % (part['calibration'],)
    if part['calibration_restored'] and not part['calibrated']:
        return 'calibration_restored without calibrated: a restored profile sets the latch'
    return None


def _imu_calibration(reply: _Reply, context: dict) -> str | None:
    """health's `imu_calibration`, when the board carries it: the BNO055's CALIB_STAT, four levels 0..3."""
    value = reply.payload.get('imu_calibration')
    if value is not None and not _calib_stat(value):
        return 'imu_calibration is %r, expected [sys, gyr, acc, mag] in 0..3' % (value,)
    return None


def _figure_eight_or_saved(reply: _Reply, context: dict) -> str | None:
    """
    `calibrate <bno055>`: null once the learned profile is saved, else what is still owed, the figure-8 (plan B
    bno055.calibrate).
    """
    for name, answer in reply.payload.items():
        if answer is not None and not answer.startswith('move the airframe'):
            return '%s answered %r: neither saved (null) nor the figure-8 owed' % (name, answer)
    return None


def _gnss_figures(reply: _Reply, context: dict) -> str | None:
    """The GNSS snapshot adds up: a fix has a position, the C/N0 lists the four best, no more used than in view."""
    gnss = reply.payload
    if gnss['fix'] and gnss['position'] is None:
        return 'a fix with no position'
    if len(gnss['cn0']) != 4:
        return 'cn0 is %r, expected the four best' % (gnss['cn0'],)
    if gnss['in_view'] and gnss['used'] > gnss['in_view']:
        return 'used %d satellites with %d in view' % (gnss['used'], gnss['in_view'])
    return None


def _distinct_slots(reply: _Reply, context: dict) -> str | None:
    """`ota status`: the next slot is not the running one (an OTA never writes the image it runs)."""
    if reply.named['next'] == reply.named['running']:
        return 'next=%s is the running slot' % reply.named['next']
    return None


def _bmi323_evidence(reply: _Reply, context: dict) -> str | None:
    """
    The BMI323's evidence adds up: `interrupt_silent` is null exactly when no INT1 line is declared (`interrupt`
    false), a measured ODR is a rate, and the counters count from 0 -- the INT1 runs, the lapses of each watched
    channel ({accel, rate}), and the raced and identical halves ([accel, rate]).
    """
    imu = reply.payload
    if imu['interrupt'] != (imu['interrupt_silent'] is not None):
        return ('interrupt is %r but interrupt_silent is %r: silent is null exactly when no INT1 is declared'
                % (imu['interrupt'], imu['interrupt_silent']))
    if imu['odr_hz'] is not None and imu['odr_hz'] <= 0:
        return 'odr_hz %r is no rate' % imu['odr_hz']
    if imu['irq_runs'] < 0:
        return 'irq_runs %d is negative' % imu['irq_runs']
    lapses = imu['lapses']
    if sorted(lapses) != ['accel', 'rate'] or not _counts(list(lapses.values()), 2):
        return 'lapses is %r, expected {accel: count, rate: count}' % (lapses,)
    for key in ('raced', 'identical'):
        if not _counts(imu[key], 2):
            return '%s is %r, expected [accel, rate] counts' % (key, imu[key])
    return None


def _bmp581_guard(reply: _Reply, context: dict) -> str | None:
    """The BMP581's double-read guard counts from 0: the torn pairs and the implausible reads since boot."""
    for key in ('torn', 'implausible'):
        if reply.payload[key] < 0:
            return '%s %d is negative' % (key, reply.payload[key])
    return None


def _bmm350_evidence(reply: _Reply, context: dict) -> str | None:
    """
    The BMM350's calibration evidence adds up: its span [x_min, x_max, y_min, y_max] in counts (null before any
    field), ordered; 0 to 8 octants of the level turn covered; a sample count from 0.
    """
    magnetometer = reply.payload
    span = magnetometer['span']
    if span is not None and not (len(span) == 4 and all(_is_type(item, 'int') for item in span)
                                 and span[0] <= span[1] and span[2] <= span[3]):
        return 'span is %r, expected [x_min, x_max, y_min, y_max] counts' % (span,)
    if not 0 <= magnetometer['sectors'] <= 8:
        return 'sectors %d is outside 0..8' % magnetometer['sectors']
    if magnetometer['samples'] < 0:
        return 'samples %d is negative' % magnetometer['samples']
    return None


def _angles_are_decimals(reply: _Reply, context: dict) -> str | None:
    """inspect attitude carries its angles and its magnetic offset as plan B's fixed.to_str text: '-6.49' degrees."""
    for key in ('roll', 'pitch', 'heading', 'mag_offset'):
        if not _DECIMAL.fullmatch(reply.payload[key]):
            return '%s %r is not a decimal with two places' % (key, reply.payload[key])
    return None


def _databoard_sources(reply: _Reply, context: dict) -> str | None:
    """stats databoard maps each quantity to its sources: distinct names, sorted (plan B Databoard.stats)."""
    for quantity, sources in reply.payload.items():
        if not isinstance(sources, list) or not all(isinstance(source, str) for source in sources):
            return '%s is %r, expected a list of source names' % (quantity, sources)
        if sources != sorted(set(sources)):
            return '%s sources %s are not sorted and distinct' % (quantity, sources)
    return None


def _databoard_quantities(reply: _Reply, context: dict) -> str | None:
    """
    inspect databoard: each quantity {value, source, age_ms, offsets} (plan B Databoard.inspect). A fused value has
    the source it came from and an age; none fresh reads value, source and age null. Where stats databoard was read,
    the quantities are its own, and a source or an offset's key is one of that quantity's sources.
    """
    sources = context.get('databoard_sources')
    for quantity, entry in reply.payload.items():
        problem = _shape(_QUANTITY, entry, 1)
        if problem is not None:
            return '%s: %s' % (quantity, problem)
        if entry['source'] is None and (entry['value'] is not None or entry['age_ms'] is not None):
            return '%s has a value or an age but no source' % quantity
        if entry['source'] is not None and not (_is_type(entry['age_ms'], 'int') and entry['age_ms'] >= 0):
            return '%s from %s has no age (%r)' % (quantity, entry['source'], entry['age_ms'])
        if sources is None:
            continue
        known = sources.get(quantity, [])
        if entry['source'] is not None and entry['source'] not in known:
            return '%s comes from %s, which stats databoard does not list (%s)' % (quantity, entry['source'], known)
        if not set(entry['offsets']) <= set(known):
            return '%s offsets %s name sources stats databoard does not list (%s)' % (
                quantity, sorted(entry['offsets']), known)
    if sources is not None and set(sources) != set(reply.payload):
        return 'the quantities differ from stats databoard: %s' % sorted(set(sources) ^ set(reply.payload))
    return None


def _nothing_written(reply: _Reply, context: dict) -> str | None:
    """The databoard is read-only: an update changes nothing (a fused value comes from its sources alone)."""
    if reply.status != 'ok' or reply.payload['changed'] == []:
        return None  # plan B raises at the update (err internal): nothing written either
    return 'the databoard took %s' % reply.payload['changed']


def _sequencer_binding(reply: _Reply, context: dict) -> str | None:
    """
    inspect sequencer: its binding (`sources`) and its ignored sources name parts the board lists; edges from 0. An
    ignored source is its bring-up line, `<source> ignored: the stage machine's <quantity> slots are full (<n>)`
    (crates/sequencer `Ignored`).
    """
    snapshot = reply.payload
    ignored = snapshot['ignored']
    if not all(isinstance(item, str) for item in ignored):
        return 'ignored holds a non-text item: %s' % ignored
    malformed = [item for item in ignored if _IGNORED.fullmatch(item) is None]
    if malformed:
        return 'ignored %r is not `<source> ignored: the stage machine\'s <quantity> slots are full (<n>)`' % malformed
    bound = [name.strip() for group in _BOUND.findall(snapshot['sources']) for name in group.split(',')
             if name.strip()]
    unknown = sorted(set(bound + [_IGNORED.fullmatch(item).group(1) for item in ignored]) - set(context['objects']))
    if unknown:
        return 'the sequencer names %s, which the board does not list' % unknown
    return None if snapshot['veto_edges'] >= 0 else 'veto_edges %d is negative' % snapshot['veto_edges']


def _disagreements_named(reply: _Reply, context: dict) -> str | None:
    """
    health's M9 items name what they are about. `sources-disagree: accel <a>/<b> <x.y> deg` (two accelerometers at
    rest more than 10 degrees apart) and `sources-disagree: elevation <a>/<b> <x.y> m` (two barometers' elevations
    apart) name two different sources of that quantity -- the databoard's, where stats databoard was read.
    `attitude-blind` comes only from a board that lists an attitude filter, `attitude-backup` only from one where
    something provides attitude.
    """
    sources, objects = context.get('databoard_sources'), context.get('objects')
    for item in reply.payload['degraded']:
        if item.startswith('sources-disagree'):
            match = _DISAGREEMENT.fullmatch(item)
            if match is None:
                return '%r is not `sources-disagree: <accel|elevation> <a>/<b> <x.y> <deg|m>`' % item
            quantity, first, second, amount, unit = match.groups()
            if unit != _DISAGREEMENT_UNIT[quantity]:
                return '%r: %s apart is in %s' % (item, quantity, _DISAGREEMENT_UNIT[quantity])
            if first == second:
                return '%r names one source twice' % item
            if sources is not None and not {first, second} <= set(sources.get(quantity, [])):
                return '%r names a source the databoard does not list for %s (%s)' % (item, quantity,
                                                                                      sources.get(quantity, []))
            if quantity == 'accel' and float(amount) < _ACCEL_DISAGREE_DEGREES:
                return '%r: accelerometers within %.0f degrees agree' % (item, _ACCEL_DISAGREE_DEGREES)
        elif item == 'attitude-blind' and objects is not None and 'attitude' not in objects:
            return 'attitude-blind from a board that lists no attitude filter'
        elif item == 'attitude-backup' and sources is not None and 'attitude' not in sources:
            return 'attitude-backup from a board where nothing provides attitude'
    return None


def _ground_reads(expected: typing.Callable) -> typing.Callable:
    """An invariant: the snapshot's ground (altitude - elevation) reads expected(context), within the tolerance."""

    def check(reply: _Reply, context: dict) -> str | None:
        """altitude_m - elevation_m is expected(context) within _GROUND_TOLERANCE_M."""
        altitude, elevation = reply.payload['altitude_m'], reply.payload['elevation_m']
        if not (_is_type(altitude, 'number') and _is_type(elevation, 'number')):
            return 'no fresh altitude and elevation to read the ground from'
        ground, wanted = altitude - elevation, expected(context)
        if abs(ground - wanted) <= _GROUND_TOLERANCE_M:
            return None
        return 'the ground reads %.3f m (altitude - elevation), expected %.3f m' % (ground, wanted)

    return check


def _still_board_refusal(reply: _Reply, context: dict) -> str | None:
    """
    A board lying still never calibrates its magnetometer: `calibrate <BMM350>` answers that one device with the
    refusal or the outstanding instruction -- text, never null (which would mean a turn it never made was taken).
    '' is plan B's instruction once a calibration is in force and too few samples came since boot.
    """
    target = cc.parse(reply.request).args[0]
    answer = reply.values[0]
    if sorted(answer) != [target]:
        return 'calibrate %s answered for %s' % (target, sorted(answer))
    verdict = answer[target]
    if not isinstance(verdict, str):
        return 'a still board answered %s: %r -- a calibration from no turn' % (target, verdict)
    if verdict and not _STILL_REFUSAL.fullmatch(verdict):
        return 'a still board answered %r: neither a refusal nor the instruction' % verdict
    return None


def _refused_back_to_its_clock(reply: _Reply, context: dict) -> str | None:
    """An armed board refuses a rung off the IMU bus's clock, naming the one it would run: the configured clock."""
    expected = _BUSTUNE_ONLY_BACK % _imu_bus(context)[1]
    return None if reply.values[1] == expected else 'the refusal reads %r, expected %r' % (reply.values[1], expected)


"""Not-yet answers: legitimate for a board that has not grown the feature."""


_PUSH_NOT_PORTED: _NotYet = _NotYet('err', 'badcmd', 'push-* is not ported: the Rust board\'s firmware OTA is '
                                                     '`ota` (M15)')
# A case no milestone owes: plan B's answer, which the port replaced by decision (doc/specs/avionics-port.md §6)
_NEVER: int = 1_000_000
_SAVED_NOT_KNOWN: _NotYet = _NotYet('err', 'badargs', 'a firmware older than `get-config saved`')
_NOT_IN_PLAN_B: _NotYet = _NotYet('err', 'badcmd', 'a Rust-only bus diagnostic (M9): plan B has no such command')
_GCL_ONLY: _NotYet = _NotYet('err', 'badcmd', 'a GCL command this build does not register')

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


def _while_held(request: object) -> typing.Callable:
    """A request (a line, or a builder) sent only while the hold took."""

    def gated(context: dict) -> str:
        """The request, once the hold took."""
        if not context.get('held'):
            raise Blocked('the stage hold did not take')
        return request(context) if callable(request) else request

    return gated


def _return_to_ground(context: dict) -> str:
    """`stage setting` (back on the ground, sequencer free): sent whenever the hold went out."""
    if not context.get('hold_sent'):
        raise Blocked('nothing to undo: the hold was not sent')
    return 'stage setting'


def _arm(context: dict) -> str:
    """
    `arm`; marks it, so the disarm after it runs whatever it answered, and the board counts as armed till then.

    Whether it took is _arm_outcome's to say: until then the armed checks are blocked.
    """
    context['arm_sent'] = True
    context['disarmed'] = False
    context['armed_now'] = False
    context['arm_outcome'] = 'the arm answered outside the contract'
    return 'arm'


def _arm_outcome(reply: _Reply, context: dict) -> None:
    """
    Remember whether the arm took: the armed checks run only while it holds. A refusal becomes the reason they are
    skipped -- a board's own config with entries not ported yet does not arm; a bench copy without them does (M7).
    """
    if reply.status == 'ok':
        context['armed_now'] = True
        return
    context['arm_outcome'] = ('the board did not arm (%s) -- the armed checks need a config whose verify passes, '
                              'e.g. a bench copy without the not-ported entries' % ', '.join(sorted(reply.payload)))


def _while_armed(request: object) -> typing.Callable:
    """A request (a line, or a builder) sent only while the arm case's arm holds: it armed, and no disarm since."""

    def gated(context: dict) -> str:
        """The request, while the board is armed."""
        if not context.get('armed_now') or context.get('disarmed'):
            raise Blocked(context.get('arm_outcome', 'no arm was sent'))
        return request(context) if callable(request) else request

    return gated


def _disarm_after_arm(context: dict) -> str:
    """`disarm`, whenever an arm went out."""
    if not context.get('arm_sent'):
        raise Blocked('no arm was sent')
    return 'disarm'


def _arm_while_held(context: dict) -> str:
    """
    `arm` while the hold took; marked as _arm marks it, and as sent while held.

    Held at gliding, an armed board flies its control loop on the fins. A board that arms here anyway is disarmed
    by the case after this one, or by the undo of a run that stops.
    """
    if not context.get('held'):
        raise Blocked('the stage hold did not take')
    context['held_arm_sent'] = True
    return _arm(context)


def _disarm_after_held_arm(context: dict) -> str:
    """`disarm`, whenever an arm went out while held: before the board returns to the ground."""
    if not context.get('held_arm_sent'):
        raise Blocked('no arm was sent while held')
    return 'disarm'


def _reboot(context: dict) -> str:
    """`reboot`, once whoami told which boot it ends: the boot id a new boot must differ from."""
    if 'boot_id' not in context:
        raise Blocked('whoami was unreadable: nothing could prove the board really reset')
    return 'reboot'


def _inspect_each(context: dict) -> list:
    """`inspect <name>` for every object the board lists."""
    return [cc.build('inspect', [name]) for name in context['objects']]


def _device(context: dict, driver: str) -> str:
    """
    The name of the device with this driver that came up: a config entry (sensors or components) the board
    lists as an object.

    Raises:
        Blocked - none came up: a board without the part, or plan B on CPython, which sets no device up.
    """
    entries = context['running'].get('sensors', []) + context['running'].get('components', [])
    for entry in entries:
        if entry.get('driver') == driver and entry.get('name') in context['objects']:
            return entry['name']
    raise Blocked('no %s device came up (plan B on CPython sets none up)' % driver)


def _on_device(command: str, driver: str, properties: dict | None = None) -> typing.Callable:
    """A request: `<command> <the device with this driver>`, with its properties as JSON when given."""

    def request(context: dict) -> str:
        """The line, once the device is known."""
        arguments = [_device(context, driver)]
        if properties is not None:
            arguments.append(json.dumps(properties))
        return cc.build(command, arguments)

    return request


def _ground_write(context: dict) -> str:
    """`update <barometer> {"ground": 100}`; marks it, so the re-zero after it (or the undo) puts a live ground back."""
    line = _on_device('update', 'icp10111', {'ground': 100})(context)
    context['ground_written'] = True
    return line


def _probe_devices(context: dict) -> list:
    """`probe <device>` for each M6 part that came up."""
    names = []
    for driver in _M6_DRIVERS:
        try:
            names.append(_device(context, driver))
        except Blocked:
            continue
    if not names:
        raise Blocked('none of %s came up (plan B on CPython sets none up)' % ', '.join(_M6_DRIVERS))
    return [cc.build('probe', [name]) for name in names]


def _gcl(line: str) -> typing.Callable:
    """
    A request: a GCL line (doc/specs/cc-protocol.md "The Glider Command Language"), sent only to a board that speaks
    GCL -- its `help` lists commands (the preflight's `gcl`). Plan B answers `help` badcmd and has none of them.
    """

    def request(context: dict) -> str:
        """The line, on a GCL board."""
        if not context.get('gcl'):
            raise Blocked('a GCL command: this board speaks no GCL (plan B answers help badcmd)')
        return line

    return request


def _on_any_device(command: str, drivers: tuple) -> typing.Callable:
    """A request: `<command> <the first device that came up with one of these drivers>`."""

    def request(context: dict) -> str:
        """The line, once one of the devices is known."""
        for driver in drivers:
            try:
                return cc.build(command, [_device(context, driver)])
            except Blocked:
                continue
        raise Blocked('none of %s came up (plan B on CPython sets none up)' % ', '.join(drivers))

    return request


def _probe_sen0253(context: dict) -> list:
    """`probe <device>` for each SEN0253 part (M18: the v0.1 boards' BNO055 and BMP280) that came up."""
    names = []
    for driver in _M18_DRIVERS:
        try:
            names.append(_device(context, driver))
        except Blocked:
            continue
    if not names:
        raise Blocked('none of %s came up (plan B on CPython sets none up)' % ', '.join(_M18_DRIVERS))
    return [cc.build('probe', [name]) for name in names]


def _probe_sen0697(context: dict) -> list:
    """`probe <device>` for each SEN0697 part (M9) that came up."""
    names = []
    for driver in _M9_DRIVERS:
        try:
            names.append(_device(context, driver))
        except Blocked:
            continue
    if not names:
        raise Blocked('none of %s came up (plan B on CPython sets none up)' % ', '.join(_M9_DRIVERS))
    return [cc.build('probe', [name]) for name in names]


def _on_object(command: str, name: str, properties: dict | None = None) -> typing.Callable:
    """A request: `<command> <name>` (with its properties as JSON when given), once the board lists the object."""

    def request(context: dict) -> str:
        """The line, once `objects` named it."""
        if name not in context['objects']:
            raise Blocked('the board lists no %s (plan B on CPython sets none up)' % name)
        arguments = [name] if properties is None else [name, json.dumps(properties)]
        return cc.build(command, arguments)

    return request


def _i2c_part(context: dict) -> str:
    """
    A part that came up on an I2C bus: the owner a bus diagnostic would hold off its parts.

    Raises:
        Blocked - none came up (plan B on CPython sets none up).
    """
    for entry in context['running'].get('sensors', []) + context['running'].get('components', []):
        if entry.get('bus') == 'i2c' and entry.get('name') in context['objects']:
            return entry['name']
    raise Blocked('no part came up on an I2C bus (plan B on CPython sets none up)')


def _spi_imu(context: dict) -> str:
    """The SPI IMU that `imu mute` silences (raising Blocked when it did not come up)."""
    return _device(context, 'lsm6dso32')


def _with_part(line: str, part: typing.Callable) -> typing.Callable:
    """
    A request: `line`, once part(context) names the part it acts on.

    The bus diagnostics go out bare: an armed or held board refuses them before it reads an argument, and a
    disarmed one in SETTING refuses the missing one -- so the line touches no bus whatever the gate does.
    """

    def request(context: dict) -> str:
        """The line, once the part came up."""
        part(context)
        return line

    return request


def _imu_bus(context: dict) -> tuple:
    """
    The IMU's SPI bus: (its id, its configured baud).

    Raises:
        Blocked - no IMU came up on an SPI bus with a configured clock.
    """
    imu = _device(context, 'lsm6dso32')
    entry = next(entry for entry in context['running'].get('sensors', []) if entry.get('name') == imu)
    spec = context['running']['buses'].get('spi', {}).get(str(entry.get('id')), {})
    if entry.get('bus') != 'spi' or not isinstance(spec.get('baud'), int):
        raise Blocked('%s is not on an SPI bus with a configured baud' % imu)
    return str(entry['id']), spec['baud']


def _imu_bus_at_its_clock(context: dict) -> str:
    """
    `bustune spi <id> <baud>`: the IMU's SPI bus run at its configured clock, a rung that changes nothing.

    Raises:
        Blocked - no IMU came up on an SPI bus with a configured clock.
    """
    identity, baud = _imu_bus(context)
    return cc.build('bustune', ['spi', identity, str(baud)])


def _imu_bus_off_clock(context: dict) -> str:
    """
    `bustune spi <id> <half its baud>`: a rung off the IMU bus's clock, which an armed board refuses; marks it, so
    the rung back to the configured clock follows it (or the undo) on a board whose gate let it run.
    """
    identity, baud = _imu_bus(context)
    context['off_clock_sent'] = True
    return cc.build('bustune', ['spi', identity, str(baud // 2)])


def _bustune_back(context: dict) -> str:
    """The IMU bus back at its configured clock: sent whenever the off-clock rung went out, whatever it answered."""
    if not context.get('off_clock_sent'):
        raise Blocked('nothing to put back: the off-clock rung was not sent')
    return _imu_bus_at_its_clock(context)


def _remember_ground(reply: _Reply, context: dict) -> None:
    """context['bmp581_ground'] = the BMP581's ground (altitude - elevation, m AMSL, to the mm), when both read."""
    altitude, elevation = reply.payload['altitude_m'], reply.payload['elevation_m']
    if _is_type(altitude, 'number') and _is_type(elevation, 'number'):
        context['bmp581_ground'] = round(altitude - elevation, 3)


def _bmp581_ground_write(context: dict) -> str:
    """
    `update <BMP581> {"ground": <the ground read + _GROUND_STEP_M>}`; marks it, so the restore after it (or the
    undo) puts the ground read back.
    """
    if 'bmp581_ground' not in context:
        raise Blocked('the BMP581 read no fresh altitude and elevation: there would be no ground to restore')
    line = _on_device('update', 'bmp581', {'ground': round(context['bmp581_ground'] + _GROUND_STEP_M, 3)})(context)
    context['bmp581_ground_written'] = True
    return line


def _after_ground_write(request: typing.Callable) -> typing.Callable:
    """A request sent only once the BMP581 ground write went out: the restore after it then puts back its change too."""

    def gated(context: dict) -> str:
        """The request, once the write went out."""
        if not context.get('bmp581_ground_written'):
            raise Blocked('the BMP581 ground write was not sent: nothing would restore the ground after this')
        return request(context)

    return gated


def _bmp581_settled(context: dict) -> str:
    """`inspect <BMP581>` once the ground write went out, _CONVERSION_SETTLE_S on: its conversions since read it."""
    if not context.get('bmp581_ground_written'):
        raise Blocked('the BMP581 ground write was not sent')
    line = _on_device('inspect', 'bmp581')(context)
    time.sleep(_CONVERSION_SETTLE_S)
    return line


def _bmp581_ground_restore(context: dict) -> str:
    """`update <BMP581> {"ground": <the ground read>}`: sent whenever the write went out, whatever it answered."""
    if not context.get('bmp581_ground_written'):
        raise Blocked('nothing to restore: the BMP581 ground write was not sent')
    return _on_device('update', 'bmp581', {'ground': context['bmp581_ground']})(context)


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
# every readiness key plan B can report: cc_client._readiness's config checks, and verify's live imu_calibration
_READINESS_CHECKS: tuple = ('watchdog', 'flight', 'gains', 'fins.limit_multiplier', 'fins', 'zone', 'imu_calibration')
_DETECT: dict = {'detected': 'str|null', 'applied': 'str|null', 'detail': 'any'}
_PUSH_STATUS: dict = {'uploading': 'str|null', 'orphans': 'list', 'received?': 'int', 'size?': 'int',
                      'chunks?': 'int'}
_ANY_OBJECT: dict = {}
# The M6 parts' snapshots: plan B Task.inspect (name, ok, healthy) and each driver's own keys, a stale or never-read
# channel null (lsm6dso32.py, icp10111.py, separation.py `inspect`)
_IMU: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'interrupt': 'bool', 'interrupt_silent': 'bool',
              'accel_g': 'list|null', 'rate_dps': 'list|null'}
_BAROMETER: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'altitude_m': 'number|null',
                    'temperature_c': 'number|null', 'pressure_pa': 'number|null', 'elevation_m': 'number|null'}
_SEPARATION: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'separated': 'bool'}
# plan B's bluetooth.py `inspect`: the configured radio, and the radio's actual state (null on a board with no BLE)
_BLUETOOTH: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'radio': 'bool', 'active': 'bool|null'}
_BUSTUNE: dict = {'kind': 'str', 'id': 'str', 'freq': 'int', 'devices': 'object', 'all_ok': 'bool'}
_M6_DRIVERS: tuple = ('lsm6dso32', 'icp10111', 'separation')
"""
The M9 parts' snapshots: plan B's drivers' keys (bmi323.py, bmp581.py, bmm350.py `inspect`; tasks/attitude.py) and
the Rust board's evidence beside them. The BMI323 reports `rate_dps` (deg/s, as the LSM6DSO32) where plan B had
`rate_cds`, a listed divergence; its INT1, measured ODR, lapses and raced/identical halves are the Rust board's. Every
barometer gains its climb rate `change_m_s` (m/s, climb positive), and the BMP581 its double-read guard's counters.
"""
_BMI323: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'addr': 'str', 'interrupt': 'bool',
                 'interrupt_silent': 'bool|null', 'accel_g': 'list|null', 'rate_dps': 'list|null', 'irq_runs': 'int',
                 'odr_hz': 'number|null', 'lapses': 'object', 'raced': 'list', 'identical': 'list'}
_BAROMETER_CHANGE: dict = {**_BAROMETER, 'change_m_s': 'number|null'}
_BMP581: dict = {**_BAROMETER_CHANGE, 'addr': 'str', 'torn': 'int', 'implausible': 'int'}
_BMM350: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'addr': 'str', 'mag_counts': 'list|null',
                 'calibrated': 'bool', 'span': 'list|null', 'samples': 'int', 'sectors': 'int'}
_ATTITUDE: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'free_running': 'bool', 'seeded': 'bool',
                   'blind': 'bool', 'roll': 'str', 'pitch': 'str', 'heading': 'str', 'mag_offset': 'str',
                   'mag_known': 'bool'}
_SEQUENCER: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'sources': 'str', 'ignored': 'list',
                    'veto_edges': 'int'}
_QUANTITY: dict = {'value': 'any', 'source': 'str|null', 'age_ms': 'int|null', 'offsets': 'object'}  # databoard's
_M9_DRIVERS: tuple = ('bmi323', 'bmp581', 'bmm350')
"""
The M10-M18 parts' snapshots: plan B's drivers' keys (sdp810.py, ina226.py, vl53l1x.py, gnss.py, gnss_calib.py,
sg90.py, bno055.py, bmp280.py `inspect`), a stale or never-read channel null. The port's own: the GNSS's `lines`, the
backup barometers' `change` and guard counters (as the BMP581's), the BNO055's `addr`.
"""
_PITOT: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'dynamic_pressure_pa': 'number|null',
                'airspeed_ms': 'number|null', 'temperature_c': 'number|null', 'scale': 'int|null',
                'zero_offset_pa': 'number|null', 'tared': 'bool|null', 'air_density': 'number|null'}
_POWER: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'voltage_mv': 'int|null', 'current_ma': 'int|null',
                'power_mw': 'int|null', 'alerts': 'int'}
_RANGER: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'interrupt': 'bool', 'agl_m': 'number|null'}
_GNSS: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'fix': 'bool', 'satellites': 'int', 'hdop': 'number',
               'fix_quality': 'int', 'position': 'list|null', 'altitude_m': 'number|null',
               'elevation_m': 'number|null', 'speed_ms': 'number|null', 'in_view': 'int', 'used': 'int', 'mode': 'int',
               'cn0': 'list', 'cn0_mean': 'number', 'antenna': 'str', 'lines?': 'int'}
_GNSS_CALIB: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'drift_e': 'number', 'drift_n': 'number',
                     'samples': 'int', 'frozen': 'bool'}
_SERVO: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'angle': 'int', 'pulse_us': 'int', 'trim': 'int',
                'feedback': 'int|null'}
_ABSOLUTE: dict = {'name': 'str', 'ok': 'bool', 'healthy': 'bool', 'addr': 'str', 'attitude_deg': 'list|null',
                   'accel_g': 'list|null', 'fusion_stalled': 'bool', 'calibration': 'list|null', 'calibrated': 'bool',
                   'calibration_restored': 'bool'}
_GNSS_DRIVERS: tuple = ('neo6mv2', 'atgm336h')
_LASER_DRIVERS: tuple = ('vl53l4cx', 'vl53l1x')
_SERVO_DRIVERS: tuple = ('sg90', 'mg90s')
_M18_DRIVERS: tuple = ('bno055', 'bmp280')
"""
The Rust board's own read-only GCL commands (doc/specs/cc-protocol.md "The Glider Command Language"): their figures
as `name=value`, each value a pattern: a count, a ring cursor `<block>/<offset>`, a volume's `<free>/<total>` MB.
"""
_COUNT: str = r'\d+'
_CURSOR: str = r'\d+/\d+'
_DF: dict = {'layout': r'[a-z_]+', 'recordings?': _CURSOR, 'videos?': _CURSOR, 'bounced': _COUNT}
_RING: dict = {'erased': _COUNT, 'saved': _COUNT, 'unsaved': _COUNT, 'lost': _COUNT, 'bad': _COUNT, 'reserve': _COUNT,
               'writer': _CURSOR, 'saver': _CURSOR, 'flash_errors': _COUNT, 'saver_card_errors?': _COUNT}
_OTA_STATUS: dict = {'running': r'ota_[01]', 'boot': r'ota_[01]', 'next': r'ota_[01]', 'build': r'[0-9a-f]+',
                     'transfer': r'\S+', 'received': r'\S+'}
_VIDEO_STATS: dict = {'captured': _COUNT, 'missed': _COUNT, 'encoded': _COUNT, 'dropped': _COUNT,
                      'write_errors': _COUNT, 'segments': _COUNT, 'preview': _COUNT}
# The bus diagnostics an armed board refuses (M9): the line, the part it needs up, its answer bare while disarmed
_BUS_DIAGNOSTICS: tuple = (
    ('i2c read', _i2c_part, 'i2c read needs port= (the I2C port)'),
    ('i2c touch', _i2c_part, 'i2c touch needs port= (the I2C port)'),
    ('i2c general-call', _i2c_part, 'i2c general-call needs port= (the I2C port)'),
    ('imu mute', _spi_imu, 'imu mute needs ms= (how long, milliseconds)'),
)


def _bad_arguments(message: object = TEXT) -> tuple:
    """The accepted answer `err badargs <message>` (any message by default)."""
    return ('err', 'badargs', message)


def _core(name: str, request: str, replies: tuple, invariants: tuple = (), note: str = '') -> Case:
    """A core-tier case: it has no not-yet answer, because the core must pass from the first milestone on."""
    return Case('core', name, request, replies, invariants, not_yet=(), note=note)


def _case_name(line: str) -> str:
    """A bus diagnostic's line as a case name's stem: `i2c general-call` -> i2c_general_call."""
    return line.replace(' ', '_').replace('-', '_')


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
    Case('inspect', 'inspect_imu', _on_device('inspect', 'lsm6dso32'), (('ok', _IMU),), (_names_itself,),
         milestone=6),
    Case('inspect', 'inspect_barometer', _on_device('inspect', 'icp10111'), (('ok', _BAROMETER),), (_names_itself,),
         milestone=6),
    Case('inspect', 'inspect_separation', _on_device('inspect', 'separation'), (('ok', _SEPARATION),),
         (_names_itself,), milestone=6),
    Case('inspect', 'inspect_bluetooth', _on_device('inspect', 'bluetooth'), (('ok', _BLUETOOTH),), (_names_itself,),
         milestone=7, note='the Rust board is a stub in plan B\'s no-BLE shape: radio as configured, active null'),
    Case('inspect', 'update_barometer_ground', _ground_write, (('ok', {'changed': 'list'}),), (_changed_ground,),
         allow=('active',), milestone=6,
         note='sets the ground 100 m AMSL directly (the warm start\'s rebase); the re-zero below puts a live one back'),
    Case('inspect', 'update_barometer_text_ground', _on_device('update', 'icp10111', {'ground': 'high'}),
         (('err', 'refused', TEXT),), allow=('active',), milestone=6,
         note='plan B\'s float() raises inside the refusal guard'),
    Case('inspect', 'update_barometer_rezero', _on_device('update', 'icp10111', {'rezero': True}),
         (('ok', {'changed': 'list'}),), (_changed_ground,), allow=('active',), milestone=6,
         note='re-captures the ground from the live altitude, as setup did'),
    Case('inspect', 'stats_databoard', _on_object('stats', 'databoard'), (('ok', _ANY_OBJECT),),
         (_databoard_sources,), remember=_remember('databoard_sources'), milestone=9,
         note='plan B Databoard.stats: each quantity\'s sources'),
    Case('inspect', 'inspect_databoard', _on_object('inspect', 'databoard'), (('ok', _ANY_OBJECT),),
         (_databoard_quantities,), milestone=9, note='plan B Databoard.inspect: each quantity\'s fused value'),
    Case('inspect', 'update_databoard_nothing', _on_object('update', 'databoard', {}),
         (('ok', {'changed': 'list'}), ('err', 'internal', TEXT)), (_nothing_written,), milestone=9,
         note='plan B\'s Databoard has no update(): its dispatcher raises and answers err internal, writing nothing; '
              'the Rust board answers changed []'),
    Case('inspect', 'update_databoard_read_only', _on_object('update', 'databoard', {'altitude': 0}),
         (('ok', {'changed': 'list'}), ('err', 'internal', TEXT)), (_nothing_written,), allow=('active',),
         milestone=9, note='nothing on the databoard is writable: a board that took it would move the fused altitude. '
                           'Plan B raises (err internal), as for the empty update'),
    Case('inspect', 'inspect_bmi323', _on_device('inspect', 'bmi323'), (('ok', _BMI323),),
         (_names_itself, _configured_address, _triples('accel_g', 'rate_dps'), _bmi323_evidence), milestone=9,
         note='rate_dps where plan B had rate_cds (a listed divergence)'),
    Case('inspect', 'inspect_bmp581', _on_device('inspect', 'bmp581'), (('ok', _BMP581),),
         (_names_itself, _configured_address, _bmp581_guard), remember=_remember_ground, milestone=9),
    Case('inspect', 'inspect_barometer_change', _on_device('inspect', 'icp10111'), (('ok', _BAROMETER_CHANGE),),
         (_names_itself,), milestone=9, note='every barometer\'s climb rate, new beside plan B\'s four quantities'),
    Case('inspect', 'inspect_bmm350', _on_device('inspect', 'bmm350'), (('ok', _BMM350),),
         (_names_itself, _configured_address, _triples('mag_counts'), _bmm350_evidence), milestone=9),
    Case('inspect', 'inspect_attitude', _on_object('inspect', 'attitude'), (('ok', _ATTITUDE),),
         (_names_itself, _angles_are_decimals), milestone=9),
    Case('inspect', 'inspect_sequencer', _on_object('inspect', 'sequencer'), (('ok', _SEQUENCER),),
         (_names_itself, _sequencer_binding), milestone=9,
         note='the multi-source stage machine\'s binding: plan B\'s sequencer had no snapshot of its own'),
    Case('inspect', 'health_names_its_disagreements', 'health', (('ok', _HEALTH),), (_disagreements_named,),
         milestone=9, note='sources-disagree is information for the reader, never an arm gate (the operator\'s Q5)'),
    Case('inspect', 'update_bmp581_ground', _bmp581_ground_write, (('ok', {'changed': 'list'}),), (_changed_ground,),
         allow=('active',), milestone=9,
         note='sets the ground %.0f m above the one read; the restore below puts the read one back' % _GROUND_STEP_M),
    Case('inspect', 'bmp581_ground_moved', _bmp581_settled, (('ok', _BMP581),),
         (_ground_reads(lambda context: context['bmp581_ground'] + _GROUND_STEP_M),), allow=('active',), milestone=9,
         note='its elevation reads against the ground written: an update that answers changed but applies nothing '
              'fails here'),
    Case('inspect', 'update_bmp581_text_ground',
         _after_ground_write(_on_device('update', 'bmp581', {'ground': 'high'})),
         (('err', 'refused', 'ground must be a number'),
          ('err', 'refused', "could not convert string to float: 'high'")), allow=('active',), milestone=9,
         note='the Rust board names the refusal; plan B\'s float() raises with the interpreter\'s text'),
    Case('inspect', 'update_bmp581_rezero', _after_ground_write(_on_device('update', 'bmp581', {'rezero': True})),
         (('ok', {'changed': 'list'}),), (_changed_ground,), allow=('active',), milestone=9,
         note='re-captures the ground from its own fresh altitude'),
    Case('inspect', 'update_bmp581_restore', _bmp581_ground_restore, (('ok', {'changed': 'list'}),),
         (_changed_ground,), allow=('active',), milestone=9),
    Case('inspect', 'bmp581_ground_restored', _bmp581_settled, (('ok', _BMP581),),
         (_ground_reads(lambda context: context['bmp581_ground']),), allow=('active',), milestone=9),
    Case('inspect', 'inspect_sdp810', _on_device('inspect', 'sdp810'), (('ok', _PITOT),), (_names_itself,),
         milestone=10),
    Case('inspect', 'inspect_ina226', _on_device('inspect', 'ina226'), (('ok', _POWER),), (_names_itself,),
         milestone=10, note='alerts: the over-current ALERT trips since boot, 0 where no alert_pin is wired'),
    Case('inspect', 'inspect_laser', _on_any_device('inspect', _LASER_DRIVERS), (('ok', _RANGER),), (_names_itself,),
         milestone=10),
    Case('inspect', 'inspect_gnss', _on_any_device('inspect', _GNSS_DRIVERS), (('ok', _GNSS),),
         (_names_itself, _gnss_figures), milestone=11, note='`lines` is the port\'s own: the NMEA lines read'),
    Case('inspect', 'inspect_gnss_calib', _on_object('inspect', 'gnss_calib'), (('ok', _GNSS_CALIB),),
         (_names_itself,), milestone=11),
    Case('inspect', 'inspect_servo', _on_any_device('inspect', _SERVO_DRIVERS), (('ok', _SERVO),), (_names_itself,),
         milestone=13, note='the COMMANDED angle and pulse: an SG90 is open-loop, so feedback is null'),
    Case('inspect', 'inspect_bno055', _on_device('inspect', 'bno055'), (('ok', _ABSOLUTE),),
         (_names_itself, _configured_address, _triples('attitude_deg', 'accel_g'), _bno055_state), milestone=18,
         note='addr is the port\'s own; the rest are plan B\'s keys'),
    Case('inspect', 'inspect_bmp280', _on_device('inspect', 'bmp280'), (('ok', _BMP581),),
         (_names_itself, _configured_address, _bmp581_guard), milestone=18,
         note='plan B\'s four quantities and addr, then the port\'s change and its own guard counters (the BMP581\'s)'),
    Case('inspect', 'health_imu_calibration', 'health', (('ok', _HEALTH),), (_imu_calibration,), milestone=18,
         note='present while a BNO055 runs and has read its CALIB_STAT (CC\'s IMU column)'),
)

_CONTROL: tuple = (
    Case('control', 'stage_read', 'stage', (('ok', _STAGE),), (_known_stage,)),
    Case('control', 'report', 'report', (('ok', _REPORT),), (_known_stage,)),
    Case('control', 'disarm', 'disarm', (('ok', _ARMED),), (_armed_is(False),), remember=_mark('disarmed')),
    Case('control', 'arm', _arm, (('ok', _ARMED), ('err', 'unsafe', _ANY_OBJECT)), (_arm_answer,),
         allow=('active',), remember=_arm_outcome,
         note='arm probes every device -- each servo sweeps its full travel -- before it answers'),
    *(Case('control', '%s_refused_armed' % _case_name(line), _while_armed(_with_part(line, part)),
           (('err', 'unsafe', _REFUSED_ARMED),), not_yet=(_UNSUPPORTED, _NOT_IN_PLAN_B), allow=('active',),
           milestone=9, note='a bus diagnostic holds a bus owner off its parts: never in the last seconds on the pad')
      for line, part, _usage in _BUS_DIAGNOSTICS),
    Case('control', 'bustune_off_clock_refused_armed', _while_armed(_imu_bus_off_clock), (('err', 'unsafe', TEXT),),
         (_refused_back_to_its_clock,), allow=('active',), milestone=9,
         note='armed, only the rung back to the configured clock runs, so an interrupted sweep can still be undone'),
    Case('control', 'bustune_back_while_armed', _bustune_back, (('ok', _BUSTUNE),), (_rung_passed,),
         allow=('active',), milestone=9,
         note='the way back runs armed; it also puts the bus back on a board whose gate let the rung above run'),
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
    Case('control', 'calibrate_while_held', _while_held('calibrate nosuch'),
         (('err', 'unsafe', _REFUSED_IN_FLIGHT), ('err', 'unsafe', _REFUSED_OUTSIDE_SETTING)), allow=('stage',),
         milestone=6,
         note='a baro calibrate mid-air re-zeroes elevation: both boards refuse before they look the name up'),
    Case('control', 'bus_diagnostic_while_held', _while_held(_with_part('i2c read', _i2c_part)),
         (('err', 'unsafe', _REFUSED_OUTSIDE_SETTING),), not_yet=(_UNSUPPORTED, _NOT_IN_PLAN_B), allow=('stage',),
         milestone=9, note='every bus diagnostic runs only in SETTING, refused before its arguments are read'),
    Case('control', 'arm_while_held', _arm_while_held, (('err', 'unsafe', {'stage': 'str'}),),
         (_refused_for_the_hold,), allow=('stage', 'active'), milestone=7,
         note='a held board never arms: plan B refuses at its gate, the Rust board also when the arm applies. Held '
              'off the ground neither board probes, so nothing sweeps'),
    Case('control', 'disarm_after_held_arm', _disarm_after_held_arm, (('ok', _ARMED),), (_armed_is(False),),
         allow=('stage', 'active'), remember=_mark('disarmed')),
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
    Case('diagnostics', 'calibrate_not_a_device', 'calibrate mission', (_bad_arguments('no calibrate for mission'),),
         allow=('active',), milestone=6),
    Case('diagnostics', 'calibrate_barometer', _on_device('calibrate', 'icp10111'), (('ok', _ANY_OBJECT),),
         (_succeeded,), allow=('active',), milestone=6,
         note='re-captures the ground (and logs `calibrated: ground <m> m AMSL`): the glider still on the pad'),
    Case('diagnostics', 'calibrate_imu', _on_device('calibrate', 'lsm6dso32'), (('ok', _ANY_OBJECT),), (_succeeded,),
         allow=('active',), milestone=6, note='nothing outstanding: plan B\'s Task.calibrate answers null'),
    Case('diagnostics', 'probe_all', 'probe', (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps,), allow=('active',)),
    Case('diagnostics', 'probe_mission', 'probe mission', (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target), allow=('active',)),
    Case('diagnostics', 'probe_unknown_object', 'probe nosuch', (_bad_arguments('no probe for nosuch'),),
         allow=('active',)),
    Case('diagnostics', 'probe_devices', _probe_devices, (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target, _succeeded), allow=('active',), milestone=6),
    Case('diagnostics', 'verify', 'verify', (('ok', _VERIFY),), (_verify_consistent, _readiness_from_plan_b),
         allow=('active',)),
    Case('diagnostics', 'detect', 'detect', (('ok', _DETECT),), (), allow=('active',), milestone=5,
         note='read-only, but the spec counts it active: it re-scans both I2C buses'),
    Case('diagnostics', 'bustune_without_arguments', 'bustune', (_bad_arguments('bustune <kind> <id> <freq>'),),
         allow=('active',), milestone=5),
    Case('diagnostics', 'bustune_imu_at_its_clock', _imu_bus_at_its_clock, (('ok', _BUSTUNE),), (_rung_passed,),
         allow=('active',), milestone=6,
         note='the CC ladder\'s restore rung; plan B on CPython has no SPI (machine.SPI is a stub there)'),
    *(Case('diagnostics', '%s_without_arguments' % _case_name(line), _with_part(line, part),
           (_bad_arguments(usage),), not_yet=(_UNSUPPORTED, _NOT_IN_PLAN_B), allow=('active',), milestone=9,
           note='disarmed in SETTING the gate is open: the bare line is refused for its missing argument')
      for line, part, usage in _BUS_DIAGNOSTICS),
    Case('diagnostics', 'probe_attitude', _on_object('probe', 'attitude'),
         (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)), (_probe_maps, _names_its_target, _succeeded),
         allow=('active',), milestone=9, note='plan B\'s probe: the gyro rate the filter steps on is fresh'),
    Case('diagnostics', 'probe_sen0697', _probe_sen0697, (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target, _succeeded), allow=('active',), milestone=9),
    Case('diagnostics', 'calibrate_bmp581', _on_device('calibrate', 'bmp581'), (('ok', _ANY_OBJECT),), (_succeeded,),
         allow=('active',), milestone=9,
         note='re-captures its ground from guarded reads: a divergence, plan B\'s BMP581 had no calibrate'),
    Case('diagnostics', 'calibrate_bmm350_still', _on_device('calibrate', 'bmm350'), (('ok', _ANY_OBJECT),),
         (_still_board_refusal,), allow=('active',), milestone=9,
         note='a board lying still: the answer is the refusal, and nothing is persisted. Turned through a full level '
              'circle since boot, the board would calibrate and persist it -- run it on a board at rest'),
    Case('diagnostics', 'probe_sen0253', _probe_sen0253, (('ok', _ANY_OBJECT), ('ok', _ANY_OBJECT, _ANY_OBJECT)),
         (_probe_maps, _names_its_target, _succeeded), allow=('active',), milestone=18),
    Case('diagnostics', 'calibrate_bmp280', _on_device('calibrate', 'bmp280'), (('ok', _ANY_OBJECT),), (_succeeded,),
         allow=('active',), milestone=18, note='re-captures its ground from guarded reads, as the BMP581'),
    Case('diagnostics', 'calibrate_bno055', _on_device('calibrate', 'bno055'), (('ok', _ANY_OBJECT),),
         (_figure_eight_or_saved,), allow=('active',), milestone=18,
         note='saves the learned profile to NVS once the figure-8 converged (the fusion pauses ~50 ms); before that '
              'it answers the figure-8 owed, as plan B'),
    Case('diagnostics', 'entry_show', _gcl('entry show'), (('ok', {'lines': 'list'}),), (),
         (_UNSUPPORTED, _GCL_ONLY), milestone=21,
         note='the entry file (M21): `verify` and `arm` refuse while it holds a line'),
    Case('diagnostics', 'df', _gcl('df'), (('ok', 'df'),), (), (_UNSUPPORTED, _GCL_ONLY), milestone=21, named=_DF,
         note='the card per volume; `bounced` counts the transfers whose memory the SDMMC DMA cannot reach'),
    Case('diagnostics', 'ls', _gcl('ls'), (('ok', {'entries': 'list', 'path?': 'str'}),), (),
         (_UNSUPPORTED, _GCL_ONLY), milestone=21,
         note='the card\'s root: [name, bytes] per file, [name/, null] per directory'),
    Case('diagnostics', 'ring', _gcl('ring'), (('ok', 'ring'),), (), (_UNSUPPORTED, _GCL_ONLY), milestone=4,
         named=_RING, note='the flash ring and its savers (M3, M4, M19)'),
    Case('diagnostics', 'video_stats', _gcl('video stats'), (('ok', 'video', TEXT),), (),
         (_UNSUPPORTED, _GCL_ONLY), milestone=7, named=_VIDEO_STATS,
         note='running or idle, then the counters since the last start'),
)

_OTA: tuple = (
    Case('ota', 'push_status', 'push-status', (('ok', _PUSH_STATUS),), (), (_UNSUPPORTED, _PUSH_NOT_PORTED),
         milestone=_NEVER,
         note='the spec lists {uploading, received, size, chunks, orphans}; with no upload open plan B answers '
              '{uploading: null, orphans}'),
    Case('ota', 'ota_status', _gcl('ota status'), (('ok', 'ota'),), (_distinct_slots,), (_UNSUPPORTED, _GCL_ONLY),
         milestone=15, named=_OTA_STATUS, note='the Rust board\'s firmware OTA (M15): its slots and any transfer'),
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
    Read whoami, health and help once, before any case: the context every case shares, and the safety refusal.

    A failing read leaves its keys out (the cases that need them are then blocked) -- the whoami and health
    cases themselves report the failure. That is enough for a read-only run, but not for one a flag lets change
    the board: the runner never touches a board it has not seen on the ground and disarmed.

    Args:
        transport - the transport to the board.
        board_id - the id the operator addressed (checked against iam), or None for the fake.
        allowed - the ALLOW_FLAGS the operator gave.

    Returns:
        The context: board_id and tees, and from the board id/stage/config_source/boot_id/epoch (whoami),
        armed (health), gcl (help lists commands: a Rust board).

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
    context['gcl'] = _speaks_gcl(transport)
    return context


def _speaks_gcl(transport: transports.Transport) -> bool:
    """Whether the board speaks GCL: its `help` lists commands (a Rust board; plan B answers badcmd)."""
    try:
        lines = transport.exchange('help')
        message = cc.parse(lines[0]) if len(lines) == 1 else None
        payload = json.loads(message.args[-1]) if message is not None and message.args else None
    except (transports.NoAnswer, transports.Unreachable, OSError, ValueError, IndexError):
        return False
    return message.command == 'ok' and isinstance(payload, dict) and isinstance(payload.get('commands'), list)


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
    exception (a USB unplug, Ctrl-C during a 30 s probe) skips them, and could leave the board held at gliding,
    carrying the probe launch id, with its barometer's ground at 100 m AMSL or its BMP581's stepped, or its IMU bus
    off its clock. Every undo is idempotent, so one a case already sent is merely repeated.
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
    if context.get('ground_written'):
        undo.append(('barometer re-zero', _on_device('update', 'icp10111', {'rezero': True})(context)))
    if context.get('bmp581_ground_written'):
        undo.append(('BMP581 ground restore', _bmp581_ground_restore(context)))
    if context.get('off_clock_sent'):
        undo.append(('IMU bus back to its clock', _imu_bus_at_its_clock(context)))
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
    problem = _envelope(request, lines, case.named is not None)
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
        if problem is None and case.named is not None:
            problem = _named(case.named, message.named)
        if problem is None:
            reply = _Reply(request, message.command, values, dict(message.named))
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
