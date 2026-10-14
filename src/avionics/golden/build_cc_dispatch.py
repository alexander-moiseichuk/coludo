"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture cc_dispatch.json: the CC dispatcher's error paths, envelope rules and reply shapes, as the
REAL plan B dispatcher (cc_client.create_dispatcher) answers them on CPython.

The board is plan B as main.py boots it, minus the hardware: the config load with no board.config on disk
(the built-in default, source 'default'), the boot counted, the layout resolved (a host scan finds nothing,
so 'undecided' -- what the Rust board reports until it scans), the Mission registered with no launch.config
(the HPRC default), and plan B's own Controller over that config. The Controller is never set up, so it
runs no tasks and records no failures: its stage, hold, arm gate and disarm are plan B's, and nothing it
answers depends on hardware. Everything runs inside a scratch directory, so nothing reads or writes a
board.config or launch.config lying in the working directory.

The cases run IN ORDER against that one board: plan B keeps state between commands (the Inspector registry,
the databoard, the stage, the log window), so a reply can depend on what ran before it -- each case where
that matters says so.

A case is {name, request, status, arguments_kind, expected}, plus where it applies:
  * note -- why the case is there, or where plan B's answer is not the obvious one;
  * extra_keys -- {key: type} a json_keys answer may carry beyond the keys plan B sent here: plan B's
    conditional keys and the Rust board's own;
  * owed -- {status, arguments_kind, expected}: the answer the Rust board gives ON PURPOSE instead of plan
    B's (a documented not-yet answer, or a plan B defect not ported). A consumer checks a port against
    `owed` when it is present, and against the plan B answer otherwise.
`arm` with nothing held is left out: with no setup run, plan B would arm a board whose every device is
down, a verdict no real board gives; `arm` is pinned while the stage is held, where every board refuses.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import json
import os
import tempfile

import cc_client
import cc_protocol as cc
import config as config_module
import controller as controller_module
import layout
import mission
import recorder

_BOOT_ID: int = 1  # stands in for main.py's NVS boot counter, which has no host equivalent; any count reads alike
_CLOCK_SET_EPOCH: int = 1791244800  # the Unix epoch a hub sends to sync the clock: 2026-10-06T00:00:00Z

"""
How each case's `expected` is compared with the reply's decoded positional arguments. The cases pick the
strictest kind the Rust board can meet: Python repr text and volatile values (uptime, epoch, clock, mem_free,
temp) are never pinned.
"""
_ARGUMENTS_KINDS: dict = {
    'exact': 'the decoded arguments, verbatim strings, as many as listed',
    'any': ('the leading arguments listed (the error code) match verbatim, and exactly one more follows: '
            'free text (Python repr is never pinned)'),
    'json': 'each argument parsed as JSON and compared by value (separators and key order are free)',
    'json_keys': ('per argument: a string matches verbatim; an object {key: type} means the argument is a '
                  'JSON object with exactly these keys (plus the case extra_keys, when sent), each value of '
                  "that type; a type is one of null, bool, int, float, str, list, object, or several joined "
                  "by '|' (float|null)"),
}
_TYPE_NAMES: tuple = ('null', 'bool', 'int', 'float', 'str', 'list', 'object')

"""
The heartbeat keys plan B sends only where they apply (an outstanding calibration, a flight task, a BNO055),
and the Rust board's own (its scarce internal RAM, and the SD free space cached at mount).
"""
_HEALTH_EXTRA_KEYS: dict = {
    'calibration': 'object',
    'flight': 'object',
    'imu_calibration': 'list|null',
    'internal_free': 'int',
    'sd_free_mb': 'int|null',
}

"""
The requests, in the order they run. `types` (json_keys only) declares the types a key can take on a board
beyond the one sent here, such as null for a value that is not fresh; the builder checks the observed type is
among them and records the declaration.
"""
_CASES: tuple = (
    {'name': 'objects_lists_the_registry', 'request': 'objects', 'arguments_kind': 'json',
     'note': ('the Controller and the Mission, as every board registers them; device tasks join as they come '
              'up. Runs before any health: plan B registers the databoard on the first parameter read (health '
              'does one), and lists it from then on')},
    {'name': 'empty_line_no_reply', 'request': '', 'arguments_kind': 'exact'},
    {'name': 'blank_line_no_reply', 'request': '   ', 'arguments_kind': 'exact'},
    {'name': 'ping', 'request': 'ping', 'arguments_kind': 'exact'},
    {'name': 'command_is_lowercased', 'request': 'PING', 'arguments_kind': 'exact'},
    {'name': 'unknown_command', 'request': 'frobnicate', 'arguments_kind': 'exact'},
    {'name': 'unknown_command_is_lowercased', 'request': 'FROB', 'arguments_kind': 'exact'},
    {'name': 'base64_bad_alphabet', 'request': 'inspect base64:@@@', 'arguments_kind': 'any',
     'note': ("CPython's base64 is lenient: it drops the '@'s, the token decodes to '' and plan B answers "
              "'no object ' instead of a parse error. The code is badargs either way; the Rust parser is "
              'strict and answers it as a parse error')},
    {'name': 'base64_bad_padding', 'request': 'inspect base64:QQ', 'arguments_kind': 'any'},
    {'name': 'base64_not_utf8', 'request': 'inspect base64:/w==', 'arguments_kind': 'any'},
    {'name': 'handler_exception_is_internal', 'request': 'update mission 5', 'arguments_kind': 'any',
     'note': ('pins the envelope rule -- a handler that raises answers err internal and the link survives. '
              'The trigger is a plan B defect (a JSON number reaches Mission.update, which iterates it); the '
              'Rust board refuses properties that are not a JSON object instead'),
     'owed': {'status': 'err', 'arguments_kind': 'any', 'expected': ['refused']}},
    {'name': 'inspect_without_arguments', 'request': 'inspect', 'arguments_kind': 'exact'},
    {'name': 'inspect_named_only', 'request': 'inspect key=value', 'arguments_kind': 'exact',
     'note': 'a key=value token is named, not positional, so the object is missing'},
    {'name': 'inspect_unknown_object', 'request': 'inspect nosuch', 'arguments_kind': 'exact'},
    {'name': 'update_without_arguments', 'request': 'update', 'arguments_kind': 'exact'},
    {'name': 'update_without_json', 'request': 'update mission', 'arguments_kind': 'exact'},
    {'name': 'update_unknown_object', 'request': cc.build('update', ['nosuch', '{}']), 'arguments_kind': 'exact'},
    {'name': 'update_bad_json', 'request': 'update mission nojson', 'arguments_kind': 'any',
     'note': ('refused, not badargs: plan B parses the JSON inside the guard that turns a driver refusal '
              '(ValueError) into err refused, and a JSON decode error is a ValueError')},
    {'name': 'stats_without_arguments', 'request': 'stats', 'arguments_kind': 'exact'},
    {'name': 'stats_unknown_object', 'request': 'stats nosuch', 'arguments_kind': 'exact'},
    {'name': 'get_config_defaults_to_board', 'request': 'get-config', 'arguments_kind': 'json_keys'},
    {'name': 'get_config_unknown_name', 'request': 'get-config nosuch', 'arguments_kind': 'exact'},
    {'name': 'set_config_without_arguments', 'request': 'set-config', 'arguments_kind': 'exact'},
    {'name': 'set_config_without_json', 'request': 'set-config launch', 'arguments_kind': 'exact'},
    {'name': 'set_config_bad_json', 'request': 'set-config launch nojson', 'arguments_kind': 'exact'},
    {'name': 'log_duration_not_integer', 'request': 'log abc', 'arguments_kind': 'exact'},
    {'name': 'log_duration_fraction', 'request': 'log 1.5', 'arguments_kind': 'exact'},
    {'name': 'log_duration_hexadecimal', 'request': 'log 0x10', 'arguments_kind': 'exact'},
    {'name': 'tlm_duration_not_integer', 'request': 'tlm x', 'arguments_kind': 'exact'},
    {'name': 'log_stop_when_idle', 'request': 'log 0', 'arguments_kind': 'json'},
    {'name': 'log_duration_negative', 'request': 'log -1', 'arguments_kind': 'json',
     'note': 'int() takes the sign, and a window of 0 or less stops the tee'},
    {'name': 'log_duration_plus_sign', 'request': 'log +5', 'arguments_kind': 'json',
     'note': 'opens a 5 ms window; the next two cases run inside it on the virtual clock'},
    {'name': 'log_duration_underscores', 'request': 'log 1_000', 'arguments_kind': 'json',
     'note': ("int() accepts '1_000' on CPython and on the board alike (MicroPython's py/parsenum.c skips "
              "every '_'), so a port's duration parse must too. MicroPython also takes '_1000' and "
              "'1__000', which CPython refuses: this oracle cannot speak for them, so no case pins them")},
    {'name': 'log_stop_after_a_window', 'request': 'log 0', 'arguments_kind': 'json',
     'note': 'returns what the window buffered (nothing: no request logs) and stops it'},
    {'name': 'tlm_stop_when_idle', 'request': 'tlm 0', 'arguments_kind': 'json'},
    {'name': 'tlm_window_at_the_ticks_range', 'request': 'tlm 536870', 'arguments_kind': 'json',
     'note': 'the longest window plan B can arm: 536870000 us is below the 2**29 us ticks_add accepts'},
    {'name': 'tlm_stop_after_a_window', 'request': 'tlm 0', 'arguments_kind': 'json'},
    {'name': 'tlm_window_past_the_ticks_range', 'request': 'tlm 536871', 'arguments_kind': 'any',
     'note': ("plan B arms the window with time.ticks_add(now, ms * 1000), which raises OverflowError from "
              '2**29 us (8.9 minutes) on, so a longer window answers err internal. The Rust board keeps its '
              'window in 64 bits and has no such limit'),
     'owed': {'status': 'ok', 'arguments_kind': 'json', 'expected': [{'dropped': 0, 'samples': []}]}},
    {'name': 'tlm_duration_past_64_bits', 'request': 'tlm 99999999999999999999', 'arguments_kind': 'any',
     'note': ('plan B takes any integer, then fails arming the window (err internal); the Rust board parses a '
              '64-bit integer and refuses a longer one as badargs'),
     'owed': {'status': 'err', 'arguments_kind': 'exact', 'expected': ['badargs', 'tlm <duration_ms>']}},
    {'name': 'stage_read', 'request': 'stage', 'arguments_kind': 'json'},
    {'name': 'stage_unknown_name', 'request': 'stage nosuch', 'arguments_kind': 'exact'},
    {'name': 'stage_hold_on_the_ground', 'request': 'stage done', 'arguments_kind': 'json',
     'note': 'done is a ground stage, so nothing is gated, but the stage is held (manual)'},
    {'name': 'arm_refused_while_held', 'request': 'arm', 'arguments_kind': 'any',
     'note': 'a held stage is an arm problem on every board: err unsafe, the problems as JSON'},
    {'name': 'stage_hold_in_flight', 'request': 'stage gliding', 'arguments_kind': 'json'},
    {'name': 'ground_only_refused_in_flight', 'request': 'reset-config', 'arguments_kind': 'exact',
     'note': 'the gate text is verbatim: it starts with the word refused, but its code is unsafe',
     'owed': {'status': 'err', 'arguments_kind': 'exact',
              'expected': ['unsafe', 'refused -- stage is gliding (changes only in setting)']}},
    {'name': 'update_while_held', 'request': 'update mission base64:e30=', 'arguments_kind': 'json',
     'note': 'plan B leaves update open in every stage; the Rust board refuses every change outside SETTING '
             '(the operator\'s rule, doc/specs/avionics-port.md §6)',
     'owed': {'status': 'err', 'arguments_kind': 'exact',
              'expected': ['unsafe', 'refused -- stage is gliding (changes only in setting)']}},
    {'name': 'stage_auto_releases_the_hold', 'request': 'stage auto', 'arguments_kind': 'json',
     'note': 'the stage stays where it was held; only the hold (manual) goes'},
    {'name': 'stage_setting_returns_to_the_ground', 'request': 'stage setting', 'arguments_kind': 'json',
     'note': 'a return to the ground, not a hold: manual stays false'},
    {'name': 'disarm', 'request': 'disarm', 'arguments_kind': 'json'},
    {'name': 'report', 'request': 'report', 'arguments_kind': 'json',
     'note': 'the Controller runs no tasks here, and the Rust board none until its drivers are ported'},
    {'name': 'probe_unknown_object', 'request': 'probe nosuch', 'arguments_kind': 'exact'},
    {'name': 'verify', 'request': 'verify', 'arguments_kind': 'json_keys',
     'note': 'the shape only: which devices are up, and so pass and ready, depend on the hardware'},
    {'name': 'update_mission_sets_the_clock',
     'request': cc.build('update', ['mission', json.dumps({'epoch': _CLOCK_SET_EPOCH})]), 'arguments_kind': 'json',
     'note': ('the hub syncs a cold board this way (one whose clock reads inside 2000, as this one did until '
              'here); epoch is listed whenever the RTC was set')},
    {'name': 'whoami', 'request': 'whoami', 'arguments_kind': 'json_keys', 'types': {'boot_id': 'int|null'},
     'note': 'boot_id is null when nothing counted the boot (an NVS failure)'},
    {'name': 'health', 'request': 'health', 'arguments_kind': 'json_keys',
     'types': {'temp': 'float|null', 'position': 'list|null', 'agl': 'float|null', 'launchpad': 'list|null',
               'site': 'str|null'},
     'extra_keys': _HEALTH_EXTRA_KEYS,
     'note': ('temp is null when the die sensor fails. position and agl are always present: '
              'Databoard.parameter() creates the channel on first read, so they are null until fresh. '
              'launchpad is null with no launch point and no fix, site null with no site selected')},
    {'name': 'inspect_mission', 'request': 'inspect mission', 'arguments_kind': 'json_keys',
     'note': 'the HPRC default mission: launch point set and zone in range, so the geometry keys are present'},
    {'name': 'get_config_launch', 'request': 'get-config launch', 'arguments_kind': 'json_keys'},
)


def _type_name(value: object) -> str:
    """
    The JSON type name a json_keys case records for one value.

    Args:
        value - a value json.loads produced.

    Returns:
        One of _TYPE_NAMES.
    """
    if value is None:
        return 'null'
    if isinstance(value, bool):  # before int: a bool is an int in Python
        return 'bool'
    names = {int: 'int', float: 'float', str: 'str', list: 'list', dict: 'object'}
    return names[type(value)]


def _check_types(declared: dict) -> None:
    """
    Refuse a {key: type} declaration that names a type outside _TYPE_NAMES.

    Args:
        declared - {key: 'type' or 'type|type'}.

    Returns:
        None.

    Raises:
        ValueError - an unknown type name.
    """
    for key, specification in declared.items():
        unknown = [name for name in specification.split('|') if name not in _TYPE_NAMES]
        if unknown:
            raise ValueError('%s: unknown type %s' % (key, ', '.join(unknown)))


def _json_keys(argument: str, types: dict) -> object:
    """
    One reply argument as a json_keys case records it.

    Args:
        argument - a decoded reply argument.
        types - {key: declared type} for the keys whose type on a board is wider than the one sent here.

    Returns:
        {key: type} when the argument is a JSON object (the declared type where there is one, else the
        observed one); the argument itself otherwise (the iam board id).

    Raises:
        ValueError - a declaration that misses the observed type, or names a key plan B did not send.
    """
    try:
        value = json.loads(argument)
    except ValueError:
        return argument
    if not isinstance(value, dict):
        return argument
    absent = set(types) - set(value)
    if absent:
        raise ValueError('types declared for keys plan B did not send: %s' % ', '.join(sorted(absent)))
    recorded = {}
    for key, item in value.items():
        observed = _type_name(item)
        declared = types.get(key, observed)
        if observed not in declared.split('|'):
            raise ValueError('%s: plan B sent %s, the case declares %s' % (key, observed, declared))
        recorded[key] = declared
    return recorded


def _expected(case: dict, arguments: list) -> list:
    """
    What a case expects of the reply's decoded positional arguments, by its kind.

    Args:
        case - the case spec (its arguments_kind, and its types for json_keys).
        arguments - the reply's decoded positional arguments.

    Returns:
        The `expected` list for the case.

    Raises:
        ValueError - an `any` reply that is not a code and one free text.
    """
    kind = case['arguments_kind']
    if kind == 'exact':
        return list(arguments)
    if kind == 'any':
        if len(arguments) != 2:
            raise ValueError('an any case keeps the code of a code-and-text reply, and this one has %d arguments'
                             % len(arguments))
        return arguments[:1]
    if kind == 'json':
        return [json.loads(argument) for argument in arguments]
    return [_json_keys(argument, case.get('types', {})) for argument in arguments]


def _record(case: dict, reply: str | None) -> dict:
    """
    One case as the fixture writes it.

    Args:
        case - the case spec from _CASES.
        reply - plan B's reply line; None when it sent none.

    Returns:
        {name, request, status, arguments_kind, expected} plus the note, extra_keys and owed it has.

    Raises:
        ValueError - the spec names an unknown kind or type, the reply carries named parameters (no case
        could pin them), or _expected refuses it.
    """
    if case['arguments_kind'] not in _ARGUMENTS_KINDS:
        raise ValueError('%s: unknown arguments_kind %s' % (case['name'], case['arguments_kind']))
    _check_types(case.get('types', {}))
    _check_types(case.get('extra_keys', {}))
    owed = case.get('owed')
    if owed is not None and owed['arguments_kind'] not in _ARGUMENTS_KINDS:
        raise ValueError('%s: unknown owed arguments_kind %s' % (case['name'], owed['arguments_kind']))
    message = cc.parse(reply) if reply is not None else None
    if message is not None and message.named:
        raise ValueError('%s: the reply carries named parameters %s' % (case['name'], message.named))
    record = {key: value for key, value in case.items() if key != 'types'}
    record['status'] = message.command if message is not None else None
    record['expected'] = _expected(case, message.args if message is not None else [])
    return record


async def _replies(requests: list) -> list:
    """
    Boot a plan B board without hardware in a scratch directory and send it every request in order.

    Args:
        requests - the request lines.

    Returns:
        The reply line for each request; None where plan B sends no reply.
    """
    with tempfile.TemporaryDirectory() as scratch:
        board_path = os.path.join(scratch, 'board.config')
        board, source, _errors = config_module.load(board_path)
        config_module.BOOT_SOURCE = source
        recorder.Recorder.boot_id = _BOOT_ID
        layout.resolve(board, log=lambda line: None)
        field = config_module.device(board, name='field') or {}
        mission.Mission(path=os.path.join(scratch, 'launch.config'), max_range_m=field.get('max_range_m', 200))
        controller = controller_module.Controller(board)
        dispatcher = cc_client.create_dispatcher(board, controller=controller, config_path=board_path)
        return [await dispatcher.handle(request) for request in requests]


def build() -> dict:
    """
    The cc_dispatch fixture document.

    Args:
        (none)

    Returns:
        {meta, arguments_kinds, cases}; status is the reply's first token, null for no reply.

    Raises:
        ValueError - two cases share a name, or _record refuses one.
    """
    names = [case['name'] for case in _CASES]
    if len(set(names)) != len(names):
        raise ValueError('case names repeat: %s' % sorted({name for name in names if names.count(name) > 1}))
    replies = asyncio.run(_replies([case['request'] for case in _CASES]))
    return {
        'meta': {'source': 'src/glider/cc_client.py, src/glider/controller.py'},
        'arguments_kinds': _ARGUMENTS_KINDS,
        'cases': [_record(case, reply) for case, reply in zip(_CASES, replies, strict=True)],
    }
