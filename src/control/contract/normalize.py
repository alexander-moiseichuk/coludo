"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The contract runner's --diff: the same cases answered by a board and by plan B on CPython (the `fake`
transport), compared reply by reply.

Two boards never answer alike to the byte: the uptime, the free heap, the clock and the boot's identity change
on every reading, and none of that says anything about the contract. Those fields are masked -- removed, by key
name at any depth -- before the replies are compared, so what is left is a difference in shape or meaning: a
key one side lacks, a different error code, a firmware that reports another config. Free text the case does
not pin (an error message after its code) is masked the same way: CPython's repr of a base64 error and Rust's
are both right. What a log or telemetry window carried is compared by its shape only: no two boards log alike.

Two fields differ between the Rust board and plan B by design -- its firmware version, and (until the config
store lands, M2) its config source. They are KNOWN differences: masked like the volatile ones, and listed apart
by known() so a report still shows them, never counted.
"""

import json

import cc_protocol as cc

from contract import cases

"""
The fields that differ between any two readings of the same board: masked wherever they occur. psram_free,
internal_free and sd_free_mb are the Rust board's health additions (a key one side lacks is no difference
either); the rest are the vitals and identity plan B reports too.
"""
VOLATILE: frozenset = frozenset(
    {'uptime', 'mem_free', 'psram_free', 'internal_free', 'sd_free_mb', 'temp', 'epoch', 'clock', 'session',
     'boot_id'}
)
KNOWN: frozenset = frozenset({'firmware_version', 'config_source'})  # differ by design: listed, never counted
CONTENT: frozenset = frozenset({'lines', 'samples'})  # a tee's content (log, tlm): compared by shape only
FREE: str = '<free text>'  # what an unpinned text argument reads as after masking
_MAX_LINES: int = 12  # differences listed per case; a config document that differs everywhere stops here
"""
Two floats this close (relative) are the same value. The Rust board computes in f32 (about 7 significant
digits) where plan B prints CPython's f64, so 1.764 against 1.7639999 is the port's documented f32 text, not
a difference in meaning.
"""
_F32_RELATIVE: float = 1e-6


def mask(value: object) -> object:
    """
    A JSON value without its VOLATILE and KNOWN keys, and with each CONTENT value reduced to its shape, at any depth.

    Args:
        value - a decoded JSON value.

    Returns:
        A masked copy; the input is not changed.
    """
    if isinstance(value, dict):
        return {key: _shape(item) if key in CONTENT else mask(item) for key, item in value.items()
                if key not in VOLATILE and key not in KNOWN}
    if isinstance(value, list):
        return [mask(item) for item in value]
    return value


def _shape(value: object) -> str:
    """What a tee's content compares as: `<list of text>` for the lines a window carried, else its type."""
    if isinstance(value, list) and all(isinstance(item, str) for item in value):
        return '<list of text>'
    return '<%s>' % type(value).__name__


def reply(line: str, free: tuple) -> dict:
    """
    One reply line, decoded and masked for comparison.

    Args:
        line - the raw reply line.
        free - the argument positions the case leaves free (their text is masked).

    Returns:
        {status, arguments}: each argument decoded from JSON when it is an object or a list, masked; a free
        one reads FREE. An unparsable line keeps its text as the status, so it still compares.
    """
    try:
        message = cc.parse(line)
    except Exception:
        return {'status': line, 'arguments': []}
    arguments = [FREE if position in free else mask(_decode(argument))
                 for position, argument in enumerate(message.args)]
    return {'status': message.command, 'arguments': arguments}


def _decode(argument: str) -> object:
    """One reply argument: its JSON value when it is an object or a list, else its text."""
    if argument[:1] in ('{', '['):
        try:
            return json.loads(argument)
        except ValueError:
            pass
    return argument


def _arguments(line: str) -> list:
    """A reply line's arguments, decoded and NOT masked ([] for an unparsable line)."""
    try:
        message = cc.parse(line)
    except Exception:
        return []
    return [_decode(argument) for argument in message.args]


def differences(board: object, fake: object, path: str = '') -> list:
    """
    Where two normalized values differ, one line per difference.

    Lists of text compare as sets (an object list or a degraded list is an inventory, and its order is the
    board's own); any other list compares item by item.

    Args:
        board - the board's value.
        fake - plan B's value for the same request.
        path - where these values sit, for the lines ('' at the top).

    Returns:
        Lines `<path>: board <value> / fake <value>`; empty when the values are equal.
    """
    if isinstance(board, dict) and isinstance(fake, dict):
        lines = []
        for key in sorted(set(board) | set(fake)):
            where = '%s.%s' % (path, key)
            if key not in fake:
                lines.append('%s: board only (%s)' % (where, _short(board[key])))
            elif key not in board:
                lines.append('%s: fake only (%s)' % (where, _short(fake[key])))
            else:
                lines.extend(differences(board[key], fake[key], where))
        return lines
    if isinstance(board, list) and isinstance(fake, list):
        if all(isinstance(item, str) for item in board + fake):
            only_board = sorted(set(board) - set(fake))
            only_fake = sorted(set(fake) - set(board))
            if not only_board and not only_fake:
                return []
            return ['%s: board only %s / fake only %s' % (path or '.', only_board, only_fake)]
        if len(board) != len(fake):
            return ['%s: board %d items / fake %d items' % (path or '.', len(board), len(fake))]
        lines = []
        for position, (board_item, fake_item) in enumerate(zip(board, fake)):
            lines.extend(differences(board_item, fake_item, '%s[%d]' % (path, position)))
        return lines
    if type(board) is not type(fake):  # 1 and 1.0 differ: the JSON type is part of the contract
        return ['%s: board %s / fake %s' % (path or '.', _short(board), _short(fake))]
    if isinstance(board, float) and abs(board - fake) <= _F32_RELATIVE * max(abs(board), abs(fake)):
        return []
    if board == fake:
        return []
    return ['%s: board %s / fake %s' % (path or '.', _short(board), _short(fake))]


def diff(board_results: list, fake_results: list) -> list:
    """
    The cases whose answers differ between the board and plan B, after masking.

    Results pair by (tier, name). A case plan B skipped (it has no hub and tees no log) has no answer to
    differ from and is left out; a case only the board skipped, or only one side ran, is listed with its
    outcome; a case both ran is compared on its outcome and then on its masked reply.

    Args:
        board_results - the board's cases.Result list.
        fake_results - plan B's cases.Result list, from the same tiers and allow flags.

    Returns:
        [(case label 'tier/name', [difference lines])] for every case that differs, in the board's order.
    """
    fake_by_name = {(result.tier, result.name): result for result in fake_results}
    found = []
    for result in board_results:
        label = '%s/%s' % (result.tier, result.name)
        reference = fake_by_name.pop((result.tier, result.name), None)
        if reference is None:
            found.append((label, ['board only: %s' % result.outcome]))
            continue
        if reference.outcome == cases.SKIP:
            continue
        lines = []
        if result.outcome != reference.outcome:
            lines.append('outcome: board %s / fake %s' % (result.outcome, reference.outcome))
        if result.replies or reference.replies:
            board_replies = [reply(line, result.free) for line in result.replies]
            fake_replies = [reply(line, reference.free) for line in reference.replies]
            lines.extend(differences(board_replies, fake_replies))
        if lines:
            found.append((label, lines[:_MAX_LINES] + (['...'] if len(lines) > _MAX_LINES else [])))
    for (tier, name), result in fake_by_name.items():
        found.append(('%s/%s' % (tier, name), ['fake only: %s' % result.outcome]))
    return found


def known(board_results: list, fake_results: list) -> list:
    """
    The KNOWN fields whose values differ between the board and plan B, case by case: reported, never counted.

    Args:
        board_results - the board's cases.Result list.
        fake_results - plan B's, from the same tiers and allow flags.

    Returns:
        Lines `<tier>/<case> <key>: board <value> / fake <value>`, in the board's order.
    """
    fake_by_name = {(result.tier, result.name): result for result in fake_results}
    lines = []
    for result in board_results:
        reference = fake_by_name.get((result.tier, result.name))
        if reference is None:
            continue
        board_values = _known_values([_arguments(line) for line in result.replies])
        fake_values = _known_values([_arguments(line) for line in reference.replies])
        for key in sorted(set(board_values) | set(fake_values)):
            if board_values.get(key) != fake_values.get(key):
                lines.append('%s/%s %s: board %s / fake %s' % (result.tier, result.name, key,
                                                               _short(board_values.get(key)),
                                                               _short(fake_values.get(key))))
    return lines


def _known_values(value: object) -> dict:
    """The KNOWN keys found anywhere in a value (a list of decoded replies), with the first value of each."""
    found = {}
    if isinstance(value, dict):
        for key, item in value.items():
            if key in KNOWN:
                found.setdefault(key, item)
            else:
                for inner_key, inner_value in _known_values(item).items():
                    found.setdefault(inner_key, inner_value)
    elif isinstance(value, list):
        for item in value:
            for inner_key, inner_value in _known_values(item).items():
                found.setdefault(inner_key, inner_value)
    return found


def _short(value: object) -> str:
    """A value as one short line of JSON, cut at 80 characters."""
    text = json.dumps(value, sort_keys=True)
    return text if len(text) <= 80 else text[:77] + '...'
