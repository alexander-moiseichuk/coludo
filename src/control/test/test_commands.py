"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the operator command registry (commands/) -- the six handlers that had no
test of their own: who / list / select / help / cache, plus the registry itself.

These are the operator's entire vocabulary on the console, and each is small enough that the risk is
not complexity but SILENCE: a handler that returns a malformed reply line, or a missing-argument path
that raises instead of answering `err badargs`, fails only in front of someone standing at a launch
site. So every handler is exercised on BOTH paths, and the replies are parsed rather than
string-matched, because a reply the operator's client cannot decode is as broken as no reply.

Run by `make test`.
"""

import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import commands  # noqa: E402


class _StubBoard:
    """The one Board surface these handlers touch."""

    def properties(self) -> dict:
        return {'config': {'board': {'id': 'taster'}}, 'health': {'mem_free': 90000}}


class _StubHub:
    """The Server surface the handlers touch: a board registry and a rows view."""

    def __init__(self, boards=None):
        self.boards = boards if boards is not None else {}

    def board_rows(self) -> list:
        return [{'id': name, 'connected': True} for name in sorted(self.boards)]


def _payload(lines: list) -> dict:
    """
    The decoded JSON body of a single `from cc ok <json>` reply.

    Asserts the shape as it goes: exactly one line, the `ok` verdict, and a body that actually parses.
    A handler returning two lines, or `ok` with unparseable text, would otherwise slip through a test
    that only checked for the substring 'ok'.
    """
    assert len(lines) == 1, lines
    head, _, body = lines[0].partition(' ok ')
    assert head == 'from cc', lines
    return json.loads(body)


def _error(lines: list) -> str:
    """The error kind of a single `from cc err <kind> ...` reply."""
    assert len(lines) == 1, lines
    parts = lines[0].split()
    assert parts[:3] == ['from', 'cc', 'err'], lines
    return parts[3]


def test_registry():
    registry = commands.load()
    assert registry is commands.REGISTRY
    for name in ('who', 'list', 'select', 'help', 'cache'):
        assert name in registry, name
    # every command must carry help text -- `help` publishes it, so an empty one is a silent hole
    missing = [name for name, spec in registry.items() if not spec.help]
    assert not missing, missing
    # load() is idempotent: the hub calls it once, but a re-import must not duplicate or drop entries
    assert commands.load() is registry and len(commands.load()) == len(registry)


def test_who_and_list():
    hub = _StubHub({'taster': _StubBoard(), 'spare': _StubBoard()})
    who = commands.REGISTRY['who'].handler

    assert _payload(who(hub, ['who'], {'selected': 'taster'}))['selected'] == 'taster'
    # NEGATIVE: nothing selected yet -> reports None rather than inventing a board or raising
    assert _payload(who(hub, ['who'], {'selected': None}))['selected'] is None

    rows = _payload(commands.REGISTRY['list'].handler(hub, ['list'], {'selected': None}))
    assert [row['id'] for row in rows] == ['spare', 'taster']


def test_select():
    select = commands.REGISTRY['select'].handler
    session = {'selected': None}

    assert _payload(select(_StubHub(), ['select', 'taster'], session))['selected'] == 'taster'
    assert session['selected'] == 'taster', 'select must persist onto the session, not just reply'
    # NEGATIVE: no board named -> badargs, and the previous selection is left ALONE. Clobbering it to
    # None on a typo would silently redirect the operator's next unqualified command.
    assert _error(select(_StubHub(), ['select'], session)) == 'badargs'
    assert session['selected'] == 'taster'


def test_help():
    commands.load()
    help_handler = commands.REGISTRY['help'].handler

    listing = _payload(help_handler(_StubHub(), ['help'], {'selected': None}))
    assert 'who' in listing and 'select' in listing
    assert '<board> <command>' in listing, 'the routing form must be advertised, it has no module'

    one = _payload(help_handler(_StubHub(), ['help', 'who'], {'selected': None}))
    assert list(one) == ['who'] and one['who']

    # NEGATIVE: an unknown name is an answerable error, not a KeyError
    assert _error(help_handler(_StubHub(), ['help', 'nosuch'], {'selected': None})) == 'badargs'


def test_cache():
    hub = _StubHub({'taster': _StubBoard()})
    cache = commands.REGISTRY['cache'].handler

    assert 'config' in _payload(cache(hub, ['cache', 'taster'], {'selected': None}))
    # falls back to the session's sticky board when no target is given
    assert 'health' in _payload(cache(hub, ['cache'], {'selected': 'taster'}))
    # NEGATIVE: neither an argument nor a selection -> badargs; a board that is not connected -> noboard
    assert _error(cache(hub, ['cache'], {'selected': None})) == 'badargs'
    assert _error(cache(hub, ['cache', 'ghost'], {'selected': None})) == 'noboard'


commands.load()
test_registry()
test_who_and_list()
test_select()
test_help()
test_cache()
print('ok: commands -- registry load/idempotence/help-text, who, list, select, help, cache, '
      'each with its missing-argument and unknown-target path')
