# `cache <board>` — the Control-side cached properties for a board (config / inspect / stats /
# health), last-known values without touching the board. Defaults to the session's selected board.

import json

from . import command


@command('cache', 'cached board properties (config, inspect, stats, health)')
def cache_command(hub, tokens, session) -> list:
    target = tokens[1] if len(tokens) >= 2 else session.get('selected')
    if not target:
        return ['from cc err badargs cache-needs-a-board']
    board = hub.boards.get(target)
    if board is None:
        return ['from cc err noboard %s' % target]
    return ['from cc ok %s' % json.dumps(board.properties())]
