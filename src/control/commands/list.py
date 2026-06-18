# `list` — the connected boards and their last-known status.

import json

from . import command


@command('list', 'connected boards and their status')
def list_command(hub, tokens, session) -> list:
    return ['from cc ok %s' % json.dumps(hub.board_rows())]
