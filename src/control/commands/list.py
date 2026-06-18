# `list` — the connected boards and their last-known status.

import json

from . import command


@command('list', 'connected boards and their status')
def list_command(hub, tokens, session) -> list:
    rows = [
        {'id': board.id, 'online': board.online, 'state': board.info.get('state'),
         'config_id': board.info.get('config_id')}
        for board in hub.boards.values()
    ]
    return ['from cc ok %s' % json.dumps(rows)]
