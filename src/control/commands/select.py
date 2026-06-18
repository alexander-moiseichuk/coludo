# `select <board>` — set this session's sticky target; a later bare command routes to it.

import json

from . import command


@command('select', "set this session's sticky target board")
def select_command(hub, tokens, session) -> list:
    if len(tokens) < 2:
        return ['from cc err badargs select-needs-a-board']
    session['selected'] = tokens[1]
    return ['from cc ok %s' % json.dumps({'selected': tokens[1]})]
