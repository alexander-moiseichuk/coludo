"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

`select <board>` -- set this session's sticky target; a later bare command routes to it.
"""

import json

from . import command


@command('select', "set this session's sticky target board")
def select_command(hub, tokens, session) -> list:
    if len(tokens) < 2:
        return ['from cc err badargs select-needs-a-board']
    # a typo must not become the sticky target: it would surface only on the NEXT command, as noboard
    if tokens[1] not in hub.boards:
        return ['from cc err noboard %s' % tokens[1]]
    session['selected'] = tokens[1]
    return ['from cc ok %s' % json.dumps({'selected': tokens[1]})]
