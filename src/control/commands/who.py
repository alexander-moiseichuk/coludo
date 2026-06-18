# `who` — show this session's currently selected board.

import json

from . import command


@command('who', "show this session's selected board")
def who_command(hub, tokens, session) -> list:
    return ['from cc ok %s' % json.dumps({'selected': session['selected']})]
