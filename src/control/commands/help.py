# `help` — list operator commands, or `help <command>` for one.

import json

from . import REGISTRY, command


@command('help', 'list commands, or `help <command>` for one')
def help_command(hub, tokens, session) -> list:
    if len(tokens) >= 2:
        spec = REGISTRY.get(tokens[1])
        if spec is None:
            return ['from cc err badargs no-such-command %s' % tokens[1]]
        return ['from cc ok %s' % json.dumps({tokens[1]: spec.help})]
    listing = {name: REGISTRY[name].help for name in sorted(REGISTRY)}
    listing['<board> <command>'] = 'route a command to a board (or all)'
    return ['from cc ok %s' % json.dumps(listing)]
