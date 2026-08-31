"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

`gps` -- the host GPS fix status (3D + satellites), so the operator knows when the launch site has a
usable position. `gps <board>` also fetches that board's on-board GNSS (`inspect gnss`) and shows it
beside the host fix, to check what the on-board receiver delivers against the USB reference before
trusting it / using `assist`. Requires a GPS attached to the Control host (main.py --gps-device).
"""

import json

from . import command


@command('gps', 'host GPS fix; `gps <board>` also shows that board\'s on-board GNSS for comparison')
async def gps_command(hub, tokens, session) -> list:
    if hub.gps is None:
        return ['from cc err unsupported no-host-gps']
    host = hub.gps.status()
    target = tokens[1] if len(tokens) >= 2 else None  # explicit board -> compare; bare `gps` = host only
    if not target:
        return ['from cc ok %s' % json.dumps(host)]  # bare `gps`: the raw host fix (unchanged)
    board = hub.boards.get(target)
    if board is None or not board.online:
        return ['from cc err noboard %s' % target]
    resp = await board.command('inspect', 'gnss')  # the board's own fix/position
    # A malformed `ok` payload must not crash the handler. json.loads raises ValueError, which
    # server._handle logs as "handler gps crashed" and answers the operator with `internal` -- an
    # unhelpful reply to a board that is merely confused, and it happens on any garbled link.
    onboard = None
    if resp and resp.command == 'ok' and resp.args:
        try:
            onboard = json.loads(resp.args[0])
        except ValueError:
            return ['from cc err board sent a malformed gnss reply']
    return ['from cc ok %s' % json.dumps({'host': host, 'board': target, 'onboard': onboard})]
