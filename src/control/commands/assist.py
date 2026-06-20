# `assist <board>` — push the host GPS position to a board's mission (sync the launch site), then
# persist it to the board's launch.config. Only sends a usable 3D fix; defaults to the selected
# board. Requires a GPS attached to the Control host (main.py --gps-device).

import json

from . import command


@command('assist', 'push the host GPS position to a board mission (launch-site sync)')
async def assist_command(hub, tokens, session) -> list:
    if hub.gps is None:
        return ['from cc err unsupported no-host-gps']
    target = tokens[1] if len(tokens) >= 2 else session.get('selected')
    if not target:
        return ['from cc err badargs assist-needs-a-board']
    board = hub.boards.get(target)
    if board is None or not board.online:
        return ['from cc err noboard %s' % target]
    position = hub.gps.position()
    if position is None:
        return ['from cc err nofix host-gps-not-3d']
    updated = await board.command('update', 'mission', json.dumps(position))
    if updated is None:
        return ['from cc err offline %s' % target]
    if updated.command != 'ok':
        return ['from cc err update %s' % ' '.join(str(a) for a in updated.args)]
    saved = await board.command('save-mission')  # persist into the board's launch.config
    persisted = bool(saved and saved.command == 'ok')
    return ['from cc ok %s' % json.dumps({'assisted': target, 'position': position, 'saved': persisted})]
