# `log <board> [ms|off]` — stream a board's logs to the Control console and the /logs SSE feed.
# `log <board>` starts at 1000 ms, `log <board> 500` sets the cadence, `log <board> off` (or `0`)
# stops it. Defaults to the session's selected board. Each line shows on the console as
# `<board>: <line>`. Poll model: the hub re-sends the board-facing `log <ms>` each tick; the board
# collects only while polled (a lost link lapses the window and the board stops on its own).

import json

from . import command


@command('log', 'stream a board log: `log <board> [ms]` to start (default 1000), `off` to stop')
async def log_command(hub, tokens, session) -> list:
    target = tokens[1] if len(tokens) >= 2 else session.get('selected')
    if not target:
        return ['from cc err badargs log-needs-a-board']
    client = hub.boards.get(target)
    if client is None or not client.online:
        return ['from cc err noboard %s' % target]
    arg = tokens[2] if len(tokens) >= 3 else '1000'
    if arg == 'off':
        await hub.stop_stream(target)
        return ['from cc ok %s' % json.dumps({'log': target, 'streaming': False})]
    try:
        interval_ms = int(arg)
    except ValueError:
        return ['from cc err badargs log <board> [ms|off]']
    if interval_ms <= 0:
        await hub.stop_stream(target)
        return ['from cc ok %s' % json.dumps({'log': target, 'streaming': False})]
    hub.start_stream(client, interval_ms)
    return ['from cc ok %s' % json.dumps({'log': target, 'interval_ms': interval_ms})]
