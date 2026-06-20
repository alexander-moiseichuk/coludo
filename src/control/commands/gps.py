# `gps` — the host GPS fix status (3D + satellites), so the operator knows when the launch site has
# a usable position. Requires a GPS attached to the Control host (main.py --gps-device).

import json

from . import command


@command('gps', 'host GPS fix status (3D fix + satellites)')
def gps_command(hub, tokens, session) -> list:
    if hub.gps is None:
        return ['from cc err unsupported no-host-gps']
    return ['from cc ok %s' % json.dumps(hub.gps.status())]
