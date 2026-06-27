#!/usr/bin/env python3
# cc.py -- script a board over CC from the shell, through the running Control hub.
#
# The hub (src/control/main.py) exposes POST /api/cmd on :8080: it relays one CC command to a named
# board and returns the board's reply as JSON. This wraps that into one line so on-board tests do not
# need the interactive console (telnet 1235 / rshell) -- every CC command becomes a scriptable call
# with a meaningful exit code (0 == the board replied `ok`/`pong`/`iam`, non-zero == `err` or no board):
#
#   tools/cc.py taster verify                 # pre-flight pass/fail (exit code is the verdict)
#   tools/cc.py taster probe imu_lsm6dso32    # one device self-test
#   tools/cc.py taster inspect mission        # an inspectable's snapshot
#   tools/cc.py taster tlm 2000               # telemetry rows buffered since the last tlm
#   tools/cc.py taster get-config board       # the running board config
#   tools/cc.py taster set-config board @board.config   # push a config file (@path -> its contents)
#   tools/cc.py taster update mission '{"launch_id":"flight.8"}'
#
# A param of the form @path is replaced by that file's contents (for the JSON-heavy config commands);
# everything else is passed verbatim. The hub encodes each param on the wire (JSON -> base64), so pass
# plain strings. Reply args are printed decoded -- a single JSON arg is pretty-printed.

import argparse
import json
import sys
import urllib.error
import urllib.request

_OK_STATUSES: tuple = ('ok', 'pong', 'iam')


def _resolve(param: str) -> str:
    """A `@path` param becomes that file's contents (config push); anything else is verbatim."""
    if param.startswith('@'):
        with open(param[1:]) as handle:
            return handle.read().strip()
    return param


# The hub is on the trusted LAN (or localhost), so never route through an HTTP proxy -- a system
# http_proxy would otherwise hijack the request and return a proxy error page instead of the board reply.
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def _payload(raw: bytes, code: int) -> dict:
    """Decode an /api/cmd response body to a dict; a non-JSON body becomes a synthetic error payload."""
    try:
        return json.loads(raw)
    except ValueError:
        return {'error': 'HTTP %d: %s' % (code, raw.decode(errors='replace').strip()[:200])}


def _post(host: str, port: int, board: str, command: str, params: list) -> tuple:
    """POST one command to the hub's /api/cmd. Returns (http_status, payload_dict). Never raises on an
    HTTP error status -- the hub reports board/command problems as JSON with a 4xx/5xx code."""
    body = json.dumps({'board': board, 'command': command, 'params': params}).encode()
    request = urllib.request.Request('http://%s:%d/api/cmd' % (host, port), data=body,
                                     headers={'Content-Type': 'application/json'})
    try:
        with _OPENER.open(request) as response:
            return response.status, _payload(response.read(), response.status)
    except urllib.error.HTTPError as error:
        return error.code, _payload(error.read(), error.code)
    except urllib.error.URLError as error:
        print('cc: cannot reach hub at %s:%d (%s) -- is `python3 main.py` running?' % (host, port, error.reason),
              file=sys.stderr)
        raise SystemExit(2) from None


def _render(payload: dict) -> None:
    """Print a board reply: each arg decoded, a lone JSON arg pretty-printed; errors clearly marked."""
    status = payload.get('status')
    args = payload.get('args', [])
    if 'error' in payload:  # a hub-level problem (no such board, offline, bad json) -- not a board reply
        print('cc: %s' % payload['error'], file=sys.stderr)
        return
    if len(args) == 1:
        try:
            print('%s %s' % (status, json.dumps(json.loads(args[0]), indent=2, sort_keys=True)))
            return
        except (ValueError, TypeError):
            pass
    print(' '.join([status] + [str(a) for a in args]) if status else '(no reply)')


def main() -> int:
    parser = argparse.ArgumentParser(description='Run one CC command on a board via the Control hub.')
    parser.add_argument('--host', default='127.0.0.1', help='hub host (default 127.0.0.1)')
    parser.add_argument('--port', type=int, default=8080, help='hub web port (default 8080)')
    parser.add_argument('board', help='target board id (e.g. taster)')
    parser.add_argument('command', help='CC command (verify, probe, inspect, get-config, ...)')
    parser.add_argument('params', nargs='*', help='positional params; @path is replaced by file contents')
    args = parser.parse_args()

    params = [_resolve(param) for param in args.params]
    http_status, payload = _post(args.host, args.port, args.board, args.command, params)
    _render(payload)
    # exit code is the verdict: 0 only when the board itself replied ok/pong/iam, so tests can chain
    return 0 if http_status == 200 and payload.get('status') in _OK_STATUSES else 1


if __name__ == '__main__':
    raise SystemExit(main())
