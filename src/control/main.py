# main.py — CLI entry point for the Control hub. Run it headless on a LAN box (it binds 0.0.0.0 by
# default) and telnet / browse to it from another workstation, instead of opening a browser locally.
#
#   python3 main.py [--host H] [--port N] [--operator-port N] [--web-port N]   (--help for all)

import argparse
import asyncio

import gps as gps_mod
import server


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Coludo Control hub — boards, operator console, web dashboard.')
    parser.add_argument('--host', default='0.0.0.0',
                        help='bind address (default 0.0.0.0 — all interfaces, reachable across the LAN)')
    parser.add_argument('--port', type=int, default=1234, help='board listener port (default 1234)')
    parser.add_argument('--operator-port', type=int, default=1235,
                        help='telnet operator console port (default 1235)')
    parser.add_argument('--web-port', type=int, default=8080, help='HTTP + SSE dashboard port (default 8080)')
    parser.add_argument('--gps-device', default=None,
                        help='serial GPS for launch-site assist, e.g. /dev/ttyUSB0 (off by default)')
    parser.add_argument('--gps-baud', type=int, default=9600, help='host GPS baud (default 9600)')
    return parser.parse_args()


async def _run(args, hub) -> None:
    """Run the hub, plus the host GPS reader when a --gps-device is configured."""
    if hub.gps is not None:
        await asyncio.gather(hub.run(), hub.gps.serve(args.gps_device, args.gps_baud))
    else:
        await hub.run()


def main() -> None:
    args = _parse_args()
    gps = gps_mod.Gps() if args.gps_device else None
    hub = server.Server(host=args.host, port=args.port, operator_port=args.operator_port,
                        web_port=args.web_port, gps=gps)
    print('control :: hub on %s — boards:%d operators:%d web:%d%s (Ctrl-C to stop)' % (
        args.host, args.port, args.operator_port, args.web_port,
        ' gps:%s' % args.gps_device if gps else ''))
    try:
        asyncio.run(_run(args, hub))
    except KeyboardInterrupt:
        print('\ncontrol :: stopped')


if __name__ == '__main__':
    main()
