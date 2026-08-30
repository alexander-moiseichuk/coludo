"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

CLI entry point for the Control hub. Run it headless on a LAN box (it binds 0.0.0.0 by default) and
telnet / browse to it from another workstation, instead of opening a browser locally.

  python3 main.py [--host H] [--port N] [--operator-port N] [--web-port N]   (--help for all)
"""

import argparse
import asyncio
import datetime

import gps as gps_mod
import server


def _log(message: str) -> None:
    """
    Console logger: every line stamped `YYYY-MM-DD HH:MM:SS`.

    No `control ::` prefix -- the timestamp carries the context. Wired into the hub so boards,
    operators and the web bridge share one logger.

    Args:
        message - the line to log.

    Returns:
        None; prints the stamped line to stdout.
    """
    print('%s %s' % (datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'), message))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Coludo Control hub — boards, operator console, web dashboard.')
    parser.add_argument('--host', default='0.0.0.0',
                        help='bind address (default 0.0.0.0 — all interfaces, reachable across the LAN)')
    parser.add_argument('--port', type=int, default=1234, help='board listener port (default 1234)')
    parser.add_argument('--operator-port', type=int, default=1235,
                        help='telnet operator console port (default 1235)')
    parser.add_argument('--web-port', type=int, default=8080, help='HTTP + SSE dashboard port (default 8080)')
    parser.add_argument('--gps-device', default='auto',
                        help="serial GPS for launch-site assist: 'auto' (default) picks the first "
                             "/dev/ttyUSB*, an explicit path overrides, 'off' disables")
    parser.add_argument('--gps-baud', type=int, default=9600, help='host GPS baud (default 9600)')
    return parser.parse_args()


def _resolve_gps_device(arg: str):
    """
    Resolve the --gps-device argument to a device path or None.

    So a USB GPS is used by default when present, without a flag.

    Args:
        arg - the raw --gps-device value.

    Returns:
        'auto' -> the first /dev/ttyUSB* if any (else None); 'off'/'none'/'' -> None; an explicit
        path -> itself.
    """
    if arg in ('off', 'none', ''):
        return None
    if arg == 'auto':
        import glob

        import serial

        """
        PROBE for NMEA before claiming a port. /dev/ttyUSB* is whatever happened to enumerate first --
        a printer, an SDR, the Luckfox UART -- and taking it blind meant the hub logged "host gps on
        /dev/ttyUSB0" and then "host gps lost", with the real receiver sitting unused on ttyUSB1.
        Worse, opening someone else's serial port can disturb it.

        A GPS emits `$G...` continuously, so a short listen is a definitive test. Each candidate gets
        one second; the first that talks NMEA wins, and if none does we return None rather than
        guessing -- an explicit --gps-device is the honest fallback.
        """
        for candidate in sorted(glob.glob('/dev/ttyUSB*')):
            try:
                with serial.Serial(candidate, 9600, timeout=0.25) as link:
                    for _attempt in range(4):        # ~1 s: a live receiver sends several sentences
                        if b'$G' in link.readline():
                            return candidate
            except Exception:
                continue                              # busy, permission-denied, not a serial device
        return None
    return arg


async def _run(args, hub) -> None:
    """
    Run the hub, plus the host GPS reader when a --gps-device is configured.

    Args:
        args - the parsed CLI namespace (for the GPS device / baud).
        hub - the server.Server to run.

    Returns:
        None; runs until cancelled.
    """
    if hub.gps is not None:
        """
        The hub is the job; the host GPS is an OPTIONAL extra. gather() couples them.

        A bare gather() propagates the FIRST exception and cancels the sibling, so anything escaping
        serve() -- it catches OSError, but not CancelledError or an unexpected error mid-read -- takes
        the board listener down with it. Losing the whole fleet because a USB GPS was unplugged is the
        wrong trade, and it is a field-day failure: that dongle gets knocked constantly.

        return_exceptions keeps the pair independent, and the GPS outcome is logged rather than
        swallowed so a dead assist is visible instead of merely absent.
        """
        results = await asyncio.gather(hub.run(),
                                       hub.gps.serve(args.gps_device, args.gps_baud),
                                       return_exceptions=True)
        for label, outcome in zip(('hub', 'host gps'), results):
            if isinstance(outcome, BaseException) and not isinstance(outcome, asyncio.CancelledError):
                hub.log('%s stopped: %r' % (label, outcome))
    else:
        await hub.run()


def main() -> None:
    args = _parse_args()
    args.gps_device = _resolve_gps_device(args.gps_device)  # 'auto' -> first /dev/ttyUSB* (or None)
    gps = gps_mod.Gps(log=_log) if args.gps_device else None
    hub = server.Server(host=args.host, port=args.port, operator_port=args.operator_port,
                        web_port=args.web_port, gps=gps, log=_log)
    _log('hub on %s — boards:%d operators:%d web:%d%s (Ctrl-C to stop)' % (
        args.host, args.port, args.operator_port, args.web_port,
        ' gps:%s' % args.gps_device if gps else ''))
    try:
        asyncio.run(_run(args, hub))
    except KeyboardInterrupt:
        _log('stopped')


if __name__ == '__main__':
    main()
