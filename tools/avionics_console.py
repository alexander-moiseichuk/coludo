#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Talk to an avionics (Rust) board's console over its USB-serial port (UART0, the CH343P).

The console speaks the CC line protocol (doc/specs/cc-protocol.md), so a command is `<command> [params]`
with the params encoded exactly as the hub encodes them; every line the board prints in the window comes
back, log lines included. Opening the port leaves DTR/RTS released, so the board is NOT reset -- unless
--reset asks for exactly that, which then shows the boot log.

  tools/avionics_console.py whoami
  tools/avionics_console.py --wifi panda          # the password comes from src/glider/panda.creds
  tools/avionics_console.py --reset --wait 8      # reboot and show the boot log
"""

import argparse
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
import cc_protocol as cc  # noqa: E402 -- the shared protocol module lives in src/glider

_CREDS_DIR: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider')


def _password(ssid: str) -> str:
    """
    The Wi-Fi password for `ssid`, from the gitignored src/glider/<ssid>.creds (first line).

    Args:
        ssid - the network name.

    Returns:
        The password; exits with a message when the file is missing.
    """
    path = os.path.join(_CREDS_DIR, '%s.creds' % ssid)
    try:
        with open(path) as creds:
            return creds.readline().strip()
    except OSError:
        sys.exit('no %s -- the password file the board config uses' % path)


def _open(port: str, reset: bool) -> serial.Serial:
    """
    Open the console port without the auto-reset pulse; with `reset`, pulse EN on purpose.

    The board's auto-reset circuit pulls EN low only while RTS is asserted and DTR is not. Linux asserts
    both on open, in one request, which is harmless; releasing them afterwards goes through "RTS alone"
    (pyserial updates DTR first) and resets the board. So both stay asserted, and a reset is DTR released
    with RTS pulsed.

    Args:
        port - the serial device.
        reset - True to hard-reset the board (RTS drives EN through the auto-reset circuit).

    Returns:
        The open port.
    """
    link = serial.Serial()
    link.port = port
    link.baudrate = 115200
    link.timeout = 0.1
    link.dtr = True
    link.rts = True
    link.open()
    if reset:
        link.dtr = False  # RTS alone -> EN low
        time.sleep(0.1)
        link.rts = False  # EN released with DTR released, so the boot strap reads high: a normal boot
    return link


def _relay(link: serial.Serial, seconds: float) -> None:
    """
    Print every line the board sends for `seconds`.

    Args:
        link - the open port.
        seconds - how long to listen.
    """
    deadline = time.monotonic() + seconds
    pending = b''
    while time.monotonic() < deadline:
        pending += link.read(512)
        while b'\n' in pending:
            line, pending = pending.split(b'\n', 1)
            print(line.decode(errors='replace').rstrip('\r'))
    if pending:
        print(pending.decode(errors='replace'))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', nargs='*', help='the console command and its params')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--wait', type=float, default=2.0, help='seconds to print what comes back')
    parser.add_argument('--wifi', metavar='SSID', help='store this network (password from src/glider/SSID.creds)')
    parser.add_argument('--reset', action='store_true', help='hard-reset the board first (shows the boot log)')
    args = parser.parse_args()

    if args.wifi:
        line = cc.build('wifi', [args.wifi, _password(args.wifi)])
    elif args.command:
        line = cc.build(args.command[0], args.command[1:])
    else:
        line = None
    link = _open(args.port, args.reset)
    if line is not None:
        if args.reset:
            _relay(link, args.wait)  # let the boot finish before talking
        link.write((line + '\n').encode())
        print('> %s' % (line if not args.wifi else 'wifi %s ********' % args.wifi))
    _relay(link, args.wait)
    link.close()


if __name__ == '__main__':
    main()
