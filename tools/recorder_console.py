#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The flight recorder's card from the host, over its USB console (src/avionics/recorder, crate console): list,
measure, remove and pull its files.

  tools/recorder_console.py ls [dir]                  # a directory's entries and sizes
  tools/recorder_console.py df                        # the card's free and total space
  tools/recorder_console.py rm <path>                 # a file, or a directory with all it holds
  tools/recorder_console.py get <path> [-o <file>]    # one file, checked by its size and CRC-32
  tools/recorder_console.py pull <dir> [-o <outdir>]  # every file under <dir>, each checked

The port is found by its USB product (Espressif's "USB JTAG/serial debug unit"), never by probing every port with
esptool, whose connect resets any ESP32 it reaches (--port names one). It is opened with DTR and RTS both asserted:
releasing them one after the other passes through "RTS alone", which resets the board.

The recorder's status line (every 5 s) and its log lines interleave with the replies; the reader keeps only a reply's
own lines.
"""
import argparse
import base64
import binascii
import glob
import os
import re
import sys
import time

_PRODUCT: str = 'USB JTAG/serial debug unit'
_LOG_LINE: re.Pattern = re.compile(r'^[EWIDV] \(\d+\) ')
_BASE64_LINE: re.Pattern = re.compile(r'^[A-Za-z0-9+/]+={0,2}$')
_TIMEOUT_S: float = 30.0


def find_port() -> str:
    """
    The flight recorder's console port, by its USB product.

    Raises:
        SystemExit - none, or more than one, is plugged in.
    """
    ports = []
    for tty in sorted(glob.glob('/sys/class/tty/ttyACM*')):
        try:
            with open(os.path.join(tty, 'device', '..', 'product')) as product:
                if product.read().strip() == _PRODUCT:
                    ports.append('/dev/' + os.path.basename(tty))
        except OSError:
            continue
    if len(ports) != 1:
        raise SystemExit('flight recorder ports: %s -- name one with --port' % (ports or 'none'))
    return ports[0]


def reply_lines(text: str, command: str) -> list:
    """
    A reply's own lines out of the console's text: from the first line of `command`'s reply to its last (`<command>
    ::` or `err <command>`), the log and status lines in between left out, `\\r` stripped.

    Returns:
        The lines; [] when the reply has not ended yet.
    """
    lines, started = [], False
    for line in text.replace('\r', '').split('\n'):
        if _LOG_LINE.match(line):
            continue
        if not started:
            started = line.startswith((command + ' ', 'err ' + command, 'file ', 'files ::')) or \
                (command == 'get' and line.startswith('get '))
            if not started:
                continue
        lines.append(line)
        if line.startswith(('%s ::' % command, 'err %s' % command)) or (command == 'df' and line.startswith('df ')):
            return lines
    return []


def parse_ls(lines: list) -> list:
    """`ls` lines as (name, size) pairs; a directory's size is None, its name ending in `/`."""
    entries = []
    for line in lines:
        if not line.startswith('ls ') or line.startswith('ls ::'):
            continue
        item = line[3:]
        if item.endswith('/'):
            entries.append((item, None))
        else:
            name, _, size = item.rpartition(';')
            entries.append((name, int(size)))
    return entries


def parse_files(lines: list) -> list:
    """`files` lines as (path, size, crc32) triples."""
    files = []
    for line in lines:
        if line.startswith('file '):
            path, size, crc = line[5:].rsplit(';', 2)
            files.append((path, int(size), int(crc, 16)))
    return files


def decode_get(lines: list) -> bytes:
    """
    A `get` reply's bytes, checked against its size and its CRC-32.

    Raises:
        ValueError - a refusal, a cut reply, or bytes that do not match the size or the CRC.
    """
    if lines and lines[0].startswith('err '):
        raise ValueError(lines[0])
    if not lines or not lines[0].startswith('get ') or not lines[-1].startswith('get ::'):
        raise ValueError('a cut reply')
    size = int(lines[0].split()[1])
    data = b''.join(base64.b64decode(line) for line in lines[1:-1] if _BASE64_LINE.match(line))
    crc = int(lines[-1].split()[2], 16)
    if len(data) != size or binascii.crc32(data) != crc:
        raise ValueError('%d of %d bytes, crc %08x against %08x' % (len(data), size, binascii.crc32(data), crc))
    return data


class Console:
    """The flight recorder's console: one command, its reply's lines."""

    def __init__(self, port: str) -> None:
        import serial  # only here: the parsers above are host-testable without pyserial
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = 115200
        self._serial.timeout = 0.2
        self._serial.dtr = True
        self._serial.rts = True
        self._serial.open()
        time.sleep(0.3)
        self._serial.reset_input_buffer()

    def ask(self, line: str) -> list:
        """
        Send `line`, return its reply's lines.

        Raises:
            SystemExit - no whole reply within the timeout.
        """
        command = line.split()[0]
        self._serial.write((line + '\n').encode())
        text, started = '', time.monotonic()
        while time.monotonic() - started < _TIMEOUT_S:
            text += self._serial.read(1 << 16).decode(errors='replace')
            lines = reply_lines(text, command)
            if lines:
                return lines
        raise SystemExit('no whole reply to %r in %d s' % (line, _TIMEOUT_S))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--port', help='the console port (default: found by its USB product)')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('ls').add_argument('dir', nargs='?', default='')
    sub.add_parser('df')
    sub.add_parser('rm').add_argument('path')
    get = sub.add_parser('get')
    get.add_argument('path')
    get.add_argument('-o', '--out', help='the file to write (default: the path\'s last name)')
    pull = sub.add_parser('pull')
    pull.add_argument('dir')
    pull.add_argument('-o', '--out', default='.', help='the directory the files go under (default: .)')
    args = parser.parse_args()
    console = Console(args.port or find_port())
    if args.command == 'ls':
        for name, size in parse_ls(console.ask(('ls ' + args.dir).strip())):
            print('%12s  %s' % ('' if size is None else size, name))
    elif args.command == 'df':
        lines = console.ask('df')
        if lines[0].startswith('err'):
            raise SystemExit(lines[0])
        free, total = (int(value) for value in lines[0].split()[1:3])
        print('%.1f MB free of %.1f MB' % (free / 1e6, total / 1e6))
    elif args.command == 'rm':
        lines = console.ask('rm ' + args.path)
        print(lines[-1])
        return 0 if lines[-1].startswith('rm ::') else 1
    elif args.command == 'get':
        try:
            data = decode_get(console.ask('get ' + args.path))
        except ValueError as error:
            raise SystemExit(str(error)) from None
        out = args.out or os.path.basename(args.path.rstrip('/'))
        with open(out, 'wb') as stream:
            stream.write(data)
        print('%s: %d bytes' % (out, len(data)))
    else:
        files = parse_files(console.ask(('files ' + args.dir).strip()))
        failed = 0
        for path, size, crc in files:
            try:
                data = decode_get(console.ask('get ' + path))
            except ValueError as error:
                print('FAIL %s: %s' % (path, error))
                failed += 1
                continue
            out = os.path.join(args.out, path)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            with open(out, 'wb') as stream:
                stream.write(data)
            # a file being filed may have grown since `files` listed it: its listed size is then a prefix's
            note = '' if (len(data), binascii.crc32(data)) == (size, crc) else ' (grew since the listing)'
            print('%s: %d bytes%s' % (path, len(data), note))
        print('%d files, %d failed' % (len(files), failed))
        return 1 if failed else 0
    return 0


if __name__ == '__main__':
    sys.exit(main())
