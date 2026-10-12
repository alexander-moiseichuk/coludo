#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Firmware OTA for a Rust board (M15, src/avionics/firmware/src/ota.rs), over the network, on the pad:

  tools/ota.py --console /dev/ttyACM0 <avionics ELF>   # over the USB console (the operator: more reliable)
  tools/ota.py <board> <ip> <avionics ELF>             # over Wi-Fi, through the hub

Makes the app image the way firmware/flash.sh does (esptool elf2image, the same flags) and sends it:
- over the console: `console baud rate=2000000`, then `ota begin size= sha256=`, the image in `ota write offset=
  crc= data=<base64url>` lines of 192 bytes (a damaged one sent again), `ota end`, `ota commit confirm`, `reboot`,
  and back at 115200;
- over Wi-Fi: `POST /ota?sha256=<hex>` to the board's file server, then `ota commit confirm` and `reboot` through the
  hub (the upload starves the board's CC link a few seconds: the commit waits for it to come back).
Either way it proves the board now runs this build: `ota status` names the ELF's build id (the first 8 bytes of its
SHA-256, as esptool stamps it) and the running slot is the boot slot. The board must be in SETTING and disarmed.
"""
import argparse
import base64
import binascii
import collections
import hashlib
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(_ROOT, 'src', 'control'))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'glider'))
import cc_protocol as cc  # noqa: E402
from contract import transports  # noqa: E402

_PORT: int = 8000  # the board's file server (files.rs PORT)
_UPLOAD_TIMEOUT_S: float = 300.0  # the slot's erase, then ~2 MB over Wi-Fi
_RELINK_S: float = 90.0  # how long the board may stay off the hub after the upload
_CONSOLE_BAUD: int = 115200  # the console's rate at boot
_TRANSFER_BAUD: int = 2000000  # the console's rate for the transfer (the CH343P takes up to 6 Mbaud)
_CONSOLE_CHUNK: int = 192  # bytes an `ota write` line carries: ~300 characters, the whole line within the board's
# UART receive ring and FIFO (~384 B); 4 KB lines failed their CRC every time
_REBOOT_S: float = 12.0  # until the console answers after the reboot
_LINE_ATTEMPTS: int = 5  # sends of one `ota write` line before the transfer is given up
_ELF2IMAGE: tuple = ('esptool', '--chip', 'esp32p4', 'elf2image', '--flash-mode', 'qio', '--flash-freq', '80m',
                     '--flash-size', '32MB', '--min-rev-full', '100', '--max-rev-full', '199',
                     '--elf-sha256-offset', '0xb0')


def build_id(elf: str) -> str:
    """The build id `ota status` names: the first 8 bytes of the ELF's SHA-256, in hex."""
    with open(elf, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()[:16]


def named(line: str) -> dict:
    """A reply line's `key=value` params."""
    return dict(token.split('=', 1) for token in line.split() if '=' in token)


class _Console:
    """The board's USB console: one line, its `ok`/`err` answer (the log lines between left out)."""

    def __init__(self, port: str) -> None:
        import serial  # only here: the Wi-Fi path needs no pyserial
        self._serial = serial.Serial()
        self._serial.port = port
        self._serial.baudrate = _CONSOLE_BAUD
        self._serial.timeout = 0.05
        self._serial.dtr = True  # both held: releasing them one by one resets the board
        self._serial.rts = True
        self._serial.open()
        self._pending = b''
        self._seen = collections.deque(maxlen=60)  # the last lines that were no answer: a crash dump, the log

    def rate(self, baud: int) -> None:
        """Reopen the port's side at `baud` (the board changed first)."""
        self._serial.baudrate = baud
        self._pending = b''
        self._serial.reset_input_buffer()

    def ask(self, line: str, timeout: float = 15.0) -> object:
        """Send `line`, return its answer parsed (cc_protocol), or raise SystemExit on silence."""
        self._serial.write((line + '\n').encode())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            # what is waiting, or one byte: read(65536) waited out its whole timeout for every short reply
            self._pending += self._serial.read(max(1, self._serial.in_waiting))
            while b'\n' in self._pending:
                raw, self._pending = self._pending.split(b'\n', 1)
                text = raw.decode(errors='replace').strip()
                if text.startswith(('ok', 'err')):
                    return cc.parse(text)
                if text:
                    self._seen.append(text)
        raise SystemExit('no answer to %r within %.0f s; the console said last:\n  %s'
                         % (line.split(' data=')[0], timeout, '\n  '.join(self._seen)))


def _console_transfer(port: str, data: bytes, digest: str) -> dict:
    """The image over the console (see the module notes); `ota status` after the reboot."""
    console = _Console(port)
    try:
        reply = console.ask('console baud rate=%d' % _TRANSFER_BAUD, timeout=3.0)
    except SystemExit:
        # an earlier transfer left the console at the transfer rate (it holds until a reboot)
        console.rate(_TRANSFER_BAUD)
        reply = console.ask('console baud rate=%d' % _TRANSFER_BAUD)
    if reply.command != 'ok':
        raise SystemExit('console baud refused: %s' % reply.args)
    time.sleep(0.2)
    console.rate(_TRANSFER_BAUD)
    reply = console.ask('ota begin size=%d sha256=%s' % (len(data), digest), timeout=120.0)  # the slot's erase
    if reply.command != 'ok':
        raise SystemExit('ota begin refused: %s' % reply.args)
    started = time.monotonic()
    retried = 0
    for offset in range(0, len(data), _CONSOLE_CHUNK):
        part = data[offset:offset + _CONSOLE_CHUNK]
        line = 'ota write offset=%d crc=%08x data=%s' % (
            offset, binascii.crc32(part), base64.urlsafe_b64encode(part).rstrip(b'=').decode())
        # a line the UART damaged writes nothing (its CRC, its offset, or a word lost): send it again
        why = None
        for _ in range(_LINE_ATTEMPTS):
            try:
                reply = console.ask(line, timeout=5.0)
                why = reply.args
            except SystemExit as silence:  # its text carries the console's last lines (a crash dump)
                reply, why = None, str(silence)
            if reply is not None and reply.command == 'ok':
                break
            retried += 1
        else:
            raise SystemExit('ota write at %d failed %d times: %s' % (offset, _LINE_ATTEMPTS, why))
    print('lines sent again: %d' % retried)
    reply = console.ask('ota end', timeout=60.0)  # the image's verify
    print('sent %d bytes in %.1f s: %s %s' % (len(data), time.monotonic() - started, reply.command, reply.args))
    if reply.command != 'ok':
        raise SystemExit('ota end: %s' % reply.args)
    reply = console.ask('ota commit confirm')
    print(reply.command, reply.args)
    if reply.command != 'ok':
        raise SystemExit('ota commit: %s' % reply.args)
    console.ask('reboot')
    time.sleep(0.5)
    console.rate(_CONSOLE_BAUD)  # the next boot is back at 115200
    time.sleep(_REBOOT_S)
    return named(' '.join('%s=%s' % item for item in console.ask('ota status').named.items()))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--console', help='send it over this USB console port instead of Wi-Fi')
    parser.add_argument('target', nargs='+', help='<avionics ELF> with --console; else <board> <ip> <avionics ELF>')
    args = parser.parse_args()
    if args.console and len(args.target) != 1 or not args.console and len(args.target) != 3:
        parser.error('--console <port> <ELF>, or <board> <ip> <ELF>')
    elf = args.target[-1]
    wanted = build_id(elf)
    with tempfile.TemporaryDirectory() as work:
        image = os.path.join(work, 'app.bin')
        subprocess.run(_ELF2IMAGE + ('-o', image, elf), check=True, stdout=subprocess.DEVNULL)
        with open(image, 'rb') as stream:
            data = stream.read()
    digest = hashlib.sha256(data).hexdigest()
    if args.console:
        after = _console_transfer(args.console, data, digest)
        return _verdict(after, wanted, args.console)
    args.board, args.ip = args.target[0], args.target[1]
    import hitl_host  # here: it pulls in the sim, which only this path needs
    board = hitl_host._Board(transports.Hub(args.board))
    before = named(board.raw('ota status'))
    print('before: running %s, build %s' % (before.get('running'), before.get('build')))
    request = urllib.request.Request('http://%s:%d/ota?sha256=%s' % (args.ip, _PORT, digest), data=data,
                                     method='POST', headers={'Content-Type': 'application/octet-stream'})
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # the host's proxy hijacks the panda net
    started = time.monotonic()
    try:
        with opener.open(request, timeout=_UPLOAD_TIMEOUT_S) as response:
            answer = response.read().decode(errors='replace').strip()
    except urllib.error.HTTPError as error:
        why = error.read().decode(errors='replace').strip()
        raise SystemExit('POST /ota: HTTP %d %s' % (error.code, why)) from None
    print('sent %d bytes in %.1f s: %s' % (len(data), time.monotonic() - started, answer))
    if wanted not in answer:
        raise SystemExit('the board received build %s, not %s' % (answer, wanted))
    # the upload starves the board's CC link for a few seconds (3 MB over Wi-Fi into flash) and the hub may drop it:
    # the board keeps the received image, so wait for it to come back
    commit, deadline = None, time.monotonic() + _RELINK_S
    while commit is None:
        try:
            commit = board.raw('ota commit confirm')
        except (transports.NoAnswer, transports.Unavailable):
            if time.monotonic() > deadline:
                raise SystemExit('the board did not come back to the hub within %d s' % _RELINK_S) from None
            time.sleep(3)
    print(commit)
    if not commit.startswith('ok'):
        return 1
    board.reboot()
    return _verdict(named(board.raw('ota status')), wanted, args.board)


def _verdict(after: dict, wanted: str, board: str) -> int:
    """0 when the board runs build `wanted` from its boot slot."""
    print('after: running %s, boot %s, build %s' % (after.get('running'), after.get('boot'), after.get('build')))
    if after.get('build') != wanted or after.get('running') != after.get('boot'):
        print('FAIL: the board does not run build %s from its boot slot' % wanted)
        return 1
    print('OK: %s runs build %s from %s' % (board, wanted, after.get('running')))
    return 0


if __name__ == '__main__':
    sys.exit(main())
