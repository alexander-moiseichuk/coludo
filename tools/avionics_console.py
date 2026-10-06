#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The operator's console for an avionics (Rust) board, over its USB-serial port (UART0, the CH343P).

The board speaks the CC line protocol everywhere (doc/specs/cc-protocol.md), the Glider Command Language included
(src/avionics/firmware/src/gcl.rs): `command [sub] [word ...] name=value ...`. This tool writes exactly that --
each `name=value` goes as a named parameter, its value encoded as the hub encodes it (`base64:` for a space) -- and
renders the replies for a person: base64 decoded, JSON pretty-printed, `help` as a readable list.

  tools/avionics_console.py                                # interactive: history, tab completion from `help`
  tools/avionics_console.py help
  tools/avionics_console.py video start fps=25 bars
  tools/avionics_console.py -c 'stage' -c 'health'         # several commands, one attach
  tools/avionics_console.py --wifi panda                   # the password comes from src/glider/panda.creds
  tools/avionics_console.py --reset --wait 8               # reboot and show the boot log

Opening the port does NOT reset the board (DTR/RTS stay asserted); --reset asks for exactly that. Each command
waits for its reply line (`ok`, `err`, `iam`, `pong`), up to --timeout, printing the board's log lines meanwhile,
so a long `pace seconds=20` needs no tuning.
"""

import argparse
import json
import os
import re
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))
import cc_protocol as cc  # noqa: E402 -- the shared protocol module lives in src/glider

_CREDS_DIR: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider')
_HISTORY: str = os.path.expanduser('~/.coludo_console_history')
_REPLIES: tuple = ('ok', 'err', 'iam', 'pong')  # the first word of a reply line
_NAMED: re.Pattern = re.compile(r'^([a-z][a-z0-9_]*)=(.*)$')  # a `name=value` token, as the CC parser reads one


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


def _line(words: list) -> str:
    """
    One command line from typed words: positional words stay positional, `name=value` become named parameters.

    Args:
        words - the command and its words, as typed (shell-split or console-split).

    Returns:
        The protocol line, each value encoded as the hub encodes it.
    """
    positional = []
    named = {}
    for word in words[1:]:
        match = _NAMED.match(word)
        if match:
            named[match.group(1)] = match.group(2)
        else:
            positional.append(word)
    return cc.build(words[0], positional, named)


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


class _Board:
    """The open console: send one line, collect the board's lines until its reply comes."""

    def __init__(self, link: serial.Serial, timeout: float):
        self.link = link
        self.timeout = timeout
        self.pending = b''

    def _lines(self, seconds: float):
        """
        Yield each line the board sends within `seconds`.

        Args:
            seconds - how long to listen.

        Yields:
            Each line, decoded, without its line end.
        """
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pending += self.link.read(512)
            while b'\n' in self.pending:
                line, self.pending = self.pending.split(b'\n', 1)
                yield line.decode(errors='replace').rstrip('\r')

    def ask(self, line: str, echo: str = None) -> str | None:
        """
        Send one line and return its reply, printing the board's other lines (its log) meanwhile.

        Args:
            line - the protocol line.
            echo - what to show for it ('' shows nothing; None shows the line itself).

        Returns:
            The reply line, or None when none came within the timeout.
        """
        if echo != '':
            print('> %s' % (line if echo is None else echo))
        self.link.write((line + '\n').encode())
        for received in self._lines(self.timeout):
            if received.split(' ', 1)[0] in _REPLIES:
                return received
            print('  %s' % received)
        print('  (no reply in %.0f s)' % self.timeout)
        return None

    def listen(self, seconds: float) -> None:
        """
        Print what the board says for `seconds` (the boot log after a reset).

        Args:
            seconds - how long to listen.
        """
        for received in self._lines(seconds):
            print(received)


def _render(reply: str) -> str:
    """
    A reply line for a person: base64 decoded, JSON pretty-printed, `help` as a list, `name=value` aligned.

    Args:
        reply - the raw reply line.

    Returns:
        The text to print.
    """
    try:
        message = cc.parse(reply)
    except Exception:  # an unparsable reply is shown as it came
        return reply
    head = ' '.join([message.command] + [argument for argument in message.args if not argument.startswith(('{', '['))])
    documents = []
    for argument in message.args:
        if argument.startswith(('{', '[')):
            try:
                documents.append(json.loads(argument))
            except ValueError:
                documents.append(argument)
    if documents and isinstance(documents[-1], dict) and 'grammar' in documents[-1]:
        return _render_help(documents[-1])
    lines = [head]
    for document in documents:
        lines.append(json.dumps(document, indent=2, sort_keys=True))
    if message.named:
        width = max(len(name) for name in message.named)
        lines.extend('  %-*s %s' % (width, name, value) for name, value in message.named.items())
    return '\n'.join(lines)


def _render_help(document: dict) -> str:
    """
    The `help` reply as a readable list: each command, what it does, its words and parameters with defaults.

    Args:
        document - the help JSON.

    Returns:
        The text to print.
    """
    lines = ['grammar: %s' % document.get('grammar', '')]
    for command in document.get('commands', []):
        marks = [mark for mark, flag in (('SETTING only', command.get('setting_only')),
                                         ('console only', command.get('console_only'))) if flag]
        lines.append('')
        lines.append('%s  -- %s%s' % (command['command'], command['about'],
                                      ' [%s]' % ', '.join(marks) if marks else ''))
        for word in command.get('words', []):
            lines.append('    %-16s %s' % (word['name'], word['about']))
        for parameter in command.get('parameters', []):
            default = parameter.get('default')
            lines.append('    %-16s %s (%s%s)' % (parameter['name'] + '=', parameter['about'], parameter['accepts'],
                                                  ', default %s' % default if default is not None else ', required'))
    if document.get('cc'):
        lines.append('')
        lines.append('CC commands (doc/specs/cc-protocol.md): %s' % ' '.join(document['cc']))
    return '\n'.join(lines)


def _vocabulary(board: _Board) -> dict:
    """
    What tab completion offers, from the board's own `help`: each command word and what may follow it.

    Args:
        board - the open console.

    Returns:
        {command word: [subcommands, words and `name=` parameters]}.
    """
    reply = board.ask('help', echo='')
    vocabulary = {}
    try:
        document = json.loads(cc.parse(reply).args[0]) if reply else {}
    except (ValueError, IndexError):
        document = {}
    for command in document.get('commands', []):
        first, _, rest = command['command'].partition(' ')
        following = vocabulary.setdefault(first, [])
        if rest:
            following.append(rest)
        following.extend(word['name'] for word in command.get('words', []))
        following.extend(parameter['name'] + '=' for parameter in command.get('parameters', []))
    for name in document.get('cc', []):
        vocabulary.setdefault(name, [])
    vocabulary.setdefault('help', [])
    return vocabulary


def _interactive(board: _Board) -> None:
    """
    The interactive console: read a line, send it, render the reply; history across sessions, tab completion.

    Args:
        board - the open console.
    """
    import readline
    import shlex

    vocabulary = _vocabulary(board)

    def complete(text: str, state: int):
        words = readline.get_line_buffer().split()
        if not words or (len(words) == 1 and not readline.get_line_buffer().endswith(' ')):
            choices = sorted(name for name in vocabulary if name.startswith(text))
        else:
            choices = sorted(option for option in vocabulary.get(words[0], []) if option.startswith(text))
        return choices[state] if state < len(choices) else None

    readline.set_completer(complete)
    readline.parse_and_bind('tab: complete')
    readline.set_completer_delims(' ')
    try:
        readline.read_history_file(_HISTORY)
    except OSError:
        pass
    print('coludo console: %d commands; `help` lists them, Tab completes, Ctrl-D leaves' % len(vocabulary))
    try:
        while True:
            try:
                typed = input('coludo> ').strip()
            except EOFError:
                print()
                break
            if not typed:
                continue
            try:
                words = shlex.split(typed)
            except ValueError as error:
                print('  %s' % error)
                continue
            reply = board.ask(_line(words), echo='')
            if reply is not None:
                print(_render(reply))
    finally:
        readline.write_history_file(_HISTORY)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', nargs='*', help='one command and its words (none: the interactive console)')
    parser.add_argument('-c', dest='commands', action='append', default=[],
                        help='a command line to send (repeatable: several commands in one attach)')
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--timeout', type=float, default=35.0, help='seconds to wait for each reply')
    parser.add_argument('--wait', type=float, default=0.0, help='seconds to keep printing after the last reply')
    parser.add_argument('--wifi', metavar='SSID', help='store this network (password from src/glider/SSID.creds)')
    parser.add_argument('--raw', action='store_true', help='print replies exactly as the board sent them')
    parser.add_argument('--reset', action='store_true', help='hard-reset the board first (shows the boot log)')
    args = parser.parse_args()

    import shlex

    lines = []
    if args.wifi:
        lines.append((cc.build('wifi', [], {'ssid': args.wifi, 'password': _password(args.wifi)}),
                      'wifi ssid=%s password=********' % args.wifi))
    if args.command:
        lines.append((_line(args.command), None))
    lines.extend((_line(shlex.split(command)), None) for command in args.commands)
    board = _Board(_open(args.port, args.reset), args.timeout)
    if args.reset:
        board.listen(max(args.wait, 8.0) if not lines else 8.0)  # let the boot finish before talking
    if not lines and not args.reset:
        _interactive(board)
    for line, echo in lines:
        reply = board.ask(line, echo)
        if reply is not None:
            print(reply if args.raw else _render(reply))
    if args.wait and lines:
        board.listen(args.wait)
    board.link.close()


if __name__ == '__main__':
    main()
