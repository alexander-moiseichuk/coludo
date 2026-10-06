"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

How the contract runner reaches a board. Every transport sends one board-facing request line (the line a
board receives after the hub strips the routing id) and returns the reply lines it got back:

  exchange(line) -> [reply line, ...]   [] when the board sent nothing (the answer to an empty line)
  await_reboot()                        give an `ok` to reboot the time its reset needs to fire
  close()                               release the transport
  name                                  what the report calls it
  tees                                  whether the board copies its log to CC (`log <ms>`) at all

An answer after a reboot proves nothing by itself: a board answers until its reset fires, 200 ms after the `ok`,
and one that never resets answers forever. The runner proves the reset with a whoami from a new boot
(cases._await_new_boot); the transports only wait.

Three of them:
  * Fake -- the REAL plan B dispatcher (src/glider/cc_client.py) on CPython, through the golden shims
    (src/avionics/golden/shims.py), in a child process (contract/plan_b.py) spoken to over its pipes. It is
    the reference the Rust board is compared with, and it is safe: every file it writes lives in a scratch
    directory and its reset is intercepted.
  * Hub -- the live hub's HTTP bridge, POST /api/cmd {board, command, params} (src/control/web.py). The hub
    rebuilds the line from the command and its decoded params, so a line it cannot rebuild verbatim (an empty
    line, a malformed base64 token) raises Unreachable instead of testing something else. One reply per
    request is all /api/cmd can show: a second reply line stays in the hub's socket. Its row() is the board's
    line in GET /api/boards, which the hub tier reads.
  * Serial -- the Rust board's console on UART0 (the CH343P USB-serial port), which speaks the same line
    protocol. ESP-IDF log lines and the ROM's boot banner share the port and are filtered out of the replies.
"""

import contextlib
import json
import os
import subprocess
import sys
import tempfile
import time
import typing
import urllib.error
import urllib.request

import board  # the hub's own per-command reply deadlines (board.timeout_for)
import cc_protocol as cc

HUB_URL: str = 'http://127.0.0.1:8080'  # the hub's web bridge (src/control/main.py --web-port default)

_PLAN_B: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plan_b.py')  # the fake board's program
_STDERR_LINES: int = 5  # of a dead fake board's stderr, quoted in the error
_CHILD_EXIT_S: float = 10.0  # a fake board exits at once on EOF; past this it is killed

_HTTP_TIMEOUT_S: float = 40.0  # past the hub's own 30 s for the fin-sweeping commands, so the hub answers first
"""
A rebooting board resets 200 ms after its `ok`. The hub needs a moment more to see the link go, and the console
to scroll the ROM banner past, before a request can only reach the new boot.
"""
_REBOOT_SETTLE_S: float = 3.0

_BAUD: int = 115200  # the console UART (tools/avionics_console.py)
_READ_TIMEOUT_S: float = 0.05  # one serial read; the loops below own the real deadlines
"""
After a reply, the console is watched this long for a SECOND reply to the same line: one line in, one line out
is the contract, and a board that answers twice desyncs the hub's lockstep. It is also how long an empty line
waits to prove it gets no answer.
"""
_SETTLE_S: float = 0.3
_EMPTY_LINE_WAIT_S: float = 1.0
_LOG_PREFIXES: tuple = ('E (', 'W (', 'I (', 'D (', 'V (', '\x1b')  # ESP-IDF log lines (plain or colored)
"""
The ESP32-P4 ROM's boot banner, printed before any ESP-IDF logger exists: `ESP-ROM:esp32p4-eco2-20240710`,
`Build:Jul 10 2024`, `rst:0xc (SW_CPU_RESET),boot:0x30f (SPI_FAST_FLASH_BOOT)`, `Core0 Saved PC:0x...`,
`SPI mode:DIO, clock div:1`, `load:0x4ff33ce0,len:0x164c`, `entry 0x4ff29ae0`. None is a CC reply, which always
starts with a status.
"""
_ROM_PREFIXES: tuple = ('ESP-ROM:', 'Build:', 'rst:', 'Core0 Saved PC:', 'Core1 Saved PC:', 'Saved PC:',
                        'SPI mode:', 'SPIWP:', 'mode:', 'load:', 'entry 0x')


class Unreachable(Exception):
    """This transport cannot carry the request verbatim, so the case is skipped rather than run as another."""


class NoAnswer(Exception):
    """The board did not answer: timed out, dropped off the hub, or never came back from a reboot."""


class Unavailable(Exception):
    """The transport cannot reach a board at all (no hub, no such board online, no serial port)."""


class Transport(typing.Protocol):
    """What the runner needs of a transport (see the module docstring)."""

    name: str
    tees: bool

    def exchange(self, line: str) -> list:
        """Send one request line; the reply lines ([] for none)."""
        ...

    def await_reboot(self) -> None:
        """Give an `ok` to reboot the time its reset needs to fire (the runner then proves the new boot)."""
        ...

    def close(self) -> None:
        """Release the transport."""
        ...


class Fake:
    """
    Plan B's own dispatcher as a board: contract/plan_b.py in a child process, spoken to over its pipes.

    The child boots plan B as main.py does, minus the hardware, inside a scratch directory: its Controller is
    built and never set up, so the stage, the hold, the arm gate and disarm are plan B's own and no answer
    depends on a device (plan_b.py's docstring has the details). A child process, not this one, because the
    shims plan B needs are process-wide: they would put this process's clock in 2000.
    """

    name: str = 'fake (plan B on CPython)'
    tees: bool = False  # plan B's Recorder is never set up on the host, so its log tee stays empty

    def __init__(self) -> None:
        """
        Start plan B and wait until it has booted.

        Raises:
            Unavailable - plan B did not boot (its stderr is in the message).
        """
        self.reboots: int = 0  # the intercepted resets, as the child last counted them
        self._errors: typing.IO = tempfile.TemporaryFile(mode='w+')  # the child's stderr: prints, or why it died
        self._child: subprocess.Popen = subprocess.Popen([sys.executable, _PLAN_B], stdin=subprocess.PIPE,
                                                         stdout=subprocess.PIPE, stderr=self._errors, text=True)
        started = self._child.stdout.readline()
        if not started:
            reason = self._stderr_tail()
            self.close()
            raise Unavailable('plan B did not boot: %s' % reason)

    def _stderr_tail(self) -> str:
        """The last lines the child wrote to stderr (a traceback when it died)."""
        self._errors.seek(0)
        return ' | '.join(self._errors.read().strip().splitlines()[-_STDERR_LINES:]) or 'no output'

    def _ask(self, request: dict) -> str | None:
        """
        One protocol round trip with the child (plan_b.py's docstring has the protocol).

        Args:
            request - the protocol request.

        Returns:
            The reply line plan B answered, or None for no reply.

        Raises:
            NoAnswer - the child has exited.
        """
        try:
            self._child.stdin.write(json.dumps(request) + '\n')
            self._child.stdin.flush()
            answer = self._child.stdout.readline()
        except OSError:
            answer = ''
        if not answer:
            raise NoAnswer('plan B exited: %s' % self._stderr_tail())
        decoded = json.loads(answer)
        self.reboots = decoded['reboots']
        return decoded['reply']

    def exchange(self, line: str) -> list:
        """
        Hand one line to plan B's Dispatcher.handle(), as cc_client.Client.serve() does.

        Args:
            line - the board-facing request line.

        Returns:
            The reply split into wire lines (serve() writes it with one newline, so an embedded newline would
            be two lines on the wire); [] when plan B answers nothing.

        Raises:
            NoAnswer - plan B has exited.
        """
        reply = self._ask({'line': line})
        return [] if reply is None else reply.split('\n')

    def await_reboot(self) -> None:
        """
        Let plan B's deferred reset fire, and check it did (its next whoami then reports a new boot id).

        Raises:
            NoAnswer - the reset never fired: an `ok` to reboot that does not reboot.
        """
        before = self.reboots
        self._ask({'await_reboot': True})
        if self.reboots == before:
            raise NoAnswer('the reboot answered ok but the reset never fired')

    def close(self) -> None:
        """Close the child's stdin (it exits, removing its scratch directory) and wait for it."""
        with contextlib.suppress(OSError):  # a child that already died leaves a broken pipe to close
            self._child.stdin.close()
        try:
            self._child.wait(timeout=_CHILD_EXIT_S)
        except subprocess.TimeoutExpired:
            self._child.kill()
            self._child.wait()
        self._child.stdout.close()
        self._errors.close()


def hub_request(line: str) -> tuple:
    """
    The /api/cmd command and params that make the hub send exactly `line`.

    The hub runs Board.command(command, *params), which rebuilds the line with cc.build(): bare tokens stay,
    anything else is base64-encoded afresh. A request survives that only when decoding its tokens and building
    them again gives the same line.

    Args:
        line - the board-facing request line.

    Returns:
        (command, [param, ...]) with every param decoded.

    Raises:
        Unreachable - the hub would send a different line (an empty or blank line, a token that does not decode,
            a token it encodes differently, or extra whitespace).
    """
    tokens = line.split()
    if not tokens:
        raise Unreachable('the hub sends no empty line')
    try:
        params = [cc.decode(token) for token in tokens[1:]]
    except Exception as error:
        raise Unreachable('the hub re-encodes params: a token that does not decode (%r) cannot pass' % error) from None
    if cc.build(tokens[0], params) != line:
        raise Unreachable('the hub would rebuild this line differently')
    return tokens[0], params


def hub_reply(body: dict) -> str:
    """
    The reply line the board sent, rebuilt from the hub's /api/cmd answer {board, status, args}.

    Args:
        body - the decoded /api/cmd JSON (args already base64-decoded by the hub).

    Returns:
        The reply line, encoded as the board would send it.
    """
    return cc.build(body['status'], body.get('args', []))


class Hub:
    """
    A board behind the live hub, addressed by id through the web bridge's POST /api/cmd.

    The hub keeps its heartbeat going to the same board; its lockstep orders the runner's requests among its
    own, so neither sees the other's replies.
    """

    tees: bool = True

    def __init__(self, board_id: str, url: str = HUB_URL) -> None:
        """
        Address `board_id` through the hub at `url`, after checking it is online there.

        Args:
            board_id - the board's id (its `iam` id).
            url - the hub's web bridge.

        Raises:
            Unavailable - the hub does not answer, or lists no online board by that id.
        """
        self.name: str = 'hub %s, board %s' % (url, board_id)
        self._board_id: str = board_id
        self._url: str = url.rstrip('/')
        """
        No HTTP proxy: the hub is on the trusted LAN or localhost, and a system http_proxy would answer with
        its own error page instead of the board's reply (tools/cc.py does the same).
        """
        self._opener: urllib.request.OpenerDirector = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        try:
            row = self.row()
        except NoAnswer as error:
            raise Unavailable('cannot reach the hub at %s (%s) -- is `python3 main.py` running?'
                              % (url, error)) from None
        if row is None or not row.get('online'):
            raise Unavailable('the hub at %s has no online board %s' % (url, board_id))

    def row(self) -> dict | None:
        """
        The board's row in the hub's GET /api/boards (server.Server.board_rows): what the dashboard shows of it.

        Returns:
            The row, or None when the hub lists no board by this id.

        Raises:
            NoAnswer - the hub does not answer, or answers something that is not its row list.
        """
        try:
            with self._opener.open(self._url + '/api/boards', timeout=_HTTP_TIMEOUT_S) as response:
                rows = json.loads(response.read())
        except (urllib.error.URLError, OSError, ValueError) as error:
            raise NoAnswer('hub: %s' % error) from None
        if not isinstance(rows, list):
            raise NoAnswer('hub: /api/boards is not a list of rows')
        return next((row for row in rows if isinstance(row, dict) and row.get('id') == self._board_id), None)

    def exchange(self, line: str) -> list:
        """
        Send one request line through the hub.

        Args:
            line - the board-facing request line.

        Returns:
            [the reply line] (the hub returns exactly one reply per request).

        Raises:
            Unreachable - the hub cannot send this line verbatim (hub_request).
            NoAnswer - the hub reports the board offline or timed out, or does not answer itself.
        """
        command, params = hub_request(line)
        body = json.dumps({'board': self._board_id, 'command': command, 'params': params}).encode()
        request = urllib.request.Request(self._url + '/api/cmd', data=body,
                                         headers={'Content-Type': 'application/json'})
        try:
            with self._opener.open(request, timeout=_HTTP_TIMEOUT_S) as response:
                answer = json.loads(response.read())
        except urllib.error.HTTPError as error:
            text = error.read().decode(errors='replace')
            try:
                reason = json.loads(text).get('error', text)
            except (ValueError, AttributeError):
                reason = text.strip()[:200]
            raise NoAnswer('hub HTTP %d: %s' % (error.code, reason)) from None
        except (urllib.error.URLError, OSError, ValueError) as error:
            raise NoAnswer('hub: %s' % error) from None
        return [hub_reply(answer)]

    def await_reboot(self) -> None:
        """Let the reset fire and the hub see the link drop: a request after this reaches only the new boot."""
        time.sleep(_REBOOT_SETTLE_S)

    def close(self) -> None:
        """Nothing to release: every request is its own HTTP connection."""


def is_log_line(text: str) -> bool:
    """True for an ESP-IDF log line (`I (1234) tag: ...`, possibly colored) or a ROM boot line: not a CC reply."""
    return text.startswith(_LOG_PREFIXES + _ROM_PREFIXES)


def open_console(port: str) -> object:
    """
    Open the board's console port WITHOUT resetting the board (the approach of tools/avionics_console.py).

    The board's auto-reset circuit pulls EN low only while RTS is asserted and DTR is not. Linux asserts both
    on open, in one request, which is harmless; releasing them afterwards passes through "RTS alone" and resets
    the board. So both are asserted before the open and stay asserted.

    Args:
        port - the serial device (e.g. /dev/ttyACM0).

    Returns:
        The open serial.Serial.

    Raises:
        Unavailable - pyserial is missing or the port does not open.
    """
    try:
        import serial
    except ImportError:
        raise Unavailable('pyserial is not installed (pip install pyserial)') from None
    link = serial.Serial()
    link.port = port
    link.baudrate = _BAUD
    link.timeout = _READ_TIMEOUT_S
    link.dtr = True
    link.rts = True
    try:
        link.open()
    except (serial.SerialException, OSError) as error:
        raise Unavailable('cannot open %s (%s)' % (port, error)) from None
    return link


class Serial:
    """The Rust board's console: request lines out, reply lines back, ESP-IDF log lines dropped."""

    tees: bool = True

    def __init__(self, port: str) -> None:
        """
        Open the console on `port`, leaving the board running.

        Args:
            port - the serial device.

        Raises:
            Unavailable - the port does not open.
        """
        self.name: str = 'serial %s' % port
        self._link: object = open_console(port)  # a pyserial Serial (imported only when a console is opened)

    def exchange(self, line: str) -> list:
        """
        Write one request line and collect what comes back.

        Waits for the first reply up to the hub's own deadline for the command (board.timeout_for: 10 s, 30 s
        for the fin-sweeping ones), then _SETTLE_S more for a second one. An empty line waits
        _EMPTY_LINE_WAIT_S and expects nothing. Input already pending is discarded first: it belongs to no
        request of ours.

        Args:
            line - the board-facing request line.

        Returns:
            The reply lines, log lines and blank lines dropped.

        Raises:
            NoAnswer - a non-empty line got no reply in time.
        """
        self._link.reset_input_buffer()
        self._link.write((line + '\n').encode())
        blank = not line.strip()
        deadline = time.monotonic() + (_EMPTY_LINE_WAIT_S if blank else board.timeout_for(line))
        replies = []
        pending = b''
        while time.monotonic() < deadline:
            pending += self._link.read(512)
            while b'\n' in pending:
                raw, pending = pending.split(b'\n', 1)
                text = raw.decode(errors='replace').rstrip('\r')
                if not text.strip() or is_log_line(text):
                    continue
                if not replies:
                    deadline = time.monotonic() + _SETTLE_S  # the first reply: now watch for a second
                replies.append(text)
        if not replies and not blank:
            raise NoAnswer('no reply on the console within %.0f s' % board.timeout_for(line))
        return replies

    def await_reboot(self) -> None:
        """Let the reset fire and the ROM banner scroll by: a request after this reaches only the new boot."""
        time.sleep(_REBOOT_SETTLE_S)

    def close(self) -> None:
        """Close the port (DTR and RTS are released together, which does not reset the board)."""
        self._link.close()

