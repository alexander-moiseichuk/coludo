"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

GNSS antenna checker -- ESP32-C6 SuperMini, MicroPython >= 1.29: an ATGM336H on UART(2) (TX -> GPIO4,
RX <- GPIO5) and a NEO-6M on UART(1) (TX -> GPIO7, RX <- GPIO6), 9600 baud.

    power-on     session N starts by itself (1, 2, ... across power cycles), no button: once a module talks,
                 it gets the FLIGHT init then the diagnostics every ~10 s (GSV, GSA, the ATGM's antenna text),
                 exactly as drivers/atgm336h.py (hz 10) and drivers/neo6mv2.py (hz 5) send them -- the drivers
                 send the diagnostics too since this bench; its first 10 lines go to its file, `336.N` / `neo6.N`
    every 10 s   a status line per module: what it is doing, fix or not
    3D fix       a module's first 3D fix saves the uptime, the situation and its last 100 lines. The flight init
                 turns GSA off, so a fix counts as 3D on a GGA with quality > 0 and >= 4 satellites (the fewest
                 a 3D solution needs), or on a GSA mode 3 if a module prints GSA
    every 30 s   moment X: `336.N.x` / `neo6.N.x` are rewritten with the situation and the last 100 lines, so
                 pulling the battery keeps what each module was doing at most 30 s before
    LED          the SuperMini's RGB LED (WS2812, GPIO8): solid green while it works, solid red once a save
                 failed

Each NMEA line is saved as `<uptime ms>;<line verbatim>`.
Events are `start;<uptime ms>`, `talking;<uptime ms>` (or `silent;` after 5 s), `tx;<uptime ms>;<command>`
(UBX as hex), `fix3d;<uptime ms>;<s since start>`, `status;<uptime ms>;<s since start>;<summary>`,
`moment;<uptime ms>;<s since start>` and `situation;<summary>`.
The summary is `rmc=;mode=;quality=;sats=;hdop=;in_view=;cn0=;antenna=;utc=;lat=;lon=;alt_m=` from the latest
RMC (A = valid fix, V = none), GSA (none/2D/3D), GGA, GSV (satellites in view and the best four C/N0, dB-Hz,
from the last burst, <= ~10 s old) and the ATGM's antenna text (OK/OPEN/SHORT). Read out with

    mpremote connect /dev/ttyACM0 cp :336.1 :336.1.x :neo6.1 :neo6.1.x <dir>/        (every N)

A power-on for the read-out starts a session too: it is the one with no status lines.
"""

import os
import struct
import time

from machine import UART, Pin
from micropython import const
from neopixel import NeoPixel

_FIRST = const(10)      # lines saved at the session start, per module
_RECENT = const(100)    # lines saved at the 3D fix and in every moment, per module
_LED = const(8)         # the SuperMini's RGB LED (WS2812) data line
_TALK_MS = const(5000)  # how long a module may stay silent after power-on before it is configured anyway
_NOISE = const(512)     # bytes without a newline: noise, not NMEA -- dropped
_SATELLITES_3D = const(4)  # the fewest satellites a 3D solution needs
_STATUS_MS = const(10000)  # a status line per module this often
_MOMENT_MS = const(30000)  # the moment files are rewritten this often
_GREEN: tuple = (0, 40, 0)   # working
_RED: tuple = (40, 0, 0)     # a save failed

"""
The flight inits, command for command with their gaps (ms): drivers/atgm336h.py at hz 10 (every
launches/20261003 config) and drivers/neo6mv2.py at hz 5 (config_default). A str is an NMEA body, bytes a
UBX class + id + payload.
"""
_ATGM_FLIGHT: tuple = (
    ('PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0', 80),
    ('PCAS02,100', 80),
    ('PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0', 80),
    ('PMTK220,100', 80),
)
_NEO_FLIGHT: tuple = (
    ('PUBX,40,RMC,0,1,0,0,0,0', 40),
    ('PUBX,40,GGA,0,5,0,0,0,0', 40),
    ('PUBX,40,GLL,0,0,0,0,0,0', 40),
    ('PUBX,40,GSA,0,0,0,0,0,0', 40),
    ('PUBX,40,GSV,0,0,0,0,0,0', 40),
    ('PUBX,40,VTG,0,0,0,0,0,0', 40),
    (b'\x06\x08' + struct.pack('<HHH', 200, 1, 1), 40),
    (b'\x06\x24' + struct.pack('<HB', 0x0001, 8) + bytes(33), 40),  # UBX-CFG-NAV5: airborne < 4 g (since 10-05)
)
"""
The diagnostics, sent after the flight init: GSV, GSA and (ATGM) the antenna text, every ~10 s -- the flight's
PCAS03 again with those dividers raised from 0 to 99 (every 99th fix at 10 Hz), and $PUBX,40 GSA/GSV every
50th fix at 5 Hz. Both only choose what is printed, so acquisition and the RMC/GGA rates stay the flight's
(measured on the bench: 10 Hz RMC + 1 Hz GGA on the ATGM, 5 Hz + 1 Hz on the NEO, GSV every ~9.9 s). Every
fix would overflow 9600 baud at 10 Hz RMC. The flight drivers adopted them as measured here: each appends
these to its init (the divider 10 x hz, the ATGM's capped at 99), so _FLIGHT + _DIAGNOSTICS is what a
board sends at hz 10 / 5, and gnss.Gnss records the result as `<name>_sky.csv`.
"""
_ATGM_DIAGNOSTICS: tuple = (('PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0', 80),)
_NEO_DIAGNOSTICS: tuple = (('PUBX,40,GSA,0,50,0,0,0,0', 40), ('PUBX,40,GSV,0,50,0,0,0,0', 40))


def _frame(command) -> bytes:
    """An NMEA body as `$body*hh\\r\\n`, or a UBX class + id + payload with its sync, length and checksum."""
    if isinstance(command, str):
        checksum = 0
        for byte in command.encode():
            checksum ^= byte
        return ('$%s*%02X\r\n' % (command, checksum)).encode()
    body = command[:2] + struct.pack('<H', len(command) - 2) + command[2:]
    first = second = 0
    for byte in body:
        first = (first + byte) & 0xFF
        second = (second + first) & 0xFF
    return b'\xb5\x62' + body + bytes((first, second))


class _Module:
    """One GNSS module: its UART, its last _RECENT lines, the latest GGA and GSA mode, and its session state."""

    def __init__(self, name: str, uart: UART, init: tuple) -> None:
        self.name = name
        self.uart = uart
        self.init = init
        self.partial = b''
        self.recent = []
        self.first = []
        self.gga = b''
        self.mode = b''
        self.rmc = b''  # the latest RMC status: A = valid fix, V = none
        self.in_view = {}  # talker -> satellites in view, from its last whole GSV group
        self.cn0 = {}  # talker -> the C/N0 values (dB-Hz) of its last whole GSV group
        self.group = []  # the C/N0 values of the GSV group being received
        self.antenna = b''
        self.path = ''
        self.first_saved = False
        self.fixed = False  # a 3D fix seen since the session start
        self.fix_saved = False
        self.active = False  # no session yet

    def start(self, session: int) -> None:
        """Begin session `session`: a fresh file name, the first lines collected anew, nothing saved yet."""
        self.path = '%s.%d' % (self.name, session)
        self.first = []
        self.first_saved = False
        self.fixed = False
        self.fix_saved = False
        self.active = True

    def poll(self, now: int) -> None:
        """Take whatever arrived: every complete line is kept, stamped `now`, and read for GGA and the GSA mode."""
        data = self.uart.read()
        if not data:
            return
        self.partial += data
        while b'\n' in self.partial:
            line, self.partial = self.partial.split(b'\n', 1)
            line = line.rstrip(b'\r')
            self.recent.append((now, line))
            if len(self.recent) > _RECENT:
                self.recent.pop(0)
            if self.active and len(self.first) < _FIRST:
                self.first.append((now, line))
            kind = line[3:7]  # '$GNGGA,...': the talker is two letters
            if kind == b'GGA,':
                self.gga = line
                fields = line.split(b',')
                if len(fields) > 7 and fields[6] not in (b'', b'0') and fields[7].isdigit():
                    self.fixed |= int(fields[7]) >= _SATELLITES_3D
            elif kind == b'GSA,':
                fields = line.split(b',')
                if len(fields) > 2:
                    self.mode = fields[2]
                    # latched: the ATGM sends a GSA per system, and a later one must not hide a 3D fix
                    self.fixed |= fields[2] == b'3'
            elif kind == b'RMC,':
                fields = line.split(b',')
                if len(fields) > 2:
                    self.rmc = fields[2]
            elif kind == b'GSV,':
                self._gsv(line)
            elif kind == b'TXT,' and b'ANTENNA' in line:
                self.antenna = line.split(b'ANTENNA ')[-1].split(b'*')[0]
        if len(self.partial) > _NOISE:
            self.partial = b''

    def _gsv(self, line: bytes) -> None:
        """One GSV sentence: collect its C/N0 values; the group's last sentence replaces that talker's view."""
        fields = line.split(b'*')[0].split(b',')
        if len(fields) < 4 or not (fields[1].isdigit() and fields[2].isdigit()):
            return
        if fields[2] == b'1':
            self.group = []
        satellites = fields[4:]
        satellites = satellites[:len(satellites) - len(satellites) % 4]  # NMEA 4.1 appends a signal id
        for index in range(3, len(satellites), 4):
            if satellites[index].isdigit():
                self.group.append(int(satellites[index]))
        if fields[2] == fields[1]:
            talker = line[1:3]
            self.in_view[talker] = int(fields[3]) if fields[3].isdigit() else 0
            self.cn0[talker] = self.group
            self.group = []

    def summary(self) -> bytes:
        """What the module is doing: RMC status, GSA mode, GGA quality/satellites/HDOP, view, C/N0, antenna, where."""
        fields = self.gga.split(b',')
        if len(fields) < 10:
            fields = [b''] * 10
        mode = {b'1': b'none', b'2': b'2D', b'3': b'3D'}.get(self.mode, b'')  # empty: no GSA yet
        best = sorted([value for values in self.cn0.values() for value in values], reverse=True)[:4]
        return (b'rmc=' + self.rmc + b';mode=' + mode + b';quality=' + fields[6] + b';sats=' + fields[7] +
                b';hdop=' + fields[8] + b';in_view=' + str(sum(self.in_view.values())).encode() +
                b';cn0=' + b','.join(str(value).encode() for value in best) + b';antenna=' + self.antenna +
                b';utc=' + fields[1] + b';lat=' + fields[2] + fields[3] + b';lon=' + fields[4] + fields[5] +
                b';alt_m=' + fields[9])


def _append(path: str, lines: list, mode: str = 'ab') -> bool:
    """
    Write `lines` (bytes, no newlines) to `path` and sync.

    Args:
        path - the file.
        lines - the lines to write.
        mode - 'ab' appends, 'wb' replaces.

    Returns:
        True once on flash; False when the write failed (printed).
    """
    try:
        with open(path, mode) as file:
            for line in lines:
                file.write(line + b'\n')
        if hasattr(os, 'sync'):
            os.sync()
        return True
    except OSError as error:
        print('gnss_bench :: %s NOT saved: %s' % (path, error))
        return False


def _stamped(lines: list) -> list:
    """(uptime ms, line) pairs as `<ms>;<line>`."""
    return [str(stamp).encode() + b';' + line for stamp, line in lines]


def _save(module: _Module, event: bytes, now: int, started: int) -> bool:
    """
    The module's 3D-fix save: the event with its uptime and seconds since the start, the situation, and the
    last _RECENT lines.

    Args:
        module - the module.
        event - b'fix3d'.
        now - ticks_ms() of the event.
        started - ticks_ms() of the session start.

    Returns:
        True once on flash.
    """
    seconds = '%.1f' % (time.ticks_diff(now, started) / 1000)
    head = event + b';' + str(now).encode() + b';' + seconds.encode()
    return _append(module.path, [head, b'situation;' + module.summary()] + _stamped(module.recent))


def _moment(module: _Module, now: int, started: int) -> bool:
    """
    Rewrite the module's moment file, `<path>.x`: the moment, the situation and the last _RECENT lines. It goes to
    a temporary file renamed over the old one, so a power cut mid-write leaves the previous moment whole.

    Args:
        module - the module.
        now - ticks_ms() of the moment.
        started - ticks_ms() of the session start.

    Returns:
        True once on flash.
    """
    seconds = '%.1f' % (time.ticks_diff(now, started) / 1000)
    head = b'moment;' + str(now).encode() + b';' + seconds.encode()
    temporary = module.path + '.tmp'
    if not _append(temporary, [head, b'situation;' + module.summary()] + _stamped(module.recent), 'wb'):
        return False
    try:
        os.rename(temporary, module.path + '.x')
        return True
    except OSError as error:
        print('gnss_bench :: %s.x NOT saved: %s' % (module.path, error))
        return False


def _configure(module: _Module) -> list:
    """Send `module` its flight init, the drivers' gaps between commands; returns the `tx` lines to save."""
    sent = []
    for command, gap_ms in module.init:
        module.uart.write(_frame(command))
        text = command.encode() if isinstance(command, str) else _frame(command).hex().encode()
        sent.append(b'tx;' + str(time.ticks_ms()).encode() + b';' + text)
        time.sleep_ms(gap_ms)
    return sent


def _sessions() -> int:
    """The highest session number on flash (0 when none), so numbering continues across power cycles."""
    highest = 0
    for name in os.listdir():
        if name.startswith('336.') and name[4:].isdigit():
            highest = max(highest, int(name[4:]))
    return highest


def _show(led: NeoPixel, colour: tuple) -> None:
    """Light the RGB LED `colour`."""
    led[0] = colour
    led.write()


def main() -> None:
    """Start session N at power-on and record until the power goes. Ctrl-C (a read-out) ends it."""
    led = NeoPixel(Pin(_LED, Pin.OUT), 1)
    _show(led, _GREEN)
    modules = [_Module('336', UART(2, 9600, rxbuf=4096), _ATGM_FLIGHT + _ATGM_DIAGNOSTICS),
               _Module('neo6', UART(1, 9600, tx=6, rx=7, rxbuf=4096), _NEO_FLIGHT + _NEO_DIAGNOSTICS)]
    session = _sessions() + 1
    started = time.ticks_ms()
    healthy = True
    for module in modules:
        module.start(session)
        healthy &= _append(module.path, [b'start;' + str(started).encode()])
    # configure a module once it talks: one powered up with the C6 may not listen yet
    while time.ticks_diff(time.ticks_ms(), started) < _TALK_MS and not all(module.recent for module in modules):
        for module in modules:
            module.poll(time.ticks_ms())
        time.sleep_ms(20)
    for module in modules:
        state = b'talking;' if module.recent else b'silent;'
        healthy &= _append(module.path, [state + str(time.ticks_ms()).encode()] + _configure(module))
    print('gnss_bench :: session %d started' % session)
    last_status = last_moment = time.ticks_ms()
    shown = None
    while True:
        now = time.ticks_ms()
        for module in modules:
            module.poll(now)
            if not module.first_saved and (len(module.first) == _FIRST or module.fixed):
                healthy &= _append(module.path, _stamped(module.first))  # fewer than 10 on an instant fix
                module.first_saved = True
            if module.fixed and not module.fix_saved:
                healthy &= _save(module, b'fix3d', now, started)
                module.fix_saved = True
                print('gnss_bench :: session %d: %s 3D fix after %.1f s' % (
                    session, module.name, time.ticks_diff(now, started) / 1000))
        if time.ticks_diff(now, last_status) >= _STATUS_MS:
            last_status = now
            seconds = ('%.1f' % (time.ticks_diff(now, started) / 1000)).encode()
            for module in modules:
                healthy &= _append(module.path, [b'status;' + str(now).encode() + b';' + seconds + b';' +
                                                 module.summary()])
        if time.ticks_diff(now, last_moment) >= _MOMENT_MS:
            last_moment = now
            for module in modules:
                healthy &= _moment(module, now, started)
        colour = _GREEN if healthy else _RED
        if colour != shown:
            _show(led, colour)
            shown = colour
        time.sleep_ms(20)


if __name__ == '__main__':
    main()
