# gps.py — host-side GPS assist for the Control hub (finding #10).
#
# The flight board carries its own GNSS (ATGM336H); a GPS plugged into the Control host (e.g.
# /dev/ttyUSB0) is an ASSIST, not the source of truth. Two jobs:
#   1. tell the operator when a usable fix is available — the ideal launch condition is a 3D fix
#      with 4+ satellites (so the board's own cold start has a good almanac/position seed);
#   2. hand a launch position to the board (operator `assist <board>` -> `update mission` +
#      `save-mission`, persisted in the board's launch.config) when the on-board GPS has no fix yet.
#
# Pure NMEA parsing (GGA position/sats, GSA 2D/3D mode) is split from the serial transport so it is
# unit-tested without hardware (test_gps.py); the Linux serial open + read loop is exercised by
# itest_gps.py against a real receiver. CPython 3.12, stdlib asyncio only — no pyserial.

import asyncio

IDEAL_SATELLITES: int = 4  # a 3D fix with this many satellites is the ideal launch condition


def _checksum_ok(sentence: str) -> bool:
    """Verify the NMEA `*hh` XOR checksum. Sentences without one are tolerated (some emit none)."""
    if '*' not in sentence:
        return True
    body, _, checksum = sentence[1:].partition('*')
    got = 0
    for character in body:
        got ^= ord(character)
    try:
        return got == int(checksum[:2], 16)
    except ValueError:
        return False


def _degrees(value: str, hemisphere: str):
    """NMEA ddmm.mmmm + N/S/E/W -> signed decimal degrees (None when the field is empty)."""
    if not value:
        return None
    dot = value.find('.')
    split = dot - 2  # the last two digits before the dot are minutes; the rest are whole degrees
    decimal = int(value[:split]) + float(value[split:]) / 60.0
    return -decimal if hemisphere in ('S', 'W') else decimal


class Fix:
    """The latest GNSS fix, accumulated from GGA (position/altitude/satellites) and GSA (2D/3D)."""

    def __init__(self):
        self.latitude = None  # decimal degrees, None until known
        self.longitude = None
        self.altitude = None  # metres MSL, None until known
        self.satellites: int = 0
        self.mode: int = 1  # GSA fix type: 1 none, 2 2D, 3 3D

    @property
    def fix_3d(self) -> bool:
        return self.mode >= 3

    @property
    def has_position(self) -> bool:
        return self.latitude is not None and self.longitude is not None

    @property
    def usable(self) -> bool:
        """The ideal launch condition: a 3D fix with enough satellites and an actual position."""
        return self.fix_3d and self.satellites >= IDEAL_SATELLITES and self.has_position


class Gps:
    """Host GPS reader: feed NMEA lines, expose the latest fix + a launch position for board assist."""

    def __init__(self, log=print):
        self.fix = Fix()
        self.log = log
        self.lines: int = 0  # accepted sentences (a liveness sign for the operator)

    def feed(self, line: str) -> bool:
        """Parse one NMEA sentence into the running fix. Returns False for non-NMEA, a bad checksum,
        an unhandled sentence, or a malformed field — robust to the line noise a serial GPS emits."""
        line = line.strip()
        if not line.startswith('$') or not _checksum_ok(line):
            return False
        parts = line[1:].split('*')[0].split(',')
        kind = parts[0][2:]  # drop the talker id (GP/GN/GL/...) -> GGA / GSA / RMC / ...
        try:
            if kind == 'GGA':
                self.fix.latitude = _degrees(parts[2], parts[3])
                self.fix.longitude = _degrees(parts[4], parts[5])
                self.fix.satellites = int(parts[7]) if parts[7] else 0
                self.fix.altitude = float(parts[9]) if parts[9] else None
            elif kind == 'GSA':
                self.fix.mode = int(parts[2]) if parts[2] else 1
            else:
                return False
        except (ValueError, IndexError):
            return False
        self.lines += 1
        return True

    def status(self) -> dict:
        """Operator-facing fix snapshot: is it a usable 3D fix, how many satellites, where."""
        fix = self.fix
        return {'usable': fix.usable, 'fix_3d': fix.fix_3d, 'satellites': fix.satellites,
                'latitude': fix.latitude, 'longitude': fix.longitude, 'altitude': fix.altitude,
                'lines': self.lines}

    def position(self):
        """The host position as a mission dict (latitude/longitude[/altitude]) when the fix is
        usable, else None — so `assist` only pushes a position worth trusting."""
        if not self.fix.usable:
            return None
        position = {'latitude': self.fix.latitude, 'longitude': self.fix.longitude}
        if self.fix.altitude is not None:
            position['altitude'] = self.fix.altitude
        return position

    async def run(self, reader: asyncio.StreamReader) -> None:
        """Feed every line from an NMEA stream until it ends (the read loop, transport-agnostic)."""
        while True:
            raw = await reader.readline()
            if not raw:
                return
            self.feed(raw.decode('ascii', 'ignore'))

    async def serve(self, device: str, baud: int = 9600) -> None:
        """Open the serial GPS and feed it forever (the wired host-assist path)."""
        reader = await open_serial(device, baud)
        self.log('control :: host gps on %s @ %d' % (device, baud))
        await self.run(reader)


async def open_serial(device: str, baud: int = 9600) -> asyncio.StreamReader:
    """Open a Linux serial tty as an asyncio StreamReader: raw 8N1 at `baud`, stdlib only (termios +
    connect_read_pipe). Hardware path — covered by itest_gps.py, not the host unit tests."""
    import os
    import termios

    descriptor = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    iflag, oflag, cflag, lflag, _ispeed, _ospeed, control = termios.tcgetattr(descriptor)
    speed = getattr(termios, 'B%d' % baud)
    cflag = (cflag | termios.CLOCAL | termios.CREAD | termios.CS8) & ~termios.PARENB & ~termios.CSTOPB
    termios.tcsetattr(descriptor, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, control])  # raw
    reader = asyncio.StreamReader()
    loop = asyncio.get_event_loop()
    await loop.connect_read_pipe(lambda: asyncio.StreamReaderProtocol(reader),
                                 os.fdopen(descriptor, 'rb', buffering=0))
    return reader
