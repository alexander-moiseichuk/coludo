"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host-side GPS assist for the Control hub.

The flight board carries its own GNSS (ATGM336H); a GPS plugged into the Control host (e.g.
/dev/ttyUSB0) is an ASSIST, not the source of truth. Two jobs:
  1. tell the operator when a usable fix is available -- the ideal launch condition is a 3D fix
     with 4+ satellites (so the board's own cold start has a good almanac/position seed);
  2. hand a launch position to the board (operator `assist <board>` -> `update mission` +
     `set-config launch`, persisted in the board's launch.config) when the on-board GPS has no fix yet.

Pure NMEA parsing (GGA position/sats, GSA 2D/3D mode) is split from the serial transport so it is
unit-tested without hardware (test_gps.py); the Linux serial open + read loop is exercised by
itest_gps.py against a real receiver. CPython 3.12, stdlib asyncio only -- no pyserial.
"""

import asyncio
import time

IDEAL_SATELLITES: int = 4  # a 3D fix with this many satellites is the ideal launch condition
STALE_S: float = 5.0  # a fix whose last GGA is older than this is not usable (receivers talk at 1 Hz)
_REOPEN_S: float = 5.0  # a lost receiver is retried this often -- the dongle gets knocked, then re-seated


def _checksum_ok(sentence: str) -> bool:
    """
    Verify the NMEA `*hh` XOR checksum. A sentence WITHOUT one is REJECTED.

    This matches `glider/gnss.py:_checksum_ok`, which returns False when it finds no `*`, and the two
    must agree because this parser is not just a display: `assist <board>` turns the fix it produces
    into `update mission`, i.e. the board's launch position. Tolerating an unverifiable sentence here
    meant the hub could hand the board a pad coordinate the board's own parser would have thrown away
    -- the two `position` sources diverging precisely under the noisy serial that makes checksums
    matter. NMEA 0183 mandates the field; a receiver omitting it is broken or being misread, and
    finding that out is worth more than a fix nobody can verify.
    """
    if '*' not in sentence:
        return False
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
        self.seen: float = 0.0  # time.monotonic() of the last accepted GGA; 0.0 = never

    @property
    def age(self):
        """Seconds since the last accepted GGA, None before the first."""
        return time.monotonic() - self.seen if self.seen else None

    @property
    def fix_3d(self) -> bool:
        return self.mode >= 3

    @property
    def has_position(self) -> bool:
        return self.latitude is not None and self.longitude is not None

    @property
    def usable(self) -> bool:
        """
        The ideal launch condition: a FRESH 3D fix with enough satellites and an actual position.

        Fresh matters because `sync GNSS` / `assist` persist this fix as the board's launch point, which
        beats the board's own fix at arm. Without an age a receiver that went silent -- unplugged, or a
        hung stream that never reaches EOF -- kept its last fix usable for the rest of the day.
        """
        age = self.age
        return (self.fix_3d and self.satellites >= IDEAL_SATELLITES and self.has_position
                and age is not None and age < STALE_S)


class Gps:
    """Host GPS reader: feed NMEA lines, expose the latest fix + a launch position for board assist."""

    def __init__(self, log=print):
        self.fix = Fix()
        self.log = log
        self.lines: int = 0  # accepted sentences (a liveness sign for the operator)

    def feed(self, line: str) -> bool:
        """
        Parse one NMEA sentence into the running fix.

        Robust to the line noise a serial GPS emits -- a bad sentence is rejected, never raised.

        Args:
            line - one raw NMEA sentence.

        Returns:
            True if the sentence updated the fix; False for non-NMEA, a bad checksum, an unhandled
            sentence, or a malformed field.
        """
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
                self.fix.seen = time.monotonic()
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
        age = fix.age
        return {'usable': fix.usable, 'fix_3d': fix.fix_3d, 'satellites': fix.satellites,
                'latitude': fix.latitude, 'longitude': fix.longitude, 'altitude': fix.altitude,
                'age': None if age is None else round(age, 1), 'lines': self.lines}

    def position(self):
        """
        The host position as a mission dict, when the fix is usable.

        So `assist` only pushes a position worth trusting.

        Args:
            (none)

        Returns:
            {latitude, longitude[, altitude]} when the fix is usable; None otherwise.
        """
        if not self.fix.usable:
            return None
        position = {'latitude': self.fix.latitude, 'longitude': self.fix.longitude}
        if self.fix.altitude is not None:
            position['altitude'] = self.fix.altitude
        return position

    async def run(self, reader: asyncio.StreamReader) -> None:
        """
        Feed every line from an NMEA stream until it ends (the read loop, transport-agnostic).

        Args:
            reader - an asyncio StreamReader over NMEA text.

        Returns:
            None; returns when the stream ends (an empty read).
        """
        while True:
            raw = await reader.readline()
            if not raw:
                return
            self.feed(raw.decode('ascii', 'ignore'))

    async def serve(self, device: str, baud: int = 9600) -> None:
        """
        Open the serial GPS and feed it forever (the wired host-assist path).

        A device that cannot be opened at START is REPORTED to the operator and skipped -- host GPS is
        an optional assist, so its failure must not take down the hub (serve() is gathered with
        hub.run(); an unhandled raise cancels both). One that opened and was then LOST drops its fix at
        once and is retried every _REOPEN_S: it was plugged in, so it is expected back.

        Args:
            device - the serial device path (e.g. /dev/ttyUSB0).
            baud - the serial baud rate (default 9600).

        Returns:
            None when the device is unavailable at start; otherwise runs forever.
        """
        try:
            reader = await open_serial(device, baud)
        except OSError as error:
            self.log('host gps unavailable: %s' % error)
            return
        self.log('host gps on %s @ %d' % (device, baud))
        while True:
            try:
                await self.run(reader)
                self.log('host gps lost: stream ended')
            except OSError as error:  # mid-read serial teardown (USB unplug / driver drop): the optional
                self.log('host gps lost: %s' % error)  # assist dies quietly -- never the gathered hub
            self.fix = Fix()  # a dead receiver's last fix is never handed out as the launch point
            reader = await self._reopen(device, baud)

    async def _reopen(self, device: str, baud: int) -> asyncio.StreamReader:
        """Retry the lost device every _REOPEN_S until it opens again -- quietly, it was already reported."""
        while True:
            await asyncio.sleep(_REOPEN_S)
            try:
                reader = await open_serial(device, baud)
            except OSError:
                continue
            self.log('host gps back on %s @ %d' % (device, baud))
            return reader


async def open_serial(device: str, baud: int = 9600) -> asyncio.StreamReader:
    """
    Open a Linux serial tty as an asyncio StreamReader: raw 8N1 at `baud`.

    Stdlib only (termios + connect_read_pipe). Hardware path -- covered by itest_gps.py, not the host
    unit tests.

    Args:
        device - the serial device path.
        baud - the serial baud rate (default 9600).

    Returns:
        An asyncio.StreamReader over the opened tty.

    Raises:
        FileNotFoundError - the device cannot be opened.
    """
    import os
    import termios

    try:
        descriptor = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        raise FileNotFoundError('cannot open %s: %s' % (device, error)) from None
    try:
        iflag, oflag, cflag, lflag, _ispeed, _ospeed, control = termios.tcgetattr(descriptor)
        speed = getattr(termios, 'B%d' % baud)
        cflag = (cflag | termios.CLOCAL | termios.CREAD | termios.CS8) & ~termios.PARENB & ~termios.CSTOPB
        termios.tcsetattr(descriptor, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, control])  # raw
        reader = asyncio.StreamReader()
        loop = asyncio.get_event_loop()
        await loop.connect_read_pipe(lambda: asyncio.StreamReaderProtocol(reader),
                                     os.fdopen(descriptor, 'rb', buffering=0))
    except:
        os.close(descriptor)
        raise
    return reader
