"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Shared GNSS infrastructure (sibling of i2cbus/spibus/servo). NMEA helpers + a Gnss base Task: read
NMEA over a dedicated UART, parse RMC -> 'position' (lat, lon) and GGA -> 'altitude' (m MSL) +
'elevation' (m above the GNSS ground zero, a barometer backup). Module-specific sentence selection +
rate is the subclass's _configure(); ATGM336H (CASIC/PCAS) and NEO-6M (u-blox) differ only there.
Talker-agnostic (GP/GN/BD). Best-effort -- lock drops under boost, so the channels go stale and
consumers fall back.
"""

import asyncio

import config
import databoard
import micropython
import recorder
import task
from machine import UART  # board-only, like `micropython` above (this module never imports off-board)

_KNOTS_TO_MS: float = 0.514444  # NMEA RMC speed is in knots; the airspeed governor wants m/s


@micropython.viper
def _xor_checksum(data: ptr8, start: int, end: int) -> int:  # noqa: F821 -- ptr8 is a viper builtin type
    """
    XOR of the bytes data[start:end] -- the NMEA checksum inner loop as native integer code.

    A viper pointer walk (no per-char str iterator + ord()). `data` is a bytes-like (callers
    .encode()).

    Args:
        data - a bytes-like buffer (ptr8).
        start - the first index (inclusive).
        end - the end index (exclusive).

    Returns:
        The XOR checksum of the byte range, as an int.
    """
    checksum = 0
    for index in range(start, end):
        checksum ^= int(data[index])
    return checksum


def checksum_ok(sentence: str) -> bool:
    """
    Verify the NMEA `*hh` XOR checksum (over the chars between '$' and '*').

    The inner XOR loop is _xor_checksum.

    Args:
        sentence - the full NMEA sentence including the '$' and the '*hh' suffix.

    Returns:
        True when the computed checksum matches the sentence's; False on a missing '*' or a bad /
        absent hex suffix.
    """
    star = sentence.rfind('*')
    if star < 0:
        return False
    got = _xor_checksum(sentence.encode(), 1, star)
    try:
        return got == int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False


def degrees(value: str, hemisphere: str):
    """
    Convert an NMEA ddmm.mmmm value + hemisphere to signed decimal degrees.

    Args:
        value - the ddmm.mmmm field (empty -> None).
        hemisphere - 'N'/'S'/'E'/'W' (S and W give a negative result).

    Returns:
        The signed decimal degrees, or None when the field is empty.
    """
    if not value:
        return None
    dot = value.find('.')
    decimal = int(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -decimal if hemisphere in ('S', 'W') else decimal


def nmea(body: str) -> bytes:
    """
    Wrap a command body in `$...*hh\\r\\n` with its XOR checksum.

    For building PCAS/PMTK/PUBX config sentences.

    Args:
        body - the sentence body between '$' and '*' (no delimiters).

    Returns:
        The full sentence as bytes, ready to write to the UART.
    """
    checksum = _xor_checksum(body.encode(), 0, len(body))
    return ('$%s*%02X\r\n' % (body, checksum)).encode()


class Gnss(task.Task):
    """
    Base GNSS driver over a dedicated UART.

    RMC -> 'position' (lat, lon); GGA -> 'altitude' (m MSL) + 'elevation' (m above the GNSS ground
    zero, a baro backup). Subclasses set the module-specific sentence selection + rate in
    _configure().
    """

    _uart = None  # class default: no transport until setup() opens it (diagnose reads directly)

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return False
        self._uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        self._reader = asyncio.StreamReader(self._uart)
        await self._configure(self.config.get('hz', 1))
        (self._position, self._altitude, self._elevation, self._speed,
         self._course) = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}),
            'position', 'altitude', 'elevation', 'speed', 'course')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('lat', 'lon', 'speed_kn', 'course'),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._fix: bool = False
        self._fix_quality: int = 0  # GGA field 6: 0 none / 1 GPS / 2 DGPS -- signal-quality snapshot
        self._satellites: int = 0   # GGA field 7: satellites used in the fix (more = a better antenna/sky)
        self._hdop: float = 0.0     # GGA field 8: horizontal dilution of precision (LOWER is better)
        self._lines: int = 0  # NMEA lines seen (a liveness counter for probe(), no reader contention)
        self._ground = None  # GNSS ground-zero altitude (first valid GGA), so elevation is offset-free
        self._ok = True
        return True

    async def _configure(self, _unused_hz: int) -> None:
        """Module-specific sentence selection + rate. Default: accept the module's own stream as-is."""
        pass

    def _parse(self, line: str) -> None:
        """Parse one NMEA sentence: RMC -> position (+ telemetry), GGA -> altitude + elevation."""
        if not line.startswith('$') or not checksum_ok(line):
            return
        fields = line.split('*')[0].split(',')
        kind = fields[0][3:]  # drop '$' + the 2-char talker id (GP/GN/BD) -> RMC / GGA / ...
        if kind == 'RMC' and len(fields) > 9:
            self._fix = fields[2] == 'A'  # A = valid fix, V = void
            latitude = degrees(fields[3], fields[4])
            longitude = degrees(fields[5], fields[6])
            if self._fix and latitude is not None and longitude is not None:
                self._position.push((latitude, longitude))
                speed = float(fields[7]) if fields[7] else 0.0  # knots (RMC field 7)
                course = float(fields[8]) if fields[8] else 0.0
                self._speed.push(speed * _KNOTS_TO_MS)  # m/s -> airspeed governor corrector (fix-gated)
                if fields[8]:  # ground-track bearing (deg) -> the attitude backup's absolute yaw ref
                    self._course.push(course)
                self._telemetry.push((latitude, longitude, speed, course))
        elif kind == 'GGA' and len(fields) > 9:
            # signal quality (parsed even with no altitude yet): fix quality, satellites used, HDOP --
            # the numbers that quantify an antenna/sky change (more sats + lower HDOP = a better antenna).
            self._fix_quality = int(fields[6]) if fields[6] else 0
            self._satellites = int(fields[7]) if fields[7] else 0
            self._hdop = float(fields[8]) if fields[8] else 0.0
            if fields[9]:  # altitude (metres MSL) -- present once there is a fix
                altitude = float(fields[9])
                self._altitude.push(altitude)
                if self._ground is None:
                    self._ground = altitude  # first valid GGA fixes the GNSS ground reference
                self._elevation.push(altitude - self._ground)

    async def run(self) -> None:
        """
        Read NMEA lines forever and parse them.

        Non-ASCII noise and malformed fields are skipped (decode raises on a high byte -- MicroPython
        has no errors='ignore'). A silent receiver simply yields nothing.

        Args:
            (none)

        Returns:
            None (runs forever).
        """
        while True:
            raw = await self._reader.readline()
            if raw:
                self._lines += 1
                try:
                    self._parse(raw.decode().strip())
                except (UnicodeError, ValueError, IndexError):
                    pass  # noise byte / malformed field -> drop the line

    async def probe(self) -> str:
        """
        On-demand self-test: NMEA is arriving on the UART.

        The run loop counts lines; this checks the count advances. A satellite fix needs sky view, so
        it is logged (fix true/false), not treated as a failure.

        Args:
            (none)

        Returns:
            None when NMEA is flowing; an error message string when no lines arrived within the
            window.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: nmea link ...')
            before = self._lines
            await asyncio.sleep_ms(1500)  # longer than one NMEA interval
            if self._lines == before:
                raise ValueError('no NMEA on uart:%s in 1.5s' % self.config.get('id'))
            recorder.Recorder.log(self.name, 'probe: nmea link ok (+%d lines, fix=%s, sats=%d, hdop=%.1f)' % (
                self._lines - before, self._fix, self._satellites, self._hdop))
        except Exception as error:
            message = 'nmea link: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: is NMEA arriving on the UART?

        Opens the port and listens briefly. Silence = GNSS unpowered / TX-RX swapped / no module;
        lines = the link is alive (a fix still needs sky view). Shared by atgm336h + neo6mv2. The
        Controller folds this into the reason.

        Args:
            (none)

        Returns:
            A human-readable string: 'no transport ...' when the bus is undefined, 'no NMEA ...' on
            silence, else 'NMEA flowing ...' when the link is alive.
        """
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return 'no transport -- uart bus %s undefined in config' % bus_id
        uart = self._uart  # None until setup opens the port
        if uart is None:
            uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        reader = asyncio.StreamReader(uart)
        seen = 0
        try:
            for _ in range(8):  # ~2 s window (longer than one NMEA interval)
                raw = await asyncio.wait_for_ms(reader.readline(), 250)
                if raw:
                    seen += 1
        except asyncio.TimeoutError:
            pass
        if seen == 0:
            return 'no NMEA on uart:%s -- GNSS unpowered / TX-RX swapped / no module' % bus_id
        return 'NMEA flowing (%d lines) on uart:%s -- link alive (a fix needs sky view)' % (seen, bus_id)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['fix'] = self._fix
        status['satellites'] = self._satellites  # used in the fix -- watch this rise with a better antenna
        status['hdop'] = self._hdop              # horizontal dilution of precision (lower = better geometry)
        status['fix_quality'] = self._fix_quality  # 0 none / 1 GPS / 2 DGPS
        status['position'] = self._position.value()  # (lat, lon) or None until a fix
        status['altitude_m'] = self._altitude.value()
        status['elevation_m'] = self._elevation.value()
        status['speed_ms'] = self._speed.value()  # GNSS ground speed (m/s) or None until a fix
        return status
