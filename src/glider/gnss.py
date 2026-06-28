# gnss.py — shared GNSS infrastructure (sibling of i2cbus/spibus/servo). NMEA helpers + a Gnss base
# Task: read NMEA over a dedicated UART, parse RMC -> 'position' (lat, lon) and GGA -> 'altitude'
# (m MSL) + 'elevation' (m above the GNSS ground zero, a barometer backup). Module-specific sentence
# selection + rate is the subclass's _configure(); ATGM336H (CASIC/PCAS) and NEO-6M (u-blox) differ
# only there. Talker-agnostic (GP/GN/BD). Best-effort -- lock drops under boost, so the channels go
# stale and consumers fall back.

import asyncio

import config
import databoard
import micropython
import recorder
import task

_KNOTS_TO_MS: float = 0.514444  # NMEA RMC speed is in knots; the airspeed governor wants m/s


@micropython.viper
def _xor_checksum(data: ptr8, start: int, end: int) -> int:  # noqa: F821 -- ptr8 is a viper builtin type
    """XOR of the bytes data[start:end] -- the NMEA checksum inner loop as native integer code (/
    a viper pointer walk, no per-char str iterator + ord()). `data` is a bytes-like (callers .encode())."""
    checksum = 0
    for index in range(start, end):
        checksum ^= int(data[index])
    return checksum


def checksum_ok(sentence: str) -> bool:
    """Verify the NMEA `*hh` XOR checksum (over the chars between '$' and '*'); inner loop = _xor_checksum."""
    star = sentence.rfind('*')
    if star < 0:
        return False
    got = _xor_checksum(sentence.encode(), 1, star)
    try:
        return got == int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False


def degrees(value: str, hemisphere: str):
    """NMEA ddmm.mmmm + N/S/E/W -> signed decimal degrees (None when the field is empty)."""
    if not value:
        return None
    dot = value.find('.')
    decimal = int(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -decimal if hemisphere in ('S', 'W') else decimal


def nmea(body: str) -> bytes:
    """Wrap a command body in `$...*hh\\r\\n` with its XOR checksum (PCAS/PMTK/PUBX config sentences)."""
    checksum = _xor_checksum(body.encode(), 0, len(body))
    return ('$%s*%02X\r\n' % (body, checksum)).encode()


class Gnss(task.Task):
    """Base GNSS driver over a dedicated UART: RMC -> 'position' (lat, lon); GGA -> 'altitude' (m MSL)
    + 'elevation' (m above the GNSS ground zero, a baro backup). Subclasses set the module-specific
    sentence selection + rate in _configure()."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return False
        from machine import UART

        self._uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        self._reader = asyncio.StreamReader(self._uart)
        await self._configure(self.config.get('hz', 1))
        self._position, self._altitude, self._elevation, self._speed = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'position', 'altitude', 'elevation', 'speed')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('lat', 'lon', 'speed_kn', 'course'),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._fix: bool = False
        self._lines: int = 0  # NMEA lines seen (a liveness counter for probe(), no reader contention)
        self._ground = None  # GNSS ground-zero altitude (first valid GGA), so elevation is offset-free
        self._ok = True
        return True

    async def _configure(self, hz: int) -> None:
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
                self._telemetry.push((latitude, longitude, speed, course))
        elif kind == 'GGA' and len(fields) > 9 and fields[9]:
            altitude = float(fields[9])  # metres MSL
            self._altitude.push(altitude)
            if self._ground is None:
                self._ground = altitude  # first valid GGA fixes the GNSS ground reference
            self._elevation.push(altitude - self._ground)

    async def run(self) -> None:
        """Read NMEA lines forever and parse them; non-ASCII noise and malformed fields are skipped
        (decode raises on a high byte -- MicroPython has no errors='ignore'). A silent receiver simply
        yields nothing."""
        while True:
            raw = await self._reader.readline()
            if raw:
                self._lines += 1
                try:
                    self._parse(raw.decode().strip())
                except (UnicodeError, ValueError, IndexError):
                    pass  # noise byte / malformed field -> drop the line

    async def probe(self) -> str:
        """On-demand self-test: NMEA is arriving on the UART (the run loop counts lines). A satellite
        fix needs sky view, so it is logged (fix true/false), not treated as a failure."""
        try:
            recorder.Recorder.log(self.name, 'probe: nmea link ...')
            before = self._lines
            await asyncio.sleep_ms(1500)  # longer than one NMEA interval
            if self._lines == before:
                raise ValueError('no NMEA on uart:%s in 1.5s' % self.config.get('id'))
            recorder.Recorder.log(self.name, 'probe: nmea link ok (+%d lines, fix=%s)' % (
                self._lines - before, self._fix))
        except Exception as error:
            message = 'nmea link: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """Deeper analysis when setup() failed: is NMEA arriving on the UART? Open the port and listen
        briefly. Silence = GNSS unpowered / TX-RX swapped / no module; lines = the link is alive (a fix
        still needs sky view). Shared by atgm336h + neo6mv2. The Controller folds this into the reason."""
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return 'no transport -- uart bus %s undefined in config' % bus_id
        uart = getattr(self, '_uart', None)
        if uart is None:
            from machine import UART

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
        status['position'] = self._position.value()  # (lat, lon) or None until a fix
        status['altitude_m'] = self._altitude.value()
        status['elevation_m'] = self._elevation.value()
        status['speed_ms'] = self._speed.value()  # GNSS ground speed (m/s) or None until a fix
        return status
