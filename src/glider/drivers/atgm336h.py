# drivers/atgm336h.py — ATGM336H GNSS (GPS + BDS) over UART: the position channel. @task.driver(
# 'atgm336h'). At setup it reconfigures the module to RMC-only at the configured rate (default 10 Hz)
# via PMTK commands -- RMC alone (~70 B) fits 10 Hz inside 9600 baud (~960 B/s), so no baud switch is
# needed. run() reads NMEA lines asynchronously (asyncio.StreamReader), parses RMC for the fix and
# writes (latitude, longitude) to the databoard 'position' slot; a GGA sentence, if the module still
# emits one, supplies altitude (a deep fallback to the baro). Lock is lost easily under high-g, so
# position is best-effort and the consumers fall back when it goes stale.
#
# NMEA is talker-agnostic (GP/GN/BD). The UART is dedicated (uart:2), not a shared bus, so the driver
# owns the peripheral. Graceful: an undefined bus -> setup False -> the Controller skips it.

import asyncio

import config
import databoard
import recorder
import task


def _checksum_ok(sentence: str) -> bool:
    """Verify the NMEA `*hh` XOR checksum (over the chars between '$' and '*')."""
    star = sentence.rfind('*')
    if star < 0:
        return False
    got = 0
    for character in sentence[1:star]:
        got ^= ord(character)
    try:
        return got == int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False


def _degrees(value: str, hemisphere: str):
    """NMEA ddmm.mmmm + N/S/E/W -> signed decimal degrees (None when the field is empty)."""
    if not value:
        return None
    dot = value.find('.')
    decimal = int(value[:dot - 2]) + float(value[dot - 2:]) / 60.0
    return -decimal if hemisphere in ('S', 'W') else decimal


def _nmea(body: str) -> bytes:
    """Wrap a command body in `$...*hh\\r\\n` with its XOR checksum (for the PMTK config sentences)."""
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return ('$%s*%02X\r\n' % (body, checksum)).encode()


@task.driver('atgm336h')
class Atgm336h(task.Task):
    """GNSS: reconfigures to RMC-only at `hz`, then writes (latitude, longitude) to 'position' (and
    altitude to 'altitude' if a GGA is seen). Best-effort -- lock can drop under boost."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return False
        from machine import UART

        self._uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        self._reader = asyncio.StreamReader(self._uart)
        await self._configure(self.config.get('hz', 1))
        self._position, self._altitude = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'position', 'altitude')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('lat', 'lon', 'speed_kn', 'course'),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._fix: bool = False
        self._ok = True
        return True

    async def _configure(self, hz: int) -> None:
        """Set RMC-only output at `hz`. RMC carries the fix + position + speed/course; dropping the
        other sentences keeps 10 Hz within 9600 baud. The ATGM336H is a CASIC chip (PCAS commands);
        the PMTK pair is a fallback for MTK-variant modules. Each side ignores the other's sentences,
        so both are sent. If neither is honoured, run() still parses the default stream (slower)."""
        period_ms = 1000 // max(hz, 1)  # 10 Hz -> 100 ms
        writer = asyncio.StreamWriter(self._uart, {})
        commands = (
            'PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0',  # CASIC: output RMC only
            'PCAS02,%d' % period_ms,  # CASIC: measurement period (ms)
            'PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0',  # MTK fallback: RMC only
            'PMTK220,%d' % period_ms,  # MTK fallback: update period (ms)
        )
        for body in commands:
            writer.write(_nmea(body))
            await writer.drain()
            await asyncio.sleep_ms(80)

    def _parse(self, line: str) -> None:
        """Parse one NMEA sentence: RMC -> position (+ telemetry), GGA -> altitude. Talker-agnostic."""
        if not line.startswith('$') or not _checksum_ok(line):
            return
        fields = line.split('*')[0].split(',')
        kind = fields[0][3:]  # drop the '$' + 2-char talker id (GP/GN/BD) -> RMC / GGA / ...
        if kind == 'RMC' and len(fields) > 9:
            self._fix = fields[2] == 'A'  # A = valid fix, V = void
            latitude = _degrees(fields[3], fields[4])
            longitude = _degrees(fields[5], fields[6])
            if self._fix and latitude is not None and longitude is not None:
                self._position.push((latitude, longitude))
                speed = float(fields[7]) if fields[7] else 0.0
                course = float(fields[8]) if fields[8] else 0.0
                self._telemetry.push((latitude, longitude, speed, course))
        elif kind == 'GGA' and len(fields) > 9 and fields[9]:
            self._altitude.push(float(fields[9]))  # metres MSL (a deep fallback to the baro)

    async def run(self) -> None:
        """Read NMEA lines forever and parse them; non-ASCII noise lines and malformed fields are
        skipped (decode raises on a high byte -- MicroPython has no errors='ignore'). A wedged or
        silent receiver simply yields nothing."""
        while True:
            raw = await self._reader.readline()
            if raw:
                try:
                    self._parse(raw.decode().strip())
                except (UnicodeError, ValueError, IndexError):
                    pass  # noise byte / malformed field -> drop the line

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['fix'] = self._fix
        status['position'] = self._position.value()  # (lat, lon) or None until a fix
        status['altitude_m'] = self._altitude.value()
        return status
