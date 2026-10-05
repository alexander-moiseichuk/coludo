"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Shared GNSS infrastructure (sibling of i2cbus/spibus/servo). NMEA helpers + a Gnss base Task: read
NMEA over a dedicated UART, parse RMC -> 'position' (lat, lon) and GGA -> 'altitude' (m MSL) +
'elevation' (m above the GNSS ground zero, a barometer backup). Module-specific sentence selection +
rate is the subclass's _configure(); ATGM336H (CASIC/PCAS) and NEO-6M (u-blox) differ only there.
Talker-agnostic (GP/GN/BD). Best-effort -- lock drops under boost, so the channels go stale and
consumers fall back.

The sky diagnostics -- GSV (satellites in view, C/N0), GSA (fix mode, satellites used) and the ATGM's
antenna text -- come every ~DIAGNOSTICS_S as one burst and go to `<name>_sky.csv`, one row per burst,
plus inspect() for the pad. They are what tells a weak antenna from a dead receiver: the 10-03 flights
never fixed, and with these sentences switched off their logs could not say why. They run on the pad
only -- before BOOSTING and again from DONE -- and are off in flight, where a burst would stall the
position (_sky_window()).
"""

import asyncio
import time

import commons
import config
import controller
import databoard
import micropython
import recorder
import task
from machine import UART  # board-only, like `micropython` above

# WHY THIS MODULE IS NOT HOST-IMPORTABLE, and what it would cost to change.
# The audit asked for guarded imports so drivers can be unit-tested on CPython. A guard alone is
# not enough here: _xor_checksum is @micropython.viper and annotates `ptr8`, a viper BUILTIN TYPE that
# exists only under the real compiler, so the annotation fails at def time on CPython however the
# imports are written. Making this importable means restructuring that hot path (quote the
# annotations, or split the viper body into a board-only module).
# Deliberately not done: it is a testability improvement, not a defect, and it would rewrite the NMEA
# checksum -- the one function in here that a GNSS dropout depends on -- for no in-flight benefit.
# The rest of the tree already guards these imports; this module is the single exception, and the
# reason is recorded here so the next reader does not "fix" it with a try/except that cannot work.

_SWEEP_MS: int = commons.const(1000)  # the outage sweep's own clock, for a receiver that sends nothing
_SENTENCE_GAP_US: int = commons.const(3000000)  # a sentence quiet this long (3 s) is an OUTAGE, not jitter. The
# module is configured at 1 Hz, so three missed intervals -- loose enough that a busy loop or a single
# dropped line never cries wolf, tight enough to catch the loss well inside a <60 s flight.

_KNOTS_TO_MS: float = 0.514444  # NMEA RMC speed is in knots; the airspeed governor wants m/s

DIAGNOSTICS_S: int = commons.const(10)  # the sky diagnostics come this often (s) at any rate: the drivers' dividers
"""
A burst's sentences come back to back, at most one RMC/GGA between two of them (~75 ms at 9600 baud; on
the bench a whole ATGM burst took 0.43-0.54 s, no two of its sentences more than ~0.17 s apart), and
bursts are ~DIAGNOSTICS_S apart. So half a second with none of them ends a burst, with margin both ways.
"""
_BURST_QUIET_US: int = commons.const(500000)
"""
The ATGM's antenna supervisor ('$GPTXT,01,01,01,ANTENNA OPEN') as the sky row's code. OPEN means the feed
draws no current: a PASSIVE antenna reads OPEN too, and on the 2026-10-04 bench the passive 12x12 patch the
airframes flew cost the ATGM ~8-10 dB against the active ones (OK). The NEO-6M reports no antenna status
over NMEA, so its rows carry 0.
"""
_ANTENNA_CODES: dict = {'ANTENNA OK': 1, 'ANTENNA OPEN': 2, 'ANTENNA SHORT': 3}
_ANTENNA_NAMES: tuple = ('unknown', 'OK', 'OPEN', 'SHORT')  # the code as a word, for inspect() and the events


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
        return False  # no `*hh` -> unverifiable; control/gps.py:_checksum_ok holds the SAME policy
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
    zero, a baro backup); GSV/GSA/antenna text -> the sky row. Subclasses set the module-specific
    sentence selection + rate in _configure().
    """

    _uart = None  # class default: no transport until setup() opens it (diagnose reads directly)

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 2)
        spec = config.bus(self.controller.config, self.config.get('bus', 'uart'), bus_id)
        if spec is None:
            return False
        self._uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        self._reader = asyncio.StreamReader(self._uart)
        self._writer = asyncio.StreamWriter(self._uart, {})  # the init, then the sky switch (_sky_window())
        frames = await self._configure(self.config.get('hz', 1))
        self._sky_off: bytes = frames[0]  # the flight init's own selection: no diagnostics (BOOSTING to DONE)
        self._sky_on: bytes = frames[1]  # the diagnostics' (the pad: before BOOSTING, and from DONE)
        self._diagnosing: bool = True  # the init ends with the diagnostics on: the pad
        (self._position, self._altitude, self._elevation, self._speed,
         self._course) = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}),
            'position', 'altitude', 'elevation', 'speed', 'course')
        telemetry_us = self.config.get('telemetry_ms', 0) * 1000
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('lat', 'lon', 'speed_kn', 'course'),
                                       decimate_us=telemetry_us)
        """
        GGA gets its OWN stream rather than extra columns on the RMC one. The two sentences carry
        different things (RMC the fix, GGA the signal quality behind it) and arrive independently, so
        merging them means padding every row with the other's stale values and losing which sentence
        actually updated. A separate stream keeps each one's real cadence -- and makes an outage in one
        of them visible as a gap in that stream alone.

        These fields were parsed but never recorded: fix quality, satellites and HDOP are exactly the
        numbers that say whether a fix loss was the sky, the antenna or the receiver, and after a flight
        with no GNSS they are the evidence for which.
        """
        self._gga_telemetry = recorder.Telemetry(
            '%s_gga.csv' % self.name, ('altitude_m', 'elevation_m', 'quality', 'satellites', 'hdop_cd'),
            decimate_us=telemetry_us)
        """
        The sky gets a third stream, one row per diagnostics burst: satellites in view (GSV, all systems),
        used (GSA, all systems), the fix mode (GSA: 1 none, 2 2D, 3 3D), the four strongest C/N0 (dB-Hz, 0
        where fewer were heard) and the antenna code (0 none reported, 1 OK, 2 OPEN, 3 SHORT). All ints,
        like hdop_cd: no boxed float in a row. _settle() pushes it, once per burst -- see there.
        """
        self._sky_telemetry = recorder.Telemetry(
            '%s_sky.csv' % self.name, ('in_view', 'used', 'mode', 'cn0_1', 'cn0_2', 'cn0_3', 'cn0_4', 'antenna'),
            decimate_us=telemetry_us)
        self._views: dict = {}  # GSV talker -> (in view, its four strongest C/N0) from its last WHOLE group
        self._systems: dict = {}  # GSA system -> (fix mode, satellites used) from its last sentence
        self._group_talker: str = None  # the GSV group being received: its talker ...
        self._group_next: int = 0  # ... the part it needs next (0: no group in progress) ...
        self._group_cn0: list = []  # ... and the C/N0 values collected so far
        self._antenna: int = 0  # the last antenna text, as a code (_ANTENNA_CODES)
        self._pending: bool = False  # a diagnostics sentence arrived that no sky row carries yet ...
        self._pending_us: int = 0  # ... the latest one (ticks_us)
        self._sky_seen: bool = False  # a sky row went out: only the first burst is evented
        self._seen_us: dict = {'RMC': 0, 'GGA': 0}  # last arrival per sentence -> the outage events
        self._absent: dict = {'RMC': False, 'GGA': False}  # currently-out flag, so each edge fires once
        self._fix: bool = False
        self._fix_quality: int = 0  # GGA field 6: 0 none / 1 GPS / 2 DGPS -- signal-quality snapshot
        self._satellites: int = 0   # GGA field 7: satellites used in the fix (more = a better antenna/sky)
        self._hdop: float = 0.0     # GGA field 8: horizontal dilution of precision (LOWER is better)
        self._lines: int = 0  # NMEA lines seen (a liveness counter for probe(), no reader contention)
        self._ground = None  # GNSS ground-zero altitude (first valid GGA), so elevation is offset-free
        self._ok = True
        return True

    async def _configure(self, _unused_hz: int) -> tuple:
        """
        Module-specific sentence selection + rate, sent once at setup, and the sky switch's frames.

        Default: accept the module's own stream as-is -- nothing is sent, so there is nothing to switch.

        Args:
            _unused_hz - the fix rate (Hz).

        Returns:
            (off, on): the bytes _sky_window() writes to switch the diagnostics; b'' and b'' here.
        """
        return b'', b''

    def _parse(self, line: str) -> None:
        """Parse one NMEA sentence: RMC -> position (+ telemetry), GGA -> altitude + elevation, else the sky."""
        if not line.startswith('$') or not checksum_ok(line):
            return
        fields = line.split('*')[0].split(',')
        kind = fields[0][3:]  # drop '$' + the 2-char talker id (GP/GN/BD) -> RMC / GGA / ...
        if kind in self._seen_us:
            self._mark(kind, time.ticks_us())
        if kind == 'RMC' and len(fields) > 9:
            self._fix = fields[2] == 'A'  # A = valid fix, V = void
            latitude = degrees(fields[3], fields[4])
            longitude = degrees(fields[5], fields[6])
            if self._fix and latitude is not None and longitude is not None:
                self._position.push((latitude, longitude))
                speed = float(fields[7]) if fields[7] else 0.0  # knots (RMC field 7)
                course = float(fields[8]) if fields[8] else 0.0
                self._speed.push(speed * _KNOTS_TO_MS)  # m/s -> airspeed governor corrector (fix-gated)
                if fields[8]:  # ground-track bearing (deg) -> the attitude filter's absolute yaw ref
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
                elevation = altitude - self._ground
                self._elevation.push(elevation)
            else:
                altitude = elevation = 0.0  # quality-only GGA (no fix yet) -- still worth recording
            # hdop as centi-units (int), matching the fixnum convention: no boxed float in the row
            self._gga_telemetry.push((altitude, elevation, self._fix_quality, self._satellites,
                                      int(self._hdop * 100)))
        elif kind in ('GSV', 'GSA', 'TXT'):  # after RMC/GGA: the 10 Hz path tests nothing more than before
            self._sky(kind, fields)

    def _sky(self, kind: str, fields: list) -> None:
        """
        Fold one diagnostics sentence into the sky view and mark the burst it belongs to.

        GSA comes once per system: the ATGM sends one for GPS and one for BDS (the NMEA 4.1 system id
        closes each), the NEO-6M a single one. The PRN numbers repeat across systems -- GPS 22 and BDS 22
        were both in use on the bench -- so `used` is the SUM of the systems' counts, never a set of PRNs,
        and the mode is the best of them. Keyed per system, a second burst replaces rather than adds.

        Every GSV/GSA, and an antenna text, restarts the burst's quiet clock (_settle()). Any other text
        -- the NEO's boot banner, the ATGM's firmware id -- is not the sky and is not a burst.

        Args:
            kind - 'GSV', 'GSA' or 'TXT'.
            fields - the sentence split on ',' (checksum dropped).

        Returns:
            None; an antenna change pushes an event.
        """
        if kind == 'GSV' and len(fields) > 3:
            self._gsv(fields)
        elif kind == 'GSA' and len(fields) > 14:
            used = 0
            for index in range(3, 15):  # the twelve PRN slots
                if fields[index]:
                    used += 1
            system = fields[18] if len(fields) > 18 else fields[0][1:3]  # NMEA 4.1 id (1 GPS, 4 BDS), else talker
            self._systems[system] = (int(fields[2]) if fields[2] else 0, used)
        elif kind == 'TXT' and len(fields) > 4 and fields[4] in _ANTENNA_CODES:
            code = _ANTENNA_CODES[fields[4]]
            if code != self._antenna:  # on a change only: the ATGM repeats its status every burst
                self.event('antenna %s (was %s)' % (_ANTENNA_NAMES[code], _ANTENNA_NAMES[self._antenna]))
                self._antenna = code
        else:
            return
        self._pending = True
        self._pending_us = time.ticks_us()

    def _gsv(self, fields: list) -> None:
        """
        One GSV sentence: collect its C/N0; the group's last part replaces that talker's view.

        A talker (GP GPS, BD BeiDou, GL GLONASS, GN combined) reports its satellites in view as a group
        of sentences `<parts>,<part>,<in view>`, then four satellites each as PRN, elevation, azimuth,
        C/N0 -- and NMEA 4.1 (the ATGM) appends a signal id after the last. The C/N0 sits at fields 7,
        11, 15, 19 whatever the count, and the signal id at a multiple of 4, so it is never read as one.
        A satellite in view but not tracked has an EMPTY C/N0: in view, no signal to rank (not a 0).

        A group replaces the talker's view only when it arrived WHOLE and in order: a part lost to a bad
        checksum would otherwise blank that system's half of the sky until the next burst.

        Args:
            fields - the sentence split on ',' (checksum dropped).

        Returns:
            None; on a group's last part, self._views[talker] = (in view, its four strongest C/N0).
        """
        talker = fields[0][1:3]
        part = int(fields[2])
        if part == 1:
            self._group_talker = talker
            self._group_cn0 = []
        elif part != self._group_next or talker != self._group_talker:
            self._group_next = 0  # a part out of order, or of another talker: this group cannot be whole
            return
        self._group_next = part + 1
        for index in range(7, len(fields), 4):
            if fields[index]:
                self._group_cn0.append(int(fields[index]))
        if part == int(fields[1]):
            self._views[talker] = (int(fields[3]) if fields[3] else 0, sorted(self._group_cn0, reverse=True)[:4])
            self._group_next = 0

    def _sky_row(self) -> tuple:
        """
        The sky as the `<name>_sky.csv` row, summed over the systems.

        Each GSV talker's last whole group and each GSA system's last sentence count; a system that was
        never heard counts nothing.

        Args:
            (none)

        Returns:
            (in_view, used, mode, cn0_1, cn0_2, cn0_3, cn0_4, antenna), all ints: the four strongest C/N0
            of all systems, 0 where fewer were heard; all zeros before the first diagnostics.
        """
        in_view = 0
        strongest = []
        for satellites, values in self._views.values():
            in_view += satellites
            strongest += values
        strongest.sort(reverse=True)
        strongest += [0, 0, 0, 0]
        used = mode = 0
        for system_mode, system_used in self._systems.values():
            used += system_used
            mode = max(mode, system_mode)
        return (in_view, used, mode, strongest[0], strongest[1], strongest[2], strongest[3], self._antenna)

    def _settle(self, now_us: int) -> None:
        """
        Push the sky row once a diagnostics burst is over: exactly one row per burst.

        A row goes out on the first _sweeping() tick (1 s) that finds diagnostics no row carries yet AND
        none of them in the last _BURST_QUIET_US. A burst's sentences are under that apart and bursts are
        ~DIAGNOSTICS_S apart, so every tick inside a burst waits and the first one after it pushes the
        whole burst -- the ATGM's antenna text, which comes AFTER the epoch's RMC, included. Ending the
        burst on a sentence instead could not work for both modules (the ATGM's last is the TXT, the
        NEO's a GSV), and closing it on the next RMC would add work to the 10 Hz path and still cut the
        ATGM's TXT off. Nothing arrives in HITL (the sim publishes no GSV), so the stream stays empty there.

        Two limits, neither seen on the bench. A reader stalled _BURST_QUIET_US or more inside a burst can
        split it into a partial row and a whole one -- when a tick falls due during the stall, which is
        certain only for a stall of a whole _SWEEP_MS (the widest gap measured inside a burst was ~0.17 s).
        And an ATGM whose init never took stays at its default 1 Hz output, diagnostics every second with
        only ~0.3 s between them by line time (not observed): no row then ever goes out, inspect() still
        shows the sky.

        Args:
            now_us - the current time (ticks_us).

        Returns:
            None; pushes one sky row -- and for the first burst one event -- when a burst has just ended.

        Raises:
            ValueError - the Recorder's (_RecorderError) when the row does not fit; the burst is spent.
        """
        if not self._pending or commons.ticks_diff(now_us, self._pending_us) < _BURST_QUIET_US:
            return
        self._pending = False
        row = self._sky_row()
        if not self._sky_seen:
            self._sky_seen = True
            self.event('first sky view: %d in view, %d used, mode %d, C/N0 %d/%d/%d/%d dB-Hz, antenna %s' % (
                row[:7] + (_ANTENNA_NAMES[row[7]],)))
        self._sky_telemetry.push(row)

    def _sky_window(self, stage: int) -> bool:
        """
        Keep the sky diagnostics to the pad: switch them when the stage crosses the flight window.

        On before BOOSTING and again from DONE (the post-landing search), off from BOOSTING to DONE -- the
        window drivers/wifi.py stops its radio work in. In flight a burst costs the position: on the
        2026-10-04 bench each one delayed the epoch's RMC by up to ~0.6 s, so the 200 ms 'position'
        channel went stale for ~0.1-0.4 s once per burst and guidance dead-reckoned through it. Off is the
        flight init's own selection, so the flight output is exactly what it was before the diagnostics
        existed; a receiver that missed it keeps bursting, which costs that staleness and nothing else.

        Only a change writes -- one write per edge, none while the window holds -- and a switch formats
        nothing: the frames are bytes the driver made at setup (the GC is off from BOOSTING), and the
        write hands them to the UART's buffer, idle since setup. Its event row and the caller's drain are
        all it allocates. A reset into a flight stage (a warm start) re-ran the init with the diagnostics
        on, so the first tick switches them off.

        Args:
            stage - the controller's flight stage.

        Returns:
            True when it wrote (the caller drains); False while the window holds.
        """
        diagnosing = not (controller.Stage.BOOSTING <= stage < controller.Stage.DONE)
        if diagnosing == self._diagnosing:
            return False
        if diagnosing:
            self._writer.write(self._sky_on)
            self.event('sky diagnostics on')
        else:
            self._writer.write(self._sky_off)
            self.event('sky diagnostics off for flight')
        self._diagnosing = diagnosing
        return True

    def _mark(self, kind: str, now_us: int) -> None:
        """
        Note that `kind` just arrived, and EVENT the transitions in and out of an outage.

        RMC and GGA are tracked separately because they fail separately: a receiver holding almanac but
        no fix keeps emitting GGA (quality 0) while RMC goes void, and a sentence-selection mistake can
        silence one alone. Recording only "GNSS is quiet" would blur those into one symptom.

        These go through event() rather than log(), because a lost fix is precisely the kind of fact a
        flight must not be able to lose: logs flush roughly every 1000 telemetry messages and a short
        flight ends without them, while telemetry commits per line. Guidance now dead-reckons through a
        GNSS outage (guidance._reckon), which is by design almost invisible in the trajectory -- so
        without these events the capture would show a normal-looking flight and no record of the
        receiver having been dead for most of it.

        Args:
            kind - the sentence type that arrived ('RMC' or 'GGA').
            now_us - the arrival time (ticks_us).

        Returns:
            None; pushes an event on each edge, nothing on a steady stream.
        """
        if self._absent[kind]:
            self._absent[kind] = False
            self.event('%s back after %d ms' % (
                kind, commons.ticks_diff(now_us, self._seen_us[kind]) // 1000))
        self._seen_us[kind] = now_us

    def _sweep(self, now_us: int) -> None:
        """
        Fire the outage event for any sentence that has gone quiet past _SENTENCE_GAP_US.

        Detecting absence needs a clock that runs when nothing arrives, so it cannot live in _parse --
        a receiver that goes completely silent parses nothing and would never notice. The run loop
        calls this on every line AND on its read timeout.

        Args:
            now_us - the current time (ticks_us).

        Returns:
            None; pushes one event per sentence per outage.
        """
        for kind in self._seen_us:
            if not self._absent[kind] and self._seen_us[kind] and \
                    commons.ticks_diff(now_us, self._seen_us[kind]) > _SENTENCE_GAP_US:
                self._absent[kind] = True
                self.event('%s lost (quiet %d ms)' % (
                    kind, commons.ticks_diff(now_us, self._seen_us[kind]) // 1000))

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
        asyncio.create_task(self._sweeping())  # a silent receiver parses nothing -- see _sweeping()
        while True:
            raw = await self._reader.readline()
            if raw:
                self._lines += 1
                try:
                    self._parse(raw.decode().strip())
                except (UnicodeError, ValueError, IndexError):
                    pass  # noise byte / malformed field -> drop the line
            self._sweep(time.ticks_us())  # also on an empty read: a silent receiver must still be seen

    async def _sweeping(self) -> None:
        """
        Sweep for outages on a CLOCK, not only per line.

        readline() on the UART stream has no timeout, so a receiver that goes completely silent (power
        lost, connector out) never returns a line -- and the per-line sweep in run() never ran, so the
        outage events that exist for exactly this never fired. One sleep per second (48 B) instead of a
        wait_for_ms per line (560 B, at NMEA's line rate).

        The same clock ends the diagnostics bursts (_settle()), which keeps that off the per-line path, and
        keeps the diagnostics to the pad (_sky_window(): an int compare a tick, a write per flight edge).
        """
        while True:
            await asyncio.sleep_ms(_SWEEP_MS)
            now_us = time.ticks_us()
            self._sweep(now_us)
            if self._sky_window(self.controller.stage):
                await self._writer.drain()
            try:
                self._settle(now_us)
            except ValueError:
                pass  # a full telemetry ring drops the row, as run() drops a fix; the outage sweep lives on

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
        """
        OWN what we open. diagnose() is called precisely when setup FAILED, which is when self._uart
        is None -- so this path opened a fresh UART peripheral every call and never released it. A few
        rounds of `probe` on a dead GNSS would leak one peripheral instance each, and the later ones
        can collide with the earlier on the same pins. Only the UART opened HERE is deinited; a live
        one belongs to the run loop and must survive.
        """
        borrowed = uart is not None
        if uart is None:
            uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        seen = 0
        try:
            reader = asyncio.StreamReader(uart)
            for _ in range(8):  # ~2 s window (longer than one NMEA interval)
                raw = await asyncio.wait_for_ms(reader.readline(), 250)
                if raw:
                    seen += 1
        except asyncio.TimeoutError:
            pass
        finally:
            if not borrowed:
                try:
                    uart.deinit()
                except Exception:
                    pass  # a peripheral that will not close must not sink the diagnosis it carried
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
        """
        The sky, for "is it good enough to launch" on the pad: the top-4 C/N0 mean is the yardstick. On
        the 2026-10-04 bench (both modules, the flight init, open sky): >= ~40 dB-Hz gave a usable
        (RMC-valid) fix by ~42 s on the ATGM, in 1-38 s on the NEO; ~35 is marginal (the passive 12x12
        patch the airframes flew -- it fixed, but with nothing around it); <= ~30 never fixed cold (26 with
        no antenna, 17 min). All zeros until the first burst, ~DIAGNOSTICS_S after setup; in flight the
        last pad burst (the diagnostics are off from BOOSTING to DONE).
        """
        row = self._sky_row()
        status['in_view'] = row[0]  # satellites in view, all systems (GSV)
        status['used'] = row[1]  # satellites used, all systems (GSA)
        status['mode'] = row[2]  # GSA fix mode: 0 not reported yet / 1 none / 2 2D / 3 3D
        status['cn0'] = row[3:7]  # the four strongest C/N0 (dB-Hz), 0 where fewer were heard
        status['cn0_mean'] = sum(row[3:7]) / 4  # their mean: the yardstick above
        status['antenna'] = _ANTENNA_NAMES[row[7]]  # the ATGM's antenna text; 'unknown' on the NEO-6M
        return status
