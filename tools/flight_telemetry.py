"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Parse a Coludo recorder capture (the UART stream to the Luckfox) into aligned telemetry streams + log
lines, for offline analysis. The recorder interleaves two record kinds on uart:1 (recorder.py):
    @<session>_<file>@<row>               telemetry; first row per file is `uptime;<field>;...`, then
                                          each data row is `<uptime_us>;<v>;<v>;...`  (';'-separated)
    <ticks_us> <descriptor> :: <message>  best-effort log line
parse() reads a raw capture (both kinds interleaved) and returns the streams + logs. Stdlib only, so it
stays importable in the test suite; the plotly rendering lives in flight_report.py.
"""

import re

# the session prefix on each telemetry @tag: YYYYMMDD_HHMMSS, optionally with a _<rand> disambiguator
# (recorder.session()); both shapes strip down to the bare file name.
_SESSION = re.compile(r'^\d{8}_\d{6}(?:_\d+)?_')
_SERVO = re.compile(r'^servo_(.+)\.csv$')  # a board's per-servo stream -> the surface name it drives


class Stream:
    """One telemetry file: its field names and numeric rows (uptime first)."""

    def __init__(self, name: str):
        self.name: str = name  # the file, e.g. 'adxl375.csv' (session prefix stripped)
        self.fields: list = []  # column names after the leading 'uptime'
        self.rows: list = []  # [uptime_us, v1, v2, ...] per row (floats; '' for a missing/blank cell)

    def column(self, field: str):
        """
        The (time_seconds, value) series for one field name; blank cells skipped.

        Args:
            field - the column name to extract.

        Returns:
            (times, values) parallel lists; ([], []) when the field is absent.
        """
        if field not in self.fields:
            return [], []
        index = self.fields.index(field) + 1  # +1 past the uptime column
        times, values = [], []
        for row in self.rows:
            if len(row) > index and row[index] != '':
                times.append(row[0] / 1e6)
                values.append(row[index])
        return times, values


def find_stream(streams, *fields, prefer=None):
    """
    The stream carrying all the given fields -- match by ROLE (what it measures), never by file name.

    A capture's file names track the fitted hardware: the primary baro may be icp10111 or bmp280, the
    accel adxl375 or lsm6dso32, and a fallback flight is exactly when the OTHER one is the survivor. A
    renderer keyed on file names silently loses those panels (findings §27.5), so every tool resolves
    streams here instead.

    Args:
        streams - the parsed streams, keyed by name.
        fields - the field names the stream must carry (all of them).
        prefer - a name substring to break ties toward a preferred stream (e.g. the dedicated high-g
            accel over the IMU's low-g one).

    Returns:
        The matching stream, or None when none carry every field.
    """
    matches = [stream for stream in streams.values() if all(field in stream.fields for field in fields)]
    if prefer:
        for stream in matches:
            if prefer in stream.name:
                return stream
    return matches[0] if matches else None


def _number(token: str):
    """A telemetry cell -> float when it parses; '' stays '' (blank), other non-numeric -> nan."""
    try:
        return float(token)
    except ValueError:
        return token if token == '' else float('nan')  # nan keeps downstream arithmetic from TypeError


def _synthesise_fins(streams: dict) -> None:
    """
    Build a virtual 'fins.csv' from the per-servo streams when a capture has none.

    The SIM (tasks/hitl.py) records ONE fused 'fins.csv' with a column per surface, but a real board
    records one stream PER SERVO ('servo_<surface>.csv', column 'angle' -- drivers/sg90.py). Every
    fin-aware tool looks for the fused shape, so on a board capture they would all silently find nothing
    (findings §27.1). Rebuilding it here -- in the one parser every tool shares -- makes board and sim
    captures render identically, and works retroactively on captures already recorded.

    Servo rows are EVENT-based (sg90 compare-and-sets: a held fin writes nothing), so each surface is
    FORWARD-FILLED across the merged timeline. That is the physical truth, not an approximation: a servo
    holds its last commanded angle until it is told otherwise. Surfaces are discovered from the stream
    names rather than hardcoded, so a renamed or extra fin still appears.

    Args:
        streams - the parsed {file -> Stream} map, mutated in place.

    Returns:
        None; adds a synthetic 'fins.csv' when per-servo streams exist and no fused one does.
    """
    if 'fins.csv' in streams:
        return  # a sim capture (or a board that already fuses them) -- nothing to rebuild
    series = {}
    for name, stream in streams.items():
        match = _SERVO.match(name)
        if not match or 'angle' not in stream.fields:
            continue
        index = stream.fields.index('angle') + 1  # +1 past the uptime column
        points = [(row[0], row[index]) for row in stream.rows if len(row) > index and row[index] != '']
        if points:
            series[match.group(1)] = sorted(points)
    if not series:
        return
    surfaces = sorted(series)  # stable column order across captures
    fused = Stream('fins.csv')
    fused.fields = surfaces
    cursors = dict.fromkeys(surfaces, 0)
    latest = dict.fromkeys(surfaces, '')  # '' = not commanded yet; column() skips blank cells
    for moment in sorted({stamp for points in series.values() for stamp, _ in points}):
        for surface in surfaces:
            points, index = series[surface], cursors[surface]
            while index < len(points) and points[index][0] <= moment:
                latest[surface] = points[index][1]
                index += 1
            cursors[surface] = index
        fused.rows.append([moment] + [latest[surface] for surface in surfaces])
    streams['fins.csv'] = fused


_TICKS_PERIOD = 1 << 30  # MicroPython ticks_us wraps here (~1073.7 s ~ 17.9 min of board uptime)


def _unwrap(streams: dict, logs: list) -> None:
    """
    Undo the ticks_us WRAPAROUND so a long-uptime capture is not silently mangled.

    MicroPython's `time.ticks_us()` wraps at 2**30 us -- about **17.9 minutes** of board uptime -- and
    the recorder stamps rows with it raw. A board powered up, set up, waiting on a GNSS fix and then
    flown will cross that boundary mid-capture, after which stamps jump BACKWARDS by 2**30 and every
    duration computed downstream goes negative. Seen twice in one bench session (a flight reported as
    -1015.6 s long, at 7.3e12 deg/s of fin travel) -- and it is a plausible field-day sequence, not a
    bench artifact.

    Fixed here, in the one parser every tool shares, so it also repairs captures ALREADY recorded
    rather than only those taken after a firmware change. Per stream: walk in file order and add one
    period each time a stamp drops by more than half a period (a real gap is never that large; a wrap
    always is).

    Args:
        streams - {name: Stream}, mutated in place.
        logs - the (uptime_us | None, line) list, mutated in place.

    Returns:
        None.
    """
    half = _TICKS_PERIOD // 2
    for stream in streams.values():
        offset, previous = 0, None
        for row in stream.rows:
            if previous is not None and row[0] + offset < previous - half:
                offset += _TICKS_PERIOD
            row[0] += offset
            previous = row[0]
    offset, previous = 0, None
    for index, (stamp, line) in enumerate(logs):
        if stamp is None:
            continue
        if previous is not None and stamp + offset < previous - half:
            offset += _TICKS_PERIOD
        logs[index] = (stamp + offset, line)
        previous = stamp + offset


def parse(text: str):
    """
    Parse a raw capture into aligned streams and log lines.

    Args:
        text - the raw recorder capture (both record kinds interleaved).

    Returns:
        ({file -> Stream}, logs), where logs is a list of (uptime_us | None, line). Every timestamp is
        normalised to a flight-relative origin (the earliest stamp seen is subtracted), so a capture
        starts at t=0 rather than at the board's raw boot uptime.
    """
    streams = {}
    logs = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith('@'):
            tag, _, row = line[1:].partition('@')
            if not row:
                continue
            name = _SESSION.sub('', tag)  # 'YYYYMMDD_HHMMSS_imu.csv' -> 'imu.csv'
            stream = streams.get(name)
            if stream is None:
                stream = streams[name] = Stream(name)
            cells = row.split(';')
            if not stream.fields and cells[0] == 'uptime':
                stream.fields = cells[1:]  # the header row
            else:
                values = [_number(cell) for cell in cells]
                try:
                    values[0] = int(float(values[0]))  # uptime as integer microseconds
                except (ValueError, TypeError, IndexError):
                    continue  # skip the row -- bad uptime would crash column() downstream
                stream.rows.append(values)
        else:  # a log line: '<ticks_us> <descriptor> :: <message>'
            first = line.split(' ', 1)[0]
            logs.append((int(first) if first.isdigit() else None, line))
    _unwrap(streams, logs)
    """
    Normalise every timestamp to a flight-relative origin. The recorder stamps raw board uptime
    (ticks_us), which starts wherever the board happened to be at boot -- so an un-normalised plot reads
    ~600 s at boost, not 0. Subtract the earliest stamp seen so the capture (and every renderer keyed on
    these times) starts at t=0.
    """
    stamps = [row[0] for stream in streams.values() for row in stream.rows]
    stamps += [ts for ts, _ in logs if ts is not None]
    if stamps:
        origin = min(stamps)
        for stream in streams.values():
            for row in stream.rows:
                row[0] -= origin
        logs = [((ts - origin) if ts is not None else None, line) for ts, line in logs]
    _synthesise_fins(streams)  # after normalisation, so the virtual rows share the flight-relative origin
    return streams, logs
