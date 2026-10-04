"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Parse a Coludo recorder capture (the UART stream to the Luckfox) into aligned telemetry streams + log
lines, for offline analysis. The recorder interleaves two record kinds on uart:1 (recorder.py):
    @<session>_<file>@<row>               telemetry; first row per file is `uptime;<field>;...`, then
                                          each data row is `<uptime_us>;<v>;<v>;...`  (';'-separated)
    <ticks_us> <descriptor> :: <message>  best-effort log line
Since doc/specs/recorder-wire.md every line also carries the integrity wrapper `{OPEN};<row>;<CLOSE>`
(tools/recorder_wire.py). A capture holding wrapped lines is read strictly: a line counts only when its
checks pass, or when salvage proves which routing it belongs to, and a salvaged record is placed by its
time, never where it happened to sit. An older capture, with no wrapper at all, is read on trust as it
always was. line_counts() says how each line of the last capture was judged.
parse() reads a raw capture (both kinds interleaved) and returns the streams + logs; load() reads a
capture file into it as UTF-8 with errors='surrogateescape', so the checks see the bytes the board sent
and a byte the link damaged costs its row, never the whole read. Stdlib only, so it stays importable in
the test suite; the plotly rendering lives in flight_report.py.
"""

import bisect
import re

import recorder_wire

"""
The session prefix every tag opens with: the boot id ('000123_', '%06u' from NVS) on current firmware,
or the date/time of the older eras. What FOLLOWS the date/time varies by era and config: a 6-digit
random the board synthesised, an operator label CC set via `recorder.session`, or -- on the oldest
captures, before the disambiguator existed -- nothing at all. Those are the same SHAPE
('taster_imu_bno055.csv' vs 'imu_bno055.csv'), so the extra token is DERIVED FROM THE DATA by
_session_tail rather than guessed by pattern. The date/time alternative comes first, so an old tag never
loses only its date to the boot-id branch.
"""
_SESSION = re.compile(r'^(?:\d{8}_\d{6}_|\d{6,}_)')
_SERVO = re.compile(r'^servo_(.+)\.csv$')  # a board's per-servo stream -> the surface name it drives


class Stream:
    """One telemetry file: its field names and numeric rows (uptime first)."""

    def __init__(self, name: str):
        self.name: str = name  # the file, e.g. 'adxl375.csv' (session prefix stripped)
        self.fields: list = []  # column names after the leading 'uptime'
        self.rows: list = []  # [uptime_us, v1, v2, ...] per row (floats; '' for a missing/blank cell)
        self.spliced: bool = False  # a second header row appeared -> two boots appended into this file

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
        width = len(self.fields) + 1          # what a COMPLETE row of this stream looks like
        times, values = [], []
        for row in self.rows:
            """
            A row whose width does not match the header is TRUNCATED or MERGED, and its boundary cell
            cannot be trusted -- so the guard is on the row's shape, not just on the cell existing.

            Both failure modes were measured on real captures, and both survive a cell-exists check:

              SHORT -- the record was cut on the wire. `...;4500;-6` is a truncated `-600`, which
              reads as a perfectly reasonable wrong number.

              LONG -- the newline was lost and the next record ran on, so cells past the header's width
              belong to a different stream. One capture read airspeed_cms as 1441938442469, which is
              simply 1441 with the next record's timestamp glued to it.

            A short row is dropped ENTIRELY rather than trimmed to its last cell, and that is the
            conservative choice on purpose. Trimming assumes the loss was at the tail; if a cell went
            missing mid-row instead, every later cell shifts left and lands in the wrong column while
            still parsing cleanly. A real capture showed exactly that -- heading_err reading 255 in a
            9-cell row of a 14-cell stream, at a position the tail-trim left untouched. There is no way
            to tell from the row where the loss happened, so no part of it is trustworthy.

            The cost is negligible and was measured: 20 mis-width rows in 1,004,804 across 48 flights.
            """
            if len(row) != width:
                continue                      # see above: a mis-width row is not partially trustworthy
            if index < len(row) and row[index] != '':
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


_TICKS_PERIOD: int = recorder_wire.TICKS_PERIOD  # ticks_us wraps here (~1073.7 s ~ 17.9 min of board uptime)


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
            # A malformed log line carries NO stamp, so there is nothing to correct and nothing to go
            # stale. Both `offset` and `previous` survive the skip, so the next stamped line is still
            # compared against the last VALID one and a wrap across the gap is still detected.
            continue
        if previous is not None and stamp + offset < previous - half:
            offset += _TICKS_PERIOD
        logs[index] = (stamp + offset, line)
        previous = stamp + offset


def _session_tail(names: list) -> str:
    """
    The extra session token shared by every tag in ONE capture, or '' when there is none.

    Args:
        names - the post-date/time remainders of every tag seen in the capture.

    Returns:
        The common leading token including its underscore, or '' when there is none.
    """
    if not names:
        return ''
    if any(_SERVO.match(name) for name in names):
        """
        A name that ALREADY parses as a bare 'servo_<surface>.csv' proves there is no tag: with one
        present those names read '<tag>_servo_yaw.csv' and cannot match. Without this, a capture whose
        streams happen to share a structural prefix -- a servo-only capture does -- has that prefix
        mistaken for a session tag, and stripping it breaks the very fin synthesis that depends on it.
        """
        return ''
    heads = {name.split('_', 1)[0] for name in names if '_' in name}
    if len(heads) != 1:
        return ''  # streams disagree -> no shared tag
    head = heads.pop()
    if '.' in head:
        return ''  # a tag never contains a dot; this is a lone stream name like 'laser_agl.csv'
    if head.isdigit():
        return '%s_' % head  # unambiguous: no stream is named '<digits>_...', so it can only be the tag
    """
    A WORD tag ('taster') is shape-identical to a stream's first word ('imu'), so it needs corroborating
    evidence: EVERY stream must carry it. A single name with no underscore at all ('health.csv') is proof
    there is no shared tag -- and every real capture has several such streams.
    """
    if len(names) >= 2 and all('_' in name for name in names):
        return '%s_' % head
    return ''


def simulated(streams: dict) -> bool:
    """
    Did this capture come from the HITL sim rather than a real flight?

    The sim task records its own `hitl_clock.csv`; nothing on a real flight writes it. The distinction
    matters because a simulated capture cannot support every kind of analysis: in the sim the pitot
    reading and the GNSS ground speed are both derived from one body state, so fitting one to the other
    measures the model rather than the atmosphere. A tool that recommends a HARDWARE setting from that
    is confidently wrong, and nothing about the output would say so.

    Measuring the sim on purpose is legitimate (it is how glide_polar's accuracy was calibrated), so
    this reports rather than forbids -- the caller decides whether it is validation or a mistake.

    Args:
        streams - the parsed {file -> Stream} map.

    Returns:
        True when the capture carries the sim's own clock stream.
    """
    return 'hitl_clock.csv' in streams


def spliced(streams: dict) -> list:
    """
    The streams that carry more than one recorder session, i.e. two boots appended into one file.

    Args:
        streams - the parsed {file -> Stream} map.

    Returns:
        The sorted names of spliced streams; empty when the capture is one clean session.
    """
    return sorted(name for name, stream in streams.items() if stream.spliced)


_MARKER = re.compile(r'@(?:[0-9]{8}_[0-9]{6}_[A-Za-z0-9]*_?|[0-9]{6,}_)[a-z0-9_]+\.csv@')
_BOOT_ID = re.compile(r'^\d{6,}_(?=[a-z])')  # a boot-id prefix: digits, then a stream name (never a date's digit)
_PLACE_SLACK: int = 60_000_000  # us a recovered record may lie past the good lines around it (stamped, then
                                # drained; logs flushed in batches) -- any window under a wrap still places uniquely
_SPLICED: list = []      # stream names whose line was found spliced, reset per parse()
_COUNTS: dict = dict.fromkeys(('good', 'salvaged', 'rejected', 'legacy'), 0)  # line verdicts, per parse()


def name_hint(tag: str) -> str:
    """The stream name from a record tag, for reporting which stream lost a row."""
    return _SESSION.sub('', tag)


def spliced_rows() -> int:
    """How many spliced lines the LAST parse() split apart (0 on a clean capture)."""
    return len(_SPLICED)


def line_counts() -> dict:
    """
    How the LAST parse() judged its lines, as {verdict: count}.

    good -- wrapped, and both checks pass under its own routing. salvaged -- a record of a damaged line
    that checks out under a routing the capture uses (or, by its leftover `<name>@`, a stream of its
    boot-id session) and is placed by time. rejected -- dropped: a damaged record nothing claims, the
    truncated piece of a spliced line, a proven record whose time cannot be placed unambiguously, or an
    unwrapped line inside a wrapped capture. legacy -- a line of a capture with no wrapper at all, taken
    on trust. A line holding several records is counted as those records.

    Returns:
        {'good', 'salvaged', 'rejected', 'legacy'} -> count; all 0 before the first parse().
    """
    return dict(_COUNTS)


def line_summary() -> str:
    """The LAST parse()'s line verdicts as one phrase, for the capture-health lines a tool prints."""
    if _COUNTS['legacy']:
        return '%d unchecked (a capture from before the integrity wrapper)' % _COUNTS['legacy']
    return '%(good)d good, %(salvaged)d salvaged, %(rejected)d rejected' % _COUNTS


def _entry(line: str) -> tuple:
    """A capture line with its verdict: (line, status, routing, payload or raw body)."""
    return (line,) + recorder_wire.split(line)


def _stream(streams: dict, tail: str, tag: str) -> Stream:
    """
    The stream a record's tag names, created on first use.

    Args:
        streams - {file -> Stream}, added to.
        tail - the capture's shared session token (_session_tail), stripped after the session prefix.
        tag - the record's routing, '<session>_<file>'.

    Returns:
        The Stream keyed by the bare file name.
    """
    name = _SESSION.sub('', tag)  # 'YYYYMMDD_HHMMSS_<tail>imu.csv' -> '<tail>imu.csv'
    if tail and name.startswith(tail):
        name = name[len(tail):]  # ... -> 'imu.csv'
    stream = streams.get(name)
    if stream is None:
        stream = streams[name] = Stream(name)
    return stream


def _values(cells: list) -> list | None:
    """A data row's cells as numbers, the uptime as integer microseconds; None when the uptime does not parse."""
    values = [_number(cell) for cell in cells]
    try:
        values[0] = int(float(values[0]))
    except (ValueError, TypeError, IndexError):
        return None  # bad uptime would crash column() downstream
    return values


def _row(streams: dict, tail: str, tag: str, row: str) -> list | None:
    """
    File one telemetry row under the stream its tag names.

    Args:
        streams - {file -> Stream}, mutated in place.
        tail - the capture's shared session token (_session_tail), stripped after the session prefix.
        tag - the record's routing, '<session>_<file>'.
        row - the row as recorded: a header `uptime;<field>;...` or `<uptime_us>;<v>;...`.

    Returns:
        The row as filed; None for a header, and for a row whose uptime does not parse (dropped).
    """
    stream = _stream(streams, tail, tag)
    cells = row.split(';')
    if cells[0] == 'uptime':
        """
        A header row. A SECOND one in the same stream means two boots wrote the same file -- the
        Luckfox appends, so their rows are now interleaved with uptime restarting midway, and no
        downstream parsing can separate them. That happens when two sessions land on the same prefix:
        the old 3-digit random collided about 12 times in 150 unsynced boots, and a stale
        `recorder.session` in a saved config collides EVERY boot. The corruption used to be invisible
        here -- the repeat header failed the uptime parse and was dropped silently -- so it is flagged on
        the stream, and `spliced()` puts it in front of whoever reads the capture. Nothing is thrown
        away: the rows still parse, they are just not one flight. The one exception is the shared
        session.csv, whose header every boot sends, and again before its anchor row, by design.
        """
        if not stream.fields:
            stream.fields = cells[1:]
        elif stream.name != recorder_wire.SESSION_INDEX:
            stream.spliced = True
        return None
    values = _values(cells)
    if values is not None:
        stream.rows.append(values)
    return values


def _log(logs: list, line: str) -> None:
    """
    Keep a log line with its stamp.

    Args:
        logs - the (uptime_us | None, line) list, appended to.
        line - the log line, '<ticks_us> <descriptor> :: <message>'; its stamp is None when it does not
            open with digits.

    Returns:
        None.
    """
    first = line.split(' ', 1)[0]
    logs.append((int(first) if first.isdigit() else None, line))


def _legacy(streams: dict, logs: list, tail: str, line: str, queue: list) -> None:
    """
    Read one line of an UNWRAPPED capture, exactly as every capture was read before the wrapper.

    Args:
        streams - {file -> Stream}, mutated in place.
        logs - the (uptime_us | None, line) list, appended to.
        tail - the capture's shared session token.
        line - the stripped capture line.
        queue - parse()'s line queue; the second record of a spliced line is appended to it.

    Returns:
        None.
    """
    if not line.startswith('@'):
        _log(logs, line)
        return
    tag, _, row = line[1:].partition('@')
    if not row:
        return
    """
    SPLICED LINE -- two records that ran together because the first lost its newline.

    Measured on the real recorder path (board -> UART -> Luckfox -> adb): 20 lines in 1,004,804 across
    48 flights, so about one in three flights carries one. The first record is TRUNCATED mid-field and
    the next record's whole `@session_stream@...` text follows it on the same line.

    Left alone this is silently destructive, not merely lossy: the truncated row keeps parsing, and the
    SECOND record's fields land in the FIRST record's columns. That is where the impossible values come
    from -- an airspeed of 1.4e12 cm/s and a heading error of 15330 deg, both of which are simply the
    next stream's numbers read in the wrong place. A tool then treats them as flight data (this class
    already produced a reported L/D of 64).

    So the line is SPLIT at the second marker and both halves parsed where they belong. The truncated
    half loses its tail to the short-row guard, which is correct -- that data really is gone -- but
    nothing is misattributed, and spliced_rows() counts them so a capture can say how much it lost
    rather than looking clean.
    """
    marker = _MARKER.search(row)
    if marker is not None:
        queue.append(_entry('@' + row[marker.start() + 1:]))   # re-queue the second record intact
        row = row[: marker.start()]
        _SPLICED.append(name_hint(tag))
        if not row:
            return
    _row(streams, tail, tag, row)


def _strict(streams: dict, logs: list, tail: str, entries: list, known: list) -> None:
    """
    Read a WRAPPED capture: only what the checks prove is kept, and a recovered record is placed by time.

    A good line is filed where it stands. A damaged one is recovered (recorder_wire.recover): salvaged
    whole when one routing makes it check out -- a known one, the log, or the `<name>@` it kept of a
    stream of its own boot-id session -- else cut into the records that ran together in it, each judged on
    its own. An unwrapped line here is damage, not history. A recovered record has no trustworthy place
    in the capture: assemble_capture sorts a junk-named file anywhere among the streams, and a merged
    piece sits in another stream's file. Filed where it was found, _unwrap read it as a ticks wrap and
    shifted a whole stream by 17.9 minutes. So recovered records are held until the good lines are
    unwrapped, then placed by time (_place).

    Args:
        streams - {file -> Stream}, filled.
        logs - the (uptime_us | None, line) list, filled.
        tail - the capture's shared session token.
        entries - the capture's lines with their verdicts (_entry()), in capture order.
        known - every routing the capture is known to use, for salvage.

    Returns:
        None; _COUNTS records every verdict.
    """
    prefixes = tuple(sorted({found.group(0) for found in map(_BOOT_ID.match, known) if found}))
    timelines, order, held = {}, [], []  # good stamped lines per source; each log's capture index; held records
    for index, (_line, status, routing, text) in enumerate(entries):
        if status == recorder_wire.GOOD:
            _COUNTS['good'] += 1
            if routing is None:
                _log(logs, text)
                order.append(index)
                if logs[-1][0] is not None:
                    timelines.setdefault(None, []).append((index, len(logs) - 1))
            else:
                row = _row(streams, tail, routing, text)
                if row is not None:
                    timelines.setdefault(routing, []).append((index, row))
        elif status == recorder_wire.LEGACY:
            _COUNTS['rejected'] += 1
        else:
            records, lost = recorder_wire.recover(routing, text, known, prefixes)
            _COUNTS['rejected'] += lost
            if len(records) + lost > 1:
                _SPLICED.append(name_hint(routing or ''))
            held += [(index, routing, found, payload) for found, payload, _record in records]
    _unwrap(streams, logs)
    _place(streams, logs, tail, held, timelines, order)


def _window(low: int | None, high: int | None, bounds: tuple | None) -> tuple | None:
    """
    The window a held record's time must fall in.

    Args:
        low, high - the unwrapped stamps of the good lines just before and after it; None when there is none.
        bounds - (first, last) standing in for a missing side; None when there is nothing to stand in.

    Returns:
        (low, high) widened by _PLACE_SLACK; None when a side is missing and `bounds` too.
    """
    if bounds is None and (low is None or high is None):
        return None
    return ((bounds[0] if low is None else low) - _PLACE_SLACK, (bounds[1] if high is None else high) + _PLACE_SLACK)


def _place(streams: dict, logs: list, tail: str, held: list, timelines: dict, order: list) -> None:
    """
    File the held records by time, once the good lines around them are unwrapped.

    A record is timed by the good lines of the SOURCE it was found in -- the routing its line arrived
    under, i.e. its Luckfox file, which is written in time order: it lies between the nearest good line
    before it and the nearest one after it, give or take _PLACE_SLACK. A side with neither is bounded by
    the capture's span; a junk file holds no good line at all, so the record's own stream's span stands
    in. Its uptime is then the one recorded + k * 2**30 inside that window (recorder_wire.fit). None, or
    more than one, and it is rejected rather than guessed; so is any row for a stream that holds two
    boots, which has no single timeline. A row joins its stream in time order, a log line the logs at its
    place in the capture.

    A SPARSE stream is the one limit. _unwrap() unwraps each stream by its own rows, and a stream whose
    rows lie half a wrap or more apart (a servo holding still through a 20-minute pad dwell, the
    sequencer's few events), or whose first row comes after a wrap, never shows that wrap. In a capture
    longer than half a wrap its good rows can then keep their recorded time, while a row placed by another
    file's lines gets the true one (a junk file's row, timed by the stream's own span, keeps the stream's
    time). Its own time cannot be had instead: that needs the stream's good rows on the common timeline,
    and an assembled capture lays each Luckfox file out whole, so nothing orders one stream's rows against
    another's. recorder_flight cuts such a stream modulo 2**30 into the flight window, where both agree.

    Args:
        streams - {file -> Stream}, mutated in place.
        logs - the (uptime_us | None, line) list, unwrapped; held log lines are merged in.
        tail - the capture's shared session token.
        held - [(capture index, source routing, routing, payload)] of the recovered records.
        timelines - {source routing: [(capture index, its row, or its index in `logs`)]} of good stamped lines.
        order - the capture index of each entry of `logs`.

    Returns:
        None; _COUNTS gains a 'salvaged' or a 'rejected' per held record.
    """
    marks = {source: ([index for index, _mark in lines], [logs[mark][0] if source is None else mark[0]
                                                          for _index, mark in lines])
             for source, lines in timelines.items()}
    stamps = [row[0] for stream in streams.values() for row in stream.rows]
    capture = (min(stamps), max(stamps)) if stamps else None
    spans = {name: (min(row[0] for row in stream.rows), max(row[0] for row in stream.rows))
             for name, stream in streams.items() if stream.rows}
    twice = {name for name, stream in streams.items()
             if any(earlier[0] > later[0] for earlier, later in zip(stream.rows, stream.rows[1:]))}
    placed, extra = set(), []
    for index, source, routing, payload in held:
        stream = None if routing is None else _stream(streams, tail, routing)
        head = payload.split(' ', 1)[0] if stream is None else payload.split(';', 1)[0]
        if stream is not None and head == 'uptime':
            _row(streams, tail, routing, payload)  # a header: nothing to place in time
            _COUNTS['salvaged'] += 1
            continue
        if stream is None and not head.isdigit():
            extra.append((index, (None, payload)))  # a log line with no stamp keeps its place in the capture
            _COUNTS['salvaged'] += 1
            continue
        values = [int(head)] if stream is None else _values(payload.split(';'))
        indices, times = marks.get(source, ([], []))
        at = bisect.bisect(indices, index)
        low, high = (times[at - 1] if at else None), (times[at] if at < len(times) else None)
        alone = low is None and high is None and stream is not None
        window = _window(low, high, spans.get(stream.name, capture) if alone else capture)
        stamp = None
        if values is not None and window is not None and (stream is None or stream.name not in twice):
            stamp = recorder_wire.fit(values[0], window[0], window[1])
        if stamp is None:
            _COUNTS['rejected'] += 1
            continue
        if stream is None:
            extra.append((index, (stamp, payload)))
        else:
            values[0] = stamp
            stream.rows.append(values)
            placed.add(stream.name)
        _COUNTS['salvaged'] += 1
    for name in placed:
        streams[name].rows.sort(key=lambda row: row[0])  # two sorted runs: the good rows and the placed ones
    if extra:
        entries = sorted(list(zip(order, logs)) + extra, key=lambda entry: entry[0])
        logs[:] = [entry for _index, entry in entries]


def load(path: str) -> tuple:
    """
    Read and parse a capture file -- the one way every tool reads one.

    Args:
        path - the capture file.

    Returns:
        parse()'s ({file -> Stream}, logs) for the file's bytes, decoded as UTF-8 with surrogateescape.
    """
    with open(path, encoding='utf-8', errors='surrogateescape') as handle:
        return parse(handle.read())


def parse(text: str) -> tuple:
    """
    Parse a raw capture into aligned streams and log lines.

    Args:
        text - the raw recorder capture (both record kinds interleaved), as read with
            errors='surrogateescape' so a CRC sees the bytes the board sent.

    Returns:
        ({file -> Stream}, logs), where logs is a list of (uptime_us | None, line). Every timestamp is
        normalised to a flight-relative origin (the earliest stamp seen is subtracted), so a capture
        starts at t=0 rather than at the board's raw boot uptime. line_counts() then says how the
        lines were judged.
    """
    streams = {}
    logs = []
    del _SPLICED[:]          # per-parse, so spliced_rows() describes THIS capture
    _COUNTS.update(dict.fromkeys(_COUNTS, 0))
    queue = [_entry(line) for line in (raw.strip() for raw in text.splitlines()) if line]
    good = {routing for _line, status, routing, _text in queue if status == recorder_wire.GOOD}
    """
    One line that checks out makes the whole capture WRAPPED, and then nothing unproven is taken. The
    decision rests on a passing CRC, never on a line merely LOOKING wrapped: a damaged old line that
    happened to resemble the wrapper would otherwise turn a whole legacy capture into rejects.
    Host-built lines in a wrapped capture (assemble_capture's stage marks, hitl_collect's build note)
    are wrapped by the tools that add them, so they still count.
    """
    wrapped = bool(good)
    if wrapped:
        known = sorted((good - {None}) | {recorder_wire.SESSION_INDEX})
        tags = known
    else:
        tags = [line[1:].partition('@')[0] for line, _status, _routing, _text in queue
                if line.startswith('@') and line[1:].partition('@')[2]]
    """
    Learn this capture's session tail before any stream is keyed by it. The shared session.csv carries no
    session prefix at all, so it says nothing about one -- and it must not, or its underscore-free name
    would veto a label every other stream shares. (The CC tee keeps lines unwrapped, so an unwrapped
    capture of current firmware holds it too.)
    """
    tail = _session_tail(sorted({_SESSION.sub('', tag) for tag in tags if tag != recorder_wire.SESSION_INDEX}))
    if wrapped:
        _strict(streams, logs, tail, queue, known)  # unwraps its good lines, then places the recovered ones
    else:
        for entry in queue:  # a spliced line appends its second record, which this loop then reaches
            _COUNTS['legacy'] += 1
            _legacy(streams, logs, tail, entry[0], queue)
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
