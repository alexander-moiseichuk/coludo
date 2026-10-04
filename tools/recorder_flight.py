"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Cut one flight out of a Luckfox recorder dump (`adb pull /userdata/recordings`) -- stdlib only.

The Luckfox demuxes the board's UART stream into one CSV per stream, `<session>_<stream>.csv`, plus a
shared recorder.log. A flight is a few seconds inside a session that can run for many minutes, and the
board's clock is often unset, so every session is named 2000-01-01: the flight is found by its BOOST, not
its date. This writes, for the window from `--before` s ahead of ignition to the session's last row:

    <out>/recorder/<stream>.csv   the ORIGINAL lines, verbatim, whose leading uptime is in the window
    <out>/recorder/board.log      the session's board log lines from recorder.log, verbatim
    <out>/flight/<stream>.csv     the same rows parsed and NAMED, t_s from ignition, ';' like the device

    python3 tools/recorder_flight.py <dump>/recordings --session 20000101_000006_898573 \\
        -o launches/20261003/TMS-7C

Ignition is the rule src/logger/flight.py uses: the first |a| over 3 g held 0.3 s, walked back to where
|a| left 1.1 g -- on the LSM6DSO32 by default (the ADXL375 sits ~0.8 g off zero, too far for 1.1 g).

UART corruption is the norm on these dumps, and the tool assumes it. Only known streams are read (the
Luckfox spins corrupted names off into thousands of one-row files); a row is kept only where it continues
its file's own time sequence -- an uptime that goes backwards or jumps more than 5 s is corrupted; and a
row with the wrong field count stays in recorder/ (it is original) but not in flight/.

THE UPTIME WRAPS: the board stamps rows with MicroPython ticks_us, which wraps at 2**30 us (17.9 min). A
pad dwell longer than that wraps -- TMS-7D sat 23.5 min powered -- and the drop looks like a reboot. A drop
that matches the wrap (the next rows confirm it) is UNWRAPPED, so a session stays one timeline and
flight/'s uptime_us is the unwrapped value; recorder/ keeps the lines as recorded. A drop that does not
match the wrap is a real reboot appending to the same session: files are split there, and the flight is
taken from the boot whose accelerometer holds the boost -- boot k of every continuous stream (`--boot`
overrides). A sparse stream (a sequencer event, a servo move) never shows a restart; its rows inside the
window are kept from every boot and the stream is flagged to check.

A CRASH ENDS THE SESSION MID-BUFFER: each stream's tail is whatever the Luckfox had flushed when power
went, so the streams end at different times, and a slow stream may never have been written at all.

A WRAPPED SESSION (doc/specs/recorder-wire.md, firmware that names its session by boot id, `000123`) is
read by its checks instead of by its time sequence. A row counts when it checks out under its file's name.
A row that does not is salvaged: the routing it proves among the session's own is where it belongs. That
covers a corrupted name (the Luckfox's junk files), a lost leading '@' (recorder.log), a lost second '@'
(recorder.log too), and two records on one line -- merged, or a splice -- which is cut where the next
record starts. A salvaged row is placed by its TIME among its stream's rows, read across the ticks_us
wrap from the good lines around it where it was found; a time no single place fits is rejected, as is
whatever proves nothing, and every stream reports good / salvaged in / rejected. Under a boot id each row
is proven, so any `<session>_<stream>.csv` holding one is a stream, not only the known ones; under a label
only the known ones are. recorder/ keeps each row as its own file holds it, wrapper included, byte for
byte; flight/ gets the payloads. The shared session.csv lists every boot: the report says which of its
rows date the session's boot, or that its clock was never set, and then names it by boot id, board,
firmware and config; a dated boot's ignition gets its UTC. A dump from before the wrapper is read exactly
as before.
"""

import argparse
import bisect
import csv
import datetime
import itertools
import math
import os
import re
import sys

import recorder_wire

# The firmware's Telemetry declarations (src/glider), used when a stream's own header row was lost to
# UART corruption -- which is the usual case: the header goes out once, at the start of a session.
_FIELDS: dict = {
    'imu_lsm6dso32': ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'irq_runs'),
    'accel_adxl375': ('ax', 'ay', 'az', 'irq_runs'),
    'imu_bno055': ('heading', 'roll', 'pitch', 'ax', 'ay', 'az'),
    'baro_bmp280': ('altitude', 'temperature', 'pressure', 'elevation'),
    'baro_icp10111': ('altitude', 'temperature', 'pressure', 'elevation'),
    'airspeed_sdp810': ('dynamic_pressure', 'airspeed_cms', 'temperature'),
    'laser_agl': ('agl', 'irq_runs'),
    'health': ('temp', 'mem_free', 'load', 'oom_s', 'land_s', 'leak_kbps', 'rescues', 'rescue_ms'),
    'gnss_gga': ('altitude_m', 'elevation_m', 'quality', 'satellites', 'hdop_cd'),
    'power_ina226': ('voltage_mv', 'current_ma', 'power_mw', 'alerts'),
    'servo_eleron_left': ('angle', 'pulse_us', 'done'),
    'servo_eleron_right': ('angle', 'pulse_us', 'done'),
    'servo_yaw': ('angle', 'pulse_us', 'done'),
    'checkpoint': ('stage', 'altitude', 'speed', 'airspeed', 'ticks_ms'),
    'separation': ('event', 'stage'),
    'sequencer': ('stage', 'reason'),
    'sequencer_events': ('stage', 'reason'),
}
_BOOST_G: float = 3.0
_BOOST_HOLD_S: float = 0.3
_BOOST_SAMPLES: int = 10       # ... over at least this many real samples, never one corrupted row
_ONSET_G: float = 1.1
_PLAUSIBLE_G: float = 250.0    # above every accelerometer fitted (ADXL375 +/-200 g): a corrupted value
_GAP_US: int = 5_000_000       # a jump past this needs the next row to confirm it (else: corrupted uptime)
_RESTART_US: int = 10_000_000  # a drop of more than this, continued by the next rows, is a wrap or a new boot
_WRAP_US: int = recorder_wire.TICKS_PERIOD  # MicroPython ticks_us period (17.9 min): a drop of this size is a wrap
_WRAP_SLACK_US: int = 60_000_000  # ... give or take the rows a slow stream has before and after it
_CONFIRM_ROWS: int = 3
_PROBE_LINES: int = 100       # lines per file _wrapped() reads: one good row decides
_STATUS = re.compile(r"'session': '([^']*)'")
_TEXT_FIELDS: tuple = ('stage', 'reason', 'event')  # recorded as words; every other field is a number
_CODEC: dict = {'encoding': 'utf-8', 'errors': 'surrogateescape'}  # writes give back the bytes _lines() read
_NO_STREAM: str = '(no stream)'  # the tally of this session's lines that prove no row at all
"""
The shared session.csv (doc/specs/recorder-wire.md): a row per boot, per clock set and per anchor. utc and
utc_offset are empty while the clock is unset -- the RTC reads before 2001 -- and such a row dates nothing.
"""
_INDEX_FIELDS: tuple = ('uptime', 'boot', 'session', 'utc', 'utc_offset', 'board', 'firmware', 'config_id',
                        'source', 'cc_lat', 'cc_lon')
_CLOCK_SET: str = '2001-01-01'  # a utc before this is a clock never set (the board leaves the cell empty then)
_ANCHOR_US: int = 60_000_000    # the boot's 'anchor' row goes out a minute after the Recorder starts


def _uptime(line: str) -> int | None:
    """The leading uptime (us) of a row, or None."""
    head = line.split(';', 1)[0]
    return int(head) if head.isdigit() else None


def _payload(line: str) -> str:
    """A row's recorded fields: the payload of a wrapped row, the row itself on an unwrapped dump."""
    parts = recorder_wire.unwrap(line)
    return line if parts is None else parts[1]


def _in_sequence(rows: list) -> list:
    """
    The rows that continue their file's time sequence, in file order: never before the last kept row,
    and a jump of more than _GAP_US only where the NEXT row carries on from it. A stream can genuinely go
    quiet for seconds (the laser has no range); a corrupted uptime is one row the sequence never follows.
    """
    if len(rows) == 1:
        return list(rows)  # a lone event (the launch detection itself) has nothing to confirm it against
    kept = []
    for index, (uptime, line) in enumerate(rows):
        following = rows[index + 1][0] if index + 1 < len(rows) else None
        confirmed = following is not None and 0 <= following - uptime <= _GAP_US
        if not kept:
            if confirmed:
                kept.append((uptime, line))
        elif 0 <= uptime - kept[-1][0] and (uptime - kept[-1][0] <= _GAP_US or confirmed):
            kept.append((uptime, line))
    return kept


def _boots(rows: list) -> list:
    """
    The file's rows (file order) split into boots, uptimes unwrapped: a confirmed drop of the ticks_us
    period is a wrap and is added back; any other drop of more than _RESTART_US starts a new boot when the
    next _CONFIRM_ROWS rows carry on from it -- one corrupted small uptime never does.
    """
    boots, current, last_good, offset, wraps = [], [], None, 0, 0
    for index, (recorded, line) in enumerate(rows):
        uptime = recorded + offset
        following = [u + offset for u, _line in rows[index + 1:index + 1 + _CONFIRM_ROWS]]
        confirmed = len(following) == _CONFIRM_ROWS and all(
            0 <= b - a <= _GAP_US for a, b in zip([uptime] + following, following))
        # measured against the last uptime its successors CONFIRMED: a corrupted large uptime kept in the
        # file would otherwise make the next genuine row look like a restart
        if current and last_good is not None and uptime < last_good - _RESTART_US and confirmed:
            if abs(last_good - uptime - _WRAP_US) <= _WRAP_SLACK_US:
                offset += _WRAP_US       # the ticks_us wrap: the same boot carries on
                uptime += _WRAP_US
                wraps += 1
            else:
                boots.append(current)    # a real reboot appended to the same session
                current = []
        current.append((uptime, line))
        if confirmed:
            last_good = uptime
    if current:
        boots.append(current)
    return boots, wraps


def _streams(directory: str, session: str) -> dict:
    """stream -> (header or None, [boot rows [(uptime_us, line)] in sequence], wraps seen) for known streams."""
    streams = {}
    for stream in _FIELDS:
        path = os.path.join(directory, '%s_%s.csv' % (session, stream))
        if not os.path.exists(path):
            continue
        header, rows = None, []
        with open(path, 'rb') as handle:
            for raw in handle:
                line = raw.decode('ascii', 'replace').rstrip('\r\n')
                if line.startswith('uptime;'):
                    header = tuple(line.split(';')[1:])
                    continue
                uptime = _uptime(line)
                if uptime is not None:
                    rows.append((uptime, line))
        boots, wraps = _boots(rows)
        streams[stream] = (header, [_in_sequence(boot) for boot in boots], wraps)
    return streams


def _lines(path: str) -> list:
    """
    A dump file's lines as the Luckfox wrote them, newlines stripped. Decoded as UTF-8 with
    errors='surrogateescape', so a byte the link damaged stays itself and a CRC sees exactly what was written.
    """
    with open(path, 'rb') as handle:
        return [raw.decode('utf-8', 'surrogateescape').rstrip('\r\n') for raw in handle]


def _ascii_lines(path: str) -> list:
    """A file's lines as the cuts of unwrapped dumps in launches/ were made: a byte past ASCII reads U+FFFD."""
    with open(path, 'rb') as handle:
        return [raw.decode('ascii', 'replace').rstrip('\r\n') for raw in handle]


def _wrapped(directory: str, session: str) -> bool:
    """
    Does this session carry the integrity wrapper?

    Decided by one row that checks out under its own file name: a CRC match is proof, while a row merely
    SHAPED like the wrapper could be damage in an old dump. Only the first _PROBE_LINES of each file are
    read, so an old session is not read in full to say no.

    Args:
        directory - the pulled /userdata/recordings directory.
        session - the session prefix.

    Returns:
        True when a row of one of the session's files checks out under that file's name.
    """
    prefix = session + '_'
    for name in sorted(os.listdir(directory)):
        if name.startswith(prefix) and name.endswith('.csv'):
            with open(os.path.join(directory, name), 'rb') as handle:
                for raw in itertools.islice(handle, _PROBE_LINES):
                    if recorder_wire.verify(name, raw.decode('utf-8', 'surrogateescape').rstrip('\r\n')) is not None:
                        return True
    return False


def _session_range(lines: list, session: str) -> tuple:
    """
    The session's part of recorder.log: from the line after the previous session's last status line (a
    board logs its boot before the recorder's first status names the new session) to the line before the
    next session's first status line. Its head can still hold the previous boot's last lines, logged after
    that boot's last status line; _log_timeline() tells them apart by their ticks.

    Args:
        lines - recorder.log's lines.
        session - the session prefix the recorder's status lines name.

    Returns:
        (first index, index of the session's first status line, end index) of the part; (0, 0, 0) when no
        status line names the session.
    """
    tags = [(index, found.group(1)) for index, line in enumerate(lines) for found in [_STATUS.search(line)]
            if found]
    mine = [index for index, tag in tags if tag == session]
    if not mine:
        return 0, 0, 0
    before = [index for index, tag in tags if index < mine[0] and tag != session]
    after = [index for index, tag in tags if index > mine[-1] and tag != session]
    return (before[-1] + 1 if before else 0), mine[0], (after[0] if after else len(lines))


def _log_timeline(lines: list, start: int, first: int, end: int) -> tuple:
    """
    The good log lines of one session's part of recorder.log, for timing the rows stranded among them.

    A row that lost its leading '@' lands in recorder.log when it arrives, so this session's log lines
    around it say when that was -- give or take a log flush, since the board batches its log lines. A
    board stamps its log lines in order, so their ticks only fall back at a ticks_us wrap or at a reboot.
    A fall counts as a wrap only when it is one, 2**30 give or take _WRAP_SLACK_US (as in _boots()), and
    only after the session's first status line: the lines before it are this boot's first few seconds, or
    the previous boot's last ones. Any other fall of more than _RESTART_US is another boot's lines ending
    there, so the timeline restarts at the line after it -- a previous boot that ran a while would
    otherwise read as a wrap, and file every row stranded here 17.9 minutes late. A smaller fall cannot
    bracket anything in order, and is left out.

    Args:
        lines - recorder.log, from _lines().
        start, first, end - the session's part of it and its first status line (_session_range()).

    Returns:
        (valid, [line index], [ticks]): the timeline holds for the lines from index `valid` on, and lists
        the lines that check out as stamped log lines, in file order, the ticks unwrapped across the wrap.
    """
    valid, indices, ticks, offset = start, [], [], 0
    for index in range(start, end):
        payload = recorder_wire.verify(None, lines[index])
        head = '' if payload is None else payload.split(' ', 1)[0]
        if not head.isdigit():
            continue
        stamp = int(head) + offset
        if ticks and stamp < ticks[-1]:
            if index > first and abs(ticks[-1] - stamp - _WRAP_US) <= _WRAP_SLACK_US:
                offset += _WRAP_US       # the ticks_us wrap: the same boot carries on
                stamp += _WRAP_US
            elif stamp < ticks[-1] - _RESTART_US:
                valid, indices, ticks, offset, stamp = index, [], [], 0, int(head)  # another boot ended here
            else:
                continue
        indices.append(index)
        ticks.append(stamp)
    return valid, indices, ticks


def _bracket(positions: list, stamps: list, position: int) -> tuple:
    """
    The good lines either side of a stranded one, as (low, high).

    Args:
        positions - the line numbers of the good lines, ascending.
        stamps - their unwrapped uptimes, in the same order.
        position - the stranded line's number in the same file.

    Returns:
        The uptimes of the nearest good line before and after `position`; None for a side with none.
    """
    at = bisect.bisect(positions, position)
    return (stamps[at - 1] if at else None), (stamps[at] if at < len(stamps) else None)


def _place(boots: list, wraps: int, recorded: int, bracket: tuple, span: tuple) -> tuple | None:
    """
    Where a recovered row joins its stream, by its time.

    The row is timed by the good lines around it where it was found -- its own file's rows, or this
    session's log lines when it was stranded in recorder.log, a file written in time order either way. It
    must fall between them, give or take _WRAP_SLACK_US, at exactly one count of wraps (recorder_wire.fit);
    a side with no good line is bounded by the session's span. A junk file holds no good line at all, so a
    row from one must instead fall inside one boot's span at exactly one count of wraps; in a boot longer
    than a wrap, a time recorded within its first (span - wrap) has two, and is rejected. The row then
    joins the boot whose span holds that time: the only one, when the stream has one.

    A sparse stream (a sequencer event, a servo move) has rows too far apart for _boots() to see its own
    wraps, so its good rows keep their recorded time while a row placed here gets the unwrapped one. That
    stays contained: main() takes the rows of every stream whose wraps or boots differ from the
    accelerometer's modulo 2**30, and places each one again at the single time the flight window allows,
    so the two kinds land on the same timeline (with no wrap at all, the recorded time is the time).

    Args:
        boots - the stream's boots, [(unwrapped uptime, line)] each, from _boots().
        wraps - the wraps _boots() found in the stream.
        recorded - the row's uptime as recorded (raw ticks_us).
        bracket - (low, high), the unwrapped uptimes of the good lines before and after it (_bracket());
            (None, None) for a row from a junk file.
        span - (first, last) unwrapped uptime over the session's streams.

    Returns:
        (boot index, unwrapped uptime); None when no single place fits.
    """
    low, high = bracket
    if low is None and high is None:
        if len(boots) == 1 and not wraps:
            return 0, recorded  # one timeline, never wrapped: the recorded time is the time
        fits = [(index, recorded + count * _WRAP_US) for count in range(wraps + 1)
                for index, boot in enumerate(boots)
                if boot[0][0] - _GAP_US <= recorded + count * _WRAP_US <= boot[-1][0] + _GAP_US]
        return fits[0] if len(fits) == 1 else None
    uptime = recorder_wire.fit(recorded, (span[0] if low is None else low) - _WRAP_SLACK_US,
                               (span[1] if high is None else high) + _WRAP_SLACK_US)
    if uptime is None:
        return None
    if len(boots) == 1:
        return 0, uptime
    owners = [index for index, boot in enumerate(boots) if boot[0][0] - _GAP_US <= uptime <= boot[-1][0] + _GAP_US]
    return (owners[0], uptime) if len(owners) == 1 else None


def _wrapped_streams(directory: str, session: str) -> tuple:
    """
    A WRAPPED session's streams, in _streams()'s shape, with what the checks found.

    Rows are proven, so nothing is judged by its time sequence; _boots still finds the ticks_us wraps.
    Damaged rows come from three places: the session's own files, every junk file of the dump (a name no
    row checks out under), and recorder.log. Each is recovered (recorder_wire.recover) into the session's
    routings, then placed into its stream by time (_place), timed by the good lines around it where it
    was found. Under a boot id, any `<session>_<stream>.csv` holding a good row is a stream. Under a label
    only the streams the firmware declares (_FIELDS) are: an older label could hold '_', so another
    session's files may share this prefix (`hitl_f15_imu_lsm6dso32.csv` beside `hitl_imu_lsm6dso32.csv`).

    Every line that proves nothing is counted where it is this session's: under its stream in the
    stream's own file, and under _NO_STREAM in a junk file carrying the session's prefix or in the
    session's part of recorder.log. A junk file without the prefix, or another boot's part of recorder.log,
    may well hold another session's damage, so what it proves nothing of here is only counted, apart.

    Args:
        directory - the pulled /userdata/recordings directory.
        session - the session prefix, e.g. '000123'.

    Returns:
        (streams {stream: (header or None, boots, wraps)}, tally {stream or _NO_STREAM: [good, salvaged in,
        rejected]}, clock [the session.csv payloads, this session's and any other's], elsewhere: the lines
        of the dump's other junk files and other boots' part of recorder.log that prove no row of this session).
    """
    prefix, boot_id = session + '_', session.isdigit()
    names = sorted(name for name in os.listdir(directory) if os.path.isfile(os.path.join(directory, name)))
    headers, rows, places, stranded, clock = {}, {}, {}, [], []
    tally = {_NO_STREAM: [0, 0, 0]}
    for name in names:
        if not (name.startswith(prefix) and name.endswith('.csv')):
            continue
        stream, header, kept, at, lost, good = name[len(prefix):-len('.csv')], None, [], [], [], 0
        for position, line in enumerate(_lines(os.path.join(directory, name))):
            payload = recorder_wire.verify(name, line)
            if payload is None:
                lost.append((position, line))
                continue
            good += 1
            if payload.startswith('uptime;'):
                header = tuple(payload.split(';')[1:])
            elif _uptime(payload) is not None:
                kept.append((_uptime(payload), line))
                at.append(position)
        if not good:  # a junk name that happens to share the prefix: nothing in it times a row
            owner, source = _NO_STREAM, None
        elif boot_id or stream in _FIELDS:
            owner = source = stream
            rows[stream], places[stream], tally[stream] = kept, at, [good, 0, 0]
            if header is not None:
                headers[stream] = header
        else:
            continue  # a label's file of a stream the firmware does not declare: perhaps another session's
        stranded += [(name, source, position, line, owner) for position, line in lost if recorder_wire.wrapped(line)]
        tally[owner][2] += sum(1 for _position, line in lost if not recorder_wire.wrapped(line))  # damage
    valid, logged, end = 0, ([], []), 0
    if 'recorder.log' in names:
        log = _lines(os.path.join(directory, 'recorder.log'))
        start, first, end = _session_range(log, session)
        valid, *logged = _log_timeline(log, start, first, end)
        for index, line in enumerate(log):
            status, mine = recorder_wire.split(line)[0], valid <= index < end
            if status == recorder_wire.BAD:
                stranded.append((None, 'recorder.log' if mine else None, index, line, _NO_STREAM if mine else None))
            elif status == recorder_wire.LEGACY and mine:
                tally[_NO_STREAM][2] += 1  # the board wraps every line: one with neither end is damage
    for name in names:
        if (name.startswith(prefix) and name.endswith('.csv')) or name == 'recorder.log':
            continue
        path = os.path.join(directory, name)
        with open(path, 'rb') as handle:
            head = handle.readline().decode('utf-8', 'surrogateescape').rstrip('\r\n')
        if name != recorder_wire.SESSION_INDEX and (
                not recorder_wire.wrapped(head) or recorder_wire.verify(name, head) is not None):
            continue  # a file from before the wrapper, or a real stream of another session
        for line in _lines(path):
            payload = recorder_wire.verify(name, line)
            if payload is not None:
                if name == recorder_wire.SESSION_INDEX:
                    clock.append(payload)
            elif recorder_wire.wrapped(line):
                stranded.append((name, None, None, line, None))
    found = {stream: _boots(kept) for stream, kept in rows.items() if kept}
    timelines = {stream: [uptime for boot in boots for uptime, _line in boot]
                 for stream, (boots, _wraps) in found.items()}  # each good row's unwrapped uptime, in file order
    firsts = [timeline[0] for timeline in timelines.values()]
    span = (min(firsts), max(timeline[-1] for timeline in timelines.values())) if firsts else (0, 0)
    routings = sorted({prefix + stream + '.csv' for stream in list(rows) + list(_FIELDS)})
    routings.append(recorder_wire.SESSION_INDEX)
    orphans, elsewhere = {}, 0  # stream -> [(recorded uptime, row)] of a stream no good row placed in time
    for name, source, position, line, owner in stranded:
        if source == 'recorder.log':
            bracket = _bracket(logged[0], logged[1], position)
        elif source is not None:
            bracket = _bracket(places[source], timelines.get(source, []), position)
        else:
            bracket = (None, None)  # a junk file, or recorder.log outside this session's timeline
        records, lost = recorder_wire.recover(name, line, routings, (prefix,) if boot_id else ())
        claimed = False
        for routing, payload, row in records:
            if routing == recorder_wire.SESSION_INDEX:
                clock.append(payload)
                claimed = True
                continue
            if not (routing and routing.startswith(prefix) and routing.endswith('.csv')):
                continue  # a log line, or another session's record
            claimed = True
            stream = routing[len(prefix):-len('.csv')]
            counts = tally.setdefault(stream, [0, 0, 0])
            if payload.startswith('uptime;'):
                headers.setdefault(stream, tuple(payload.split(';')[1:]))
                counts[1] += 1
                continue
            recorded = _uptime(payload)
            spot = None
            if recorded is not None and stream in found:
                spot = _place(found[stream][0], found[stream][1], recorded, bracket, span)
            elif recorded is not None:
                orphans.setdefault(stream, []).append((recorded, row))  # placed by the window, as a sparse stream
                counts[1] += 1
                continue
            if spot is None:
                counts[2] += 1  # proven, but its time fits no single place in the stream
            else:
                bisect.insort(found[stream][0][spot[0]], (spot[1], row))
                counts[1] += 1
        if owner is not None:
            tally[owner][2] += lost
        elif not claimed:
            elsewhere += 1
    streams = {}
    for stream in sorted(set(rows) | set(orphans)):
        boots, wraps = found[stream] if stream in found else _boots(sorted(orphans.get(stream, [])))
        streams[stream] = (headers.get(stream), boots, wraps)
    return streams, tally, clock, elsewhere


def _clean(token: str, field: str) -> str:
    """
    A recorded token as flight/ keeps it: VERBATIM when it is a number (float formatting would cost digits,
    e.g. a 7-digit tick), the device's 'None', or a text field; blank when UART corruption left a fragment
    such as '0.340101_000006_898573_laser_agl.csv@855753238' in a numeric column (recorder/ keeps it).
    """
    if field in _TEXT_FIELDS or token == 'None':
        return token
    try:
        float(token)
        return token
    except ValueError:
        return ''


def _numbers(line: str, count: int) -> list | None:
    """The fields after the uptime as floats (None where not numeric), or None if the count is wrong."""
    parts = line.split(';')[1:]
    if len(parts) != count:
        return None
    out = []
    for part in parts:
        try:
            out.append(float(part))
        except ValueError:
            out.append(None)
    return out


def ignition(rows: list, fields: tuple) -> int:
    """Uptime (us) of the first sample of the boost; ValueError if there is none."""
    axes = [fields.index(axis) for axis in ('ax', 'ay', 'az')]
    samples = []
    for uptime, line in rows:
        values = _numbers(_payload(line), len(fields))
        if values and all(values[i] is not None for i in axes):
            magnitude = math.sqrt(sum(values[i] ** 2 for i in axes))
            if magnitude < _PLAUSIBLE_G:
                samples.append((uptime, magnitude))
    for index, (start, magnitude) in enumerate(samples):
        if magnitude <= _BOOST_G:
            continue
        held = [m for t, m in samples[index:index + 200] if t - start <= _BOOST_HOLD_S * 1e6]
        spans = samples[index + len(held) - 1][0] - start >= 0.8 * _BOOST_HOLD_S * 1e6
        if len(held) >= _BOOST_SAMPLES and spans and min(held) > _BOOST_G:
            while index and samples[index - 1][1] > _ONSET_G:
                index -= 1
            return samples[index][0]
    raise ValueError('no boost: |a| never held %.1f g for %.1f s' % (_BOOST_G, _BOOST_HOLD_S))


def board_log(lines: list, session: str) -> list:
    """
    The session's lines of recorder.log, verbatim.

    Args:
        lines - recorder.log's lines: _lines() for a wrapped session, _ascii_lines() for an older one.
        session - the session prefix.

    Returns:
        The lines of the session's part (_session_range()); [] when no status line names it.
    """
    start, _first, end = _session_range(lines, session)
    return lines[start:end]


def _report_checks(tally: dict, elsewhere: int) -> None:
    """
    Print what the checks found in a wrapped session.

    Args:
        tally - {stream or _NO_STREAM: [good, salvaged in, rejected]} over the whole session, from
            _wrapped_streams().
        elsewhere - the lines elsewhere in the dump that prove no row of this session.

    Returns:
        None.
    """
    print('rows checked over the whole session:')
    for stream in sorted(tally):
        good, salvaged, rejected = tally[stream]
        print('  %-18s %6d good, %6d salvaged in, %6d rejected' % (stream, good, salvaged, rejected))
    if elsewhere:
        print('  %d more damaged line(s) in junk files without this prefix or in other boots\' part of '
              'recorder.log prove no row of this session' % elsewhere)


def _index_rows(clock: list) -> list:
    """
    session.csv's rows as {field: cell}, in the order found, headers left out.

    A row is told from the header by its uptime cell, which is digits -- never by its session cell, since
    a label may be 'session', which the header's session cell reads too.

    Args:
        clock - the session.csv payloads (_wrapped_streams()).

    Returns:
        [{field: cell}] of every boot's rows.
    """
    rows = []
    for payload in clock:
        cells = payload.split(';')
        if cells[0].isdigit() and len(cells) > _INDEX_FIELDS.index('source'):
            rows.append(dict(zip(_INDEX_FIELDS, cells)))
    return rows


def _utc(row: dict) -> datetime.datetime | None:
    """The instant a session.csv row dates; None for a clock never set (an empty utc, or one before 2001)."""
    if not row['utc'] or row['utc'] < _CLOCK_SET:
        return None
    try:
        return datetime.datetime.strptime(row['utc'], '%Y-%m-%dT%H:%M:%SZ').replace(tzinfo=datetime.timezone.utc)
    except ValueError:
        return None


def _clock_times(rows: list, span: tuple) -> list:
    """
    One boot's session.csv rows on the session's unwrapped timeline.

    A row's uptime is raw ticks_us, so it is placed like a recovered row: at the one count of wraps that
    puts it inside the boot's span, give or take _WRAP_SLACK_US. Its 'boot' row went out as the Recorder
    started the streams, so it lies at the span's start, and the 'anchor' row a minute after that; a set
    can come any time. In a boot longer than a wrap that still leaves a row two places, and the DATED rows
    are then placed together: each one's uptime less its utc is the same instant -- the boot's start on
    the wall clock -- give or take the few seconds a later set corrects, never half a wrap. The one choice
    of wraps under which every dated row agrees places them all; with none, or more than one, the rows
    the span did not place stay unplaced.

    Args:
        rows - the boot's rows (_index_rows()).
        span - (first, last), the boot's unwrapped uptime on its streams.

    Returns:
        The unwrapped uptime of each row, in order; None where no single one fits.
    """
    first, last = span
    candidates = []
    for row in rows:
        recorded = int(row['uptime'])
        low = first - _WRAP_SLACK_US
        high = {'boot': first, 'anchor': first + _ANCHOR_US}.get(row['source'], last) + _WRAP_SLACK_US
        candidates.append([recorded + count * _WRAP_US for count in range(-((recorded - low) // _WRAP_US),
                                                                           (high - recorded) // _WRAP_US + 1)])
    times = [fits[0] if len(fits) == 1 else None for fits in candidates]
    dated = {index: _utc(row).timestamp() * 1e6 for index, row in enumerate(rows) if _utc(row) and candidates[index]}
    choices = []  # every choice of wraps under which all dated rows agree on the boot's start
    for time in candidates[min(dated)] if dated else []:
        start = time - dated[min(dated)]
        agreeing = {index: [fit for fit in candidates[index] if abs(fit - instant - start) < _WRAP_US / 2]
                    for index, instant in dated.items()}
        if all(len(fits) == 1 for fits in agreeing.values()):
            choices.append({index: fits[0] for index, fits in agreeing.items()})
    if len(choices) == 1:
        for index, time in choices[0].items():
            times[index] = time
    return times


def _neighbours(rows: list, row: dict) -> str:
    """
    Where a boot that was never dated sits: boot ids only grow on a board, so between its dated neighbours.

    Args:
        rows - every boot's rows (_index_rows()).
        row - a row of the boot.

    Returns:
        A clause naming the nearest dated boots of the same board before and after it; '' when there is none.
    """
    if not row['boot'].isdigit():
        return ''
    boot = int(row['boot'])
    dated = [(int(other['boot']), _utc(other)) for other in rows
             if other['board'] == row['board'] and other['boot'].isdigit() and _utc(other) is not None]
    before = max((entry for entry in dated if entry[0] < boot), default=None)
    after = min((entry for entry in dated if entry[0] > boot), default=None)
    sides = ['%s boot %d (%s)' % (word, entry[0], entry[1].strftime('%Y-%m-%dT%H:%M:%SZ'))
             for word, entry in (('after', before), ('before', after)) if entry is not None]
    return '; it ran %s' % ' and '.join(sides) if sides else ''


def _report_clock(clock: list, session: str, span: tuple | None, zero: int) -> None:
    """
    Print how session.csv dates the session's boots, and the ignition's UTC.

    Every boot is listed: a 'boot' row, a row per time set, an 'anchor' row. Only a row with a utc dates its
    boot; a boot none dates is named by its boot id, board, firmware and config instead. The ignition's UTC
    comes from the nearest dated row on the timeline, the latest one before it preferred, since a later set
    corrects an earlier one (doc/specs/recorder-wire.md): utc + (uptime - its uptime).

    Args:
        clock - the session.csv payloads (_wrapped_streams()).
        session - the session prefix.
        span - (first, last) unwrapped uptime of the flight's boot on its streams; None when the session
            holds several boots, which its rows cannot be told apart in time for.
        zero - the ignition's unwrapped uptime.

    Returns:
        None.
    """
    every = _index_rows(clock)
    boots = {}
    for row in every:
        if row['session'] == session:
            boots.setdefault(row['boot'], []).append(row)
    if not boots:
        print('  session.csv names no boot of this session: nothing dates it')
        return
    for boot, rows in boots.items():
        identity = 'board %s, firmware %s, config %s' % (rows[0]['board'] or '?', rows[0]['firmware'] or '?',
                                                       rows[0]['config_id'] or '?')
        dated = [row for row in rows if _utc(row) is not None]
        if not dated:
            print('  boot %s (%s): clock never set%s' % (boot or '?', identity, _neighbours(every, rows[0])))
        for row in dated:
            print('  boot %s (%s): dated by %s at uptime %s us (raw ticks): %s, local offset %s min'
                  % (boot or '?', identity, row['source'] or '?', row['uptime'], row['utc'], row['utc_offset'] or '?'))
    rows = next(iter(boots.values()))
    if span is None or len(boots) > 1 or not any(_utc(row) for row in rows):
        return
    placed = [(time, row) for time, row in zip(_clock_times(rows, span), rows) if time is not None and _utc(row)]
    if not placed:
        print('  ignition UTC unknown: no dated row has a single place on the timeline across the ticks_us wrap')
        return
    before = [entry for entry in placed if entry[0] <= zero]
    time, row = max(before, key=lambda entry: entry[0]) if before else min(placed, key=lambda entry: entry[0])
    instant = _utc(row) + datetime.timedelta(microseconds=zero - time)
    print('  ignition at %s.%03dZ UTC, from the %s row at uptime %.6f s'
          % (instant.strftime('%Y-%m-%dT%H:%M:%S'), instant.microsecond // 1000, row['source'] or '?', time / 1e6))


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('recordings', help='the pulled /userdata/recordings directory')
    parser.add_argument('--session', required=True, help='e.g. 000123 (a boot id) or 20000101_000006_898573')
    parser.add_argument('-o', '--out', required=True, help='the launch folder to write recorder/ and flight/ into')
    parser.add_argument('--before', type=float, default=30.0, help='seconds kept ahead of ignition')
    parser.add_argument('--accel', default='imu_lsm6dso32', help='the stream ignition is found on')
    parser.add_argument('--boot', type=int, help='the boot (0-based) to cut; default: the first with a boost')
    args = parser.parse_args()

    tally, clock, elsewhere = None, [], 0
    if _wrapped(args.recordings, args.session):
        found, tally, clock, elsewhere = _wrapped_streams(args.recordings, args.session)
    else:
        found = _streams(args.recordings, args.session)
    if args.accel not in found:
        sys.exit('no %s stream in session %s' % (args.accel, args.session))
    header, boots, wraps = found[args.accel]
    fields = header or _FIELDS[args.accel]
    boot, zero = None, None
    for index, rows in enumerate(boots):
        if args.boot is not None and index != args.boot:
            continue
        try:
            boot, zero = index, ignition(rows, fields)
            break
        except ValueError:
            continue
    if boot is None:
        sys.exit('no boost in %s boot(s) of session %s' % (len(boots), args.session))
    print('session %s: %d boot(s) on %s (%s); the flight is boot %d' % (
        args.session, len(boots), args.accel,
        ', '.join('%.0f..%.0f s' % (b[0][0] / 1e6, b[-1][0] / 1e6) for b in boots if b), boot))
    if tally is not None:
        _report_checks(tally, elsewhere)
    streams, unmatched = {}, []
    for stream, (header_of, boots_of, wraps_of) in found.items():
        if len(boots_of) == len(boots) and wraps_of == wraps:
            streams[stream] = (header_of, boots_of[boot])
        elif boots_of:
            # sparse: too few rows to show its own wraps or restarts, so each row is placed below at the one
            # time (recorded + k wraps) that falls inside the window -- or dropped when that is not unique
            pooled = [(uptime % _WRAP_US, line) for rows_of in boots_of for uptime, line in rows_of]
            streams[stream] = (header_of, pooled)
            unmatched.append(stream)
    start = zero - int(args.before * 1e6)
    boot_end = max(uptime for uptime, _line in boots[boot])
    end = max(uptime for stream, (_header, rows) in streams.items() if stream not in unmatched
              for uptime, _line in rows if uptime <= boot_end + int(60e6))
    for stream in unmatched:
        header_of, rows = streams[stream]
        placed, ambiguous = [], 0
        for recorded, line in rows:
            fits = [recorded + k * _WRAP_US for k in range(wraps + 1) if start <= recorded + k * _WRAP_US <= end]
            if len(fits) == 1:
                placed.append((fits[0], line))
            elif fits:
                ambiguous += 1
        streams[stream] = (header_of, placed)
        print('  NOTE: %s is sparse: %d row(s) placed in the window by their unique time, %d ambiguous dropped'
              % (stream, len(placed), ambiguous))
    print('ignition at uptime %.6f s, window %.1f s .. %+.3f s' % (zero / 1e6, -args.before, (end - zero) / 1e6))
    if tally is not None:
        _report_clock(clock, args.session, (boots[boot][0][0], boots[boot][-1][0]) if len(boots) == 1 else None, zero)
    for folder in ('recorder', 'flight'):
        os.makedirs(os.path.join(args.out, folder), exist_ok=True)

    for stream, (header, rows) in sorted(streams.items()):
        fields = header or _FIELDS.get(stream)
        kept = sorted((uptime, line) for uptime, line in rows if start <= uptime <= end)
        if not kept:
            continue
        with open(os.path.join(args.out, 'recorder', stream + '.csv'), 'w', **_CODEC) as handle:
            handle.writelines(line + '\n' for _uptime_us, line in kept)
        named = 0
        if fields:
            with open(os.path.join(args.out, 'flight', stream + '.csv'), 'w', newline='', **_CODEC) as handle:
                writer = csv.writer(handle, delimiter=';', lineterminator='\n')  # ';' like the device
                writer.writerow(('t_s',) + tuple(fields) + ('uptime_us',))
                for uptime, line in kept:
                    tokens = _payload(line).split(';')[1:]
                    if len(tokens) != len(fields):
                        continue
                    writer.writerow(['%.6f' % ((uptime - zero) / 1e6)] +
                                    [_clean(token, field) for token, field in zip(tokens, fields)] + [uptime])
                    named += 1
        print('  %-18s %6d original rows, %6d named, last at %+.3f s%s' % (
            stream, len(kept), named, (kept[-1][0] - zero) / 1e6, '' if fields else '  (no field names known)'))

    log_path = os.path.join(args.recordings, 'recorder.log')
    log = board_log(_ascii_lines(log_path) if tally is None else _lines(log_path), args.session)
    with open(os.path.join(args.out, 'recorder', 'board.log'), 'w', **_CODEC) as handle:
        handle.writelines(line + '\n' for line in log)
    print('  board.log          %6d lines' % len(log))


if __name__ == '__main__':
    main()
