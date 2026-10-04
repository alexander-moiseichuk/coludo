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
"""

import argparse
import csv
import math
import os
import re
import sys

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
_WRAP_US: int = 1 << 30        # MicroPython ticks_us period (17.9 min): a drop of this size is a wrap
_WRAP_SLACK_US: int = 60_000_000  # ... give or take the rows a slow stream has before and after it
_CONFIRM_ROWS: int = 3
_STATUS = re.compile(r"'session': '([^']*)'")
_TEXT_FIELDS: tuple = ('stage', 'reason', 'event')  # recorded as words; every other field is a number


def _uptime(line: str):
    """The leading uptime (us) of a row, or None."""
    head = line.split(';', 1)[0]
    return int(head) if head.isdigit() else None


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


def _numbers(line: str, count: int):
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
        values = _numbers(line, len(fields))
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


def board_log(path: str, session: str) -> list:
    """
    The session's lines of recorder.log, verbatim: from the line after the previous session's last
    status line (a board logs its boot before the recorder's first status names the new session) to the
    line before the next session's first status line.
    """
    with open(path, 'rb') as handle:
        lines = [raw.decode('ascii', 'replace').rstrip('\r\n') for raw in handle]
    tags = [(index, found.group(1)) for index, line in enumerate(lines) for found in [_STATUS.search(line)]
            if found]
    mine = [index for index, tag in tags if tag == session]
    if not mine:
        return []
    before = [index for index, tag in tags if index < mine[0] and tag != session]
    after = [index for index, tag in tags if index > mine[-1] and tag != session]
    return lines[(before[-1] + 1 if before else 0):(after[0] if after else len(lines))]


def main() -> None:
    """Command line."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('recordings', help='the pulled /userdata/recordings directory')
    parser.add_argument('--session', required=True, help='e.g. 20000101_000006_898573')
    parser.add_argument('-o', '--out', required=True, help='the launch folder to write recorder/ and flight/ into')
    parser.add_argument('--before', type=float, default=30.0, help='seconds kept ahead of ignition')
    parser.add_argument('--accel', default='imu_lsm6dso32', help='the stream ignition is found on')
    parser.add_argument('--boot', type=int, help='the boot (0-based) to cut; default: the first with a boost')
    args = parser.parse_args()

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
    for folder in ('recorder', 'flight'):
        os.makedirs(os.path.join(args.out, folder), exist_ok=True)

    for stream, (header, rows) in sorted(streams.items()):
        fields = header or _FIELDS.get(stream)
        kept = sorted((uptime, line) for uptime, line in rows if start <= uptime <= end)
        if not kept:
            continue
        with open(os.path.join(args.out, 'recorder', stream + '.csv'), 'w') as handle:
            handle.writelines(line + '\n' for _uptime_us, line in kept)
        named = 0
        if fields:
            with open(os.path.join(args.out, 'flight', stream + '.csv'), 'w', newline='') as handle:
                writer = csv.writer(handle, delimiter=';', lineterminator='\n')  # ';' like the device
                writer.writerow(('t_s',) + tuple(fields) + ('uptime_us',))
                for uptime, line in kept:
                    tokens = line.split(';')[1:]
                    if len(tokens) != len(fields):
                        continue
                    writer.writerow(['%.6f' % ((uptime - zero) / 1e6)] +
                                    [_clean(token, field) for token, field in zip(tokens, fields)] + [uptime])
                    named += 1
        print('  %-18s %6d original rows, %6d named, last at %+.3f s%s' % (
            stream, len(kept), named, (kept[-1][0] - zero) / 1e6, '' if fields else '  (no field names known)'))

    log = board_log(os.path.join(args.recordings, 'recorder.log'), args.session)
    with open(os.path.join(args.out, 'recorder', 'board.log'), 'w') as handle:
        handle.writelines(line + '\n' for line in log)
    print('  board.log          %6d lines' % len(log))


if __name__ == '__main__':
    main()
