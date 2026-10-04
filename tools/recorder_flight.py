"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Cut one flight out of a Luckfox recorder dump (`adb pull /userdata/recordings`) -- stdlib only.

The Luckfox demuxes the board's UART stream into one CSV per stream, `<session>_<stream>.csv`, plus a
shared recorder.log. A flight is a few seconds inside a session that can run for many minutes, and the
board's clock is often unset, so every session is named 2000-01-01: the flight is found by its BOOST, not
its date. This writes, for the window from `--before` s ahead of ignition to the session's last row:

    <out>/recorder/<stream>.csv   the ORIGINAL lines, verbatim, whose leading uptime is in the window
    <out>/recorder/board.log      the session's board log lines from recorder.log, verbatim
    <out>/flight/<stream>.csv     the same rows parsed and NAMED, t_s = seconds from ignition

    python3 tools/recorder_flight.py <dump>/recordings --session 20000101_000006_898573 \\
        -o launches/20261003/TMS-7C

Ignition is the rule src/logger/flight.py uses: the first |a| over 3 g held 0.3 s, walked back to where
|a| left 1.1 g -- on the LSM6DSO32 by default (the ADXL375 sits ~0.8 g off zero, too far for 1.1 g).

UART corruption is the norm on these dumps, and the tool assumes it. Only known streams are read (the
Luckfox spins corrupted names off into thousands of one-row files); a row is kept only where it continues
its file's own time sequence -- an uptime that goes backwards or jumps more than 5 s is corrupted; and a
row with the wrong field count stays in recorder/ (it is original) but not in flight/.

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
_STATUS = re.compile(r"'session': '([^']*)'")


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


def _streams(directory: str, session: str) -> dict:
    """stream -> (header fields or None, [(uptime_us, line)] in sequence) for the session's known streams."""
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
        streams[stream] = (header, _in_sequence(rows))
    return streams


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
    args = parser.parse_args()

    streams = _streams(args.recordings, args.session)
    if args.accel not in streams:
        sys.exit('no %s stream in session %s' % (args.accel, args.session))
    header, rows = streams[args.accel]
    zero = ignition(rows, header or _FIELDS[args.accel])
    start = zero - int(args.before * 1e6)
    end = max(uptime for _header, rows in streams.values() for uptime, _line in rows)
    print('session %s: ignition at uptime %.6f s, window %.1f s .. %+.3f s' % (
        args.session, zero / 1e6, -args.before, (end - zero) / 1e6))
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
                writer = csv.writer(handle, lineterminator='\n')
                writer.writerow(('t_s',) + tuple(fields) + ('uptime_us',))
                for uptime, line in kept:
                    values = _numbers(line, len(fields))
                    if values is None:
                        continue
                    writer.writerow(['%.6f' % ((uptime - zero) / 1e6)] +
                                    ['' if v is None else ('%g' % v) for v in values] + [uptime])
                    named += 1
        print('  %-18s %6d original rows, %6d named, last at %+.3f s%s' % (
            stream, len(kept), named, (kept[-1][0] - zero) / 1e6, '' if fields else '  (no field names known)'))

    log = board_log(os.path.join(args.recordings, 'recorder.log'), args.session)
    with open(os.path.join(args.out, 'recorder', 'board.log'), 'w') as handle:
        handle.writelines(line + '\n' for line in log)
    print('  board.log          %6d lines' % len(log))


if __name__ == '__main__':
    main()
