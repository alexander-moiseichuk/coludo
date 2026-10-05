#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Summarise a GNSS bench folder: the session files src/gnss_bench/main.py writes, `336.N` (ATGM336H) and
`neo6.N` (NEO-6M) per power-on N, and their moment files `336.N.x` / `neo6.N.x`.

    python3 tools/gnss_bench_report.py doc/benches/gnss-20261004/raw \\
        --label 1=none 5=small 6=patch 7=neo-kit 8=bt-580 -o summary.csv

Per session x module, from the checker's ~10 s status lines: the run time, the 3D-fix time (its `fix3d`
event), when the RMC turned valid, the status lines with a valid RMC of all of them, the most satellites in
view and used, the best C/N0, the best mean of the strongest four (`top-4`: the signal the receiver has to
work with, over every system it reports), the antenna text (the ATGM's OK/OPEN/SHORT; the NEO prints none
on the flight init), and the scatter of the fixed positions about their median (CEP50 and the farthest).
The position itself is never printed: a bench sits at someone's home.

The 3D-fix time is the checker's rule, a GGA with quality >= 1 and >= 4 satellites, which the ATGM can meet
before its RMC turns valid. A flight uses a position only from a valid RMC (`A`), so the usable fix is the
RMC bound: after the reading before the first `A`, by that first `A` -- `(after, by]`. The readings are the
status lines, ~10 s apart, and the `fix3d` record's situation line, the module's summary at its 3D fix.

Then the same satellite at nearly the same moment: both moment files are written at one instant, though
the last GSV burst in each can be up to ~10 s older than the other, so a GPS PRN that both modules track
there compares the two receivers on one signal with the sky all but held still. Each PRN counts at its
latest report in the file; NEO minus ATGM per PRN, and the mean. Labels name the antennas; `-o` writes the
table as a ';' CSV, the session comparison repeated on both module rows.

`--provenance <folder>` prints `<file>;<digest>` per session and moment file instead: an HMAC-SHA256 keyed
with a local pepper, so a published digest pins the file without being a test for a guessed position.
"""

import argparse
import csv
import hashlib
import hmac
import math
import os
import re
import statistics
import sys

_MODULES: dict = {'336': 'ATGM336H', 'neo6': 'NEO-6M'}  # file prefix -> module, in report order
_SESSION_FILE: re.Pattern = re.compile(r'^(336|neo6)\.(\d+)$')  # `336.N` / `neo6.N`: (file prefix, session N)
_PEPPER_FILE: str = '.pepper'  # the provenance key, in the bench folder: git-ignored, never printed
_PEPPER_BYTES: int = 32  # the key's length: 256 random bits, beyond any search
_METRES_PER_DEGREE: float = 111320.0  # a degree of latitude; a degree of longitude is this x cos(latitude)
_SCATTER_MIN: int = 3  # fewer fixed positions than this give no scatter: one point is never 0 m off
_GPS: range = range(1, 33)  # GPS PRNs: the ATGM's $GPGSV also carries QZSS (193..), the NEO's SBAS (33..64)
_COLUMNS: tuple = ('session', 'antenna', 'module', 'run_s', 'fix3d_s', 'rmc_valid_after_s', 'rmc_valid_by_s',
                   'valid', 'status', 'in_view_max', 'used_max', 'best_cn0', 'top4_max', 'antenna_text', 'cep50_m',
                   'max_m', 'common_prns', 'neo_minus_atgm_db')  # the CSV's columns, in order: rows() keys


def _summary(text: str) -> dict:
    """A status line's summary, `rmc=A;mode=3D;...`, as {key: value} (values verbatim, '' when empty)."""
    return dict(part.split('=', 1) for part in text.split(';') if '=' in part)


def _degrees(value: str) -> float:
    """
    An NMEA coordinate with its hemisphere letter, `4807.03800N` / `01131.00000E`, in signed degrees.

    Args:
        value - (d)ddmm.mmmmm followed by N/S/E/W, as the checker's summary holds it.

    Returns:
        Degrees, negative south and west.
    """
    number, hemisphere = value[:-1], value[-1]
    dot = number.index('.')
    degrees = float(number[:dot - 2]) + float(number[dot - 2:]) / 60
    return -degrees if hemisphere in 'SW' else degrees


def _scatter(positions: list) -> tuple:
    """
    How the fixed positions spread about their median.

    The medians are `median_high`, a value one of the fixes has: the centre is a real latitude and longitude,
    and at least half the fixes lie within the CEP50.

    Args:
        positions - (latitude, longitude) pairs in degrees.

    Returns:
        (CEP50, farthest) in metres from the median position; (None, None) with fewer than _SCATTER_MIN
        positions.
    """
    if len(positions) < _SCATTER_MIN:
        return None, None
    latitude = statistics.median_high(position[0] for position in positions)
    longitude = statistics.median_high(position[1] for position in positions)
    east = _METRES_PER_DEGREE * math.cos(math.radians(latitude))
    distances = [math.hypot((north - latitude) * _METRES_PER_DEGREE, (west - longitude) * east)
                 for north, west in positions]
    return statistics.median_high(distances), max(distances)


def _rmc_valid(readings: list) -> tuple:
    """
    When the RMC first turned valid, bounded by the readings either side.

    The readings are the status lines and the `fix3d` situation, which the checker writes with the same
    summary: a 3D fix that already says `A` tightens `by` (the NEO's warm fixes at 1-4 s, against a first
    status line at ~11 s), and one that says `V` can tighten `after`. Only `A` is valid: `V`, and an empty
    `rmc=` before any RMC arrived, are not.

    Args:
        readings - (seconds since the session start verbatim, summary) per reading, in time order.

    Returns:
        (after, by): `by` the first reading with `A`, `after` the one before it; after is None when the first
        reading already had `A` (valid since the session start), both None when none had.
    """
    for index, (seconds, summary) in enumerate(readings):
        if summary.get('rmc') == 'A':
            return (readings[index - 1][0] if index else None), seconds
    return None, None


def summarise(path: str) -> dict:
    """
    One module's session file read: what the module did over the session.

    Args:
        path - a `336.N` / `neo6.N` file; empty, or `start` alone, for a power bounce.

    Returns:
        {run_s, fix3d_s, rmc_valid_after_s, rmc_valid_by_s, valid, status, in_view_max, used_max, best_cn0,
        top4_max, antenna_text, cep50_m, max_m}: the times verbatim from the file (str), the rest numbers;
        None where the session has nothing to say (no status line, no fix, too few positions, an RMC never
        valid, no reading before the first valid one).
    """
    with open(path, encoding='ascii', errors='replace') as handle:
        lines = handle.read().splitlines()
    statuses = []  # (seconds, summary) per whole status line
    readings = []  # the RMC's readings, in file order -- the checker appends, so time order
    fix3d_s = None
    for index, line in enumerate(lines):
        fields = line.split(';', 3)
        if fields[0] == 'status' and len(fields) == 4:
            statuses.append((fields[2], _summary(fields[3])))
            readings.append(statuses[-1])
        elif fields[0] == 'fix3d' and len(fields) == 3 and fix3d_s is None:
            fix3d_s = fields[2]
            following = lines[index + 1] if index + 1 < len(lines) else ''
            if following.startswith('situation;'):
                readings.append((fix3d_s, _summary(following)))
    summaries = [summary for _seconds, summary in statuses]
    best = [[int(value) for value in summary.get('cn0', '').split(',') if value] for summary in summaries]
    antennas = []
    positions = []
    for summary in summaries:
        if summary.get('antenna') and summary['antenna'] not in antennas:
            antennas.append(summary['antenna'])
        if summary.get('quality', '0') not in ('', '0') and summary.get('lat') and summary.get('lon'):
            positions.append((_degrees(summary['lat']), _degrees(summary['lon'])))
    cep50, farthest = _scatter(positions)
    top4 = [sum(values[:4]) / 4 for values in best if len(values) >= 4]
    after, by = _rmc_valid(readings)
    return {
        'run_s': statuses[-1][0] if statuses else None,
        'fix3d_s': fix3d_s,
        'rmc_valid_after_s': after,
        'rmc_valid_by_s': by,
        'valid': sum(summary.get('rmc') == 'A' for summary in summaries),
        'status': len(summaries),
        'in_view_max': max((int(summary.get('in_view') or 0) for summary in summaries), default=None),
        'used_max': max((int(summary.get('sats') or 0) for summary in summaries), default=None),
        'best_cn0': max((value for values in best for value in values), default=None),
        'top4_max': max(top4, default=None),
        'antenna_text': '/'.join(antennas) or None,
        'cep50_m': cep50,
        'max_m': farthest,
    }


def _checksum_ok(sentence: str) -> bool:
    """Is `$body*hh` intact: the XOR of the body's bytes equals hh."""
    body, star, checksum = sentence[1:].partition('*')
    if not star:
        return False
    value = 0
    for byte in body.encode('ascii', errors='replace'):
        value ^= byte
    return checksum[:2].upper() == '%02X' % value


def tracked(path: str) -> dict:
    """
    The GPS satellites a moment file shows tracked, each at its latest report.

    The ATGM appends a signal id to its GSV (NMEA 4.1), so a sentence's satellites are its fields after the
    count, cut to whole groups of four (PRN, elevation, azimuth, C/N0). A sentence with a broken checksum
    is skipped; a PRN whose latest report carries no C/N0 is in view but not tracked.

    Args:
        path - a `336.N.x` / `neo6.N.x` moment file.

    Returns:
        {PRN: C/N0 dB-Hz} of the GPS PRNs tracked at the moment.
    """
    with open(path, encoding='ascii', errors='replace') as handle:
        lines = handle.read().splitlines()
    latest = {}
    for line in lines:
        start = line.find('$GPGSV,')
        if start < 0 or not _checksum_ok(line[start:]):
            continue
        fields = line[start:].split('*')[0].split(',')[4:]
        for index in range(0, len(fields) - len(fields) % 4, 4):
            if fields[index].isdigit() and int(fields[index]) in _GPS:
                latest[int(fields[index])] = fields[index + 3]
    return {prn: int(cn0) for prn, cn0 in latest.items() if cn0.isdigit()}


def same_satellites(atgm: str, neo: str) -> list:
    """
    The GPS satellites both modules track in their moment files: one instant, each module's latest GSV burst
    (the two up to ~10 s apart).

    Args:
        atgm - the ATGM336H's moment file.
        neo - the NEO-6M's moment file, written at the same instant.

    Returns:
        (PRN, ATGM C/N0, NEO C/N0) per common PRN, by PRN; empty when no satellite is tracked by both.
    """
    first = tracked(atgm)
    second = tracked(neo)
    return [(prn, first[prn], second[prn]) for prn in sorted(first.keys() & second.keys())]


def sessions(folder: str) -> dict:
    """
    The sessions a bench folder holds.

    Args:
        folder - the folder with the `336.N` / `neo6.N` files.

    Returns:
        {N: [file prefix, ...]} by session number, the prefixes in _MODULES order.

    Raises:
        SystemExit - the folder cannot be read, or holds no session file.
    """
    try:
        names = os.listdir(folder)
    except OSError as error:
        raise SystemExit('%s: %s' % (folder, error.strerror)) from None
    found = {}
    for name in names:
        match = _SESSION_FILE.match(name)
        if match:
            found.setdefault(int(match.group(2)), set()).add(match.group(1))
    if not found:
        raise SystemExit('%s: no session files (336.N / neo6.N)' % folder)
    return {session: [prefix for prefix in _MODULES if prefix in found[session]] for session in sorted(found)}


def labels(items: list) -> dict:
    """
    The antenna names, `N=name` per session.

    Args:
        items - the `--label` arguments.

    Returns:
        {N: name}.

    Raises:
        ValueError - an item is not `<session number>=<name>`.
    """
    named = {}
    for item in items:
        session, equals, name = item.partition('=')
        if not (equals and session.isdigit() and name):
            raise ValueError('label %r is not <session>=<antenna>' % item)
        named[int(session)] = name
    return named


def rows(folder: str, named: dict) -> list:
    """
    The report table: a row per session x module, with the session's same-satellite comparison.

    Args:
        folder - the bench folder.
        named - {session: antenna name} from labels().

    Returns:
        A dict per row, keyed by _COLUMNS plus `satellites`, the session's same_satellites(); None where a
        session or module has nothing to say.
    """
    table = []
    for session, prefixes in sessions(folder).items():
        moments = [os.path.join(folder, '%s.%d.x' % (prefix, session)) for prefix in _MODULES]
        common = same_satellites(*moments) if all(os.path.exists(moment) for moment in moments) else []
        difference = statistics.mean(neo - atgm for _prn, atgm, neo in common) if common else None
        for prefix in prefixes:
            row = {'session': session, 'antenna': named.get(session), 'module': _MODULES[prefix]}
            row.update(summarise(os.path.join(folder, '%s.%d' % (prefix, session))))
            row.update({'common_prns': len(common), 'neo_minus_atgm_db': difference, 'satellites': common})
            table.append(row)
    return table


def _pepper(folder: str) -> bytes:
    """
    The bench folder's provenance key, made on its first use.

    _PEPPER_BYTES from os.urandom in `<folder>/.pepper`, created owner-only (0600 where the OS has modes)
    and never printed. It stays local: the bench's raw folder is git-ignored, and `.pepper` is ignored by
    name too, should the raw files be committed one day. Lost, the published digests can no longer be
    checked, so it is backed up with the raw files.

    Args:
        folder - the bench folder.

    Returns:
        The key.

    Raises:
        SystemExit - the key cannot be made or read, or is not _PEPPER_BYTES long (a damaged key would give
        digests that never match, or a weak one).
    """
    path = os.path.join(folder, _PEPPER_FILE)
    try:
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            pass
        else:
            with os.fdopen(descriptor, 'wb') as handle:
                handle.write(os.urandom(_PEPPER_BYTES))
        with open(path, 'rb') as handle:
            pepper = handle.read()
    except OSError as error:
        raise SystemExit('%s: %s' % (path, error.strerror)) from None
    if len(pepper) != _PEPPER_BYTES:
        raise SystemExit('%s: not a %d-byte key' % (path, _PEPPER_BYTES))
    return pepper


def provenance(folder: str) -> list:
    """
    A keyed digest of every session and moment file, to show later that a file is the one the report read.

    HMAC-SHA256 under the folder's pepper, not a plain SHA-256: the files hold the bench's position, and a
    plain hash of a position, or of a record whose one unknown is a position, is a public test of a guess --
    the record format is public (src/gnss_bench/main.py), and a few square kilometres at the logged
    0.00001' (~2 cm) are ~1e10 candidates, seconds for one GPU. Without the pepper a digest says nothing;
    with it, its holder checks a file.

    Args:
        folder - the bench folder.

    Returns:
        (file name, HMAC hex) per file: by session, the modules in _MODULES order, each session file then
        its moment file when there is one.

    Raises:
        SystemExit - the folder holds no session file (sessions()), or the key is unusable (_pepper()).
    """
    names = []
    for session, prefixes in sessions(folder).items():
        for prefix in prefixes:
            names += [name for name in ('%s.%d' % (prefix, session), '%s.%d.x' % (prefix, session))
                      if os.path.exists(os.path.join(folder, name))]
    pepper = _pepper(folder)
    digests = []
    for name in names:
        with open(os.path.join(folder, name), 'rb') as handle:
            digests.append((name, hmac.new(pepper, handle.read(), hashlib.sha256).hexdigest()))
    return digests


def _text(value: object, form: str = '%s') -> str:
    """
    A printed table cell.

    Args:
        value - the cell's value, None for nothing to say.
        form - its % format.

    Returns:
        `value` in `form`; '-' for None.
    """
    return '-' if value is None else form % value


def _interval(after: str | None, by: str | None) -> str:
    """
    The RMC bound as a printed cell.

    Args:
        after - rmc_valid_after_s, None when valid from the first status line.
        by - rmc_valid_by_s, None when never valid.

    Returns:
        `(after,by]`, `<=by` from the first status line, or '-' never.
    """
    if by is None:
        return '-'
    return '<=%s' % by if after is None else '(%s,%s]' % (after, by)


def _print(table: list) -> None:
    """
    Print the table and the per-PRN same-satellite comparison of every session that has one.

    Args:
        table - rows().

    Returns:
        None.
    """
    width = max([len('antenna')] + [len(row['antenna'] or '') for row in table])
    form = '%%-7s %%-%ds %%-8s %%7s %%7s %%-13s %%7s %%7s %%4s %%4s %%5s %%-12s %%7s %%5s' % width
    print(form % ('session', 'antenna', 'module', 'run_s', 'fix3d_s', 'rmc_valid_s', 'valid', 'in_view', 'used',
                  'best', 'top4', 'antenna_text', 'cep50_m', 'max_m'))
    for row in table:
        print(form % (row['session'], _text(row['antenna']), row['module'], _text(row['run_s']),
                      _text(row['fix3d_s']), _interval(row['rmc_valid_after_s'], row['rmc_valid_by_s']),
                      '%d/%d' % (row['valid'], row['status']), _text(row['in_view_max']),
                      _text(row['used_max']), _text(row['best_cn0']), _text(row['top4_max'], '%.1f'),
                      _text(row['antenna_text']), _text(row['cep50_m'], '%.1f'), _text(row['max_m'], '%.1f')))
    print('\nsame satellite, the moment files (GSV bursts up to ~10 s apart): NEO-6M minus ATGM336H, then per GPS'
          ' PRN ATGM/NEO C/N0 (dB-Hz)')
    shown = set()
    for row in table:
        if row['satellites'] and row['session'] not in shown:
            shown.add(row['session'])
            print('  session %d: %+.1f dB over %d PRNs   %s' % (
                row['session'], row['neo_minus_atgm_db'], row['common_prns'],
                '  '.join('G%02d %d/%d' % satellite for satellite in row['satellites'])))


def _write(path: str, table: list) -> None:
    """
    Write the table as a ';' CSV, numbers as computed (the file's own tokens verbatim), '' for None.

    Args:
        path - the CSV to write.
        table - rows().

    Returns:
        None.
    """
    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle, delimiter=';', lineterminator='\n')  # ';' like the device
        writer.writerow(_COLUMNS)
        for row in table:
            cells = []
            for column in _COLUMNS:
                value = row[column]
                if isinstance(value, float):
                    value = '%.1f' % value  # computed: the C/N0 means and the metres, to one decimal
                cells.append('' if value is None else value)
            writer.writerow(cells)


def main() -> None:
    """Read the folder, print the report, and write the CSV when asked; or print the provenance digests."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('folder', nargs='?', help='the folder with the 336.N / neo6.N session files')
    source.add_argument('--provenance', metavar='FOLDER',
                        help='print <file>;<HMAC-SHA256> per session and moment file, keyed by FOLDER/.pepper')
    parser.add_argument('--label', nargs='+', default=[], help='N=antenna per session, e.g. 1=none 5=small')
    parser.add_argument('-o', '--out', help='also write the table as a ; CSV')
    arguments = parser.parse_args()
    if arguments.provenance is not None:
        if arguments.label or arguments.out:
            parser.error('--provenance takes no --label or -o')
        for name, digest in provenance(arguments.provenance):
            print('%s;%s' % (name, digest))
        return
    try:
        named = labels(arguments.label)
    except ValueError as error:
        parser.error(str(error))
    table = rows(arguments.folder, named)
    _print(table)
    if arguments.out:
        _write(arguments.out, table)
        print('\nwrote %s' % arguments.out, file=sys.stderr)


if __name__ == '__main__':
    main()
