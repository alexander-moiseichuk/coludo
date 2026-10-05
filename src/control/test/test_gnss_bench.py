"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/gnss_bench_report.py, the reader of src/gnss_bench's session files, on
SYNTHETIC sessions only, in the checker's own record format: a fix, no fix, when the RMC turned valid (the
status lines and the 3D fix's situation), a power bounce, an empty file, OPEN then OK, a moment file with
QZSS, SBAS, a signal id that is also a PRN, a broken checksum and a satellite that dropped out; the labels,
a missing folder, the printed table and the ';' CSV; and the peppered provenance digests. Never the real
bench files: they hold the bench site's position, stay local and are better not used. Stdlib only; every
folder a test writes lives in one temporary directory, gone at exit. Run by `make test`.
"""

import base64
import contextlib
import csv
import hashlib
import hmac
import io
import os
import stat
import subprocess
import sys
import tempfile

_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import gnss_bench_report  # noqa: E402

_SCRATCH: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # every folder a test writes; gone at exit
_REPORT: str = os.path.join(_ROOT, 'tools', 'gnss_bench_report.py')
_STEP_M: float = 111320.0 / 60000  # 0.001 arc minute of latitude in metres, at the report's own scale
_HEAD: list = ['start;250', 'talking;870', 'tx;871;PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0',
               '847;$GNGGA,,,,,,0,00,25.5,,,,,,*64']  # a session's opening, as the checker saves it


def _folder() -> str:
    """A fresh folder inside _SCRATCH: a run leaves nothing behind in /tmp, whatever fails."""
    return tempfile.mkdtemp(dir=_SCRATCH.name)


def _write(folder: str, name: str, lines: list) -> str:
    """
    A session or moment file, the lines as given.

    Args:
        folder - where it goes.
        name - its file name, e.g. `336.5`.
        lines - its lines, no newlines.

    Returns:
        Its path.
    """
    path = os.path.join(folder, name)
    with open(path, 'w') as handle:
        handle.write(''.join(line + '\n' for line in lines))
    return path


def _status(seconds: float, rmc: str = 'V', quality: str = '0', satellites: str = '00', in_view: int = 0,
            cn0: str = '', antenna: str = '', latitude: str = '', longitude: str = '') -> str:
    """
    A status line as the checker writes it.

    Args:
        seconds - seconds since the session start.
        rmc - A (valid), V, or '' before any RMC arrived.
        quality - the GGA quality.
        satellites - the GGA satellites used, two digits.
        in_view - satellites in view.
        cn0 - the best four C/N0, strongest first, comma-separated.
        antenna - the ATGM's antenna text.
        latitude - ddmm.mmmmmH, '' with no fix.
        longitude - dddmm.mmmmmH, '' with no fix.

    Returns:
        The line.
    """
    return 'status;%d;%.1f;rmc=%s;mode=;quality=%s;sats=%s;hdop=1.0;in_view=%d;cn0=%s;antenna=%s;utc=;lat=%s;' \
           'lon=%s;alt_m=' % (seconds * 1000 + 250, seconds, rmc, quality, satellites, in_view, cn0, antenna,
                              latitude, longitude)


def _nmea(body: str) -> str:
    """An NMEA body as `$body*hh`, the XOR checksum correct."""
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '$%s*%02X' % (body, checksum)


def _fixed(seconds: float, minutes: str, antenna: str = 'OK') -> str:
    """
    A fixed status line at latitude 48 deg `minutes`' N, 11 deg 31' E (no bench's position).

    Args:
        seconds - seconds since the session start.
        minutes - the latitude's minutes, mm.mmmmm.
        antenna - the ATGM's antenna text.

    Returns:
        The line: valid RMC, 8 satellites used, 12 in view, best four 43,41,39,38.
    """
    return _status(seconds, 'A', '1', '08', 12, '43,41,39,38', antenna, '48%sN' % minutes, '01131.00000E')


def test_a_fix_is_summarised() -> None:
    """
    A session that fixes: the fix time, the RMC bound, valid/total, the maxima (the session ends below them),
    the top four and the scatter. A status line before any RMC (`rmc=` empty) is not a valid one, and the 3D
    fix's situation, still V, is a reading that narrows the bound.
    """
    folder = _folder()
    path = _write(folder, '336.5', _HEAD + [
        _status(11.5, rmc='', in_view=1, cn0='39'),
        _status(21.5, in_view=9, cn0='41,39,39,38', antenna='OK'),
        'fix3d;24797;24.5', 'situation;rmc=V;mode=none;quality=1;sats=05', '24700;$GNRMC,,V,,,,,,,,,,N,V*37',
        _fixed(31.5, '07.03800'), _fixed(41.5, '07.03900'), _fixed(51.5, '07.03700'), _fixed(61.5, '07.04000'),
        _fixed(71.5, '07.03800'),
        _status(81.5, 'A', '1', '06', 10, '42,40,38,37', 'OK', '4807.03800N', '01131.00000E')])  # fewer at the end
    summary = gnss_bench_report.summarise(path)
    assert summary['run_s'] == '81.5' and summary['fix3d_s'] == '24.5', summary
    # the checker's 3D fix (GGA) at 24.5 s, the RMC still V there: valid after 24.5 s (not 21.5), by 31.5 s
    assert (summary['rmc_valid_after_s'], summary['rmc_valid_by_s']) == ('24.5', '31.5'), summary
    assert (summary['valid'], summary['status']) == (6, 8), summary
    assert (summary['in_view_max'], summary['used_max'], summary['best_cn0']) == (12, 8, 43), summary
    assert summary['top4_max'] == 40.25, summary  # (43 + 41 + 39 + 38) / 4 beats (41 + 39 + 39 + 38) / 4
    assert summary['antenna_text'] == 'OK', summary
    # minutes 0, +1, -1, +2, 0, 0 thousandths about the median 07.03800: half within one step, the farthest two
    assert abs(summary['cep50_m'] - _STEP_M) < 1e-6 and abs(summary['max_m'] - 2 * _STEP_M) < 1e-6, summary


def _bound(folder: str, lines: list) -> tuple:
    """
    A session file's RMC bound and its count of valid status lines.

    Args:
        folder - where the file goes.
        lines - its lines.

    Returns:
        (rmc_valid_after_s, rmc_valid_by_s, valid).
    """
    summary = gnss_bench_report.summarise(_write(folder, '336.6', lines))
    return summary['rmc_valid_after_s'], summary['rmc_valid_by_s'], summary['valid']


def test_rmc_valid_is_bounded_by_status_lines() -> None:
    """
    The RMC bound: the first status line with `A` and the one before it; a later V changes nothing. Valid
    from the first status line has no lower bound; never valid (V, or `rmc=` empty) has none at all.
    """
    folder = _folder()
    lines = [_status(10.0, rmc=''), _status(20.0), _status(30.0, 'A'), _status(40.0), _status(50.0, 'A')]
    assert _bound(folder, lines) == ('20.0', '30.0', 2)
    assert _bound(folder, ['fix3d;3600;3.4', _status(10.0, 'A'), _status(20.0, 'A')]) == (None, '10.0', 2)
    assert _bound(folder, [_status(10.0, rmc=''), _status(20.0, rmc=''), _status(30.0)]) == (None, None, 0)


def test_rmc_valid_reads_the_fix3d_situation() -> None:
    """
    The 3D fix's situation line is a reading too: `A` there is the bound's `by` (a warm fix long before the
    first status line, or one between two status lines), `V` there its `after`. A fix3d without a situation
    line, or cut short, gives no reading; neither changes the valid count.
    """
    folder = _folder()
    warm = ['fix3d;1781;1.0', 'situation;rmc=A;mode=3D;quality=1', _status(11.1, 'A'), _status(21.1, 'A')]
    assert _bound(folder, warm) == (None, '1.0', 2)
    between = [_status(31.9), 'fix3d;37797;37.5', 'situation;rmc=A;mode=none;quality=1', _status(41.9, 'A')]
    assert _bound(folder, between) == ('31.9', '37.5', 1)
    late = [_status(21.1), 'fix3d;24000;24.0', 'situation;rmc=V;quality=1', _status(31.7, 'A')]
    assert _bound(folder, late) == ('24.0', '31.7', 1)
    bare = [_status(21.1), 'fix3d;24000;24.0', _status(31.7, 'A'), 'situation;rmc=A']  # not the fix3d's line
    assert _bound(folder, bare) == ('21.1', '31.7', 1)
    summary = gnss_bench_report.summarise(_write(folder, '336.6', [_status(10.0), 'fix3d;14000']))  # power cut
    assert (summary['fix3d_s'], summary['rmc_valid_by_s'], summary['run_s']) == (None, None, '10.0'), summary
    # the power cut fell between a whole fix3d line and its situation: the file ends on the fix3d
    summary = gnss_bench_report.summarise(_write(folder, '336.7', [_status(10.0), 'fix3d;14000;13.8']))
    assert (summary['fix3d_s'], summary['rmc_valid_by_s']) == ('13.8', None), summary


def test_no_fix_is_summarised() -> None:
    """A session that never fixes: no fix time, no scatter, no top four with fewer than four tracked."""
    folder = _folder()
    path = _write(folder, 'neo6.1', _HEAD + [_status(seconds, in_view=2, cn0='29,25') for seconds in (10.9, 21.0)])
    summary = gnss_bench_report.summarise(path)
    assert summary['run_s'] == '21.0' and summary['fix3d_s'] is None, summary
    assert (summary['valid'], summary['status'], summary['in_view_max']) == (0, 2, 2), summary
    assert summary['top4_max'] is None and summary['best_cn0'] == 29, summary
    assert summary['cep50_m'] is None and summary['max_m'] is None and summary['antenna_text'] is None, summary


def test_too_few_positions_give_no_scatter() -> None:
    """
    Two fixed status lines are not a scatter: one or two points would read as a tight 0 m. A position on a
    GGA without a fix (quality 0) is not a fixed position.
    """
    folder = _folder()
    unfixed = _status(30.0, 'V', '0', '00', 3, '30,28,25', 'OK', '4807.09900N', '01131.00000E')
    path = _write(folder, '336.7', [_fixed(10.0, '07.03800'), _fixed(20.0, '07.03900'), unfixed])
    summary = gnss_bench_report.summarise(path)
    assert summary['valid'] == 2 and summary['cep50_m'] is None and summary['max_m'] is None, summary


def test_scatter_of_an_even_count() -> None:
    """
    Four fixes: the centre and the CEP50 are fixes' own values (the upper median), not means of the middle two.
    """
    folder = _folder()
    path = _write(folder, '336.7', [_fixed(seconds, minutes) for seconds, minutes in (
        (10.0, '07.03800'), (20.0, '07.03900'), (30.0, '07.04000'), (40.0, '07.04300'))])
    summary = gnss_bench_report.summarise(path)
    # about 07.04000: 2, 1, 0 and 3 steps; the upper median of 0, 1, 2, 3 is 2 (a plain median: 1.5 about 07.0395)
    assert abs(summary['cep50_m'] - 2 * _STEP_M) < 1e-6 and abs(summary['max_m'] - 3 * _STEP_M) < 1e-6, summary


def test_an_empty_session_is_summarised() -> None:
    """A power bounce (`start` alone) and an empty file read as a session with nothing to say, not an error."""
    folder = _folder()
    for path in (_write(folder, '336.2', ['start;452']), _write(folder, 'neo6.3', [])):
        summary = gnss_bench_report.summarise(path)
        assert (summary['valid'], summary['status']) == (0, 0), summary
        assert all(summary[key] is None for key in summary if key not in ('valid', 'status')), summary


def test_antenna_text_open_then_ok() -> None:
    """The ATGM's antenna text in the order it changed, each once; a status with no text adds nothing."""
    folder = _folder()
    path = _write(folder, '336.6', [
        _status(10.0, antenna=''), _status(20.0, antenna='OPEN'), _status(30.0, antenna='OPEN'),
        _status(40.0, antenna='OK'), _status(50.0, antenna='OPEN')])
    assert gnss_bench_report.summarise(path)['antenna_text'] == 'OPEN/OK'
    path = _write(folder, '336.8', [_status(10.0, antenna='OPEN')])
    assert gnss_bench_report.summarise(path)['antenna_text'] == 'OPEN'


def _moments(folder: str, session: int) -> tuple:
    """
    An ATGM and a NEO moment file for `session`, built so each rule of tracked() shows.

    Args:
        folder - where they go.
        session - the session number.

    Returns:
        (ATGM path, NEO path).
    """
    atgm = _write(folder, '336.%d.x' % session, [
        'moment;992000;991.5', 'situation;rmc=A;mode=3D',
        '1000;' + _nmea('GNRMC,001846.500,A,4807.03800,N,01131.00000,E,0.00,0.00,051026,,,A,V'),
        # NMEA 4.1: a signal id after the satellites, 1 (GPS L1 C/A) -- a PRN too, never read as one; QZSS 194
        # is in $GPGSV but is not GPS
        '1100;' + _nmea('GPGSV,2,1,06,01,37,138,26,02,48,105,20,04,16,182,19,07,73,337,27,1'),
        '1101;' + _nmea('GPGSV,2,2,06,09,32,214,40,194,,,25,1'),
        '1102;' + _nmea('BDGSV,1,1,01,11,66,159,26,1'),  # BeiDou 11 is not GPS 11
        '1103;' + _nmea('GPGSV,1,1,01,22,09,281,99,1')[:-2] + '00',  # a broken checksum: never read
        # the latest report wins: 02 drops out, 04 strengthens
        '1200;' + _nmea('GPGSV,1,1,02,02,48,105,,04,16,182,23,1')])
    neo = _write(folder, 'neo6.%d.x' % session, [
        'moment;992000;991.5', 'situation;rmc=A;mode=3D',
        '1000;' + _nmea('GPGSV,2,1,06,01,37,138,33,02,48,105,22,04,16,181,25,07,73,337,26'),
        '1001;' + _nmea('GPGSV,2,2,06,09,32,213,39,46,29,248,40'),  # SBAS 46 is not GPS
        '1002;garbage\xb5b ?' + _nmea('GPGSV,1,1,01,11,66,159,30')])  # a sentence behind junk still counts
    return atgm, neo


def test_moment_files_compare_the_same_satellites() -> None:
    """
    The latest report per GPS PRN, tracked by both: QZSS, SBAS, BeiDou, a broken checksum and a dropped
    satellite all stay out.
    """
    atgm, neo = _moments(_folder(), 5)
    assert gnss_bench_report.tracked(atgm) == {1: 26, 4: 23, 7: 27, 9: 40}
    assert gnss_bench_report.tracked(neo) == {1: 33, 2: 22, 4: 25, 7: 26, 9: 39, 11: 30}
    assert gnss_bench_report.same_satellites(atgm, neo) == [(1, 26, 33), (4, 23, 25), (7, 27, 26), (9, 40, 39)]


def test_moment_files_without_a_common_satellite() -> None:
    """No satellite tracked by both, and a module with no moment file: no comparison, not an error."""
    folder = _folder()
    atgm = _write(folder, '336.1.x', ['moment;1;0.0', '1;' + _nmea('GPGSV,1,1,01,01,,,26,0')])
    neo = _write(folder, 'neo6.1.x', ['moment;1;0.0', '1;' + _nmea('GPGSV,1,1,01,02,,,30')])
    assert gnss_bench_report.same_satellites(atgm, neo) == []
    _write(folder, '336.1', [_status(10.0)])
    _write(folder, 'neo6.1', [_status(10.0)])
    _write(folder, '336.2', [_status(10.0)])
    _write(folder, '336.2.x', ['moment;1;0.0', '1;' + _nmea('GPGSV,1,1,01,01,,,26,0')])  # the NEO wrote no moment
    table = gnss_bench_report.rows(folder, {})
    assert [(row['session'], row['module']) for row in table] == [(1, 'ATGM336H'), (1, 'NEO-6M'), (2, 'ATGM336H')]
    assert all(row['common_prns'] == 0 and row['neo_minus_atgm_db'] is None for row in table), table


def test_rows_carry_labels_and_the_comparison() -> None:
    """A row per session x module, the label and the session's mean difference on both rows."""
    folder = _folder()
    _moments(folder, 5)
    _write(folder, '336.5', [_fixed(10.0, '07.03800')])
    _write(folder, 'neo6.5', [_fixed(10.0, '07.03800')])
    _write(folder, 'neo6.5.tmp', ['a moment caught mid-write'])  # not a session file
    _write(folder, 'README', ['not a session file'])
    table = gnss_bench_report.rows(folder, gnss_bench_report.labels(['5=small']))
    assert [row['module'] for row in table] == ['ATGM336H', 'NEO-6M'], table
    for row in table:
        assert row['antenna'] == 'small' and row['common_prns'] == 4, row
        assert row['neo_minus_atgm_db'] == (7 + 2 - 1 - 1) / 4, row


def test_labels() -> None:
    """`N=name` per session; anything else is refused, by labels() and by the command line."""
    assert gnss_bench_report.labels(['1=none', '5=small 15x5', '6=a=b']) == {1: 'none', 5: 'small 15x5', 6: 'a=b'}
    assert gnss_bench_report.labels([]) == {}
    for bad in ('small', '=small', '5=', 'x=small', '-1=small'):
        try:
            gnss_bench_report.labels([bad])
        except ValueError:
            continue
        raise AssertionError('label %r accepted' % bad)
    folder = _folder()
    _write(folder, '336.1', [_status(10.0)])
    result = subprocess.run([sys.executable, _REPORT, folder, '--label', 'small'], capture_output=True, text=True)
    assert result.returncode == 2 and 'not <session>=<antenna>' in result.stderr, result


def test_a_folder_without_sessions_is_refused() -> None:
    """
    No `336.N` / `neo6.N` file, or no folder at all: a clear exit, never an empty report that reads as
    'nothing fixed', and never a traceback.
    """
    folder = _folder()
    _write(folder, '336.1.x', ['moment;1;0.0'])
    _write(folder, 'neo6.one', [_status(10.0)])
    missing = os.path.join(folder, 'missing')
    for path, reason in ((folder, 'no session files'), (missing, missing + ': ')):
        try:
            gnss_bench_report.sessions(path)
        except SystemExit as error:
            assert reason in str(error), error
        else:
            raise AssertionError('%s was accepted' % path)
    result = subprocess.run([sys.executable, _REPORT, missing], capture_output=True, text=True)
    assert result.returncode == 1 and 'Traceback' not in result.stderr, result
    assert result.stderr.startswith(missing + ': '), result


def _main(arguments: list) -> str:
    """
    Run the report's main() in-process, as the command line would.

    Args:
        arguments - the command line after the program name.

    Returns:
        What it printed to stdout.
    """
    saved = sys.argv
    sys.argv = [_REPORT] + arguments
    try:
        with contextlib.redirect_stdout(io.StringIO()) as printed, contextlib.redirect_stderr(io.StringIO()):
            gnss_bench_report.main()
    finally:
        sys.argv = saved
    return printed.getvalue()


def test_csv_is_semicolon_and_verbatim() -> None:
    """
    `-o`: a ';' CSV, the file's own tokens verbatim, the computed ones to one decimal, '' for nothing. The
    report makes no pepper: only --provenance does.
    """
    folder = _folder()
    _moments(folder, 5)
    _write(folder, '336.5', _HEAD + ['fix3d;24797;24.5', _fixed(31.0, '07.03800'), _fixed(41.0, '07.03900'),
                                     _fixed(51.0, '07.04000')])
    _write(folder, 'neo6.5', ['start;250'])
    out = os.path.join(folder, 'summary.csv')
    printed = _main([folder, '--label', '5=small', '-o', out])
    assert 'session 5: +1.8 dB over 4 PRNs' in printed, printed
    assert not os.path.exists(os.path.join(folder, '.pepper')), 'the report made a pepper'
    with open(out) as handle:
        table = list(csv.DictReader(handle, delimiter=';'))
    assert list(table[0]) == list(gnss_bench_report._COLUMNS), table[0]
    atgm, neo = table
    assert (atgm['run_s'], atgm['fix3d_s'], atgm['valid'], atgm['status']) == ('51.0', '24.5', '3', '3'), atgm
    assert (atgm['rmc_valid_after_s'], atgm['rmc_valid_by_s']) == ('', '31.0'), atgm  # valid from the first
    assert (atgm['top4_max'], atgm['antenna_text'], atgm['neo_minus_atgm_db']) == ('40.2', 'OK', '1.8'), atgm
    assert atgm['cep50_m'] == atgm['max_m'] == '%.1f' % _STEP_M, atgm  # one step either side of the median
    assert (neo['module'], neo['run_s'], neo['fix3d_s'], neo['status']) == ('NEO-6M', '', '', '0'), neo
    assert (neo['rmc_valid_after_s'], neo['rmc_valid_by_s']) == ('', ''), neo


def test_printed_rmc_valid_column() -> None:
    """
    The printed table's rmc_valid_s cell: `(after,by]` between two readings, `<=by` valid from the first
    one, '-' never valid -- the order of after and by is the point.
    """
    folder = _folder()
    _write(folder, '336.1', [_status(10.0), _status(20.0, 'A')])
    _write(folder, 'neo6.1', ['fix3d;3500;3.5', 'situation;rmc=A', _status(10.0, 'A')])
    _write(folder, '336.2', [_status(10.0), _status(20.0)])
    cells = {}
    for line in _main([folder, '--label', '1=kit', '2=none']).splitlines():
        fields = line.split()
        if len(fields) == 14 and fields[0].isdigit():
            cells[fields[0], fields[2]] = fields[5]
    assert cells == {('1', 'ATGM336H'): '(10.0,20.0]', ('1', 'NEO-6M'): '<=3.5', ('2', 'ATGM336H'): '-'}, cells


def _bench(folder: str) -> None:
    """
    Two sessions of bench files in `folder`: their contents only need to differ, so no position in them.

    Args:
        folder - where they go.

    Returns:
        None.
    """
    _write(folder, '336.1', [_status(10.0)])
    _write(folder, '336.1.x', ['moment;1;0.0'])
    _write(folder, 'neo6.1', [_status(10.0, 'A')])
    _write(folder, '336.2', ['start;452'])
    _write(folder, 'neo6.2', [])
    _write(folder, 'neo6.2.tmp', ['a moment caught mid-write'])  # not a bench file: never digested


def _digest(pepper: bytes, folder: str, name: str) -> str:
    """The HMAC-SHA256 of a file under `pepper`, computed here, independently of the tool."""
    with open(os.path.join(folder, name), 'rb') as handle:
        return hmac.new(pepper, handle.read(), hashlib.sha256).hexdigest()


def test_provenance_is_a_peppered_hmac() -> None:
    """
    `--provenance`: every session and moment file in report order, HMAC-SHA256 under the folder's pepper --
    deterministic for one pepper, another pepper another digest, and never a plain SHA-256 of the file.
    """
    folder = _folder()
    _bench(folder)
    pepper = bytes(range(32))
    with open(os.path.join(folder, '.pepper'), 'wb') as handle:
        handle.write(pepper)
    digests = gnss_bench_report.provenance(folder)
    names = ['336.1', '336.1.x', 'neo6.1', '336.2', 'neo6.2']
    assert digests == [(name, _digest(pepper, folder, name)) for name in names], digests
    assert gnss_bench_report.provenance(folder) == digests  # deterministic
    with open(os.path.join(folder, '336.1'), 'rb') as handle:
        assert digests[0][1] != hashlib.sha256(handle.read()).hexdigest()
    printed = _main(['--provenance', folder])
    assert printed == ''.join('%s;%s\n' % digest for digest in digests), printed
    with open(os.path.join(folder, '.pepper'), 'wb') as handle:
        handle.write(bytes(range(1, 33)))
    other = gnss_bench_report.provenance(folder)
    assert all(first[1] != second[1] for first, second in zip(digests, other)), other


def test_one_changed_byte_changes_the_digest() -> None:
    """One byte changed in one file changes that file's digest, and no other."""
    folder = _folder()
    _bench(folder)
    with open(os.path.join(folder, '.pepper'), 'wb') as handle:
        handle.write(bytes(32))
    before = dict(gnss_bench_report.provenance(folder))
    path = os.path.join(folder, 'neo6.1')
    with open(path, 'rb') as handle:
        data = bytearray(handle.read())
    data[-2] ^= 0x01  # one bit of the last character before the newline
    with open(path, 'wb') as handle:
        handle.write(data)
    after = dict(gnss_bench_report.provenance(folder))
    assert [name for name in before if before[name] != after[name]] == ['neo6.1'], (before, after)


def test_pepper_is_made_once_owner_only_and_never_printed() -> None:
    """
    No pepper: the first use makes one, 32 random bytes, owner-only where the OS has modes, and keeps it --
    the second use gives the same digests. The pepper never shows in the output, in any common spelling.
    """
    folder = _folder()
    _bench(folder)
    pepper_path = os.path.join(folder, '.pepper')
    result = subprocess.run([sys.executable, _REPORT, '--provenance', folder], capture_output=True)
    assert result.returncode == 0 and result.stderr == b'', result
    with open(pepper_path, 'rb') as handle:
        pepper = handle.read()
    assert len(pepper) == 32 and pepper != bytes(32), pepper.hex()
    if os.name == 'posix':
        assert stat.S_IMODE(os.stat(pepper_path).st_mode) & 0o077 == 0, oct(os.stat(pepper_path).st_mode)
    names = ['336.1', '336.1.x', 'neo6.1', '336.2', 'neo6.2']
    lines = result.stdout.decode().splitlines()
    assert lines == ['%s;%s' % (name, _digest(pepper, folder, name)) for name in names], lines
    for spelling in (pepper, pepper.hex().encode(), pepper.hex().upper().encode(), base64.b64encode(pepper),
                     pepper.hex()[:16].encode()):
        assert spelling not in result.stdout, 'the pepper was printed'
    again = subprocess.run([sys.executable, _REPORT, '--provenance', folder], capture_output=True)
    assert again.stdout == result.stdout, again
    with open(pepper_path, 'rb') as handle:
        assert handle.read() == pepper  # kept, not remade


def test_peppers_are_random() -> None:
    """
    Two fresh folders holding the same files get different peppers, and so different digests: a constant or a
    derived key would make the HMAC a public hash again, and a position has too little entropy to survive one.
    """
    peppers = []
    for _ in range(2):
        folder = _folder()
        _bench(folder)
        digests = gnss_bench_report.provenance(folder)
        with open(os.path.join(folder, '.pepper'), 'rb') as handle:
            peppers.append((handle.read(), digests))
    assert peppers[0][0] != peppers[1][0]
    assert peppers[0][1][0][1] != peppers[1][1][0][1]  # the same 336.1, another key: another digest


def test_provenance_refuses() -> None:
    """
    A damaged pepper (empty, short, long) is refused without a digest and without echoing the key; a folder
    with no session file is refused and gets no pepper; --provenance takes no --label or -o.
    """
    folder = _folder()
    _bench(folder)
    pepper_path = os.path.join(folder, '.pepper')
    for damaged in (b'', bytes(range(31)), bytes(range(33))):
        with open(pepper_path, 'wb') as handle:
            handle.write(damaged)
        result = subprocess.run([sys.executable, _REPORT, '--provenance', folder], capture_output=True)
        assert result.returncode == 1 and result.stdout == b'', result
        assert b'not a 32-byte key' in result.stderr and b'Traceback' not in result.stderr, result
        assert not damaged or damaged.hex().encode() not in result.stderr, result
    empty = _folder()
    _write(empty, '336.1.x', ['moment;1;0.0'])
    try:
        gnss_bench_report.provenance(empty)
    except SystemExit as error:
        assert 'no session files' in str(error), error
    else:
        raise AssertionError('a folder without session files was accepted')
    assert not os.path.exists(os.path.join(empty, '.pepper')), 'a pepper made where nothing is digested'
    for extra in (['--label', '1=kit'], ['-o', os.path.join(folder, 'summary.csv')]):
        result = subprocess.run([sys.executable, _REPORT, '--provenance', folder] + extra, capture_output=True)
        assert result.returncode == 2 and b'takes no --label or -o' in result.stderr, result
    result = subprocess.run([sys.executable, _REPORT], capture_output=True)
    assert result.returncode == 2 and b'required' in result.stderr, result


test_a_fix_is_summarised()
test_rmc_valid_is_bounded_by_status_lines()
test_rmc_valid_reads_the_fix3d_situation()
test_no_fix_is_summarised()
test_too_few_positions_give_no_scatter()
test_scatter_of_an_even_count()
test_an_empty_session_is_summarised()
test_antenna_text_open_then_ok()
test_moment_files_compare_the_same_satellites()
test_moment_files_without_a_common_satellite()
test_rows_carry_labels_and_the_comparison()
test_labels()
test_a_folder_without_sessions_is_refused()
test_csv_is_semicolon_and_verbatim()
test_printed_rmc_valid_column()
test_provenance_is_a_peppered_hmac()
test_one_changed_byte_changes_the_digest()
test_pepper_is_made_once_owner_only_and_never_printed()
test_peppers_are_random()
test_provenance_refuses()
print('ok: gnss_bench_report -- a fix, the RMC bound (status lines and the fix3d situation), no fix, too few '
      'positions, an even count, a power bounce and an empty file, OPEN then OK, moment files (GPS only, a signal '
      'id that is a PRN, latest report, broken checksum, junk, no common satellite, a missing moment), rows with '
      'labels and the comparison, labels refused, a folder without sessions or missing, the ; CSV, the printed '
      'RMC column, the peppered provenance (HMAC, one byte, made once owner-only, never printed, refusals)')
