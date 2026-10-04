"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the flight telemetry parser (tools/flight_telemetry.py): demux a recorder
capture into streams + logs, against both an explicit fixture and the synthetic flight, and read a
capture carrying the integrity wrapper (doc/specs/recorder-wire.md) strictly -- good, salvaged and
rejected lines -- while an unwrapped capture reads as it always did. Stdlib only (no plotly). Run by
`make test`.
"""

import os
import sys
import zlib

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import flight_synth_capture  # noqa: E402
import flight_telemetry  # noqa: E402
import recorder_wire  # noqa: E402

# new session shape YYYYMMDD_HHMMSS_<rand>_<file> -- the parser strips it to the bare file name
FIXTURE = '\n'.join([
    '@20260609_101715_500_accel.csv@uptime;ax;ay;az',
    '@20260609_101715_500_accel.csv@1000000;0.0;0.0;1.0',
    '@20260609_101715_500_accel.csv@1010000;0.1;-0.2;1.0',
    '@20260609_101715_500_atgm336h.csv@uptime;lat;lon;speed_kn;course',
    '@20260609_101715_500_atgm336h.csv@1000000;48.1173;11.5167;0.0;0.0',
    '161221274 health :: probe: vitals ok (mem_free 31480912, temp 35)',
    '161300000 controller :: stage -> gliding',
])


def test_fixture():
    streams, logs = flight_telemetry.parse(FIXTURE)
    assert set(streams) == {'accel.csv', 'atgm336h.csv'}, list(streams)  # session prefix stripped

    accel = streams['accel.csv']
    assert accel.fields == ['ax', 'ay', 'az']  # header row consumed, not data
    # parse() NORMALISES all stamps to a flight-relative origin (the earliest stamp seen -> t=0),
    # so the first accel row reads 0 us, and the 10 ms gap to the second row is preserved.
    assert len(accel.rows) == 2 and accel.rows[0][0] == 0 and accel.rows[1][0] == 10000
    times, az = accel.column('az')
    assert az == [1.0, 1.0] and abs(times[0] - 0.0) < 1e-9 and abs(times[1] - 0.01) < 1e-9  # us -> s

    assert streams['atgm336h.csv'].column('lat')[1][0] == 48.1173  # GNSS numeric parse
    """
    A non-numeric DATA cell becomes nan, not the raw string (findings §26.32): the string used to flow
    downstream into arithmetic like `row[0] / 1e6` and raise TypeError far from the cause. nan keeps the
    row parseable and poisons only what actually touches that cell. A BLANK cell still stays '' -- that
    is the "no sample here" marker column() skips.
    """
    junk = flight_telemetry.parse('@20260609_101715_x.csv@1;notanumber')[0]['x.csv'].rows[0][1]
    assert junk != junk, junk  # nan is the only value not equal to itself
    assert flight_telemetry.parse('@20260609_101715_x.csv@1;')[0]['x.csv'].rows[0][1] == ''  # blank preserved
    # a non-numeric UPTIME drops the whole row (else column() would divide a str by 1e6 -> TypeError)
    gated = flight_telemetry.parse('@20260609_101715_x.csv@uptime;v\n'
                                   '@20260609_101715_x.csv@badtime;5\n'
                                   '@20260609_101715_x.csv@2000000;7')[0]['x.csv']
    assert len(gated.rows) == 1 and gated.rows[0][0] == 0  # bad-uptime row skipped; survivor is the origin

    # logs ride the same flight-relative origin (min stamp = the first accel row at 1000000 us)
    assert logs[0] == (161221274 - 1000000, FIXTURE.splitlines()[5])
    assert any('stage -> gliding' in line for _ts, line in logs)


def test_synthetic_flight():
    streams, logs = flight_telemetry.parse(flight_synth_capture.generate())
    assert {'accel_adxl375.csv', 'baro_icp10111.csv', 'imu_bno055.csv', 'gnss.csv', 'laser_agl.csv'} <= set(streams)
    assert streams['imu_bno055.csv'].fields == ['heading', 'roll', 'pitch']  # real BNO055 field names
    _times, az = streams['accel_adxl375.csv'].column('az')
    assert max(az) > 5.0  # the boost spike is present
    _times, elevation = streams['baro_icp10111.csv'].column('elevation')
    assert max(elevation) > 100.0 and min(elevation) <= 0.0  # climb to apogee, back to ground
    assert len(streams['gnss.csv'].rows) < len(streams['accel_adxl375.csv'].rows)  # GNSS slower than accel
    assert any('stage -> gliding' in line for _ts, line in logs)


# The wire vectors tools/recorder_wire.py produces, as literals: the board's implementation is held to them.
_VECTORS = [
    '@000123_imu_lsm6dso32.csv@{cb34fe60};884029;0.98;0.05;0.01;<34c67ca2>',
    '{fe52ab73};884030 sequencer :: stage -> boosting;<01a029b2>',
    '@000123_imu_lsm6dso32.csv@{229ee3fb};uptime;ax;ay;az;<dd611c04>',
    '@session.csv@{9b56a10c};884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;;<64a423cc>',
]

# one boot under the boot-id prefix, as (routing or None for a log line, payload)
_BOOT = [
    ('000123_accel.csv', 'uptime;ax;ay;az'),
    ('000123_accel.csv', '1000000;0.0;0.0;1.0'),
    ('000123_accel.csv', '1010000;0.1;-0.2;1.0'),
    ('000123_accel.csv', '1020000;0.2;-0.1;1.0'),
    ('000123_atgm336h.csv', 'uptime;lat;lon;speed_kn;course'),
    ('000123_atgm336h.csv', '1000000;48.1173;11.5167;0.0;0.0'),
    (None, '161221274 health :: probe: vitals ok (mem_free 31480912, temp 35)'),
    (None, '161300000 controller :: stage -> gliding'),
]


def _legacy_lines() -> list:
    """_BOOT as a capture from before the wrapper."""
    return ['@%s@%s' % (routing, payload) if routing else payload for routing, payload in _BOOT]


def _wire_lines() -> list:
    """_BOOT as current firmware puts it on the wire: every line wrapped."""
    return [recorder_wire.wrap(payload, routing).rstrip('\n') for routing, payload in _BOOT]


def _shape(result: tuple) -> tuple:
    """A parse result in comparable form (repr, so a nan cell compares equal to itself)."""
    streams, logs = result
    return {name: (stream.fields, repr(stream.rows)) for name, stream in streams.items()}, logs


def test_wire_vectors():
    """The reference vectors, and the parse of a capture made of them."""
    assert zlib.crc32(b'123456789') == 0xCBF43926  # the polynomial: zlib/binascii CRC-32, initial value 0
    assert recorder_wire.wrap('884029;0.98;0.05;0.01', '000123_imu_lsm6dso32.csv') == _VECTORS[0] + '\n'
    assert recorder_wire.wrap('884030 sequencer :: stage -> boosting') == _VECTORS[1] + '\n'
    assert recorder_wire.wrap('uptime;ax;ay;az', '000123_imu_lsm6dso32.csv') == _VECTORS[2] + '\n'
    session_row = '884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;'
    assert recorder_wire.wrap(session_row, 'session.csv') == _VECTORS[3] + '\n'

    streams, logs = flight_telemetry.parse('\n'.join(_VECTORS))
    # the boot-id prefix is stripped; the shared session index keeps its own name
    assert set(streams) == {'imu_lsm6dso32.csv', 'session.csv'}, sorted(streams)
    imu = streams['imu_lsm6dso32.csv']
    assert imu.fields == ['ax', 'ay', 'az'] and imu.rows == [[0, 0.98, 0.05, 0.01]], (imu.fields, imu.rows)
    assert logs == [(1, '884030 sequencer :: stage -> boosting')], logs  # the payload, not the wire line
    assert streams['session.csv'].rows[0][:3] == [2, 123.0, 123.0]  # its uptime on the same clock as the rest
    assert flight_telemetry.line_counts() == {'good': 4, 'salvaged': 0, 'rejected': 0, 'legacy': 0}


def test_wrapped_capture_reads_like_its_legacy_twin():
    """
    Wrapping changes nothing a tool sees: the same streams, rows and logs as the unwrapped capture.

    Also on the synthetic flight, so every stream shape (fins, sparse GNSS, logs) is covered.
    """
    legacy = _shape(flight_telemetry.parse('\n'.join(_legacy_lines())))
    assert flight_telemetry.line_counts()['legacy'] == len(_BOOT)
    wrapped = _shape(flight_telemetry.parse('\n'.join(_wire_lines())))
    assert flight_telemetry.line_counts() == {'good': len(_BOOT), 'salvaged': 0, 'rejected': 0, 'legacy': 0}
    assert wrapped == legacy, (wrapped, legacy)
    assert set(wrapped[0]) == {'accel.csv', 'atgm336h.csv'}  # the boot-id prefix is stripped

    synthetic = flight_synth_capture.generate()
    twin = []
    for line in synthetic.splitlines():
        routing, _at, row = line[1:].partition('@') if line.startswith('@') else (None, '', line)
        twin.append(recorder_wire.wrap(row, routing).rstrip('\n'))
    assert _shape(flight_telemetry.parse('\n'.join(twin))) == _shape(flight_telemetry.parse(synthetic))


def _damaged(index: int, line: str) -> tuple:
    """Parse the wrapped boot with line `index` replaced by `line` (None drops it)."""
    lines = _wire_lines()
    if line is None:
        del lines[index]
    else:
        lines[index] = line
    streams, logs = flight_telemetry.parse('\n'.join(lines))
    return streams, logs, flight_telemetry.line_counts()


def test_damage_is_salvaged_or_rejected_never_guessed():
    """
    Each kind of link damage lands where its checks say: salvaged into the right stream, or rejected.

    The 2026-10-03 dumps lost ~4 % of rows to the link -- a corrupted value, a mangled file name, a lost
    leading '@', a truncated line, two records run together. Before the wrapper a damaged value was read
    as data; now nothing is taken that its CRC does not prove.
    """
    line = _wire_lines()[2]  # accel 1010000
    head = line.index('{')
    # NEGATIVE: damage the checks cover -> rejected, never read as data
    for damaged in (line.replace(';-0.2;', ';-0.3;'),        # a corrupted value, never read as -0.3
                    line.replace('1010000', '1010001'),        # a corrupted uptime
                    line[:-2] + ('0' if line[-2] != '0' else '1') + '>',  # a corrupted CLOSE alone
                    line[:-15],                                # a lost tail
                    line[:head] + line[head + 11:]):           # a lost head
        streams, _logs, counts = _damaged(2, damaged)
        assert counts == {'good': 7, 'salvaged': 0, 'rejected': 1, 'legacy': 0}, (damaged, counts)
        assert [row[0] for row in streams['accel.csv'].rows] == [0, 20000], damaged
    # salvaged into the routing its CRC proves, and placed between its neighbours
    for damaged in (line.replace('000123_accel.csv', '000123_acXel.csv'),  # a corrupted routing (junk file)
                    line[1:],                                               # a lost leading '@'
                    line[5:],                                               # ... and part of the name
                    line[:head - 1] + line[head:]):                         # a lost second '@'
        streams, logs, counts = _damaged(2, damaged)
        assert counts == {'good': 7, 'salvaged': 1, 'rejected': 0, 'legacy': 0}, (damaged, counts)
        assert [row[0] for row in streams['accel.csv'].rows] == [0, 10000, 20000], damaged
        assert 'acXel.csv' not in streams and len(logs) == 2, (sorted(streams), logs)
    # a header lost its routing: salvaged, it still names the fields
    streams, _logs, counts = _damaged(0, _wire_lines()[0][1:])
    assert counts['salvaged'] == 1 and streams['accel.csv'].fields == ['ax', 'ay', 'az'], counts
    # a splice: the accel row lost its tail and newline, the GNSS row ran on -> cut, the whole one kept
    gnss = _wire_lines()[5]
    lines = _wire_lines()
    lines[2], lines[5] = line[:40] + gnss, ''
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    counts = flight_telemetry.line_counts()
    assert counts == {'good': 6, 'salvaged': 1, 'rejected': 1, 'legacy': 0}, counts
    assert flight_telemetry.spliced_rows() == 1
    assert len(streams['accel.csv'].rows) == 2 and len(streams['atgm336h.csv'].rows) == 1
    # two whole lines merged into one (only the newline lost): both kept
    lines = _wire_lines()
    lines[2], lines[5] = line + gnss, ''
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts() == {'good': 6, 'salvaged': 2, 'rejected': 0, 'legacy': 0}
    assert len(streams['accel.csv'].rows) == 3 and len(streams['atgm336h.csv'].rows) == 1
    # NEGATIVE: a leftover name from ANOTHER session proves nothing here, however sound its CRC
    foreign = recorder_wire.wrap('1015000;9.9;9.9;9.9', '000122_accel.csv').rstrip('\n')
    streams, _logs, counts = _damaged(2, foreign[1:])
    assert counts == {'good': 7, 'salvaged': 0, 'rejected': 1, 'legacy': 0}, counts
    assert [row[0] for row in streams['accel.csv'].rows] == [0, 20000] and len(streams) == 2, sorted(streams)
    # ... while one of this boot-id session's own streams, which no good row named, is proven by it
    attitude = recorder_wire.wrap('1015000;100;5', '000123_attitude.csv').rstrip('\n')
    streams, _logs, counts = _damaged(2, attitude[1:])
    assert counts['salvaged'] == 1 and streams['attitude.csv'].rows == [[15000, 100.0, 5.0]], counts


def _block(routing: str, rows: list) -> list:
    """A Luckfox file as assemble_capture lays it into a capture: its rows wrapped, one after another."""
    return [recorder_wire.wrap(row, routing).rstrip('\n') for row in rows]


def _misfiled(name: str, row: str, routing: str) -> str:
    """A row the board sent to `routing`, as a capture holds it from the junk file `name` the link made of it."""
    return '@%s@%s' % (name, recorder_wire.wrap(row, routing).rstrip('\n')[len(routing) + 2:])


def test_recovered_rows_are_placed_by_time():
    """
    A recovered row joins its stream at its TIME, never where it sat in the capture.

    An assembled capture lays the Luckfox files one after another, so a junk-named file sorts anywhere
    -- `000123_accel_aa226.csv` sorts BEFORE `000123_accel_adxl375.csv` -- and a merged or spliced piece
    sits in another stream's file. Filed where it was found, a row ahead of its stream made _unwrap read
    a ticks wrap and push the whole stream 17.9 minutes late, and a piece at the end of the capture cut a
    5-minute power stream to 30 s (and its servo energy from 221 J to 22 J).
    """
    accel, power = '000123_accel_adxl375.csv', '000123_power_ina226.csv'
    accel_rows = ['%d;0.0;0.0;1.0;1' % uptime for uptime in range(1_000_000, 300_000_001, 1_000_000)]
    power_rows = ['%d;7400;500;3700;0' % uptime for uptime in range(1_000_500, 300_000_501, 1_000_000)]
    junk = accel_rows.pop(150)  # 151 s: its routing was mangled into a junk name
    merged = power_rows.pop(30)  # 31 s: the power row ran on after an accel row that lost its newline
    lines = [_misfiled('000123_accel_aa226.csv', junk, accel)]
    accel_block = _block(accel, accel_rows)
    accel_block[29] += recorder_wire.wrap(merged, power).rstrip('\n')  # the 30 s accel row, merged
    lines += accel_block + _block(power, power_rows)
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts() == {'good': 597, 'salvaged': 3, 'rejected': 0, 'legacy': 0}
    accel_times = [row[0] for row in streams['accel_adxl375.csv'].rows]
    power_times = [row[0] for row in streams['power_ina226.csv'].rows]
    assert accel_times == list(range(0, 299_000_001, 1_000_000)), accel_times[:3]  # in order, never shifted
    assert power_times == list(range(500, 299_000_501, 1_000_000)), power_times[-3:]  # the piece in its place
    assert sorted(streams) == ['accel_adxl375.csv', 'power_ina226.csv'], sorted(streams)


def test_salvage_across_a_ticks_wrap():
    """
    Past 2**30 us (17.9 min) of uptime the stamps wrap; a recovered row is unwrapped from its neighbours.

    TMS-7D sat 23.5 minutes on the pad, so the flight came after a wrap. A row recovered there must land
    at its unwrapped time; a row nothing around it can time -- a junk file, in a capture longer than a
    wrap -- has two candidate times, and is rejected rather than guessed.
    """
    period = recorder_wire.TICKS_PERIOD
    imu, baro = '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    times = range(30_000_000, 1_500_000_001, 5_000_000)  # 25 minutes of uptime
    imu_rows = ['%d;0.0;0.0;1.0;0;0;0;1' % (uptime % period) for uptime in times]
    baro_rows = ['%d;1.0;25.0;101300;1.0' % ((uptime + 2_000) % period) for uptime in times]
    after = baro_rows.pop(250)   # 1280 s, after the wrap: merged onto the imu row before it
    before = baro_rows.pop(30)   # 180 s: stranded in a junk file, with no neighbours to time it
    imu_block = _block(imu, imu_rows)
    imu_block[249] += recorder_wire.wrap(after, baro).rstrip('\n')
    lines = imu_block + _block(baro, baro_rows) + [_misfiled('000123_baro_bXp280.csv', before, baro)]
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts() == {'good': 587, 'salvaged': 2, 'rejected': 1, 'legacy': 0}
    stamps = [row[0] for row in streams['baro_bmp280.csv'].rows]
    expected = [uptime + 2_000 - 30_000_000 for uptime in times if uptime + 2_000 != 180_002_000]
    assert stamps == expected, [(a, b) for a, b in zip(stamps, expected) if a != b][:3]
    assert 1_250_002_000 in stamps and stamps[-1] - stamps[0] > period  # placed after the wrap, in order
    imu_stamps = [row[0] for row in streams['imu_lsm6dso32.csv'].rows]
    assert imu_stamps == [uptime - 30_000_000 for uptime in times]


def test_held_records_join_their_stream_or_are_rejected():
    """
    The three ways a held record is judged that no other case reaches: a row for a stream holding two
    boots is rejected (no single timeline), a log line recovered from a merged line joins the logs at its
    place, and a junk file's row -- with no neighbour to time it -- is placed by its own stream's span.
    """
    period = recorder_wire.TICKS_PERIOD
    # two boots appended into one file: the accel rows go back in time, so no recovered row can be placed
    lines = _wire_lines()
    lines[4:4] = _block('000123_accel.csv', ['500000;0.0;0.0;1.0', '510000;0.0;0.0;1.0'])
    lines.insert(6, recorder_wire.wrap('1015000;0.5;0.5;1.0', '000123_accel.csv').rstrip('\n')[1:])  # lost '@'
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts() == {'good': 10, 'salvaged': 0, 'rejected': 1, 'legacy': 0}
    assert len(streams['accel.csv'].rows) == 5, streams['accel.csv'].rows

    # a log line that ran on after a GNSS row: cut at the '>{' seam, and merged back among the logs
    lines = _wire_lines()
    lines[5] += recorder_wire.wrap('1005000 sequencer :: stage -> armed').rstrip('\n')
    streams, logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts() == {'good': 7, 'salvaged': 2, 'rejected': 0, 'legacy': 0}
    assert [text for _stamp, text in logs] == [
        '1005000 sequencer :: stage -> armed', _BOOT[6][1], _BOOT[7][1]], logs
    assert logs[0][0] == 5000 and len(streams['atgm336h.csv'].rows) == 1, logs

    """
    A junk file's row has no good line around it, so its stream's span stands in: here 5 minutes of baro
    after the wrap, inside a 25-minute capture whose own span would hold the row's time twice.
    """
    imu, baro = '000123_imu_lsm6dso32.csv', '000123_baro_bmp280.csv'
    imu_rows = ['%d;0.0;0.0;1.0;0;0;0;1' % (uptime % period) for uptime in range(30_000_000, 1_500_000_001, 5_000_000)]
    baro_times = range(1_200_000_000, 1_500_000_001, 1_000_000)
    baro_rows = ['%d;%d;25.0;101300;1.0' % (uptime % period, uptime // 1_000_000) for uptime in baro_times]
    junk = baro_rows.pop(100)  # 1300 s
    lines = _block(imu, imu_rows) + _block(baro, baro_rows) + [_misfiled('000123_baro_bXp280.csv', junk, baro)]
    streams, _logs = flight_telemetry.parse('\n'.join(lines))
    assert flight_telemetry.line_counts()['salvaged'] == 1, flight_telemetry.line_counts()
    altitudes = [row[1] for row in streams['baro_bmp280.csv'].rows]
    assert altitudes == [float(uptime // 1_000_000) for uptime in baro_times], altitudes[98:103]  # in its place


def test_the_session_index_reads_as_rows():
    """
    session.csv lists every boot: a 'boot' row, one per time set, an 'anchor' row -- the header again before
    the boot and the anchor rows, by design, so a repeated header must not flag the stream as two boots
    spliced into one file. utc and utc_offset are empty while the clock is unset, and such a row is still a
    row; only a dated one carries a utc. A label may even be 'session', the header's own third cell: the
    header is told by its uptime cell, never by a label.
    """
    header = 'uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon'
    rows = [header, '3000000;123;000123;;;tästér;dev;3f2a91;boot;;',
            '29000000;123;000123;2026-10-03T14:21:07Z;-240;tästér;dev;3f2a91;cc-auto;25.5;-80.3', header,
            '61000000;123;000123;2026-10-03T14:21:39Z;;tästér;dev;3f2a91;anchor;;']
    capture = '\n'.join(_wire_lines() + _block('session.csv', rows)).encode('utf-8')
    streams, _logs = flight_telemetry.parse(capture.decode('utf-8', 'surrogateescape'))  # as the tools read it
    assert flight_telemetry.line_counts()['good'] == len(_BOOT) + 5
    index = streams['session.csv']
    assert len(index.rows) == 3 and not index.spliced and flight_telemetry.spliced(streams) == []
    assert index.fields == header.split(';')[1:] and [row[0] for row in index.rows] == [2e6, 28e6, 60e6]
    assert index.column('utc')[0] == [28.0, 60.0], index.column('utc')  # only the dated rows have a utc
    assert [row[4] for row in index.rows] == ['', -240.0, ''], 'an unset offset is a blank cell'

    # a boot never dated, under the label 'session', with its boot and anchor rows (one lost its '@')
    lines = [recorder_wire.wrap(payload, routing.replace('000123_', 'session_')).rstrip('\n') if routing else
             recorder_wire.wrap(payload).rstrip('\n') for routing, payload in _BOOT]
    undated = [header, '3000000;124;session;;;taster;dev;3f2a91;boot;;', header,
               '61000000;124;session;;;taster;dev;3f2a91;anchor;;']
    block = _block('session.csv', undated)
    block[3] = block[3][1:]
    streams, _logs = flight_telemetry.parse('\n'.join(lines[:6] + block + lines[6:]))  # before the boot's logs
    assert flight_telemetry.line_counts() == {'good': len(_BOOT) + 3, 'salvaged': 1, 'rejected': 0, 'legacy': 0}
    assert sorted(streams) == ['accel.csv', 'atgm336h.csv', 'session.csv'], sorted(streams)
    index = streams['session.csv']
    assert [row[0] for row in index.rows] == [2e6, 60e6] and index.column('utc') == ([], []), index.rows
    assert not index.spliced

    # NEGATIVE: a second header in any other stream still flags it
    twice = _wire_lines() + [recorder_wire.wrap('uptime;ax;ay;az', '000123_accel.csv').rstrip('\n')]
    assert flight_telemetry.spliced(flight_telemetry.parse('\n'.join(twice))[0]) == ['accel.csv']


def test_load_reads_the_bytes_the_board_sent():
    """
    load() reads a capture as UTF-8 with surrogateescape: a byte the link damaged costs its own line, never
    the read -- the plain open() it replaced raised UnicodeDecodeError on the whole capture.
    """
    import tempfile
    lines = [line.encode('utf-8') for line in _wire_lines()]
    lines[2] = lines[2].replace(b';-0.2;', b';-\xff.2;')  # damaged on the wire: fails its CRC
    lines.append(recorder_wire.wrap('161400000 health :: t\u00e4st\u00e9r ok').rstrip('\n').encode('utf-8'))
    with tempfile.NamedTemporaryFile('wb', suffix='.txt', delete=False) as handle:
        handle.write(b'\n'.join(lines) + b'\n')
    streams, logs = flight_telemetry.load(handle.name)
    os.unlink(handle.name)
    assert flight_telemetry.line_counts() == {'good': len(_BOOT), 'salvaged': 0, 'rejected': 1, 'legacy': 0}
    assert len(streams['accel.csv'].rows) == 2 and logs[-1][1] == '161400000 health :: t\u00e4st\u00e9r ok'


def test_an_unwrapped_line_in_a_wrapped_capture_is_rejected():
    """
    In a wrapped capture a line without the wrapper is damage, not history -- but only there.

    A host tool that adds a line to a wrapped capture must wrap it (assemble_capture's stage marks,
    hitl_collect's build note), or it is dropped. An old capture is never judged by the new rule: a line
    that merely LOOKS wrapped does not switch it to strict reading, only a passing CRC does.
    """
    note = '0 capture :: build 2026.10.04 3f2a91 flash'
    _streams, logs = flight_telemetry.parse('\n'.join(_wire_lines() + [note]))
    assert flight_telemetry.line_counts()['rejected'] == 1 and all(note not in text for _ts, text in logs)
    _streams, logs = flight_telemetry.parse('\n'.join(_wire_lines() + [recorder_wire.wrap(note).rstrip('\n')]))
    assert flight_telemetry.line_counts()['good'] == len(_BOOT) + 1 and logs[-1][1] == note
    # NEGATIVE: an old capture with a wrapper-shaped line stays an old capture, read on trust
    streams, logs = flight_telemetry.parse('\n'.join(_legacy_lines() + ['{deadbeef};looks wrapped;<00000000>']))
    assert flight_telemetry.line_counts() == {'good': 0, 'salvaged': 0, 'rejected': 0, 'legacy': len(_BOOT) + 1}
    assert len(streams['accel.csv'].rows) == 3 and logs[-1][1] == '{deadbeef};looks wrapped;<00000000>'


test_fixture()
test_synthetic_flight()
test_wire_vectors()
test_wrapped_capture_reads_like_its_legacy_twin()
test_damage_is_salvaged_or_rejected_never_guessed()
test_recovered_rows_are_placed_by_time()
test_salvage_across_a_ticks_wrap()
test_held_records_join_their_stream_or_are_rejected()
test_the_session_index_reads_as_rows()
test_an_unwrapped_line_in_a_wrapped_capture_is_rejected()
test_load_reads_the_bytes_the_board_sent()
print('ok: flight telemetry parser — streams + logs from fixture and synthetic flight; wrapped captures: '
      'wire vectors, legacy twin, salvage/reject per damage kind, recovered rows placed by time (across a '
      'wrap too; two boots rejected, log lines merged back, a junk row by its stream span), the session '
      'index (boot/anchor rows, a clock never set, a label "session"), unwrapped lines in a wrapped '
      'capture, load() on damaged bytes')
