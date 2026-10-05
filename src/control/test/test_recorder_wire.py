"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the recorder wire wrapper's host reference (tools/recorder_wire.py,
doc/specs/recorder-wire.md) at the LINE level: the reference vectors the board is held to, the raw-byte
CRC, and every kind of link damage, each judged on one line -- good, rejected or salvaged, never guessed.
The same matrix runs end to end through flight_telemetry (test_flight_telemetry.py) and recorder_flight
(test_tools.py). Stdlib only. Run by `make test`.
"""

import binascii
import os
import subprocess
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import recorder_wire  # noqa: E402

"""
The reference vectors as literals, (routing or None, payload, wire line): the same ones
src/glider/test/test_recorder.py holds the board's viper wrapper to.
"""
_VECTORS = (
    ('000123_imu_lsm6dso32.csv', '884029;0.98;0.05;0.01',
     '@000123_imu_lsm6dso32.csv@{cb34fe60};884029;0.98;0.05;0.01;<34c67ca2>\n'),
    ('000123_imu_lsm6dso32.csv', 'uptime;ax;ay;az',
     '@000123_imu_lsm6dso32.csv@{229ee3fb};uptime;ax;ay;az;<dd611c04>\n'),
    ('session.csv', '884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;',
     '@session.csv@{9b56a10c};884031;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;cc-auto;;'
     ';<64a423cc>\n'),
    ('a.csv', '4294967296;0', '@a.csv@{0c9ea9b2};4294967296;0;<f361564d>\n'),
    (None, '884030 sequencer :: stage -> boosting', '{fe52ab73};884030 sequencer :: stage -> boosting;<01a029b2>\n'),
    (None, '123456789', '{cbf43926};123456789;<33500bcc>\n'),
    (None, '99999999999 x :: y', '{12085fce};99999999999 x :: y;<a58147ce>\n'),
    (None, 'boot :: no digits here', '{7afd6024};boot :: no digits here;<85029fdb>\n'),
)
_IMU = '000123_imu_lsm6dso32.csv'
_BARO = '000123_baro_bmp280.csv'
_KNOWN = [_BARO, _IMU, recorder_wire.SESSION_INDEX]
_OWN = ('000123_',)  # the boot-id prefix under which an unlisted `<name>@` may still prove a stream


def _line(payload: str, routing: str | None = None) -> str:
    """A wire line without its newline."""
    return recorder_wire.wrap(payload, routing).rstrip('\n')


def _recover(line: str) -> tuple:
    """A damaged capture line through split() and recover(), as the readers do: ([(routing, payload)], lost)."""
    status, routing, body = recorder_wire.split(line)
    assert status == recorder_wire.BAD, (status, line)
    records, lost = recorder_wire.recover(routing, body, _KNOWN, _OWN)
    return [(found, payload) for found, payload, _record in records], lost


def test_reference_vectors():
    """Every vector, both ways: wrap() produces it, split() reads it back as GOOD."""
    assert binascii.crc32(b'123456789') == 0xCBF43926  # the polynomial: IEEE 802.3, initial value 0
    for routing, payload, wire in _VECTORS:
        assert recorder_wire.wrap(payload, routing) == wire, (routing, payload)
        assert recorder_wire.split(wire) == (recorder_wire.GOOD, routing, payload), wire
    # a header has U = 0, so it closes with plain ~OPEN
    opening, closing = recorder_wire.checks(_IMU, 'uptime;ax;ay;az')
    assert closing == ~opening & 0xFFFFFFFF


def test_the_crc_covers_the_raw_bytes():
    """
    OPEN is the CRC of the bytes as sent: UTF-8 for any non-ASCII text, read back with surrogateescape.

    A config-derived name such as a board or site can be non-ASCII. Read as latin-1 (the first version)
    its CRC was taken over the wrong bytes, so a correct line was rejected in every capture reader.
    """
    payload = '884031;123;000123;2026-10-03T14:21:07Z;-240;tästér;dev;3f2a91;cc-auto;;'
    wire = recorder_wire.wrap(payload, recorder_wire.SESSION_INDEX).encode('utf-8')
    assert wire.startswith(b'@session.csv@{%08x}' % binascii.crc32(b'session.csv@' + payload.encode('utf-8')))
    text = wire.decode('utf-8', 'surrogateescape')
    assert recorder_wire.split(text) == (recorder_wire.GOOD, recorder_wire.SESSION_INDEX, payload)
    # NEGATIVE: the same bytes decoded as latin-1 are not what the board sent
    assert recorder_wire.split(wire.decode('latin-1'))[0] == recorder_wire.BAD
    # a byte the link damaged survives as a surrogate: the line is BAD, and nothing raises
    damaged = wire.replace('tästér'.encode('utf-8'), b't\xffst\xe9r').decode('utf-8', 'surrogateescape')
    assert recorder_wire.split(damaged)[0] == recorder_wire.BAD
    assert _recover(damaged) == ([], 1)


def test_empty_payload_and_empty_routing():
    """The 22-byte wrapper alone is a valid line, and an empty routing is a routing: '' covers '@'."""
    assert len(recorder_wire.wrap('')) == 22 + 1
    assert recorder_wire.split(recorder_wire.wrap('')) == (recorder_wire.GOOD, None, '')
    assert recorder_wire.split(recorder_wire.wrap('', 'x.csv')) == (recorder_wire.GOOD, 'x.csv', '')
    assert recorder_wire.checks('', '7;1')[0] == binascii.crc32(b'@7;1')
    assert recorder_wire.split('@@' + _line('7;1', '')[2:]) == (recorder_wire.GOOD, '', '7;1')
    # NEGATIVE: a log line's CRC is not the empty routing's -- the two can never be confused
    assert recorder_wire.checks(None, '7;1') != recorder_wire.checks('', '7;1')
    assert recorder_wire.unwrap('{00000000};;<0000000>') is None  # 21 bytes: not a wrapper


def test_every_kind_of_damage_on_one_line():
    """
    The error matrix, one line each: what is proven is kept, what is not is rejected -- never guessed.

    Each shape is one the 2026-10-03 dumps showed, or one the spec names: a corrupted value, uptime or
    CLOSE; a lost tail or head; a corrupted routing (a junk file); a lost leading or second '@'; two lines
    merged into one; and a splice (a truncated record with the next one run on).
    """
    row = '884029;0.98;0.05;0.01'
    good = _line(row, _IMU)
    assert recorder_wire.split(good) == (recorder_wire.GOOD, _IMU, row)
    head = len(_IMU) + 2  # where the wrapper starts on a routed line

    # NEGATIVE: damage inside what the checks cover is rejected, never read as data
    assert _recover(good.replace(';0.98;', ';0.99;')) == ([], 1), 'corrupted value'
    assert _recover(good.replace('884029', '884028')) == ([], 1), 'corrupted uptime'
    assert _recover(good[:-9] + ('0' if good[-9] != '0' else '1') + good[-8:]) == ([], 1), 'corrupted CLOSE'
    assert _recover(good[:-6]) == ([], 1), 'lost tail'
    # cut inside the head, too little is left to look wrapped: a wrapped capture rejects it as unwrapped
    assert recorder_wire.split(good[:head + 5])[0] == recorder_wire.LEGACY, 'truncated at the head'
    assert _recover('@%s@%s' % (_IMU, good[head + 11:])) == ([], 1), 'lost head'
    assert recorder_wire.split(row) == (recorder_wire.LEGACY, None, row), 'an unwrapped line is not a wrapped one'

    # salvaged: the routing a damaged line lost or mangled is the known one its CRC proves
    assert _recover(good.replace('imu_lsm', 'imu_lXm')) == ([(_IMU, row)], 0), 'corrupted routing'
    assert _recover(good[1:]) == ([(_IMU, row)], 0), 'lost leading @'
    assert _recover(good[5:]) == ([(_IMU, row)], 0), 'lost leading @ and part of the name'
    assert _recover(good[:head - 1] + good[head:]) == ([(_IMU, row)], 0), 'lost second @'

    # two records on one line: each piece judged on its own
    baro = _line('884031;1.20;25.0;101300;1.20', _BARO)
    log = _line('884032 sequencer :: stage -> boosting')
    both = [(_IMU, row), (_BARO, '884031;1.20;25.0;101300;1.20')]
    assert _recover(good + baro) == (both, 0), 'merged at a >@ seam'
    assert _recover(good + log) == ([(_IMU, row), (None, '884032 sequencer :: stage -> boosting')], 0), 'merged at >{'
    broken = baro[:head + 1] + 'zz' + baro[head + 3:]  # the second record's OPEN is damaged: no head to find
    assert _recover(good + broken) == ([(_IMU, row)], 1), 'a seam still cuts when the next head is damaged'
    # ... and at a '>{' seam: a log record's damaged head is found by the seam alone
    assert _recover(good + '{zz' + log[3:]) == ([(_IMU, row)], 1), 'a >{ seam cuts with the log head damaged'
    assert _recover(good[:40] + baro) == ([both[1]], 1), 'a splice: the truncated record is the one lost'
    assert _recover(good[:40] + baro[1:]) == ([both[1]], 1), 'a splice that also lost the next leading @'

    # a header and a session.csv row are rows like any other
    assert recorder_wire.split(_line('uptime;ax;ay;az', _IMU))[0] == recorder_wire.GOOD
    index_row = '5000;123;000123;2026-10-03T14:21:07Z;-240;taster;dev;3f2a91;anchor;;'
    assert recorder_wire.split(_line(index_row, 'session.csv'))[0] == recorder_wire.GOOD
    assert _recover(_line(index_row, 'session.csv')[1:]) == ([('session.csv', index_row)], 0)


def test_a_leftover_name_proves_only_this_session():
    """
    A `<name>@` left on a line proves its routing only when it is a known one or carries this session's
    boot-id prefix: a sound CRC from another session must not file the row here.
    """
    attitude = '000123_attitude.csv'
    row = '1600000;100;5'
    stray = _line(row, attitude)[1:]  # a stream no good row named, its line stranded by a lost '@'
    assert _recover(stray) == ([(attitude, row)], 0), 'a stream of this boot-id session'
    status, routing, body = recorder_wire.split(stray)
    assert recorder_wire.recover(routing, body, _KNOWN, ()) == ([], 1), 'NEGATIVE: no prefix vouches for it'
    foreign = _line(row, '000122_imu_lsm6dso32.csv')
    assert _recover(foreign[1:]) == ([], 1), 'NEGATIVE: another session, found by its leftover name'
    assert _recover(_line('1;2', _IMU)[:30] + foreign) == ([], 2), 'NEGATIVE: another session, run on in a splice'
    assert recorder_wire.salvage(foreign[1:].replace('000122', '000123'), _KNOWN, _OWN) == (None, None)


def test_fit_places_a_time_only_when_it_is_unique():
    """recorded + k * 2**30 inside the window: exactly one, or None (none, or a window a wrap wide)."""
    period = recorder_wire.TICKS_PERIOD
    assert recorder_wire.fit(1000, 0, 2000) == 1000
    assert recorder_wire.fit(1000, -5_000_000, 2000) == 1000
    assert recorder_wire.fit(1000, period, period + 2000) == period + 1000  # after a wrap
    assert recorder_wire.fit(1000, period + 1000, period + 1000) == period + 1000  # inclusive bounds
    assert recorder_wire.fit(1000, 2 * period, 3 * period - 1) == 2 * period + 1000
    # NEGATIVE: nothing in the window, or two candidates in it
    assert recorder_wire.fit(1000, 2000, 3000) is None
    assert recorder_wire.fit(1000, 0, period + 1000) is None
    assert recorder_wire.fit(1000, 500, period + 999) == 1000  # just under a wrap wide: still one


def test_session_files_leave_out_another_label():
    """
    A session's own files out of a dump listing: every `<boot id>_*.csv` (junk names hold rows to salvage);
    under a label, not another session's files that share the prefix (an older label could hold '_') --
    and never a real stream for a junk name that happens to look like another session's.
    """
    listing = ['000123_imu_lsm6dso32.csv', '000123_health.csv', '000123_a1_health.csv', '000123_imu_ls',
               '0001234_imu_lsm6dso32.csv', 'recorder.log', 'session.csv']
    assert recorder_wire.session_files('000123', listing) == [
        '000123_imu_lsm6dso32.csv', '000123_health.csv', '000123_a1_health.csv']
    streams = ['health.csv', 'imu_lsm6dso32.csv', 'servo_yaw.csv', 'sequencer_events.csv']
    hitl = ['hitl_' + stream for stream in streams] + ['hitl_f15_' + stream for stream in streams]
    own = ['hitl_' + stream for stream in streams]
    assert recorder_wire.session_files('hitl', hitl + ['hitl-f15_health.csv']) == own
    assert recorder_wire.session_files('hitl_f15', hitl) == ['hitl_f15_' + stream for stream in streams]
    legacy = ['20000101_000006_898573_imu.csv', '20000101_000006_898573_a1_imu.csv']
    assert recorder_wire.session_files('20000101_000006_898573', legacy) == legacy  # digits: never a label

    """
    NEGATIVE: a junk name never drops a real stream. A lost `imu_` turns `tms-7d_imu_bno055.csv` into
    `tms-7d_bno055.csv`; with both IMUs' tails junk, `imu` looks like a session repeating two streams.
    """
    tms = ['tms-7d_%s.csv' % stream for stream in ('imu_bno055', 'imu_lsm6dso32', 'baro_bmp280', 'health',
                                                    'sequencer', 'power_ina226', 'flight')]
    assert recorder_wire.session_files('tms-7d', tms + ['tms-7d_bno055.csv']) == tms + ['tms-7d_bno055.csv']
    junk = tms + ['tms-7d_bno055.csv', 'tms-7d_lsm6dso32.csv']
    assert recorder_wire.session_files('tms-7d', junk) == junk
    mixed = own + ['hitl_f15_' + stream for stream in streams[:3]] + ['hitl_f15_attitude.csv']
    assert recorder_wire.session_files('hitl', mixed) == mixed, 'a stream this session lacks: no repeat, kept'
    servos = ['tms-7d_servo_%s.csv' % surface for surface in ('yaw', 'eleron_left', 'eleron_right')]
    tails = ['tms-7d_%s.csv' % surface for surface in ('yaw', 'eleron_left', 'eleron_right')]
    assert recorder_wire.session_files('tms-7d', tms + servos + tails) == tms + servos + tails, 'three servo tails'
    small = own + ['hitl_x_health.csv', 'hitl_x_servo_yaw.csv', 'hitl_x_sequencer_events.csv']
    assert recorder_wire.session_files('hitl', small) == small, 'three files are no session: kept, visibly'

    # the shell tools' pull list goes through the command line
    tool = os.path.join(_ROOT, 'tools', 'recorder_wire.py')
    result = subprocess.run([sys.executable, tool, 'files', 'hitl'], input='\n'.join(hitl) + '\n',
                            capture_output=True, text=True, check=True)
    assert result.stdout.split() == recorder_wire.session_files('hitl', hitl), result.stdout
    """
    ... as BYTES: the 2026-10-03 TMS-7D card held `20000101_000\xff` and `..._705050\xdf`, and a junk name
    of the session can hold such a byte, or a space. Read as text, the first one stopped every pull.
    """
    names = [b'20000101_000\xff', b'20000101_000256_705050\xdf', b'000123_imu_lsm6dso32.csv',
             b'000123_imu\xff_ls.csv', b'000123_imu l.csv', b'000124_health.csv', b'recorder.log']
    result = subprocess.run([sys.executable, tool, 'files', '000123'], input=b'\n'.join(names) + b'\n',
                            capture_output=True, check=True)
    assert result.stdout == b'000123_imu_lsm6dso32.csv\n000123_imu\xff_ls.csv\n000123_imu l.csv\n', result.stdout
    # NEGATIVE: a malformed call exits non-zero rather than printing an empty pull list
    bad = subprocess.run([sys.executable, tool, 'hitl'], input='', capture_output=True, text=True)
    assert bad.returncode != 0 and not bad.stdout


test_reference_vectors()
test_the_crc_covers_the_raw_bytes()
test_empty_payload_and_empty_routing()
test_every_kind_of_damage_on_one_line()
test_a_leftover_name_proves_only_this_session()
test_fit_places_a_time_only_when_it_is_unique()
test_session_files_leave_out_another_label()
print('ok: recorder wire -- reference vectors, raw-byte CRC, empty payload/routing, every damage kind '
      'on one line, leftover names, fit across the wrap, session files under a label (junk tails kept, '
      'a byte-safe pull list)')
