"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/ring_split.py, the host reader of the Rust firmware's flash ring
(doc/specs/avionics-port.md "Flash frame format", "SD mover", "SD and flash file layout"). The ring images are
built here from the spec's frame table, not with ring_split's codec, so each checks the other. One board-sized
image holds two boots across the ring's wrap: a failed program and its repeat, a frame torn by a power cut, a
damaged header passed by resynchronising, a lost frame, a damaged payload, a CONFIRM frame, and foreign,
torn-BOOT, misplaced and other-version slots. Its demuxed files are checked byte for byte, then read by the
unchanged dump tools as a Luckfox dump is (assemble_capture + flight_telemetry strict, recorder_flight's wrapped
reader). A `GET /ring` capture of the same lines must split into the same files. Negative: torn frames that only
look repeated, routings and session names that would leave the output directory, an erased or foreign ring,
an absent boot, an unproven capture, and --raw-capture over two boots. Stdlib only. Run by `make test`.
"""

import binascii
import contextlib
import io
import os
import struct
import subprocess
import sys
import tempfile

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import assemble_capture  # noqa: E402
import flight_telemetry  # noqa: E402
import recorder_flight  # noqa: E402
import recorder_wire  # noqa: E402
import ring_split  # noqa: E402

_TOOL: str = os.path.join(_ROOT, 'tools', 'ring_split.py')
_SCRATCH: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # every file a test writes; gone at exit
_BLOCK_BYTES: int = 64 << 10
_BLOCKS: int = 384
_DATA: int = 1
_BOOT: int = 2
_CONFIRM: int = 3
_IMU_FIELDS: str = 'uptime;ax;ay;az;gx;gy;gz;irq_runs'  # recorder_flight knows imu_lsm6dso32 by these


def _folder() -> str:
    """A fresh folder inside _SCRATCH."""
    return tempfile.mkdtemp(dir=_SCRATCH.name)


def _frame(kind: int, payload: bytes, block_sequence: int, boot_id: int, frame_sequence: int, uptime_us: int) -> bytes:
    """A frame from the spec's table: the 40-byte little-endian header with its two zlib CRC-32s, then the payload."""
    head = struct.pack(
        '<4sBBBBIIIIQI',
        b'\xf7CR1',
        1,
        kind,
        0,
        0,
        len(payload),
        block_sequence,
        boot_id,
        frame_sequence,
        uptime_us,
        binascii.crc32(payload),
    )
    return head + struct.pack('<I', binascii.crc32(head)) + payload


class _Board:
    """
    One boot writing a ring image as the flash ring does: a BOOT frame opens every block, frames sit at 8-byte
    alignment and are numbered from 0 across every kind, and a block lives in slot `block % 384`. Its lines
    carry rising uptimes, and `kept` lists what ring_split must keep, in order.
    """

    def __init__(self, image: bytearray, boot_id: int, session: str, uptime_us: int):
        self.image: bytearray = image
        self.boot_id: int = boot_id
        self.session: str = session
        self.identity: bytes = (
            'boot=%d;session=%s;board=taster;firmware=dev;config_id=3f2a91;reset=poweron;card=none' % (boot_id, session)
        ).encode()
        self.uptime_us: int = uptime_us  # the next line's uptime
        self.next_frame: int = 0
        self.block: int = 0  # the open block
        self.offset: int = 0  # where its next frame goes
        self.kept: list = []  # the lines ring_split must keep for this boot, in order

    def line(self, payload: str, routing: str | None = None) -> tuple:
        """(uptime, wire line) of a line stamped now; `payload` may hold '%d' for the uptime."""
        uptime_us = self.uptime_us
        self.uptime_us += 10_000
        text = payload % uptime_us if '%d' in payload else payload
        return uptime_us, recorder_wire.wrap(text, routing).encode()

    def row(self, values: str = '0.98;0.05;0.01;12;-3;5;1') -> tuple:
        """An IMU row."""
        return self.line('%d;' + values, '%s_imu_lsm6dso32.csv' % self.session)

    def index(self, source: str = 'boot', clock: str = ';') -> list:
        """session.csv's header and a row of `source`, as the writer sends them (the header first, every time)."""
        routing = recorder_wire.SESSION_INDEX
        header = self.line('uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon', routing)
        row = '%%d;%d;%s;%s;taster;dev;3f2a91;%s;;' % (self.boot_id, self.session, clock, source)
        return [header, self.line(row, routing)]

    def status(self) -> tuple:
        """The Recorder's status line. It names the session: recorder_flight finds its part of recorder.log so."""
        text = "%%d Recorder :: {'session': '%s', 'tlm': {'count': 0, 'max': 4, 'dropped': 0}}" % self.session
        return self.line(text)

    def open(self, block: int) -> None:
        """Open block `block` with its BOOT frame."""
        self.block, self.offset = block, 0
        self.program(_BOOT, self.identity, self.uptime_us)

    def program(
        self, kind: int, payload: bytes, uptime_us: int, cut: int | None = None, damage: int | None = None
    ) -> int:
        """
        Program one frame at the open block's next offset, numbered next.

        Args:
            kind - DATA, BOOT or CONFIRM.
            payload - its payload.
            uptime_us - its header's uptime.
            cut - keep only this many leading bytes (a torn program); None programs it all.
            damage - flip a bit of the byte at this index of the frame; None leaves it whole.

        Returns:
            The offset it was programmed at.
        """
        frame = bytearray(_frame(kind, payload, self.block, self.boot_id, self.next_frame, uptime_us))
        self.next_frame += 1
        if damage is not None:
            frame[damage] ^= 0x10
        at, length = self.offset, len(frame)
        if cut is not None:
            frame = frame[:cut]
        start = (self.block % _BLOCKS) * _BLOCK_BYTES + at
        self.image[start : start + len(frame)] = frame
        self.offset = at + -(-length // 8) * 8
        return at

    def data(self, entries: list, keep: bool = True, **damage: int) -> tuple:
        """
        A DATA frame of `entries` ((uptime, line) pairs), stamped with its first line's uptime.

        Args:
            entries - its lines.
            keep - whether ring_split keeps its lines whole (False for a damaged one: the test lists those).
            damage - program()'s cut and damage.

        Returns:
            (the frame's uptime, its payload, the offset it went to).
        """
        payload = b''.join(line for _uptime, line in entries)
        at = self.program(_DATA, payload, entries[0][0], **damage)
        if keep:
            self.kept.extend(line for _uptime, line in entries)
        return entries[0][0], payload, at

    def note(self, text: str) -> None:
        """Expect ring_split's wrapped note of a damaged frame next."""
        self.kept.append(recorder_wire.wrap(text).encode())


def _scenario() -> tuple:
    """
    The board-sized ring: boot 7 in blocks 382-383, boot 8 across the wrap in blocks 384-386 (slots 0-2), and
    foreign, torn-BOOT, other-version and misplaced slots.

    Returns:
        (image, the two _Boards).
    """
    image = bytearray(b'\xff' * (_BLOCKS * _BLOCK_BYTES))
    first = _Board(image, 7, '000007', 10_000_000)
    first.open(382)
    first.data(first.index())  # 1
    first.data([first.line(_IMU_FIELDS, '000007_imu_lsm6dso32.csv')])  # 2
    first.data([first.row(), first.row(), first.row(), first.status()])  # 3
    failed = [first.row(), first.row()]
    first.data(failed, keep=False, cut=70)  # 4: a failed program, 30 payload bytes in; the block closes
    first.open(383)  # 5
    first.data(failed)  # 6: the repeat, the same frame again
    first.data([first.row(), first.line('%d sequencer :: stage -> armed')])  # 7
    torn = [first.row(), first.line('%d controller :: arming'), first.row()]
    uptime, payload, _at = first.data(torn, keep=False, cut=40 + len(torn[0][1]) + len(torn[1][1]) + 20)  # 8
    first.note('%d ring :: frame 7/8: 2 salvaged, 1 lost' % uptime)
    first.kept.extend([torn[0][1], torn[1][1]])

    second = _Board(image, 8, '000008', 5_000_000)  # a new boot: its uptime starts again
    second.open(384)  # slot 0: past the wrap
    second.data(second.index())  # 1
    uptime, _payload, _at = second.data([second.line(_IMU_FIELDS, '000008_imu_lsm6dso32.csv'), second.status()])
    damaged = [second.row(), second.row()]
    _uptime, _payload, at = second.data(damaged, keep=False, damage=12)  # 3: its header CRC fails
    second.note('%d ring :: header 8/384+%d: 2 salvaged, 0 lost' % (uptime, at))
    second.kept.extend(line for _uptime, line in damaged)
    second.data([second.line('%d gnss :: no fix')])  # 4
    second.open(385)  # 5
    second.data([second.row(), second.row()])  # 6
    confirm = struct.pack('<6I', 384, 4096, 8, 3, 1, 0xDEADBEEF)
    second.program(_CONFIRM, confirm, second.uptime_us)  # 7: carries no lines
    second.data(second.index('cc-auto', '2026-10-03T14:21:07Z;-240') + [second.row()])  # 8
    second.open(386)  # 9
    second.next_frame += 1  # 10: a frame lost
    second.data([second.row(), second.row()])  # 11
    rows = [second.row(), second.row('7.5;0.05;0.01;12;-3;5;1'), second.row()]
    uptime, _payload, _at = second.data(rows, keep=False, damage=40 + len(rows[0][1]) + 5)  # 12: a damaged byte
    second.note('%d ring :: frame 8/12: 2 salvaged, 1 lost' % uptime)
    second.kept.extend([rows[0][1], rows[2][1]])

    image[100 * _BLOCK_BYTES : 101 * _BLOCK_BYTES] = bytes(range(256)) * 256  # MicroPython-era data
    image[101 * _BLOCK_BYTES : 101 * _BLOCK_BYTES + 20] = _frame(_BOOT, b'boot=1', 101, 1, 0, 0)[:20]  # a torn BOOT
    other = bytearray(_frame(_BOOT, b'boot=1', 102, 1, 0, 0))
    other[4] = 2  # layout version 2, its header CRC made good again: kept apart, never read
    other[36:40] = struct.pack('<I', binascii.crc32(bytes(other[:36])))
    image[102 * _BLOCK_BYTES : 102 * _BLOCK_BYTES + len(other)] = other
    misplaced = _frame(_BOOT, b'boot=1;session=000001', 5, 1, 0, 0)  # block 5 lives in slot 5, not 103
    image[103 * _BLOCK_BYTES : 103 * _BLOCK_BYTES + len(misplaced)] = misplaced
    return bytes(image), (first, second)


def _expected(boards: list) -> dict:
    """{path under outdir: bytes} the split must write for these boards, by the test's own reading of the layout."""
    files = {}
    for board in boards:
        for line in board.kept:
            routing, _at, filed = line[1:].partition(b'@')
            if line.startswith(b'@') and b'/' not in routing:
                name = routing.decode()
            else:
                name, filed = 'recorder.log', line
            path = os.path.join(board.session, name)
            files[path] = files.get(path, b'') + filed
            if name == recorder_wire.SESSION_INDEX:
                files[name] = files.get(name, b'') + filed
    return files


def _tree(folder: str) -> dict:
    """{path under folder: bytes} of every file in it."""
    found = {}
    for directory, _directories, names in os.walk(folder):
        for name in names:
            path = os.path.join(directory, name)
            with open(path, 'rb') as handle:
                found[os.path.relpath(path, folder)] = handle.read()
    return found


def _run(*arguments: str) -> subprocess.CompletedProcess:
    """The command line, as an operator runs it."""
    return subprocess.run([sys.executable, _TOOL, *arguments], capture_output=True, text=True)


def _write(data: bytes) -> str:
    """`data` in a fresh file; its path."""
    path = os.path.join(_folder(), 'input.bin')
    with open(path, 'wb') as handle:
        handle.write(data)
    return path


def _strict(path: str) -> dict:
    """flight_telemetry's verdicts on a capture file, which must be read strictly: nothing salvaged or worse."""
    flight_telemetry.load(path)
    counts = flight_telemetry.line_counts()
    assert counts['good'] > 0 and counts['salvaged'] == counts['rejected'] == counts['legacy'] == 0, counts
    return counts


def test_a_ring_splits_into_its_sessions():
    """Two boots across the wrap, every kind of damage: the files byte for byte, and the summary's counts."""
    image, boards = _scenario()
    out = _folder()
    result = _run(_write(image), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'ring: 384 slots: 5 block, 375 erased, 3 foreign, 1 other version',
        'boot 7 session 000007: blocks 2 (#382-#383), frames 9 (#0-#8), lines 11, salvaged 2, lost 1, '
        'torn frames 2, repeats 1, torn headers 0, missing frames 0',
        'boot 8 session 000008: blocks 3 (#384-#386), frames 11 (#0-#12), lines 12, salvaged 4, lost 1, '
        'torn frames 1, repeats 0, torn headers 1, missing frames 2',
        'wrote 7 files under %s' % out,
    ], result.stdout
    assert sorted(os.listdir(out)) == ['000007', '000008', 'session.csv']
    assert sorted(os.listdir(os.path.join(out, '000007'))) == [
        '000007_imu_lsm6dso32.csv',
        'recorder.log',
        'session.csv',
    ]
    found, expected = _tree(out), _expected(boards)
    assert sorted(found) == sorted(expected), sorted(found)
    for path, data in expected.items():
        assert found[path] == data, (path, found[path], data)
    # the repeat is filed once, never with the torn frame's salvage beside it
    imu = found[os.path.join('000007', '000007_imu_lsm6dso32.csv')].splitlines()
    assert len(imu) == len(set(imu)) == 1 + 3 + 2 + 1 + 1, imu
    # the global index holds both boots' rows, in block order
    sessions = [line.split(b';')[3] for line in found['session.csv'].splitlines() if b';uptime;' not in line]
    assert sessions == [b'000007', b'000008', b'000008'], sessions
    # a second run into the same directory gives the same files, not twice the rows
    assert _run(_write(image), out).returncode == 0 and _tree(out) == found


def test_the_sessions_are_a_luckfox_dump():
    """Each session directory reads in the unchanged dump tools as a wrapped Luckfox dump does: all good."""
    image, boards = _scenario()
    out = _folder()
    assert _run(_write(image), out).returncode == 0
    for board in boards:
        directory = os.path.join(out, board.session)
        capture = os.path.join(_folder(), 'capture.txt')
        with contextlib.redirect_stdout(io.StringIO()):
            assemble_capture.assemble(board.session, directory, capture)
        streams, _logs = flight_telemetry.load(capture)
        rows = sum(1 for line in board.kept if b'imu_lsm6dso32.csv@' in line) - 1  # less the header
        assert len(streams['imu_lsm6dso32.csv'].rows) == rows, streams
        _strict(capture)
        _found, tally, clock, elsewhere, log = recorder_flight._wrapped_streams(directory, board.session)
        assert all(rejected == 0 for _good, _salvaged, rejected in tally.values()), tally
        assert tally['imu_lsm6dso32'][0] == rows + 1 and elsewhere == 0, tally
        assert tally[recorder_flight._BOARD_LOG][0] == len(log) > 0, (tally, log)
        assert any(';boot;' in row for row in clock), clock


def test_one_boot_and_its_raw_capture():
    """--boot keeps one boot; --raw-capture writes its lines as a capture flight_telemetry reads strictly."""
    image, (_first, second) = _scenario()
    out, raw = _folder(), os.path.join(_folder(), 'raw.txt')
    result = _run(_write(image), out, '--boot', '8', '--raw-capture', raw)
    assert result.returncode == 0, result.stderr
    assert sorted(os.listdir(out)) == ['000008', 'session.csv']
    assert _tree(out) == _expected([second])
    with open(raw, 'rb') as handle:
        assert handle.read() == b''.join(second.kept)
    assert _strict(raw)['good'] == len(second.kept)


def test_a_ring_capture_splits_like_the_image():
    """A `GET /ring` capture of the same lines: detected by its first byte, split into the very same files."""
    image, boards = _scenario()
    capture = b''.join(line for board in boards for line in board.kept)
    from_image, from_capture = _folder(), _folder()
    assert _run(_write(image), from_image).returncode == 0
    result = _run(_write(capture), from_capture)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[1:3] == [
        'boot 7 session 000007: lines %d, unproven 0, lost 0' % len(boards[0].kept),
        'boot 8 session 000008: lines %d, unproven 0, lost 0' % len(boards[1].kept),
    ], result.stdout
    assert _tree(from_capture) == _tree(from_image)
    # a transfer cut short loses its unterminated line, and one that starts mid-boot is named by its routings
    cut = b''.join(boards[0].kept[2:])[:-10]
    result = _run(_write(cut), _folder())
    assert result.returncode == 0 and 'boot ? session 000007: ' in result.stdout, result.stdout
    assert result.stdout.endswith('lost 1\nwrote 2 files under %s\n' % result.args[3]), result.stdout
    # NEGATIVE: the same capture forced to be read as an image holds no block
    result = _run(_write(capture), _folder(), '--format', 'image')
    assert result.returncode == 1 and 'nothing valid found' in result.stderr, result.stderr
    # NEGATIVE: lines that never check out are no capture, whatever their shape
    junk = b'@000007_imu_lsm6dso32.csv@{00000000};1;2;<00000000>\nhello\n'
    result = _run(_write(junk), _folder())
    assert result.returncode == 1 and 'nothing valid found' in result.stderr, result.stderr


def test_a_frame_is_repeated_only_by_its_own_bytes():
    """
    A torn frame is dropped for its repeat only when the next DATA frame has its uptime, its length and its
    surviving bytes: an equal length or uptime alone is another frame, and the torn one is salvaged. A frame
    whose failed program had landed whole is repeated whole, and taken once; the same uptime and length with
    other bytes is another frame. A routing or session name that would leave the output directory is refused.
    """
    image = bytearray(b'\xff' * (4 * _BLOCK_BYTES))  # a short read: the first four slots
    board = _Board(image, 9, '000009', 1_000_000)
    board.identity = board.identity.replace(b'session=000009', b'session=../../escaped')  # the boot id names it
    board.open(0)
    evil = board.line('%d;1', '../evil.csv')  # checks out, but names a path
    board.data(board.index() + [evil])  # 1: filed whole into recorder.log
    first = [board.row(), board.row()]
    uptime, _payload, _at = board.data(first, keep=False, cut=40 + len(first[0][1]) + 5)  # 2
    board.open(1)  # 3
    longer = first + [board.row()]
    board.program(_DATA, b''.join(line for _uptime, line in longer), uptime)  # 4: its uptime, not its length
    board.note('%d ring :: frame 9/2: 1 salvaged, 1 lost' % uptime)
    board.kept.extend([first[0][1]] + [line for _uptime, line in longer])
    second = [board.row(), board.row()]
    uptime, payload, _at = board.data(second, keep=False, cut=40 + len(second[0][1]) + 5)  # 5
    board.open(2)  # 6
    other = payload.replace(b'0.98', b'0.99', 1)  # its uptime and length, other bytes
    board.program(_DATA, other, uptime)  # 7
    board.note('%d ring :: frame 9/5: 1 salvaged, 1 lost' % uptime)
    board.kept.extend([second[0][1]] + other.splitlines(keepends=True))
    landed = [board.row(), board.row()]
    uptime, payload, _at = board.data(landed)  # 8: whole, though its program reported a failure
    board.open(3)  # 9
    board.program(_DATA, payload, uptime)  # 10: the writer's repeat, byte for byte: dropped
    routing = '000009_imu_lsm6dso32.csv'
    same = [recorder_wire.wrap('%d;0.97;0.05;0.01;12;-3;5;1' % stamp, routing).encode() for stamp, _line in landed]
    assert len(b''.join(same)) == len(payload)
    board.program(_DATA, b''.join(same), uptime)  # 11: its uptime and length, other rows: kept
    board.kept.extend(same)
    out = _folder()
    result = _run(_write(bytes(image)), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[:2] == [
        'ring: 4 slots: 4 block, 0 erased, 0 foreign, 0 other version',
        'boot 9 session 000009: blocks 4 (#0-#3), frames 12 (#0-#11), lines 12, salvaged 2, lost 2, '
        'torn frames 2, repeats 1, torn headers 0, missing frames 0',
    ], result.stdout
    assert sorted(os.listdir(out)) == ['000009', 'session.csv']
    assert _tree(out) == _expected([board])
    assert evil[1] in _tree(out)[os.path.join('000009', 'recorder.log')]


def test_demux_is_recorderds_rule_with_the_movers_routing():
    """demux() files as recorderd's processBuffer does, and refuses any routing that is not a plain file name."""
    vectors = [
        (b'@000123_health.csv@{0};1;<2>\n', ('000123_health.csv', b'{0};1;<2>\n')),
        (b'{3};4 x :: y;<5>\n', (None, b'{3};4 x :: y;<5>\n')),
        (b'@abc@x\n', ('abc', b'x\n')),
        (b'@ab@x\n', (None, b'@ab@x\n')),  # the '@' at index 3 is too early
        (b'@abcx\n', (None, b'@abcx\n')),  # never closed
        (b'\x00@ab\tc@1;\x012\r\n', ('abc', b'1;2\r\n')),  # bytes below 0x0A dropped first
        (b'@abc@\x0b\x7f\x80\xff\n', ('abc', b'\x0b\x7f\x80\xff\n')),  # 0x0A and up kept
        # NEGATIVE: what recorderd would route, the mover refuses: a path, an '@' kept in it, past 64 bytes
        (b'@../x.csv@y\n', (None, b'@../x.csv@y\n')),
        (b'@a/b@y\n', (None, b'@a/b@y\n')),
        (b'@a@b@x\n', (None, b'@a@b@x\n')),
        (b'@' + b'a' * 65 + b'@y\n', (None, b'@' + b'a' * 65 + b'@y\n')),
    ]
    for line, filed in vectors:
        assert ring_split.demux(line) == filed, (line, ring_split.demux(line))
    assert ring_split.demux(b'@' + b'a' * 64 + b'@y\n') == ('a' * 64, b'y\n')


def test_nothing_valid_exits_1():
    """An erased ring, a foreign one, an empty file and an absent boot find nothing; --raw-capture wants one boot."""
    for data in (b'\xff' * (_BLOCKS * _BLOCK_BYTES), b'\x00' * (16 * _BLOCK_BYTES), b''):
        out = _folder()
        result = _run(_write(data), out)
        assert result.returncode == 1 and 'nothing valid found' in result.stderr, (result.stdout, result.stderr)
        assert os.listdir(out) == []
    assert _run(_write(b'\xff' * _BLOCK_BYTES), _folder()).stdout.startswith('ring: 1 slots: 0 block, 1 erased')
    image, _boards = _scenario()
    path = _write(image)
    result = _run(path, _folder(), '--boot', '99')
    assert result.returncode == 1 and 'for boot 99 (boots found: 7, 8)' in result.stderr, result.stderr
    out = _folder()
    result = _run(path, out, '--raw-capture', os.path.join(out, 'raw.txt'))
    assert result.returncode == 2 and 'pick one with --boot' in result.stderr, result.stderr
    assert os.listdir(out) == [], 'nothing written before the usage error'


test_a_ring_splits_into_its_sessions()
test_the_sessions_are_a_luckfox_dump()
test_one_boot_and_its_raw_capture()
test_a_ring_capture_splits_like_the_image()
test_a_frame_is_repeated_only_by_its_own_bytes()
test_demux_is_recorderds_rule_with_the_movers_routing()
test_nothing_valid_exits_1()
print(
    'ok: ring split -- two boots across the wrap (repeat, torn frame, torn header, lost frame, damaged payload, '
    'CONFIRM, foreign/torn-BOOT/misplaced/other-version slots) byte for byte, the sessions as a Luckfox dump, '
    'one boot and its raw capture, a /ring capture splitting like the image, frames that only look repeated '
    '(and a whole one that is), recorderd demux with the mover routing, nothing valid'
)
