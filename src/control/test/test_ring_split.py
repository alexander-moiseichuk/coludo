"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/ring_split.py, the host reader of the Rust firmware's flash ring
(doc/specs/avionics-port.md "Producer side", "Block and chunk format", "The ring_saver", "Duplicates", "SD and
flash file layout"). The images are built here from the spec's tables (the block header, the chunk, the record
and its cell tags), not with ring_split's codec, and every line they must give is written out by hand from the
spec's text rules, so each side checks the other.

One board-sized image holds four boots: two across the ring's wrap, in slots out of sequence order, and two under
one reused label, beside a block of another layout version. It carries every record kind and cell tag, the F32
texts Rust prints (the fewest digits, a tie rounded away from zero, never an exponent), a header repeat that prints
nothing and one with new columns that does, each boot of a reused label printing its own first headers, names the
saver would not file, a chunk programmed twice, a failed program's debris with its retry behind it, the erased gap
of a program that failed before its first byte (no torn chunk), a torn chunk with a later block of its boot behind
it, two damaged chunks (one by a record length that is no multiple of 4), a missing block, and three torn last
chunks that are losses (one whose record count never landed, one stamped with its boot's newest uptime though a
log line drained after it is older, and though an earlier boot of its label ran later uptimes). Its demuxed files
are checked byte for byte, then read by the dump tools as a Luckfox dump is (assemble_capture + flight_telemetry
strict, recorder_flight's wrapped reader), the duplicate filter dropping exactly the lines of the chunk programmed
twice, and the reused label flagged as a splice. A `GET /ring` capture of the same lines splits into the same
files, and so does one that starts mid-boot or holds a damaged `boot` row. An image cut inside a chunk reads up to
the cut with no loss for it. A ring that a reboot in flight left in two layouts (the board came back on an older
build) keeps the other layout's flight blocks apart: reported from their header, never filed. --blocks lists a pad,
flight, pad ring one line per block (sequence order past the wrap, the flight flag, the span of the records'
uptimes with a log line from the pad drained last into the first flight block, rows per file), the same ring cut
inside a chunk, and the board-sized ring with each block's torn and damaged chunks as the boots' summaries count
them and a name past ASCII with a control byte escaped. Negative: names and labels that would leave the output
directory, foreign headers (torn, a label off the alphabet or with a byte after its end, an erased header over dirty
bytes), an erased or foreign ring, an absent boot, a boot only in another layout, an unproven capture, a `boot` row
that never names the boot before it, --raw-capture over several boots, a cut before a block header fits, an image
longer than the partition, a split into a directory whose files it would replace (an SD pull's) without --force,
and under --blocks a torn block (no row counted), a torn header (no line), another layout's block (its header
alone), a capture, an outdir, no block found (with or without boots to name). Stdlib only. Run by `make test`.
"""

import binascii
import contextlib
import io
import os
import re
import struct
import subprocess
import sys
import tempfile
import types

_ROOT: str = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import assemble_capture  # noqa: E402
import flight_telemetry  # noqa: E402
import recorder_flight  # noqa: E402
import recorder_wire  # noqa: E402
import ring_split  # noqa: E402

_TOOL: str = os.path.join(_ROOT, 'tools', 'ring_split.py')
_SCRATCH: tempfile.TemporaryDirectory = tempfile.TemporaryDirectory()  # every file a test writes; gone at exit
_BLOCK_BYTES: int = 4096
_BLOCKS: int = 6144
_HEADER_BYTES: int = 56
_HEADER_KIND: int = 1
_ROW_KIND: int = 2
_LOG_KIND: int = 3
_IMU: bytes = b'imu_lsm6dso32.csv'
_IMU_FIELDS: str = 'uptime;ax;ay;az;gx;gy;gz;irq_runs'  # recorder_flight knows imu_lsm6dso32 by these
_INDEX: bytes = b'session.csv'
_INDEX_FIELDS: str = 'uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon'
_FLOAT_FIELDS: str = 'uptime;' + ';'.join('f%d' % index for index in range(17))
"""
F32 values and the text Rust's Display gives each (checked against Rust itself when written): the fewest
digits that read back, no '.0', no exponent, and of two equally near digits the one further from zero.
"""
_FLOATS: tuple = (
    (0.98, '0.98'),
    (0.05, '0.05'),
    (0.01, '0.01'),
    (1.0, '1'),
    (0.1, '0.1'),
    (-0.0, '-0'),
    (float('nan'), 'NaN'),
    (float('inf'), 'inf'),
    (float('-inf'), '-inf'),
    (1e-7, '0.0000001'),
    (16777216.0, '16777216'),
    (0.000244140625, '0.00024414063'),  # exactly halfway between 8-digit texts: away from zero
    (1048576.25, '1048576.3'),
    (3.4028234663852886e38, '340282350000000000000000000000000000000'),
    (1e-45, '0.000000000000000000000000000000000000000000001'),  # the least subnormal
    (123456.789, '123456.79'),
    (-2.5, '-2.5'),
)


def _folder() -> str:
    """A fresh folder inside _SCRATCH."""
    return tempfile.mkdtemp(dir=_SCRATCH.name)


def _block_header(version: int, flight: bool, block_sequence: int, boot_id: int, session: bytes) -> bytes:
    """A block header from the spec's table: 52 little-endian bytes, then the zlib CRC-32 over them."""
    head = struct.pack('<4sBBHII32sI', b'\xf7CB1', version, int(flight), 0, block_sequence, boot_id, session, 0)
    return head + struct.pack('<I', binascii.crc32(head))


def _chunk(records: list) -> bytes:
    """A chunk from the spec's table: payload length, record count, the CRC over both and the payload, payload."""
    payload = b''.join(records)
    head = struct.pack('<HH', len(payload), len(records))
    return head + struct.pack('<I', binascii.crc32(payload, binascii.crc32(head))) + payload


def _record(kind: int, name: bytes, uptime_us: int, body: bytes, count: int = 0) -> bytes:
    """A record from the spec's table: its length counts the whole record, padded with zeros to a multiple of 4."""
    length = 16 + len(name) + len(body)
    padded = -(-length // 4) * 4
    fixed = struct.pack('<HBBBBHQ', padded, kind, 0, len(name), count, 0, uptime_us)
    return fixed + name + body + bytes(padded - length)


def _cells(*cells: tuple) -> tuple:
    """
    A ROW's cells by the spec's tags.

    Args:
        cells - (kind, value) pairs: 'none', 'i32', 'i64', 'f32', 'bool', 'e7', or 'text' with bytes.

    Returns:
        (cell count, body).
    """
    body = b''
    for kind, value in cells:
        if kind == 'none':
            body += b'\x00'
        elif kind == 'i32':
            body += b'\x01' + struct.pack('<i', value)
        elif kind == 'i64':
            body += b'\x02' + struct.pack('<q', value)
        elif kind == 'f32':
            body += b'\x03' + struct.pack('<f', value)
        elif kind == 'bool':
            body += b'\x04' + bytes([value])
        elif kind == 'e7':
            body += b'\x05' + struct.pack('<i', value)
        else:
            body += b'\x06' + bytes([len(value)]) + value
    return len(cells), body


def _imu_cells() -> tuple:
    """An IMU row's cells: its text is '0.98;0.05;0.01;12;-3;5;1'."""
    floats = [('f32', value) for value, _text in _FLOATS[:3]]
    return _cells(*floats, ('i32', 12), ('i32', -3), ('i32', 5), ('i32', 1))


class _Board:
    """
    One boot writing a ring image as the ring_writer does: it opens a block with its header and appends chunks at
    8-byte alignment behind it, each record stamped 10 ms after the last. `kept` lists the wire lines ring_split
    must give for this boot, in order, and `newest` is the newest uptime of its records on flash: a loss line's
    stamp.
    """

    def __init__(self, image: bytearray, boot_id: int, session: str, uptime_us: int) -> None:
        self.image: bytearray = image
        self.boot_id: int = boot_id
        self.session: str = session
        self.uptime_us: int = uptime_us  # the next record's uptime
        self.slot: int = 0  # the open block's slot
        self.block_sequence: int = 0  # and its sequence
        self.offset: int = 0  # where its next chunk goes
        self.kept: list = []  # the lines ring_split must give, in order
        self.newest: int = 0  # the newest uptime of the records programmed whole

    def stamp(self) -> int:
        """The next record's uptime."""
        uptime_us = self.uptime_us
        self.uptime_us += 10_000
        return uptime_us

    def routing(self, name: bytes) -> str:
        """The routing a file name takes in this session."""
        return name.decode() if name == _INDEX else '%s_%s' % (self.session, name.decode())

    def header(self, name: bytes, columns: str, printed: bool = True) -> tuple:
        """A HEADER record and its line; None for the line of a repeat, which prints nothing."""
        line = recorder_wire.wrap(columns, self.routing(name)).encode() if printed else None
        return _record(_HEADER_KIND, name, self.stamp(), columns.encode()), line

    def row(self, name: bytes, cells: tuple, text: str, routed: bool = True) -> tuple:
        """A ROW record and its line, `<uptime>;<text>`; `routed` False for a name the saver files nowhere."""
        uptime_us = self.stamp()
        line = recorder_wire.wrap('%d;%s' % (uptime_us, text), self.routing(name) if routed else None).encode()
        return _record(_ROW_KIND, name, uptime_us, cells[1], cells[0]), line

    def log(self, descriptor: bytes, message: bytes, text: str | None = None, uptime_us: int | None = None) -> tuple:
        """
        A LOG record and its line, `<uptime> <descriptor> :: <message>`, or `text` with '%d' for the uptime. It is
        stamped next, or at `uptime_us`: the writer drains telemetry before log records more than 20 ms old, so a
        chunk's last record can be older than the rows before it.
        """
        uptime_us = self.stamp() if uptime_us is None else uptime_us
        body = bytes([len(descriptor)]) + descriptor + message
        line = '%d %s :: %s' % (uptime_us, descriptor.decode(), message.decode()) if text is None else text % uptime_us
        return _record(_LOG_KIND, b'recorder.log', uptime_us, body), recorder_wire.wrap(line).encode()

    def status(self) -> tuple:
        """The Recorder's status line. It names the session: recorder_flight finds its part of recorder.log so."""
        return self.log(b'Recorder', b"{'session': '%s', 'tlm': {'count': 0, 'max': 4}}" % self.session.encode())

    def boot_row(self) -> list:
        """session.csv's header and this boot's `boot` row, as the writer writes them first."""
        text = '%d;%s;;;taster;dev;3f2a91;boot;;' % (self.boot_id, self.session)
        texts = [('i32', self.boot_id), ('text', self.session.encode()), ('text', b''), ('text', b'')]
        texts += [('text', b'taster'), ('text', b'dev'), ('text', b'3f2a91'), ('text', b'boot')]
        texts += [('text', b''), ('text', b'')]
        return [self.header(_INDEX, _INDEX_FIELDS), self.row(_INDEX, _cells(*texts), text)]

    def open(self, slot: int, block_sequence: int, flight: bool = False) -> None:
        """Open block `block_sequence` in `slot`: its header at offset 0."""
        self.slot, self.block_sequence, self.offset = slot, block_sequence, _HEADER_BYTES
        self.place(0, _block_header(1, flight, block_sequence, self.boot_id, self.session.encode()))

    def place(self, offset: int, data: bytes) -> None:
        """Program `data` at `offset` of the open block."""
        start = self.slot * _BLOCK_BYTES + offset
        self.image[start : start + len(data)] = data

    def program(self, entries: list, keep: bool = True, cut: int | None = None, damaged_at: int | None = None) -> int:
        """
        Program one chunk at the open block's next 8-aligned offset.

        Args:
            entries - its (record, line) pairs, in order; a line None prints nothing.
            keep - whether its lines are expected: False for a torn one.
            cut - keep only this many leading bytes (a torn program); None programs it all.
            damaged_at - the index of the first entry that is no record: from it on nothing decodes, so nothing
                prints or stamps the boot's newest uptime. None when every entry is a record.

        Returns:
            The offset it went to.
        """
        chunk = _chunk([record for record, _line in entries])
        at = self.offset
        self.place(at, chunk if cut is None else chunk[:cut])
        self.offset = at + -(-len(chunk) // 8) * 8
        if keep:
            decoded = entries[:damaged_at]
            self.kept.extend(line for _record, line in decoded if line is not None)
            self.newest = max([self.newest] + [struct.unpack_from('<Q', record, 8)[0] for record, _line in decoded])
        return at

    def loss(self, offset: int, records: str) -> None:
        """Expect the saver's loss line for the torn chunk at `offset` of the open block, next."""
        text = '%d ring :: block %d: torn chunk at %d, %s records lost' % (
            self.newest,
            self.block_sequence,
            offset,
            records,
        )
        self.kept.append(recorder_wire.wrap(text).encode())


def _scenario() -> tuple:
    """
    The board-sized ring: boots 41 and 42 under one reused label in slots 10-11 (blocks 8000-8001), boot 7 in
    slots 6142-6143 (blocks 9000-9001), boot 8 past the wrap in slots 0, 1 and 4 (blocks 9002, 9003 and 9005: 9004
    is missing), a block of another layout in slot 7, and foreign slots.

    Returns:
        (image, the four _Boards in block order: 41, 42, 7, 8).
    """
    image = bytearray(b'\xff' * (_BLOCKS * _BLOCK_BYTES))
    early = _Board(image, 41, 'flightday', 500_000_000)  # a stale recorder.session: both boots record under it
    early.open(10, 8000)
    early.program(early.boot_row())
    started = [early.header(_IMU, _IMU_FIELDS), early.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')]
    early.program(started, keep=False, cut=0)  # a program that failed before its first byte: an erased gap...
    early.program(started)  # ...its retry behind it, and no torn chunk between: nothing was torn
    late = _Board(image, 42, 'flightday', 2_000_000)  # the next boot: its uptime starts again, far below boot 41's
    late.open(11, 8001)
    late.program(late.boot_row())  # its own index header prints: a new boot of the label
    late.program([late.header(_IMU, _IMU_FIELDS),  # and so does its IMU header: what flags the splice
                  late.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')])
    uneven = struct.pack('<HBBBBHQ', 22, _ROW_KIND, 0, 5, 0, 0, late.stamp()) + b'x.csv\x00'  # 22: not a 4 multiple
    late.program([late.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1'), (uneven, None)], damaged_at=1)
    newest = late.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')
    stamp = struct.unpack_from('<Q', newest[0], 8)[0]
    late.program([newest, late.log(b'Recorder', b'drained late', uptime_us=stamp - 20_000)])  # older, drained after
    torn = [late.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')]
    late.loss(late.program(torn, keep=False, cut=12), '1')  # stamped with the row's uptime: boot 42's newest
    first = _Board(image, 7, '000007', 10_000_000)
    first.open(6142, 9000)
    first.program(first.boot_row())
    first.program([first.header(_IMU, _IMU_FIELDS), first.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')])
    twice = [first.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1') for _row in range(2)] + [first.status()]
    first.program(twice)
    first.program(twice)  # a program that failed after it landed whole, programmed again: duplicates
    floats = _cells(*[('f32', value) for value, _text in _FLOATS])
    first.program([first.header(b'floats.csv', _FLOAT_FIELDS),
                   first.row(b'floats.csv', floats, ';'.join(text for _value, text in _FLOATS))])
    position = 'uptime;lat;lon;fixed;satellites;note;big'
    places = [
        (('e7', -803917950), ('e7', 255143790), ('bool', 1), ('none', None), ('text', b'rtk;fix'), ('i64', 1 << 40)),
        (('e7', -1), ('e7', 0), ('bool', 0), ('i32', -2147483648), ('text', b''), ('i64', -5)),
    ]
    texts = ['-80.3917950;25.5143790;1;None;rtk fix;1099511627776', '-0.0000001;0.0000000;0;-2147483648;;-5']
    first.program([first.header(b'position.csv', position)] +
                  [first.row(b'position.csv', _cells(*cells), text) for cells, text in zip(places, texts)])

    first.open(6143, 9001)
    first.program([first.header(_IMU, _IMU_FIELDS, printed=False),  # the writer's repeat: prints nothing
                   first.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')])
    first.program([first.header(b'floats.csv', 'uptime;f0'),  # new columns: printed
                   first.row(b'floats.csv', _cells(('e7', -2147483648)), '-214.7483648')])
    retried = [first.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1') for _row in range(2)]
    first.program(retried, keep=False, cut=30)  # a program that failed 30 bytes in, in the boot's last block...
    first.program(retried)  # ...and its retry at the next offset: whole behind the debris, so no loss
    first.program([
        first.row(b'a/b.csv', _cells(('i32', 1)), '1', routed=False),  # a path: recorder.log
        first.row(b'../evil.csv', _cells(('i32', 2)), '2', routed=False),
        first.row(b'caf\xc3\xa9\t.csv', _cells(('i32', 3)), '3', routed=False),  # not ASCII, a control byte
        (_record(_HEADER_KIND, b'ab', first.stamp(), b'uptime;x'), recorder_wire.wrap('uptime;x').encode()),
        (_record(_HEADER_KIND, b'ab', first.stamp(), b'uptime;x'), recorder_wire.wrap('uptime;x').encode()),
        first.row(b'position.csv', _cells(('i32', 1), ('text', b'\xff\xfe'), ('i32', 2)), '1'),  # not UTF-8: ends
        first.row(b'position.csv', (4, _cells(('i32', 4), ('i32', 5), ('bool', 1))[1]), '4;5;1'),  # claims 4 cells
    ])
    first.program([first.log(b'tab\there', b'one\r\ntwo\x00x;y \xff', '%d tab here :: one  two x;y �')])
    torn = [first.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1') for _row in range(3)]
    first.loss(first.program(torn, keep=False, cut=40), '3')  # the power went: boot 7's last chunk is lost

    second = _Board(image, 8, '000008', 5_000_000)  # a new boot: its uptime starts again
    second.open(0, 9002)  # slot 0: past the wrap
    second.program(second.boot_row())
    second.program([second.header(_IMU, _IMU_FIELDS), second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1'),
                    second.status()])
    retried = [second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1') for _row in range(2)]
    second.program(retried, keep=False, cut=2)  # failed with only its length landed; the retry did not fit here
    second.open(1, 9003, flight=True)
    sequencer = [second.header(b'sequencer.csv', 'uptime;stage;reason'),
                 second.row(b'sequencer.csv', _cells(('text', b'boosting'), ('text', b'launch')), 'boosting;launch')]
    second.program(retried + sequencer)  # the retry, in the boot's next block: no loss
    damaged = [second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1'), (bytes(8), None),
               second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')]
    second.program(damaged, damaged_at=1)  # a whole chunk whose records stop decoding: only the row before prints
    clock = [('i32', 8), ('text', b'000008'), ('text', b'2026-10-06T12:00:00Z'), ('i32', -240), ('text', b'taster'),
             ('text', b'dev'), ('text', b'3f2a91'), ('text', b'cc-auto'), ('e7', 255000000), ('e7', -803000000)]
    second.program([second.row(_INDEX, _cells(*clock),
                               '8;000008;2026-10-06T12:00:00Z;-240;taster;dev;3f2a91;cc-auto;25.5000000;-80.3000000')])
    second.open(4, 9005, flight=True)  # 9004 is missing: an erase took it
    second.program([second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1') for _row in range(2)])
    lost = [second.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')]
    second.loss(second.program(lost, keep=False, cut=2), '?')  # cut before its record count landed

    image[2 * _BLOCK_BYTES : 3 * _BLOCK_BYTES] = bytes(range(256)) * 16  # a bench pattern
    image[3 * _BLOCK_BYTES + 1000] = 0x00  # an erased header over a dirty byte: a cut erase
    image[5 * _BLOCK_BYTES : 5 * _BLOCK_BYTES + 20] = _block_header(1, False, 9004, 8, b'000008')[:20]  # torn
    image[6 * _BLOCK_BYTES : 6 * _BLOCK_BYTES + 56] = _block_header(1, False, 9006, 8, b'a/b')  # off the alphabet
    newer = _block_header(2, True, 8990, 6, b'000006') + _chunk([_record(_ROW_KIND, _IMU, 1, b'')])
    image[7 * _BLOCK_BYTES : 7 * _BLOCK_BYTES + len(newer)] = newer  # another build's flight: never read
    image[100 * _BLOCK_BYTES : 101 * _BLOCK_BYTES] = b'\x00coludo vfs' * 372 + bytes(4)  # MicroPython-era data
    trailing = _block_header(1, False, 9010, 9, b'000009\x00\x01')  # a byte after the label's NUL: no writer's
    image[12 * _BLOCK_BYTES : 12 * _BLOCK_BYTES + len(trailing)] = trailing
    return bytes(image), (early, late, first, second)


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
    """
    Four boots, two of them across the wrap and two under one reused label, every kind of record and damage: the
    summary's counts, the files byte for byte. A second run into the same directory replaces them only under
    --force.
    """
    image, boards = _scenario()
    out = _folder()
    result = _run(_write(image), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'ring: 6144 slots: 7 block, 6130 erased, 6 foreign, 1 other version',
        'other version 2: boot 6 session 000006: blocks 1 (#8990-#8990), flight blocks 1, not read: this tool reads '
        'layout version 1',
        # the erased gap of a program that failed before its first byte is no torn chunk
        'boot 41 session flightday: blocks 1 (#8000-#8000), missing blocks 0, flight blocks 0, chunks 2, records 4, '
        'lines 4, torn chunks 0, lost chunks 0, damaged chunks 0, duplicates 0',
        # a record 22 bytes long (not a multiple of 4) damages its chunk
        'boot 42 session flightday: blocks 1 (#8001-#8001), missing blocks 0, flight blocks 0, chunks 4, records 7, '
        'lines 8, torn chunks 1, lost chunks 1, damaged chunks 1, duplicates 0',
        'boot 7 session 000007: blocks 2 (#9000-#9001), missing blocks 0, flight blocks 0, chunks 11, records 29, '
        'lines 29, torn chunks 2, lost chunks 1, damaged chunks 0, duplicates 3',
        'boot 8 session 000008: blocks 3 (#9002-#9005), missing blocks 1, flight blocks 2, chunks 6, records 13, '
        'lines 14, torn chunks 2, lost chunks 1, damaged chunks 1, duplicates 0',
        'wrote 13 files under %s' % out,
    ], result.stdout
    assert sorted(os.listdir(out)) == ['000007', '000008', 'flightday', 'session.csv']
    assert sorted(os.listdir(os.path.join(out, '000007'))) == [
        '000007_floats.csv',
        '000007_imu_lsm6dso32.csv',
        '000007_position.csv',
        'recorder.log',
        'session.csv',
    ]
    found, expected = _tree(out), _expected(boards)
    assert sorted(found) == sorted(expected), sorted(found)
    for path, data in expected.items():
        assert found[path] == data, (path, found[path], data)
    # every F32 as Rust prints it, and the E7, None, flag, Text and i64 texts
    floats = found[os.path.join('000007', '000007_floats.csv')].splitlines()[1].decode()
    payload = recorder_wire.verify('000007_floats.csv', floats)
    assert payload.split(';')[1:] == [text for _value, text in _FLOATS], floats
    # the header repeat printed nothing; the new columns did, and so did each boot's first index header
    assert found[os.path.join('000007', '000007_imu_lsm6dso32.csv')].count(b'uptime;') == 1
    assert found[os.path.join('000007', '000007_floats.csv')].count(b'uptime;') == 2
    # a header prints once per boot, not per label: the reused label's second boot prints its own, the splice
    # flight_telemetry flags, and its loss line is stamped with its own newest uptime, the row's (not boot 41's,
    # nor the log line drained after the row though 20 ms older)
    assert found[os.path.join('flightday', 'flightday_imu_lsm6dso32.csv')].count(b'uptime;') == 2
    loss = found[os.path.join('flightday', 'recorder.log')].splitlines()[-1].decode()
    assert recorder_wire.verify(None, loss) == '2060000 ring :: block 8001: torn chunk at 632, 1 records lost', loss
    # the global index holds every boot's rows, in block order
    sessions = [line.split(b';')[3] for line in found['session.csv'].splitlines() if b';uptime;' not in line]
    assert sessions == [b'flightday', b'flightday', b'000007', b'000008', b'000008'], sessions
    # NEGATIVE: a second run into the same directory replaces nothing, and says so, before writing anything
    result = _run(_write(image), out)
    assert result.returncode == 2 and 'replace them with --force' in result.stderr, result.stderr
    assert '%s exists (13 of the files' % os.path.join(out, 'flightday', 'session.csv') in result.stderr, result.stderr
    # under --force it gives the same files, not twice the rows
    assert _run(_write(image), out, '--force').returncode == 0 and _tree(out) == found


def test_a_split_never_cuts_an_sd_pull_down():
    """
    NEGATIVE: split into an SD pull's directory, whose files hold the whole flight while the ring holds only what
    the saver had not erased yet, the run refuses before it writes anything: the longer file and the global
    session.csv with other sessions' rows stay as they were. --force replaces what it names, and only that.
    """
    image, (_early, _late, _first, second) = _scenario()
    out = _folder()
    os.makedirs(os.path.join(out, '000008'))
    flight = b''.join(line.split(b'@', 2)[2] for line in second.kept[2:4]) * 50  # the card's whole flight
    pulled = {
        os.path.join('000008', '000008_imu_lsm6dso32.csv'): flight,
        'session.csv': recorder_wire.wrap('900;123;000123;;;taster;dev;3f2a91;boot;;').encode(),  # another session
        os.path.join('000123', 'recorder.log'): b'kept\n',
    }
    os.makedirs(os.path.join(out, '000123'))
    for path, data in pulled.items():
        with open(os.path.join(out, path), 'wb') as handle:
            handle.write(data)
    result = _run(_write(image), out, '--boot', '8')
    assert result.returncode == 2, result.stdout
    assert '%s exists (2 of the files' % os.path.join(out, 'session.csv') in result.stderr, result.stderr
    assert _tree(out) == pulled, 'nothing written, nothing cut'
    assert _run(_write(image), out, '--boot', '8', '--force').returncode == 0
    assert _tree(out) == {**_expected([second]), os.path.join('000123', 'recorder.log'): b'kept\n'}


def test_the_sessions_are_a_luckfox_dump():
    """
    Each session directory reads in the dump tools as a wrapped Luckfox dump does: all good, through the duplicate
    filter, which drops exactly the lines of the chunk programmed twice (two IMU rows and a log line). A reused
    label's directory holds two boots, which flight_telemetry flags as a splice by the second IMU header.
    """
    image, (early, _late, first, second) = _scenario()
    out = _folder()
    assert _run(_write(image), out).returncode == 0
    for board, (imu_rows, dropped, board_log) in zip((first, second), ((7, 2, 8), (7, 0, 2))):
        directory = os.path.join(out, board.session)
        capture = os.path.join(_folder(), 'capture.txt')
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            assemble_capture.assemble(board.session, directory, capture)
        assert bool(re.search(r'duplicates +%d rows dropped' % dropped, printed.getvalue())) == bool(dropped)
        streams, _logs = flight_telemetry.load(capture)
        assert len(streams['imu_lsm6dso32.csv'].rows) == imu_rows - 1, streams  # less the header
        stamps = [row[0] for row in streams['imu_lsm6dso32.csv'].rows]
        assert len(stamps) == len(set(stamps)), 'each row once'
        _strict(capture)
        assert flight_telemetry.duplicates() == 0, 'assemble_capture dropped them already'
        _found, tally, clock, elsewhere, log = recorder_flight._wrapped_streams(directory, board.session)
        assert all(rejected == 0 for _good, _salvaged, rejected, _duplicates in tally.values()), tally
        assert tally['imu_lsm6dso32'] == [imu_rows, 0, 0, dropped] and elsewhere == 0, tally
        assert tally[recorder_flight._BOARD_LOG] == [board_log, 0, 0, min(dropped, 1)], tally
        assert len(log) == board_log, log
        assert any(';boot;' in row for row in clock), clock
    capture = os.path.join(_folder(), 'capture.txt')
    with contextlib.redirect_stdout(io.StringIO()):
        assemble_capture.assemble(early.session, os.path.join(out, early.session), capture)
    streams, _logs = flight_telemetry.load(capture)
    assert flight_telemetry.spliced(streams) == ['flightday_imu_lsm6dso32.csv'], flight_telemetry.spliced(streams)


def test_one_boot_and_its_raw_capture():
    """--boot keeps one boot; --raw-capture writes its lines as a capture flight_telemetry reads strictly."""
    image, (_early, _late, first, second) = _scenario()
    out, raw = _folder(), os.path.join(_folder(), 'raw.txt')
    result = _run(_write(image), out, '--boot', '8', '--raw-capture', raw)
    assert result.returncode == 0, result.stderr
    assert sorted(os.listdir(out)) == ['000008', 'session.csv']
    assert _tree(out) == _expected([second])
    with open(raw, 'rb') as handle:
        assert handle.read() == b''.join(second.kept)
    assert _strict(raw)['good'] == len(second.kept)
    # boot 7's raw capture holds the chunk programmed twice: the reader drops its three lines
    assert _run(_write(image), _folder(), '--boot', '7', '--raw-capture', raw).returncode == 0
    assert _strict(raw)['good'] == len(first.kept) - 3 and flight_telemetry.duplicates() == 3
    assert flight_telemetry.line_summary().endswith(', 3 duplicates dropped'), flight_telemetry.line_summary()


def test_a_cut_or_oversized_image():
    """
    A `GET /ring/image` cut short inside a chunk of boot 8's block 9003: that slot is read up to the cut, the
    chunk the cut runs through is no loss (the read ended, not the boot), and the summary says where the read
    ends. NEGATIVE: a cut before a block header fits reads nothing of that slot, and an input longer than the
    partition (a whole-flash dump) is refused before anything is written.
    """
    image, (_early, _late, _first, second) = _scenario()
    first_chunk = struct.unpack_from('<H', image, _BLOCK_BYTES + _HEADER_BYTES)[0]  # block 9003, in slot 1
    damaged_at = _HEADER_BYTES + -(-(8 + first_chunk) // 8) * 8  # its second chunk: the damaged one
    cut = image[: _BLOCK_BYTES + damaged_at + 20]
    out = _folder()
    result = _run(_write(cut), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'ring: 2 slots: 2 block, 0 erased, 0 foreign, 0 other version',
        'cut: the image ends %d bytes into slot 1: that slot is read up to the cut, past which it is not'
        % (damaged_at + 20),
        'boot 8 session 000008: blocks 2 (#9002-#9003), missing blocks 0, flight blocks 1, chunks 3, records 9, '
        'lines 9, torn chunks 2, lost chunks 0, damaged chunks 0, duplicates 0',
        'wrote 5 files under %s' % out,
    ], result.stdout
    assert _tree(out) == _expected([types.SimpleNamespace(session='000008', kept=second.kept[:9])])
    # NEGATIVE: cut before a block header fits, the slot is not read, nor counted
    result = _run(_write(image[: _BLOCK_BYTES + 20]), _folder())
    assert result.returncode == 0 and result.stdout.splitlines()[:2] == [
        'ring: 1 slots: 1 block, 0 erased, 0 foreign, 0 other version',
        'cut: the image ends 20 bytes into slot 1, before a block header fits: that slot is not read',
    ], result.stdout
    # NEGATIVE: a whole-flash dump holds the ring from its slot 2048 on, so read from 0 its last slots would be lost
    out = _folder()
    result = _run(_write(image + b'\xff'), out)
    assert result.returncode == 2 and 'esptool.py read_flash 0x800000 0x1800000' in result.stderr, result.stderr
    assert 'the image is %d bytes, longer than the %d' % (len(image) + 1, len(image)) in result.stderr
    assert os.listdir(out) == [] and not result.stdout


def test_a_ring_capture_splits_like_the_image():
    """
    A `GET /ring` capture of the same lines: detected by its first byte, split into the very same files, its boots
    told apart by what its lines name. A dump that starts mid-boot (newest wins erased the boot's first blocks) is
    one boot, named by its later row; a damaged `boot` row costs its boot no line.
    """
    image, boards = _scenario()
    early, late, first, second = boards
    capture = b''.join(line for board in boards for line in board.kept)
    from_image, from_capture = _folder(), _folder()
    assert _run(_write(image), from_image).returncode == 0
    result = _run(_write(capture), from_capture)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[1:5] == [
        'boot 41 session flightday: lines %d, unproven 0, lost 0, duplicates 0' % len(early.kept),
        'boot 42 session flightday: lines %d, unproven 0, lost 0, duplicates 0' % len(late.kept),
        'boot 7 session 000007: lines %d, unproven 0, lost 0, duplicates 3' % len(first.kept),
        'boot 8 session 000008: lines %d, unproven 0, lost 0, duplicates 0' % len(second.kept),
    ], result.stdout
    assert _tree(from_capture) == _tree(from_image)
    # a transfer cut short loses its unterminated line, and one that starts mid-boot is named by its routings
    cut = b''.join(first.kept[2:])[:-10]
    result = _run(_write(cut), _folder())
    assert result.returncode == 0 and 'boot ? session 000007: ' in result.stdout, result.stdout
    assert result.stdout.endswith('lost 1, duplicates 3\nwrote 4 files under %s\n' % result.args[3]), result.stdout
    # a `GET /ring?boot=8` that starts past its index header and `boot` row: its later clock-set row names the
    # boot its routings started, so --boot 8 keeps every line
    mid_boot, out = second.kept[2:], _folder()
    result = _run(_write(b''.join(mid_boot)), out, '--boot', '8')
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[:2] == [
        'capture: %d bytes, 1 boots' % len(b''.join(mid_boot)),
        'boot 8 session 000008: lines %d, unproven 0, lost 0, duplicates 0' % len(mid_boot),
    ], result.stdout
    assert _tree(out) == _expected([types.SimpleNamespace(session='000008', kept=mid_boot)])
    # NEGATIVE: a `boot` row starts its boot, so it never names the unnamed one before it, though the label is the
    # same: boot 41's mid-boot tail stays apart from boot 42
    result = _run(_write(b''.join(early.kept[2:] + late.kept)), _folder())
    assert result.returncode == 0 and result.stdout.splitlines()[1:3] == [
        'boot ? session flightday: lines %d, unproven 0, lost 0, duplicates 0' % (len(early.kept) - 2),
        'boot 42 session flightday: lines %d, unproven 0, lost 0, duplicates 0' % len(late.kept),
    ], result.stdout
    # a damaged `boot` row: boot 8's lines still go to boot 8, by their routings, and its index header with them
    damaged = list(second.kept)
    damaged[1] = damaged[1].replace(b';000008;', b';000009;')
    out = _folder()
    result = _run(_write(b''.join(first.kept + damaged)), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[1:3] == [
        'boot 7 session 000007: lines %d, unproven 0, lost 0, duplicates 3' % len(first.kept),
        'boot 8 session 000008: lines %d, unproven 1, lost 0, duplicates 0' % len(second.kept),
    ], result.stdout
    split = _tree(out)
    assert not [path for path in split if path.startswith(os.path.join('000007', '000008_'))], sorted(split)
    assert {path: data for path, data in split.items() if path.startswith('000007')} == {
        path: data for path, data in _tree(from_image).items() if path.startswith('000007')
    }
    assert split[os.path.join('000008', 'session.csv')].startswith(second.kept[0].split(b'@', 2)[2])
    # NEGATIVE: the same capture forced to be read as an image holds no block
    result = _run(_write(capture), _folder(), '--format', 'image')
    assert result.returncode == 1 and 'nothing valid found' in result.stderr, result.stderr
    # NEGATIVE: lines that never check out are no capture, whatever their shape
    junk = b'@000007_imu_lsm6dso32.csv@{00000000};1;2;<00000000>\nhello\n'
    result = _run(_write(junk), _folder())
    assert result.returncode == 1 and 'nothing valid found' in result.stderr, result.stderr


def test_a_revert_keeps_the_other_layouts_flight_apart():
    """
    A reboot in flight that came back on an older build: boot 41's blocks of layout 2 (its pad and its flight),
    then boot 42's of layout 1 under the same label, opened in flight from the crumb. Boot 42 is read; boot 41 is
    reported from its frozen header, and nothing of it is filed, though its chunk would decode under layout 1.
    """
    image = bytearray(b'\xff' * (8 * _BLOCK_BYTES))  # a short read: the first eight slots
    decodable = _chunk([_record(_ROW_KIND, _IMU, 1, _imu_cells()[1], 7)])
    for slot in range(3):
        header = _block_header(2, slot > 0, 100 + slot, 41, b'flightday')
        image[slot * _BLOCK_BYTES : slot * _BLOCK_BYTES + _HEADER_BYTES + len(decodable)] = header + decodable
    board = _Board(image, 42, 'flightday', 2_000_000)
    board.open(3, 103, flight=True)
    board.program(board.boot_row())
    board.program([board.header(_IMU, _IMU_FIELDS), board.row(_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')])
    board.open(4, 104)  # DONE: back on the pad
    board.program([board.status()])
    out = _folder()
    result = _run(_write(bytes(image)), out)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        'ring: 8 slots: 2 block, 3 erased, 0 foreign, 3 other version',
        'other version 2: boot 41 session flightday: blocks 3 (#100-#102), flight blocks 2, not read: this tool '
        'reads layout version 1',
        'boot 42 session flightday: blocks 2 (#103-#104), missing blocks 0, flight blocks 1, chunks 3, records 5, '
        'lines 5, torn chunks 0, lost chunks 0, damaged chunks 0, duplicates 0',
        'wrote 4 files under %s' % out,
    ], result.stdout
    assert _tree(out) == _expected([board])
    # NEGATIVE: the other layout's boot is not read, and the message says why
    result = _run(_write(bytes(image)), _folder(), '--boot', '41')
    assert result.returncode == 1, result.stdout
    assert result.stderr.strip() == ('nothing valid found for boot 41 (boots found: 42); blocks of layout version 2 '
                                     'are not read by this tool (it reads 1)'), result.stderr
    only = bytes(image[: 3 * _BLOCK_BYTES])
    result = _run(_write(only), _folder())
    assert result.returncode == 1 and result.stderr.strip() == (
        'nothing valid found; blocks of layout version 2 are not read by this tool (it reads 1)'), result.stderr


def _flight_ring() -> bytes:
    """
    A short ring of boot 8 through a flight, as the writer leaves it: a pad block, two flight blocks (the writer
    closes its block at BOOSTING, and again at DONE once the last LANDING records are programmed), then two pad
    blocks past the wrap, the last torn by a power cut before its only chunk landed whole. A log line stamped before
    the rise drains into the first flight block, last in its chunk (the writer merges the held log lines after it
    drains the rows), so the block's first record is not its earliest nor its last the latest. A whole chunk after
    DONE stops decoding. Beside them: a header
    torn by a cut (a foreign slot) and a block of another layout version whose chunk would decode under this one.
    Every record is stamped 10 ms after the last from 1 s on, the drained log line and the records past the damage
    and the tear excepted.

    Returns:
        The image: 8 slots.
    """
    image = bytearray(b'\xff' * (8 * _BLOCK_BYTES))
    imu = (_IMU, _imu_cells(), '0.98;0.05;0.01;12;-3;5;1')
    board = _Board(image, 8, '000008', 1_000_000)
    board.open(4, 500)  # the pad: the index header and the boot row, the IMU header and two rows
    board.program(board.boot_row())
    board.program([board.header(_IMU, _IMU_FIELDS), board.row(*imu), board.row(*imu)])
    board.open(5, 501, flight=True)  # BOOSTING: the sequencer's header and row, two rows, then a log line from the pad
    board.program(
        [
            board.header(b'sequencer.csv', 'uptime;stage;reason'),
            board.row(b'sequencer.csv', _cells(('text', b'boosting'), ('text', b'launch')), 'boosting;launch'),
            board.row(*imu),
            board.row(*imu),
            board.log(b'Recorder', b'queued before the rise', uptime_us=1_045_000),  # merged after the rows drain
        ]
    )
    board.open(6, 502, flight=True)  # GLIDING, LANDING and DONE in the flight's next block
    board.program(
        [
            board.row(b'sequencer.csv', _cells(('text', b'gliding'), ('text', b'separated')), 'gliding;separated'),
            board.row(*imu),
            board.row(b'sequencer.csv', _cells(('text', b'landing'), ('text', b'agl 3m')), 'landing;agl 3m'),
            board.row(b'sequencer.csv', _cells(('text', b'done'), ('text', b'still')), 'done;still'),
        ]
    )
    board.open(0, 503)  # after DONE, past the wrap: a row and the status line, then a chunk damaged after a row
    board.program([board.row(*imu), board.status()])
    board.program([board.row(*imu), (bytes(8), None), board.row(*imu)], damaged_at=1)
    board.open(1, 504)  # the power went while its only chunk was programmed
    board.program([board.row(*imu) for _row in range(3)], keep=False, cut=40)
    image[2 * _BLOCK_BYTES : 2 * _BLOCK_BYTES + 20] = _block_header(1, False, 505, 8, b'000008')[:20]  # torn header
    newer = _block_header(2, True, 499, 6, b'000006') + _chunk([_record(_ROW_KIND, _IMU, 1, b'')])
    image[3 * _BLOCK_BYTES : 3 * _BLOCK_BYTES + len(newer)] = newer  # another build's flight block
    return bytes(image)


def test_blocks_lists_a_pad_flight_pad_ring():
    """
    --blocks lists each block in sequence order, past the wrap, with its flight flag, the span of its records'
    uptimes from the earliest to the latest (the log line drained last into the first flight block is its earliest)
    and its rows per file, writing nothing. An image cut inside a chunk says where it ends, and its block counts the
    chunk the cut runs through as torn. NEGATIVE: the torn block's chunk counts no row, the damaged chunk none past
    its damage, the torn header lists nothing (a foreign slot), and the other layout's block is listed from its
    header alone, its chunk unread.
    """
    path = _write(_flight_ring())
    result = _run(path, '--blocks')
    assert result.returncode == 0 and not result.stderr, result.stderr
    assert result.stdout.splitlines() == [
        'ring: 8 slots: 5 block, 1 erased, 1 foreign, 1 other version',
        'block #499 boot 6 session 000006 flight: layout version 2, not read: this tool reads layout version 1',
        'block #500 boot 8 session 000008 pad: uptime 1000000-1040000, rows imu_lsm6dso32.csv=2 session.csv=1, '
        'torn chunks 0, damaged chunks 0',
        'block #501 boot 8 session 000008 flight: uptime 1045000-1080000, rows imu_lsm6dso32.csv=2 recorder.log=1 '
        'sequencer.csv=1, torn chunks 0, damaged chunks 0',
        'block #502 boot 8 session 000008 flight: uptime 1090000-1120000, rows imu_lsm6dso32.csv=1 sequencer.csv=3, '
        'torn chunks 0, damaged chunks 0',
        'block #503 boot 8 session 000008 pad: uptime 1130000-1150000, rows imu_lsm6dso32.csv=2 recorder.log=1, '
        'torn chunks 0, damaged chunks 1',
        'block #504 boot 8 session 000008 pad: uptime none, rows none, torn chunks 1, damaged chunks 0',
    ], result.stdout
    # --boot keeps the blocks whose header names that boot, another layout's included
    result = _run(path, '--blocks', '--boot', '6')
    assert result.returncode == 0 and result.stdout.splitlines()[1:] == [
        'block #499 boot 6 session 000006 flight: layout version 2, not read: this tool reads layout version 1',
    ], result.stdout
    assert len(_run(path, '--blocks', '--boot', '8').stdout.splitlines()) == 1 + 5
    # the split of the same ring counts the same flight blocks
    assert 'blocks 5 (#500-#504), missing blocks 0, flight blocks 2,' in _run(path, _folder()).stdout
    # cut 20 bytes into block 503's second chunk (slot 0): its first chunk is read, the one cut through is torn
    image = _flight_ring()
    first_chunk = struct.unpack_from('<H', image, _HEADER_BYTES)[0]
    cut = _HEADER_BYTES + -(-(8 + first_chunk) // 8) * 8 + 20
    result = _run(_write(image[:cut]), '--blocks')
    assert result.returncode == 0 and result.stdout.splitlines() == [
        'ring: 1 slots: 1 block, 0 erased, 0 foreign, 0 other version',
        'cut: the image ends %d bytes into slot 0: that slot is read up to the cut, past which it is not' % cut,
        'block #503 boot 8 session 000008 pad: uptime 1130000-1140000, rows imu_lsm6dso32.csv=1 recorder.log=1, '
        'torn chunks 1, damaged chunks 0',
    ], result.stdout


def test_blocks_over_the_board_sized_ring():
    """
    --blocks over the board-sized ring: the blocks in sequence order across the wrap (slot order is not), each
    one's flag and its torn and damaged chunks as the boots' summaries count them, a chunk programmed twice counting
    its rows twice, and names the saver would not file counted under their own names, control bytes printed as
    spaces and bytes past ASCII as escapes.
    """
    image, _boards = _scenario()
    result = _run(_write(image), '--blocks')
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    assert lines[0] == 'ring: 6144 slots: 7 block, 6130 erased, 6 foreign, 1 other version', lines[0]
    pattern = re.compile(
        r'block #(\d+) boot (\d+) session \S+ (flight|pad): (?:layout version 2|.*, torn chunks '
        r'(\d+), damaged chunks (\d+))'
    )
    found = [pattern.fullmatch(line.split(', not read:')[0]).groups() for line in lines[1:]]
    assert found == [
        ('8000', '41', 'pad', '0', '0'),
        ('8001', '42', 'pad', '1', '1'),
        ('8990', '6', 'flight', None, None),
        ('9000', '7', 'pad', '0', '0'),
        ('9001', '7', 'pad', '2', '0'),
        ('9002', '8', 'pad', '1', '0'),
        ('9003', '8', 'flight', '0', '1'),
        ('9005', '8', 'flight', '1', '0'),
    ], found
    rows = {line.split(' ')[1]: line.split(', rows ')[1].split(', torn')[0] for line in lines[1:] if ', rows ' in line}
    assert rows['#9000'] == 'floats.csv=1 imu_lsm6dso32.csv=5 position.csv=2 recorder.log=2 session.csv=1', rows
    # a name's control byte prints as a space and its bytes past ASCII as escapes
    assert rows['#9001'] == (
        '../evil.csv=1 a/b.csv=1 caf\\xc3\\xa9 .csv=1 floats.csv=1 imu_lsm6dso32.csv=3 position.csv=2 recorder.log=1'
    ), rows


def test_blocks_refuses_what_it_cannot_list():
    """
    NEGATIVE: --blocks writes nothing, so an outdir, --raw-capture or --force with it is a usage error, as is no
    outdir without it; a capture holds no blocks; an oversized image is refused as the split refuses it; an erased
    ring (asked for a boot or not, it finds none) and an absent boot list no block (exit 1).
    """
    path, out = _write(_flight_ring()), _folder()
    for extra in ([out], ['--raw-capture', os.path.join(out, 'raw.txt')], ['--force']):
        result = _run(path, '--blocks', *extra)
        assert result.returncode == 2 and 'it takes no outdir, --raw-capture or --force' in result.stderr, extra
    assert os.listdir(out) == [], 'nothing written'
    result = _run(path)
    assert result.returncode == 2 and 'the outdir is missing: only --blocks writes nothing' in result.stderr
    image, boards = _scenario()
    capture = _write(b''.join(line for board in boards for line in board.kept))
    for extra in ([], ['--format', 'capture']):
        result = _run(capture, '--blocks', *extra)
        assert result.returncode == 2 and 'a capture holds no blocks' in result.stderr, (extra, result.stderr)
    result = _run(_write(image + b'\xff'), '--blocks')
    assert result.returncode == 2 and 'longer than the %d of the log partition' % len(image) in result.stderr
    erased = _write(b'\xff' * (4 * _BLOCK_BYTES))
    result = _run(erased, '--blocks')
    assert result.returncode == 1 and result.stderr == 'no block found\n', result.stderr
    assert result.stdout == 'ring: 4 slots: 0 block, 4 erased, 0 foreign, 0 other version\n', result.stdout
    result = _run(erased, '--blocks', '--boot', '3')
    assert result.returncode == 1 and result.stderr == 'no block found for boot 3 (boots found: none)\n', result.stderr
    result = _run(path, '--blocks', '--boot', '99')
    assert result.returncode == 1 and result.stderr == 'no block found for boot 99 (boots found: 6, 8)\n'


def test_demux_is_recorderds_rule_with_the_savers_routing():
    """demux() files as recorderd's processBuffer does, and refuses any routing that is not a plain file name."""
    vectors = [
        (b'@000123_health.csv@{0};1;<2>\n', ('000123_health.csv', b'{0};1;<2>\n')),
        (b'{3};4 x :: y;<5>\n', (None, b'{3};4 x :: y;<5>\n')),
        (b'@abc@x\n', ('abc', b'x\n')),
        (b'@ab@x\n', (None, b'@ab@x\n')),  # the '@' at index 3 is too early
        (b'@abcx\n', (None, b'@abcx\n')),  # never closed
        (b'\x00@ab\tc@1;\x012\r\n', ('abc', b'1;2\r\n')),  # bytes below 0x0A dropped first
        (b'@abc@\x0b\x7f\x80\xff\n', ('abc', b'\x0b\x7f\x80\xff\n')),  # 0x0A and up kept
        # NEGATIVE: what recorderd would route, the saver refuses: a path, an '@' kept in it, past 64 bytes
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
        assert result.returncode == 1 and result.stderr == 'nothing valid found\n', (result.stdout, result.stderr)
        assert os.listdir(out) == []
    assert _run(_write(b'\xff' * _BLOCK_BYTES), _folder()).stdout.startswith('ring: 1 slots: 0 block, 1 erased')
    image, _boards = _scenario()
    path = _write(image)
    result = _run(path, _folder(), '--boot', '99')
    assert result.returncode == 1 and 'for boot 99 (boots found: 41, 42, 7, 8)\n' in result.stderr, result.stderr
    out = _folder()
    result = _run(path, out, '--raw-capture', os.path.join(out, 'raw.txt'))
    assert result.returncode == 2 and 'pick one with --boot' in result.stderr, result.stderr
    assert os.listdir(out) == [], 'nothing written before the usage error'


test_a_ring_splits_into_its_sessions()
test_the_sessions_are_a_luckfox_dump()
test_a_split_never_cuts_an_sd_pull_down()
test_one_boot_and_its_raw_capture()
test_a_cut_or_oversized_image()
test_a_ring_capture_splits_like_the_image()
test_a_revert_keeps_the_other_layouts_flight_apart()
test_blocks_lists_a_pad_flight_pad_ring()
test_blocks_over_the_board_sized_ring()
test_blocks_refuses_what_it_cannot_list()
test_demux_is_recorderds_rule_with_the_savers_routing()
test_nothing_valid_exits_1()
print(
    'ok: ring split -- four boots across the wrap and under a reused label (every record kind and cell tag, Rust '
    'F32 texts, header repeats, headers per boot, unfiled names, a chunk twice, retried debris, an erased gap, a torn '
    'chunk its boot goes on past, damaged chunks, a missing block, three losses stamped per boot) byte for byte, no '
    'file replaced without --force, the sessions as a Luckfox dump through the duplicate filter, one boot and its raw '
    'capture, a cut and an oversized image, a /ring capture splitting like the image (mid-boot, a damaged boot row), '
    "a revert leaving another layout's flight apart, --blocks over a pad/flight/pad ring and the board-sized one (a "
    'torn block, another layout, the refusals), recorderd demux with the saver routing, nothing valid'
)
