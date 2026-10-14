"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Split a flash ring dump of the Rust flight firmware (src/avionics) into session directories, the layout the
ring_saver writes on the SD (doc/specs/avionics-port.md "SD and flash file layout"). It is the recovery path when
the card failed in flight, and the check before a ring is given up: a session directory it writes is a drop-in
for a Luckfox /userdata/recordings dump, so assemble_capture, recorder_flight and flight_telemetry read it as
they read one.

The input is one of two things:
  * a raw image of the `log` partition (`GET /ring/image`, or `esptool.py read_flash 0x800000 0x1800000
    ring.bin`), or any image that starts at the partition's offset 0. It is 6144 slots of 4 KiB ("Block and
    chunk format"): a block opens with a 56-byte header naming its sequence, boot and session, and binary records
    ("Producer side") follow in CRC'd chunks at 8-byte alignment. A shorter read keeps the slots it holds, and
    judges a torn chunk by them alone: a boot that goes on in a slot past the read can show a loss there that the
    whole partition would not.
    A read that ends inside a slot (a transfer cut short) reads that slot up to the cut, as the board reads its
    writer's open block up to the writer's position: a chunk the cut runs through is torn in the read, not lost
    in flash, so it makes no loss line, and the summary says where the read ends. An input longer than the
    partition is refused, naming the read that gives the partition alone: a whole-flash dump holds the ring from
    its slot 2048 on, so read from offset 0 it would lose the ring's last 2048 slots without a word;
  * a `GET /ring[?boot=N]` dump: every block's lines as the board decodes them, already a wire capture.
A capture starts with a wire line's '@' or '{'. Anything else is an image, whatever its slot 0 holds (an erased
or foreign slot 0 is a normal ring). `--format` overrides the guess.

An image is read as the ring_saver reads the ring ("The ring_saver"), by the spec's tables:
  * Each slot is classified by its bytes. All 0xFF is erased. A block header that checks out (magic, CRC, a
    session label of the header's alphabet) is a block when its layout version is this tool's, and a block of
    another version otherwise. Anything else is foreign: MicroPython-era data, a bench pattern, a cut erase, a
    header torn by a cut, or an erased header over bytes that are not (which the board's writer refuses too).
  * A block of ANOTHER LAYOUT VERSION is another build's: after an update, or after a reboot in flight that came
    back on an older image, one ring holds the blocks of both builds, and the flight may be in either. The
    56-byte header is the same in every version, so such a block's sequence, boot, session and flight flag are
    reported; its chunks are not read, and nothing of it is filed. A later layout gets its decoder here BESIDE
    this one, never in its place: the tool of the newest build must read a ring that a revert left in two.
  * The blocks are read in block-sequence order, which the slots no longer keep once the ring wraps or newest
    wins has erased out of order. The consecutive blocks of one (boot id, session) are one boot.
  * Each chunk's CRC is checked. A TORN CHUNK (a CRC that fails, a length past the block) is stepped over in
    8-byte steps to the next whole chunk, as the board's walk does. It is a loss only when no whole chunk follows
    it in its block and no later block of its boot is in the image: a failed program that the writer retried
    behind it, or in its boot's next block, is not. A loss is noted in the session's recorder.log by a wrapped
    log line, as the saver notes it: `<uptime> ring :: block <sequence>: torn chunk at <offset>, <k> records
    lost`, stamped with the newest uptime decoded in that boot, so recorder.log never goes back in time; `k` is
    '?' when the record count never landed.
  * Each record prints as its line ("Row, log and index text"): a ROW `<uptime>;v1;...;vn`, a HEADER its column
    list, a LOG `<uptime_us> <descriptor> :: <message>`. It is wrapped with its routing: `<session>_<file>`, the
    bare `session.csv` for the index, none for a LOG or for a file name the saver would not file. A HEADER prints
    only where its file starts in its boot, or where its columns change: the writer repeats every header about
    once per MiB, and those repeats never reach the SD. A whole chunk whose records stop decoding is DAMAGED (no
    writer of this layout makes one): the records before the damage print, the rest of the chunk does not.
  * A chunk the writer programmed twice (a program that failed after it had landed whole) gives its lines twice.
    The session is written as the saver writes it, duplicates included; the readers drop them
    (recorder_wire.unique, "Duplicates"), and the summary counts them.
That is the board's decoder (src/avionics/crates/recorder/src/decoder.rs) line for line, so a session split from
an image is byte for byte what the SD would hold. A capture has no blocks: its boots are told apart by what its
verified lines name (read_capture()): session.csv's rows name a boot id and session, and every routed line opens
with its session.

Each line is then filed by recorderd's demux with the saver's routing rule (demux() below), into
`<outdir>/<session>/`, the session the block header (or the capture's session.csv) names:
  * a routed line goes to `<session>/<routing>` (`<session>_<stream>.csv`), as the bytes after the routing;
  * `session.csv` goes there and to the global `<outdir>/session.csv` too;
  * every other line goes to `<session>/recorder.log`.
Every file is written whole, so the tool never replaces a file that is there already: split into an SD pull's
directory, it would cut the card's whole-flight file down to what the ring still holds, and the global
session.csv down to this input's boots. It refuses before writing anything (exit 2), naming the file, unless
--force says to replace them; under --force a second run into the same directory gives the same files.

    python3 tools/ring_split.py <input> <outdir> [--boot N] [--raw-capture FILE] [--format image|capture] [--force]
    python3 tools/ring_split.py <image> --blocks [--boot N] [--format image]

--boot N keeps that boot alone. --raw-capture FILE also writes one boot's lines in order, as a wire capture for
flight_telemetry: what `GET /ring?boot=N` streams. It needs --boot when the input holds several boots. A summary
line is printed per slot state, per run of another version's blocks, and per boot.

--blocks lists an image's blocks instead, and writes nothing (so it takes no outdir): after the slot counts, one
line per block in block-sequence order, every block of that boot alone under --boot N:

    block #<sequence> boot <id> session <label> flight|pad: uptime <first>-<last>, rows <file>=<count> ...,
        torn chunks <n>, damaged chunks <n>

`flight` is the header's flag, set on a block opened in BOOSTING..DONE; `pad` is every other block. `uptime` spans
the block's records, the earliest to the latest (a log line drained after a row can be older than it), `none` when
no record decodes. `rows` counts the ROW and LOG records per file name, sorted, as they were programmed: a chunk
programmed twice counts twice, a HEADER record is no row, and a LOG record's file is recorder.log. A name the saver
would not file prints its control bytes as spaces and its bytes past ASCII as `\\xNN`. The chunk counts are the
walk's: whether a torn chunk is a loss (its boot may go on past it) is the split's summary to say. A block of
another layout version gets its header's fields and `layout version <v>, not read`. The listing shows which blocks
carry the flight flag: every record from BOOSTING to DONE lies in a flight block, and a block opened just before
the rise may carry the flag too (doc/specs/avionics-port.md "Ring geometry and block life cycle"). A capture holds
no blocks, so --blocks refuses one.

Exit 0 on success, 1 when nothing this tool reads was found (or not the boot asked for; under --blocks, no block
of any version), 2 for a usage error (an outdir with --blocks or none without it, --blocks over a capture), an image
longer than the partition, or a file that would be replaced without --force.
"""

import argparse
import binascii
import decimal
import math
import os
import re
import struct
import sys
from collections.abc import Iterator
from fractions import Fraction
from typing import NamedTuple

import recorder_wire

_MAGIC: bytes = b'\xf7CB1'  # 0xF7 never occurs in UTF-8, and the magic is neither all-0 nor all-1
_VERSION: int = 1  # the layout this tool decodes: the chunk and the record formats behind the header
_BLOCK_BYTES: int = 4096  # one erase sector
_BLOCKS: int = 6144  # the board's `log` partition: 24 MiB
_BLOCK_HEADER: struct.Struct = struct.Struct('<4sBBHII32sII')  # magic, version, flags, 0, sequence, boot, label, 0, CRC
_BLOCK_HEADER_CRC_AT: int = 52  # the header CRC covers the bytes before it
_FLIGHT: int = 0x01  # the flags bit of a block opened in BOOSTING..DONE
_CHUNK_HEADER: struct.Struct = struct.Struct('<HHI')  # payload length, record count, CRC over 0..4 and the payload
_ALIGNMENT: int = 8  # chunks start on it
_MAX_PAYLOAD: int = _BLOCK_BYTES - _BLOCK_HEADER.size - _CHUNK_HEADER.size  # one such chunk fills a block
_ERASED_LENGTH: int = 0xFFFF  # a chunk length field that reads erased: no chunk starts here
_RECORD_HEADER: struct.Struct = struct.Struct('<HBBBBHQ')  # length, kind, flags, name length, count, 0, uptime_us
_RECORD_ALIGNMENT: int = 4  # a record's length is a multiple of it
_HEADER_KIND: int = 1  # a record carrying a stream's column list
_ROW_KIND: int = 2  # a record of typed cells
_LOG_KIND: int = 3  # a recorder.log line's descriptor and message
_NONE_TAG: int = 0  # a ROW cell with no value: prints `None`
_I32_TAG: int = 1  # an i32
_I64_TAG: int = 2  # an i64
_F32_TAG: int = 3  # an f32
_BOOL_TAG: int = 4  # a flag, one byte
_E7_TAG: int = 5  # an i32 of 1e-7 degrees
_TEXT_TAG: int = 6  # a length byte, then that many bytes of UTF-8
_CELL_BYTES: dict = {_NONE_TAG: 0, _I32_TAG: 4, _I64_TAG: 8, _F32_TAG: 4, _BOOL_TAG: 1, _E7_TAG: 4}  # after the tag
_UNITS_PER_DEGREE: int = 10_000_000  # an E7 cell counts 1e-7 degrees
_ERASED: bytes = b'\xff'
_SPACES: bytes = bytes.maketrans(bytes(range(0x20)) + b'\x7f', b' ' * 0x21)  # control bytes print as spaces
_CELL_SPACES: bytes = bytes.maketrans(bytes(range(0x20)) + b'\x7f;', b' ' * 0x22)  # and a Text cell's ';' too
_FLOAT_SIGNIFICAND: int = 1 << 23  # an f32's significand of a binade's first value, its implicit bit alone
_FLOAT_DIGITS: int = 9  # nine significant digits always tell two f32 values apart
_FLOAT_TEXTS: dict = {}  # each f32 printed so far, by its bytes, to its text
_FLOAT_TEXTS_HELD: int = 1 << 18  # at most this many remembered: about 40 MB
_LABEL: re.Pattern = re.compile(rb'[A-Za-z0-9_-]{1,32}')  # a block header's session label (the frozen alphabet)
_BELOW_LINE_END: bytes = bytes(range(0x0A))  # recorderd drops these bytes as they arrive
_ROUTING: re.Pattern = re.compile(rb'[A-Za-z0-9._-]{3,64}')  # the saver's rule: a file name, never a path
_SESSION: re.Pattern = re.compile(r'[A-Za-z0-9_-]{1,64}')  # a capture's session cell that can name a directory
_ROUTED_SESSION: re.Pattern = re.compile(r'(\d{8}_\d{6}_\d{6}|[A-Za-z0-9-]+)_.')  # the session a routing opens with
_SOURCE_CELL: int = 8  # a session.csv row's `source` cell (after uptime, boot, session, utc, ...): what wrote it
_BOOT_SOURCE: str = 'boot'  # the source of the row every boot writes first: it starts its boot


class _BlockHeader(NamedTuple):
    """A block header's fields (flash_ring header.rs `BlockHeader`); the CRC is checked, never kept."""

    version: int
    flight: bool
    block_sequence: int
    boot_id: int
    session: str


class _Boot:
    """
    One boot's share of the input, in order: the lines kept for its session directory, and what reading them
    found. An image tells two boots apart by their block headers, a capture by what its lines name (see
    read_capture()).
    """

    def __init__(self, boot_id: int | None, session: str | None, identity: object) -> None:
        self.boot_id: int | None = boot_id  # None: a capture boot no session.csv row named
        self.session: str | None = session  # its directory; None until a capture's lines name it
        self.identity: object = identity  # what a following block or row must match to stay in this boot
        self.lines: list = []  # the wire lines kept, each with its '\n': the records' and the loss notes
        self.blocks: list = []  # the block sequences read (an image)
        self.flight: int = 0  # blocks with the flight flag
        self.chunks: int = 0  # whole chunks
        self.records: int = 0  # records in them, header repeats that printed nothing included
        self.torn: int = 0  # torn chunks
        self.lost: int = 0  # torn chunks that are losses (an image), or unterminated lines (a capture)
        self.damaged: int = 0  # whole chunks whose records stopped decoding
        self.unproven: int = 0  # a capture's lines filed whose wrapper does not check out

    def take(self, lines: list) -> None:
        """Keep a capture's lines, in order: each (line, whether it does not check out)."""
        self.lines.extend(line for line, _unproven in lines)
        self.unproven += sum(unproven for _line, unproven in lines)


class _Decoder:
    """
    The board's block decoder (crates/recorder decoder.rs) over one image: it remembers, per boot, the columns
    each file's header line printed and the newest uptime decoded, so it is fed the blocks in sequence order.
    """

    def __init__(self) -> None:
        self.printed: dict = {}  # (session, boot id, file name) -> the column list its header line printed
        self.newest: dict = {}  # (session, boot id) -> the newest uptime decoded: a loss line's stamp

    def block(self, boot: _Boot, header: _BlockHeader, block: bytes, boot_goes_on: bool) -> None:
        """
        Decode one block of this layout into `boot`'s lines, in record order.

        Args:
            boot - the boot it belongs to: its lines and counters grow.
            header - its header.
            block - its bytes from offset 0.
            boot_goes_on - a later block of the same (boot id, session) is in the image: a torn chunk with
                nothing whole behind it is then a failed program the writer retried there, not a loss.

        Returns:
            None.
        """
        key = (header.session, header.boot_id)
        for walked in _chunks(block):
            if walked[0] == 'chunk':
                _kind, _offset, _count, payload = walked
                boot.chunks += 1
                for record in _records(payload):
                    if record is None:
                        boot.damaged += 1
                        break
                    boot.records += 1
                    self._record(boot, key, record)
                continue
            _kind, offset, count, followed = walked
            boot.torn += 1
            if followed or boot_goes_on:
                continue  # a failed program: the writer programmed its records again behind it
            boot.lost += 1
            message = 'block %d: torn chunk at %d, %s records lost' % (
                header.block_sequence,
                offset,
                '?' if count is None else count,
            )
            text = '%d ring :: %s' % (self.newest.get(key, 0), message)
            boot.lines.append(recorder_wire.wrap(text).encode('utf-8'))

    def _record(self, boot: _Boot, key: tuple, record: tuple) -> None:
        """
        Print one record as its wire line, unless it is a header whose file printed these columns already.

        Args:
            boot - the boot it belongs to: its lines grow.
            key - the boot's (session, boot id).
            record - the record, as _records() gives it.

        Returns:
            None.
        """
        kind, uptime_us, name, count, body = record
        self.newest[key] = max(self.newest.get(key, uptime_us), uptime_us)
        routing = None
        if kind != _LOG_KIND and _ROUTING.fullmatch(name):
            routing = name.decode('ascii')
            if routing != recorder_wire.SESSION_INDEX:
                routing = '%s_%s' % (key[0], routing)
        if kind == _HEADER_KIND:
            columns = body.rstrip(b'\x00')
            if routing is not None:
                if self.printed.get(key + (name,)) == columns:
                    return  # a repeat: its file has these columns already
                self.printed[key + (name,)] = columns
            text = columns.translate(_SPACES)
        elif kind == _ROW_KIND:
            text = b'%d;%s' % (uptime_us, b';'.join(_cells(count, body)))
        else:
            text = _log_text(uptime_us, body)
        line = recorder_wire.wrap(text.decode('utf-8', 'surrogateescape'), routing)
        boot.lines.append(line.encode('utf-8', 'surrogateescape'))


def demux(line: bytes) -> tuple:
    """
    Where recorderd files one complete line, under the ring_saver's routing rule (doc/specs/avionics-port.md
    "The ring_saver"; recorder_wire::split in src/avionics is the same rule).

    recorderd drops every byte below 0x0A. A line starting with '@' whose next '@' sits at index 4 or later is
    routed: the routing is the bytes between, so it is at least 3 bytes long, and its file gets the bytes after
    that '@'. The saver adds one rule: a routing must match [A-Za-z0-9._-]{3,64}, so it names a file in the
    session directory and never a path. Any other line goes whole to recorder.log.

    Args:
        line - one line, its '\\n' included.

    Returns:
        (routing, the bytes its file gets) for a routed line; (None, the line) for one recorder.log gets.
    """
    line = line.translate(None, _BELOW_LINE_END)
    if line.startswith(b'@'):
        close = line.find(b'@', 4)
        if close > 0 and _ROUTING.fullmatch(line[1:close]):
            return line[1:close].decode('ascii'), line[close + 1 :]
    return None, line


def _block_header(raw: bytes) -> _BlockHeader | None:
    """
    The block header at the start of `raw`, checked as flash_ring's `BlockHeader::decode` checks it: the magic,
    the CRC, and a session label of the frozen alphabet followed by zeros only. Of any layout version.

    Args:
        raw - a slot's bytes.

    Returns:
        The header, or None when the slot holds none.
    """
    if len(raw) < _BLOCK_HEADER.size or raw[:4] != _MAGIC:
        return None
    fields = _BLOCK_HEADER.unpack_from(raw)
    _magic, version, flags, _zero, block_sequence, boot_id, field, _reserved, header_crc = fields
    if binascii.crc32(raw[:_BLOCK_HEADER_CRC_AT]) != header_crc:
        return None
    label, _end, padding = field.partition(b'\x00')
    if padding.strip(b'\x00') or not _LABEL.fullmatch(label):
        return None  # a CRC that checks out over a label no ring writer writes
    return _BlockHeader(version, bool(flags & _FLIGHT), block_sequence, boot_id, label.decode('ascii'))


def _chunk(block: bytes, offset: int) -> tuple | None:
    """
    The whole chunk at `offset` of `block` (flash_ring chunk.rs `decode`), its CRC checked over the length, the
    record count and the payload.

    Args:
        block - the block's bytes from offset 0: a chunk running past them is no chunk.
        offset - where the chunk would start.

    Returns:
        (record count, payload); None for the erased tail, a length past the block or past the largest payload,
        and a CRC that fails.
    """
    if offset + _CHUNK_HEADER.size > len(block):
        return None
    length, count, chunk_crc = _CHUNK_HEADER.unpack_from(block, offset)
    start = offset + _CHUNK_HEADER.size
    if length == _ERASED_LENGTH or length > _MAX_PAYLOAD or start + length > len(block):
        return None
    payload = block[start : start + length]
    if binascii.crc32(payload, binascii.crc32(block[offset : offset + 4])) != chunk_crc:
        return None
    return count, payload


def _chunks(block: bytes) -> Iterator[tuple]:
    """
    The chunks of a block, whole and torn, in address order, up to the erased tail (flash_ring chunk.rs `chunks`).

    Where no whole chunk starts, the walk looks for the next one in 8-byte steps (or the erased tail) and reports
    the bytes it stepped over as one torn chunk, unless they are all erased: a program that failed before its
    first byte landed left nothing.

    Args:
        block - the block's bytes from offset 0.

    Returns:
        An iterator of ('chunk', offset, record count, payload) and ('torn', offset, record count as read or None
        when it reads erased, whether a whole chunk follows it in the block).
    """
    end = len(block.rstrip(_ERASED))
    offset = _BLOCK_HEADER.size
    while offset < end:
        found = _chunk(block, offset)
        if found is not None:
            yield ('chunk', offset) + found
            offset += -(-(_CHUNK_HEADER.size + len(found[1])) // _ALIGNMENT) * _ALIGNMENT
            continue
        following = offset + _ALIGNMENT
        while following < end and _chunk(block, following) is None:
            following += _ALIGNMENT
        if block[offset : min(following, end)].strip(_ERASED):
            field = block[offset + 2 : offset + 4]
            count = int.from_bytes(field, 'little') if len(field) == 2 else None
            yield 'torn', offset, None if count == _ERASED_LENGTH else count, following < end
        offset = following


def _records(payload: bytes) -> Iterator[tuple | None]:
    """
    The records of a whole chunk's payload, in order (crates/recorder record.rs `Records`).

    Args:
        payload - the payload, CRC checked.

    Returns:
        An iterator of (kind, uptime_us, file name, cell count, body) per record, then one None when bytes that
        are no record end the walk: their length cannot be trusted to find the next.
    """
    at = 0
    while at < len(payload):
        if len(payload) - at < _RECORD_HEADER.size:
            yield None
            return
        length, kind, _flags, name_length, count, _zero, uptime_us = _RECORD_HEADER.unpack_from(payload, at)
        body = _RECORD_HEADER.size + name_length
        if length < _RECORD_HEADER.size or length % _RECORD_ALIGNMENT or length > len(payload) - at or body > length:
            yield None
            return
        if kind not in (_HEADER_KIND, _ROW_KIND, _LOG_KIND):
            yield None
            return
        record = payload[at : at + length]
        yield kind, uptime_us, record[_RECORD_HEADER.size : body], count, record[body:]
        at += length


def _float_text(raw: bytes) -> bytes:
    """
    An F32 cell as Rust's `Display` prints it (see _float_display), remembered by its bytes: sensors repeat their
    values, and a full ring holds millions of them. At most _FLOAT_TEXTS_HELD are remembered.
    """
    text = _FLOAT_TEXTS.get(raw)
    if text is None:
        text = _float_display(raw)
        if len(_FLOAT_TEXTS) < _FLOAT_TEXTS_HELD:
            _FLOAT_TEXTS[raw] = text
    return text


def _float_display(raw: bytes) -> bytes:
    """
    An F32 cell as Rust's `Display` prints it: the fewest significant digits that read back as the same f32, the
    nearest such when several do, positional (never an exponent), no '.0', and 'NaN', 'inf', '-inf', '-0'.

    The digits come from CPython's correctly rounded formatting. Away from a binade's first value an f32 reads
    back from an interval centred on it, so the nearest n digits are inside whenever any n digits are, and n+1
    digits are whenever n are: the fewest are found by halving 1..9, nine always telling two f32 values apart. A
    binade's first value has its lower neighbour twice as close, so the n digits each side of the correctly
    rounded ones may be the nearest inside; it is decided exactly, n by n. Of two equally near, Rust prints the
    one further from zero (0.000244140625 prints 0.00024414063), where CPython's formatting takes the even one.

    Args:
        raw - the cell's four little-endian bytes.

    Returns:
        Its text.
    """
    value = struct.unpack('<f', raw)[0]
    if math.isnan(value):
        return b'NaN'
    if math.isinf(value):
        return b'inf' if value > 0 else b'-inf'
    if value == 0:
        return b'-0' if math.copysign(1.0, value) < 0 else b'0'
    bits, magnitude = struct.unpack('<I', raw)[0] & 0x7FFFFFFF, abs(value)
    if bits >> 23 and not bits & (_FLOAT_SIGNIFICAND - 1):
        whole, point = _binade_first(magnitude, bits)
    else:
        fewest, most = 1, _FLOAT_DIGITS
        while fewest < most:
            middle = (fewest + most) // 2
            if _reads_back(magnitude, bits, middle):
                most = middle
            else:
                fewest = middle + 1
        whole, point = _nearest(magnitude, fewest)
    while whole % 10 == 0:
        whole, point = whole // 10, point + 1
    figures = str(whole)
    integral = len(figures) + point  # the figures read as 0.<figures> * 10**integral
    if point >= 0:
        text = figures + '0' * point
    elif integral > 0:
        text = figures[:integral] + '.' + figures[integral:]
    else:
        text = '0.' + '0' * -integral + figures
    return (b'-' if value < 0 else b'') + text.encode('ascii')


def _rounded(magnitude: float, digits: int) -> tuple:
    """`magnitude` correctly rounded to `digits` significant digits, as (whole, point): whole * 10**point."""
    mantissa, exponent = ('%.*e' % (digits - 1, magnitude)).split('e')
    return int(mantissa.replace('.', '')), int(exponent) - digits + 1


def _nearest(magnitude: float, digits: int) -> tuple:
    """
    `magnitude` to `digits` significant digits as Rust prints them: the nearest, and of two equally near (the
    value's exact decimal ends in a 5 just past them) the one further from zero, where _rounded() takes the even.

    Returns:
        (whole, point): whole * 10**point.
    """
    exact = decimal.Decimal(magnitude)  # the f32's exact value: a binary fraction's decimal always ends
    figures = ''.join(map(str, exact.as_tuple().digits)).rstrip('0')
    if len(figures) == digits + 1 and figures[-1] == '5':
        return int(figures[:digits]) + 1, exact.adjusted() - digits + 1
    return _rounded(magnitude, digits)


def _reads_back(magnitude: float, bits: int, digits: int) -> bool:
    """
    Whether `magnitude`, an f32 away from a binade's first value, correctly rounded to `digits` significant digits
    reads back as itself. The digits are read through an f64, which decides it unless that f64 lies exactly
    halfway between two f32 values (it rounded twice); then the interval decides it exactly.

    Args:
        magnitude - the value, positive.
        bits - its bits.
        digits - the significant digits tried.

    Returns:
        True when those digits read back as the value.
    """
    whole, point = _rounded(magnitude, digits)
    read = float('%de%d' % (whole, point))
    try:
        single = struct.unpack('<f', struct.pack('<f', read))[0]
    except OverflowError:
        return False  # past the largest f32: these digits read as infinity
    if single != read and _halfway(read, single):
        return _inside(bits, whole * Fraction(10) ** point)
    return single == magnitude


def _halfway(read: float, single: float) -> bool:
    """
    True when the positive f64 `read`, which rounded to the f32 `single`, lies exactly halfway between `single`
    and its f32 neighbour on read's side. Two neighbouring f32 values sum without rounding in an f64, and
    doubling rounds nothing, so the comparison is exact.
    """
    bits = struct.unpack('<I', struct.pack('<f', single))[0] + (1 if read > single else -1)
    other = struct.unpack('<f', struct.pack('<I', bits))[0]
    return not math.isinf(other) and read * 2 == single + other


def _binade_first(magnitude: float, bits: int) -> tuple:
    """
    The digits a binade's first f32 value prints with: the fewest significant digits inside its interval, the
    nearest to it when several are (the one further from zero of two equally near), among the correctly rounded
    ones and their neighbours each side.

    Args:
        magnitude - the value, positive.
        bits - its bits.

    Returns:
        (whole, point): whole * 10**point.
    """
    exact = Fraction(magnitude)
    for digits in range(1, _FLOAT_DIGITS + 1):
        whole, point = _rounded(magnitude, digits)
        scale = Fraction(10) ** point
        inside = [candidate for candidate in (whole - 1, whole, whole + 1) if _inside(bits, candidate * scale)]
        if inside:  # the nearest; of two equally near, the one further from zero
            return min(inside, key=lambda candidate: (abs(candidate * scale - exact), -candidate)), point
    return _rounded(magnitude, _FLOAT_DIGITS)  # never: nine digits are always inside


def _inside(bits: int, amount: Fraction) -> bool:
    """
    Whether `amount` reads back as the positive finite f32 of `bits`, as Rust's float printing bounds it (core
    flt2dec decoder): halfway to each neighbour, both ends included when the significand is even (round half to
    even). A normal value whose significand is a binade's first has its lower neighbour twice as close, which
    Rust takes for every normal binade, its lowest included.

    Args:
        bits - the f32's bits.
        amount - a decimal, exactly.

    Returns:
        True when it lies inside the f32's interval.
    """
    exponent = bits >> 23
    significand = bits & (_FLOAT_SIGNIFICAND - 1) | (_FLOAT_SIGNIFICAND if exponent else 0)
    unit = Fraction(2) ** (max(exponent, 1) - 150)
    value = significand * unit
    below = unit / 2 if exponent and significand == _FLOAT_SIGNIFICAND else unit
    low, high = value - below / 2, value + unit / 2
    return low < amount < high or (significand % 2 == 0 and amount in (low, high))


def _cells(count: int, body: bytes) -> list:
    """
    A ROW's cells as text (crates/recorder record.rs `cells`, recorder_wire text.rs): a tag byte, then its value.
    A cell that runs past the record, an unknown tag, or a Text that is not UTF-8 ends the row (never written so).

    Args:
        count - the record's cell count.
        body - the record after its file name, padding included.

    Returns:
        [bytes], each cell's text.
    """
    cells, at = [], 0
    for _cell in range(count):
        if at >= len(body):
            break
        tag, at = body[at], at + 1
        size = _CELL_BYTES.get(tag)
        if tag == _TEXT_TAG and at < len(body):
            size, at = body[at], at + 1
        if size is None or at + size > len(body):
            break
        value, at = body[at : at + size], at + size
        if tag == _NONE_TAG:
            cells.append(b'None')
        elif tag in (_I32_TAG, _I64_TAG):
            cells.append(b'%d' % int.from_bytes(value, 'little', signed=True))
        elif tag == _F32_TAG:
            cells.append(_float_text(value))
        elif tag == _BOOL_TAG:
            cells.append(b'1' if value[0] else b'0')
        elif tag == _E7_TAG:
            units = int.from_bytes(value, 'little', signed=True)
            whole, fraction = divmod(abs(units), _UNITS_PER_DEGREE)
            cells.append(b'%s%d.%07d' % (b'-' if units < 0 else b'', whole, fraction))
        else:
            try:
                value.decode('utf-8')
            except UnicodeDecodeError:
                break
            cells.append(value.translate(_CELL_SPACES))
    return cells


def _log_text(uptime_us: int, body: bytes) -> bytes:
    """
    A LOG record's line, `<uptime_us> <descriptor> :: <message>` (record.rs `log`, recorder_wire `log_line`): a
    descriptor length byte, the descriptor, then the message to the zero padding. Text that is not UTF-8 (never
    written so) reads with U+FFFD, as Rust's lossy read; control bytes print as spaces.

    Args:
        uptime_us - the record's uptime.
        body - the record after its file name, padding included.

    Returns:
        The line's text.
    """
    length = body[0] if body else 0
    descriptor = body[1 : 1 + length] if len(body) >= 1 + length else b''
    message = body[1 + length :].rstrip(b'\x00') if len(body) >= 1 + length else b''
    descriptor, message = (part.decode('utf-8', 'replace').encode('utf-8') for part in (descriptor, message))
    return b'%d %s :: %s' % (uptime_us, descriptor.translate(_SPACES), message.translate(_SPACES))


def _classify(slot: bytes) -> tuple:
    """
    A slot's state by its bytes (a whole slot, or a cut one's bytes up to the cut): ('erased', None), ('block',
    header), ('other version', header) or ('foreign', None). Stricter than the board's scan, which reads headers
    only: an erased header over bytes that are not is foreign here, as the writer's check of the whole block finds
    it.
    """
    if slot == _ERASED * len(slot):
        return 'erased', None
    header = _block_header(slot)
    if header is None:
        return 'foreign', None
    return ('block' if header.version == _VERSION else 'other version'), header


def read_slots(image: bytes) -> tuple:
    """
    Classify a ring image's slots (_classify()) and order its blocks.

    Args:
        image - the partition's bytes from its offset 0: at most its 6144 slots, which main() checks. When it ends
            inside a slot, that slot is read up to there if a block header fits (see cut_note()).

    Returns:
        (slots, blocks): how many slots are in each state {'block', 'erased', 'foreign', 'other version': count},
        a cut slot too short for a header not counted; and every block of any layout version as (block sequence,
        slot, header, its bytes), in block-sequence order.
    """
    slots = dict.fromkeys(('block', 'erased', 'foreign', 'other version'), 0)
    blocks = []
    whole, rest = divmod(len(image), _BLOCK_BYTES)
    for slot in range(whole + (rest >= _BLOCK_HEADER.size)):
        data = image[slot * _BLOCK_BYTES : (slot + 1) * _BLOCK_BYTES]
        state, header = _classify(data)
        slots[state] += 1
        if header is not None:
            blocks.append((header.block_sequence, slot, header, data))
    return slots, sorted(blocks)


def read_image(image: bytes) -> tuple:
    """
    Read a ring image: classify its slots, then decode its blocks in block-sequence order.

    Args:
        image - the partition's bytes from its offset 0: at most its 6144 slots, which main() checks. When it ends
            inside a slot, that slot is read up to there if a block header fits (see cut_note()), and a chunk the
            cut runs through is no loss: the read ends there, not the boot.

    Returns:
        (boots, slots, others): the _Boot of each boot found, in block order; how many slots are in each state, as
        read_slots() counts them; and the runs of another version's blocks in sequence order, each [header of its
        first block, [block sequences], flight blocks].
    """
    slots, found = read_slots(image)
    whole, rest = divmod(len(image), _BLOCK_BYTES)
    cut = whole if rest else None  # the slot the read ends inside
    blocks = [block for block in found if block[2].version == _VERSION]
    others, owner = [], None
    for _sequence, _slot, header, _data in found:
        if header.version == _VERSION:
            continue
        if owner != (header.version, header.boot_id, header.session):  # the next build's, boot's or session's run
            owner = (header.version, header.boot_id, header.session)
            others.append([header, [], 0])
        others[-1][1].append(header.block_sequence)
        others[-1][2] += header.flight
    last = {(header.boot_id, header.session): index for index, (_sequence, _slot, header, _data) in enumerate(blocks)}
    boots, decoder = [], _Decoder()
    for index, (block_sequence, slot, header, data) in enumerate(blocks):
        identity = (header.boot_id, header.session)
        if not boots or boots[-1].identity != identity:
            boots.append(_Boot(header.boot_id, header.session, identity))
        boots[-1].blocks.append(block_sequence)
        boots[-1].flight += header.flight
        decoder.block(boots[-1], header, data, last[identity] > index or slot == cut)
    return boots, slots, others


def cut_note(image: bytes) -> str | None:
    """
    The summary line of an image that ends inside a slot, None for one that ends on a slot boundary. The slot is
    read up to the cut when a block header fits there, and not at all when not even that does.
    """
    whole, rest = divmod(len(image), _BLOCK_BYTES)
    if not rest:
        return None
    if rest < _BLOCK_HEADER.size:
        return 'cut: the image ends %d bytes into slot %d, before a block header fits: that slot is not read' % (
            rest,
            whole,
        )
    return 'cut: the image ends %d bytes into slot %d: that slot is read up to the cut, past which it is not' % (
        rest,
        whole,
    )


def ring_summary(image: bytes, slots: dict) -> list:
    """An image's head lines: its slots per state (read_slots()), then cut_note()'s line when it ends in a slot."""
    counts = ', '.join('%d %s' % (count, state) for state, count in slots.items())
    note = cut_note(image)
    return ['ring: %d slots: %s' % (sum(slots.values()), counts)] + ([] if note is None else [note])


def read_capture(capture: bytes) -> list:
    """
    Read a `GET /ring` dump: wire lines in block order, each boot's opening with session.csv's header and its
    `boot` row.

    A capture has no block headers, so its boots are told apart by what its lines name, and only a line that
    checks out names anything (_named()):
      * a session.csv row names its boot (boot id and session). A `boot` row starts its boot: it is the first row
        every boot writes. Any other row (`anchor`, a clock set) naming another boot starts that one too, unless
        the current boot is still unnamed and its routings carry the row's session: then the row names it. That
        is a dump that starts mid-boot (newest wins erased the boot's first blocks, and `GET /ring?boot=N` sends
        what is left), or a boot whose `boot` row is damaged;
      * a routed line names the session its routing opens with: another session than the current boot's starts a
        new boot, unnamed until a row names it. So a damaged `boot` row costs its boot the id up to its next row,
        never its lines;
      * session.csv's header opens a boot (the board's decoder prints it once per boot: before the `boot` row, or
        before the first row a mid-boot dump holds). It, and every line after it that names nothing, waits for
        the next line that does, and goes to the boot that line names.
    A line that names nothing goes to the current boot, and so do lines still waiting at the end; lines before
    anything names a boot (a dump that starts with log lines) form a boot of their own.

    Two boots under one reused label are told apart by their rows alone: when a damaged `boot` row hides the
    second one's start, its lines up to its next row count in the first. Both share the label's directory, so
    only the summary and --boot see it.

    Every line is filed as it came, as recorderd would: the board sent only whole chunks' lines, so one that does
    not check out is damage on the way, counted in `unproven`.

    Args:
        capture - the dump's bytes.

    Returns:
        The _Boot of each boot found, in order; [] when no line checks out (nothing valid). An unterminated last
        line, a transfer cut short, counts as lost.
    """
    pieces = capture.split(b'\n')
    boots, waiting, boot = [], [], None
    for piece in pieces[:-1]:
        line = piece + b'\n'
        routing, filed = demux(line)
        payload = recorder_wire.verify(routing, filed.rstrip(b'\n').decode('utf-8', 'surrogateescape'))
        opens = routing == recorder_wire.SESSION_INDEX and payload is not None and payload.startswith('uptime;')
        named = _named(routing, payload)
        waiting.append((line, payload is None))
        if opens or (waiting[:-1] and named is None):
            continue  # it waits for the line that names its boot
        if named is not None:
            boot = _boot_of(boots, boot, named)
        elif boot is None:
            boot = _Boot(None, None, None)
            boots.append(boot)
        boot.take(waiting)
        waiting = []
    if waiting:  # nothing after them names another boot
        if boot is None:
            boot = _Boot(None, None, None)
            boots.append(boot)
        boot.take(waiting)
    if boot is None or all(each.unproven == len(each.lines) for each in boots):
        return []
    boot.lost += 1 if pieces[-1] else 0
    for each in boots:
        if each.session is None:
            found = (_ROUTED_SESSION.match(demux(line)[0] or '') for line in each.lines)
            each.session = next((match.group(1) for match in found if match), 'unknown')
    return boots


def _named(routing: str | None, payload: str | None) -> tuple | None:
    """
    What a capture line names about its boot (read_capture()). A damaged line names nothing: its bytes are not
    the board's.

    Args:
        routing - the line's routing, None for a log line.
        payload - its payload when it checks out under that routing, else None.

    Returns:
        (identity, session, starts): for a session.csv row, its (boot id or None, session cell), the directory
        that names, and whether it is a `boot` row; for a routed line, (None, the session its routing opens
        with, False). None for a line that names nothing: a log line, a header, damage.
    """
    if payload is None or routing is None:
        return None
    if routing == recorder_wire.SESSION_INDEX:
        cells = payload.split(';')
        if len(cells) <= 2 or not cells[0].isdigit():
            return None  # a row opens with its uptime; the header with 'uptime'
        boot_id = int(cells[1]) if cells[1].isdigit() else None
        starts = len(cells) > _SOURCE_CELL and cells[_SOURCE_CELL] == _BOOT_SOURCE
        return (boot_id, cells[2]), _session_name(cells[2], boot_id), starts
    found = _ROUTED_SESSION.match(routing)
    return (None, found.group(1), False) if found else None


def _boot_of(boots: list, boot: _Boot | None, named: tuple) -> _Boot:
    """
    The boot a capture line that names one goes to, by read_capture()'s rules: the current one, which an unnamed
    boot's first row or routing names, or a new one, appended to `boots`.

    Args:
        boots - the boots found so far, in order.
        boot - the current boot, None before the first.
        named - what the line names, as _named() gives it.

    Returns:
        The boot it goes to.
    """
    identity, session, starts = named
    if boot is not None:
        if identity is None:  # a routed line: the session alone
            boot.session = session if boot.session is None else boot.session
            if boot.session == session:
                return boot
        elif identity == boot.identity:
            return boot
        elif boot.identity is None and not starts and boot.session in (None, session):
            boot.identity, boot.boot_id = identity, identity[0]
            boot.session = session if boot.session is None else boot.session
            return boot
    boot = _Boot(None if identity is None else identity[0], session, identity)
    boots.append(boot)
    return boot


def _session_name(session: str, boot_id: int | None) -> str | None:
    """A capture's `session` cell when it can name a directory; else the boot id as the board prints it, or None."""
    if _SESSION.fullmatch(session):
        return session
    return '%06d' % boot_id if boot_id is not None else None


def duplicates(boot: _Boot) -> int:
    """
    How many of a boot's lines the readers' duplicate filter drops (recorder_wire.unique): the lines of a chunk
    the writer programmed twice, or of a capture that holds them twice.
    """
    keyed = []
    for line in boot.lines:
        routing, filed = demux(line)
        keyed.append((routing, recorder_wire.verify(routing, filed.rstrip(b'\n').decode('utf-8', 'surrogateescape'))))
    return recorder_wire.unique(keyed).count(False)


def demuxed(boots: list) -> dict:
    """
    The boots' lines demuxed into session directories (see the module notes): what write() writes.

    Args:
        boots - the boots, in order; boots sharing a session share its files.

    Returns:
        {path relative to the output directory: its whole bytes}, in the order each file is first written.
    """
    files = {}
    for boot in boots:
        for line in boot.lines:
            routing, filed = demux(line)
            files.setdefault(os.path.join(boot.session, routing or 'recorder.log'), bytearray()).extend(filed)
            if routing == recorder_wire.SESSION_INDEX:
                files.setdefault(recorder_wire.SESSION_INDEX, bytearray()).extend(filed)
    return files


def write(files: dict, outdir: str) -> None:
    """
    Write each file whole under `outdir`, making the directories it needs: one that is there already is replaced,
    so main() first refuses a run that would replace one without --force.

    Args:
        files - {path relative to `outdir`: bytes}, as demuxed() gives them.
        outdir - the output directory, made when missing.

    Returns:
        None.
    """
    for relative, data in files.items():
        path = os.path.join(outdir, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as handle:
            handle.write(data)


def summary(boot: _Boot, image: bool) -> str:
    """One boot's summary line: what was read, and what the damage cost."""
    head = 'boot %s session %s:' % ('?' if boot.boot_id is None else boot.boot_id, boot.session)
    if not image:
        return '%s lines %d, unproven %d, lost %d, duplicates %d' % (
            head,
            len(boot.lines),
            boot.unproven,
            boot.lost,
            duplicates(boot),
        )
    missing = boot.blocks[-1] - boot.blocks[0] + 1 - len(set(boot.blocks))
    return (
        '%s blocks %d (#%d-#%d), missing blocks %d, flight blocks %d, chunks %d, records %d, lines %d, '
        'torn chunks %d, lost chunks %d, damaged chunks %d, duplicates %d'
        % (
            head,
            len(boot.blocks),
            boot.blocks[0],
            boot.blocks[-1],
            missing,
            boot.flight,
            boot.chunks,
            boot.records,
            len(boot.lines),
            boot.torn,
            boot.lost,
            boot.damaged,
            duplicates(boot),
        )
    )


def other_summary(header: _BlockHeader, sequences: list, flight: int) -> str:
    """One run of another layout version's blocks: whose they are, as their frozen header says, never read."""
    head = 'other version %d: boot %d session %s:' % (header.version, header.boot_id, header.session)
    blocks = 'blocks %d (#%d-#%d), flight blocks %d' % (len(sequences), sequences[0], sequences[-1], flight)
    return '%s %s, not read: this tool reads layout version %d' % (head, blocks, _VERSION)


def block_line(header: _BlockHeader, block: bytes) -> str:
    """
    One block's --blocks line (see the module notes): its header's sequence, boot, session and flight flag, then
    what its chunks hold, walked as the decoder walks them.

    Args:
        header - its header, of any layout version: the chunks of another version's block are not read.
        block - its bytes from offset 0.

    Returns:
        The line.
    """
    head = 'block #%d boot %d session %s %s:' % (
        header.block_sequence,
        header.boot_id,
        header.session,
        'flight' if header.flight else 'pad',
    )
    if header.version != _VERSION:
        return '%s layout version %d, not read: this tool reads layout version %d' % (head, header.version, _VERSION)
    rows, uptimes, torn, damaged = {}, [], 0, 0
    for walked in _chunks(block):
        if walked[0] == 'torn':
            torn += 1
            continue
        for record in _records(walked[3]):
            if record is None:
                damaged += 1
                break
            kind, uptime_us, name, _count, _body = record
            uptimes.append(uptime_us)
            if kind != _HEADER_KIND:
                rows[name] = rows.get(name, 0) + 1
    span = '%d-%d' % (min(uptimes), max(uptimes)) if uptimes else 'none'
    files = ' '.join(
        '%s=%d' % (name.translate(_SPACES).decode('ascii', 'backslashreplace'), count)
        for name, count in sorted(rows.items())
    )
    return '%s uptime %s, rows %s, torn chunks %d, damaged chunks %d' % (head, span, files or 'none', torn, damaged)


def list_blocks(image: bytes, boot_id: int | None) -> int:
    """
    --blocks: print the image's head lines (ring_summary()), then one line per block (block_line()) in
    block-sequence order. Nothing is written.

    Args:
        image - the partition's bytes from its offset 0, as main() checked them.
        boot_id - list the blocks whose header names this boot alone; None lists every block.

    Returns:
        0 when a block was listed; 1 when none was (an erased or foreign ring, or no block of that boot), said on
        stderr.
    """
    slots, blocks = read_slots(image)
    print('\n'.join(ring_summary(image, slots)))
    listed = [(header, data) for _sequence, _slot, header, data in blocks if boot_id in (None, header.boot_id)]
    for header, data in listed:
        print(block_line(header, data))
    if listed:
        return 0
    found = ', '.join(str(boot) for boot in sorted({header.boot_id for _sequence, _slot, header, _data in blocks}))
    wanted = '' if boot_id is None else ' for boot %d (boots found: %s)' % (boot_id, found or 'none')
    print('no block found' + wanted, file=sys.stderr)
    return 1


def main(arguments: list | None = None) -> int:
    """
    Command line: see the module notes.

    Args:
        arguments - the arguments after the program name; None reads sys.argv.

    Returns:
        0 on success, 1 when nothing this tool reads was found (or not the boot asked for; under --blocks, no
        block of any version). A usage error, an image longer than the partition and a file the run would replace
        without --force exit 2, before anything is written.
    """
    parser = argparse.ArgumentParser(description='Split a flash ring dump into session directories.')
    parser.add_argument('input', help='a raw ring image (GET /ring/image, esptool read_flash) or a GET /ring dump')
    parser.add_argument('outdir', nargs='?', help='where the session directories and the global session.csv go')
    parser.add_argument('--boot', type=int, help='keep only this boot id')
    parser.add_argument('--raw-capture', metavar='FILE', help="also write one boot's lines as a wire capture")
    parser.add_argument('--format', choices=('image', 'capture'), help='override the guess from the first byte')
    parser.add_argument('--force', action='store_true', help='replace the files that are in outdir already')
    parser.add_argument('--blocks', action='store_true', help="list an image's blocks, one line each; write nothing")
    options = parser.parse_intermixed_args(arguments)  # an outdir after --blocks still fills `outdir`
    if options.blocks and (options.outdir is not None or options.raw_capture or options.force):
        parser.error('--blocks lists the blocks and writes nothing: it takes no outdir, --raw-capture or --force')
    if not options.blocks and options.outdir is None:
        parser.error('the outdir is missing: only --blocks writes nothing')
    with open(options.input, 'rb') as handle:
        data = handle.read()
    image = (options.format or ('capture' if data[:1] in (b'@', b'{') else 'image')) == 'image'
    if options.blocks and not image:
        parser.error('--blocks reads a ring image, and a capture holds no blocks')
    if image and len(data) > _BLOCKS * _BLOCK_BYTES:
        parser.error(
            'the image is %d bytes, longer than the %d of the log partition: give the partition alone, as GET '
            '/ring/image or `esptool.py read_flash 0x800000 0x1800000 ring.bin` reads it (out of a whole-flash dump: '
            '`dd if=<dump> of=ring.bin bs=4096 skip=2048 count=6144`)' % (len(data), _BLOCKS * _BLOCK_BYTES)
        )
    if options.blocks:
        return list_blocks(data, options.boot)
    others = []
    if image:
        boots, slots, others = read_image(data)
        print('\n'.join(ring_summary(data, slots)))
        for header, sequences, flight in others:
            print(other_summary(header, sequences, flight))
    else:
        boots = read_capture(data)
        print('capture: %d bytes, %d boots' % (len(data), len(boots)))
    found = ', '.join(sorted({'?' if boot.boot_id is None else str(boot.boot_id) for boot in boots})) or 'none'
    if options.boot is not None:
        boots = [boot for boot in boots if boot.boot_id == options.boot]
    if not boots:
        wanted = '' if options.boot is None else ' for boot %d (boots found: %s)' % (options.boot, found)
        unread = [header for header, _sequences, _flight in others if options.boot in (None, header.boot_id)]
        if unread:
            wanted += '; blocks of layout version %s are not read by this tool (it reads %d)' % (
                ', '.join(sorted({str(header.version) for header in unread})),
                _VERSION,
            )
        print('nothing valid found' + wanted, file=sys.stderr)
        return 1
    if options.raw_capture and len(boots) > 1:
        parser.error(
            '--raw-capture takes one boot, and the input holds %d (%s): pick one with --boot' % (len(boots), found)
        )
    files = demuxed(boots)
    there = [path for path in files if os.path.lexists(os.path.join(options.outdir, path))]
    if there and not options.force:
        parser.error(
            '%s exists (%d of the files this run writes are there already): each is written whole, so a file of an '
            'SD pull would be cut down to what this input holds; split into another directory, or replace them '
            'with --force' % (os.path.join(options.outdir, there[0]), len(there))
        )
    for boot in boots:
        print(summary(boot, image))
    write(files, options.outdir)
    if options.raw_capture:
        with open(options.raw_capture, 'wb') as handle:
            handle.write(b''.join(boots[0].lines))
    print('wrote %d files under %s' % (len(files), options.outdir))
    return 0


if __name__ == '__main__':
    sys.exit(main())
