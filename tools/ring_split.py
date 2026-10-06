"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Split a flash ring dump of the Rust flight firmware (src/avionics) into session directories, the layout the SD
mover writes (doc/specs/avionics-port.md "SD and flash file layout"). It is the recovery path when the card
failed in flight, and the check before a forced ring discard: a session directory it writes is a drop-in for a
Luckfox /userdata/recordings dump, so assemble_capture, recorder_flight and flight_telemetry read it unchanged.

The input is one of two things:
  * a raw image of the `log` partition, `esptool.py read_flash 0x800000 0x1800000 ring.bin`, or any image that
    starts at the partition's offset 0 (a shorter read keeps the slots it holds). It is 384 blocks of 64 KiB;
    each block opens with a BOOT frame, and DATA and CONFIRM frames follow at 8-byte alignment
    (doc/specs/avionics-port.md "Flash frame format"; src/avionics/crates/recorder_wire/src/frame.rs is the codec);
  * a `GET /ring[?boot=N]` dump: the DATA lines of the valid frames in block order, already a wire capture.
A capture starts with a wire line's '@' or '{'. Anything else is an image, whatever its slot 0 holds (an
erased or foreign slot 0 is a normal ring). `--format` overrides the guess.

An image is read as the SD mover reads the ring ("Boot scan and recovery", "SD mover"):
  * Each slot is classified by its offset-0 header, as the boot scan does. A BOOT frame whose block sequence maps
    to the slot is a block, and all 0xFF is erased. A whole header of another layout version is kept apart,
    unread. Anything else is foreign: a torn BOOT, MicroPython-era data, a bench pattern.
  * The blocks are read in block-sequence order, which runs across the ring's wrap. In each, every frame's
    header CRC is checked, then its payload CRC.
  * A TORN FRAME (the header checks out, the payload does not) keeps each line whose wrapper checks out under
    the routing it is filed by, and the rest are lost.
  * A TORN HEADER: the next header is searched for on 8-byte alignment, and the lines between are salvaged the
    same way, from where that frame's payload would start.
  * A REPEAT (src/avionics/crates/recorder/src/writer.rs, on a failed program): a torn DATA frame followed in
    its boot by a DATA frame with the same uptime_us and payload length, whose payload starts with the torn
    frame's surviving bytes, was programmed again. The repeat is taken, and nothing of the torn frame. A program
    can also fail after its frame landed whole; the writer repeats it all the same, and the ring holds the frame
    twice. A whole DATA frame followed by the same bytes under the same uptime_us is that repeat, and is taken
    once: two different frames never share their first line.
  * Each damaged frame is noted in its session's recorder.log by a wrapped log line, as the mover notes it:
    `<uptime> ring :: frame <boot>/<sequence>: k salvaged, m lost`, or `... header <boot>/<block>+<offset>: ...`
    for a torn header, whose own sequence number is unknown.
A capture has no frames: its boots are told apart by session.csv's rows, which every boot writes first (the
header, then the `boot` row naming boot id and session).

Each line is then filed by recorderd's demux with the mover's routing rule (demux() below), into
`<outdir>/<session>/`, the session named by the block's BOOT frame:
  * a routed line goes to `<session>/<routing>` (`<session>_<stream>.csv`), as the bytes after the routing;
  * `session.csv` goes there and to the global `<outdir>/session.csv` too;
  * every other line goes to `<session>/recorder.log`.
Every file is written whole, so a second run into the same directory gives the same files.

    python3 tools/ring_split.py <input> <outdir> [--boot N] [--raw-capture FILE] [--format image|capture]

--boot N keeps that boot alone. --raw-capture FILE also writes one boot's lines in order, as a wire capture
for flight_telemetry: on an undamaged ring, its DATA payloads concatenated. It needs --boot when the input holds
several boots. A summary line is printed per boot. Exit 0 on success, 1 when nothing valid was found.
"""

import argparse
import binascii
import os
import re
import struct
import sys
from typing import NamedTuple

import recorder_wire

_MAGIC: bytes = b'\xf7CR1'  # 0xF7 never occurs in UTF-8, so no line holds it
_VERSION: int = 1
_DATA: int = 1
_BOOT: int = 2
_CONFIRM: int = 3
_HEADER_BYTES: int = 40
_HEADER_CRC_AT: int = 36  # the header CRC covers the bytes before it
_ALIGNMENT: int = 8
_BLOCK_BYTES: int = 64 << 10
_BLOCKS: int = 384  # the board's `log` partition: 24 MiB
_ERASED: bytes = b'\xff'
_HEADER: struct.Struct = struct.Struct('<4sBBBBIIIIQII')  # frame.rs: magic, version, kind, flags, reserved, ...
_BELOW_LINE_END: bytes = bytes(range(0x0A))  # recorderd drops these bytes as they arrive
_ROUTING: re.Pattern = re.compile(rb'[A-Za-z0-9._-]{3,64}')  # the mover's rule: a file name, never a path
_SESSION: re.Pattern = re.compile(r'[A-Za-z0-9_-]{1,64}')  # a boot id, a label or the dated prefix: a directory
_ROUTED_SESSION: re.Pattern = re.compile(r'(\d{8}_\d{6}_\d{6}|[A-Za-z0-9-]+)_.')  # the session a routing opens with


class _Header(NamedTuple):
    """A frame header's fields (frame.rs `Header`), with the payload CRC it states."""

    kind: int
    flags: int
    payload_length: int
    block_sequence: int
    boot_id: int
    frame_sequence: int
    uptime_us: int
    payload_crc: int


class _Boot:
    """
    One boot's share of the input, in order: the lines kept for its session directory, and what reading them
    found. An image tells two boots apart by their BOOT payloads, a capture by session.csv's (boot, session).
    """

    def __init__(self, boot_id: int | None, session: str | None, identity: object):
        self.boot_id: int | None = boot_id  # None: a capture boot no session.csv row named
        self.session: str | None = session  # its directory; None until a capture's lines name it
        self.identity: object = identity  # what a following block or row must match to stay in this boot
        self.lines: list = []  # the wire lines kept, each with its '\n': whole frames', salvaged and the notes
        self.blocks: list = []  # the block sequences read (an image)
        self.frames: int = 0  # frames whose header checks out
        self.first_frame: int | None = None  # the first frame number seen
        self.last_frame: int | None = None  # the last one
        self.missing: int = 0  # frame numbers skipped between those seen: frames lost
        self.whole: int = 0  # lines of whole frames (or of a capture)
        self.unproven: int = 0  # a capture's lines filed whose wrapper does not check out
        self.salvaged: int = 0  # lines of damaged frames that check out
        self.lost: int = 0  # lines of damaged frames that do not, and unterminated lines
        self.torn_frames: int = 0  # frames whose payload CRC failed
        self.torn_headers: int = 0  # places a header was expected and did not check out
        self.repeats: int = 0  # DATA frames programmed again after a failed program: taken once
        self.torn: tuple | None = None  # (header, payload) of a torn DATA frame its next DATA frame may repeat
        self.previous: tuple | None = None  # (header, payload) of the last whole DATA frame, which it may repeat


def demux(line: bytes) -> tuple:
    """
    Where recorderd files one complete line, under the SD mover's routing rule (doc/specs/avionics-port.md "SD
    mover"; recorder_wire::split in src/avionics is the same rule).

    recorderd drops every byte below 0x0A. A line starting with '@' whose next '@' sits at index 4 or later is
    routed: the routing is the bytes between, so it is at least 3 bytes long, and its file gets the bytes after
    that '@'. The mover adds one rule: a routing must match [A-Za-z0-9._-]{3,64}, so it names a file in the
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


def _proven(line: bytes) -> bool:
    """True when a complete line's wrapper checks out under the routing demux() files it by."""
    routing, filed = demux(line)
    return recorder_wire.verify(routing, filed.rstrip(b'\n').decode('utf-8', 'surrogateescape')) is not None


def _salvage(region: bytes) -> tuple:
    """
    The lines a damaged stretch of a block still proves.

    The erased tail is cut off first: a program stops part way, and what it never reached reads 0xFF. A line no
    byte of which was programmed is not counted at all, since nothing says it existed.

    Args:
        region - a torn frame's payload, or the bytes after a torn header up to the next header.

    Returns:
        ([line], lost): each complete line that checks out, '\\n' included, and how many pieces do not (a damaged
        line, or the one the tear cut).
    """
    pieces = region.rstrip(_ERASED).split(b'\n')
    kept, lost = [], 0
    for index, piece in enumerate(pieces):
        if not piece:
            continue
        if index < len(pieces) - 1 and _proven(piece + b'\n'):
            kept.append(piece + b'\n')
        else:
            lost += 1
    return kept, lost


def _whole_header(raw: bytes) -> bool:
    """True when `raw` is 40 bytes with the magic and a header CRC that checks out: a header, of any version."""
    return (
        len(raw) == _HEADER_BYTES
        and raw[:4] == _MAGIC
        and binascii.crc32(raw[:_HEADER_CRC_AT]) == int.from_bytes(raw[_HEADER_CRC_AT:], 'little')
    )


def _header(data: bytes, offset: int) -> _Header | None:
    """
    The frame header at `offset`, checked as frame.rs `Header::decode` checks it.

    Args:
        data - a block, or the image.
        offset - where the header starts.

    Returns:
        The header when its magic, header CRC, version and kind check out, else None.
    """
    raw = data[offset : offset + _HEADER_BYTES]
    if not _whole_header(raw):
        return None
    _magic, version, kind, flags, _reserved, *fields, _header_crc = _HEADER.unpack(raw)
    if version != _VERSION or kind not in (_DATA, _BOOT, _CONFIRM):
        return None
    return _Header(kind, flags, *fields)


def _classify(slot: int, raw: bytes) -> str:
    """
    A slot's state by its offset-0 header, as flash_ring's boot scan judges it (src/avionics/crates/flash_ring/
    src/scan.rs): 'block', 'erased', 'other version' or 'foreign'.
    """
    if raw == _ERASED * _HEADER_BYTES:
        return 'erased'
    header = _header(raw, 0)
    if header is not None and header.kind == _BOOT and header.block_sequence % _BLOCKS == slot:
        return 'block'
    if header is None and _whole_header(raw) and raw[4] != _VERSION:
        return 'other version'
    return 'foreign'


def _session_name(session: str, boot_id: int | None) -> str | None:
    """`session` when it can name a directory; else the boot id as the firmware prints it, or None."""
    if _SESSION.fullmatch(session):
        return session
    return '%06d' % boot_id if boot_id is not None else None


def _boot_fields(payload: bytes) -> dict:
    """A BOOT payload's fields: `boot=<id>;session=<prefix>;board=...;card=<vsn|none>` as {key: value}."""
    text = payload.decode('utf-8', 'surrogateescape')
    return dict(field.partition('=')[::2] for field in text.split(';'))


def _note(boot: _Boot, text: str, kept: list, lost: int) -> None:
    """
    Record a damaged frame's salvage: the note as a wrapped log line, then the lines it kept.

    Args:
        boot - the boot it belongs to.
        text - the note, `<uptime> ring :: ...`.
        kept - the salvaged lines.
        lost - how many pieces were lost.

    Returns:
        None; the lines go into `boot.lines`, the counts into its counters.
    """
    boot.lines.append(recorder_wire.wrap(text).encode('utf-8', 'surrogateescape'))
    boot.lines.extend(kept)
    boot.salvaged += len(kept)
    boot.lost += lost


def _settle(boot: _Boot) -> None:
    """Salvage the torn DATA frame waiting for its repeat, if any: nothing repeated it."""
    if boot.torn is None:
        return
    header, payload = boot.torn
    boot.torn = None
    kept, lost = _salvage(payload)
    text = '%d ring :: frame %d/%d: %d salvaged, %d lost' % (
        header.uptime_us,
        header.boot_id,
        header.frame_sequence,
        len(kept),
        lost,
    )
    _note(boot, text, kept, lost)


def _repeats(earlier: tuple, header: _Header, payload: bytes) -> bool:
    """
    True when a whole DATA frame repeats the `earlier` (header, payload), torn or whole (writer.rs): the same
    uptime and length, and the bytes that survived of it.
    """
    earlier_header, earlier_payload = earlier
    return (
        header.uptime_us == earlier_header.uptime_us
        and header.payload_length == earlier_header.payload_length
        and payload.startswith(earlier_payload.rstrip(_ERASED))
    )


def _frame(boot: _Boot, header: _Header, payload: bytes, whole: bool) -> None:
    """
    Take one frame whose header checks out.

    Args:
        boot - the boot it belongs to.
        header - its header.
        payload - its payload, cut at the block's end.
        whole - whether the payload CRC checks out.

    Returns:
        None; a whole DATA frame's lines go into `boot.lines`, unless it repeats the one before. A torn DATA frame
        waits in `boot.torn` until the next DATA frame says whether it was repeated.
    """
    boot.frames += 1
    if boot.first_frame is None:
        boot.first_frame = header.frame_sequence
    elif header.frame_sequence > boot.last_frame + 1:
        boot.missing += header.frame_sequence - boot.last_frame - 1
    boot.last_frame = header.frame_sequence
    if not whole:
        boot.torn_frames += 1
    if header.kind != _DATA:
        return  # BOOT and CONFIRM carry no lines, torn or not
    if not whole:
        _settle(boot)
        boot.torn, boot.previous = (header, payload), None
        return
    if boot.torn is not None and _repeats(boot.torn, header, payload):
        boot.repeats += 1
        boot.torn = None
    elif boot.previous is not None and _repeats(boot.previous, header, payload):
        boot.repeats += 1  # the earlier program failed after it landed whole: its lines are filed already
        return
    _settle(boot)
    boot.previous = (header, payload)
    pieces = payload.split(b'\n')
    boot.lines.extend(piece + b'\n' for piece in pieces[:-1])
    boot.whole += len(pieces) - 1
    if pieces[-1]:
        boot.lost += 1  # an unterminated line, which the writer never cuts


def _next_header(block: bytes, start: int, block_sequence: int, boot_id: int) -> int | None:
    """The offset of the first header of this block on 8-byte alignment from `start` (aligned), or None."""
    found = block.find(_MAGIC, start)
    while found >= 0:
        header = _header(block, found) if found % _ALIGNMENT == 0 else None
        if header is not None and (header.block_sequence, header.boot_id) == (block_sequence, boot_id):
            return found
        found = block.find(_MAGIC, found + 1)
    return None


def _walk(boot: _Boot, block_sequence: int, block: bytes) -> None:
    """
    Read one block's frames into `boot`, from its BOOT frame at offset 0 to the erased tail.

    A frame belongs to the block when its header checks out and names the block and its boot. Anywhere else a
    header is expected, it is torn, and the next one is searched for (see the module notes).
    """
    end = len(block.rstrip(_ERASED))  # past it the block is erased
    offset, uptime_us = 0, 0
    while offset < end and offset + _HEADER_BYTES <= len(block):
        header = _header(block, offset)
        if header is not None and (header.block_sequence, header.boot_id) == (block_sequence, boot.boot_id):
            stop = offset + _HEADER_BYTES + header.payload_length
            payload = block[offset + _HEADER_BYTES : stop]
            _frame(boot, header, payload, stop <= len(block) and binascii.crc32(payload) == header.payload_crc)
            uptime_us = header.uptime_us
            offset = -(-stop // _ALIGNMENT) * _ALIGNMENT
            continue
        following = _next_header(block, offset + _ALIGNMENT, block_sequence, boot.boot_id)
        _settle(boot)
        boot.previous = None
        kept, lost = _salvage(block[offset + _HEADER_BYTES : following])
        boot.torn_headers += 1
        text = '%d ring :: header %d/%d+%d: %d salvaged, %d lost' % (
            uptime_us,
            boot.boot_id,
            block_sequence,
            offset,
            len(kept),
            lost,
        )
        _note(boot, text, kept, lost)
        if following is None:
            break
        offset = following


def read_image(image: bytes) -> tuple:
    """
    Read a ring image: classify its slots, then walk its blocks in block-sequence order.

    Args:
        image - the partition's bytes from its offset 0; slots past the 384th are ignored.

    Returns:
        (boots, slots): the _Boot of each boot found, in block order, and how many slots are in each state
        {'block', 'erased', 'foreign', 'other version': count}.
    """
    slots = dict.fromkeys(('block', 'erased', 'foreign', 'other version'), 0)
    blocks = []
    for slot in range(min(_BLOCKS, -(-len(image) // _BLOCK_BYTES))):
        start = slot * _BLOCK_BYTES
        state = _classify(slot, image[start : start + _HEADER_BYTES])
        slots[state] += 1
        if state == 'block':
            blocks.append((_header(image, start).block_sequence, image[start : start + _BLOCK_BYTES]))
    boots, boot = [], None
    for block_sequence, block in sorted(blocks):
        header = _header(block, 0)
        payload = block[_HEADER_BYTES : _HEADER_BYTES + header.payload_length]
        whole = binascii.crc32(payload) == header.payload_crc and len(payload) == header.payload_length
        identity = payload if whole else None  # a torn BOOT payload names no boot: it stays in the one before
        if boot is None or boot.boot_id != header.boot_id or identity not in (None, boot.identity):
            if boot is not None:
                _settle(boot)
            session = _boot_fields(payload).get('session', '') if whole else ''
            boot = _Boot(header.boot_id, _session_name(session, header.boot_id), identity)
            boots.append(boot)
        boot.blocks.append(block_sequence)
        _walk(boot, block_sequence, block)
    if boot is not None:
        _settle(boot)
    return boots, slots


def read_capture(capture: bytes) -> list:
    """
    Read a `GET /ring` dump: wire lines, every boot's opening with session.csv's header and `boot` row.

    A session.csv row that checks out and names another (boot, session) starts a new boot, and the header just
    before it goes with it. Lines before any such row (a dump that starts mid-boot) form a boot of their own,
    named by the session their routings open with.

    Every line is filed as it came, as recorderd would: the board sent only whole frames' lines, so one that does
    not check out is damage on the way, counted in `unproven`.

    Args:
        capture - the dump's bytes.

    Returns:
        The _Boot of each boot found, in order; [] when no line checks out (nothing valid). An unterminated last
        line, a transfer cut short, counts as lost.
    """
    pieces = capture.split(b'\n')
    boots, held, boot = [], [], None
    for piece in pieces[:-1]:
        line = piece + b'\n'
        routing, filed = demux(line)
        payload = recorder_wire.verify(routing, filed.rstrip(b'\n').decode('utf-8', 'surrogateescape'))
        identity = None
        if routing == recorder_wire.SESSION_INDEX and payload is not None:
            if payload.startswith('uptime;'):
                held.append(line)  # the header goes with the row after it
                continue
            cells = payload.split(';')
            if len(cells) > 2:
                identity = (int(cells[1]) if cells[1].isdigit() else None, cells[2])
        if boot is None or (identity is not None and identity != boot.identity):
            boot_id, session = identity if identity is not None else (None, None)
            boot = _Boot(boot_id, None if session is None else _session_name(session, boot_id), identity)
            boots.append(boot)
        boot.lines.extend(held + [line])
        boot.whole += len(held) + 1
        boot.unproven += payload is None
        held = []
    if held:  # session.csv headers at the very end, with no row after them
        if boot is None:
            boot = _Boot(None, None, None)
            boots.append(boot)
        boot.lines.extend(held)
        boot.whole += len(held)
    if boot is None or all(boot.unproven == boot.whole for boot in boots):
        return []
    boot.lost += 1 if pieces[-1] else 0
    for boot in boots:
        if boot.session is None:
            found = (_ROUTED_SESSION.match(demux(line)[0] or '') for line in boot.lines)
            boot.session = next((match.group(1) for match in found if match), 'unknown')
    return boots


def write(boots: list, outdir: str) -> list:
    """
    Demux the boots' lines into session directories under `outdir` (see the module notes).

    Args:
        boots - the boots to write, in order; boots sharing a session share its files.
        outdir - the output directory, made when missing.

    Returns:
        The files written, as paths relative to `outdir`, in the order first written.
    """
    files = {}
    for boot in boots:
        for line in boot.lines:
            routing, filed = demux(line)
            files.setdefault(os.path.join(boot.session, routing or 'recorder.log'), bytearray()).extend(filed)
            if routing == recorder_wire.SESSION_INDEX:
                files.setdefault(recorder_wire.SESSION_INDEX, bytearray()).extend(filed)
    for relative, data in files.items():
        path = os.path.join(outdir, relative)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as handle:
            handle.write(data)
    return list(files)


def summary(boot: _Boot, image: bool) -> str:
    """One boot's summary line: what was read, and what the damage cost."""
    head = 'boot %s session %s:' % ('?' if boot.boot_id is None else boot.boot_id, boot.session)
    if not image:
        return '%s lines %d, unproven %d, lost %d' % (head, boot.whole, boot.unproven, boot.lost)
    return (
        '%s blocks %d (#%d-#%d), frames %d (#%d-#%d), lines %d, salvaged %d, lost %d, torn frames %d, '
        'repeats %d, torn headers %d, missing frames %d'
        % (
            head,
            len(boot.blocks),
            boot.blocks[0],
            boot.blocks[-1],
            boot.frames,
            boot.first_frame,
            boot.last_frame,
            boot.whole,
            boot.salvaged,
            boot.lost,
            boot.torn_frames,
            boot.repeats,
            boot.torn_headers,
            boot.missing,
        )
    )


def main(arguments: list | None = None) -> int:
    """
    Command line: see the module notes.

    Args:
        arguments - the arguments after the program name; None reads sys.argv.

    Returns:
        0 on success, 1 when nothing valid was found (or not the boot asked for). A usage error exits 2.
    """
    parser = argparse.ArgumentParser(description='Split a flash ring dump into session directories.')
    parser.add_argument('input', help='a raw ring image (esptool read_flash 0x800000 0x1800000) or a GET /ring dump')
    parser.add_argument('outdir', help='where the session directories and the global session.csv go')
    parser.add_argument('--boot', type=int, help='keep only this boot id')
    parser.add_argument('--raw-capture', metavar='FILE', help="also write one boot's lines as a wire capture")
    parser.add_argument('--format', choices=('image', 'capture'), help='override the guess from the first byte')
    options = parser.parse_args(arguments)
    with open(options.input, 'rb') as handle:
        data = handle.read()
    image = (options.format or ('capture' if data[:1] in (b'@', b'{') else 'image')) == 'image'
    if image:
        boots, slots = read_image(data)
        counts = ', '.join('%d %s' % (count, state) for state, count in slots.items())
        print('ring: %d slots: %s' % (sum(slots.values()), counts))
    else:
        boots = read_capture(data)
        print('capture: %d bytes, %d boots' % (len(data), len(boots)))
    found = ', '.join(sorted({'?' if boot.boot_id is None else str(boot.boot_id) for boot in boots})) or 'none'
    if options.boot is not None:
        boots = [boot for boot in boots if boot.boot_id == options.boot]
    if not boots:
        wanted = '' if options.boot is None else ' for boot %d (boots found: %s)' % (options.boot, found)
        print('nothing valid found' + wanted, file=sys.stderr)
        return 1
    if options.raw_capture and len(boots) > 1:
        parser.error(
            '--raw-capture takes one boot, and the input holds %d (%s): pick one with --boot' % (len(boots), found)
        )
    for boot in boots:
        print(summary(boot, image))
    written = write(boots, options.outdir)
    if options.raw_capture:
        with open(options.raw_capture, 'wb') as handle:
            handle.write(b''.join(boots[0].lines))
    print('wrote %d files under %s' % (len(written), options.outdir))
    return 0


if __name__ == '__main__':
    sys.exit(main())
