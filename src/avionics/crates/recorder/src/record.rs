//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The record (doc/specs/avionics-port.md "Producer side", the record format table): what a producer queues
//! instead of text, and what the ring_writer copies, verbatim, into a flash chunk. The same bytes sit in a
//! producer's PSRAM ring, in flash, and in a CC tee; only the ring_saver and the `cc` executor print them, so
//! `core::fmt` (53 us for a 20-field float row) stays off the flight core and off the flash path.
//!
//! Little-endian, 4-byte aligned:
//!
//! | off | size | field |
//! |---|---|---|
//! | 0 | u16 | length: the whole record with this header, its name and its padding, a multiple of 4. 0 is a PSRAM ring's wrap marker and never reaches flash |
//! | 2 | u8 | kind: 1 HEADER, 2 ROW, 3 LOG |
//! | 3 | u8 | flags: bit0 durable (its producer woke the writer, so it skipped the tick) |
//! | 4 | u8 | name length |
//! | 5 | u8 | count: a ROW's cells, else 0 |
//! | 6 | u16 | reserved, 0 |
//! | 8 | u64 | uptime_us (esp_timer, unwrapped) |
//! | 16 | ... | the destination file name, then the body |
//!
//! The destination file name is the stream's file without the session prefix (`imu_lsm6dso32.csv`), the bare
//! `session.csv`, or `recorder.log` for every LOG: the block header names the session, so a record never does.
//! The body:
//! * HEADER: the stream's column list, `uptime;<field>;...`, so the record reads as `<file>@<columns>` and every
//!   block reads without the registry, even under a later firmware;
//! * ROW: the cells, a tag byte then its value: 0 None, 1 I32, 2 I64, 3 F32, 4 Bool (one byte), 5 E7 (an i32 of
//!   1e-7 degrees), 6 Text (a length byte, then the bytes);
//! * LOG: a descriptor length byte, the descriptor, then the message.
//!
//! The column list and the LOG message run to the record's end, so their zero padding is stripped when read. To
//! keep that exact, a producer copies text with its control bytes made spaces (NUL among them, and CR and LF,
//! which would split a line), and a Text cell with its `;` made a space too, which would split a cell: the record
//! never holds what the printed line may not. A Text cell and a descriptor are cut to 255 bytes and a message to
//! 1 KiB, each on a character boundary, so every text reads back as valid UTF-8.
//!
//! A record never straddles chunks, so the longest is a chunk's payload, 4032 bytes ([`LONGEST`]).
//!
//! **Versioned with the chunks.** A block of another layout version is never decoded by this build
//! (flash_ring::header::VERSION covers the chunk and the record formats alike), so any change here bumps that
//! version. The test `the_record_format_moves_only_with_the_layout_version` pins one record of each kind, byte
//! for byte, to it.

use std::borrow::Cow;
use std::string::String;
use std::vec::Vec;

use flash_ring::chunk::MAX_PAYLOAD;
use recorder_wire::{Cell, SESSION_INDEX};

/// The record's fixed part, before its name.
pub(crate) const FIXED_BYTES: usize = 16;

/// The longest record, padding included: one whole chunk payload, since a record never straddles chunks.
pub(crate) const LONGEST: usize = MAX_PAYLOAD;

/// A Text cell's and a log descriptor's longest, in bytes: their length is one byte.
pub(crate) const TEXT_BYTES: usize = u8::MAX as usize;

/// A log message's longest, in bytes (doc/specs/avionics-port.md "Producer side").
pub(crate) const MESSAGE_BYTES: usize = 1024;

/// The file every LOG record goes to, under the session's directory.
pub(crate) const LOG_FILE: &str = "recorder.log";

/// The shortest file name the ring_saver files (recorderd's routing rule: at least 3 bytes).
const NAME_SHORTEST: usize = 3;

/// The longest file name the ring_saver files.
const NAME_LONGEST: usize = 64;

/// Records start on this alignment, and their lengths are multiples of it (the PSRAM ring's own).
const ALIGNMENT: usize = 4;

/// The flags bit of a durable record.
const DURABLE: u8 = 1;

/// What a record carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum Kind {
    /// A stream's column list: the CSV header line of its file.
    Header = 1,
    /// A telemetry row of typed cells.
    Row = 2,
    /// A recorder.log line's descriptor and message.
    Log = 3,
}

impl Kind {
    /// The kind a kind byte names; None for any other byte.
    fn from_byte(byte: u8) -> Option<Kind> {
        match byte {
            1 => Some(Kind::Header),
            2 => Some(Kind::Row),
            3 => Some(Kind::Log),
            _ => None,
        }
    }
}

/// The cell tags.
mod tag {
    /// No value: prints `None`.
    pub(super) const NONE: u8 = 0;
    /// An i32.
    pub(super) const I32: u8 = 1;
    /// An i64.
    pub(super) const I64: u8 = 2;
    /// An f32.
    pub(super) const F32: u8 = 3;
    /// A flag, one byte.
    pub(super) const BOOL: u8 = 4;
    /// 1e-7 degree units, an i32.
    pub(super) const E7: u8 = 5;
    /// A length byte, then that many bytes of UTF-8.
    pub(super) const TEXT: u8 = 6;
}

/// `length` rounded up to the alignment: the bytes a record of `length` takes.
pub(crate) const fn padded(length: usize) -> usize {
    length.next_multiple_of(ALIGNMENT)
}

/// Whether `name` is a file name the ring_saver files (`[A-Za-z0-9._-]{3,64}`): never a `/`, so never a path out
/// of the session's directory. A record whose name breaks it goes to recorder.log.
pub(crate) fn is_file_name(name: &[u8]) -> bool {
    (NAME_SHORTEST..=NAME_LONGEST).contains(&name.len())
        && name
            .iter()
            .all(|&byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

/// The longest prefix of `text` that is at most `limit` bytes and ends on a character boundary.
pub(crate) fn truncated(text: &str, limit: usize) -> &str {
    if text.len() <= limit {
        return text;
    }
    let mut end = limit;
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    &text[..end]
}

/// Whether a byte is an ASCII control (below 0x20, or DEL): never part of a recorded text.
fn is_control(byte: u8) -> bool {
    byte < 0x20 || byte == 0x7F
}

/// Whether a Text cell may not hold `byte`: a control byte, or the `;` that would split the cell.
fn breaks_cell(byte: u8) -> bool {
    byte == b';' || is_control(byte)
}

/// Copy `text` into `target` with every byte `replaced` names made a space: the same length, still UTF-8.
fn put_clean(target: &mut [u8], text: &[u8], replaced: fn(u8) -> bool) {
    for (byte, &source) in target.iter_mut().zip(text) {
        *byte = if replaced(source) { b' ' } else { source };
    }
}

/// The bytes one cell takes in a ROW record.
fn cell_bytes(cell: &Cell) -> usize {
    1 + match cell {
        Cell::None => 0,
        Cell::Bool(_) => 1,
        Cell::I32(_) | Cell::F32(_) | Cell::E7(_) => 4,
        Cell::I64(_) => 8,
        Cell::Text(text) => 1 + truncated(text, TEXT_BYTES).len(),
    }
}

/// The length of a HEADER record of `file` with `columns`.
pub(crate) fn header_length(file: &str, columns: &[u8]) -> usize {
    FIXED_BYTES + file.len() + columns.len()
}

/// The length of a ROW record of `file` with `cells`, or None when it cannot be one: more than 255 cells, or
/// longer than a chunk holds ([`LONGEST`]), so no flash block could ever take it.
pub(crate) fn row_length(file: &str, cells: &[Cell]) -> Option<usize> {
    let length = FIXED_BYTES + file.len() + cells.iter().map(cell_bytes).sum::<usize>();
    (cells.len() <= usize::from(u8::MAX) && padded(length) <= LONGEST).then_some(length)
}

/// A log record's descriptor and message, cut to their limits: the producer measures and writes the same.
pub(crate) fn log_parts<'a>(descriptor: &'a str, message: &'a str) -> (&'a str, &'a str) {
    (truncated(descriptor, TEXT_BYTES), truncated(message, MESSAGE_BYTES))
}

/// The length of a LOG record of these (already cut) parts. At most 1308 bytes, which every chunk holds.
pub(crate) fn log_length(descriptor: &str, message: &str) -> usize {
    FIXED_BYTES + LOG_FILE.len() + 1 + descriptor.len() + message.len()
}

/// Write the fixed part after the length field, and the file name behind it. Returns where the body starts.
fn write_fixed(record: &mut [u8], kind: Kind, durable: bool, file: &str, count: u8, uptime_us: u64) -> usize {
    record[2] = kind as u8;
    record[3] = if durable { DURABLE } else { 0 };
    record[4] = file.len() as u8;
    record[5] = count;
    record[6..8].fill(0);
    record[8..16].copy_from_slice(&uptime_us.to_le_bytes());
    let body = FIXED_BYTES + file.len();
    record[FIXED_BYTES..body].copy_from_slice(file.as_bytes());
    body
}

/// Write a HEADER record into `record`, [`header_length`] bytes long. The length field is the PSRAM ring's to
/// write ([`encode`] writes it for a record that never passes one).
pub(crate) fn write_header(record: &mut [u8], file: &str, durable: bool, uptime_us: u64, columns: &[u8]) {
    let body = write_fixed(record, Kind::Header, durable, file, 0, uptime_us);
    put_clean(&mut record[body..], columns, is_control);
}

/// Write a ROW record into `record`, [`row_length`] bytes long.
pub(crate) fn write_row(record: &mut [u8], file: &str, durable: bool, uptime_us: u64, cells: &[Cell]) {
    let mut at = write_fixed(record, Kind::Row, durable, file, cells.len() as u8, uptime_us);
    let mut put = |bytes: &[u8]| {
        record[at..at + bytes.len()].copy_from_slice(bytes);
        at += bytes.len();
    };
    for cell in cells {
        match *cell {
            Cell::None => put(&[tag::NONE]),
            Cell::I32(value) => {
                put(&[tag::I32]);
                put(&value.to_le_bytes());
            }
            Cell::I64(value) => {
                put(&[tag::I64]);
                put(&value.to_le_bytes());
            }
            Cell::F32(value) => {
                put(&[tag::F32]);
                put(&value.to_le_bytes());
            }
            Cell::Bool(value) => put(&[tag::BOOL, u8::from(value)]),
            Cell::E7(units) => {
                put(&[tag::E7]);
                put(&units.to_le_bytes());
            }
            Cell::Text(text) => {
                let text = truncated(text, TEXT_BYTES);
                put(&[tag::TEXT, text.len() as u8]);
                let mut clean = [0u8; TEXT_BYTES];
                put_clean(&mut clean, text.as_bytes(), breaks_cell);
                put(&clean[..text.len()]);
            }
        }
    }
}

/// Write a LOG record of these (already cut) parts into `record`, [`log_length`] bytes long.
pub(crate) fn write_log(record: &mut [u8], uptime_us: u64, descriptor: &str, message: &str) {
    let body = write_fixed(record, Kind::Log, false, LOG_FILE, 0, uptime_us);
    record[body] = descriptor.len() as u8;
    let start = body + 1;
    put_clean(
        &mut record[start..start + descriptor.len()],
        descriptor.as_bytes(),
        is_control,
    );
    put_clean(&mut record[start + descriptor.len()..], message.as_bytes(), is_control);
}

/// Append one whole record of `length` bytes to `target`, as a PSRAM ring would hold it: `fill` writes it after
/// its length field, then the length field is set and the zero padding added. For the writer's own records (the
/// session.csv `boot` and `anchor` rows, the header repeats, its status line), which never pass a producer ring.
pub(crate) fn encode(target: &mut Vec<u8>, length: usize, fill: impl FnOnce(&mut [u8])) {
    let start = target.len();
    let size = padded(length);
    target.resize(start + size, 0);
    fill(&mut target[start..start + length]);
    target[start..start + 2].copy_from_slice(&(size as u16).to_le_bytes());
}

/// A record read back: from a PSRAM ring, a flash chunk or a tee.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Record<'a> {
    /// What it carries.
    pub(crate) kind: Kind,
    /// Its uptime.
    pub(crate) uptime_us: u64,
    /// The whole record, padding included: what is copied on, verbatim.
    pub(crate) bytes: &'a [u8],
    /// The destination file name.
    name: &'a [u8],
    /// The ROW cell count.
    count: u8,
    /// Everything after the name, padding included.
    body: &'a [u8],
}

/// The bytes before the zero padding at the end of `bytes`.
fn unpadded(bytes: &[u8]) -> &[u8] {
    let end = bytes.iter().rposition(|&byte| byte != 0).map_or(0, |last| last + 1);
    &bytes[..end]
}

/// UTF-8 bytes from a record as text. A producer copied whole characters from a `str`, so this never replaces
/// anything; a damaged record still reads as text rather than stopping its reader.
fn text(bytes: &[u8]) -> Cow<'_, str> {
    String::from_utf8_lossy(bytes)
}

impl<'a> Record<'a> {
    /// The record at the start of `bytes`, which may run on past it (a chunk payload holds several). None when
    /// it is no record this crate writes: shorter than its fixed part or than its length field says, a length that
    /// is not a multiple of 4, an unknown kind, or a name running past the record.
    pub(crate) fn decode(bytes: &'a [u8]) -> Option<Record<'a>> {
        let fixed = bytes.get(..FIXED_BYTES)?;
        let length = usize::from(u16::from_le_bytes([fixed[0], fixed[1]]));
        let body = FIXED_BYTES + usize::from(fixed[4]);
        if length < FIXED_BYTES || !length.is_multiple_of(ALIGNMENT) || length > bytes.len() || body > length {
            return None;
        }
        Some(Record {
            kind: Kind::from_byte(fixed[2])?,
            uptime_us: u64::from_le_bytes(fixed[8..16].try_into().ok()?),
            bytes: &bytes[..length],
            name: &bytes[FIXED_BYTES..body],
            count: fixed[5],
            body: &bytes[body..length],
        })
    }

    /// The destination file name, as recorded.
    pub(crate) fn name(&self) -> &'a [u8] {
        self.name
    }

    /// A HEADER's column list.
    pub(crate) fn columns(&self) -> &'a [u8] {
        unpadded(self.body)
    }

    /// A ROW's cells, in order. A cell that runs past the record (never written so) ends the row.
    pub(crate) fn cells(&self) -> Vec<Cell<'a>> {
        let mut cells = Vec::with_capacity(usize::from(self.count));
        let body = self.body;
        let mut at = 0;
        let mut take = |length: usize| -> Option<&'a [u8]> {
            let bytes = body.get(at..at + length)?;
            at += length;
            Some(bytes)
        };
        let four = |bytes: &[u8]| [bytes[0], bytes[1], bytes[2], bytes[3]];
        for _ in 0..self.count {
            let Some(&[cell_tag]) = take(1) else { break };
            let cell = match cell_tag {
                tag::NONE => Some(Cell::None),
                tag::I32 => take(4).map(|bytes| Cell::I32(i32::from_le_bytes(four(bytes)))),
                tag::I64 => take(8).map(|bytes| Cell::I64(i64::from_le_bytes(bytes.try_into().unwrap_or([0; 8])))),
                tag::F32 => take(4).map(|bytes| Cell::F32(f32::from_le_bytes(four(bytes)))),
                tag::BOOL => take(1).map(|bytes| Cell::Bool(bytes[0] != 0)),
                tag::E7 => take(4).map(|bytes| Cell::E7(i32::from_le_bytes(four(bytes)))),
                tag::TEXT => take(1)
                    .and_then(|length| take(usize::from(length[0])))
                    .and_then(|bytes| core::str::from_utf8(bytes).ok())
                    .map(Cell::Text),
                _ => None,
            };
            match cell {
                Some(cell) => cells.push(cell),
                None => break,
            }
        }
        cells
    }

    /// A LOG's descriptor and message, as text.
    pub(crate) fn log(&self) -> (Cow<'a, str>, Cow<'a, str>) {
        let length = self.body.first().map_or(0, |&length| usize::from(length));
        let descriptor = self.body.get(1..1 + length).unwrap_or_default();
        let message = self.body.get(1 + length..).map(unpadded).unwrap_or_default();
        (text(descriptor), text(message))
    }

    /// Append the record's line text, before any wrapper (doc/specs/avionics-port.md "Row, log and index text"):
    /// a ROW `<uptime>;v1;...;vn`, a HEADER its column list, a LOG `<uptime_us> <descriptor> :: <message>`.
    pub(crate) fn text_into(&self, line: &mut Vec<u8>) {
        match self.kind {
            Kind::Header => line.extend(
                self.columns()
                    .iter()
                    .map(|&byte| if is_control(byte) { b' ' } else { byte }),
            ),
            Kind::Row => recorder_wire::row(line, self.uptime_us, &self.cells()),
            Kind::Log => {
                let (descriptor, message) = self.log();
                // the producer cut the descriptor and the message already: no further limit here
                recorder_wire::log_line(line, self.uptime_us, &descriptor, &message, usize::MAX);
            }
        }
    }

    /// The routing the record's line carries under `session`, written into `routing`: the bare `session.csv`
    /// for the session index, else `<session>_<file>`. None for a LOG, and for a record whose name the ring_saver
    /// would not file (doc/specs/avionics-port.md "The ring_saver"): both go to recorder.log, unrouted.
    pub(crate) fn routing<'s>(&self, session: &str, routing: &'s mut String) -> Option<&'s str> {
        if self.kind == Kind::Log || !is_file_name(self.name) {
            return None;
        }
        routing.clear();
        if self.name != SESSION_INDEX.as_bytes() {
            routing.push_str(session);
            routing.push('_');
        }
        // the name passed the file name rule, so it is ASCII
        routing.push_str(core::str::from_utf8(self.name).unwrap_or_default());
        Some(routing)
    }
}

/// The records of a chunk payload, in order. A record that does not decode (never written so: the chunk's CRC
/// checked) ends the walk, since its length cannot be trusted to find the next; [`Records::damaged`] then says so.
pub(crate) struct Records<'a> {
    /// The bytes not walked yet.
    rest: &'a [u8],
    /// Whether the walk stopped at bytes that are no record.
    damaged: bool,
}

impl<'a> Records<'a> {
    /// A walk over `payload`.
    pub(crate) fn new(payload: &'a [u8]) -> Records<'a> {
        Records {
            rest: payload,
            damaged: false,
        }
    }

    /// Whether the walk stopped at bytes that are no record, rather than at the payload's end.
    pub(crate) fn damaged(&self) -> bool {
        self.damaged
    }
}

impl<'a> Iterator for Records<'a> {
    type Item = Record<'a>;

    /// The next record, or None at the payload's end or at damage.
    fn next(&mut self) -> Option<Record<'a>> {
        if self.rest.is_empty() {
            return None;
        }
        match Record::decode(self.rest) {
            Some(record) => {
                self.rest = &self.rest[record.bytes.len()..];
                Some(record)
            }
            None => {
                self.damaged = true;
                self.rest = &[];
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use flash_ring::header::VERSION;

    /// A whole record as the ring would hold it.
    fn record(length: usize, fill: impl FnOnce(&mut [u8])) -> Vec<u8> {
        let mut bytes = Vec::new();
        encode(&mut bytes, length, fill);
        bytes
    }

    /// The pinned ROW's cells: one of each tag.
    fn pinned_cells() -> [Cell<'static>; 7] {
        [
            Cell::I32(-21),
            Cell::F32(0.5),
            Cell::None,
            Cell::Bool(true),
            Cell::E7(-803_917_950),
            Cell::I64(1 << 40),
            Cell::Text("ab"),
        ]
    }

    #[test]
    fn the_record_format_moves_only_with_the_layout_version() {
        // byte-exact, from the spec's table (computed apart, in Python): a change here must bump
        // flash_ring::header::VERSION, or an older build prints a newer build's records wrongly
        let header = record(header_length("imu.csv", b"uptime;ax"), |bytes| {
            write_header(bytes, "imu.csv", false, 884_029, b"uptime;ax")
        });
        let row = record(row_length("imu.csv", &pinned_cells()).unwrap(), |bytes| {
            write_row(bytes, "imu.csv", true, 884_029, &pinned_cells())
        });
        let (descriptor, message) = log_parts("seq", "go");
        let log = record(log_length(descriptor, message), |bytes| {
            write_log(bytes, 884_030, descriptor, message)
        });
        let pinned: [&[u8]; 3] = [
            &[
                0x20, 0x00, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x3D, 0x7D, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69,
                0x6D, 0x75, 0x2E, 0x63, 0x73, 0x76, 0x75, 0x70, 0x74, 0x69, 0x6D, 0x65, 0x3B, 0x61, 0x78,
            ],
            &[
                0x38, 0x00, 0x02, 0x01, 0x07, 0x07, 0x00, 0x00, 0x3D, 0x7D, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69,
                0x6D, 0x75, 0x2E, 0x63, 0x73, 0x76, 0x01, 0xEB, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x3F, 0x00,
                0x04, 0x01, 0x05, 0x82, 0x2F, 0x15, 0xD0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x06,
                0x02, 0x61, 0x62, 0x00, 0x00,
            ],
            &[
                0x24, 0x00, 0x03, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x3E, 0x7D, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x72,
                0x65, 0x63, 0x6F, 0x72, 0x64, 0x65, 0x72, 0x2E, 0x6C, 0x6F, 0x67, 0x03, 0x73, 0x65, 0x71, 0x67, 0x6F,
                0x00, 0x00,
            ],
        ];
        assert_eq!(
            (VERSION, [header.as_slice(), row.as_slice(), log.as_slice()]),
            (1, pinned),
            "the record format changed: bump flash_ring::header::VERSION, then update these vectors"
        );
    }

    #[test]
    fn every_kind_reads_back_and_prints_its_line() {
        let cells = pinned_cells();
        let row = record(row_length("imu.csv", &cells).unwrap(), |bytes| {
            write_row(bytes, "imu.csv", true, 884_029, &cells)
        });
        let read = Record::decode(&row).unwrap();
        assert_eq!(
            (read.kind, read.bytes[3], read.uptime_us, read.name(), read.bytes.len()),
            (Kind::Row, DURABLE, 884_029, b"imu.csv".as_slice(), 56)
        );
        assert_eq!(read.cells(), cells);
        let mut line = Vec::new();
        read.text_into(&mut line);
        assert_eq!(line, b"884029;-21;0.5;None;1;-80.3917950;1099511627776;ab");

        let header = record(header_length("imu.csv", b"uptime;ax"), |bytes| {
            write_header(bytes, "imu.csv", false, 7, b"uptime;ax")
        });
        let read = Record::decode(&header).unwrap();
        assert_eq!((read.kind, read.columns()), (Kind::Header, b"uptime;ax".as_slice()));
        line.clear();
        read.text_into(&mut line);
        assert_eq!(line, b"uptime;ax");

        let (descriptor, message) = log_parts("sequencer", "stage -> boosting");
        let log = record(log_length(descriptor, message), |bytes| {
            write_log(bytes, 884_030, descriptor, message)
        });
        let read = Record::decode(&log).unwrap();
        assert_eq!((read.kind, read.name()), (Kind::Log, b"recorder.log".as_slice()));
        line.clear();
        read.text_into(&mut line);
        assert_eq!(line, b"884030 sequencer :: stage -> boosting");
    }

    #[test]
    fn a_text_never_holds_what_its_line_may_not() {
        let cells = [Cell::Text("a;b\nc\0d")];
        let row = record(row_length("e.csv", &cells).unwrap(), |bytes| {
            write_row(bytes, "e.csv", false, 1, &cells)
        });
        assert_eq!(Record::decode(&row).unwrap().cells(), [Cell::Text("a b c d")]);
        // a message keeps its ';' and loses its line ends and NULs, which would read as the padding
        let (descriptor, message) = log_parts("x\ty", "one\r\ntwo;\0");
        let log = record(log_length(descriptor, message), |bytes| {
            write_log(bytes, 1, descriptor, message)
        });
        let (descriptor, message) = Record::decode(&log).unwrap().log();
        assert_eq!((&*descriptor, &*message), ("x y", "one  two; "));
        // an empty message, and messages ending where the padding starts
        for message in ["", "a", "ab", "abc", "abcd"] {
            let log = record(log_length("d", message), |bytes| write_log(bytes, 1, "d", message));
            assert_eq!(Record::decode(&log).unwrap().log().1, message);
        }
    }

    #[test]
    fn long_texts_are_cut_on_a_character_boundary() {
        let long = "é".repeat(200); // 400 bytes
        let cells = [Cell::Text(&long)];
        assert_eq!(
            row_length("e.csv", &cells),
            Some(FIXED_BYTES + 5 + 2 + 254),
            "255 would split a character"
        );
        let row = record(row_length("e.csv", &cells).unwrap(), |bytes| {
            write_row(bytes, "e.csv", false, 5, &cells)
        });
        assert_eq!(Record::decode(&row).unwrap().cells(), [Cell::Text(&long[..254])]);
        let (long_descriptor, long_message) = ("d".repeat(300), "m".repeat(2000));
        let (descriptor, message) = log_parts(&long_descriptor, &long_message);
        assert_eq!((descriptor.len(), message.len()), (TEXT_BYTES, MESSAGE_BYTES));
        assert!(
            padded(log_length(descriptor, message)) <= LONGEST,
            "every LOG fits a chunk"
        );
        assert_eq!(
            (truncated("aé", 2), truncated("abc", 3), truncated("", 0)),
            ("a", "abc", "")
        );
    }

    #[test]
    fn a_row_no_chunk_could_hold_is_no_record() {
        assert_eq!(row_length("e.csv", &[Cell::None; 256]), None, "the count is one byte");
        assert!(row_length("e.csv", &[Cell::None; 255]).is_some());
        let text = "x".repeat(TEXT_BYTES);
        // 16 + 5 + 15 x 257 = 3876 bytes fit a 4032-byte chunk, 16 cells (4133) do not
        assert!(row_length("e.csv", &vec![Cell::Text(&text); 15]).is_some());
        assert_eq!(row_length("e.csv", &vec![Cell::Text(&text); 16]), None);
    }

    #[test]
    fn a_record_routes_by_its_name_and_the_blocks_session() {
        let mut routing = String::new();
        let row = |name: &str| {
            record(row_length(name, &[]).unwrap(), |bytes| {
                write_row(bytes, name, false, 1, &[])
            })
        };
        let imu = row("imu_lsm6dso32.csv");
        let read = Record::decode(&imu).unwrap();
        assert_eq!(read.routing("000123", &mut routing), Some("000123_imu_lsm6dso32.csv"));
        let index = row("session.csv");
        let read = Record::decode(&index).unwrap();
        assert_eq!(read.routing("000123", &mut routing), Some("session.csv"), "bare");
        // negative: a LOG, and a name the saver would not file, go to recorder.log
        let log = record(log_length("d", "m"), |bytes| write_log(bytes, 1, "d", "m"));
        assert_eq!(Record::decode(&log).unwrap().routing("000123", &mut routing), None);
        for name in ["ab", "a/b.csv", "a b.csv", &"x".repeat(65)] {
            let bad = row(name);
            assert_eq!(
                Record::decode(&bad).unwrap().routing("000123", &mut routing),
                None,
                "{name:?}"
            );
        }
        assert!(is_file_name(&[b'x'; 64]) && is_file_name(b"a.b") && is_file_name(b"t-1_x.csv"));
    }

    #[test]
    fn a_damaged_record_reads_as_little_as_it_proves() {
        let header = record(header_length("imu.csv", b"uptime"), |bytes| {
            write_header(bytes, "imu.csv", false, 42, b"uptime")
        });
        assert!(Record::decode(&header[..15]).is_none(), "shorter than the fixed part");
        assert!(Record::decode(&header[..24]).is_none(), "shorter than its length field");
        let mut damaged = header.clone();
        damaged[2] = 9;
        assert!(Record::decode(&damaged).is_none(), "no such kind");
        let mut damaged = header.clone();
        damaged[0] = 30;
        assert!(Record::decode(&damaged).is_none(), "a length off the alignment");
        let mut damaged = header.clone();
        damaged[0] = 12;
        assert!(
            Record::decode(&damaged).is_none(),
            "a length shorter than the fixed part"
        );
        let mut damaged = header.clone();
        damaged[4] = 60;
        assert!(Record::decode(&damaged).is_none(), "a name past the record");
        // a row claiming more cells than it holds ends at the last whole one
        let cells = [Cell::I32(1), Cell::Bool(true)];
        let mut row = record(row_length("e.csv", &cells).unwrap(), |bytes| {
            write_row(bytes, "e.csv", false, 5, &cells)
        });
        row[5] = 9;
        assert_eq!(Record::decode(&row).unwrap().cells(), cells);
        let mut text = String::from("ok");
        text.push('\u{1F680}');
        let cells = [Cell::Text(&text)];
        let mut row = record(row_length("e.csv", &cells).unwrap(), |bytes| {
            write_row(bytes, "e.csv", false, 5, &cells)
        });
        row[FIXED_BYTES + 5 + 2 + 3] = 0xFF; // inside the rocket
        assert!(
            Record::decode(&row).unwrap().cells().is_empty(),
            "invalid UTF-8 ends the row"
        );
    }

    #[test]
    fn a_payload_walks_record_by_record_and_stops_at_damage() {
        let mut payload = Vec::new();
        encode(&mut payload, header_length("a.csv", b"uptime;x"), |bytes| {
            write_header(bytes, "a.csv", false, 1, b"uptime;x")
        });
        encode(&mut payload, row_length("a.csv", &[Cell::I32(2)]).unwrap(), |bytes| {
            write_row(bytes, "a.csv", false, 2, &[Cell::I32(2)])
        });
        let mut records = Records::new(&payload);
        let kinds: Vec<Kind> = records.by_ref().map(|record| record.kind).collect();
        assert_eq!((kinds, records.damaged()), (vec![Kind::Header, Kind::Row], false));
        payload.extend_from_slice(&[0; 8]);
        let mut records = Records::new(&payload);
        assert_eq!(records.by_ref().count(), 2);
        assert!(records.damaged(), "8 zero bytes are no record");
        assert_eq!(Records::new(&[]).count(), 0);
    }
}
