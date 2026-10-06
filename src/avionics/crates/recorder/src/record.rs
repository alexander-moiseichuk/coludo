//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring record (doc/specs/avionics-port.md "Producer side", the ring record format table): what a producer
//! queues instead of text, so `core::fmt` (53 us for a 20-field float row) stays off the flight core. The flash
//! writer formats it later.
//!
//! Little-endian, 4-byte aligned:
//!
//! | off | size | field |
//! |---|---|---|
//! | 0 | u16 | length, the whole record padded to 4 (the byte ring writes it; 0 is its wrap marker) |
//! | 2 | u8 | kind: 1 HEADER, 2 ROW, 3 LOG, 4 LINE |
//! | 3 | u8 | flags: bit0 durable (the writer cuts a frame right after it) |
//! | 4 | u16 | stream (HEADER, ROW, LINE), or the descriptor id (LOG; always 0, the descriptor inline) |
//! | 6 | u8 | count: the cells of a ROW |
//! | 7 | u8 | reserved, 0 |
//! | 8 | u64 | uptime_us |
//! | 16 | ... | ROW: cells, a tag byte then its value; LOG: u8 descriptor length, descriptor, message; LINE: the text |
//!
//! Cell tags: 0 None, 1 I32, 2 I64, 3 F32, 4 Bool (one byte), 5 E7 (i32), 6 Text (u8 length, then bytes).
//!
//! A LOG message and a LINE run to the record's end, so their trailing zero padding is stripped when read. To
//! keep that exact, the producer copies them with every NUL made a space, as the writer would print a control
//! byte anyway. A Text cell and a log descriptor are cut to 255 bytes and a message to 1 KiB, each on a
//! character boundary, so the writer reads back valid UTF-8.

use recorder_wire::Cell;

use crate::byte_ring::MAX_RECORD_BYTES;

/// The record's fixed part, before its body.
pub(crate) const HEADER_BYTES: usize = 16;

/// A Text cell's and a log descriptor's longest, in bytes: their length is one byte.
pub(crate) const TEXT_BYTES: usize = u8::MAX as usize;

/// A log message's longest, in bytes (doc/specs/avionics-port.md "Producer side").
pub(crate) const MESSAGE_BYTES: usize = 1024;

/// The flags bit of a durable record.
const DURABLE: u8 = 1;

/// The descriptor id of a log record whose descriptor is inline: the only kind written.
const INLINE_DESCRIPTOR: u16 = 0;

/// What a record carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum Kind {
    /// A stream's CSV header: the writer prints it from the registry.
    Header = 1,
    /// A telemetry row of typed cells.
    Row = 2,
    /// A recorder.log line's descriptor and message.
    Log = 3,
    /// A preformatted routed line, such as a session.csv row.
    Line = 4,
}

/// The cell tags.
mod tag {
    /// No value.
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

/// The length of a ROW record of `cells`, or None when it cannot be one: more than 255 cells, or larger than a
/// ring record may be.
pub(crate) fn row_length(cells: &[Cell]) -> Option<usize> {
    let length = HEADER_BYTES + cells.iter().map(cell_bytes).sum::<usize>();
    (cells.len() <= usize::from(u8::MAX) && length <= MAX_RECORD_BYTES).then_some(length)
}

/// The most bytes one cell prints as (recorder_wire's row text), whatever its value: the producer must not
/// format the row to learn its width, so it takes the worst case of its kind.
fn cell_width(cell: &Cell) -> usize {
    match cell {
        // `None`
        Cell::None => 4,
        // `0` or `1`
        Cell::Bool(_) => 1,
        // -2147483648
        Cell::I32(_) => 11,
        // -9223372036854775808
        Cell::I64(_) => 20,
        // -1e-45 as Display writes it, never with an exponent: `-0.` and 45 more digits
        Cell::F32(_) => 48,
        // -214.7483648
        Cell::E7(_) => 12,
        // made one cell byte for byte, as the record cut it
        Cell::Text(text) => truncated(text, TEXT_BYTES).len(),
    }
}

/// The most bytes the row text of `cells` prints as: a u64 uptime of 20 digits and `;`, then the cells with a `;`
/// between each two.
pub(crate) fn row_width(cells: &[Cell]) -> usize {
    21 + cells.iter().map(cell_width).sum::<usize>() + cells.len().saturating_sub(1)
}

/// Write the fixed part after the length field.
fn write_head(record: &mut [u8], kind: Kind, durable: bool, stream: u16, count: u8, uptime_us: u64) {
    record[2] = kind as u8;
    record[3] = if durable { DURABLE } else { 0 };
    record[4..6].copy_from_slice(&stream.to_le_bytes());
    record[6] = count;
    record[7] = 0;
    record[8..16].copy_from_slice(&uptime_us.to_le_bytes());
}

/// Write a HEADER record (exactly [`HEADER_BYTES`] long).
pub(crate) fn write_header(record: &mut [u8], stream: u16, durable: bool, uptime_us: u64) {
    write_head(record, Kind::Header, durable, stream, 0, uptime_us);
}

/// Write a ROW record into `record`, [`row_length`] bytes long.
pub(crate) fn write_row(record: &mut [u8], stream: u16, durable: bool, uptime_us: u64, cells: &[Cell]) {
    write_head(record, Kind::Row, durable, stream, cells.len() as u8, uptime_us);
    let mut at = HEADER_BYTES;
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
                put(text.as_bytes());
            }
        }
    }
}

/// A log record's descriptor and message, cut to their limits: the producer measures and writes the same.
pub(crate) fn log_parts<'a>(descriptor: &'a str, message: &'a str) -> (&'a str, &'a str) {
    (truncated(descriptor, TEXT_BYTES), truncated(message, MESSAGE_BYTES))
}

/// The length of a LOG record of these (already cut) parts.
pub(crate) fn log_length(descriptor: &str, message: &str) -> usize {
    HEADER_BYTES + 1 + descriptor.len() + message.len()
}

/// Copy `text` with every NUL made a space: a NUL would read as the record's padding.
fn put_text(target: &mut [u8], text: &[u8]) {
    for (byte, &source) in target.iter_mut().zip(text) {
        *byte = if source == 0 { b' ' } else { source };
    }
}

/// Write a LOG record of these (already cut) parts, [`log_length`] bytes long.
pub(crate) fn write_log(record: &mut [u8], uptime_us: u64, descriptor: &str, message: &str) {
    write_head(record, Kind::Log, false, INLINE_DESCRIPTOR, 0, uptime_us);
    record[HEADER_BYTES] = descriptor.len() as u8;
    let start = HEADER_BYTES + 1;
    put_text(&mut record[start..start + descriptor.len()], descriptor.as_bytes());
    put_text(&mut record[start + descriptor.len()..], message.as_bytes());
}

/// The length of a LINE record carrying `text`.
pub(crate) fn line_length(text: &[u8]) -> usize {
    HEADER_BYTES + text.len()
}

/// Write a LINE record carrying `text`, [`line_length`] bytes long.
pub(crate) fn write_line(record: &mut [u8], stream: u16, durable: bool, uptime_us: u64, text: &[u8]) {
    write_head(record, Kind::Line, durable, stream, 0, uptime_us);
    put_text(&mut record[HEADER_BYTES..], text);
}

/// A record read back from a ring.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Record<'a> {
    /// What it carries.
    pub(crate) kind: Kind,
    /// Whether the writer cuts a frame right after it.
    pub(crate) durable: bool,
    /// Its stream (HEADER, ROW, LINE).
    pub(crate) stream: u16,
    /// Its uptime.
    pub(crate) uptime_us: u64,
    /// The ROW cell count.
    count: u8,
    /// Everything after the fixed part, padding included.
    body: &'a [u8],
}

/// The bytes before the zero padding at the end of `bytes`.
fn unpadded(bytes: &[u8]) -> &[u8] {
    let end = bytes.iter().rposition(|&byte| byte != 0).map_or(0, |last| last + 1);
    &bytes[..end]
}

/// UTF-8 bytes from a record as text. The producer copied whole characters from a `str`, so this never
/// replaces anything; a damaged record still reads as text rather than stopping the writer.
fn text(bytes: &[u8]) -> std::borrow::Cow<'_, str> {
    std::string::String::from_utf8_lossy(bytes)
}

impl<'a> Record<'a> {
    /// Read a record as the ring returns it (length field included). None when it is too short or of no kind
    /// this crate writes: never, for a ring only this crate's producers fill.
    pub(crate) fn decode(record: &'a [u8]) -> Option<Record<'a>> {
        if record.len() < HEADER_BYTES {
            return None;
        }
        let kind = match record[2] {
            1 => Kind::Header,
            2 => Kind::Row,
            3 => Kind::Log,
            4 => Kind::Line,
            _ => return None,
        };
        Some(Record {
            kind,
            durable: record[3] & DURABLE != 0,
            stream: u16::from_le_bytes([record[4], record[5]]),
            count: record[6],
            uptime_us: u64::from_le_bytes(record[8..16].try_into().ok()?),
            body: &record[HEADER_BYTES..],
        })
    }

    /// A ROW's cells, in order. A cell that runs past the record (never written so) ends the row.
    pub(crate) fn cells(&self) -> std::vec::Vec<Cell<'a>> {
        let mut cells = std::vec::Vec::with_capacity(usize::from(self.count));
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
    pub(crate) fn log(&self) -> (std::borrow::Cow<'a, str>, std::borrow::Cow<'a, str>) {
        let length = self.body.first().map_or(0, |&length| usize::from(length));
        let descriptor = self.body.get(1..1 + length).unwrap_or_default();
        let message = self.body.get(1 + length..).map(unpadded).unwrap_or_default();
        (text(descriptor), text(message))
    }

    /// A LINE's text.
    pub(crate) fn line(&self) -> &'a [u8] {
        unpadded(self.body)
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HEADER_BYTES, Kind, MESSAGE_BYTES, Record, TEXT_BYTES, line_length, log_length, log_parts, row_length,
        row_width, truncated, write_header, write_line, write_log, write_row,
    };
    use crate::byte_ring::{MAX_RECORD_BYTES, padded};
    use recorder_wire::Cell;
    use std::string::String;
    use std::vec::Vec;

    /// A record buffer as the ring hands one to the producer: `length` bytes inside a zeroed padded slot.
    fn slot(length: usize, write: impl FnOnce(&mut [u8])) -> Vec<u8> {
        let mut record = vec![0; padded(length)];
        write(&mut record[..length]);
        record
    }

    #[test]
    fn a_row_keeps_every_cell_kind() {
        let cells = [
            Cell::None,
            Cell::I32(-21),
            Cell::I64(i64::MIN),
            Cell::F32(0.982_831_9),
            Cell::F32(f32::NAN),
            Cell::Bool(true),
            Cell::Bool(false),
            Cell::E7(-803_917_950),
            Cell::Text("boosting"),
            Cell::Text(""),
        ];
        let length = row_length(&cells).unwrap();
        assert_eq!(length, HEADER_BYTES + 1 + 5 + 9 + 5 + 5 + 2 + 2 + 5 + 10 + 2);
        let bytes = slot(length, |record| write_row(record, 7, true, 884_029, &cells));
        let record = Record::decode(&bytes).unwrap();
        assert_eq!(
            (record.kind, record.durable, record.stream, record.uptime_us),
            (Kind::Row, true, 7, 884_029)
        );
        let back = record.cells();
        assert_eq!(back.len(), cells.len());
        for (back, cell) in back.iter().zip(&cells) {
            match (back, cell) {
                (Cell::F32(back), Cell::F32(cell)) => assert_eq!(back.to_bits(), cell.to_bits()),
                _ => assert_eq!(back, cell),
            }
        }
    }

    #[test]
    fn a_long_text_is_cut_on_a_character_boundary() {
        let long = "é".repeat(200); // 400 bytes
        let cells = [Cell::Text(&long)];
        assert_eq!(
            row_length(&cells),
            Some(HEADER_BYTES + 2 + 254),
            "255 would split a character"
        );
        let bytes = slot(row_length(&cells).unwrap(), |record| {
            write_row(record, 1, false, 5, &cells)
        });
        assert_eq!(Record::decode(&bytes).unwrap().cells(), [Cell::Text(&long[..254])]);
        assert_eq!(truncated("abc", 2), "ab");
        assert_eq!(truncated("abc", 3), "abc");
        assert_eq!(truncated("aé", 2), "a");
        assert_eq!(truncated("", 0), "");
    }

    #[test]
    fn a_row_past_the_limits_is_no_record() {
        assert_eq!(row_length(&[Cell::None; 256]), None, "the count is one byte");
        assert!(row_length(&[Cell::None; 255]).is_some());
        let text = "x".repeat(TEXT_BYTES);
        let wide = vec![Cell::Text(&text); 255]; // 255 x 257 bytes
        const { assert!(HEADER_BYTES + 255 * 257 > MAX_RECORD_BYTES) };
        assert_eq!(row_length(&wide), None, "larger than a ring record");
    }

    #[test]
    fn a_row_prints_no_wider_than_its_bound() {
        let text = "t".repeat(TEXT_BYTES);
        let widest = [
            Cell::None,
            Cell::Bool(true),
            Cell::I32(i32::MIN),
            Cell::I64(i64::MIN),
            Cell::F32(-f32::from_bits(1)),
            Cell::F32(-f32::MIN_POSITIVE),
            Cell::F32(f32::MIN),
            Cell::E7(i32::MIN),
            Cell::Text(&text),
        ];
        let mut line = Vec::new();
        recorder_wire::row(&mut line, u64::MAX, &widest);
        assert_eq!(
            row_width(&widest),
            20 + 1 + (4 + 1 + 11 + 20 + 48 + 48 + 48 + 12 + 255) + 8
        );
        // every kind's widest value meets its bound, but f32::MIN, 8 bytes short of -1e-45's
        assert_eq!(line.len(), row_width(&widest) - 8, "{}", String::from_utf8_lossy(&line));
        for cell in widest {
            for value in [cell, Cell::I32(0), Cell::F32(0.5), Cell::F32(f32::NAN)] {
                let mut line = Vec::new();
                recorder_wire::row(&mut line, 0, &[value]);
                assert!(line.len() <= row_width(&[value]), "{value:?}");
            }
        }
        let long = "t".repeat(300);
        assert_eq!(
            row_width(&[Cell::Text(&long)]),
            21 + TEXT_BYTES,
            "as the record cuts it"
        );
        // negative: an empty row still prints its uptime and a `;`
        let mut line = Vec::new();
        recorder_wire::row(&mut line, u64::MAX, &[]);
        assert_eq!((line.len(), row_width(&[])), (21, 21));
    }

    #[test]
    fn a_header_record_is_its_fixed_part() {
        let bytes = slot(HEADER_BYTES, |record| write_header(record, 3, false, 42));
        let record = Record::decode(&bytes).unwrap();
        assert_eq!((record.kind, record.stream, record.uptime_us), (Kind::Header, 3, 42));
        assert!(record.cells().is_empty());
    }

    #[test]
    fn a_log_record_keeps_its_text_and_loses_its_nuls() {
        let (descriptor, message) = log_parts("sequencer", "stage -> boosting\0");
        let length = log_length(descriptor, message);
        let bytes = slot(length, |record| write_log(record, 884_030, descriptor, message));
        let record = Record::decode(&bytes).unwrap();
        assert_eq!(record.kind, Kind::Log);
        let (descriptor, message) = record.log();
        assert_eq!((&*descriptor, &*message), ("sequencer", "stage -> boosting "));
        // an empty message, and a message ending where the padding starts
        for message in ["", "a", "ab", "abc", "abcd"] {
            let bytes = slot(log_length("d", message), |record| write_log(record, 1, "d", message));
            assert_eq!(Record::decode(&bytes).unwrap().log().1, message);
        }
    }

    #[test]
    fn a_log_record_is_cut_to_its_limits() {
        let descriptor = "d".repeat(300);
        let message = "m".repeat(2000);
        let (descriptor, message) = log_parts(&descriptor, &message);
        assert_eq!((descriptor.len(), message.len()), (TEXT_BYTES, MESSAGE_BYTES));
        let bytes = slot(log_length(descriptor, message), |record| {
            write_log(record, 1, descriptor, message)
        });
        let (back_descriptor, back_message) = Record::decode(&bytes).unwrap().log();
        assert_eq!((back_descriptor.len(), back_message.len()), (TEXT_BYTES, MESSAGE_BYTES));
    }

    #[test]
    fn a_line_record_carries_its_text() {
        let text = b"884031;123;000123;;;taster;dev;3f2a91;boot;;";
        let bytes = slot(line_length(text), |record| write_line(record, 0, true, 884_031, text));
        let record = Record::decode(&bytes).unwrap();
        assert_eq!((record.kind, record.durable, record.stream), (Kind::Line, true, 0));
        assert_eq!(record.line(), text);
    }

    #[test]
    fn a_damaged_record_reads_as_little_as_it_proves() {
        assert!(Record::decode(&[0; 15]).is_none(), "shorter than the fixed part");
        let mut bytes = slot(HEADER_BYTES, |record| write_header(record, 3, false, 42));
        bytes[2] = 9;
        assert!(Record::decode(&bytes).is_none(), "no such kind");
        // a row claiming more cells than it holds ends at the last whole one (28 bytes: no padding to misread)
        let cells = [Cell::I32(1), Cell::I32(2), Cell::Bool(true)];
        let mut bytes = slot(row_length(&cells).unwrap(), |record| {
            write_row(record, 1, false, 5, &cells)
        });
        bytes[6] = 5;
        assert_eq!(Record::decode(&bytes).unwrap().cells(), cells);
        let mut text = String::from("ok");
        text.push('\u{1F680}');
        let cells = [Cell::Text(&text)];
        let mut bytes = slot(row_length(&cells).unwrap(), |record| {
            write_row(record, 1, false, 5, &cells)
        });
        bytes[HEADER_BYTES + 2 + 3] = 0xFF; // inside the rocket
        assert!(
            Record::decode(&bytes).unwrap().cells().is_empty(),
            "invalid UTF-8 ends the row"
        );
    }
}
