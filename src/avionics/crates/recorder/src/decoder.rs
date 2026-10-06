//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The block decoder (doc/specs/avionics-port.md "The ring_saver", Decode; "CC tees and the HTTP exits"): one
//! flash block, its header and its chunks, turned into the wire lines recorderd wrote. The ring_saver (M4) files
//! them on the SD, `GET /ring` streams them, and the host tools read a ring image through it, so a session reads
//! the same whichever way it left the board.
//!
//! For each block:
//! * the block header names the session (the SD directory) and the boot. A block of another layout version is
//!   refused ([`BlockError::Version`]): this build cannot read its chunks, and keeps it for the build that can;
//! * the chunk walk checks every CRC and steps over a torn chunk to the next whole one
//!   ([`flash_ring::chunk::chunks`]);
//! * each record prints as its line (a ROW `<uptime>;v1;...;vn`, a HEADER its column list, a LOG
//!   `<uptime_us> <descriptor> :: <message>`), wrapped with its routing: `<session>_<file>`, the bare `session.csv`
//!   for the index, or none for recorder.log: `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`. A record whose file name
//!   the saver would not file goes to recorder.log, unrouted;
//! * a HEADER record prints only when its file starts or its columns change, per boot of a session (its block
//!   headers' session and boot id): the writer repeats every header about once per MiB, and those repeats never
//!   reach the SD. Each boot's first header does, as plan B and recorderd wrote one per boot: a session label
//!   reused over several boots (a stale `recorder.session` in a saved config) appends them all to one file, and
//!   its second header line is what makes `flight_telemetry` flag the splice. Two uncounted boots (boot id 0)
//!   under one label stay indistinguishable;
//! * a torn chunk is a loss only when nothing whole follows it in its block and no later block of its boot is in
//!   flash (spec "Block and chunk format"): a failed program the writer retried is not. A loss prints as a
//!   recorder.log line of its own, `<uptime> ring :: block <sequence>: torn chunk at <offset>, <k> records lost`,
//!   wrapped like any other, so a wrapped session never holds an unwrapped line.
//!
//! The loss line's uptime is the newest one the decoder has seen in that boot, so recorder.log never goes back in
//! time within it (a backward jump of more than half a 2^30 period reads as a ticks wrap to `flight_telemetry`),
//! and a boot that reuses a label is never stamped with an earlier boot's uptimes. Its `k` is the torn chunk's
//! record count as read, `?` when the count never landed.

use std::collections::BTreeMap;
use std::string::{String, ToString};
use std::vec::Vec;

use flash_ring::chunk::{self, Torn, Walked};
use flash_ring::header::{BlockHeader, HeaderError, VERSION};

use crate::record::{Kind, Record, Records};

/// The descriptor of the decoder's loss lines.
const LOSS_DESCRIPTOR: &str = "ring";

/// Why a block is not decoded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockError {
    /// No block header: an erased or foreign slot, or a header a cut tore or a cut erase damaged.
    Header(HeaderError),
    /// A block of another layout version: another build's (after an update, or a reboot in flight that came back
    /// on another image). Its chunks are not this build's to read; it stays in flash for the build that can.
    Version(u8),
}

/// What one block held.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockSummary {
    /// Its header: the session, the boot, the sequence, the flight flag.
    pub header: BlockHeader,
    /// The records its whole chunks held, a header repeat that printed nothing included.
    pub records: u32,
    /// The torn chunks that are losses, each printed as a loss line.
    pub lost: u32,
    /// The whole chunks whose records did not all decode: their CRC checked, so no writer of this layout wrote
    /// them. The records before the damage printed; the rest of the chunk did not.
    pub damaged: u32,
}

/// One decoded line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DecodedLine<'a> {
    /// The block's session label: the SD directory the line goes to.
    pub session: &'a str,
    /// Its routing: `<session>_<file>`, the bare `session.csv`, or None for recorder.log.
    pub routing: Option<&'a str>,
    /// The wire line, `[@<routing>@]{OPEN};<payload>;<CLOSE>\n`: what `GET /ring` streams and a capture holds.
    pub wire: &'a [u8],
}

impl<'a> DecodedLine<'a> {
    /// The line as its file holds it, `{OPEN};<payload>;<CLOSE>\n`: the wire line after `@<routing>@`, which is
    /// exactly what recorderd appended to the routing's file.
    pub fn body(&self) -> &'a [u8] {
        &self.wire[self.routing.map_or(0, |routing| routing.len() + 2)..]
    }
}

/// A boot as the block headers name it: (session label, boot id).
type Boot = (String, u32);

/// Decodes blocks into wire lines. It remembers, per boot, the columns each file's header line printed and the
/// newest uptime seen, so feed it a ring's blocks in `block_sequence` order. A new one prints every file's first
/// header again: the ring_saver keeps one for as long as it appends to the same files.
#[derive(Debug, Default)]
pub struct Decoder {
    /// The column list each file last printed, by boot and file name.
    printed: BTreeMap<(Boot, Vec<u8>), Vec<u8>>,
    /// The newest uptime seen, by boot: a loss line's stamp.
    newest_us: BTreeMap<Boot, u64>,
    /// The routing being built.
    routing: String,
    /// The line text being built.
    text: Vec<u8>,
    /// The wire line being built.
    wire: Vec<u8>,
}

impl Decoder {
    /// A decoder that has printed nothing yet.
    pub fn new() -> Decoder {
        Decoder::default()
    }

    /// Decode one block into its lines, in record order, calling `each` for every line.
    ///
    /// Args:
    /// * `block` - the block's bytes from its offset 0, the header included: the whole 4 KiB of a closed block, or
    ///   for the writer's open block the bytes up to the published `writer_position`, past which nothing is read.
    /// * `boot_goes_on` - a later block of the same boot (boot id and session) is in flash, or this is the writer's
    ///   open block: then a torn chunk is a failed program that the writer retried there, not a loss.
    /// * `each` - takes each line.
    ///
    /// Returns what the block held. Errors: `Header` when there is no block header, `Version` for a block of
    /// another layout version; nothing is printed then.
    pub fn block(
        &mut self,
        block: &[u8],
        boot_goes_on: bool,
        mut each: impl FnMut(DecodedLine<'_>),
    ) -> Result<BlockSummary, BlockError> {
        let header = BlockHeader::decode(block).map_err(BlockError::Header)?;
        if header.version != VERSION {
            return Err(BlockError::Version(header.version));
        }
        let mut summary = BlockSummary {
            header,
            records: 0,
            lost: 0,
            damaged: 0,
        };
        let session = header.session;
        let session = session.as_str();
        let boot = (session.to_string(), header.boot_id);
        for walked in chunk::chunks(block) {
            match walked {
                Walked::Chunk(found) => {
                    let mut records = Records::new(found.payload);
                    for record in records.by_ref() {
                        summary.records += 1;
                        self.record(&boot, &record, &mut each);
                    }
                    summary.damaged += u32::from(records.damaged());
                }
                Walked::Torn(torn) if !torn.followed && !boot_goes_on => {
                    summary.lost += 1;
                    self.loss(&boot, header.block_sequence, &torn, &mut each);
                }
                Walked::Torn(_) => {} // a failed program: the writer programmed its records again behind it
            }
        }
        Ok(summary)
    }

    /// Print one record of `boot`, unless it is a header its file has printed already in that boot.
    fn record(&mut self, boot: &Boot, record: &Record, each: &mut impl FnMut(DecodedLine<'_>)) {
        let session = boot.0.as_str();
        match self.newest_us.get_mut(boot) {
            Some(newest) => *newest = (*newest).max(record.uptime_us),
            None => {
                self.newest_us.insert(boot.clone(), record.uptime_us);
            }
        }
        let routing = record.routing(session, &mut self.routing);
        if record.kind == Kind::Header && routing.is_some() {
            let key = (boot.clone(), record.name().to_vec());
            if self.printed.get(&key).map(Vec::as_slice) == Some(record.columns()) {
                return;
            }
            self.printed.insert(key, record.columns().to_vec());
        }
        self.text.clear();
        record.text_into(&mut self.text);
        self.wire.clear();
        recorder_wire::wrap_into(&mut self.wire, routing, &self.text);
        each(DecodedLine {
            session,
            routing,
            wire: &self.wire,
        });
    }

    /// Print the loss line of a torn chunk of block `block_sequence` of `boot`, into its session's recorder.log.
    fn loss(&mut self, boot: &Boot, block_sequence: u32, torn: &Torn, each: &mut impl FnMut(DecodedLine<'_>)) {
        let session = boot.0.as_str();
        let uptime_us = self.newest_us.get(boot).copied().unwrap_or(0);
        let records = torn
            .records
            .map_or_else(|| String::from("?"), |records| records.to_string());
        let message = std::format!(
            "block {block_sequence}: torn chunk at {}, {records} records lost",
            torn.offset
        );
        self.text.clear();
        recorder_wire::log_line(&mut self.text, uptime_us, LOSS_DESCRIPTOR, &message, usize::MAX);
        self.wire.clear();
        recorder_wire::wrap_into(&mut self.wire, None, &self.text);
        each(DecodedLine {
            session,
            routing: None,
            wire: &self.wire,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::{BlockError, Decoder};
    use crate::record::{encode, header_length, log_length, log_parts, row_length, write_header, write_log, write_row};
    use flash_ring::BLOCK_BYTES;
    use flash_ring::chunk::{self, CHUNK_HEADER_BYTES, chunk_bytes};
    use flash_ring::header::{BlockHeader, HEADER_BYTES, HeaderError, Session, VERSION};
    use recorder_wire::Cell;
    use std::string::String;
    use std::vec::Vec;

    /// A HEADER record of `file` with `columns`.
    fn header(file: &str, columns: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        encode(&mut bytes, header_length(file, columns), |slot| {
            write_header(slot, file, false, 1, columns)
        });
        bytes
    }

    /// A ROW record of one i32 to `file` at `uptime_us`.
    fn row(file: &str, uptime_us: u64, value: i32) -> Vec<u8> {
        let cells = [Cell::I32(value)];
        let mut bytes = Vec::new();
        encode(&mut bytes, row_length(file, &cells).unwrap(), |slot| {
            write_row(slot, file, false, uptime_us, &cells)
        });
        bytes
    }

    /// A LOG record of `message` at `uptime_us`.
    fn log(uptime_us: u64, message: &str) -> Vec<u8> {
        let (descriptor, message) = log_parts("seq", message);
        let mut bytes = Vec::new();
        encode(&mut bytes, log_length(descriptor, message), |slot| {
            write_log(slot, uptime_us, descriptor, message)
        });
        bytes
    }

    /// An erased block 7 of boot 123 of `session` with a header of `version`, and these chunks (each a list of
    /// records) behind it. Returns the block and each chunk's offset.
    fn block(session: &str, version: u8, chunks: &[Vec<Vec<u8>>]) -> (Vec<u8>, Vec<usize>) {
        block_of(session, 123, version, chunks)
    }

    /// As [`block`], for boot `boot_id`.
    fn block_of(session: &str, boot_id: u32, version: u8, chunks: &[Vec<Vec<u8>>]) -> (Vec<u8>, Vec<usize>) {
        let mut bytes = vec![0xFF; BLOCK_BYTES as usize];
        let header = BlockHeader {
            version,
            flight: false,
            block_sequence: 7,
            boot_id,
            session: Session::new(session).unwrap(),
        };
        bytes[..HEADER_BYTES].copy_from_slice(&header.encode());
        let mut offset = HEADER_BYTES;
        let mut offsets = Vec::new();
        for records in chunks {
            let payload = records.concat();
            let mut encoded = Vec::new();
            chunk::encode(&mut encoded, &payload, records.len() as u16).unwrap();
            bytes[offset..offset + encoded.len()].copy_from_slice(&encoded);
            offsets.push(offset);
            offset += chunk_bytes(payload.len());
        }
        (bytes, offsets)
    }

    /// Every wire line `decoder` makes of `bytes`, as text.
    fn lines(decoder: &mut Decoder, bytes: &[u8], boot_goes_on: bool) -> Vec<String> {
        let mut found = Vec::new();
        decoder
            .block(bytes, boot_goes_on, |line| {
                found.push(String::from_utf8(line.wire.to_vec()).unwrap())
            })
            .unwrap();
        found
    }

    #[test]
    fn a_block_prints_the_wire_lines_recorderd_wrote() {
        let (bytes, _) = block(
            "000123",
            VERSION,
            &[
                vec![header("imu.csv", b"uptime;ax"), row("imu.csv", 884_029, 5)],
                vec![log(884_030, "go"), row("session.csv", 884_031, 1)],
            ],
        );
        let mut found = Vec::new();
        let summary = Decoder::new()
            .block(&bytes, false, |line| {
                found.push((
                    line.session.to_string(),
                    line.routing.map(String::from),
                    line.wire.to_vec(),
                    line.body().to_vec(),
                ))
            })
            .unwrap();
        assert_eq!((summary.records, summary.lost, summary.damaged), (4, 0, 0));
        assert_eq!((summary.header.block_sequence, summary.header.boot_id), (7, 123));
        let expected = [
            (Some("000123_imu.csv"), &b"uptime;ax"[..]),
            (Some("000123_imu.csv"), b"884029;5"),
            (None, b"884030 seq :: go"),
            (Some("session.csv"), b"884031;1"),
        ];
        assert_eq!(found.len(), expected.len());
        for ((session, routing, wire, body), (want_routing, payload)) in found.iter().zip(expected) {
            assert_eq!(session, "000123");
            assert_eq!(routing.as_deref(), want_routing);
            assert_eq!(
                *wire,
                recorder_wire::wrap(want_routing, payload),
                "{}",
                String::from_utf8_lossy(wire)
            );
            let skip = want_routing.map_or(0, |routing| routing.len() + 2);
            assert_eq!(body.as_slice(), &wire[skip..], "the bytes the routing's file holds");
        }
    }

    #[test]
    fn a_header_prints_once_per_file_until_its_columns_change() {
        let mut decoder = Decoder::new();
        let first = block(
            "000123",
            VERSION,
            &[vec![
                header("imu.csv", b"uptime;ax"),
                row("imu.csv", 1, 1),
                header("imu.csv", b"uptime;ax"),
            ]],
        )
        .0;
        assert_eq!(lines(&mut decoder, &first, true).len(), 2, "the repeat prints nothing");
        let later = block(
            "000123",
            VERSION,
            &[vec![
                header("imu.csv", b"uptime;ax"),
                header("imu.csv", b"uptime;ax;ay"),
                row("imu.csv", 2, 2),
            ]],
        )
        .0;
        let printed = lines(&mut decoder, &later, true);
        assert_eq!(printed.len(), 2, "{printed:?}");
        assert!(printed[0].contains("};uptime;ax;ay;<"), "new columns print");
        // negative: another session's file starts on its own, and a fresh decoder starts every file again
        let other = block("000124", VERSION, &[vec![header("imu.csv", b"uptime;ax;ay")]]).0;
        assert_eq!(lines(&mut decoder, &other, true).len(), 1);
        assert_eq!(lines(&mut Decoder::new(), &first, true).len(), 2);
    }

    #[test]
    fn each_boot_under_one_label_prints_its_own_header_and_stamps_its_own_losses() {
        // a stale recorder.session: boots 41 and 42 both record under `flightday`, into the same files
        let mut decoder = Decoder::new();
        let first = block_of(
            "flightday",
            41,
            VERSION,
            &[vec![
                header("imu.csv", b"uptime;ax"),
                row("imu.csv", 500_000_000, 1),
                header("imu.csv", b"uptime;ax"),
            ]],
        )
        .0;
        assert_eq!(lines(&mut decoder, &first, true).len(), 2, "the header and the row");
        let whole = vec![header("imu.csv", b"uptime;ax"), row("imu.csv", 2_000_000, 2)];
        let (mut second, offsets) = block_of("flightday", 42, VERSION, &[whole, vec![row("imu.csv", 2_010_000, 3)]]);
        second[offsets[1] + CHUNK_HEADER_BYTES + 4..offsets[1] + CHUNK_HEADER_BYTES + 12].fill(0xFF); // the cut
        let printed = lines(&mut decoder, &second, false);
        assert_eq!(printed.len(), 3, "{printed:?}");
        assert!(
            printed[0].contains("};uptime;ax;<"),
            "boot 42's first header prints, as recorderd's file had one per boot, so the splice shows: {printed:?}"
        );
        assert!(
            printed[2].contains(";2000000 ring :: block 7: torn chunk at "),
            "the loss is stamped with boot 42's newest uptime, not boot 41's: {printed:?}"
        );
        // negative: boot 42's own repeat prints nothing, as boot 41's did not
        let repeat = block_of("flightday", 42, VERSION, &[vec![header("imu.csv", b"uptime;ax")]]).0;
        assert!(lines(&mut decoder, &repeat, true).is_empty());
    }

    #[test]
    fn a_record_the_saver_would_not_file_goes_to_recorder_log() {
        let (bytes, _) = block(
            "000123",
            VERSION,
            &[vec![row("a/b.csv", 9, 3), header("ab", b"uptime;x")]],
        );
        let mut found = Vec::new();
        Decoder::new()
            .block(&bytes, false, |line| {
                found.push((line.routing.is_none(), line.wire.to_vec()))
            })
            .unwrap();
        assert_eq!(
            found,
            [
                (true, recorder_wire::wrap(None, b"9;3")),
                (true, recorder_wire::wrap(None, b"uptime;x")),
            ]
        );
    }

    #[test]
    fn a_torn_last_chunk_is_a_loss_line_and_a_retried_one_is_not() {
        let whole = vec![row("imu.csv", 500, 1), log(600, "kept")];
        let torn = vec![row("imu.csv", 700, 2), row("imu.csv", 800, 3)];
        let (mut bytes, offsets) = block("000123", VERSION, &[whole.clone(), torn.clone()]);
        let torn_at = offsets[1];
        bytes[torn_at + CHUNK_HEADER_BYTES + 10..torn_at + CHUNK_HEADER_BYTES + 40].fill(0xFF); // the cut
        let mut decoder = Decoder::new();
        let printed = lines(&mut decoder, &bytes, false);
        assert_eq!(printed.len(), 3);
        assert_eq!(
            printed[2].as_bytes(),
            recorder_wire::wrap(
                None,
                &format!("600 ring :: block 7: torn chunk at {torn_at}, 2 records lost").into_bytes()
            ),
            "stamped with the newest uptime seen, never back in time"
        );
        // negative: a later block of the boot is in flash, so the writer retried those records there
        assert_eq!(lines(&mut Decoder::new(), &bytes, true).len(), 2);
        // negative: a whole chunk behind the debris is the retry itself
        let (mut retried, offsets) = block("000123", VERSION, &[whole, torn.clone(), torn]);
        retried[offsets[1] + CHUNK_HEADER_BYTES + 10..offsets[1] + CHUNK_HEADER_BYTES + 40].fill(0xFF);
        let mut found = 0;
        let summary = Decoder::new().block(&retried, false, |_| found += 1).unwrap();
        assert_eq!((found, summary.lost, summary.records), (4, 0, 4));
        // a count that never landed is unknown
        let (mut early, offsets) = block("000123", VERSION, &[vec![row("imu.csv", 1, 1)]]);
        early[offsets[0] + 2..].fill(0xFF); // only the length field landed
        let printed = lines(&mut Decoder::new(), &early, false);
        assert!(
            printed[0].contains("0 ring :: block 7: torn chunk at 56, ? records lost"),
            "{printed:?}"
        );
    }

    #[test]
    fn a_chunk_whose_records_do_not_decode_is_damaged() {
        let mut junk = row("imu.csv", 1, 1);
        junk.extend_from_slice(&[0; 8]); // eight bytes no writer makes into a record
        let (bytes, _) = block("000123", VERSION, &[vec![junk]]);
        let mut found = 0;
        let summary = Decoder::new().block(&bytes, false, |_| found += 1).unwrap();
        assert_eq!((found, summary.records, summary.damaged), (1, 1, 1));
    }

    #[test]
    fn no_header_or_another_version_prints_nothing() {
        let mut decoder = Decoder::new();
        let mut printed = 0;
        let erased = vec![0xFF; BLOCK_BYTES as usize];
        assert_eq!(
            decoder.block(&erased, false, |_| printed += 1),
            Err(BlockError::Header(HeaderError::Magic))
        );
        let (newer, _) = block("000123", VERSION + 1, &[vec![row("imu.csv", 1, 1)]]);
        assert_eq!(
            decoder.block(&newer, false, |_| printed += 1),
            Err(BlockError::Version(VERSION + 1)),
            "another build's block is that build's to read"
        );
        assert_eq!(printed, 0);
        let (ours, _) = block("000123", VERSION, &[]);
        assert_eq!(decoder.block(&ours, false, |_| printed += 1).unwrap().records, 0);
    }
}
