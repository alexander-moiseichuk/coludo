//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The block header (doc/specs/avionics-port.md, "Block and chunk format"): 56 little-endian bytes at a block's
//! offset 0, programmed once, with one program call, when the writer opens the block.
//!
//! | off | type | field |
//! |---|---|---|
//! | 0 | `[u8; 4]` | magic `F7 43 42 31` |
//! | 4 | u8 | version = 1 |
//! | 5 | u8 | flags: bit0 flight (opened while the stage was BOOSTING..DONE) |
//! | 6 | u16 | reserved 0 |
//! | 8 | u32 | block sequence (global: +1 on every block opened, across boots) |
//! | 12 | u32 | boot id (0 = uncounted) |
//! | 16 | `[u8; 32]` | session label, ASCII, zero-padded |
//! | 48 | u32 | reserved 0 |
//! | 52 | u32 | header CRC over bytes 0..52 (zlib CRC-32; `esp_rom_crc32_le` matches) |
//!
//! The header proves itself with its CRC, so a header torn by a cut, or the bytes a cut erase leaves, never reads
//! as a block. The reserved fields are written 0 and ignored on read: the CRC already protects them.
//!
//! **The header and the slot grid are the same in every layout version.** A later version may change the chunks
//! and the records behind the header, never these 56 bytes (apart from the version byte and the reserved fields),
//! nor the 4 KiB grid of slots they sit on ([`crate::BLOCK_BYTES`]). That rule is what lets a build meet another
//! build's blocks (after an update, or when a reboot in flight comes back on an older image) and still order them
//! by sequence, keep them, and give up their flight blocks last. It cannot decode their chunks, so it never saves
//! them, and the eraser takes them only under newest wins ([`crate::Slot`]).
//!
//! The rule holds only while [`VERSION`] tells the layouts apart: a build reads a block of its own version with
//! its own codecs, and prints what it reads to the SD with valid line CRCs. So the version covers everything
//! behind the header, owned by two crates: the chunk format (this crate's [`crate::chunk`]) and the record
//! format (crates/recorder, spec "Producer side": the kinds, the cell tags, the LOG layout). A change to either
//! bumps it. Each side pins its bytes to the version in a test: this crate a byte-exact chunk, the recorder a
//! byte-exact record of each kind, so a format change fails until the version moves with it.

use core::fmt;

use recorder_wire::crc32;

/// The block magic `"\xF7CB1"`: 0xF7 never occurs in UTF-8, and the magic is neither all-0 nor all-1, so neither
/// erased flash, programmed-out flash, nor MicroPython-era text reads as a block.
pub const MAGIC: [u8; 4] = [0xF7, 0x43, 0x42, 0x31];

/// The layout version this build writes and decodes: the chunk format and the record format behind the header
/// (see the module doc). Bump it on any change to either; the tests that pin those bytes say so.
pub const VERSION: u8 = 1;

/// The header's size. A block's first chunk starts here (8-aligned).
pub const HEADER_BYTES: usize = 56;

/// The session label field's size: the longest label the block header holds.
pub const SESSION_BYTES: usize = 32;

/// Where the session label sits.
const SESSION_AT: usize = 16;

/// Where the header CRC sits. It covers every byte before it.
const HEADER_CRC_AT: usize = 52;

/// The flags bit of a flight block.
const FLIGHT_BIT: u8 = 0b1;

/// A session label as the block header holds it (doc/specs/avionics-port.md "Row, log and index text"): the
/// session prefix of every file of the boot, which names the SD directory the saver copies the block into.
///
/// It is 1 to 32 bytes of ASCII letters, digits, `-` and `_`. That takes all three forms of the prefix: a
/// `recorder.session` label, the boot id as `%06u`, and `YYYYMMDD_HHMMSS_<6 digits>`. It never holds a `/`, a
/// `.` or a `;`, so it is safe as a directory name and in a wire line. The alphabet is part of the frozen header.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Session {
    /// The label, zero-padded to the field's size: the bytes the header holds.
    field: [u8; SESSION_BYTES],
    /// The label's length.
    length: u8,
}

impl Session {
    /// The session `label`. Returns None when it is empty, longer than 32 bytes, or holds a byte outside
    /// `[A-Za-z0-9_-]`.
    pub fn new(label: &str) -> Option<Session> {
        let bytes = label.as_bytes();
        let allowed = bytes.iter().all(|&byte| is_label_byte(byte));
        if bytes.is_empty() || bytes.len() > SESSION_BYTES || !allowed {
            return None;
        }
        let mut field = [0u8; SESSION_BYTES];
        field[..bytes.len()].copy_from_slice(bytes);
        Some(Session {
            field,
            length: bytes.len() as u8,
        })
    }

    /// The label.
    pub fn as_str(&self) -> &str {
        // only the label alphabet gets in (new, decode), and it is ASCII
        core::str::from_utf8(&self.field[..usize::from(self.length)]).unwrap_or_default()
    }

    /// The session a header's zero-padded field holds: the label, then only zeros. Returns None for anything
    /// else, so a header whose CRC is good but whose label breaks the rule is not taken for a block.
    fn decode(field: &[u8]) -> Option<Session> {
        let length = field.iter().position(|&byte| byte == 0).unwrap_or(field.len());
        let padded = field[length..].iter().all(|&byte| byte == 0);
        let label = core::str::from_utf8(&field[..length]).ok()?;
        padded.then(|| Session::new(label)).flatten()
    }
}

impl fmt::Debug for Session {
    /// The label in quotes, as a string prints: the padding is noise in a failed assertion.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), formatter)
    }
}

/// Whether `byte` may stand in a session label.
fn is_label_byte(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || byte == b'-' || byte == b'_'
}

/// Why 56 bytes are not a block header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HeaderError {
    /// Fewer than 56 bytes.
    Short,
    /// The magic is wrong: erased flash, a foreign block (MicroPython-era data, a bench pattern), or a header
    /// a cut tore in its first bytes.
    Magic,
    /// The header CRC fails: a header torn by a cut, or one a cut erase damaged.
    Crc,
    /// The CRC is good but the session label breaks the rule: no ring writer wrote it.
    Session,
}

/// A block header's fields. The CRC is derived from the bytes, so [`BlockHeader::encode`] writes it and
/// [`BlockHeader::decode`] checks it; it is never held.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BlockHeader {
    /// The layout version: [`VERSION`] for every block this build writes. Another value is a block of another
    /// build, which this one keeps but cannot decode.
    pub version: u8,
    /// The block was opened while the stage was BOOSTING..DONE. Newest wins gives such blocks up last.
    pub flight: bool,
    /// The block's sequence number: blocks are read in its order, whichever slots they sit in.
    pub block_sequence: u32,
    /// The boot that opened it (0 = uncounted).
    pub boot_id: u32,
    /// The boot's session label: the SD directory its records go to.
    pub session: Session,
}

impl BlockHeader {
    /// The 56 bytes the writer programs at the block's offset 0.
    pub fn encode(&self) -> [u8; HEADER_BYTES] {
        let mut bytes = [0u8; HEADER_BYTES];
        bytes[0..4].copy_from_slice(&MAGIC);
        bytes[4] = self.version;
        bytes[5] = if self.flight { FLIGHT_BIT } else { 0 };
        bytes[8..12].copy_from_slice(&self.block_sequence.to_le_bytes());
        bytes[12..16].copy_from_slice(&self.boot_id.to_le_bytes());
        bytes[SESSION_AT..SESSION_AT + SESSION_BYTES].copy_from_slice(&self.session.field);
        let header_crc = crc32(0, &bytes[..HEADER_CRC_AT]);
        bytes[HEADER_CRC_AT..].copy_from_slice(&header_crc.to_le_bytes());
        bytes
    }

    /// The header at the start of `bytes`, checked for magic, CRC and session label, of any layout version: the
    /// header is the same in all of them. The caller compares [`BlockHeader::version`] with [`VERSION`] before it
    /// walks the chunks.
    pub fn decode(bytes: &[u8]) -> Result<BlockHeader, HeaderError> {
        let header = bytes.get(..HEADER_BYTES).ok_or(HeaderError::Short)?;
        if header[0..4] != MAGIC {
            return Err(HeaderError::Magic);
        }
        if crc32(0, &header[..HEADER_CRC_AT]) != word(header, HEADER_CRC_AT) {
            return Err(HeaderError::Crc);
        }
        let session = Session::decode(&header[SESSION_AT..SESSION_AT + SESSION_BYTES]).ok_or(HeaderError::Session)?;
        Ok(BlockHeader {
            version: header[4],
            flight: header[5] & FLIGHT_BIT != 0,
            block_sequence: word(header, 8),
            boot_id: word(header, 12),
            session,
        })
    }
}

/// The little-endian u32 at `at`.
fn word(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A header with every field distinct, so a field written to the wrong place shows.
    fn sample() -> BlockHeader {
        BlockHeader {
            version: VERSION,
            flight: true,
            block_sequence: 0x0102_0304,
            boot_id: 123,
            session: Session::new("000123").unwrap(),
        }
    }

    /// `bytes` with their header CRC recomputed: what a writer of other content would program.
    fn resealed(mut bytes: [u8; HEADER_BYTES]) -> [u8; HEADER_BYTES] {
        let header_crc = crc32(0, &bytes[..HEADER_CRC_AT]);
        bytes[HEADER_CRC_AT..].copy_from_slice(&header_crc.to_le_bytes());
        bytes
    }

    #[test]
    fn encodes_the_spec_layout_little_endian() {
        let bytes = sample().encode();
        assert_eq!(bytes[0..4], [0xF7, 0x43, 0x42, 0x31]);
        assert_eq!(bytes[4..8], [1, 1, 0, 0], "version, flags flight, reserved");
        assert_eq!(bytes[8..12], [4, 3, 2, 1]);
        assert_eq!(bytes[12..16], [123, 0, 0, 0]);
        assert_eq!(&bytes[16..22], b"000123");
        assert!(bytes[22..52].iter().all(|&byte| byte == 0), "zero padding, reserved 0");
        assert_eq!(bytes[52..56], crc32(0, &bytes[..52]).to_le_bytes());
    }

    #[test]
    fn the_frozen_layout_holds_and_the_chunk_bytes_move_only_with_the_version() {
        // frozen across builds: an older image reads a newer build's blocks by these, so they never change
        assert_eq!(MAGIC, [0xF7, 0x43, 0x42, 0x31]);
        assert_eq!((HEADER_BYTES, SESSION_BYTES, crate::BLOCK_BYTES), (56, 32, 4096));
        assert_eq!(
            crate::chunk::MAX_PAYLOAD,
            4096 - 56 - 8,
            "one chunk behind the header fills a slot"
        );
        // versioned: a change to these bytes must bump VERSION, or an older build prints a newer chunk wrongly
        let mut chunk = alloc::vec::Vec::new();
        crate::chunk::encode(&mut chunk, b"\x10\x00\x02\x00", 1).unwrap();
        assert_eq!(
            (VERSION, chunk.as_slice()),
            (1, &[4, 0, 1, 0, 0x2B, 0x37, 0x1B, 0x48, 0x10, 0x00, 0x02, 0x00][..]),
            "the chunk format changed: bump VERSION, then update this vector"
        );
    }

    #[test]
    fn round_trips_both_flags_and_any_version() {
        for (flight, version) in [(false, VERSION), (true, VERSION), (true, 2), (false, 0)] {
            let header = BlockHeader {
                flight,
                version,
                ..sample()
            };
            assert_eq!(BlockHeader::decode(&header.encode()), Ok(header));
        }
    }

    #[test]
    fn reads_the_header_alone_from_a_whole_block() {
        let mut block = [0xFFu8; 4096];
        block[..HEADER_BYTES].copy_from_slice(&sample().encode());
        assert_eq!(BlockHeader::decode(&block), Ok(sample()));
    }

    #[test]
    fn rejects_erased_flash_a_bad_magic_and_a_short_buffer() {
        assert_eq!(BlockHeader::decode(&[0xFF; 56]), Err(HeaderError::Magic));
        assert_eq!(BlockHeader::decode(&[0x00; 56]), Err(HeaderError::Magic));
        let mut bytes = sample().encode();
        bytes[0] = 0xF6;
        assert_eq!(BlockHeader::decode(&bytes), Err(HeaderError::Magic));
        assert_eq!(
            BlockHeader::decode(&resealed(bytes)),
            Err(HeaderError::Magic),
            "a good CRC does not make another magic a block"
        );
        assert_eq!(BlockHeader::decode(&sample().encode()[..55]), Err(HeaderError::Short));
    }

    #[test]
    fn rejects_any_damaged_byte_and_a_torn_header() {
        let bytes = sample().encode();
        for index in 4..HEADER_BYTES {
            let mut damaged = bytes;
            damaged[index] ^= 0x10;
            assert_eq!(BlockHeader::decode(&damaged), Err(HeaderError::Crc), "byte {index}");
        }
        for torn in 4..HEADER_BYTES {
            let mut cut = [0xFFu8; HEADER_BYTES];
            cut[..torn].copy_from_slice(&bytes[..torn]); // the program stopped after `torn` bytes
            assert_eq!(BlockHeader::decode(&cut), Err(HeaderError::Crc), "torn at {torn}");
        }
    }

    #[test]
    fn ignores_the_reserved_fields_under_a_good_crc() {
        let mut bytes = sample().encode();
        bytes[5] |= 0b1000_0010; // reserved flag bits
        bytes[6] = 7;
        bytes[48] = 9;
        assert_eq!(BlockHeader::decode(&resealed(bytes)), Ok(sample()));
    }

    #[test]
    fn rejects_a_label_no_writer_writes_even_under_a_good_crc() {
        let good = sample().encode();
        let cases: [&[u8]; 4] = [b"", b"0001/3", b"run 3", b"caf\xc3\xa9"];
        for label in cases {
            let mut bytes = good;
            bytes[16..48].fill(0);
            bytes[16..16 + label.len()].copy_from_slice(label);
            assert_eq!(
                BlockHeader::decode(&resealed(bytes)),
                Err(HeaderError::Session),
                "{label:?}"
            );
        }
        let mut gap = good;
        gap[23] = b'x'; // a byte after the padding started
        assert_eq!(BlockHeader::decode(&resealed(gap)), Err(HeaderError::Session));
        let mut full = good;
        full[16..48].fill(b'a'); // 32 bytes, no padding at all
        let header = BlockHeader::decode(&resealed(full)).unwrap();
        assert_eq!(header.session.as_str(), "a".repeat(32));
    }

    #[test]
    fn a_session_takes_the_three_prefix_forms_and_nothing_else() {
        for label in ["hitl-f15", "000123", "20261003_142107_100000", "4294967295"] {
            assert_eq!(Session::new(label).unwrap().as_str(), label);
        }
        for label in ["", "a/b", "a.b", "a;b", "a b", "é", &"x".repeat(33)] {
            assert_eq!(Session::new(label), None, "{label:?}");
        }
        assert_eq!(alloc::format!("{:?}", Session::new("run-3").unwrap()), "\"run-3\"");
    }
}
