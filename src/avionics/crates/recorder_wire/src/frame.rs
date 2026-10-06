//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flash frame (doc/specs/avionics-port.md, "Flash frame format"): a 40-byte little-endian header at an
//! 8-aligned offset, followed by the payload. The next frame starts at `align8(offset + 40 + payload_length)`.
//!
//! | off | type | field |
//! |---|---|---|
//! | 0 | `[u8; 4]` | magic `F7 43 52 31` |
//! | 4 | u8 | version 1 |
//! | 5 | u8 | kind: 1 DATA, 2 BOOT, 3 CONFIRM |
//! | 6 | u8 | flags: bit0 spilled, bit1 durable |
//! | 7 | u8 | reserved 0 |
//! | 8 | u32 | payload length |
//! | 12 | u32 | block sequence |
//! | 16 | u32 | boot id |
//! | 20 | u32 | frame sequence |
//! | 24 | u64 | uptime_us of the first record |
//! | 32 | u32 | payload CRC |
//! | 36 | u32 | header CRC over bytes 0..36 |
//!
//! The header proves itself with its own CRC, so a torn header is never taken for a frame. The payload has a
//! second CRC, so a torn payload is told apart from a whole one. Both are [`crate::crc32`], the CRC under the
//! line wrapper, so one table serves the frame and the lines inside it.
//!
//! The codec lives beside the line wrapper because every reader of the ring needs both: flash_ring (which
//! re-exports this module as `flash_ring::frame`) writes and scans frames, and the SD mover, the `/ring` dump
//! and ring_split read frames, then verify the lines in a DATA payload.

use alloc::vec::Vec;

use crate::crc::crc32;

/// The frame magic `"\xF7CR1"`. 0xF7 never occurs in UTF-8 text, and the magic is neither erased flash (all
/// 0xFF) nor fully programmed flash (all 0x00).
pub const MAGIC: [u8; 4] = [0xF7, 0x43, 0x52, 0x31];

/// The header layout version. A reader refuses any other, so a changed layout is never misread.
pub const VERSION: u8 = 1;

/// The header's size. A frame's payload starts this many bytes after the frame's offset.
pub const HEADER_BYTES: usize = 40;

/// Frames start on this alignment, so a reader that lost its place resynchronises in 8-byte steps.
pub const ALIGNMENT: usize = 8;

/// A CONFIRM payload's size: six u32.
pub const CONFIRM_BYTES: usize = 24;

/// Where the payload CRC sits in the header.
const PAYLOAD_CRC_AT: usize = 32;

/// Where the header CRC sits. It covers every header byte before it.
const HEADER_CRC_AT: usize = 36;

/// The flags bit of a spilled frame.
const SPILLED_BIT: u8 = 0b01;

/// The flags bit of a durable cut.
const DURABLE_BIT: u8 = 0b10;

/// A place in the ring: a block by its global sequence number, and a byte offset inside it. The flash writer
/// publishes the one it has programmed up to, the mover confirms one, and a CONFIRM frame carries it
/// ([`Confirm::moved`]). Positions order by block first, so a later position compares greater.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Position {
    /// The block's sequence number (`block_seq`): it grows by one for every block opened, across boots.
    pub block_sequence: u32,
    /// Bytes from the block's start.
    pub offset: u32,
}

/// What a frame carries.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// One or more complete wire lines: telemetry rows and log lines.
    Data = 1,
    /// A block's first frame, carrying the boot's identity, so every block names its session by itself.
    Boot = 2,
    /// The mover's committed cursor ([`Confirm`]), so the next boot knows what the SD holds even without
    /// the card.
    Confirm = 3,
}

impl Kind {
    /// The kind a header byte names. Returns None for any other byte.
    pub fn from_byte(byte: u8) -> Option<Kind> {
        match byte {
            1 => Some(Kind::Data),
            2 => Some(Kind::Boot),
            3 => Some(Kind::Confirm),
            _ => None,
        }
    }
}

/// A frame's flags byte. The six reserved bits are written 0 and ignored on read, since the header CRC
/// already protects them.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags {
    /// The frame was cut while no erased block was ready, and waited in the spill queue before it was
    /// programmed.
    pub spilled: bool,
    /// The frame was cut early because it held a durable record.
    pub durable: bool,
}

impl Flags {
    /// The flags byte.
    fn to_byte(self) -> u8 {
        (if self.spilled { SPILLED_BIT } else { 0 }) | (if self.durable { DURABLE_BIT } else { 0 })
    }

    /// The flags a byte holds, ignoring the reserved bits.
    fn from_byte(byte: u8) -> Flags {
        Flags {
            spilled: byte & SPILLED_BIT != 0,
            durable: byte & DURABLE_BIT != 0,
        }
    }
}

/// Why bytes are not a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameError {
    /// Fewer than 40 bytes, so there is no room for a header.
    Short,
    /// The magic is wrong: erased flash, a foreign block, or no frame starts here.
    Magic,
    /// The header CRC fails: a torn or damaged header.
    HeaderCrc,
    /// A header version this reader does not know.
    Version,
    /// A kind byte that is not DATA, BOOT or CONFIRM.
    Kind,
    /// The payload length runs past the bytes given (a block's end), or does not match the payload.
    Length,
    /// The payload CRC fails: a torn or damaged payload.
    PayloadCrc,
}

/// A frame header's fields. The spec calls them `payload_len`, `block_seq` and `frame_seq`; the two CRCs are
/// derived from the bytes, so they are written by [`Header::encode`] and checked by [`decode`], never held.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// What the payload is.
    pub kind: Kind,
    /// The spilled and durable bits.
    pub flags: Flags,
    /// The payload's length in bytes.
    pub payload_length: u32,
    /// The block the frame lives in (the slot's BOOT frame carries the same number).
    pub block_sequence: u32,
    /// The boot that wrote it (0 = uncounted).
    pub boot_id: u32,
    /// Its number in its boot: from 0, +1 for every frame of any kind, so a gap means frames were lost.
    pub frame_sequence: u32,
    /// The uptime of the frame's first record, in microseconds since the boot.
    pub uptime_us: u64,
}

impl Header {
    /// The frame for `payload` under this header: the 40 header bytes then the payload, as the ring programs
    /// it. Returns `FrameError::Length` when `payload_length` is not the payload's length.
    pub fn encode(&self, payload: &[u8]) -> Result<Vec<u8>, FrameError> {
        if payload.len() != self.payload_length as usize {
            return Err(FrameError::Length);
        }
        let mut frame = Vec::with_capacity(HEADER_BYTES + payload.len());
        frame.resize(HEADER_BYTES, 0);
        frame.extend_from_slice(payload);
        self.seal(&mut frame);
        Ok(frame)
    }

    /// Writes this header into `frame[..40]`, over a payload already in place behind it. This lets the ring
    /// build a frame without copying its payload twice.
    ///
    /// Panics when `frame` holds fewer than `40 + payload_length` bytes: the caller sized it for this header.
    pub fn seal(&self, frame: &mut [u8]) {
        let payload_crc = crc32(0, &frame[HEADER_BYTES..HEADER_BYTES + self.payload_length as usize]);
        frame[0..4].copy_from_slice(&MAGIC);
        frame[4] = VERSION;
        frame[5] = self.kind as u8;
        frame[6] = self.flags.to_byte();
        frame[7] = 0;
        frame[8..12].copy_from_slice(&self.payload_length.to_le_bytes());
        frame[12..16].copy_from_slice(&self.block_sequence.to_le_bytes());
        frame[16..20].copy_from_slice(&self.boot_id.to_le_bytes());
        frame[20..24].copy_from_slice(&self.frame_sequence.to_le_bytes());
        frame[24..32].copy_from_slice(&self.uptime_us.to_le_bytes());
        frame[PAYLOAD_CRC_AT..HEADER_CRC_AT].copy_from_slice(&payload_crc.to_le_bytes());
        let header_crc = crc32(0, &frame[..HEADER_CRC_AT]);
        frame[HEADER_CRC_AT..HEADER_BYTES].copy_from_slice(&header_crc.to_le_bytes());
    }

    /// The header at the start of `bytes`, checked for magic, header CRC, version and kind. The payload is not
    /// looked at, which is all the boot scan reads of a block.
    pub fn decode(bytes: &[u8]) -> Result<Header, FrameError> {
        let header = bytes.get(..HEADER_BYTES).ok_or(FrameError::Short)?;
        if header[0..4] != MAGIC {
            return Err(FrameError::Magic);
        }
        if crc32(0, &header[..HEADER_CRC_AT]) != word(header, HEADER_CRC_AT) {
            return Err(FrameError::HeaderCrc);
        }
        if header[4] != VERSION {
            return Err(FrameError::Version);
        }
        Ok(Header {
            kind: Kind::from_byte(header[5]).ok_or(FrameError::Kind)?,
            flags: Flags::from_byte(header[6]),
            payload_length: word(header, 8),
            block_sequence: word(header, 12),
            boot_id: word(header, 16),
            frame_sequence: word(header, 20),
            uptime_us: u64::from_le_bytes(header[24..32].try_into().expect("8 bytes")),
        })
    }
}

/// The frame at the start of `bytes`: its header and payload, with both CRCs checked and the payload inside
/// `bytes` (pass a block's remainder, and a frame running past the block's end fails with `Length`).
pub fn decode(bytes: &[u8]) -> Result<(Header, &[u8]), FrameError> {
    let header = Header::decode(bytes)?;
    let end = HEADER_BYTES
        .checked_add(header.payload_length as usize)
        .filter(|end| *end <= bytes.len())
        .ok_or(FrameError::Length)?;
    let payload = &bytes[HEADER_BYTES..end];
    if crc32(0, payload) != word(bytes, PAYLOAD_CRC_AT) {
        return Err(FrameError::PayloadCrc);
    }
    Ok((header, payload))
}

/// The bytes a frame with this payload takes in its block: the header, the payload, and the padding up to
/// the next 8-aligned offset (left erased).
pub const fn frame_bytes(payload_length: usize) -> usize {
    (HEADER_BYTES + payload_length).next_multiple_of(ALIGNMENT)
}

/// The little-endian u32 at `at`.
fn word(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().expect("4 bytes"))
}

/// A CONFIRM payload: how far the mover has committed the ring to the SD, and on which card.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Confirm {
    /// The position the mover has moved up to: every frame before it is on the card.
    pub moved: Position,
    /// The boot of the frame at `moved`.
    pub boot_id: u32,
    /// The number of the frame at `moved`.
    pub frame_sequence: u32,
    /// The journal record that committed it.
    pub journal_sequence: u32,
    /// The FAT volume serial of the card it went to, so a boot that finds another card re-copies the ring.
    pub card_volume_serial: u32,
}

impl Confirm {
    /// The 24-byte payload: block sequence, offset, boot id, frame sequence, journal sequence and card serial,
    /// each a little-endian u32.
    pub fn encode(&self) -> [u8; CONFIRM_BYTES] {
        let fields = [
            self.moved.block_sequence,
            self.moved.offset,
            self.boot_id,
            self.frame_sequence,
            self.journal_sequence,
            self.card_volume_serial,
        ];
        let mut payload = [0u8; CONFIRM_BYTES];
        for (index, field) in fields.into_iter().enumerate() {
            payload[index * 4..index * 4 + 4].copy_from_slice(&field.to_le_bytes());
        }
        payload
    }

    /// The confirmation a CONFIRM payload holds. Returns `FrameError::Length` unless it is exactly 24 bytes.
    pub fn decode(payload: &[u8]) -> Result<Confirm, FrameError> {
        if payload.len() != CONFIRM_BYTES {
            return Err(FrameError::Length);
        }
        Ok(Confirm {
            moved: Position {
                block_sequence: word(payload, 0),
                offset: word(payload, 4),
            },
            boot_id: word(payload, 8),
            frame_sequence: word(payload, 12),
            journal_sequence: word(payload, 16),
            card_volume_serial: word(payload, 20),
        })
    }
}

/// One frame found in a block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    /// Its offset in the block.
    pub offset: u32,
    /// Its header.
    pub header: Header,
    /// Its payload, both CRCs checked.
    pub payload: &'a [u8],
}

/// The whole frames of a block's bytes, in address order from offset 0. See [`frames`].
#[derive(Clone, Debug)]
pub struct Frames<'a> {
    /// The block's bytes.
    block: &'a [u8],
    /// Where the next frame starts.
    offset: usize,
}

/// The frames of a block's bytes, from offset 0 up to the first one that does not decode. That is the erased
/// tail, or a frame torn by a power cut: the writer abandons a block after a failed program, so nothing valid
/// follows it. The caller checks each frame's block sequence against the block's own.
pub fn frames(block: &[u8]) -> Frames<'_> {
    Frames { block, offset: 0 }
}

impl<'a> Iterator for Frames<'a> {
    type Item = Frame<'a>;

    fn next(&mut self) -> Option<Frame<'a>> {
        let (header, payload) = decode(self.block.get(self.offset..)?).ok()?;
        let offset = self.offset;
        self.offset += frame_bytes(payload.len());
        Some(Frame {
            offset: offset as u32,
            header,
            payload,
        })
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    /// A header with every field distinct, so a field written to the wrong place shows.
    fn sample(payload_length: u32) -> Header {
        Header {
            kind: Kind::Data,
            flags: Flags {
                spilled: false,
                durable: true,
            },
            payload_length,
            block_sequence: 0x0102_0304,
            boot_id: 123,
            frame_sequence: 0x0A0B_0C0D,
            uptime_us: 0x1122_3344_5566_7788,
        }
    }

    #[test]
    fn encodes_the_spec_layout_little_endian() {
        let frame = sample(3).encode(b"a;b").unwrap();
        assert_eq!(frame.len(), 43);
        assert_eq!(frame[0..4], [0xF7, 0x43, 0x52, 0x31]);
        assert_eq!(
            frame[4..8],
            [1, 1, 0b10, 0],
            "version, kind DATA, flags durable, reserved"
        );
        assert_eq!(frame[8..12], [3, 0, 0, 0]);
        assert_eq!(frame[12..16], [4, 3, 2, 1]);
        assert_eq!(frame[16..20], [123, 0, 0, 0]);
        assert_eq!(frame[20..24], [0x0D, 0x0C, 0x0B, 0x0A]);
        assert_eq!(frame[24..32], [0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
        assert_eq!(frame[32..36], crc32(0, b"a;b").to_le_bytes());
        assert_eq!(frame[36..40], crc32(0, &frame[..36]).to_le_bytes());
        assert_eq!(&frame[40..], b"a;b");
    }

    #[test]
    fn round_trips_every_kind_and_flag() {
        for kind in [Kind::Data, Kind::Boot, Kind::Confirm] {
            for flags in [(false, false), (true, false), (false, true), (true, true)] {
                let header = Header {
                    kind,
                    flags: Flags {
                        spilled: flags.0,
                        durable: flags.1,
                    },
                    ..sample(5)
                };
                let mut frame = header.encode(b"hello").unwrap();
                frame.extend_from_slice(&[0xFF; 3]); // the erased padding behind it
                assert_eq!(decode(&frame), Ok((header, &b"hello"[..])));
                assert_eq!(Header::decode(&frame), Ok(header));
            }
        }
    }

    #[test]
    fn an_empty_payload_is_a_frame() {
        let frame = sample(0).encode(b"").unwrap();
        assert_eq!(decode(&frame), Ok((sample(0), &b""[..])));
    }

    #[test]
    fn rejects_a_short_buffer() {
        let frame = sample(0).encode(b"").unwrap();
        assert_eq!(Header::decode(&frame[..39]), Err(FrameError::Short));
        assert_eq!(decode(&[]), Err(FrameError::Short));
    }

    #[test]
    fn rejects_erased_flash_and_a_bad_magic() {
        assert_eq!(decode(&[0xFF; 64]), Err(FrameError::Magic));
        assert_eq!(decode(&[0x00; 64]), Err(FrameError::Magic));
        let mut frame = sample(1).encode(b"x").unwrap();
        frame[0] = 0xF6;
        assert_eq!(decode(&frame), Err(FrameError::Magic));
    }

    #[test]
    fn rejects_any_damaged_header_byte() {
        let frame = sample(1).encode(b"x").unwrap();
        for index in 4..HEADER_BYTES {
            let mut damaged = frame.clone();
            damaged[index] ^= 0x10;
            assert_eq!(decode(&damaged), Err(FrameError::HeaderCrc), "byte {index}");
        }
    }

    #[test]
    fn rejects_an_unknown_version_or_kind_even_with_a_good_crc() {
        for (at, value, error) in [
            (4, 2, FrameError::Version),
            (5, 0, FrameError::Kind),
            (5, 4, FrameError::Kind),
        ] {
            let mut frame = sample(1).encode(b"x").unwrap();
            frame[at] = value;
            let header_crc = crc32(0, &frame[..36]);
            frame[36..40].copy_from_slice(&header_crc.to_le_bytes());
            assert_eq!(decode(&frame), Err(error));
        }
    }

    #[test]
    fn rejects_a_torn_or_damaged_payload() {
        let frame = sample(4).encode(b"abcd").unwrap();
        let mut damaged = frame.clone();
        damaged[42] = b'X';
        assert_eq!(decode(&damaged), Err(FrameError::PayloadCrc));
        let mut torn = frame.clone();
        torn[41..].fill(0xFF); // the program stopped after the payload's first byte
        assert_eq!(decode(&torn), Err(FrameError::PayloadCrc));
        assert_eq!(Header::decode(&torn), Ok(sample(4)), "the header alone still reads");
    }

    #[test]
    fn rejects_a_length_past_the_bytes() {
        let frame = sample(4).encode(b"abcd").unwrap();
        assert_eq!(
            decode(&frame[..43]),
            Err(FrameError::Length),
            "the block ends inside the payload"
        );
        let mismatched = sample(u32::MAX).encode(b"abcd");
        assert_eq!(
            mismatched,
            Err(FrameError::Length),
            "the header must state the payload's own length"
        );
        let mut frame = frame.clone();
        frame[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        let header_crc = crc32(0, &frame[..36]);
        frame[36..40].copy_from_slice(&header_crc.to_le_bytes());
        assert_eq!(decode(&frame), Err(FrameError::Length), "a length past any block");
    }

    #[test]
    fn frames_take_8_aligned_room() {
        assert_eq!(frame_bytes(0), 40);
        assert_eq!(frame_bytes(1), 48);
        assert_eq!(frame_bytes(8), 48);
        assert_eq!(frame_bytes(9), 56);
    }

    #[test]
    fn confirm_round_trips_and_refuses_another_length() {
        let confirm = Confirm {
            moved: Position {
                block_sequence: 7,
                offset: 4096,
            },
            boot_id: 12,
            frame_sequence: 900,
            journal_sequence: 33,
            card_volume_serial: 0xDEAD_BEEF,
        };
        let payload = confirm.encode();
        assert_eq!(payload[0..8], [7, 0, 0, 0, 0, 16, 0, 0]);
        assert_eq!(Confirm::decode(&payload), Ok(confirm));
        assert_eq!(Confirm::decode(&payload[..23]), Err(FrameError::Length));
        assert_eq!(Confirm::decode(&[0; 25]), Err(FrameError::Length));
    }

    #[test]
    fn frames_walk_to_the_erased_tail_or_a_torn_frame() {
        let mut block = vec![0xFF; 256];
        let first = sample(3).encode(b"one").unwrap();
        let second = Header {
            frame_sequence: 2,
            ..sample(5)
        }
        .encode(b"three")
        .unwrap();
        block[..first.len()].copy_from_slice(&first);
        block[48..48 + second.len()].copy_from_slice(&second);
        let walked: Vec<(u32, &[u8])> = frames(&block).map(|frame| (frame.offset, frame.payload)).collect();
        assert_eq!(walked, vec![(0, &b"one"[..]), (48, &b"three"[..])]);
        let third = sample(10).encode(b"0123456789").unwrap();
        block[96..96 + 30].copy_from_slice(&third[..30]); // torn inside its header
        assert_eq!(frames(&block).count(), 2);
        assert_eq!(
            frames(&block[..60]).count(),
            1,
            "a frame past the block's end is not one"
        );
    }
}
