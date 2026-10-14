//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The chunk (doc/specs/avionics-port.md, "Block and chunk format"): what one writer program call appends to a
//! block, at an 8-aligned offset behind the block header. The next chunk starts at `align8(offset + 8 + length)`;
//! the padding in between is never programmed (bench gate G1 proved partial-page programs safe, so chunks stay
//! unpadded).
//!
//! | off | type | field |
//! |---|---|---|
//! | 0 | u16 | length = payload bytes; `0xFFFF` = the erased tail, no chunk here |
//! | 2 | u16 | record count |
//! | 4 | u32 | CRC over bytes 0..4 and the payload (zlib CRC-32) |
//! | 8 | … | payload: whole records (spec "Producer side"), verbatim |
//!
//! A record never straddles chunks or blocks, so every whole chunk decodes on its own. The records themselves are
//! the recorder's codec; to this crate a payload is bytes and a count.
//!
//! [`chunks`] walks a block for the saver, the `/ring` exit and the tools. A chunk whose CRC fails, or whose
//! length runs past the block, is **torn**: a program call that a cut or a flash error stopped part way. After a
//! cut it is the last chunk of its boot's last block, since a new boot never appends to an old block. After a
//! failed call without a cut, the writer retries the same records at the next offset (spec "The ring_writer", step
//! 7), so a whole chunk may follow it. The walk therefore steps over a torn chunk in 8-byte steps until the next
//! whole chunk or the erased tail: the CRC tells a whole chunk from the debris. [`Torn::followed`] says which of
//! the two it was, so a retried program is not logged as a loss.

use alloc::vec::Vec;

use recorder_wire::crc32;

use crate::BLOCK_BYTES;
use crate::header::HEADER_BYTES;

/// The chunk header's size: length, record count, CRC.
pub const CHUNK_HEADER_BYTES: usize = 8;

/// Chunks start on this alignment, so a walk that lost its place finds the next chunk in 8-byte steps.
pub const ALIGNMENT: usize = 8;

/// The largest payload: a block less its header and one chunk header (4032 bytes). One such chunk fills a block.
pub const MAX_PAYLOAD: usize = BLOCK_BYTES as usize - HEADER_BYTES - CHUNK_HEADER_BYTES;

/// The length field of erased flash: no chunk starts here.
const ERASED_LENGTH: u16 = 0xFFFF;

/// The value of an erased byte.
const ERASED: u8 = 0xFF;

/// The bytes a chunk with this payload takes in its block: the header, the payload, and the padding up to the
/// next 8-aligned offset (left erased).
pub const fn chunk_bytes(payload_length: usize) -> usize {
    (CHUNK_HEADER_BYTES + payload_length).next_multiple_of(ALIGNMENT)
}

/// Why bytes are not a whole chunk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkError {
    /// Fewer than 8 bytes: no room for a chunk header.
    Short,
    /// The length field reads 0xFFFF: erased flash, the block's unused tail.
    Erased,
    /// The payload runs past the bytes given (the block's end, or the writer's published position), or is longer
    /// than [`MAX_PAYLOAD`].
    Length,
    /// The CRC fails: a chunk torn by a cut or a failed program, or damaged by a cut erase.
    Crc,
}

/// Writes the chunk of `payload` and its `records` into `chunk` (cleared first): the bytes one program call puts at
/// the chunk's offset. Returns `ChunkError::Length` when the payload is longer than [`MAX_PAYLOAD`], writing
/// nothing.
pub fn encode(chunk: &mut Vec<u8>, payload: &[u8], records: u16) -> Result<(), ChunkError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(ChunkError::Length);
    }
    chunk.clear();
    chunk.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    chunk.extend_from_slice(&records.to_le_bytes());
    let chunk_crc = crc32(crc32(0, &chunk[..4]), payload);
    chunk.extend_from_slice(&chunk_crc.to_le_bytes());
    chunk.extend_from_slice(payload);
    Ok(())
}

/// The chunk at the start of `bytes`: its record count and payload, with the CRC checked and the payload inside
/// `bytes`. Pass the rest of a block, so a chunk running past the block's end fails with `Length`.
pub fn decode(bytes: &[u8]) -> Result<(u16, &[u8]), ChunkError> {
    let header = bytes.get(..CHUNK_HEADER_BYTES).ok_or(ChunkError::Short)?;
    let length = u16::from_le_bytes([header[0], header[1]]);
    if length == ERASED_LENGTH {
        return Err(ChunkError::Erased);
    }
    let end = CHUNK_HEADER_BYTES + usize::from(length);
    if usize::from(length) > MAX_PAYLOAD || end > bytes.len() {
        return Err(ChunkError::Length);
    }
    let payload = &bytes[CHUNK_HEADER_BYTES..end];
    let stored = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
    if crc32(crc32(0, &header[..4]), payload) != stored {
        return Err(ChunkError::Crc);
    }
    Ok((u16::from_le_bytes([header[2], header[3]]), payload))
}

/// One whole chunk found in a block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Chunk<'a> {
    /// Its offset in the block.
    pub offset: u32,
    /// How many records its payload holds.
    pub records: u16,
    /// Its payload, CRC checked: whole records, verbatim.
    pub payload: &'a [u8],
}

/// A torn chunk: bytes at a chunk offset that are neither a whole chunk nor the erased tail.
///
/// Whether its records are lost depends on what tore it. A cut tears only the last chunk of its boot's last block
/// (a new boot never appends to an old block), and those records are lost. A failed program is retried by the
/// writer at the next offset, so its records are in a later chunk: of the same block when a whole chunk follows
/// the debris ([`Torn::followed`]), else of the boot's next block, since a retry that does not fit closes the
/// block. The saver's rule (spec "The ring_saver") is therefore: a torn chunk is a loss, logged as `block
/// <sequence>: torn chunk at <offset>, k records lost`, only when nothing whole follows it in its block and no
/// later block of the same boot (boot id and session) is in flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Torn {
    /// Where the torn chunk starts in the block.
    pub offset: u32,
    /// Its record count as read, for the loss line. The CRC did not check it, so a cut may have torn it too. None
    /// when the field reads erased (the program stopped before it).
    pub records: Option<u16>,
    /// A whole chunk follows it in the block: no cut tore it (a cut tears only a block's last chunk), so it is a
    /// failed program the writer retried behind it, and its records are not lost.
    pub followed: bool,
}

/// What a block walk finds at a chunk offset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Walked<'a> {
    /// A whole chunk.
    Chunk(Chunk<'a>),
    /// A torn chunk. The walk goes on at the next whole chunk behind it, if there is one.
    Torn(Torn),
}

/// The chunks of a block, in address order. See [`chunks`].
#[derive(Clone, Debug)]
pub struct Chunks<'a> {
    /// The block's bytes, from its offset 0.
    block: &'a [u8],
    /// Where the next chunk may start.
    offset: usize,
    /// One past the block's last byte that is not erased: from here on, the block is its erased tail.
    end: usize,
}

/// The chunks of `block` (its bytes from offset 0, the header included): every whole chunk and every torn one,
/// in address order, from the first chunk offset up to the erased tail.
///
/// Pass the whole block for a closed one. For the writer's open block, pass the bytes up to its published
/// position, which the walk then treats as the block's end: nothing is read past it.
pub fn chunks(block: &[u8]) -> Chunks<'_> {
    let end = block
        .iter()
        .rposition(|&byte| byte != ERASED)
        .map_or(0, |last| last + 1);
    Chunks {
        block,
        offset: HEADER_BYTES,
        end,
    }
}

impl<'a> Iterator for Chunks<'a> {
    type Item = Walked<'a>;

    /// The next whole or torn chunk.
    ///
    /// Where no whole chunk starts, the walk looks for the next one in 8-byte steps (or the erased tail) and
    /// reports the bytes it stepped over as one torn chunk. Bytes that are all erased are no chunk at all: a
    /// program that failed before its first byte landed, whose records the writer then retried behind it. A whole
    /// chunk's length is never 0xFF in its high byte, so every chunk starts before the end of the bytes that are
    /// not erased; past that end, the walk is done.
    fn next(&mut self) -> Option<Walked<'a>> {
        while self.offset < self.end {
            let offset = self.offset;
            if let Ok((records, payload)) = decode(&self.block[offset..]) {
                self.offset = offset + chunk_bytes(payload.len());
                return Some(Walked::Chunk(Chunk {
                    offset: offset as u32,
                    records,
                    payload,
                }));
            }
            let mut next = offset + ALIGNMENT;
            while next < self.end && decode(&self.block[next..]).is_err() {
                next += ALIGNMENT;
            }
            self.offset = next;
            let stepped_over = &self.block[offset..next.min(self.end)];
            if stepped_over.iter().any(|&byte| byte != ERASED) {
                let records = self
                    .block
                    .get(offset + 2..offset + 4)
                    .map(|field| u16::from_le_bytes([field[0], field[1]]))
                    .filter(|&records| records != ERASED_LENGTH);
                return Some(Walked::Torn(Torn {
                    offset: offset as u32,
                    records,
                    followed: next < self.end, // the step-over stopped at a whole chunk
                }));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;

    /// The chunk of `payload` with `records`.
    fn chunk(payload: &[u8], records: u16) -> Vec<u8> {
        let mut chunk = Vec::new();
        encode(&mut chunk, payload, records).unwrap();
        chunk
    }

    /// An erased block with `chunks` programmed one after another from the first chunk offset, as the writer
    /// would. Returns the block and each chunk's offset.
    fn block_of(chunks: &[Vec<u8>]) -> (Vec<u8>, Vec<usize>) {
        let mut block = vec![ERASED; BLOCK_BYTES as usize];
        block[..HEADER_BYTES].fill(0x11); // a header, as far as the walk cares
        let mut offset = HEADER_BYTES;
        let mut offsets = Vec::new();
        for chunk in chunks {
            block[offset..offset + chunk.len()].copy_from_slice(chunk);
            offsets.push(offset);
            offset += chunk_bytes(chunk.len() - CHUNK_HEADER_BYTES);
        }
        (block, offsets)
    }

    #[test]
    fn encodes_the_spec_layout_little_endian() {
        let bytes = chunk(b"abc", 0x0102);
        assert_eq!(bytes.len(), 11, "unpadded: the padding is never programmed");
        assert_eq!(bytes[0..4], [3, 0, 2, 1]);
        let expected = crc32(crc32(0, &bytes[..4]), b"abc");
        assert_eq!(
            bytes[4..8],
            expected.to_le_bytes(),
            "over the length, the count and the payload"
        );
        assert_eq!(&bytes[8..], b"abc");
        assert_eq!(crc32(0, &[3, 0, 2, 1, b'a', b'b', b'c']), expected);
    }

    #[test]
    fn round_trips_from_empty_to_the_largest_payload() {
        for length in [0, 1, 7, 8, 100, MAX_PAYLOAD] {
            let payload = vec![0xA5; length];
            let mut bytes = chunk(&payload, 3);
            bytes.extend_from_slice(&[ERASED; 7]); // the padding and tail behind it
            assert_eq!(decode(&bytes), Ok((3, &payload[..])), "length {length}");
        }
        assert_eq!(chunk_bytes(0), 8);
        assert_eq!(chunk_bytes(1), 16);
        assert_eq!(chunk_bytes(8), 16);
        assert_eq!(chunk_bytes(MAX_PAYLOAD), 4040);
        assert_eq!(
            HEADER_BYTES + chunk_bytes(MAX_PAYLOAD),
            BLOCK_BYTES as usize,
            "one largest chunk fills a block"
        );
    }

    #[test]
    fn refuses_to_encode_past_the_largest_payload() {
        let mut out = b"kept".to_vec();
        assert_eq!(encode(&mut out, &[0; MAX_PAYLOAD + 1], 1), Err(ChunkError::Length));
        assert_eq!(out, b"kept", "nothing written");
    }

    #[test]
    fn rejects_erased_flash_a_short_buffer_and_a_bad_length() {
        assert_eq!(decode(&[ERASED; 64]), Err(ChunkError::Erased));
        assert_eq!(decode(&[1, 0, 0]), Err(ChunkError::Short));
        let bytes = chunk(b"abcd", 1);
        assert_eq!(
            decode(&bytes[..11]),
            Err(ChunkError::Length),
            "the block ends inside the payload"
        );
        let mut long = vec![0u8; 5000];
        long[..2].copy_from_slice(&(MAX_PAYLOAD as u16 + 1).to_le_bytes());
        assert_eq!(decode(&long), Err(ChunkError::Length), "longer than any block holds");
    }

    #[test]
    fn rejects_a_damaged_or_torn_chunk() {
        let bytes = chunk(b"abcd", 2);
        for index in 0..bytes.len() {
            let mut damaged = bytes.clone();
            damaged[index] ^= 0x01;
            assert!(
                matches!(decode(&damaged), Err(ChunkError::Crc | ChunkError::Length)),
                "byte {index}"
            );
        }
        for torn in 1..bytes.len() {
            let mut cut = vec![ERASED; 16];
            cut[..torn].copy_from_slice(&bytes[..torn]);
            assert!(decode(&cut).is_err(), "torn after {torn} bytes");
        }
    }

    #[test]
    fn walks_whole_chunks_to_the_erased_tail() {
        let (block, offsets) = block_of(&[chunk(b"one", 1), chunk(&[7; 200], 4), chunk(b"", 0)]);
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        assert_eq!(
            walked,
            [
                Walked::Chunk(Chunk {
                    offset: 56,
                    records: 1,
                    payload: b"one"
                }),
                Walked::Chunk(Chunk {
                    offset: 72,
                    records: 4,
                    payload: &[7; 200]
                }),
                Walked::Chunk(Chunk {
                    offset: 280,
                    records: 0,
                    payload: b""
                }),
            ]
        );
        assert_eq!(offsets, [56, 72, 280]);
        let empty = vec![ERASED; BLOCK_BYTES as usize];
        assert_eq!(chunks(&empty).count(), 0, "a block with no chunk");
        assert_eq!(chunks(&block[..20]).count(), 0, "shorter than its header");
    }

    #[test]
    fn a_walk_up_to_the_published_position_reads_nothing_past_it() {
        let (block, offsets) = block_of(&[chunk(b"one", 1), chunk(b"two", 1)]);
        let walked: Vec<Walked<'_>> = chunks(&block[..offsets[1]]).collect();
        assert_eq!(walked.len(), 1);
    }

    #[test]
    fn a_torn_last_chunk_is_reported_once_with_its_count() {
        let whole = chunk(&[5; 300], 9);
        for torn in [1, 2, 3, 4, 7, 8, 9, 200, whole.len() - 1] {
            let (mut block, offsets) = block_of(&[chunk(b"kept", 2), whole.clone()]);
            block[offsets[1] + torn..offsets[1] + whole.len()].fill(ERASED);
            let walked: Vec<Walked<'_>> = chunks(&block).collect();
            let records = match torn {
                1 | 2 => None,     // the count field never landed
                3 => Some(0xFF09), // half of it did: as read, unverified
                _ => Some(9),
            };
            assert_eq!(walked.len(), 2, "torn after {torn} bytes");
            assert!(matches!(walked[0], Walked::Chunk(Chunk { records: 2, .. })));
            assert_eq!(
                walked[1],
                Walked::Torn(Torn {
                    offset: offsets[1] as u32,
                    records,
                    followed: false, // as a cut leaves it: a loss
                }),
                "torn {torn}"
            );
        }
    }

    #[test]
    fn a_chunk_that_never_landed_is_no_chunk_at_all() {
        let (block, _) = block_of(&[chunk(b"kept", 2)]);
        assert_eq!(
            chunks(&block).count(),
            1,
            "a cut before the first byte leaves the tail erased"
        );
        // a failed program that landed nothing, then its retry at the next offset: an erased gap, not debris
        let retried = chunk(&[4; 40], 3);
        let (mut block, offsets) = block_of(&[chunk(b"kept", 2), retried.clone(), retried.clone()]);
        block[offsets[1]..offsets[1] + retried.len()].fill(ERASED);
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        assert_eq!(walked.len(), 2);
        assert_eq!(
            walked[1],
            Walked::Chunk(Chunk {
                offset: offsets[2] as u32,
                records: 3,
                payload: &[4; 40]
            })
        );
    }

    #[test]
    fn steps_over_a_torn_chunk_to_the_retry_behind_it() {
        // a failed program left a prefix of chunk 2; the writer retried its records at the next offset
        let failed = chunk(&[3; 100], 5);
        let (mut block, offsets) = block_of(&[chunk(b"one", 1), failed.clone(), failed.clone(), chunk(b"four", 1)]);
        block[offsets[1] + 30..offsets[1] + failed.len()].fill(ERASED);
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        let offsets: Vec<u32> = offsets.iter().map(|&offset| offset as u32).collect();
        assert_eq!(
            walked[1],
            Walked::Torn(Torn {
                offset: offsets[1],
                records: Some(5),
                followed: true, // the retry is behind it: no loss
            })
        );
        assert_eq!(
            walked[2],
            Walked::Chunk(Chunk {
                offset: offsets[2],
                records: 5,
                payload: &[3; 100]
            })
        );
        assert!(matches!(walked[3], Walked::Chunk(Chunk { payload: b"four", .. })));
        assert_eq!(walked.len(), 4, "the debris is reported once");
    }

    #[test]
    fn debris_with_only_more_debris_behind_it_is_one_unfollowed_torn_chunk() {
        // a failed program, then its retry cut by the power: nothing whole behind the first debris, a loss
        let failed = chunk(&[3; 100], 5);
        let (mut block, offsets) = block_of(&[chunk(b"one", 1), failed.clone(), failed.clone()]);
        block[offsets[1] + 30..offsets[1] + failed.len()].fill(ERASED);
        block[offsets[2] + 50..offsets[2] + failed.len()].fill(ERASED);
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        assert_eq!(
            walked[1..],
            [Walked::Torn(Torn {
                offset: offsets[1] as u32,
                records: Some(5),
                followed: false,
            })]
        );
    }

    #[test]
    fn a_program_whose_length_never_landed_still_shows_as_torn() {
        // the program stopped inside the length field: it reads 0xFFxx, past any block
        let (mut block, offsets) = block_of(&[chunk(b"one", 1)]);
        block[offsets[0] + 16] = 0x20;
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        assert_eq!(
            walked[1],
            Walked::Torn(Torn {
                offset: 72,
                records: None,
                followed: false,
            })
        );
        assert_eq!(walked.len(), 2);
    }

    #[test]
    fn damage_behind_an_erased_length_is_torn_not_the_tail() {
        // an erased length field over bytes that are not erased: a cut erase, or a failed program's debris
        let (mut block, _) = block_of(&[chunk(b"one", 1)]);
        block[100] = 0;
        let walked: Vec<Walked<'_>> = chunks(&block).collect();
        assert_eq!(
            walked[1],
            Walked::Torn(Torn {
                offset: 72,
                records: None,
                followed: false,
            })
        );
        assert_eq!(walked.len(), 2);
    }
}
