//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Helpers shared by the ring's integration tests: a small geometry, deterministic chunk payloads, the three parts
//! of a boot over one in-memory flash, and an oracle that reads a flash image block by block in sequence order,
//! as the saver will. The oracle classifies a slot by its own reading of the header bytes, so a scan bug cannot
//! grade itself.

// Each test binary uses its own subset of these helpers.
#![allow(dead_code)]

use flash_ring::chunk::{self, Torn, Walked};
use flash_ring::header::{BlockHeader, HEADER_BYTES, Session};
use flash_ring::{BLOCK_BYTES, BlockTable, Eraser, Erasure, Geometry, Hints, MemoryFlash, Ring, Scan, Slot, scan};

/// 8 blocks: small enough to cut the power at every operation of a workload that wraps the ring twice, with
/// a reserve of one block.
pub fn small() -> Geometry {
    Geometry::new(8).unwrap()
}

/// Boot `boot`'s session label, `%06u` as the board prints it.
pub fn session(boot: u32) -> Session {
    Session::new(&format!("{boot:06}")).unwrap()
}

/// A fresh, erased flash just big enough for `geometry`.
pub fn erased(geometry: Geometry) -> MemoryFlash {
    MemoryFlash::new(geometry.bytes())
}

/// The bring-up scan of `flash`, which must succeed.
pub fn bring_up(flash: &MemoryFlash, geometry: Geometry, hints: Hints) -> Scan {
    scan(flash, geometry, hints).unwrap()
}

/// Boot `boot`'s writer over `flash`, after `scanned`: a boot on the pad (no crumb in flight).
pub fn writer<'a>(flash: &'a MemoryFlash, scanned: &'a Scan, boot: u32) -> Ring<'a, &'a MemoryFlash> {
    Ring::new(flash, &scanned.table, scanned.next_sequence, boot, session(boot), false).unwrap()
}

/// The eraser over `flash`, sharing `table`, with no trusted saver hint (a first boot, or another card).
pub fn eraser<'a>(flash: &'a MemoryFlash, table: &'a BlockTable) -> Eraser<'a, &'a MemoryFlash> {
    Eraser::new(flash, table, None).unwrap()
}

/// A step of xorshift32: deterministic noise for sizes and foreign bytes.
pub fn next(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

/// `length` noise bytes from `seed`.
pub fn noise(seed: u32, length: usize) -> Vec<u8> {
    let mut state = seed.max(1);
    (0..length).map(|_| next(&mut state) as u8).collect()
}

/// The payload of chunk `index` of boot `boot`: `<boot>:<index>:` then lowercase filler, 20 to 1200 bytes. The
/// length comes from a hash, so sizes vary between chunks but repeat between runs. To the ring a payload is
/// opaque; the recorder fills it with whole records.
pub fn payload(boot: u32, index: u32) -> Vec<u8> {
    let mut state = boot.wrapping_mul(0x9E37_79B9) ^ index.wrapping_mul(0x85EB_CA6B) ^ 0x2545_F491;
    let length = 20 + (next(&mut state) % 1181) as usize;
    let mut payload = format!("{boot}:{index}:").into_bytes();
    while payload.len() < length {
        payload.push(b'a' + (payload.len() % 26) as u8);
    }
    payload
}

/// The record count chunk `index` claims.
pub fn records(index: u32) -> u16 {
    1 + (index % 7) as u16
}

/// The (boot, index) a [`payload`] names.
pub fn identify(payload: &[u8]) -> (u32, u32) {
    let text = std::str::from_utf8(payload).expect("a test payload is text");
    let mut fields = text.split(':');
    let boot = fields.next().unwrap().parse().unwrap();
    let index = fields.next().unwrap().parse().unwrap();
    (boot, index)
}

/// One slot's bytes in `image`.
pub fn slot_bytes(image: &[u8], slot: u32) -> &[u8] {
    let start = (slot * BLOCK_BYTES) as usize;
    &image[start..start + BLOCK_BYTES as usize]
}

/// The oracle's reading of a slot: all 0xFF is erased; the magic and a header CRC that checks make a block, of
/// this layout when its version byte is 1; anything else is foreign. Written from the spec's table, not from the
/// crate's decoder.
pub fn oracle_slot(bytes: &[u8]) -> Slot {
    let header = &bytes[..56];
    if header.iter().all(|&byte| byte == 0xFF) {
        return Slot::Erased;
    }
    let crc = recorder_wire::crc32(0, &header[..52]).to_le_bytes();
    if header[..4] != [0xF7, 0x43, 0x42, 0x31] || header[52..56] != crc {
        return Slot::Foreign;
    }
    let block_sequence = u32::from_le_bytes(header[8..12].try_into().unwrap());
    let flight = header[5] & 1 == 1;
    match header[4] {
        1 => Slot::Valid { block_sequence, flight },
        version => Slot::OtherVersion {
            block_sequence,
            flight,
            version,
        },
    }
}

/// The oracle's reading of every slot of `image`.
pub fn oracle_slots(image: &[u8], geometry: Geometry) -> Vec<Slot> {
    (0..geometry.blocks())
        .map(|slot| oracle_slot(slot_bytes(image, slot)))
        .collect()
}

/// A slot as the table holds it, as the scan of the same bytes would hold it: an open block reads as valid.
pub fn as_scanned(slot: Slot) -> Slot {
    match slot {
        Slot::Open { block_sequence, flight } => Slot::Valid { block_sequence, flight },
        other => other,
    }
}

/// One whole chunk of an image.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    /// The slot it is in.
    pub slot: u32,
    /// Its block's header.
    pub header: BlockHeader,
    /// Its offset in the block.
    pub offset: u32,
    /// Its record count.
    pub records: u16,
    /// Its payload.
    pub payload: Vec<u8>,
}

/// A block of an image: its slot, its header, and what its chunk walk found.
#[derive(Clone, Debug)]
pub struct Block {
    /// The slot.
    pub slot: u32,
    /// The header.
    pub header: BlockHeader,
    /// Its whole chunks, in address order.
    pub chunks: Vec<Found>,
    /// Its torn chunks.
    pub torn: Vec<Torn>,
}

/// The blocks of this layout in `image`, oldest first, each with its chunk walk: what the saver reads.
pub fn blocks(image: &[u8], geometry: Geometry) -> Vec<Block> {
    let mut blocks: Vec<Block> = Vec::new();
    for (slot, state) in (0..).zip(oracle_slots(image, geometry)) {
        if !matches!(state, Slot::Valid { .. }) {
            continue;
        }
        let bytes = slot_bytes(image, slot);
        let header = BlockHeader::decode(&bytes[..HEADER_BYTES]).unwrap();
        let mut block = Block {
            slot,
            header,
            chunks: Vec::new(),
            torn: Vec::new(),
        };
        for walked in chunk::chunks(bytes) {
            match walked {
                Walked::Chunk(found) => block.chunks.push(Found {
                    slot,
                    header,
                    offset: found.offset,
                    records: found.records,
                    payload: found.payload.to_vec(),
                }),
                Walked::Torn(torn) => block.torn.push(torn),
            }
        }
        blocks.push(block);
    }
    blocks.sort_by_key(|block| block.header.block_sequence);
    blocks
}

/// Every whole chunk of every block of `image`, oldest block first.
pub fn walk(image: &[u8], geometry: Geometry) -> Vec<Found> {
    blocks(image, geometry)
        .into_iter()
        .flat_map(|block| block.chunks)
        .collect()
}

/// The (boot, index) of every chunk in `found`, in order.
pub fn data(found: &[Found]) -> Vec<(u32, u32)> {
    found.iter().map(|found| identify(&found.payload)).collect()
}

/// The slot the table holds block `block_sequence` in (open or valid).
pub fn slot_of(table: &BlockTable, block_sequence: u32) -> u32 {
    (0..table.geometry().blocks())
        .find(|&slot| match table.slot(slot) {
            Slot::Open {
                block_sequence: found, ..
            }
            | Slot::Valid {
                block_sequence: found, ..
            } => found == block_sequence,
            _ => false,
        })
        .expect("the block is in the table")
}

/// Every slot of `table`, now.
pub fn snapshot(table: &BlockTable) -> Vec<Slot> {
    (0..table.geometry().blocks()).map(|slot| table.slot(slot)).collect()
}

/// The sequences of the losses among `erasures`, in order.
pub fn lost(erasures: &[Erasure]) -> Vec<u32> {
    erasures
        .iter()
        .filter_map(|erasure| match erasure {
            Erasure::Lost(loss) => Some(loss.block_sequence),
            _ => None,
        })
        .collect()
}
