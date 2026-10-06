//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Helpers shared by the ring's integration tests: small geometries, deterministic payloads, and a reader
//! that walks a flash image block by block, as the mover will.

// Each test binary uses its own subset of these helpers.
#![allow(dead_code)]

use flash_ring::frame::{self, Header, Kind};
use flash_ring::scan::{self, Slot};
use flash_ring::{Geometry, MemoryFlash, Options, Ring};

/// 5 blocks of 4 sectors (16 KiB): small enough to cut the power at every operation of a workload, and big
/// enough for the reverse sector order to show.
pub fn small() -> Geometry {
    Geometry::new(5, 4).unwrap()
}

/// The tests' tuning: a 32 KiB spill queue (two small blocks), and one block kept erased ahead in flight.
pub const OPTIONS: Options = Options {
    spill_capacity: 32 << 10,
    reserve_blocks: 1,
};

/// Boot `boot_id`'s BOOT payload, in the spec's shape.
pub fn boot_payload(boot_id: u32) -> Vec<u8> {
    format!("boot={boot_id};session={boot_id:06};board=test;firmware=dev;config_id=0;reset=poweron;card=none")
        .into_bytes()
}

/// A ring for boot `boot_id` over `flash`.
pub fn mount(flash: MemoryFlash, geometry: Geometry, boot_id: u32) -> Ring<MemoryFlash> {
    Ring::mount(flash, geometry, OPTIONS, boot_id, &boot_payload(boot_id)).unwrap()
}

/// A fresh, erased flash just big enough for `geometry`.
pub fn erased(geometry: Geometry) -> MemoryFlash {
    MemoryFlash::new(geometry.bytes())
}

/// The uptime stamped on frame `index`: one 50 ms cut after another from a second in.
pub fn uptime(index: u32) -> u64 {
    1_000_000 + u64::from(index) * 50_000
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

/// The payload of DATA frame `index` of boot `boot`: `<boot>:<index>:` then lowercase filler and a newline,
/// 20 to 700 bytes long. The length comes from a hash, so sizes vary between frames but repeat between runs.
pub fn payload(boot: u32, index: u32) -> Vec<u8> {
    let mut state = boot.wrapping_mul(0x9E37_79B9) ^ index.wrapping_mul(0x85EB_CA6B) ^ 0x2545_F491;
    let length = 20 + (next(&mut state) % 681) as usize;
    let mut payload = format!("{boot}:{index}:").into_bytes();
    while payload.len() < length - 1 {
        payload.push(b'a' + (payload.len() % 26) as u8);
    }
    payload.push(b'\n');
    payload
}

/// The (boot, index) a [`payload`] names.
pub fn identify(payload: &[u8]) -> (u32, u32) {
    let text = std::str::from_utf8(payload).expect("a test payload is text");
    let mut fields = text.split(':');
    let boot = fields.next().unwrap().parse().unwrap();
    let index = fields.next().unwrap().parse().unwrap();
    (boot, index)
}

/// One frame found in an image.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    /// The slot it is in.
    pub slot: u32,
    /// Its offset in the block.
    pub offset: u32,
    /// Its header.
    pub header: Header,
    /// Its payload.
    pub payload: Vec<u8>,
}

/// One slot's bytes in `image`.
pub fn slot_bytes(image: &[u8], geometry: Geometry, slot: u32) -> &[u8] {
    let start = geometry.slot_offset(slot) as usize;
    &image[start..start + geometry.block_bytes() as usize]
}

/// What each slot of `image` holds, by the boot scan's rules.
pub fn classify(image: &[u8], geometry: Geometry) -> Vec<Slot> {
    (0..geometry.blocks())
        .map(|slot| {
            scan::classify(
                slot,
                &slot_bytes(image, geometry, slot)[..frame::HEADER_BYTES],
                geometry,
            )
        })
        .collect()
}

/// The valid blocks of `image` as (block sequence, slot), oldest first.
pub fn blocks(image: &[u8], geometry: Geometry) -> Vec<(u32, u32)> {
    let mut blocks: Vec<(u32, u32)> = classify(image, geometry)
        .into_iter()
        .zip(0..)
        .filter_map(|(slot, index)| slot.block_sequence().map(|block| (block, index)))
        .collect();
    blocks.sort();
    blocks
}

/// The frames of the block in `slot`, up to the first that does not decode or names another block.
pub fn block_frames(image: &[u8], geometry: Geometry, slot: u32, block_sequence: u32) -> Vec<Found> {
    frame::frames(slot_bytes(image, geometry, slot))
        .take_while(|found| found.header.block_sequence == block_sequence)
        .map(|found| Found {
            slot,
            offset: found.offset,
            header: found.header,
            payload: found.payload.to_vec(),
        })
        .collect()
}

/// Every frame of every valid block of `image` whose sequence is at least `from`, oldest block first.
pub fn walk_from(image: &[u8], geometry: Geometry, from: u32) -> Vec<Found> {
    blocks(image, geometry)
        .into_iter()
        .filter(|(block, _)| *block >= from)
        .flat_map(|(block, slot)| block_frames(image, geometry, slot, block))
        .collect()
}

/// Every frame of every valid block of `image`, oldest block first.
pub fn walk(image: &[u8], geometry: Geometry) -> Vec<Found> {
    walk_from(image, geometry, 0)
}

/// The (boot, index) of every DATA frame in `found`, in order.
pub fn data(found: &[Found]) -> Vec<(u32, u32)> {
    found
        .iter()
        .filter(|found| found.header.kind == Kind::Data)
        .map(|found| identify(&found.payload))
        .collect()
}

/// Appends DATA frames of boot `boot` from `index` on until the writer has programmed into block `block` at
/// or past `offset`, and returns the next index. Nothing may spill.
pub fn fill_to(ring: &mut Ring<MemoryFlash>, boot: u32, mut index: u32, block: u32, offset: u32) -> u32 {
    while ring
        .programmed()
        .is_none_or(|at| (at.block_sequence, at.offset) < (block, offset))
    {
        ring.append(Kind::Data, &payload(boot, index), false, uptime(index))
            .unwrap();
        assert_eq!(ring.figures().spill_frames, 0, "filling must not spill");
        index += 1;
    }
    index
}
