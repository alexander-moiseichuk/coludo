//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring's shape: how many blocks of how many 4 KiB sectors, and where block `k` lives (slot `k % blocks`).

use crate::SECTOR_BYTES;

/// How many blocks the ring has and how big each is. The board's is [`Geometry::BOARD`]. The tests use smaller
/// ones, so that a power cut can land on every flash operation of a workload within seconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    /// The number of blocks (slots).
    blocks: u32,
    /// The number of 4 KiB sectors in a block.
    sectors_per_block: u32,
}

impl Geometry {
    /// The board's `log` partition: 384 blocks of 64 KiB (16 sectors each), 24 MiB.
    pub const BOARD: Geometry = Geometry {
        blocks: 384,
        sectors_per_block: 16,
    };

    /// A geometry. Returns None when it has fewer than two blocks (the writer needs one open block and one
    /// ahead of it to erase), no sectors, or more than 4 GiB, which a u32 offset cannot address.
    pub fn new(blocks: u32, sectors_per_block: u32) -> Option<Geometry> {
        let usable = blocks >= 2 && sectors_per_block >= 1;
        let addressable = blocks
            .checked_mul(sectors_per_block)
            .and_then(|sectors| sectors.checked_mul(SECTOR_BYTES))
            .is_some();
        (usable && addressable).then_some(Geometry {
            blocks,
            sectors_per_block,
        })
    }

    /// The number of blocks (slots).
    pub fn blocks(self) -> u32 {
        self.blocks
    }

    /// The number of 4 KiB sectors in a block.
    pub fn sectors_per_block(self) -> u32 {
        self.sectors_per_block
    }

    /// A block's size in bytes.
    pub fn block_bytes(self) -> u32 {
        self.sectors_per_block * SECTOR_BYTES
    }

    /// The whole ring's size in bytes.
    pub fn bytes(self) -> u32 {
        self.blocks * self.block_bytes()
    }

    /// The slot block `block_sequence` lives in: the ring is a strict FIFO, so block `k` is always in slot
    /// `k % blocks`.
    pub fn slot(self, block_sequence: u32) -> u32 {
        block_sequence % self.blocks
    }

    /// The flash offset of a slot's first byte.
    pub fn slot_offset(self, slot: u32) -> u32 {
        slot * self.block_bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::Geometry;

    #[test]
    fn the_board_is_24_mebibytes_of_64_kibibyte_blocks() {
        assert_eq!(Geometry::BOARD.block_bytes(), 64 << 10);
        assert_eq!(Geometry::BOARD.bytes(), 24 << 20);
        assert_eq!(Geometry::new(384, 16), Some(Geometry::BOARD));
        assert_eq!(Geometry::BOARD.slot(384 + 5), 5);
        assert_eq!(Geometry::BOARD.slot_offset(2), 128 << 10);
    }

    #[test]
    fn refuses_a_ring_the_writer_cannot_run_or_address() {
        assert_eq!(
            Geometry::new(1, 16),
            None,
            "one block leaves nothing to erase ahead of the writer"
        );
        assert_eq!(Geometry::new(4, 0), None, "a block needs a sector");
        assert_eq!(Geometry::new(1 << 16, 1 << 4), None, "4 GiB is past a u32 offset");
        assert!(Geometry::new(2, 1).is_some());
    }
}
