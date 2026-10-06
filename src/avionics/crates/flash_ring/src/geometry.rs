//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring's shape: how many 4 KiB blocks (slots) it has, where slot `k` lives, and the erased reserve. A block
//! is one erase sector, so the shape is a single number.

use crate::BLOCK_BYTES;

/// The share of the ring the eraser keeps free (erased, or saved and about to be): 1 %. At it, newest wins erases
/// unsaved blocks (spec "The ring_eraser and newest wins").
const RESERVE_DIVISOR: u32 = 100;

/// How many blocks the ring has. The board's is [`Geometry::BOARD`]; the tests use small ones, so that a power cut
/// can land on every flash operation of a workload within seconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Geometry {
    /// The number of blocks (slots).
    blocks: u32,
}

impl Geometry {
    /// The board's `log` partition: 6144 blocks of 4 KiB, 24 MiB at 0x800000.
    pub const BOARD: Geometry = Geometry { blocks: 6144 };

    /// A geometry of `blocks` slots. Returns None below two blocks (the writer fills one while the eraser frees
    /// another), or past 4 GiB, which a u32 offset cannot address.
    pub fn new(blocks: u32) -> Option<Geometry> {
        let addressable = blocks.checked_mul(BLOCK_BYTES).is_some();
        (blocks >= 2 && addressable).then_some(Geometry { blocks })
    }

    /// The number of blocks (slots).
    pub fn blocks(self) -> u32 {
        self.blocks
    }

    /// The whole ring's size in bytes.
    pub fn bytes(self) -> u32 {
        self.blocks * BLOCK_BYTES
    }

    /// The flash offset of a slot's first byte.
    pub fn slot_offset(self, slot: u32) -> u32 {
        slot * BLOCK_BYTES
    }

    /// The blocks the eraser keeps free: about 1 % of the ring, at least one. 61 blocks on the board (~245 KB,
    /// about 15 s at the flight rate). Fewer erased slots is `ring-low`, and the arm gate does not pass (spec "The
    /// ring_eraser and newest wins"). The operator decided it is a constant with no config key.
    ///
    /// Newest wins erases the oldest unsaved block once no more than this many are free, so it keeps one block
    /// more: the writer runs above the eraser and may open a block before the eraser's next step, and that open
    /// must not take the ring under the reserve. Without the margin, the steady state of newest wins (no card, or a
    /// pad longer than the ring) would flicker `ring-low` and the gate on every block the writer opens.
    pub fn reserve_blocks(self) -> u32 {
        (self.blocks / RESERVE_DIVISOR).max(1)
    }
}

#[cfg(test)]
mod tests {
    use super::Geometry;

    #[test]
    fn the_board_is_24_mebibytes_of_4_kibibyte_blocks() {
        assert_eq!(Geometry::BOARD.bytes(), 24 << 20);
        assert_eq!(Geometry::new(6144), Some(Geometry::BOARD));
        assert_eq!(Geometry::BOARD.slot_offset(2), 8192);
        assert_eq!(Geometry::BOARD.reserve_blocks(), 61, "about 1 %, as the spec counts it");
    }

    #[test]
    fn a_small_ring_still_keeps_one_block_in_reserve() {
        assert_eq!(Geometry::new(8).unwrap().reserve_blocks(), 1);
        assert_eq!(Geometry::new(250).unwrap().reserve_blocks(), 2);
    }

    #[test]
    fn refuses_a_ring_the_writer_cannot_run_or_address() {
        assert_eq!(Geometry::new(0), None);
        assert_eq!(
            Geometry::new(1),
            None,
            "one block leaves nothing to free while it fills"
        );
        assert_eq!(Geometry::new(1 << 20), None, "4 GiB is past a u32 offset");
        assert!(Geometry::new((1 << 20) - 1).is_some());
        assert!(Geometry::new(2).is_some());
    }
}
