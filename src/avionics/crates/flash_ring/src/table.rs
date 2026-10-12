//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The block table (doc/specs/avionics-port.md, "Bring-up: the scan, the positions and the session-start
//! record"): what each of the ring's slots holds, shared by the three ring tasks through atomics. It has 5120
//! entries on the board (48 KiB in PSRAM): a state word and a block sequence per slot, each an AtomicU32.
//!
//! Who changes a slot is fixed by its state, so the tasks never race on one:
//! * the writer moves a slot out of ERASED: to OPEN when it opens the block there, or to FOREIGN when its check
//!   finds the slot not wholly erased (a cut erase left an erased header over dirty bytes), or cannot read it. It
//!   moves its own OPEN slot to VALID when it closes the block;
//! * the eraser moves VALID, OTHER-VERSION and FOREIGN slots to ERASED, and nothing else;
//! * the saver, the figures and the tools only read.
//!
//! A slot is written sequence first, then its state with Release; a reader loads the state with Acquire, then the
//! sequence. A reader racing an erase and a reopen of the same slot can still pair an old state with a new
//! sequence, so the saver checks the header it reads from flash against the table before it trusts a block.

use alloc::boxed::Box;
use core::sync::atomic::{AtomicU32, Ordering};

use crate::{BLOCK_BYTES, Geometry, Position};

/// The state word's kind field.
const KIND_MASK: u32 = 0b111;

/// Kind: all 0xFF.
const KIND_ERASED: u32 = 0;

/// Kind: the writer's open block.
const KIND_OPEN: u32 = 1;

/// Kind: a closed block of this layout version.
const KIND_VALID: u32 = 2;

/// Kind: a block of another layout version.
const KIND_OTHER_VERSION: u32 = 3;

/// Kind: neither a block header nor erased.
const KIND_FOREIGN: u32 = 4;

/// The state word's flight bit.
const FLIGHT_BIT: u32 = 1 << 3;

/// Where the state word holds another layout's version byte.
const VERSION_SHIFT: u32 = 8;

/// What a slot holds.
///
/// The spec's UNSAVED and SAVED states are both [`Slot::Valid`]: which one a block is depends on the saver's
/// position, which the table does not keep ([`BlockTable::figures`] takes it).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    /// All 0xFF, as far as its header shows. The writer reads the whole block before it opens one.
    Erased,
    /// The writer's current block, this boot.
    Open {
        /// The block's sequence number.
        block_sequence: u32,
        /// Opened in flight.
        flight: bool,
    },
    /// A closed block of this layout version: unsaved, or saved and about to be erased.
    Valid {
        /// The block's sequence number.
        block_sequence: u32,
        /// Opened in flight.
        flight: bool,
    },
    /// A whole block header of another layout version: ring data another build wrote (a newer one before an
    /// older image booted, or the reverse). Its chunks do not decode here, so it is never saved. It is kept
    /// rather than taken for foreign, since it may be a flight nobody has copied yet: only newest wins erases it,
    /// in sequence order and flight blocks last, as its header says.
    OtherVersion {
        /// The block's sequence number, which the header keeps where it is in every version.
        block_sequence: u32,
        /// Opened in flight.
        flight: bool,
        /// Its layout version.
        version: u8,
    },
    /// Anything else at the block start: MicroPython-era data, a bench pattern, a header torn by a cut, a cut
    /// erase, or an erased header over dirty bytes. It holds nothing recoverable, so the eraser takes it first
    /// and logs no loss.
    Foreign,
}

impl Slot {
    /// The sequence of a block that holds ring data (open, valid or of another version). None for the others.
    pub fn block_sequence(self) -> Option<u32> {
        match self {
            Slot::Open { block_sequence, .. }
            | Slot::Valid { block_sequence, .. }
            | Slot::OtherVersion { block_sequence, .. } => Some(block_sequence),
            Slot::Erased | Slot::Foreign => None,
        }
    }

    /// The state word: the kind, the flight bit, and another layout's version.
    fn pack(self) -> u32 {
        let flight_bit = |flight: bool| if flight { FLIGHT_BIT } else { 0 };
        match self {
            Slot::Erased => KIND_ERASED,
            Slot::Open { flight, .. } => KIND_OPEN | flight_bit(flight),
            Slot::Valid { flight, .. } => KIND_VALID | flight_bit(flight),
            Slot::OtherVersion { flight, version, .. } => {
                KIND_OTHER_VERSION | flight_bit(flight) | u32::from(version) << VERSION_SHIFT
            }
            Slot::Foreign => KIND_FOREIGN,
        }
    }

    /// The slot a state word and its sequence describe. An unknown kind cannot be stored, and reads as foreign.
    fn unpack(word: u32, block_sequence: u32) -> Slot {
        let flight = word & FLIGHT_BIT != 0;
        match word & KIND_MASK {
            KIND_ERASED => Slot::Erased,
            KIND_OPEN => Slot::Open { block_sequence, flight },
            KIND_VALID => Slot::Valid { block_sequence, flight },
            KIND_OTHER_VERSION => Slot::OtherVersion {
                block_sequence,
                flight,
                version: (word >> VERSION_SHIFT) as u8,
            },
            _ => Slot::Foreign,
        }
    }
}

/// The ring's budget, for health and inspect (`ring:{…}`), the degraded tokens and the arm gate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Figures {
    /// Erased slots. The writer takes the next erased slot after its own wherever it lies, so every one of them
    /// is ahead of it. Fewer than [`Geometry::reserve_blocks`] is `ring-low`.
    pub erased_ahead: u32,
    /// Closed blocks the saver has passed, which the eraser erases within 100-200 ms: free space a moment away.
    pub saved: u32,
    /// Closed blocks of this layout the saver has not passed: the flash-to-SD backlog (`ring_unsaved_s`). The open
    /// block is not counted.
    pub unsaved: u32,
    /// Blocks of another layout version: kept, never saved by this build (see [`Slot::OtherVersion`]).
    pub other_version: u32,
    /// Foreign slots waiting for their erase. Any at all after a boot whose scan found no block
    /// ([`crate::Scan::unformatted`]) is `ring-unformatted`.
    pub foreign: u32,
    /// Unsaved blocks newest wins has erased this boot (`ring.lost`).
    pub lost: u32,
    /// Slots the eraser gave up this boot, after [`crate::ERASE_ATTEMPTS`] failed erases in a row: bad sectors,
    /// passed over until the next boot. Each is still counted above by what it holds.
    pub bad: u32,
}

impl Figures {
    /// `ring_erased_s`: how long the writer goes before newest wins, at `bytes_per_second` (the measured write
    /// rate). Erased and saved blocks count, since the saved ones are erased within 200 ms. A zero rate gives
    /// infinity: nothing is written, so newest wins never comes.
    pub fn erased_seconds(&self, bytes_per_second: f32) -> f32 {
        seconds(self.erased_ahead + self.saved, bytes_per_second)
    }

    /// `ring_unsaved_s`: the flash-to-SD backlog at `bytes_per_second`, by the same measure. `saver-behind` is this
    /// growing for more than 60 s with a card mounted.
    pub fn unsaved_seconds(&self, bytes_per_second: f32) -> f32 {
        seconds(self.unsaved, bytes_per_second)
    }
}

/// The time `blocks` blocks take to write at `bytes_per_second`: 0 for none, infinity at a zero rate.
fn seconds(blocks: u32, bytes_per_second: f32) -> f32 {
    if blocks == 0 {
        return 0.0;
    }
    blocks as f32 * BLOCK_BYTES as f32 / bytes_per_second
}

/// What each slot holds, shared by the writer, the eraser and the saver through atomics.
#[derive(Debug)]
pub struct BlockTable {
    /// The ring's shape.
    geometry: Geometry,
    /// Each slot's state word (see [`Slot::pack`]).
    states: Box<[AtomicU32]>,
    /// Each slot's block sequence, meaningful while the state holds a block.
    sequences: Box<[AtomicU32]>,
    /// The slot the writer last opened a block in (at bring-up, the newest block's): it opens the next erased
    /// slot after this one, and the eraser formats foreign slots in that order too, so the writer's next slot is
    /// the first one freed.
    cursor: AtomicU32,
    /// Unsaved blocks newest wins erased this boot.
    lost: AtomicU32,
    /// Slots the eraser gave up this boot.
    bad: AtomicU32,
}

impl BlockTable {
    /// A table of `geometry`'s slots, every one erased, with the writer's cursor on the last slot (so it opens
    /// slot 0 first). The bring-up [`crate::scan()`] fills it.
    pub(crate) fn new(geometry: Geometry) -> BlockTable {
        let slots = || (0..geometry.blocks()).map(|_| AtomicU32::new(0)).collect();
        BlockTable {
            geometry,
            states: slots(),
            sequences: slots(),
            cursor: AtomicU32::new(geometry.blocks() - 1),
            lost: AtomicU32::new(0),
            bad: AtomicU32::new(0),
        }
    }

    /// The ring's shape.
    pub fn geometry(&self) -> Geometry {
        self.geometry
    }

    /// What `slot` holds now.
    pub fn slot(&self, slot: u32) -> Slot {
        let word = self.states[slot as usize].load(Ordering::Acquire);
        Slot::unpack(word, self.sequences[slot as usize].load(Ordering::Relaxed))
    }

    /// Records that `slot` holds `state`: the sequence first, then the state word with Release.
    pub(crate) fn set(&self, slot: u32, state: Slot) {
        if let Some(block_sequence) = state.block_sequence() {
            self.sequences[slot as usize].store(block_sequence, Ordering::Relaxed);
        }
        self.states[slot as usize].store(state.pack(), Ordering::Release);
    }

    /// The slot the writer last opened a block in.
    pub(crate) fn cursor(&self) -> u32 {
        self.cursor.load(Ordering::Relaxed)
    }

    /// Moves the writer's cursor to `slot`.
    pub(crate) fn set_cursor(&self, slot: u32) {
        self.cursor.store(slot, Ordering::Relaxed);
    }

    /// Counts one unsaved block newest wins erased.
    pub(crate) fn count_loss(&self) {
        self.lost.fetch_add(1, Ordering::Relaxed);
    }

    /// Counts one slot the eraser gave up.
    pub(crate) fn count_bad(&self) {
        self.bad.fetch_add(1, Ordering::Relaxed);
    }

    /// The budget figures, walking every slot (5120 pairs of loads on the board, tens of microseconds). `saver` is
    /// the saver's published position: the blocks before its block are saved. None (no card, or no saver yet)
    /// counts every closed block unsaved.
    pub fn figures(&self, saver: Option<Position>) -> Figures {
        let saved_before = saver.map_or(0, |saver| saver.block_sequence);
        let mut figures = Figures {
            lost: self.lost.load(Ordering::Relaxed),
            bad: self.bad.load(Ordering::Relaxed),
            ..Figures::default()
        };
        for slot in 0..self.geometry.blocks() {
            match self.slot(slot) {
                Slot::Erased => figures.erased_ahead += 1,
                Slot::Valid { block_sequence, .. } if block_sequence < saved_before => figures.saved += 1,
                Slot::Valid { .. } => figures.unsaved += 1,
                Slot::OtherVersion { .. } => figures.other_version += 1,
                Slot::Foreign => figures.foreign += 1,
                Slot::Open { .. } => {}
            }
        }
        figures
    }

    /// The block of this layout, closed or the writer's open one, with the lowest sequence at or after `from`, as
    /// (slot, block sequence); None when there is none. One walk over every slot, as [`BlockTable::figures`].
    ///
    /// The ring_saver reads blocks in sequence order, which the slots no longer keep once the ring wraps or newest
    /// wins has erased out of order, so it asks here for the block it stands in, and for the one after it. When
    /// the block it stands in is gone (newest wins erased it unsaved, and the eraser logged the loss), the answer
    /// is the next one. A block of another layout version is never the saver's to read, so it is never named.
    pub fn next_block(&self, from: u32) -> Option<(u32, u32)> {
        let mut next: Option<(u32, u32)> = None;
        for slot in 0..self.geometry.blocks() {
            if let Slot::Valid { block_sequence, .. } | Slot::Open { block_sequence, .. } = self.slot(slot)
                && block_sequence >= from
                && next.is_none_or(|(_, lowest)| block_sequence < lowest)
            {
                next = Some((slot, block_sequence));
            }
        }
        next
    }

    /// Where the saver starts (spec "Bring-up", step 4): at its `hint` from the `cfg` NVS when the hint names a
    /// block of this layout still in flash, else at the oldest such block, so nothing in flash is left out. With
    /// none at all it is the ring's start, before any block this boot will open.
    ///
    /// The firmware passes None for the hint when the mounted card is not the one the previous boot's
    /// session-start record names, so a new card gets everything still in flash. A stale or lost hint costs at
    /// most about a second of re-saved records, since saved blocks are erased within 200 ms: duplicates, never a
    /// loss (spec "Duplicates").
    pub fn saver_start(&self, hint: Option<Position>) -> Position {
        let mut oldest: Option<u32> = None;
        let mut named = false;
        for slot in 0..self.geometry.blocks() {
            if let Slot::Valid { block_sequence, .. } | Slot::Open { block_sequence, .. } = self.slot(slot) {
                named |= hint.is_some_and(|hint| hint.block_sequence == block_sequence);
                oldest = Some(oldest.map_or(block_sequence, |oldest| oldest.min(block_sequence)));
            }
        }
        match hint {
            Some(hint) if named && hint.offset <= BLOCK_BYTES => hint,
            _ => Position {
                block_sequence: oldest.unwrap_or(0),
                offset: 0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_state_survives_the_table() {
        let table = BlockTable::new(Geometry::new(4).unwrap());
        let states = [
            Slot::Erased,
            Slot::Open {
                block_sequence: 7,
                flight: true,
            },
            Slot::Valid {
                block_sequence: u32::MAX,
                flight: false,
            },
            Slot::OtherVersion {
                block_sequence: 9,
                flight: true,
                version: 0xFE,
            },
            Slot::Foreign,
        ];
        for state in states {
            table.set(2, state);
            assert_eq!(table.slot(2), state);
        }
        assert_eq!(Slot::unpack(KIND_MASK, 3), Slot::Foreign, "an unknown kind is foreign");
        assert_eq!(table.slot(0), Slot::Erased, "a new table is erased");
        assert_eq!(table.cursor(), 3, "so the writer starts at slot 0");
    }

    #[test]
    fn counts_every_state_against_the_saver() {
        let table = BlockTable::new(Geometry::new(8).unwrap());
        let valid = |block_sequence, flight| Slot::Valid { block_sequence, flight };
        table.set(0, valid(10, false));
        table.set(1, valid(11, true));
        table.set(2, valid(12, false));
        table.set(
            3,
            Slot::Open {
                block_sequence: 13,
                flight: false,
            },
        );
        table.set(4, Slot::Foreign);
        table.set(
            5,
            Slot::OtherVersion {
                block_sequence: 3,
                flight: false,
                version: 2,
            },
        );
        table.count_loss();
        table.count_bad();
        table.count_bad();
        let saver = Position {
            block_sequence: 12,
            offset: 100,
        };
        let figures = Figures {
            erased_ahead: 2,
            saved: 2,
            unsaved: 1,
            other_version: 1,
            foreign: 1,
            lost: 1,
            bad: 2,
        };
        assert_eq!(table.figures(Some(saver)), figures, "block 12 is only part saved");
        assert_eq!(
            table.figures(None).unsaved,
            3,
            "no saver: every closed block is unsaved"
        );
    }

    #[test]
    fn the_seconds_follow_the_write_rate() {
        let figures = Figures {
            erased_ahead: 3,
            saved: 1,
            unsaved: 2,
            ..Figures::default()
        };
        assert_eq!(figures.erased_seconds(4096.0), 4.0);
        assert_eq!(figures.unsaved_seconds(8192.0), 1.0);
        assert_eq!(
            figures.erased_seconds(0.0),
            f32::INFINITY,
            "nothing written: never full"
        );
        assert_eq!(Figures::default().unsaved_seconds(0.0), 0.0, "no backlog at any rate");
    }

    #[test]
    fn the_next_block_is_the_lowest_sequence_of_this_layout_from_where_the_saver_stands() {
        let table = BlockTable::new(Geometry::new(8).unwrap());
        assert_eq!(table.next_block(0), None, "an empty ring");
        // out of slot order, as after a wrap: 41 in slot 6, 43 in slot 1, the open 44 in slot 2
        let valid = |block_sequence| Slot::Valid {
            block_sequence,
            flight: false,
        };
        table.set(6, valid(41));
        table.set(1, valid(43));
        table.set(
            2,
            Slot::Open {
                block_sequence: 44,
                flight: true,
            },
        );
        table.set(
            0,
            Slot::OtherVersion {
                block_sequence: 42,
                flight: false,
                version: 2,
            },
        );
        table.set(3, Slot::Foreign);
        assert_eq!(table.next_block(0), Some((6, 41)));
        assert_eq!(table.next_block(41), Some((6, 41)), "the block the saver stands in");
        assert_eq!(
            table.next_block(42),
            Some((1, 43)),
            "another layout's block is never the saver's"
        );
        assert_eq!(table.next_block(44), Some((2, 44)), "the writer's open block");
        // negative: past the newest block there is nothing to read
        assert_eq!(table.next_block(45), None);
        assert_eq!(table.next_block(u32::MAX), None);
    }

    #[test]
    fn the_saver_starts_at_its_hint_or_the_oldest_block() {
        let table = BlockTable::new(Geometry::new(8).unwrap());
        assert_eq!(
            table.saver_start(None),
            Position::default(),
            "an empty ring: from the start"
        );
        for (slot, block_sequence) in [(5, 40), (6, 41), (7, 42)] {
            table.set(
                slot,
                Slot::Valid {
                    block_sequence,
                    flight: false,
                },
            );
        }
        table.set(
            0,
            Slot::OtherVersion {
                block_sequence: 39,
                flight: false,
                version: 2,
            },
        );
        let oldest = Position {
            block_sequence: 40,
            offset: 0,
        };
        let hint = Position {
            block_sequence: 41,
            offset: 2000,
        };
        assert_eq!(table.saver_start(Some(hint)), hint);
        assert_eq!(
            table.saver_start(None),
            oldest,
            "another card: everything this build can save"
        );
        for stale in [(39, 0), (43, 0), (2, 0), (41, BLOCK_BYTES + 8)] {
            let stale = Position {
                block_sequence: stale.0,
                offset: stale.1,
            };
            assert_eq!(
                table.saver_start(Some(stale)),
                oldest,
                "{stale:?} names no block it can save"
            );
        }
    }
}
