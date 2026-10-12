//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flash ring (doc/specs/avionics-port.md, "Storage design"): the 20 MiB `log` partition as 5120 blocks of one
//! 4 KiB erase sector each. Power can drop at any moment, so getting every record into flash fast is the critical
//! path, and the SD copy comes second. Three firmware tasks share the partition, each in one role:
//! * the ring_writer ([`Ring`]) appends chunks of binary records, verbatim, into its open block, and opens the next
//!   erased block (its [`header`] first) when a chunk does not fit;
//! * the ring_eraser ([`Eraser`]) erases blocks the saver has passed, and when fewer than about 1 % of the blocks
//!   are free it erases the oldest block even unsaved (newest wins), non-flight blocks first;
//! * the ring_saver reads blocks in sequence order ([`BlockTable::next_block`]) and walks their [`chunk`]s, to
//!   print them for the SD. It lives in crates/recorder (`Saver`), beside the record decoder it prints with: this
//!   crate cannot depend on that one, nor on `std::fs`. The eraser erases behind the position it publishes.
//!
//! They coordinate through the [`BlockTable`], one pair of AtomicU32 per slot that the bring-up [`scan()`] builds,
//! and through two published positions, which the firmware keeps (each two AtomicU32 under a seqlock: the P4 has
//! no 64-bit atomics). Nothing here takes a lock. The writer acts only on erased slots and its own open one, and
//! the eraser only on the others, so the two never act on the same slot; the flash driver serialises the calls.
//!
//! A flash error never wedges a task on one slot: the writer hands a slot it cannot read to the eraser and opens
//! the next one, and the eraser gives up a slot whose erase keeps failing ([`ERASE_ATTEMPTS`]) and erases the next
//! one by its rules. One bad sector costs one slot, never the logging.
//!
//! Builds meet each other's blocks: after an update, or when a reboot in flight comes back on an older image. The
//! block header and the 4 KiB slot grid are therefore frozen across builds, and everything behind the header is
//! versioned ([`header`]).
//!
//! The crate ports no src/glider module (plan B sent text over a UART to the Luckfox), so the spec is the plan.
//! Everything runs over the [`Flash`] trait: the firmware plugs in `esp_partition_*`, and the host tests run the
//! same code over [`MemoryFlash`], an in-memory NOR flash that cuts power on demand. `no_std` + `alloc`.

#![no_std]

extern crate alloc;

mod backlog;
pub mod chunk;
mod eraser;
mod geometry;
pub mod header;
mod memory;
mod ring;
mod scan;
mod table;

pub use backlog::{BacklogTrend, SAVER_BEHIND_MS};
pub use eraser::{ERASE_ATTEMPTS, ERASE_DELAY_MS, Eraser, Erasure, Loss};
pub use geometry::Geometry;
pub use memory::{MemoryError, MemoryFlash};
pub use ring::Ring;
pub use scan::{HINT_REACH, Hints, Scan, classify, scan};
pub use table::{BlockTable, Figures, Slot};

/// A block: one NOR erase sector. The eraser erases one per step, because a 4 KiB erase (13-17 ms) fits the
/// writer's power-cut bound and a 64 KiB one (95-107 ms) does not.
///
/// **Frozen across builds, with the block header** ([`header`]). Every build reads a block header at each 4 KiB
/// slot start. A build with another block size would leave chunk bytes at some of those starts, and an older
/// build would take them for foreign slots and erase them first, with no loss line, cutting the newer build's
/// blocks in half. No layout version can say "my blocks are 8 KiB" to a build that predates it.
pub const BLOCK_BYTES: u32 = 4096;

/// A NOR flash region (the ring's partition); offsets are bytes from the region's start.
///
/// The ring relies on NOR semantics, and [`MemoryFlash`] enforces them. A program only clears bits (1 -> 0), so it
/// only ever lands on erased bytes. Only an erase sets bits back, one whole 4 KiB sector at a time. Each task holds
/// its own handle to the same region (on the board a partition pointer, on the host a `&MemoryFlash`).
pub trait Flash {
    /// What a failed call reports (an `EspError` on the board, a [`MemoryError`] on the host).
    type Error: core::fmt::Debug;

    /// Reads `buffer.len()` bytes at `offset`.
    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Programs `data` at `offset`, 1 -> 0 bits only and in address order. The ring makes one call per block
    /// header and one per chunk, its header first, so a cut can tear only a call's tail.
    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Erases the sector at `offset` (a multiple of [`BLOCK_BYTES`]) back to 0xFF.
    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error>;

    /// The region's size in bytes.
    fn size(&self) -> u32;
}

/// A place in the ring: a block by its global sequence number, and a byte offset inside it. The writer publishes
/// the end of what it has programmed ([`Ring::position`]), and the saver how far it has saved. Positions order by
/// block first, so a later position compares greater.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Position {
    /// The block's sequence number: +1 for every block opened, across boots.
    pub block_sequence: u32,
    /// Bytes from the block's start.
    pub offset: u32,
}

/// Why a ring call failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// The geometry is larger than the flash.
    Geometry,
    /// A chunk payload longer than [`chunk::MAX_PAYLOAD`]: it can never fit a block behind the block header.
    TooLarge,
    /// No erased block is left to open. Nothing was programmed: the caller keeps its records in PSRAM and appends
    /// them again once the eraser has freed a block (spec "The ring_writer", step 7).
    WouldBlock,
    /// Every block sequence number is spent. The next block would need u32::MAX, which no block is given, so that
    /// no number is ever given twice. Writing gets nowhere near it (2^32 blocks are 16 TiB, centuries at the
    /// flight rate): only a block numbered near u32::MAX that another build or a corrupt hint left behind does.
    /// Nothing was programmed, the records stay in PSRAM, and no block opens until the `log` partition is erased
    /// and the ring positions in the `cfg` NVS are cleared.
    Exhausted,
    /// A flash call failed. A failed chunk program may still have landed in part or whole, so the writer goes on
    /// behind it, and a retry of the same records is a duplicate at worst (spec "Duplicates"). A slot that failed
    /// is never tried again and again: see the module doc.
    Flash(E),
}
