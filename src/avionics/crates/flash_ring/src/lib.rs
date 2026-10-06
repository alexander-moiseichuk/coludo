//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flash ring (doc/specs/avionics-port.md, "Storage design"): the 24 MiB `log` partition as a strict FIFO
//! of 64 KiB blocks. The flash writer appends frames to it, and a GC erases a block once the SD mover has
//! confirmed it. It replaces plan B's UART to the Luckfox, so it ports no src/glider module, and the plan is the
//! spec.
//!
//! * [`frame`]: the frame codec, which is the 40-byte header with its two zlib CRC-32s and the CONFIRM payload,
//!   plus the walk over a block's frames. It is `recorder_wire::frame`, re-exported: the spec files it beside the
//!   line wrapper, whose CRC-32 it shares and whose lines every ring reader verifies next.
//! * [`scan`]: the boot scan, which reads each block's offset-0 header and classifies it.
//! * [`Ring`]: the writer side. Append with a bounded spill queue, the reverse-order sector GC, the published
//!   position of what is programmed, and the budget figures.
//! * [`MemoryFlash`]: an in-memory NOR flash that cuts power on demand, for the host tests and host tools.
//!
//! Everything runs over the [`Flash`] trait, so the firmware plugs in its `esp_partition_*` calls and the host
//! tests run the same code. The crate is `no_std` + `alloc`, and holds no atomics. The firmware publishes
//! [`Ring::programmed`] itself, as two AtomicU32 under a seqlock, since the P4 has no 64-bit atomics.

#![no_std]

extern crate alloc;

mod geometry;
mod memory;
mod ring;
pub mod scan;
mod spill;

pub use geometry::Geometry;
pub use memory::{MemoryError, MemoryFlash};
pub use recorder_wire::frame;
pub use ring::{CONFIRM_SEARCH_BLOCKS, Erasure, Error, Figures, Options, Ring};

/// The erase unit: one NOR sector. GC erases one per step, because a sector erase (13-17 ms) fits between
/// two frame programs and a 64 KiB block erase (95-107 ms) does not.
pub const SECTOR_BYTES: u32 = 4096;

/// A NOR flash region (the ring's partition); offsets are bytes from the region's start.
///
/// The ring relies on NOR semantics, and [`MemoryFlash`] enforces them. A program only clears bits (1 -> 0), so
/// it only ever lands on erased bytes. Only an erase sets bits back, one whole 4 KiB sector at a time.
pub trait Flash {
    /// What a failed call reports (an `EspError` on the board, a [`MemoryError`] on the host).
    type Error: core::fmt::Debug;

    /// Reads `buffer.len()` bytes at `offset`.
    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error>;

    /// Programs `data` at `offset`, 1 -> 0 bits only and in address order. The ring makes one call per frame,
    /// header first, so a cut can tear only the frame's tail.
    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Erases the sector at `offset` (a multiple of [`SECTOR_BYTES`]) back to 0xFF.
    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error>;

    /// The region's size in bytes.
    fn size(&self) -> u32;
}

/// The writer publishes one as [`Ring::programmed`], and the mover confirms one ([`Ring::confirm_moved`]). It is
/// the frame format's own type, so a CONFIRM frame carries exactly what the ring publishes.
pub use recorder_wire::frame::Position;
