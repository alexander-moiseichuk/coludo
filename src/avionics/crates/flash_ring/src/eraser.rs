//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_eraser's side of the ring (doc/specs/avionics-port.md, "The ring_eraser and newest wins"): the only
//! task that erases the `log` partition, one 4 KiB block per [`Eraser::erase_step`] (13-17 ms; a 64 KiB erase is
//! never issued). Its thread (core 0, priority 3) calls the step again and again, paced by the saver's progress
//! and the erased reserve, on the pad and in flight alike.
//!
//! Each step erases the first of:
//! 1. a **foreign** slot: it holds nothing recoverable, so no loss. The search runs from the writer's slot on, so
//!    on an unformatted ring the slot the writer needs next is the first one formatted;
//! 2. a **saved** block: one the saver passed at least [`ERASE_DELAY_MS`] ago (the normal rule), or one behind
//!    the previous boot's saver position when the bring-up trusts it ([`Eraser::new`]);
//! 3. under **newest wins**, once no more than [`crate::Geometry::reserve_blocks`] slots are erased or saved: the
//!    **oldest** block even unsaved, a non-flight one while any is left, flight blocks only after them. Each is a
//!    loss: the step returns it for the log line and counts it in the table. Starting at the reserve rather than
//!    below it keeps one block of margin for the block the writer opens before the next step.
//!
//! It never erases the writer's open block, nor an erased slot (which is where the writer opens its next block).
//! A slot whose erase failed [`ERASE_ATTEMPTS`] times in a row is given up until the next boot and passed over,
//! so one bad sector never stops the eraser.

use alloc::boxed::Box;
use alloc::vec;

use crate::header::{BlockHeader, HEADER_BYTES};
use crate::{BlockTable, Error, Flash, Position, Slot};

/// How long a block stays after the saver has passed it, before the normal rule erases it: 100 ms. A saver
/// position must stay published this long before it counts, so a block is erased 100-200 ms after the saver
/// passed it (spec "The ring_eraser and newest wins").
pub const ERASE_DELAY_MS: u32 = 100;

/// How many erases in a row may fail on one slot before the eraser gives the slot up for the rest of the boot: 3.
///
/// The choice of a step is deterministic, so without a limit a sector whose erase keeps failing (worn out, or a
/// driver error that repeats) would be chosen on every step, nothing else would ever be erased, the writer would
/// block for good and the producer rings would overflow: all telemetry and logs dropped. A slot given up is passed
/// over as if it were not there (neither free nor a candidate), so newest wins takes the next-oldest block
/// instead, still flight blocks last. It counts in [`crate::Figures::bad`]; the next boot tries it again.
pub const ERASE_ATTEMPTS: u8 = 3;

/// An unsaved block newest wins erased: the log line `<uptime> ring :: block <sequence> (boot <id>, session <s>,
/// flight <0|1>) erased unsaved` is made from it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Loss {
    /// The slot it was in.
    pub slot: u32,
    /// Its sequence number.
    pub block_sequence: u32,
    /// Whether it was a flight block.
    pub flight: bool,
    /// Its header, read just before the erase, for the boot id and session. None when the header could not be
    /// read, no longer decodes, or names another block (an earlier erase of it failed part way).
    pub header: Option<BlockHeader>,
}

/// What one [`Eraser::erase_step`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Erasure {
    /// Erased a foreign slot: no ring data, no loss.
    Foreign {
        /// The slot.
        slot: u32,
    },
    /// Erased a block the saver had passed (the normal rule).
    Saved {
        /// The slot.
        slot: u32,
        /// The block's sequence number.
        block_sequence: u32,
    },
    /// Newest wins erased an unsaved block: log it.
    Lost(Loss),
    /// Nothing to erase: no foreign slot, no saved block, and the reserve is held.
    Idle,
}

/// What a step chose to erase.
#[derive(Clone, Copy, Debug)]
enum Choice {
    /// A foreign slot.
    Foreign(u32),
    /// A saved block: slot, sequence.
    Saved(u32, u32),
    /// An unsaved block under newest wins: slot, sequence, flight.
    Unsaved(u32, u32, bool),
}

/// The oldest of some blocks, as (sequence, slot, flight).
type Oldest = Option<(u32, u32, bool)>;

/// Keeps `candidate` in `oldest` when it is older.
fn keep_oldest(oldest: &mut Oldest, candidate: (u32, u32, bool)) {
    if oldest.is_none_or(|(block_sequence, _, _)| candidate.0 < block_sequence) {
        *oldest = Some(candidate);
    }
}

/// The ring as its one eraser sees it.
pub struct Eraser<'table, F: Flash> {
    /// The partition, through the eraser's own handle.
    flash: F,
    /// The slots, shared with the writer and the saver.
    table: &'table BlockTable,
    /// The newest saver position seen that has not yet stood for [`ERASE_DELAY_MS`], and when it was first seen
    /// (uptime ms, wrapping).
    pending: Option<(Position, u32)>,
    /// The newest saver position that has stood for [`ERASE_DELAY_MS`]: blocks before its block are erasable.
    settled: Position,
    /// A block header, read before a newest-wins erase for the loss line.
    header: [u8; HEADER_BYTES],
    /// Each slot's failed erases in a row this boot; at [`ERASE_ATTEMPTS`] the slot is given up. One byte a slot
    /// (6 KiB on the board), the eraser's alone, so it needs no atomics.
    failures: Box<[u8]>,
}

impl<'table, F: Flash> Eraser<'table, F> {
    /// The eraser of this boot, after the bring-up scan.
    ///
    /// Args:
    /// * `flash` - the eraser's handle on the `log` partition.
    /// * `table` - the scan's [`BlockTable`], shared with the writer and the saver.
    /// * `saved` - the saver position the previous boot left in the `cfg` NVS, when the bring-up trusts it: the
    ///   mounted card is the one the previous boot's session-start record names, or no card is mounted (spec
    ///   "Bring-up", step 5: the saved blocks first). Every block before its block is on that card and fsynced,
    ///   so those blocks are erased at once as saved, with no loss line, even with no saver this boot. None with
    ///   another card, which must get everything still in flash, or with no hint. A hint that names no block in
    ///   flash (as [`BlockTable::saver_start`] judges it) is not trusted either: a corrupt one would otherwise
    ///   pass unsaved blocks off as saved.
    ///
    /// Returns the eraser, or `Geometry` when the table's geometry is larger than the flash.
    pub fn new(
        flash: F,
        table: &'table BlockTable,
        saved: Option<Position>,
    ) -> Result<Eraser<'table, F>, Error<F::Error>> {
        if table.geometry().bytes() > flash.size() {
            return Err(Error::Geometry);
        }
        let settled = saved
            .filter(|&hint| table.saver_start(Some(hint)) == hint)
            .unwrap_or_default();
        Ok(Eraser {
            flash,
            table,
            pending: None,
            settled,
            header: [0; HEADER_BYTES],
            failures: vec![0; table.geometry().blocks() as usize].into_boxed_slice(),
        })
    }

    /// One step: erases at most one block, by the rules in the module doc.
    ///
    /// Args:
    /// * `saver` - the saver's published position (`saver_position`): every block before its block is on the SD
    ///   and fsynced. None while there is no saver (no card, or M3, before the saver exists), which leaves only
    ///   foreign slots, the blocks behind a trusted hint ([`Eraser::new`]) and newest wins.
    /// * `now_ms` - the uptime in milliseconds, wrapping (a u32, as the tees keep it): it times the delay after
    ///   the saver.
    ///
    /// Returns what it erased, or `Flash` when the erase failed: log it. The table is unchanged then, and a loss
    /// is counted once, when its erase succeeds. The next step tries the same slot again, up to
    /// [`ERASE_ATTEMPTS`] failures in a row, then passes it over for the rest of the boot.
    pub fn erase_step(&mut self, saver: Option<Position>, now_ms: u32) -> Result<Erasure, Error<F::Error>> {
        self.settle(saver, now_ms);
        let Some(choice) = self.choose(saver) else {
            return Ok(Erasure::Idle);
        };
        match choice {
            Choice::Foreign(slot) => {
                self.erase(slot)?;
                Ok(Erasure::Foreign { slot })
            }
            Choice::Saved(slot, block_sequence) => {
                self.erase(slot)?;
                Ok(Erasure::Saved { slot, block_sequence })
            }
            Choice::Unsaved(slot, block_sequence, flight) => {
                // the header only names the loss: a failed read must not keep the block from going
                let offset = self.table.geometry().slot_offset(slot);
                let header = match self.flash.read(offset, &mut self.header) {
                    Ok(()) => BlockHeader::decode(&self.header)
                        .ok()
                        .filter(|header| header.block_sequence == block_sequence),
                    Err(_) => None,
                };
                self.erase(slot)?;
                self.table.count_loss();
                Ok(Erasure::Lost(Loss {
                    slot,
                    block_sequence,
                    flight,
                    header,
                }))
            }
        }
    }

    /// Moves the delay along: the pending saver position settles once it has stood for [`ERASE_DELAY_MS`], and
    /// only then does a newer one start its wait. A saver that moves on every batch therefore has its blocks
    /// erased 100-200 ms (plus a step) after it passed them, never sooner.
    fn settle(&mut self, saver: Option<Position>, now_ms: u32) {
        if let Some((position, seen_ms)) = self.pending
            && now_ms.wrapping_sub(seen_ms) >= ERASE_DELAY_MS
        {
            self.settled = self.settled.max(position);
            self.pending = None;
        }
        if self.pending.is_none()
            && let Some(saver) = saver
            && saver > self.settled
        {
            self.pending = Some((saver, now_ms));
        }
    }

    /// What to erase, by one walk over the table from the slot after the writer's: the first foreign slot, else
    /// the oldest settled-saved block, else, when no more than the reserve are erased or saved, the oldest unsaved
    /// non-flight block, else the oldest unsaved flight block. Slots given up are passed over. None when nothing
    /// is due.
    fn choose(&self, saver: Option<Position>) -> Option<Choice> {
        let geometry = self.table.geometry();
        let blocks = geometry.blocks();
        let cursor = self.table.cursor();
        let saved_before = saver
            .map_or(0, |saver| saver.block_sequence)
            .max(self.settled.block_sequence);
        let mut free = 0;
        let mut saved: Oldest = None;
        let mut oldest_non_flight: Oldest = None;
        let mut oldest_flight: Oldest = None;
        for step in 1..=blocks {
            let slot = (cursor + step) % blocks;
            if self.failures[slot as usize] >= ERASE_ATTEMPTS {
                continue;
            }
            match self.table.slot(slot) {
                Slot::Foreign => return Some(Choice::Foreign(slot)),
                Slot::Erased => free += 1,
                Slot::Open { .. } => {}
                Slot::Valid { block_sequence, flight } if block_sequence < saved_before => {
                    free += 1;
                    if block_sequence < self.settled.block_sequence {
                        keep_oldest(&mut saved, (block_sequence, slot, flight));
                    }
                }
                Slot::Valid { block_sequence, flight }
                | Slot::OtherVersion {
                    block_sequence, flight, ..
                } => {
                    let oldest = if flight {
                        &mut oldest_flight
                    } else {
                        &mut oldest_non_flight
                    };
                    keep_oldest(oldest, (block_sequence, slot, flight));
                }
            }
        }
        if let Some((block_sequence, slot, _)) = saved {
            return Some(Choice::Saved(slot, block_sequence));
        }
        if free > geometry.reserve_blocks() {
            return None;
        }
        let (block_sequence, slot, flight) = oldest_non_flight.or(oldest_flight)?;
        Some(Choice::Unsaved(slot, block_sequence, flight))
    }

    /// Erases `slot` and records it erased, clearing its failures. On failure the table is left as it was, and
    /// the failure counted: the [`ERASE_ATTEMPTS`]th in a row gives the slot up.
    fn erase(&mut self, slot: u32) -> Result<(), Error<F::Error>> {
        let offset = self.table.geometry().slot_offset(slot);
        let erased = self.flash.erase_sector(offset);
        let failures = &mut self.failures[slot as usize];
        match erased {
            Ok(()) => {
                *failures = 0;
                self.table.set(slot, Slot::Erased);
                Ok(())
            }
            Err(error) => {
                *failures += 1;
                if *failures == ERASE_ATTEMPTS {
                    self.table.count_bad();
                }
                Err(Error::Flash(error))
            }
        }
    }
}
