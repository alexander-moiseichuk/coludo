//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The writer side of the ring (doc/specs/avionics-port.md: "The flash writer", "Ring geometry and block life
//! cycle", "GC and erase policy", "Spill"). The flash writer thread owns the [`Ring`] and makes every call. The
//! mover reads the flash through its own handle, never beyond the published [`Ring::programmed`] position.
//!
//! A block's life:
//! 1. **Erased.** The writer opens it only after one read of the whole block proves it erased.
//! 2. **Open.** A BOOT frame at offset 0, then frames at 8-aligned offsets. Each frame is one program call,
//!    header first, and never crosses the block's end; the tail stays 0xFF. A frame that does not fit closes
//!    the block for good, so a new boot never appends to an old block, nor the writer to a closed one. The
//!    writer cuts its frames at [`Ring::room_behind_spill`], so that only happens to the last few bytes.
//! 3. **Closed.** Valid ring data. It stays until the mover confirms it, whatever the writer needs.
//! 4. **Moved.** Its sequence number lies below the confirmed cursor's. GC erases it one 4 KiB sector per step,
//!    in reverse order (the header sector last), and it is erased again.
//!
//! When the next block is not ready, frames wait in a bounded spill queue. They are programmed in arrival order
//! once GC frees the block, so nothing is reordered or skipped. A frame is stamped (block, frame number, header)
//! when it is programmed, not when it is cut. Numbering at program time keeps `frame_sequence` rising in address
//! order through the BOOT frames of the blocks a long spill fills, so a gap still means a lost frame. A full
//! spill queue is back-pressure ([`Error::SpillFull`]), never a drop, so a frame once cut is always stored.

use alloc::vec;
use alloc::vec::Vec;

use crate::frame::{self, Confirm, Flags, HEADER_BYTES, Header, Kind, frame_bytes};
use crate::scan::{Slot, scan};
use crate::spill::Spill;
use crate::{Flash, Geometry, Position, SECTOR_BYTES};

/// How many of the newest blocks [`Ring::recover_confirm`] walks back for a CONFIRM frame. That bounds the
/// boot's reads (16 x 64 KiB, ~100 ms). Past it, the cursor stays where it was: conservative, because GC then
/// erases nothing valid until the mover confirms again.
pub const CONFIRM_SEARCH_BLOCKS: u32 = 16;

/// The value of an erased byte.
const ERASED: u8 = 0xFF;

/// The ring's tuning, apart from its geometry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Options {
    /// The most bytes the spill queue holds (14 bytes of head per frame plus its payload). Past it,
    /// [`Ring::append`] returns [`Error::SpillFull`], which is back-pressure on the writer.
    pub spill_capacity: usize,
    /// The erased blocks GC keeps ahead of the writer even while erases are paused for flight.
    pub reserve_blocks: u32,
}

impl Options {
    /// The board's: a 2 MiB PSRAM spill queue, and 8 blocks (512 KiB, ~15 s at the flight rate) erased ahead
    /// before the flight pause holds.
    pub const BOARD: Options = Options {
        spill_capacity: 2 << 20,
        reserve_blocks: 8,
    };
}

/// Why a ring call failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error<E> {
    /// The geometry is larger than the flash.
    Geometry,
    /// A payload (or, at mount, the BOOT payload) that can never fit a block behind its BOOT frame.
    TooLarge,
    /// [`Ring::append`] was asked for a BOOT frame. Those are the ring's own, one at each block's start.
    BootKind,
    /// The spill queue has no room, so the frame is neither stored nor numbered. It is back-pressure, not a drop
    /// (spec "Spill", invariant 2): the writer keeps the frame it cut and stops draining the producer rings. It
    /// keeps calling [`Ring::erase_step`] and [`Ring::drain`], then appends the same frame again. Records are
    /// dropped only where the spec counts them: at `Telemetry::push` and `Recorder::log`, once their rings fill.
    SpillFull,
    /// A flash call failed. A frame whose program failed is not stored, and its block is closed, so a retry
    /// lands in a new block.
    Flash(E),
}

/// What one [`Ring::erase_step`] did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Erasure {
    /// Erased sector `sector` (counting down to 0, the header sector) of slot `slot`.
    Sector {
        /// The slot being erased.
        slot: u32,
        /// The sector index inside the block.
        sector: u32,
    },
    /// Nothing, because erases are paused for flight and enough blocks are erased ahead.
    Paused,
    /// Nothing to erase: every slot ahead is erased, or the oldest holds data the mover has not confirmed.
    Idle,
}

/// The ring's budget, as the writer reports it (`ring:{...}`, the degraded tokens, the arm gate).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Figures {
    /// Erased blocks lying ahead of the writer in FIFO order, before the first that is not erased.
    /// `ring_erased_s` = this x block bytes / the measured write rate.
    pub erased_ahead: u32,
    /// Valid blocks wholly before the confirmed cursor and not yet erased. `ring_reclaim_s` comes from it.
    pub moved_not_erased: u32,
    /// Slots holding no ring data that still need an erase: the format backlog. Any at all is
    /// `ring-unformatted`. It is counted from the slots rather than latched at the boot scan, so a format that
    /// a reboot interrupted still shows. A torn BOOT or a refused dirty block shows too, as erase work.
    pub foreign: u32,
    /// Slots holding a whole header of another layout version ([`Slot::OtherVersion`]). GC never erases them,
    /// so any at all is a degraded token that asks for the operator's discard.
    pub other_version: u32,
    /// Valid blocks the mover has not wholly confirmed, the open one included.
    pub unconfirmed: u32,
    /// Frames waiting in the spill queue. Any at all is `ring-full`.
    pub spill_frames: u32,
    /// The bytes they take.
    pub spill_bytes: u32,
}

/// The block GC is erasing, sector by sector.
#[derive(Clone, Copy, Debug)]
struct Erasing {
    /// Its slot.
    slot: u32,
    /// Sectors still to erase. The next one is `remaining - 1`.
    remaining: u32,
}

/// The flash ring as its one writer sees it.
pub struct Ring<F: Flash> {
    /// The partition.
    flash: F,
    /// Its shape.
    geometry: Geometry,
    /// [`Options::reserve_blocks`].
    reserve_blocks: u32,
    /// This boot's id, in every header.
    boot_id: u32,
    /// This boot's BOOT payload, at the start of every block it opens.
    boot_payload: Vec<u8>,
    /// The largest payload that fits a block behind the BOOT frame.
    max_payload: usize,
    /// What each slot holds, kept current as the writer opens and GC erases.
    slots: Vec<Slot>,
    /// The open block and where its next frame goes. None before the first frame, and after a block closes.
    open: Option<Position>,
    /// The block the writer opens next.
    next_block: u32,
    /// The number the next programmed frame gets.
    next_frame: u32,
    /// How far the mover may read: the 8-aligned end of the last programmed frame, or at mount the start of
    /// the block the writer opens next. See [`Ring::programmed`].
    programmed: Option<Position>,
    /// The mover's confirmed cursor. Blocks wholly before it may be erased.
    confirmed: Position,
    /// Frames waiting for an erased block.
    spill: Spill,
    /// The block GC is part way through.
    erasing: Option<Erasing>,
    /// Whether erases are paused for flight.
    paused: bool,
    /// The frame being programmed, as header + payload, so one program call writes it.
    frame: Vec<u8>,
    /// A block-sized buffer for the erased check and the CONFIRM walk (64 KiB, PSRAM on the board).
    scratch: Vec<u8>,
}

impl<F: Flash> Ring<F> {
    /// Mounts the ring: the boot scan, then the writer positioned at the slot after the newest valid block.
    /// A new boot never appends to an old block, so every valid block is closed, and [`Ring::programmed`]
    /// already lets the mover read them all. With no valid block anywhere the ring is unformatted, and the
    /// writer starts at block 0 in slot 0, spilling until GC has erased it. A slot of another layout version is
    /// kept: GC idles at it, as at an unconfirmed block, until the operator discards it.
    ///
    /// Nothing is programmed or erased here. The first [`Ring::append`] opens the first block. Call
    /// [`Ring::recover_confirm`] next to recover the GC cursor.
    ///
    /// Errors: `Geometry` when the geometry is larger than the flash, `TooLarge` when the BOOT frame leaves no
    /// room for a frame in a block, and `Flash` when a header read fails.
    pub fn mount(
        flash: F,
        geometry: Geometry,
        options: Options,
        boot_id: u32,
        boot_payload: &[u8],
    ) -> Result<Ring<F>, Error<F::Error>> {
        if geometry.bytes() > flash.size() {
            return Err(Error::Geometry);
        }
        let block_bytes = geometry.block_bytes() as usize;
        if boot_payload.len() >= block_bytes || frame_bytes(boot_payload.len()) + HEADER_BYTES > block_bytes {
            return Err(Error::TooLarge);
        }
        let slots = scan(&flash, geometry).map_err(Error::Flash)?;
        let newest = slots.iter().filter_map(|slot| slot.block_sequence()).max();
        let next_block = newest.map_or(0, |newest| newest.wrapping_add(1));
        Ok(Ring {
            flash,
            geometry,
            reserve_blocks: options.reserve_blocks,
            boot_id,
            boot_payload: boot_payload.to_vec(),
            max_payload: block_bytes - frame_bytes(boot_payload.len()) - HEADER_BYTES,
            slots,
            open: None,
            next_block,
            next_frame: 0,
            programmed: newest.map(|_| Position {
                block_sequence: next_block,
                offset: 0,
            }),
            confirmed: Position::default(),
            spill: Spill::new(options.spill_capacity),
            erasing: None,
            paused: false,
            frame: Vec::new(),
            scratch: vec![ERASED; block_bytes],
        })
    }

    /// Appends one frame of `kind` (DATA or CONFIRM). Earlier spilled frames go first. The frame is then
    /// programmed with one call at the open block's next 8-aligned offset, or in a newly opened block (BOOT
    /// first) when it does not fit. When no block is ready, the frame is spilled instead.
    ///
    /// Args:
    /// * `kind` - DATA or CONFIRM.
    /// * `payload` - complete wire lines, or a [`Confirm`] payload.
    /// * `durable` - the frame was cut early for a durable record.
    /// * `uptime_us` - the uptime of its first record. A BOOT frame opened for it carries the same uptime.
    ///
    /// Returns Ok once the frame is programmed or queued. `SpillFull` stores and numbers nothing, and the writer
    /// appends the same frame again later (back-pressure, see [`Error::SpillFull`]). `BootKind` and `TooLarge`
    /// refuse the frame. `Flash` means a flash call failed (this frame's program, its block's erased check or
    /// BOOT, or an earlier spilled frame's program): this frame is not stored, and the producer keeps its
    /// records to retry.
    pub fn append(&mut self, kind: Kind, payload: &[u8], durable: bool, uptime_us: u64) -> Result<(), Error<F::Error>> {
        if kind == Kind::Boot {
            return Err(Error::BootKind);
        }
        if payload.len() > self.max_payload {
            return Err(Error::TooLarge);
        }
        self.drain()?;
        if self.spill.is_empty()
            && let Some(at) = self.place(payload.len(), uptime_us)?
        {
            self.frame.clear();
            self.frame.resize(HEADER_BYTES, 0);
            self.frame.extend_from_slice(payload);
            return self.program(
                at,
                kind,
                Flags {
                    spilled: false,
                    durable,
                },
                uptime_us,
            );
        }
        if self.spill.push(kind, durable, uptime_us, payload) {
            Ok(())
        } else {
            Err(Error::SpillFull)
        }
    }

    /// Programs spilled frames, oldest first, while blocks can be opened for them. [`Ring::append`] does this
    /// first. The writer also calls it when idle, so that frames do not sit in RAM after GC freed a block.
    /// A failed program leaves its frame first in the queue, and the retry lands in a new block.
    pub fn drain(&mut self) -> Result<(), Error<F::Error>> {
        while let Some(front) = self.spill.front() {
            let Some(at) = self.place(front.payload_length, front.uptime_us)? else {
                break;
            };
            self.frame.clear();
            self.frame.resize(HEADER_BYTES + front.payload_length, 0);
            self.spill.copy_front_payload(&mut self.frame[HEADER_BYTES..]);
            let flags = Flags {
                spilled: true,
                durable: front.durable,
            };
            self.program(at, front.kind, flags, front.uptime_us)?;
            self.spill.pop_front();
        }
        Ok(())
    }

    /// One GC step: erases one 4 KiB sector of the oldest erasable block ahead of the writer.
    ///
    /// The slots ahead are walked in FIFO order from the one the writer opens next, skipping erased ones. The
    /// first slot that is not erased is the only candidate. It is erased when it holds no ring data (foreign),
    /// or a block before the confirmed cursor's. Any other block is unconfirmed or of another layout version,
    /// and GC idles rather than skip it, since the writer cannot pass it anyway.
    ///
    /// A block's sectors go in reverse order (the last first, sector 0 last), so a cut leaves either a valid
    /// header over a partly erased tail (a moved block, erased again) or an erased header. While erases are
    /// paused for flight, nothing is erased unless fewer than [`Options::reserve_blocks`] are erased ahead.
    /// A failed erase is retried on the same sector next step.
    pub fn erase_step(&mut self) -> Result<Erasure, Error<F::Error>> {
        if self.paused && self.erased_ahead() >= self.reserve_blocks {
            return Ok(Erasure::Paused);
        }
        let erasing = match self.erasing {
            Some(erasing) => erasing,
            None => match self.erasable() {
                Some(slot) => Erasing {
                    slot,
                    remaining: self.geometry.sectors_per_block(),
                },
                None => return Ok(Erasure::Idle),
            },
        };
        let sector = erasing.remaining - 1;
        self.erasing = Some(erasing);
        let offset = self.geometry.slot_offset(erasing.slot) + sector * SECTOR_BYTES;
        self.flash.erase_sector(offset).map_err(Error::Flash)?;
        if sector == 0 {
            self.slots[erasing.slot as usize] = Slot::Erased;
            self.erasing = None;
        } else {
            self.erasing = Some(Erasing {
                slot: erasing.slot,
                remaining: sector,
            });
        }
        Ok(Erasure::Sector {
            slot: erasing.slot,
            sector,
        })
    }

    /// Raises the confirmed cursor to `moved`, and never lowers it. Blocks wholly before `moved` become
    /// erasable.
    ///
    /// Call it with the mover's committed cursor (its journal COMMIT is the authority, already durable on the
    /// card), then append the CONFIRM frame. The cursor takes effect at once, even if that frame has to
    /// spill. Otherwise a full ring would wait for an erase that waits for the CONFIRM.
    pub fn confirm_moved(&mut self, moved: Position) {
        self.confirmed = self.confirmed.max(moved);
    }

    /// Finds the newest CONFIRM frame by walking the newest blocks backwards, at most
    /// [`CONFIRM_SEARCH_BLOCKS`] of them, one block read each. It raises the confirmed cursor to that frame's,
    /// and returns it, since the mover's copy cursor needs its card serial. Returns None when there is none:
    /// the cursor stays put, so GC erases nothing valid until the mover confirms (the conservative "oldest valid
    /// block" rule).
    pub fn recover_confirm(&mut self) -> Result<Option<Confirm>, Error<F::Error>> {
        let Some(newest) = self.next_block.checked_sub(1) else {
            return Ok(None);
        };
        for back in 0..CONFIRM_SEARCH_BLOCKS.min(self.geometry.blocks()) {
            let Some(block_sequence) = newest.checked_sub(back) else {
                break;
            };
            let slot = self.geometry.slot(block_sequence);
            if self.slots[slot as usize].block_sequence() != Some(block_sequence) {
                continue;
            }
            let offset = self.geometry.slot_offset(slot);
            self.flash.read(offset, &mut self.scratch).map_err(Error::Flash)?;
            let found = frame::frames(&self.scratch)
                .take_while(|found| found.header.block_sequence == block_sequence)
                .filter(|found| found.header.kind == Kind::Confirm)
                .filter_map(|found| Confirm::decode(found.payload).ok())
                .last();
            if let Some(confirm) = found {
                self.confirm_moved(confirm.moved);
                return Ok(Some(confirm));
            }
        }
        Ok(None)
    }

    /// Pauses erases for flight (BOOSTING through LANDING), or resumes them. While paused, GC still erases
    /// whenever fewer than [`Options::reserve_blocks`] blocks are erased ahead.
    pub fn pause_erases(&mut self, paused: bool) {
        self.paused = paused;
    }

    /// The published cursor of what is programmed. The mover reads up to here, and the firmware publishes it
    /// through two AtomicU32 under a seqlock.
    ///
    /// Once this boot has programmed a frame, it is the block last written and the 8-aligned end of its last
    /// frame. Before that, it is offset 0 of the block the writer opens next. Positions order by block first,
    /// so every older block, all closed at mount, is readable whole even while the writer only spills (a ring
    /// full of unconfirmed blocks, which the mover must copy before GC can free a slot). It is None only on an
    /// unformatted ring, until its first frame.
    pub fn programmed(&self) -> Option<Position> {
        self.programmed
    }

    /// The largest payload the next [`Ring::append`] programs without closing the open block: the rest of that
    /// block behind one more header. When no block is open, or the open one cannot take a one-byte payload, the
    /// next append opens a new block anyway, and it is [`Ring::max_payload`]. While frames are spilled no block
    /// is open, so it is `max_payload` then too, and the spilled frames go first: [`Ring::room_behind_spill`]
    /// places them, and the writer cuts its frame before it grows past that (spec "would cross the block end"),
    /// so a frame does not close a block with room left in it.
    pub fn room(&self) -> usize {
        let block_bytes = self.geometry.block_bytes() as usize;
        match self.open {
            Some(open) if block_bytes - open.offset as usize > HEADER_BYTES => {
                block_bytes - open.offset as usize - HEADER_BYTES
            }
            _ => self.max_payload,
        }
    }

    /// The largest payload the next [`Ring::append`] programs without closing a block, once the frames waiting in
    /// the spill have gone first: [`Ring::room`] behind them. They are placed as they will be once blocks are
    /// ready, each behind the one before when it fits, else after the BOOT frame of a new block. With nothing
    /// spilled it is [`Ring::room`].
    ///
    /// The writer sizes its frames by this, so a frame cut while the spill holds frames still fills the block
    /// they leave open. A spilled frame whose program then fails goes to a new block, and this moves with it.
    pub fn room_behind_spill(&self) -> usize {
        let block_bytes = self.geometry.block_bytes() as usize;
        let boot = block_bytes - self.max_payload - HEADER_BYTES;
        let mut end = self.open.map(|open| open.offset as usize);
        for length in self.spill.payload_lengths() {
            end = Some(match end {
                Some(offset) if offset + HEADER_BYTES + length <= block_bytes => offset + frame_bytes(length),
                _ => boot + frame_bytes(length),
            });
        }
        match end {
            Some(offset) if block_bytes - offset > HEADER_BYTES => block_bytes - offset - HEADER_BYTES,
            _ => self.max_payload,
        }
    }

    /// The confirmed cursor GC erases behind.
    pub fn confirmed(&self) -> Position {
        self.confirmed
    }

    /// The budget figures, computed by walking the slots (384 on the board).
    pub fn figures(&self) -> Figures {
        let mut figures = Figures {
            erased_ahead: self.erased_ahead(),
            spill_frames: self.spill.frames(),
            spill_bytes: self.spill.bytes() as u32,
            ..Figures::default()
        };
        for slot in &self.slots {
            match *slot {
                Slot::Valid { block_sequence, .. } if block_sequence < self.confirmed.block_sequence => {
                    figures.moved_not_erased += 1;
                }
                Slot::Valid { .. } => figures.unconfirmed += 1,
                Slot::Foreign => figures.foreign += 1,
                Slot::OtherVersion => figures.other_version += 1,
                Slot::Erased => {}
            }
        }
        figures
    }

    /// What each slot holds, as the scan found it and kept current since.
    pub fn slots(&self) -> &[Slot] {
        &self.slots
    }

    /// The ring's shape.
    pub fn geometry(&self) -> Geometry {
        self.geometry
    }

    /// The largest payload [`Ring::append`] takes: a block, less its BOOT frame and one header. The writer
    /// cuts its frames below this.
    pub fn max_payload(&self) -> usize {
        self.max_payload
    }

    /// The flash, for reading it back.
    pub fn flash(&self) -> &F {
        &self.flash
    }

    /// The flash, giving the ring up (a test's power cut, before mounting again).
    pub fn into_flash(self) -> F {
        self.flash
    }

    /// Where a frame of `payload_length` bytes goes: the open block when the frame fits, else a newly opened
    /// block. A full block is closed for good. Returns None when the next block is not ready, and the frame
    /// must spill.
    fn place(&mut self, payload_length: usize, uptime_us: u64) -> Result<Option<Position>, Error<F::Error>> {
        if let Some(open) = self.open {
            if open.offset as usize + HEADER_BYTES + payload_length <= self.geometry.block_bytes() as usize {
                return Ok(Some(open));
            }
            self.open = None;
        }
        self.open_next(uptime_us)
    }

    /// Opens the next block when its slot is erased. One read of the whole block proves it, because a torn
    /// header-sector erase can leave an erased header over dirty bytes. Then its BOOT frame is programmed.
    ///
    /// Returns where the next frame goes, or None when the slot is not erased. A slot that fails the check
    /// becomes foreign, for GC to erase. A failed BOOT program leaves the slot foreign too, and the same block
    /// is opened there again once it is erased, so the FIFO keeps no hole.
    fn open_next(&mut self, uptime_us: u64) -> Result<Option<Position>, Error<F::Error>> {
        let block_sequence = self.next_block;
        let slot = self.geometry.slot(block_sequence);
        if self.slots[slot as usize] != Slot::Erased {
            return Ok(None);
        }
        let offset = self.geometry.slot_offset(slot);
        self.flash.read(offset, &mut self.scratch).map_err(Error::Flash)?;
        if self.scratch.iter().any(|&byte| byte != ERASED) {
            self.slots[slot as usize] = Slot::Foreign;
            return Ok(None);
        }
        self.frame.clear();
        self.frame.resize(HEADER_BYTES, 0);
        self.frame.extend_from_slice(&self.boot_payload);
        let start = Position {
            block_sequence,
            offset: 0,
        };
        if let Err(error) = self.program(start, Kind::Boot, Flags::default(), uptime_us) {
            self.slots[slot as usize] = Slot::Foreign;
            return Err(error);
        }
        self.slots[slot as usize] = Slot::Valid {
            block_sequence,
            boot_id: self.boot_id,
        };
        self.next_block = block_sequence.wrapping_add(1);
        Ok(self.open)
    }

    /// Programs the frame waiting in `self.frame` (40 header bytes, then the payload in place) at `at`, with
    /// ONE program call, header first, stamped with the next frame number. On success the block stays open
    /// behind it and the published position moves. On failure the block is closed, because a torn frame must
    /// stay its block's last, and the frame number stays spent: the gap shows the loss.
    fn program(&mut self, at: Position, kind: Kind, flags: Flags, uptime_us: u64) -> Result<(), Error<F::Error>> {
        let payload_length = self.frame.len() - HEADER_BYTES;
        let header = Header {
            kind,
            flags,
            payload_length: payload_length as u32,
            block_sequence: at.block_sequence,
            boot_id: self.boot_id,
            frame_sequence: self.next_frame,
            uptime_us,
        };
        self.next_frame = self.next_frame.wrapping_add(1);
        header.seal(&mut self.frame);
        let offset = self.geometry.slot_offset(self.geometry.slot(at.block_sequence)) + at.offset;
        if let Err(error) = self.flash.program(offset, &self.frame) {
            self.open = None;
            return Err(Error::Flash(error));
        }
        let after = Position {
            block_sequence: at.block_sequence,
            offset: at.offset + frame_bytes(payload_length) as u32,
        };
        self.open = Some(after);
        self.programmed = Some(after);
        Ok(())
    }

    /// The slots ahead of the writer in FIFO order: from the one it opens next, all but the newest block's.
    /// The open block, or the last one this ring wrote, is never a GC candidate.
    fn ahead(&self) -> impl Iterator<Item = u32> + use<F> {
        let blocks = self.geometry.blocks();
        let first = self.geometry.slot(self.next_block);
        (0..blocks - 1).map(move |step| (first + step) % blocks)
    }

    /// How many slots ahead are erased before the first that is not.
    fn erased_ahead(&self) -> u32 {
        self.ahead()
            .take_while(|&slot| self.slots[slot as usize] == Slot::Erased)
            .count() as u32
    }

    /// The block GC should erase next: the one in progress, else the first non-erased slot ahead when it is
    /// foreign or moved. Returns None when that slot holds unconfirmed data or another layout version's block,
    /// or every slot ahead is erased.
    fn erasable(&self) -> Option<u32> {
        if let Some(erasing) = self.erasing {
            return Some(erasing.slot);
        }
        let slot = self.ahead().find(|&slot| self.slots[slot as usize] != Slot::Erased)?;
        match self.slots[slot as usize] {
            Slot::Foreign => Some(slot),
            Slot::Valid { block_sequence, .. } => (block_sequence < self.confirmed.block_sequence).then_some(slot),
            Slot::OtherVersion | Slot::Erased => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec;

    use super::*;
    use crate::MemoryFlash;

    /// A ring of four 4 KiB blocks, every slot foreign: frames spill until GC erases one.
    fn unformatted() -> Ring<MemoryFlash> {
        let geometry = Geometry::new(4, 1).unwrap();
        let options = Options {
            spill_capacity: 64 << 10,
            reserve_blocks: 0,
        };
        let flash = MemoryFlash::from_image(vec![0; geometry.bytes() as usize]);
        Ring::mount(flash, geometry, options, 1, b"boot=1").unwrap()
    }

    /// Spill frames of 1000, 2000 and 1500 bytes, then erase the slots they and one more block need.
    fn spill_three(ring: &mut Ring<MemoryFlash>) {
        for (index, size) in [1000, 2000, 1500].into_iter().enumerate() {
            ring.append(Kind::Data, &vec![b'a'; size], false, index as u64).unwrap();
        }
        assert_eq!(ring.figures().spill_frames, 3);
        for _ in 0..3 {
            assert!(matches!(ring.erase_step(), Ok(Erasure::Sector { .. })));
        }
    }

    #[test]
    fn the_room_behind_the_spill_is_where_the_next_frame_lands() {
        let mut ring = unformatted();
        assert_eq!(
            ring.room_behind_spill(),
            ring.max_payload(),
            "nothing spilled, no block open"
        );
        spill_three(&mut ring);
        assert_eq!(ring.room(), ring.max_payload(), "room() alone does not see the spill");
        // 1000 and 2000 share block 0 behind its BOOT frame (48 bytes); 1500 does not fit, and opens block 1
        let room = 4096 - (frame_bytes(6) + frame_bytes(1500)) - HEADER_BYTES;
        assert_eq!(ring.room_behind_spill(), room);
        ring.append(Kind::Data, &vec![b'b'; room], false, 3).unwrap();
        assert_eq!(ring.figures().spill_frames, 0);
        assert_eq!(
            ring.programmed(),
            Some(Position {
                block_sequence: 1,
                offset: 4096
            }),
            "it fills block 1 to its last byte"
        );
        assert_eq!(
            ring.room_behind_spill(),
            ring.room(),
            "nothing spilled: the open block's rest"
        );
    }

    #[test]
    fn a_frame_past_the_room_behind_the_spill_opens_a_block() {
        let mut ring = unformatted();
        spill_three(&mut ring);
        let room = ring.room_behind_spill();
        ring.append(Kind::Data, &vec![b'b'; room + 1], false, 3).unwrap();
        assert_eq!(
            ring.programmed().unwrap().block_sequence,
            2,
            "block 1 closed with room left"
        );
    }
}
