//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_writer's side of the ring (doc/specs/avionics-port.md, "The ring_writer" and "Ring geometry and block
//! life cycle"). The writer thread owns the [`Ring`] and makes every call; the eraser and the saver work on the
//! same partition through their own handles and the shared [`BlockTable`].
//!
//! A block's life, as the writer sees it:
//! 1. **Erased.** The writer takes the next erased slot after its current one, reads the whole block once to
//!    prove it erased (one 4 KiB read, ~0.4 ms), and programs the block header there: one program call.
//! 2. **Open.** Each [`Ring::append`] programs one chunk at the next 8-aligned offset with one program call. A
//!    chunk that does not fit closes the block. A record never straddles blocks, and a new boot never appends to
//!    a previous boot's block, so a torn chunk left by a cut is always its block's last.
//! 3. **Closed.** Valid ring data, which the saver copies and the eraser erases once the saver has passed it, or
//!    under newest wins.
//!
//! When no slot is erased, an append returns [`Error::WouldBlock`] and programs nothing: the records stay in the
//! producers' PSRAM rings until the eraser frees a block. Nothing is spilled or queued here.
//!
//! The flight flag of a block is fixed when the block opens. Every record from BOOSTING on must lie in a flight
//! block (spec "Ring geometry and block life cycle"), and a reboot in flight brings storage up long before the
//! warm start decides anything (spec "Bring-up order": storage at step 3, the warm restore at step 8). So the
//! writer starts with the flag the crumb gives ([`Ring::new`]) and [`Ring::set_flight`] corrects it later.

use alloc::vec;
use alloc::vec::Vec;

use crate::chunk::{self, CHUNK_HEADER_BYTES, MAX_PAYLOAD, chunk_bytes};
use crate::header::{BlockHeader, HEADER_BYTES, Session, VERSION};
use crate::{BLOCK_BYTES, BlockTable, Error, Flash, Position, Slot};

/// The value of an erased byte.
const ERASED: u8 = 0xFF;

/// The block the writer is filling.
#[derive(Clone, Copy, Debug)]
struct Open {
    /// Its slot.
    slot: u32,
    /// Its sequence number.
    block_sequence: u32,
    /// Where its next chunk goes: 8-aligned, behind every chunk programmed or tried so far.
    offset: u32,
    /// The flight flag its header carries.
    flight: bool,
}

/// The flash ring as its one writer sees it.
pub struct Ring<'table, F: Flash> {
    /// The partition, through the writer's own handle.
    flash: F,
    /// The slots, shared with the eraser and the saver.
    table: &'table BlockTable,
    /// This boot's id, in every block header it programs.
    boot_id: u32,
    /// This boot's session label, in every block header it programs.
    session: Session,
    /// Whether the stage is in flight (BOOSTING..DONE), for the blocks it opens from now on.
    flight: bool,
    /// The open block. None before the first chunk, and after a block closed.
    open: Option<Open>,
    /// The sequence the next opened block gets. u32::MAX is never given: it means every number is spent.
    next_sequence: u32,
    /// What the saver may read: the 8-aligned end of the last chunk programmed (see [`Ring::position`]).
    position: Position,
    /// The chunk being programmed, header then payload, so one program call writes it. Allocated once, at its
    /// largest, so an append never allocates.
    chunk: Vec<u8>,
    /// One block, for the check that a slot is wholly erased before the writer opens it.
    scratch: Vec<u8>,
}

impl<'table, F: Flash> Ring<'table, F> {
    /// The writer of this boot, after the bring-up scan. Nothing is read or programmed here; the first
    /// [`Ring::append`] opens the first block.
    ///
    /// Args:
    /// * `flash` - the writer's handle on the `log` partition.
    /// * `table` - the scan's [`BlockTable`] (on the board leaked to `'static`, as the other two tasks share it).
    /// * `next_sequence` - [`crate::Scan::next_sequence`].
    /// * `boot_id` - this boot's id (NVS `coludo/boot`; 0 when uncounted).
    /// * `session` - this boot's session label.
    /// * `flight` - whether the first block opens as a flight block. The bring-up passes true when the crumb it
    ///   read at step 1 names BOOSTING..LANDING: a reboot in flight, which the warm start will most likely resume
    ///   (spec "Warm-start, boot rules and reset classes"). The reset cause, the provenance line, the device setup
    ///   and the warm-start decision then lie in a flight block, which newest wins gives up last, with the flight
    ///   they belong to: the evidence of the reboot is never the first flight-period data to go. Should the gate
    ///   refuse the warm start, `set_flight(false)` closes that one block, and a pad block kept a little longer is
    ///   all it costs. False for every other boot (no crumb, or one at DONE: a new session in SETTING).
    ///
    /// Returns the writer, or `Geometry` when the table's geometry is larger than the flash.
    pub fn new(
        flash: F,
        table: &'table BlockTable,
        next_sequence: u32,
        boot_id: u32,
        session: Session,
        flight: bool,
    ) -> Result<Ring<'table, F>, Error<F::Error>> {
        if table.geometry().bytes() > flash.size() {
            return Err(Error::Geometry);
        }
        Ok(Ring {
            flash,
            table,
            boot_id,
            session,
            flight,
            open: None,
            next_sequence,
            position: Position {
                block_sequence: next_sequence,
                offset: 0,
            },
            chunk: Vec::with_capacity(CHUNK_HEADER_BYTES + MAX_PAYLOAD),
            scratch: vec![ERASED; BLOCK_BYTES as usize],
        })
    }

    /// Programs one chunk of `payload` (whole records, verbatim) holding `records` records, with one program call
    /// into the open block, or into a newly opened one when it does not fit there.
    ///
    /// The chunk goes at the open block's next 8-aligned offset when it fits behind it. Otherwise the open block
    /// is closed, and the next erased slot after it is checked (one 4 KiB read), given the next sequence and
    /// this boot's header (one program call), and the chunk goes behind the header.
    ///
    /// Returns where the chunk starts. Errors:
    /// * `TooLarge` - longer than [`MAX_PAYLOAD`]: no block takes it. Nothing changes.
    /// * `WouldBlock` - no erased slot to open. Nothing is programmed: keep the records and append them again once
    ///   the eraser has freed a block.
    /// * `Exhausted` - no block sequence number is left (see [`Error::Exhausted`]). Nothing is programmed.
    /// * `Flash` - the header program or the chunk program failed, or no erased slot could be read. The chunk is
    ///   not counted as programmed, so keep its records and append them again. A failed chunk program may have
    ///   landed in part or whole, and NOR cannot program over it, so the retry goes at the next offset: the
    ///   saver's walk steps over the debris, and a chunk that had landed whole is a duplicate (spec "Duplicates").
    ///   A failed header program spends its sequence and leaves the slot foreign, for the eraser.
    pub fn append(&mut self, payload: &[u8], records: u16) -> Result<Position, Error<F::Error>> {
        if payload.len() > MAX_PAYLOAD {
            return Err(Error::TooLarge);
        }
        let open = match self.open {
            Some(open) if open.offset as usize + CHUNK_HEADER_BYTES + payload.len() <= BLOCK_BYTES as usize => open,
            _ => {
                self.close();
                self.open_next()?
            }
        };
        chunk::encode(&mut self.chunk, payload, records).map_err(|_| Error::TooLarge)?;
        let end = open.offset + chunk_bytes(payload.len()) as u32;
        // the next chunk goes behind this one even when its program fails: debris may be there
        self.open = Some(Open { offset: end, ..open });
        let at = self.table.geometry().slot_offset(open.slot) + open.offset;
        self.flash.program(at, &self.chunk).map_err(Error::Flash)?;
        self.position = Position {
            block_sequence: open.block_sequence,
            offset: end,
        };
        Ok(Position {
            block_sequence: open.block_sequence,
            offset: open.offset,
        })
    }

    /// The largest payload the next [`Ring::append`] programs into the open block: the rest of the block behind
    /// one more chunk header. When no block is open, or the open one cannot take another chunk header, the next
    /// append opens a new block anyway, and it is [`MAX_PAYLOAD`]. The writer copies records into its chunk while
    /// they fit this, so a block fills before it closes.
    pub fn room(&self) -> usize {
        match self.open {
            Some(open) if open.offset as usize + CHUNK_HEADER_BYTES <= BLOCK_BYTES as usize => {
                BLOCK_BYTES as usize - open.offset as usize - CHUNK_HEADER_BYTES
            }
            _ => MAX_PAYLOAD,
        }
    }

    /// Sets whether the stage is in flight (the caller sets it from BOOSTING to DONE). A change closes the open
    /// block, so every record appended from now on lies in a block whose header carries the new flag: newest wins
    /// gives up flight blocks last.
    ///
    /// After a reboot in flight the writer started as a flight writer ([`Ring::new`]); the warm start either
    /// resumes the flight, and the call changes nothing, or is refused, and `set_flight(false)` closes the one
    /// block written as flight before the decision.
    pub fn set_flight(&mut self, flight: bool) {
        if flight != self.flight {
            self.flight = flight;
            self.close();
        }
    }

    /// Whether the blocks opened from now on carry the flight flag.
    pub fn flight(&self) -> bool {
        self.flight
    }

    /// The writer position to publish (`writer_position`, two AtomicU32 under the firmware's seqlock): the saver
    /// reads up to here and never further.
    ///
    /// Once this boot has programmed a chunk, it is that chunk's block and the 8-aligned end of the chunk. Before
    /// that, it is offset 0 of the block this boot opens first, so every older block, all closed at bring-up, is
    /// readable whole. Positions order by block first.
    pub fn position(&self) -> Position {
        self.position
    }

    /// The block table this writer shares with the eraser and the saver. The recorder's writer reads its budget
    /// figures from it for the status line ([`BlockTable::figures`]), so the figures always describe the ring it
    /// writes and never a table passed in beside it.
    pub fn table(&self) -> &'table BlockTable {
        self.table
    }

    /// Closes the open block, if any: its slot becomes a valid block, the eraser's to erase once saved.
    fn close(&mut self) {
        if let Some(open) = self.open.take() {
            self.table.set(
                open.slot,
                Slot::Valid {
                    block_sequence: open.block_sequence,
                    flight: open.flight,
                },
            );
        }
    }

    /// Opens a block in the first erased slot after the cursor that a full read proves erased, under the next
    /// sequence and this boot's header.
    ///
    /// A slot that fails the check becomes foreign, and the search goes on: one whose bytes are not all erased (an
    /// erased header over dirty bytes, from a cut erase), and one the read fails on. The eraser erases a foreign
    /// slot first, which gives it a fresh start, or gives it up when its erase keeps failing; either way the writer
    /// is never stuck reading one bad slot while others are erased.
    ///
    /// Returns the open block. Errors: `Exhausted` when no sequence number is left; `WouldBlock` when no erased
    /// slot is left; `Flash` when no slot opened and a check read failed (the first such error), or when the
    /// header program failed. A failed header program spends the sequence, since the header may have landed
    /// whole, and leaves the slot foreign: the next open takes the next erased slot.
    fn open_next(&mut self) -> Result<Open, Error<F::Error>> {
        if self.next_sequence == u32::MAX {
            return Err(Error::Exhausted);
        }
        let geometry = self.table.geometry();
        let blocks = geometry.blocks();
        let cursor = self.table.cursor();
        let mut unreadable = None;
        for step in 1..=blocks {
            let slot = (cursor + step) % blocks;
            if self.table.slot(slot) != Slot::Erased {
                continue;
            }
            let offset = geometry.slot_offset(slot);
            match self.flash.read(offset, &mut self.scratch) {
                Ok(()) if self.scratch.iter().all(|&byte| byte == ERASED) => {}
                Ok(()) => {
                    self.table.set(slot, Slot::Foreign);
                    continue;
                }
                Err(error) => {
                    self.table.set(slot, Slot::Foreign);
                    unreadable.get_or_insert(error);
                    continue;
                }
            }
            let block_sequence = self.next_sequence;
            self.next_sequence = block_sequence + 1; // below u32::MAX, checked above
            self.table.set_cursor(slot);
            let header = BlockHeader {
                version: VERSION,
                flight: self.flight,
                block_sequence,
                boot_id: self.boot_id,
                session: self.session,
            };
            if let Err(error) = self.flash.program(offset, &header.encode()) {
                self.table.set(slot, Slot::Foreign);
                return Err(Error::Flash(error));
            }
            self.table.set(
                slot,
                Slot::Open {
                    block_sequence,
                    flight: self.flight,
                },
            );
            return Ok(Open {
                slot,
                block_sequence,
                offset: HEADER_BYTES as u32,
                flight: self.flight,
            });
        }
        Err(unreadable.map_or(Error::WouldBlock, Error::Flash))
    }
}
