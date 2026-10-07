//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_saver (doc/specs/avionics-port.md "The ring_saver", "Duplicates", "SD and flash file layout"), minus its
//! thread: the firmware's saver thread (core 0, priority 4, below the video writer) calls [`Saver::step`] again and
//! again. It copies the flash ring to the SD card as text, in `block_sequence` order, and publishes how far it got
//! (`saver_position`), behind which the eraser erases.
//!
//! **Why here and not in crates/flash_ring**, where the spec's M4 scope places the walk, the decode and the batches:
//! the decode is this crate's [`Decoder`] over this crate's record codec, and flash_ring cannot depend on this crate
//! (this crate depends on it), nor on `std::fs`. So flash_ring gives the table query the walk needs
//! ([`BlockTable::next_block`]) and keeps the ring's own rules; the walk, the decode and the batches live here.
//!
//! Two parts, so that a later uart_saver (M19) can share the decode without touching the SD path:
//! * [`Walk`]: a cursor over the ring and the decoder. [`Walk::peek`] reads the next unread part of the ring, one
//!   4 KiB read of one block (of the writer's open block only up to the writer's published position, never past
//!   it), and [`Walk::decode`] turns it into wire lines. With no card mounted, a walk of its own, with its own
//!   cursor, can feed the uart_saver, and `saver_position` keeps meaning "on the SD and fsynced";
//! * [`Saver`]: a walk, the [`Fs`] of the card, and the batches. Each part is decoded into a staging area first,
//!   then joins the batch: the point where M19's handoff takes each block's lines, before any SD write. Staging
//!   touches no card (every card decision is made when the batch goes to the card), and a part is staged once per
//!   run, a card error included, so M19 would get each line once and an SD stall never repeats a decode.
//!
//! **Routing** (recorderd's, spec "The ring_saver"), under the block header's session directory: a stream line to
//! `<session>/<session>_<file>`, a session.csv line to `<session>/session.csv` and to the global `session.csv`, a
//! log line (and a record the saver would not file) to `<session>/recorder.log`. The bytes are the wire line after
//! its routing, `{OPEN};<payload>;<CLOSE>\n`, as recorderd appended them.
//!
//! **Batches** are at most [`BATCH_BLOCKS`] blocks or [`BATCH_MS`] of collecting, whichever comes first, of one boot
//! (a stricter rule than the spec's one session: two boots under one label are two batches), touching at most
//! [`BATCH_FILES`] files: a block that would pass that starts the next batch (a single block that names more files
//! alone goes as a batch of its own). Then, for each file in turn: open it, append its lines in writes of up to
//! [`WRITE_BYTES`], fsync, close; and only once every file is fsynced is `saver_position` published past the batch.
//! The eraser may then erase those blocks 100-200 ms later. There is no journal: a cut anywhere in a batch leaves
//! the published position before it, and the next boot saves the batch again, duplicates (spec "Duplicates") that
//! the tools drop.
//!
//! **A card error** keeps the batch and the walk: the files already fsynced stay done, and every later step goes on
//! with the file that failed, before anything else. It looks at that file's size again and appends only what did
//! not land, since what a file holds after a failed call is a prefix of what was appended to it ([`Fs`]). So a card
//! that refuses one file for a while (FatFs's FR_LOCKED while an HTTP pull reads it) or every call (a dead card)
//! costs a card call or two a step: nothing is read, decoded or appended again, and nothing new joins meanwhile.
//!
//! **A run** is the saver's life from [`Saver::new`]: one per boot and card (a card put in later gets a saver of
//! its own). Before its first append to a file, and again after the walk has left that file's boot behind, the run
//! looks at the file's end (its last [`TAIL_BYTES`]): a line a cut left torn gets its `\n` first, so the fragment
//! stays a line of its own (damage the tools count) and never merges with a line saved again.
//!
//! **Headers after a reboot.** The tools can never drop a header line as a duplicate (it carries no uptime), and a
//! file's second header reads as two boots appended into it (`flight_telemetry` flags the splice). A run that
//! starts behind what the card holds (a reboot resumes at the `cfg` NVS hint, written about once a second; a lost
//! hint starts at the oldest block) meets headers the card holds already, as the writer's repeats or among the
//! lines it saves again, and must not print them again. Each file is judged on its own, by what it held when the
//! run first looked at it (whole lines: a fragment a cut left alone is none):
//! * under a label of one boot (the boot id `%06u`, or the dated form; never a `recorder.session` label) a file that
//!   held a line holds that boot's header, whatever the hint;
//! * the global session.csv holds every boot: it holds a boot's header when its last row names that boot (a row
//!   carries the boot id and the session). A hint stale across a boot boundary prints the earlier boot's header
//!   there once more, which the tools leave out of session.csv anyway;
//! * under a `recorder.session` label reused across boots, a file holds several boots' lines and no stream line
//!   names its boot. A header of the boot the run began in is dropped from a file that held lines; any later
//!   boot's prints, so the splice still shows. That is exact when the run begins in the label's last boot on the
//!   card, which the firmware secures by writing [`Saved::position`] to the `cfg` NVS hint after every committed
//!   batch, before the next step (about once a second when caught up). One case stays open: a file that held an
//!   earlier boot's lines, while every line of the run's first boot for it lies past the published position (a cut
//!   in that boot's first batch, its first second when caught up, or in the batch where the stream first wrote),
//!   misses that boot's header. The lines are all there; only that file's splice mark is missing.
//!
//! **Never a line left out.** The walk moves past a block only once it is decoded, or once it is gone from the ring:
//! newest wins erased it unsaved, and the eraser logged that loss. A decoded part stays in the batch, or staged
//! beside it, until the card has it. A flash read that fails is tried again at the next step, from the same place.
//!
//! **Memory** stays that of one boot: the decoder keeps the boot it is in, and the run forgets a boot's files once
//! the walk has left it (a boot's blocks are numbered one after another), so a backlog of many short boots, a
//! reset loop on the bench, costs no more than one.

use std::collections::BTreeMap;
use std::ops::Range;
use std::string::String;
use std::vec::Vec;

use flash_ring::header::{BlockHeader, HEADER_BYTES, Session, VERSION};
use flash_ring::{BLOCK_BYTES, BlockTable, Flash, Position};
use recorder_wire::{SESSION_INDEX, is_label};

use crate::decoder::{BlockSummary, DecodedLine, Decoder};
use crate::fs::Fs;
use crate::record::LOG_FILE;
use crate::recorder::Recorder;

/// The most blocks one batch holds (256 KiB of flash): a part of a block read earlier counts with it.
pub const BATCH_BLOCKS: u32 = 64;

/// The most files one batch appends to, the global session.csv included. With per-session directories that is
/// every stream of a boot, its recorder.log and both indexes; the board's `max_files` (40) also serves video and HTTP.
pub const BATCH_FILES: usize = 32;

/// How long a batch collects before it goes to the card, in ms. A saver that keeps up with the writer therefore
/// commits once a second; one behind commits as soon as a batch holds [`BATCH_BLOCKS`].
pub const BATCH_MS: u32 = 1000;

/// The most bytes one write call appends.
pub const WRITE_BYTES: usize = 32 << 10;

/// How much of a file's end the saver reads when it looks at the file: more than any header or session.csv line, so
/// a fragment a cut left alone is told from a file that holds a whole line, and the global session.csv's last row
/// is read whole.
pub const TAIL_BYTES: usize = 1024;

/// A boot as its block headers name it: (session label, boot id).
type Boot = (Session, u32);

/// What lies ahead of a walk's cursor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ahead {
    /// The next part of the ring, read and waiting for [`Walk::decode`]: the block its header names.
    Block(BlockHeader),
    /// Nothing more to read until the writer publishes more: the cursor is at its published position.
    CaughtUp,
}

/// The part a peek read.
#[derive(Clone, Copy, Debug)]
struct Part {
    /// The bytes read, from the block's offset 0: the whole block once it is closed, else up to the writer's position.
    end: usize,
    /// A later block of its boot is in flash, or its boot is this one: a torn chunk in it is no loss.
    boot_goes_on: bool,
    /// Where the cursor goes once the part is decoded.
    after: Position,
}

/// A cursor over the ring, in `block_sequence` order, and the decoder that turns each part it reads into wire lines.
/// See the module notes.
pub struct Walk<'r, 't, F: Flash> {
    /// This boot's recorder: the writer's published position, and this boot's id and session.
    recorder: &'r Recorder,
    /// The `log` partition, through the walk's own handle.
    flash: F,
    /// The slots, shared with the writer and the eraser.
    table: &'t BlockTable,
    /// The decoder: which headers printed, the newest uptime of each boot.
    decoder: Decoder,
    /// The next byte to decode: everything before it was decoded, or is gone.
    cursor: Position,
    /// The part last read. 4 KiB, allocated once (internal RAM on the board: flash reads into PSRAM bounce).
    block: Vec<u8>,
    /// What [`Walk::peek`] read and [`Walk::decode`] decodes.
    part: Option<Part>,
}

impl<'r, 't, F: Flash> Walk<'r, 't, F> {
    /// A walk from `start`, with a decoder that has printed nothing yet.
    ///
    /// Args:
    /// * `recorder` - this boot's recorder.
    /// * `flash` - the walk's handle on the `log` partition.
    /// * `table` - the scan's block table, shared with the writer and the eraser.
    /// * `start` - where to begin: [`BlockTable::saver_start`] of the saver's hint (or of none, for a new card).
    pub fn new(recorder: &'r Recorder, flash: F, table: &'t BlockTable, start: Position) -> Walk<'r, 't, F> {
        Walk {
            recorder,
            flash,
            table,
            decoder: Decoder::new(),
            cursor: start,
            block: vec![0; BLOCK_BYTES as usize],
            part: None,
        }
    }

    /// The next byte to decode.
    pub fn cursor(&self) -> Position {
        self.cursor
    }

    /// Reads the next unread part of the ring: the block the cursor stands in, from its offset 0 up to the block's
    /// end once closed, or up to the writer's published position in the writer's own block, with one flash read.
    /// A block that is gone (newest wins erased it, or it was never opened) is passed over to the next one. Nothing
    /// past the writer's published position is ever read.
    ///
    /// Returns the part's block header, or [`Ahead::CaughtUp`] when nothing is unread (or the writer has not
    /// published yet). Errors: a flash read failed; the cursor stays, and the next peek reads the same part.
    pub fn peek(&mut self) -> Result<Ahead, F::Error> {
        self.part = None;
        let Some(writer) = self.recorder.writer_position() else {
            return Ok(Ahead::CaughtUp);
        };
        loop {
            let Some((slot, block_sequence)) = self.table.next_block(self.cursor.block_sequence) else {
                return Ok(Ahead::CaughtUp);
            };
            if block_sequence > writer.block_sequence {
                return Ok(Ahead::CaughtUp); // the writer's next block: nothing in it is published yet
            }
            if block_sequence != self.cursor.block_sequence {
                // the block the cursor stood in is gone: newest wins erased it unsaved, and the eraser logged it
                self.cursor = Position {
                    block_sequence,
                    offset: 0,
                };
            }
            let closed = block_sequence < writer.block_sequence;
            let end = if closed { BLOCK_BYTES } else { writer.offset };
            if !closed && end <= self.cursor.offset {
                return Ok(Ahead::CaughtUp);
            }
            // a closed block is read even when the cursor stands at its end, as an empty part: a run that starts
            // there begins in that block's boot, not in the next one (see "Headers after a reboot")
            let end = end as usize;
            let at = self.table.geometry().slot_offset(slot);
            self.flash.read(at, &mut self.block[..end])?;
            let header = match BlockHeader::decode(&self.block[..end]) {
                Ok(header) if header.version == VERSION && header.block_sequence == block_sequence => header,
                // the slot changed since the table was read: erased under newest wins, or opened again
                _ if closed => {
                    self.cursor = self.past(block_sequence, writer);
                    continue;
                }
                _ => return Ok(Ahead::CaughtUp), // never so for the writer's open block; look again next step
            };
            let boot_goes_on = self.boot_goes_on(&header, writer)?;
            let after = if closed {
                self.past(block_sequence, writer)
            } else {
                Position {
                    block_sequence,
                    offset: end as u32,
                }
            };
            self.part = Some(Part {
                end,
                boot_goes_on,
                after,
            });
            return Ok(Ahead::Block(header));
        }
    }

    /// Decodes the part the last [`Walk::peek`] read, from the cursor's offset on (each chunk once, across the
    /// parts of the writer's open block), calling `each` with every line; then the cursor moves past the part.
    ///
    /// Returns what the part held; None when no peek read a part since the last decode.
    pub fn decode(&mut self, each: impl FnMut(DecodedLine<'_>)) -> Option<BlockSummary> {
        let part = self.part.take()?;
        let block = &self.block[..part.end];
        // the peek checked the header and its version, so the decoder takes the block
        let summary = self
            .decoder
            .block_from(block, self.cursor.offset, part.boot_goes_on, each)
            .ok();
        self.cursor = part.after;
        summary
    }

    /// Where the cursor goes past closed block `block_sequence`: offset 0 of the next block of this layout, or of
    /// the writer's published block when no other lies before it. Every block between is gone, or another build's.
    fn past(&self, block_sequence: u32, writer: Position) -> Position {
        let next = self
            .table
            .next_block(block_sequence.saturating_add(1))
            .map(|(_, next)| next)
            .filter(|&next| next <= writer.block_sequence);
        Position {
            block_sequence: next.unwrap_or(writer.block_sequence),
            offset: 0,
        }
    }

    /// Whether the boot of the block `header` names goes on past it, so that a torn chunk in it is a failed program
    /// the writer retried later, not a loss (spec "Block and chunk format"): it is this boot, whose writer still
    /// runs, or the next block in sequence order is of the same boot. A boot's blocks are numbered one after
    /// another, and a later boot's after them all, so the next block tells.
    fn boot_goes_on(&self, header: &BlockHeader, writer: Position) -> Result<bool, F::Error> {
        if header.session.as_str() == self.recorder.session() && header.boot_id == self.recorder.boot_id() {
            return Ok(true);
        }
        let Some((slot, next)) = self.table.next_block(header.block_sequence.saturating_add(1)) else {
            return Ok(false);
        };
        if next >= writer.block_sequence {
            return Ok(false); // the writer's block or a newer one: this boot's, never another's, and not read past
        }
        let mut bytes = [0u8; HEADER_BYTES];
        self.flash.read(self.table.geometry().slot_offset(slot), &mut bytes)?;
        Ok(BlockHeader::decode(&bytes)
            .is_ok_and(|later| later.session == header.session && later.boot_id == header.boot_id))
    }
}

/// What one committed batch put on the card.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Saved {
    /// The blocks it held lines of; a part of a block counts as one.
    pub blocks: u32,
    /// The lines it decoded and kept (a session.csv line counts once, though it goes to two files).
    pub lines: u32,
    /// The files it appended to and fsynced.
    pub files: u32,
    /// The bytes it put on the card: its lines, less a header a file held already, plus the `\n` that ended a torn
    /// line. Bytes a card error made the saver append again are not counted twice.
    pub bytes: u32,
    /// The `saver_position` it published: where the next batch starts. The firmware writes it to the `cfg` NVS hint
    /// before the next step (see the module notes, "Headers after a reboot").
    pub position: Position,
}

/// Why a step stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveError<FlashError, CardError> {
    /// A flash read failed. Nothing was lost: the next step reads the same part again.
    Flash(FlashError),
    /// A card call failed. The batch stays as it is: the next step goes on with the file that failed, appending
    /// what did not land, before it reads anything new.
    Card(CardError),
}

/// One file's lines in a batch.
#[derive(Debug)]
struct Pending {
    /// The file, relative to the recordings directory.
    path: String,
    /// Its lines, as they go on the card; final once `base` is known.
    bytes: Vec<u8>,
    /// Where its first header line lies in `bytes`, until the file is looked at: the card may hold it already.
    header: Option<Range<usize>>,
    /// The file's size before the batch's first attempt to append to it; None before that attempt.
    base: Option<u64>,
}

/// Lines decoded and not yet on the card: a batch, or one part staged before it joins one.
#[derive(Debug, Default)]
struct Batch {
    /// The boot of its parts; None while it is empty.
    boot: Option<Boot>,
    /// The blocks it holds lines of.
    blocks: u32,
    /// The last block it took a part of.
    last_block: Option<u32>,
    /// The lines it holds.
    lines: u32,
    /// When it took its first part (uptime ms, wrapping).
    started_ms: u32,
    /// The walk's cursor past its last part: the position to publish once it is on the card.
    end: Position,
    /// Its files, in the order first touched.
    files: Vec<Pending>,
    /// How many of `files`, from the first, are on the card and fsynced.
    done: usize,
}

impl Batch {
    /// Whether it holds no part.
    fn is_empty(&self) -> bool {
        self.boot.is_none()
    }

    /// Whether it goes to the card now: [`BATCH_BLOCKS`] blocks, or [`BATCH_MS`] since its first part.
    fn due(&self, now_ms: u32) -> bool {
        !self.is_empty() && (self.blocks >= BATCH_BLOCKS || now_ms.wrapping_sub(self.started_ms) >= BATCH_MS)
    }

    /// Appends `bytes` to the lines of `path`; `header` is where a header line lies in `bytes`. A file keeps its
    /// first header line's place.
    fn append(&mut self, path: &str, bytes: &[u8], header: Option<Range<usize>>) {
        let index = match self.files.iter().position(|pending| pending.path == path) {
            Some(index) => index,
            None => {
                self.files.push(Pending {
                    path: String::from(path),
                    bytes: Vec::new(),
                    header: None,
                    base: None,
                });
                self.files.len() - 1
            }
        };
        let pending = &mut self.files[index];
        if pending.header.is_none()
            && let Some(header) = header
        {
            let at = pending.bytes.len();
            pending.header = Some(at + header.start..at + header.end);
        }
        pending.bytes.extend_from_slice(bytes);
    }

    /// How many files it would touch with `other`'s too.
    fn files_with(&self, other: &Batch) -> usize {
        let new = other
            .files
            .iter()
            .filter(|pending| !self.files.iter().any(|mine| mine.path == pending.path))
            .count();
        self.files.len() + new
    }

    /// Takes `other`'s part in, emptying it; `now_ms` starts the clock of an empty batch.
    fn absorb(&mut self, other: &mut Batch, now_ms: u32) {
        if other.is_empty() {
            return;
        }
        if self.is_empty() {
            self.started_ms = now_ms;
        }
        if other.last_block != self.last_block {
            self.blocks += 1;
        }
        self.boot = other.boot;
        self.last_block = other.last_block;
        self.lines += other.lines;
        self.end = other.end;
        for pending in other.files.drain(..) {
            self.append(&pending.path, &pending.bytes, pending.header);
        }
        other.clear();
    }

    /// Empties it.
    fn clear(&mut self) {
        *self = Batch {
            files: std::mem::take(&mut self.files),
            ..Batch::default()
        };
        self.files.clear();
    }
}

/// What the saver knows of a file: what its end showed when the run looked at it, and its size since.
#[derive(Clone, Copy, Debug)]
struct Known {
    /// Its size: as the look found it, then after each append the saver fsynced.
    size: u64,
    /// It held a whole line when looked at: a fragment a cut left alone is none.
    held: bool,
    /// Its last line has no `\n`: a cut tore it, so a `\n` goes before the next append.
    torn: bool,
    /// For the global session.csv: the boot its last whole line named, when that line is a row. None for any other
    /// file.
    last_row: Option<Boot>,
}

/// Looks at the end of `path` on the card, reading it into `tail`.
fn look<S: Fs>(card: &mut S, tail: &mut [u8], path: &str) -> Result<Known, S::Error> {
    let size = card.tail(path, tail)?;
    let end = &tail[..size.min(tail.len() as u64) as usize];
    Ok(Known {
        size,
        // a line end in the tail, or more bytes than the tail holds, which no fragment of one line has
        held: end.contains(&b'\n') || size > end.len() as u64,
        torn: end.last().is_some_and(|&byte| byte != b'\n'),
        last_row: (path == SESSION_INDEX).then(|| last_row(end)).flatten(),
    })
}

/// The boot (session, boot id) a session.csv row names, read from the last whole line in `end`; None when that
/// line is a header, does not verify, or `end` holds none. A boot nothing counted has an empty boot cell: id 0.
fn last_row(end: &[u8]) -> Option<Boot> {
    let whole = &end[..end.iter().rposition(|&byte| byte == b'\n')?];
    let line = whole.rsplit(|&byte| byte == b'\n').next()?;
    let payload = recorder_wire::verify(Some(SESSION_INDEX.as_bytes()), line)?;
    let mut cells = payload.split(|&byte| byte == b';');
    let uptime = cells.next()?;
    if uptime.is_empty() || !uptime.iter().all(u8::is_ascii_digit) {
        return None; // the header: its first cell is a column name
    }
    let boot_id = match std::str::from_utf8(cells.next()?).ok()? {
        "" => 0,
        boot => boot.parse().ok()?,
    };
    let session = Session::new(std::str::from_utf8(cells.next()?).ok()?)?;
    Some((session, boot_id))
}

/// Whether the file `known` describes holds the header line of `boot` already (see the module notes, "Headers
/// after a reboot"); `began` is the boot the run began in.
fn holds_header(path: &str, known: &Known, boot: Boot, began: Option<Boot>) -> bool {
    if path == SESSION_INDEX {
        return known.last_row == Some(boot);
    }
    known.held && (!is_label(boot.0.as_str()) || began == Some(boot))
}

/// The ring_saver: a [`Walk`] from the published position, the card, and the batch being collected. See the module
/// notes.
pub struct Saver<'r, 't, F: Flash, S: Fs> {
    /// This boot's recorder: `saver_position` is published there.
    recorder: &'r Recorder,
    /// The cursor and the decoder.
    walk: Walk<'r, 't, F>,
    /// The card's recordings directory.
    card: S,
    /// What is on the card and fsynced: everything before it.
    published: Position,
    /// The batch being collected, or going to the card.
    batch: Batch,
    /// One part decoded, before it joins the batch.
    staged: Batch,
    /// The batch went to the card, and the card failed a call: it goes on before anything else.
    sealed: bool,
    /// The boot this run began in: its header lines stay out of a reused label's files that held lines.
    began: Option<Boot>,
    /// The boot of the part staged last.
    boot: Option<Boot>,
    /// Each file the run has looked at, while the walk is in its boot (the global session.csv for the whole run).
    files: BTreeMap<String, Known>,
    /// The file path being built for a line.
    path: String,
    /// A file's end as it is looked at, [`TAIL_BYTES`].
    tail: Vec<u8>,
}

impl<'r, 't, F: Flash, S: Fs> Saver<'r, 't, F, S> {
    /// The saver of this boot, starting a run at `start`, which it publishes as `saver_position` at once: every
    /// block before it is on this card (a trusted hint), or there is none (a new card starts at the oldest block).
    ///
    /// Args:
    /// * `recorder` - this boot's recorder.
    /// * `flash` - the saver's handle on the `log` partition.
    /// * `table` - the scan's block table, shared with the writer and the eraser.
    /// * `card` - the mounted card's recordings directory.
    /// * `start` - [`BlockTable::saver_start`] of the `cfg` NVS hint when the mounted card is the one the previous
    ///   boot's session-start record names, else of none (spec "Bring-up", step 4).
    pub fn new(
        recorder: &'r Recorder,
        flash: F,
        table: &'t BlockTable,
        card: S,
        start: Position,
    ) -> Saver<'r, 't, F, S> {
        recorder.saver_position.store(start);
        Saver {
            recorder,
            walk: Walk::new(recorder, flash, table, start),
            card,
            published: start,
            batch: Batch::default(),
            staged: Batch::default(),
            sealed: false,
            began: None,
            boot: None,
            files: BTreeMap::new(),
            path: String::new(),
            tail: vec![0; TAIL_BYTES],
        }
    }

    /// One step: collects what the walk reads into the batch, and puts the batch on the card once it is due (see
    /// the module notes), publishing `saver_position` past it. A batch the card failed goes on first, and nothing
    /// is read until it is on the card. Commits one batch at most; call it again at once while it commits full
    /// batches (a backlog), else every 100 ms or so.
    ///
    /// Args:
    /// * `now_ms` - the uptime in milliseconds, wrapping: it times a batch's collecting.
    ///
    /// Returns what the committed batch put on the card, or None when no batch was due. Errors: see [`SaveError`].
    pub fn step(&mut self, now_ms: u32) -> Result<Option<Saved>, SaveError<F::Error, S::Error>> {
        loop {
            if self.sealed || self.batch.due(now_ms) {
                return self.commit(now_ms).map(Some);
            }
            let header = match self.walk.peek().map_err(SaveError::Flash)? {
                Ahead::Block(header) => header,
                Ahead::CaughtUp => return Ok(None),
            };
            let boot = (header.session, header.boot_id);
            if self.batch.boot.is_some_and(|batch| batch != boot) {
                return self.commit(now_ms).map(Some); // a batch holds one boot: the peeked part waits for the next
            }
            self.stage(header);
            if !self.batch.is_empty() && self.batch.files_with(&self.staged) > BATCH_FILES {
                return self.commit(now_ms).map(Some); // the staged part joins the next batch
            }
            self.batch.absorb(&mut self.staged, now_ms);
        }
    }

    /// The published `saver_position`: everything before it is on the card and fsynced.
    pub fn position(&self) -> Position {
        self.published
    }

    /// Decodes the part the walk peeked into the staging area, routing each line to its files. No card is touched.
    /// When the part is of another boot than the last, the run forgets the files it looked at (the global
    /// session.csv aside): the walk never comes back to a boot it left.
    ///
    /// This is where M19's uart_saver takes the part's lines, before any of them goes to the card.
    fn stage(&mut self, header: BlockHeader) {
        let boot = (header.session, header.boot_id);
        self.began.get_or_insert(boot);
        if self.boot.replace(boot) != Some(boot) {
            self.files.retain(|path, _| path == SESSION_INDEX);
        }
        let Saver { walk, staged, path, .. } = self;
        walk.decode(|line| stage_line(staged, path, line));
        staged.boot = Some(boot);
        staged.last_block = Some(header.block_sequence);
        staged.end = walk.cursor();
    }

    /// Puts the batch on the card, file by file (open, append, fsync, close), then publishes its end and lets a part
    /// that waited for it in. A card error leaves the batch sealed: the next step goes on where it stopped.
    fn commit(&mut self, now_ms: u32) -> Result<Saved, SaveError<F::Error, S::Error>> {
        self.sealed = true;
        let saved = self.write_batch().map_err(SaveError::Card)?;
        self.sealed = false;
        self.published = self.batch.end;
        self.recorder.saver_position.store(self.published);
        self.batch.clear();
        self.batch.absorb(&mut self.staged, now_ms);
        Ok(saved)
    }

    /// Appends each file's lines to the card and fsyncs it, from the first file not yet done.
    fn write_batch(&mut self) -> Result<Saved, S::Error> {
        let Saver {
            card,
            batch,
            files,
            began,
            tail,
            ..
        } = self;
        if let Some(boot) = batch.boot {
            if batch.done == 0 && !batch.files.is_empty() {
                card.create_dir_all(boot.0.as_str())?;
            }
            while let Some(pending) = batch.files.get_mut(batch.done) {
                put(card, files, tail, pending, boot, *began)?;
                batch.done += 1;
            }
        }
        Ok(Saved {
            blocks: batch.blocks,
            lines: batch.lines,
            files: batch.files.len() as u32,
            bytes: batch.files.iter().map(|pending| pending.bytes.len() as u32).sum(),
            position: batch.end,
        })
    }
}

/// Appends `pending`, of `boot`, to its file and fsyncs it.
///
/// The first attempt looks at the file unless the run knows it, drops the header line the file holds already (see
/// the module notes), and puts a `\n` first when a cut tore the file's last line: the lines are final from then on.
/// An attempt after a card error asks the file's size again and appends only what did not land.
fn put<S: Fs>(
    card: &mut S,
    files: &mut BTreeMap<String, Known>,
    tail: &mut [u8],
    pending: &mut Pending,
    boot: Boot,
    began: Option<Boot>,
) -> Result<(), S::Error> {
    let (size, from) = match pending.base {
        Some(base) => {
            let size = card.tail(&pending.path, &mut [])?;
            // a prefix of the lines landed; a file shorter than before (no FatFs does that) gets them all again
            let landed = size
                .checked_sub(base)
                .map_or(0, |landed| landed.min(pending.bytes.len() as u64));
            (size, landed as usize)
        }
        None => {
            let known = match files.get(&pending.path) {
                Some(&known) => known,
                None => {
                    let known = look(card, tail, &pending.path)?;
                    files.insert(pending.path.clone(), known);
                    known
                }
            };
            if let Some(header) = pending.header.take()
                && holds_header(&pending.path, &known, boot, began)
            {
                pending.bytes.drain(header);
            }
            if known.torn {
                pending.bytes.insert(0, b'\n');
            }
            pending.base = Some(known.size);
            (known.size, 0)
        }
    };
    let mut file = card.open_append(&pending.path)?;
    for piece in pending.bytes[from..].chunks(WRITE_BYTES) {
        card.write(&mut file, piece)?;
    }
    card.fsync(&mut file)?;
    if let Some(known) = files.get_mut(&pending.path) {
        known.size = size + (pending.bytes.len() - from) as u64;
        known.torn = false;
    }
    Ok(())
}

/// Routes one decoded line into `staged`, its path built in `path`: to its session's file, and a session.csv line
/// to the global index too. A header line's place is kept, for the card to decide on.
fn stage_line(staged: &mut Batch, path: &mut String, line: DecodedLine<'_>) {
    path.clear();
    path.push_str(line.session);
    path.push('/');
    path.push_str(line.routing.unwrap_or(LOG_FILE));
    let body = line.body();
    let header = line.header.then_some(0..body.len());
    staged.append(path, body, header.clone());
    if line.routing == Some(SESSION_INDEX) {
        staged.append(SESSION_INDEX, body, header);
    }
    staged.lines += 1;
}
