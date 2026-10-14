//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The ring_writer (doc/specs/avionics-port.md "The ring_writer"), minus its thread: the firmware's writer thread
//! (core 0, priority 7) calls [`Writer::step`] on its ~10 ms tick ([`TICK_US`]), and at once when a producer's
//! durable record wakes it ([`crate::Wake`]). Each step:
//! 1. programs first a chunk the flash ring refused last time (see below);
//! 2. writes this boot's session.csv `boot` row (its HEADER record, then the ROW) before anything else, and the
//!    `anchor` row likewise a minute later ([`ANCHOR_AFTER_US`]); and about once per MiB written
//!    ([`HEADER_REPEAT_BYTES`]) copies the HEADER record of every stream this boot has started again, so a
//!    session whose first blocks newest wins erased unsaved still gets its headers within the next MiB;
//! 3. drains the producer rings: telemetry records (HEADER, ROW) at once, LOG records only once they are
//!    [`MERGE_HORIZON_US`] old, then merged across the rings by uptime, so recorder.log stays in uptime order;
//! 4. copies whole records, verbatim, into a chunk while they fit the open block, so a block fills before it
//!    closes: a record never straddles chunks or blocks. A record that no longer fits programs the chunk, and the
//!    next chunk goes into the next block;
//! 5. programs each chunk with one call ([`flash_ring::Ring::append`]). Then it copies each record into the CC tee
//!    whose window is open, releases the records from their rings, and publishes `writer_position`;
//! 6. logs the status line every `stats_ms`, a LOG record of its own merged by uptime with the producers': plan
//!    B's counters, plus the flash ring's figures against the ring_saver's published position, both positions and
//!    the write rate measured since the last status line, which `stats` reports too.
//!
//! The writer prints no record: the ring_saver and the `cc` executor do (invariant 4). It makes three short texts
//! of its own, rarely: the status message once per `stats_ms`, and the utc cell of the `boot` and `anchor` rows.
//!
//! **A record leaves its ring only after its chunk is programmed** (invariant 2: "better duplicates in both than
//! lost data"). A ring's tail moves only after a successful program, and never past a LOG record still held for
//! the merge. When the flash ring refuses a chunk, the chunk is kept, whole, and programmed first at the next
//! step; nothing more is read meanwhile, the producer rings fill, and only then does [`crate::Telemetry::push`]
//! raise (and a log line drop). The flash ring refuses a chunk when:
//! * no erased block is left to open (`WouldBlock`): the eraser has not freed one yet;
//! * a program failed (`Flash`): the flash ring has moved past whatever the call left, so the retry goes at the
//!   next offset, and a call that had landed whole leaves the chunk in flash twice: a duplicate, which the tools
//!   drop (spec "Duplicates"). The torn bytes of one that had not are stepped over by the saver's walk;
//! * every block sequence number is spent (`Exhausted`), which only a corrupt hint or another build's blocks
//!   bring about: the records then wait for good, and the writer logs why once.
//!
//! **Flight blocks.** The writer applies the stage the firmware last told the Recorder
//! ([`crate::Recorder::set_flight`]), and the flash ring closes its block on a change, so that every record from
//! BOOSTING to DONE lies in a block with the flight flag (newest wins gives those up last):
//! * a rise (BOOSTING) applies before every chunk, and the flag is read after the records in it, so a record
//!   queued after the rise is in a flight block (one queued just before it may be too, which costs nothing);
//! * a fall (DONE) applies only at the start of a step, once every record queued before it is programmed: a whole
//!   step has drained the rings and merged the logs held for the [`MERGE_HORIZON_US`] since the fall was seen, and
//!   no chunk waits. Applied at once, the last LANDING rows and log lines, queued before DONE but drained after,
//!   would land in a pad block, which newest wins gives up first.

use std::collections::VecDeque;
use std::string::String;
use std::sync::atomic::Ordering;
use std::vec::Vec;

use flash_ring::BLOCK_BYTES;
use flash_ring::chunk::MAX_PAYLOAD;
use flash_ring::{Error, Figures, Flash, Position, Ring};
use recorder_wire::Counters;

use crate::record::{self, Kind, Record, Records};
use crate::recorder::{Class, Recorder};
use crate::registry::StreamId;

/// The writer thread's tick: the firmware calls [`Writer::step`] this often, and at once on a durable record's
/// wake. With an erase and an NVS page erase holding the flash, it bounds what a power cut loses: about 11 ms of
/// rows typical, at most about 45 ms (spec "The ring_writer", the power-cut bound).
pub const TICK_US: u64 = 10_000;

/// A LOG record is programmed once it is this old, so a line a slower thread queues a little later still lands
/// in uptime order. It adds to a log line's power-cut bound (at most about 65 ms).
pub const MERGE_HORIZON_US: u64 = 20_000;

/// The session.csv `anchor` row goes out this long after the writer starts: it lists the boot for a reader that
/// missed its `boot` row, dated if the clock was set by then.
pub const ANCHOR_AFTER_US: u64 = 60_000_000;

/// The flash between two repeats of the started streams' HEADER records: 1 MiB, about a minute at the flight
/// rate, counted by the writer position (block headers and the unused tails of closed blocks included), so a
/// reader that starts anywhere finds every header within the next MiB of flash. The repeats cost one copy each,
/// and the ring_saver prints none of them unless a file starts there or its columns change.
pub const HEADER_REPEAT_BYTES: u32 = 1 << 20;

/// The writer's own LOG records waiting for the merge (status lines, refusals). A flash that refuses chunks for
/// long keeps adding them; past this they drop, counted.
const OWN_LOGS: usize = 64;

/// The descriptor of the writer's own log lines: plan B's, so the readers find the status line.
const OWN_DESCRIPTOR: &str = "Recorder";

/// What one step did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    /// Records programmed, the writer's own included.
    pub records: u32,
    /// Chunks programmed.
    pub chunks: u32,
}

/// A LOG record read from a ring and held for the merge.
#[derive(Clone, Copy, Debug)]
struct Held {
    /// Where it starts in its ring.
    start: usize,
    /// Its uptime.
    uptime_us: u64,
}

/// The writer's view of one producer ring.
#[derive(Debug, Default)]
struct Source {
    /// The next record to read: past every record read so far, held ones included.
    read: usize,
    /// The LOG records read but not yet in a chunk, oldest first. The tail stops at the first.
    held: VecDeque<Held>,
}

/// A LOG record the writer makes itself.
#[derive(Debug)]
struct OwnLog {
    /// Its uptime.
    uptime_us: u64,
    /// The whole record.
    record: Vec<u8>,
}

/// The chunk being filled.
#[derive(Debug, Default)]
struct Chunk {
    /// Whole records, verbatim.
    payload: Vec<u8>,
    /// The payload size it is programmed before: the open block's room when its first record fits there, else a
    /// whole new block's.
    limit: usize,
    /// The records in it, the chunk header's count.
    records: u16,
    /// The producer records in it, per class (the writer's own are not counted).
    classes: [u32; 2],
    /// Whether the flash ring refused it: programmed before anything else.
    refused: bool,
}

/// The writer's claim on its Recorder: one writer at a time.
struct Claim<'r>(&'r Recorder);

impl Drop for Claim<'_> {
    /// Marks the Recorder's writer ENDED, never back to none: the counts of what is not yet on flash started with
    /// this writer, and its rings' read positions die with it, so no second consumer may ever take the rings
    /// (and [`crate::Telemetry::due`] answers false from now on, so producers stop sampling).
    fn drop(&mut self) {
        self.0.end_writer();
    }
}

/// The ring_writer over the flash ring of `F`. See the module notes.
pub struct Writer<'r, 't, F: Flash> {
    /// The shared state.
    recorder: &'r Recorder,
    /// Released when the writer drops.
    _claim: Claim<'r>,
    /// The flash ring, as its one writer.
    ring: Ring<'t, F>,
    /// One per producer ring, `misc` last.
    sources: Vec<Source>,
    /// The writer's own LOG records, oldest first.
    own: VecDeque<OwnLog>,
    /// The chunk being filled.
    chunk: Chunk,
    /// The step's uptime.
    now_us: u64,
    /// Producer records programmed or dropped here, per class (wrapping, as the producers' counts).
    settled: [u32; 2],
    /// The most records ever queued, per class.
    max: [u32; 2],
    /// The first step's uptime.
    started_us: Option<u64>,
    /// Whether the session.csv `boot` row is in a chunk.
    booted: bool,
    /// Whether the session.csv `anchor` row is in a chunk.
    anchored: bool,
    /// When the status line was last logged.
    last_status_us: u64,
    /// The writer position then: the write rate is the flash between it and the next status line's.
    status_position: Position,
    /// Per stream: its HEADER record is in a chunk this boot, so the repeats carry it.
    headed: Vec<bool>,
    /// Where the last header repeat (at first, this boot's first block) went: the next one is due a MiB of flash
    /// later.
    repeated_at: Position,
    /// The uptime of the first step that saw the stage fall out of flight, while the ring still opens flight
    /// blocks; None otherwise.
    falling_since_us: Option<u64>,
    /// The uptime of the last step, completed.
    last_step_us: u64,
    /// Chunks programmed this boot.
    chunks: u32,
    /// Flash calls that failed this boot.
    flash_errors: u32,
    /// Whether a run of refusals has begun: its first is logged, and the run ends at the next program.
    refusing: bool,
    /// The writer position last published.
    published: Option<Position>,
}

impl<'r, 't, F: Flash> Writer<'r, 't, F> {
    /// The writer of `recorder` into `ring` (after the bring-up scan). From now on the Recorder runs:
    /// [`crate::Telemetry::due`] answers, until the writer drops.
    ///
    /// The ring takes the Recorder's flight flag, the stage as last told: the crumb's from [`Recorder::new`], or
    /// a refused warm start's [`Recorder::set_flight`] that main made while this thread still scanned. The ring
    /// has no block open yet, so nothing closes, and its first block opens with that flag. The ring's own flag is
    /// never copied back, which would undo a call made before the writer existed.
    ///
    /// Returns None when a writer has taken this Recorder before, running or ended: each ring has one consumer,
    /// and the counts of what is not yet on flash start with the first writer, so no other could carry them on.
    /// On the board the writer thread never ends.
    pub fn new(recorder: &'r Recorder, mut ring: Ring<'t, F>) -> Option<Writer<'r, 't, F>> {
        if !recorder.claim_writer() {
            return None;
        }
        ring.set_flight(recorder.flight());
        let sources = (0..recorder.sources())
            .map(|index| Source {
                read: recorder.source(index).ring.tail(),
                held: VecDeque::new(),
            })
            .collect();
        let position = ring.position();
        Some(Writer {
            recorder,
            _claim: Claim(recorder),
            repeated_at: position,
            status_position: position,
            ring,
            sources,
            own: VecDeque::new(),
            chunk: Chunk {
                payload: Vec::with_capacity(MAX_PAYLOAD),
                ..Chunk::default()
            },
            now_us: 0,
            settled: [0; 2],
            max: [0; 2],
            started_us: None,
            booted: false,
            anchored: false,
            last_status_us: 0,
            headed: vec![false; recorder.registry().streams().len()],
            falling_since_us: None,
            last_step_us: 0,
            chunks: 0,
            flash_errors: 0,
            refusing: false,
            published: None,
        })
    }

    /// One pass of the loop at uptime `now_us`, with the wall clock `epoch_s` (Unix seconds) for the session.csv
    /// rows. See the module notes.
    pub fn step(&mut self, now_us: u64, epoch_s: i64) -> Progress {
        let mut progress = Progress::default();
        self.now_us = now_us;
        self.apply_fall();
        if !self.chunk.refused || self.program(&mut progress) {
            self.index_rows(epoch_s, &mut progress);
            self.repeat_headers(&mut progress);
            self.drain_telemetry(&mut progress);
            self.status();
            self.merge_logs(&mut progress);
            if !self.chunk.refused {
                self.program(&mut progress);
            }
        }
        self.publish();
        self.last_step_us = now_us;
        progress
    }

    /// The flash ring.
    pub fn ring(&self) -> &Ring<'t, F> {
        &self.ring
    }

    /// The flash ring, giving the writer up (and with it the Recorder's claim).
    pub fn into_ring(self) -> Ring<'t, F> {
        self.ring
    }

    /// Write the session.csv `boot` row on the first step and the `anchor` row once `ANCHOR_AFTER_US` later, each
    /// after the HEADER record: a reader that starts at either finds the columns there (plan B's rule).
    fn index_rows(&mut self, epoch_s: i64, progress: &mut Progress) {
        let started_us = *self.started_us.get_or_insert(self.now_us);
        if !self.booted {
            self.last_status_us = self.now_us;
            self.booted = self.index_row(epoch_s, "boot", progress);
        }
        if self.booted && !self.anchored && self.now_us.saturating_sub(started_us) >= ANCHOR_AFTER_US {
            self.anchored = self.index_row(epoch_s, "anchor", progress);
        }
    }

    /// One session.csv HEADER record and the ROW of `source`, stamped now. Returns false when a chunk could not
    /// take them (retried next step, header again: a repeat the ring_saver does not print).
    fn index_row(&mut self, epoch_s: i64, source: &str, progress: &mut Progress) -> bool {
        let recorder = self.recorder;
        let Some(index) = recorder.registry().stream(StreamId::SESSION_INDEX) else {
            return true; // every registry holds it (Registry::new)
        };
        let (file, columns, durable) = (index.file(), index.columns(), index.durable());
        let now_us = self.now_us;
        let session_row = recorder.session_row(now_us, epoch_s, None, source, None);
        let mut date = String::new();
        let cells = session_row.cells(&mut date);
        let Some(row_length) = record::row_length(file, &cells) else {
            return true; // ten short cells always fit a chunk
        };
        let mut records = Vec::new();
        record::encode(&mut records, record::header_length(file, columns), |bytes| {
            record::write_header(bytes, file, durable, now_us, columns)
        });
        let header_end = records.len();
        record::encode(&mut records, row_length, |bytes| {
            record::write_row(bytes, file, durable, now_us, &cells)
        });
        self.add(&records[..header_end], None, progress) && self.add(&records[header_end..], None, progress)
    }

    /// Copy the HEADER record of every stream this boot has started into the chunks again, once
    /// `HEADER_REPEAT_BYTES` of flash lie between the writer position and the last repeat. A stream that never
    /// pushed gets no file. When a chunk cannot take them all, they all go again next step (a repeat twice costs a
    /// copy, never a line).
    fn repeat_headers(&mut self, progress: &mut Progress) {
        let position = self.ring.position();
        if flash_between(self.repeated_at, position) < u64::from(HEADER_REPEAT_BYTES) {
            return;
        }
        let recorder = self.recorder;
        let mut header = Vec::new();
        for (index, stream) in recorder.registry().streams().iter().enumerate() {
            if !self.headed[index] {
                continue;
            }
            header.clear();
            let (file, columns) = (stream.file(), stream.columns());
            record::encode(&mut header, record::header_length(file, columns), |bytes| {
                record::write_header(bytes, file, stream.durable(), self.now_us, columns)
            });
            if !self.add(&header, None, progress) {
                return;
            }
        }
        self.repeated_at = position;
    }

    /// Read every ring's queued records: telemetry into the chunks now, LOG records into the merge.
    fn drain_telemetry(&mut self, progress: &mut Progress) {
        let recorder = self.recorder;
        for index in 0..self.sources.len() {
            let ring = &recorder.source(index).ring;
            while !self.chunk.refused {
                let read = self.sources[index].read;
                // SAFETY: the writer is every producer ring's one consumer (its claim), and `read` is the ring's
                // tail or the after of a record it read; tails never pass it.
                let Some((start, bytes)) = (unsafe { ring.record(read) }) else {
                    break;
                };
                match Record::decode(bytes) {
                    Some(found) if found.kind == Kind::Log => self.sources[index].held.push_back(Held {
                        start,
                        uptime_us: found.uptime_us,
                    }),
                    Some(_) => {
                        if !self.add(bytes, Some(Class::Telemetry), progress) {
                            break;
                        }
                    }
                    None => {
                        // never queued so: dropped, counted, rather than stopping the drain
                        let log = bytes.get(2) == Some(&(Kind::Log as u8));
                        self.settle_dropped(if log { Class::Log } else { Class::Telemetry }, 1);
                    }
                }
                self.sources[index].read = ring.after(start, bytes.len());
            }
        }
    }

    /// Queue the status line when `stats_ms` has passed since the last: plan B's `Recorder :: <report>`, plus the
    /// flash ring's figures as `'ring': {...}`, which go to `stats` as well, with the write rate measured since the
    /// last status line.
    ///
    /// The figures count the closed blocks before the saver's published position saved, the rest unsaved (with no
    /// saver, no card, every closed block). The rate is the flash the writer position moved a second, block headers
    /// and unused block tails included, since those fill the ring as rows do: what turns the figures into
    /// `ring_erased_s` and `ring_unsaved_s` (spec "The ring_eraser and newest wins").
    fn status(&mut self) {
        let period_us = u64::from(self.recorder.settings().stats_ms) * 1000;
        let elapsed_us = self.now_us.saturating_sub(self.last_status_us);
        if elapsed_us < period_us {
            return;
        }
        self.last_status_us = self.now_us;
        let position = self.ring.position();
        let written = flash_between(self.status_position, position);
        self.status_position = position;
        let write_rate = (written * 1_000_000 / elapsed_us.max(1)).min(u64::from(u32::MAX)) as u32;
        let saver = self.recorder.saver_position();
        let figures = self.ring.table().figures(saver);
        self.recorder.published.store_ring(figures, write_rate);
        let count = self.count();
        let counters = |class: Class| Counters {
            count: count[class as usize],
            max: self.max[class as usize].max(count[class as usize]),
            dropped: self.recorder.dropped[class as usize].load(Ordering::Relaxed),
        };
        let ring = ring_member(&figures, self.chunks, self.flash_errors, position, saver, write_rate);
        let message = recorder_wire::status_message(
            self.recorder.session(),
            &counters(Class::Telemetry),
            &counters(Class::Log),
            &[("ring", &ring)],
        );
        self.own_log(&message);
    }

    /// Queue one of the writer's own log lines, stamped now, for the merge.
    fn own_log(&mut self, message: &str) {
        if self.own.len() >= OWN_LOGS {
            self.recorder.dropped[Class::Log as usize].fetch_add(1, Ordering::Relaxed);
            return;
        }
        let (descriptor, message) = record::log_parts(OWN_DESCRIPTOR, message);
        let mut bytes = Vec::new();
        record::encode(&mut bytes, record::log_length(descriptor, message), |slot| {
            record::write_log(slot, self.now_us, descriptor, message)
        });
        self.own.push_back(OwnLog {
            uptime_us: self.now_us,
            record: bytes,
        });
    }

    /// Put held LOG records into the chunks in uptime order across the rings (and the writer's own), each once it
    /// is `MERGE_HORIZON_US` old. Ties go to the lower ring, the writer's own last.
    fn merge_logs(&mut self, progress: &mut Progress) {
        let recorder = self.recorder;
        while !self.chunk.refused {
            let mut oldest: Option<(u64, Option<usize>)> = None;
            for (index, source) in self.sources.iter().enumerate() {
                if let Some(held) = source.held.front()
                    && oldest.is_none_or(|(uptime_us, _)| held.uptime_us < uptime_us)
                {
                    oldest = Some((held.uptime_us, Some(index)));
                }
            }
            if let Some(own) = self.own.front()
                && oldest.is_none_or(|(uptime_us, _)| own.uptime_us < uptime_us)
            {
                oldest = Some((own.uptime_us, None));
            }
            let Some((uptime_us, which)) = oldest else {
                break;
            };
            if self.now_us.saturating_sub(uptime_us) < MERGE_HORIZON_US {
                break;
            }
            match which {
                Some(index) => {
                    let start = self.sources[index].held.front().map_or(0, |held| held.start);
                    // SAFETY: a held record lies between its ring's tail and head (tails stop at the first held),
                    // and the writer is the ring's one consumer.
                    let held = unsafe { recorder.source(index).ring.record(start) };
                    match held {
                        Some((_, bytes)) => {
                            if !self.add(bytes, Some(Class::Log), progress) {
                                break;
                            }
                        }
                        None => self.settle_dropped(Class::Log, 1), // unreachable while the contract holds
                    }
                    self.sources[index].held.pop_front();
                }
                None => {
                    let Some(own) = self.own.pop_front() else {
                        break;
                    };
                    if !self.add(&own.record, None, progress) {
                        self.own.push_front(own);
                        break;
                    }
                }
            }
        }
    }

    /// Copy one whole record into the chunk: program the chunk first when the record would pass its limit.
    /// `class` names the producer record it is (None for the writer's own).
    ///
    /// Returns false when that program was refused: the record is not in the chunk, and the caller keeps it to
    /// try again. True once it is in the chunk, or when no chunk could ever hold it and it is dropped, counted.
    fn add(&mut self, bytes: &[u8], class: Option<Class>, progress: &mut Progress) -> bool {
        if self.chunk.refused {
            return false;
        }
        let full = self.chunk.payload.len() + bytes.len() > self.chunk.limit || self.chunk.records == u16::MAX;
        if !self.chunk.payload.is_empty() && full && !self.program(progress) {
            return false;
        }
        if self.chunk.payload.is_empty() {
            self.chunk.limit = self.limit(bytes.len());
            if bytes.len() > self.chunk.limit {
                // longer than a chunk: no producer queues one (row_length refuses it)
                if let Some(class) = class {
                    self.settle_dropped(class, 1);
                }
                return true;
            }
        }
        self.chunk.payload.extend_from_slice(bytes);
        self.chunk.records += 1;
        if let Some(class) = class {
            self.chunk.classes[class as usize] += 1;
        }
        if bytes.get(2) == Some(&(Kind::Header as u8))
            && let Some(found) = Record::decode(bytes)
            && let Some(stream) = self.recorder.registry().find(found.name())
        {
            self.headed[usize::from(stream.0)] = true;
        }
        true
    }

    /// The payload size a chunk whose first record is `first` bytes is programmed before: the open block's room
    /// when the record fits there ([`Ring::room`]), so a block fills before it closes; else a whole new block's.
    /// The flight flag is applied first, since a change closes the open block.
    fn limit(&mut self, first: usize) -> usize {
        self.apply_flight();
        let room = self.ring.room();
        if room >= first { room } else { MAX_PAYLOAD }
    }

    /// Apply a rise of the stage into flight, before a chunk: from now on the blocks the ring opens are flight
    /// blocks. A fall waits for [`Writer::apply_fall`].
    fn apply_flight(&mut self) {
        if self.recorder.flight() {
            self.ring.set_flight(true);
            self.falling_since_us = None;
        }
    }

    /// Apply a fall of the stage out of flight, at the start of a step, once every record queued before it is
    /// programmed: a completed step at least `MERGE_HORIZON_US` after the first step that saw the fall (that one
    /// drained every telemetry record queued before it, this one merged every log line), and no chunk waiting.
    fn apply_fall(&mut self) {
        if self.recorder.flight() || !self.ring.flight() {
            self.falling_since_us = None;
            return;
        }
        match self.falling_since_us {
            None => self.falling_since_us = Some(self.now_us),
            Some(since_us) if self.last_step_us >= since_us.saturating_add(MERGE_HORIZON_US) && !self.chunk.refused => {
                self.ring.set_flight(false);
                self.falling_since_us = None;
            }
            Some(_) => {}
        }
    }

    /// Program the chunk with one append. On success its records are settled and teed, the rings' tails move up
    /// to what is still unprogrammed, and `writer_position` is published.
    ///
    /// Returns false when the flash ring refused the chunk: it stays, marked refused, to be programmed first next
    /// step. True when it went out (or was empty).
    fn program(&mut self, progress: &mut Progress) -> bool {
        if self.chunk.payload.is_empty() {
            self.chunk.refused = false;
            return true;
        }
        self.apply_flight();
        match self.ring.append(&self.chunk.payload, self.chunk.records) {
            Ok(_) => {
                for class in [Class::Telemetry, Class::Log] {
                    self.settled[class as usize] =
                        self.settled[class as usize].wrapping_add(self.chunk.classes[class as usize]);
                }
                self.tee_chunk();
                self.chunks = self.chunks.wrapping_add(1);
                progress.chunks += 1;
                progress.records += u32::from(self.chunk.records);
                self.refusing = false;
            }
            Err(Error::WouldBlock) => return self.refuse("no erased block to open yet"),
            Err(Error::Exhausted) => return self.refuse("every block sequence number is spent"),
            Err(Error::Flash(_)) => {
                self.flash_errors = self.flash_errors.wrapping_add(1);
                return self.refuse("a flash call failed, so it goes again at the next offset");
            }
            Err(error @ (Error::TooLarge | Error::Geometry)) => {
                // a chunk this writer never fills so: dropped rather than retried forever
                for class in [Class::Telemetry, Class::Log] {
                    self.settle_dropped(class, self.chunk.classes[class as usize]);
                }
                self.own_log(&std::format!(
                    "a chunk of {} bytes was refused ({error:?}): lost",
                    self.chunk.payload.len()
                ));
            }
        }
        self.chunk.payload.clear();
        self.chunk.records = 0;
        self.chunk.classes = [0; 2];
        self.chunk.refused = false;
        self.release();
        self.publish_position();
        true
    }

    /// Mark the chunk refused, logging only the first refusal of a run. Returns false, the refusal.
    fn refuse(&mut self, why: &str) -> bool {
        self.chunk.refused = true;
        if !self.refusing {
            self.refusing = true;
            self.own_log(&std::format!(
                "a chunk of {} bytes waits: {why}; its records stay in PSRAM",
                self.chunk.payload.len()
            ));
        }
        false
    }

    /// Copy each record of the programmed chunk into its CC tee, verbatim: a LOG into the log tee, any other into
    /// the telemetry tee. A tee whose window is closed takes nothing.
    fn tee_chunk(&self) {
        let now_ms = (self.now_us / 1000) as u32;
        for found in Records::new(&self.chunk.payload) {
            let tee = if found.kind == Kind::Log {
                &self.recorder.log_tee
            } else {
                &self.recorder.telemetry_tee
            };
            // SAFETY: the writer is the tees' one producer (its claim).
            unsafe { tee.push(now_ms, found.bytes) };
        }
    }

    /// Settle `records` of `class` as dropped by the writer: counted in its drops, and no longer queued.
    fn settle_dropped(&mut self, class: Class, records: u32) {
        self.settled[class as usize] = self.settled[class as usize].wrapping_add(records);
        self.recorder.dropped[class as usize].fetch_add(records, Ordering::Relaxed);
    }

    /// Free each ring up to its first record not yet on flash: the first held LOG, else everything read. Called
    /// only with the chunk empty, so every record read and not held is programmed (or was dropped).
    fn release(&self) {
        for (index, source) in self.sources.iter().enumerate() {
            let tail = source.held.front().map_or(source.read, |held| held.start);
            // SAFETY: the writer is the ring's one consumer; `tail` is a record boundary between its tail and head
            // (a held record's start, or the after of the last record read), and nothing before it is used again.
            unsafe { self.recorder.source(index).ring.release(tail) };
        }
    }

    /// Records queued and not yet settled, per class: (pushed - settled), never below 0 (a producer counts its
    /// record just after publishing it, so the writer may settle it first).
    fn count(&self) -> [u32; 2] {
        let mut pushed = [0u32; 2];
        for index in 0..self.recorder.sources() {
            for (class, total) in pushed.iter_mut().enumerate() {
                *total = total.wrapping_add(self.recorder.source(index).pushed[class].load(Ordering::Relaxed));
            }
        }
        [0, 1].map(|class| (pushed[class].wrapping_sub(self.settled[class]) as i32).max(0) as u32)
    }

    /// Publish `writer_position` when it moved: the end of the last chunk programmed, or before the first one,
    /// offset 0 of the block this boot opens first.
    fn publish_position(&mut self) {
        let position = self.ring.position();
        if self.published != Some(position) {
            self.recorder.writer_position.store(position);
            self.published = Some(position);
        }
    }

    /// Publish the position, the counters, the chunks and the flash errors.
    fn publish(&mut self) {
        self.publish_position();
        let count = self.count();
        for (max, count) in self.max.iter_mut().zip(count) {
            *max = (*max).max(count);
        }
        self.recorder
            .published
            .store(count, self.max, self.chunks, self.flash_errors);
    }
}

/// The status line's `'ring'` member, a Python dict's repr: the flash ring's figures (with no saver, every closed
/// block counts unsaved), the chunks programmed, the flash calls that failed, `writer_position` and
/// `saver_position` (`None` with no saver) as `(block_sequence, offset)`, and the write rate in bytes a second.
fn ring_member(
    figures: &Figures,
    chunks: u32,
    flash_errors: u32,
    writer: Position,
    saver: Option<Position>,
    write_rate: u32,
) -> String {
    let pair = |position: Position| std::format!("({}, {})", position.block_sequence, position.offset);
    std::format!(
        "{{'erased': {}, 'saved': {}, 'unsaved': {}, 'other_version': {}, 'foreign': {}, 'lost': {}, 'bad': {}, \
         'chunks': {chunks}, 'flash_errors': {flash_errors}, 'writer': {}, 'saver': {}, 'rate': {write_rate}}}",
        figures.erased_ahead,
        figures.saved,
        figures.unsaved,
        figures.other_version,
        figures.foreign,
        figures.lost,
        figures.bad,
        pair(writer),
        saver.map_or_else(|| String::from("None"), pair),
    )
}

/// The flash between two writer positions, `from` no later than `to`: the blocks between them at 4 KiB each, block
/// headers and unused tails included, as a reader walking the ring meets them.
fn flash_between(from: Position, to: Position) -> u64 {
    let blocks = u64::from(to.block_sequence.wrapping_sub(from.block_sequence));
    (blocks * u64::from(BLOCK_BYTES) + u64::from(to.offset)).saturating_sub(u64::from(from.offset))
}
