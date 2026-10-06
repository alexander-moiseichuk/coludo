//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flash writer (doc/specs/avionics-port.md "The flash writer"; the drain of src/glider recorder.py `run`),
//! minus its thread: the firmware's writer thread (core 0, priority 7) calls [`Writer::step`] in a loop, every
//! 10 ms at most. Each step:
//! 1. drains the producer rings: telemetry records at once, log records held back until they are 20 ms old
//!    and then merged across rings by uptime, so recorder.log stays in uptime order;
//! 2. formats each record into its line (recorder_wire's text), copies the bare line into the CC tee whose window
//!    is open, wraps it with the CRC wrapper, and appends it to the open frame;
//! 3. cuts the frame into the flash ring when its oldest line is 50 ms old, when the next line would pass
//!    16 KiB or the end of the block the frame will land in (behind any frames waiting in the flash ring's
//!    spill), or right after a durable record;
//! 4. writes this boot's session.csv `boot` row first, the `anchor` row 60 s later, and the status line every
//!    `stats_ms`;
//! 5. runs one GC erase step after each frame, or once when idle, and publishes the programmed position (for
//!    the mover) and the figures.
//!
//! A record leaves its PSRAM ring only after the frame holding it is programmed (or queued in the flash ring's
//! spill, also PSRAM): the operator's rule, "better duplicates in both than lost data". So a ring's tail moves
//! only after a successful append, and never past a log record still held for the merge. When the flash ring
//! refuses a frame (its spill is full, or a program failed), the frame stays and is retried first next step;
//! nothing more is read meanwhile, the producer rings fill, and only then does [`crate::Telemetry::push`] raise.
//!
//! **A failed program is repeated whole, and the repeat names the torn frame.** A program call that fails
//! without a power cut can still have programmed a prefix of its frame, header first, in a block the flash ring
//! then closes. The frame goes again, byte for byte, as the boot's next DATA frame: from this writer's retry,
//! or from the ring's spill drain for a spilled one. So for the SD mover's salvage (doc/specs/avionics-port.md
//! "SD mover", torn frame): a torn DATA frame whose header verifies, followed in the same boot by a DATA frame
//! with the same `uptime_us` and `payload_length` whose payload starts with the torn frame's surviving bytes,
//! was repeated, and the mover files the repeat alone, salvaging nothing of the torn one. Otherwise its
//! salvaged lines would land twice, and the rows going back in uptime would read as a reboot. A torn header
//! needs no rule: the header goes first, so nothing of its payload was programmed.

use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use std::vec::Vec;

use flash_ring::frame::Kind as FrameKind;
use flash_ring::{Error, Figures, Flash, Position, Ring};
use recorder_wire::{Counters, SESSION_INDEX, SESSION_INDEX_HEADER, WRAPPER_BYTES};

use crate::record::{Kind, Record};
use crate::recorder::{Class, Recorder};

/// The largest frame payload: a frame is one flash program, and its buffer lives in internal RAM.
pub const FRAME_BYTES: usize = 16 << 10;

/// A frame is cut once its oldest line is this old: with the program and an erase in progress, it bounds what
/// a power cut loses (~70 ms typical, ~115 ms worst).
pub const CUT_AGE_US: u64 = 50_000;

/// A log record is emitted once it is this old, so a line a slower thread queues a little later still lands in
/// uptime order.
pub const MERGE_HORIZON_US: u64 = 20_000;

/// The session.csv `anchor` row goes out this long after the writer starts: it lists the boot for a reader that
/// missed its `boot` row, dated if the clock was set by then.
pub const ANCHOR_AFTER_US: u64 = 60_000_000;

/// The writer's own log lines waiting for the merge (status lines, flash errors). A flash that refuses frames
/// for long keeps adding them; past this they drop, counted.
const OWN_LOGS: usize = 64;

/// The bytes a line of `text` payload bytes takes in a frame once wrapped: `@<routing>@` (none for a log line),
/// the CRC wrapper and the line end. The producer side checks a row's and a header's worst case against
/// [`FRAME_BYTES`] with it, so that every telemetry record queued can be programmed.
pub(crate) fn wrapped_bytes(routing: Option<&str>, text: usize) -> usize {
    routing.map_or(0, |routing| routing.len() + 2) + text + WRAPPER_BYTES + 1
}

/// What one step did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    /// Lines appended to frames.
    pub lines: u32,
    /// Frames appended to the flash ring.
    pub frames: u32,
}

/// A log record read from a ring and held for the merge.
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
    /// The log records read but not yet emitted, oldest first. The tail stops at the first.
    held: VecDeque<Held>,
}

/// A log line the writer makes itself, already formatted.
#[derive(Debug)]
struct OwnLog {
    /// Its uptime.
    uptime_us: u64,
    /// The line, unwrapped.
    payload: Vec<u8>,
}

/// The frame being filled.
#[derive(Debug, Default)]
struct Frame {
    /// Wrapped lines.
    bytes: Vec<u8>,
    /// The size it is cut before, set when its first line goes in, and again when the flash ring's spill
    /// drains under it.
    limit: usize,
    /// Its first line's uptime: the frame header's, and the cut age's start.
    first_uptime_us: u64,
    /// Whether a durable record cut it.
    durable: bool,
    /// The producer records in it, per class.
    records: [u32; 2],
    /// Whether the flash ring refused it: retried before anything else.
    refused: bool,
}

/// The flash writer over the flash ring of `F`. See the module notes.
pub struct Writer<'r, F: Flash> {
    /// The shared state.
    recorder: &'r Recorder,
    /// Released when the writer drops.
    _claim: Claim<'r>,
    /// The flash ring, owned.
    ring: Ring<F>,
    /// One per producer ring, `misc` last.
    sources: Vec<Source>,
    /// The writer's own log lines, oldest first.
    own: VecDeque<OwnLog>,
    /// The frame being filled.
    frame: Frame,
    /// The line being formatted, unwrapped.
    line: Vec<u8>,
    /// The step's uptime.
    now_us: u64,
    /// Producer records programmed or dropped here, per class (wrapping, as the producers' counts).
    settled: [u32; 2],
    /// The most records ever queued, per class.
    max: [u32; 2],
    /// The first step's uptime.
    started_us: Option<u64>,
    /// Whether the session.csv `boot` row is written.
    booted: bool,
    /// Whether the session.csv `anchor` row is written.
    anchored: bool,
    /// When the status line was last logged.
    last_status_us: u64,
    /// DATA frames appended.
    frames: u32,
    /// Flash calls that failed.
    flash_errors: u32,
    /// The programmed position last published.
    published: Option<Position>,
}

/// The writer's claim on its Recorder: one writer at a time.
struct Claim<'r>(&'r Recorder);

impl Drop for Claim<'_> {
    fn drop(&mut self) {
        self.0.end_writer();
    }
}

impl<'r, F: Flash> Writer<'r, F> {
    /// The writer of `recorder` into `ring` (mounted, its CONFIRM recovered). From now on the Recorder runs:
    /// [`crate::Telemetry::due`] answers, until the writer drops.
    ///
    /// Returns None when a writer has taken this Recorder before, running or ended: each ring has one consumer,
    /// and the counts of what is not yet on flash start with the first writer, so no other could carry them on.
    /// On the board the writer thread never ends.
    pub fn new(recorder: &'r Recorder, ring: Ring<F>) -> Option<Writer<'r, F>> {
        if !recorder.claim_writer() {
            return None;
        }
        let sources = (0..recorder.sources())
            .map(|index| Source {
                read: recorder.source(index).ring.tail(),
                held: VecDeque::new(),
            })
            .collect();
        Some(Writer {
            recorder,
            _claim: Claim(recorder),
            ring,
            sources,
            own: VecDeque::new(),
            frame: Frame::default(),
            line: Vec::new(),
            now_us: 0,
            settled: [0; 2],
            max: [0; 2],
            started_us: None,
            booted: false,
            anchored: false,
            last_status_us: 0,
            frames: 0,
            flash_errors: 0,
            published: None,
        })
    }

    /// One pass of the loop at uptime `now_us`, with the wall clock `epoch_s` (Unix seconds) for the session.csv
    /// rows. See the module notes.
    pub fn step(&mut self, now_us: u64, epoch_s: i64) -> Progress {
        let mut progress = Progress::default();
        self.now_us = now_us;
        self.ring.pause_erases(self.recorder.erases_paused());
        if !self.frame.refused || self.cut(&mut progress) {
            self.index_rows(epoch_s, &mut progress);
            self.drain_telemetry(&mut progress);
            self.status();
            self.merge_logs(&mut progress);
            let aged = now_us.saturating_sub(self.frame.first_uptime_us) >= CUT_AGE_US;
            if !self.frame.refused && !self.frame.bytes.is_empty() && aged {
                self.cut(&mut progress);
            }
        }
        if progress.frames == 0 {
            // idle: program what waits in the spill as soon as GC freed a block, and keep GC going
            if let Err(error) = self.ring.drain() {
                self.flash_failed(&error);
            }
            if !self.frame.bytes.is_empty() && !self.frame.refused {
                // a drain places the spilled frames where the frame's limit expected them; one that failed
                // closed that block, and the frame now goes behind it
                self.frame.limit = self.limit(self.frame.bytes.len());
            }
            self.collect();
        }
        self.publish();
        progress
    }

    /// The flash ring.
    pub fn ring(&self) -> &Ring<F> {
        &self.ring
    }

    /// The flash ring, giving the writer up (and with it the Recorder's claim).
    pub fn into_ring(self) -> Ring<F> {
        self.ring
    }

    /// Write the session.csv `boot` row on the first step and the `anchor` row once `ANCHOR_AFTER_US` later,
    /// each after the header: the first header may have reached nobody (plan B's rule, kept for the SD's
    /// session.csv, which every boot appends to).
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

    /// One session.csv header and row of `source`; false when the frame could not take them (retried next step,
    /// header again).
    fn index_row(&mut self, epoch_s: i64, source: &str, progress: &mut Progress) -> bool {
        self.line.clear();
        self.line.extend_from_slice(SESSION_INDEX_HEADER.as_bytes());
        if !self.emit(Some(SESSION_INDEX), self.now_us, false, None, progress) {
            return false;
        }
        self.line.clear();
        let recorder = self.recorder;
        recorder
            .session_row(self.now_us, epoch_s, None, source, None)
            .append_to(&mut self.line);
        self.emit(Some(SESSION_INDEX), self.now_us, true, None, progress)
    }

    /// Read every ring's queued records: telemetry into the frame now, logs into the merge.
    fn drain_telemetry(&mut self, progress: &mut Progress) {
        let recorder = self.recorder;
        for index in 0..self.sources.len() {
            let ring = &recorder.source(index).ring;
            while !self.frame.refused {
                let read = self.sources[index].read;
                // SAFETY: the writer is every producer ring's one consumer (its claim), and `read` is the ring's
                // tail or the after of a record it read; tails never pass it.
                let Some((start, bytes)) = (unsafe { ring.record(read) }) else {
                    break;
                };
                let after = ring.after(start, bytes.len());
                let Some(record) = Record::decode(bytes) else {
                    self.sources[index].read = after; // never written so: skip it rather than stop the drain
                    continue;
                };
                if record.kind == Kind::Log {
                    self.sources[index].held.push_back(Held {
                        start,
                        uptime_us: record.uptime_us,
                    });
                    self.sources[index].read = after;
                    continue;
                }
                let routing = match recorder.registry().stream(crate::StreamId(record.stream)) {
                    Some(stream) => {
                        self.line.clear();
                        match record.kind {
                            Kind::Header => self.line.extend_from_slice(stream.header()),
                            Kind::Row => recorder_wire::row(&mut self.line, record.uptime_us, &record.cells()),
                            _ => self.line.extend_from_slice(record.line()),
                        }
                        stream.routing()
                    }
                    None => {
                        // a stream this registry lacks: never queued so; settled as dropped
                        self.settle_dropped(Class::Telemetry, 1);
                        self.sources[index].read = after;
                        continue;
                    }
                };
                if !self.emit(
                    Some(routing),
                    record.uptime_us,
                    record.durable,
                    Some(Class::Telemetry),
                    progress,
                ) {
                    break;
                }
                self.sources[index].read = after;
            }
        }
    }

    /// Queue the status line when `stats_ms` has passed since the last: plan B's `Recorder :: <report>`, plus the
    /// flash ring's figures as `'ring': {...}` (doc/specs/avionics-port.md "The flash writer").
    fn status(&mut self) {
        let period_us = u64::from(self.recorder.settings().stats_ms) * 1000;
        if self.now_us.saturating_sub(self.last_status_us) < period_us {
            return;
        }
        self.last_status_us = self.now_us;
        let count = self.count();
        let counters = |class: Class| Counters {
            count: count[class as usize],
            max: self.max[class as usize].max(count[class as usize]),
            dropped: self.recorder.dropped[class as usize].load(Ordering::Relaxed),
        };
        let ring = ring_member(&self.ring.figures(), self.frames, self.flash_errors);
        let message = recorder_wire::status_message(
            self.recorder.registry().session(),
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
        let mut payload = Vec::new();
        recorder_wire::log_line(&mut payload, self.now_us, "Recorder", message, usize::MAX);
        self.own.push_back(OwnLog {
            uptime_us: self.now_us,
            payload,
        });
    }

    /// Emit held log lines in uptime order across the rings (and the writer's own), each once it is
    /// `MERGE_HORIZON_US` old. Ties go to the lower ring, the writer's own last.
    fn merge_logs(&mut self, progress: &mut Progress) {
        let recorder = self.recorder;
        while !self.frame.refused {
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
            self.line.clear();
            match which {
                Some(index) => {
                    let start = self.sources[index].held.front().map_or(0, |held| held.start);
                    // SAFETY: a held record lies between its ring's tail and head (tails stop at the first held),
                    // and the writer is the ring's one consumer.
                    let record = unsafe { recorder.source(index).ring.record(start) }
                        .and_then(|(_, bytes)| Record::decode(bytes));
                    if let Some(record) = record {
                        let (descriptor, message) = record.log();
                        // the producer cut the descriptor and message already: no further limit here
                        recorder_wire::log_line(&mut self.line, uptime_us, &descriptor, &message, usize::MAX);
                        if !self.emit(None, uptime_us, false, Some(Class::Log), progress) {
                            break;
                        }
                    } else {
                        self.settle_dropped(Class::Log, 1);
                    }
                    self.sources[index].held.pop_front();
                }
                None => {
                    if let Some(own) = self.own.front() {
                        self.line.extend_from_slice(&own.payload);
                    }
                    if !self.emit(None, uptime_us, false, None, progress) {
                        break;
                    }
                    self.own.pop_front();
                }
            }
        }
    }

    /// Append the line in `self.line` (routed, or a log line for `routing` None) to the frame: cut first when it
    /// would pass the frame's limit, tee it, wrap it, and cut after it when durable. `class` names the producer
    /// record it came from (None for the writer's own lines).
    ///
    /// Returns false when a cut before it failed: the line is not in the frame, and the caller keeps its record
    /// to retry. True once the line is in the frame (a failed cut after a durable line leaves the frame refused,
    /// for the next step), or when it can never fit any frame and is dropped, counted.
    fn emit(
        &mut self,
        routing: Option<&str>,
        uptime_us: u64,
        durable: bool,
        class: Option<Class>,
        progress: &mut Progress,
    ) -> bool {
        let wrapped = wrapped_bytes(routing, self.line.len());
        if !self.frame.bytes.is_empty() && self.frame.bytes.len() + wrapped > self.frame.limit && !self.cut(progress) {
            return false;
        }
        if self.frame.bytes.is_empty() {
            self.frame.limit = self.limit(wrapped);
            self.frame.first_uptime_us = uptime_us;
            self.frame.durable = false;
            if wrapped > self.frame.limit {
                if let Some(class) = class {
                    self.settle_dropped(class, 1);
                }
                return true;
            }
        }
        self.tee(routing);
        recorder_wire::wrap_into(&mut self.frame.bytes, routing, &self.line);
        if let Some(class) = class {
            self.frame.records[class as usize] += 1;
        }
        progress.lines += 1;
        if durable {
            self.frame.durable = true;
            self.cut(progress);
        }
        true
    }

    /// Copy the line into its CC tee: a log line as it is, a routed one as `@<routing>@<line>`.
    fn tee(&self, routing: Option<&str>) {
        let now_ms = (self.now_us / 1000) as u32;
        // SAFETY: the writer is the tees' one producer (its claim).
        match routing {
            None => unsafe { self.recorder.log_tee.push(now_ms, &[&self.line]) },
            Some(routing) => unsafe {
                self.recorder
                    .telemetry_tee
                    .push(now_ms, &[b"@", routing.as_bytes(), b"@", &self.line])
            },
        }
    }

    /// The size a frame of `bytes` wrapped bytes (its first line, or what it holds so far) is cut before: 16 KiB,
    /// or less when they fit the rest of the block the frame will land in, behind the frames waiting in the
    /// spill ([`Ring::room_behind_spill`]), so a frame never closes a block with room left in it. A frame that
    /// does not fit there goes to a new block.
    fn limit(&self, bytes: usize) -> usize {
        let room = self.ring.room_behind_spill();
        if room >= bytes {
            room.min(FRAME_BYTES)
        } else {
            self.ring.max_payload().min(FRAME_BYTES)
        }
    }

    /// Append the frame to the flash ring. On success the records in it are settled, the rings' tails move up
    /// to what is still unprogrammed, the programmed position is published, and GC takes one step.
    ///
    /// Returns false when the flash ring refused the frame (its spill is full, or a flash call failed): the
    /// frame stays, marked refused, to be retried first next step. True when it went out (or was empty).
    fn cut(&mut self, progress: &mut Progress) -> bool {
        if self.frame.bytes.is_empty() {
            self.frame.refused = false;
            return true;
        }
        let appended = self.ring.append(
            FrameKind::Data,
            &self.frame.bytes,
            self.frame.durable,
            self.frame.first_uptime_us,
        );
        match appended {
            Ok(()) => {
                for class in [Class::Telemetry, Class::Log] {
                    self.settled[class as usize] =
                        self.settled[class as usize].wrapping_add(self.frame.records[class as usize]);
                }
                self.frame.refused = false;
                self.frames = self.frames.wrapping_add(1);
                progress.frames += 1;
            }
            Err(Error::SpillFull) => {
                self.refuse("the flash ring's spill is full");
                return false;
            }
            Err(error @ Error::Flash(_)) => {
                // retried whole, as the boot's next DATA frame: see the module notes on a torn frame's repeat
                self.flash_failed(&error);
                self.refuse("a flash program failed");
                return false;
            }
            Err(error) => {
                // TooLarge, BootKind, Geometry: a frame this writer never cuts. Drop it rather than retry forever.
                for class in [Class::Telemetry, Class::Log] {
                    self.settle_dropped(class, self.frame.records[class as usize]);
                }
                self.frame.refused = false;
                self.own_log(&std::format!(
                    "a frame of {} bytes was refused ({error:?}): lost",
                    self.frame.bytes.len()
                ));
            }
        }
        self.frame.bytes.clear();
        self.frame.records = [0; 2];
        self.commit();
        self.collect();
        true
    }

    /// Mark the frame refused, logging only the first refusal of a run.
    fn refuse(&mut self, why: &str) {
        if !self.frame.refused {
            self.frame.refused = true;
            self.own_log(&std::format!(
                "frame of {} bytes waits: {why}; records stay in PSRAM",
                self.frame.bytes.len()
            ));
        }
    }

    /// Count a failed flash call.
    fn flash_failed<E: core::fmt::Debug>(&mut self, _error: &Error<E>) {
        self.flash_errors = self.flash_errors.wrapping_add(1);
    }

    /// Settle `records` of `class` as dropped by the writer: counted in its drops, and no longer queued.
    fn settle_dropped(&mut self, class: Class, records: u32) {
        self.settled[class as usize] = self.settled[class as usize].wrapping_add(records);
        self.recorder.dropped[class as usize].fetch_add(records, Ordering::Relaxed);
    }

    /// Free each ring up to its first record not yet on flash: the first held log, else everything read.
    /// Called only with the frame empty, so every record read and not held is programmed.
    fn commit(&self) {
        for (index, source) in self.sources.iter().enumerate() {
            let tail = source.held.front().map_or(source.read, |held| held.start);
            // SAFETY: the writer is the ring's one consumer; `tail` is a record boundary between its tail and head
            // (a held record's start, or the after of the last record read), and nothing before it is used again.
            unsafe { self.recorder.source(index).ring.release(tail) };
        }
    }

    /// One GC step, counting a failed erase.
    fn collect(&mut self) {
        if let Err(error) = self.ring.erase_step() {
            self.flash_failed(&error);
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

    /// Publish the programmed position (for the mover; only when it moved), the counters, the ring figures and
    /// the frame and error counts.
    fn publish(&mut self) {
        let programmed = self.ring.programmed();
        if programmed != self.published {
            if let Some(position) = programmed {
                self.recorder.programmed.store(position);
            }
            self.published = programmed;
        }
        let count = self.count();
        for (max, count) in self.max.iter_mut().zip(count) {
            *max = (*max).max(count);
        }
        self.recorder
            .published
            .store(count, self.max, self.ring.figures(), self.frames, self.flash_errors);
    }
}

/// The status line's `'ring'` member, a Python dict's repr: the flash ring's figures, the frames appended and the
/// flash calls that failed.
fn ring_member(figures: &Figures, frames: u32, flash_errors: u32) -> std::string::String {
    std::format!(
        "{{'erased_ahead': {}, 'moved_not_erased': {}, 'foreign': {}, 'other_version': {}, 'unconfirmed': {}, \
         'spill_frames': {}, 'spill_bytes': {}, 'frames': {frames}, 'flash_errors': {flash_errors}}}",
        figures.erased_ahead,
        figures.moved_not_erased,
        figures.foreign,
        figures.other_version,
        figures.unconfirmed,
        figures.spill_frames,
        figures.spill_bytes,
    )
}
