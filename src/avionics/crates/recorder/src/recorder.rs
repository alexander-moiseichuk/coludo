//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The Recorder (src/glider recorder.py `Recorder`; doc/specs/avionics-port.md "Producer side"): the state every
//! thread shares, built once in bring-up before any thread spawns and leaked to `&'static` on the board.
//!
//! Producers never format text and never take a lock on their path:
//! * a producing thread [`Recorder::attach`]es itself to its own PSRAM ring, once (control, i2c0, i2c1, gnss,
//!   supervisor, executor). From then on its [`crate::Telemetry`] rows and [`Recorder::log`] lines go into that
//!   ring alone: a single-producer, single-consumer byte ring, with the ring_writer the one consumer;
//! * any other thread (an unregistered one, IDF's vprintf hook) shares the `misc` ring, behind a `try_lock`:
//!   contention drops the record and counts it, it never waits.
//!
//! A thread finds its ring through a thread-local binding set by `attach`, so a log line reaches the right ring
//! from anywhere (the `log` crate bridge has no handle to pass). The binding names the Recorder by a process-
//! unique id, so a thread attached to one Recorder writes into the `misc` ring of another.
//!
//! A durable record (a durable stream's, a session.csv row) wakes the writer through [`Wake`] as soon as it is
//! queued, so it skips the writer's ~10 ms tick (spec "The ring_writer": a durable record is at risk for at most
//! ~35 ms, not ~45).
//!
//! The error policy is plan B's: telemetry is evidence and raises ([`Overflow`] when its ring is full, counted
//! in `tlm.dropped`), a log line is a convenience and is dropped with a count (`log.dropped`).

use std::cell::Cell as LocalCell;
use std::marker::PhantomData;
use std::string::String;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Mutex, TryLockError};
use std::vec::Vec;

use flash_ring::Position;
use flash_ring::header::Session;
use recorder_wire::{Cell, SessionRow};

use crate::byte_ring::ByteRing;
use crate::published::{PublishedPosition, PublishedStats, Stats};
use crate::record;
use crate::registry::{Registry, StreamId};
use crate::tee::{Tee, TeeBatch};

/// A record class: the status line and `stats` count them apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Class {
    /// HEADER and ROW records (session.csv rows among them): evidence.
    Telemetry = 0,
    /// LOG records: best effort.
    Log = 1,
}

/// A telemetry record that could not be queued: its producer's ring is full (or, for the `misc` ring, busy), or
/// the row is longer than a flash chunk holds (4032 bytes) or has more than 255 cells. Counted in `tlm.dropped`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Overflow;

impl core::fmt::Display for Overflow {
    /// The text a caller logs when it reports the refused row: what went wrong, not which stream (the caller knows
    /// that, and `tlm.dropped` has already counted it).
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str("telemetry dropped: no room in the recorder ring")
    }
}

impl std::error::Error for Overflow {}

/// Who this boot is, for the session.csv rows and the status line.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    /// The NVS boot id; None when nothing counted the boot.
    pub boot_id: Option<u32>,
    /// The session prefix (recorder_wire's `session_prefix`), the label every block header of this boot carries:
    /// the bring-up passes the same one to `flash_ring::Ring::new`.
    pub session: Session,
    /// The board id from the config.
    pub board: String,
    /// The firmware version.
    pub firmware: String,
    /// The config id the firmware announces.
    pub config_id: String,
}

/// The recorder's config (`recorder.*`), resolved by bring-up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// `recorder.telemetry_ms`: the window of every stream that sets none of its own, in ms (0 = none).
    pub telemetry_ms: u32,
    /// `recorder.stats_ms`: how often the writer logs the status line, in ms. The operator may change it at
    /// runtime ([`Recorder::set_stats_ms`]).
    pub stats_ms: u32,
}

/// What `inspect` reports (plan B `Recorder.inspect`): the session, the status line's period, and the PSRAM the
/// producer rings hold, which replace plan B's `tlm_capacity`, `log_capacity` and `cell_size`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Inspection<'r> {
    /// The session prefix.
    pub session: &'r str,
    /// The status line's period in ms, as it is now.
    pub stats_ms: u32,
    /// Each attachable producer ring's capacity in bytes, by the index [`Recorder::attach`] takes.
    pub producer_bytes: Vec<usize>,
    /// The `misc` ring's capacity in bytes.
    pub misc_bytes: usize,
}

/// The writer claim's states: none yet, one running, and one that ran and ended.
mod writer_state {
    /// No writer has taken the Recorder yet.
    pub(super) const NONE: u32 = 0;
    /// A writer runs.
    pub(super) const RUNNING: u32 = 1;
    /// The writer ended; no other may take its place.
    pub(super) const ENDED: u32 = 2;
}

/// The PSRAM the rings and tees live in. The firmware allocates each buffer once (`heap_caps_malloc`, SPIRAM)
/// and never frees it, so it hands them over as `&'static mut`.
pub struct Buffers {
    /// One ring per producing thread, by the index [`Recorder::attach`] takes.
    pub producers: Vec<&'static mut [u8]>,
    /// The ring unregistered threads share.
    pub misc: &'static mut [u8],
    /// The log tee's records (256 KiB in the plan).
    pub log_tee: &'static mut [u8],
    /// The telemetry tee's records, likewise.
    pub telemetry_tee: &'static mut [u8],
}

/// The uptime in microseconds (`esp_timer_get_time` on the board): what a log line and a session.csv time-set
/// row are stamped with.
pub type Clock = Box<dyn Fn() -> u64 + Send + Sync>;

/// Wakes the writer thread at once (`xTaskNotifyGive` on the board): called right after a durable record is
/// queued, from the producer's thread. It must not block.
pub type Wake = Box<dyn Fn() + Send + Sync>;

/// One producer ring and its counters.
pub(crate) struct Producer {
    /// The records, the ring_writer its consumer.
    pub(crate) ring: ByteRing,
    /// Whether a thread holds it ([`Recorder::attach`]); always false for the `misc` ring, which a lock guards.
    claimed: AtomicBool,
    /// Records queued so far, per [`Class`] (wrapping).
    pub(crate) pushed: [AtomicU32; 2],
}

impl Producer {
    /// A producer ring over `buffer`.
    fn new(buffer: &'static mut [u8]) -> Producer {
        Producer {
            ring: ByteRing::new(buffer),
            claimed: AtomicBool::new(false),
            pushed: [AtomicU32::new(0), AtomicU32::new(0)],
        }
    }

    /// Queue one record of `class`; false when it does not fit.
    ///
    /// # Safety
    /// The caller is this ring's one producer: the thread that claimed it, or the holder of the misc lock.
    unsafe fn push(&self, class: Class, length: usize, fill: impl FnOnce(&mut [u8])) -> bool {
        // SAFETY: the caller's contract.
        let stored = unsafe { self.ring.push(length, fill) };
        if stored {
            self.pushed[class as usize].fetch_add(1, Ordering::Relaxed);
        }
        stored
    }
}

/// A thread's binding to one ring of one Recorder.
#[derive(Clone, Copy, Debug)]
struct Binding {
    /// The Recorder's [`Recorder::id`].
    recorder: u32,
    /// The ring's index.
    ring: usize,
}

std::thread_local! {
    /// This thread's ring, set by [`Recorder::attach`] and cleared when its [`Attachment`] drops.
    static ATTACHED: LocalCell<Option<Binding>> = const { LocalCell::new(None) };
}

/// The next Recorder's id. A u32, as the P4 has no 64-bit atomics; ids are never reused within a boot.
static NEXT_RECORDER: AtomicU32 = AtomicU32::new(1);

/// Why a thread cannot attach to a ring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachError {
    /// The Recorder has no ring of that index.
    NoSuchRing,
    /// Another thread holds that ring: a ring has one producer.
    Claimed,
    /// This thread already holds a ring.
    ThreadAttached,
}

/// A thread's hold on its ring. While it lives, the thread's telemetry and log records go into that ring; when
/// it drops, the ring is free for another thread. It stays on the thread that attached (it is not Send).
pub struct Attachment<'r> {
    /// The Recorder whose ring it holds.
    recorder: &'r Recorder,
    /// The ring's index.
    ring: usize,
    /// Not Send: the binding is this thread's.
    _thread: PhantomData<*const ()>,
}

impl Drop for Attachment<'_> {
    /// Gives the ring up: first this thread's binding is cleared, so none of its later records can reach the ring,
    /// then the claim is released with a Release store. The next thread to attach takes the claim with an Acquire
    /// swap, so it is ordered after every push this thread made, and the ring keeps one producer at a time (the
    /// byte ring's contract). `try_with`: during thread teardown the binding may already be gone, and nothing is
    /// left to clear then.
    fn drop(&mut self) {
        let _ = ATTACHED.try_with(|attached| attached.set(None));
        self.recorder.producers[self.ring]
            .claimed
            .store(false, Ordering::Release);
    }
}

/// The recorder's shared state. See the module notes.
pub struct Recorder {
    /// A process-unique id, which thread bindings name.
    id: u32,
    /// The streams.
    registry: Registry,
    /// The boot, for the session.csv rows and the status line.
    identity: Identity,
    /// `recorder.telemetry_ms`, fixed for the boot.
    telemetry_ms: u32,
    /// `recorder.stats_ms`, which the operator may change: the writer reads it every step.
    stats_ms: AtomicU32,
    /// The uptime clock.
    clock: Clock,
    /// Wakes the writer for a durable record.
    wake: Wake,
    /// One ring per producing thread.
    pub(crate) producers: Vec<Producer>,
    /// The ring unregistered threads share.
    pub(crate) misc: Producer,
    /// Guards the `misc` ring's producer side.
    misc_lock: Mutex<()>,
    /// The CC copy of the LOG records.
    pub(crate) log_tee: Tee,
    /// The CC copy of the HEADER and ROW records.
    pub(crate) telemetry_tee: Tee,
    /// Records refused per [`Class`]: by a full or busy ring, or by the writer as never programmable.
    pub(crate) dropped: [AtomicU32; 2],
    /// The writer's figures.
    pub(crate) published: PublishedStats,
    /// `writer_position`: the end of what is programmed, for the ring_saver.
    pub(crate) writer_position: PublishedPosition,
    /// Whether the stage is in flight (BOOSTING..DONE): the blocks the writer opens carry the flight flag.
    flight: AtomicBool,
    /// Whether a [`crate::Writer`] has taken this Recorder, and whether it still runs (`writer_state`).
    writer: AtomicU32,
}

impl Recorder {
    /// The Recorder over `buffers`, with its streams, boot identity and settings, the uptime `clock` and the
    /// writer's `wake`. Nothing runs until a [`crate::Writer`] takes it; records queue in the rings meanwhile.
    ///
    /// `flight` is the stage the bring-up starts from: true when the crumb names BOOSTING..LANDING (a reboot in
    /// flight, spec "Ring geometry and block life cycle"), false for every other boot. The Recorder owns the flag
    /// from here on: [`Recorder::set_flight`] may change it at any time, before the writer exists too, and the
    /// writer applies it to its ring ([`crate::Writer::new`]), never the other way round. So a warm start refused
    /// on main while the writer thread still scans is not lost.
    pub fn new(
        registry: Registry,
        identity: Identity,
        settings: Settings,
        flight: bool,
        buffers: Buffers,
        clock: Clock,
        wake: Wake,
    ) -> Recorder {
        Recorder {
            id: NEXT_RECORDER.fetch_add(1, Ordering::Relaxed),
            registry,
            identity,
            telemetry_ms: settings.telemetry_ms,
            stats_ms: AtomicU32::new(settings.stats_ms),
            clock,
            wake,
            producers: buffers.producers.into_iter().map(Producer::new).collect(),
            misc: Producer::new(buffers.misc),
            misc_lock: Mutex::new(()),
            log_tee: Tee::new(buffers.log_tee),
            telemetry_tee: Tee::new(buffers.telemetry_tee),
            dropped: [AtomicU32::new(0), AtomicU32::new(0)],
            published: PublishedStats::default(),
            writer_position: PublishedPosition::new(),
            flight: AtomicBool::new(flight),
            writer: AtomicU32::new(writer_state::NONE),
        }
    }

    /// Attach the calling thread to producer ring `ring`, as its one producer, until the returned
    /// [`Attachment`] drops.
    ///
    /// Errors: `NoSuchRing` for an index past the rings, `Claimed` when another thread holds it,
    /// `ThreadAttached` when this thread already holds one.
    pub fn attach(&self, ring: usize) -> Result<Attachment<'_>, AttachError> {
        let producer = self.producers.get(ring).ok_or(AttachError::NoSuchRing)?;
        if ATTACHED.with(LocalCell::get).is_some() {
            return Err(AttachError::ThreadAttached);
        }
        if producer.claimed.swap(true, Ordering::Acquire) {
            return Err(AttachError::Claimed);
        }
        ATTACHED.with(|attached| {
            attached.set(Some(Binding {
                recorder: self.id,
                ring,
            }))
        });
        Ok(Attachment {
            recorder: self,
            ring,
            _thread: PhantomData,
        })
    }

    /// Queue one record of `class` from the calling thread: into its own ring when it is attached to one of this
    /// Recorder's, else into `misc` if the lock is free. Counts a drop when it does not fit.
    fn produce(&self, class: Class, length: usize, fill: impl FnOnce(&mut [u8])) -> bool {
        let binding = ATTACHED.try_with(LocalCell::get).ok().flatten();
        let stored = match binding {
            Some(binding) if binding.recorder == self.id => {
                // SAFETY: the binding exists only while this thread holds the ring's claim (`attach`).
                unsafe { self.producers[binding.ring].push(class, length, fill) }
            }
            _ => match self.misc_lock.try_lock() {
                // SAFETY: the lock makes this thread the misc ring's one producer.
                Ok(_guard) => unsafe { self.misc.push(class, length, fill) },
                // a panic while holding it reboots the board; on the host the ring is still whole
                Err(TryLockError::Poisoned(_guard)) => unsafe { self.misc.push(class, length, fill) },
                Err(TryLockError::WouldBlock) => false,
            },
        };
        if !stored {
            self.dropped[class as usize].fetch_add(1, Ordering::Relaxed);
        }
        stored
    }

    /// Queue one telemetry record of `length` that `fill` writes, waking the writer when it is `durable`.
    fn telemetry(&self, length: usize, durable: bool, fill: impl FnOnce(&mut [u8])) -> Result<(), Overflow> {
        if !self.produce(Class::Telemetry, length, fill) {
            return Err(Overflow);
        }
        if durable {
            (self.wake)();
        }
        Ok(())
    }

    /// Count one telemetry record refused before it reached a ring, and say so.
    fn refuse(&self) -> Result<(), Overflow> {
        self.dropped[Class::Telemetry as usize].fetch_add(1, Ordering::Relaxed);
        Err(Overflow)
    }

    /// Queue a stream's HEADER record: its column list, copied from the registry.
    pub(crate) fn push_header(&self, stream: StreamId, uptime_us: u64) -> Result<(), Overflow> {
        let Some(entry) = self.registry.stream(stream) else {
            return self.refuse();
        };
        let (file, columns) = (entry.file(), entry.columns());
        let fill = |slot: &mut [u8]| record::write_header(slot, file, entry.durable(), uptime_us, columns);
        self.telemetry(record::header_length(file, columns), entry.durable(), fill)
    }

    /// Queue a ROW record of `cells`. A row no flash chunk could hold (more than 255 cells, or longer than 4032
    /// bytes) is refused here, where its producer learns of it: the writer could never program it.
    pub(crate) fn push_row(&self, stream: StreamId, uptime_us: u64, cells: &[Cell]) -> Result<(), Overflow> {
        let Some(entry) = self.registry.stream(stream) else {
            return self.refuse();
        };
        let Some(length) = record::row_length(entry.file(), cells) else {
            return self.refuse();
        };
        let fill = |slot: &mut [u8]| record::write_row(slot, entry.file(), entry.durable(), uptime_us, cells);
        self.telemetry(length, entry.durable(), fill)
    }

    /// A best-effort recorder.log line, `<uptime_us> <descriptor> :: <message>` (plan B `Recorder.log`): stamped
    /// now, the descriptor cut to 255 bytes and the message to 1 KiB, queued as a LOG record.
    ///
    /// Returns true when queued; false when dropped (a full ring, or a busy `misc` ring), counted in
    /// `log.dropped`.
    pub fn log(&self, descriptor: &str, message: &str) -> bool {
        let uptime_us = (self.clock)();
        let (descriptor, message) = record::log_parts(descriptor, message);
        let fill = |slot: &mut [u8]| record::write_log(slot, uptime_us, descriptor, message);
        self.produce(Class::Log, record::log_length(descriptor, message), fill)
    }

    /// Append a session.csv row for a time set (plan B `Recorder.index_session`): this boot tied to the wall
    /// clock, stamped now, queued as a durable ROW to `session.csv` (a time set wakes the writer). The writer
    /// writes the `boot` and `anchor` rows itself, each after the HEADER record; a time set always comes after
    /// the boot row, so it needs no header of its own.
    ///
    /// Args:
    /// * `epoch_s` - the clock just set, in Unix seconds; before 2001 it dates nothing (empty utc, utc_offset).
    /// * `utc_offset` - the setter's offset from UTC in minutes, when it gave one.
    /// * `source` - what set the time (`cc-auto`, `dashboard`); a text cell.
    /// * `cc_position` - CC's own (latitude, longitude) in 1e-7 degrees, when it gave one.
    ///
    /// Errors: [`Overflow`] when the row cannot be queued: the time set still counts, the row is lost (plan B's
    /// rule for this file), and `tlm.dropped` counts it.
    pub fn index_session(
        &self,
        epoch_s: i64,
        utc_offset: Option<i32>,
        source: &str,
        cc_position: Option<(i32, i32)>,
    ) -> Result<(), Overflow> {
        let uptime_us = (self.clock)();
        let session_row = self.session_row(uptime_us, epoch_s, utc_offset, source, cc_position);
        let mut date = String::new();
        self.push_row(StreamId::SESSION_INDEX, uptime_us, &session_row.cells(&mut date))
    }

    /// A session.csv row of this boot.
    pub(crate) fn session_row<'a>(
        &'a self,
        uptime_us: u64,
        epoch_s: i64,
        utc_offset: Option<i32>,
        source: &'a str,
        cc_position: Option<(i32, i32)>,
    ) -> SessionRow<'a> {
        SessionRow {
            uptime_us,
            boot_id: self.identity.boot_id,
            session: self.session(),
            epoch: epoch_s,
            utc_offset,
            board: &self.identity.board,
            firmware: &self.identity.firmware,
            config_id: &self.identity.config_id,
            source,
            cc_position,
        }
    }

    /// The CC `log [ms]` command: the log lines buffered since the last call, keeping the window open
    /// `duration_ms` more (0 or less stops it).
    pub fn drain_log_tee(&self, duration_ms: i64) -> TeeBatch {
        self.log_tee.drain(self.now_ms(), duration_ms, self.session())
    }

    /// The CC `tlm [ms]` command: the telemetry lines (`@<routing>@<row>`) buffered since the last call, keeping
    /// the window open `duration_ms` more (0 or less stops it).
    pub fn drain_telemetry_tee(&self, duration_ms: i64) -> TeeBatch {
        self.telemetry_tee.drain(self.now_ms(), duration_ms, self.session())
    }

    /// The uptime in wrapping milliseconds, the tees' unit.
    fn now_ms(&self) -> u32 {
        ((self.clock)() / 1000) as u32
    }

    /// The recorder's health for `stats`: what the writer published last, and the drop counts.
    pub fn stats(&self) -> Stats {
        self.published.load([
            self.dropped[Class::Telemetry as usize].load(Ordering::Relaxed),
            self.dropped[Class::Log as usize].load(Ordering::Relaxed),
        ])
    }

    /// `writer_position`: the end of what the writer has programmed (block sequence, 8-aligned offset), for the
    /// ring_saver, which never reads beyond it. None until the writer's first step. Before this boot's first chunk
    /// it is offset 0 of the block this boot opens first, so every older block is readable whole.
    ///
    /// Any thread at any priority may call it: it never waits for the writer, even one it preempted mid-store.
    pub fn writer_position(&self) -> Option<Position> {
        self.writer_position.load()
    }

    /// Tell the recorder whether the stage is in flight: true from BOOSTING, false from DONE, and false when the
    /// warm start of a reboot in flight is refused (doc/specs/avionics-port.md "Ring geometry and block life
    /// cycle"). The writer applies a rise before each chunk it programs and a fall once everything queued before it
    /// is programmed, and a change closes its open block, so every record queued in flight lies in a block whose
    /// header carries the flag (newest wins gives flight blocks up last).
    ///
    /// Any thread may call it at any time, before the writer exists too: the Recorder holds the flag from
    /// [`Recorder::new`] on, and the writer takes it from here when it starts ([`crate::Writer::new`]).
    pub fn set_flight(&self, flight: bool) {
        self.flight.store(flight, Ordering::Release);
    }

    /// Whether the stage is in flight, as last told.
    pub(crate) fn flight(&self) -> bool {
        self.flight.load(Ordering::Acquire)
    }

    /// Whether a writer runs: until one does, [`crate::Telemetry::due`] is false, and records only queue.
    pub fn running(&self) -> bool {
        self.writer.load(Ordering::Acquire) == writer_state::RUNNING
    }

    /// Take the Recorder for its one writer; false when a writer has taken it before, running or ended.
    pub(crate) fn claim_writer(&self) -> bool {
        self.writer
            .compare_exchange(
                writer_state::NONE,
                writer_state::RUNNING,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok()
    }

    /// The writer ended: nothing drains the rings any more.
    pub(crate) fn end_writer(&self) {
        self.writer.store(writer_state::ENDED, Ordering::Release);
    }

    /// The streams.
    pub fn registry(&self) -> &Registry {
        &self.registry
    }

    /// The session prefix.
    pub(crate) fn session(&self) -> &str {
        self.identity.session.as_str()
    }

    /// The recorder config, with `stats_ms` as it is now.
    pub fn settings(&self) -> Settings {
        Settings {
            telemetry_ms: self.telemetry_ms,
            stats_ms: self.stats_ms.load(Ordering::Relaxed),
        }
    }

    /// The operator's `update recorder {"stats_ms": n}` (plan B `Recorder.update`): a new status-line period in
    /// ms, which the writer takes at its next step. Only a positive value that differs from the current one
    /// counts.
    ///
    /// Returns whether it changed, for the update's list of changed properties.
    pub fn set_stats_ms(&self, stats_ms: u32) -> bool {
        stats_ms > 0 && self.stats_ms.swap(stats_ms, Ordering::Relaxed) != stats_ms
    }

    /// The `inspect` snapshot: see [`Inspection`].
    pub fn inspect(&self) -> Inspection<'_> {
        Inspection {
            session: self.session(),
            stats_ms: self.stats_ms.load(Ordering::Relaxed),
            producer_bytes: self.producers.iter().map(|producer| producer.ring.capacity()).collect(),
            misc_bytes: self.misc.ring.capacity(),
        }
    }

    /// The producer ring `index`: the attachable ones, then `misc` (index `producers.len()`).
    pub(crate) fn source(&self, index: usize) -> &Producer {
        self.producers.get(index).unwrap_or(&self.misc)
    }

    /// How many rings the writer drains: the attachable ones and `misc`.
    pub(crate) fn sources(&self) -> usize {
        self.producers.len() + 1
    }
}

#[cfg(test)]
mod tests {
    use super::{AttachError, Buffers, Class, Identity, Overflow, Recorder, Settings};
    use crate::registry::{Registry, StreamId};
    use crate::telemetry::Telemetry;
    use flash_ring::header::Session;
    use recorder_wire::Cell;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::vec::Vec;

    /// A leaked buffer of `bytes`.
    fn buffer(bytes: usize) -> &'static mut [u8] {
        Vec::leak(vec![0; bytes])
    }

    /// A Recorder of `rings` producer rings of `bytes` each over `registry`, starting in flight when `flight` says
    /// so, at a fixed uptime, counting its wakes.
    fn recorder_with(registry: Registry, rings: usize, bytes: usize, flight: bool) -> (Recorder, Arc<AtomicU32>) {
        let buffers = Buffers {
            producers: (0..rings).map(|_| buffer(bytes)).collect(),
            misc: buffer(bytes),
            log_tee: buffer(1024),
            telemetry_tee: buffer(1024),
        };
        let identity = Identity {
            boot_id: Some(123),
            session: Session::new("000123").unwrap(),
            board: "taster".into(),
            firmware: "dev".into(),
            config_id: "3f2a91".into(),
        };
        let settings = Settings {
            telemetry_ms: 0,
            stats_ms: 1000,
        };
        let wakes = Arc::new(AtomicU32::new(0));
        let counted = wakes.clone();
        let recorder = Recorder::new(
            registry,
            identity,
            settings,
            flight,
            buffers,
            Box::new(|| 884_030),
            Box::new(move || {
                counted.fetch_add(1, Ordering::Relaxed);
            }),
        );
        (recorder, wakes)
    }

    /// A Recorder of `rings` producer rings of `bytes` each, with only the session index.
    fn recorder(rings: usize, bytes: usize) -> Recorder {
        recorder_with(Registry::new(), rings, bytes, false).0
    }

    #[test]
    fn a_ring_has_one_producer_thread_and_a_thread_one_ring() {
        let recorder = recorder(2, 256);
        let attached = recorder.attach(0).unwrap();
        assert_eq!(recorder.attach(1).err(), Some(AttachError::ThreadAttached));
        assert_eq!(recorder.attach(2).err(), Some(AttachError::NoSuchRing));
        std::thread::scope(|scope| {
            scope.spawn(|| {
                assert_eq!(recorder.attach(0).err(), Some(AttachError::Claimed));
                let other = recorder.attach(1).unwrap();
                assert!(recorder.log("i2c1", "into ring 1"));
                drop(other);
            });
        });
        assert!(recorder.log("control", "into ring 0"));
        drop(attached);
        assert!(recorder.log("main", "into misc, now unattached"));
        let pushed = |producer: &super::Producer| producer.pushed[Class::Log as usize].load(Ordering::Relaxed);
        assert_eq!(
            (
                pushed(&recorder.producers[0]),
                pushed(&recorder.producers[1]),
                pushed(&recorder.misc)
            ),
            (1, 1, 1)
        );
        let again = recorder.attach(1).unwrap();
        drop(again);
    }

    #[test]
    fn a_binding_names_its_own_recorder() {
        let first = recorder(1, 256);
        let second = recorder(1, 256);
        let _attached = first.attach(0).unwrap();
        assert!(second.log("x", "goes to the second's misc ring"));
        assert_eq!(second.misc.pushed[Class::Log as usize].load(Ordering::Relaxed), 1);
        assert_eq!(
            first.producers[0].pushed[Class::Log as usize].load(Ordering::Relaxed),
            0
        );
    }

    #[test]
    fn a_full_ring_drops_logs_and_raises_telemetry() {
        let recorder = recorder(1, 128);
        let _attached = recorder.attach(0).unwrap();
        for _ in 0..3 {
            assert!(recorder.log("a", "b"), "16 + 12 + 1 + 1 + 1 = 31: 32 bytes each");
        }
        assert!(!recorder.log("a", "b"), "a fourth would make head meet tail");
        assert_eq!(
            recorder.index_session(1_791_037_267, Some(-240), "cc-auto", None),
            Err(Overflow)
        );
        let stats = recorder.stats();
        assert_eq!((stats.log.dropped, stats.telemetry.dropped), (1, 1));
    }

    #[test]
    fn a_busy_misc_ring_drops_instead_of_waiting() {
        let recorder = recorder(0, 256);
        let held = recorder.misc_lock.lock().unwrap();
        std::thread::scope(|scope| {
            scope.spawn(|| assert!(!recorder.log("busy", "dropped")));
        });
        drop(held);
        assert!(recorder.log("free", "queued"));
        assert_eq!(recorder.stats().log.dropped, 1);
    }

    #[test]
    fn a_durable_record_wakes_the_writer_and_no_other_does() {
        let mut registry = Registry::new();
        let events = registry.add("flight_events.csv", &["event"], -1, true).unwrap();
        let imu = registry.add("imu.csv", &["ax"], -1, false).unwrap();
        let (recorder, wakes) = recorder_with(registry, 0, 4096, false);
        let mut imu = Telemetry::new(&recorder, imu).unwrap();
        imu.push(1, &[Cell::F32(1.0)]).unwrap();
        assert!(recorder.log("x", "y"));
        assert_eq!(
            wakes.load(Ordering::Relaxed),
            0,
            "rows of a plain stream and logs wait for the tick"
        );
        let mut events = Telemetry::new(&recorder, events).unwrap();
        events.push(2, &[Cell::Text("apogee")]).unwrap();
        assert_eq!(wakes.load(Ordering::Relaxed), 2, "its header, then its row");
        recorder.index_session(0, None, "cc-auto", None).unwrap();
        assert_eq!(wakes.load(Ordering::Relaxed), 3, "a session.csv row is durable");
        // negative: a refused durable row wakes nobody
        let text = "x".repeat(255);
        assert_eq!(events.push(3, &vec![Cell::Text(&text); 16]), Err(Overflow));
        assert_eq!(wakes.load(Ordering::Relaxed), 3);
        assert_eq!(recorder.stats().telemetry.dropped, 1);
        assert!(recorder.registry().stream(StreamId::SESSION_INDEX).unwrap().durable());
    }

    #[test]
    fn the_flight_flag_is_whatever_was_told_last() {
        let recorder = recorder(0, 64);
        assert!(!recorder.flight(), "a boot with no crumb in flight starts on the pad");
        recorder.set_flight(true);
        assert!(recorder.flight());
        recorder.set_flight(false);
        assert!(!recorder.flight());
        // a reboot in flight starts in flight, and a refused warm start, told before any writer exists, holds
        let (rebooted, _) = recorder_with(Registry::new(), 0, 64, true);
        assert!(rebooted.flight(), "the crumb named BOOSTING..LANDING");
        rebooted.set_flight(false);
        assert!(!rebooted.flight());
    }

    #[test]
    fn stats_ms_changes_only_to_a_new_positive_period() {
        let recorder = recorder(2, 256);
        assert_eq!(recorder.settings().stats_ms, 1000);
        assert!(recorder.set_stats_ms(250));
        assert_eq!(recorder.settings().stats_ms, 250);
        // negative: plan B's rule, nothing changes for 0 or the same value
        assert!(!recorder.set_stats_ms(0));
        assert!(!recorder.set_stats_ms(250));
        assert_eq!(recorder.settings().stats_ms, 250);
        assert_eq!(recorder.settings().telemetry_ms, 0, "fixed for the boot");
        let inspection = recorder.inspect();
        assert_eq!(
            (
                inspection.session,
                inspection.stats_ms,
                inspection.producer_bytes,
                inspection.misc_bytes
            ),
            ("000123", 250, vec![256, 256], 256)
        );
    }
}
