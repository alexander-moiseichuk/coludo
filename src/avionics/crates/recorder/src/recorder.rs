//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The Recorder (src/glider recorder.py `Recorder`; doc/specs/avionics-port.md "Producer side"): the state every
//! thread shares, built once in bring-up before any thread spawns and leaked to `&'static` on the board.
//!
//! Producers never format text and never take a lock on their path:
//! * a producing thread [`Recorder::attach`]es itself to its own PSRAM ring, once (control, i2c0, i2c1, gnss,
//!   supervisor, executor). From then on its [`crate::Telemetry`] rows and [`Recorder::log`] lines go into that
//!   ring alone: a single-producer, single-consumer byte ring, with the flash writer the one consumer;
//! * any other thread (an unregistered one, IDF's vprintf hook) shares the `misc` ring, behind a `try_lock`:
//!   contention drops the record and counts it, it never waits.
//!
//! A thread finds its ring through a thread-local binding set by `attach`, so a log line reaches the right ring
//! from anywhere (the `log` crate bridge has no handle to pass). The binding names the Recorder by a process-
//! unique id, so a thread attached to one Recorder writes into the `misc` ring of another.
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
use recorder_wire::{Cell, SessionRow};

use crate::byte_ring::ByteRing;
use crate::published::{PublishedPosition, PublishedStats, Stats};
use crate::record;
use crate::registry::{Registry, StreamId};
use crate::tee::{Tee, TeeBatch};
use crate::writer::{FRAME_BYTES, wrapped_bytes};

/// A record class: the status line and `stats` count them apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Class {
    /// HEADER, ROW and LINE records: evidence.
    Telemetry = 0,
    /// LOG records: best effort.
    Log = 1,
}

/// A telemetry record that could not be queued: its producer's ring is full (or, for the `misc` ring, busy), or
/// the row is larger than a ring record may be, or its line could be longer than a frame holds. Counted in
/// `tlm.dropped`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Overflow;

impl core::fmt::Display for Overflow {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str("telemetry dropped: no room in the recorder ring")
    }
}

impl std::error::Error for Overflow {}

/// Who this boot is, for the session.csv rows and the BOOT frame payload.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    /// The NVS boot id; None when nothing counted the boot.
    pub boot_id: Option<u32>,
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
    /// The log tee's lines (1024 x 256 B in the plan).
    pub log_tee: &'static mut [u8],
    /// The telemetry tee's lines, likewise.
    pub telemetry_tee: &'static mut [u8],
}

/// The uptime in microseconds (`esp_timer_get_time` on the board): what a log line is stamped with.
pub type Clock = Box<dyn Fn() -> u64 + Send + Sync>;

/// One producer ring and its counters.
pub(crate) struct Producer {
    /// The records, the flash writer its consumer.
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
    /// The boot, for the session.csv rows and the BOOT payload.
    identity: Identity,
    /// `recorder.telemetry_ms`, fixed for the boot.
    telemetry_ms: u32,
    /// `recorder.stats_ms`, which the operator may change: the writer reads it every step.
    stats_ms: AtomicU32,
    /// The uptime clock.
    clock: Clock,
    /// One ring per producing thread.
    pub(crate) producers: Vec<Producer>,
    /// The ring unregistered threads share.
    pub(crate) misc: Producer,
    /// Guards the `misc` ring's producer side.
    misc_lock: Mutex<()>,
    /// The CC copy of the log lines.
    pub(crate) log_tee: Tee,
    /// The CC copy of the telemetry lines.
    pub(crate) telemetry_tee: Tee,
    /// Records refused per [`Class`]: by a full or busy ring, or by the writer as never programmable.
    pub(crate) dropped: [AtomicU32; 2],
    /// The writer's figures.
    pub(crate) published: PublishedStats,
    /// The end of what is programmed, for the mover.
    pub(crate) programmed: PublishedPosition,
    /// Whether GC pauses its erases (flight, BOOSTING through LANDING).
    erases_paused: AtomicBool,
    /// Whether a [`crate::Writer`] has taken this Recorder, and whether it still runs (`writer_state`).
    writer: AtomicU32,
}

impl Recorder {
    /// The Recorder over `buffers`, with its streams, boot identity, settings and clock. Nothing runs until a
    /// [`crate::Writer`] takes it; records queue in the rings meanwhile.
    pub fn new(registry: Registry, identity: Identity, settings: Settings, buffers: Buffers, clock: Clock) -> Recorder {
        Recorder {
            id: NEXT_RECORDER.fetch_add(1, Ordering::Relaxed),
            registry,
            identity,
            telemetry_ms: settings.telemetry_ms,
            stats_ms: AtomicU32::new(settings.stats_ms),
            clock,
            producers: buffers.producers.into_iter().map(Producer::new).collect(),
            misc: Producer::new(buffers.misc),
            misc_lock: Mutex::new(()),
            log_tee: Tee::new(buffers.log_tee),
            telemetry_tee: Tee::new(buffers.telemetry_tee),
            dropped: [AtomicU32::new(0), AtomicU32::new(0)],
            published: PublishedStats::default(),
            programmed: PublishedPosition::new(),
            erases_paused: AtomicBool::new(false),
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

    /// Queue a stream's HEADER record.
    pub(crate) fn push_header(&self, stream: StreamId, durable: bool, uptime_us: u64) -> Result<(), Overflow> {
        let fill = |slot: &mut [u8]| record::write_header(slot, stream.0, durable, uptime_us);
        self.produce(Class::Telemetry, record::HEADER_BYTES, fill)
            .then_some(())
            .ok_or(Overflow)
    }

    /// Queue a ROW record. A row whose line could pass a frame once printed and wrapped is refused here, where
    /// its producer learns of it: the writer could never program it.
    pub(crate) fn push_row(
        &self,
        stream: StreamId,
        durable: bool,
        uptime_us: u64,
        cells: &[Cell],
    ) -> Result<(), Overflow> {
        let routing = self.registry.stream(stream).map(|stream| stream.routing());
        let fits = wrapped_bytes(routing, record::row_width(cells)) <= FRAME_BYTES;
        let Some(length) = record::row_length(cells).filter(|_| fits) else {
            self.dropped[Class::Telemetry as usize].fetch_add(1, Ordering::Relaxed);
            return Err(Overflow);
        };
        let fill = |slot: &mut [u8]| record::write_row(slot, stream.0, durable, uptime_us, cells);
        self.produce(Class::Telemetry, length, fill)
            .then_some(())
            .ok_or(Overflow)
    }

    /// A best-effort recorder.log line, `<uptime_us> <descriptor> :: <message>` (plan B `Recorder.log`): stamped
    /// now, the descriptor cut to 255 bytes and the message to 1 KiB.
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
    /// clock, stamped now. The writer writes the `boot` and `anchor` rows, each after the header, itself; a time
    /// set always comes after the boot row, so it needs no header of its own.
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
        let mut text = Vec::new();
        self.session_row(uptime_us, epoch_s, utc_offset, source, cc_position)
            .append_to(&mut text);
        let durable = self.index_durable();
        let fill = |slot: &mut [u8]| record::write_line(slot, StreamId::SESSION_INDEX.0, durable, uptime_us, &text);
        self.produce(Class::Telemetry, record::line_length(&text), fill)
            .then_some(())
            .ok_or(Overflow)
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
            session: self.registry.session(),
            epoch: epoch_s,
            utc_offset,
            board: &self.identity.board,
            firmware: &self.identity.firmware,
            config_id: &self.identity.config_id,
            source,
            cc_position,
        }
    }

    /// Whether the session index is a durable stream (it is, from [`Registry::new`]).
    fn index_durable(&self) -> bool {
        self.registry
            .stream(StreamId::SESSION_INDEX)
            .is_some_and(|stream| stream.durable())
    }

    /// The BOOT frame payload every block this boot opens starts with (doc/specs/avionics-port.md "Flash frame
    /// format"): `boot=<id>;session=<prefix>;board=<id>;firmware=<v>;config_id=<id>;reset=<cause>;card=<vsn|none>`.
    /// A boot id nothing counted is empty, the card's FAT volume serial is 8 lowercase hex digits, and the text
    /// fields follow session.csv's cell rule, so a `;` can never split one.
    pub fn boot_payload(&self, reset_cause: &str, card: Option<u32>) -> Vec<u8> {
        let boot = self.identity.boot_id.map(|id| id.to_string()).unwrap_or_default();
        let card = card.map_or_else(|| String::from("none"), |serial| std::format!("{serial:08x}"));
        std::format!(
            "boot={boot};session={};board={};firmware={};config_id={};reset={};card={card}",
            self.registry.session(),
            recorder_wire::cell(&self.identity.board),
            recorder_wire::cell(&self.identity.firmware),
            recorder_wire::cell(&self.identity.config_id),
            recorder_wire::cell(reset_cause),
        )
        .into_bytes()
    }

    /// The CC `log [ms]` command: the log lines buffered since the last call, keeping the window open
    /// `duration_ms` more (0 or less stops it).
    pub fn drain_log_tee(&self, duration_ms: i64) -> TeeBatch {
        self.log_tee.drain(self.now_ms(), duration_ms)
    }

    /// The CC `tlm [ms]` command: the telemetry lines (`@<routing>@<row>`) buffered since the last call, keeping
    /// the window open `duration_ms` more (0 or less stops it).
    pub fn drain_telemetry_tee(&self, duration_ms: i64) -> TeeBatch {
        self.telemetry_tee.drain(self.now_ms(), duration_ms)
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

    /// The end of what the writer has programmed (block sequence, 8-aligned offset), for the SD mover: never
    /// read beyond it. None until the writer's first step, and on an unformatted ring until its first frame.
    /// Before this boot's first frame it is offset 0 of the block this boot opens, so every older block is
    /// readable whole.
    ///
    /// Any thread at any priority may call it: it never waits for the writer, even one it preempted mid-store.
    pub fn programmed(&self) -> Option<Position> {
        self.programmed.load()
    }

    /// Pause GC erases for flight (BOOSTING through LANDING) or resume them; the writer applies it next step.
    pub fn pause_erases(&self, paused: bool) {
        self.erases_paused.store(paused, Ordering::Relaxed);
    }

    /// Whether erases are paused.
    pub(crate) fn erases_paused(&self) -> bool {
        self.erases_paused.load(Ordering::Relaxed)
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
            session: self.registry.session(),
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
    use super::{AttachError, Buffers, Class, Identity, Recorder, Settings};
    use crate::registry::Registry;
    use std::sync::atomic::Ordering;
    use std::vec::Vec;

    /// A leaked buffer of `bytes`.
    fn buffer(bytes: usize) -> &'static mut [u8] {
        Vec::leak(vec![0; bytes])
    }

    /// A Recorder of `rings` producer rings of `bytes` each, at a fixed uptime.
    fn recorder(rings: usize, bytes: usize) -> Recorder {
        let buffers = Buffers {
            producers: (0..rings).map(|_| buffer(bytes)).collect(),
            misc: buffer(bytes),
            log_tee: buffer(1024),
            telemetry_tee: buffer(1024),
        };
        let identity = Identity {
            boot_id: Some(123),
            board: "taster".into(),
            firmware: "dev".into(),
            config_id: "3f2a91".into(),
        };
        let settings = Settings {
            telemetry_ms: 0,
            stats_ms: 1000,
        };
        Recorder::new(
            Registry::new("000123").unwrap(),
            identity,
            settings,
            buffers,
            Box::new(|| 884_030),
        )
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
        let recorder = recorder(1, 64);
        let _attached = recorder.attach(0).unwrap();
        assert!(recorder.log("a", "b")); // 16 + 1 + 1 + 1 = 19 -> 20 bytes
        assert!(recorder.log("a", "b"));
        assert!(recorder.log("a", "b"), "60 of 64 bytes used");
        assert!(!recorder.log("a", "b"), "a fourth would make head meet tail");
        assert!(
            recorder
                .index_session(1_791_037_267, Some(-240), "cc-auto", None)
                .is_err()
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

    #[test]
    fn the_boot_payload_names_the_boot() {
        let recorder = recorder(0, 64);
        assert_eq!(
            recorder.boot_payload("poweron", None),
            b"boot=123;session=000123;board=taster;firmware=dev;config_id=3f2a91;reset=poweron;card=none"
        );
        assert!(
            recorder
                .boot_payload("sw;x", Some(0x1234_ABCD))
                .ends_with(b";reset=;card=1234abcd"),
            "a ';' empties its cell"
        );
    }
}
