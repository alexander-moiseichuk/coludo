//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder on the board (doc/specs/avionics-port.md "Storage design", and "Bring-up order" step 3): the glue
//! between crates/recorder, crates/flash_ring and the ESP32-P4. Power can drop at any moment, so every record goes
//! to flash as soon as it can; the SD copy (the ring_saver) comes with M4.
//!
//! ```text
//! producers (any core) --binary records--> PSRAM rings --> ring_writer (core 0, prio 7) --> `log` partition, 24 MiB
//!   supervisor, executor, eraser,          + misc ring          |  one chunk per ~10 ms tick       ^
//!   everyone else (log bridge, IDF)                             +--> CC tees (`log`, `tlm`)         | one 4 KiB erase
//!                                                                         ring_eraser (core 0, prio 3)
//! ```
//!
//! The bring-up runs in two steps, so no line of the boot is lost and boot never waits on flash:
//! 1. [`open`], on main right after the board config: the stream registry, the producer rings and the CC tees in
//!    PSRAM, and the Recorder, published to every thread. From then on every log line and telemetry row queues in
//!    PSRAM, whoever makes it.
//! 2. [`start`], on main once the SD card's state is known: spawns the ring_writer. Its bring-up routine (spec "The
//!    scan, the positions and the session-start record") reads the ring positions from the `cfg` NVS, scans the 6144
//!    block headers, writes the session-start record, spawns the ring_eraser, and only then starts programming; boot
//!    goes on meanwhile, and the producers queue.
//!
//! What lives in the `cfg` NVS, namespace `recorder`, all of it hints the scan can do without:
//! * `writer`: the writer position, written about once a second by the ring_eraser, never by the writer itself (a
//!   u64: block sequence << 32 | offset); it only keeps the block numbering from going back after the newest blocks
//!   were erased;
//! * `saver`: the saver position, written by the ring_saver (M4); read here, and never trusted yet;
//! * `start`: the session-start record, once per boot (JSON): boot id, session, config id, reset cause, whether a
//!   card is mounted, and the positions this boot started from.
//!
//! Nothing here runs on core 1, the flight core, and no producer takes a lock (crates/recorder). Every big buffer is
//! in PSRAM; what stays in internal RAM is the three thread stacks (writer 8 KiB, eraser 6 KiB, supervisor 6 KiB in
//! supervisor.rs) and the crates' small working buffers (about 18 KiB: the writer's chunk, the ring's chunk and its
//! 4 KiB erased-check buffer, and the eraser's per-slot failure counts). The 48 KiB block table is in PSRAM: the heap
//! puts any allocation of 16 KiB or more there (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL).

use std::ffi::CStr;
use std::num::NonZeroU32;
use std::ptr;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};
use std::time::{Duration, Instant};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::task;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::{EspCustomNvsPartition, EspNvs, NvsCustom};
use esp_idf_svc::sys;
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use flash_ring::header::Session;
use flash_ring::{BLOCK_BYTES, BlockTable, Eraser, Erasure, Flash, Geometry, Hints, Loss, Position, Ring};
use recorder::{Buffers, Identity, MERGE_HORIZON_US, Recorder, Registry, Settings, StreamId, TICK_US, Writer};
use serde_json::{Value, json};

use crate::{board, clock, config, supervisor};

/// The producer rings a thread attaches to, by the index `Recorder::attach` takes (spec "Producer side": one ring per
/// producing thread). The flight threads (control, i2c0, i2c1, gnss) join as they are ported. Every other thread
/// shares the `misc` ring.
#[derive(Clone, Copy)]
pub enum Producer {
    /// The supervisor: health.csv at 1 Hz.
    Supervisor = 0,
    /// The CC executor: the session.csv row of a clock set, and the log lines of every command.
    Executor = 1,
    /// The ring_eraser: the log line of every block newest wins erases unsaved, which must not lose a race for the
    /// shared `misc` ring.
    Eraser = 2,
}

/// Each producer ring's PSRAM in bytes, by [`Producer`] index: the spec's sizes for the supervisor (256 KiB) and the
/// executor (128 KiB), and 16 KiB for the eraser's loss lines (~100 B each, at most a few a second).
const PRODUCER_BYTES: [usize; 3] = [256 << 10, 128 << 10, 16 << 10];

/// The `misc` ring the unregistered threads share (the link, the console, httpd, IDF's log hook), in bytes.
const MISC_BYTES: usize = 256 << 10;

/// Each CC tee's records in bytes (spec "CC tees"): a full one prints as ~600 KB of text, under the hub's 2 MiB line.
const TEE_BYTES: usize = 256 << 10;

/// The ring_writer's priority on core 0: above the video SD writer (6), below the encoder (10) (spec "The
/// ring_writer", Placement).
const WRITER_PRIORITY: u8 = 7;

/// The ring_writer's priority during its bring-up routine: the main task's (1), so the scan (~0.15-0.3 s of flash
/// reads) shares core 0 with the rest of boot instead of holding it up (spec "Bring-up": "the rest of boot goes
/// on"). It rises to [`WRITER_PRIORITY`] before the first step.
const BRING_UP_PRIORITY: u8 = 1;

/// The ring_writer's stack: the scan, the NVS writes, the flash calls and the status line's text; every buffer it
/// fills is on the heap.
const WRITER_STACK: usize = 8 << 10;

/// The ring_eraser's priority on core 0: the lowest of the ring tasks (spec "Threads, cores, priorities").
const ERASER_PRIORITY: u8 = 3;

/// The ring_eraser's stack: one erase call, the writer position hint's NVS write (the writer did both its NVS writes
/// and the scan in 8 KiB) and, rarely, a loss line; every text it builds is on the heap.
const ERASER_STACK: usize = 6 << 10;

/// How long the eraser rests when nothing is due: the reserve (61 blocks) lasts ~15 s at the flight rate, so a
/// tenth of a second costs nothing, and its scan of the 6144-slot table stays well under 1 % of core 0.
const ERASER_IDLE: Duration = Duration::from_millis(100);

/// The least the eraser rests after an erase: one tick. It rests as long as the erase took, at least this, so
/// back-to-back erases (a first boot formatting an unformatted ring: 6144 x ~15 ms) hold core 0 at most half the time
/// and main, the link and the idle task (which the task watchdog watches) go on; that still frees ~30 blocks a
/// second, where the flight rate takes ~4.
const ERASER_PAUSE: Duration = Duration::from_millis(1);

/// How often the writer position is written to the `cfg` NVS when it moved: about once a second (spec "Positions").
const HINT_PERIOD_US: u64 = 1_000_000;

/// The longest a CC drain waits for the writer to catch up with its request ([`catch_up`]), ms: a writer that
/// cannot program (no erased block yet) must not hold the executor.
const CATCH_UP_LIMIT_MS: u32 = 100;

/// How often a waiting drain looks again, ms.
const CATCH_UP_POLL: Duration = Duration::from_millis(2);

/// How many times the bring-up scan is tried before the ring is given up for this boot: a read error is a flash
/// call failing, which a second try usually passes.
const SCAN_ATTEMPTS: u32 = 3;

/// The `cfg` NVS namespace of the recorder's records.
const NAMESPACE: &str = "recorder";

/// The key of the session-start record.
const START_KEY: &str = "start";

/// The key of the writer position hint.
const WRITER_KEY: &str = "writer";

/// The key of the saver position hint (the ring_saver writes it, M4).
const SAVER_KEY: &str = "saver";

/// `recorder.telemetry_ms` when the config sets none: plan B's code fallback (recorder.py `_DEFAULT_TELEMETRY_MS`,
/// 50 Hz); the shipped config sets 0, every row.
const DEFAULT_TELEMETRY_MS: u32 = 20;

/// `recorder.stats_ms` when the config sets none: plan B's status line once a second.
const DEFAULT_STATS_MS: u32 = 1000;

/// The recorder of this boot, as the threads share it.
struct Recording {
    /// The shared state: the rings, the tees, the counters.
    recorder: &'static Recorder,
    /// The supervisor's health.csv stream.
    health: StreamId,
    /// This boot's NVS count; None when uncounted (block headers then carry 0).
    boot_id: Option<u32>,
    /// This boot's file prefix, which every block header carries.
    session: Session,
    /// The running config's id, for the session-start record.
    config_id: String,
}

/// The recorder once [`open`] built it.
static RECORDING: OnceLock<Recording> = OnceLock::new();

/// The flash ring once the bring-up scan has run: what `GET /ring`, `GET /ring/image` and the `ring` command read.
pub struct RingView {
    /// The `log` partition.
    pub partition: Partition,
    /// The block table the writer and the eraser share.
    pub table: &'static BlockTable,
    /// The sequence of the first block this boot opens: every older block was in flash at boot.
    pub first_sequence: u32,
    /// The scan found no block of any layout: the eraser formats the foreign slots ahead of the writer.
    pub unformatted: bool,
}

/// The flash ring, set once by the writer's bring-up.
static RING: OnceLock<RingView> = OnceLock::new();

/// The ring_writer's task, for a producer's wake; null until the writer thread runs.
static WRITER_TASK: AtomicPtr<sys::tskTaskControlBlock> = AtomicPtr::new(ptr::null_mut());

/// Every record queued before this uptime (wrapping ms) has been through a completed writer step, and so is in flash
/// and in its open tee: the last step's start less the merge horizon a log line waits (a row goes sooner).
static TEED_MS: AtomicU32 = AtomicU32::new(0);

/// The running recorder; None before [`open`], or when PSRAM would not hold its rings.
pub fn get() -> Option<&'static Recorder> {
    RECORDING.get().map(|recording| recording.recorder)
}

/// The health.csv stream; None without a recorder.
pub fn health_stream() -> Option<StreamId> {
    RECORDING.get().map(|recording| recording.health)
}

/// The flash ring; None until the writer's bring-up scan has run (or when it found no `log` partition).
pub fn ring() -> Option<&'static RingView> {
    RING.get()
}

/// This boot's file prefix (spec "Row, log and index text", Session prefix; plan B `Recorder.session()`): the board
/// config's `recorder.session` when it is a valid label, else the boot id as `%06u`, else
/// `YYYYMMDD_HHMMSS_<6 random digits>` from the clock (2000-01-01 plus the uptime while it is unset).
///
/// Args:
/// * `document` - the board config this boot runs.
/// * `boot_id` - this boot's NVS count; None when uncounted.
///
/// Returns the prefix: it names the SD directory, every file of the boot, the video directory and whoami's session.
pub fn session_prefix(document: &Value, boot_id: Option<i32>) -> String {
    let label = recorder_wire::session_label(document["recorder"]["session"].as_str().unwrap_or_default());
    let boot_id = boot_id.and_then(|boot_id| u32::try_from(boot_id).ok());
    // SAFETY: a plain read of the hardware random number generator.
    let random = unsafe { sys::esp_random() };
    recorder_wire::session_prefix(&label, boot_id, clock::epoch(), random)
}

/// Builds the recorder and publishes it ([`get`]): the stream registry, the producer rings and the CC tees in PSRAM,
/// and the Recorder itself. From here on every log line and telemetry row queues in PSRAM, to be programmed once
/// [`start`] has the ring_writer running. Registers the recorder's commands (`ring`, `recorder update`).
///
/// Args:
/// * `document` - the board config this boot runs: `recorder.telemetry_ms`, `recorder.stats_ms`,
///   `recorder.session` (only to log a label that was ignored) and the board id.
/// * `boot_id` - this boot's NVS count; None when uncounted.
/// * `session` - this boot's file prefix ([`session_prefix`]).
/// * `config_id` - the running config's id, for session.csv and the session-start record.
///
/// Errors: PSRAM would not hold a ring, the prefix is not a session label, the registry refused a
/// stream, or the recorder opened before. The board runs on without recording then.
pub fn open(document: &Value, boot_id: Option<i32>, session: &str, config_id: &str) -> anyhow::Result<()> {
    let session = Session::new(session).ok_or_else(|| anyhow::anyhow!("{session:?} is not a session label"))?;
    let mut registry = Registry::new();
    let health = registry
        .add(supervisor::HEALTH_FILE, &supervisor::HEALTH_FIELDS, 0, false)
        .map_err(|error| anyhow::anyhow!("{}: {error:?}", supervisor::HEALTH_FILE))?;
    let buffers = Buffers {
        producers: PRODUCER_BYTES
            .iter()
            .map(|&bytes| psram(bytes))
            .collect::<anyhow::Result<_>>()?,
        misc: psram(MISC_BYTES)?,
        log_tee: psram(TEE_BYTES)?,
        telemetry_tee: psram(TEE_BYTES)?,
    };
    let boot_id = boot_id.and_then(|boot_id| u32::try_from(boot_id).ok());
    let identity = Identity {
        boot_id,
        session,
        board: board_config::board_id(document),
        firmware: board::FIRMWARE_VERSION.into(),
        config_id: config_id.into(),
    };
    // the crumb that would start the writer in flight comes with the warm start (M8): every boot starts on the pad
    let flight = false;
    let recorder = Recorder::new(
        registry,
        identity,
        settings(document),
        flight,
        buffers,
        Box::new(uptime_us),
        Box::new(wake),
    );
    let recording = Recording {
        recorder: Box::leak(Box::new(recorder)),
        health,
        boot_id,
        session,
        config_id: config_id.into(),
    };
    RECORDING
        .set(recording)
        .map_err(|_| anyhow::anyhow!("the recorder opened twice"))?;
    let recorder = get().ok_or_else(|| anyhow::anyhow!("the recorder did not publish"))?;
    let configured = document["recorder"]["session"].as_str().unwrap_or_default();
    if let recorder_wire::SessionLabel::Ignored(label) = recorder_wire::session_label(configured) {
        log::warn!(
            "Recorder :: recorder.session {label:?} ignored (letters, digits and '-' only, one letter at least, 32 at most)"
        );
    }
    register_commands(recorder);
    Ok(())
}

/// Spawns the ring_writer (core 0, priority 7). Its bring-up routine, run at the main task's priority, scans the `log`
/// partition, writes the session-start record and spawns the ring_eraser; then the writer steps forever. Boot goes
/// on meanwhile.
///
/// Args:
/// * `settings` - the `cfg` partition: the ring positions and the session-start record live there.
/// * `card` - whether a microSD card is mounted; the session-start record says so (M4 adds its volume serial,
///   which tells the saver whether the previous boot's saver position is this card's).
///
/// Errors: the recorder did not open, or the thread would not spawn.
pub fn start(settings: EspCustomNvsPartition, card: bool) -> anyhow::Result<()> {
    let recording = RECORDING
        .get()
        .ok_or_else(|| anyhow::anyhow!("the recorder did not open"))?;
    spawn(c"ring_writer", WRITER_STACK, BRING_UP_PRIORITY, move || {
        write(recording, settings, card)
    })
}

/// The ring_writer thread: the bring-up routine, then [`Writer::step`] on every ~10 ms tick or durable record's
/// wake, forever. A ring that cannot come up (no `log` partition, a scan that keeps failing) parks the thread with a
/// log line: the records then stay in PSRAM until their rings fill, and drop counted.
fn write(recording: &'static Recording, settings: EspCustomNvsPartition, card: bool) {
    if let Some(current) = task::current() {
        WRITER_TASK.store(current, Ordering::Release);
    }
    let recorder = recording.recorder;
    let store = RecorderStore::open(settings)
        .inspect_err(|error| log::warn!("ring :: the cfg NVS would not open, no positions kept: {error}"))
        .ok();
    let hints = store.as_ref().map_or_else(Hints::default, RecorderStore::hints);
    let Some(partition) = Partition::find() else {
        log::error!("ring :: no `log` partition in the partition table: nothing reaches flash this boot");
        park();
    };
    let started_us = uptime_us();
    let Some(scan) = (1..=SCAN_ATTEMPTS).find_map(|attempt| {
        flash_ring::scan(&partition, Geometry::BOARD, hints)
            .inspect_err(|error| log::warn!("ring :: bring-up scan {attempt}/{SCAN_ATTEMPTS} failed: {error:?}"))
            .ok()
    }) else {
        log::error!("ring :: the bring-up scan keeps failing: nothing reaches flash this boot");
        park();
    };
    let table: &'static BlockTable = Box::leak(Box::new(scan.table));
    let figures = table.figures(None);
    log::info!(
        "ring :: scanned {} blocks in {} ms: {} erased, {} closed, {} of another version, {} foreign; this boot opens \
         block {}{}",
        Geometry::BOARD.blocks(),
        uptime_us().saturating_sub(started_us) / 1000,
        figures.erased_ahead,
        figures.unsaved,
        figures.other_version,
        figures.foreign,
        scan.next_sequence,
        if scan.unformatted {
            " (unformatted: the eraser formats it first)"
        } else {
            ""
        },
    );
    // spec "Bring-up", step 6: what a fast start needs, written once per boot
    let first = Position {
        block_sequence: scan.next_sequence,
        offset: 0,
    };
    let record = json!({
        "boot": recording.boot_id,
        "session": recording.session.as_str(),
        "config_id": recording.config_id,
        "reset_cause": board::reset_reason(),
        "card": card,
        "writer": [first.block_sequence, first.offset],
        "saver": pair(table.saver_start(hints.saver)),
    });
    if let Some(store) = &store
        && let Err(error) = store.record_start(&record)
    {
        log::warn!("ring :: the session-start record was not written: {error}");
    }
    let _ = RING.set(RingView {
        partition,
        table,
        first_sequence: scan.next_sequence,
        unformatted: scan.unformatted,
    });
    // the store goes to the eraser, which keeps the writer position hint from here on: this thread never touches
    // the NVS again
    start_eraser(partition, table, recorder, store);
    let ring = match Ring::new(
        partition,
        table,
        scan.next_sequence,
        recording.boot_id.unwrap_or(0),
        recording.session,
        false, // the writer takes the Recorder's flight flag (Writer::new)
    ) {
        Ok(ring) => ring,
        Err(error) => {
            log::error!("ring :: the writer would not open the ring: {error:?}");
            park();
        }
    };
    let Some(mut writer) = Writer::new(recorder, ring) else {
        log::error!("ring :: another writer took the recorder");
        park();
    };
    // SAFETY: sets this task's own priority (a null handle is the calling task).
    unsafe { sys::vTaskPrioritySet(ptr::null_mut(), WRITER_PRIORITY.into()) };
    let tick = TickType::new_millis(TICK_US / 1000).ticks();
    loop {
        // the tick, or sooner: a producer that queued a durable record notifies this task (`wake`)
        task::wait_notification(tick);
        let now_us = uptime_us();
        writer.step(now_us, clock::epoch());
        TEED_MS.store(
            (now_us.saturating_sub(MERGE_HORIZON_US) / 1000) as u32,
            Ordering::Release,
        );
    }
}

/// Waits until the ring_writer has programmed, and so teed, every record queued before this call, for at most
/// [`CATCH_UP_LIMIT_MS`]; at once when no writer runs. The CC `log` and `tlm` drains call it first.
///
/// A log line reaches its tee only once the writer programs it, at least the merge horizon (20 ms) after it was
/// logged, where plan B teed it at once. Without the wait a drain that closes the window (`log 0` right after the
/// command whose lines the operator wants) would miss the last ~30 ms of lines, and the writer would then find the
/// window closed and drop them: the contract's `probe mission` then `log 0` would come back empty.
pub fn catch_up() {
    if !get().is_some_and(Recorder::running) {
        return;
    }
    let asked_ms = uptime_ms();
    // strictly past the asking millisecond: a line logged earlier within it is then behind the horizon too
    while (TEED_MS.load(Ordering::Acquire).wrapping_sub(asked_ms) as i32) <= 0
        && uptime_ms().wrapping_sub(asked_ms) < CATCH_UP_LIMIT_MS
    {
        std::thread::sleep(CATCH_UP_POLL);
    }
}

/// Spawns the ring_eraser (core 0, priority 3), the only task that erases the `log` partition, and the keeper of the
/// writer position hint. A spawn that fails is logged: the writer then fills the erased slots there are, and holds
/// its records in PSRAM after that; no hint is written then, and none is needed, since only erasing the newest blocks
/// could make the numbering go back.
///
/// Args:
/// * `partition` - the `log` partition.
/// * `table` - the block table the writer shares.
/// * `recorder` - for the loss lines and the writer position.
/// * `store` - the `cfg` NVS records; None when they would not open (no hint is kept then).
fn start_eraser(
    partition: Partition,
    table: &'static BlockTable,
    recorder: &'static Recorder,
    store: Option<RecorderStore>,
) {
    // no saver hint is trusted: the spec trusts one only with the card it was saved to (or none), and naming the
    // card is the saver's part (M4); until then the blocks behind it go only under newest wins
    let eraser = match Eraser::new(partition, table, None) {
        Ok(eraser) => eraser,
        Err(error) => {
            log::error!("ring :: the eraser would not open the ring: {error:?}");
            return;
        }
    };
    if let Err(error) = spawn(c"ring_eraser", ERASER_STACK, ERASER_PRIORITY, move || {
        erase(eraser, recorder, store)
    }) {
        log::error!("ring :: the eraser would not start: {error}");
    }
}

/// The ring_eraser thread: one [`Eraser::erase_step`] after another, forever. M3 has no saver, so the steps erase
/// foreign slots and, once no more than ~1 % of the ring is free, the oldest blocks unsaved (newest wins, pad blocks
/// before flight blocks), each a loss logged into recorder.log.
///
/// Between steps it writes the writer position hint to the `cfg` NVS, about once a second ([`Hint`]). The hint is
/// kept here and not on the ring_writer: IDF serialises every NVS call, on every partition, behind one global lock,
/// which a config save (the executor's ~10 KB blob) holds for tens of ms. A writer waiting on it would program
/// nothing meanwhile, past the power-cut bound of spec "The ring_writer"; the eraser, the lowest of the ring tasks,
/// can wait without harm (the erased reserve lasts ~15 s at the flight rate).
fn erase(mut eraser: Eraser<'static, Partition>, recorder: &'static Recorder, store: Option<RecorderStore>) {
    let _attachment = recorder
        .attach(Producer::Eraser as usize)
        .inspect_err(|error| log::warn!("ring :: the eraser shares the misc ring: {error:?}"))
        .ok();
    let mut hint = Hint::default();
    loop {
        let started_us = uptime_us();
        let erased = match eraser.erase_step(None, uptime_ms()) {
            Ok(Erasure::Idle) => false,
            Ok(Erasure::Lost(loss)) => {
                recorder.log("ring", &loss_message(&loss));
                true
            }
            Ok(Erasure::Foreign { .. } | Erasure::Saved { .. }) => true,
            Err(error) => {
                // the eraser tries the slot again, and gives it up after flash_ring::ERASE_ATTEMPTS failures
                log::warn!("ring :: an erase failed: {error:?}");
                true
            }
        };
        let rest = if erased {
            Duration::from_micros(uptime_us().saturating_sub(started_us)).max(ERASER_PAUSE)
        } else {
            ERASER_IDLE
        };
        if let Some(store) = &store {
            hint.publish(store, recorder.writer_position(), uptime_us());
        }
        std::thread::sleep(rest);
    }
}

/// The loss line of a block newest wins erased unsaved (spec "The ring_eraser and newest wins"): `block <sequence>
/// (boot <id>, session <s>, flight <0|1>) erased unsaved`, `?` where the header no longer told.
fn loss_message(loss: &Loss) -> String {
    let (boot, session) = loss.header.map_or_else(
        || ("?".to_string(), "?".to_string()),
        |header| (header.boot_id.to_string(), header.session.as_str().to_string()),
    );
    format!(
        "block {} (boot {boot}, session {session}, flight {}) erased unsaved",
        loss.block_sequence,
        u8::from(loss.flight)
    )
}

/// The writer position hint's bookkeeping: what was written to the NVS last, and when it was last looked at.
#[derive(Default)]
struct Hint {
    /// The position last written; None before the first.
    written: Option<Position>,
    /// The uptime of the last look, in µs.
    looked_us: u64,
    /// Whether the last write failed: only the first failure of a run is logged.
    failing: bool,
}

impl Hint {
    /// Writes `position` to the NVS when a second has passed since the last look and it moved (spec "Positions":
    /// about once a second). Runs on the ring_eraser: what reaches the writer is only the flash lock of the NVS's
    /// own flash calls (a page erase ≤ 16 ms, inside the writer's power-cut bound), never the NVS lock.
    fn publish(&mut self, store: &RecorderStore, position: Option<Position>, now_us: u64) {
        if now_us.saturating_sub(self.looked_us) < HINT_PERIOD_US {
            return;
        }
        self.looked_us = now_us;
        let Some(position) = position.filter(|&position| Some(position) != self.written) else {
            return;
        };
        match store.publish_writer(position) {
            Ok(()) => {
                self.written = Some(position);
                self.failing = false;
            }
            Err(error) if !self.failing => {
                log::warn!("ring :: the writer position was not written to the cfg NVS: {error}");
                self.failing = true;
            }
            Err(_) => {}
        }
    }
}

/// The recorder's records in the `cfg` NVS, namespace `recorder` (see the module notes).
struct RecorderStore {
    /// The namespace.
    nvs: EspNvs<NvsCustom>,
}

impl RecorderStore {
    /// Opens the namespace on the `cfg` partition.
    fn open(partition: EspCustomNvsPartition) -> anyhow::Result<RecorderStore> {
        Ok(RecorderStore {
            nvs: EspNvs::new(partition, NAMESPACE, true)?,
        })
    }

    /// The positions the previous boot left; a key that is missing or unreadable is no hint.
    fn hints(&self) -> Hints {
        Hints {
            writer: self.position(WRITER_KEY),
            saver: self.position(SAVER_KEY),
        }
    }

    /// The position under `key`, or None.
    fn position(&self, key: &str) -> Option<Position> {
        let packed = self.nvs.get_u64(key).ok()??;
        Some(Position {
            block_sequence: (packed >> 32) as u32,
            offset: packed as u32,
        })
    }

    /// Writes the writer position: one u64 entry, so a cut keeps the old one or the new one, never half of each.
    fn publish_writer(&self, position: Position) -> anyhow::Result<()> {
        let packed = (u64::from(position.block_sequence) << 32) | u64::from(position.offset);
        Ok(self.nvs.set_u64(WRITER_KEY, packed)?)
    }

    /// Writes the session-start record, replacing the previous boot's only once it is written.
    fn record_start(&self, record: &Value) -> anyhow::Result<()> {
        config::replace_blob(&self.nvs, START_KEY, record.to_string().as_bytes())
    }
}

/// A position as the JSON pair `[block_sequence, offset]`.
fn pair(position: Position) -> [u32; 2] {
    [position.block_sequence, position.offset]
}

/// The `log` partition (partitions.csv: 24 MiB at 0x800000), found by its label, as the ring's [`Flash`]. Each ring
/// task holds its own copy; esp_flash serialises the calls, NVS's on the same chip among them.
#[derive(Clone, Copy)]
pub struct Partition(*const sys::esp_partition_t);

// SAFETY: the pointer names an entry of the partition table IDF keeps for the life of the program, and every
// esp_partition_* call is thread-safe.
unsafe impl Send for Partition {}
// SAFETY: as for Send.
unsafe impl Sync for Partition {}

impl Partition {
    /// The `log` partition; None when the partition table has none.
    fn find() -> Option<Partition> {
        // SAFETY: a lookup in the static partition table with a valid C string.
        let partition = unsafe {
            sys::esp_partition_find_first(
                sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
                c"log".as_ptr(),
            )
        };
        (!partition.is_null()).then_some(Partition(partition))
    }
}

impl Flash for Partition {
    type Error = sys::EspError;

    /// Reads with esp_partition_read. The callers read into internal RAM (a ≤ 4 KiB buffer): esp_flash reads a PSRAM
    /// buffer through a bounce buffer of up to 16 KiB of internal DMA memory, the kind esp_hosted cannot do without.
    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error> {
        // SAFETY: esp_partition_read checks the range against the partition and fills at most the buffer's length.
        sys::esp!(unsafe { sys::esp_partition_read(self.0, offset as usize, buffer.as_mut_ptr().cast(), buffer.len()) })
    }

    /// Programs with esp_partition_write_raw: no encryption, and the ring carries its own CRCs.
    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        // SAFETY: the range is checked against the partition; the data is read, never kept.
        sys::esp!(unsafe { sys::esp_partition_write_raw(self.0, offset as usize, data.as_ptr().cast(), data.len()) })
    }

    /// Erases the one 4 KiB sector at `offset` (13-17 ms; a 64 KiB erase is never issued).
    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
        // SAFETY: the range is checked against the partition and its erase size.
        sys::esp!(unsafe { sys::esp_partition_erase_range(self.0, offset as usize, BLOCK_BYTES as usize) })
    }

    /// The partition's size in bytes.
    fn size(&self) -> u32 {
        // SAFETY: the table entry lives for the life of the program.
        unsafe { (*self.0).size }
    }
}

/// The recorder's settings from the board config: `recorder.telemetry_ms` (0 = every row) and `recorder.stats_ms`
/// (a positive period; the config validator refuses 0, and a 0 that got here anyway takes the default rather than a
/// status line on every writer step).
fn settings(document: &Value) -> Settings {
    let number = |key: &str| {
        document["recorder"][key]
            .as_u64()
            .and_then(|value| u32::try_from(value).ok())
    };
    Settings {
        telemetry_ms: number("telemetry_ms").unwrap_or(DEFAULT_TELEMETRY_MS),
        stats_ms: number("stats_ms")
            .filter(|&stats_ms| stats_ms > 0)
            .unwrap_or(DEFAULT_STATS_MS),
    }
}

/// `bytes` of zeroed PSRAM, never freed: the rings live as long as the board runs.
fn psram(bytes: usize) -> anyhow::Result<&'static mut [u8]> {
    // SAFETY: an allocation of `bytes`, checked below; calloc zeroes it.
    let pointer = unsafe { sys::heap_caps_calloc(1, bytes, sys::MALLOC_CAP_SPIRAM | sys::MALLOC_CAP_8BIT) };
    anyhow::ensure!(!pointer.is_null(), "no {bytes} B of PSRAM for the recorder");
    // SAFETY: the allocation above, `bytes` long, handed out once and never freed, so nothing else refers to it.
    Ok(unsafe { core::slice::from_raw_parts_mut(pointer.cast(), bytes) })
}

/// The uptime in µs (esp_timer, unwrapped): every record's stamp.
fn uptime_us() -> u64 {
    // SAFETY: a plain read of the monotonic system timer, never negative.
    unsafe { sys::esp_timer_get_time() as u64 }
}

/// The uptime in wrapping milliseconds, the eraser's unit.
fn uptime_ms() -> u32 {
    (uptime_us() / 1000) as u32
}

/// Wakes the ring_writer at once, for a durable record (`recorder::Wake`): runs on the producer's thread and never
/// blocks. Before the writer runs there is nobody to wake, and the record waits for its first step.
fn wake() {
    let writer = WRITER_TASK.load(Ordering::Acquire);
    if !writer.is_null() {
        // SAFETY: the writer's task never ends, so its handle stays valid.
        unsafe { task::notify(writer, NonZeroU32::MIN) };
    }
}

/// Spawns `body` as a thread on core 0 with this name, stack and priority: none of the ring's work runs on core 1,
/// the flight core.
fn spawn(
    name: &'static CStr,
    stack_size: usize,
    priority: u8,
    body: impl FnOnce() + Send + 'static,
) -> anyhow::Result<()> {
    ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        priority,
        pin_to_core: Some(Core::Core0),
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new().stack_size(stack_size).spawn(body);
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    Ok(())
}

/// Keeps a ring thread that cannot run alive for good, asleep: its loop never ends, so nobody waits on it ending.
fn park() -> ! {
    loop {
        std::thread::sleep(Duration::from_secs(3600));
    }
}

/// `recorder update`'s parameter: the status line's period.
const UPDATE_PARAMETERS: &[Parameter] = &[Parameter {
    name: "stats_ms",
    kind: Kind::Integer {
        low: 1,
        high: 3_600_000,
    },
    default: None,
    about: "how often the writer logs its status line ('session', the ring counters) into recorder.log, ms",
}];

/// `ring load`'s parameters: the flight-rate stand-in.
const LOAD_PARAMETERS: &[Parameter] = &[
    Parameter {
        name: "lines_per_s",
        kind: Kind::Integer { low: 1, high: 2000 },
        default: Some("200"),
        about: "log lines a second; 200 lines of ~80 bytes is about the v0.1 flight rate (14.6 KB/s of binary records)",
    },
    Parameter {
        name: "seconds",
        kind: Kind::Integer { low: 1, high: 3600 },
        default: Some("60"),
        about: "how long the load runs",
    },
];

/// A `ring load` is running: one at a time.
static LOADING: AtomicBool = AtomicBool::new(false);

/// The flight-rate stand-in until the sensors (M5 on) produce rows: a core-0 thread below the writer logs
/// `lines_per_s` fixed-size lines a second into recorder.log for `seconds`, so the writer, the eraser and the flash
/// carry the flight's byte rate while `pace`, `video` and resets run beside it (the M3 proof). It answers at once; the
/// thread logs `done: <sent> lines in <ms> ms, <dropped> dropped` at the end. The lines go through the shared `misc`
/// ring (the thread has no ring of its own), never to the console.
fn start_load(recorder: &'static Recorder, lines_per_s: u32, seconds: u32) -> String {
    if LOADING.swap(true, Ordering::AcqRel) {
        return cc_protocol::build("err", &["ring", "a load is running already"], &[]);
    }
    let body = move || {
        let total = lines_per_s.saturating_mul(seconds);
        let mut sent: u32 = 0;
        let mut dropped: u32 = 0;
        let start = Instant::now();
        while sent < total {
            // every line due by now: a late wake catches up rather than stretching the run
            let elapsed_us = start.elapsed().as_micros() as u64;
            let due = (elapsed_us * u64::from(lines_per_s) / 1_000_000).min(u64::from(total)) as u32;
            while sent < due {
                let message = format!("line {sent:08} of {total:08} 0123456789abcdefghijklmnopqrstuvwxyz0123456789");
                dropped += u32::from(!recorder.log("ring load", &message));
                sent += 1;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        let summary = format!(
            "done: {sent} lines in {} ms, {dropped} dropped",
            start.elapsed().as_millis()
        );
        recorder.log("ring load", &summary);
        LOADING.store(false, Ordering::Release);
    };
    match spawn(c"ring_load", 4096, 5, body) {
        Ok(()) => cc_protocol::build(
            "ok",
            &["ring", "load"],
            &[
                ("lines_per_s", &lines_per_s.to_string()),
                ("seconds", &seconds.to_string()),
            ],
        ),
        Err(error) => {
            LOADING.store(false, Ordering::Release);
            cc_protocol::build("err", &["ring", &error.to_string()], &[])
        }
    }
}

/// The recorder's commands (the Glider Command Language, firmware::gcl), registered when it opens: `ring`, its
/// figures, `ring load` (the flight-rate stand-in), and `recorder update stats_ms=`, plan B's
/// `update recorder {"stats_ms": n}`.
fn register_commands(recorder: &'static Recorder) {
    gcl::register(vec![
        Command {
            owner: "recorder",
            words: "ring load",
            about: "log fixed-size lines at a flight-like rate from a core-0 thread for a while (the M3 proof's load)",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: LOAD_PARAMETERS,
            bare_words: &[],
            run: Box::new(move |arguments| {
                let lines_per_s = u32::try_from(arguments.integer("lines_per_s")).unwrap_or(200);
                let seconds = u32::try_from(arguments.integer("seconds")).unwrap_or(60);
                start_load(recorder, lines_per_s, seconds)
            }),
        },
        Command {
            owner: "recorder",
            words: "ring",
            about: "the flash ring: blocks erased, closed, lost, the writer position, chunks, and the record counters",
            gate: Gate::Open,
            reach: Reach::Everywhere,
            parameters: &[],
            bare_words: &[],
            run: Box::new(move |_| ring_report(recorder)),
        },
        Command {
            owner: "recorder",
            words: "recorder update",
            about: "set the status line's period (plan B `update recorder {\"stats_ms\": n}`), until the next boot",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: UPDATE_PARAMETERS,
            bare_words: &[],
            run: Box::new(move |arguments| {
                let stats_ms = u32::try_from(arguments.integer("stats_ms")).unwrap_or(DEFAULT_STATS_MS);
                let changed = recorder.set_stats_ms(stats_ms);
                cc_protocol::build(
                    "ok",
                    &["recorder"],
                    &[
                        ("stats_ms", &recorder.settings().stats_ms.to_string()),
                        ("changed", if changed { "1" } else { "0" }),
                    ],
                )
            }),
        },
    ]);
}

/// The `ring` command's answer: `ok ring name=value ...`, or `err ring <why>` before the bring-up scan has run.
///
/// The block figures are walked from the table now (no saver yet, so every closed block counts unsaved); the
/// record counters are what the writer published last.
fn ring_report(recorder: &'static Recorder) -> String {
    let Some(view) = ring() else {
        return cc_protocol::build(
            "err",
            &[
                "ring",
                "not up: the bring-up scan has not finished, or found no log partition (see the log)",
            ],
            &[],
        );
    };
    let figures = view.table.figures(None);
    let stats = recorder.stats();
    let writer = recorder.writer_position().map_or_else(
        || "none".to_string(),
        |position| format!("{}/{}", position.block_sequence, position.offset),
    );
    // SAFETY: a plain read of a counter components/log_hook keeps.
    let idf_dropped = unsafe { sys::log_hook_dropped() };
    let values = [
        ("erased", figures.erased_ahead.to_string()),
        ("saved", figures.saved.to_string()),
        ("unsaved", figures.unsaved.to_string()),
        ("other_version", figures.other_version.to_string()),
        ("foreign", figures.foreign.to_string()),
        ("lost", figures.lost.to_string()),
        ("bad", figures.bad.to_string()),
        (
            "unformatted",
            u8::from(view.unformatted && figures.foreign > 0).to_string(),
        ),
        ("first", view.first_sequence.to_string()),
        ("writer", writer),
        ("chunks", stats.chunks.to_string()),
        ("flash_errors", stats.flash_errors.to_string()),
        ("tlm_count", stats.telemetry.count.to_string()),
        ("tlm_max", stats.telemetry.max.to_string()),
        ("tlm_dropped", stats.telemetry.dropped.to_string()),
        ("log_count", stats.log.count.to_string()),
        ("log_max", stats.log.max.to_string()),
        ("log_dropped", stats.log.dropped.to_string()),
        ("idf_dropped", idf_dropped.to_string()),
    ];
    let named: Vec<(&str, &str)> = values.iter().map(|(key, value)| (*key, value.as_str())).collect();
    cc_protocol::build("ok", &["ring"], &named)
}
