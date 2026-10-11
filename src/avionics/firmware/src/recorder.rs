//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder on the board (doc/specs/avionics-port.md "Storage design", and "Bring-up order" step 3): the glue
//! between crates/recorder, crates/flash_ring and the ESP32-P4. Power can drop at any moment, so every record goes
//! to flash as soon as it can; the ring_saver copies the flash to the SD card behind it, and the ring_eraser erases
//! what the saver has passed.
//!
//! ```text
//! producers (any core) --binary records--> PSRAM rings --> ring_writer (core 0, prio 7) --> `log` partition, 24 MiB
//!   supervisor, executor, eraser,          + misc ring          |  one chunk per ~10 ms tick    |          ^
//!   everyone else (log bridge, IDF)                             +--> CC tees (`log`, `tlm`)     |          | 4 KiB erases
//!                                                                                               v          |
//!                       /sdcard/<session>/recordings/... <-- ring_saver (core 0, prio 4) -- saver_position --+
//!                       /sdcard/sessions.csv                 text, batches, fsync           ring_eraser (core 0, prio 3)
//! ```
//!
//! The bring-up runs in two steps, so no line of the boot is lost and boot never waits on flash:
//! 1. [`open`], on main right after the board config: the stream registry, the producer rings and the CC tees in
//!    PSRAM, and the Recorder, published to every thread. From then on every log line and telemetry row queues in
//!    PSRAM, whoever makes it.
//! 2. [`start`], on main once the SD card's state is known and the camera's encoder has its internal RAM: spawns the
//!    ring_writer. Its bring-up routine (spec "The scan, the positions and the session-start record") reads the ring
//!    positions and the previous boot's session-start record from the `cfg` NVS, scans the 6144 block headers,
//!    writes this boot's record, spawns the ring_eraser and, with a card mounted, the ring_saver, and only then
//!    starts programming; boot goes on meanwhile, and the producers queue.
//!
//! What lives in the `cfg` NVS, namespace `recorder`, all of it hints the scan can do without:
//! * `writer`: the writer position, written about once a second by the ring_eraser, never by the writer itself (a
//!   u64: block sequence << 32 | offset); it only keeps the block numbering from going back after the newest blocks
//!   were erased;
//! * `saver`: the saver position, the same u64, written by the ring_saver after every batch it commits (about once a
//!   second when it keeps up), and by the bring-up once, before the session-start record names a new card. It is
//!   the saver's only record of progress, and is trusted only for the card it was saved to;
//! * `start`: the session-start record, once per boot (JSON): boot id, session, config id, reset cause, the card
//!   (its FAT volume serial `XXXX-XXXX`, `unknown` when unreadable, null with none), and the positions this boot
//!   started from.
//!
//! **The card the saver hint belongs to** (spec "Bring-up", steps 4 and 5). A boot compares the mounted card's
//! volume serial with the card the previous boot's record names. The same card: the saver starts at the hint, and
//! the eraser takes every block behind it as saved. Another card, or one whose serial would not read: the saver
//! starts at the oldest block in flash, so the new card gets everything still there, and nothing counts as saved.
//! No card: no saver (M19's uart_saver may later walk with a cursor of its own), and the blocks behind the hint
//! count as saved, since they are on the card it was saved to. The bring-up writes the new saver start as the hint
//! BEFORE the record that names the new card, so a cut between the two never leaves an old card's position beside
//! a record naming the new card: the worst a cut there does is re-save into duplicates (spec "Duplicates").
//!
//! IDF serialises every NVS call, on every partition, behind one global lock, which a config save (the executor's
//! ~10 KB blob) holds for tens of ms. So the ring_writer never touches the NVS after its bring-up: the eraser and
//! the saver, the two lowest ring tasks, write the hints and can wait for the lock without harm. What reaches the
//! writer is only the flash lock of the NVS's own flash calls (a page erase, at most ~16 ms, inside the writer's
//! power-cut bound).
//!
//! Nothing here runs on core 1, the flight core, and no producer takes a lock (crates/recorder). Every big buffer is
//! in PSRAM. What stays in internal RAM is the three ring threads' stacks (writer 8 KiB, eraser 6 KiB, saver 8 KiB;
//! the supervisor's runs in PSRAM, supervisor.rs) and the crates' working buffers: about 18 KiB for the writer and
//! the eraser (the writer's chunk, the ring's chunk and its 4 KiB erased-check buffer, the eraser's per-slot failure
//! counts), and for the saver its 4 KiB block buffer, which esp_flash fills directly (a PSRAM buffer would bounce
//! through internal DMA memory), its 1 KiB tail buffer, and its batch while it collects: the lines of up to a second,
//! one growing buffer per file, internal while a file's buffer is 4 KiB or less. The 48 KiB block table is in PSRAM:
//! the heap puts any allocation over 4 KiB there (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL).

use std::ffi::CStr;
use std::num::NonZeroU32;
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};
use std::sync::{Mutex, OnceLock, PoisonError};
use std::time::{Duration, Instant};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::task;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::{EspCustomNvsPartition, EspNvs, NvsCustom};
use esp_idf_svc::sys;
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use firmware::sd;
use flash_ring::header::Session;
use flash_ring::{
    BLOCK_BYTES, BacklogTrend, BlockTable, Eraser, Erasure, Figures, Flash, Geometry, Hints, Position, Ring,
};
use recorder::{
    BATCH_BLOCKS, Buffers, Identity, LineRing, MERGE_HORIZON_US, Recorder, Registry, SaveError, Saved, Saver, Settings,
    StdFs, StreamId, TICK_US, Writer, layout,
};
use serde::Serialize;
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
    /// The control thread: the IMU, the separation switch, and from M7 the stage machine and the flight law.
    Control = 3,
    /// Port 0's owner: its parts' rows and log lines.
    I2c0 = 4,
    /// Port 1's owner.
    I2c1 = 5,
}

/// Each producer ring's PSRAM in bytes, by [`Producer`] index: the spec's sizes for the supervisor (256 KiB), the
/// executor (128 KiB), the control thread (1 MiB), i2c0 (1 MiB) and i2c1 (512 KiB), and 16 KiB for the eraser's loss
/// lines (~100 B each, at most a few a second).
const PRODUCER_BYTES: [usize; 6] = [256 << 10, 128 << 10, 16 << 10, 1 << 20, 1 << 20, 512 << 10];

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

/// The ring_saver's priority on core 0: below the video SD writer (6) and the encoder (10), so on the card video comes
/// first and logging second, and above the eraser (3), which erases behind it (spec "The ring_saver", Thread).
const SAVER_PRIORITY: u8 = 4;

/// The ring_saver's stack: a step's flash reads, the decode (f32 `Display` included), FatFs's open, write and fsync
/// through the VFS and the SD driver, and the hint's NVS write; every buffer it fills is on the heap. `stats recorder`
/// reports how much of it was never used (`saver.stack_free`), to tune on the board.
const SAVER_STACK: usize = 8 << 10;

/// How long the saver rests when it has caught up with the writer: the walk reads the writer's open block in parts,
/// one every rest, so ten parts a second at most; a batch still goes to the card about once a second (BATCH_MS).
const SAVER_IDLE: Duration = Duration::from_millis(100);

/// The least the saver rests between two full batches of a backlog: one tick, so the eraser below it (and the idle
/// task the task watchdog watches) gets core 0 between batches, as the eraser leaves the writer room between erases.
const SAVER_PAUSE: Duration = Duration::from_millis(1);

/// The longest the saver rests after a call that failed. The rest doubles from [`SAVER_IDLE`] up to this while the
/// calls keep failing: a dead card's calls run into the SD driver's timeouts, on the SDMMC host the C6's Wi-Fi link
/// shares (slot 1), and a card that refuses one file (an HTTP pull holding it open) is a second or two.
const SAVER_BACKOFF: Duration = Duration::from_secs(5);

/// How long `ring-full` stays up after newest wins last erased an unsaved block, ms: with no card the ring loses a
/// block every few seconds at the flight rate but only every minute or so on the pad, and the token must not flicker.
const RING_FULL_HOLD_MS: u32 = 60_000;

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

/// The key of the saver position hint (the ring_saver writes it after every batch it commits; the bring-up once).
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
    /// Each recorded device's stream, by entry name (controller::stream).
    devices: Vec<(String, StreamId)>,
    /// This boot's NVS count; None when uncounted (block headers then carry 0).
    boot_id: Option<u32>,
    /// This boot's file prefix, which every block header carries.
    session: Session,
}

/// The recorder once [`open`] built it.
static RECORDING: OnceLock<Recording> = OnceLock::new();

/// The recorder's object name (plan B `Recorder.name`): `inspect`, `stats`, `update` and `probe` reach it by it while
/// a recorder runs, whatever its config entry is called, and its probe lines start with it.
pub const NAME: &str = "recorder";

/// The device key `vetoes.csv` is registered under ([`device_stream`]): the stage machine's veto episodes, written by
/// its entry beside `<name>.csv` (crates/controller `vetoes`).
pub const VETOES: &str = "vetoes";

/// The microSD card as the ring knows it at boot: mounted, and named by its FAT volume serial (spec "Bring-up", step
/// 4: the saver hint is trusted only for the card it was saved to).
#[derive(Clone, Copy, Debug)]
pub struct Card {
    /// Its FAT volume serial (firmware::sd::volume_serial); None when the boot sector would not read, so it is never
    /// taken for the previous boot's card.
    pub serial: Option<u32>,
}

impl Card {
    /// The card as the session-start record names it: partition 1's serial as `blkid` shows it (`1A2B-3C4D`), or
    /// `unknown`.
    pub fn name(self) -> String {
        self.serial.map_or_else(|| "unknown".to_string(), sd::serial_name)
    }
}

/// The flash ring once the bring-up scan has run: what `GET /ring`, `GET /ring/image`, the `ring` command, `stats`
/// and `health` read.
pub struct RingView {
    /// The `log` partition.
    pub partition: Partition,
    /// The block table the writer, the eraser and the saver share.
    pub table: &'static BlockTable,
    /// The sequence of the first block this boot opens: every older block was in flash at boot.
    pub first_sequence: u32,
    /// The scan found no block of any layout: the eraser formats the foreign slots ahead of the writer.
    pub unformatted: bool,
    /// The card mounted at boot; None without one (no saver runs then).
    pub card: Option<Card>,
    /// Where the saver started: the previous boot's position when the card is the one it was saved to, else the
    /// oldest block in flash. None without a card.
    pub saver_start: Option<Position>,
    /// The card is the one the previous boot's session-start record names, so the saver resumed at its hint.
    pub same_card: bool,
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

/// The stream of device `name` (controller::stream); None without a recorder, or for a device that writes none.
pub fn device_stream(name: &str) -> Option<StreamId> {
    let recording = RECORDING.get()?;
    recording
        .devices
        .iter()
        .find(|(device, _)| device == name)
        .map(|(_, stream)| *stream)
}

/// The flash ring; None until the writer's bring-up scan has run (or when it found no `log` partition).
pub fn ring() -> Option<&'static RingView> {
    RING.get()
}

/// This boot's file prefix (spec "Row, log and index text", Session prefix; plan B `Recorder.session()`): the board
/// config's `recorder.session` when it is a valid label, else the boot id as `%06u`, else
/// `YYYYMMDD_HHMMSS_<6 random digits>` from the clock (2000-01-01 plus the uptime while it is unset).
///
/// A label the card's layout reserves (`videos`, `recordings` or `video`, in any case: crates/recorder layout
/// `is_reserved`) counts as no label: `videos` would put the session's recordings into the videos directory, on a
/// two-partition card the videos partition, and the other two are an M4 card's root directories, which the tools
/// still read. [`open`] logs that it was ignored.
///
/// Args:
/// * `document` - the board config this boot runs.
/// * `boot_id` - this boot's NVS count; None when uncounted.
///
/// Returns the prefix: it names the SD directory, every file of the boot, the video directory and whoami's session.
pub fn session_prefix(document: &Value, boot_id: Option<i32>) -> String {
    let label = match recorder_wire::session_label(document["recorder"]["session"].as_str().unwrap_or_default()) {
        recorder_wire::SessionLabel::Valid(label) if layout::is_reserved(&label) => recorder_wire::SessionLabel::Unset,
        label => label,
    };
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
/// * `flight` - whether the bring-up starts in flight: a crumb in BOOSTING..LANDING (crate::warm::peek), a reboot in
///   flight. The warm start's restore lowers it again unless the boot is warm.
///
/// The config id comes later, with [`start`]: the layout rewrites the config after this, and the id is the
/// rewritten config's (plan B main.py:120-124).
///
/// Errors: PSRAM would not hold a ring, the prefix is not a session label, the registry refused a
/// stream, or the recorder opened before. The board runs on without recording then.
pub fn open(document: &Value, boot_id: Option<i32>, session: &str, flight: bool) -> anyhow::Result<()> {
    let session = Session::new(session).ok_or_else(|| anyhow::anyhow!("{session:?} is not a session label"))?;
    let mut registry = Registry::new();
    let health = registry
        .add(supervisor::HEALTH_FILE, &supervisor::HEALTH_FIELDS, 0, false)
        .map_err(|error| anyhow::anyhow!("{}: {error:?}", supervisor::HEALTH_FILE))?;
    // the devices' streams, from every entry of the saved config, enabled or not: the layout moves no name but may
    // enable a part (a fitted SEN0253), and the registry is fixed from here; a part that never comes up never writes
    // its header, so a stream registered for it costs nothing
    let mut devices = Vec::new();
    for entry in board_config::entries(document) {
        let Some(stream) = controller::stream(&entry) else {
            continue;
        };
        match registry.add(&stream.file, stream.fields, stream.decimate_us, stream.durable) {
            Ok(id) => devices.push((entry.name.to_string(), id)),
            Err(error) => log::warn!("Recorder :: {} not recorded: {error:?}", stream.file),
        }
        // the GNSS also writes its GGA, sky and events streams (M11), each under its own device key
        for (key, companion) in controller::companions(&entry) {
            match registry.add(
                &companion.file,
                companion.fields,
                companion.decimate_us,
                companion.durable,
            ) {
                Ok(id) => devices.push((key, id)),
                Err(error) => log::warn!("Recorder :: {} not recorded: {error:?}", companion.file),
            }
        }
        // the stage machine's entry also writes the veto episodes (M9), once whatever the entries
        if let Some(vetoes) = controller::vetoes(&entry)
            && !devices.iter().any(|(name, _)| name == VETOES)
        {
            match registry.add(&vetoes.file, vetoes.fields, vetoes.decimate_us, vetoes.durable) {
                Ok(id) => devices.push((VETOES.to_string(), id)),
                Err(error) => log::warn!("Recorder :: {} not recorded: {error:?}", vetoes.file),
            }
        }
    }
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
    };
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
        devices,
        boot_id,
        session,
    };
    RECORDING
        .set(recording)
        .map_err(|_| anyhow::anyhow!("the recorder opened twice"))?;
    let recorder = get().ok_or_else(|| anyhow::anyhow!("the recorder did not publish"))?;
    let configured = document["recorder"]["session"].as_str().unwrap_or_default();
    match recorder_wire::session_label(configured) {
        recorder_wire::SessionLabel::Ignored(label) => log::warn!(
            "Recorder :: recorder.session {label:?} ignored (letters, digits and '-' only, one letter at least, 32 at most)"
        ),
        recorder_wire::SessionLabel::Valid(label) if layout::is_reserved(&label) => log::warn!(
            "Recorder :: recorder.session {label:?} ignored: it names a directory at the card's root that is no \
             session's (the videos partition's mount point, or an M4 card's recordings/ or video/), so the boot id \
             names this session"
        ),
        _ => {}
    }
    register_commands(recorder);
    Ok(())
}

/// Spawns the ring_writer (core 0, priority 7). Its bring-up routine, run at the main task's priority, scans the `log`
/// partition, writes the session-start record and spawns the ring_eraser and, with a card, the ring_saver; then the
/// writer steps forever. Boot goes on meanwhile.
///
/// Call it after the camera's encoder has reserved its internal RAM (video::reserve): the three ring threads'
/// stacks and buffers come from the same heap, and would split the encoder's ~138 KB block.
///
/// Args:
/// * `settings` - the `cfg` partition: the ring positions and the session-start record live there.
/// * `card` - the mounted microSD card; None without one. Its volume serial tells whether the previous boot's saver
///   position is this card's (see the module notes).
/// * `config_id` - the running config's id, the post-layout one: bound into the session.csv rows and written into
///   the session-start record.
///
/// Errors: the recorder did not open, or the thread would not spawn.
pub fn start(settings: EspCustomNvsPartition, card: Option<Card>, config_id: &str) -> anyhow::Result<()> {
    let recording = RECORDING
        .get()
        .ok_or_else(|| anyhow::anyhow!("the recorder did not open"))?;
    recording.recorder.bind_config_id(config_id);
    let config_id = config_id.to_string();
    spawn(c"ring_writer", WRITER_STACK, BRING_UP_PRIORITY, move || {
        write(recording, settings, card, &config_id)
    })
}

/// The ring_writer thread: the bring-up routine, then [`Writer::step`] on every ~10 ms tick or durable record's
/// wake, forever. A ring that cannot come up (no `log` partition, a scan that keeps failing) parks the thread with a
/// log line: the records then stay in PSRAM until their rings fill, and drop counted.
fn write(recording: &'static Recording, settings: EspCustomNvsPartition, card: Option<Card>, config_id: &str) {
    if let Some(current) = task::current() {
        WRITER_TASK.store(current, Ordering::Release);
    }
    let recorder = recording.recorder;
    let store = RecorderStore::open(settings.clone())
        .inspect_err(|error| log::warn!("ring :: the cfg NVS would not open, no positions kept: {error}"))
        .ok();
    let hints = store.as_ref().map_or_else(Hints::default, RecorderStore::hints);
    // the card the previous boot's record names: its saver hint is that card's (read before this boot's record)
    let previous_card = store.as_ref().and_then(RecorderStore::previous_card);
    let Some(partition) = Partition::by_label(c"log") else {
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
    // spec "Bring-up", steps 4 and 5: the saver hint is this card's only when the previous boot named this card
    let same_card =
        card.is_some_and(|card| card.serial.is_some() && previous_card.as_deref() == Some(card.name().as_str()));
    let saver_start = card.map(|_| table.saver_start(hints.saver.filter(|_| same_card)));
    // as Eraser::new judges it too: a hint that names no block in flash passes nothing off as saved
    let saved = hints
        .saver
        .filter(|&hint| (card.is_none() || same_card) && table.saver_start(Some(hint)) == hint);
    log_saver_start(card, previous_card.as_deref(), same_card, saver_start, saved);
    // the new card's start becomes the hint before the record names the new card (see the module notes)
    if let (Some(store), Some(start)) = (&store, saver_start)
        && hints.saver != Some(start)
        && let Err(error) = store.publish(SAVER_KEY, start)
    {
        log::warn!("ring :: the saver's start was not written to the cfg NVS: {error}");
    }
    // spec "Bring-up", step 6: what a fast start needs, written once per boot
    let first = Position {
        block_sequence: scan.next_sequence,
        offset: 0,
    };
    let record = json!({
        "boot": recording.boot_id,
        "session": recording.session.as_str(),
        "config_id": config_id,
        "reset_cause": board::reset_reason(),
        "card": card.map(Card::name),
        "writer": [first.block_sequence, first.offset],
        "saver": saver_start.map(pair),
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
        card,
        saver_start,
        same_card,
    });
    // the store goes to the eraser, which keeps the writer position hint from here on, and the saver opens its own
    // for its hint: this thread never touches the NVS again
    start_eraser(partition, table, recorder, store, saved);
    if let Some(start) = saver_start {
        start_saver(partition, table, recorder, settings, start);
    }
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
/// * `table` - the block table the writer and the saver share.
/// * `recorder` - for the loss lines and the two positions.
/// * `store` - the `cfg` NVS records; None when they would not open (no hint is kept then).
/// * `saved` - the previous boot's saver position when the bring-up trusts it (the same card, or none): every
///   block before it is on that card, so the eraser erases those blocks at once, with no loss line.
fn start_eraser(
    partition: Partition,
    table: &'static BlockTable,
    recorder: &'static Recorder,
    store: Option<RecorderStore>,
    saved: Option<Position>,
) {
    let eraser = match Eraser::new(partition, table, saved) {
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

/// When newest wins last erased an unsaved block (uptime ms, wrapping), for `ring-full`; the table counts the losses.
static LOST_MS: AtomicU32 = AtomicU32::new(0);

/// Set as the ring_eraser thread starts, never cleared (its loop runs forever): an eraser that would not open the
/// ring or spawn only logs, and then the ring gate refuses `arm` and `verify` ([`probe`]).
static ERASER_RUNNING: AtomicBool = AtomicBool::new(false);

/// The ring_eraser thread: one [`Eraser::erase_step`] after another, forever. A step erases a foreign slot, else a
/// block the saver passed 100-200 ms ago (the saver's published position, or behind a trusted hint), else, once no
/// more than ~1 % of the ring is free, the oldest block unsaved (newest wins, pad blocks before flight blocks), each a
/// loss logged into recorder.log.
///
/// Between steps it writes the writer position hint to the `cfg` NVS, about once a second ([`Hint`]). The hint is
/// kept here and not on the ring_writer: a writer waiting on IDF's one global NVS lock (see the module notes) would
/// program nothing meanwhile, past the power-cut bound of spec "The ring_writer"; the eraser, the lowest of the ring
/// tasks, can wait without harm (the erased reserve lasts ~15 s at the flight rate).
fn erase(mut eraser: Eraser<'static, Partition>, recorder: &'static Recorder, store: Option<RecorderStore>) {
    ERASER_RUNNING.store(true, Ordering::Release);
    let _attachment = recorder
        .attach(Producer::Eraser as usize)
        .inspect_err(|error| log::warn!("ring :: the eraser shares the misc ring: {error:?}"))
        .ok();
    let mut hint = Hint::default();
    loop {
        let started_us = uptime_us();
        let erased = match eraser.erase_step(recorder.saver_position(), uptime_ms()) {
            Ok(Erasure::Idle) => false,
            Ok(Erasure::Lost(loss)) => {
                LOST_MS.store(uptime_ms(), Ordering::Relaxed);
                recorder.log("ring", &loss.to_string());
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

/// Logs where the ring_saver starts and why (spec "Bring-up", steps 4 and 5): the card, whether it is the one the
/// previous boot saved to, and which blocks the eraser takes as saved.
///
/// Args:
/// * `card` - the mounted card; None without one.
/// * `previous` - the card the previous boot's session-start record names; None when it names none, or there is no
///   record.
/// * `same_card` - the mounted card is that one.
/// * `start` - where the saver starts; None without a card.
/// * `saved` - the hint the eraser trusts; None when it trusts none.
fn log_saver_start(
    card: Option<Card>,
    previous: Option<&str>,
    same_card: bool,
    start: Option<Position>,
    saved: Option<Position>,
) {
    let position = |position: Position| format!("block {} offset {}", position.block_sequence, position.offset);
    let behind = saved.map_or_else(
        || "no block counts as saved".to_string(),
        |saved| format!("the blocks before {} count as saved", position(saved)),
    );
    match (card, start) {
        (Some(card), Some(start)) => log::info!(
            "ring :: card {}, {}: the saver starts at {}; {behind}",
            card.name(),
            if same_card {
                "the one the previous boot saved to".to_string()
            } else {
                format!("not the previous boot's ({})", previous.unwrap_or("none"))
            },
            position(start),
        ),
        _ => log::info!("ring :: no card, no saver; {behind}"),
    }
}

/// The ring_saver's figures, for `stats recorder`, `inspect recorder`, the `ring` command and `saver-behind`: each an
/// AtomicU32 (the P4 has no 64-bit atomics) that only the saver thread stores, read on its own by any thread.
struct SaverFigures {
    /// The saver thread runs.
    running: AtomicBool,
    /// Batches committed this boot.
    batches: AtomicU32,
    /// Blocks they held (a part of a block counts once per batch).
    blocks: AtomicU32,
    /// Lines they put on the card.
    lines: AtomicU32,
    /// Bytes they put on the card, in KiB.
    kilobytes: AtomicU32,
    /// Card calls that failed (a step that failed counts once).
    card_errors: AtomicU32,
    /// Flash reads that failed.
    flash_errors: AtomicU32,
    /// When a step last found nothing more to read: the walk at the writer's published position (uptime ms).
    caught_up_ms: AtomicU32,
    /// The least stack the saver thread has left since it started, bytes.
    stack_free: AtomicU32,
}

/// The ring_saver's figures this boot.
static SAVER: SaverFigures = SaverFigures {
    running: AtomicBool::new(false),
    batches: AtomicU32::new(0),
    blocks: AtomicU32::new(0),
    lines: AtomicU32::new(0),
    kilobytes: AtomicU32::new(0),
    card_errors: AtomicU32::new(0),
    flash_errors: AtomicU32::new(0),
    caught_up_ms: AtomicU32::new(0),
    stack_free: AtomicU32::new(0),
};

/// Spawns the ring_saver (core 0, priority 4) over the mounted card, from its root, from `start`. A spawn
/// that fails is logged: nothing then reaches the card, `saver-behind` comes up after a minute, and newest wins keeps
/// the newest ~26 min in flash, as with no card.
///
/// Args:
/// * `partition` - the `log` partition.
/// * `table` - the block table the writer and the eraser share.
/// * `recorder` - this boot's recorder: the writer's position, and where the saver publishes its own.
/// * `settings` - the `cfg` partition, for the saver's own handle on the hint (see the module notes).
/// * `start` - where the saver starts (see the module notes).
fn start_saver(
    partition: Partition,
    table: &'static BlockTable,
    recorder: &'static Recorder,
    settings: EspCustomNvsPartition,
    start: Position,
) {
    let body = move || {
        let store = RecorderStore::open(settings)
            .inspect_err(|error| log::warn!("ring :: the saver keeps no position, the cfg NVS would not open: {error}"))
            .ok();
        // the saver's buffers are allocated here, on its own thread, after the encoder's reservation (start)
        // the card's root: the saver files each session where crates/recorder's layout says, under <session>/recordings
        let saver = Saver::new(recorder, partition, table, StdFs::new(sd::MOUNT), start);
        // every line it stages to the uart_saver too (M19), when the recorder's link to its receiver runs
        let saver = match UART_QUEUE.get() {
            Some(queue) => saver.with_uart(queue),
            None => saver,
        };
        save(saver, store)
    };
    if let Err(error) = spawn(c"ring_saver", SAVER_STACK, SAVER_PRIORITY, body) {
        log::error!("ring :: the saver would not start, nothing reaches the card: {error}");
    }
}

/// The uart_saver's queue (M19): the ring_saver pushes each line it stages, the uart_saver sends them; recency first
/// (crates/recorder LineRing). Set before the recorder starts, when the config gives the recorder its UART.
static UART_QUEUE: OnceLock<LineRing> = OnceLock::new();

/// The uart_saver's queue, bytes (PSRAM): ~10 s of the flight's ~26 KB/s of text, so a receiver that stalls a few
/// seconds loses nothing.
const UART_QUEUE_BYTES: usize = 256 << 10;

/// The most the uart_saver hands the driver at once, bytes.
const UART_BATCH_BYTES: usize = 4096;

/// The receive buffer IDF installs for the transmit-only UART, bytes: more than its 128-byte FIFO, else refused.
const UART_RX_BYTES: usize = 256;

/// The uart_saver's rest while its queue is empty.
const UART_IDLE: Duration = Duration::from_millis(20);

/// The uart_saver's priority: below the ring_saver's, which feeds it (§4.2).
const UART_PRIORITY: u8 = 2;

/// The uart_saver's stack: a batch's copy and the driver's write.
const UART_STACK: usize = 4 << 10;

/// The uart_saver (M19, doc/specs/avionics-port.md M19): the recorder's wire lines out of `uart`'s TX on GPIO `tx` at
/// `baud`, to a receiver that keeps them when the board and its card are lost (a Luckfox's recorderd, the C6 box of
/// M20): the bytes plan B sent. Its thread (core 0, below the ring_saver) takes what the saver queued, in order, and
/// never holds the saver: a stalled link costs the oldest lines, counted. Lines reach it from the saver, so a board
/// with no card sends none yet (the no-card walk is M19's next step).
///
/// Errors: the driver would not install, or the thread would not spawn.
pub fn start_uart<U: esp_idf_svc::hal::uart::Uart + 'static>(_uart: U, tx: u8, baud: u32) -> anyhow::Result<()> {
    let queue = UART_QUEUE.get_or_init(|| LineRing::new(UART_QUEUE_BYTES));
    // IDF's own driver, installed here: esp-idf-hal's transmit-only driver installs no receive buffer, which IDF
    // refuses ("uart rx buffer length error"), so it never starts. The peripheral is taken, so nothing else drives it.
    let port = U::port();
    let config = sys::uart_config_t {
        baud_rate: baud as i32,
        data_bits: sys::uart_word_length_t_UART_DATA_8_BITS,
        parity: sys::uart_parity_t_UART_PARITY_DISABLE,
        stop_bits: sys::uart_stop_bits_t_UART_STOP_BITS_1,
        flow_ctrl: sys::uart_hw_flowcontrol_t_UART_HW_FLOWCTRL_DISABLE,
        __bindgen_anon_1: sys::uart_config_t__bindgen_ty_1 {
            source_clk: sys::soc_periph_uart_clk_src_legacy_t_UART_SCLK_DEFAULT,
        },
        ..Default::default()
    };
    // SAFETY: plain driver calls on the UART the board config gives the recorder alone; the driver lives for good.
    unsafe {
        sys::esp!(sys::uart_param_config(port, &config))?;
        sys::esp!(sys::uart_set_pin(port, i32::from(tx), -1, -1, -1))?;
        sys::esp!(sys::uart_driver_install(
            port,
            UART_RX_BYTES as i32,
            (UART_BATCH_BYTES * 2) as i32,
            0,
            ptr::null_mut(),
            0
        ))?;
    }
    spawn(c"uart_saver", UART_STACK, UART_PRIORITY, move || {
        let mut line = Vec::new();
        let mut batch = Vec::with_capacity(UART_BATCH_BYTES + 1024);
        loop {
            batch.clear();
            while batch.len() < UART_BATCH_BYTES && queue.pop(&mut line) {
                batch.extend_from_slice(&line);
            }
            if batch.is_empty() {
                std::thread::sleep(UART_IDLE);
                continue;
            }
            let mut sent = 0;
            while sent < batch.len() {
                // SAFETY: the installed driver's write: it copies the bytes into its transmit buffer, waiting for room
                let count = unsafe { sys::uart_write_bytes(port, batch[sent..].as_ptr().cast(), batch.len() - sent) };
                if count <= 0 {
                    break; // a refused write loses this batch; the next goes on
                }
                sent += count as usize;
            }
        }
    })?;
    log::info!("ring :: uart_saver on GPIO{tx} at {baud} baud");
    Ok(())
}

/// The ring_saver thread: [`Saver::step`] again and again, forever (spec "The ring_saver"). The saver collects what
/// the writer has published into a batch, and once a batch is due (a second of collecting, or 64 blocks) appends it
/// to the card's files, fsyncs them and publishes `saver_position` past it; the eraser then erases those blocks.
///
/// After every batch it commits, it writes [`Saved::position`] to the `cfg` NVS hint, before its next step: about
/// once a second when it keeps up, at every batch of a backlog. The hint is the saver's only record of progress, and
/// the crate relies on it being the last committed batch's: a restart that began in an earlier boot than the card's
/// last would print that boot's headers again into files that hold them (crates/recorder saver.rs, "Headers after a
/// reboot"). The saver writes it itself, rather than handing it to the eraser as the writer does, because it may
/// wait for IDF's global NVS lock without harm (its batch is on the card already; only the next one waits), and one
/// thread per key keeps a commit and its hint in order.
///
/// Rests: [`SAVER_IDLE`] when caught up, [`SAVER_PAUSE`] between full batches of a backlog, and after a failed call
/// a rest that doubles up to [`SAVER_BACKOFF`]. A failed card call leaves the batch as it was: the next step goes on
/// with the file that failed, so a card that refuses a file for a while (FR_LOCKED while an HTTP pull reads it)
/// costs only the wait. Only the first failure of a run is logged.
fn save(mut saver: Saver<'static, 'static, Partition, StdFs>, store: Option<RecorderStore>) {
    SAVER.running.store(true, Ordering::Relaxed);
    SAVER.caught_up_ms.store(uptime_ms(), Ordering::Relaxed);
    let mut backoff = SAVER_IDLE;
    let mut failing = false;
    let mut hint_failing = false;
    let mut bytes: u64 = 0;
    loop {
        let rest = match saver.step(uptime_ms()) {
            Ok(Some(saved)) => {
                count_batch(&saved, &mut bytes);
                if let Some(store) = &store {
                    match store.publish(SAVER_KEY, saved.position) {
                        Ok(()) => hint_failing = false,
                        Err(error) if !hint_failing => {
                            log::warn!("ring :: the saver position was not written to the cfg NVS: {error}");
                            hint_failing = true;
                        }
                        Err(_) => {}
                    }
                }
                failing = false;
                backoff = SAVER_IDLE;
                if saved.blocks >= BATCH_BLOCKS {
                    SAVER_PAUSE
                } else {
                    SAVER_IDLE
                }
            }
            Ok(None) => {
                SAVER.caught_up_ms.store(uptime_ms(), Ordering::Relaxed);
                failing = false;
                backoff = SAVER_IDLE;
                SAVER_IDLE
            }
            Err(error) => {
                let (counter, what) = match &error {
                    SaveError::Card(error) => (&SAVER.card_errors, format!("a card call failed ({error})")),
                    SaveError::Flash(error) => (&SAVER.flash_errors, format!("a flash read failed ({error})")),
                };
                counter.fetch_add(1, Ordering::Relaxed);
                if !failing {
                    log::warn!(
                        "ring :: {what}: the saver goes on where it stopped, after a rest of up to {} s",
                        SAVER_BACKOFF.as_secs()
                    );
                    failing = true;
                }
                let rest = backoff;
                backoff = (backoff * 2).min(SAVER_BACKOFF);
                rest
            }
        };
        // SAFETY: the calling task's own high-water mark (a null handle is the calling task).
        let stack_free = unsafe { sys::uxTaskGetStackHighWaterMark(ptr::null_mut()) };
        SAVER.stack_free.store(stack_free, Ordering::Relaxed);
        std::thread::sleep(rest);
    }
}

/// Adds a committed batch to the saver's figures; `bytes` is the running total the KiB figure is taken from.
fn count_batch(saved: &Saved, bytes: &mut u64) {
    *bytes += u64::from(saved.bytes);
    SAVER.batches.fetch_add(1, Ordering::Relaxed);
    SAVER.blocks.fetch_add(saved.blocks, Ordering::Relaxed);
    SAVER.lines.fetch_add(saved.lines, Ordering::Relaxed);
    SAVER.kilobytes.store((*bytes >> 10) as u32, Ordering::Relaxed);
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
        match store.publish(WRITER_KEY, position) {
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

    /// Writes a position hint under `key` ([`WRITER_KEY`], [`SAVER_KEY`]): one u64 entry, so a cut keeps the old one
    /// or the new one, never half of each.
    fn publish(&self, key: &str, position: Position) -> anyhow::Result<()> {
        let packed = (u64::from(position.block_sequence) << 32) | u64::from(position.offset);
        Ok(self.nvs.set_u64(key, packed)?)
    }

    /// The card the previous boot's session-start record names (`1A2B-3C4D`, or `unknown`); None when it named none,
    /// or there is no record, or it does not read (an M3 record said only whether a card was mounted).
    fn previous_card(&self) -> Option<String> {
        let length = self.nvs.blob_len(START_KEY).ok()??;
        let mut buffer = vec![0u8; length];
        let bytes = self.nvs.get_blob(START_KEY, &mut buffer).ok()??;
        let record: Value = serde_json::from_slice(bytes).ok()?;
        record["card"].as_str().map(str::to_string)
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

/// A data partition found by its label, as a [`Flash`]: the `log` partition (partitions.csv: 24 MiB at 0x800000), the
/// ring's, and the `panic` partition, the panic log's. Each ring task holds its own copy; esp_flash serialises the
/// calls, NVS's on the same chip among them.
#[derive(Clone, Copy)]
pub struct Partition(*const sys::esp_partition_t);

// SAFETY: the pointer names an entry of the partition table IDF keeps for the life of the program, and every
// esp_partition_* call is thread-safe.
unsafe impl Send for Partition {}
// SAFETY: as for Send.
unsafe impl Sync for Partition {}

impl Partition {
    /// The data partition labelled `label` (`log`, the ring; `panic`, crate::panics_logger); None when the table has
    /// none.
    pub fn by_label(label: &CStr) -> Option<Partition> {
        // SAFETY: a lookup in the static partition table with a valid C string.
        let partition = unsafe {
            sys::esp_partition_find_first(
                sys::esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                sys::esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
                label.as_ptr(),
            )
        };
        (!partition.is_null()).then_some(Partition(partition))
    }
}

impl Flash for Partition {
    type Error = sys::EspError;

    /// Reads with esp_partition_read. The callers read into internal RAM (a ≤ 4 KiB buffer; on the file server's
    /// PSRAM stack an allocator::InternalBuffer): esp_flash reads a PSRAM buffer through a bounce buffer of up to
    /// 16 KiB of internal DMA memory, the kind esp_hosted cannot do without.
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
            about: "the flash ring: blocks erased, saved, unsaved, lost, both positions, the backlog in seconds, the record \
                    counters, and the ring_saver's card, batches and errors",
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

/// The flash->SD backlog for the recovery beacon's page (crate::beacon): the ring's unsaved blocks and how many
/// seconds of recording they hold; None before the bring-up scan has run.
pub fn backlog() -> Option<(u32, Option<f32>)> {
    let report = RingReport::new(ring()?, get()?);
    Some((report.unsaved, report.unsaved_s))
}

/// The `ring` command's answer: `ok ring name=value ...`, or `err ring <why>` before the bring-up scan has run.
///
/// The block figures are walked from the table now, against the saver's published position (with no saver, every
/// closed block counts unsaved); the record counters and the write rate are what the writer published last. The
/// `saver_*` figures are the ring_saver's own ([`SaverReport`]).
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
    let report = RingReport::new(view, recorder);
    let saver = SaverReport::new(Some(view), recorder);
    let stats = recorder.stats();
    let position = |position: Option<[u32; 2]>| {
        position.map_or_else(
            || "none".to_string(),
            |[sequence, offset]| format!("{sequence}/{offset}"),
        )
    };
    let seconds = |seconds: Option<f32>| seconds.map_or_else(|| "None".to_string(), |seconds| seconds.to_string());
    // SAFETY: a plain read of a counter components/log_hook keeps.
    let idf_dropped = unsafe { sys::log_hook_dropped() };
    let values = [
        ("erased", report.erased.to_string()),
        ("saved", report.saved.to_string()),
        ("unsaved", report.unsaved.to_string()),
        ("other_version", report.other_version.to_string()),
        ("foreign", report.foreign.to_string()),
        ("lost", report.lost.to_string()),
        ("bad", report.bad.to_string()),
        ("reserve", report.reserve.to_string()),
        ("unformatted", u8::from(report.unformatted).to_string()),
        ("first", report.first.to_string()),
        ("writer", position(report.writer)),
        ("saver", position(report.saver)),
        ("rate", report.rate.to_string()),
        ("erased_s", seconds(report.erased_s)),
        ("unsaved_s", seconds(report.unsaved_s)),
        ("chunks", report.chunks.to_string()),
        ("flash_errors", report.flash_errors.to_string()),
        (
            "uart_pushed",
            UART_QUEUE
                .get()
                .map_or_else(|| "None".to_string(), |queue| queue.pushed().to_string()),
        ),
        (
            "uart_dropped",
            UART_QUEUE
                .get()
                .map_or_else(|| "None".to_string(), |queue| queue.dropped().to_string()),
        ),
        (
            "uart_queued",
            UART_QUEUE
                .get()
                .map_or_else(|| "None".to_string(), |queue| queue.queued().to_string()),
        ),
        ("tlm_count", stats.telemetry.count.to_string()),
        ("tlm_max", stats.telemetry.max.to_string()),
        ("tlm_dropped", stats.telemetry.dropped.to_string()),
        ("log_count", stats.log.count.to_string()),
        ("log_max", stats.log.max.to_string()),
        ("log_dropped", stats.log.dropped.to_string()),
        ("idf_dropped", idf_dropped.to_string()),
        ("card", saver.card.unwrap_or_else(|| "none".to_string())),
        ("saver_running", u8::from(saver.running).to_string()),
        ("saver_same_card", u8::from(saver.same_card).to_string()),
        ("saver_start", position(saver.start)),
        ("saver_batches", saver.batches.to_string()),
        ("saver_blocks", saver.blocks.to_string()),
        ("saver_lines", saver.lines.to_string()),
        ("saver_kb", saver.kilobytes.to_string()),
        ("saver_card_errors", saver.card_errors.to_string()),
        ("saver_flash_errors", saver.flash_errors.to_string()),
        (
            "saver_behind_ms",
            saver.behind_ms.map_or_else(|| "None".to_string(), |ms| ms.to_string()),
        ),
        ("saver_stack_free", saver.stack_free.to_string()),
    ];
    let named: Vec<(&str, &str)> = values.iter().map(|(key, value)| (*key, value.as_str())).collect();
    cc_protocol::build("ok", &["ring"], &named)
}

/// The flash ring's figures (spec "The ring_eraser and newest wins", budget figures) as `stats recorder` and `inspect
/// recorder` carry them (`ring: {...}`), and the `ring` command prints them. A typed struct rather than a `json!`
/// value: a Value widens an f32 to f64, and 12.3 would print as 12.300000190734863.
#[derive(Serialize)]
struct RingReport {
    /// Erased slots, all of them ahead of the writer: fewer than `reserve` is `ring-low`.
    erased: u32,
    /// Closed blocks the saver has passed, which the eraser erases within 100-200 ms.
    saved: u32,
    /// Closed blocks the saver has not passed: the flash-to-SD backlog (every closed block with no saver).
    unsaved: u32,
    /// Blocks of another layout version: kept, never saved by this build.
    other_version: u32,
    /// Foreign slots waiting for their erase.
    foreign: u32,
    /// Unsaved blocks newest wins erased this boot.
    lost: u32,
    /// Slots the eraser gave up this boot.
    bad: u32,
    /// The erased reserve the eraser keeps ahead of the writer, blocks (~1 % of the ring).
    reserve: u32,
    /// The eraser is still formatting a first boot's foreign slots (`ring-unformatted`).
    unformatted: bool,
    /// The sequence of the first block this boot opened.
    first: u32,
    /// `writer_position` as [block_sequence, offset]; None before the writer's first step.
    writer: Option<[u32; 2]>,
    /// `saver_position` as [block_sequence, offset]; None while no saver runs.
    saver: Option<[u32; 2]>,
    /// The measured write rate, bytes of flash a second, as of the writer's last status line.
    rate: u32,
    /// `ring_erased_s`: erased and saved blocks at that rate, the time before newest wins; None at a zero rate.
    erased_s: Option<f32>,
    /// `ring_unsaved_s`: the unsaved blocks at that rate, the flash-to-SD backlog; None at a zero rate.
    unsaved_s: Option<f32>,
    /// Chunks programmed this boot.
    chunks: u32,
    /// The writer's flash calls that failed this boot.
    flash_errors: u32,
}

impl RingReport {
    /// The figures now: the table walked against the saver's published position, and the writer's counters.
    fn new(view: &RingView, recorder: &Recorder) -> RingReport {
        let saver = recorder.saver_position();
        let figures = view.table.figures(saver);
        let stats = recorder.stats();
        let rate = stats.write_rate;
        // a zero rate gives infinity, which JSON has no number for: the figure is None then
        let seconds = |seconds: fn(&Figures, f32) -> f32| {
            (rate > 0).then(|| (seconds(&figures, rate as f32) * 10.0).round() / 10.0)
        };
        RingReport {
            erased: figures.erased_ahead,
            saved: figures.saved,
            unsaved: figures.unsaved,
            other_version: figures.other_version,
            foreign: figures.foreign,
            lost: figures.lost,
            bad: figures.bad,
            reserve: view.table.geometry().reserve_blocks(),
            unformatted: view.unformatted && figures.foreign > 0,
            first: view.first_sequence,
            writer: recorder.writer_position().map(pair),
            saver: saver.map(pair),
            rate,
            erased_s: seconds(Figures::erased_seconds),
            unsaved_s: seconds(Figures::unsaved_seconds),
            chunks: stats.chunks,
            flash_errors: stats.flash_errors,
        }
    }
}

/// The ring_saver as `stats recorder` and `inspect recorder` carry it (`saver: {...}`), and the `ring` command
/// prints it (`saver_*`).
#[derive(Serialize)]
struct SaverReport {
    /// The saver thread runs (a card is mounted and the thread started).
    running: bool,
    /// The card mounted at boot, by its FAT volume serial (`1A2B-3C4D`, `unknown`); None without a card.
    card: Option<String>,
    /// The card is the one the previous boot saved to, so the saver resumed at the previous boot's position.
    same_card: bool,
    /// Where the saver started, [block_sequence, offset]; None without a card.
    start: Option<[u32; 2]>,
    /// Batches committed this boot.
    batches: u32,
    /// Blocks they held.
    blocks: u32,
    /// Lines they put on the card.
    lines: u32,
    /// What they put on the card, KiB.
    kilobytes: u32,
    /// Card calls that failed this boot.
    card_errors: u32,
    /// Flash reads that failed this boot.
    flash_errors: u32,
    /// How long since the saver last caught up with the writer, ms; None while no saver runs.
    behind_ms: Option<u32>,
    /// The least stack the saver thread has had left, bytes (of [`SAVER_STACK`]).
    stack_free: u32,
}

impl SaverReport {
    /// The saver's figures now; `view` is None before the bring-up scan has run.
    fn new(view: Option<&RingView>, recorder: &Recorder) -> SaverReport {
        let running = SAVER.running.load(Ordering::Relaxed) && recorder.saver_position().is_some();
        let load = |counter: &AtomicU32| counter.load(Ordering::Relaxed);
        SaverReport {
            running,
            card: view.and_then(|view| view.card).map(Card::name),
            same_card: view.is_some_and(|view| view.same_card),
            start: view.and_then(|view| view.saver_start).map(pair),
            batches: load(&SAVER.batches),
            blocks: load(&SAVER.blocks),
            lines: load(&SAVER.lines),
            kilobytes: load(&SAVER.kilobytes),
            card_errors: load(&SAVER.card_errors),
            flash_errors: load(&SAVER.flash_errors),
            behind_ms: running.then(|| uptime_ms().wrapping_sub(load(&SAVER.caught_up_ms))),
            stack_free: load(&SAVER.stack_free),
        }
    }
}

/// A queue's counters as plan B's `stats recorder` prints them: `{count, max, dropped}`.
#[derive(Serialize)]
struct QueueReport {
    /// Records queued now.
    count: u32,
    /// The most ever queued.
    max: u32,
    /// Records dropped.
    dropped: u32,
}

impl From<recorder_wire::Counters> for QueueReport {
    fn from(counters: recorder_wire::Counters) -> QueueReport {
        QueueReport {
            count: counters.count,
            max: counters.max,
            dropped: counters.dropped,
        }
    }
}

/// `stats recorder`: plan B's report (the session and the two queues' counters), plus the flash ring and the saver.
#[derive(Serialize)]
pub struct StatsReport {
    /// This boot's file prefix.
    session: &'static str,
    /// The telemetry records' counters.
    tlm: QueueReport,
    /// The log records' counters.
    log: QueueReport,
    /// The flash ring; None before the bring-up scan has run.
    ring: Option<RingReport>,
    /// The ring_saver.
    saver: SaverReport,
}

/// `inspect recorder`: plan B's properties (the session, the status line's period, the queues' sizes), plus the flash
/// ring and the saver.
#[derive(Serialize)]
pub struct InspectReport {
    /// This boot's file prefix.
    session: &'static str,
    /// The status line's period, ms (`update recorder {"stats_ms": n}` changes it).
    stats_ms: u32,
    /// Each producer ring's size in bytes, by [`Producer`] index.
    producer_bytes: Vec<usize>,
    /// The shared `misc` ring's size in bytes.
    misc_bytes: usize,
    /// The flash ring; None before the bring-up scan has run.
    ring: Option<RingReport>,
    /// The ring_saver.
    saver: SaverReport,
}

/// The `stats recorder` report; None without a recorder (it is then not among the objects).
pub fn stats_report() -> Option<StatsReport> {
    let recording = RECORDING.get()?;
    let recorder = recording.recorder;
    let stats = recorder.stats();
    Some(StatsReport {
        session: recording.session.as_str(),
        tlm: stats.telemetry.into(),
        log: stats.log.into(),
        ring: ring().map(|view| RingReport::new(view, recorder)),
        saver: SaverReport::new(ring(), recorder),
    })
}

/// The `inspect recorder` report; None without a recorder.
pub fn inspect_report() -> Option<InspectReport> {
    let recorder = get()?;
    let inspection = recorder.inspect();
    Some(InspectReport {
        session: inspection.session,
        stats_ms: inspection.stats_ms,
        producer_bytes: inspection.producer_bytes,
        misc_bytes: inspection.misc_bytes,
        ring: ring().map(|view| RingReport::new(view, recorder)),
        saver: SaverReport::new(ring(), recorder),
    })
}

/// The recorder's probe, the ring gate (controller::ring_gate): the flash ring can take a flight once its bring-up
/// scan has run, its eraser runs and the eraser holds its reserve erased ahead of the writer. Logged as plan B's
/// recorder probe logs (tasks/recorder.py:30-48): `recorder :: probe: recorder rings ...`, then `probe: rings ok (<the
/// figures judged>)` or `probe FAILED: recorder rings: <why>`. It walks the block table as `health` does, but never
/// moves health's backlog trend ([`degraded`]).
///
/// Returns: None without a recorder (nothing to probe); else None when the ring can take a flight, or the verdict.
pub fn probe() -> Option<Option<String>> {
    let recorder = get()?;
    log::info!("{NAME} :: probe: recorder rings ...");
    let figures = ring().map(|view| {
        let erased_ahead = view.table.figures(recorder.saver_position()).erased_ahead;
        (erased_ahead, view.table.geometry().reserve_blocks())
    });
    let (erased_ahead, reserve_blocks) = figures.unwrap_or_default();
    let eraser_running = ERASER_RUNNING.load(Ordering::Acquire);
    let verdict = controller::ring_gate(figures.is_some(), eraser_running, erased_ahead, reserve_blocks);
    match &verdict {
        None => log::info!(
            "{NAME} :: probe: rings ok ({erased_ahead} blocks erased ahead of the writer, the reserve {reserve_blocks})"
        ),
        Some(why) => log::warn!("{NAME} :: probe FAILED: {why}"),
    }
    Some(verdict)
}

/// The flash-to-SD backlog's trend from one `health` read to the next, for `saver-behind` (flash_ring's
/// [`BacklogTrend`]: a window of a minute at a time, judged by its lowest backlog, since a slow card's every commit
/// drops the backlog while it grows across the minute). Only `health` takes it, on the executor, so the lock is never
/// contended, and no producer comes near it.
static BACKLOG: Mutex<BacklogTrend> = Mutex::new(BacklogTrend::new());

/// The flash ring's degraded tokens for `health` (spec "The ring_eraser and newest wins", Degraded tokens):
/// * `ring-unformatted`: the eraser is formatting the foreign slots of a first boot;
/// * `ring-low`: fewer erased blocks ahead of the writer than the ~1 % reserve: the eraser is not keeping up;
/// * `ring-full`: newest wins erased an unsaved block within the last minute ([`RING_FULL_HOLD_MS`]);
/// * `saver-behind`: a card is mounted, and over a minute the backlog did not fall and the saver never caught up
///   ([`BacklogTrend::behind`]): a dead card, a card slower than the writer, a saver that never started.
///
/// Nothing before the bring-up scan has run. Call it from `health` alone: it is where the backlog's trend is followed,
/// from one health read to the next.
pub fn degraded() -> Vec<&'static str> {
    let (Some(view), Some(recorder)) = (ring(), get()) else {
        return Vec::new();
    };
    let now_ms = uptime_ms();
    let figures = view.table.figures(recorder.saver_position());
    let mut tokens = Vec::new();
    if view.unformatted && figures.foreign > 0 {
        tokens.push("ring-unformatted");
    }
    if figures.erased_ahead < view.table.geometry().reserve_blocks() {
        tokens.push("ring-low");
    }
    if figures.lost > 0 && now_ms.wrapping_sub(LOST_MS.load(Ordering::Relaxed)) < RING_FULL_HOLD_MS {
        tokens.push("ring-full");
    }
    if view.card.is_some() {
        let caught_up_ms = SAVER
            .running
            .load(Ordering::Relaxed)
            .then(|| SAVER.caught_up_ms.load(Ordering::Relaxed));
        let mut trend = BACKLOG.lock().unwrap_or_else(PoisonError::into_inner);
        if trend.behind(figures.unsaved, caught_up_ms, now_ms) {
            tokens.push("saver-behind");
        }
    }
    tokens
}
