//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The panics logger (crates/panics, doc/specs/avionics-port.md "Panic log"): the board's last panics as tagged text
//! records in the `panic` partition, round-robin, the way Linux's mtdoops keeps oopses.
//! - A fatal panic (a panic, a watchdog, a brownout: the board reset): components/panic_capture keeps what the panic
//!   handler printed, and a Rust panic's message, in RTC RAM across the reset, and [`start`] writes it at the next boot
//!   with the OS up -- never from the panic handler. A watchdog or brownout reset with nothing captured is recorded
//!   too, a header alone.
//! - An oops (the board noticed something broken and went on): [`oops`] hands it to the panic log's own thread, which
//!   writes it; the caller never waits for the flash.
//!
//! Each fatal record is also logged at boot, so the session's recorder.log and CC's log stream carry it. Read the log
//! with `panic log` (a summary) or `GET /panic` on the file server (the records as text, newest first).

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::sync::{Mutex, MutexGuard, OnceLock, PoisonError};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use firmware::allocator::InternalBuffer;
use firmware::gcl::{self, Command, Gate, Reach};
use flash_ring::{BLOCK_BYTES, Flash};
use panics::{PanicLog, PanicLogError, Tags};

use crate::board;
use crate::recorder::Partition;

/// The partition's label.
const LABEL: &std::ffi::CStr = c"panic";

/// The resets that are a fatal panic, recorded even with nothing captured (board::reset_reason's names).
const FATAL_RESETS: [&str; 7] = [
    "panic",
    "interrupt-watchdog",
    "task-watchdog",
    "watchdog",
    "brownout",
    "power-glitch",
    "cpu-lockup",
];

/// The resets that a panic leads to: a capture names its cause by them, any other reset after it (a brownout or the
/// reset pin before the record was written) leaves the cause `panic`.
const PANIC_RESETS: [&str; 4] = ["panic", "interrupt-watchdog", "task-watchdog", "cpu-lockup"];

/// The line between a fatal panic's text and the log lines before it, in its record.
const TAIL_HEADING: &str = "\n--- the log before it, oldest first ---\n";

// the capture's text room is one slot's (components/panic_capture keeps its own copy of the number)
const _: () = assert!(sys::PANIC_CAPTURE_TEXT_BYTES == panics::TEXT_BYTES);

/// How many lines of a fatal panic's text the boot logs (the whole text is in the record).
const LOGGED_LINES: usize = 16;

/// How many oopses may wait for the panic log's thread; one more is dropped, counted.
const OOPS_QUEUE: usize = 8;

/// The panic log thread's stack: it formats a record and writes the flash.
const STACK_SIZE: usize = 6 << 10;

/// The partition's records, once [`start`] opened them.
static LOG: Mutex<Option<PanicLog<Bounced>>> = Mutex::new(None);

/// This boot's names, for an oops's tags.
static BOOT: OnceLock<Boot> = OnceLock::new();

/// Where [`oops`] hands its record.
static QUEUE: OnceLock<SyncSender<Oops>> = OnceLock::new();

/// Oopses dropped: the queue was full, or the panic log is not running.
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// This boot's names.
struct Boot {
    boot_id: u32,
    session: String,
    /// The config id: the saved config's at [`start`], the post-layout one from [`name_config`] on.
    config_id: Mutex<String>,
}

/// An oops on its way to the flash.
struct Oops {
    cause: String,
    text: String,
    uptime_ms: u32,
}

/// The panic partition, its reads bounced through internal RAM: the file server and the oops thread run on PSRAM
/// stacks, so their buffers are in PSRAM, and esp_flash_read reads into anything but internal RAM through a bounce it
/// allocates per call after walking the internal heap with interrupts masked. Programs and erases pass through.
struct Bounced(Partition);

impl Flash for Bounced {
    type Error = sys::EspError;

    /// Reads at most a block at a time into an internal buffer, then copies it out.
    fn read(&self, offset: u32, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let room = buffer.len().clamp(1, BLOCK_BYTES as usize);
        let no_memory = || sys::EspError::from(sys::ESP_ERR_NO_MEM as sys::esp_err_t).expect("an error code");
        let mut bounce = InternalBuffer::zeroed(room).ok_or_else(no_memory)?;
        for (index, chunk) in buffer.chunks_mut(room).enumerate() {
            let piece = &mut bounce[..chunk.len()];
            self.0.read(offset + (index * room) as u32, piece)?;
            chunk.copy_from_slice(piece);
        }
        Ok(())
    }

    fn program(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.0.program(offset, data)
    }

    fn erase_sector(&mut self, offset: u32) -> Result<(), Self::Error> {
        self.0.erase_sector(offset)
    }

    fn size(&self) -> u32 {
        self.0.size()
    }
}

/// The panic log, locked; a poisoned lock is taken over (a panic aborts here, so it never stays poisoned).
fn log() -> MutexGuard<'static, Option<PanicLog<Bounced>>> {
    LOG.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// The uptime in ms.
fn uptime_ms() -> u32 {
    // SAFETY: a plain read of the monotonic timer.
    (unsafe { sys::esp_timer_get_time() } / 1000) as u32
}

/// A C string field of the capture, as text.
fn field(bytes: &[core::ffi::c_char]) -> String {
    // SAFETY: panic_capture NUL-terminates every name within its array.
    unsafe { std::ffi::CStr::from_ptr(bytes.as_ptr()) }
        .to_string_lossy()
        .into_owned()
}

/// Opens the panic log and records what the last boot left; then names this boot for its own capture and installs the
/// Rust panic hook. Call it on main right after the recorder opens, so its lines are kept.
///
/// Args:
/// * `boot_id` - this boot's id.
/// * `session` - its session label.
/// * `config_id` - the saved board config's id: the layout has not rewritten it yet ([`name_config`] names the
///   rewritten one).
pub fn start(boot_id: u32, session: &str, config_id: &str) {
    let opened = Partition::by_label(LABEL)
        .ok_or_else(|| "the partition table has no `panic` partition".to_string())
        .and_then(|partition| PanicLog::open(Bounced(partition)).map_err(|error| format!("{error:?}")));
    match opened {
        Ok(log) => *self::log() = Some(log),
        Err(error) => log::error!("panic :: the panic log is not open, nothing is recorded this boot: {error}"),
    }
    if record_last_boot(boot_id) {
        // SAFETY: the previous boot's capture is in the log (or never can be): forget it
        unsafe { sys::panic_capture_clear() };
    }
    name_context(boot_id, session, config_id);
    let _ = BOOT.set(Boot {
        boot_id,
        session: session.to_string(),
        config_id: Mutex::new(config_id.to_string()),
    });
    install_hook();
}

/// Names the config this boot runs once the layout has rewritten it (plan B main.py:120-124: the post-layout id),
/// for this boot's capture and its oopses from here on. [`start`] named the saved config's id, since a panic in the
/// layout itself is that config's.
pub fn name_config(config_id: &str) {
    let Some(boot) = BOOT.get() else {
        return;
    };
    name_context(boot.boot_id, &boot.session, config_id);
    *boot.config_id.lock().unwrap_or_else(PoisonError::into_inner) = config_id.to_string();
}

/// Hands this boot's names to panic_capture, which copies them into a capture when one begins.
fn name_context(boot_id: u32, session: &str, config_id: &str) {
    let names = |text: &str| std::ffi::CString::new(text.replace('\0', "")).unwrap_or_default();
    let (session_name, firmware, config) = (names(session), names(board::FIRMWARE_VERSION), names(config_id));
    // SAFETY: NUL-terminated strings; panic_capture copies them.
    unsafe { sys::panic_capture_context(boot_id, session_name.as_ptr(), firmware.as_ptr(), config.as_ptr()) };
}

/// Writes the record of the last boot's fatal panic: when it reset in one of [`FATAL_RESETS`], or left a capture. Its
/// text is the capture's, then the log lines before it ([`TAIL_HEADING`]).
///
/// Returns whether the capture may be cleared: it is in the log (now, or by an earlier boot that was reset before it
/// could clear it), or no log can ever take it. A failed write keeps it for the next boot.
fn record_last_boot(boot_id: u32) -> bool {
    let reset = board::reset_reason();
    let mut capture = sys::panic_capture_t::default();
    // SAFETY: fills a plain struct; its text pointers stay valid until panic_capture_clear.
    let captured = unsafe { sys::panic_capture_pending(&mut capture) };
    if !captured && !FATAL_RESETS.contains(&reset) {
        return true;
    }
    let (panic_text, tail): (&[u8], &[u8]) = if captured {
        // SAFETY: `length` and `tail_length` bytes of RTC RAM that panic_capture keeps until it is cleared.
        unsafe {
            (
                std::slice::from_raw_parts(capture.text.cast::<u8>(), capture.length as usize),
                std::slice::from_raw_parts(capture.tail.cast::<u8>(), capture.tail_length as usize),
            )
        }
    } else {
        (&[], &[])
    };
    let mut text = panic_text.to_vec();
    if !tail.is_empty() {
        text.extend_from_slice(TAIL_HEADING.as_bytes());
        text.extend_from_slice(tail);
    }
    // the cause: the reset, unless a capture says a panic began it and another reset came before its record
    let cause = if !captured || PANIC_RESETS.contains(&reset) {
        reset
    } else {
        "panic"
    };
    let tags = Tags {
        fatal: true,
        cause: cause.to_string(),
        boot: if captured { capture.boot_id } else { 0 },
        session: if captured {
            field(&capture.session)
        } else {
            String::new()
        },
        uptime_ms: if captured { capture.uptime_ms } else { 0 },
        firmware: if captured {
            field(&capture.firmware)
        } else {
            String::new()
        },
        config: if captured {
            field(&capture.config)
        } else {
            String::new()
        },
        recorded: boot_id,
        intact: captured && capture.intact,
        ..Tags::default()
    };
    let mut log = log();
    // a boot reset after writing the record but before clearing the capture: the record is there already
    let recorded_before = log.as_ref().and_then(|log| log.newest()).is_some_and(|newest| {
        captured && newest.fatal && newest.boot == tags.boot && newest.uptime_ms == tags.uptime_ms
    });
    let (seq, clear) = match log.as_mut() {
        _ if recorded_before => ("already recorded".to_string(), true),
        Some(log) => match log.append(tags.clone(), &text) {
            Ok(written) => (written.seq.to_string(), true),
            Err(error) => {
                log::error!("panic :: the record was not written ({error:?}); its text follows here only");
                // a flash error may pass by the next boot; a record that can never fit never will
                ("-".to_string(), !matches!(error, PanicLogError::Flash(_)))
            }
        },
        None => ("-".to_string(), true),
    };
    drop(log);
    let boot = if tags.boot == 0 {
        "an unknown boot".to_string()
    } else {
        format!("boot {:06}", tags.boot)
    };
    log::error!(
        "panic :: {boot} ended in a {} (panic record {seq}, {} bytes captured, {} of log before it{})",
        tags.cause,
        panic_text.len(),
        tail.len(),
        if captured && !tags.intact { ", not intact" } else { "" }
    );
    for line in String::from_utf8_lossy(panic_text)
        .lines()
        .filter(|line| !line.trim().is_empty())
        .take(LOGGED_LINES)
    {
        log::error!("panic :: | {line}");
    }
    clear
}

/// The Rust panic hook: the message and its place into the capture, before abort() reaches the panic handler (whose
/// own text, the registers and the stack, follows it); then std's own hook prints it.
fn install_hook() {
    let default = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        // the FreeRTOS task's name: a thread spawned through IDF's spawn configuration has no std name
        // SAFETY: the running task's name, a NUL-terminated string FreeRTOS keeps in its control block.
        let task = unsafe { std::ffi::CStr::from_ptr(sys::pcTaskGetName(std::ptr::null_mut())) };
        let note = format!("Rust panic in task {}: {info}\n", task.to_string_lossy());
        // SAFETY: a byte slice valid for the call; panic_capture copies it.
        unsafe { sys::panic_capture_note(note.as_ptr().cast(), note.len()) };
        default(info);
    }));
}

/// Starts the panic log's thread (core 0, priority 2, PSRAM stack: it never calls esp_mmu_map/unmap, the one
/// runtime path that turns the cache off), which writes the oopses [`oops`] queues; until it runs they are dropped.
/// Call it after the camera's encoder reserved its internal block (the thread's control block is internal).
///
/// Errors: the thread would not spawn.
pub fn start_oops() -> anyhow::Result<()> {
    let (queue, oopses) = sync_channel(OOPS_QUEUE);
    ThreadSpawnConfiguration {
        name: Some(c"panic_log"),
        stack_size: STACK_SIZE,
        priority: 2,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(move || write_oopses(oopses));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    let _ = QUEUE.set(queue);
    Ok(())
}

/// The panic log thread: each oops into the log, forever.
fn write_oopses(oopses: Receiver<Oops>) {
    for oops in oopses {
        let Some(boot) = BOOT.get() else {
            DROPPED.fetch_add(1, Ordering::Relaxed);
            continue;
        };
        let tags = Tags {
            fatal: false,
            cause: oops.cause.clone(),
            boot: boot.boot_id,
            session: boot.session.clone(),
            uptime_ms: oops.uptime_ms,
            firmware: board::FIRMWARE_VERSION.to_string(),
            config: boot.config_id.lock().unwrap_or_else(PoisonError::into_inner).clone(),
            recorded: boot.boot_id,
            intact: true,
            ..Tags::default()
        };
        match log().as_mut().map(|log| log.append(tags, oops.text.as_bytes())) {
            Some(Ok(written)) => log::warn!("panic :: oops {} recorded (panic record {})", oops.cause, written.seq),
            Some(Err(error)) => log::error!("panic :: oops {} not recorded: {error:?}", oops.cause),
            None => {
                DROPPED.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// Records an oops: something broke and the board went on. Never waits: the record goes to the panic log's thread,
/// and when its queue is full (or the log is not running) the oops is dropped, counted (`panic log`'s `dropped`).
///
/// Args:
/// * `cause` - what happened, one word (`c6-link-parked`): the record's `cause` tag.
/// * `text` - what the record says about it.
pub fn oops(cause: &str, text: &str) {
    let record = Oops {
        cause: cause.to_string(),
        text: text.to_string(),
        uptime_ms: uptime_ms(),
    };
    let sent = QUEUE.get().map(|queue| queue.try_send(record));
    if !matches!(sent, Some(Ok(()))) {
        DROPPED.fetch_add(1, Ordering::Relaxed);
        if let Some(Err(TrySendError::Full(record))) = sent {
            log::warn!("panic :: oops {} dropped: the queue is full", record.cause);
        }
    }
}

/// The records as text, newest first, each followed by an empty line; only `seq`'s when given (empty when no record
/// has it). A record whose read fails is a line saying so. None when the log is not open.
pub fn texts(seq: Option<u32>) -> Option<Vec<u8>> {
    let log = log();
    let log = log.as_ref()?;
    let mut seqs: Vec<u32> = log.records().iter().map(|(_, tags)| tags.seq).collect();
    seqs.reverse();
    let mut text = Vec::new();
    for wanted in seqs
        .into_iter()
        .filter(|candidate| seq.is_none_or(|seq| seq == *candidate))
    {
        match log.text(wanted) {
            Ok(Some(record)) => {
                text.extend_from_slice(&record);
                if !record.ends_with(b"\n") {
                    text.push(b'\n');
                }
            }
            Ok(None) => {}
            Err(error) => text.extend_from_slice(format!("(panic record {wanted} unreadable: {error:?})\n").as_bytes()),
        }
        text.push(b'\n');
    }
    Some(text)
}

/// `panic log`: the records as one CC line, oldest first: `seq<N>=<fatal|oops>:<cause>:<boot>:<text bytes>`.
fn summary() -> String {
    let log = log();
    let Some(log) = log.as_ref() else {
        return cc_protocol::build("err", &["panic", "the panic log is not open (see the boot log)"], &[]);
    };
    let records = log.records();
    let mut values = vec![
        ("records".to_string(), records.len().to_string()),
        ("capacity".to_string(), log.capacity().to_string()),
        ("dropped".to_string(), DROPPED.load(Ordering::Relaxed).to_string()),
    ];
    values.extend(records.iter().map(|(_, tags)| {
        (
            format!("seq{}", tags.seq),
            format!(
                "{}:{}:{:06}:{}",
                if tags.fatal { "fatal" } else { "oops" },
                tags.cause,
                tags.boot,
                tags.length
            ),
        )
    }));
    let named: Vec<(&str, &str)> = values
        .iter()
        .map(|(key, value)| (key.as_str(), value.as_str()))
        .collect();
    cc_protocol::build("ok", &["panic", "log"], &named)
}

/// The panic log's console command (the Glider Command Language): `panic log`.
pub fn register_commands() {
    gcl::register(vec![Command {
        owner: "panic",
        words: "panic log",
        about: "the panic log's records, oldest first: seq<N>=<fatal|oops>:<cause>:<boot>:<bytes>; their text is \
                GET /panic",
        gate: Gate::Open,
        reach: Reach::Everywhere,
        parameters: &[],
        bare_words: &[],
        run: Box::new(|_| summary()),
    }]);
}
