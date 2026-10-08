//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! Every log line into recorder.log (doc/specs/avionics-port.md "Producer side": `Recorder::log`), next to the
//! console as before: the `log` crate's records from Rust, and IDF's own ESP_LOGx lines from C through a vprintf
//! hook (components/log_hook). From recorder.log a line reaches the flash ring and, while an operator window is
//! open, the CC `log` tee (commands/streaming.rs).
//!
//! A recorder.log line reads `<uptime_us> <descriptor> :: <message>`. Our own messages carry their `descriptor :: `
//! already (`ring :: scanned ...`); any other gets the last segment of its module path, and an IDF line its tag.
//!
//! The logging path never blocks and allocates at most one bounded string (error policy: logs are best effort). A
//! message with arguments is formatted into a string cut at [`LINE_BYTES`] (the recorder cuts a message at 1 KiB
//! anyway); a plain one is taken as it is. The text is on the heap, not the stack, because a `log` call can run on
//! a task with a tight stack (IDF's system event task has 2.3 KiB). `Recorder::log` then copies it into the thread's
//! own PSRAM ring, or `try_lock`s the shared `misc` one: a full or busy ring drops the line and counts it. Lines
//! logged before the recorder opens reach the console only; main opens it right after the board config, so that is
//! the boot count and the config load.
//!
//! A line logged on the flight core (core 1: the control thread, the bus owners) never prints in place: a console
//! write blocks once the UART's transmit buffer fills (115200 baud drains ~11 KB a second, and a CC reply can be
//! kilobytes), and a blocked control tick ran 4.5 ms late (M6, measured). It goes into a bounded FreeRTOS queue with
//! a non-blocking send instead, dropped and counted when the queue is full, and a printer thread on core 0 writes it
//! out. Its recorder.log copy is unchanged: the thread's own ring, lock-free. The supervisor's lines take the same
//! way ([`queue_this_task`]): it feeds the task watchdog, and a console write waits out a whole reply (IDF's UART
//! VFS holds its write lock for a write's whole buffer, and a `tlm` drain can be hundreds of KB, tens of seconds at
//! 115200 baud), so a line in place would let a healthy board's watchdog fire.
//!
//! A line logged to the [`CONSOLE_ONLY`] target reaches the console (and the panic capture) and never recorder.log.

use core::fmt::Write;
use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};
use std::io::Write as _;
use std::time::{Duration, Instant};

use esp_idf_svc::log::{EspIdfLogFilter, EspIdfLogger};
use esp_idf_svc::sys;

/// The longest log text kept for recorder.log, descriptor and message together, bytes: the recorder's own message
/// limit (1 KiB). The console prints the whole line regardless.
const LINE_BYTES: usize = 1024;

/// The core the flight threads run on: their lines go through the printer's queue, never straight to the console.
const FLIGHT_CORE: usize = 1;

/// The log target whose lines reach the console only: never recorder.log, so neither the recording nor the CC `log`
/// tee, which reads recorder.log. Plan B printed its Controller's lines to the console only (main.py:75, 101), and
/// the tools find the stage events in a recording by the text `stage ->` (tools/flight_kpi.py, flight_report.py,
/// flight_video.py), which must stay the stage machine's: so the controller's `controller :: stage -> X` logs here
/// (doc/specs/avionics-port.md §4.4).
pub const CONSOLE_ONLY: &str = "console";

/// A queued console line's room, bytes: two of them carry the length, the rest the line, cut at a character
/// boundary.
const QUEUED_LINE_BYTES: usize = 200;

/// How many queued lines wait for the printer at most (PSRAM: 6.4 KB); a full queue drops the line.
const QUEUED_LINES: u32 = 32;

/// The printer thread's stack (PSRAM: a slow core-0 thread that never calls esp_mmu_map/unmap).
const PRINTER_STACK: usize = 4 << 10;

/// The printer's queue, once [`install`] made it (null before: the flight core's lines print in place then, which
/// is only the boot, before any flight thread runs).
static QUEUE: AtomicPtr<sys::QueueDefinition> = AtomicPtr::new(core::ptr::null_mut());

/// Queued lines the full queue dropped (their recorder.log copies were kept).
static DROPPED: AtomicU32 = AtomicU32::new(0);

/// The one core-0 task whose lines go through the printer's queue as the flight core's do, null before
/// [`queue_this_task`]: the supervisor.
static QUEUED_TASK: AtomicPtr<core::ffi::c_void> = AtomicPtr::new(core::ptr::null_mut());

/// Lines handed to the printer's queue: counted before each send, and taken back when the queue refuses it.
static SENT: AtomicU32 = AtomicU32::new(0);

/// Lines the printer wrote out.
static PRINTED: AtomicU32 = AtomicU32::new(0);

/// How often [`wait_for_printer`] looks.
const PRINTER_POLL: Duration = Duration::from_millis(2);

/// Queued console lines (the flight core's and the supervisor's) dropped because the printer's queue was full.
pub fn console_dropped() -> u32 {
    DROPPED.load(Ordering::Relaxed)
}

/// The core this code runs on.
fn current_core() -> usize {
    let core: usize;
    // SAFETY: reads the hart id CSR, nothing else.
    unsafe { core::arch::asm!("csrr {}, mhartid", out(reg) core) };
    core
}

/// From now on the calling task's lines reach the console through the printer's queue, never in place: the
/// supervisor's, which feeds the task watchdog and so must never wait on the console.
pub fn queue_this_task() {
    // SAFETY: a plain read of this task's own handle; the supervisor never ends.
    QUEUED_TASK.store(unsafe { sys::xTaskGetCurrentTaskHandle() }.cast(), Ordering::Relaxed);
}

/// Whether the calling task's console lines go through `queue` (null before the printer runs): the flight core's,
/// and the [`queue_this_task`] task's.
fn queued(queue: sys::QueueHandle_t) -> bool {
    if queue.is_null() {
        return false;
    }
    if current_core() == FLIGHT_CORE {
        return true;
    }
    // SAFETY: a plain read of this task's own handle.
    let task = unsafe { sys::xTaskGetCurrentTaskHandle() };
    task.cast::<core::ffi::c_void>() == QUEUED_TASK.load(Ordering::Relaxed)
}

/// Waits until the printer wrote out every line queued before this call, for at most `limit`: a line logged right
/// before a restart then reaches the console. At once without a printer.
pub fn wait_for_printer(limit: Duration) {
    let sent = SENT.load(Ordering::Acquire);
    let started = Instant::now();
    while (PRINTED.load(Ordering::Acquire).wrapping_sub(sent) as i32) < 0 && started.elapsed() < limit {
        std::thread::sleep(PRINTER_POLL);
    }
}

/// The mark a queued line cut short ends with.
const CUT: &str = " [...]";

/// Queues `line` for the printer without waiting; a full queue drops it, counted. A line longer than the slot is cut
/// and marked.
fn enqueue(queue: sys::QueueHandle_t, line: &str) {
    let room = QUEUED_LINE_BYTES - 2;
    let cut = line.len() > room;
    let mut taken = if cut { room - CUT.len() } else { line.len() };
    while !line.is_char_boundary(taken) {
        taken -= 1;
    }
    let mut item = [0u8; QUEUED_LINE_BYTES];
    item[2..2 + taken].copy_from_slice(&line.as_bytes()[..taken]);
    if cut {
        item[2 + taken..2 + taken + CUT.len()].copy_from_slice(CUT.as_bytes());
        taken += CUT.len();
    }
    item[..2].copy_from_slice(&(taken as u16).to_le_bytes());
    // counted first: a wait for the printer that reads it then also waits for this line
    SENT.fetch_add(1, Ordering::AcqRel);
    // SAFETY: a live queue of QUEUED_LINE_BYTES items; the item is copied in; 0 ticks: never waits.
    if unsafe { sys::xQueueGenericSend(queue, item.as_ptr().cast(), 0, 0) } != 1 {
        SENT.fetch_sub(1, Ordering::AcqRel);
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

/// The printer thread (core 0, priority 3): each queued line to the console, forever.
fn print(queue: usize) {
    let queue = queue as sys::QueueHandle_t;
    let mut item = [0u8; QUEUED_LINE_BYTES];
    loop {
        // SAFETY: a live queue of QUEUED_LINE_BYTES items, received into a buffer of that size.
        if unsafe { sys::xQueueReceive(queue, item.as_mut_ptr().cast(), u32::MAX) } != 1 {
            continue;
        }
        let length = usize::from(u16::from_le_bytes([item[0], item[1]])).min(QUEUED_LINE_BYTES - 2);
        let mut stdout = std::io::stdout().lock();
        let _ = stdout.write_all(&item[2..2 + length]);
        let _ = stdout.write_all(b"\n");
        PRINTED.fetch_add(1, Ordering::AcqRel);
    }
}

/// Makes the printer's queue and spawns the printer on core 0.
fn start_printer() -> anyhow::Result<()> {
    use esp_idf_svc::hal::cpu::Core;
    use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
    // SAFETY: a plain queue allocation, in PSRAM.
    let queue = unsafe { sys::xQueueCreateWithCaps(QUEUED_LINES, QUEUED_LINE_BYTES as u32, sys::MALLOC_CAP_SPIRAM) };
    anyhow::ensure!(!queue.is_null(), "no memory for the console queue");
    ThreadSpawnConfiguration {
        name: Some(c"log_print"),
        stack_size: PRINTER_STACK,
        priority: 3,
        pin_to_core: Some(Core::Core0),
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    }
    .set()?;
    let address = queue as usize;
    let spawned = std::thread::Builder::new()
        .stack_size(PRINTER_STACK)
        .spawn(move || print(address));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    QUEUE.store(queue, Ordering::Release);
    Ok(())
}

/// The console logger with IDF's own settings filter (esp-idf-svc's default). It prints straight to stdout, never
/// through IDF's vprintf, so a Rust line does not come back through the hook a second time.
static CONSOLE: EspIdfLogger<EspIdfLogFilter> = EspIdfLogger::new(EspIdfLogFilter::new());

/// The `log` crate's logger: the console as before, plus recorder.log.
struct Logger;

impl log::Log for Logger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        log::Log::enabled(&CONSOLE, metadata)
    }

    fn log(&self, record: &log::Record) {
        let queue = QUEUE.load(Ordering::Acquire);
        let queued = queued(queue);
        // the console filter locks a mutex and IDF's level table: for a queued line the global level decides,
        // lock-free (the per-target exceptions are for other core-0 targets)
        let enabled = if queued {
            record.level() <= log::max_level()
        } else {
            log::Log::enabled(&CONSOLE, record.metadata())
        };
        if !enabled {
            return;
        }
        if !queued {
            log::Log::log(&CONSOLE, record);
        }
        let mut line = Line::default();
        let text = match record.args().as_str() {
            Some(text) => text,
            None => {
                let _ = write!(line, "{}", record.args()); // a full line cuts the text: Line never fails
                line.0.as_str()
            }
        };
        // the panic log keeps the newest lines before a panic (components/panic_capture), shaped as IDF's and on its
        // clock, so the two kinds of line interleave in order
        let mut tail = Line::default();
        let level = record.level().as_str().as_bytes()[0] as char;
        // SAFETY: a plain read of IDF's log clock.
        let timestamp = unsafe { sys::esp_log_timestamp() };
        let _ = write!(tail, "{level} ({timestamp}) {}: {text}", record.target());
        // SAFETY: the line's bytes, valid for the call; panic_capture copies them.
        unsafe { sys::panic_capture_log(tail.0.as_ptr().cast(), tail.0.len()) };
        // the same shape is the console's: the printer writes it for a queued line
        if queued {
            enqueue(queue, &tail.0);
        }
        if record.target() == CONSOLE_ONLY {
            return;
        }
        let Some(recorder) = crate::recorder::get() else {
            return;
        };
        match text.split_once(" :: ") {
            Some((descriptor, message)) => recorder.log(descriptor, message),
            None => recorder.log(record.target().rsplit("::").next().unwrap_or("firmware"), text),
        };
    }

    fn flush(&self) {}
}

/// The logger, once.
static LOGGER: Logger = Logger;

/// Route the `log` crate through the console and recorder.log, and IDF's C log lines into recorder.log too (instead
/// of `EspLogger::initialize_default`). Call first thing in main: lines before the recorder opens reach the console.
pub fn install() {
    if log::set_logger(&LOGGER).is_ok() {
        CONSOLE.filter().initialize();
        // esp-idf-svc logs every NVS handle it drops at INFO; the credential store opens one per join attempt
        let _ = CONSOLE
            .filter()
            .set_target_level("esp_idf_svc::nvs", log::LevelFilter::Warn);
    }
    // SAFETY: installs a plain C callback once; it records the line and never logs, blocks or allocates.
    unsafe { sys::log_hook_install(Some(idf_line)) };
    if let Err(error) = start_printer() {
        log::warn!("logging :: the flight core's lines print in place: {error}");
    }
}

/// A formatted log text, cut to [`LINE_BYTES`] so a runaway message cannot grow the heap.
#[derive(Default)]
struct Line(String);

impl Write for Line {
    /// Appends what fits, cut at a character boundary; never an error, so the formatting runs to its end.
    fn write_str(&mut self, text: &str) -> core::fmt::Result {
        let mut taken = text.len().min(LINE_BYTES - self.0.len());
        while !text.is_char_boundary(taken) {
            taken -= 1;
        }
        self.0.push_str(&text[..taken]);
        Ok(())
    }
}

/// One IDF log line from components/log_hook, `I (1234) wifi: connected\n` (LOG_VERSION 1, maybe in ANSI colour):
/// recorded as `wifi :: connected`. A line of another shape (a bare printf through the hook) is kept whole under
/// `idf`.
///
/// Runs on whichever task logged, IDF's own with tight stacks among them, so it stays shallow: no allocation, no
/// copy, no logging (a log here would re-enter the hook, which then skips the line).
unsafe extern "C" fn idf_line(text: *const core::ffi::c_char, length: usize) -> bool {
    // SAFETY: the hook hands over `length` bytes of its line buffer, valid for this call.
    let bytes = unsafe { core::slice::from_raw_parts(text.cast::<u8>(), length) };
    let line = plain(bytes);
    // a cut line may end inside a character: keep the whole ones
    let line = match core::str::from_utf8(line) {
        Ok(line) => line,
        Err(error) => core::str::from_utf8(&line[..error.valid_up_to()]).unwrap_or_default(),
    };
    // a queued task's console copy goes to the printer: the hook then prints nothing in place
    let queue = QUEUE.load(Ordering::Acquire);
    let queued = queued(queue);
    if queued && !line.is_empty() {
        enqueue(queue, line);
    }
    let Some(recorder) = crate::recorder::get() else {
        return queued;
    };
    if line.is_empty() {
        return queued;
    }
    let parsed = line
        .strip_prefix(['E', 'W', 'I', 'D', 'V'])
        .and_then(|rest| rest.strip_prefix(" ("))
        .and_then(|rest| rest.split_once(") "))
        .and_then(|(_, rest)| rest.split_once(": "));
    match parsed {
        Some((tag, message)) => recorder.log(tag, message),
        None => recorder.log("idf", line),
    };
    queued
}

/// The line without its end and its colour: IDF wraps a coloured line in `ESC [ 0 ; 3x m` ... `ESC [ 0 m` (only with
/// CONFIG_LOG_COLORS, off in sdkconfig.defaults) and ends it with `\n`.
fn plain(line: &[u8]) -> &[u8] {
    const ESCAPE: u8 = 0x1b;
    let mut line = line.trim_ascii_end();
    if let Some(rest) = line.strip_prefix(&[ESCAPE]) {
        line = rest
            .iter()
            .position(|&byte| byte == b'm')
            .map_or(rest, |end| &rest[end + 1..]);
    }
    if let Some(start) = line.iter().rposition(|&byte| byte == ESCAPE) {
        line = &line[..start];
    }
    line.trim_ascii_end()
}
