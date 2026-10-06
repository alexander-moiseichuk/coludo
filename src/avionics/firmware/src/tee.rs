//! The log stream's copy for the CC `log <ms>` command (src/glider recorder `_TeeSink` + `cc_logs`), and the logger
//! that feeds it: every `log` record still goes to the console as before, and while an operator window is open a
//! copy goes into a bounded buffer the next `log` call collects.
//!
//! Poll model: `log 1000` returns the lines buffered since the last call and keeps the window open 1000 ms more;
//! `log 0` stops. A window nobody renews lapses, and the next record then discards the buffer, so a lost link
//! cannot grow memory. Best effort: a full buffer drops and counts, it never blocks a logging thread.
//!
//! Temporary home: the flash-ring writer takes the tees over with the recorder (port plan M3).

use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};

use esp_idf_svc::log::{EspIdfLogFilter, EspIdfLogger};
use esp_idf_svc::sys;

/// Lines one window may hold: 1000 ms at a busy 1 kHz of log lines; past it the tee drops and counts.
const CAPACITY: usize = 1024;

/// One poll-model copy of a stream.
pub struct Tee {
    /// End of the open window, in uptime milliseconds (wrapping u32: rv32 has no 64-bit atomics); 0 = closed.
    deadline_ms: AtomicU32,
    /// The lines buffered since the last drain.
    lines: Mutex<VecDeque<String>>,
    /// Lines the full buffer (or a busy lock) discarded since the last drain.
    dropped: AtomicU32,
}

/// The log stream's tee.
pub static LOG: Tee = Tee {
    deadline_ms: AtomicU32::new(0),
    lines: Mutex::new(VecDeque::new()),
    dropped: AtomicU32::new(0),
};

impl Tee {
    /// Copy one line in while the window is open; `line` is only built then.
    pub fn push(&self, line: impl FnOnce() -> String) {
        let deadline = self.deadline_ms.load(Ordering::Acquire);
        if deadline == 0 {
            return;
        }
        if (uptime_ms().wrapping_sub(deadline) as i32) > 0 {
            // nobody renewed the window: close it and let the buffer go -- only the window read above, so a drain
            // that renewed it meanwhile keeps its new window
            let closed = self
                .deadline_ms
                .compare_exchange(deadline, 0, Ordering::AcqRel, Ordering::Acquire)
                .is_ok();
            if let (true, Ok(mut lines)) = (closed, self.lines.try_lock()) {
                lines.clear();
            }
            return;
        }
        match self.lines.try_lock() {
            Ok(mut lines) if lines.len() < CAPACITY => lines.push_back(line()),
            _ => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// The lines buffered since the last call and how many were dropped; keeps the window open `duration_ms` more
    /// (0 or less closes it).
    pub fn drain(&self, duration_ms: i64) -> (Vec<String>, u32) {
        let deadline = if duration_ms > 0 {
            // a window longer than the wrap would read as lapsed: cap it at a day; 0 is the closed mark
            uptime_ms().wrapping_add(duration_ms.min(86_400_000) as u32).max(1)
        } else {
            0
        };
        // renewed under the lines lock, so a lapse closing the old window cannot clear lines of the new one
        let lines: Vec<String> = self
            .lines
            .lock()
            .map(|mut lines| {
                self.deadline_ms.store(deadline, Ordering::Release);
                lines.drain(..).collect()
            })
            .unwrap_or_default();
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        (lines, dropped)
    }
}

/// Milliseconds since boot, wrapping at 2^32 (49 days).
fn uptime_ms() -> u32 {
    (uptime_microseconds() / 1000) as u32
}

/// Microseconds since boot.
fn uptime_microseconds() -> i64 {
    // SAFETY: a plain read of the monotonic system timer.
    unsafe { sys::esp_timer_get_time() }
}

/// The console logger IDF's own settings filter (esp-idf-svc's default).
static CONSOLE: EspIdfLogger<EspIdfLogFilter> = EspIdfLogger::new(EspIdfLogFilter::new());

/// The `log` crate's logger: the console as before, plus the tee.
struct Logger;

impl log::Log for Logger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        log::Log::enabled(&CONSOLE, metadata)
    }

    fn log(&self, record: &log::Record) {
        log::Log::log(&CONSOLE, record);
        if log::Log::enabled(&CONSOLE, record.metadata()) {
            // the recorder.log line shape, `<uptime_us> <source> :: <message>`; our messages carry their
            // `source :: ` already, the rest get the module's last path segment
            LOG.push(|| {
                let message = record.args().to_string();
                if message.contains(" :: ") {
                    format!("{} {message}", uptime_microseconds())
                } else {
                    let source = record.target().rsplit("::").next().unwrap_or("firmware");
                    format!("{} {source} :: {message}", uptime_microseconds())
                }
            });
        }
    }

    fn flush(&self) {}
}

/// The logger, once.
static LOGGER: Logger = Logger;

/// Route the `log` crate through the console and the tee (instead of `EspLogger::initialize_default`).
pub fn install() {
    if log::set_logger(&LOGGER).is_ok() {
        CONSOLE.filter().initialize();
        // esp-idf-svc logs every NVS handle it drops at INFO; the credential store opens one per join attempt
        let _ = CONSOLE
            .filter()
            .set_target_level("esp_idf_svc::nvs", log::LevelFilter::Warn);
    }
}
