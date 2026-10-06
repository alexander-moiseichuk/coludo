//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's wall clock: Unix seconds (UTC) from the system time, which survives a software reset (IDF keeps it
//! in RTC memory) but not a power cycle.
//!
//! A cold board starts the clock at 2000-01-01 plus its uptime, the way MicroPython's RTC does: the hub reads a
//! whoami `epoch` inside 2000 as "unset" and sets it from its own clock with `update mission {epoch}`, in SETTING
//! only (src/control/server.py `_sync_clock`). A board left at 1970 would never be synced.

use esp_idf_svc::sys;

/// 2000-01-01T00:00:00Z in Unix seconds: a clock before it was never set (src/glider mission `_EPOCH_OFFSET`).
pub const UNSET_BEFORE: i64 = 946_684_800;

/// Start the clock at 2000-01-01 plus the uptime when nothing set it since power-on.
pub fn start() {
    if epoch() < UNSET_BEFORE {
        // SAFETY: a plain read of the monotonic system timer.
        let uptime_seconds = unsafe { sys::esp_timer_get_time() } / 1_000_000;
        set(UNSET_BEFORE + uptime_seconds);
    }
}

/// Now, in Unix seconds.
pub fn epoch() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |since| since.as_secs() as i64)
}

/// Set the clock to `epoch` Unix seconds.
pub fn set(epoch: i64) {
    let time = sys::timeval {
        tv_sec: epoch as sys::time_t,
        tv_usec: 0,
    };
    // SAFETY: settimeofday reads the struct it is handed; no time zone.
    unsafe { sys::settimeofday(&time, core::ptr::null()) };
}

/// Now as `YYYY-MM-DDTHH:MM:SS` (UTC), what the dashboard shows.
pub fn text() -> String {
    mission::clock_text(epoch())
}
