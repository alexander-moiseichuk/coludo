//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's monotonic clock: microseconds since boot (esp_timer), the one place that reads it.

/// Microseconds since boot, from esp_timer: monotonic, never negative, safe from any context (an ISR included).
#[inline]
pub fn uptime_us() -> u64 {
    // SAFETY: a plain read of the monotonic system timer.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }
}
