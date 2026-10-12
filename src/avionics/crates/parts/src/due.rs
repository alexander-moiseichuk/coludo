//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! [`Due`]: when a polled run reads next.

/// When a polled run is due: every period, on its own grid, a stalled run starting a new grid (a burst never
/// follows a gap).
#[derive(Clone, Copy)]
pub(crate) struct Due {
    /// The period, microseconds.
    period_us: u64,
    /// When the next read is due, microseconds.
    next_us: u64,
}

impl Due {
    /// Every `period_ms`, the first read at once.
    pub(crate) fn every(period_ms: u32) -> Due {
        Due {
            period_us: u64::from(period_ms.max(1)) * 1000,
            next_us: 0,
        }
    }

    /// Whether a read is due at `now_us`; when it is, the next one is set a period on.
    pub(crate) fn take(&mut self, now_us: u64) -> bool {
        if now_us < self.next_us {
            return false;
        }
        self.next_us = (self.next_us + self.period_us).max(now_us + self.period_us / 2);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_fall_on_the_grid() {
        let mut due = Due::every(20);
        assert!(due.take(0), "the first read at once");
        assert!(!due.take(19_999));
        assert!(due.take(20_000));
        // a little late: the grid holds
        assert!(due.take(40_500));
        assert!(!due.take(59_999));
        assert!(due.take(60_000));
    }

    #[test]
    fn a_stall_starts_a_new_grid_without_a_burst() {
        let mut due = Due::every(20);
        assert!(due.take(0));
        // 100 ms late: one read now, the next half a period on, never five at once
        assert!(due.take(120_000));
        assert!(!due.take(120_001));
        assert!(due.take(130_000));
    }

    #[test]
    fn a_zero_period_is_one_millisecond() {
        let mut due = Due::every(0);
        assert!(due.take(0));
        assert!(!due.take(999));
        assert!(due.take(1000));
    }
}
