//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The error policy's latches (doc/skills.md "Error policy"): a failure that comes back every tick is logged once a
//! run, the first of the run, and the run ends at the next success. [`LogOnce`] is the latch itself; [`Rows`] is a
//! stream that uses it for its refused rows. The recorder counts every refused row (`tlm.dropped`); the log says only
//! the first of a run.

use crate::{Cell, Telemetry};

/// A failure's log latch: open until the first failure of a run, shut until the run ends.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LogOnce {
    /// A run of failures is on: its first was logged.
    shut: bool,
}

impl LogOnce {
    /// A failure: true when it is the first of its run, the one to log.
    pub fn first(&mut self) -> bool {
        !core::mem::replace(&mut self.shut, true)
    }

    /// A success: the run of failures ended, and the next failure logs again.
    pub fn clear(&mut self) {
        self.shut = false;
    }

    /// Whether a run of failures is on.
    pub fn on(&self) -> bool {
        self.shut
    }
}

/// A stream as a run records it: its [`Telemetry`] (None when the board records nothing, or the registry has no such
/// stream) and the latch of its refused rows, logged as `<name> :: <label> <overflow>`.
pub struct Rows<'r> {
    /// The run's name, the log line's first word.
    name: &'static str,
    /// What the line calls the stream (`telemetry`, `record`, `vetoes`).
    label: &'static str,
    /// The stream, when it records.
    telemetry: Option<Telemetry<'r>>,
    /// The refusals' latch.
    refusing: LogOnce,
    /// Whether an accepted row ends a run of refusals (plan B's runs), or the first refusal is the only one ever logged
    /// (plan B's sequencer, whose `note` is never re-armed).
    rearms: bool,
}

impl<'r> Rows<'r> {
    /// The rows of the run `name`, its stream called `label` in the log; a refusal logs again after an accepted row.
    pub fn new(name: &'static str, label: &'static str, telemetry: Option<Telemetry<'r>>) -> Rows<'r> {
        Rows {
            name,
            label,
            telemetry,
            refusing: LogOnce::default(),
            rearms: true,
        }
    }

    /// As [`Rows::new`], but only the first refusal ever is logged.
    pub fn logged_once(mut self) -> Rows<'r> {
        self.rearms = false;
        self
    }

    /// Whether the stream records at all.
    pub fn recording(&self) -> bool {
        self.telemetry.is_some()
    }

    /// Queue `row` at `now_us` (nothing when the stream does not record). Returns whether the row was refused and the
    /// refusal is the one logged.
    pub fn push(&mut self, now_us: u64, row: &[Cell]) -> bool {
        let Some(telemetry) = &mut self.telemetry else {
            return false;
        };
        match telemetry.push(now_us, row) {
            Ok(()) => {
                if self.rearms {
                    self.refusing.clear();
                }
                false
            }
            Err(overflow) => {
                let first = self.refusing.first();
                if first {
                    log::warn!("{} :: {} {overflow}", self.name, self.label);
                }
                first
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::LogOnce;

    #[test]
    fn the_latch_logs_the_first_failure_of_each_run() {
        let mut latch = LogOnce::default();
        assert!(!latch.on());
        assert!(latch.first(), "the first failure logs");
        assert!(!latch.first() && !latch.first(), "the rest of the run is quiet");
        assert!(latch.on());
        latch.clear();
        assert!(!latch.on());
        assert!(latch.first(), "a new run logs again");
        // NEGATIVE: clearing a latch that is open changes nothing
        let mut open = LogOnce::default();
        open.clear();
        assert!(open.first());
    }
}
