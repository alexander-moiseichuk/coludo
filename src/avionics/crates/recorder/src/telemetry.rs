//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A typed telemetry stream (src/glider recorder.py `Telemetry`; doc/specs/avionics-port.md "Producer side"),
//! owned by its producer thread.
//!
//! The first push queues the stream's HEADER record and then its row, whatever the decimation. After that a row
//! goes out only when the stream's window has passed since the last row that did: the stream's own
//! `decimate_us` when positive, else the global `recorder.telemetry_ms` (0 = every row), never for a negative
//! one. The global is read at each use, never latched: plan B once folded it in at construction, before the
//! recorder had read its config, and capped a 100 Hz accelerometer at 50 Hz while the config said uncapped.
//!
//! A hot producer asks [`Telemetry::due`] first and builds its row only then. Time is an argument (the
//! producer's own `esp_timer` read), never a clock read here. A push copies typed cells into the producer's
//! PSRAM ring and formats nothing.

use recorder_wire::Cell;

use crate::recorder::{Overflow, Recorder};
use crate::registry::StreamId;

/// One stream's producer state.
pub struct Telemetry<'r> {
    /// Where its records go.
    recorder: &'r Recorder,
    /// Its id.
    stream: StreamId,
    /// Its decimation setting, from the registry.
    decimate_us: i32,
    /// The uptime of the last row queued; None before the first.
    last_us: Option<u64>,
    /// Whether the HEADER record is queued.
    header_sent: bool,
}

impl<'r> Telemetry<'r> {
    /// The producer side of `stream`; None when the Recorder's registry has no such stream.
    pub fn new(recorder: &'r Recorder, stream: StreamId) -> Option<Telemetry<'r>> {
        let entry = recorder.registry().stream(stream)?;
        Some(Telemetry {
            recorder,
            stream,
            decimate_us: entry.decimate_us(),
            last_us: None,
            header_sent: false,
        })
    }

    /// The decimation window in us; None when the stream is never decimated.
    fn window_us(&self) -> Option<u64> {
        match self.decimate_us {
            own if own > 0 => Some(own as u64),
            0 => Some(u64::from(self.recorder.settings().telemetry_ms) * 1000),
            _ => None,
        }
    }

    /// Whether a row at `now_us` would be decimated away: its window has not passed since the last row.
    fn decimated(&self, now_us: u64) -> bool {
        match (self.last_us, self.window_us()) {
            (Some(last_us), Some(window_us)) => now_us.saturating_sub(last_us) < window_us,
            _ => false,
        }
    }

    /// Whether a push at `now_us` would queue a row, so a hot producer builds its row only then. False while no
    /// writer runs: with nowhere to go, a stream is not due (plan B's rule, which kept a recorder-less bench from
    /// paying for a raised and caught error every tick).
    pub fn due(&self, now_us: u64) -> bool {
        self.recorder.running() && !self.decimated(now_us)
    }

    /// Queue `row` at `now_us`, the stream's HEADER first on the first push. A decimated row is skipped, Ok.
    ///
    /// Errors: [`Overflow`] when the header or the row cannot be queued (counted in `tlm.dropped`): telemetry is
    /// evidence, so the caller learns of it. A header that failed goes out with the next push, and a row that
    /// failed does not start a new window.
    pub fn push(&mut self, now_us: u64, row: &[Cell]) -> Result<(), Overflow> {
        if !self.header_sent {
            self.recorder.push_header(self.stream, now_us)?;
            self.header_sent = true;
        }
        if self.decimated(now_us) {
            return Ok(());
        }
        self.recorder.push_row(self.stream, now_us, row)?;
        self.last_us = Some(now_us);
        Ok(())
    }
}
