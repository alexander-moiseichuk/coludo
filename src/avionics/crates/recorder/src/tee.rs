//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A CC tee (doc/specs/avionics-port.md "CC tees and the HTTP exits"; src/glider recorder.py `_TeeSink`): a
//! poll-model copy of the log or the telemetry lines for the CC `log [ms]` and `tlm [ms]` commands.
//!
//! The flash writer copies each line in, without its CRC wrapper, while the operator's window is open; the
//! command's executor takes the batch out. `drain(ms)` returns what was buffered since the last call and keeps
//! the window open `ms` more (0 or less closes it). A window nobody renews lapses: the writer marks it so at
//! its next line, and the next drain throws the stale batch away, so a lost link never fills the buffer with
//! lines from long ago. Best effort: a full buffer drops the line and counts it, never blocking the writer.
//!
//! A drain also throws away what it finds in a closed tee. The writer checks the window and then copies its
//! line, so a line whose check came just before a closing drain lands after that drain read the buffer: plan
//! B, single-threaded, never had such a line, and returned to the next session hours later it would read as
//! fresh.
//!
//! The deadline is a wrapping u32 of uptime milliseconds (the P4 has no 64-bit atomics), as the M1 firmware
//! tee keeps it: 0 means closed and 1 lapsed, so an open deadline is at least 2. The lines go through a byte
//! ring in PSRAM: the writer is its one producer, and the drain its one consumer, serialised by a lock that only
//! the executor side takes.

use std::string::String;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::vec::Vec;

use crate::byte_ring::{ByteRing, LENGTH_BYTES};

/// The deadline of a closed window.
const CLOSED: u32 = 0;

/// The deadline of a window the writer found lapsed: the next drain discards its batch.
const LAPSED: u32 = 1;

/// The longest window, a day: a longer one would read as lapsed once the u32 milliseconds wrap past it.
const LONGEST_WINDOW_MS: i64 = 86_400_000;

/// A tee record: the ring's length field, then the line's own length, then the line.
const TEE_HEAD_BYTES: usize = LENGTH_BYTES + 2;

/// What one drain returns.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TeeBatch {
    /// The lines buffered since the last drain, in order, without their line ends.
    pub lines: Vec<String>,
    /// The lines the full buffer dropped since the last drain: a gap the operator would not see otherwise.
    pub dropped: u32,
}

/// One tee.
pub(crate) struct Tee {
    /// The window's end in uptime ms (wrapping); [`CLOSED`] or [`LAPSED`] otherwise.
    deadline_ms: AtomicU32,
    /// Lines dropped since the last drain.
    dropped: AtomicU32,
    /// The buffered lines.
    ring: ByteRing,
    /// Held by a drain: the ring's consumer side, one caller at a time.
    consumer: Mutex<()>,
}

impl Tee {
    /// A closed tee over `buffer`.
    pub(crate) fn new(buffer: &'static mut [u8]) -> Tee {
        Tee {
            deadline_ms: AtomicU32::new(CLOSED),
            dropped: AtomicU32::new(0),
            ring: ByteRing::new(buffer),
            consumer: Mutex::new(()),
        }
    }

    /// Copies one line (the concatenation of `parts`) in while the window is open at `now_ms`, marks a lapsed
    /// window, and counts a line the full buffer drops.
    ///
    /// # Safety
    /// Only the flash writer calls it: it is the ring's one producer.
    pub(crate) unsafe fn push(&self, now_ms: u32, parts: &[&[u8]]) {
        let deadline = self.deadline_ms.load(Ordering::Acquire);
        if deadline <= LAPSED {
            return;
        }
        if now_ms.wrapping_sub(deadline) as i32 > 0 {
            // only the deadline read above: a drain that renewed the window meanwhile keeps its new one
            let _ = self
                .deadline_ms
                .compare_exchange(deadline, LAPSED, Ordering::AcqRel, Ordering::Acquire);
            return;
        }
        // SAFETY: the caller's contract.
        unsafe { self.store(parts) };
    }

    /// Copies one line (the concatenation of `parts`) into the buffer, whatever the window: [`Tee::push`] after
    /// its check. Counts a line the full buffer drops.
    ///
    /// # Safety
    /// As [`Tee::push`]: only the flash writer calls it.
    unsafe fn store(&self, parts: &[&[u8]]) {
        let length: usize = parts.iter().map(|part| part.len()).sum();
        let Ok(line_length) = u16::try_from(length) else {
            self.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        };
        let fill = |record: &mut [u8]| {
            record[LENGTH_BYTES..TEE_HEAD_BYTES].copy_from_slice(&line_length.to_le_bytes());
            let mut at = TEE_HEAD_BYTES;
            for part in parts {
                record[at..at + part.len()].copy_from_slice(part);
                at += part.len();
            }
        };
        // SAFETY: the writer is the one producer (the caller's contract).
        if !unsafe { self.ring.push(TEE_HEAD_BYTES + length, fill) } {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// The batch buffered since the last drain, then the window kept open `duration_ms` more from `now_ms` (0
    /// or less closes it). A batch whose window lapsed, or that a closed tee holds, is discarded, not returned.
    pub(crate) fn drain(&self, now_ms: u32, duration_ms: i64) -> TeeBatch {
        let _consumer = self.consumer.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        let previous = self.deadline_ms.swap(CLOSED, Ordering::AcqRel);
        let mut lines = Vec::new();
        let mut read = self.ring.tail();
        // SAFETY: the lock makes this the ring's one consumer, reading from its tail record by record.
        while let Some((start, record)) = unsafe { self.ring.record(read) } {
            if previous > LAPSED {
                let length = usize::from(u16::from_le_bytes([record[LENGTH_BYTES], record[LENGTH_BYTES + 1]]));
                let line = record.get(TEE_HEAD_BYTES..TEE_HEAD_BYTES + length).unwrap_or_default();
                lines.push(String::from_utf8_lossy(line).into_owned());
            }
            read = self.ring.after(start, record.len());
        }
        // SAFETY: as above; every line was copied out.
        unsafe { self.ring.release(read) };
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        if duration_ms > 0 {
            let deadline = now_ms.wrapping_add(duration_ms.min(LONGEST_WINDOW_MS) as u32);
            self.deadline_ms.store(deadline.max(LAPSED + 1), Ordering::Release);
        }
        TeeBatch { lines, dropped }
    }
}

#[cfg(test)]
mod tests {
    use super::{Tee, TeeBatch};
    use std::string::String;
    use std::vec::Vec;

    /// A tee over a leaked buffer of `capacity` bytes.
    fn tee(capacity: usize) -> Tee {
        Tee::new(Vec::leak(vec![0; capacity]))
    }

    /// Offer one line at `now_ms` (the test is the one producer).
    fn offer(tee: &Tee, now_ms: u32, line: &str) {
        // SAFETY: the test thread is the tee's only producer.
        unsafe { tee.push(now_ms, &[line.as_bytes()]) };
    }

    /// A batch of these lines and this drop count.
    fn batch(lines: &[&str], dropped: u32) -> TeeBatch {
        TeeBatch {
            lines: lines.iter().map(|line| String::from(*line)).collect(),
            dropped,
        }
    }

    #[test]
    fn a_closed_tee_copies_nothing() {
        let tee = tee(1024);
        offer(&tee, 10, "before any window");
        assert_eq!(tee.drain(10, 1000), batch(&[], 0));
    }

    #[test]
    fn an_open_window_collects_until_the_next_drain_renews_it() {
        let tee = tee(1024);
        tee.drain(1_000, 1000);
        offer(&tee, 1_500, "a");
        // SAFETY: the test is the one producer.
        unsafe { tee.push(2_000, &[b"@", b"000123_health.csv", b"@", b"1;2"]) };
        assert_eq!(tee.drain(2_000, 1000), batch(&["a", "@000123_health.csv@1;2"], 0));
        offer(&tee, 2_900, "b");
        assert_eq!(tee.drain(2_950, 0), batch(&["b"], 0), "0 returns the batch and closes");
        offer(&tee, 3_000, "after closing");
        assert_eq!(tee.drain(3_000, 1000), batch(&[], 0));
    }

    #[test]
    fn a_lapsed_window_loses_its_batch() {
        let tee = tee(1024);
        tee.drain(0, 100);
        offer(&tee, 50, "in time");
        offer(&tee, 101, "after the window: marks it lapsed");
        offer(&tee, 102, "ignored");
        assert_eq!(tee.drain(200, 100), batch(&[], 0), "the stale batch goes");
        offer(&tee, 250, "the new window works");
        assert_eq!(tee.drain(260, 100), batch(&["the new window works"], 0));
        // negative: a late drain with no line after the deadline still gets its batch, as plan B's did
        offer(&tee, 300, "kept");
        assert_eq!(tee.drain(5_000, 100), batch(&["kept"], 0));
    }

    #[test]
    fn a_line_that_lands_after_a_closing_drain_is_stale() {
        let tee = tee(1024);
        tee.drain(0, 1000);
        offer(&tee, 10, "in the window");
        assert_eq!(tee.drain(20, 0), batch(&["in the window"], 0), "closes the window");
        // the writer checked the window before that drain closed it, and copies its line only now
        // SAFETY: the test is the one producer.
        unsafe { tee.store(&[b"checked while open"]) };
        assert_eq!(
            tee.drain(7_200_000, 1000),
            batch(&[], 0),
            "the next session, hours later, does not get it"
        );
        offer(&tee, 7_200_010, "fresh");
        assert_eq!(tee.drain(7_200_020, 1000), batch(&["fresh"], 0));
        // negative: an open window's lines are returned whole, the store path included
        // SAFETY: as above.
        unsafe { tee.store(&[b"stored while open"]) };
        assert_eq!(tee.drain(7_200_030, 0), batch(&["stored while open"], 0));
    }

    #[test]
    fn a_full_tee_drops_and_counts() {
        let tee = tee(64);
        tee.drain(0, 1000);
        for index in 0..10 {
            offer(&tee, 1, &std::format!("line {index}")); // 4 + 6 bytes, 12 with padding
        }
        let first = tee.drain(2, 1000);
        assert_eq!(first.lines.len(), 5, "64 bytes hold 5 records and the gap");
        assert_eq!(first.dropped, 5);
        offer(&tee, 3, "room again");
        assert_eq!(tee.drain(4, 1000), batch(&["room again"], 0), "the count restarts");
    }

    #[test]
    fn the_deadline_wraps_with_the_milliseconds() {
        let tee = tee(1024);
        tee.drain(u32::MAX - 10, 100); // the window ends at 89 after the wrap
        offer(&tee, 5, "after the wrap, in time");
        assert_eq!(tee.drain(6, i64::MAX), batch(&["after the wrap, in time"], 0));
        offer(&tee, 6 + 86_400_000, "a day later: still open");
        assert_eq!(tee.drain(7, 1).lines.len(), 1, "the longest window is a day");
        tee.drain(u32::MAX, 1); // ends at 0, which would read as closed
        offer(&tee, 0, "the deadline 0 is moved to 2");
        assert_eq!(tee.drain(1, 0).lines.len(), 1);
    }
}
