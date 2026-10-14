//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A CC tee (doc/specs/avionics-port.md "CC tees and the HTTP exits"; src/glider recorder.py `_TeeSink`): a
//! poll-model copy of the log or the telemetry records for the CC `log [ms]` and `tlm [ms]` commands.
//!
//! The ring_writer copies each record in, verbatim and never as text, while the operator's window is open; the
//! command's executor takes the batch out and only then prints it (invariant 4: text is made off the flight
//! path). `drain(ms)` returns what was buffered since the last call and keeps the window open `ms` more (0 or
//! less closes it). A window nobody renews lapses: the writer marks it so at its next record, and the next drain
//! throws the stale batch away, so a lost link never fills the buffer with records from long ago. Best effort: a
//! full buffer drops the record and counts it, never blocking the writer.
//!
//! The writer and the drain run on their own threads, and the writer preempts the executor, so a drain never closes
//! the window while it works. It reads where the writer has stored up to, then moves the window in one swap (the
//! new deadline, or closed), and takes only the records before that point. A record the writer checked against the
//! new deadline is stored past it and waits for the next drain; so is one it checked just before the swap and
//! stored just after it, which that next drain prints as long as the window it was checked in was open. Holding
//! the window closed for a whole drain (lock, walk, formatting up to 256 KiB) would make the writer pass over every
//! record meanwhile, uncounted, and the CC reads `dropped` to tell an incomplete window from a quiet sensor.
//!
//! A drain also throws away what it finds in a closed tee. The writer checks the window and then copies its
//! record, so a record whose check came just before a closing drain lands after that drain's batch ended: plan B,
//! single-threaded, never had such a line, and returned to the next session hours later it would read as fresh.
//!
//! The deadline is a wrapping u32 of uptime milliseconds (the P4 has no 64-bit atomics), as the M1 firmware
//! tee keeps it: 0 means closed and 1 lapsed, so an open deadline is at least 2. The records go through a byte
//! ring in PSRAM: the writer is its one producer, and the drain its one consumer, serialised by a lock that only
//! the executor side takes.

use std::string::String;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU32, Ordering};
use std::vec::Vec;

use crate::byte_ring::{ByteRing, LENGTH_BYTES};
use crate::record::Record;

/// The deadline of a closed window.
const CLOSED: u32 = 0;

/// The deadline of a window the writer found lapsed: the next drain discards its batch.
const LAPSED: u32 = 1;

/// The longest window, a day: a longer one would read as lapsed once the u32 milliseconds wrap past it.
const LONGEST_WINDOW_MS: i64 = 86_400_000;

/// What one drain returns.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TeeBatch {
    /// The lines buffered since the last drain, in order, without their CRC wrapper or line end: a log line as
    /// `<uptime_us> <descriptor> :: <message>`, a telemetry line as `@<routing>@<row>`.
    pub lines: Vec<String>,
    /// The records the full buffer dropped since the last drain: a gap the operator would not see otherwise.
    pub dropped: u32,
}

/// One tee.
pub(crate) struct Tee {
    /// The window's end in uptime ms (wrapping); [`CLOSED`] or [`LAPSED`] otherwise.
    deadline_ms: AtomicU32,
    /// Records dropped since the last drain.
    dropped: AtomicU32,
    /// The buffered records.
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

    /// Copies one whole record in, verbatim, while the window is open at `now_ms`; marks a lapsed window, and
    /// counts a record the full buffer drops.
    ///
    /// # Safety
    /// Only the ring_writer calls it: it is the ring's one producer.
    pub(crate) unsafe fn push(&self, now_ms: u32, record: &[u8]) {
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
        unsafe { self.store(record) };
    }

    /// Copies one whole record into the buffer, whatever the window: [`Tee::push`] after its check. A record's
    /// length field is the ring's own (both the padded size), so the ring writes it again unchanged. Counts a
    /// record the full buffer drops.
    ///
    /// # Safety
    /// As [`Tee::push`]: only the ring_writer calls it.
    unsafe fn store(&self, record: &[u8]) {
        let fill = |slot: &mut [u8]| slot[LENGTH_BYTES..].copy_from_slice(&record[LENGTH_BYTES..]);
        // SAFETY: the writer is the one producer (the caller's contract).
        if record.len() < LENGTH_BYTES || !unsafe { self.ring.push(record.len(), fill) } {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// The batch buffered since the last drain, printed under `session`, with the window kept open `duration_ms`
    /// more from `now_ms` (0 or less closes it). A batch whose window lapsed, or that a closed tee holds, is
    /// discarded, not returned.
    ///
    /// The batch ends where the writer had stored up to when the window moved (see the module notes): what the
    /// writer stores while this drain prints stays for the next one.
    ///
    /// A LOG record prints as its log line, any other as `@<routing>@<text>` (`@session.csv@...` for the index),
    /// and a record whose file name no routing takes as its bare text, as the ring_saver would file it in
    /// recorder.log. The CC replies carry no CRC wrapper.
    pub(crate) fn drain(&self, now_ms: u32, duration_ms: i64, session: &str) -> TeeBatch {
        let _consumer = self.consumer.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        // the batch's end, read before the swap: every record the writer checks against the new deadline is
        // stored after it (the Acquire load keeps the swap behind it)
        let end = self.ring.head();
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        let deadline = if duration_ms > 0 {
            now_ms
                .wrapping_add(duration_ms.min(LONGEST_WINDOW_MS) as u32)
                .max(LAPSED + 1)
        } else {
            CLOSED
        };
        let previous = self.deadline_ms.swap(deadline, Ordering::AcqRel);
        let mut lines = Vec::new();
        let mut routing = String::new();
        let mut text = Vec::new();
        let mut read = self.ring.tail();
        while read != end {
            // SAFETY: the lock makes this the ring's one consumer, reading from its tail record by record up to a
            // head it read, so every record before `end` is published.
            let Some((start, bytes)) = (unsafe { self.ring.record(read) }) else {
                break; // unreachable: `end` is a published record boundary
            };
            if previous > LAPSED
                && let Some(record) = Record::decode(bytes)
            {
                text.clear();
                if let Some(routing) = record.routing(session, &mut routing) {
                    text.push(b'@');
                    text.extend_from_slice(routing.as_bytes());
                    text.push(b'@');
                }
                record.text_into(&mut text);
                lines.push(String::from_utf8_lossy(&text).into_owned());
            }
            read = self.ring.after(start, bytes.len());
        }
        // SAFETY: as above; every record before it was printed first.
        unsafe { self.ring.release(read) };
        TeeBatch { lines, dropped }
    }
}

#[cfg(test)]
mod tests {
    use super::{Tee, TeeBatch};
    use crate::record::{encode, log_length, log_parts, row_length, write_log, write_row};
    use recorder_wire::Cell;
    use std::string::String;
    use std::vec::Vec;

    /// A tee over a leaked buffer of `capacity` bytes.
    fn tee(capacity: usize) -> Tee {
        Tee::new(Vec::leak(vec![0; capacity]))
    }

    /// A LOG record of `message` at `uptime_us`, from the descriptor `x`.
    fn log(uptime_us: u64, message: &str) -> Vec<u8> {
        let (descriptor, message) = log_parts("x", message);
        let mut record = Vec::new();
        encode(&mut record, log_length(descriptor, message), |bytes| {
            write_log(bytes, uptime_us, descriptor, message)
        });
        record
    }

    /// A ROW record of one i32 to `file` at `uptime_us`.
    fn row(file: &str, uptime_us: u64, value: i32) -> Vec<u8> {
        let cells = [Cell::I32(value)];
        let mut record = Vec::new();
        encode(&mut record, row_length(file, &cells).unwrap(), |bytes| {
            write_row(bytes, file, false, uptime_us, &cells)
        });
        record
    }

    /// Offer one record at `now_ms` (the test is the one producer).
    fn offer(tee: &Tee, now_ms: u32, record: &[u8]) {
        // SAFETY: the test thread is the tee's only producer.
        unsafe { tee.push(now_ms, record) };
    }

    /// Drain under the session `000123`.
    fn drain(tee: &Tee, now_ms: u32, duration_ms: i64) -> TeeBatch {
        tee.drain(now_ms, duration_ms, "000123")
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
        offer(&tee, 10, &log(1, "before any window"));
        assert_eq!(drain(&tee, 10, 1000), batch(&[], 0));
    }

    #[test]
    fn an_open_window_prints_its_records_at_the_next_drain() {
        let tee = tee(1024);
        drain(&tee, 1_000, 1000);
        offer(&tee, 1_500, &log(1_500_000, "a"));
        offer(&tee, 2_000, &row("health.csv", 2_000_000, 7));
        offer(&tee, 2_000, &row("session.csv", 2_000_001, 8));
        offer(&tee, 2_000, &row("a/b.csv", 2_000_002, 9));
        assert_eq!(
            drain(&tee, 2_000, 1000),
            batch(
                &[
                    "1500000 x :: a",
                    "@000123_health.csv@2000000;7",
                    "@session.csv@2000001;8",
                    "2000002;9", // no routing for a name the saver would not file: recorder.log's
                ],
                0
            )
        );
        offer(&tee, 2_900, &log(2_900_000, "b"));
        assert_eq!(
            drain(&tee, 2_950, 0),
            batch(&["2900000 x :: b"], 0),
            "0 returns the batch and closes"
        );
        offer(&tee, 3_000, &log(3_000_000, "after closing"));
        assert_eq!(drain(&tee, 3_000, 1000), batch(&[], 0));
    }

    #[test]
    fn a_lapsed_window_loses_its_batch() {
        let tee = tee(1024);
        drain(&tee, 0, 100);
        offer(&tee, 50, &log(50_000, "in time"));
        offer(&tee, 101, &log(101_000, "after the window: marks it lapsed"));
        offer(&tee, 102, &log(102_000, "ignored"));
        assert_eq!(drain(&tee, 200, 100), batch(&[], 0), "the stale batch goes");
        offer(&tee, 250, &log(250_000, "the new window works"));
        assert_eq!(drain(&tee, 260, 100), batch(&["250000 x :: the new window works"], 0));
        // negative: a late drain with no record after the deadline still gets its batch, as plan B's did
        offer(&tee, 300, &log(300_000, "kept"));
        assert_eq!(drain(&tee, 5_000, 100), batch(&["300000 x :: kept"], 0));
    }

    #[test]
    fn a_record_that_lands_after_a_closing_drain_is_stale() {
        let tee = tee(1024);
        drain(&tee, 0, 1000);
        offer(&tee, 10, &log(10_000, "in the window"));
        assert_eq!(
            drain(&tee, 20, 0),
            batch(&["10000 x :: in the window"], 0),
            "closes the window"
        );
        // the writer checked the window before that drain closed it, and copies its record only now
        // SAFETY: the test is the one producer.
        unsafe { tee.store(&log(15_000, "checked while open")) };
        assert_eq!(
            drain(&tee, 7_200_000, 1000),
            batch(&[], 0),
            "the next session, hours later, does not get it"
        );
        offer(&tee, 7_200_010, &log(7_200_010_000, "fresh"));
        assert_eq!(drain(&tee, 7_200_020, 1000), batch(&["7200010000 x :: fresh"], 0));
        // negative: after a renewing drain, the same late record is the next batch's, never lost
        // SAFETY: as above.
        unsafe { tee.store(&log(7_200_015_000, "checked before the renewal")) };
        assert_eq!(
            drain(&tee, 7_200_025, 1000),
            batch(&["7200015000 x :: checked before the renewal"], 0)
        );
        // negative: an open window's records are returned whole, the store path included
        // SAFETY: as above.
        unsafe { tee.store(&log(7_200_030_000, "stored while open")) };
        assert_eq!(
            drain(&tee, 7_200_030, 0),
            batch(&["7200030000 x :: stored while open"], 0)
        );
    }

    #[test]
    fn a_full_tee_drops_and_counts() {
        let tee = tee(128);
        drain(&tee, 0, 1000);
        for index in 0..10 {
            offer(&tee, 1, &log(index, "y")); // 16 + 12 + 1 + 1 + 1 = 31: 32 bytes each
        }
        let first = drain(&tee, 2, 1000);
        assert_eq!(first.lines.len(), 3, "128 bytes hold 3 records and the gap");
        assert_eq!(first.dropped, 7);
        offer(&tee, 3, &log(3, "room again"));
        assert_eq!(
            drain(&tee, 4, 1000),
            batch(&["3 x :: room again"], 0),
            "the count restarts"
        );
    }

    #[test]
    fn the_deadline_wraps_with_the_milliseconds() {
        let tee = tee(1024);
        drain(&tee, u32::MAX - 10, 100); // the window ends at 89 after the wrap
        offer(&tee, 5, &log(1, "after the wrap, in time"));
        assert_eq!(drain(&tee, 6, i64::MAX).lines, ["1 x :: after the wrap, in time"]);
        offer(&tee, 6 + 86_400_000, &log(2, "a day later: still open"));
        assert_eq!(drain(&tee, 7, 1).lines.len(), 1, "the longest window is a day");
        drain(&tee, u32::MAX, 1); // ends at 0, which would read as closed
        offer(&tee, 0, &log(3, "the deadline 0 is moved to 2"));
        assert_eq!(drain(&tee, 1, 0).lines.len(), 1);
    }
}
