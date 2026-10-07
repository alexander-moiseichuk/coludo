//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! What the ring tasks publish for other threads, with no 64-bit atomic (the P4 has none):
//! * [`PublishedPosition`], a ring position: the end of what is programmed (`writer_position`,
//!   doc/specs/avionics-port.md "The ring_writer", step 6), which the ring_saver never reads past, and where the
//!   saver goes on (`saver_position`, "The ring_saver"), which the eraser erases behind. A block sequence and an
//!   offset that must be read as a pair, so a seqlock over AtomicU32, in its two-copy "latch" form so that a reader
//!   never waits for the task that stores it;
//! * [`PublishedStats`], the counters and ring figures `stats`/`inspect` report. Each counter is an AtomicU32 read
//!   on its own, since a report may mix two steps' values without harm; the ring figures are copied whole.

use core::sync::atomic::{AtomicU32, Ordering, fence};
use std::sync::Mutex;

use flash_ring::{Figures, Position};
use recorder_wire::Counters;

/// One of the two copies of a [`Position`]: its block sequence and offset.
struct Replica {
    /// The position's block sequence.
    block_sequence: AtomicU32,
    /// The position's offset.
    offset: AtomicU32,
}

impl Replica {
    /// A zero copy, never read before a store fills it.
    const fn new() -> Replica {
        Replica {
            block_sequence: AtomicU32::new(0),
            offset: AtomicU32::new(0),
        }
    }

    /// Write `position` into this copy.
    fn write(&self, position: Position) {
        self.block_sequence.store(position.block_sequence, Ordering::Relaxed);
        self.offset.store(position.offset, Ordering::Relaxed);
    }

    /// Read this copy, maybe torn: the caller checks the sequence after.
    fn read(&self) -> Position {
        Position {
            block_sequence: self.block_sequence.load(Ordering::Relaxed),
            offset: self.offset.load(Ordering::Relaxed),
        }
    }
}

/// A [`Position`] one task stores (the ring_writer its own, the ring_saver its own) and any thread loads whole: a
/// seqlock latch over two copies.
///
/// A store bumps the sequence and writes copy 0, then bumps it again and writes copy 1. A reader takes the copy
/// the sequence's low bit names, the one no store is writing, and keeps it when the sequence did not move
/// meanwhile. So a reader that preempts the writer mid-store (the supervisor at priority 9 over the writer at 7,
/// both on core 0) reads the other, whole copy at once, where a plain seqlock would spin forever: FreeRTOS never
/// runs the lower-priority writer on that core to finish the store. A reader retries only when a store moved on
/// during its own two loads, which takes the writer's progress, never its waiting. 0 means nothing was stored.
pub(crate) struct PublishedPosition {
    /// Bumped twice per store, so it is odd while copy 0 is written and even while copy 1 is; 0 means never.
    sequence: AtomicU32,
    /// The two copies: readers take copy `sequence & 1`.
    copies: [Replica; 2],
}

impl PublishedPosition {
    /// Nothing stored yet.
    pub(crate) const fn new() -> PublishedPosition {
        PublishedPosition {
            sequence: AtomicU32::new(0),
            copies: [Replica::new(), Replica::new()],
        }
    }

    /// Store `position`. Only the position's own task calls it: one storer, so the sequence needs no
    /// read-modify-write.
    pub(crate) fn store(&self, position: Position) {
        let sequence = self.sequence.load(Ordering::Relaxed);
        if sequence == 0 {
            // readers see copy 1 first (the odd sequence below), and no reader looks before then
            self.copies[1].write(position);
        }
        // odd: readers move to copy 1, which the Release keeps ordered after its last write
        self.sequence.store(sequence.wrapping_add(1), Ordering::Release);
        fence(Ordering::Release);
        self.copies[0].write(position);
        // even: readers move back to copy 0. Past 2^32 stores it wraps: skip 0, which means "never stored"
        let even = sequence.wrapping_add(2);
        self.sequence.store(if even == 0 { 2 } else { even }, Ordering::Release);
        fence(Ordering::Release);
        self.copies[1].write(position);
    }

    /// The last position stored, or None before the first store. It never waits for the writer: see the type.
    pub(crate) fn load(&self) -> Option<Position> {
        loop {
            let before = self.sequence.load(Ordering::Acquire);
            if before == 0 {
                return None;
            }
            let position = self.copies[(before & 1) as usize].read();
            fence(Ordering::Acquire);
            if self.sequence.load(Ordering::Relaxed) == before {
                return Some(position);
            }
        }
    }
}

/// One record class's figures in atomics.
#[derive(Default)]
struct PublishedCounters {
    /// Records not yet on flash.
    count: AtomicU32,
    /// The most ever not yet on flash.
    max: AtomicU32,
}

/// The counters and figures the writer publishes.
#[derive(Default)]
pub(crate) struct PublishedStats {
    /// Telemetry records (headers and rows, session.csv rows among them).
    telemetry: PublishedCounters,
    /// Log records.
    log: PublishedCounters,
    /// The flash ring's figures, copied whole with each status line (they walk the whole block table, so not on
    /// every step). The writer only `try_lock`s it, skipping a copy rather than waiting for a `stats` reader; it is
    /// not a producer path.
    ring: Mutex<Figures>,
    /// Chunks the writer programmed this boot.
    chunks: AtomicU32,
    /// Flash calls of the writer that failed this boot.
    flash_errors: AtomicU32,
    /// The measured write rate in bytes of flash a second, as of the last status line.
    write_rate: AtomicU32,
}

/// The recorder's health: what `stats` and the status line report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Telemetry records: `count` not yet on flash, its `max`, and `dropped` (refused by a full ring, or never
    /// programmable).
    pub telemetry: Counters,
    /// Log records, likewise.
    pub log: Counters,
    /// The flash ring's budget figures, as of the last status line (every `stats_ms`).
    pub ring: Figures,
    /// Chunks programmed into the ring this boot.
    pub chunks: u32,
    /// Flash calls of the writer that failed this boot (a chunk or a block header program, a check read).
    pub flash_errors: u32,
    /// The measured write rate: bytes of flash the writer position moved a second (block headers and the unused
    /// tails of closed blocks included, as they fill the ring), over the last `stats_ms`. It turns the ring figures
    /// into time: `ring.erased_seconds(rate)` is `ring_erased_s`, the time before newest wins, and
    /// `ring.unsaved_seconds(rate)` is `ring_unsaved_s`, the flash-to-SD backlog (spec "The ring_eraser and newest
    /// wins", budget figures). 0 before the first status line.
    pub write_rate: u32,
}

impl PublishedStats {
    /// Publish a step's counters; `count` and `max` are (telemetry, log).
    pub(crate) fn store(&self, count: [u32; 2], max: [u32; 2], chunks: u32, flash_errors: u32) {
        for (counters, (count, max)) in [&self.telemetry, &self.log].into_iter().zip(count.into_iter().zip(max)) {
            counters.count.store(count, Ordering::Relaxed);
            counters.max.store(max, Ordering::Relaxed);
        }
        self.chunks.store(chunks, Ordering::Relaxed);
        self.flash_errors.store(flash_errors, Ordering::Relaxed);
    }

    /// Publish the ring's figures, unless a `stats` reader holds them right now, and the write rate that goes with
    /// them (bytes a second), which never waits.
    pub(crate) fn store_ring(&self, ring: Figures, write_rate: u32) {
        if let Ok(mut figures) = self.ring.try_lock() {
            *figures = ring;
        }
        self.write_rate.store(write_rate, Ordering::Relaxed);
    }

    /// The published figures, with the drop counts (telemetry, log) the producers keep.
    pub(crate) fn load(&self, dropped: [u32; 2]) -> Stats {
        let counters = |published: &PublishedCounters, dropped: u32| Counters {
            count: published.count.load(Ordering::Relaxed),
            max: published.max.load(Ordering::Relaxed),
            dropped,
        };
        Stats {
            telemetry: counters(&self.telemetry, dropped[0]),
            log: counters(&self.log, dropped[1]),
            ring: *self.ring.lock().unwrap_or_else(|poisoned| poisoned.into_inner()),
            chunks: self.chunks.load(Ordering::Relaxed),
            flash_errors: self.flash_errors.load(Ordering::Relaxed),
            write_rate: self.write_rate.load(Ordering::Relaxed),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PublishedPosition, PublishedStats};
    use flash_ring::{Figures, Position};
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn a_position_reads_back_whole_or_not_at_all() {
        let published = PublishedPosition::new();
        assert_eq!(published.load(), None, "never stored");
        let position = Position {
            block_sequence: 7,
            offset: 4096,
        };
        published.store(position);
        assert_eq!(published.load(), Some(position));
        published.sequence.store(u32::MAX - 1, Ordering::Relaxed);
        published.store(Position::default());
        assert_eq!(published.sequence.load(Ordering::Relaxed), 2, "the wrap skips 0");
        assert_eq!(published.load(), Some(Position::default()));
    }

    #[test]
    fn a_reader_never_waits_for_a_writer_stopped_mid_store() {
        let published = PublishedPosition::new();
        let old = Position {
            block_sequence: 3,
            offset: 800,
        };
        published.store(old);
        // the writer preempted half way through writing copy 0: the reader takes copy 1, still whole
        published.sequence.store(3, Ordering::Relaxed);
        published.copies[0].block_sequence.store(4, Ordering::Relaxed);
        assert_eq!(
            published.load(),
            Some(old),
            "returns at once, with the last whole store"
        );
        // and half way through copy 1: copy 0 holds the new position whole
        let new = Position {
            block_sequence: 4,
            offset: 40,
        };
        published.copies[0].write(new);
        published.sequence.store(4, Ordering::Relaxed);
        published.copies[1].block_sequence.store(4, Ordering::Relaxed);
        assert_eq!(published.load(), Some(new));
        // negative: before the first store has made its sequence odd, nothing reads as stored
        let fresh = PublishedPosition::new();
        fresh.copies[1].write(new);
        assert_eq!(fresh.load(), None);
    }

    #[test]
    fn a_reader_never_sees_a_torn_pair() {
        let published = PublishedPosition::new();
        let done = AtomicBool::new(false);
        std::thread::scope(|scope| {
            scope.spawn(|| {
                for step in 1..=300_000u32 {
                    // the pair always satisfies offset == 3 * block_sequence
                    published.store(Position {
                        block_sequence: step,
                        offset: step.wrapping_mul(3),
                    });
                }
                done.store(true, Ordering::Release);
            });
            let mut last = 0;
            while !done.load(Ordering::Acquire) {
                if let Some(position) = published.load() {
                    assert_eq!(position.offset, position.block_sequence.wrapping_mul(3), "torn");
                    assert!(position.block_sequence >= last, "stores are seen in order");
                    last = position.block_sequence;
                }
            }
        });
    }

    #[test]
    fn stats_round_trip() {
        let published = PublishedStats::default();
        let ring = Figures {
            erased_ahead: 1,
            saved: 2,
            unsaved: 3,
            other_version: 4,
            foreign: 5,
            lost: 6,
            bad: 7,
        };
        published.store([10, 20], [11, 21], 7, 8);
        published.store_ring(ring, 16_384);
        let stats = published.load([12, 22]);
        assert_eq!(
            (stats.telemetry.count, stats.telemetry.max, stats.telemetry.dropped),
            (10, 11, 12)
        );
        assert_eq!((stats.log.count, stats.log.max, stats.log.dropped), (20, 21, 22));
        assert_eq!((stats.ring, stats.chunks, stats.flash_errors), (ring, 7, 8));
        assert_eq!(stats.write_rate, 16_384);
        let held = published.ring.lock().unwrap();
        published.store_ring(Figures::default(), 99);
        published.store([0, 0], [0, 0], 9, 9);
        drop(held);
        assert_eq!(
            published.load([0, 0]).ring,
            ring,
            "a busy copy is skipped, not waited for"
        );
        assert_eq!(published.load([0, 0]).chunks, 9, "the counters never wait");
        assert_eq!(published.load([0, 0]).write_rate, 99, "nor does the rate");
    }
}
