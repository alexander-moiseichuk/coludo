//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! `saver-behind`'s judge (doc/specs/avionics-port.md "The ring_eraser and newest wins", Degraded tokens: SD mounted
//! and `ring_unsaved_s` growing for more than 60 s). The firmware looks at the flash-to-SD backlog
//! ([`crate::Figures::unsaved`]) at every `health` read and asks [`BacklogTrend::behind`].
//!
//! The backlog is a sawtooth, not a line: it climbs while the saver collects a batch and drops by up to a batch (64
//! blocks) when the batch goes to the card. So a fall between two looks says nothing: a card slower than the writer
//! still commits a batch now and then, and the backlog still grows across a minute. What tells growth from noise is
//! the bottom of the sawtooth, the backlog just after a commit: each window of [`SAVER_BEHIND_MS`] keeps its lowest
//! look, and the saver is behind when that low is no lower than the previous window's and the saver never caught up
//! with the writer in the window. The health reads come every 2 s from the hub, so a window's low misses a commit's
//! bottom by at most 2 s of writing (8 blocks at the flight rate), where a batch is 64.
//!
//! It judges the block count, of which `ring_unsaved_s` is the count at the measured write rate: a change of rate
//! alone is not the saver falling behind.

/// How long a window of the backlog's trend is, ms: the spec's 60 s.
pub const SAVER_BEHIND_MS: u32 = 60_000;

/// The flash-to-SD backlog followed from one look to the next, a window of [`SAVER_BEHIND_MS`] at a time.
///
/// Behind, when over a whole window the backlog's low did not fall below the previous window's and the saver never
/// caught up: a dead card, a card slower than the writer, a saver that never started. A saver that keeps up catches
/// up ten times a second, and one draining a backlog at the card's speed lowers the low from window to window. The
/// verdict holds until the next window ends, or until the saver catches up.
#[derive(Clone, Copy, Debug)]
pub struct BacklogTrend {
    /// When this window started (uptime ms, wrapping); None before the first look.
    started_ms: Option<u32>,
    /// The lowest backlog of the last full window, in blocks (before the first full window: the first look's).
    previous_low: u32,
    /// The lowest backlog of this window so far, in blocks.
    low: u32,
    /// The last full window's verdict, cleared early when the saver catches up.
    behind: bool,
}

impl BacklogTrend {
    /// A trend with no look yet.
    pub const fn new() -> BacklogTrend {
        BacklogTrend {
            started_ms: None,
            previous_low: 0,
            low: 0,
            behind: false,
        }
    }

    /// Takes a look at the backlog, and says whether the saver is behind.
    ///
    /// Args:
    /// * `unsaved` - the closed blocks the saver has not passed now ([`crate::Figures::unsaved`]).
    /// * `caught_up_ms` - when the saver last found nothing more to read (uptime ms, wrapping); None when no saver
    ///   runs (its thread never started), which never catches up.
    /// * `now_ms` - the uptime in ms, wrapping.
    ///
    /// Returns the verdict: false until a whole window has passed since the first look.
    pub fn behind(&mut self, unsaved: u32, caught_up_ms: Option<u32>, now_ms: u32) -> bool {
        let Some(started_ms) = self.started_ms else {
            *self = BacklogTrend {
                started_ms: Some(now_ms),
                previous_low: unsaved,
                low: unsaved,
                behind: false,
            };
            return false;
        };
        self.low = self.low.min(unsaved);
        let elapsed = now_ms.wrapping_sub(started_ms);
        // caught up since this window started (both times wrap alike, and a window is far shorter than a wrap)
        let caught_up = caught_up_ms.is_some_and(|caught_up_ms| now_ms.wrapping_sub(caught_up_ms) <= elapsed);
        if elapsed >= SAVER_BEHIND_MS {
            self.behind = self.low >= self.previous_low && !caught_up;
            // this look closes the window and opens the next
            self.previous_low = self.low;
            self.low = unsaved;
            self.started_ms = Some(now_ms);
        } else if caught_up {
            self.behind = false;
        }
        self.behind
    }
}

impl Default for BacklogTrend {
    fn default() -> BacklogTrend {
        BacklogTrend::new()
    }
}

#[cfg(test)]
mod tests {
    use alloc::vec::Vec;

    use super::*;

    /// The hub's health period, ms.
    const LOOK_MS: u32 = 2000;

    /// A ring and its saver, a millisecond at a time: the writer closes a block every `write_ms`; the saver, once
    /// `batch` blocks are closed, takes `commit_ms` to put them on the card and then passes them (a card that never
    /// finishes a commit is dead). It catches up when nothing closed is left after a commit, and last did at 0.
    struct Bench {
        unsaved: u32,
        caught_up_ms: Option<u32>,
        committing_since: Option<u32>,
        write_ms: u32,
        batch: u32,
        commit_ms: u32,
    }

    impl Bench {
        fn new(unsaved: u32, write_ms: u32, batch: u32, commit_ms: u32) -> Bench {
            Bench {
                unsaved,
                caught_up_ms: Some(0),
                committing_since: None,
                write_ms,
                batch,
                commit_ms,
            }
        }

        /// Runs from `from_ms` to `to_ms`, looking every [`LOOK_MS`]; returns each look's verdict.
        fn run(&mut self, trend: &mut BacklogTrend, from_ms: u32, to_ms: u32) -> Vec<bool> {
            let mut verdicts = Vec::new();
            for now_ms in from_ms..to_ms {
                if self.write_ms > 0 && now_ms % self.write_ms == 0 {
                    self.unsaved += 1;
                }
                match self.committing_since {
                    Some(since) if now_ms - since >= self.commit_ms => {
                        self.unsaved -= self.batch.min(self.unsaved);
                        self.committing_since = None;
                        if self.unsaved == 0 {
                            self.caught_up_ms = Some(now_ms);
                        }
                    }
                    None if self.unsaved >= self.batch => {
                        self.committing_since = Some(now_ms);
                    }
                    _ => {}
                }
                if now_ms % LOOK_MS == 0 {
                    verdicts.push(trend.behind(self.unsaved, self.caught_up_ms, now_ms));
                }
            }
            verdicts
        }
    }

    #[test]
    fn a_card_slower_than_the_writer_is_behind_though_every_commit_drops_the_backlog() {
        // the writer closes 4 blocks a second; the card takes 20 s for a batch of 64: +16 blocks every 20 s
        let mut trend = BacklogTrend::new();
        let mut bench = Bench::new(0, 250, 64, 20_000);
        let warm = bench.run(&mut trend, 0, 2 * SAVER_BEHIND_MS);
        assert!(!warm[0], "never behind at the first look");
        let verdicts = bench.run(&mut trend, 2 * SAVER_BEHIND_MS, 3_600_000);
        assert!(bench.unsaved > 600, "the backlog grew: {}", bench.unsaved);
        assert!(
            verdicts.iter().all(|&behind| behind),
            "behind at every look after two windows"
        );
    }

    #[test]
    fn a_dead_card_is_behind_after_a_window_even_at_the_pad_rate() {
        // the pad: a block every 40 s, and the saver's batch never reaches the card; it last caught up at 0, and
        // the first look (the first window's start) is the one at 2 s
        let mut trend = BacklogTrend::new();
        let mut bench = Bench::new(3, 40_000, 64, u32::MAX);
        let verdicts = bench.run(&mut trend, 1, 3 * SAVER_BEHIND_MS);
        let first = verdicts.iter().position(|&behind| behind).expect("raised");
        assert_eq!(
            (first as u32 + 1) * LOOK_MS,
            LOOK_MS + SAVER_BEHIND_MS,
            "raised as the first window closes"
        );
        assert!(verdicts[first..].iter().all(|&behind| behind), "and held");
    }

    #[test]
    fn a_stuck_backlog_with_no_saver_is_behind() {
        let mut trend = BacklogTrend::new();
        assert!(!trend.behind(40, None, 0));
        assert!(
            !trend.behind(40, None, SAVER_BEHIND_MS - 1),
            "not before a whole window"
        );
        assert!(
            trend.behind(40, None, SAVER_BEHIND_MS),
            "a backlog that did not fall, and no saver"
        );
    }

    #[test]
    fn a_saver_that_keeps_up_is_never_behind() {
        // the flight rate, a batch of 4 blocks a second, which the card takes 100 ms for
        let mut trend = BacklogTrend::new();
        let mut bench = Bench::new(0, 250, 4, 100);
        let verdicts = bench.run(&mut trend, 0, 600_000);
        assert!(verdicts.iter().all(|&behind| !behind));
    }

    #[test]
    fn a_saver_draining_a_backlog_slowly_is_never_behind() {
        // a new card, 3000 blocks to save: 64 every 15 s against 4 written a second, a net 4 blocks fewer every 15 s
        // under a sawtooth 60 blocks high
        let mut trend = BacklogTrend::new();
        let mut bench = Bench::new(3000, 250, 64, 15_000);
        let verdicts = bench.run(&mut trend, 0, 1_800_000);
        assert!(bench.unsaved < 3000, "it drains: {}", bench.unsaved);
        assert!(verdicts.iter().all(|&behind| !behind));
    }

    #[test]
    fn the_verdict_clears_as_soon_as_the_saver_catches_up() {
        // the saver last caught up at 0, before the first window starts at 1 s
        let mut trend = BacklogTrend::new();
        trend.behind(10, Some(0), 1000);
        assert!(trend.behind(12, Some(0), 1000 + SAVER_BEHIND_MS));
        assert!(
            trend.behind(14, Some(0), 3000 + SAVER_BEHIND_MS),
            "held within the next window"
        );
        assert!(
            !trend.behind(0, Some(4000 + SAVER_BEHIND_MS), 5000 + SAVER_BEHIND_MS),
            "the saver caught up: not behind, without waiting for the window to end"
        );
    }

    #[test]
    fn the_windows_run_across_the_uptime_wrap() {
        let start = u32::MAX - 1000;
        let mut trend = BacklogTrend::new();
        trend.behind(10, None, start);
        assert!(trend.behind(11, None, start.wrapping_add(SAVER_BEHIND_MS)));
        let caught_up = start.wrapping_add(SAVER_BEHIND_MS + 500);
        assert!(!trend.behind(0, Some(caught_up), caught_up.wrapping_add(10)));
    }
}
