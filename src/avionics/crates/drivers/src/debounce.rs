//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A debounce for a two-level input read once per tick, with or without an edge interrupt: the contact a latch stands
//! behind. It knows levels, not what they mean. The owner keeps the latch, says on every read which level the latch
//! stands for (`latched`) and acts on a verdict that differs from it: the separation switch's pads
//! ([`crate::separation`]) are the first owner, and a remove-before-flight pin or a breakwire would be the same. It
//! grew from plan B's separation `_settle` (src/glider/drivers/separation.py), stepped one read at a time.
//!
//! THE EVIDENCE ([`Edges`]): each read carries its level and what the input's both-edge interrupt saw since the
//! previous read: [`Edges::Seen`], the first and the last edge's stamps, or [`Edges::Quiet`], none. An input with no
//! armed interrupt passes [`Edges::Polled`] on every read, and the read is then the only evidence. An input passes one
//! kind for good: Polled, or Quiet and Seen.
//!
//! A SETTLE runs from the read that starts it to a verdict. Idle, an edge starts one; so does THE LEVEL RULE, a read
//! whose level differs from `latched` with no edge (a change before the interrupt was armed, a lost edge, any change on
//! a polled input). An idle read of the latched level with no edge does nothing, and the read that starts a settle
//! never concludes it. Under way:
//!  * THE RESTART RULE: an edge, or a read of the other level, restarts the run of agreeing reads: since the last edge
//!    (the line last changed then, not at the read), else since the read. Without it, chatter of a few ms between
//!    10 ms reads reads as a held level: toggles of 0.5-2 ms false-deployed 23-81 % of seeded separation runs with plan
//!    B's rule on the tick, none with this one.
//!  * THE HOLD, armed: a Quiet read that agrees with the run's level, `debounce_us` or more after the run began,
//!    concludes it, [`Verdict::Held`]. No count of reads: the interrupt proves the line held between them, so the hold
//!    is timed from the last edge's own stamp, to the µs, and a debounce of 0 concludes at the first quiet agreeing
//!    read.
//!  * THE RETURN, armed: a run at `latched` whose level held `debounce_us` or more up to an edge has concluded, though
//!    no quiet read came while it held: the edge's read concludes it, [`Verdict::Held`] at `latched` (the owner
//!    records nothing), and that read's edges start the next settle, departed at its first. So whether a return ends
//!    the episode is the line's own timing, not where the reads fell: a nest held 10-19 ms between a blip and the real
//!    separation once dated the separation by the blip at most tick phases. Home only: a verdict that moves the latch
//!    still waits for a quiet read, a read that sees the level held.
//!  * THE HOLD, polled: [`MINIMUM_SAMPLES`] agreeing reads spanning `debounce_us` conclude it, plan B's rule: such an
//!    input cannot see between its reads, so one read is never enough, however long the debounce.
//!  * THE DEADLINE: a settle with no verdict `debounce_us` x [`SETTLE_WINDOWS`] after its first read concludes
//!    [`Verdict::Unsettled`], but only once it has taken [`MINIMUM_SAMPLES`] reads (the floor), so any debounce settles
//!    a held level at any tick. Plan B gave up at the deadline alone, which a debounce under two read spacings reaches
//!    before a third read: its 0 ms debounce never settled. A deadline ends the episode, not the run under way: with
//!    the run at `latched` the line is home and the debounce goes idle; away, the next settle starts on the deadline's
//!    read with that run, departed when the run began. So a deadline delays no verdict and keeps the run's stamps: it
//!    only re-dates the departure.
//!
//! THE DEPARTURE ([`Verdict::Held`]'s `departed_us`): when the line first left `latched` in the settle -- the first
//! edge of the read that started it, else that read (the level rule, all a polled input has), or after a deadline the
//! start of the run carried over. It is kept across the restarts: a contact that wobbles as it parts restarts the run
//! on every bounce, and the verdict still dates the episode from its first departure. An edge a later read brings,
//! stamped before the departure, pulls it back: the edge contract lets an edge change the level a read sees after
//! that read took its edges, so the level rule dates that departure at the read and the edge's own stamp reaches the
//! next one. THE SETTLING (`settled_us`): when the held level began for good, the start of the run that concluded --
//! its last edge on an armed input, the first read of the final run on a polled one, so a polled input's settling, and
//! its span from the departure, are quantized to its reads; never before the departure. A settle that concludes, at
//! either level, ends the episode, and so does a deadline. So a departure whose return held for the debounce is an
//! episode of its own, and one whose return did not is part of the episode that follows.
//!
//! THE EDGE CONTRACT: [`Edges::Seen`]'s stamps are on the reads' clock, `first_us <= last_us <= now_us`, and cover
//! every edge since the previous read took its edges (an edge may reach two reads, never none): take the edges, then
//! the level, then the read's time. An armed input that loses an edge still settles: a Quiet read of the other level
//! restarts the run, since that read. An edge passed on every read restarts every run, so no settle away from the
//! latch concludes but on its deadline, Unsettled: the latch would never move. A polled input passes Polled, never an
//! edge.
//!
//! THE LATENCY, after the last change. Armed: the first quiet read at least `debounce_us` after the last edge; a quiet
//! read comes a read spacing after it at the earliest, so that is max(debounce, spacing) to that plus a spacing, and a
//! debounce up to the spacing decides on the same read (though a shorter one ends a return sooner: THE RETURN). Polled:
//! the read that is both `debounce_us` after the first read to see the level and the third to see it, max(2,
//! ceil(debounce / spacing)) spacings after that first read, which comes up to a spacing after the change. A deadline
//! adds nothing to either. The owner states the figures for its own tick ([`crate::separation`], THE LATENCY).
//!
//! THE CLOCK is the caller's wrapping u32 of microseconds (esp_timer's, truncated): it wraps every 71.6 minutes. Every
//! comparison is a wrapping difference read as i32 ([`elapsed_us`]), valid within 35 minutes, and the hold and the
//! deadline are both elapsed times, so the clock may wrap mid-settle. A debounce of any size is safe: a window past
//! the clock's half range never runs out, and never reads as run out. How long a debounce is sane is the config
//! validator's to bound (board_config's `LONGEST_DEBOUNCE_MS`, 1000 ms: a 10 s deadline).

/// Agreeing reads a polled input's verdict needs, however long the window: one read is never enough. Also the reads a
/// settle takes before its deadline may conclude it, on either kind of input.
pub const MINIMUM_SAMPLES: u32 = 3;

/// A settle with no verdict gives up this many debounce windows after its first read, once it has taken
/// [`MINIMUM_SAMPLES`] reads.
pub const SETTLE_WINDOWS: u32 = 10;

/// What the input's edge interrupt saw between the previous read and this one: a read's evidence besides its level.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Edges {
    /// The input has no armed interrupt (none wired, or arming it failed): the read is the only evidence, on every read.
    Polled,
    /// The interrupt is armed and saw no edge since the previous read: the line held its level between the two.
    Quiet,
    /// The interrupt saw at least one edge since the previous read, stamped on the reads' clock:
    /// `first_us <= last_us <=` the read's `now_us`.
    Seen {
        /// The first edge's stamp, µs: where a settle this read starts departs.
        first_us: u32,
        /// The last edge's stamp, µs: the line has held the read's level since.
        last_us: u32,
    },
}

/// What a settle concluded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// The line held one level for the debounce.
    Held {
        /// The level it held (true = HIGH).
        level: bool,
        /// When the line first left the latched level in this settle, µs: the first edge of the read that started
        /// it, else that read, or an earlier edge a later read brought (see the module notes, THE DEPARTURE).
        departed_us: u32,
        /// When the held level began for good, µs: the last edge on an armed input, the first read of the final run
        /// on a polled one (THE SETTLING). At or after `departed_us` under the edge contract.
        settled_us: u32,
    },
    /// The line never held still for the debounce before the settle's deadline.
    Unsettled,
}

/// The signed microseconds from `then_us` to `now_us` on the wrapping u32 clock: negative when `now_us` is the earlier
/// one. Valid while the two are less than half the clock's period apart, 35.8 minutes.
pub fn elapsed_us(now_us: u32, then_us: u32) -> i32 {
    now_us.wrapping_sub(then_us) as i32
}

/// Whether [`elapsed_us`] from `since_us` to `now_us` has reached `threshold_us`. Widened to i64, so a negative
/// interval compares below every threshold, and a threshold past the clock's half range is never reached.
fn reached_us(now_us: u32, since_us: u32, threshold_us: u32) -> bool {
    i64::from(elapsed_us(now_us, since_us)) >= i64::from(threshold_us)
}

/// One settle under way: the settle's own record, and the run of agreeing reads it is on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Settle {
    /// When its first read came, µs: the deadline counts from it.
    began_us: u32,
    /// The reads it has taken, its first included: the deadline waits for [`MINIMUM_SAMPLES`] of them.
    reads: u32,
    /// When the line first left the latched level in it, µs (THE DEPARTURE).
    departed_us: u32,
    /// The level the run under way agrees on (true = HIGH).
    candidate: bool,
    /// When the run began, µs: its last edge, else its first read. The hold counts from it.
    since_us: u32,
    /// The run's agreeing reads, its first included: a polled verdict needs [`MINIMUM_SAMPLES`] of them.
    agreeing: u32,
}

impl Settle {
    /// A settle whose first read came at `began_us`, departed at `departed_us`, on a run of `candidate` since
    /// `since_us`: one read taken, one agreeing.
    fn new(began_us: u32, departed_us: u32, candidate: bool, since_us: u32) -> Settle {
        Settle {
            began_us,
            reads: 1,
            departed_us,
            candidate,
            since_us,
            agreeing: 1,
        }
    }
}

/// The debounce of a two-level input, one read per step (see the module notes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Debounce {
    /// How long a level must hold, µs.
    debounce_us: u32,
    /// The settle under way, None while the line is idle.
    settle: Option<Settle>,
}

impl Debounce {
    /// An idle debounce whose level must hold `debounce_us`: nothing happens until an edge is seen or a read differs
    /// from the latched level.
    pub fn new(debounce_us: u32) -> Debounce {
        Debounce {
            debounce_us,
            settle: None,
        }
    }

    /// Whether a settle is under way.
    pub fn settling(&self) -> bool {
        self.settle.is_some()
    }

    /// Take one read: `level` is the input at `now_us` (true = HIGH), `edges` what the interrupt saw since the
    /// previous read ([`Edges::Polled`] on every read of an input without one: the edge contract), and `latched` the
    /// level the owner's latch stands for.
    ///
    /// Idle, an edge or a level other than `latched` (the level rule) starts a settle on this read: its departure is
    /// the first edge, else this read, and its run begins at the last edge, else this read. An idle read with neither
    /// does nothing. Under way, an edge that ends a run held at `latched` for the debounce concludes it and starts the
    /// next settle (the return); else an edge or a read of the other level restarts the run (the restart rule), an
    /// agreeing read counts, and the hold or the deadline concludes the settle (see the module notes).
    ///
    /// Returns the verdict on the read that concludes a settle, else None.
    pub fn step(&mut self, now_us: u32, level: bool, edges: Edges, latched: bool) -> Option<Verdict> {
        let Some(mut settle) = self.settle else {
            self.settle = match edges {
                Edges::Seen { first_us, last_us } => Some(Settle::new(now_us, first_us, level, last_us)),
                Edges::Polled | Edges::Quiet if level != latched => Some(Settle::new(now_us, now_us, level, now_us)),
                Edges::Polled | Edges::Quiet => None,
            };
            return None;
        };
        if let Edges::Seen { first_us, last_us } = edges {
            if settle.candidate == latched && reached_us(first_us, settle.since_us, self.debounce_us) {
                // the return: home for the debounce before this read's first edge, which departs anew
                self.settle = Some(Settle::new(now_us, first_us, level, last_us));
                return Some(Verdict::Held {
                    level: latched,
                    departed_us: settle.departed_us,
                    settled_us: settle.since_us,
                });
            }
            // an edge that changed the level an earlier read saw after that read took its edges
            if elapsed_us(first_us, settle.departed_us) < 0 {
                settle.departed_us = first_us;
            }
        }
        settle.reads = settle.reads.saturating_add(1);
        let agrees = level == settle.candidate && !matches!(edges, Edges::Seen { .. });
        if agrees {
            settle.agreeing = settle.agreeing.saturating_add(1);
        } else {
            settle.candidate = level;
            settle.since_us = match edges {
                Edges::Seen { last_us, .. } => last_us,
                Edges::Polled | Edges::Quiet => now_us,
            };
            settle.agreeing = 1;
        }
        // the interrupt proves an armed line held between the reads; a polled one needs the reads themselves
        let enough = agrees && (edges != Edges::Polled || settle.agreeing >= MINIMUM_SAMPLES);
        let window_us = self.debounce_us.saturating_mul(SETTLE_WINDOWS);
        let verdict = if enough && reached_us(now_us, settle.since_us, self.debounce_us) {
            Some(Verdict::Held {
                level: settle.candidate,
                departed_us: settle.departed_us,
                settled_us: settle.since_us,
            })
        } else if settle.reads >= MINIMUM_SAMPLES && reached_us(now_us, settle.began_us, window_us) {
            Some(Verdict::Unsettled)
        } else {
            None
        };
        self.settle = match verdict {
            None => Some(settle),
            // a deadline away from the latch keeps the run: the next settle departs where it began
            Some(Verdict::Unsettled) if settle.candidate != latched => Some(Settle {
                began_us: now_us,
                reads: 1,
                departed_us: settle.since_us,
                ..settle
            }),
            Some(_) => None,
        };
        verdict
    }
}

#[cfg(test)]
mod tests;
