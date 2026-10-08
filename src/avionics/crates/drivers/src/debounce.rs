//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! A debounce for a two-level input behind a latch, in two stamps: when the input first moved and when it last moved.
//! It knows edges and times, not what a level means: the owner keeps the latch and decides what a concluded episode
//! means from the level it reads at the conclusion. A remove-before-flight pin or a breakwire would be owners alike.
//!
//! THE MODEL (the operator's): an EPISODE is the edges no quiet time separates. Its first edge sets the start of the
//! [`StampPair`], every edge its finish ([`StampPair::record`]: wait-free and allocation-free, so the input's both-edge
//! interrupt calls it). The owner's tick takes the pair once the finish is `quiet_us` old ([`StampPair::take`]), and
//! the [`Episode`] it gets is the first and the last edge; the pair is empty again for the next one. The owner reads
//! the level for that same tick and decides by it: a level the latch does not hold moves the latch, the level it
//! holds is a blip (the input moved and came back). No count of reads and no deadline: an input that never holds still
//! for the quiet keeps its episode open, dated by the first edge of the whole run. An episode ends when a tick SEES it
//! quiet, so an edge that comes after the quiet ran out but before the tick took the pair is still the episode's: the
//! episode is the edges no gap of a quiet and the tick's own delay separates.
//!
//! THE TICK'S OWN READ is evidence too ([`StampPair::observe`]): when no episode is open and the level the tick reads
//! is not the latch's, the tick opens one at its read. That covers a change before the interrupt was armed, an edge
//! the interrupt lost, and a latch that started from a level the input has since left: the latch follows the input a
//! quiet after the read that saw it, with or without an interrupt. A read of the latch's own level records nothing.
//!
//! A POLLED input (no interrupt: none wired, or arming it failed) feeds the same pair from its reads
//! ([`StampPair::poll`]): a read whose level differs from the previous read's is an edge at the read's time, any other
//! read is observed. One path for both: a polled episode is dated by its reads, so its moment and its duration are
//! quantized to them, and a change and its undoing between two reads go unseen.
//!
//! THE TICK, in order: read the level, read the clock, observe (or poll), take, and on an episode decide with that
//! same level. The level comes before the take: an edge after the take's load belongs to the next episode, so it can
//! never decide this one. The residual: an edge in the instant before the read, inside the interrupt's entry latency
//! (well under a µs on one core), is in the level yet reaches the pair after the take, so it decides the concluding
//! episode as well as starting the next one, which then decides again by its own read. At worst a blip whose quiet
//! ran out on this very tick concludes on the next episode's first edge: the latch moves, and the next episode moves
//! it back if the line returns; an owner whose move cannot be undone states what that costs it.
//!
//! THE RACE: the interrupt may record while the tick takes. A take is decided by its load of the finish: an edge
//! recorded before that load is the episode's (it is the latest, so the episode is not quiet and goes on), any edge
//! after it is the next episode's. The take then releases the start and reloads the finish: an edge between the load
//! and the release stored a finish but found the start set, so the take re-seeds the start with it, and an edge after
//! the release started the next episode itself. So no episode loses its start and no later edge joins a concluded
//! episode. The residual: when two or more edges land inside those few instructions (stretched by the interrupts' own
//! run time, a few µs), the next episode starts at a later one of them, late by at most that span; its finish, and
//! the episode concluded, are exact. The tick's own read opens an episode by an exchange from empty, and sets the
//! finish only where no edge stored one since: an edge the interrupt records meanwhile is never overwritten by the
//! read's older time. Only the tick observes, polls and takes.
//!
//! THE MEMORY ORDER is sequentially consistent throughout. A take releases the start, then reloads the finish; an edge
//! stores the finish, then exchanges the start: each side writes one word and then reads the other, which only one
//! total order makes safe across cores (else a take could miss an edge that also missed its release, and the next
//! episode would never start). The fences cost a few instructions per edge and per tick.
//!
//! ONE CORE is what THE RACE assumes: the interrupt runs to its end before the tick goes on, so a record is one step
//! for the tick. On the board it is (the interrupt and the tick share core 1). Across cores a record's two writes can
//! straddle the tick's few instructions, and two residuals come in, both for an edge inside them. A take that loads
//! the edge's finish but releases the start before the edge's exchange concludes the edge, and the exchange then
//! opens the next episode with it too; a take concludes only an edge a quiet old, so that needs a record that outlasts
//! the quiet: in practice a quiet of 0. The next take concludes that edge again (a single-edge episode at the
//! concluded finish), or with the edges after it, dated from it. An observation the edge straddles may date the
//! episode by the read's time or the edge's, whichever the race leaves, never with a finish before the start. Closing
//! the first would take a third word (the concluded finish); an interrupt moved off the tick's core is the owner's to
//! weigh against it.
//!
//! THE CLOCK is the caller's wrapping u32 of microseconds (esp_timer's, truncated): it wraps every 71.6 minutes. A
//! stamp of 0 means empty, so an edge that would stamp 0 stamps 1, a µs late. The quiet is an elapsed time, a wrapping
//! difference read as i32 ([`elapsed_us`]), valid within 35.8 minutes, so the clock may wrap mid-episode; a finish
//! after the take's `now_us` (an edge between the caller's clock read and the take) is a negative interval, never
//! quiet. How long a quiet is sane is the config validator's to bound (board_config's `LONGEST_DEBOUNCE_MS`).

use core::sync::atomic::{AtomicU32, Ordering};

/// The empty stamp: no episode open.
const EMPTY: u32 = 0;

/// The memory order of every access (see the module notes, THE MEMORY ORDER).
const ORDER: Ordering = Ordering::SeqCst;

/// The signed microseconds from `then_us` to `now_us` on the wrapping u32 clock: negative when `now_us` is the earlier
/// one. Valid while the two are less than half the clock's period apart, 35.8 minutes.
pub fn elapsed_us(now_us: u32, then_us: u32) -> i32 {
    now_us.wrapping_sub(then_us) as i32
}

/// The stamp of an edge at `now_us`: the time itself, but 1 for 0, the empty stamp.
fn stamp(now_us: u32) -> u32 {
    now_us.max(1)
}

/// The points inside the tick's calls where an interrupt may land: the race tests record an edge there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Point {
    /// A take read the start, not yet the finish.
    StartLoaded,
    /// A take found the episode quiet, and has not released the start.
    Concluded,
    /// A take released the start, and has not reloaded the finish.
    Released,
    /// A take reloaded a finish an edge stored since its load, and has not re-seeded the start with it.
    Reloaded,
    /// An observation opened an episode at its read, and has not set the finish.
    Opened,
}

/// One episode, as [`StampPair::take`] concludes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Episode {
    /// The first edge, µs.
    pub start_us: u32,
    /// The last edge, µs: `start_us` when only one edge came.
    pub finish_us: u32,
}

impl Episode {
    /// From the first edge to the last, µs (wrapping): 0 for a single edge.
    pub fn duration_us(&self) -> u32 {
        self.finish_us.wrapping_sub(self.start_us)
    }
}

/// The open episode's first and latest edge, shared by the input's interrupt and the owner's tick (see the module
/// notes). A static: [`StampPair::new`] is const.
#[derive(Debug, Default)]
pub struct StampPair {
    /// The episode's first edge, µs; [`EMPTY`] while none is open. Set only from empty; emptied only by a take.
    start: AtomicU32,
    /// The episode's latest edge, µs: stored by every edge, the first included, so while an episode is open it is
    /// never older than the start; left as it is when an episode concludes, and meaningless while none is open.
    finish: AtomicU32,
}

impl StampPair {
    /// An empty pair: no episode open.
    pub const fn new() -> StampPair {
        StampPair {
            start: AtomicU32::new(EMPTY),
            finish: AtomicU32::new(EMPTY),
        }
    }

    /// An edge at `now_us`: it is the finish, and the start when no episode is open. Wait-free, for an interrupt.
    ///
    /// The finish is stored first, then the start exchanged from empty: so a tick that sees the start sees this
    /// finish or a later one, never one from a concluded episode.
    pub fn record(&self, now_us: u32) {
        let stamp = stamp(now_us);
        self.finish.store(stamp, ORDER);
        let _ = self.start.compare_exchange(EMPTY, stamp, ORDER, ORDER);
    }

    /// The tick's own read, `level` at `now_us`, against the level the owner's latch stands for, `latched`: when they
    /// differ and no episode is open, it opens one at the read (THE TICK'S OWN READ). Anything else records nothing.
    pub fn observe(&self, now_us: u32, level: bool, latched: bool) {
        self.observe_interrupted(now_us, level, latched, |_| {});
    }

    /// [`StampPair::observe`], with `interrupt` run at its [`Point`]: the race tests' edge.
    ///
    /// It opens by an exchange from empty, so an episode the interrupt opened first is left alone; and it sets the
    /// finish only from the value it found before, so a later edge's finish is never replaced by the read's time. An
    /// edge stamped before the read whose finish lands after it (its record on another core) pulls the start back to
    /// itself: never a finish before the start.
    fn observe_interrupted(&self, now_us: u32, level: bool, latched: bool, mut interrupt: impl FnMut(Point)) {
        if level == latched {
            return;
        }
        let stamp = stamp(now_us);
        let found = self.finish.load(ORDER);
        if self.start.compare_exchange(EMPTY, stamp, ORDER, ORDER).is_ok() {
            interrupt(Point::Opened);
            if let Err(latest_us) = self.finish.compare_exchange(found, stamp, ORDER, ORDER)
                && elapsed_us(stamp, latest_us) > 0
            {
                // only a take empties a set start, and the tick is the only taker: the exchange cannot fail
                let _ = self.start.compare_exchange(stamp, latest_us, ORDER, ORDER);
            }
        }
    }

    /// A polled input's read, `level` at `now_us` (no interrupt): an edge when it differs from the `previous` read's
    /// level, else [`StampPair::observe`]d against `latched`. Nothing else writes the pair of a polled input, so the
    /// edge is a plain [`StampPair::record`]. The caller keeps `previous`: at first the level its latch started from.
    pub fn poll(&self, now_us: u32, level: bool, previous: bool, latched: bool) {
        if level == previous {
            self.observe(now_us, level, latched);
        } else {
            self.record(now_us);
        }
    }

    /// Take the open episode once it is quiet at `now_us`: its latest edge `quiet_us` old or more.
    ///
    /// Returns None while no episode is open, or while it is not quiet (an edge stamped after `now_us` is younger than
    /// any quiet). Else the episode, the pair emptied for the next one (re-seeded with an edge that came during the
    /// take: THE RACE in the module notes).
    pub fn take(&self, now_us: u32, quiet_us: u32) -> Option<Episode> {
        self.take_interrupted(now_us, quiet_us, |_| {})
    }

    /// [`StampPair::take`], with `interrupt` run at each of its [`Point`]s: the race tests' edges.
    fn take_interrupted(&self, now_us: u32, quiet_us: u32, mut interrupt: impl FnMut(Point)) -> Option<Episode> {
        let start_us = self.start.load(ORDER);
        if start_us == EMPTY {
            return None;
        }
        interrupt(Point::StartLoaded);
        let finish_us = self.finish.load(ORDER);
        // widened, so a negative interval is below every quiet and a quiet past the clock's half range is never met
        if i64::from(elapsed_us(now_us, finish_us)) < i64::from(quiet_us) {
            return None;
        }
        interrupt(Point::Concluded);
        // only a take empties a set start: no edge can change it before this store
        self.start.store(EMPTY, ORDER);
        interrupt(Point::Released);
        let latest_us = self.finish.load(ORDER);
        if latest_us != finish_us {
            // an edge since the load: the next episode's first, unless one after the release opened it already
            interrupt(Point::Reloaded);
            let _ = self.start.compare_exchange(EMPTY, latest_us, ORDER, ORDER);
        }
        Some(Episode { start_us, finish_us })
    }
}

#[cfg(test)]
mod tests;
