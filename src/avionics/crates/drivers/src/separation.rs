//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage-separation switch (src/glider/drivers/separation.py): two adhesive copper pads, one on the glider and
//! one on the booster, that route 3V3 to a pin while nested (HIGH) and open on separation (LOW, held there by the
//! pin's internal pull-down). A confirmed separation in BOOSTING drives the documented BOOSTING -> GLIDING
//! transition (the booster ejects the glider at apogee); anywhere else it is recorded and moves nothing, so a ground
//! test in SETTING keeps go/no-go correct.
//!
//! THE DEBOUNCE ([`Debounce`]) is the hard-won part. Plan B once read the pin ONCE, debounce_ms after the first edge,
//! and committed on that sample: a ~7 ms open blip that happened to span the sample instant was a separation, which
//! in boost forced BOOSTING -> GLIDING, a transition that cannot be undone (reproduced on the board). Now a verdict
//! needs at least [`MINIMUM_SAMPLES`] agreeing reads spanning `debounce_ms`, the window restarts on every change of
//! level, and a line still chattering after [`SETTLE_WINDOWS`] windows is no verdict at all -- its own edges start
//! another settle once it quietens. A real separation leaves the pads open for good, so this costs a few tens of
//! milliseconds; a vibration blip can no longer win.
//!
//! Rust runs it differently and decides the same (avionics-port.md §4.3): plan B's IRQ woke a coroutine that re-read
//! the pin every 5 ms; here the GPIO ISR only flags the edge, and the control thread steps the debounce with one pin
//! read per tick. [`Debounce::step`] is one read: plan B's reads and Rust's are the same sequence of (time, level)
//! pairs, which is how the golden fixture separation.json replays plan B's own `_settle` against it. One edge flag
//! carries over a settle exactly as plan B's ThreadSafeFlag did: an edge during a settle starts the next one.
//!
//! The transition is the control thread's own write to the StageCell, not the stage machine's: separation is the
//! PRIMARY BOOSTING -> GLIDING trigger and separation.csv (event + stage, durable) its authoritative record; the
//! sequencer's burnout timeout is only the fallback, recorded in sequencer.csv. A post-flight tool reading the
//! BOOSTING -> GLIDING reason consults separation.csv first.
//!
//! Not here (firmware glue): the GPIO, its pull-down and both-edge interrupt, the telemetry push and the log line.

use sequencer::Stage;
use sequencer::commons::{elapsed_ms, reached};
use serde_json::Value;

/// The default debounce, ms: how long the level must hold (`debounce_ms`).
pub const DEFAULT_DEBOUNCE_MS: u32 = 20;

/// Agreeing reads a verdict needs, however long the window: one read is never enough.
pub const MINIMUM_SAMPLES: u32 = 3;

/// A line still chattering after this many debounce windows is not a verdict.
pub const SETTLE_WINDOWS: u32 = 10;

/// What a settle concluded (plan B's `_settle` return: True, False, None).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// The pads held open: separated.
    Separated,
    /// The pads held closed: nested.
    Nested,
    /// The line never held still for a window before the settle's deadline.
    Unsettled,
}

/// One settle under way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Settle {
    /// The level the reads agree on so far (true = HIGH = nested).
    candidate: bool,
    /// When the candidate level was first read in this run.
    since_ms: u32,
    /// How many consecutive reads agree on the candidate.
    agreeing: u32,
    /// When the settle gives up: debounce_ms x [`SETTLE_WINDOWS`] after it began.
    deadline_ms: u32,
}

/// The separation debounce: one pin read per step, the edge flag carried across settles (see the module notes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Debounce {
    /// How long the level must hold, ms.
    debounce_ms: u32,
    /// The settle under way, None while the line is idle.
    settle: Option<Settle>,
    /// An edge was flagged and no settle has started on it yet (plan B's ThreadSafeFlag, set by the IRQ).
    pending: bool,
}

impl Debounce {
    /// An idle debounce: nothing happens until an edge is flagged.
    pub fn new(debounce_ms: u32) -> Debounce {
        Debounce {
            debounce_ms,
            settle: None,
            pending: false,
        }
    }

    /// Whether a settle is under way.
    pub fn settling(&self) -> bool {
        self.settle.is_some()
    }

    /// Take one pin read: `high` is the level at `now_ms`, and `edge` says the interrupt flagged an edge since the
    /// previous read. Returns the verdict on the read that concludes a settle, else None.
    ///
    /// With no settle under way, a flagged edge (now or earlier) starts one on this read, which is its first agreeing
    /// read; an idle read with no edge does nothing. Under way, a read of the other level restarts the run on it; an
    /// agreeing read counts; [`MINIMUM_SAMPLES`] agreeing reads spanning `debounce_ms` conclude it, and so does the
    /// deadline, without a verdict. Edges flagged during a settle wait for the next read after it concludes, which
    /// starts the next settle -- as plan B's loop went straight back to its flag.
    pub fn step(&mut self, now_ms: u32, high: bool, edge: bool) -> Option<Verdict> {
        self.pending |= edge;
        let Some(mut settle) = self.settle else {
            if self.pending {
                self.pending = false;
                self.settle = Some(Settle {
                    candidate: high,
                    since_ms: now_ms,
                    agreeing: 1,
                    deadline_ms: now_ms.wrapping_add(self.debounce_ms.saturating_mul(SETTLE_WINDOWS)),
                });
            }
            return None;
        };
        if high != settle.candidate {
            settle = Settle {
                candidate: high,
                since_ms: now_ms,
                agreeing: 1,
                deadline_ms: settle.deadline_ms,
            };
        } else {
            settle.agreeing += 1;
        }
        let verdict = if settle.agreeing >= MINIMUM_SAMPLES && reached(now_ms, settle.since_ms, self.debounce_ms) {
            Some(if settle.candidate {
                Verdict::Nested
            } else {
                Verdict::Separated
            })
        } else if elapsed_ms(now_ms, settle.deadline_ms) >= 0 {
            Some(Verdict::Unsettled)
        } else {
            None
        };
        self.settle = if verdict.is_some() { None } else { Some(settle) };
        verdict
    }
}

/// A confirmed change of the latch, as separation.csv records it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// The new latch level: true = pads open = separated.
    pub separated: bool,
    /// The stage AFTER the event: GLIDING when it advanced BOOSTING, else the stage as it was.
    pub stage: Stage,
    /// Whether the event advanced BOOSTING -> GLIDING; the caller writes the StageCell when it did.
    pub advanced: bool,
}

impl Event {
    /// The event name: `separated` or `nested`.
    pub fn name(&self) -> &'static str {
        if self.separated { "separated" } else { "nested" }
    }

    /// The separation.csv row: (`event`, `stage`).
    pub fn row(&self) -> [&'static str; 2] {
        [self.name(), self.stage.name()]
    }
}

/// The separation switch's settings, from its config block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Settings {
    /// How long the level must hold, ms (`debounce_ms`, default [`DEFAULT_DEBOUNCE_MS`]).
    pub debounce_ms: u32,
}

impl Settings {
    /// Read a config block; a missing or non-integer key takes plan B's default.
    pub fn from_config(config: &Value) -> Settings {
        Settings {
            debounce_ms: config
                .get("debounce_ms")
                .and_then(Value::as_u64)
                .and_then(|debounce| u32::try_from(debounce).ok())
                .unwrap_or(DEFAULT_DEBOUNCE_MS),
        }
    }
}

/// The separation switch: the debounced latch and its debounce.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Separation {
    /// The last debounced level: true = pads open = separated.
    separated: bool,
    /// The debounce the reads go through.
    debounce: Debounce,
}

impl Separation {
    /// A switch whose pin read `high` at setup: LOW (pads open) starts it separated.
    pub fn new(settings: &Settings, high: bool) -> Separation {
        Separation {
            separated: !high,
            debounce: Debounce::new(settings.debounce_ms),
        }
    }

    /// The last debounced level, true = separated (separation.separated): the AUTHORITATIVE reading for the
    /// warm-start gate, from this driver's own pin rather than a second reader of the same GPIO.
    pub fn separated(&self) -> bool {
        self.separated
    }

    /// Whether a settle is under way.
    pub fn settling(&self) -> bool {
        self.debounce.settling()
    }

    /// One tick's pin read (see [`Debounce::step`]); a held level is applied in `stage`.
    pub fn step(&mut self, now_ms: u32, high: bool, edge: bool, stage: Stage) -> Option<Event> {
        match self.debounce.step(now_ms, high, edge)? {
            Verdict::Separated => self.apply(true, stage),
            Verdict::Nested => self.apply(false, stage),
            Verdict::Unsettled => None,
        }
    }

    /// Act on a confirmed level (separation._apply): None when it is the latch's level already; else the latch
    /// moves, and a separation in BOOSTING advances to GLIDING. A re-nest is recorded but never reverses a stage.
    pub fn apply(&mut self, separated: bool, stage: Stage) -> Option<Event> {
        if separated == self.separated {
            return None;
        }
        self.separated = separated;
        let advanced = separated && stage == Stage::Boosting;
        Some(Event {
            separated,
            stage: if advanced { Stage::Gliding } else { stage },
            advanced,
        })
    }
}

#[cfg(test)]
mod tests;
