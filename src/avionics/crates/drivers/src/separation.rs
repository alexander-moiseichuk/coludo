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
//! level, and a line still chattering after [`SETTLE_WINDOWS`] windows is no verdict at all -- another settle takes
//! it up once it quietens. A real separation leaves the pads open for good, so this costs a few tens of
//! milliseconds; a vibration blip can no longer win.
//!
//! Rust runs it on the control tick (avionics-port.md §4.3): plan B's IRQ woke a coroutine that re-read the pin
//! every 5 ms; here the GPIO ISR only flags the edge, and the control thread steps the debounce with one pin read
//! per tick. [`Debounce::step`] is one read. At a 10 ms tick a read no longer sees what plan B's 5 ms reads saw, so
//! two rules are the port's own (D15):
//!  * THE LEVEL RULE ([`Separation::step`]): a level that disagrees with the latch starts a settle with no edge
//!    needed. Plan B started one only on its flag, so a change between setup and arming the interrupt, or under an
//!    arm that failed, was never seen;
//!  * THE RESTART RULE ([`Debounce::step`]): an edge flagged since the previous read restarts the run under way on
//!    this read, because the level did not hold between the reads even when both read it. Without it, chatter of
//!    a few ms between 10 ms reads reads as a held level: toggles of 0.5-2 ms false-deployed 23-81 % of runs with
//!    plan B's rule on the tick, none with this one. An edge seen during a settle no longer starts the next
//!    settle, as plan B's ThreadSafeFlag did; the level rule starts it when the line has anything left to settle.
//!
//! THE EDGE CONTRACT: `edge` is true only when an armed interrupt flagged a change since the previous read. A
//! polled pin (no interrupt, an arm that failed) passes false on every read: the level rule alone settles it, and
//! chatter between its reads is invisible to it. An edge forced on every read restarts every run, so no settle
//! ever concludes but on its deadline, as Unsettled: the switch would never separate.
//!
//! THE LATENCY, at the default 20 ms debounce on the 10 ms tick (the one place this crate states it): a held open
//! is separated 20-30 ms after it opened, with or without the interrupt; after a bounce, 20-30 ms after the last
//! edge, up to 50 ms when the bounce ran the settle out to its deadline (the deadline's read concludes nothing,
//! the next starts a settle on the level rule).
//!
//! A debounce whose settle window cannot hold [`MINIMUM_SAMPLES`] tick reads cannot settle reliably, so
//! [`Settings::for_tick`] refuses it. Fed plan B's own reads (golden separation.json), the port records plan B's
//! separation.csv events on the same reads in 36 of its 42 scenarios. In the other six, on that fixture's reads, it
//! records an event at most one tick later, or drops a change plan B read through a bounce together with the
//! change that undid it: never earlier, never a GLIDING plan B did not make.
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

/// The separation debounce: one pin read per step, an edge restarting the run under way (see the module notes).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Debounce {
    /// How long the level must hold, ms.
    debounce_ms: u32,
    /// The settle under way, None while the line is idle.
    settle: Option<Settle>,
}

impl Debounce {
    /// An idle debounce: nothing happens until an edge is flagged or a read disagrees with the latch.
    pub fn new(debounce_ms: u32) -> Debounce {
        Debounce {
            debounce_ms,
            settle: None,
        }
    }

    /// Whether a settle is under way.
    pub fn settling(&self) -> bool {
        self.settle.is_some()
    }

    /// Take one pin read: `high` is the level at `now_ms`, `edge` says an armed interrupt flagged an edge since the
    /// previous read (false on every read of a polled pin: the edge contract, see the module notes), and
    /// `disagrees` that the level is not the latch's (the level rule, [`Separation::step`]). Returns the verdict on
    /// the read that concludes a settle, else None.
    ///
    /// Idle, an edge or a disagreeing level starts a settle on this read, its first agreeing read; an idle read
    /// with neither does nothing. Under way, `disagrees` plays no part: a read of the other level, or one with an
    /// edge since the previous read (the restart rule: the level did not hold between the reads), restarts the run
    /// on it; an agreeing read counts; [`MINIMUM_SAMPLES`] agreeing reads spanning `debounce_ms` conclude it, and so
    /// does the deadline, without a verdict. An edge seen during a settle starts nothing after it.
    pub fn step(&mut self, now_ms: u32, high: bool, edge: bool, disagrees: bool) -> Option<Verdict> {
        let Some(mut settle) = self.settle else {
            if edge || disagrees {
                self.settle = Some(Settle {
                    candidate: high,
                    since_ms: now_ms,
                    agreeing: 1,
                    deadline_ms: now_ms.wrapping_add(self.debounce_ms.saturating_mul(SETTLE_WINDOWS)),
                });
            }
            return None;
        };
        if edge || high != settle.candidate {
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
    /// Read a config block; a missing or non-integer `debounce_ms` takes plan B's default. It checks no bound:
    /// bring-up takes [`Settings::for_tick`], which reads through this.
    pub fn from_config(config: &Value) -> Settings {
        Settings {
            debounce_ms: config
                .get("debounce_ms")
                .and_then(Value::as_u64)
                .and_then(|debounce| u32::try_from(debounce).ok())
                .unwrap_or(DEFAULT_DEBOUNCE_MS),
        }
    }

    /// Read a config block ([`Settings::from_config`]) for a switch read once every `tick_ms`, refusing a debounce
    /// that cannot settle on that tick.
    ///
    /// Refused, as `debounce_ms <n> cannot settle on the <t> ms tick`, when the settle window (`debounce_ms` x
    /// [`SETTLE_WINDOWS`]) cannot hold [`MINIMUM_SAMPLES`] reads a nominal tick apart: such a debounce cannot settle
    /// reliably. At the 10 ms tick that refuses 0 and 1 and takes 2 or more. The floor is conservative: a settle
    /// concludes once its second read comes before the deadline, so jitter that brings two reads closer (1 settles
    /// on reads at 1, 10 and 20 ms), or a longer tick (3 on exact 20 ms ticks), settles some refused values -- by
    /// luck, not by design. Plan B took any value: 0 never settled there either, and 1 settled on its 5 ms reads. A
    /// tick under 1 ms counts as 1 ms, the clock's resolution, so 0 is refused on every tick.
    ///
    /// Refused too, as `debounce_ms <n> is too long for the ms clock`, when the window passes `i32::MAX` ms, the
    /// wrapping clock's half range: the deadline can then read as passed on the settle's second read, and no settle
    /// would ever conclude (plan B's `ticks_add` raised there).
    pub fn for_tick(config: &Value, tick_ms: u32) -> Result<Settings, String> {
        let settings = Settings::from_config(config);
        let debounce_ms = settings.debounce_ms;
        let window_ms = u64::from(debounce_ms) * u64::from(SETTLE_WINDOWS);
        if window_ms > i32::MAX as u64 {
            return Err(format!("debounce_ms {debounce_ms} is too long for the ms clock"));
        }
        if window_ms < u64::from(MINIMUM_SAMPLES - 1) * u64::from(tick_ms.max(1)) {
            return Err(format!(
                "debounce_ms {debounce_ms} cannot settle on the {tick_ms} ms tick"
            ));
        }
        Ok(settings)
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

    /// One tick's pin read (see [`Debounce::step`]): `high` is the level at `now_ms`, `edge` says an armed interrupt
    /// flagged an edge since the previous read, and a held level is applied in `stage`.
    ///
    /// The edge contract: a caller with no interrupt (a polled pin, an arm that failed) passes `edge` false on every
    /// read, never true: the level rule below covers it, while an edge forced on every read restarts every run and
    /// the switch never separates.
    ///
    /// The level rule: a level that disagrees with the latch starts a settle with no edge needed, so a change
    /// between setup and arming the interrupt, or under an arm that failed, is settled all the same. A level equal
    /// to the latch, with no edge, starts nothing.
    pub fn step(&mut self, now_ms: u32, high: bool, edge: bool, stage: Stage) -> Option<Event> {
        // HIGH is nested: the level disagrees when it reads as the latch's opposite
        let disagrees = high == self.separated;
        match self.debounce.step(now_ms, high, edge, disagrees)? {
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
