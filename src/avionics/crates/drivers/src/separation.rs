//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage-separation switch (src/glider/drivers/separation.py): two adhesive copper pads, one on the glider and
//! one on the booster, that route 3V3 to a pin while nested (HIGH) and open on separation (LOW, held there by the
//! pin's internal pull-down). A latch over the generic [`crate::debounce`]: the debounce settles the pin, this module
//! says what a held level means. A confirmed separation in BOOSTING drives the documented BOOSTING -> GLIDING
//! transition (the booster ejects the glider at apogee); anywhere else it is recorded and moves nothing, so a ground
//! test in SETTING keeps go/no-go correct. A re-nest is recorded and never reverses a stage.
//!
//! Why the debounce: plan B once read the pin ONCE, debounce_ms after the first edge, and committed on that sample:
//! a ~7 ms open blip that happened to span the sample instant was a separation, which in boost forced BOOSTING ->
//! GLIDING, a transition that cannot be undone (reproduced on the board). Through the debounce the pads must hold open
//! for the debounce after their last edge, as the interrupt's stamps prove (polled, three reads spanning it), so a
//! vibration blip can no longer win; a real separation leaves the pads open for good.
//!
//! THE PHYSICS, as the operator measures it: the pads are 2 cm long and part at about 3 m/s, so they slide apart over
//! about 7 ms, and contact may be lost and restored while they do: within those 7 ms at worst, most likely over less
//! of the travel. THE SEPARATION MOMENT ([`Event::moment_us`]) is the first 1 -> 0 of the episode that ended 0 for
//! good: each bounce restarts the debounce's run, but the debounce keeps the departure across the restarts
//! (crate::debounce, THE DEPARTURE), so the moment is the wobble's first edge -- with no interrupt, the first read that
//! saw LOW -- and not the read that confirmed it. A return to nested that held the debounce ends an episode (a boost
//! blip), so the separation after it is dated by its own first edge, whatever the tick's phase. THE DURATION
//! ([`Event::duration_us`]) is the wobble, from the moment to the sustained 0: its last edge, so 0 for a clean single
//! edge. On a polled pin it runs to the first read of the final run of LOW reads, so it is a multiple of the tick, and
//! a wobble shorter than the tick reads 0. For a re-nest both describe the closing bounce, by the same rule.
//!
//! THE DEBOUNCE is 10 ms by default ([`DEFAULT_DEBOUNCE_MS`]: the operator's "10 ms or less"; plan B's was 20). It
//! must outlast the longest contact gap inside the wobble: a gap that holds the debounce ends the episode there, so
//! the moment moves to a later departure, and where a quiet read falls inside it (a faster tick: schedule_hz goes up
//! to 1 kHz) it splits one separation into separated / nested / separated rows (the stage never reverses either way).
//! The wobble is 7 ms at most, so 10 ms keeps it one event dated by its first edge. On the default 100 Hz tick every
//! debounce up to 10 ms decides on the same read (a quiet read comes a whole tick after the last edge), so less would
//! buy no latency there and would still move the moment. Every shipped document still writes plan B's 20 --
//! config_default, which a board also boots when no stored config passes validate, and the 20261003 launch records --
//! so a board debounces 10 ms once its config says 10 or drops the key.
//!
//! THE LATENCY, at the default 10 ms debounce on the 10 ms control tick (the one place this crate states it): with the
//! interrupt, a separation is decided 10-20 ms after the wobble's last edge, the first quiet read at least 10 ms after
//! it, a chatter's deadline or not (a deadline carries the run under way into the next settle); polled, 20 ms after
//! the first read of the final run of LOW reads, which comes up to a tick after the pads opened for good. An edge
//! that lands between the tick taking its edges and reading the level costs a tick: that read sees the level with no
//! edge, the next sees the edge, and only the one after is quiet. separation.csv shows it: a row's `uptime` is the
//! verdict's read, its `moment` the departure, its `duration` the wobble.
//!
//! THE EDGE CONTRACT ([`Separation::step`], crate::debounce): each read passes [`Edges::Seen`] with the first and the
//! last edge the both-edge interrupt stamped since the previous read, [`Edges::Quiet`] when it stamped none, and
//! [`Edges::Polled`] on every read when the interrupt is not armed. The level rule needs no edge: a change between setup
//! and arming the interrupt, or under an arm that failed, is settled all the same.
//!
//! Fed plan B's own reads (golden separation.json, each edge flag stamped at its read), the port records plan B's
//! separation.csv events on the same reads in 34 of its 42 scenarios. In seven others, on that fixture's reads, it
//! records an event at most one plan B read earlier (a debounce of at most one of plan B's 5 ms reads: only then is
//! the edge's next read a debounce after it, where plan B waits for a third read) or one tick later, or drops a change
//! plan B read through a bounce together with the change that undid it: never a separation, nor a GLIDING, plan B did
//! not make. The eighth is plan B's 0 ms debounce, whose own arithmetic never settled a held open: the port separates
//! it on its first quiet read.
//!
//! `debounce_ms` is read as every driver reads its block ([`Settings::from_config`], the default for a missing or
//! malformed value) and converted to the debounce's µs once, by [`Separation::new`]; how long a debounce is sane is
//! board_config's validate to refuse (`LONGEST_DEBOUNCE_MS`, 1000 ms), so every config a board runs has passed it.
//!
//! The transition is the control thread's own write to the StageCell, not the stage machine's: separation is the
//! PRIMARY BOOSTING -> GLIDING trigger and separation.csv (event, stage, moment, duration; durable) its authoritative
//! record; the sequencer's burnout timeout is only the fallback, recorded in sequencer.csv. A post-flight tool reading
//! the BOOSTING -> GLIDING reason consults separation.csv first.
//!
//! Not here (firmware glue): the GPIO, its pull-down and both-edge interrupt, the edges' stamps, the telemetry push
//! (the moment rebuilt on the uptime column's clock, the duration in µs) and the log line.

use sequencer::Stage;
use serde_json::Value;

use crate::debounce::{Debounce, Edges, Verdict};

/// The default debounce, ms: how long the level must hold after its last edge (`debounce_ms`), for a block without
/// the key -- every shipped document still writes plan B's 20. The operator's "10 ms or less", outlasting the pads'
/// wobble of 7 ms at most (see the module notes, THE DEBOUNCE).
pub const DEFAULT_DEBOUNCE_MS: u32 = 10;

/// Microseconds in a millisecond: `debounce_ms` to the debounce's clock.
const MICROSECONDS_PER_MILLISECOND: u32 = 1000;

/// A confirmed change of the latch, as separation.csv records it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// The new latch level: true = pads open = separated.
    pub separated: bool,
    /// The stage AFTER the event: GLIDING when it advanced BOOSTING, else the stage as it was.
    pub stage: Stage,
    /// Whether the event advanced BOOSTING -> GLIDING; the caller writes the StageCell when it did.
    pub advanced: bool,
    /// When the pin first left the latch's old level in the episode this event concludes, µs on the steps' clock:
    /// for a separation the separation moment, for a re-nest when the pads first closed (see the module notes). At or
    /// before the read that recorded the event.
    pub moment_us: u32,
    /// How long the pin wobbled before it held the new level, µs: from [`Event::moment_us`] to the last edge, 0 for a
    /// clean single edge; on a polled pin, to the first read of the final run, a multiple of the tick.
    pub duration_us: u32,
}

impl Event {
    /// The event name: `separated` or `nested`.
    pub fn name(&self) -> &'static str {
        if self.separated { "separated" } else { "nested" }
    }

    /// The separation.csv row's text cells, plan B's two columns: (`event`, `stage`). The `moment` and `duration`
    /// columns after them are the caller's: [`Event::moment_us`] on the uptime column's clock, which the caller knows,
    /// and [`Event::duration_us`].
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
    /// Read a config block: `debounce_ms`, [`DEFAULT_DEBOUNCE_MS`] when it is missing or not a whole number of ms that
    /// fits a u32 (negative, fractional, text). It bounds nothing: board_config's validate refuses an absurd one.
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
    /// A switch whose pin read `high` at setup: LOW (pads open) starts it separated. Its debounce is
    /// `settings.debounce_ms` in µs; past the µs clock's range (4 294 967 ms, which the validator's bound keeps out of
    /// every config a board runs) it saturates, a debounce that never concludes.
    pub fn new(settings: &Settings, high: bool) -> Separation {
        Separation {
            separated: !high,
            debounce: Debounce::new(settings.debounce_ms.saturating_mul(MICROSECONDS_PER_MILLISECOND)),
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

    /// One tick's pin read through the debounce: `high` is the level at `now_us`, `edges` what the interrupt stamped
    /// since the previous read ([`Edges::Polled`] on every read of a pin without one: the edge contract), and a held
    /// level is applied in `stage`. The latch stands for HIGH while nested.
    ///
    /// Returns the event when a settle concludes at the level the latch does not hold; None on every other read, a
    /// settle that ends Unsettled or at the latch's own level included.
    pub fn step(&mut self, now_us: u32, high: bool, edges: Edges, stage: Stage) -> Option<Event> {
        match self.debounce.step(now_us, high, edges, !self.separated)? {
            Verdict::Held {
                level,
                departed_us,
                settled_us,
            } => self.apply(!level, stage, departed_us, settled_us.wrapping_sub(departed_us)),
            Verdict::Unsettled => None,
        }
    }

    /// Act on a confirmed level (separation._apply): None when it is the latch's level already; else the latch
    /// moves, dated `moment_us` after a wobble of `duration_us`, and a separation in BOOSTING advances to GLIDING. A
    /// re-nest is recorded but never reverses a stage.
    fn apply(&mut self, separated: bool, stage: Stage, moment_us: u32, duration_us: u32) -> Option<Event> {
        if separated == self.separated {
            return None;
        }
        self.separated = separated;
        let advanced = separated && stage == Stage::Boosting;
        Some(Event {
            separated,
            stage: if advanced { Stage::Gliding } else { stage },
            advanced,
            moment_us,
            duration_us,
        })
    }
}

#[cfg(test)]
mod tests;
