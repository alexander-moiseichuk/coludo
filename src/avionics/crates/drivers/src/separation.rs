//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage-separation switch (src/glider/drivers/separation.py): a copper ring on the booster that bridges two copper
//! strips on the glider, routing 3V3 to a pin while nested (HIGH) and opening on separation (LOW, held there by the
//! pin's internal pull-down). A latch over the generic [`crate::debounce`]: the interrupt and the tick stamp the
//! contact's episodes there, this module says what a concluded one means. A separation in BOOSTING drives the
//! documented BOOSTING -> GLIDING transition (the booster ejects the glider at apogee); anywhere else it is recorded
//! and moves nothing, so a ground test in SETTING keeps go/no-go correct. A re-nest is recorded and never reverses a
//! stage.
//!
//! THE DECISION ([`Separation::conclude`]): the level read when the episode concludes, a quiet after its last edge,
//! decides it. LOW with the latch nested is `separated`, HIGH with the latch separated is `nested`, and the latch's own
//! level is a `blip`: the contact moved and came back. Blips are recorded too, so a flight shows every movement of the
//! contact: a boost blip is a blip row and never a separation, but for one instant. The level is read for the tick
//! just before the take, so a blip whose quiet runs out on that very tick, with the next blip's 1 -> 0 inside the
//! interrupt's entry latency before the read (well under a µs a tick), concludes separated (crate::debounce, THE
//! TICK), and in BOOSTING that GLIDING stands; the spec's thrust-end gate for boost closes that instant. A board
//! powered up unassembled reads LOW at bring-up, so its latch starts separated, and assembling it concludes `nested`
//! (with the interrupt, or by the tick's own read when no edge came): the latch ends at 1.
//!
//! THE PHYSICS, as the operator measures it: at separation the glider slides out over the strips' 2 cm at about 3 m/s,
//! so contact may be lost and restored for about 7 ms, under 10 at worst. THE SEPARATION MOMENT ([`Event::moment_us`])
//! is the episode's first edge, the first 1 -> 0; THE DURATION ([`Event::duration_us`]) runs from it to the episode's
//! last edge, so 0 for a clean single edge. With the interrupt both are its stamps, exact to the µs; on a polled pin,
//! or a change the tick's own read saw, they are the reads that saw the changes, multiples of the tick. For a re-nest
//! or a blip both describe its own episode alike.
//!
//! THE QUIET is `debounce_ms`, 10 ms by default ([`DEFAULT_DEBOUNCE_MS`]: the operator's "10 ms or less"; plan B's was
//! 20): how long the pin must see no edge before the episode concludes. It must outlast the longest contact gap inside
//! the wobble: a gap that holds the quiet up to a tick ends the episode there, so one separation splits into rows (a
//! blip and then the separation, which its own first edge then dates; the stage never reverses either way). It is also
//! the line between a blip and a separation: a contact that stays open past the quiet, until a tick reads it, is a
//! separation. Every shipped document still writes plan B's 20 -- config_default, which a board also boots when no
//! stored config passes validate, and the 20261003 launch records -- so a board waits 10 ms once its config says 10
//! or drops the key.
//!
//! THE LATENCY, at the default 10 ms quiet on the 10 ms control tick (the one place this crate states it): an episode
//! concludes on the first tick at least 10 ms after its last edge, 10-20 ms after it. Polled, the last change is seen
//! by the read after it, up to a tick late, and the episode concludes a quiet after that read: 10 ms later at the
//! default. separation.csv shows it: a row's `uptime` is the conclusion's tick, its `moment` the first edge, its
//! `duration` the wobble.
//!
//! Fed plan B's own reads (golden separation.json, each flagged edge recorded at its read), the port records plan B's
//! separations and re-nests on the same reads in 34 of its 42 scenarios, and mostly a blip where plan B's settle
//! ended on the latch's own level; tests/golden_separation.rs lists every difference. None is a GLIDING plan B did not
//! make, but under plan B's own 0 ms debounce, which never settled a held open.
//!
//! `debounce_ms` is read as every driver reads its block ([`Settings::from_config`], the default for a missing or
//! malformed value) and converted to the quiet's µs once, by [`Separation::new`]; how long a quiet is sane is
//! board_config's validate to refuse (`LONGEST_DEBOUNCE_MS`, 1000 ms), so every config a board runs has passed it.
//!
//! The transition is the control thread's own write to the StageCell, not the stage machine's: separation is the
//! PRIMARY BOOSTING -> GLIDING trigger and separation.csv (event, stage, moment, duration; durable) its authoritative
//! record; the sequencer's burnout timeout is only the fallback, recorded in sequencer.csv. A post-flight tool reading
//! the BOOSTING -> GLIDING reason consults separation.csv first.
//!
//! Not here (firmware glue): the GPIO, its pull-down and its both-edge interrupt recording into a static
//! [`crate::debounce::StampPair`], the tick's read, the telemetry push (the moment rebuilt on the uptime column's
//! clock, the duration in µs) and the log line.

use sequencer::Stage;
use serde_json::Value;

use crate::debounce::Episode;

/// The default quiet, ms: how long the pin must see no edge before an episode concludes (`debounce_ms`), for a block
/// without the key -- every shipped document still writes plan B's 20. The operator's "10 ms or less", outlasting the
/// contact's wobble of about 7 ms (see the module notes, THE QUIET).
pub const DEFAULT_DEBOUNCE_MS: u32 = 10;

/// Microseconds in a millisecond: `debounce_ms` to the quiet's clock.
const MICROSECONDS_PER_MILLISECOND: u32 = 1000;

/// What a concluded episode was, by the level read at its conclusion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// The pin read LOW with the latch nested: the latch moves to separated.
    Separated,
    /// The pin read HIGH with the latch separated: the latch moves to nested.
    Nested,
    /// The pin read the latch's own level: the contact moved and came back, and the latch stays.
    Blip,
}

/// A concluded episode, as separation.csv records it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// What the episode was.
    pub kind: Kind,
    /// The stage AFTER the event: GLIDING when it advanced BOOSTING, else the stage as it was.
    pub stage: Stage,
    /// Whether the event advanced BOOSTING -> GLIDING; the caller writes the StageCell when it did.
    pub advanced: bool,
    /// The episode's first edge, µs on the stamps' clock: for a separation the separation moment (see the module
    /// notes, THE PHYSICS).
    pub moment_us: u32,
    /// From the first edge to the last, µs: the wobble, 0 for a single edge.
    pub duration_us: u32,
}

impl Event {
    /// The event name: `separated`, `nested` or `blip`.
    pub fn name(&self) -> &'static str {
        match self.kind {
            Kind::Separated => "separated",
            Kind::Nested => "nested",
            Kind::Blip => "blip",
        }
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
    /// The quiet after the latest edge before an episode concludes, ms (`debounce_ms`, plan B's key; default
    /// [`DEFAULT_DEBOUNCE_MS`]).
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

/// The separation switch: the latch and its quiet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Separation {
    /// The latch: true = separated (the pin LOW).
    separated: bool,
    /// The quiet after an episode's latest edge before it concludes, µs.
    quiet_us: u32,
}

impl Separation {
    /// A switch whose pin read `high` at setup: LOW (the contact open) starts it separated. Its quiet is
    /// `settings.debounce_ms` in µs; past the µs clock's range (4 294 967 ms, which the validator's bound keeps out of
    /// every config a board runs) it saturates, a quiet that never comes.
    pub fn new(settings: &Settings, high: bool) -> Separation {
        Separation {
            separated: !high,
            quiet_us: settings.debounce_ms.saturating_mul(MICROSECONDS_PER_MILLISECOND),
        }
    }

    /// The latch, true = separated (separation.separated): the AUTHORITATIVE reading for the warm-start gate, from
    /// this driver's own pin rather than a second reader of the same GPIO.
    pub fn separated(&self) -> bool {
        self.separated
    }

    /// The pin level the latch stands for, true = HIGH = nested: the `latched` the tick's own read is observed against
    /// (crate::debounce).
    pub fn latched(&self) -> bool {
        !self.separated
    }

    /// The quiet, µs: the tick takes an episode once its latest edge is this old.
    pub fn quiet_us(&self) -> u32 {
        self.quiet_us
    }

    /// Conclude `episode` by the pin level `high` the tick read for it, in `stage` (separation._apply): separated,
    /// nested or a blip (see the module notes, THE DECISION). A separation in BOOSTING advances to GLIDING; nothing
    /// else moves a stage, so a re-nest never reverses one.
    ///
    /// Returns the event: its kind, the stage after it, whether it advanced, and the episode's moment and duration.
    pub fn conclude(&mut self, episode: Episode, high: bool, stage: Stage) -> Event {
        let kind = match (self.separated, high) {
            (false, false) => Kind::Separated,
            (true, true) => Kind::Nested,
            (false, true) | (true, false) => Kind::Blip,
        };
        self.separated = !high;
        let advanced = kind == Kind::Separated && stage == Stage::Boosting;
        Event {
            kind,
            stage: if advanced { Stage::Gliding } else { stage },
            advanced,
            moment_us: episode.start_us,
            duration_us: episode.duration_us(),
        }
    }
}

#[cfg(test)]
mod tests;
