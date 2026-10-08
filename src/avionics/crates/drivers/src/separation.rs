//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The stage-separation switch (src/glider/drivers/separation.py): a copper ring on the booster that bridges two copper
//! strips on the glider, routing 3V3 to a pin while nested (HIGH) and opening on separation (LOW, held there by the
//! pin's internal pull-down). A latch over the generic [`crate::debounce`]: the interrupt and the tick stamp the
//! contact's episodes there, this module says what a concluded one means. A separation in BOOSTING drives the
//! documented BOOSTING -> GLIDING transition (the booster ejects the glider at apogee) once the thrust gate lets it;
//! anywhere else it is recorded and moves nothing, so a ground test in SETTING keeps go/no-go correct. A re-nest is
//! recorded and never reverses a stage.
//!
//! THE DECISION ([`Separation::conclude`]): the level read when the episode concludes, a quiet after its last edge,
//! decides it. LOW with the latch nested is `separated`, HIGH with the latch separated is `nested`, and the latch's own
//! level is a `blip`: the contact moved and came back. Blips are recorded too, so a flight shows every movement of the
//! contact: a boost blip is a blip row and never a separation, but for one instant. The level is read for the tick
//! just before the take, so a blip whose quiet runs out on that very tick, with the next blip's 1 -> 0 inside the
//! interrupt's entry latency before the read (well under a µs a tick), concludes separated (crate::debounce, THE
//! TICK). Under thrust the thrust gate holds that separation pending and the next blip's re-nest clears it; once the
//! gate is open, in BOOSTING that GLIDING stands. A board powered up unassembled reads LOW at bring-up, so its latch
//! starts separated, and assembling it concludes `nested` (with the interrupt, or by the tick's own read when no edge
//! came): the latch ends at 1.
//!
//! THE THRUST GATE ([`ThrustGate`], the operator's): in BOOSTING a separation counts only once the booster has stopped
//! pushing the glider, so vibration under thrust can never deploy. The gate reads the glider's own axial specific
//! force, a_x on the body frame's +x (the nose; +y left, +z up), from a fresh `accel` every tick. Thrust is SEEN when
//! a_x holds above the sequencer's `launch_g` for its `launch_ms` of net time (the launch detector's leaky dwell,
//! sequencer::commons::dwell_step), and has ENDED when a_x then holds below 0 g for 20 ms of net time: the drag
//! deceleration the glider feels the moment nothing pushes it, by burnout or by an early aerodynamic separation of a
//! destabilized stack, which so opens the gate itself. Both start afresh at each BOOSTING entry. A missing a_x drains
//! a dwell like a sample that misses. A wrong axis or sign, no IMU, or a bench hold never sees thrust, so the gate
//! stays shut and only the backstop decides: a mis-mounted IMU can never open the gate under thrust.
//!
//! THE PENDING SEPARATION ([`Separation::tick`]): a separation concluded in BOOSTING while the gate is shut moves
//! nothing and writes no row: it is pending. It counts (GLIDING, the row `separated;gliding`, dated by its episode's
//! first edge) only on a tick that reads the pin LOW with no episode open, the contact held LOW since its last edge:
//! the first such tick with the gate open, or by THE BACKSTOP, 100 ms after its last edge, far beyond the contact's
//! wobble and any blip, so a dead or confused IMU can never hide a real separation. A LOW read inside an open episode
//! (a bounce of the contact closing again) counts nothing: the episode's conclusion decides. One that ends LOW again
//! (the contact touched and let go) is part of the pending separation: its last edge becomes the separation's, the
//! backstop runs from there, and an open gate counts it on the conclusion's own tick. A re-nest clears it: the open and
//! its re-nest are one `blip` row from the first edge to the last, and the counted latch never moved
//! ([`Separation::separated`]). When BOOSTING ends otherwise (the sequencer's apogee or burnout, the flight timeout, an
//! operator hold), a pending separation counts on the first such tick, as a separation anywhere outside BOOSTING does:
//! the row in the new stage, nothing advanced. Outside BOOSTING the gate changes nothing, and with the gate held open
//! the tick is [`Separation::conclude`] exactly (plan B's parity, tests/golden_separation.rs).
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
//! `duration` the wobble. Under the thrust gate in BOOSTING: the nominal separation is the ejection charge, about 4 s
//! after thrust end, long after the gate opened, so it counts at its conclusion as above. An early separation counts
//! when the gate opens, 20-30 ms after a_x went negative (the 20 ms dwell and the tick's phase), or at its conclusion
//! if that is later. Under a shut gate an open counts by the backstop, 100-110 ms after its last edge; an open of up to
//! 100 ms is a blip, and a quiet of 100 ms or more meets the backstop at the conclusion itself.
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
//! The transition is the control thread's own commit to the flight word (sequencer::FlightWriter::separate), not the
//! stage machine's: separation is the PRIMARY BOOSTING -> GLIDING trigger and separation.csv (event, stage, moment,
//! duration; durable) its authoritative record; the sequencer's apogee and burnout timeout are only the fallbacks,
//! recorded in sequencer.csv. A post-flight tool reading the BOOSTING -> GLIDING reason consults separation.csv first.
//!
//! Not here (firmware glue): the GPIO, its pull-down and its both-edge interrupt recording into a static
//! [`crate::debounce::StampPair`], the tick's read, the fresh a_x the gate is fed, the commit, the telemetry push (the
//! moment rebuilt on the uptime column's clock, the duration in µs) and the log line.

use sequencer::Stage;
use sequencer::commons::{Dwell, dwell_step};
use serde_json::Value;

use crate::debounce::{Episode, StampPair, elapsed_us};

/// The default quiet, ms: how long the pin must see no edge before an episode concludes (`debounce_ms`), for a block
/// without the key -- every shipped document still writes plan B's 20. The operator's "10 ms or less", outlasting the
/// contact's wobble of about 7 ms (see the module notes, THE QUIET).
pub const DEFAULT_DEBOUNCE_MS: u32 = 10;

/// Microseconds in a millisecond: `debounce_ms` to the quiet's clock.
const MICROSECONDS_PER_MILLISECOND: u32 = 1000;

/// The thrust's end, g: a_x below it is no push but drag (see the module notes, THE THRUST GATE).
const THRUST_END_G: f32 = 0.0;

/// How long a_x must hold below [`THRUST_END_G`] for the thrust to have ended, ms of net time: two ticks of the
/// default schedule, long enough that one sample of vibration under thrust does not end it.
const THRUST_END_MS: u32 = 20;

/// The backstop, µs: a pending separation counts once the pin has held LOW this long after its last edge, whatever
/// the gate says (100 ms: far beyond the contact's wobble of about 7 ms and any blip).
const BACKSTOP_US: u32 = 100 * MICROSECONDS_PER_MILLISECOND;

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

/// What separation.csv records: a concluded episode, or under the thrust gate a pending separation counted or cleared.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// What the episode was.
    pub kind: Kind,
    /// The stage AFTER the event: GLIDING when it advanced BOOSTING, else the stage as it was.
    pub stage: Stage,
    /// Whether the event advanced BOOSTING -> GLIDING; the caller commits GLIDING to the flight word when it did.
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

/// The separation switch: the latch, its quiet, and a separation pending under the thrust gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Separation {
    /// The latch: true = separated (the pin LOW), a pending separation included.
    separated: bool,
    /// The quiet after an episode's latest edge before it concludes, µs.
    quiet_us: u32,
    /// A separation concluded in BOOSTING under the shut thrust gate and not yet counted: its first edge, and the
    /// latest edge of it and of the episodes that ended LOW since (see the module notes, THE PENDING SEPARATION).
    pending: Option<Episode>,
}

impl Separation {
    /// A switch whose pin read `high` at setup: LOW (the contact open) starts it separated. Its quiet is
    /// `settings.debounce_ms` in µs; past the µs clock's range (4 294 967 ms, which the validator's bound keeps out of
    /// every config a board runs) it saturates, a quiet that never comes.
    pub fn new(settings: &Settings, high: bool) -> Separation {
        Separation {
            separated: !high,
            quiet_us: settings.debounce_ms.saturating_mul(MICROSECONDS_PER_MILLISECOND),
            pending: None,
        }
    }

    /// The counted latch, true = separated (separation.separated): the latch, but for a separation still pending
    /// under the thrust gate, which is no separation yet. The AUTHORITATIVE reading for the warm-start gate, from this
    /// driver's own pin rather than a second reader of the same GPIO.
    pub fn separated(&self) -> bool {
        self.separated && self.pending.is_none()
    }

    /// Whether a separation is pending under the shut thrust gate (`control`'s `pending`).
    pub fn pending(&self) -> bool {
        self.pending.is_some()
    }

    /// The pin level the latch stands for, true = HIGH = nested: the `latched` the tick's own read is observed against
    /// (crate::debounce). A pending separation is in it: the latch follows the pin, the gate decides what counts.
    pub fn latched(&self) -> bool {
        !self.separated
    }

    /// The quiet, µs: the tick takes an episode once its latest edge is this old.
    pub fn quiet_us(&self) -> u32 {
        self.quiet_us
    }

    /// Conclude `episode` by the pin level `high` the tick read for it, in `stage` (separation._apply): separated,
    /// nested or a blip (see the module notes, THE DECISION). A separation in BOOSTING advances to GLIDING; nothing
    /// else moves a stage, so a re-nest never reverses one. Ungated: [`Separation::tick`] adds the thrust gate.
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

    /// One tick under the thrust gate at `now_us` in `stage`: take `pair`'s episode once it is quiet, conclude it by
    /// the level `high` the tick read for it ([`Separation::conclude`]), and hold a separation in BOOSTING pending
    /// while the gate is shut (`gate_open`, [`ThrustGate::open`]) until a LOW read with no episode open counts it or a
    /// re-nest clears it (see the module notes, THE PENDING SEPARATION). The caller has offered the same read to the
    /// pair first (observed, or polled), against [`Separation::latched`].
    ///
    /// Returns the event to record, at most one a tick: an episode concluded as [`Separation::conclude`] gives it; a
    /// pending separation counted (`separated`, advanced in BOOSTING), dated by its first edge; or one a re-nest
    /// cleared (`blip`, from its first edge to the re-nest's last). None for nothing to record, a separation that
    /// becomes pending included.
    pub fn tick(&mut self, pair: &StampPair, high: bool, now_us: u32, stage: Stage, gate_open: bool) -> Option<Event> {
        if let Some(episode) = pair.take(now_us, self.quiet_us) {
            let event = self.conclude(episode, high, stage);
            match self.pending {
                None if event.advanced && !gate_open => self.pending = Some(episode),
                None => return Some(event),
                Some(pending) if event.kind == Kind::Nested => {
                    // the contact closed again: one blip from the open to the re-nest, the counted latch never moved
                    self.pending = None;
                    return Some(Event {
                        kind: Kind::Blip,
                        stage,
                        advanced: false,
                        moment_us: pending.start_us,
                        duration_us: episode.finish_us.wrapping_sub(pending.start_us),
                    });
                }
                Some(pending) => {
                    // a touch that let go again (a blip of the latch): the pending separation's, its last edge now
                    self.pending = Some(Episode {
                        start_us: pending.start_us,
                        finish_us: episode.finish_us,
                    });
                }
            }
        }
        let pending = self.pending?;
        // widened, so a negative interval is below the backstop
        let backstop = i64::from(elapsed_us(now_us, pending.finish_us)) >= i64::from(BACKSTOP_US);
        // a LOW read inside an open episode (a bounce of the contact closing again) is no held LOW: its conclusion
        // decides, folding it in or clearing the separation. A HIGH read off the pending latch has opened an episode
        // already (the caller's observe or poll); `high` states the rule on its own: the pin still 0
        if high || pair.episode_open() || !(gate_open || backstop || stage != Stage::Boosting) {
            return None;
        }
        self.pending = None;
        let advanced = stage == Stage::Boosting;
        Some(Event {
            kind: Kind::Separated,
            stage: if advanced { Stage::Gliding } else { stage },
            advanced,
            moment_us: pending.start_us,
            duration_us: pending.duration_us(),
        })
    }
}

/// What the thrust gate has seen of the boost since BOOSTING entry: `control`'s `thrust`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Thrust {
    /// Not seen (`none`): a_x has not yet held above `launch_g` for `launch_ms`. The gate is shut.
    Unseen,
    /// Seen and not ended: the booster pushes. The gate is shut.
    Seen,
    /// Seen, then ended: nothing pushes the glider any more. The gate is open.
    Ended,
}

impl Thrust {
    /// The name `control` shows: `none`, `seen` or `ended`.
    pub fn name(self) -> &'static str {
        match self {
            Thrust::Unseen => "none",
            Thrust::Seen => "seen",
            Thrust::Ended => "ended",
        }
    }
}

/// The separation's thrust gate (see the module notes, THE THRUST GATE): stepped every tick with the glider's fresh
/// a_x, open once the boost was seen and then ended, both afresh at each BOOSTING entry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustGate {
    /// The thrust's level, g: the sequencer's `launch_g`.
    launch_g: f32,
    /// How long a_x must hold above it, ms of net time: the sequencer's `launch_ms`.
    launch_ms: u32,
    /// What it has seen since BOOSTING entry.
    thrust: Thrust,
    /// The running leaky dwell: the thrust's while it is unseen, its end's once seen.
    dwell: Dwell,
    /// Whether the previous step was in BOOSTING: a step in BOOSTING after one elsewhere (or none) is an entry.
    boosting: bool,
}

impl ThrustGate {
    /// A shut gate on the sequencer's `launch_g` and `launch_ms`: `settings` are the sequencer entry's, or
    /// sequencer::Settings::default() (plan B's) for a board without one.
    pub fn new(settings: &sequencer::Settings) -> ThrustGate {
        ThrustGate {
            launch_g: settings.launch_g,
            launch_ms: settings.launch_ms,
            thrust: Thrust::Unseen,
            dwell: Dwell::default(),
            boosting: false,
        }
    }

    /// One tick at `now_ms` in `stage`, with `axial_g`, the glider's a_x in g from a fresh `accel` (None when no
    /// source is fresh). A BOOSTING entry starts the gate afresh, shut. In BOOSTING the sample feeds the dwell of what
    /// the gate waits for: a_x above `launch_g` for the thrust, then below 0 g for its end, each a leaky dwell that a
    /// missing or contrary sample drains. Anywhere else the gate keeps what it saw.
    ///
    /// `now_ms` is the stage machine's clock: the uptime's ms as a wrapping u32. Never the stamps' wrapping µs divided
    /// down: that clock jumps back at its wrap, and a step backwards FEEDS a leaky dwell that misses
    /// (sequencer::commons::dwell_step).
    pub fn step(&mut self, now_ms: u32, stage: Stage, axial_g: Option<f32>) {
        let boosting = stage == Stage::Boosting;
        if boosting && !self.boosting {
            self.thrust = Thrust::Unseen;
            self.dwell = Dwell::default();
        }
        self.boosting = boosting;
        if !boosting {
            return;
        }
        // NaN compares false: no thrust and no end
        let (active, threshold_ms, next) = match self.thrust {
            Thrust::Unseen => (
                axial_g.is_some_and(|axial| axial > self.launch_g),
                self.launch_ms,
                Thrust::Seen,
            ),
            Thrust::Seen => (
                axial_g.is_some_and(|axial| axial < THRUST_END_G),
                THRUST_END_MS,
                Thrust::Ended,
            ),
            Thrust::Ended => return,
        };
        let (dwell, held) = dwell_step(active, now_ms, self.dwell, threshold_ms);
        self.dwell = dwell;
        if held {
            self.thrust = next;
            self.dwell = Dwell::default();
        }
    }

    /// What the gate has seen since BOOSTING entry (or of the last BOOSTING, outside it).
    pub fn thrust(&self) -> Thrust {
        self.thrust
    }

    /// Whether the gate is open: the thrust seen and then ended.
    pub fn open(&self) -> bool {
        self.thrust == Thrust::Ended
    }
}

#[cfg(test)]
mod tests;
