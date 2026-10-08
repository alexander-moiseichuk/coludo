//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The thrust gate (the operator's, new in Rust: plan B has none): what one accelerometer has seen of the boost since
//! BOOSTING entry. It reads the glider's own axial specific force, a_x on the body frame's +x (the nose; +y left, +z
//! up). Thrust is SEEN when a_x holds above the sequencer's `launch_g` for its `launch_ms` of net time (the launch
//! detector's leaky dwell, [`crate::commons::dwell_step`]), and has ENDED when a_x then holds below 0 g for 20 ms of
//! net time: the drag deceleration the glider feels the moment nothing pushes it, by burnout or by an early
//! aerodynamic separation of a destabilized stack. Both start afresh at each BOOSTING entry. A missing a_x drains a
//! dwell like a sample that misses. A wrong axis or sign, no IMU, or a bench hold never sees thrust.
//!
//! Two readers, each with its own gates:
//! - the separation switch (crates/drivers, `separation`, which re-exports this module's types): one gate on the
//!   fused `accel`, stepped every control tick, whose verdict holds a separation in BOOSTING pending until the boost
//!   has ended, so vibration under thrust can never deploy;
//! - the stage machine ([`crate::StageMachine`]): one gate per accelerometer, each on its own +x, stepped every
//!   machine step, whose verdicts hold the apogee while every live accelerometer still sees the boost.

use crate::commons::{Dwell, dwell_step};
use crate::controller::Stage;
use crate::sequencer::Settings;

/// The thrust's end, g: a_x below it is no push but drag.
const THRUST_END_G: f32 = 0.0;

/// How long a_x must hold below 0 g for the thrust to have ended, ms of net time: two ticks of the control thread's
/// default schedule, long enough that one sample of vibration under thrust does not end it.
pub const THRUST_END_MS: u32 = 20;

/// What a thrust gate has seen of the boost since BOOSTING entry: `control`'s `thrust`.
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

/// One accelerometer's thrust gate (see the module notes): stepped with its fresh a_x, open once the boost was seen
/// and then ended, both afresh at each BOOSTING entry.
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
    /// [`Settings::default`] (plan B's) for a board without one.
    pub fn new(settings: &Settings) -> ThrustGate {
        ThrustGate {
            launch_g: settings.launch_g,
            launch_ms: settings.launch_ms,
            thrust: Thrust::Unseen,
            dwell: Dwell::default(),
            boosting: false,
        }
    }

    /// One step at `now_ms` in `stage`, with `axial_g`, the glider's a_x in g from a fresh `accel` (None when it is
    /// not fresh). A BOOSTING entry starts the gate afresh, shut. In BOOSTING the sample feeds the dwell of what the
    /// gate waits for: a_x above `launch_g` for the thrust, then below 0 g for its end, each a leaky dwell that a
    /// missing or contrary sample drains. Anywhere else the gate keeps what it saw.
    ///
    /// `now_ms` is the stage machine's clock: the uptime's ms as a wrapping u32. Never a wrapping µs clock divided
    /// down: that clock jumps back at its wrap, and a step backwards FEEDS a leaky dwell that misses
    /// ([`crate::commons::dwell_step`]).
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
mod tests {
    use super::{THRUST_END_MS, Thrust, ThrustGate};
    use crate::controller::Stage;
    use crate::sequencer::Settings;

    /// Step `gate` every 10 ms from `from_ms` up to (not including) `to_ms` in `stage`, with the same a_x each time.
    fn feed(gate: &mut ThrustGate, from_ms: u32, to_ms: u32, stage: Stage, axial_g: Option<f32>) {
        for now_ms in (from_ms..to_ms).step_by(10) {
            gate.step(now_ms, stage, axial_g);
        }
    }

    #[test]
    fn thrust_is_seen_then_ended_on_net_time() {
        // plan B's defaults: a_x above 2.5 g for 100 ms, then below 0 g for 20 ms
        let mut gate = ThrustGate::new(&Settings::default());
        feed(&mut gate, 0, 100, Stage::Boosting, Some(4.0));
        assert_eq!(gate.thrust(), Thrust::Unseen, "90 ms of net time");
        gate.step(100, Stage::Boosting, Some(4.0));
        assert_eq!((gate.thrust(), gate.open()), (Thrust::Seen, false));
        // NEGATIVE: exactly 0 g is no drag yet, however long
        feed(&mut gate, 110, 1000, Stage::Boosting, Some(0.0));
        assert_eq!(gate.thrust(), Thrust::Seen);
        feed(&mut gate, 1000, 1000 + THRUST_END_MS, Stage::Boosting, Some(-0.3));
        assert!(!gate.open(), "10 ms of net time");
        gate.step(1000 + THRUST_END_MS, Stage::Boosting, Some(-0.3));
        assert_eq!((gate.thrust(), gate.open()), (Thrust::Ended, true));
        assert_eq!(
            [Thrust::Unseen, Thrust::Seen, Thrust::Ended].map(Thrust::name),
            ["none", "seen", "ended"]
        );
    }

    #[test]
    fn a_missing_or_still_sample_never_sees_thrust() {
        // NEGATIVE: a missing a_x drains the dwell like a sample that misses, and a bench hold nose up (1 g) is no push
        let mut gate = ThrustGate::new(&Settings::default());
        for now_ms in (0..2000).step_by(10) {
            let axial_g = (now_ms % 20 == 0).then_some(4.0);
            gate.step(now_ms, Stage::Boosting, axial_g);
        }
        assert_eq!(gate.thrust(), Thrust::Unseen, "every push followed by a missing sample");
        feed(&mut gate, 2000, 5000, Stage::Boosting, Some(1.0));
        assert_eq!(gate.thrust(), Thrust::Unseen);
    }

    #[test]
    fn each_boosting_entry_starts_afresh_and_other_stages_keep_the_verdict() {
        let mut gate = ThrustGate::new(&Settings::default());
        feed(&mut gate, 0, 200, Stage::Boosting, Some(4.0));
        assert_eq!(gate.thrust(), Thrust::Seen);
        // NEGATIVE: GLIDING keeps what it saw, whatever a_x does
        feed(&mut gate, 200, 400, Stage::Gliding, Some(-1.0));
        assert_eq!(gate.thrust(), Thrust::Seen);
        // the next BOOSTING entry shuts it at once
        gate.step(400, Stage::Boosting, Some(4.0));
        assert_eq!(gate.thrust(), Thrust::Unseen);
    }
}
