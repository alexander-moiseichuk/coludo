//! The flight state every thread reads: the stage, the operator hold, the arming. Plain atomics, no lock: each
//! field has one writer (the CC executor until the control thread takes the stage machine over), and a reader
//! anywhere sees a whole value.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use sequencer::Stage;

/// The board's flight state (src/glider Controller: `stage`, `manual`, `armed`, `warm_started`).
pub struct Status {
    /// The stage id (sequencer::Stage as u8).
    stage: AtomicU8,
    /// An operator `stage <name>` holds the stage: the stage machine leaves it alone.
    manual: AtomicBool,
    /// Actuation enabled: the arm gate passed. Disarmed fins hold neutral.
    armed: AtomicBool,
}

/// The one flight state of this board.
pub static STATUS: Status = Status {
    stage: AtomicU8::new(Stage::Setting as u8),
    manual: AtomicBool::new(false),
    armed: AtomicBool::new(false),
};

impl Status {
    /// The current stage.
    pub fn stage(&self) -> Stage {
        // only `set_stage` writes it, always with a valid id
        Stage::from_id(self.stage.load(Ordering::Acquire)).unwrap_or(Stage::Setting)
    }

    /// Move to `stage`.
    pub fn set_stage(&self, stage: Stage) {
        self.stage.store(stage as u8, Ordering::Release);
    }

    /// Whether an operator hold suppresses the stage machine.
    pub fn manual(&self) -> bool {
        self.manual.load(Ordering::Acquire)
    }

    /// Set or clear the operator hold.
    pub fn set_manual(&self, manual: bool) {
        self.manual.store(manual, Ordering::Release);
    }

    /// Whether actuation is enabled.
    pub fn armed(&self) -> bool {
        self.armed.load(Ordering::Acquire)
    }

    /// Enable or disable actuation.
    pub fn set_armed(&self, armed: bool) {
        self.armed.store(armed, Ordering::Release);
    }
}
