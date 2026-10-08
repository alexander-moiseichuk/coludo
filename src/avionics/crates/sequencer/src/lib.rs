//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight stages, the flight word and the stage machine, one module per plan B module it ports:
//! - [`controller`]: the flight stages and the flight word (src/glider controller.py `Stage`, and the Controller's
//!   stage, hold and arming with the rules `hold`, `resume`, `arm` and `disarm` change them by), its one writer, what
//!   each commit does to the recorder's flight flag, and the stages a change may run in;
//! - [`slot`]: the request slot an operator request reaches the flight word's writer through (new in Rust: plan B's
//!   one asyncio loop called the Controller directly);
//! - [`commons`]: the detector primitives, the leaky dwell and the apogee tracker (src/glider commons.py,
//!   `dwell_step`/`apogee_step`);
//! - [`sequencer`]: the stage machine (src/glider tasks/sequencer.py), a pure `step` the control thread runs every
//!   `period_ms` (20 Hz by default), and its settings, read from the board config;
//! - [`video`]: the camera by stage, the rules the firmware's video thread follows from the flight word (new in Rust:
//!   plan B records no video).
//!
//! The machine allocates nothing and reads no clock. The crate is `std` for `f32::sqrt`, which the reason text's
//! |accel| takes and which `core` lacks on stable Rust, for serde_json, which reads the settings, and for the
//! requesting thread's wait on its slot (a clock and a sleep, never on the control thread).

pub mod commons;
pub mod controller;
pub mod sequencer;
pub mod slot;
pub mod video;

pub use controller::{
    ChangeRefused, Commit, Flight, FlightFlag, FlightWord, FlightWriter, Request, Stage, StageProblem,
};
pub use sequencer::{
    Reading, Readings, Reason, Settings, SettingsError, SettingsProblem, Source, StageMachine, Step, Transition,
};
pub use slot::{Answer, RequestSlot, Ticket, Unanswered};
