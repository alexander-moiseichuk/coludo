//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight stages and the stage machine, one module per plan B module it ports:
//! - [`controller`]: the flight stages (src/glider controller.py `Stage`);
//! - [`commons`]: the detector primitives, the leaky dwell and the apogee tracker (src/glider commons.py,
//!   `dwell_step`/`apogee_step`);
//! - [`sequencer`]: the stage machine (src/glider tasks/sequencer.py), a pure `step` the control thread runs at
//!   20 Hz.
//!
//! The machine allocates nothing and reads no clock. The crate is `std` for one reason: `f32::sqrt`, which the
//! reason text's |accel| takes and which `core` lacks on stable Rust.

pub mod commons;
pub mod controller;
pub mod sequencer;

pub use controller::Stage;
pub use sequencer::{Reading, Readings, Reason, Settings, Source, StageMachine, Step, Transition};
