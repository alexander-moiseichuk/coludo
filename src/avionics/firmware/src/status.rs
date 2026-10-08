//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight word (doc/specs/avionics-port.md §4.1): the stage, the operator hold, the arming and the launch in one
//! atomic. A reader takes one [`sequencer::FlightWord::load`], a consistent snapshot, so health never pairs one
//! moment's stage with another's arming. Its one writer is handed to the control thread (crate::control), which
//! commits the stage machine's advances, the separation's, and the operator's requests, posted to its request slot.

use sequencer::FlightWord;

/// This board's flight word: on the ground until the control thread commits a change.
pub static FLIGHT: FlightWord = FlightWord::new();
