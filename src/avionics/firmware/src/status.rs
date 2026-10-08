//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight word (doc/specs/avionics-port.md §4.1): the stage, the operator hold, the arming and the launch in one
//! atomic. A reader takes one [`sequencer::FlightWord::load`], a consistent snapshot, so health never pairs one
//! moment's stage with another's arming. Its one writer commits a warm start's restored flight on main, before any
//! thread runs (crate::warm), and is then handed to the control thread (crate::control), which commits the stage
//! machine's advances, the separation's, and the operator's requests, posted to its request slot.

use core::sync::atomic::AtomicBool;

use sequencer::FlightWord;

/// This board's flight word: on the ground until a warm start's restore or the control thread commits a change.
pub static FLIGHT: FlightWord = FlightWord::new();

/// Whether this boot is a warm start: a reset in flight restored the stage the crumb saved (crate::warm). Set once on
/// main before the threads start, never cleared; health's `degraded` names it.
pub static WARM_STARTED: AtomicBool = AtomicBool::new(false);
