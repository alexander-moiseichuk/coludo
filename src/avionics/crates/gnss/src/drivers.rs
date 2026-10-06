//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GNSS module drivers (src/glider drivers/atgm336h.py, drivers/neo6mv2.py): the bytes each module's setup
//! sends, and the sky switch pair it keeps. Both share the NMEA receiver of the crate root; they differ only in how
//! they configure the module. Either sits on the same UART: the board config's `driver` picks one.

pub mod atgm336h;
pub mod neo6mv2;
