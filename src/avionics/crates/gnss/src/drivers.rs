//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The GNSS module drivers (src/glider drivers/atgm336h.py, drivers/neo6mv2.py; the NEO-M8N's is the port's own, M16):
//! the bytes each module's setup sends, and the sky switch pair it keeps. The ATGM336H and the NEO-6M share the NMEA
//! receiver of the crate root; the NEO-M8N delivers UBX ([`crate::ubx`]). Each sits on the same UART: the board
//! config's `driver` picks one.

pub mod atgm336h;
pub mod neo6mv2;
pub mod neom8n;
