//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The recorder's line wire codec (doc/specs/recorder-wire.md; doc/specs/avionics-port.md "Storage design"),
//! the wire parts of src/glider recorder.py and of the host reference tools/recorder_wire.py:
//!  * [`crc32`], zlib's CRC-32;
//!  * the integrity wrapper: [`wrap`]/[`wrap_into`] write a line, [`verify`], [`unwrap`], [`wrapped`] and
//!    [`classify`] read one back;
//!  * [`split`], recorderd's demux of a byte stream into routed lines, whose filing rule the ring_saver follows;
//!  * the line text before it is wrapped: a telemetry [`header`] and [`row`] of [`Cell`]s, a [`log_line`],
//!    the session.csv [`SessionRow`] and [`SESSION_INDEX_HEADER`], the session prefix ([`session_label`],
//!    [`session_prefix`], [`is_label`]), the text-cell rule [`cell`], and the Recorder's [`status_message`].
//!
//! The flash ring's binary format (the block header and the chunks) is flash_ring's own; it takes [`crc32`] from
//! here, so one table serves the ring and the lines the saver prints from it.
//!
//! Uptimes are u64 microseconds with no 2^30 wrap; a wrapper's U is the uptime mod 2^32, which is what plan B
//! and the host reference compute for any uptime text. The module docs list where the text deliberately
//! differs from plan B's. golden/recorder_wire.json pins all of it to plan B and the host reference.
//!
//! `no_std` with `alloc`: it builds for the ring_saver on the board and for the host tools alike.

#![no_std]

extern crate alloc;

mod crc;
mod demux;
mod text;
mod wrapper;

pub use crc::crc32;
pub use demux::split;
pub use text::{
    Cell, Counters, SESSION_INDEX, SESSION_INDEX_CELLS, SESSION_INDEX_HEADER, SessionLabel, SessionRow, cell, header,
    is_label, log_line, row, session_label, session_prefix, status_message,
};
pub use wrapper::{
    Line, Status, WRAPPER_BYTES, Wrapper, checks, classify, leading_number, unwrap, verify, wrap, wrap_into, wrapped,
};
