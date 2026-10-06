//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! log / tlm -- poll-model streaming (src/glider cc_client `_register_streaming`): each call returns what was
//! buffered since the last one and keeps the window open `ms` more (default 1000; 0 or less stops).
//!
//! Both drain the recorder's CC tees (doc/specs/avionics-port.md "CC tees and the HTTP exits"): while a window is
//! open, the ring_writer copies each record it programs into the matching tee, verbatim, and the drain here, on the
//! executor thread, prints them. So the stream shows exactly what reached the flash ring, and the text is made off the
//! flight path. Replies keep plan B's shape, without the CRC wrapper: `{"lines": [...], "dropped": n}` for `log`,
//! `{"samples": ["@<routing>@<row>", ...], "dropped": n}` for `tlm`. A board whose recorder did not open answers an
//! empty batch.
//!
//! The writer tees a record only once it is programmed, and a log line only after the 20 ms merge horizon, so a
//! drain first waits for the writer to catch up with the request (crate::recorder::catch_up, ~20-30 ms): it then
//! carries every line and row queued before it, as plan B's immediate tee did.

use cc_protocol::Message;
use serde_json::json;

use super::{Code, Context, Fault, Reply};

/// The window a bare `log`/`tlm` asks for.
const DEFAULT_WINDOW_MS: i64 = 1000;

/// `log [ms]`: {lines, dropped}.
pub fn log(message: &Message, _context: &mut Context) -> Reply {
    let window = window(message, "log <duration_ms>")?;
    crate::recorder::catch_up();
    let batch = crate::recorder::get()
        .map(|recorder| recorder.drain_log_tee(window))
        .unwrap_or_default();
    super::ok_json(&json!({"lines": batch.lines, "dropped": batch.dropped}))
}

/// `tlm [ms]`: {samples, dropped}, each sample `@<routing>@<row>`.
pub fn telemetry(message: &Message, _context: &mut Context) -> Reply {
    let window = window(message, "tlm <duration_ms>")?;
    crate::recorder::catch_up();
    let batch = crate::recorder::get()
        .map(|recorder| recorder.drain_telemetry_tee(window))
        .unwrap_or_default();
    super::ok_json(&json!({"samples": batch.lines, "dropped": batch.dropped}))
}

/// The requested window in milliseconds; `usage` when it is not an integer. Plan B parses with int(), which on
/// the board skips every '_' (MicroPython py/parsenum.c), so `1_000` is a thousand here too.
fn window(message: &Message, usage: &str) -> Result<i64, Fault> {
    match message.args.first() {
        None => Ok(DEFAULT_WINDOW_MS),
        Some(text) => text
            .replace('_', "")
            .parse()
            .map_err(|_| Fault::new(Code::BadArguments, usage)),
    }
}
