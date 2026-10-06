//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! log / tlm -- poll-model streaming (src/glider cc_client `_register_streaming`): each call returns what was
//! buffered since the last one and keeps the window open `ms` more (default 1000; 0 or less stops).

use cc_protocol::Message;
use serde_json::json;

use super::{Code, Context, Fault, Reply};
use crate::logging;

/// The window a bare `log`/`tlm` asks for.
const DEFAULT_WINDOW_MS: i64 = 1000;

/// `log [ms]`: {lines, dropped}.
pub fn log(message: &Message, _context: &mut Context) -> Reply {
    let window = window(message, "log <duration_ms>")?;
    let (lines, dropped) = logging::LOG.drain(window);
    super::ok_json(&json!({"lines": lines, "dropped": dropped}))
}

/// `tlm [ms]`: {samples, dropped}. No telemetry streams exist until the recorder lands (port plan M3).
pub fn telemetry(message: &Message, _context: &mut Context) -> Reply {
    window(message, "tlm <duration_ms>")?;
    super::ok_json(&json!({"samples": [], "dropped": 0}))
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
