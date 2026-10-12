//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! reboot -- restart the board (src/glider cc_client `_register_system`).

use std::time::Duration;

use cc_protocol::Message;

use super::{Context, Reply};

/// How long the reset waits, so the `ok` reaches the hub before the socket dies.
const FLUSH: Duration = Duration::from_millis(200);

/// `reboot`: replies `ok`, then restarts (components/safe_restart).
pub fn reboot(_message: &Message, _context: &mut Context) -> Reply {
    std::thread::spawn(|| {
        std::thread::sleep(FLUSH);
        // SAFETY: called from a task; the restart never returns.
        unsafe { esp_idf_svc::sys::safe_restart() }
    });
    Ok(cc_protocol::build("ok", &[], &[]))
}
