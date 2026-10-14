//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! reboot -- restart the board (src/glider cc_client `_register_system`).

use std::time::Duration;

use cc_protocol::Message;

use super::{Context, Reply};

/// How long the reset waits, so the `ok` reaches the hub before the socket dies.
const FLUSH: Duration = Duration::from_millis(200);

/// `reboot`: replies `ok`, then restarts (components/safe_restart). When internal RAM cannot give the delayed restart
/// a stack, it restarts at once: the reply is lost, the reboot asked for is not.
pub fn reboot(_message: &Message, _context: &mut Context) -> Reply {
    let delayed = std::thread::Builder::new().spawn(|| {
        std::thread::sleep(FLUSH);
        // SAFETY: called from a task; the restart never returns.
        unsafe { esp_idf_svc::sys::safe_restart() }
    });
    if let Err(error) = delayed {
        log::error!("reboot :: no thread for the delayed restart ({error}): restarting now");
        // SAFETY: called from a task; the restart never returns.
        unsafe { esp_idf_svc::sys::safe_restart() }
    }
    Ok(cc_protocol::build("ok", &[], &[]))
}
