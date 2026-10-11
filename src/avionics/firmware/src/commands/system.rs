//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! reboot -- restart the board (src/glider cc_client `_register_system`).

use std::time::Duration;

use cc_protocol::Message;

use super::{Context, Reply};

/// How long the reset waits, so the `ok` reaches the hub before the socket dies.
const FLUSH: Duration = Duration::from_millis(200);

/// The longest the reset waits for the recorder's UART link to send what is queued (crate::recorder `drain_uart`).
const UART_DRAIN: Duration = Duration::from_secs(1);

/// `reboot`: replies `ok`, then restarts, once the recorder's UART link has sent what it queued.
pub fn reboot(_message: &Message, _context: &mut Context) -> Reply {
    std::thread::spawn(|| {
        std::thread::sleep(FLUSH);
        crate::recorder::drain_uart(UART_DRAIN);
        esp_idf_svc::hal::reset::restart();
    });
    Ok(cc_protocol::build("ok", &[], &[]))
}
