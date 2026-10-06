//! The CC commands this firmware answers (doc/specs/cc-protocol.md), one line in, one line out.

use crate::board::{self, Board};
use esp_idf_svc::sys;

/// Answer one request line from the hub or the console; None for an empty line.
pub fn handle(line: &str, board: &Board) -> Option<String> {
    let message = match cc_protocol::parse(line) {
        Ok(message) => message,
        Err(error) => return Some(cc_protocol::build("err", &["badline", &format!("{error:?}")], &[])),
    };
    let command = message.command?;
    Some(match command.as_str() {
        "whoami" => whoami(board),
        "ping" => cc_protocol::build("pong", &[], &[]),
        "health" => health(),
        _ => cc_protocol::build("err", &["badcmd", &command], &[]),
    })
}

/// `iam <id> <json>`: who the board is -- the one reply carrying the id (src/glider cc_client whoami).
fn whoami(board: &Board) -> String {
    let info = serde_json::json!({
        "mcu": "esp32p4",
        "firmware_version": board::FIRMWARE_VERSION,
        "config_id": null,
        "config_source": "none",
        "stage": "setting",
        "uptime": board::uptime_ms(),
        "boot_id": board.boot_id,
        "session": board.session(),
    });
    cc_protocol::build("iam", &[board.id, &info.to_string()], &[])
}

/// `ok <json>`: the heartbeat vitals the dashboard top table shows.
fn health() -> String {
    let info = serde_json::json!({
        "temp": null,
        "mem_free": board::heap_free(sys::MALLOC_CAP_INTERNAL),
        "psram_free": board::heap_free(sys::MALLOC_CAP_SPIRAM),
        "uptime": board::uptime_ms(),
        "stage": "setting",
        "layout": null,
        "degraded": [],
    });
    cc_protocol::build("ok", &[&info.to_string()], &[])
}
