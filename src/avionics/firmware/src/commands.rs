//! The CC commands this firmware answers (doc/specs/cc-protocol.md), one line in, one line out.

use crate::board::{self, Board};
use esp_idf_svc::sys;
use serde::Serialize;

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
        "health" => health(board),
        _ => cc_protocol::build("err", &["badcmd", &command], &[]),
    })
}

/// The `iam` payload (src/glider cc_client whoami). Replies are typed structs, not `json!` values: a Value
/// widens every f32 to f64 (a 33.3 °C reading prints as 33.29999923706055); a struct field serialises as f32.
#[derive(Serialize)]
struct Identity {
    mcu: &'static str,
    firmware_version: &'static str,
    config_id: Option<String>,
    config_source: &'static str,
    stage: &'static str,
    uptime: i64,
    boot_id: Option<i32>,
    session: Option<String>,
}

/// The `health` payload: the heartbeat vitals the dashboard top table shows.
#[derive(Serialize)]
struct Health {
    temp: Option<f32>,
    mem_free: usize,
    psram_free: usize,
    uptime: i64,
    stage: &'static str,
    layout: Option<&'static str>,
    degraded: Vec<&'static str>,
}

/// `iam <id> <json>`: who the board is -- the one reply carrying the id.
fn whoami(board: &Board) -> String {
    let info = Identity {
        mcu: "esp32p4",
        firmware_version: board::FIRMWARE_VERSION,
        config_id: None,
        config_source: "none",
        stage: "setting",
        uptime: board::uptime_ms(),
        boot_id: board.boot_id,
        session: board.session(),
    };
    cc_protocol::build(
        "iam",
        &[board.id, &serde_json::to_string(&info).unwrap_or_default()],
        &[],
    )
}

/// `ok <json>`: the heartbeat vitals.
fn health(board: &Board) -> String {
    let info = Health {
        temp: board.celsius(),
        mem_free: board::heap_free(sys::MALLOC_CAP_INTERNAL),
        psram_free: board::heap_free(sys::MALLOC_CAP_SPIRAM),
        uptime: board::uptime_ms(),
        stage: "setting",
        layout: None,
        degraded: Vec::new(),
    };
    cc_protocol::build("ok", &[&serde_json::to_string(&info).unwrap_or_default()], &[])
}
