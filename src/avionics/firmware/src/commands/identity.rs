//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! whoami / ping / health -- who the board is and how it is doing (src/glider cc_client `_register_identity`).

use cc_protocol::Message;
use esp_idf_svc::sys;
use serde::Serialize;

use super::{Context, Reply};
use crate::board;
use crate::clock;
use crate::status::STATUS;

/// The `iam` payload. Replies are typed structs, not `json!` values: a Value widens every f32 to f64 (a 33.3 °C
/// reading prints as 33.29999923706055); a struct field serialises as f32.
#[derive(Serialize)]
struct Identity<'a> {
    mcu: String,
    firmware_version: &'static str,
    config_id: &'a str,
    config_source: &'a str,
    stage: &'static str,
    uptime: i64,
    boot_id: Option<i32>,
    /// The prefix this boot's capture files carry (plan B `Recorder.session()`).
    session: &'a str,
    /// The wall clock: a value inside 2000 tells the hub to set it.
    epoch: i64,
}

/// `iam <id> <json>`: who the board is -- the one reply carrying the id.
pub fn whoami(_message: &Message, context: &mut Context) -> Reply {
    let info = Identity {
        mcu: board_config::mcu(&context.config.document),
        firmware_version: board::FIRMWARE_VERSION,
        config_id: &context.config_id,
        config_source: &context.config.source,
        stage: STATUS.stage().name(),
        uptime: board::uptime_ms(),
        boot_id: context.board.boot_id,
        session: &context.board.session,
        epoch: clock::epoch(),
    };
    let json =
        serde_json::to_string(&info).map_err(|error| super::Fault::new(super::Code::Internal, error.to_string()))?;
    Ok(cc_protocol::build("iam", &[&context.board.id, &json], &[]))
}

/// `pong`.
pub fn ping(_message: &Message, _context: &mut Context) -> Reply {
    Ok(cc_protocol::build("pong", &[], &[]))
}

/// The heartbeat vitals the dashboard top table shows (plan B's keys; `internal_free` and `sd_free_mb` are this
/// firmware's own).
#[derive(Serialize)]
struct Health<'a> {
    /// Die temperature, °C to 0.1.
    temp: Option<f32>,
    /// Free heap of every kind (internal RAM + PSRAM), bytes.
    mem_free: usize,
    /// Free internal RAM, bytes: the scarce kind (DMA, interrupt-safe code paths).
    internal_free: usize,
    /// Free space on the microSD's recordings volume (the ring_saver's: partition 1, or a one-partition card's only
    /// volume) in MB, measured at mount and after writes (never on this path: f_getfree stalls the link).
    sd_free_mb: Option<u32>,
    uptime: i64,
    stage: &'static str,
    /// The board revision the firmware runs as: 'v1.1', 'v0.1 (declared)' or 'undecided' (layout::Resolved::label).
    layout: &'a str,
    /// The board's own GNSS fix, null until there is a fresh one (the GNSS driver lands with port plan M11).
    position: Option<[f64; 2]>,
    /// The laser height above ground, null unless fresh (the laser lands with port plan M10).
    agl: Option<f32>,
    clock: String,
    epoch: i64,
    /// The launch point the board would fly with: [latitude, longitude] degrees.
    launchpad: Option<[f64; 2]>,
    /// Whether that point is set (by CC or frozen at arm) rather than taken live from GNSS.
    launchpad_set: bool,
    /// The selected launch site's name.
    site: Option<String>,
    armed: bool,
    /// The device tasks that came up, false once a task's loop crashed (plan B's `{name, ok}`).
    tasks: Vec<Task<'a>>,
    degraded: Vec<String>,
}

/// One `health.tasks` item.
#[derive(Serialize)]
struct Task<'a> {
    /// The device's name.
    name: &'a str,
    /// False once its loop crashed.
    ok: bool,
}

/// `ok <json>`: the heartbeat vitals.
pub fn health(_message: &Message, context: &mut Context) -> Reply {
    let mission = &context.mission;
    let info = Health {
        temp: context.board.celsius(),
        mem_free: board::heap_free(sys::MALLOC_CAP_8BIT),
        internal_free: board::heap_free(sys::MALLOC_CAP_INTERNAL),
        sd_free_mb: if context.board.sd_mounted {
            firmware::sd::free_megabytes()
        } else {
            None
        },
        uptime: board::uptime_ms(),
        stage: STATUS.stage().name(),
        layout: &context.layout,
        position: None,
        agl: None,
        clock: clock::text(),
        epoch: clock::epoch(),
        // decimal degrees on the wire: f64 at this JSON boundary only (f32 prints 25.514379 as 25.51438)
        launchpad: mission.launch_point(None).map(|point| point.degrees()),
        launchpad_set: mission.position_set(),
        site: Some(mission.site.clone()).filter(|site| !site.is_empty()),
        armed: STATUS.armed(),
        tasks: context
            .devices
            .tasks()
            .into_iter()
            .map(|(name, ok)| Task { name, ok })
            .collect(),
        degraded: degraded(context),
    };
    super::ok_json(&info)
}

/// The non-nominal states, one operator signal (src/glider cc_client health `degraded`). Empty = nominal.
fn degraded(context: &Context) -> Vec<String> {
    let (board, mission) = (&context.board, &context.mission);
    let mut degraded = Vec::new();
    // SAFETY: plain reads of flags hosted_no_reboot keeps.
    unsafe {
        if !sys::hosted_no_reboot_installed() {
            degraded.push("c6-reboot-guard-missing".into()); // a C6 glitch would reboot this board
        }
        if !sys::hosted_tasks_pinned() {
            degraded.push("c6-tasks-unpinned".into()); // the radio's tasks may run on the flight core
        }
        if sys::hosted_link_down() {
            degraded.push("c6-link-parked".into()); // the Wi-Fi link failed; it is gone until the next boot
        }
        if !sys::hosted_buffers_in_psram() {
            // Wi-Fi frames take internal DMA memory: an assert when it runs out
            degraded.push("c6-buffers-internal".into());
        }
    }
    if !board.sd_mounted {
        degraded.push("no-sd".into()); // video and the flash-ring copy have nowhere to go
    }
    // the flash ring: ring-unformatted, ring-low, ring-full, saver-behind (crate::recorder::degraded)
    degraded.extend(crate::recorder::degraded().into_iter().map(String::from));
    if STATUS.manual() {
        degraded.push("STAGE HELD".into()); // an operator hold suppresses every stage detector
    }
    if context.config.source.starts_with("default(fallback") {
        // board.config failed to load, so this airframe booted the bench default: id taster, watchdog off
        degraded.push(format!("CONFIG FALLBACK -- {}", context.config.source));
    }
    if mission.site == "fallback" {
        degraded.push("cc-less-fallback".into());
    }
    // the flight loop is not ported yet: say so, so the dashboard's NOT READY carries a true reason
    degraded.push("not-flight-capable".into());
    degraded
}
