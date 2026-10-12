//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! whoami / ping / health -- who the board is and how it is doing (src/glider cc_client `_register_identity`).

use std::collections::BTreeMap;
use std::sync::atomic::Ordering;

use cc_protocol::Message;
use esp_idf_svc::sys;
use sequencer::Flight;
use serde::Serialize;
use serde_json::Value;

use super::{Context, Reply};
use crate::board;
use crate::clock;
use crate::status::FLIGHT;
use firmware::time::uptime_us;

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
        stage: FLIGHT.load().stage.name(),
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
    /// The board's own GNSS fix, [latitude, longitude] degrees, null while none is fresh (M11).
    position: Option<[f64; 2]>,
    /// The laser height above ground, null unless fresh (M10).
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
    /// The live flight panel (plan B cc_client.py:510-512, flight.vitals): the flight loop's vitals while it runs
    /// (M13), else the fused attitude and airspeed; absent without an attitude or an airspeed provider.
    #[serde(skip_serializing_if = "Option::is_none")]
    flight: Option<Value>,
    /// The calibrations the devices wait for, {name: instruction}; absent when none is pending (plan B
    /// cc_client.py:443-514).
    #[serde(skip_serializing_if = "Option::is_none")]
    calibration: Option<BTreeMap<&'static str, String>>,
    /// The BNO055's live CALIB_STAT (sys, gyr, acc, mag), so CC's IMU column shows mag climbing through the figure-8;
    /// absent with no BNO055 up or before its first read (plan B cc_client.py:494-499).
    #[serde(skip_serializing_if = "Option::is_none")]
    imu_calibration: Option<[u8; 4]>,
    degraded: Vec<String>,
}

/// `health.flight` of a config without a flight loop: the fused attitude and airspeed, each null while no source is
/// fresh. The airspeed is the fused `airspeed` (the pitot); with the loop it is the governor's estimate
/// (crate::flight::vitals).
#[derive(Serialize)]
struct FlightPanel {
    /// m/s to 0.1.
    airspeed: Option<f32>,
    /// Whole degrees, as plan B's walk-test HUD rounds them.
    attitude: Option<AttitudePanel>,
}

/// `health.flight.attitude`: whole degrees, rounded half to even as plan B's Python round().
#[derive(Serialize)]
struct AttitudePanel {
    heading: i32,
    roll: i32,
    pitch: i32,
}

/// The flight panel from `topology` at `now_us`: the flight loop's vitals while it runs (crate::flight), else the
/// minimal panel; None without an attitude or an airspeed quantity.
fn flight_panel(topology: &crate::devices::Topology, now_us: u64) -> Option<Value> {
    let attitude = topology
        .vector("attitude")
        .and_then(|parameter| parameter.read(now_us))
        .map(|reading| reading.value.map(|degrees| degrees.round_ties_even() as i32));
    if let Some(vitals) = crate::flight::vitals(attitude) {
        return Some(vitals);
    }
    let airspeed = topology.scalar("airspeed");
    if topology.vector("attitude").is_none() && airspeed.is_none() {
        return None;
    }
    serde_json::to_value(FlightPanel {
        airspeed: airspeed
            .and_then(|parameter| parameter.read(now_us))
            .map(|reading| (reading.value * 10.0).round_ties_even() / 10.0),
        attitude: attitude.map(|[heading, roll, pitch]| AttitudePanel { heading, roll, pitch }),
    })
    .ok()
}

/// One `health.tasks` item.
#[derive(Serialize)]
struct Task<'a> {
    /// The device's name.
    name: &'a str,
    /// False once its loop crashed.
    ok: bool,
}

/// `ok <json>`: the heartbeat vitals. The stage, the arming and the hold come from one snapshot of the flight word.
pub fn health(_message: &Message, context: &mut Context) -> Reply {
    let mission = &context.mission;
    let flight = FLIGHT.load();
    let fix = crate::gnss::position(context.topology);
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
        stage: flight.stage.name(),
        layout: &context.layout,
        position: fix.map(|fix| fix.degrees()),
        // plan B cc_client: the laser's AGL when fresh (a stale range-limited value is no AGL)
        agl: context
            .topology
            .scalar("agl")
            .and_then(|agl| agl.read(uptime_us()).map(|reading| reading.value)),
        clock: clock::text(),
        epoch: clock::epoch(),
        // decimal degrees on the wire: f64 at this JSON boundary only (f32 prints 25.514379 as 25.51438)
        launchpad: mission.launch_point(fix).map(|point| point.degrees()),
        launchpad_set: mission.position_set(),
        site: Some(mission.site.clone()).filter(|site| !site.is_empty()),
        armed: flight.armed,
        tasks: context
            .devices
            .tasks()
            .into_iter()
            .map(|(name, ok)| Task { name, ok })
            .collect(),
        flight: flight_panel(context.topology, uptime_us()),
        calibration: Some(crate::owner::calibrations().into_iter().collect::<BTreeMap<_, _>>())
            .filter(|pending| !pending.is_empty()),
        imu_calibration: crate::sensors::absolute_state().and_then(|state| state.calibration),
        degraded: degraded(context, flight),
    };
    super::ok_json(&info)
}

/// How far apart two accelerometers at rest may read before `health` says so: 10 degrees (D15 of the M9 plan).
const ACCEL_DISAGREE_DEGREES: f32 = 10.0;

/// The live sources of one quantity that disagree (D15 of the M9 plan; the operator's Q5: information for the
/// reader, never an arm gate -- the stage machine's own peer vote handles an outlier): two accelerometers more than
/// [`ACCEL_DISAGREE_DEGREES`] apart while both read inside the still band (a part mounted off its `axes`), and two
/// barometers' elevations more than half `launch_alt_m` apart (a re-zero missed). Each its own line; none when they
/// agree or fewer than two are live.
fn disagreements(context: &Context) -> Vec<String> {
    let settings = board_config::entries(&context.config.document)
        .iter()
        .find(|entry| entry.implementation == "sequencer")
        .and_then(|entry| sequencer::Settings::from_config(&serde_json::Value::Object(entry.fields.clone())).ok())
        .unwrap_or_default();
    let now_us = uptime_us();
    let mut lines = Vec::new();
    if let Some(accel) = context.topology.vector("accel") {
        let at_rest: Vec<(&str, [f32; 3])> = accel
            .sources()
            .filter_map(|source| Some((source, accel.channel(source)?.recent(accel.window_us(), now_us)?)))
            .filter(|(_, [x, y, z])| ((x * x + y * y + z * z).sqrt() - 1.0).abs() <= settings.still_g)
            .take(2)
            .collect();
        if let [(first, a), (second, b)] = at_rest[..] {
            let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            let norms = (a.iter().map(|v| v * v).sum::<f32>() * b.iter().map(|v| v * v).sum::<f32>()).sqrt();
            let degrees = (dot / norms).clamp(-1.0, 1.0).acos().to_degrees();
            if degrees > ACCEL_DISAGREE_DEGREES {
                lines.push(format!("sources-disagree: accel {first}/{second} {degrees:.1} deg"));
            }
        }
    }
    if let Some(elevation) = context.topology.scalar("elevation") {
        let live: Vec<(&str, f32)> = elevation
            .sources()
            .filter(|source| *source != crate::control::GNSS_SOURCE)
            .filter_map(|source| {
                Some((
                    source,
                    elevation.channel(source)?.recent(elevation.window_us(), now_us)?,
                ))
            })
            .take(2)
            .collect();
        if let [(first, a), (second, b)] = live[..]
            && (a - b).abs() > settings.launch_alt_m / 2.0
        {
            lines.push(format!(
                "sources-disagree: elevation {first}/{second} {:.1} m",
                (a - b).abs()
            ));
        }
    }
    lines
}

/// The non-nominal states, one operator signal (src/glider cc_client health `degraded`), `flight` the snapshot the
/// rest of the reply shows. Empty = nominal.
fn degraded(context: &Context, flight: Flight) -> Vec<String> {
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
    // the attitude flags, in plan B's place (cc_client `_degraded`): flying on a backup's attitude, and blind
    if let Some(attitude) = context.topology.vector("attitude") {
        let now_us = uptime_us();
        let winner = attitude.read(now_us).map(|reading| reading.source);
        if control::attitude::on_backup(&context.config.document, "attitude", winner) {
            degraded.push("attitude-backup".into());
        }
    }
    if crate::control::attitude_blind() {
        degraded.push("attitude-blind".into());
    }
    degraded.extend(disagreements(context));
    if flight.manual {
        degraded.push("STAGE HELD".into()); // an operator hold suppresses every stage detector
    }
    if context.config.source.starts_with("default(fallback") {
        // board.config failed to load, so this airframe booted the bench default: id taster, watchdog off
        degraded.push(format!("CONFIG FALLBACK -- {}", context.config.source));
    }
    if !crate::owner::calibrations().is_empty() {
        degraded.push("needs-calibration".into()); // a device waits for its calibration (health.calibration says how)
    }
    if mission.site == "fallback" {
        degraded.push("cc-less-fallback".into());
    }
    if crate::status::WARM_STARTED.load(Ordering::Relaxed) {
        // a reset in flight resumed the stage the crumb saved (crate::warm)
        degraded.push("WARM-STARTED (rebooted in flight)".into());
    }
    degraded
}
