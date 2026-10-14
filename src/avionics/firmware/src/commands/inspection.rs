//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! objects / inspect / update / stats -- the operator-facing objects (src/glider cc_client `_register_inspection`,
//! inspector.Inspector). The registry: the controller (the flight state), the mission, the recorder (with the flash
//! ring and the ring_saver) when it opened, and every device that came up (plan B registers each task as it comes
//! up), whose snapshot is plan B's `Task.inspect` -- name, ok, healthy -- and, for the parts the port drives so far,
//! their own keys read off the databoard (the IMU, the barometer, the separation switch), null once stale. The
//! bluetooth stub (drivers::bluetooth) adds plan B's `radio`, which `update` changes, and `active`, always null. The
//! checkpoint adds the crumb NVS holds now, this boot's gate, the NVS action it owes and its tallies (crate::warm);
//! the watchdog its settings, whether the task watchdog is armed, whether the board controls a flight, the control
//! tick's age, the longest gaps between feeds and between stall checks, and whether its restart is suppressed.

use std::collections::BTreeMap;
use std::sync::{Mutex, PoisonError};

use cc_protocol::Message;
use drivers::bluetooth::Bluetooth;
use serde::Serialize;
use serde_json::{Map, Value, json};

use super::{Code, Context, Fault, Reply};
use crate::clock;
use crate::recorder::NAME as RECORDER;
use crate::status::FLIGHT;
use firmware::time::uptime_us;

/// The inspectable objects every board has, sorted (plan B `Inspector.names()`).
const OBJECTS: [&str; 3] = ["controller", "databoard", "mission"];

/// Each bluetooth stub by its entry's name, made from its config entry when first inspected or updated. Only the
/// executor's commands reach it, one at a time, so the lock is never contended.
static BLUETOOTH: Mutex<BTreeMap<String, Bluetooth>> = Mutex::new(BTreeMap::new());

/// The object names `inspect`/`update`/`stats` accept now, sorted: the fixed ones, the recorder when it opened, and
/// every device that came up.
fn names(context: &Context) -> Vec<&'static str> {
    let mut names = OBJECTS.to_vec();
    if crate::recorder::get().is_some() {
        names.push(RECORDER);
    }
    names.extend(context.devices.tasks().into_iter().map(|(name, _)| name));
    names.sort_unstable();
    names.dedup();
    names
}

/// `objects`: the names `inspect`/`update`/`stats` accept.
pub fn objects(_message: &Message, context: &mut Context) -> Reply {
    super::ok_json(&names(context))
}

/// A device's snapshot, plan B's `Task.inspect` (name, ok, healthy) and its driver's own keys, in plan B's order.
/// Typed: a `json!` value would widen every f32 to f64 (a reading of 0.774944 would print as 0.774944007396698).
#[derive(Serialize)]
struct Snapshot<'a> {
    name: &'a str,
    /// It came up and its loop has not crashed (plan B's `ok`).
    ok: bool,
    /// Its latest read succeeded (plan B's `healthy`), as its producer recorded it in the table.
    healthy: bool,
    /// The driver's own keys.
    #[serde(flatten)]
    own: Own,
}

/// A driver's own inspect keys (plan B lsm6dso32.py:300-308, separation.py:190-193, ...).
#[derive(Serialize)]
#[serde(untagged)]
enum Own {
    /// An I2C part's (crates/parts): its published state and its channels' latest.
    Part(parts::Own),
    /// The LSM6DSO32: its INT1 line, and its own channels' latest values (g, deg/s).
    Imu {
        interrupt: bool,
        interrupt_silent: bool,
        accel_g: Option<[f32; 3]>,
        rate_dps: Option<[f32; 3]>,
    },
    /// The GNSS (plan B gnss.py inspect): the fix, its quality, satellites and HDOP, its own channels' latest (the
    /// position in degrees), the sky (in view, used, the GSA mode, the four strongest C/N0 and their mean, the
    /// antenna), and the port's own: the NMEA lines read since boot.
    Gnss {
        fix: bool,
        satellites: i32,
        hdop: f32,
        fix_quality: i32,
        position: Option<[f64; 2]>,
        altitude_m: Option<f32>,
        elevation_m: Option<f32>,
        speed_ms: Option<f32>,
        in_view: i32,
        used: i32,
        mode: i32,
        cn0: [i32; 4],
        cn0_mean: f32,
        antenna: &'static str,
        lines: u32,
    },
    /// The pad drift calibration (plan B gnss_calib.py inspect): the drift east and north (m/s, to 3 decimals), the
    /// samples since its restart, whether it froze.
    GnssCalib {
        drift_e: f32,
        drift_n: f32,
        samples: u32,
        frozen: bool,
    },
    /// The HITL sim-link (the port's own): its UDP port and socket, the frames applied, refused by reason and stale,
    /// the last frame's seq, sim time and age, and its sender.
    Hitl {
        port: u16,
        bound: bool,
        frames: u32,
        refused: serde_json::Map<String, Value>,
        stale: u32,
        last_seq: Option<u32>,
        sim_time_ms: Option<u32>,
        age_ms: Option<u64>,
        sender: Option<String>,
    },
    /// A fin servo (plan B sg90.py inspect): the COMMANDED angle and pulse -- an SG90 is open-loop, nothing is
    /// measured -- the trim, and the feedback it has none of (null).
    Servo {
        angle: i32,
        pulse_us: i32,
        trim: i32,
        feedback: Option<i32>,
    },
    /// The stage machine (M9): which source holds each slot, the sources past their class's slots, and the veto
    /// episode edges it reported.
    Sequencer {
        sources: String,
        ignored: Vec<String>,
        veto_edges: u32,
    },
    /// The attitude filter (plan B tasks/attitude.py `inspect`): its flags, angles and magnetic reference.
    Attitude(Value),
    /// The separation switch: its debounced latch.
    Separation { separated: bool },
    /// The checkpoint: the crumb in NVS, this boot's gate, the NVS action owed, and its tallies (crate::warm).
    Checkpoint(crate::warm::Inspection),
    /// The watchdog: the task watchdog, the stall check and their margins (crate::supervisor).
    Watchdog(crate::supervisor::Inspection),
    /// The bluetooth stub (plan B bluetooth.py:67-77): the radio state asked for, and the radio's own, always null:
    /// this firmware starts no BLE radio, plan B's answer on a board with none.
    Bluetooth { radio: Value, active: Option<bool> },
    /// The Wi-Fi station (plan B drivers/wifi.py inspect): the network it is on, whether associated, its address,
    /// the signal, the transmit power cap, and every configured network in order; `channel` beyond plan B's.
    Wifi {
        ssid: Option<String>,
        connected: bool,
        ip: Option<String>,
        rssi: Option<i32>,
        tx_power: Option<f32>,
        networks: Vec<String>,
        channel: Option<u8>,
    },
    /// Any other device: nothing of its own yet.
    None {},
}

/// A device's snapshot (plan B `Task.inspect` and its drivers' own keys): None for a name that is no device that
/// came up. `healthy` is the table's, which the part's producer writes from each read (plan B's `Task._healthy`); each
/// own value is the part's latest while it is inside its quantity's window, else null (§6: a stale value reads null).
fn device<'a>(context: &Context, name: &'a str) -> Option<Snapshot<'a>> {
    let entry = context.devices.find(name).filter(|entry| entry.active())?;
    let ok = matches!(entry.status(), controller::Status::Up);
    let now_us = uptime_us();
    let lookup = parts::Lookup {
        topology: context.topology,
        name,
        now_us,
        addr: address(context, name),
    };
    let own = match parts::own(
        entry.implementation,
        crate::sensors::shared(name),
        &lookup,
        &crate::sensors::PLATFORM,
    ) {
        Some(own) => Own::Part(own),
        None => component(context, entry.implementation, name, now_us)?,
    };
    Some(Snapshot {
        name,
        ok,
        healthy: entry.healthy(),
        own,
    })
}

/// The own keys of a device that is no I2C part (crates/parts answers for those): the SPI IMU, the GNSS and its
/// calibration, the sim-link, the servos, the control thread's components, the supervisor's, the bluetooth stub; None
/// for the SPI IMU when the control thread does not hold it.
fn component(context: &Context, implementation: &str, name: &str, now_us: u64) -> Option<Own> {
    let topology = context.topology;
    let vector = |quantity: &str| {
        let parameter = topology.vector(quantity)?;
        parameter.channel(name)?.recent(parameter.window_us(), now_us)
    };
    Some(match implementation {
        "lsm6dso32" if crate::control::imu().is_some_and(|(imu, ..)| imu == name) => {
            let (_, interrupt, interrupt_silent) = crate::control::imu()?;
            Own::Imu {
                interrupt,
                interrupt_silent,
                accel_g: vector("accel"),
                rate_dps: vector("rate"),
            }
        }
        "neo6mv2" | "atgm336h" | "neom8n" => gnss(topology, name, now_us),
        "gnss_calib" => match crate::gnss::calibration_inspection() {
            Some(([east, north], samples, frozen)) => Own::GnssCalib {
                drift_e: (east * 1000.0).round() / 1000.0,
                drift_n: (north * 1000.0).round() / 1000.0,
                samples,
                frozen,
            },
            None => Own::None {},
        },
        "hitl" => hitl(),
        "sg90" | "mg90s" => match crate::fins::inspection(name) {
            Some([angle, pulse_us, trim]) => Own::Servo {
                angle,
                pulse_us,
                trim,
                feedback: None,
            },
            None => Own::None {},
        },
        "attitude" => crate::control::attitude_inspection().map_or(Own::None {}, Own::Attitude),
        "sequencer" => match crate::control::sequencer_inspection() {
            Some((sources, ignored, veto_edges)) => Own::Sequencer {
                sources,
                ignored,
                veto_edges,
            },
            None => Own::None {},
        },
        "separation" => Own::Separation {
            separated: crate::control::separated(),
        },
        "checkpoint" => Own::Checkpoint(crate::warm::inspect(&context.nvs)),
        "watchdog" => Own::Watchdog(crate::supervisor::inspect()),
        "wifi" => wifi(context),
        "bluetooth" => Own::Bluetooth {
            radio: with_bluetooth(context, name, |stub| stub.radio().clone()),
            active: None,
        },
        _ => Own::None {},
    })
}

/// The GNSS's own keys: its fix, sky and line figures, and its latest position, altitude, elevation and speed while
/// inside their windows; no keys before its thread runs.
fn gnss(topology: &crate::devices::Topology, name: &str, now_us: u64) -> Own {
    let scalar = |quantity: &str| {
        let parameter = topology.scalar(quantity)?;
        parameter.channel(name)?.recent(parameter.window_us(), now_us)
    };
    match crate::gnss::inspection() {
        Some(state) => Own::Gnss {
            fix: state.fix,
            satellites: state.satellites,
            hdop: state.hdop,
            fix_quality: state.fix_quality,
            position: topology
                .position("position")
                .and_then(|parameter| parameter.channel(name)?.recent(parameter.window_us(), now_us))
                .map(|[latitude, longitude]| [f64::from(latitude) * 1e-7, f64::from(longitude) * 1e-7]),
            altitude_m: scalar("altitude"),
            elevation_m: scalar("elevation"),
            speed_ms: scalar("speed"),
            in_view: state.sky[0],
            used: state.sky[1],
            mode: state.sky[2],
            cn0: [state.sky[3], state.sky[4], state.sky[5], state.sky[6]],
            cn0_mean: state.sky[3..7].iter().sum::<i32>() as f32 / 4.0,
            antenna: state.antenna,
            lines: state.lines,
        },
        None => Own::None {},
    }
}

/// The Wi-Fi station's own keys (plan B wifi.py inspect): the C6's answer now, every key null (not connected) before
/// the driver is up; the configured networks either way.
fn wifi(context: &Context) -> Own {
    let networks = board_config::wifi(&context.config.document)
        .networks
        .into_iter()
        .map(|network| network.ssid)
        .collect();
    let station = crate::link::inspection();
    Own::Wifi {
        connected: station.as_ref().is_some_and(|station| station.connected),
        ssid: station.as_ref().and_then(|station| station.ssid.clone()),
        ip: station.as_ref().and_then(|station| station.ip.clone()),
        rssi: station.as_ref().and_then(|station| station.rssi),
        tx_power: station.as_ref().and_then(|station| station.tx_power),
        networks,
        channel: station.and_then(|station| station.channel),
    }
}

/// The sim-link's own keys: its port, the frames applied, refused (per reason) and stale, the last frame; no keys
/// before its thread runs.
fn hitl() -> Own {
    match crate::sim_link::inspection() {
        Some(state) => Own::Hitl {
            port: state.port,
            bound: state.bound,
            frames: state.applied,
            refused: state
                .refused
                .iter()
                .map(|(reason, count)| (reason.to_string(), Value::from(*count)))
                .collect(),
            stale: state.stale,
            last_seq: state.last.map(|(seq, _)| seq),
            sim_time_ms: state.last.map(|(_, sim_time_ms)| sim_time_ms),
            age_ms: state.age_ms,
            sender: state.sender,
        },
        None => Own::None {},
    }
}

/// `inspect <object>`: its property snapshot.
pub fn inspect(message: &Message, context: &mut Context) -> Reply {
    let name = message
        .args
        .first()
        .ok_or_else(|| Fault::new(Code::BadArguments, "inspect <object>"))?;
    match name.as_str() {
        "controller" => {
            let inspection = context.devices.inspect();
            let flight = FLIGHT.load();
            super::ok_json(&json!({
                "stage": flight.stage.name(),
                "armed": flight.armed,
                "manual": flight.manual,
                "tasks": inspection.tasks,
                "failures": inspection.failures.into_iter().collect::<BTreeMap<_, _>>(),
                "alternatives": inspection.alternatives.into_iter().collect::<BTreeMap<_, _>>(),
            }))
        }
        // the live fix resolves the pad before arm, as the arm's probe does (plan B mission.inspect)
        "mission" => super::ok_json(&context.mission.inspect(
            crate::gnss::position(context.topology),
            &clock::text(),
            clock::epoch(),
        )),
        "databoard" => {
            let now_us = uptime_us();
            super::ok_json(&context.topology.inspect(now_us))
        }
        RECORDER => super::ok_json(&crate::recorder::inspect_report().ok_or_else(|| no_object(RECORDER))?),
        other => super::ok_json(&device(context, other).ok_or_else(|| no_object(other))?),
    }
}

/// `update <object> <json>`: apply the writable properties; replies {changed: [names actually changed]}. A change: only
/// in SETTING, and at DONE too for a barometer's re-zero (super::change_refused), checked first as the table's gate
/// is.
pub fn update(message: &Message, context: &mut Context) -> Reply {
    let target = message.args.first().map_or("", String::as_str);
    if let Some(refusal) = super::change_refused(context, target) {
        return Err(Fault::new(Code::Unsafe, refusal));
    }
    let [name, properties, ..] = message.args.as_slice() else {
        return Err(Fault::new(Code::BadArguments, "update <object> <json>"));
    };
    // plan B's order: the JSON is parsed inside the refusal guard (bad JSON answers `refused`), the object is
    // looked up next (`badargs`), and only the object sees the properties
    let properties: Value = super::parse_json(properties, Code::Refused, None)?;
    if !names(context).contains(&name.as_str()) {
        return Err(no_object(name));
    }
    let properties = properties
        .as_object()
        .ok_or_else(|| Fault::new(Code::Refused, "the properties must be a JSON object"))?;
    let changed: Vec<&str> = match name.as_str() {
        "mission" => apply_mission(context, properties),
        RECORDER => apply_recorder(properties),
        "controller" | "databoard" => Vec::new(), // nothing on them is writable
        device if is_bluetooth(context, device) => with_bluetooth(context, device, |stub| stub.update(properties)),
        device if is_wifi(context, device) => apply_wifi(properties)?,
        // a fin servo: on the control thread, its owner (trim, angle)
        device if crate::fins::inspection(device).is_some() => {
            crate::control::update_fin(device, &Value::Object(properties.clone()))
                .map_err(|error| Fault::new(Code::Internal, error))?
                .map_err(|refusal| Fault::new(Code::Refused, refusal))?
        }
        // a device: the part on its owner (the barometer's ground and rezero); the others change nothing
        device => {
            let changed = crate::owner::update(device, &Value::Object(properties.clone()))
                .map_err(|error| Fault::new(Code::Internal, error))?
                .unwrap_or(Ok(Vec::new()))
                .map_err(|refusal| Fault::new(Code::Refused, refusal))?;
            // a tare the update captured (the pitot's `zero`) is persisted here, off the flight core
            if let Some(why) = crate::sensors::persist_pitot_tare(&context.nvs) {
                log::warn!("{device} :: {why}");
            }
            changed
        }
    };
    super::ok_json(&json!({"changed": changed}))
}

/// Whether the device `name` is the Wi-Fi station.
fn is_wifi(context: &Context, name: &str) -> bool {
    context
        .devices
        .find(name)
        .is_some_and(|entry| entry.implementation == "wifi")
}

/// Apply a Wi-Fi update (plan B wifi.update): `tx_power`, dBm, when it is a number other than the current cap; the
/// driver's refusal answers `refused`.
fn apply_wifi(properties: &serde_json::Map<String, Value>) -> Result<Vec<&'static str>, Fault> {
    let Some(dbm) = properties.get("tx_power").and_then(Value::as_f64) else {
        return Ok(Vec::new());
    };
    let dbm = dbm as f32;
    let current = crate::link::inspection().and_then(|station| station.tx_power);
    if current.is_some_and(|current| (current - dbm).abs() < 0.25) {
        return Ok(Vec::new());
    }
    crate::link::set_tx_power(dbm).map_err(|refusal| Fault::new(Code::Refused, refusal))?;
    Ok(vec!["tx_power"])
}

/// Whether the device `name` is a bluetooth stub.
fn is_bluetooth(context: &Context, name: &str) -> bool {
    context
        .devices
        .find(name)
        .is_some_and(|entry| entry.implementation == "bluetooth")
}

/// Runs `action` on the bluetooth stub of the device `name`, made from its config entry the first time (plan B's
/// setup reads `radio` from it).
fn with_bluetooth<T>(context: &Context, name: &str, action: impl FnOnce(&mut Bluetooth) -> T) -> T {
    let mut stubs = BLUETOOTH.lock().unwrap_or_else(PoisonError::into_inner);
    let stub = stubs.entry(name.to_string()).or_insert_with(|| {
        let entry = board_config::directory(&context.config.document)
            .into_iter()
            .find(|entry| entry.name == name);
        Bluetooth::from_config(entry.map_or(&Map::new(), |entry| entry.fields))
    });
    action(stub)
}

/// Apply a mission update (plan B Mission.update): the launch fields, and `epoch` sets the wall clock.
fn apply_mission(context: &mut Context, properties: &serde_json::Map<String, Value>) -> Vec<&'static str> {
    let mut changed = context.mission.update(properties);
    if set_clock(properties) {
        changed.push("epoch");
    }
    changed
}

/// Apply a recorder update (plan B Recorder.update): `stats_ms`, the status line's period, when it is a positive
/// integer other than the current one; anything else changes nothing.
fn apply_recorder(properties: &serde_json::Map<String, Value>) -> Vec<&'static str> {
    let stats_ms = properties
        .get("stats_ms")
        .and_then(Value::as_u64)
        .and_then(|stats_ms| u32::try_from(stats_ms).ok());
    match (crate::recorder::get(), stats_ms) {
        (Some(recorder), Some(stats_ms)) if recorder.set_stats_ms(stats_ms) => vec!["stats_ms"],
        _ => Vec::new(),
    }
}

/// Set the wall clock when the properties carry an acceptable `epoch` (plan B Mission.set_time), and tie this boot
/// to it with a session.csv row (plan B Mission._index_time); whether the clock was set.
///
/// `utc_offset`, `cc_position` and `source` ride in the same properties as momentary inputs: written to the row,
/// never stored, an invalid one an empty cell (mission::TimeSet). A row the full ring refuses is logged and lost,
/// never the time set: failing the update would have CC report a time set that happened as refused.
pub fn set_clock(properties: &serde_json::Map<String, Value>) -> bool {
    let Some(epoch) = mission::epoch_request(properties) else {
        return false;
    };
    let before = clock::text();
    clock::set(epoch);
    log::info!("mission :: clock set {before} -> {}", clock::text());
    if let Some(recorder) = crate::recorder::get() {
        let time_set = mission::TimeSet::from_properties(properties);
        let cc_position = time_set
            .cc_position
            .map(|position| (position.latitude_e7, position.longitude_e7));
        let source = time_set.source.as_deref().unwrap_or_default();
        // the clock as it reads now, as plan B's time.time(): a time set into 2000 dates nothing
        if let Err(overflow) = recorder.index_session(clock::epoch(), time_set.utc_offset, source, cc_position) {
            log::warn!("Mission :: session.csv row lost, the clock stays set: {overflow}");
        }
    }
    true
}

/// `stats <object>`: its runtime numbers.
pub fn stats(message: &Message, context: &mut Context) -> Reply {
    let name = message
        .args
        .first()
        .ok_or_else(|| Fault::new(Code::BadArguments, "stats <object>"))?;
    match name.as_str() {
        "controller" => super::ok_json(&json!({"stage": FLIGHT.load().stage.name(), "tasks": {}})),
        "mission" => super::ok_json(&json!({})),
        // plan B's Databoard.stats: each quantity's sources
        "databoard" => super::ok_json(&context.topology.stats()),
        // plan B's Attitude.stats: the inspect snapshot
        "attitude" if crate::control::attitude_inspection().is_some() => {
            super::ok_json(&crate::control::attitude_inspection().unwrap_or_default())
        }
        RECORDER => super::ok_json(&crate::recorder::stats_report().ok_or_else(|| no_object(RECORDER))?),
        // plan B's Wifi.stats
        device if is_wifi(context, device) => {
            let station = crate::link::inspection();
            super::ok_json(&json!({
                "connected": station.as_ref().is_some_and(|station| station.connected),
                "rssi": station.and_then(|station| station.rssi),
            }))
        }
        // a device: plan B's Inspectable.stats() default
        device if context.devices.find(device).is_some_and(|entry| entry.active()) => super::ok_json(&json!({})),
        other => Err(no_object(other)),
    }
}

/// An I2C device's address as plan B's inspect shows it (`hex(self._addr)`, "0x69"): its entry's `addr`, else its
/// driver's datasheet default; None for a device not in the config.
fn address(context: &Context, name: &str) -> Option<String> {
    let entries = board_config::entries(&context.config.document);
    let entry = entries.iter().find(|entry| entry.name == name)?;
    let address = entry
        .fields
        .get("addr")
        .and_then(Value::as_u64)
        .or_else(|| parts::default_address(entry.implementation).map(u64::from))?;
    Some(format!("{address:#x}"))
}

/// The answer for a name that is not an object.
fn no_object(name: &str) -> Fault {
    Fault::new(Code::BadArguments, format!("no object {name}"))
}
