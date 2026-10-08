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

/// The inspectable objects every board has, sorted (plan B `Inspector.names()`).
const OBJECTS: [&str; 2] = ["controller", "mission"];

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

/// A driver's own inspect keys (plan B lsm6dso32.py:300-308, icp10111.py:455-459, separation.py:190-193).
#[derive(Serialize)]
#[serde(untagged)]
enum Own {
    /// The LSM6DSO32: its INT1 line, and its own channels' latest values (g, deg/s).
    Imu {
        interrupt: bool,
        interrupt_silent: bool,
        accel_g: Option<[f32; 3]>,
        rate_dps: Option<[f32; 3]>,
    },
    /// The BMI323 (M9): the LSM6DSO32's keys -- its INT1 wired and silent (None: no wire declared, polled by
    /// design, plan B bmi323.py:310-315), its own channels' latest values -- and the edges its latest read took; the
    /// port's own: its measured ODR (null before two conversions), its channels' lapses by quantity, and its raced
    /// and identical halves (accel, rate).
    Bmi323 {
        interrupt: bool,
        interrupt_silent: Option<bool>,
        accel_g: Option<[f32; 3]>,
        rate_dps: Option<[f32; 3]>,
        irq_runs: u32,
        odr_hz: Option<f32>,
        lapses: serde_json::Map<String, Value>,
        raced: [u32; 2],
        identical: [u32; 2],
    },
    /// A barometer, the ICP-10111 or the BMP581: its own channels' latest values, and its climb rate (m/s, M9).
    Barometer {
        altitude_m: Option<f32>,
        temperature_c: Option<f32>,
        pressure_pa: Option<f32>,
        elevation_m: Option<f32>,
        change_m_s: Option<f32>,
    },
    /// The BMP581 (M9): the barometer keys, and its double read's guard: the pairs that disagreed and the agreeing
    /// reads outside the part's operating range, since boot.
    Bmp581 {
        altitude_m: Option<f32>,
        temperature_c: Option<f32>,
        pressure_pa: Option<f32>,
        elevation_m: Option<f32>,
        change_m_s: Option<f32>,
        torn: u32,
        implausible: u32,
    },
    /// The BMM350 (plan B bmm350.py:382-387): its channel's latest counts, whether its calibration is in force, the
    /// octants its turn covered and the samples it has seen.
    Magnetometer {
        mag_counts: Option<[f32; 3]>,
        calibrated: bool,
        sectors: u32,
        samples: u32,
    },
    /// The separation switch: its debounced latch.
    Separation { separated: bool },
    /// The checkpoint: the crumb in NVS, this boot's gate, the NVS action owed, and its tallies (crate::warm).
    Checkpoint(crate::warm::Inspection),
    /// The watchdog: the task watchdog, the stall check and their margins (crate::supervisor).
    Watchdog(crate::supervisor::Inspection),
    /// The bluetooth stub (plan B bluetooth.py:67-77): the radio state asked for, and the radio's own, always null:
    /// this firmware starts no BLE radio, plan B's answer on a board with none.
    Bluetooth { radio: Value, active: Option<bool> },
    /// Any other device: nothing of its own yet.
    None {},
}

/// A device's snapshot (plan B `Task.inspect` and its drivers' own keys): None for a name that is no device that
/// came up. `healthy` is the table's, which the part's producer writes from each read (plan B's `Task._healthy`); each
/// own value is the part's latest while it is inside its quantity's window, else null (§6: a stale value reads null).
fn device<'a>(context: &Context, name: &'a str) -> Option<Snapshot<'a>> {
    let entry = context.devices.find(name).filter(|entry| entry.active())?;
    let ok = matches!(entry.status(), controller::Status::Up);
    let topology = context.topology;
    // SAFETY: a plain read of the monotonic timer.
    let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64;
    let scalar = |quantity: &str| {
        let parameter = topology.scalar(quantity)?;
        parameter.channel(name)?.recent(parameter.window_us(), now_us)
    };
    let vector = |quantity: &str| {
        let parameter = topology.vector(quantity)?;
        parameter.channel(name)?.recent(parameter.window_us(), now_us)
    };
    let own = match entry.implementation {
        "lsm6dso32" if crate::control::imu().is_some_and(|(imu, ..)| imu == name) => {
            let (_, interrupt, interrupt_silent) = crate::control::imu()?;
            Own::Imu {
                interrupt,
                interrupt_silent,
                accel_g: vector("accel"),
                rate_dps: vector("rate"),
            }
        }
        "bmi323" => {
            let (interrupt_silent, irq_runs, odr_hz, raced, identical) = crate::sensors::bmi323_inspection();
            Own::Bmi323 {
                interrupt: interrupt_silent.is_some(),
                interrupt_silent,
                accel_g: vector("accel"),
                rate_dps: vector("rate"),
                irq_runs,
                odr_hz,
                lapses: crate::control::lapses_of(name)
                    .into_iter()
                    .map(|(quantity, count)| (quantity, Value::from(count)))
                    .collect(),
                raced,
                identical,
            }
        }
        "icp10111" => Own::Barometer {
            altitude_m: scalar("altitude"),
            temperature_c: scalar("temperature"),
            pressure_pa: scalar("pressure"),
            elevation_m: scalar("elevation"),
            change_m_s: scalar("change"),
        },
        "bmp581" => {
            let (torn, implausible) = crate::sensors::bmp581_inspection();
            Own::Bmp581 {
                altitude_m: scalar("altitude"),
                temperature_c: scalar("temperature"),
                pressure_pa: scalar("pressure"),
                // on its row, not yet the databoard's (M9 stage 6): null until then
                elevation_m: scalar("elevation"),
                change_m_s: scalar("change"),
                torn,
                implausible,
            }
        }
        "bmm350" => {
            let (calibrated, sectors, samples) = crate::sensors::magnetometer_inspection();
            Own::Magnetometer {
                mag_counts: vector("mag"),
                calibrated,
                sectors,
                samples,
            }
        }
        "separation" => Own::Separation {
            separated: crate::control::separated(),
        },
        "checkpoint" => Own::Checkpoint(crate::warm::inspect(&context.nvs)),
        "watchdog" => Own::Watchdog(crate::supervisor::inspect()),
        "bluetooth" => Own::Bluetooth {
            radio: with_bluetooth(context, name, |stub| stub.radio().clone()),
            active: None,
        },
        _ => Own::None {},
    };
    Some(Snapshot {
        name,
        ok,
        healthy: entry.healthy(),
        own,
    })
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
        "mission" => super::ok_json(&context.mission.inspect(None, &clock::text(), clock::epoch())),
        "databoard" => {
            // SAFETY: a plain read of the monotonic timer.
            let now_us = unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64;
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
        "controller" => Vec::new(), // nothing on it is writable
        device if is_bluetooth(context, device) => with_bluetooth(context, device, |stub| stub.update(properties)),
        // a device: the part on its owner (the barometer's ground and rezero); the others change nothing
        device => crate::owner::update(device, &Value::Object(properties.clone()))
            .map_err(|error| Fault::new(Code::Internal, error))?
            .unwrap_or(Ok(Vec::new()))
            .map_err(|refusal| Fault::new(Code::Refused, refusal))?,
    };
    super::ok_json(&json!({"changed": changed}))
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
        RECORDER => super::ok_json(&crate::recorder::stats_report().ok_or_else(|| no_object(RECORDER))?),
        // a device: plan B's Inspectable.stats() default
        device if context.devices.find(device).is_some_and(|entry| entry.active()) => super::ok_json(&json!({})),
        other => Err(no_object(other)),
    }
}

/// The answer for a name that is not an object.
fn no_object(name: &str) -> Fault {
    Fault::new(Code::BadArguments, format!("no object {name}"))
}
