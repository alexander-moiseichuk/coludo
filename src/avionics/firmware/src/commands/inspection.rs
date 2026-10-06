//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! objects / inspect / update / stats -- the operator-facing objects (src/glider cc_client `_register_inspection`,
//! inspector.Inspector). The registry is fixed: the controller (the flight state) and the mission; device tasks
//! join as their drivers are ported.

use cc_protocol::Message;
use serde_json::{Value, json};

use super::{Code, Context, Fault, Reply};
use crate::clock;
use crate::status::STATUS;

/// The inspectable objects, sorted (plan B `Inspector.names()`).
const OBJECTS: [&str; 2] = ["controller", "mission"];

/// `objects`: the names `inspect`/`update`/`stats` accept.
pub fn objects(_message: &Message, _context: &mut Context) -> Reply {
    super::ok_json(&OBJECTS)
}

/// `inspect <object>`: its property snapshot.
pub fn inspect(message: &Message, context: &mut Context) -> Reply {
    let name = message
        .args
        .first()
        .ok_or_else(|| Fault::new(Code::BadArguments, "inspect <object>"))?;
    match name.as_str() {
        "controller" => super::ok_json(&json!({
            "stage": STATUS.stage().name(),
            "armed": STATUS.armed(),
            "manual": STATUS.manual(),
            "tasks": [],
            "failures": {},
            "alternatives": {},
        })),
        "mission" => super::ok_json(&context.mission.inspect(None, &clock::text(), clock::epoch())),
        other => Err(no_object(other)),
    }
}

/// `update <object> <json>`: apply the writable properties; replies {changed: [names actually changed]}.
pub fn update(message: &Message, context: &mut Context) -> Reply {
    let [name, properties, ..] = message.args.as_slice() else {
        return Err(Fault::new(Code::BadArguments, "update <object> <json>"));
    };
    // plan B's order: the JSON is parsed inside the refusal guard (bad JSON answers `refused`), the object is
    // looked up next (`badargs`), and only the object sees the properties
    let properties: Value = super::parse_json(properties, Code::Refused, None)?;
    if !OBJECTS.contains(&name.as_str()) {
        return Err(no_object(name));
    }
    let properties = properties
        .as_object()
        .ok_or_else(|| Fault::new(Code::Refused, "the properties must be a JSON object"))?;
    let changed: Vec<&str> = match name.as_str() {
        "mission" => apply_mission(context, properties),
        _ => Vec::new(), // the controller: nothing on it is writable
    };
    super::ok_json(&json!({"changed": changed}))
}

/// Apply a mission update (plan B Mission.update): the launch fields, and `epoch` sets the wall clock.
fn apply_mission(context: &mut Context, properties: &serde_json::Map<String, Value>) -> Vec<&'static str> {
    let mut changed = context.mission.update(properties);
    if set_clock(properties) {
        changed.push("epoch");
    }
    changed
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
pub fn stats(message: &Message, _context: &mut Context) -> Reply {
    let name = message
        .args
        .first()
        .ok_or_else(|| Fault::new(Code::BadArguments, "stats <object>"))?;
    match name.as_str() {
        "controller" => super::ok_json(&json!({"stage": STATUS.stage().name(), "tasks": {}})),
        "mission" => super::ok_json(&json!({})),
        other => Err(no_object(other)),
    }
}

/// The answer for a name that is not an object.
fn no_object(name: &str) -> Fault {
    Fault::new(Code::BadArguments, format!("no object {name}"))
}
