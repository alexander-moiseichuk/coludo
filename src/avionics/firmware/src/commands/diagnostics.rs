//! probe / verify / calibrate / detect / bustune -- the self-tests and the pre-flight check (src/glider cc_client
//! `_register_diagnostics`). Only the mission has a probe so far; the device self-tests, the bus scan and the bus
//! tuning arrive with their drivers (port plan M5-M7), and until then the answers say so instead of passing.

use std::collections::BTreeMap;

use cc_protocol::Message;
use serde_json::json;

use super::{Code, Context, Fault, Reply};

/// What `detect` and `bustune` answer until the buses are ported.
const NO_BUSES: &str = "the buses come with port plan M5";

/// `probe [name|all]`: {name: null | problem} for the objects with a self-test.
pub fn probe(message: &Message, context: &mut Context) -> Reply {
    match message.args.first().map_or("all", String::as_str) {
        "all" | "mission" => super::ok_json(&json!({"mission": probe_mission(context)})),
        other => Err(Fault::new(Code::BadArguments, format!("no probe for {other}"))),
    }
}

/// `verify`: the pre-flight check -- devices, problems, and the flight-readiness verdict. Never passes until the
/// flight loop exists.
pub fn verify(_message: &Message, context: &mut Context) -> Reply {
    let mut problems: BTreeMap<&str, String> = BTreeMap::new();
    problems.insert("flight", super::control::NOT_PORTED.into());
    if let Some(problem) = super::stage_problem() {
        problems.insert("stage", problem);
    }
    match super::in_flight() {
        Some(airborne) => {
            problems.insert("probe", format!("not run: {airborne} (the probes sweep the fins)"));
        }
        None => {
            if let Some(problem) = probe_mission(context) {
                problems.insert("mission", problem);
            }
        }
    }
    let readiness = json!({"flight": "not ported yet: fins hold neutral -- ballistic"});
    super::ok_json(&json!({
        "pass": problems.is_empty(),
        "devices": {},
        "problems": problems,
        "alternatives": {},
        "ready": false,
        "readiness": readiness,
    }))
}

/// The mission's self-test (a launch position is set, the zone is in range), logged as plan B's mission.probe logs
/// it, so an operator's log window shows the check and its verdict.
pub fn probe_mission(context: &Context) -> Option<String> {
    log::info!("mission :: probe: launch position ...");
    // plan B logs the position as soon as it resolves, before the zone range check can fail
    if let Some(origin) = context.mission.launch_point(None) {
        let [latitude, longitude] = origin.degrees();
        log::info!("mission :: probe: position ok ({latitude:.5}, {longitude:.5})");
    }
    let problem = context.mission.probe(None);
    if let Some(problem) = &problem {
        log::warn!("mission :: probe FAILED: {problem}");
    }
    problem
}

/// `calibrate`: what needs calibrating (nothing yet); `calibrate <device>` runs one (no device has one yet).
pub fn calibrate(message: &Message, _context: &mut Context) -> Reply {
    let Some(target) = message.args.first() else {
        return super::ok_json(&json!({}));
    };
    if let Some(airborne) = super::in_flight() {
        // a baro calibrate mid-air re-zeroes elevation: the ground, to the stage machine
        return Err(Fault::new(Code::Unsafe, airborne));
    }
    Err(Fault::new(Code::BadArguments, format!("no calibrate for {target}")))
}

/// `detect`: the bus scan and the revision it looks like.
pub fn detect(_message: &Message, _context: &mut Context) -> Reply {
    Err(Fault::new(Code::Unsupported, NO_BUSES))
}

/// `bustune <kind> <id> <freq>`: retune a bus live.
pub fn bustune(_message: &Message, _context: &mut Context) -> Reply {
    Err(Fault::new(Code::Unsupported, NO_BUSES))
}
