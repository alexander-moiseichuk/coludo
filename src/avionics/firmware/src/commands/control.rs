//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! stage / arm / disarm / report -- the stage hold and the actuation gate (src/glider cc_client
//! `_register_control`, controller.hold/resume/arm/disarm).
//!
//! The flight word has one writer, the control thread: `stage` and `disarm` post their request to the executor's
//! request slot (crate::control::request), which the next tick applies by plan B's Controller rules, and log the
//! controller's lines from its answer, in plan B's order.

use std::collections::BTreeMap;

use cc_protocol::Message;
use sequencer::{Flight, Request, Stage};
use serde_json::json;

use super::{Code, Context, Fault, Reply};
use crate::logging::CONSOLE_ONLY;
use crate::status::FLIGHT;

/// What blocks the arm gate until the flight loop is ported (port plan M7): the gate is honest, never open.
pub const NOT_PORTED: &str = "flight -- not ported yet";

/// `stage` reports the stage; `stage <name>` forces it and holds it (a ground test), except that `stage setting`
/// is a return to the ground, not a hold; `stage auto` releases a hold. Replies {stage, manual}, the flight as the
/// control thread committed it.
///
/// A hold logs `controller :: stage -> <stage>` to the console only, then `controller :: stage <stage> (held|auto)`,
/// even when the stage did not change, as plan B's `hold` logs through `set_stage`; a release logs `controller ::
/// stage auto`. A name that is no stage is refused before anything is posted.
pub fn stage(message: &Message, _context: &mut Context) -> Reply {
    let request = match message.args.first().map(String::as_str) {
        None => return stage_reply(FLIGHT.load()),
        Some("auto") => Request::Resume,
        Some(name) => Request::Hold(
            Stage::from_name(name).ok_or_else(|| Fault::new(Code::BadArguments, format!("no stage {name}")))?,
        ),
    };
    let flight = applied(request)?;
    if let Request::Hold(_) = request {
        let stage = flight.stage.name();
        log::info!(target: CONSOLE_ONLY, "controller :: stage -> {stage}");
        let state = if flight.manual { "held" } else { "auto" };
        log::info!("controller :: stage {stage} ({state})");
    } else {
        log::info!("controller :: stage auto");
    }
    stage_reply(flight)
}

/// The `stage` reply: {stage, manual} of `flight`.
fn stage_reply(flight: Flight) -> Reply {
    super::ok_json(&json!({"stage": flight.stage.name(), "manual": flight.manual}))
}

/// `request` applied by the control thread (crate::control::request): the flight it committed.
///
/// Errors: `err unsafe` with the stage problem when the control thread refused it (an arm off the ground or under a
/// hold); `err internal` when it did not answer, saying whether the request was applied.
fn applied(request: Request) -> Result<Flight, Fault> {
    crate::control::request(request)
        .map_err(|unanswered| Fault::new(Code::Internal, unanswered))?
        .map_err(|problem| Fault::new(Code::Unsafe, problem.to_string()))
}

/// `arm`: enable actuation only when nothing is wrong; refused with every problem otherwise. Until the flight
/// loop exists it always refuses, naming that first.
pub fn arm(_message: &Message, context: &mut Context) -> Reply {
    // every device that is not up, as plan B's arm lists its controller's failures (cc_client.py:566)
    let mut problems: BTreeMap<&str, String> = context.devices.failures().into_iter().collect();
    problems.insert("flight", NOT_PORTED.into());
    let flight = FLIGHT.load();
    if let Some(problem) = flight.stage_problem() {
        problems.insert("stage", problem.to_string());
    }
    if flight.stage.change_refused(false).is_none() {
        // the probes sweep the fins: only in SETTING
        if let Some(problem) = super::diagnostics::probe_mission(context) {
            problems.insert("mission", problem);
        }
        problems.extend(super::diagnostics::probe_problems(context));
    }
    let json = serde_json::to_string(&problems).map_err(|error| Fault::new(Code::Internal, error.to_string()))?;
    Err(Fault::new(Code::Unsafe, json))
}

/// `disarm`: actuation off -- always allowed, it is how the operator recovers. Replies {armed}, the arming the control
/// thread committed (always false).
pub fn disarm(_message: &Message, _context: &mut Context) -> Reply {
    let flight = applied(Request::Disarm)?;
    log::info!("controller :: disarmed");
    super::ok_json(&json!({"armed": flight.armed}))
}

/// `report`: the controller's stats (src/glider Controller.stats): the stage and the per-task snapshots.
pub fn report(_message: &Message, _context: &mut Context) -> Reply {
    super::ok_json(&json!({"stage": FLIGHT.load().stage.name(), "tasks": {}}))
}
