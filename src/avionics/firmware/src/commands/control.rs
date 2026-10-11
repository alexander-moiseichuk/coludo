//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! stage / arm / disarm / report -- the stage hold and the actuation gate (src/glider cc_client
//! `_register_control`, controller.hold/resume/arm/disarm).
//!
//! The flight word has one writer, the control thread: `stage`, `arm` and `disarm` post their request to the
//! executor's request slot (crate::control::request), which the next tick applies by plan B's Controller rules, and
//! log the controller's lines from its answer, in plan B's order.

use std::collections::BTreeMap;

use cc_protocol::Message;
use sequencer::{Flight, Request, Stage};
use serde_json::json;

use super::{Code, Context, Fault, Reply};
use crate::logging::CONSOLE_ONLY;
use crate::status::FLIGHT;

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

/// `arm`: enable actuation only when nothing is wrong (plan B cc_client.py:547-576); replies {armed}, the arming the
/// control thread committed, pins the live GNSS fix as the launch point when the operator set none, and logs
/// `controller :: armed`.
///
/// Refused, `err unsafe` with every problem as {name: why}: each device that is not up (an entry the firmware cannot
/// run yet included, plan B's rule), the stage problem (off the ground, or held), and in SETTING each failing probe of
/// the sweep: the mission, the parts, the recorder's ring gate. A request that waited on the board until the hub may
/// have given up on its answer is refused under `arm` (controller::stale_arm), checked last, after the sweep. The
/// control thread checks the stage again when it applies the arm, since a launch may be detected during the sweep:
/// refused then under `stage` alone.
pub fn arm(_message: &Message, context: &mut Context) -> Reply {
    // every device that is not up, as plan B's arm lists its controller's failures (cc_client.py:566)
    let mut problems: BTreeMap<&str, String> = context.devices.failures().into_iter().collect();
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
    if let Some(stale) = controller::stale_arm(super::waited()) {
        problems.insert("arm", stale);
    }
    if !problems.is_empty() {
        return Err(refusal(&problems));
    }
    let answer = crate::control::request(Request::Arm).map_err(|unanswered| Fault::new(Code::Internal, unanswered))?;
    let flight = answer.map_err(|problem| refusal(&BTreeMap::from([("stage", problem.to_string())])))?;
    // the last moment the board is known to be on the pad: the live fix becomes the launch point, unless the operator
    // set one (plan B Controller.arm -> mission.freeze_launch)
    if let Some(point) = context.mission.freeze_launch(crate::gnss::position(context.topology)) {
        let [latitude, longitude] = point.degrees();
        log::info!("mission :: launch point frozen ({latitude:.5}, {longitude:.5})");
    }
    log::info!("controller :: armed");
    super::ok_json(&json!({"armed": flight.armed}))
}

/// `arm`'s refusal: `err unsafe {name: why}`.
fn refusal(problems: &BTreeMap<&str, String>) -> Fault {
    Fault::new(Code::Unsafe, json!(problems).to_string())
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
