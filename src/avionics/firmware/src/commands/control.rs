//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! stage / arm / disarm / report -- the stage hold and the actuation gate (src/glider cc_client
//! `_register_control`, controller.hold/resume/arm/disarm).

use std::collections::BTreeMap;

use cc_protocol::Message;
use sequencer::Stage;
use serde_json::json;

use super::{Code, Context, Fault, Reply};
use crate::status::STATUS;

/// What blocks the arm gate until the flight loop is ported (port plan M7): the gate is honest, never open.
pub const NOT_PORTED: &str = "flight -- not ported yet";

/// `stage` reports the stage; `stage <name>` forces it and holds it (a ground test), except that `stage setting`
/// is a return to the ground, not a hold; `stage auto` releases a hold. Replies {stage, manual}.
pub fn stage(message: &Message, _context: &mut Context) -> Reply {
    match message.args.first().map(String::as_str) {
        None => {}
        Some("auto") => {
            STATUS.set_manual(false);
            log::info!("controller :: stage auto");
        }
        Some(name) => {
            let stage =
                Stage::from_name(name).ok_or_else(|| Fault::new(Code::BadArguments, format!("no stage {name}")))?;
            STATUS.set_stage(stage);
            // holding SETTING suppressed every launch detector, so a board that had a fin check and then
            // `stage setting` flew its whole flight recorded as SETTING (plan B controller.hold)
            STATUS.set_manual(stage != Stage::Setting);
            let state = if STATUS.manual() { "held" } else { "auto" };
            log::info!("controller :: stage {name} ({state})");
        }
    }
    super::ok_json(&json!({"stage": STATUS.stage().name(), "manual": STATUS.manual()}))
}

/// `arm`: enable actuation only when nothing is wrong; refused with every problem otherwise. Until the flight
/// loop exists it always refuses, naming that first.
pub fn arm(_message: &Message, context: &mut Context) -> Reply {
    let mut problems: BTreeMap<&str, String> = BTreeMap::new();
    problems.insert("flight", NOT_PORTED.into());
    if let Some(problem) = super::stage_problem() {
        problems.insert("stage", problem);
    }
    if super::outside_setting().is_none() {
        // the probes sweep the fins: only in SETTING
        if let Some(problem) = super::diagnostics::probe_mission(context) {
            problems.insert("mission", problem);
        }
    }
    let json = serde_json::to_string(&problems).map_err(|error| Fault::new(Code::Internal, error.to_string()))?;
    Err(Fault::new(Code::Unsafe, json))
}

/// `disarm`: actuation off -- always allowed, it is how the operator recovers.
pub fn disarm(_message: &Message, _context: &mut Context) -> Reply {
    STATUS.set_armed(false);
    log::info!("controller :: disarmed");
    super::ok_json(&json!({"armed": false}))
}

/// `report`: the controller's stats (src/glider Controller.stats): the stage and the per-task snapshots.
pub fn report(_message: &Message, _context: &mut Context) -> Reply {
    super::ok_json(&json!({"stage": STATUS.stage().name(), "tasks": {}}))
}
