//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! get-config / set-config / reset-config -- the named configs (src/glider cc_client `_register_config`).
//!
//! - `board`/`running`: the config this boot runs (immutable for the run: a change is save + reboot).
//! - `saved`: what the next boot loads -- the saved board.config through the same load the boot uses, so a
//!   read-modify-write starts from it and never reverts a save not yet rebooted.
//! - `default`: the built-in default.
//! - `launch`: launch.config, merge-applied into the mission and saved.

use cc_protocol::Message;
use serde_json::{Value, json};

use super::{Code, Context, Fault, Reply};

/// `get-config [name]`: board/running, saved, default, or launch (default `board`).
pub fn get_config(message: &Message, context: &mut Context) -> Reply {
    match message.args.first().map_or("board", String::as_str) {
        "board" | "running" => super::ok_json(&context.config.document),
        "saved" => {
            let saved = board_config::load(context.store.board().as_deref(), crate::config::defaults());
            super::ok_json(&saved.document)
        }
        "default" => super::ok_json(crate::config::defaults()),
        "launch" => super::ok_json(&context.mission.persisted()),
        other => Err(Fault::new(Code::BadArguments, format!("unknown config {other}"))),
    }
}

/// `set-config <name> <json>`: `board` validates and saves the whole document (it runs from the next boot) and
/// replies its config_id; `launch` merges into the mission and is saved.
pub fn set_config(message: &Message, context: &mut Context) -> Reply {
    let [name, payload, ..] = message.args.as_slice() else {
        return Err(Fault::new(Code::BadArguments, "set-config <name> <json>"));
    };
    let payload: Value = super::parse_json(payload, Code::BadArguments, Some("bad json"))?;
    match name.as_str() {
        "board" => {
            // plan B config.save: an invalid config is never written
            let errors = board_config::validate(&payload);
            if !errors.is_empty() {
                return Err(Fault::new(
                    Code::Invalid,
                    format!("invalid config: {}", errors.join("; ")),
                ));
            }
            context
                .store
                .save_board(&board_config::write(&payload))
                .map_err(|error| Fault::new(Code::Internal, format!("board.config not saved: {error}")))?;
            let config_id = board_config::config_id(&payload);
            log::info!("config :: saved {config_id} -- it runs from the next boot");
            super::ok_json(&json!({"config_id": config_id}))
        }
        "launch" => {
            let fields = payload
                .as_object()
                .ok_or_else(|| Fault::new(Code::BadArguments, "bad json"))?;
            // a key-wise merge: a field the payload leaves out keeps its value, never wiped; unlike plan B, a
            // `sites` list replaces the stored one (plan B's save dropped them)
            context.mission.merge(fields);
            super::inspection::set_clock(fields);
            let persisted = context.mission.persisted();
            context
                .store
                .save_launch(&persisted.to_string())
                .map_err(|error| Fault::new(Code::Internal, format!("launch.config not saved: {error}")))?;
            super::ok_json(&persisted)
        }
        other => Err(Fault::new(Code::BadArguments, format!("unknown config {other}"))),
    }
}

/// `reset-config`: forget the saved board.config, so the next boot runs the default.
pub fn reset_config(_message: &Message, context: &mut Context) -> Reply {
    context
        .store
        .remove_board()
        .map_err(|error| Fault::new(Code::Internal, format!("board.config not removed: {error}")))?;
    Ok(cc_protocol::build("ok", &[], &[]))
}
