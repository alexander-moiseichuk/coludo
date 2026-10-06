//! get-config / set-config / reset-config -- the named configs (src/glider cc_client `_register_config`).
//!
//! `launch` is launch.config: merge-applied into the mission and saved in the settings store. The board config
//! (`board`/`running`/`saved`/`default`) is plan B's built-in default, embedded from the golden fixture generated
//! out of src/glider config_default.py, until the config store lands (port plan M2); writing it is refused.

use std::sync::OnceLock;

use cc_protocol::Message;
use serde::Deserialize;
use serde_json::Value;

use super::{Code, Context, Fault, Reply};

/// What a board config write answers until the store exists.
const NO_STORE: &str = "config store -- port plan M2";

/// Plan B's built-in board config, as the golden generator wrote it (src/avionics/golden/config_default.json).
#[derive(Deserialize)]
pub struct Defaults {
    /// config_default.default(), the whole document, stamped with this firmware's version.
    pub document: Value,
    /// Its config_id, computed by plan B's config.config_id (which leaves board.firmware_version out, so the
    /// stamp does not change it).
    pub config_id: String,
}

/// The embedded default, parsed and stamped once. The fixture pins board.firmware_version to 'dev' (plan B reads
/// it from a deploy-generated file), so the running document carries this build's version instead.
pub fn defaults() -> &'static Defaults {
    static DEFAULTS: OnceLock<Defaults> = OnceLock::new();
    DEFAULTS.get_or_init(|| {
        let mut defaults: Defaults = serde_json::from_str(include_str!("../../../golden/config_default.json"))
            .expect("golden/config_default.json is generated, so it parses");
        defaults.document["board"]["firmware_version"] = crate::board::FIRMWARE_VERSION.into();
        defaults
    })
}

impl Defaults {
    /// The document's board.mcu.
    pub fn mcu(&self) -> &str {
        self.document["board"]["mcu"].as_str().unwrap_or("esp32p4")
    }
}

/// `get-config [name]`: board/running/saved/default (all the built-in default for now) or launch.
pub fn get_config(message: &Message, context: &mut Context) -> Reply {
    match message.args.first().map_or("board", String::as_str) {
        "board" | "running" | "saved" | "default" => super::ok_json(&defaults().document),
        "launch" => super::ok_json(&context.mission.persisted()),
        other => Err(Fault::new(Code::BadArguments, format!("unknown config {other}"))),
    }
}

/// `set-config <name> <json>`: launch merges into the mission and is saved; board waits for the config store.
pub fn set_config(message: &Message, context: &mut Context) -> Reply {
    let [name, payload, ..] = message.args.as_slice() else {
        return Err(Fault::new(Code::BadArguments, "set-config <name> <json>"));
    };
    let payload: Value = super::parse_json(payload, Code::BadArguments, Some("bad json"))?;
    match name.as_str() {
        "board" => Err(Fault::new(Code::Unsupported, NO_STORE)),
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

/// `reset-config`: waits for the config store.
pub fn reset_config(_message: &Message, _context: &mut Context) -> Reply {
    Err(Fault::new(Code::Unsupported, NO_STORE))
}
