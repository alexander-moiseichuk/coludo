//! The CC commands (doc/specs/cc-protocol.md; src/glider cc_client.py): one request line in, at most one reply line
//! out, through one table.
//!
//! Every line -- from the hub (link.rs) and from the console -- runs on one executor thread, one at a time, so a
//! handler owns the state it touches (the mission) without a lock, and two operators can never interleave half a
//! command. The envelope is plan B's: an unparsable line answers `err badargs`, an unknown command `err badcmd`,
//! a handler failure `err internal`, an empty line nothing; an active command outside the ground stages answers
//! `err unsafe` with plan B's refusal text.

mod config;
mod control;
mod diagnostics;
mod identity;
mod inspection;
mod streaming;
mod system;

use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};

use cc_protocol::Message;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use mission::Mission;
use sequencer::Stage;

use crate::board::Board;
use crate::status::STATUS;

/// What the handlers share: the board, the settings store, and the state the executor owns.
pub struct Context {
    /// Who this board is, and its vitals.
    pub board: Arc<Board>,
    /// The `cfg` settings store (board.config, launch.config).
    pub store: crate::config::Store,
    /// The per-launch mission (src/glider mission.Mission), owned here until a supervisor thread takes it.
    pub mission: Mission,
    /// The board config this boot runs, how it was chosen, and its validation errors (plan B config.load).
    pub config: board_config::Loaded,
    /// The running config's id, hashed once at boot (it is immutable for the run).
    pub config_id: String,
}

/// The error codes on the wire (cc-protocol.md "Errors"), as plan B answers them.
#[derive(Clone, Copy, Debug)]
pub enum Code {
    /// An unknown command.
    BadCommand,
    /// Missing or malformed arguments.
    BadArguments,
    /// A config the validator rejects.
    Invalid,
    /// Refused for safety: in flight, or a failed arm gate.
    Unsafe,
    /// A device refused a property change.
    Refused,
    /// Not on this board (yet).
    Unsupported,
    /// The handler itself failed.
    Internal,
}

impl Code {
    /// The code's token on the wire.
    fn token(self) -> &'static str {
        match self {
            Code::BadCommand => "badcmd",
            Code::BadArguments => "badargs",
            Code::Invalid => "invalid",
            Code::Unsafe => "unsafe",
            Code::Refused => "refused",
            Code::Unsupported => "unsupported",
            Code::Internal => "internal",
        }
    }
}

/// A command's failure: `err <code> <detail>`.
#[derive(Debug)]
pub struct Fault {
    /// The error code.
    pub code: Code,
    /// One line saying what was wrong.
    pub detail: String,
}

impl Fault {
    /// A fault with this code and text.
    pub fn new(code: Code, detail: impl Into<String>) -> Self {
        Self {
            code,
            detail: detail.into(),
        }
    }
}

/// A handler's answer: the whole reply line, or the fault to report.
pub type Reply = Result<String, Fault>;

/// One command's handler.
type Handler = fn(&Message, &mut Context) -> Reply;

/// Where a command may run.
#[derive(Clone, Copy)]
enum Gate {
    /// Anywhere: the reads, and the commands the operator recovers with (`stage`, `disarm`).
    Open,
    /// Only in a ground stage (setting, done): it moves fins, rewrites state the flight reads, or reboots.
    Ground,
}

/// The command table: name, gate, handler (src/glider cc_client `_register_*`). `push-*` is plan B's `.mpy`
/// upload and has no place on this firmware, so it answers `err badcmd` like any unknown command.
const TABLE: &[(&str, Gate, Handler)] = &[
    ("whoami", Gate::Open, identity::whoami),
    ("ping", Gate::Open, identity::ping),
    ("health", Gate::Open, identity::health),
    ("stage", Gate::Open, control::stage),
    ("arm", Gate::Open, control::arm),
    ("disarm", Gate::Open, control::disarm),
    ("report", Gate::Open, control::report),
    ("objects", Gate::Open, inspection::objects),
    ("inspect", Gate::Open, inspection::inspect),
    ("update", Gate::Open, inspection::update),
    ("stats", Gate::Open, inspection::stats),
    ("get-config", Gate::Open, config::get_config),
    ("set-config", Gate::Ground, config::set_config),
    ("reset-config", Gate::Ground, config::reset_config),
    ("probe", Gate::Ground, diagnostics::probe),
    ("verify", Gate::Open, diagnostics::verify),
    ("calibrate", Gate::Open, diagnostics::calibrate),
    ("detect", Gate::Ground, diagnostics::detect),
    ("bustune", Gate::Ground, diagnostics::bustune),
    ("log", Gate::Open, streaming::log),
    ("tlm", Gate::Open, streaming::telemetry),
    ("reboot", Gate::Ground, system::reboot),
];

/// Answer one request line; None for an empty line.
pub fn handle(line: &str, context: &mut Context) -> Option<String> {
    let message = match cc_protocol::parse(line) {
        Ok(message) => message,
        Err(error) => return Some(failure(&Fault::new(Code::BadArguments, format!("{error:?}")))),
    };
    let command = message.command.as_deref()?;
    let Some((_, gate, handler)) = TABLE.iter().find(|(name, _, _)| *name == command) else {
        return Some(failure(&Fault::new(Code::BadCommand, command)));
    };
    if let (Gate::Ground, Some(refusal)) = (gate, in_flight()) {
        return Some(failure(&Fault::new(Code::Unsafe, refusal)));
    }
    Some(handler(&message, context).unwrap_or_else(|fault| failure(&fault)))
}

/// Why an active command must not run now, or None on the ground (src/glider cc_client `_in_flight`).
pub fn in_flight() -> Option<String> {
    let stage = STATUS.stage();
    (!stage.ground()).then(|| {
        format!(
            "refused in flight -- stage is {} (ground-only: {})",
            stage.name(),
            sequencer::GROUND_NAMES
        )
    })
}

/// The `err <code> <detail>` line.
fn failure(fault: &Fault) -> String {
    cc_protocol::build("err", &[fault.code.token(), &fault.detail], &[])
}

/// How deep a CC JSON payload may nest. serde_json recurses ~240 B of stack per level, so its own 128-level limit
/// would overrun the executor's stack and reboot the board -- in flight too, `update` is open there. Real payloads
/// nest 4 deep (a launch.config zone).
const JSON_DEPTH: usize = 16;

/// Parse a CC JSON payload, refusing one nested deeper than [`JSON_DEPTH`] before serde_json recurses into it.
/// A failure answers `code` with `detail` (None: the parser's own message).
pub fn parse_json(text: &str, code: Code, detail: Option<&str>) -> Result<serde_json::Value, Fault> {
    let fault = |message: String| Fault::new(code, detail.map_or(message, str::to_string));
    let (mut depth, mut in_string, mut escaped) = (0usize, false, false);
    for byte in text.bytes() {
        match (in_string, escaped, byte) {
            (true, true, _) => escaped = false,
            (true, false, b'\\') => escaped = true,
            (true, false, b'"') => in_string = false,
            (true, false, _) => {}
            (false, _, b'"') => in_string = true,
            (false, _, b'[' | b'{') => {
                depth += 1;
                if depth > JSON_DEPTH {
                    return Err(fault(format!("JSON nested deeper than {JSON_DEPTH}")));
                }
            }
            (false, _, b']' | b'}') => depth = depth.saturating_sub(1),
            _ => {}
        }
    }
    serde_json::from_str(text).map_err(|error| fault(error.to_string()))
}

/// `ok <json>`: the usual successful reply.
pub fn ok_json(value: &impl serde::Serialize) -> Reply {
    serde_json::to_string(value)
        .map(|json| cc_protocol::build("ok", &[&json], &[]))
        .map_err(|error| Fault::new(Code::Internal, error.to_string()))
}

/// Why this board is not on the ground under automatic sequencing, or None when it is (src/glider cc_client
/// `_stage_problem`): a stage left forced or held on the pad means the flight's stage record will be wrong.
pub fn stage_problem() -> Option<String> {
    let stage = STATUS.stage();
    if STATUS.manual() {
        return Some(format!(
            "stage held at {} by an operator command -- `stage setting` to return to the ground",
            stage.name()
        ));
    }
    (stage != Stage::Setting).then(|| {
        format!(
            "stage is {}, not setting -- `stage setting`, then power-cycle before flight",
            stage.name()
        )
    })
}

/// One line to run, and where its reply goes.
struct Request {
    /// The request line, trimmed.
    line: String,
    /// The reply (None: no reply for this line).
    reply: SyncSender<Option<String>>,
}

/// The executor's stack: replies are JSON documents (get-config is ~10 KB), built on the heap.
const STACK_SIZE: usize = 16384;
/// Requests waiting for the executor: the hub's lockstep exchange plus the console; a full queue blocks the
/// caller, it never drops a line.
const QUEUE_DEPTH: usize = 4;

/// The handle callers post lines through; clone one per caller.
#[derive(Clone)]
pub struct Executor {
    /// The executor thread's queue.
    queue: SyncSender<Request>,
}

impl Executor {
    /// Spawn the executor thread (core 0, priority 5, beside the link and the console) owning `context`.
    pub fn start(context: Context) -> anyhow::Result<Self> {
        let (queue, requests) = sync_channel(QUEUE_DEPTH);
        ThreadSpawnConfiguration {
            name: Some(c"cc"),
            stack_size: STACK_SIZE,
            priority: 5,
            pin_to_core: Some(Core::Core0),
            ..Default::default()
        }
        .set()?;
        let spawned = std::thread::Builder::new()
            .stack_size(STACK_SIZE)
            .spawn(move || serve(requests, context));
        ThreadSpawnConfiguration::default().set()?;
        spawned?;
        Ok(Self { queue })
    }

    /// Run one line and wait for its reply; None for a line with no reply.
    pub fn ask(&self, line: &str) -> Option<String> {
        let (reply, answer) = sync_channel(1);
        self.queue
            .send(Request {
                line: line.to_string(),
                reply,
            })
            .ok()?;
        answer.recv().ok()?
    }
}

/// The executor thread: one line at a time, forever.
fn serve(requests: Receiver<Request>, mut context: Context) {
    for request in requests {
        let reply = handle(&request.line, &mut context);
        let _ = request.reply.send(reply); // the caller may have gone (a dropped link); nothing to do
    }
}
