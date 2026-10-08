//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The CC commands (doc/specs/cc-protocol.md; src/glider cc_client.py): one request line in, at most one reply line
//! out, through one table.
//!
//! Every line -- from the hub (link.rs) and from the console -- runs on one executor thread, one at a time, so a
//! handler owns the state it touches (the mission) without a lock, and two operators can never interleave half a
//! command. The envelope is plan B's: an unparsable line answers `err badargs`, an unknown command `err badcmd`,
//! a handler failure `err internal`, an empty line nothing.
//!
//! Changes run only in SETTING (the operator's rule, 2026-10-06): every command that changes the board --
//! `update`, `set-config`, `reset-config`, `probe`, `detect`, `bustune`, `reboot`, `calibrate <device>` -- answers
//! `err unsafe` in any other stage, DONE included (no reboot there: the beacon and the flash-to-SD copy run on). The
//! one exception is a barometer's re-zero, its `update` and `calibrate`, which also run at DONE: the way back from a
//! false launch without a power cycle (`stage done`, the re-zero, `stage setting`). Reads, `stage` and `disarm` stay
//! open: they are how the operator recovers. Plan B left `update` open in flight and allowed the rest at DONE
//! (doc/specs/avionics-port.md §6). The thread that serves a change checks the stage again (the bus owners, the
//! control thread's mailbox), since a probe sweep takes seconds and a launch may be detected meanwhile.

mod config;
mod control;
mod diagnostics;
mod identity;
mod inspection;
mod streaming;
mod system;

use std::cell::Cell;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::time::{Duration, Instant};

use cc_protocol::Message;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use firmware::gcl::Origin;
use mission::Mission;

use crate::board::Board;
use crate::status::FLIGHT;

/// What the handlers share: the board, the settings store, and the state the executor owns.
pub struct Context {
    /// Who this board is, and its vitals.
    pub board: Arc<Board>,
    /// The `cfg` settings store (board.config, launch.config).
    pub store: crate::config::Store,
    /// The shared `nvs` partition: the warm start's crumb, which `inspect` reads (crate::warm).
    pub nvs: esp_idf_svc::nvs::EspDefaultNvsPartition,
    /// The per-launch mission (src/glider mission.Mission), owned here: its zone and launch point are republished
    /// for the checkpoint after every line (crate::warm::publish_mission).
    pub mission: Mission,
    /// The board config this boot runs, as the layout rewrote it, how it was chosen, and its validation errors (plan
    /// B config.load, then layout.resolve); frozen for the run.
    pub config: &'static board_config::Loaded,
    /// The running config's id, hashed once at boot after the layout rewrote the document (it is immutable for the
    /// run).
    pub config_id: String,
    /// The revision this boot runs as (layout::Resolved::label): health's `layout`, detect's `applied`.
    pub layout: String,
    /// Every directory entry and how its bring-up ended (crate::devices): health's tasks, verify's devices, the
    /// failures `arm` refuses with.
    pub devices: &'static controller::Table<'static>,
    /// The databoard's quantities: what `inspect` reads a device's latest values from.
    pub topology: &'static crate::devices::Topology,
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
    /// Only in SETTING: it changes the board -- moves fins, rewrites state the flight reads, or reboots.
    Setting,
    /// Only in SETTING and disarmed: a bus diagnostic ([`diagnostic_refused`]).
    Disarmed,
    /// A change to a named device: only in SETTING, and at DONE too when it re-zeroes a barometer ([`change_refused`]).
    /// The handler checks it, since it depends on the target (and `calibrate` with no target is a read).
    Rezero,
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
    ("update", Gate::Rezero, inspection::update),
    ("stats", Gate::Open, inspection::stats),
    ("get-config", Gate::Open, config::get_config),
    ("set-config", Gate::Setting, config::set_config),
    ("reset-config", Gate::Setting, config::reset_config),
    ("probe", Gate::Setting, diagnostics::probe),
    ("verify", Gate::Open, diagnostics::verify),
    ("calibrate", Gate::Rezero, diagnostics::calibrate),
    ("detect", Gate::Setting, diagnostics::detect),
    ("bustune", Gate::Disarmed, diagnostics::bustune),
    ("log", Gate::Open, streaming::log),
    ("tlm", Gate::Open, streaming::telemetry),
    ("reboot", Gate::Setting, system::reboot),
];

/// Answer one request line -- the one point of injection for the hub and the console alike; None for an empty
/// line. A CC command answers from [`TABLE`]; any other name goes to the Glider Command Language registry
/// (firmware::gcl: `help`, and every command a component registered), under the same SETTING gate; a name neither
/// knows answers `err badcmd`.
pub fn handle(line: &str, context: &mut Context, origin: Origin) -> Option<String> {
    let message = match cc_protocol::parse(line) {
        Ok(message) => message,
        Err(error) => return Some(failure(&Fault::new(Code::BadArguments, format!("{error:?}")))),
    };
    let command = message.command.as_deref()?;
    let Some((_, gate, handler)) = TABLE.iter().find(|(name, _, _)| *name == command) else {
        let refusal = outside_setting();
        let diagnostic_refusal = diagnostic_refused();
        return Some(
            firmware::gcl::handle(line, origin, refusal.as_deref(), diagnostic_refusal.as_deref())
                .unwrap_or_else(|| failure(&Fault::new(Code::BadCommand, command))),
        );
    };
    let refused = match gate {
        Gate::Setting => outside_setting(),
        Gate::Disarmed => diagnostic_refused(),
        Gate::Open | Gate::Rezero => None,
    };
    if let Some(refusal) = refused {
        return Some(failure(&Fault::new(Code::Unsafe, refusal)));
    }
    Some(handler(&message, context).unwrap_or_else(|fault| failure(&fault)))
}

/// The CC commands' names, for the GCL `help` (firmware::gcl::set_cc_names).
pub fn names() -> Vec<&'static str> {
    TABLE.iter().map(|(name, _, _)| *name).collect()
}

/// Why a command that changes the board must not run now, or None in SETTING (plan B's `_in_flight`, narrowed).
pub fn outside_setting() -> Option<String> {
    FLIGHT
        .load()
        .stage
        .change_refused(false)
        .map(|refusal| refusal.to_string())
}

/// Why a bus diagnostic (`bustune`, `i2c touch`, `i2c read`, `i2c general-call`, `imu mute`) must not run now, or
/// None in SETTING while disarmed: it holds a bus owner off its parts, so never in the last seconds on the pad (M9; plan
/// B gates only on the stage).
pub fn diagnostic_refused() -> Option<String> {
    outside_setting().or_else(|| {
        FLIGHT
            .load()
            .armed
            .then(|| "refused -- armed (bus diagnostics only while disarmed)".to_string())
    })
}

/// Why a change to the device `target` must not run now, or None when it may: in SETTING, and at DONE when the
/// target is a barometer that re-zeroes (crate::sensors::rezeroes), its `update` and `calibrate`.
pub fn change_refused(context: &Context, target: &str) -> Option<String> {
    let rezero = context
        .devices
        .find(target)
        .is_some_and(|entry| crate::sensors::rezeroes(entry.implementation));
    FLIGHT
        .load()
        .stage
        .change_refused(rezero)
        .map(|refusal| refusal.to_string())
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

/// One line to run, who sent it, when it arrived, and where its reply goes.
struct Request {
    /// The request line, trimmed.
    line: String,
    /// The console or the hub: a console-only command refuses the hub.
    origin: Origin,
    /// When it arrived, stamped before the queue send: what it waits behind counts too (§4.4).
    arrived: Instant,
    /// The reply (None: no reply for this line).
    reply: SyncSender<Option<String>>,
}

std::thread_local! {
    /// When the line the executor runs now arrived; None before its first. The executor's own: set before each line.
    static ARRIVED: Cell<Option<Instant>> = const { Cell::new(None) };
}

/// How long the line the executor runs now has waited since it arrived: in the queue and in its own handler so far
/// (`arm` refuses one that waited too long, controller::stale_arm). Zero off the executor.
pub fn waited() -> Duration {
    ARRIVED.with(|arrived| arrived.get().map_or(Duration::ZERO, |arrived| arrived.elapsed()))
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
            // stack in PSRAM (avionics-port.md §4.7, internal RAM budget): a slow core-0 thread that never calls
            // esp_mmu_map/unmap, the one runtime path that turns the cache off (no partition mmap; M15's OTA runs
            // elsewhere)
            stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
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

    /// Run one line from `origin` and wait for its reply; None for a line with no reply. The line is stamped as it
    /// arrives, before it queues behind the lines ahead of it.
    pub fn ask(&self, line: &str, origin: Origin) -> Option<String> {
        let (reply, answer) = sync_channel(1);
        self.queue
            .send(Request {
                line: line.to_string(),
                origin,
                arrived: Instant::now(),
                reply,
            })
            .ok()?;
        answer.recv().ok()?
    }
}

/// The executor thread: one line at a time, forever, the mission republished for the checkpoint after each. It
/// records through its own recorder ring: the session.csv row of a clock set, and every command's log lines.
fn serve(requests: Receiver<Request>, mut context: Context) {
    let _attachment = crate::recorder::get().and_then(|recorder| {
        recorder
            .attach(crate::recorder::Producer::Executor as usize)
            .inspect_err(|error| log::warn!("cc :: the executor shares the misc ring: {error:?}"))
            .ok()
    });
    for request in requests {
        ARRIVED.with(|arrived| arrived.set(Some(request.arrived)));
        let reply = handle(&request.line, &mut context, request.origin);
        // whatever the line changed of the mission, before the checkpoint's next freeze can read it
        crate::warm::publish_mission(&context.mission);
        let _ = request.reply.send(reply); // the caller may have gone (a dropped link); nothing to do
    }
}
