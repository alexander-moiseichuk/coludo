//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bus owners (doc/specs/avionics-port.md §4.2, M5 design decisions): one thread per I2C port, on core 1, that
//! creates its port (so the port's interrupt runs on core 1) and is the only user of it from then on. At boot the
//! owner lends the port to main, which scans it for the layout and sets its parts up in config order (plan B's
//! `Controller.setup`, nothing else running), then hands it back ([`start`]); from then the owner serves its mailbox
//! ([`ask`]): the commands that need the bus (`i2c touch`, `i2c general-call`, `detect`'s scan, `probe`, `bustune`)
//! run on the owner, never beside it, over the parts that came up on its port (crate::devices). M6 adds the owner's
//! read table to the same thread.
//!
//! The stacks are internal: the bus threads are flight-path threads (§4.7, internal RAM budget), and an internal
//! stack also keeps the thread's allocations internal (crate::allocator's rule).

use std::sync::OnceLock;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};
use std::time::Duration;

use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::sys;
use firmware::buses::i2c::{I2cError, I2cPort, PortFigures};
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use layout::Addresses;

use crate::devices::Member;

/// The owners' priorities, by port: the bus owners sit below the control thread (§4.2).
const PRIORITIES: [u8; 2] = [18, 17];

/// The owners' thread names, by port.
const NAMES: [&std::ffi::CStr; 2] = [c"i2c0", c"i2c1"];

/// An owner's stack: a transfer, a driver's register work and a reply text. Measured peak 3.4 KB (M5, `heap stacks`
/// after probe all, the bustune ladders and general calls with every part re-armed, a VL53's re-arm being its whole
/// setup), plus a margin; M6 measures again once the read table runs here.
const STACK_SIZE: usize = 5 << 10;

/// How many requests may wait for an owner.
const MAILBOX_DEPTH: usize = 2;

/// How long [`ask`] waits for an owner's answer: a touch is a few transfers of at most 50 ms each, a probe of every
/// part a conversion or two each.
pub const ANSWER_DEADLINE: Duration = Duration::from_secs(8);

/// How long the parts settle after a general call before anything talks to them (plan B icp10111.py:285-289).
const GENERAL_CALL_SETTLE: Duration = Duration::from_millis(30);

/// Each started owner's mailbox, by port.
static MAILBOXES: [OnceLock<SyncSender<Envelope>>; 2] = [OnceLock::new(), OnceLock::new()];

/// Each owner's FreeRTOS task, by port, published as the thread starts: the control tick and [`ask`] notify it.
static TASKS: [AtomicPtr<core::ffi::c_void>; 2] = [
    AtomicPtr::new(core::ptr::null_mut()),
    AtomicPtr::new(core::ptr::null_mut()),
];

/// The longest an owner sleeps with nothing to wake it, in FreeRTOS ticks (milliseconds here: CONFIG_FREERTOS_HZ is
/// 1000): before the control tick runs, and should a notify be lost.
const IDLE_WAIT_MS: u32 = 20;

/// How many requests an owner serves on one wake: the rest wait for the next, so a burst never delays a tick's work.
const REQUESTS_PER_WAKE: usize = 2;

/// What a command asks an owner to do.
pub enum Request {
    /// Start the port's failure run from zero, then read one register of `address` `times` times (plan B
    /// test_i2cbus.py:43-77): with an absent address, the 4th failure in a row cures a wedge.
    Touch { address: u8, register: u8, times: u8 },
    /// List what acknowledges on the port (the `detect` command's scan).
    Scan,
    /// Probe the port's parts (plan B's device `probe`): the one named, or every one.
    Probe { name: Option<String> },
    /// Run the port at `clock_hz` (a bustune rung, never saved), then probe every part.
    Bustune { clock_hz: u32 },
    /// Send the general call's software reset, let the parts settle, re-arm them unless `rearm` is false, then probe
    /// every part (`i2c general-call`).
    GeneralCall { rearm: bool },
}

/// A part's probe verdict: None when healthy, else why (plan B's `probe()`).
pub type Verdict = (&'static str, Option<String>);

/// An owner's answer.
pub enum Answer {
    /// Each read's verdict, and the port's counters after them.
    Touch { results: Vec<String>, figures: PortFigures },
    /// The addresses that acknowledged, or why the scan failed.
    Scan(Result<Addresses, I2cError>),
    /// The probed parts' verdicts, in the owner's order.
    Probed(Vec<Verdict>),
    /// The rung's verdicts, or why the port would not run at that speed; and the port's counters.
    Tuned {
        verdicts: Result<Vec<Verdict>, I2cError>,
        figures: PortFigures,
    },
    /// Whether the general call was acknowledged, each re-arm, each probe, and the port's counters.
    GeneralCalled {
        acknowledged: bool,
        rearmed: Vec<(&'static str, Result<(), String>)>,
        probed: Vec<Verdict>,
        figures: PortFigures,
    },
}

/// A request and where its answer goes.
struct Envelope {
    request: Request,
    reply: SyncSender<Answer>,
}

/// A spawned owner waiting for its port back (from [`spawn`] to [`start`]).
pub struct Parked {
    /// Where the port and its parts go back to the owner.
    resume: SyncSender<(I2cPort, Vec<Member>)>,
    /// The mailbox the owner will serve, published at [`start`].
    mailbox: SyncSender<Envelope>,
    /// The port number.
    id: u8,
}

/// Spawns port `id`'s owner (core 1), which creates the port on pins `sda`/`scl` at `clock_hz` and lends it to the
/// caller until [`start`].
///
/// Errors: the thread would not spawn, or the port would not be created (its owner then ends).
pub fn spawn(id: u8, sda: u8, scl: u8, clock_hz: u32) -> anyhow::Result<(Parked, I2cPort)> {
    let index = id as usize;
    anyhow::ensure!(index < NAMES.len(), "no i2c port {id}");
    let (lend, loaned) = sync_channel::<anyhow::Result<I2cPort>>(1);
    let (resume, resumed) = sync_channel::<(I2cPort, Vec<Member>)>(1);
    let (mailbox, requests) = sync_channel::<Envelope>(MAILBOX_DEPTH);
    ThreadSpawnConfiguration {
        name: Some(NAMES[index]),
        stack_size: STACK_SIZE,
        priority: PRIORITIES[index],
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }
    .set()?;
    let spawned = std::thread::Builder::new()
        .stack_size(STACK_SIZE)
        .spawn(move || own(id, sda, scl, clock_hz, lend, resumed, requests));
    ThreadSpawnConfiguration::default().set()?;
    spawned?;
    let port = loaned
        .recv()
        .map_err(|_| anyhow::anyhow!("i2c:{id} :: the owner ended before lending its port"))??;
    Ok((Parked { resume, mailbox, id }, port))
}

/// Hands the port back to its owner with the parts that came up on it (crate::devices), and the owner serves its
/// mailbox from then on.
///
/// Errors: the owner is gone.
pub fn start(parked: Parked, port: I2cPort, mut members: Vec<Member>) -> anyhow::Result<()> {
    // by name, the order plan B's Inspector walks them for probes and re-arms
    members.sort_by(|left, right| left.name.cmp(right.name));
    parked
        .resume
        .send((port, members))
        .map_err(|_| anyhow::anyhow!("i2c:{} :: the owner is gone", parked.id))?;
    let _ = MAILBOXES[parked.id as usize].set(parked.mailbox);
    Ok(())
}

/// The owner thread: creates the port, lends it until it comes back, then serves requests forever.
fn own(
    id: u8,
    sda: u8,
    scl: u8,
    clock_hz: u32,
    lend: SyncSender<anyhow::Result<I2cPort>>,
    resumed: Receiver<(I2cPort, Vec<Member>)>,
    requests: Receiver<Envelope>,
) {
    // SAFETY: a plain read of this task's own handle; the task lives for the program.
    TASKS[usize::from(id)].store(unsafe { sys::xTaskGetCurrentTaskHandle() }.cast(), Ordering::Release);
    let port = match I2cPort::create(id, sda, scl, clock_hz) {
        Ok(port) => port,
        Err(error) => {
            let _ = lend.send(Err(error));
            return;
        }
    };
    log::info!("i2c:{id} :: bus created on core 1 (SDA {sda}, SCL {scl}, {clock_hz} Hz, internal pull-ups on)");
    if lend.send(Ok(port)).is_err() {
        return;
    }
    // the parts live as long as their owner: a ranger's XSHUT pin is released when its part is dropped
    let Ok((mut port, mut members)) = resumed.recv() else {
        return;
    };
    // the owner's own producer ring: its parts' rows and log lines never contend with another thread's
    let ring = [crate::recorder::Producer::I2c0, crate::recorder::Producer::I2c1][usize::from(id)];
    let _attachment = crate::recorder::get().and_then(|recorder| recorder.attach(ring as usize).ok());
    let mut delay = Delay::new_default();
    loop {
        // a control tick (crate::control) or a request ([`ask`]) wakes the owner; the work is due by time, so a
        // request's wake costs only the check
        // SAFETY: this task's own notification value, index 0, cleared on take.
        unsafe { sys::ulTaskGenericNotifyTake(0, 1, IDLE_WAIT_MS) };
        for _ in 0..REQUESTS_PER_WAKE {
            match requests.try_recv() {
                Ok(envelope) => {
                    let answer = serve(&mut port, &mut members, &mut delay, envelope.request);
                    let _ = envelope.reply.send(answer);
                }
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => break,
            }
        }
    }
}

/// Port `id`'s owner task, once its thread started: the control tick wakes it.
pub fn task(id: u8) -> Option<sys::TaskHandle_t> {
    let task = TASKS.get(usize::from(id))?.load(Ordering::Acquire);
    (!task.is_null()).then_some(task.cast())
}

/// Probes `members`, in their order.
fn probe(port: &mut I2cPort, members: &mut [Member], delay: &mut Delay) -> Vec<Verdict> {
    members
        .iter_mut()
        .map(|member| (member.name, member.part.probe(member.name, port, delay)))
        .collect()
}

/// One request, on the owner.
fn serve(port: &mut I2cPort, members: &mut [Member], delay: &mut Delay, request: Request) -> Answer {
    match request {
        Request::Touch {
            address,
            register,
            times,
        } => {
            port.restart_run();
            let results = (0..times)
                .map(|_| {
                    let mut byte = [0u8];
                    match port.write_read(address, &[register], &mut byte) {
                        Ok(()) => format!("ok:{:#04x}", byte[0]),
                        Err(I2cError::NotAcknowledged) => "nak".to_string(),
                        Err(I2cError::BusBusy) => "busy".to_string(),
                        Err(I2cError::Driver(code)) => format!("error:{code:#x}"),
                    }
                })
                .collect();
            Answer::Touch {
                results,
                figures: port.figures(),
            }
        }
        Request::Scan => Answer::Scan(port.scan()),
        Request::Probe { name } => {
            let named = |member: &&mut Member| name.as_deref().is_none_or(|name| member.name == name);
            Answer::Probed(
                members
                    .iter_mut()
                    .filter(named)
                    .map(|member| (member.name, member.part.probe(member.name, port, delay)))
                    .collect(),
            )
        }
        Request::Bustune { clock_hz } => {
            let verdicts = port.retune(clock_hz).map(|()| probe(port, members, delay));
            Answer::Tuned {
                verdicts,
                figures: port.figures(),
            }
        }
        Request::GeneralCall { rearm } => {
            let acknowledged = port.general_call_reset().is_ok();
            let _ = port.take_general_call();
            std::thread::sleep(GENERAL_CALL_SETTLE);
            let rearmed = if rearm {
                self::rearm(port, members, "console", delay)
            } else {
                Vec::new()
            };
            // the operator's call is always on record, re-armed or not: it is what the measurement is about
            let text = format!(
                "i2c:{} general call by console (acknowledged: {acknowledged}): {}",
                port.id(),
                if rearm {
                    format!("re-armed {}", walk(&rearmed))
                } else {
                    "no re-arm".to_string()
                }
            );
            log::warn!("{text}");
            crate::panics_logger::oops("i2c-general-call", &text);
            Answer::GeneralCalled {
                acknowledged,
                rearmed,
                probed: probe(port, members, delay),
                figures: port.figures(),
            }
        }
    }
}

/// One part's re-arm after a general call: its name, and the fault when the re-arm failed.
pub type Rearmed = (&'static str, Result<(), String>);

/// After a general call on `port` (an I2C software reset of every part that honours it), re-arms each of its
/// `members` but the `sender`, in their order (by name, as plan B walks `Inspector.names()`, icp10111.py:293-329),
/// logging plan B's line for each one that fails. Returns each re-arm; the caller records the walk ([`record`]).
pub fn rearm(port: &mut I2cPort, members: &mut [Member], sender: &str, delay: &mut Delay) -> Vec<Rearmed> {
    members
        .iter_mut()
        .filter(|member| member.name != sender)
        .map(|member| {
            let outcome = member.part.rearm(port, delay);
            if let Err(error) = &outcome {
                log::warn!("{} :: rearm failed: {error}", member.name);
            }
            (member.name, outcome)
        })
        .collect()
}

/// Records general calls on port `port` by `sender`: `calls` of them, and `rearmed` (every re-arm they caused, in
/// order). One log line, and one oops record (doc/specs/avionics-port.md §4.8) when anything was re-armed: a bring-up
/// whose ICP keeps calling would otherwise fill the 16-slot panic log with records of nothing and push the real
/// panics out.
pub fn record(port: u8, sender: &str, calls: u32, rearmed: &[Rearmed]) {
    let calls = if calls == 1 {
        "general call".to_string()
    } else {
        format!("{calls} general calls")
    };
    if rearmed.is_empty() {
        log::warn!("i2c:{port} {calls} by {sender}: no other part up to re-arm");
        return;
    }
    let text = format!("i2c:{port} {calls} by {sender}: re-armed {}", walk(rearmed));
    log::warn!("{text}");
    crate::panics_logger::oops("i2c-general-call", &text);
}

/// A re-arm walk as one text: `name ok, name failed: <why>, ...`.
fn walk(rearmed: &[Rearmed]) -> String {
    let walked: Vec<String> = rearmed
        .iter()
        .map(|(name, outcome)| match outcome {
            Ok(()) => format!("{name} ok"),
            Err(error) => format!("{name} failed: {error}"),
        })
        .collect();
    walked.join(", ")
}

/// Asks port `port`'s owner to run `request`, and waits up to `deadline` for its answer.
///
/// Errors: no such port, its owner not started (yet) or gone, or no answer in time.
pub fn ask(port: u8, request: Request, deadline: Duration) -> Result<Answer, String> {
    let mailbox = MAILBOXES
        .get(port as usize)
        .and_then(OnceLock::get)
        .ok_or_else(|| format!("i2c port {port} has no owner (not defined in the board config, or not started)"))?;
    let (reply, answer) = sync_channel(1);
    mailbox
        .try_send(Envelope { request, reply })
        .map_err(|_| format!("i2c port {port}'s owner is busy or gone"))?;
    if let Some(task) = task(port) {
        // SAFETY: the owner's task lives for the program; a notify only wakes it early
        unsafe { sys::xTaskGenericNotify(task, 0, 0, sys::eNotifyAction_eIncrement, core::ptr::null_mut()) };
    }
    answer
        .recv_timeout(deadline)
        .map_err(|_| format!("i2c port {port}'s owner did not answer in {} s", deadline.as_secs()))
}

/// Port `port`'s scan, on its owner (the `detect` command): what acknowledges.
///
/// Errors: no owner, no answer in time, or the scan failed (the bus busy or broken).
pub fn scan(port: u8) -> Result<Addresses, String> {
    match ask(port, Request::Scan, ANSWER_DEADLINE)? {
        Answer::Scan(found) => found.map_err(|error| format!("i2c port {port}'s scan failed: {error}")),
        _ => Err(format!("i2c port {port}'s owner answered another request")),
    }
}

/// Port `port`'s parts probed on its owner: the one named, or every one (`probe`, `verify`, `arm`). An owner that
/// is not started (no such port in the config) has no parts: an empty list.
///
/// Errors: the owner did not answer in time, or is gone.
pub fn probe_port(port: u8, name: Option<&str>) -> Result<Vec<Verdict>, String> {
    if MAILBOXES.get(usize::from(port)).and_then(OnceLock::get).is_none() {
        return Ok(Vec::new());
    }
    let request = Request::Probe {
        name: name.map(str::to_string),
    };
    match ask(port, request, ANSWER_DEADLINE)? {
        Answer::Probed(verdicts) => Ok(verdicts),
        _ => Err(format!("i2c port {port}'s owner answered another request")),
    }
}

/// Port `port` run at `clock_hz` on its owner, and its parts probed there (`bustune`).
///
/// Errors: no owner, or no answer in time; the inner error is the port refusing the speed.
pub fn bustune(port: u8, clock_hz: u32) -> Result<Result<Vec<Verdict>, I2cError>, String> {
    match ask(port, Request::Bustune { clock_hz }, ANSWER_DEADLINE)? {
        Answer::Tuned { verdicts, figures } => {
            if figures.speed_hz != clock_hz && verdicts.is_ok() {
                // a wedge cure during the rung's probes put the configured speed back, as plan B's reclock did
                log::warn!(
                    "i2c:{port} :: bustune {clock_hz} Hz: a wedge cure restored {} Hz during the probes",
                    figures.speed_hz
                );
            }
            Ok(verdicts)
        }
        _ => Err(format!("i2c port {port}'s owner answered another request")),
    }
}

/// The owners' console commands (the Glider Command Language): `i2c general-call`, and `i2c touch`, which proves the
/// wedge rule on the board.
pub fn register_commands() {
    gcl::register(vec![
        general_call_command(),
        Command {
            owner: "buses",
            words: "i2c touch",
            about: "zero a port's failure run, then read one register of an address n times: an absent address's 4th \
                failure in a row cures a wedge (plan B's rule); answers each read and the port's counters",
            gate: Gate::Setting,
            reach: Reach::Everywhere,
            parameters: &[
                Parameter {
                    name: "port",
                    kind: Kind::Integer { low: 0, high: 1 },
                    default: None,
                    about: "the I2C port",
                },
                Parameter {
                    name: "address",
                    kind: Kind::Integer { low: 0x08, high: 0x7f },
                    default: None,
                    about: "the 7-bit address",
                },
                Parameter {
                    name: "register",
                    kind: Kind::Integer { low: 0, high: 0xff },
                    default: Some("0"),
                    about: "the register read",
                },
                Parameter {
                    name: "times",
                    kind: Kind::Integer { low: 1, high: 16 },
                    default: Some("4"),
                    about: "how many reads",
                },
            ],
            bare_words: &[],
            run: Box::new(|arguments| {
                let port = arguments.integer("port") as u8;
                let request = Request::Touch {
                    address: arguments.integer("address") as u8,
                    register: arguments.integer("register") as u8,
                    times: arguments.integer("times") as u8,
                };
                match ask(port, request, ANSWER_DEADLINE) {
                    Ok(Answer::Touch { results, figures }) => cc_protocol::build(
                        "ok",
                        &["i2c", "touch"],
                        &[
                            ("port", &port.to_string()),
                            ("results", &results.join("/")),
                            ("failures", &figures.failures.to_string()),
                            ("wedges", &figures.wedges.to_string()),
                            ("general_calls", &figures.general_calls.to_string()),
                            ("speed_hz", &figures.speed_hz.to_string()),
                            ("configured_hz", &figures.configured_hz.to_string()),
                        ],
                    ),
                    Ok(_) => cc_protocol::build("err", &["i2c", "the owner answered another request"], &[]),
                    Err(error) => cc_protocol::build("err", &["i2c", &error], &[]),
                }
            }),
        },
    ]);
}

/// GCL `i2c general-call port=<0..1> rearm=<yes|no> confirm`: plan B has no command that sends one, only the ICP's
/// own recovery does (doc/specs/avionics-port.md M5, "Forced proofs"). Measures which parts honour the call
/// (`rearm=no`) and proves the re-arm (`rearm=yes`): answers whether the call was acknowledged, each re-arm and each
/// probe after it.
fn general_call_command() -> Command {
    Command {
        owner: "buses",
        words: "i2c general-call",
        about: "send the I2C general call's software reset on a port (every part that honours it resets), re-arm the \
                port's parts unless rearm=no, then probe them all; written to the panic log as an oops",
        gate: Gate::Setting,
        reach: Reach::Everywhere,
        parameters: &[
            Parameter {
                name: "port",
                kind: Kind::Integer { low: 0, high: 1 },
                default: None,
                about: "the I2C port",
            },
            Parameter {
                name: "rearm",
                kind: Kind::Choice(&["yes", "no"]),
                default: Some("yes"),
                about: "re-arm the port's parts after the call",
            },
        ],
        bare_words: &[gcl::CONFIRM],
        run: Box::new(|arguments| {
            let port = arguments.integer("port") as u8;
            let request = Request::GeneralCall {
                rearm: arguments.text("rearm") == "yes",
            };
            match ask(port, request, ANSWER_DEADLINE) {
                Ok(Answer::GeneralCalled {
                    acknowledged,
                    rearmed,
                    probed,
                    figures,
                }) => {
                    let rearmed: serde_json::Map<String, serde_json::Value> = rearmed
                        .into_iter()
                        .map(|(name, outcome)| (name.to_string(), outcome.err().unwrap_or_else(|| "ok".into()).into()))
                        .collect();
                    let probed: serde_json::Map<String, serde_json::Value> = probed
                        .into_iter()
                        .map(|(name, verdict)| (name.to_string(), verdict.unwrap_or_else(|| "ok".into()).into()))
                        .collect();
                    let reply = serde_json::json!({
                        "port": port,
                        "acked": acknowledged,
                        "rearmed": rearmed,
                        "probed": probed,
                        "speed_hz": figures.speed_hz,
                    });
                    cc_protocol::build("ok", &[&reply.to_string()], &[])
                }
                Ok(_) => cc_protocol::build("err", &["i2c", "the owner answered another request"], &[]),
                Err(error) => cc_protocol::build("err", &["i2c", &error], &[]),
            }
        }),
    }
}
