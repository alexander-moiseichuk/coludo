//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bus owners (doc/specs/avionics-port.md §4.2, M5 design decisions): one thread per I2C port, on core 1, that
//! creates its port (so the port's interrupt runs on core 1) and is the only user of it from then on. At boot the
//! owner lends the port to main, which scans it for the layout and sets its parts up in config order (plan B's
//! `Controller.setup`, nothing else running), then hands it back ([`start`]); from then the owner serves its mailbox
//! ([`ask`]): the commands that need the bus (`i2c touch`, `i2c general-call`, `detect`'s scan, `probe`, `bustune`)
//! run on the owner, never beside it, over the parts that came up on its port (crate::devices). M6 adds the owner's
//! read table to the same thread. Every request is a change, run only in SETTING (a barometer's re-zero at DONE too):
//! the owner checks the stage again as it serves one, since the executor checked its gate once and a probe sweep then
//! takes seconds, so a launch detected meanwhile never runs a probe in flight.
//!
//! The stacks are internal: the bus threads are flight-path threads (§4.7, internal RAM budget), and an internal
//! stack also keeps the thread's allocations internal (crate::allocator's rule).

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicPtr, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, sync_channel};
use std::sync::{Mutex, OnceLock, PoisonError};
use std::time::Duration;

use controller::Table;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::sys;
use firmware::buses::i2c::{I2cError, I2cPort, PortFigures};
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};
use layout::Addresses;
use sequencer::ChangeRefused;

use crate::devices::Topology;
use crate::sensors::Member;
use firmware::threads::{self, Spec, Stack};
use firmware::time::uptime_us;
use parts::{Calibrated, Event, Run};

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

/// At DONE the beacon's sources run this often, µs (M11b): the barometers, the laser and the INA226.
const DONE_PERIOD_US: u64 = 1_000_000;

/// How many requests an owner serves on one wake: the rest wait for the next, so a burst never delays a tick's work.
const REQUESTS_PER_WAKE: usize = 2;

/// What a command asks an owner to do.
pub enum Request {
    /// Start the port's failure run from zero, then read one register of `address` `times` times (plan B
    /// test_i2cbus.py:43-77): with an absent address, the 4th failure in a row cures a wedge.
    Touch { address: u8, register: u8, times: u8 },
    /// Read `length` bytes from `register` of `address`, `times` times back to back (`i2c read`: a part's register
    /// facts, M9). Answered raw; the executor formats them.
    Read {
        address: u8,
        register: u8,
        length: u8,
        times: u8,
    },
    /// List what acknowledges on the port (the `detect` command's scan).
    Scan,
    /// Probe the port's parts (plan B's device `probe`): the one named, or every one.
    Probe { name: Option<String> },
    /// Run the port at `clock_hz` (a bustune rung, never saved), then probe every part.
    Bustune { clock_hz: u32 },
    /// Send the general call's software reset, let the parts settle, re-arm them unless `rearm` is false, then probe
    /// every part (`i2c general-call`).
    GeneralCall { rearm: bool },
    /// Apply an operator property change to the part `name` (CC `update`).
    Update {
        name: String,
        properties: serde_json::Value,
    },
    /// Run the part `name`'s calibration (CC `calibrate <name>`).
    Calibrate { name: String },
    /// Put the calibration `blob` the executor persisted in force on the part `name` (D9 of the M9 plan).
    ApplyCalibration { name: String, blob: Vec<u8> },
}

/// A part's probe verdict: None when healthy, else why (plan B's `probe()`).
pub type Verdict = (&'static str, Option<String>);

/// An owner's answer.
pub enum Answer {
    /// Each read's verdict, and the port's counters after them.
    Touch { results: Vec<String>, figures: PortFigures },
    /// Each read's bytes or its fault, and how long the reads took together, in microseconds.
    Read {
        reads: Vec<Result<Vec<u8>, I2cError>>,
        elapsed_us: u64,
    },
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
    /// None when no part of that name is on this port; else the changed properties, or the refusal.
    Updated(Option<Result<Vec<&'static str>, String>>),
    /// None when no part of that name is on this port; else its calibrate answer: plan B's (None success, else
    /// why), or a calibration to persist before it is applied.
    Calibrated(Option<Calibrated>),
    /// Not served in this stage ([`refusal`]): a value, so the owner makes no text off the pad. [`ask`] answers its
    /// text as its error.
    Refused(ChangeRefused),
}

/// The calibrations the parts wait for, by name: what `health.calibration` and `calibrate` list (plan B
/// `Inspector.calibration_all()`). The owners keep it current; the executor reads it.
static CALIBRATIONS: Mutex<BTreeMap<&'static str, &'static str>> = Mutex::new(BTreeMap::new());

/// Records `member`'s pending calibration, or that none is pending.
fn note_calibration(member: &Member) {
    let mut calibrations = CALIBRATIONS.lock().unwrap_or_else(PoisonError::into_inner);
    match member.part.calibration() {
        "" => calibrations.remove(member.name),
        pending => calibrations.insert(member.name, pending),
    };
}

/// Every part's pending calibration, by name, sorted: empty when nothing is pending. The magnetometer's and the
/// BNO055's are worded here, on the executor, from their published state (crate::sensors::owed_calibrations).
pub fn calibrations() -> Vec<(&'static str, String)> {
    let mut calibrations: BTreeMap<&'static str, String> = CALIBRATIONS
        .lock()
        .unwrap_or_else(PoisonError::into_inner)
        .iter()
        .map(|(name, text)| (*name, (*text).to_string()))
        .collect();
    calibrations.extend(crate::sensors::owed_calibrations());
    calibrations.into_iter().collect()
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
    threads::spawn(
        Spec {
            name: NAMES[index],
            stack_size: STACK_SIZE,
            stack: Stack::Internal,
            priority: PRIORITIES[index],
            core: Core::Core1,
        },
        move || own(id, sda, scl, clock_hz, lend, resumed, requests),
    )?;
    let port = loaned
        .recv()
        .map_err(|_| anyhow::anyhow!("i2c:{id} :: the owner ended before lending its port"))??;
    Ok((Parked { resume, mailbox, id }, port))
}

/// Hands the port back to its owner with the parts that came up on it (crate::devices), each run with its `table`
/// entry, whose health the run writes; the owner serves its mailbox from then on.
///
/// Errors: the owner is gone.
pub fn start(
    parked: Parked,
    port: I2cPort,
    mut members: Vec<Member>,
    topology: &'static Topology,
    table: &'static Table<'static>,
) -> anyhow::Result<()> {
    // by name, the order plan B's Inspector walks them for probes and re-arms
    members.sort_by(|left, right| left.name.cmp(right.name));
    for member in &mut members {
        member.run = Run::new(member, topology, table.find(member.name), &crate::sensors::PLATFORM);
        note_calibration(member);
    }
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
    // published before the port is created, so the control thread finds it (main starts the control thread right
    // after this owner's start); withdrawn on every way out, so the tick never notifies a task that ended
    // SAFETY: a plain read of this task's own handle.
    TASKS[usize::from(id)].store(unsafe { sys::xTaskGetCurrentTaskHandle() }.cast(), Ordering::Release);
    let withdraw = || TASKS[usize::from(id)].store(core::ptr::null_mut(), Ordering::Release);
    let port = match I2cPort::create(id, sda, scl, clock_hz) {
        Ok(port) => port,
        Err(error) => {
            withdraw();
            let _ = lend.send(Err(error));
            return;
        }
    };
    log::info!("i2c:{id} :: bus created on core 1 (SDA {sda}, SCL {scl}, {clock_hz} Hz, internal pull-ups on)");
    if lend.send(Ok(port)).is_err() {
        withdraw();
        return;
    }
    // the parts live as long as their owner: a ranger's XSHUT pin is released when its part is dropped
    let Ok((mut port, mut members)) = resumed.recv() else {
        withdraw();
        return;
    };
    // the owner's own producer ring: its parts' rows and log lines never contend with another thread's
    let ring = [crate::recorder::Producer::I2c0, crate::recorder::Producer::I2c1][usize::from(id)];
    let _attachment = crate::recorder::get().and_then(|recorder| recorder.attach(ring as usize).ok());
    let mut delay = Delay::new_default();
    let mut next_done_us = 0;
    loop {
        // a control tick (crate::control) or a request ([`ask`]) wakes the owner; the work is due by time, so a
        // request's wake costs only the check
        // SAFETY: this task's own notification value, index 0, cleared on take.
        unsafe { sys::ulTaskGenericNotifyTake(0, 1, IDLE_WAIT_MS) };
        for _ in 0..REQUESTS_PER_WAKE {
            match requests.try_recv() {
                Ok(envelope) => {
                    let answer = match refusal(&envelope.request, &members) {
                        Some(refusal) => Answer::Refused(refusal),
                        None => serve(&mut port, &mut members, &mut delay, envelope.request),
                    };
                    let _ = envelope.reply.send(answer);
                }
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => break,
            }
        }
        let now_us = uptime_us();
        // DONE (M11b): the IMU, the magnetometer and the pitot stop, the beacon's sources run once a second
        let done = crate::status::FLIGHT.load().stage == sequencer::Stage::Done;
        if done {
            if now_us < next_done_us {
                continue;
            }
            next_done_us = now_us + DONE_PERIOD_US;
        }
        run(&mut port, &mut members, &mut delay, now_us, done);
    }
}

/// The parts' run loops on one wake (§4.3): each part with a run, due by time, the IMU first (D13 of the M9 plan:
/// the backup IMU's reads at the lowest latency); a part's recovery re-arms the port's other parts, logged as plan B's
/// icp10111 logs it and recorded once. `done` (DONE, M11b) skips the IMU, the magnetometer and the pitot.
fn run(port: &mut I2cPort, members: &mut [Member], delay: &mut Delay, now_us: u64, done: bool) {
    // two passes, the IMU's then the rest: no allocation on the wake
    for imu in [true, false] {
        for index in 0..members.len() {
            let member = &mut members[index];
            if member.part.first() != imu {
                continue;
            }
            let Some(run) = member.run.as_mut() else {
                continue;
            };
            if done && run.stops_at_done() {
                continue;
            }
            if let Some(Event::Rearm) = member.part.tick(member.name, run, port, delay, now_us) {
                let name = member.name;
                let _ = port.take_general_call();
                let rearmed = rearm(port, members, name, delay);
                log::warn!(
                    "{name} :: read recovery: general-call reset + peer rearm after {} failures",
                    drivers::icp10111::RECOVER_AFTER
                );
                record(port.id(), name, 1, &rearmed);
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

/// Why the owner must not serve `request` in the flight word's stage now, None when it may: every request is a
/// change, run only in SETTING, and an update or a calibrate of a part that re-zeroes a barometer at DONE too
/// (sequencer::Stage::change_refused). One that names a part this port does not hold is served whatever the stage:
/// it changes nothing here, and its answer says so, so the part's own owner decides.
fn refusal(request: &Request, members: &[Member]) -> Option<ChangeRefused> {
    let rezero = match request {
        Request::Update { name, .. } | Request::Calibrate { name } | Request::ApplyCalibration { name, .. } => {
            members.iter().find(|member| member.name == name)?.part.rezeroes()
        }
        _ => false,
    };
    crate::status::FLIGHT.load().stage.change_refused(rezero)
}

/// One request, on the owner.
fn serve(port: &mut I2cPort, members: &mut [Member], delay: &mut Delay, request: Request) -> Answer {
    match request {
        Request::Touch {
            address,
            register,
            times,
        } => touch(port, address, register, times),
        Request::Read {
            address,
            register,
            length,
            times,
        } => read(port, address, register, length, times),
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
        Request::Update { name, properties } => {
            Answer::Updated(members.iter_mut().find(|member| member.name == name).map(|member| {
                let now_us = uptime_us();
                let changed = member.update(&properties, now_us);
                note_calibration(member);
                changed
            }))
        }
        Request::Calibrate { name } => {
            Answer::Calibrated(members.iter_mut().find(|member| member.name == name).map(|member| {
                let result = member.calibrate(port, delay);
                note_calibration(member);
                result
            }))
        }
        Request::ApplyCalibration { name, blob } => Answer::Calibrated(
            members
                .iter_mut()
                .find(|member| member.name == name)
                .map(|member| Calibrated::Done(member.apply_calibration(&blob))),
        ),
        Request::GeneralCall { rearm } => general_call(port, members, delay, rearm),
    }
}

/// `i2c touch`: `times` one-byte reads of `register` at `address`, each `ok:<byte>`, `nak`, `busy` or a driver error,
/// the port's run of figures restarted first.
fn touch(port: &mut I2cPort, address: u8, register: u8, times: u8) -> Answer {
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

/// `i2c read`: `times` reads of `length` bytes from `register` at `address`, and the time they took.
fn read(port: &mut I2cPort, address: u8, register: u8, length: u8, times: u8) -> Answer {
    let start_us = uptime_us();
    let reads = (0..times)
        .map(|_| {
            let mut bytes = vec![0u8; usize::from(length)];
            port.write_read(address, &[register], &mut bytes).map(|()| bytes)
        })
        .collect();
    Answer::Read {
        reads,
        elapsed_us: uptime_us() - start_us,
    }
}

/// `i2c general-call`: the general call reset, the bus left to settle, the port's parts re-armed when `rearm` asks,
/// the call kept on record (log and oops) whatever came of it, then every part probed.
fn general_call(port: &mut I2cPort, members: &mut [Member], delay: &mut Delay, rearm: bool) -> Answer {
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
/// Errors: no such port, its owner not started (yet) or gone, no answer in time, or the owner refused the request in
/// this stage (the refusal's text).
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
    match answer.recv_timeout(deadline) {
        Ok(Answer::Refused(refusal)) => Err(refusal.to_string()),
        Ok(answer) => Ok(answer),
        Err(_) => Err(format!(
            "i2c port {port}'s owner did not answer in {} s",
            deadline.as_secs()
        )),
    }
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

/// The part `name`'s property change, on whichever owner holds it (CC `update`): None when no owner holds it.
///
/// Errors: the owner did not answer in time; the inner error is the part's refusal.
pub fn update(name: &str, properties: &serde_json::Value) -> Result<Option<Result<Vec<&'static str>, String>>, String> {
    for port in 0..2 {
        if task(port).is_none() || MAILBOXES[usize::from(port)].get().is_none() {
            continue;
        }
        let request = Request::Update {
            name: name.to_string(),
            properties: properties.clone(),
        };
        if let Answer::Updated(Some(result)) = ask(port, request, ANSWER_DEADLINE)? {
            return Ok(Some(result));
        }
    }
    Ok(None)
}

/// The part `name`'s calibration on its owner (CC `calibrate <name>`): None when no owner holds such a part, else
/// plan B's answer (None success, else why). A calibration computed but not in force (the BMM350's) is handed to
/// `persist` (NVS, on this executor thread) and only then applied on the owner (D9 of the M9 plan: plan B's order); a
/// calibration that would not persist is not applied.
///
/// Errors: the owner did not answer in time.
pub fn calibrate(
    name: &str,
    persist: impl Fn(&'static str, &[u8]) -> Result<(), String>,
) -> Result<Option<Option<String>>, String> {
    for port in 0..2 {
        if MAILBOXES[usize::from(port)].get().is_none() {
            continue;
        }
        let request = Request::Calibrate { name: name.to_string() };
        let Answer::Calibrated(Some(answer)) = ask(port, request, ANSWER_DEADLINE)? else {
            continue;
        };
        let (key, blob) = match answer {
            Calibrated::Done(result) => return Ok(Some(result)),
            Calibrated::Persist { key, blob } => (key, blob),
        };
        if let Err(error) = persist(key, &blob) {
            return Ok(Some(Some(format!("calibration computed but not persisted: {error}"))));
        }
        let request = Request::ApplyCalibration {
            name: name.to_string(),
            blob,
        };
        return match ask(port, request, ANSWER_DEADLINE)? {
            Answer::Calibrated(Some(Calibrated::Done(result))) => Ok(Some(result)),
            _ => Err(format!("i2c port {port}'s owner answered another request")),
        };
    }
    Ok(None)
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

/// The owners' console commands (the Glider Command Language): `i2c general-call`, `i2c read` (a part's register
/// facts), and `i2c touch`, which proves the wedge rule on the board. All three hold the owner off its parts while
/// they run, so they run only in SETTING while disarmed.
pub fn register_commands() {
    gcl::register(vec![
        general_call_command(),
        read_command(),
        Command {
            owner: "buses",
            words: "i2c touch",
            about: "zero a port's failure run, then read one register of an address n times: an absent address's 4th \
                failure in a row cures a wedge (plan B's rule); answers each read and the port's counters",
            gate: Gate::Disarmed,
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
                    Ok(_) => cc_protocol::build("err", &["internal", "the owner answered another request"], &[]),
                    Err(error) => cc_protocol::build("err", &["internal", &error], &[]),
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
        gate: Gate::Disarmed,
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
                Ok(_) => cc_protocol::build("err", &["internal", "the owner answered another request"], &[]),
                Err(error) => cc_protocol::build("err", &["internal", &error], &[]),
            }
        }),
    }
}

/// GCL `i2c read port=<0..1> address=<0x08..0x7f> register=<0..0xff> length=<1..32> times=<1..64>`: reads `length`
/// bytes from a register `times` times back to back, for a part's register facts (M9: whether the BMI323's STATUS
/// carries data-ready bits and what clears them; whether the BMP581 survives a general call). No write: YAGNI. Answers
/// each read as hex (or its fault) and how long the reads took together.
fn read_command() -> Command {
    Command {
        owner: "buses",
        words: "i2c read",
        about: "read bytes from a register of an address, back to back: answers each read as hex (or nak/busy/error) and \
                the reads' total microseconds",
        gate: Gate::Disarmed,
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
                default: None,
                about: "the first register read",
            },
            Parameter {
                name: "length",
                kind: Kind::Integer { low: 1, high: 32 },
                default: Some("1"),
                about: "how many bytes each read takes",
            },
            Parameter {
                name: "times",
                kind: Kind::Integer { low: 1, high: 64 },
                default: Some("1"),
                about: "how many reads, back to back",
            },
        ],
        bare_words: &[],
        run: Box::new(|arguments| {
            let port = arguments.integer("port") as u8;
            let request = Request::Read {
                address: arguments.integer("address") as u8,
                register: arguments.integer("register") as u8,
                length: arguments.integer("length") as u8,
                times: arguments.integer("times") as u8,
            };
            match ask(port, request, ANSWER_DEADLINE) {
                Ok(Answer::Read { reads, elapsed_us }) => {
                    let reads: Vec<String> = reads
                        .iter()
                        .map(|read| match read {
                            Ok(bytes) => bytes.iter().map(|byte| format!("{byte:02x}")).collect(),
                            Err(I2cError::NotAcknowledged) => "nak".to_string(),
                            Err(I2cError::BusBusy) => "busy".to_string(),
                            Err(I2cError::Driver(code)) => format!("error:{code:#x}"),
                        })
                        .collect();
                    let reply = serde_json::json!({"port": port, "reads": reads, "elapsed_us": elapsed_us});
                    cc_protocol::build("ok", &[&reply.to_string()], &[])
                }
                Ok(_) => cc_protocol::build("err", &["internal", "the owner answered another request"], &[]),
                Err(error) => cc_protocol::build("err", &["internal", &error], &[]),
            }
        }),
    }
}
