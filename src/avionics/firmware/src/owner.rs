//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The bus owners (doc/specs/avionics-port.md §4.2, M5 design decisions): one thread per I2C port, on core 1, that
//! creates its port (so the port's interrupt runs on core 1) and is the only user of it from then on. At boot the
//! owner lends the port to main, which scans it for the layout and sets its parts up in config order (plan B's
//! `Controller.setup`, nothing else running), then hands it back ([`start`]); from then the owner serves its mailbox
//! ([`ask`]): the commands that need the bus (`i2c touch` now; probes, bustune and the general call later) run on
//! the owner, never beside it. M6 adds the owner's read table to the same thread.
//!
//! The stacks are internal: the bus threads are flight-path threads (§4.7, internal RAM budget), and an internal
//! stack also keeps the thread's allocations internal (crate::allocator's rule).

use std::sync::OnceLock;
use std::sync::mpsc::{Receiver, SyncSender, sync_channel};
use std::time::Duration;

use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use firmware::buses::i2c::{I2cError, I2cPort, PortFigures};
use firmware::gcl::{self, Command, Gate, Kind, Parameter, Reach};

/// The owners' priorities, by port: the bus owners sit below the control thread (§4.2).
const PRIORITIES: [u8; 2] = [18, 17];

/// The owners' thread names, by port.
const NAMES: [&std::ffi::CStr; 2] = [c"i2c0", c"i2c1"];

/// An owner's stack: a transfer, a driver's register work and a reply text; trimmed once `heap stacks` has measured
/// the busiest request.
const STACK_SIZE: usize = 6 << 10;

/// How many requests may wait for an owner.
const MAILBOX_DEPTH: usize = 2;

/// How long [`ask`] waits for an owner's answer: a touch is a few transfers of at most 50 ms each.
pub const ANSWER_DEADLINE: Duration = Duration::from_secs(8);

/// Each started owner's mailbox, by port.
static MAILBOXES: [OnceLock<SyncSender<Envelope>>; 2] = [OnceLock::new(), OnceLock::new()];

/// What a command asks an owner to do.
pub enum Request {
    /// Start the port's failure run from zero, then read one register of `address` `times` times (plan B
    /// test_i2cbus.py:43-77): with an absent address, the 4th failure in a row cures a wedge.
    Touch { address: u8, register: u8, times: u8 },
}

/// An owner's answer.
pub enum Answer {
    /// Each read's verdict, and the port's counters after them.
    Touch { results: Vec<String>, figures: PortFigures },
}

/// A request and where its answer goes.
struct Envelope {
    request: Request,
    reply: SyncSender<Answer>,
}

/// A spawned owner waiting for its port back (from [`spawn`] to [`start`]).
pub struct Parked {
    /// Where the port goes back to its owner.
    resume: SyncSender<I2cPort>,
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
    let (resume, resumed) = sync_channel::<I2cPort>(1);
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

/// Hands the port back to its owner, which serves its mailbox from then on.
///
/// Errors: the owner is gone.
pub fn start(parked: Parked, port: I2cPort) -> anyhow::Result<()> {
    parked
        .resume
        .send(port)
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
    resumed: Receiver<I2cPort>,
    requests: Receiver<Envelope>,
) {
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
    let Ok(mut port) = resumed.recv() else {
        return;
    };
    for envelope in requests {
        let answer = serve(&mut port, envelope.request);
        let _ = envelope.reply.send(answer);
    }
}

/// One request, on the owner.
fn serve(port: &mut I2cPort, request: Request) -> Answer {
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
    }
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
    answer
        .recv_timeout(deadline)
        .map_err(|_| format!("i2c port {port}'s owner did not answer in {} s", deadline.as_secs()))
}

/// The owners' console command (the Glider Command Language): `i2c touch`, which proves the wedge rule on the board.
pub fn register_commands() {
    gcl::register(vec![Command {
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
                Err(error) => cc_protocol::build("err", &["i2c", &error], &[]),
            }
        }),
    }]);
}
