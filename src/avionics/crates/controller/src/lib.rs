//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device bring-up's rules: the port of src/glider controller.py's setup pass, and of the operator lines
//! cc_client.py builds from its outcome (doc/specs/avionics-port.md M5, "The registry"). The firmware's controller
//! (M5 stage 5) owns the buses and the parts and runs each attempt; this crate decides everything around them, with
//! no bus and no clock of its own:
//!  * [`set_up`] -- one entry's attempts: `board.setup_retries` of them, [`RETRY_GAP_MS`] apart and never after the
//!    last, the driver's diagnose folded into the reason on the last, plan B's log lines word for word
//!    (controller.py:385-445);
//!  * [`candidate`] and [`sort_alternatives`] -- the alternatives pass: a failed entry that an up device stands in
//!    for (the same I2C socket, the same data, another driver, and no XSHUT pulse of its own) is not fitted rather
//!    than failed (controller.py:130-241);
//!  * [`Table`] -- the outcome per directory entry (Up, Failed, Unfitted, and Crashed once an Up part's loop dies,
//!    M6+), the lines the bring-up ends with (controller.py:378-382), and the readers CC answers from: `health.tasks`,
//!    `verify`'s devices, problems and alternatives, the failure lines `probe` and `arm` print (cc_client.py:292-349,
//!    513, 807-816), and `inspect controller` (controller.py:630-632);
//!  * [`binding`] -- plan B's bind() defaults: the bus and address an entry's driver talks over (i2cbus.py:275-301,
//!    spibus.py:196-219), ported once so no call site compares the raw `bus`, `id` and `addr` fields itself;
//!  * [`stream`] and [`vetoes`] -- the telemetry stream a recorded entry writes, and the stage machine's second, its
//!    veto episodes: the file, columns, window (never decimated: measured -> reported) and durability (module
//!    `streams`);
//!  * [`readiness`] -- `verify`'s flight-readiness verdict, the config choices no probe can see (cc_client.py:47-107);
//!  * [`ring_gate`] -- the flash ring's refusal of `arm` and `verify`, under plan B's recorder probe's words;
//!  * [`stale_arm`] -- the refusal of an `arm` that waited on the board until the hub may have given up on it;
//!  * [`Stall`] -- the supervisor's rule for a control thread that stopped ticking while the board controls a flight
//!    (module `watchdog`).
//!
//! The device directory, its pins and its `provides` come from board_config's typed view, read from the post-layout
//! document, as plan B's Controller reads the config layout.resolve has rewritten (main.py:97-101).
//!
//! The port, against plan B:
//!  * Plan B creates a Task per attempt and awaits its setup(); here the caller's closure makes the attempt and
//!    reports its [`Attempt`], and [`set_up`] keeps the rules: the reasons, the retry gaps, the diagnose on the last
//!    attempt, the log lines.
//!  * Plan B logs `cleanup raised` when a half-set-up task's finish() raises; a Rust part is dropped, and a drop cannot
//!    fail, so the line has no counterpart. Nor has `[diagnose raised ...]`: a Rust diagnose answers a text, it does
//!    not raise.
//!  * Plan B's Controller may run setup() again, keeping the tasks already up; the firmware brings a board up once
//!    per boot, so a [`Table`] is built from one pass. [`sort_alternatives`] itself takes any up set in any order, as
//!    plan B's re-run hands it, and the golden fixture replays that too.
//!  * The socket's bus id is Python's `str(id)`: an entry with no `id` (absent or null) sits on bus 'None', which
//!    matches only another id-less entry, never id 0 (controller.py:149), although bind() puts both on i2c:0. Kept,
//!    because the alternatives rule is plan B's.
//!  * An `addr` outside 0..=0x7f, or not an int, is no address (board_config's `Entry`), so the entry has no socket;
//!    plan B keeps it as a socket and its driver fails at the first transfer. validate() refuses all but the first.
//!  * A device whose loop crashed stays in plan B's `failures` in crash order; [`Table`] lists every reader in
//!    directory order. Only the key order of a JSON object differs.

mod readiness;
mod ring_gate;
mod stale_arm;
mod streams;
mod watchdog;

use std::fmt;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, Ordering};

use board_config::Entry;
use board_config::python::formatted;
use serde_json::Value;

pub use readiness::readiness;
pub use ring_gate::ring_gate;
pub use stale_arm::stale_arm;
pub use streams::{
    BAROMETER_FIELDS, CHECKPOINT_FIELDS, IMU_FIELDS, MAGNETOMETER_FIELDS, SEPARATION_FIELDS, SEQUENCER_FIELDS,
    SERVO_FIELDS, Stream, VETOES_FIELDS, companions, stream, vetoes,
};
pub use watchdog::Stall;

/// The pause between two setup attempts of one device, in milliseconds: a flaky breadboard contact settles
/// (controller.py:418 `asyncio.sleep_ms(200)`).
pub const RETRY_GAP_MS: u32 = 200;

/// The failure reason of an entry whose implementation nothing runs (controller.py:398), and the start of the line
/// logged for it (controller.py:283).
pub const NO_DRIVER: &str = "no driver/activity";

/// The failure reason of a setup that did not bring its part up (controller.py:406), before diagnose adds the why.
pub const SETUP_FAILED: &str = "setup failed (absent / miswired?)";

/// The bus kind whose address identifies a place on the board: only I2C's makes a socket (controller.py:134-138).
const SOCKET_KIND: &str = I2C_KIND;

/// The bus id plan B's `str(comp.get('id'))` gives an entry that names none: Python's `str(None)`.
const NO_BUS_ID: &str = "None";

/// The entry key naming the pin a part's setup pulses to reset it before it has identified the part: the lasers'
/// XSHUT, the only such pin (controller.py:164-183).
const RESET_PIN: &str = "xshut_pin";

/// The bus id bind() reads when an I2C entry names none (i2cbus.py:297 `device.get('id', 0)`).
const DEFAULT_I2C_BUS_ID: &str = "0";

/// The bus id bind() reads when an SPI entry names none: spi:1, the only SPI bus a board declares (spibus.py:217
/// `device.get('id', 1)`).
const DEFAULT_SPI_BUS_ID: &str = "1";

/// The I2C bus kind, as an entry's `bus` names it and bind() opens it (i2cbus.py:275-301).
const I2C_KIND: &str = "i2c";

/// The SPI bus kind, as an entry's `bus` names it and bind() opens it (spibus.py:196-219).
const SPI_KIND: &str = "spi";

/// An I2C socket: the place a part answers, a bus and an address (controller.py:130-149 `_socket`).
///
/// Only an I2C address identifies one. Two UART or PWM devices on one bus are two devices, and an SPI entry's
/// `addr` is its fallback for an I2C wiring: on SPI the chip select is the place, so two SPI parts sharing a fallback
/// address are two parts.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Socket {
    /// The bus id as plan B compares it, as text (`str(comp.get('id'))`): an int 1 and a text '1' are one bus, and an
    /// entry with no id sits on 'None'.
    pub id: String,
    /// The 7-bit address.
    pub address: u8,
}

impl fmt::Display for Socket {
    /// The socket as plan B names it in a line: 'i2c:1 0x29' (controller.py:259 `'%s:%s 0x%02x' % socket`).
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "{SOCKET_KIND}:{} 0x{:02x}", self.id, self.address)
    }
}

/// One directory entry, as the alternatives pass compares it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Candidate<'d> {
    /// The device's name.
    pub name: &'d str,
    /// What runs it: its `driver`, else its `activity` (controller.py:111-128 `driver`). Empty when that is not text.
    pub implementation: &'d str,
    /// Where it answers; None for anything but an I2C entry with an address (see [`Socket`]).
    pub socket: Option<Socket>,
    /// The data it feeds the databoard: its `provides` keys, sorted (controller.py:151-162 `_quantities`).
    pub quantities: Vec<&'d str>,
    /// The GPIO its setup pulses before it identifies its part: its `xshut_pin` through the pins map; None when the
    /// entry routes none (controller.py:164-183 `_reset_gpio`).
    pub reset_gpio: Option<u8>,
}

/// The candidate a directory entry of `document` is.
///
/// The socket is plan B's `_socket`: an entry whose `bus` is 'i2c' and that has an address; an entry without a `bus`
/// key has none, whatever its driver defaults to (bind() would still put it on I2C). The quantities are its
/// `provides` keys as board_config reads them: a quantity whose spec is not an object is not one (validate()
/// refuses it; plan B's setup raises on it, so the entry fails anyway). The reset pin is plan B's `_reset_gpio`,
/// the board's `pins` map through `xshut_pin`, read as task.py `_pin_gpio` reads a pin.
pub fn candidate<'d>(document: &Value, entry: &Entry<'d>) -> Candidate<'d> {
    let socket = entry
        .address
        .filter(|_| entry.bus == Some(SOCKET_KIND))
        .map(|address| Socket {
            id: entry.bus_id.clone().unwrap_or_else(|| NO_BUS_ID.to_string()),
            address,
        });
    Candidate {
        name: entry.name,
        implementation: entry.implementation,
        socket,
        quantities: board_config::provides(entry)
            .into_iter()
            .map(|provide| provide.quantity)
            .collect(),
        reset_gpio: board_config::pin(document, entry, RESET_PIN),
    }
}

/// What one setup attempt did, as the caller reports it to [`set_up`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Attempt {
    /// The part came up (plan B's `setup()` returned True).
    Up,
    /// The part did not come up (`setup()` returned False; a Rust driver's `Err`).
    Failed {
        /// What the driver's diagnose answered: asked on the LAST attempt only, while the transport is still alive
        /// (controller.py:410-411). None when the driver has none, and on any earlier attempt, where it is ignored.
        diagnosis: Option<String>,
    },
    /// The setup failed outside the part's own failure path (plan B's `setup()` raised): a part that could not even
    /// be built, and the golden fixture's raising stubs.
    ///
    /// Plan B logs `setup raised` BEFORE it diagnoses (controller.py:409-411), but [`set_up`] logs it when the attempt
    /// reports, so after its diagnosis was made. Anything logged while making that diagnosis (the bus's wedge line,
    /// for one) then comes before the `setup raised` line, not after it. A producer that answers no diagnosis for a
    /// raised attempt keeps plan B's order; one that diagnoses it must log nothing while doing so, or accept the
    /// other order.
    Raised {
        /// The error, as the line and the reason carry it (plan B: the exception's repr).
        error: String,
        /// As for [`Attempt::Failed`].
        diagnosis: Option<String>,
    },
    /// Nothing runs this entry's implementation (plan B's create() found no class): retrying cannot help.
    NoDriver,
    /// Another part answered at the address (its identity read as another silicon's: the VL53L4CX and the VL53L1X
    /// share 0x29): retrying cannot help, so the attempts end here and the diagnosis is taken whatever the attempt.
    /// New (plan B retried it, ~0.2 s a gap, on every boot of a board that declares both lasers; the fast warm boot,
    /// operator 2026-10-11).
    Foreign {
        /// As for [`Attempt::Failed`], but used: this attempt is the last.
        diagnosis: Option<String>,
    },
}

/// Bring one device up: plan B's `_bring_up` (controller.py:385-420) with its `_diagnose` (controller.py:422-445).
///
/// `attempt(number, last)` makes attempt `number` (from 1) and reports it; `last` is true on the final one, the only
/// one whose diagnosis is used. There are `attempts` of them, at least 1 (plan B's `max(1, setup_retries)`,
/// controller.py:368; board_config::setup_retries reads the count). An attempt that fails is followed by
/// `sleep_ms(RETRY_GAP_MS)` unless it was the last. `log` receives plan B's lines:
///  * `controller :: task '<name>' up` when an attempt brings the part up (controller.py:376);
///  * `controller :: no driver/activity '<implementation>' for '<name>'` for [`Attempt::NoDriver`], which ends the
///    attempts (controller.py:283, 402);
///  * `controller :: task '<name>' setup raised: <error>` for [`Attempt::Raised`] (controller.py:409);
///  * `controller :: task '<name>' setup attempt <n>/<attempts> failed, retrying` before each gap (controller.py:417);
///  * `controller :: task '<name>' setup found another part, not retried` for [`Attempt::Foreign`] (new).
///
/// Returns the failure reason when no attempt brought the part up: [`SETUP_FAILED`], or a raised attempt's error,
/// as the last attempt left it, with its diagnosis folded in ([`diagnosed`]); [`NO_DRIVER`] when the first attempt
/// found no driver. A later attempt that finds no driver keeps the reason the attempt before it left, as plan B's
/// loop does.
pub fn set_up(
    name: &str,
    implementation: &str,
    attempts: u32,
    mut attempt: impl FnMut(u32, bool) -> Attempt,
    mut sleep_ms: impl FnMut(u32),
    mut log: impl FnMut(String),
) -> Result<(), String> {
    let attempts = attempts.max(1);
    let mut reason = NO_DRIVER.to_string();
    for number in 1..=attempts {
        let last = number == attempts;
        let diagnosis = match attempt(number, last) {
            Attempt::Up => {
                log(format!("controller :: task '{name}' up"));
                return Ok(());
            }
            Attempt::NoDriver => {
                log(format!("controller :: {NO_DRIVER} '{implementation}' for '{name}'"));
                break;
            }
            Attempt::Failed { diagnosis } => {
                reason = SETUP_FAILED.to_string();
                diagnosis
            }
            Attempt::Raised { error, diagnosis } => {
                log(format!("controller :: task '{name}' setup raised: {error}"));
                reason = error;
                diagnosis
            }
            Attempt::Foreign { diagnosis } => {
                log(format!(
                    "controller :: task '{name}' setup found another part, not retried"
                ));
                reason = SETUP_FAILED.to_string();
                if let Some(detail) = diagnosis {
                    reason = diagnosed(&reason, &detail);
                }
                break;
            }
        };
        if last {
            if let Some(detail) = diagnosis {
                reason = diagnosed(&reason, &detail);
            }
        } else {
            log(format!(
                "controller :: task '{name}' setup attempt {number}/{attempts} failed, retrying"
            ));
            sleep_ms(RETRY_GAP_MS);
        }
    }
    Err(reason)
}

/// A failure reason with the driver's diagnosis folded in: `'<reason> -- <detail>'`, or the reason alone when the
/// diagnosis is empty (controller.py:445).
pub fn diagnosed(reason: &str, detail: &str) -> String {
    if detail.is_empty() {
        reason.to_string()
    } else {
        format!("{reason} -- {detail}")
    }
}

/// Plan B's alternatives pass (controller.py:185-241): move each failed entry that an up device stands in for from
/// `failures` to the returned (loser, winner) pairs.
///
/// Two parts that cannot share one address on one bus are declared for one socket (the VL53L4CX and the VL53L1X at
/// 0x29), and the part that is soldered wins; the other entry's setup fails BY DESIGN. Each failure, in `failures`'
/// order, takes the FIRST entry of `up_in_setup_order` that passes the four rules:
///  1. the same I2C socket ([`Socket`]): an SPI entry, or one without an address, is never a candidate;
///  2. the same non-empty set of quantities: an entry that provides nothing has nothing to stand in for;
///  3. a different implementation: one driver twice on one socket is a config mistake, and the duplicate's setup
///     re-initialises the winner's own part;
///  4. no XSHUT of the loser's own: its setup pulses that pin and reboots whatever part sits in the socket, so a
///     loser with a [`Candidate::reset_gpio`] stays a failure, in either setup order, its reason extended with
///     `'; shares XSHUT GPIO<n> with <winner> (<driver>): its setup resets the fitted laser -- enable exactly one of
///     <both names, sorted> on this board'`.
///
/// When no up entry passes, the failure stays: a missing laser must still block arming. `candidates` is the
/// directory, and the other arguments hold indices into it (an index past it names no candidate). `failures` keeps
/// its order; the pairs come in that order too.
pub fn sort_alternatives(
    candidates: &[Candidate],
    up_in_setup_order: &[usize],
    failures: &mut Vec<(usize, String)>,
) -> Vec<(usize, usize)> {
    let mut alternatives = Vec::new();
    failures.retain_mut(|(loser, reason)| {
        let Some(failed) = candidates.get(*loser) else {
            return true;
        };
        let Some(socket) = &failed.socket else {
            return true;
        };
        if failed.quantities.is_empty() {
            return true;
        }
        let stands_in = |winner: &&usize| {
            candidates.get(**winner).is_some_and(|up| {
                up.socket.as_ref() == Some(socket)
                    && up.quantities == failed.quantities
                    && up.implementation != failed.implementation
            })
        };
        let Some(&winner) = up_in_setup_order.iter().find(stands_in) else {
            return true;
        };
        let Some(gpio) = failed.reset_gpio else {
            alternatives.push((*loser, winner));
            return false;
        };
        let up = &candidates[winner];
        let mut both = [failed.name, up.name];
        // by code point, as Python's sorted() orders str
        both.sort_unstable();
        reason.push_str(&format!(
            "; shares XSHUT GPIO{gpio} with {} ({}): its setup resets the fitted laser -- enable exactly one of {} on \
             this board",
            up.name,
            up.implementation,
            both.join(", ")
        ));
        true
    });
    alternatives
}

/// The operator's line for an unfitted alternative: which part is absent and which one took its socket
/// (controller.py:243-263).
///
/// `'<loser driver> not fitted -- <winner driver> (<winner>) answers i2c:1 0x29'` while the winner runs; once its
/// loop has crashed, `'... answered i2c:1 0x29 at setup, now down'`: the loser stays not fitted (the winner
/// identified itself at setup), but the line must not say the winner answers while `probe` and `arm` call it failed.
pub fn unfitted(
    loser_implementation: &str,
    winner_name: &str,
    winner_implementation: &str,
    socket: &Socket,
    winner_down: bool,
) -> String {
    if winner_down {
        format!(
            "{loser_implementation} not fitted -- {winner_implementation} ({winner_name}) answered {socket} at setup, \
             now down"
        )
    } else {
        format!("{loser_implementation} not fitted -- {winner_implementation} ({winner_name}) answers {socket}")
    }
}

/// A verdict that names the device's implementation: `'<driver> -- <why>'`, or `why` alone when there is none (a
/// name that is no configured device, such as 'mission') (cc_client.py:292-308 `_named`).
pub fn named(implementation: &str, why: &str) -> String {
    if implementation.is_empty() {
        why.to_string()
    } else {
        format!("{implementation} -- {why}")
    }
}

/// The verdict for a device that failed (cc_client.py:311-326 `_failure`): `'<driver> -- not connected: <reason>'`
/// for one that never came up, `'<driver> -- <reason>'` for one that came up and whose loop then crashed (it was
/// connected).
pub fn failure_line(implementation: &str, reason: &str, crashed: bool) -> String {
    if crashed {
        named(implementation, reason)
    } else {
        named(implementation, &format!("not connected: {reason}"))
    }
}

/// The bus a device's driver talks over, as plan B's bind() resolves an entry (i2cbus.py:275-301, spibus.py:196-219).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Binding {
    /// An I2C bus and the address the driver uses on it.
    I2c {
        /// The bus id as text, as config.bus looks it up (`str(ident)`, config.py:696): a bus is found only by the
        /// key its config spells, so '1.0' or '01' names no bus a config defines as '1'.
        id: String,
        /// The address: the entry's `addr`, else the driver's datasheet address; None for an explicit null (the
        /// driver gets no address and its setup fails) or an `addr` board_config reads as none.
        address: Option<u8>,
    },
    /// An SPI bus; the chip select is a pin, which the driver resolves itself (spibus.py:203-206).
    Spi {
        /// The bus id as text, as for [`Binding::I2c`].
        id: String,
    },
    /// No bus plan B's bind() could find: a `bus` that is null, not text or another kind (bind() serves I2C and SPI;
    /// a UART part opens its port itself), or an explicit null `id` (config.bus then looks up 'None', which no config
    /// defines).
    Unbound,
}

/// Plan B's bind() defaults for one entry, ported once: which transport its driver opens and where.
///
/// `default_kind` is the driver's own transport when the entry names no `bus`: 'i2c' for most drivers and the
/// ADXL375 (adxl375.py:107 `get('bus', 'i2c') == 'spi'`), 'spi' for the LSM6DSO32 (lsm6dso32.py:144). An absent `id`
/// is 0 on I2C (i2cbus.py:297) and 1 on SPI (spibus.py:217); an absent `addr` is `default_address`, the driver's
/// datasheet address (i2cbus.py:301). An explicit null is NOT absent: plan B's `get(key, default)` returns it, so a
/// null `id` binds no bus and a null `addr` gives the driver no address. A single-transport driver reads an answer of
/// the other kind as no bus (plan B's would raise building its bus from the other kind's spec).
pub fn binding(entry: &Entry, default_kind: &str, default_address: Option<u8>) -> Binding {
    let kind = match entry.fields.get("bus") {
        None => default_kind,
        Some(Value::String(kind)) => kind.as_str(),
        Some(_) => return Binding::Unbound,
    };
    let id = |default: &str| match entry.fields.get("id") {
        None => Some(default.to_string()),
        Some(Value::Null) => None,
        Some(id) => Some(formatted(id)),
    };
    match kind {
        I2C_KIND => id(DEFAULT_I2C_BUS_ID).map_or(Binding::Unbound, |id| Binding::I2c {
            id,
            address: if entry.fields.contains_key("addr") {
                entry.address
            } else {
                default_address
            },
        }),
        SPI_KIND => id(DEFAULT_SPI_BUS_ID).map_or(Binding::Unbound, |id| Binding::Spi { id }),
        _ => Binding::Unbound,
    }
}

/// How a directory entry's bring-up ended.
#[derive(Clone, Debug, PartialEq, Eq)]
enum State {
    /// Its part came up.
    Up,
    /// It did not come up, for this reason.
    Failed {
        /// The reason [`set_up`] gave, extended by the XSHUT rule when it applies.
        reason: String,
    },
    /// It did not come up because another part answers its socket.
    Unfitted {
        /// The winner's index in the table.
        winner: usize,
    },
}

/// A directory entry's status, as CC reads it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status<'t> {
    /// The part came up and runs.
    Up,
    /// The part never came up, for this reason.
    Failed(&'t str),
    /// An alternative: the entry at this index of the table took its socket.
    Unfitted(usize),
    /// The part came up and its loop then died, for this reason (plan B: in `tasks` AND in `failures`).
    Crashed(&'t str),
}

/// One directory entry of the board and how its bring-up ended.
///
/// The table is built once at boot and shared read-only (the firmware leaks it). It sees two changes afterwards, and
/// neither needs a lock: a crash, set at most once, and the part's runtime health, an atomic its producer writes.
/// That atomic is why an entry is neither `Clone` nor `PartialEq`: it is a live flag, not a value.
#[derive(Debug)]
pub struct DeviceEntry<'d> {
    /// The device's name.
    pub name: &'d str,
    /// What runs it (see [`Candidate::implementation`]).
    pub implementation: &'d str,
    /// Its I2C socket, when it has one.
    pub socket: Option<Socket>,
    /// How its bring-up ended.
    state: State,
    /// Why its loop died, once it has (M6+: errors that escape the per-part glue).
    crash: OnceLock<String>,
    /// Whether the part's latest read succeeded: plan B's `Task._healthy`, the RUNTIME health `note()` tracks, where
    /// [`DeviceEntry::active`] is setup's. True from Up until a producer says otherwise; false for an entry that never
    /// came up.
    healthy: AtomicBool,
}

impl DeviceEntry<'_> {
    /// The entry's status: its bring-up's outcome, or [`Status::Crashed`] once an up part's loop died.
    pub fn status(&self) -> Status<'_> {
        match (&self.state, self.crash.get()) {
            (State::Up, Some(reason)) => Status::Crashed(reason),
            (State::Up, None) => Status::Up,
            (State::Failed { reason }, _) => Status::Failed(reason),
            (State::Unfitted { winner }, _) => Status::Unfitted(*winner),
        }
    }

    /// Whether the part came up, crashed or not: plan B's `active(name)` is not None (controller.py:287-299), which
    /// is what `health.tasks` lists.
    pub fn active(&self) -> bool {
        self.state == State::Up
    }

    /// Record that the part's loop died: true when this is the entry's crash, false for an entry that never came up
    /// or has crashed already (the first reason stays, as a dead loop dies once).
    pub fn crash(&self, reason: String) -> bool {
        self.active() && self.crash.set(reason).is_ok()
    }

    /// Whether the part's latest read succeeded (plan B's `inspect()['healthy']`).
    pub fn healthy(&self) -> bool {
        self.healthy.load(Ordering::Relaxed)
    }

    /// Record the outcome of the part's latest read, from its producer (its one writer): true when this changed the
    /// health, the transition plan B's `note()` logs on. It writes only on a change, so a producer may call it every
    /// read without dirtying the cache line the readers on the other core share. Relaxed: the flag publishes nothing
    /// else. An entry that never came up has no reads and stays unhealthy: false, nothing written, as
    /// [`DeviceEntry::crash`] refuses it too.
    pub fn set_healthy(&self, healthy: bool) -> bool {
        if !self.active() || self.healthy.load(Ordering::Relaxed) == healthy {
            return false;
        }
        self.healthy.store(healthy, Ordering::Relaxed);
        true
    }

    /// The reason the entry is in plan B's `failures` (controller.py:85), and whether it crashed after coming up;
    /// None for an entry that runs or is an alternative.
    fn failure(&self) -> Option<(&str, bool)> {
        match self.status() {
            Status::Failed(reason) => Some((reason, false)),
            Status::Crashed(reason) => Some((reason, true)),
            Status::Up | Status::Unfitted(_) => None,
        }
    }
}

/// The board's devices after bring-up: one entry per directory entry, in directory order, and the readers CC
/// answers from.
#[derive(Debug)]
pub struct Table<'d> {
    /// The entries, in directory order.
    entries: Vec<DeviceEntry<'d>>,
}

/// What `inspect controller` reports of the bring-up: plan B's `inspect()` keys `tasks`, `failures` and
/// `alternatives` (controller.py:630-632), each as plan B holds it, not as CC words it. Its `stage`, `armed` and
/// `manual` are the sequencer's, not the table's.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Inspection<'t> {
    /// Every part that came up, crashed or not, in setup order (plan B's `list(self.tasks.keys())`).
    pub tasks: Vec<&'t str>,
    /// Every entry in plan B's `failures` with its bare reason: the never-up ones and the crashed ones, in directory
    /// order (the lines CC prints for them are [`Table::failures`]).
    pub failures: Vec<(&'t str, &'t str)>,
    /// Every alternative with the name of the entry that took its socket: plan B's `alternatives`, loser to winner.
    pub alternatives: Vec<(&'t str, &'t str)>,
}

impl<'d> Table<'d> {
    /// The table one bring-up pass leaves: each directory entry's candidate with what [`set_up`] returned for it, in
    /// directory order (the order plan B sets them up in), and the alternatives pass run over the result
    /// ([`sort_alternatives`], with the entries that came up in that order).
    pub fn new(bring_up: Vec<(Candidate<'d>, Result<(), String>)>) -> Table<'d> {
        let (candidates, outcomes): (Vec<Candidate<'d>>, Vec<Result<(), String>>) = bring_up.into_iter().unzip();
        let up: Vec<usize> = (0..outcomes.len()).filter(|index| outcomes[*index].is_ok()).collect();
        let mut failures: Vec<(usize, String)> = outcomes
            .into_iter()
            .enumerate()
            .filter_map(|(index, outcome)| outcome.err().map(|reason| (index, reason)))
            .collect();
        let alternatives = sort_alternatives(&candidates, &up, &mut failures);
        let mut states: Vec<State> = vec![State::Up; candidates.len()];
        for (index, reason) in failures {
            states[index] = State::Failed { reason };
        }
        for (loser, winner) in alternatives {
            states[loser] = State::Unfitted { winner };
        }
        let entries = candidates
            .into_iter()
            .zip(states)
            .map(|(candidate, state)| DeviceEntry {
                name: candidate.name,
                implementation: candidate.implementation,
                socket: candidate.socket,
                healthy: AtomicBool::new(state == State::Up),
                state,
                crash: OnceLock::new(),
            })
            .collect();
        Table { entries }
    }

    /// The entries, in directory order; [`Status::Unfitted`] indexes this slice.
    pub fn entries(&self) -> &[DeviceEntry<'d>] {
        &self.entries
    }

    /// The entry named `name`, if the directory has one.
    pub fn find(&self, name: &str) -> Option<&DeviceEntry<'d>> {
        self.entries.iter().find(|entry| entry.name == name)
    }

    /// `health.tasks`: every part that came up, with false once its loop crashed (cc_client.py:513 `{name, ok:
    /// validate()}` over the Controller's tasks).
    pub fn tasks(&self) -> Vec<(&str, bool)> {
        self.entries
            .iter()
            .filter(|entry| entry.active())
            .map(|entry| (entry.name, entry.crash.get().is_none()))
            .collect()
    }

    /// The failure line of every entry in plan B's `failures`: the never-up ones and the crashed ones, as `verify`'s
    /// and `arm`'s problems and `probe all` list them (cc_client.py:566, 765, 816).
    pub fn failures(&self) -> Vec<(&str, String)> {
        self.entries
            .iter()
            .filter_map(|entry| {
                let (reason, crashed) = entry.failure()?;
                Some((entry.name, failure_line(entry.implementation, reason, crashed)))
            })
            .collect()
    }

    /// The not-fitted line of every alternative: `verify.alternatives` and `probe`'s second map (cc_client.py:768,
    /// 807), and what [`Table::bring_up_lines`] logs for each.
    pub fn alternatives(&self) -> Vec<(&str, String)> {
        self.entries
            .iter()
            .enumerate()
            .filter_map(|(index, entry)| Some((entry.name, self.unfitted_at(index)?)))
            .collect()
    }

    /// `verify.devices` (cc_client.py:807-815): every entry's line, `'<driver> -- up'`, `'<driver> -- down:
    /// <reason>'` (a crashed part is down too), or its not-fitted line.
    pub fn devices(&self) -> Vec<(&str, String)> {
        self.entries
            .iter()
            .enumerate()
            .map(|(index, entry)| {
                let line = match entry.status() {
                    Status::Up => named(entry.implementation, "up"),
                    Status::Failed(reason) | Status::Crashed(reason) => {
                        named(entry.implementation, &format!("down: {reason}"))
                    }
                    Status::Unfitted(_) => self.unfitted_at(index).unwrap_or_default(),
                };
                (entry.name, line)
            })
            .collect()
    }

    /// The bring-up's summary line, `controller :: <n> device(s) not up: <names, sorted>`, over plan B's `failures`
    /// (controller.py:380-382); None when every entry is up or an alternative.
    pub fn not_up_line(&self) -> Option<String> {
        let mut names: Vec<&str> = self
            .entries
            .iter()
            .filter(|entry| entry.failure().is_some())
            .map(|entry| entry.name)
            .collect();
        if names.is_empty() {
            return None;
        }
        // by code point, as Python's sorted() orders str
        names.sort_unstable();
        Some(format!(
            "controller :: {} device(s) not up: {}",
            names.len(),
            names.join(", ")
        ))
    }

    /// The lines plan B's setup() logs once every entry has had its attempts (controller.py:378-382): `controller ::
    /// <name>: <not-fitted line>` for each alternative, in directory order, then [`Table::not_up_line`] when there is
    /// one. Read them before any crash, as plan B logs them at bring-up: a crashed winner's line says it is down.
    pub fn bring_up_lines(&self) -> Vec<String> {
        let mut lines: Vec<String> = self
            .alternatives()
            .into_iter()
            .map(|(name, line)| format!("controller :: {name}: {line}"))
            .collect();
        lines.extend(self.not_up_line());
        lines
    }

    /// The `inspect controller` snapshot: see [`Inspection`].
    pub fn inspect(&self) -> Inspection<'_> {
        let mut inspection = Inspection {
            tasks: Vec::new(),
            failures: Vec::new(),
            alternatives: Vec::new(),
        };
        for entry in &self.entries {
            if entry.active() {
                inspection.tasks.push(entry.name);
            }
            if let Some((reason, _)) = entry.failure() {
                inspection.failures.push((entry.name, reason));
            }
            if let Status::Unfitted(winner) = entry.status()
                && let Some(winner) = self.entries.get(winner)
            {
                inspection.alternatives.push((entry.name, winner.name));
            }
        }
        inspection
    }

    /// The failure line for one device (cc_client.py:311-326 `_failure`); None for a name that is no failed or crashed
    /// entry (plan B raises KeyError there).
    pub fn failure(&self, name: &str) -> Option<String> {
        let entry = self.find(name)?;
        let (reason, crashed) = entry.failure()?;
        Some(failure_line(entry.implementation, reason, crashed))
    }

    /// One probe result as the operator reads it (cc_client.py:329-349 `_probed`): a failing `result` named after the
    /// device's implementation (unchanged for a name that is no entry); for a passing one (None), the failure line of
    /// a device in `failures`, because a crashed part whose probe still answers is not healthy; otherwise None.
    pub fn probed(&self, name: &str, result: Option<&str>) -> Option<String> {
        match result {
            Some(why) => Some(named(self.find(name).map_or("", |entry| entry.implementation), why)),
            None => self.failure(name),
        }
    }

    /// The not-fitted line for one alternative (controller.py:243-263 `unfitted`); None for a name that is no
    /// alternative.
    pub fn unfitted(&self, name: &str) -> Option<String> {
        let index = self.entries.iter().position(|entry| entry.name == name)?;
        self.unfitted_at(index)
    }

    /// The not-fitted line of the entry at `index`, when it is an alternative ([`unfitted`] over its winner).
    fn unfitted_at(&self, index: usize) -> Option<String> {
        let loser = self.entries.get(index)?;
        let Status::Unfitted(winner) = loser.status() else {
            return None;
        };
        let winner = self.entries.get(winner)?;
        // the winner passed the socket rule, so it has one
        let socket = winner.socket.as_ref()?;
        Some(unfitted(
            loser.implementation,
            winner.name,
            winner.implementation,
            socket,
            winner.crash.get().is_some(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::{
        Attempt, Binding, Candidate, Inspection, NO_DRIVER, RETRY_GAP_MS, SETUP_FAILED, Socket, Status, Table, binding,
        candidate, diagnosed, failure_line, named, set_up, sort_alternatives, unfitted,
    };
    use serde_json::{Value, json};

    /// What one [`set_up`] call did.
    struct Trace {
        /// What it returned.
        result: Result<(), String>,
        /// The `last` flag each attempt was made with, in order.
        lasts: Vec<bool>,
        /// The gaps it slept, in milliseconds.
        sleeps: Vec<u32>,
        /// The lines it logged.
        log: Vec<String>,
    }

    /// Run [`set_up`] for 'part' (implementation 'driver') with `attempts`, each attempt answered by `script`.
    fn trace(attempts: u32, mut script: impl FnMut(u32, bool) -> Attempt) -> Trace {
        let (mut lasts, mut sleeps, mut log) = (Vec::new(), Vec::new(), Vec::new());
        let result = set_up(
            "part",
            "driver",
            attempts,
            |number, last| {
                lasts.push(last);
                script(number, last)
            },
            |milliseconds| sleeps.push(milliseconds),
            |line| log.push(line),
        );
        Trace {
            result,
            lasts,
            sleeps,
            log,
        }
    }

    /// A failed attempt that diagnoses `detail` (used only on the last).
    fn failed(detail: &str) -> Attempt {
        Attempt::Failed {
            diagnosis: Some(detail.to_string()),
        }
    }

    #[test]
    fn a_part_that_comes_up_first_time_logs_up_and_never_sleeps() {
        let run = trace(3, |_, _| Attempt::Up);
        assert_eq!(run.result, Ok(()));
        assert_eq!(run.lasts, [false]);
        assert_eq!(run.sleeps, Vec::<u32>::new());
        assert_eq!(run.log, ["controller :: task 'part' up"]);
    }

    #[test]
    fn another_part_at_the_address_ends_the_attempts_at_once_with_its_diagnosis() {
        let run = trace(3, |_, _| Attempt::Foreign {
            diagnosis: Some("model 0xeacc".to_string()),
        });
        assert_eq!(run.result, Err(format!("{SETUP_FAILED} -- model 0xeacc")));
        assert_eq!(run.lasts, [false], "one attempt, not the configured three");
        assert_eq!(run.sleeps, Vec::<u32>::new());
        assert_eq!(
            run.log,
            ["controller :: task 'part' setup found another part, not retried"]
        );
        // NEGATIVE: a plain failure still retries to the last attempt
        let run = trace(3, |_, _| failed("silent"));
        assert_eq!(run.lasts, [false, false, true]);
        assert_eq!(run.sleeps, [RETRY_GAP_MS, RETRY_GAP_MS]);
    }

    #[test]
    fn a_flaky_part_retries_after_a_gap() {
        let run = trace(3, |number, _| if number == 2 { Attempt::Up } else { failed("unused") });
        assert_eq!(run.result, Ok(()));
        assert_eq!(run.sleeps, [RETRY_GAP_MS]);
        assert_eq!(
            run.log,
            [
                "controller :: task 'part' setup attempt 1/3 failed, retrying",
                "controller :: task 'part' up"
            ]
        );
    }

    #[test]
    fn the_diagnosis_counts_on_the_last_attempt_only_and_no_gap_follows_it() {
        let run = trace(3, |number, _| failed(&format!("detail {number}")));
        assert_eq!(run.result, Err(format!("{SETUP_FAILED} -- detail 3")));
        assert_eq!(run.lasts, [false, false, true]);
        assert_eq!(run.sleeps, [RETRY_GAP_MS, RETRY_GAP_MS]);
        assert_eq!(run.log.len(), 2, "{:?}", run.log);
        assert!(run.log[1].ends_with("setup attempt 2/3 failed, retrying"));
    }

    #[test]
    fn no_diagnosis_or_an_empty_one_keeps_the_reason() {
        let run = trace(1, |_, _| Attempt::Failed { diagnosis: None });
        assert_eq!(run.result, Err(SETUP_FAILED.to_string()));
        let run = trace(1, |_, _| failed(""));
        assert_eq!(run.result, Err(SETUP_FAILED.to_string()));
        assert_eq!(diagnosed("why", ""), "why");
        assert_eq!(diagnosed("why", "because"), "why -- because");
    }

    #[test]
    fn no_driver_is_never_retried() {
        let run = trace(3, |_, _| Attempt::NoDriver);
        assert_eq!(run.result, Err(NO_DRIVER.to_string()));
        assert_eq!(run.lasts, [false]);
        assert_eq!(run.sleeps, Vec::<u32>::new());
        assert_eq!(run.log, ["controller :: no driver/activity 'driver' for 'part'"]);
    }

    #[test]
    fn no_driver_after_a_failure_keeps_that_failure() {
        // NEGATIVE: plan B's loop breaks with the reason the attempt before left, not 'no driver/activity'
        let run = trace(3, |number, _| {
            if number == 1 {
                Attempt::Failed { diagnosis: None }
            } else {
                Attempt::NoDriver
            }
        });
        assert_eq!(run.result, Err(SETUP_FAILED.to_string()));
        assert_eq!(run.sleeps, [RETRY_GAP_MS]);
    }

    #[test]
    fn a_raised_attempt_is_logged_and_is_the_reason() {
        let run = trace(2, |_, _| Attempt::Raised {
            error: "RuntimeError('boom')".to_string(),
            diagnosis: Some("not acking".to_string()),
        });
        assert_eq!(run.result, Err("RuntimeError('boom') -- not acking".to_string()));
        assert_eq!(
            run.log,
            [
                "controller :: task 'part' setup raised: RuntimeError('boom')",
                "controller :: task 'part' setup attempt 1/2 failed, retrying",
                "controller :: task 'part' setup raised: RuntimeError('boom')"
            ]
        );
    }

    #[test]
    fn zero_attempts_is_one() {
        let run = trace(0, |_, _| failed("absent"));
        assert_eq!(run.lasts, [true]);
        assert_eq!(run.result, Err(format!("{SETUP_FAILED} -- absent")));
    }

    #[test]
    fn a_diagnosis_before_the_last_attempt_is_ignored() {
        // NEGATIVE: an earlier attempt's diagnosis never reaches the reason, the last one's does
        let run = trace(2, |number, _| {
            if number == 1 {
                failed("early")
            } else {
                Attempt::Failed { diagnosis: None }
            }
        });
        assert_eq!(run.result, Err(SETUP_FAILED.to_string()));
    }

    /// A document holding these sensors and pins.
    fn board(sensors: Value, pins: Value) -> Value {
        json!({"board": {"id": "t"}, "sensors": sensors, "pins": pins})
    }

    /// A laser entry on i2c:0 0x29 feeding `agl`, with extra fields.
    fn laser(name: &str, driver: &str, extra: Value) -> Value {
        let mut entry = json!({"name": name, "driver": driver, "bus": "i2c", "id": 0, "addr": 0x29,
                               "provides": {"agl": {"priority": 0, "timeout_ms": 100}}});
        for (key, value) in extra.as_object().expect("extra fields").iter() {
            entry[key] = value.clone();
        }
        entry
    }

    /// The candidates of a document's directory.
    fn candidates(document: &Value) -> Vec<Candidate<'_>> {
        board_config::directory(document)
            .iter()
            .map(|entry| candidate(document, entry))
            .collect()
    }

    /// Pairs of names: (loser, winner) alternatives, or (name, reason) failures.
    type Named = Vec<(String, String)>;

    /// The alternatives pass over a document: entries named in `up` came up (in that order), the rest failed with
    /// 'why'. Answers the (loser, winner) names and the failures left, by name.
    fn sorted(document: &Value, up: &[&str]) -> (Named, Named) {
        let all = candidates(document);
        let index = |name: &str| all.iter().position(|each| each.name == name).expect("a directory name");
        let up: Vec<usize> = up.iter().map(|name| index(name)).collect();
        let mut failures: Vec<(usize, String)> = (0..all.len())
            .filter(|each| !up.contains(each))
            .map(|each| (each, "why".to_string()))
            .collect();
        let pairs = sort_alternatives(&all, &up, &mut failures);
        let name = |each: usize| all[each].name.to_string();
        (
            pairs
                .into_iter()
                .map(|(loser, winner)| (name(loser), name(winner)))
                .collect(),
            failures
                .into_iter()
                .map(|(each, reason)| (name(each), reason))
                .collect(),
        )
    }

    /// One (loser, winner) pair by name.
    fn pair(loser: &str, winner: &str) -> Named {
        vec![(loser.to_string(), winner.to_string())]
    }

    #[test]
    fn a_candidate_has_a_socket_only_on_i2c_with_an_address() {
        let document = board(
            json!([
                laser("a", "vl53l4cx", json!({})),
                laser("spi", "x", json!({"bus": "spi"})),
                laser("bare", "x", json!({"addr": null})),
                laser("busless", "x", json!({"bus": null})),
                laser("idless", "x", json!({"id": null})),
                laser("text", "x", json!({"id": "1"}))
            ]),
            json!({}),
        );
        let all = candidates(&document);
        let socket = |id: &str| {
            Some(Socket {
                id: id.to_string(),
                address: 0x29,
            })
        };
        assert_eq!(all[0].socket, socket("0"));
        assert_eq!(
            all[0].socket.as_ref().map(ToString::to_string),
            Some("i2c:0 0x29".to_string())
        );
        assert_eq!(
            all[1].socket, None,
            "an SPI entry's addr is its I2C fallback, not a socket"
        );
        assert_eq!(all[2].socket, None, "no address, no socket");
        assert_eq!(all[3].socket, None, "no bus, no socket");
        assert_eq!(all[4].socket, socket("None"), "plan B's str(None)");
        assert_eq!(all[5].socket, socket("1"));
        assert_eq!(all[0].quantities, ["agl"]);
    }

    #[test]
    fn a_candidate_reads_its_reset_pin_through_the_pins_map() {
        let document = board(
            json!([
                laser("routed", "x", json!({"xshut_pin": "xshut"})),
                laser("zero", "x", json!({"xshut_pin": "zero"})),
                laser("nulled", "x", json!({"xshut_pin": "off"})),
                laser("negative", "x", json!({"xshut_pin": "minus"})),
                laser("unnamed", "x", json!({})),
                laser("unknown", "x", json!({"xshut_pin": "nowhere"}))
            ]),
            json!({"xshut": 5, "zero": 0, "off": null, "minus": -1}),
        );
        let gpios: Vec<Option<u8>> = candidates(&document).iter().map(|each| each.reset_gpio).collect();
        assert_eq!(gpios, [Some(5), Some(0), None, None, None, None]);
    }

    #[test]
    fn the_part_that_answers_stands_in_for_the_one_that_does_not() {
        let document = board(
            json!([
                laser("laser_agl", "vl53l4cx", json!({})),
                laser("laser_agl_l1x", "vl53l1x", json!({}))
            ]),
            json!({}),
        );
        let (pairs, failures) = sorted(&document, &["laser_agl_l1x"]);
        assert_eq!(pairs, pair("laser_agl", "laser_agl_l1x"));
        assert_eq!(failures, []);
    }

    #[test]
    fn rule_1_an_spi_entry_or_one_without_an_address_is_never_a_candidate() {
        for (loser, winner) in [
            (json!({"bus": "spi"}), json!({"bus": "spi"})),
            (json!({"addr": null}), json!({"addr": null})),
            (json!({}), json!({"addr": 0x30})),
            (json!({}), json!({"id": 1})),
        ] {
            let document = board(
                json!([
                    laser("loser", "vl53l4cx", loser.clone()),
                    laser("winner", "vl53l1x", winner)
                ]),
                json!({}),
            );
            let (pairs, failures) = sorted(&document, &["winner"]);
            assert_eq!(pairs, [], "{loser}");
            assert_eq!(failures, [("loser".to_string(), "why".to_string())], "{loser}");
        }
    }

    #[test]
    fn rule_2_an_empty_provides_matches_nothing_and_the_set_must_be_the_same() {
        for (loser, winner) in [
            (json!({"provides": {}}), json!({"provides": {}})),
            (json!({"provides": null}), json!({"provides": null})),
            (json!({}), json!({"provides": {"agl": {}, "altitude": {}}})),
            (json!({"provides": {"altitude": {}}}), json!({})),
        ] {
            let document = board(
                json!([
                    laser("loser", "vl53l4cx", loser.clone()),
                    laser("winner", "vl53l1x", winner)
                ]),
                json!({}),
            );
            assert_eq!(sorted(&document, &["winner"]).0, [], "{loser}");
        }
    }

    #[test]
    fn rule_3_the_same_driver_twice_is_not_an_alternative() {
        let document = board(
            json!([
                laser("loser", "vl53l1x", json!({})),
                laser("winner", "vl53l1x", json!({}))
            ]),
            json!({}),
        );
        let (pairs, failures) = sorted(&document, &["winner"]);
        assert_eq!(pairs, []);
        assert_eq!(failures.len(), 1);
    }

    #[test]
    fn rule_4_a_loser_that_pulses_xshut_stays_a_failure_and_says_why() {
        let shared = json!({"xshut_pin": "laser_xshut"});
        for winner_first in [false, true] {
            let mut sensors = vec![
                laser("laser_agl", "vl53l4cx", shared.clone()),
                laser("laser_agl_l1x", "vl53l1x", shared.clone()),
            ];
            if winner_first {
                sensors.reverse();
            }
            let document = board(json!(sensors), json!({"laser_xshut": 5}));
            let (pairs, failures) = sorted(&document, &["laser_agl_l1x"]);
            assert_eq!(pairs, []);
            assert_eq!(
                failures,
                [(
                    "laser_agl".to_string(),
                    "why; shares XSHUT GPIO5 with laser_agl_l1x (vl53l1x): its setup resets the fitted laser -- \
                     enable exactly one of laser_agl, laser_agl_l1x on this board"
                        .to_string()
                )]
            );
        }
        // POSITIVE: only the loser's own pin counts -- the winner pulsing it never resets the winner's socket twice
        let winner_only = board(
            json!([
                laser("laser_agl", "vl53l4cx", json!({})),
                laser("laser_agl_l1x", "vl53l1x", shared)
            ]),
            json!({"laser_xshut": 5}),
        );
        assert_eq!(
            sorted(&winner_only, &["laser_agl_l1x"]).0,
            pair("laser_agl", "laser_agl_l1x")
        );
    }

    #[test]
    fn the_first_up_winner_in_setup_order_is_taken() {
        let document = board(
            json!([
                laser("loser", "vl53l4cx", json!({})),
                laser("first", "vl53l1x", json!({})),
                laser("second", "other", json!({}))
            ]),
            json!({}),
        );
        assert_eq!(sorted(&document, &["second", "first"]).0, pair("loser", "second"));
        assert_eq!(sorted(&document, &["first", "second"]).0, pair("loser", "first"));
    }

    #[test]
    fn an_index_past_the_directory_names_no_candidate() {
        let document = board(
            json!([
                laser("loser", "vl53l4cx", json!({})),
                laser("winner", "vl53l1x", json!({}))
            ]),
            json!({}),
        );
        let all = candidates(&document);
        let mut failures = vec![(0, "why".to_string()), (9, "stray".to_string())];
        assert_eq!(sort_alternatives(&all, &[7, 1], &mut failures), [(0, 1)]);
        assert_eq!(failures, [(9, "stray".to_string())]);
    }

    #[test]
    fn the_operator_lines_read_as_plan_bs() {
        let socket = Socket {
            id: "1".to_string(),
            address: 0x29,
        };
        assert_eq!(
            unfitted("vl53l4cx", "laser_agl_l1x", "vl53l1x", &socket, false),
            "vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:1 0x29"
        );
        assert_eq!(
            unfitted("vl53l4cx", "laser_agl_l1x", "vl53l1x", &socket, true),
            "vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answered i2c:1 0x29 at setup, now down"
        );
        assert_eq!(named("bmp581", "up"), "bmp581 -- up");
        assert_eq!(
            named("", "probe FAILED"),
            "probe FAILED",
            "no implementation: the verdict alone"
        );
        assert_eq!(
            failure_line("vl53l1x", "absent", false),
            "vl53l1x -- not connected: absent"
        );
        assert_eq!(
            failure_line("vl53l1x", "run loop crashed", true),
            "vl53l1x -- run loop crashed"
        );
    }

    /// The table for the laser pair plus a failing UART part: the L1X up, the L4CX reading the other part.
    fn laser_table(document: &Value) -> Table<'_> {
        let outcomes = [
            Err(format!("{SETUP_FAILED} -- wrong part")),
            Ok(()),
            Err(SETUP_FAILED.to_string()),
        ];
        Table::new(candidates(document).into_iter().zip(outcomes).collect())
    }

    /// The laser pair and a UART GNSS.
    fn laser_board() -> Value {
        board(
            json!([laser("laser_agl", "vl53l4cx", json!({})), laser("laser_agl_l1x", "vl53l1x", json!({})),
                   {"name": "gnss", "driver": "neo6mv2", "bus": "uart", "id": 2, "provides": {"position": {}}}]),
            json!({}),
        )
    }

    #[test]
    fn the_table_sorts_its_bring_up() {
        let document = laser_board();
        let table = laser_table(&document);
        let statuses: Vec<Status> = table.entries().iter().map(|entry| entry.status()).collect();
        assert_eq!(
            statuses,
            [Status::Unfitted(1), Status::Up, Status::Failed(SETUP_FAILED)]
        );
        assert_eq!(table.tasks(), [("laser_agl_l1x", true)]);
        assert_eq!(
            table.failures(),
            [("gnss", format!("neo6mv2 -- not connected: {SETUP_FAILED}"))]
        );
        let line = "vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:0 0x29".to_string();
        assert_eq!(table.alternatives(), [("laser_agl", line.clone())]);
        assert_eq!(table.unfitted("laser_agl"), Some(line.clone()));
        assert_eq!(table.unfitted("laser_agl_l1x"), None, "an up part is no alternative");
        assert_eq!(
            table.devices(),
            [
                ("laser_agl", line),
                ("laser_agl_l1x", "vl53l1x -- up".to_string()),
                ("gnss", format!("neo6mv2 -- down: {SETUP_FAILED}"))
            ]
        );
        assert_eq!(
            table.not_up_line(),
            Some("controller :: 1 device(s) not up: gnss".to_string())
        );
        assert_eq!(
            table.bring_up_lines(),
            [
                "controller :: laser_agl: vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:0 0x29",
                "controller :: 1 device(s) not up: gnss"
            ]
        );
        assert_eq!(
            table.inspect(),
            Inspection {
                tasks: vec!["laser_agl_l1x"],
                failures: vec![("gnss", SETUP_FAILED)],
                alternatives: vec![("laser_agl", "laser_agl_l1x")],
            },
            "the bare reason and the winner's name, not CC's lines"
        );
    }

    #[test]
    fn a_board_with_every_part_up_has_no_summary_line() {
        let document = laser_board();
        let table = Table::new(candidates(&document).into_iter().map(|each| (each, Ok(()))).collect());
        assert_eq!(table.not_up_line(), None);
        assert_eq!(
            table.bring_up_lines(),
            Vec::<String>::new(),
            "nothing to say after the per-task lines"
        );
        assert_eq!(table.failures(), []);
        assert_eq!(table.alternatives(), []);
        let inspection = table.inspect();
        assert_eq!(inspection.tasks, ["laser_agl", "laser_agl_l1x", "gnss"]);
        assert_eq!(inspection.failures, []);
        assert_eq!(inspection.alternatives, []);
    }

    #[test]
    fn a_board_with_no_alternative_ends_with_the_summary_alone() {
        // NEGATIVE: a failed loser with no winner is a failure, so no not-fitted line comes before the summary
        let document = laser_board();
        let outcomes = [Err(SETUP_FAILED.to_string()), Err(SETUP_FAILED.to_string()), Ok(())];
        let table = Table::new(candidates(&document).into_iter().zip(outcomes).collect());
        assert_eq!(
            table.bring_up_lines(),
            ["controller :: 2 device(s) not up: laser_agl, laser_agl_l1x"]
        );
        let inspection = table.inspect();
        assert_eq!(inspection.tasks, ["gnss"]);
        assert_eq!(
            inspection.failures,
            [("laser_agl", SETUP_FAILED), ("laser_agl_l1x", SETUP_FAILED)]
        );
        assert_eq!(inspection.alternatives, []);
    }

    #[test]
    fn a_crash_marks_only_an_up_part_and_only_once() {
        let document = laser_board();
        let table = laser_table(&document);
        let [loser, winner, gnss] = table.entries() else {
            panic!("three entries")
        };
        // NEGATIVE: a part that never came up, or an alternative, cannot crash
        assert!(!gnss.crash("loop died".to_string()));
        assert!(!loser.crash("loop died".to_string()));
        assert!(winner.crash("run loop crashed: first".to_string()));
        assert!(
            !winner.crash("run loop crashed: second".to_string()),
            "the first reason stays"
        );
        assert_eq!(winner.status(), Status::Crashed("run loop crashed: first"));
        assert!(winner.active(), "a crashed part is still listed in health.tasks");
        assert_eq!(table.tasks(), [("laser_agl_l1x", false)]);
        assert_eq!(
            table.unfitted("laser_agl"),
            Some("vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answered i2c:0 0x29 at setup, now down".to_string())
        );
        assert_eq!(
            table.failure("laser_agl_l1x"),
            Some("vl53l1x -- run loop crashed: first".to_string()),
            "a crashed part was connected"
        );
        assert_eq!(
            table.not_up_line(),
            Some("controller :: 2 device(s) not up: gnss, laser_agl_l1x".to_string())
        );
        // a crashed part is still a task, and in failures with its bare reason; its loser keeps naming it
        assert_eq!(
            table.inspect(),
            Inspection {
                tasks: vec!["laser_agl_l1x"],
                failures: vec![("laser_agl_l1x", "run loop crashed: first"), ("gnss", SETUP_FAILED)],
                alternatives: vec![("laser_agl", "laser_agl_l1x")],
            }
        );
    }

    #[test]
    fn an_up_part_starts_healthy_and_its_producer_flips_it_on_a_change_only() {
        let document = laser_board();
        let table = laser_table(&document);
        let [loser, winner, gnss] = table.entries() else {
            panic!("three entries")
        };
        assert!(winner.healthy(), "an up part starts healthy, as plan B's Task does");
        assert!(winner.set_healthy(false), "a failed read is a change");
        assert!(!winner.set_healthy(false), "failing again changes nothing");
        assert!(!winner.healthy());
        assert!(winner.set_healthy(true), "the next good read recovers it");
        assert!(!winner.set_healthy(true));
        assert!(winner.healthy());
        // the runtime health is not setup's: an unhealthy part is still up, a task, and no failure
        winner.set_healthy(false);
        assert_eq!(winner.status(), Status::Up);
        assert_eq!(table.tasks(), [("laser_agl_l1x", true)]);
        assert_eq!(table.failure("laser_agl_l1x"), None);
        // NEGATIVE: a part that never came up, and an alternative, are not healthy, and no write makes them so
        assert!(!gnss.healthy());
        assert!(!loser.healthy());
        assert!(!gnss.set_healthy(true), "a failed part has no reads to be healthy by");
        assert!(!loser.set_healthy(true), "nor has an alternative");
        assert!(!gnss.healthy());
        assert!(!loser.healthy());
    }

    #[test]
    fn a_producer_thread_s_health_reaches_a_reader() {
        // the firmware's producers write from the bus owners and the control thread, CC reads on the executor: the
        // table is shared by reference, so it must be Sync (this does not compile otherwise)
        let document = laser_board();
        let table = laser_table(&document);
        std::thread::scope(|scope| {
            scope.spawn(|| table.find("laser_agl_l1x").map(|entry| entry.set_healthy(false)));
        });
        assert_eq!(table.find("laser_agl_l1x").map(|entry| entry.healthy()), Some(false));
    }

    #[test]
    fn a_probe_result_is_named_and_a_failed_part_never_probes_healthy() {
        let document = laser_board();
        let table = laser_table(&document);
        assert_eq!(table.probed("laser_agl_l1x", None), None);
        assert_eq!(
            table.probed("laser_agl_l1x", Some("no ack")),
            Some("vl53l1x -- no ack".to_string())
        );
        assert_eq!(
            table.probed("gnss", None),
            Some(format!("neo6mv2 -- not connected: {SETUP_FAILED}"))
        );
        assert_eq!(table.probed("laser_agl", None), None, "an alternative is not a failure");
        assert_eq!(table.probed("mission", Some("no fix")), Some("no fix".to_string()));
        assert_eq!(table.probed("mission", None), None);
        assert_eq!(table.failure("laser_agl_l1x"), None, "an up part has no failure line");
        assert_eq!(table.failure("nosuch"), None);
    }

    /// The binding of a single-entry document's entry.
    fn bound(entry: Value, default_kind: &str, default_address: Option<u8>) -> Binding {
        let document = json!({"sensors": [entry]});
        let directory = board_config::directory(&document);
        binding(&directory[0], default_kind, default_address)
    }

    /// An I2C binding.
    fn i2c(id: &str, address: Option<u8>) -> Binding {
        Binding::I2c {
            id: id.to_string(),
            address,
        }
    }

    /// An SPI binding.
    fn spi(id: &str) -> Binding {
        Binding::Spi { id: id.to_string() }
    }

    #[test]
    fn binding_applies_plan_bs_defaults() {
        assert_eq!(bound(json!({"name": "a"}), "i2c", Some(0x63)), i2c("0", Some(0x63)));
        assert_eq!(bound(json!({"name": "a"}), "spi", Some(0x6a)), spi("1"));
        assert_eq!(
            bound(
                json!({"name": "a", "bus": "i2c", "id": 1, "addr": 0x29}),
                "i2c",
                Some(0x52)
            ),
            i2c("1", Some(0x29))
        );
        // the driver's own default names the transport only when the entry names none
        assert_eq!(bound(json!({"name": "a", "bus": "spi"}), "i2c", Some(0x53)), spi("1"));
        assert_eq!(
            bound(json!({"name": "a", "bus": "i2c"}), "spi", Some(0x6a)),
            i2c("0", Some(0x6a))
        );
        // the id as config.bus looks it up: str(id)
        assert_eq!(bound(json!({"name": "a", "id": "1"}), "i2c", None), i2c("1", None));
        assert_eq!(bound(json!({"name": "a", "id": 1.0}), "i2c", None), i2c("1.0", None));
    }

    #[test]
    fn an_explicit_null_is_not_absent() {
        // NEGATIVE: a null id finds no bus; a null addr leaves the driver without one
        assert_eq!(
            bound(json!({"name": "a", "id": null}), "i2c", Some(0x63)),
            Binding::Unbound
        );
        assert_eq!(
            bound(json!({"name": "a", "bus": "spi", "id": null}), "i2c", None),
            Binding::Unbound
        );
        assert_eq!(
            bound(json!({"name": "a", "addr": null}), "i2c", Some(0x63)),
            i2c("0", None)
        );
        assert_eq!(
            bound(json!({"name": "a", "addr": 0x80}), "i2c", Some(0x63)),
            i2c("0", None)
        );
    }

    #[test]
    fn a_bus_that_is_no_driver_transport_binds_nothing() {
        for bus in [json!(null), json!("uart"), json!(5), json!("I2C")] {
            assert_eq!(
                bound(json!({"name": "a", "bus": bus}), "i2c", Some(0x63)),
                Binding::Unbound,
                "{bus}"
            );
        }
    }
}
