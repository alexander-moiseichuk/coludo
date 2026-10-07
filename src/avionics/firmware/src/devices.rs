//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The device bring-up (doc/specs/avionics-port.md M5, "The registry"; plan B's Controller.setup, controller.py:
//! 353-420): every enabled entry of the post-layout config, sensors then components, in config order, one at a time,
//! on main while nothing else runs. crates/controller holds the rules (the attempts and their gaps, the diagnose on
//! the last, the alternatives pass, the operator's lines); this module makes each attempt on the hardware
//! (crate::sensors), keeps the parts that came up for their bus owners, and builds the databoard topology from them.
//!
//! - An I2C part runs on the port its entry binds, lent to main by its owner (crate::owner); the parts go back with
//!   the port when the owner starts.
//! - SPI: the chip select of every part with no driver yet is held high before any SPI part is set up (a floating
//!   select could fight MISO, plan B spibus.py:42); the LSM6DSO32 is set up on one handle reused across its attempts
//!   (the first-use discard follows the peripheral, not the handle). Its owner is M6's control thread.
//! - A general call during a setup (the ICP-10111 frees a latched bus with one) re-arms every part of that port
//!   already up, the sender excepted, and is written as an oops record (§4.8).
//! - The topology: one Parameter per quantity with a provider that came up, its sources registered in setup order
//!   (plan B's `provide` at the end of each successful setup), then frozen.

use std::time::{Duration, Instant};

use board_config::Entry;
use controller::{Attempt, Binding, Table};
use databoard::Parameter;
use drivers::lsm6dso32::{Lsm6dso32, OverSpi};
use esp_idf_svc::hal::delay::Delay;
use firmware::buses::i2c::I2cPort;
use firmware::buses::spi::{SpiBus, SpiHandle};
use serde_json::Value;

use crate::owner;
use crate::sensors::{self, Kind, Part};

/// An I2C part that came up, as its port's owner keeps it.
pub struct Member {
    /// The entry's name.
    pub name: &'static str,
    /// The driver.
    pub part: Part,
}

/// The LSM6DSO32 on SPI2 once it came up: the control thread, SPI2's owner, takes it (crate::control).
pub struct SpiMember {
    /// The entry's name.
    pub name: &'static str,
    /// Its chip select's handle.
    pub handle: SpiHandle,
    /// The driver.
    pub part: Lsm6dso32<OverSpi>,
    /// Its INT1 data-ready GPIO, when the config wires one (`int_pin`).
    pub interrupt: Option<u8>,
}

/// The quantities scalars are (D14 of the M5 plan): one f32 each.
const SCALARS: [&str; 12] = [
    "altitude",
    "elevation",
    "pressure",
    "temperature",
    "airspeed",
    "dynamic_pressure",
    "agl",
    "voltage",
    "current",
    "power",
    "speed",
    "course",
];

/// The quantities that are vectors: three f32 (plan B pushes 3-tuples).
const VECTORS: [&str; 4] = ["accel", "rate", "mag", "attitude"];

/// The quantities that are positions: latitude and longitude in 1e-7 degrees.
const POSITIONS: [&str; 1] = ["position"];

/// The databoard's quantities, by sample type, each with its sources: built once at bring-up, then shared
/// read-only.
///
/// Each Parameter is boxed on its own (~0.42 KB), so every one is allocated internal, the flight readers' memory
/// (§4.7): a run of them in one Vec would pass SPIRAM_MALLOC_ALWAYSINTERNAL (4 KB) at ten scalars and land in PSRAM.
#[expect(
    clippy::vec_box,
    reason = "each Parameter is boxed for its memory: see the type's notes"
)]
pub struct Topology {
    /// The scalar quantities.
    scalars: Vec<Box<Parameter<f32>>>,
    /// The vector quantities.
    vectors: Vec<Box<Parameter<[f32; 3]>>>,
    /// The positions.
    positions: Vec<Box<Parameter<[i32; 2]>>>,
}

impl Topology {
    /// The vector quantity `quantity` (accel, rate, mag, attitude), when a provider of it came up.
    pub fn vector(&self, quantity: &str) -> Option<&Parameter<[f32; 3]>> {
        self.vectors
            .iter()
            .find(|parameter| parameter.name() == quantity)
            .map(|parameter| &**parameter)
    }

    /// The topology of the entries that came up, in setup order, with one log line per quantity: its sources in rank
    /// order with their priorities, and its shared window. M6 adds the readers.
    fn build(entries: &[Entry<'static>], table: &Table) -> Topology {
        let mut topology = Topology {
            scalars: Vec::new(),
            vectors: Vec::new(),
            positions: Vec::new(),
        };
        let up = entries
            .iter()
            .zip(table.entries())
            .filter(|(_, device)| device.active());
        for (entry, _) in up {
            for provide in board_config::provides(entry) {
                let quantity = provide.quantity;
                let added = if SCALARS.contains(&quantity) {
                    add(&mut topology.scalars, entry.name, &provide)
                } else if VECTORS.contains(&quantity) {
                    add(&mut topology.vectors, entry.name, &provide)
                } else if POSITIONS.contains(&quantity) {
                    add(&mut topology.positions, entry.name, &provide)
                } else {
                    log::warn!(
                        "databoard :: '{quantity}' of {} is no quantity the board knows: skipped",
                        entry.name
                    );
                    continue;
                };
                if !added {
                    log::warn!(
                        "databoard :: {quantity} has {} sources already: {} not added",
                        databoard::MAXIMUM_SOURCES,
                        entry.name
                    );
                }
            }
        }
        for line in topology
            .scalars
            .iter()
            .map(|parameter| describe(parameter))
            .chain(topology.vectors.iter().map(|parameter| describe(parameter)))
            .chain(topology.positions.iter().map(|parameter| describe(parameter)))
        {
            log::info!("{line}");
        }
        topology
    }
}

/// Registers `source` for `provide`'s quantity in `parameters`, making the quantity on its first source. Whether it
/// was added (a quantity holds [`databoard::MAXIMUM_SOURCES`] at most).
#[expect(
    clippy::vec_box,
    reason = "each Parameter is boxed for its memory: see Topology's notes"
)]
fn add<T: databoard::Sample>(
    parameters: &mut Vec<Box<Parameter<T>>>,
    source: &'static str,
    provide: &board_config::Provide<'static>,
) -> bool {
    let index = match parameters
        .iter()
        .position(|parameter| parameter.name() == provide.quantity)
    {
        Some(index) => index,
        None => {
            parameters.push(Box::new(Parameter::new(provide.quantity)));
            parameters.len() - 1
        }
    };
    parameters[index]
        .add_source(source, provide.priority, provide.timeout_ms, provide.reconcile)
        .is_ok()
}

/// A quantity's log line: `databoard :: pressure <- baro_icp10111 p0, baro_bmp581 p1; window 200 ms`.
fn describe<T: databoard::Sample>(parameter: &Parameter<T>) -> String {
    let sources: Vec<String> = parameter
        .sources()
        .map(|source| format!("{source} p{}", parameter.rank(source).unwrap_or_default()))
        .collect();
    format!(
        "databoard :: {} <- {}; window {} ms",
        parameter.name(),
        sources.join(", "),
        parameter.window_us() / 1000
    )
}

/// What the bring-up leaves.
pub struct Devices {
    /// Every directory entry and how its bring-up ended, for CC's readers.
    pub table: &'static Table<'static>,
    /// The databoard's quantities.
    pub topology: &'static Topology,
    /// The I2C parts that came up, by port.
    pub members: [Vec<Member>; 2],
    /// The SPI parts that came up.
    pub spi: Vec<SpiMember>,
    /// SPI2, when an SPI part is enabled: it lives for the program.
    pub spi_bus: Option<SpiBus>,
}

/// Brings every enabled entry of `document` (the post-layout config) up, in config order, on the ports main holds on
/// loan (`ports`), and logs plan B's lines and the topology. `recording` is whether the recorder opened: its entry
/// is up only then.
pub fn bring_up(document: &'static Value, ports: &mut [(owner::Parked, I2cPort)], recording: bool) -> Devices {
    let started = Instant::now();
    let entries = board_config::directory(document);
    let attempts = board_config::setup_retries(document);
    let mut delay = Delay::new_default();
    let mut members: [Vec<Member>; 2] = [Vec::new(), Vec::new()];
    let mut spi = Vec::new();
    let spi_bus = open_spi(document, &entries);
    let mut outcomes = Vec::with_capacity(entries.len());
    for entry in &entries {
        let log = |line: String| log::info!("{line}");
        let sleep = |ms: u32| std::thread::sleep(Duration::from_millis(u64::from(ms)));
        let outcome = match sensors::classify(entry) {
            Kind::Native => {
                let answer = |_, _| {
                    if entry.implementation != "recorder" || recording {
                        Attempt::Up
                    } else {
                        Attempt::Raised {
                            error: "not recording this boot".to_string(),
                            diagnosis: None,
                        }
                    }
                };
                controller::set_up(entry.name, entry.implementation, 1, answer, sleep, log)
            }
            Kind::NotPorted => controller::set_up(
                entry.name,
                entry.implementation,
                1,
                |_, _| Attempt::NoDriver,
                sleep,
                log,
            )
            .map_err(|_| sensors::NOT_PORTED_YET.to_string()),
            Kind::Unknown => controller::set_up(
                entry.name,
                entry.implementation,
                1,
                |_, _| Attempt::NoDriver,
                sleep,
                log,
            ),
            Kind::I2c => set_up_i2c(document, entry, attempts, ports, &mut members, &mut delay),
            Kind::Spi => set_up_spi(document, entry, attempts, spi_bus.as_ref(), &mut spi, &mut delay),
        };
        outcomes.push((controller::candidate(document, entry), outcome));
    }
    let table: &'static Table<'static> = Box::leak(Box::new(Table::new(outcomes)));
    for line in table.bring_up_lines() {
        log::info!("{line}");
    }
    log::info!("controller :: bring-up {} ms", started.elapsed().as_millis());
    let topology: &'static Topology = Box::leak(Box::new(Topology::build(&entries, table)));
    Devices {
        table,
        topology,
        members,
        spi,
        spi_bus,
    }
}

/// SPI2 for the SPI parts the port drives, after the chip select of every other SPI part was held high; None when
/// no SPI part is to be set up, or the bus would not open (their setups then fail with no transport).
fn open_spi(document: &Value, entries: &[Entry]) -> Option<SpiBus> {
    let on_spi = |entry: &&Entry| entry.bus == Some("spi");
    for entry in entries.iter().filter(on_spi) {
        if sensors::classify(entry) == Kind::Spi {
            continue;
        }
        let Some(chip_select) = board_config::pin(document, entry, "cs_pin") else {
            continue;
        };
        match firmware::buses::spi::hold_high(chip_select) {
            Ok(()) => log::info!("spi:1 {} :: CS GPIO{chip_select} held high: no driver yet", entry.name),
            Err(error) => log::warn!("spi:1 {} :: CS GPIO{chip_select} not held high: {error}", entry.name),
        }
    }
    if !entries.iter().any(|entry| sensors::classify(entry) == Kind::Spi) {
        return None;
    }
    match board_config::spi_bus(document, 1).map(SpiBus::open) {
        Some(Ok(bus)) => Some(bus),
        Some(Err(error)) => {
            log::error!("spi:1 :: no bus: {error}");
            None
        }
        None => None,
    }
}

/// One I2C entry's bring-up on its port: a fresh part per attempt, set up; a general call it sent re-arms the
/// port's members already up; the part that came up joins them.
fn set_up_i2c(
    document: &Value,
    entry: &Entry<'static>,
    attempts: u32,
    ports: &mut [(owner::Parked, I2cPort)],
    members: &mut [Vec<Member>; 2],
    delay: &mut Delay,
) -> Result<(), String> {
    let (port_id, address) = match controller::binding(entry, "i2c", sensors::default_address(entry.implementation)) {
        Binding::I2c { id, address } => (id.parse::<u8>().ok(), address),
        Binding::Spi { .. } | Binding::Unbound => (None, None),
    };
    // the port only when the config defines its bus: port 0 runs on the stock pins without one (the camera's), and
    // plan B had no bus for its parts then
    let mut port = port_id
        .filter(|id| board_config::i2c_bus(document, *id).is_some())
        .and_then(|id| ports.iter_mut().find(|(_, port)| port.id() == id).map(|(_, port)| port));
    let mut came_up = None;
    let mut calls = 0;
    let mut rearmed = Vec::new();
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        attempts,
        |_, last| {
            let Some(port) = port.as_deref_mut() else {
                return Attempt::Failed {
                    diagnosis: last.then(|| no_transport(entry)),
                };
            };
            let Some(address) = address else {
                return Attempt::Failed {
                    diagnosis: last.then(|| "no address -- addr is null or not a 7-bit int".to_string()),
                };
            };
            let mut part = match Part::build(document, entry, address) {
                Ok(part) => part,
                Err(error) => return Attempt::Raised { error, diagnosis: None },
            };
            let result = part.setup(port, delay);
            // the port sees any general call on it, the part only its own: ask both, so both forget it
            if port.take_general_call() | part.take_general_call() {
                let index = usize::from(port.id());
                members[index].sort_by(|left, right| left.name.cmp(right.name));
                calls += 1;
                rearmed.extend(owner::rearm(port, &mut members[index], entry.name, delay));
            }
            match result {
                Ok(()) => {
                    came_up = Some(part);
                    Attempt::Up
                }
                Err(fault) => {
                    log::debug!("{} :: setup: {fault}", entry.name);
                    Attempt::Failed {
                        diagnosis: last.then(|| part.diagnose(port)),
                    }
                }
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    // one record for the entry's bring-up, however many attempts called
    if let (Some(id), true) = (port_id, calls > 0) {
        owner::record(id, entry.name, calls, &rearmed);
    }
    if let (Some(part), Some(id)) = (came_up, port_id) {
        members[usize::from(id)].push(Member { name: entry.name, part });
    }
    outcome
}

/// Plan B's diagnose text for a part whose transport was never built: the I2C drivers' (bmp581.py:211, ...), and the
/// LSM6DSO32's own, which names the SPI fields whatever its binding (lsm6dso32.py:295-297).
fn no_transport(entry: &Entry) -> String {
    let field = |key: &str, absent: &str| {
        entry
            .fields
            .get(key)
            .map_or_else(|| absent.to_string(), board_config::python::formatted)
    };
    if entry.implementation == "lsm6dso32" {
        format!(
            "no transport -- bus {}:{} undefined or cs_pin {} unwired",
            field("bus", "None"),
            field("id", "None"),
            field("cs_pin", "None")
        )
    } else {
        format!("no transport -- i2c bus {} undefined in config", field("id", "0"))
    }
}

/// The LSM6DSO32's bring-up on SPI2: one handle on its chip select for every attempt.
fn set_up_spi(
    document: &Value,
    entry: &Entry<'static>,
    attempts: u32,
    bus: Option<&SpiBus>,
    members: &mut Vec<SpiMember>,
    delay: &mut Delay,
) -> Result<(), String> {
    let bound = matches!(
        controller::binding(entry, "spi", Some(drivers::lsm6dso32::ADDRESS)),
        Binding::Spi { id } if id == "1"
    );
    let chip_select = board_config::pin(document, entry, "cs_pin");
    let mut handle = match (bound, bus, chip_select) {
        (true, Some(bus), Some(chip_select)) => bus
            .handle(chip_select)
            .inspect_err(|error| log::warn!("spi:1 {} :: CS GPIO{chip_select} not on the bus: {error}", entry.name))
            .ok(),
        _ => None,
    };
    let interrupt = board_config::pin(document, entry, "int_pin");
    let interrupt_wired = interrupt.is_some();
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        attempts,
        |_, last| {
            let Some(handle) = handle.as_mut() else {
                return Attempt::Failed {
                    diagnosis: last.then(|| no_transport(entry)),
                };
            };
            let mut part = Lsm6dso32::new(OverSpi, interrupt_wired);
            match drivers::device::Device::setup(&mut part, handle, delay) {
                Ok(()) => {
                    came_up = Some(part);
                    Attempt::Up
                }
                Err(fault) => {
                    log::debug!("{} :: setup: {fault}", entry.name);
                    Attempt::Failed {
                        diagnosis: last.then(|| part.diagnose(handle)),
                    }
                }
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    if let (Some(part), Some(handle)) = (came_up, handle) {
        members.push(SpiMember {
            name: entry.name,
            handle,
            part,
            interrupt,
        });
    }
    outcome
}
