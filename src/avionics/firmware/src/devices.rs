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
//! - The stage machine (M7): the first enabled `sequencer` entry's settings, read by sequencer::Settings::from_config;
//!   a key it refuses fails the entry, naming the key, so `arm` refuses. Its steps are the control thread's.
//! - The checkpoint and the watchdog (M8): the first enabled `checkpoint` entry's cadence and the first enabled
//!   `watchdog` entry's settings, read by board_config's views, which refuse a key plan B could not run with and so
//!   fail the entry. Both run on the supervisor (crate::warm, crate::supervisor); a refused watchdog leaves the task
//!   watchdog at IDF's warn-only default.
//! - The topology: one Parameter per quantity with a provider that came up, its sources registered in setup order
//!   (plan B's `provide` at the end of each successful setup), then frozen.

use std::time::{Duration, Instant};

use board_config::Entry;
use controller::{Attempt, Binding, Table};
use databoard::Parameter;
use drivers::axes::Axes;
use drivers::lsm6dso32::{Lsm6dso32, OverSpi};
use drivers::separation::Separation;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::sys;
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
    /// What it runs on the owner's tick, from the owner's start (crate::owner).
    pub run: Option<sensors::Run>,
    /// How its axes map onto the airframe's (`axes`, M9): what the BMI323's run applies; the identity for a part
    /// with no vectors, and for the BMM350, which maps its own after its calibration.
    pub axes: Axes,
    /// Its data-ready GPIO, when the config wires one (`int_pin`): the BMI323's INT1, counted as evidence (M9).
    pub interrupt: Option<u8>,
    /// Its read period, ms (`period_ms`), when the config sets one: the SDP810's, the INA226's and the lasers' runs
    /// take their driver's default otherwise (M10).
    pub period_ms: Option<u32>,
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
    /// Its sampling period, ms (`period_ms`, plan B's 10; at least 1).
    pub period_ms: u32,
    /// How its axes map onto the airframe's (`axes`, M9).
    pub axes: Axes,
}

/// The separation switch once it came up: the control thread reads it every tick (crate::control).
pub struct SeparationMember {
    /// The entry's name.
    pub name: &'static str,
    /// Its GPIO.
    pub gpio: u8,
    /// The debounced latch.
    pub part: Separation,
}

/// The stage machine once its entry came up: the control thread steps it (crate::control).
pub struct SequencerMember {
    /// The entry's name: its stream's (`<name>.csv`) and its log lines'.
    pub name: &'static str,
    /// Its thresholds and step period, read from the entry.
    pub settings: sequencer::Settings,
}

/// The attitude filter once its entry came up: the control thread steps it (crate::control, M9).
pub struct AttitudeMember {
    /// The entry's name: the databoard source of `attitude`, its log lines' and its stream's.
    pub name: &'static str,
    /// Its knobs (crates/control `attitude::Settings`), its rank among them.
    pub settings: control::attitude::Settings,
}

/// The checkpoint once its entry came up: the supervisor runs it (crate::warm::Keeping).
#[derive(Clone, Copy)]
pub struct CheckpointMember {
    /// The entry's name: its log lines' (its stream is checkpoint.csv whatever the name).
    pub name: &'static str,
    /// The airborne cadence in seconds (`checkpoint_s`, at least 1).
    pub checkpoint_s: f32,
}

/// The watchdog once its entry came up: the supervisor runs it (crate::supervisor).
pub struct WatchdogMember {
    /// The entry's name: its log lines'.
    pub name: &'static str,
    /// The task watchdog's timeout, the stall check's period and the stall limit.
    pub settings: board_config::Watchdog,
}

/// The GNSS once its entry came up: its thread takes it with UART2 (crate::gnss, M11).
pub struct GnssMember {
    /// The entry's name: the source of its quantities and its streams' prefix.
    pub name: &'static str,
    /// The module.
    pub module: crate::gnss::Module,
    /// The fix rate asked of the module (`hz`, plan B's default 1).
    pub hz: i32,
    /// Its UART, `buses.uart.<id>`.
    pub uart: board_config::UartSpecification,
}

/// The pad drift calibration once its entry came up: the supervisor passes it (crate::gnss::Calibration, M11).
#[derive(Clone, Copy)]
pub struct GnssCalibMember {
    /// The entry's name: its log lines'.
    pub name: &'static str,
    /// Fewer samples freeze a zero drift (`min_samples`, plan B's 5).
    pub min_samples: u32,
    /// The pass period, ms (`period_ms`, plan B's 1000; at least 1).
    pub period_ms: u32,
}

/// The CC-less field agent once its entry came up: the executor ticks it (crate::commands field, M11).
pub struct FieldMember {
    /// The entry's name: its log lines'.
    pub name: &'static str,
    /// Its knobs, read from the entry.
    pub settings: mission::field::Settings,
}

/// The HITL sim-link once its entry came up: its thread binds the port (crate::sim_link, M12).
pub struct HitlMember {
    /// The entry's name: the source of every simulated quantity.
    pub name: &'static str,
    /// The UDP port the host sends to (`sim_port`).
    pub port: u16,
}

/// The sim-link's UDP port when the entry names none (doc/specs/sim-link.md).
const SIM_PORT: u16 = 1236;

/// The quantities the `hitl` entry provides (plan B hitl.py's `Databoard.provide` at setup, its entry has no
/// `provides`), each at priority 0 with plan B's 1 s timeout; `change` the port's own, as every barometer's.
const HITL_QUANTITIES: [&str; 13] = [
    "accel",
    "rate",
    "attitude",
    "mag",
    "agl",
    "altitude",
    "elevation",
    "change",
    "position",
    "speed",
    "course",
    "airspeed",
    "dynamic_pressure",
];

/// The `hitl` source's timeout, ms (plan B hitl.py `provided`).
const HITL_TIMEOUT_MS: u32 = 1000;

/// The UART the GNSS runs on: the one the firmware hands its thread (plan B's default `id`).
const GNSS_UART: u8 = 2;

/// The LSM6DSO32's sampling period when its entry gives none, ms (plan B lsm6dso32.py:39 `period_ms` 10).
const IMU_PERIOD_MS: u32 = 10;

/// The pin name plan B's separation driver takes when its entry names none (`_pin_gpio('pin', 'separation_switch')`).
const SEPARATION_PIN: &str = "separation_switch";

/// The quantities scalars are (D14 of the M5 plan): one f32 each. `change` is the port's own (M9): each barometer's
/// climb rate, registered with its `elevation` ([`BAROMETERS`]).
const SCALARS: [&str; 13] = [
    "altitude",
    "elevation",
    "change",
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

/// The barometers: each one that provides `elevation` also provides `change` (m/s, climb positive; M9, the operator's
/// fifth barometer quantity) at that elevation's priority and timeout, with no entry in the board config, so no board
/// file or `config_id` changes; never reconciled (a rate has no offset).
const BAROMETERS: [&str; 3] = ["icp10111", "bmp581", "bmp280"];

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
    /// The scalar quantity `quantity` (altitude, pressure, ...), when a provider of it came up.
    pub fn scalar(&self, quantity: &str) -> Option<&Parameter<f32>> {
        self.scalars
            .iter()
            .find(|parameter| parameter.name() == quantity)
            .map(|parameter| &**parameter)
    }

    /// The vector quantity `quantity` (accel, rate, mag, attitude), when a provider of it came up.
    pub fn vector(&self, quantity: &str) -> Option<&Parameter<[f32; 3]>> {
        self.vectors
            .iter()
            .find(|parameter| parameter.name() == quantity)
            .map(|parameter| &**parameter)
    }

    /// The position quantity `quantity` (`position`, 1e-7 degree), when a provider of it came up (M11).
    pub fn position(&self, quantity: &str) -> Option<&Parameter<[i32; 2]>> {
        self.positions
            .iter()
            .find(|parameter| parameter.name() == quantity)
            .map(|parameter| &**parameter)
    }

    /// Plan B's `inspect databoard` (databoard.py:506-511): each quantity's fused reading at `now_us` -- its value
    /// (a scalar, a vector, a position in degrees), the source that won, its age in ms, all null when no source is
    /// fresh -- and the learned reconcile offsets by source.
    pub fn inspect(&self, now_us: u64) -> serde_json::Map<String, Value> {
        let mut quantities = serde_json::Map::new();
        for parameter in &self.scalars {
            quantities.insert(
                parameter.name().to_string(),
                fused(parameter, now_us, |value| value.into()),
            );
        }
        for parameter in &self.vectors {
            quantities.insert(
                parameter.name().to_string(),
                fused(parameter, now_us, |value| value.to_vec().into()),
            );
        }
        for parameter in &self.positions {
            quantities.insert(
                parameter.name().to_string(),
                fused(parameter, now_us, |[latitude, longitude]| {
                    vec![f64::from(latitude) * 1e-7, f64::from(longitude) * 1e-7].into()
                }),
            );
        }
        quantities
    }

    /// Plan B's `stats databoard` (databoard.py:513-515): each quantity's sources, sorted by name as plan B's
    /// `sources()` sorted them.
    pub fn stats(&self) -> serde_json::Map<String, Value> {
        let sources = |names: Vec<&'static str>| {
            let mut names = names;
            names.sort_unstable();
            Value::from(names)
        };
        self.scalars
            .iter()
            .map(|parameter| (parameter.name().to_string(), sources(parameter.sources().collect())))
            .chain(
                self.vectors
                    .iter()
                    .map(|parameter| (parameter.name().to_string(), sources(parameter.sources().collect()))),
            )
            .chain(
                self.positions
                    .iter()
                    .map(|parameter| (parameter.name().to_string(), sources(parameter.sources().collect()))),
            )
            .collect()
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
            let mut provides = board_config::provides(entry);
            if entry.implementation == "hitl" {
                provides = HITL_QUANTITIES
                    .iter()
                    .map(|quantity| board_config::Provide {
                        quantity,
                        priority: 0,
                        timeout_ms: HITL_TIMEOUT_MS,
                        reconcile: false,
                    })
                    .collect();
            }
            if BAROMETERS.contains(&entry.implementation) {
                let change = provides
                    .iter()
                    .find(|provide| provide.quantity == "elevation")
                    .map(|elevation| board_config::Provide {
                        quantity: "change",
                        reconcile: false,
                        ..*elevation
                    });
                provides.extend(change);
            }
            for provide in provides {
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

/// One quantity's `inspect databoard` entry: `value` through `json`, `source` and `age_ms` of the fused reading at
/// `now_us` (all null when no source is fresh), and the reconcile `offsets` by source.
fn fused<T: databoard::Sample>(parameter: &Parameter<T>, now_us: u64, json: impl Fn(T) -> Value) -> Value {
    let reading = parameter.read(now_us);
    let offsets: serde_json::Map<String, Value> = parameter
        .offsets()
        .map(|(source, offset)| (source.to_string(), offset.into()))
        .collect();
    serde_json::json!({
        "value": reading.map(|reading| json(reading.value)),
        "source": reading.map(|reading| reading.source),
        "age_ms": reading.map(|reading| reading.age_us / 1000),
        "offsets": offsets,
    })
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
    /// The separation switch, when it came up.
    pub separation: Option<SeparationMember>,
    /// The stage machine, when its entry came up.
    pub sequencer: Option<SequencerMember>,
    /// The attitude filter, when its entry came up.
    pub attitude: Option<AttitudeMember>,
    /// The checkpoint, when its entry came up.
    pub checkpoint: Option<CheckpointMember>,
    /// The watchdog, when its entry came up.
    pub watchdog: Option<WatchdogMember>,
    /// The GNSS, when its entry came up.
    pub gnss: Option<GnssMember>,
    /// The pad drift calibration, when its entry came up.
    pub gnss_calib: Option<GnssCalibMember>,
    /// The CC-less field agent, when its entry came up.
    pub field: Option<FieldMember>,
    /// The HITL sim-link, when its entry came up.
    pub hitl: Option<HitlMember>,
}

/// Crashes each entry of `table` named in `names` that came up, with `reason`: the thread that would run it did not
/// start (the control thread, the supervisor), so nothing runs it this boot, verify lists it and `arm` refuses.
pub fn crash<'name>(table: &Table, names: impl IntoIterator<Item = Option<&'name str>>, reason: &str) {
    for entry in names.into_iter().flatten().filter_map(|name| table.find(name)) {
        entry.crash(reason.to_string());
    }
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
    let mut separation = None;
    let mut sequencer = None;
    let mut attitude = None;
    let mut checkpoint = None;
    let mut first_checkpoint = None;
    let mut watchdog = None;
    let mut first_watchdog = None;
    let mut gnss = None;
    let mut gnss_calib = None;
    let mut field = None;
    let mut hitl = None;
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
            Kind::Separation => set_up_separation(document, entry, attempts, &mut separation),
            Kind::Sequencer => set_up_sequencer(entry, &mut sequencer),
            Kind::Attitude => set_up_attitude(entry, &mut attitude),
            Kind::Gnss => set_up_gnss(document, entry, &mut gnss),
            Kind::GnssCalib => set_up_gnss_calib(entry, &mut gnss_calib),
            Kind::Field => set_up_field(entry, &mut field),
            Kind::Hitl => set_up_hitl(document, entry, &mut hitl),
            Kind::Checkpoint => {
                let (outcome, came_up) =
                    set_up_supervised(entry, &mut first_checkpoint, board_config::checkpoint(document));
                if let Some(checkpoint_s) = came_up {
                    checkpoint = Some(CheckpointMember {
                        name: entry.name,
                        checkpoint_s,
                    });
                }
                outcome
            }
            Kind::Watchdog => {
                let (outcome, came_up) =
                    set_up_supervised(entry, &mut first_watchdog, board_config::watchdog(document));
                if let Some(settings) = came_up {
                    watchdog = Some(WatchdogMember {
                        name: entry.name,
                        settings,
                    });
                }
                outcome
            }
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
        separation,
        sequencer,
        attitude,
        checkpoint,
        watchdog,
        gnss,
        gnss_calib,
        field,
        hitl,
    }
}

/// The bring-up of an entry the supervisor runs once a board (the checkpoint, the watchdog), in one attempt, since a
/// config read again reads the same. `settings` are board_config's view of the first enabled entry of its
/// implementation, the first one the bring-up meets, since both walk the directory in order: that entry comes up with
/// them unless the view refused a key, which fails it naming the key, as plan B's arm refused a task whose setup or
/// run loop failed on one. A later entry of the same implementation fails, since one runs (plan B would run both, over
/// one crumb or one task watchdog).
///
/// Args:
/// * `entry` - the entry.
/// * `first` - the first entry of its implementation the bring-up met, None before it: this one, from here on.
/// * `settings` - the view's reading: None only for a config with no enabled entry of the implementation, which this
///   entry contradicts.
///
/// Returns: the entry's outcome, and its settings when it came up.
fn set_up_supervised<T: Copy>(
    entry: &Entry<'static>,
    first: &mut Option<&'static str>,
    settings: Option<Result<T, String>>,
) -> (Result<(), String>, Option<T>) {
    let earlier = *first;
    first.get_or_insert(entry.name);
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (earlier, &settings) {
            (Some(earlier), _) => Attempt::Raised {
                error: format!(
                    "one {} only: the first, '{earlier}', is the board's",
                    entry.implementation
                ),
                diagnosis: None,
            },
            (None, Some(Ok(settings))) => {
                came_up = Some(*settings);
                Attempt::Up
            }
            (None, Some(Err(refused))) => Attempt::Raised {
                error: refused.clone(),
                diagnosis: None,
            },
            (None, None) => Attempt::Raised {
                error: "the board config's view found no enabled entry".to_string(),
                diagnosis: None,
            },
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    (outcome, came_up)
}

/// The stage machine's bring-up (plan B tasks/sequencer.py `setup`): its settings read from the entry, in one attempt,
/// since a config read again reads the same. A key [`sequencer::Settings::from_config`] refuses fails the entry as
/// plan B's setup raised (`controller :: task '<name>' setup raised: <key>: <value> <why>`), so verify lists it and
/// `arm` refuses. The first entry's machine runs on the control thread; another fails, since one stage machine commits
/// to the flight word (plan B would run both against one Controller).
fn set_up_sequencer(entry: &Entry<'static>, sequencer: &mut Option<SequencerMember>) -> Result<(), String> {
    let settings = sequencer::Settings::from_config(&Value::Object(entry.fields.clone()));
    let running = sequencer.as_ref().map(|member| member.name);
    controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (&settings, running) {
            (Err(refused), _) => Attempt::Raised {
                error: refused.to_string(),
                diagnosis: None,
            },
            (Ok(_), Some(running)) => Attempt::Raised {
                error: format!("one stage machine only: '{running}' runs it"),
                diagnosis: None,
            },
            (Ok(settings), None) => {
                *sequencer = Some(SequencerMember {
                    name: entry.name,
                    settings: *settings,
                });
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    )
}

/// The attitude filter's bring-up (plan B tasks/attitude.py setup): its settings from the entry; it fails without an
/// `attitude` it provides (plan B's setup raised: there would be nothing to publish into), and a second filter
/// fails (one control thread steps one).
fn set_up_attitude(entry: &Entry<'static>, attitude: &mut Option<AttitudeMember>) -> Result<(), String> {
    let fields = Value::Object(entry.fields.clone());
    let provides = fields.pointer("/provides/attitude").is_some();
    let running = attitude.as_ref().map(|member| member.name);
    controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (provides, running) {
            (false, _) => Attempt::Raised {
                error: "no `provides.attitude`: nothing to publish into".to_string(),
                diagnosis: None,
            },
            (true, Some(running)) => Attempt::Raised {
                error: format!("one attitude filter only: '{running}' runs it"),
                diagnosis: None,
            },
            (true, None) => {
                *attitude = Some(AttitudeMember {
                    name: entry.name,
                    settings: control::attitude::Settings::from_json(&fields),
                });
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    )
}

/// The separation switch's bring-up (plan B separation.py:49-64): its pin (the entry's `pin`, else the board's
/// `separation_switch`) an input with the internal pull-down, its level read as the latch's start, with no debounce,
/// event or row.
fn set_up_separation(
    document: &Value,
    entry: &Entry<'static>,
    attempts: u32,
    separation: &mut Option<SeparationMember>,
) -> Result<(), String> {
    let gpio = if entry.fields.contains_key("pin") {
        board_config::pin(document, entry, "pin")
    } else {
        document
            .get("pins")
            .and_then(|pins| pins.get(SEPARATION_PIN))
            .and_then(Value::as_u64)
            .and_then(|gpio| u8::try_from(gpio).ok())
    };
    let settings = drivers::separation::Settings::from_config(&Value::Object(entry.fields.clone()));
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        attempts,
        |_, last| {
            let Some(gpio) = gpio else {
                let name = entry
                    .fields
                    .get("pin")
                    .and_then(Value::as_str)
                    .unwrap_or(SEPARATION_PIN);
                return Attempt::Failed {
                    diagnosis: last.then(|| format!("no pin -- '{name}' not defined in config pins")),
                };
            };
            match input_pulled_down(gpio) {
                Ok(()) => {
                    // SAFETY: a plain read of the input just configured.
                    let high = unsafe { sys::gpio_get_level(i32::from(gpio)) } != 0;
                    // the latch from now: the warm start (M8) reads it before the control thread runs
                    crate::control::publish_separated(!high);
                    came_up = Some(SeparationMember {
                        name: entry.name,
                        gpio,
                        part: Separation::new(&settings, high),
                    });
                    Attempt::Up
                }
                Err(error) => Attempt::Raised {
                    error: format!("GPIO{gpio}: {error}"),
                    diagnosis: None,
                },
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    *separation = came_up;
    outcome
}

/// The GNSS's bring-up (plan B gnss.py setup): its UART's spec from `buses.<bus>.<id>` (`uart`, 2 by default), in
/// one attempt, since a config read again reads the same. No spec fails the entry with plan B's diagnosis (`no
/// transport -- uart bus <id> undefined in config`); a UART other than UART2, or a second GNSS, fails it too (the
/// firmware hands UART2 to one GNSS thread). The UART opens when the thread starts (main), which crashes the entry if
/// it would not.
fn set_up_gnss(document: &Value, entry: &Entry<'static>, gnss: &mut Option<GnssMember>) -> Result<(), String> {
    let id = entry
        .fields
        .get("id")
        .map_or(Some(GNSS_UART), |id| id.as_u64().and_then(|id| u8::try_from(id).ok()));
    let kind = entry.fields.get("bus").and_then(Value::as_str).unwrap_or("uart");
    let specification = id
        .filter(|_| kind == "uart")
        .and_then(|id| board_config::uart_bus(document, id));
    let module = if entry.implementation == "atgm336h" {
        crate::gnss::Module::Atgm
    } else {
        crate::gnss::Module::Neo6
    };
    let hz = entry
        .fields
        .get("hz")
        .and_then(Value::as_i64)
        .and_then(|hz| i32::try_from(hz).ok())
        .unwrap_or(1);
    let running = gnss.as_ref().map(|member: &GnssMember| member.name);
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (specification, running) {
            (_, Some(running)) => Attempt::Raised {
                error: format!("one GNSS only: '{running}' has uart:{GNSS_UART}"),
                diagnosis: None,
            },
            (None, None) => Attempt::Failed {
                diagnosis: Some(format!(
                    "no transport -- {kind} bus {} undefined in config",
                    entry
                        .fields
                        .get("id")
                        .map_or_else(|| GNSS_UART.to_string(), Value::to_string)
                )),
            },
            (Some(uart), None) if uart.id != GNSS_UART => Attempt::Raised {
                error: format!(
                    "uart:{} is not the GNSS's: the firmware runs it on uart:{GNSS_UART}",
                    uart.id
                ),
                diagnosis: None,
            },
            (Some(uart), None) => {
                came_up = Some(GnssMember {
                    name: entry.name,
                    module,
                    hz,
                    uart,
                });
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    if came_up.is_some() {
        *gnss = came_up;
    }
    outcome
}

/// The pad drift calibration's bring-up (plan B tasks/gnss_calib.py setup): `min_samples` (5) and `period_ms` (1000)
/// read from the entry, in one attempt. A value that is no whole number in range fails the entry naming the key (plan
/// B would run with it: a negative `min_samples` divides by zero at a sample-less launch, a non-positive `period_ms`
/// spins its loop); a second calibration fails too, since one drift feeds the wind.
fn set_up_gnss_calib(entry: &Entry<'static>, gnss_calib: &mut Option<GnssCalibMember>) -> Result<(), String> {
    let count = |key: &str, default: u32, least: u32| match entry.fields.get(key) {
        None => Ok(default),
        Some(value) => value
            .as_u64()
            .and_then(|value| u32::try_from(value).ok())
            .filter(|value| *value >= least)
            .ok_or_else(|| format!("{key}: {value} is not a whole number from {least}")),
    };
    let settings = count("min_samples", 5, 0).and_then(|min_samples| {
        count("period_ms", 1000, 1).map(|period_ms| GnssCalibMember {
            name: entry.name,
            min_samples,
            period_ms,
        })
    });
    let running = gnss_calib.map(|member: GnssCalibMember| member.name);
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (&settings, running) {
            (_, Some(running)) => Attempt::Raised {
                error: format!("one gnss_calib only: '{running}' runs it"),
                diagnosis: None,
            },
            (Err(refused), None) => Attempt::Raised {
                error: refused.clone(),
                diagnosis: None,
            },
            (Ok(member), None) => {
                came_up = Some(*member);
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    if came_up.is_some() {
        *gnss_calib = came_up;
    }
    outcome
}

/// The field agent's bring-up (plan B tasks/field.py setup): its knobs read from the entry
/// (mission::field::Settings::from_config), in one attempt. A knob plan B could not fly with fails the entry naming
/// the key; a second agent fails too, since one decides the site and the arm.
fn set_up_field(entry: &Entry<'static>, field: &mut Option<FieldMember>) -> Result<(), String> {
    let settings = mission::field::Settings::from_config(entry.fields);
    let running = field.as_ref().map(|member: &FieldMember| member.name);
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (&settings, running) {
            (_, Some(running)) => Attempt::Raised {
                error: format!("one field agent only: '{running}' runs it"),
                diagnosis: None,
            },
            (Err(refused), None) => Attempt::Raised {
                error: refused.clone(),
                diagnosis: None,
            },
            (Ok(settings), None) => {
                came_up = Some(FieldMember {
                    name: entry.name,
                    settings: *settings,
                });
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    if came_up.is_some() {
        *field = came_up;
    }
    outcome
}

/// The sim-link's bring-up (doc/specs/sim-link.md): `sim_port` (1236) read from the entry, in one attempt; the
/// physics keys are the host's and ignored here. It fails when a simulated sensor still runs (avionics-port.md R10:
/// main masked them, and one left enabled would be a silent second source), and for a second entry.
fn set_up_hitl(document: &Value, entry: &Entry<'static>, hitl: &mut Option<HitlMember>) -> Result<(), String> {
    let port = match entry.fields.get("sim_port") {
        None => Ok(SIM_PORT),
        Some(value) => value
            .as_u64()
            .and_then(|port| u16::try_from(port).ok())
            .filter(|port| *port > 0)
            .ok_or_else(|| format!("sim_port: {value} is not a UDP port")),
    };
    let live: Vec<&str> = board_config::directory(document)
        .iter()
        .map(|sensor| sensor.name)
        .filter(|name| board_config::SIM_SENSORS.contains(name))
        .collect();
    let running = hitl.as_ref().map(|member: &HitlMember| member.name);
    let mut came_up = None;
    let outcome = controller::set_up(
        entry.name,
        entry.implementation,
        1,
        |_, _| match (&port, running) {
            (_, Some(running)) => Attempt::Raised {
                error: format!("one hitl only: '{running}' runs the sim-link"),
                diagnosis: None,
            },
            _ if !live.is_empty() => Attempt::Raised {
                error: format!("simulated sensors still enabled: {}", live.join(", ")),
                diagnosis: None,
            },
            (Err(refused), None) => Attempt::Raised {
                error: refused.clone(),
                diagnosis: None,
            },
            (Ok(port), None) => {
                came_up = Some(HitlMember {
                    name: entry.name,
                    port: *port,
                });
                Attempt::Up
            }
        },
        |ms| std::thread::sleep(Duration::from_millis(u64::from(ms))),
        |line| log::info!("{line}"),
    );
    if came_up.is_some() {
        *hitl = came_up;
    }
    outcome
}

/// GPIO `gpio` as an input with the internal pull-down (the pads route 3V3 to it while nested).
fn input_pulled_down(gpio: u8) -> Result<(), sys::EspError> {
    let config = sys::gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode: sys::gpio_mode_t_GPIO_MODE_INPUT,
        pull_up_en: sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: sys::gpio_pulldown_t_GPIO_PULLDOWN_ENABLE,
        intr_type: sys::gpio_int_type_t_GPIO_INTR_DISABLE,
        ..Default::default()
    };
    // SAFETY: plain GPIO configuration of the pin the board config gives the switch.
    sys::esp!(unsafe { sys::gpio_config(&config) })
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
        members[usize::from(id)].push(Member {
            name: entry.name,
            part,
            run: None,
            // Part::build refused a malformed key, so the part up has a good one
            axes: sensors::imu_axes(entry).unwrap_or_default(),
            interrupt: board_config::pin(document, entry, "int_pin"),
            period_ms: entry
                .fields
                .get("period_ms")
                .and_then(Value::as_u64)
                .map(|period| period.clamp(1, u64::from(u32::MAX)) as u32),
        });
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
    let axes = sensors::imu_axes(entry);
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
            if let Err(error) = &axes {
                return Attempt::Raised {
                    error: error.clone(),
                    diagnosis: None,
                };
            }
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
    if let (Some(part), Some(mut handle)) = (came_up, handle) {
        // the part's own ODR: one slower than the tick reads stale now and then, one faster skips conversions
        match part.frequency_fine(&mut handle) {
            Ok(fine) => log::info!(
                "{} :: ODR {:.2} Hz (INTERNAL_FREQ_FINE {fine})",
                entry.name,
                drivers::lsm6dso32::odr_hz(fine)
            ),
            Err(fault) => log::warn!("{} :: ODR unknown: INTERNAL_FREQ_FINE {fault}", entry.name),
        }
        let period_ms = entry
            .fields
            .get("period_ms")
            .and_then(Value::as_u64)
            .map_or(IMU_PERIOD_MS, |period| period.clamp(1, u64::from(u32::MAX)) as u32);
        members.push(SpiMember {
            name: entry.name,
            handle,
            part,
            interrupt,
            period_ms,
            axes: axes.unwrap_or_default(),
        });
    }
    outcome
}
