//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The board's quantities, by sample type, each with its sources: built once at bring-up (crates/controller
//! `topology`, from the entries that came up), then shared read-only by the flight threads, the firmware's and the
//! SITL's alike.

use board_config::Entry;
use serde_json::Value;

use databoard::{Parameter, Sample};

use crate::Table;

/// The quantities the `hitl` entry provides (plan B hitl.py's `Databoard.provide` at setup, its entry has no
/// `provides`), each at priority 0 with plan B's 1 s timeout; `change` the port's own, as every barometer's.
pub const HITL_QUANTITIES: [&str; 13] = [
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

/// The topology of the entries `table` holds as up, in setup order, and its warnings: a quantity no board knows, a
/// source past its quantity's limit. The firmware's bring-up and the SITL build it alike.
pub fn topology(entries: &[Entry<'static>], table: &Table) -> (Topology, Vec<String>) {
    let mut topology = Topology::default();
    let mut lines = Vec::new();
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
            let kind = if SCALARS.contains(&quantity) {
                Kind::Scalar
            } else if VECTORS.contains(&quantity) {
                Kind::Vector
            } else if POSITIONS.contains(&quantity) {
                Kind::Position
            } else {
                lines.push(format!(
                    "databoard :: '{quantity}' of {} is no quantity the board knows: skipped",
                    entry.name
                ));
                continue;
            };
            if !topology.add(
                kind,
                quantity,
                entry.name,
                provide.priority,
                provide.timeout_ms,
                provide.reconcile,
            ) {
                lines.push(format!(
                    "databoard :: {quantity} has {} sources already: {} not added",
                    databoard::MAXIMUM_SOURCES,
                    entry.name
                ));
            }
        }
    }
    (topology, lines)
}

/// The databoard's quantities, by sample type, each with its sources: built once at bring-up, then shared
/// read-only.
///
/// Each Parameter is boxed on its own (~0.42 KB), so on the board every one is allocated internal, the flight readers'
/// memory (avionics-port.md §4.7): a run of them in one Vec would pass SPIRAM_MALLOC_ALWAYSINTERNAL (4 KB) at ten
/// scalars and land in PSRAM.
#[expect(
    clippy::vec_box,
    reason = "each Parameter is boxed for its memory: see the type's notes"
)]
#[derive(Default)]
pub struct Topology {
    /// The scalar quantities.
    scalars: Vec<Box<Parameter<f32>>>,
    /// The vector quantities.
    vectors: Vec<Box<Parameter<[f32; 3]>>>,
    /// The positions.
    positions: Vec<Box<Parameter<[i32; 2]>>>,
}

/// One kind of quantity: what its samples are.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// One f32.
    Scalar,
    /// Three f32.
    Vector,
    /// Latitude and longitude, 1e-7 degree.
    Position,
}

impl Topology {
    /// The scalar quantity `quantity` (altitude, pressure, ...), when a provider of it came up.
    pub fn scalar(&self, quantity: &str) -> Option<&Parameter<f32>> {
        find(&self.scalars, quantity)
    }

    /// The vector quantity `quantity` (accel, rate, mag, attitude), when a provider of it came up.
    pub fn vector(&self, quantity: &str) -> Option<&Parameter<[f32; 3]>> {
        find(&self.vectors, quantity)
    }

    /// The position quantity `quantity` (`position`), when a provider of it came up.
    pub fn position(&self, quantity: &str) -> Option<&Parameter<[i32; 2]>> {
        find(&self.positions, quantity)
    }

    /// Registers `source` for `quantity` of `kind` at `priority` with `timeout_ms`, the quantity made on its first
    /// source. Whether it was added (a quantity holds [`databoard::MAXIMUM_SOURCES`] at most).
    pub fn add(
        &mut self,
        kind: Kind,
        quantity: &'static str,
        source: &'static str,
        priority: i32,
        timeout_ms: u32,
        reconcile: bool,
    ) -> bool {
        match kind {
            Kind::Scalar => add(&mut self.scalars, quantity, source, priority, timeout_ms, reconcile),
            Kind::Vector => add(&mut self.vectors, quantity, source, priority, timeout_ms, reconcile),
            Kind::Position => add(&mut self.positions, quantity, source, priority, timeout_ms, reconcile),
        }
    }

    /// One line per quantity: `databoard :: pressure <- baro_icp10111 p0, baro_bmp581 p1; window 200 ms`.
    pub fn describe(&self) -> Vec<String> {
        self.scalars
            .iter()
            .map(|parameter| describe(parameter))
            .chain(self.vectors.iter().map(|parameter| describe(parameter)))
            .chain(self.positions.iter().map(|parameter| describe(parameter)))
            .collect()
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
}

/// The quantity named `quantity` among `parameters`.
fn find<'topology, T: Sample>(
    parameters: &'topology [Box<Parameter<T>>],
    quantity: &str,
) -> Option<&'topology Parameter<T>> {
    parameters
        .iter()
        .find(|parameter| parameter.name() == quantity)
        .map(|parameter| &**parameter)
}

/// Registers `source` for `quantity` in `parameters`, making the quantity on its first source; whether it was added.
#[expect(
    clippy::vec_box,
    reason = "each Parameter is boxed for its memory: see Topology's notes"
)]
fn add<T: Sample>(
    parameters: &mut Vec<Box<Parameter<T>>>,
    quantity: &'static str,
    source: &'static str,
    priority: i32,
    timeout_ms: u32,
    reconcile: bool,
) -> bool {
    let index = match parameters.iter().position(|parameter| parameter.name() == quantity) {
        Some(index) => index,
        None => {
            parameters.push(Box::new(Parameter::new(quantity)));
            parameters.len() - 1
        }
    };
    parameters[index]
        .add_source(source, priority, timeout_ms, reconcile)
        .is_ok()
}

/// A quantity's log line.
fn describe<T: Sample>(parameter: &Parameter<T>) -> String {
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

/// One quantity's `inspect databoard` entry: `value` through `json`, `source` and `age_ms` of the fused reading at
/// `now_us` (all null when no source is fresh), and the reconcile `offsets` by source.
fn fused<T: Sample>(parameter: &Parameter<T>, now_us: u64, json: impl Fn(T) -> Value) -> Value {
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
