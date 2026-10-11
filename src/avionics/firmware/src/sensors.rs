//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The parts behind the board config's entries (doc/specs/avionics-port.md M5, "The registry"): what the port runs
//! for each implementation ([`Kind`], plan B's task registry), and the I2C parts as one [`Part`] over the drivers
//! crate. A part is built afresh from its entry for every setup attempt, as plan B creates a task per attempt; its
//! bus owner then sets it up, re-arms it after a general call and asks its diagnose when the last attempt failed.
//!
//! Not here yet, each with its milestone: the readings and their channels (M6, M9, M10), what a plan B setup
//! restores from storage (the SDP810's tare, the BMM350's hard-iron calibration: M9, M10), the `axes` of the parts
//! whose vectors reach the airframe through the glue (the BMI323's and the LSM6DSO32's, with their reads: M6, M9),
//! and the SPI LSM6DSO32's probe, which needs SPI2's owner (M6).

use std::mem::ManuallyDrop;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU32, Ordering};

use board_config::Entry;
use control::change::Change;
use controller::DeviceEntry;
use databoard::Writer;
use drivers::axes::Axes;
use drivers::bmi323::Bmi323;
use drivers::bmm350::Bmm350;
use drivers::bmp581;
use drivers::bmp581::Bmp581;
use drivers::device::Device;
use drivers::freshness;
use drivers::icp10111::{Icp10111, Poll, Reading};
use drivers::ina226::Ina226;
use drivers::lsm6dso32::{Lsm6dso32, OverI2c};
use drivers::sdp810::Sdp810;
use drivers::vl53l1x::Vl53;
use embedded_hal::digital::OutputPin;
use esp_idf_svc::hal::delay::Delay;
use esp_idf_svc::hal::gpio::{AnyOutputPin, GpioError, Output, PinDriver};
use esp_idf_svc::sys::EspError;
use firmware::buses::i2c::I2cPort;
use recorder::{Cell, Telemetry};
use serde_json::Value;

use crate::devices::Topology;

/// The failure reason of a plan B implementation this port has not reached yet (D13 of the M5 plan): plan B's
/// `no driver/activity`, with why.
pub const NOT_PORTED_YET: &str = "no driver/activity -- not ported yet";

/// What runs an entry's implementation (its `driver`, else its `activity`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A [`Part`] on an I2C port.
    I2c,
    /// A part on SPI2: the LSM6DSO32 on its default transport.
    Spi,
    /// The separation switch: a GPIO the control thread reads.
    Separation,
    /// The stage machine: its settings read from the entry, its steps on the control thread.
    Sequencer,
    /// The attitude filter: its settings read from the entry, its steps on the control thread (M9).
    Attitude,
    /// The checkpoint: its cadence read from the entry, its passes on the supervisor (crate::warm).
    Checkpoint,
    /// The watchdog: its settings read from the entry, the task watchdog and the stall check on the supervisor.
    Watchdog,
    /// The GNSS on its UART: its own thread reads it (crate::gnss, M11).
    Gnss,
    /// The pad drift calibration: its settings read from the entry, its passes on the supervisor (M11).
    GnssCalib,
    /// The CC-less field agent: its settings read from the entry, its passes on the executor (M11).
    Field,
    /// The HITL sim-link: its port read from the entry, its frames on its own thread (M12).
    Hitl,
    /// A fin servo (SG90, MG90S): its pin and settings read from the entry, its PWM on LEDC (M13).
    Servo,
    /// The flight loop: its law read from the entry and the fins section, its steps on the control thread (M13).
    Flight,
    /// Run natively by the firmware, with nothing to set up: up.
    Native,
    /// A plan B implementation not ported yet: failed with [`NOT_PORTED_YET`].
    NotPorted,
    /// No implementation plan B knows: failed with plan B's own `no driver/activity`.
    Unknown,
}

/// Implementations the firmware runs natively: the recorder, the CC link, Wi-Fi, the supervisor's health, the camera,
/// and the BLE radio switch, a stub in plan B's no-BLE shape (drivers::bluetooth; its inspect, update and probe are
/// the commands').
const NATIVE: [&str; 6] = ["recorder", "cc", "wifi", "health", "ov5647", "bluetooth"];

/// Plan B implementations whose port comes with a later milestone.
const NOT_PORTED: [&str; 3] = ["adxl375", "bno055", "bmp280"];

/// What runs `entry`. The LSM6DSO32 is an SPI part unless its entry binds it to I2C (plan B's lsm6dso32.py:144: the
/// `bus` defaults to 'spi').
pub fn classify(entry: &Entry) -> Kind {
    match entry.implementation {
        "bmi323" | "bmm350" | "bmp581" | "icp10111" | "ina226" | "sdp810" | "vl53l1x" | "vl53l4cx" => Kind::I2c,
        "lsm6dso32" => match controller::binding(entry, "spi", Some(drivers::lsm6dso32::ADDRESS)) {
            controller::Binding::I2c { .. } => Kind::I2c,
            _ => Kind::Spi,
        },
        "separation" => Kind::Separation,
        "sequencer" => Kind::Sequencer,
        "attitude" => Kind::Attitude,
        "checkpoint" => Kind::Checkpoint,
        "watchdog" => Kind::Watchdog,
        "neo6mv2" | "atgm336h" => Kind::Gnss,
        "gnss_calib" => Kind::GnssCalib,
        "field" => Kind::Field,
        "hitl" => Kind::Hitl,
        "sg90" | "mg90s" => Kind::Servo,
        "flight" => Kind::Flight,
        implementation if NATIVE.contains(&implementation) => Kind::Native,
        implementation if NOT_PORTED.contains(&implementation) => Kind::NotPorted,
        _ => Kind::Unknown,
    }
}

/// Whether the implementation re-zeroes a barometer's ground: the ICP-10111 and the BMP581 (M9), whose `calibrate`
/// and `update` (`ground`, `rezero`) also run at DONE, the way back from a false launch ([`Part::rezeroes`] says it
/// of a built part).
pub fn rezeroes(implementation: &str) -> bool {
    matches!(implementation, "icp10111" | "bmp581")
}

/// The `axes` of an IMU entry (drivers::axes, M9): the identity when absent.
///
/// Errors: a malformed key or a mirror, naming the key (the part's setup fails with it).
pub fn imu_axes(entry: &Entry) -> Result<Axes, String> {
    Axes::from_config(&Value::Object(entry.fields.clone())).map_err(|error| error.to_string())
}

/// The datasheet address an I2C part's driver binds when its entry has no `addr` (plan B i2cbus.py:301).
pub fn default_address(implementation: &str) -> Option<u8> {
    match implementation {
        "bmi323" => Some(drivers::bmi323::ADDRESS),
        "bmm350" => Some(drivers::bmm350::ADDRESS),
        "bmp581" => Some(drivers::bmp581::ADDRESS),
        "icp10111" => Some(drivers::icp10111::ADDRESS),
        "ina226" => Some(drivers::ina226::ADDRESS),
        "lsm6dso32" => Some(drivers::lsm6dso32::ADDRESS),
        "sdp810" => Some(drivers::sdp810::ADDRESS),
        "vl53l1x" | "vl53l4cx" => Some(drivers::vl53l1x::ADDRESS),
        _ => None,
    }
}

/// What a part does on its owner's tick beyond its bus work: where its readings go (M6: the ICP-10111; M9: the
/// BMI323). Each variant is boxed, so the other members of a port do not pay for the largest inline (M9).
pub enum Run {
    /// A barometer's channels, its climb rate and its stream.
    Barometer(Box<Barometer>),
    /// The second IMU's channels and stream.
    Imu(Box<Inertial>),
    /// The magnetometer's channel and stream.
    Magnetometer(Box<Magnetometer>),
    /// The pitot's channels and stream (M10).
    Pitot(Box<Pitot>),
    /// The power monitor's channels and stream (M10).
    Power(Box<Power>),
    /// A laser ranger's channel and stream (M10).
    Ranger(Box<Ranger>),
}

impl Run {
    /// The run `member`'s part needs on its owner's tick, its writers claimed from `topology` for its source name,
    /// writing the health of its table `entry`; None for a part with nothing to run yet.
    pub fn new(
        member: &crate::devices::Member,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Option<Run> {
        let name = member.name;
        match &member.part {
            Part::Icp10111(_) => Some(Run::Barometer(Box::new(Barometer::new(name, topology, entry)))),
            // its elevation on the databoard too: the multi-source machine reads each barometer against itself, so an
            // ICP -> BMP581 handover is never a climb (M9)
            Part::Bmp581(_) => Some(Run::Barometer(Box::new(Barometer::new(name, topology, entry)))),
            Part::Bmm350(_) => Some(Run::Magnetometer(Box::new(Magnetometer::new(name, topology, entry)))),
            Part::Sdp810(_) => Some(Run::Pitot(Box::new(Pitot::new(
                name,
                member.period_ms.unwrap_or(drivers::sdp810::DEFAULT_PERIOD_MS),
                topology,
                entry,
            )))),
            Part::Ina226(_) => Some(Run::Power(Box::new(Power::new(
                name,
                member.period_ms.unwrap_or(drivers::ina226::DEFAULT_PERIOD_MS),
                topology,
                entry,
            )))),
            Part::Vl53(_) => Some(Run::Ranger(Box::new(Ranger::new(
                name,
                member.period_ms.unwrap_or(drivers::vl53l1x::DEFAULT_PERIOD_MS),
                topology,
                entry,
            )))),
            Part::Bmi323(_) => Some(Run::Imu(Box::new(Inertial::new(
                name,
                member.axes,
                member.interrupt,
                topology,
                entry,
            )))),
            Part::Lsm6dso32(_) => {
                log::info!("{name} :: no read loop on I2C (SPI only)");
                None
            }
        }
    }
}

/// What a tick asks of the owner beyond the part itself.
pub enum Event {
    /// The part's recovery sent a general call and the bus has settled: re-arm the port's other parts.
    Rearm,
}

/// The BMP581's read pairs that disagreed (drivers::bmp581::Outcome::Torn): mostly a conversion landing between the
/// two bursts (about 1 read in 80), the rest the bus garbage the double read exists for (M9).
static BMP581_TORN: AtomicU32 = AtomicU32::new(0);
/// Its agreeing pairs outside the part's operating range (drivers::bmp581::Outcome::Implausible): bus garbage read
/// twice alike.
static BMP581_IMPLAUSIBLE: AtomicU32 = AtomicU32::new(0);
/// How many BMP581 reads in a row may go unjudged (torn or implausible) before the part is unhealthy: 60 ms, three
/// conversion periods, as its stale rule.
const UNJUDGED_LIMIT: u32 = 6;

/// What `inspect` shows of the BMP581's guard: its torn pairs and its implausible reads since boot.
pub fn bmp581_inspection() -> (u32, u32) {
    (
        BMP581_TORN.load(Ordering::Relaxed),
        BMP581_IMPLAUSIBLE.load(Ordering::Relaxed),
    )
}

/// One barometer conversion, whichever part made it.
#[derive(Clone, Copy)]
struct Conversion {
    /// m AMSL.
    altitude: f32,
    /// Degrees C.
    temperature: f32,
    /// Pa.
    pressure: f32,
}

impl From<&Reading> for Conversion {
    fn from(reading: &Reading) -> Conversion {
        Conversion {
            altitude: reading.altitude,
            temperature: reading.temperature,
            pressure: reading.pressure,
        }
    }
}

impl From<&drivers::bmp581::Reading> for Conversion {
    fn from(reading: &drivers::bmp581::Reading) -> Conversion {
        Conversion {
            altitude: reading.altitude,
            temperature: reading.temperature,
            pressure: reading.pressure,
        }
    }
}

/// A barometer's outputs: its five databoard channels (plan B's four in its push order, then `change`) and its stream.
pub struct Barometer {
    /// The `altitude` writer (m AMSL), when the quantity is on the databoard.
    altitude: Option<Writer<'static, f32>>,
    /// The `temperature` writer (degrees C).
    temperature: Option<Writer<'static, f32>>,
    /// The `pressure` writer (Pa).
    pressure: Option<Writer<'static, f32>>,
    /// The `elevation` writer (m above the ground).
    elevation: Option<Writer<'static, f32>>,
    /// The `change` writer (m/s, climb positive; M9).
    change: Option<Writer<'static, f32>>,
    /// The climb rate's samples: the least-squares slope of the altitude over the last half second.
    climb: Change,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes (plan B's `Task._healthy`).
    entry: Option<&'static DeviceEntry<'static>>,
    /// A read failed and none succeeded since: one log line per run (plan B's `note`).
    failing: bool,
    /// The part read stale (the BMP581's unchanged bytes for three conversions) and nothing fresh since: one line.
    stale: bool,
    /// The BMP581's reads in a row that were neither judged nor published (torn pairs, implausible values).
    unjudged_run: u32,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Barometer {
    /// The outputs of source `name`, writing the health of `entry`.
    fn new(name: &'static str, topology: &'static Topology, entry: Option<&'static DeviceEntry<'static>>) -> Barometer {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Barometer {
            altitude: writer("altitude"),
            temperature: writer("temperature"),
            pressure: writer("pressure"),
            elevation: writer("elevation"),
            change: writer("change"),
            climb: Change::default(),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            failing: false,
            stale: false,
            unjudged_run: 0,
            overflowing: false,
        }
    }

    /// A read that failed: unhealthy, one line per run.
    fn fail(&mut self, name: &str, fault: &dyn std::fmt::Display) {
        self.set_healthy(false);
        if !self.failing {
            log::warn!("{name} :: read {fault}");
            self.failing = true;
        }
    }

    /// A part that stopped converting (its bytes unchanged for three conversion periods): unhealthy, one line.
    fn freeze(&mut self, name: &str) {
        self.set_healthy(false);
        if !self.stale {
            log::warn!("{name} :: no new conversion -- stale");
            self.stale = true;
        }
    }

    /// Records the latest read's outcome in the table entry.
    fn set_healthy(&self, healthy: bool) {
        if let Some(entry) = self.entry {
            entry.set_healthy(healthy);
        }
    }

    /// Publishes a reading (plan B icp10111.py:346-362): altitude, temperature, pressure, then the elevation above
    /// the ground, then the climb rate once its window holds enough (M9), and the row -- one for every conversion,
    /// never decimated (measured -> reported, the operator).
    fn publish(&mut self, name: &str, elevation: Option<f32>, reading: Conversion, now_us: u64) {
        self.failing = false;
        self.stale = false;
        self.set_healthy(true);
        let change = self.climb.push(reading.altitude, now_us);
        for (writer, value) in [
            (&mut self.altitude, Some(reading.altitude)),
            (&mut self.temperature, Some(reading.temperature)),
            (&mut self.pressure, Some(reading.pressure)),
            (&mut self.elevation, elevation),
            (&mut self.change, change),
        ] {
            if let (Some(writer), Some(value)) = (writer, value) {
                writer.push(value, now_us);
            }
        }
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        let row = [
            Cell::F32(reading.altitude),
            Cell::F32(reading.temperature),
            Cell::F32(reading.pressure),
            elevation.map_or(Cell::None, Cell::F32),
            change.map_or(Cell::None, Cell::F32),
        ];
        match telemetry.push(now_us, &row) {
            Ok(()) => self.overflowing = false,
            Err(overflow) => {
                if !self.overflowing {
                    log::warn!("{name} :: telemetry {overflow}");
                    self.overflowing = true;
                }
            }
        }
    }
}

/// The BMI323's raced halves (accel, rate): its data-ready flag clear but its bytes changed (M9, `inspect`).
static BMI323_RACED: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// The BMI323's halves flagged over bytes identical to the last read's (accel, rate).
static BMI323_IDENTICAL: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
/// The BMI323's fresh accel conversions, the first one's and the latest one's uptime in ms: its real ODR.
static BMI323_CONVERSIONS: AtomicU32 = AtomicU32::new(0);
/// The uptime of the BMI323's first fresh accel conversion, ms.
static BMI323_FIRST_MS: AtomicU32 = AtomicU32::new(0);
/// The uptime of its latest, ms.
static BMI323_LAST_MS: AtomicU32 = AtomicU32::new(0);
/// The INT1 edges its latest read took (`irq_runs`).
static BMI323_IRQ_RUNS: AtomicU32 = AtomicU32::new(0);
/// Its INT1 line's verdict: 0 no wire declared (polled by design), 1 driving, 2 declared and silent.
static BMI323_SILENT: AtomicU8 = AtomicU8::new(0);

/// What `inspect` shows of the BMI323 beyond its channels (plan B bmi323.py:305-317 and the port's own): whether its
/// INT1 is silent (None: no wire declared), the edges its latest read took, its measured ODR (None before two
/// conversions), and its raced and identical halves (accel, rate).
pub fn bmi323_inspection() -> (Option<bool>, u32, Option<f32>, [u32; 2], [u32; 2]) {
    let silent = match BMI323_SILENT.load(Ordering::Relaxed) {
        0 => None,
        verdict => Some(verdict == 2),
    };
    let conversions = BMI323_CONVERSIONS.load(Ordering::Relaxed);
    let span_ms = BMI323_LAST_MS
        .load(Ordering::Relaxed)
        .wrapping_sub(BMI323_FIRST_MS.load(Ordering::Relaxed));
    let odr_hz = (conversions > 1 && span_ms > 0).then(|| (conversions - 1) as f32 * 1000.0 / span_ms as f32);
    let pair = |counters: &[AtomicU32; 2]| counters.each_ref().map(|counter| counter.load(Ordering::Relaxed));
    (
        silent,
        BMI323_IRQ_RUNS.load(Ordering::Relaxed),
        odr_hz,
        pair(&BMI323_RACED),
        pair(&BMI323_IDENTICAL),
    )
}

/// The BMI323's outputs (M9): its `accel` and `rate` channels (a backup to the SPI IMU, at its config priority) and
/// its stream, every conversion through the airframe's axes. Read every owner wake, paced by the control tick; its
/// INT1, when wired, is evidence only (D12 of the M9 plan; plan B sampled on the edge).
pub struct Inertial {
    /// The `accel` writer (g).
    accel: Option<Writer<'static, [f32; 3]>>,
    /// The `rate` writer (deg/s).
    rate: Option<Writer<'static, [f32; 3]>>,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// How its axes map onto the airframe's.
    axes: Axes,
    /// Its INT1 GPIO until armed: armed once the control thread runs (its GPIO interrupt service first).
    interrupt: Option<u8>,
    /// Whether INT1 is armed: its edges are counted.
    armed: bool,
    /// Reads in a row with no new conversion.
    stale_run: u32,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Inertial {
    /// The outputs of source `name`, mapped through `axes`, counting INT1 on `interrupt` once armed, writing the health
    /// of `entry`.
    fn new(
        name: &'static str,
        axes: Axes,
        interrupt: Option<u8>,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Inertial {
        let writer = |quantity: &str| topology.vector(quantity).and_then(|parameter| parameter.writer(name));
        Inertial {
            accel: writer("accel"),
            rate: writer("rate"),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            axes,
            interrupt,
            armed: false,
            stale_run: 0,
            failing: false,
            overflowing: false,
        }
    }

    /// Records the latest read's outcome in the table entry: true when that changed it.
    fn set_healthy(&self, healthy: bool) -> bool {
        self.entry.is_some_and(|entry| entry.set_healthy(healthy))
    }

    /// One owner wake for the part named `name`: its INT1 edges, the 16-byte status burst, and what is new published
    /// through the airframe's axes, with a row (plan B bmi323.py's run loop, read on the tick).
    fn tick(&mut self, name: &str, imu: &mut Bmi323, port: &mut I2cPort, now_us: u64) {
        if let (Some(gpio), false) = (self.interrupt, self.armed)
            && crate::control::running()
        {
            self.armed = true;
            match crate::control::arm_second_interrupt(gpio) {
                Ok(()) => log::info!("{name} :: INT1 GPIO{gpio} counted (evidence; reads paced by the tick)"),
                Err(error) => log::warn!("{name} :: INT1 GPIO{gpio} not armed ({error}): no edges counted"),
            }
        }
        let runs = if self.armed {
            crate::control::take_second_edges()
        } else {
            0
        };
        BMI323_IRQ_RUNS.store(runs, Ordering::Relaxed);
        if self.armed && imu.wake(runs) {
            log::warn!("{name} :: INT1 silent -- sampling on the tick, the line unproven");
        }
        BMI323_SILENT.store(
            match imu.interrupt_silent() {
                None => 0,
                Some(false) => 1,
                Some(true) => 2,
            },
            Ordering::Relaxed,
        );
        let (fresh, sample) = match imu.read_fresh(port) {
            Ok(read) => read,
            Err(fault) => {
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("{name} :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        for (half, raced, identical) in [
            (0, fresh.accel_raced, fresh.accel_identical),
            (1, fresh.rate_raced, fresh.rate_identical),
        ] {
            if raced {
                BMI323_RACED[half].fetch_add(1, Ordering::Relaxed);
            }
            if identical {
                BMI323_IDENTICAL[half].fetch_add(1, Ordering::Relaxed);
            }
        }
        if !fresh.accel && !fresh.rate {
            self.stale_run = self.stale_run.saturating_add(1);
            if self.stale_run >= drivers::bmi323::STALE_LIMIT && self.set_healthy(false) {
                log::warn!("{name} :: no new conversion -- stale");
            }
            return;
        }
        self.stale_run = 0;
        self.set_healthy(true);
        if fresh.accel {
            let now_ms = (now_us / 1000) as u32;
            if BMI323_CONVERSIONS.fetch_add(1, Ordering::Relaxed) == 0 {
                BMI323_FIRST_MS.store(now_ms, Ordering::Relaxed);
            }
            BMI323_LAST_MS.store(now_ms, Ordering::Relaxed);
        }
        let accel = self.axes.apply(sample.accel_g());
        if let (true, Some(writer)) = (fresh.accel, &mut self.accel) {
            writer.push(accel, now_us);
        }
        if let (true, Some(writer)) = (fresh.rate, &mut self.rate) {
            writer.push(self.axes.apply(sample.rate_degrees_per_second()), now_us);
        }
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        let [gx, gy, gz] = self.axes.apply(sample.rate_centidegrees_per_second());
        let row = [
            Cell::F32(accel[0]),
            Cell::F32(accel[1]),
            Cell::F32(accel[2]),
            Cell::I32(gx),
            Cell::I32(gy),
            Cell::I32(gz),
            Cell::I32(runs as i32),
        ];
        match telemetry.push(now_us, &row) {
            Ok(()) => self.overflowing = false,
            Err(overflow) => {
                if !self.overflowing {
                    log::warn!("{name} :: telemetry {overflow}");
                    self.overflowing = true;
                }
            }
        }
    }
}

/// The magnetometer's run name, once its run is built: whose calibration [`magnetometer_instruction`] words.
static MAGNETOMETER: std::sync::OnceLock<&'static str> = std::sync::OnceLock::new();
/// Whether the magnetometer's hard/soft-iron calibration is in force.
static MAGNETOMETER_CALIBRATED: AtomicBool = AtomicBool::new(false);
/// How many of the eight octants its turn has covered.
static MAGNETOMETER_COVERED: AtomicU32 = AtomicU32::new(0);
/// How many samples it has seen since boot.
static MAGNETOMETER_SEEN: AtomicU32 = AtomicU32::new(0);
/// The extrema its evidence holds (x min, x max, y min, y max), once it has seen a sample.
static MAGNETOMETER_SPAN: [AtomicI32; 4] = [const { AtomicI32::new(0) }; 4];

/// The magnetometer's pending calibration, in plan B's words (bmm350.calibration()), from the run's atomics: the
/// executor words it, so the owner never takes the calibrations' mutex on its tick (D9 of the M9 plan). None when no
/// magnetometer runs or it is calibrated.
pub fn magnetometer_instruction() -> Option<(&'static str, String)> {
    let name = *MAGNETOMETER.get()?;
    let text = drivers::bmm350::format_instruction(
        MAGNETOMETER_CALIBRATED.load(Ordering::Relaxed),
        MAGNETOMETER_COVERED.load(Ordering::Relaxed),
        MAGNETOMETER_SEEN.load(Ordering::Relaxed),
    );
    (!text.is_empty()).then_some((name, text))
}

/// What `inspect` shows of the magnetometer beyond its channel (plan B bmm350.py:382-387): whether it is calibrated,
/// the extrema its evidence holds (x min, x max, y min, y max; None before a sample), the octants its turn covered,
/// the samples it has seen.
pub fn magnetometer_inspection() -> (bool, Option<[i32; 4]>, u32, u32) {
    let seen = MAGNETOMETER_SEEN.load(Ordering::Relaxed);
    (
        MAGNETOMETER_CALIBRATED.load(Ordering::Relaxed),
        (seen > 0).then(|| {
            MAGNETOMETER_SPAN
                .each_ref()
                .map(|extreme| extreme.load(Ordering::Relaxed))
        }),
        MAGNETOMETER_COVERED.load(Ordering::Relaxed),
        seen,
    )
}

/// Publishes the magnetometer's calibration state (its run's, and the executor's apply).
fn note_magnetometer(magnetometer: &Bmm350) {
    if let Some(span) = magnetometer.span() {
        for (extreme, value) in
            MAGNETOMETER_SPAN
                .iter()
                .zip([span.x_minimum, span.x_maximum, span.y_minimum, span.y_maximum])
        {
            extreme.store(value, Ordering::Relaxed);
        }
    }
    MAGNETOMETER_CALIBRATED.store(magnetometer.calibrated(), Ordering::Relaxed);
    MAGNETOMETER_COVERED.store(magnetometer.covered(), Ordering::Relaxed);
    MAGNETOMETER_SEEN.store(magnetometer.seen(), Ordering::Relaxed);
}

/// The BMM350's outputs (M9): its `mag` channel and stream, read every [`drivers::bmm350::POLL_PERIOD_MS`] (twice a
/// conversion at 12.5 Hz), each fresh conversion observed once for the calibration's evidence (plan B bmm350.run).
pub struct Magnetometer {
    /// The `mag` writer (counts: raw, or centred and normalised once calibrated).
    mag: Option<Writer<'static, [f32; 3]>>,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When the next read is due, microseconds.
    next_due_us: u64,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// The part read stale and nothing fresh since: one log line.
    stale: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Magnetometer {
    /// The outputs of source `name`, writing the health of `entry`.
    fn new(
        name: &'static str,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Magnetometer {
        let _ = MAGNETOMETER.set(name);
        Magnetometer {
            mag: topology.vector("mag").and_then(|parameter| parameter.writer(name)),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            next_due_us: 0,
            failing: false,
            stale: false,
            overflowing: false,
        }
    }

    /// Records the latest read's outcome in the table entry.
    fn set_healthy(&self, healthy: bool) {
        if let Some(entry) = self.entry {
            entry.set_healthy(healthy);
        }
    }

    /// One owner wake for the part named `name`: when due, the read; a fresh conversion observed, pushed and rowed.
    fn tick(&mut self, name: &str, magnetometer: &mut Bmm350, port: &mut I2cPort, now_us: u64) {
        if now_us < self.next_due_us {
            return;
        }
        let period_us = u64::from(drivers::bmm350::POLL_PERIOD_MS) * 1000;
        self.next_due_us = (self.next_due_us + period_us).max(now_us + period_us / 2);
        let field = match magnetometer.read_fresh(port, now_us) {
            Ok((freshness::Change::Fresh, field)) => field,
            Ok((freshness::Change::Unchanged, _)) => return,
            Ok((freshness::Change::Stale, _)) => {
                self.set_healthy(false);
                if !self.stale {
                    log::warn!("{name} :: no new conversion -- stale");
                    self.stale = true;
                }
                return;
            }
            Err(fault) => {
                self.set_healthy(false);
                if !self.failing {
                    log::warn!("bmm350 :: read {fault}");
                    self.failing = true;
                }
                return;
            }
        };
        self.failing = false;
        self.stale = false;
        self.set_healthy(true);
        let [x, y, z] = magnetometer.observe(&field);
        note_magnetometer(magnetometer);
        if let Some(writer) = &mut self.mag {
            writer.push([x as f32, y as f32, z as f32], now_us);
        }
        let Some(telemetry) = &mut self.telemetry else {
            return;
        };
        match telemetry.push(now_us, &[Cell::I32(x), Cell::I32(y), Cell::I32(z)]) {
            Ok(()) => self.overflowing = false,
            Err(overflow) => {
                if !self.overflowing {
                    log::warn!("{name} :: telemetry {overflow}");
                    self.overflowing = true;
                }
            }
        }
    }
}

/// When a polled run is due: every period, on its own grid, a stalled run starting a new grid (a burst never
/// follows a gap).
#[derive(Clone, Copy)]
struct Due {
    /// The period, microseconds.
    period_us: u64,
    /// When the next read is due, microseconds.
    next_us: u64,
}

impl Due {
    /// Every `period_ms`, the first read at once.
    fn every(period_ms: u32) -> Due {
        Due {
            period_us: u64::from(period_ms.max(1)) * 1000,
            next_us: 0,
        }
    }

    /// Whether a read is due at `now_us`; when it is, the next one is set a period on.
    fn take(&mut self, now_us: u64) -> bool {
        if now_us < self.next_us {
            return false;
        }
        self.next_us = (self.next_us + self.period_us).max(now_us + self.period_us / 2);
        true
    }
}

/// A row pushed to a run's stream, the refusal logged once per run of refusals.
pub(crate) fn record(
    name: &str,
    telemetry: &mut Option<Telemetry<'static>>,
    overflowing: &mut bool,
    now_us: u64,
    row: &[Cell],
) {
    let Some(telemetry) = telemetry else {
        return;
    };
    match telemetry.push(now_us, row) {
        Ok(()) => *overflowing = false,
        Err(overflow) => {
            if !*overflowing {
                log::warn!("{name} :: telemetry {overflow}");
                *overflowing = true;
            }
        }
    }
}

/// The pitot's pending calibration in plan B's words (sdp810.calibration, its live reading left out: the text is a
/// constant on the owner).
const PITOT_INSTRUCTION: &str = "keep the pitot in STILL AIR -- do NOT blow into it, this captures the zero tare";

/// The pitot's state for `inspect` (plan B sdp810.py inspect), published by its run: the temperature (C x 100), the
/// scale, the zero offset (Pa x 100), whether a tare is in force, the air density (kg/m^3 x 10000).
static PITOT_STATE: [AtomicI32; 5] = [const { AtomicI32::new(0) }; 5];
/// Whether the pitot's state was published at least once (its run exists).
static PITOT_KNOWN: AtomicBool = AtomicBool::new(false);
/// A tare the owner captured that the executor has not persisted yet (Pa x 100), and whether one waits: the NVS write
/// is the executor's ([`persist_pitot_tare`]), never the flight core's.
static PITOT_TARE: AtomicI32 = AtomicI32::new(0);
/// Whether [`PITOT_TARE`] waits to be persisted.
static PITOT_TARE_PENDING: AtomicBool = AtomicBool::new(false);

/// Publishes the pitot's state for `inspect`.
fn note_pitot(pitot: &Sdp810) {
    let state = [
        (pitot.temperature() * 100.0).round() as i32,
        i32::from(pitot.scale()),
        pitot.zero_offset(),
        i32::from(pitot.tared()),
        (pitot.air_density() * 10_000.0).round() as i32,
    ];
    for (published, value) in PITOT_STATE.iter().zip(state) {
        published.store(value, Ordering::Relaxed);
    }
    PITOT_KNOWN.store(true, Ordering::Relaxed);
}

/// A tare the owner captured, for the executor to persist.
fn owe_pitot_tare(tare: i32) {
    PITOT_TARE.store(tare, Ordering::Relaxed);
    PITOT_TARE_PENDING.store(true, Ordering::Release);
}

/// Persists the pitot tare the owner captured, if one waits (plan B sdp810._persist_zero: NVS `sdp810_zero`, the
/// Pa x 100 i32 shared with plan B), on the executor after the owner answered a `calibrate` or an `update`. None when
/// nothing waited or it persisted, else why it did not (the tare stays in force either way).
pub fn persist_pitot_tare(partition: &esp_idf_svc::nvs::EspDefaultNvsPartition) -> Option<String> {
    use warmstart::Nvs;
    if !PITOT_TARE_PENDING.swap(false, Ordering::Acquire) {
        return None;
    }
    let tare = PITOT_TARE.load(Ordering::Relaxed);
    let written = crate::warm::ColudoNvs::open(partition)
        .map_err(|error| error.to_string())
        .and_then(|mut nvs| {
            nvs.set_i32(drivers::sdp810::NVS_ZERO_KEY, tare)
                .and_then(|()| nvs.commit())
                .map_err(|error| format!("NVS error {:#x}", error.0))
        });
    match written {
        Ok(()) => {
            log::info!("sdp810 :: tare {:.2} Pa persisted", tare as f32 / 100.0);
            None
        }
        Err(why) => Some(format!("tared but not persisted: {why}")),
    }
}

/// What `inspect` shows of the pitot beyond its channels: (temperature C, scale, zero offset Pa, tared, air density);
/// None before its run published.
pub fn pitot_inspection() -> Option<(f32, i32, f32, bool, f32)> {
    PITOT_KNOWN.load(Ordering::Relaxed).then(|| {
        let [temperature, scale, zero, tared, density] =
            PITOT_STATE.each_ref().map(|value| value.load(Ordering::Relaxed));
        (
            temperature as f32 / 100.0,
            scale,
            zero as f32 / 100.0,
            tared != 0,
            density as f32 / 10_000.0,
        )
    })
}

/// The SDP810's outputs (M10, plan B sdp810.run): `dynamic_pressure` (Pa) and `airspeed` (m/s), read every
/// `period_ms` (20: 50 Hz) from the part's continuous mode, a row of plan B's integers per read; a bus failure counts
/// toward the driver's restart ladder, a CRC reject is only noted.
pub struct Pitot {
    /// The `dynamic_pressure` writer.
    pressure: Option<Writer<'static, f32>>,
    /// The `airspeed` writer.
    airspeed: Option<Writer<'static, f32>>,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a read is due.
    due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Pitot {
    /// The outputs of source `name`, read every `period_ms`, writing the health of `entry`.
    fn new(
        name: &'static str,
        period_ms: u32,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Pitot {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Pitot {
            pressure: writer("dynamic_pressure"),
            airspeed: writer("airspeed"),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            due: Due::every(period_ms),
            failing: false,
            overflowing: false,
        }
    }

    /// One owner wake: when due, a frame read and published, or the failure noted (and the restart's outcome).
    fn tick(&mut self, name: &str, pitot: &mut Sdp810, port: &mut I2cPort, delay: &mut Delay, now_us: u64) {
        if !self.due.take(now_us) {
            return;
        }
        match pitot.read(port, delay) {
            Ok(sample) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                for (writer, value) in [
                    (&mut self.pressure, sample.dynamic_pressure),
                    (&mut self.airspeed, sample.airspeed),
                ] {
                    if let Some(writer) = writer {
                        writer.push(value, now_us);
                    }
                }
                note_pitot(pitot);
                record(
                    name,
                    &mut self.telemetry,
                    &mut self.overflowing,
                    now_us,
                    &sample.row.map(Cell::I32),
                );
            }
            Err(failure) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("sdp810 :: read {}", failure.fault);
                    self.failing = true;
                }
                match failure.restart {
                    Some(Ok(())) => log::warn!(
                        "sdp810 :: restarted continuous mode after {} failures",
                        drivers::sdp810::RESTART_AFTER
                    ),
                    Some(Err(fault)) => log::warn!("sdp810 :: restart failed: {fault}"),
                    None => {}
                }
            }
        }
    }
}

/// The INA226's latest reading for `inspect` (mV, mA, mW) and whether it has one.
static POWER_STATE: [AtomicI32; 3] = [const { AtomicI32::new(0) }; 3];
/// Whether [`POWER_STATE`] holds a reading.
static POWER_KNOWN: AtomicBool = AtomicBool::new(false);

/// What `inspect` shows of the power monitor (plan B ina226.py inspect): mV, mA, mW and the ALERT trips (0: the
/// over-current ALERT is counted with the servos, M13); None before a reading.
pub fn power_inspection() -> Option<([i32; 3], u32)> {
    POWER_KNOWN
        .load(Ordering::Relaxed)
        .then(|| (POWER_STATE.each_ref().map(|value| value.load(Ordering::Relaxed)), 0))
}

/// The INA226's outputs (M10, plan B ina226.run): `voltage` (mV), `current` (mA) and `power` (mW), read every
/// `period_ms` (100), a row of plan B's integers per read.
pub struct Power {
    /// The `voltage`, `current` and `power` writers.
    writers: [Option<Writer<'static, f32>>; 3],
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a read is due.
    due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Power {
    /// The outputs of source `name`, read every `period_ms`, writing the health of `entry`.
    fn new(
        name: &'static str,
        period_ms: u32,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Power {
        let writer = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(name));
        Power {
            writers: [writer("voltage"), writer("current"), writer("power")],
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            due: Due::every(period_ms),
            failing: false,
            overflowing: false,
        }
    }

    /// One owner wake: when due, a reading published, or the failure noted.
    fn tick(&mut self, name: &str, monitor: &Ina226, port: &mut I2cPort, now_us: u64) {
        if !self.due.take(now_us) {
            return;
        }
        match monitor.read(port) {
            Ok(reading) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                let values = [reading.voltage_mv, reading.current_ma, reading.power_mw];
                for (writer, value) in self.writers.iter_mut().zip(values) {
                    if let Some(writer) = writer {
                        writer.push(value as f32, now_us);
                    }
                }
                for (published, value) in POWER_STATE.iter().zip(values) {
                    published.store(value, Ordering::Relaxed);
                }
                POWER_KNOWN.store(true, Ordering::Relaxed);
                let row = [
                    Cell::I32(values[0]),
                    Cell::I32(values[1]),
                    Cell::I32(values[2]),
                    Cell::I32(0),
                ];
                record(name, &mut self.telemetry, &mut self.overflowing, now_us, &row);
            }
            Err(fault) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("ina226 :: read {fault}");
                    self.failing = true;
                }
            }
        }
    }
}

/// A laser ranger's outputs (M10, plan B vl53l1x.run): `agl` (m), polled every `period_ms` (50), published only for a
/// waiting, valid range (the data-ready check plan B lacked; an invalid status is read, cleared, never published).
pub struct Ranger {
    /// The `agl` writer.
    agl: Option<Writer<'static, f32>>,
    /// Its stream, with a recorder.
    telemetry: Option<Telemetry<'static>>,
    /// Its table entry, whose `healthy` the run writes.
    entry: Option<&'static DeviceEntry<'static>>,
    /// When a poll is due.
    due: Due,
    /// A read failed and none succeeded since: one log line per run.
    failing: bool,
    /// A row was refused and none written since: one log line per run.
    overflowing: bool,
}

impl Ranger {
    /// The outputs of source `name`, polled every `period_ms`, writing the health of `entry`.
    fn new(
        name: &'static str,
        period_ms: u32,
        topology: &'static Topology,
        entry: Option<&'static DeviceEntry<'static>>,
    ) -> Ranger {
        Ranger {
            agl: topology.scalar("agl").and_then(|parameter| parameter.writer(name)),
            telemetry: crate::recorder::get()
                .zip(crate::recorder::device_stream(name))
                .and_then(|(recorder, stream)| Telemetry::new(recorder, stream)),
            entry,
            due: Due::every(period_ms),
            failing: false,
            overflowing: false,
        }
    }

    /// One owner wake: when due, a range read and, when valid, published.
    fn tick(&mut self, name: &str, ranger: &Vl53<Xshut>, port: &mut I2cPort, now_us: u64) {
        if !self.due.take(now_us) {
            return;
        }
        match ranger.range(port) {
            Ok(ranging) => {
                self.failing = false;
                if let Some(entry) = self.entry {
                    entry.set_healthy(true);
                }
                let Some(agl) = ranging.agl() else {
                    return;
                };
                if let Some(writer) = &mut self.agl {
                    writer.push(agl, now_us);
                }
                // irq_runs: the lasers are polled (no data-ready wire on v1.x), plan B's 0
                record(
                    name,
                    &mut self.telemetry,
                    &mut self.overflowing,
                    now_us,
                    &[Cell::F32(agl), Cell::I32(0)],
                );
            }
            Err(fault) => {
                if let Some(entry) = self.entry {
                    entry.set_healthy(false);
                }
                if !self.failing {
                    log::warn!("{name} :: read {fault}");
                    self.failing = true;
                }
            }
        }
    }
}

/// What a part's `calibrate` answers on its owner.
pub enum Calibrated {
    /// Plan B's answer: None success, else why.
    Done(Option<String>),
    /// A calibration computed but not in force: the executor persists `blob` under NVS `key`, then has the owner apply
    /// it (D9 of the M9 plan: persist, then apply, as plan B; no flash work on the flight core).
    Persist {
        key: &'static str,
        blob: [u8; drivers::bmm350::CALIBRATION_BYTES],
    },
}

/// Restores each magnetometer's hard/soft-iron calibration from NVS after its setup (plan B bmm350._restore), so a
/// power cycle does not cost the operator a turn: once at bring-up, on main, before the owners run.
pub fn restore_calibrations(
    partition: &esp_idf_svc::nvs::EspDefaultNvsPartition,
    members: &mut [Vec<crate::devices::Member>; 2],
) {
    use warmstart::Nvs;
    for member in members.iter_mut().flatten() {
        let Part::Sdp810(pitot) = &mut member.part else {
            continue;
        };
        // the tare in NVS wins over the config's zero_offset_pa (plan B sdp810.setup), so a reboot mid-glide keeps it
        let stored = crate::warm::ColudoNvs::open(partition)
            .map_err(|error| error.to_string())
            .and_then(|mut nvs| {
                nvs.get_i32(drivers::sdp810::NVS_ZERO_KEY)
                    .map_err(|error| format!("NVS error {:#x}", error.0))
            });
        match stored {
            Ok(zero) => {
                pitot.restore_zero(zero);
                log::info!(
                    "{} :: zero restored from NVS: {:.2} Pa",
                    member.name,
                    zero as f32 / 100.0
                );
            }
            Err(why) => log::info!(
                "{} :: no saved tare ({why}): the config's zero until a calibrate",
                member.name
            ),
        }
        note_pitot(pitot);
    }
    let magnetometers = members
        .iter_mut()
        .flatten()
        .filter_map(|member| match &mut member.part {
            Part::Bmm350(magnetometer) => Some((member.name, magnetometer)),
            _ => None,
        });
    for (name, magnetometer) in magnetometers {
        let mut blob = [0u8; drivers::bmm350::CALIBRATION_BYTES];
        let read = crate::warm::ColudoNvs::open(partition)
            .map_err(|error| error.to_string())
            .and_then(|mut nvs| {
                nvs.get_blob(drivers::bmm350::NVS_KEY, &mut blob)
                    .map_err(|error| format!("NVS error {:#x}", error.0))
            });
        match read.map(|length| magnetometer.restore(&blob[..length])) {
            Ok(Some(calibration)) => log::info!(
                "{name} :: calibration restored: centre ({}, {}) radii ({}, {})",
                calibration.centre_x,
                calibration.centre_y,
                calibration.radius_x,
                calibration.radius_y
            ),
            Ok(None) => log::warn!("{name} :: the saved calibration is no calibration (wrong length): axes raw"),
            Err(why) => log::info!("{name} :: no saved calibration ({why}): axes raw until the operator's turn"),
        }
        note_magnetometer(magnetometer);
    }
}

/// A VL53 ranger's XSHUT line, when the config wires one (v0.1; layout nulls it on v1.x), driven for the rest of the
/// run once a part first takes it. Dropping an esp-idf-hal PinDriver disables the pad, so the line would float when a
/// failed attempt's part is dropped: on a pin two laser entries share (the default config's GPIO5), the fitted
/// laser's XSHUT after the other entry's attempts. Plan B's `Pin` was never released either.
pub struct Xshut(ManuallyDrop<PinDriver<'static, Output>>);

impl Xshut {
    /// GPIO `gpio` as an output for a ranger's XSHUT.
    ///
    /// Errors: the pin would not configure.
    fn output(gpio: u8) -> Result<Xshut, EspError> {
        // SAFETY: the board config gives this GPIO to the rangers' XSHUT alone; the driver is never released.
        let driver = PinDriver::output(unsafe { AnyOutputPin::steal(gpio) })?;
        Ok(Xshut(ManuallyDrop::new(driver)))
    }
}

impl embedded_hal::digital::ErrorType for Xshut {
    type Error = GpioError;
}

impl OutputPin for Xshut {
    fn set_low(&mut self) -> Result<(), GpioError> {
        OutputPin::set_low(&mut *self.0)
    }

    fn set_high(&mut self) -> Result<(), GpioError> {
        OutputPin::set_high(&mut *self.0)
    }
}

/// One I2C part, as its bus owner drives it.
pub enum Part {
    /// The SEN0697's IMU.
    Bmi323(Bmi323),
    /// The SEN0697's magnetometer.
    Bmm350(Bmm350),
    /// The SEN0697's barometer.
    Bmp581(Bmp581),
    /// The primary barometer.
    Icp10111(Icp10111),
    /// The power monitor.
    Ina226(Ina226),
    /// The IMU when its entry binds it to I2C.
    Lsm6dso32(Lsm6dso32<OverI2c>),
    /// The pitot's differential pressure sensor.
    Sdp810(Sdp810),
    /// A laser ranger: the VL53L1X or the VL53L4CX silicon.
    Vl53(Vl53<Xshut>),
}

/// Runs `$body` with `$driver` bound to whichever driver `$part` holds.
macro_rules! each {
    ($part:expr, $driver:ident => $body:expr) => {
        match $part {
            Part::Bmi323($driver) => $body,
            Part::Bmm350($driver) => $body,
            Part::Bmp581($driver) => $body,
            Part::Icp10111($driver) => $body,
            Part::Ina226($driver) => $body,
            Part::Lsm6dso32($driver) => $body,
            Part::Sdp810($driver) => $body,
            Part::Vl53($driver) => $body,
        }
    };
}

impl Part {
    /// A fresh driver for `entry`, an I2C part ([`Kind::I2c`]) at `address`, its binding's (plan B's task created
    /// for one attempt). Its INT, ALERT and XSHUT wires resolve through the board's pins (plan B `_pin_gpio`).
    ///
    /// Errors: the config refuses the part: a magnetometer `axes` that is no level rotation, an XSHUT pin that would
    /// not drive, an implementation that is not an I2C part.
    pub fn build(document: &Value, entry: &Entry, address: u8) -> Result<Part, String> {
        let wired = |key: &str| board_config::pin(document, entry, key).is_some();
        let fields = Value::Object(entry.fields.clone());
        Ok(match entry.implementation {
            "bmi323" => {
                imu_axes(entry)?;
                Part::Bmi323(Bmi323::new(address, wired("int_pin")))
            }
            "bmm350" => {
                let axes = Axes::from_config(&fields).map_err(|error| error.to_string())?;
                Part::Bmm350(Bmm350::new(address).mounted(axes).map_err(|error| error.to_string())?)
            }
            "bmp581" => Part::Bmp581(Bmp581::new(address)),
            "icp10111" => {
                let period_ms = fields
                    .get("period_ms")
                    .and_then(Value::as_u64)
                    .and_then(|period| u32::try_from(period).ok())
                    .unwrap_or(drivers::icp10111::DEFAULT_PERIOD_MS);
                Part::Icp10111(Icp10111::new(address, period_ms))
            }
            "ina226" => Part::Ina226(Ina226::new(&drivers::ina226::Settings::from_config(
                &fields,
                wired("alert_pin"),
            ))),
            "lsm6dso32" => Part::Lsm6dso32(Lsm6dso32::new(OverI2c { address }, wired("int_pin"))),
            "sdp810" => Part::Sdp810(Sdp810::new(&drivers::sdp810::Settings::from_config(&fields))),
            "vl53l1x" | "vl53l4cx" => {
                let silicon = match entry.implementation {
                    "vl53l1x" => &drivers::vl53l1x::VL53L1X,
                    _ => &drivers::vl53l4cx::VL53L4CX,
                };
                let xshut = match board_config::pin(document, entry, "xshut_pin") {
                    Some(gpio) => Some(Xshut::output(gpio).map_err(|error| format!("xshut GPIO{gpio}: {error}"))?),
                    None => None,
                };
                Part::Vl53(Vl53::new(
                    silicon,
                    &drivers::vl53l1x::Settings::from_config(&fields),
                    xshut,
                ))
            }
            other => return Err(format!("'{other}' is not an I2C part")),
        })
    }

    /// Plan B's setup (its bus half): the part from its power-on state to producing data.
    ///
    /// Errors: the fault the setup ended in, as one line.
    pub fn setup(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), String> {
        each!(self, driver => Device::<I2cPort>::setup(driver, port, delay).map_err(|fault| fault.to_string()))
    }

    /// What a general call took from the part, restored (never its setup: that would re-zero a barometer's ground
    /// and reload the pitot's tare).
    ///
    /// Errors: the fault the re-arm ended in, as one line.
    pub fn rearm(&mut self, port: &mut I2cPort, delay: &mut Delay) -> Result<(), String> {
        each!(self, driver => Device::<I2cPort>::rearm(driver, port, delay).map_err(|fault| fault.to_string()))
    }

    /// Plan B's diagnose after the last failed attempt: what the part answers at the wire level.
    pub fn diagnose(&self, port: &mut I2cPort) -> String {
        let id = port.id();
        match self {
            Part::Bmi323(driver) => driver.diagnose(port, id),
            Part::Bmm350(driver) => driver.diagnose(port, id),
            Part::Bmp581(driver) => driver.diagnose(port),
            Part::Icp10111(driver) => driver.diagnose(port),
            Part::Ina226(driver) => driver.diagnose(port),
            Part::Lsm6dso32(driver) => driver.diagnose(port),
            Part::Sdp810(driver) => driver.diagnose(port),
            Part::Vl53(driver) => driver.diagnose(port),
        }
    }

    /// Plan B's on-demand self-test (`probe`): the part identifies itself and one reading comes back, logged in plan
    /// B's shape (`<name> :: probe: <step> ...`, then `probe: <step> ok <value>` or `probe FAILED: <step>: <why>`).
    /// None when healthy, else the failure, as plan B answers. The ICP-10111 and the SDP810 take one reading of their
    /// own, where plan B waited for its run loop's (D16 of the M5 plan: the owner cannot race itself). A driver
    /// identifies and reads in one call, so a two-step probe logs its steps as one.
    pub fn probe(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay) -> Option<String> {
        let step = match self {
            Part::Bmi323(_) | Part::Bmm350(_) | Part::Bmp581(_) => "chip id + read",
            Part::Icp10111(_) | Part::Sdp810(_) => "data",
            Part::Ina226(_) => "die id + read",
            Part::Lsm6dso32(_) => "who_am_i + sample",
            Part::Vl53(_) => "model id",
        };
        log::info!("{name} :: probe: {step} ...");
        let outcome = match self {
            Part::Bmi323(driver) => driver
                .probe(port)
                .map(|sample| format!("|a| {:.3} g", magnitude(sample.accel_g())))
                .map_err(|fault| fault.to_string()),
            Part::Bmm350(driver) => match driver.probe(port) {
                Ok([x, y, z]) => Ok(format!("({x}, {y}, {z})")),
                Err(drivers::bmm350::ProbeFailure::Fault(fault)) => Err(fault.to_string()),
                Err(drivers::bmm350::ProbeFailure::NotMeasuring(why)) => Err(why.to_string()),
            },
            Part::Bmp581(driver) => driver
                .probe(port)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Icp10111(driver) => driver
                .probe(port, delay)
                .map(|reading| format!("{:.0} Pa", reading.pressure))
                .map_err(|fault| fault.to_string()),
            Part::Ina226(driver) => driver
                .probe(port)
                .map(|reading| {
                    format!(
                        "{} mV {} mA {} mW",
                        reading.voltage_mv, reading.current_ma, reading.power_mw
                    )
                })
                .map_err(|fault| fault.to_string()),
            Part::Lsm6dso32(driver) => driver
                .probe(port)
                .map(|sample| {
                    let [x, y, z] = sample.rate_degrees_per_second();
                    format!("{:.2}g ({x:.1},{y:.1},{z:.1}) dps", magnitude(sample.accel_g()))
                })
                .map_err(|fault| fault.to_string()),
            Part::Sdp810(driver) => driver
                .read(port, delay)
                .map(|sample| format!("{:.2} Pa ({:.1} m/s)", sample.dynamic_pressure, sample.airspeed))
                .map_err(|failure| failure.fault.to_string()),
            Part::Vl53(driver) => driver
                .probe(port)
                .map(|model| format!("0x{model:04x}"))
                .map_err(|fault| fault.to_string()),
        };
        match outcome {
            Ok(value) => {
                log::info!("{name} :: probe: {step} ok {value}");
                None
            }
            Err(why) => {
                let message = format!("{step}: {why}");
                log::warn!("{name} :: probe FAILED: {message}");
                Some(message)
            }
        }
    }

    /// One owner tick for the part named `name` (M6: the ICP-10111's two-phase measurement, plan B's run loop),
    /// publishing through `run`. Returns [`Event::Rearm`] when the part's recovery asks the owner to re-arm the port's
    /// other parts.
    pub fn tick(
        &mut self,
        name: &str,
        run: &mut Run,
        port: &mut I2cPort,
        delay: &mut Delay,
        now_us: u64,
    ) -> Option<Event> {
        match (self, run) {
            (Part::Icp10111(barometer), Run::Barometer(outputs)) => match barometer.poll(port, now_us) {
                Poll::Waiting | Poll::Issued | Poll::GeneralCall => None,
                Poll::Reading(reading) => {
                    outputs.publish(name, barometer.elevation(&reading), Conversion::from(&reading), now_us);
                    None
                }
                Poll::Failed { fault, .. } => {
                    outputs.set_healthy(false);
                    if !outputs.failing {
                        log::warn!("icp10111 :: read {fault}");
                        outputs.failing = true;
                    }
                    None
                }
                Poll::Rearm => Some(Event::Rearm),
            },
            (Part::Bmi323(imu), Run::Imu(outputs)) => {
                outputs.tick(name, imu, port, now_us);
                None
            }
            (Part::Bmp581(barometer), Run::Barometer(outputs)) => {
                let unjudged = match barometer.read_fresh(port, now_us) {
                    Ok(bmp581::Outcome::Fresh(reading)) => {
                        let elevation = barometer.elevation(&reading);
                        outputs.publish(name, Some(elevation), Conversion::from(&reading), now_us);
                        false
                    }
                    Ok(bmp581::Outcome::Unchanged) => false,
                    Ok(bmp581::Outcome::Stale) => {
                        outputs.freeze(name);
                        false
                    }
                    Ok(bmp581::Outcome::Torn) => {
                        BMP581_TORN.fetch_add(1, Ordering::Relaxed);
                        true
                    }
                    Ok(bmp581::Outcome::Implausible(_)) => {
                        BMP581_IMPLAUSIBLE.fetch_add(1, Ordering::Relaxed);
                        true
                    }
                    Err(fault) => {
                        outputs.fail(name, &fault);
                        false
                    }
                };
                // a part that answers only garbage is never judged stale: unhealthy after a run of it all the same
                outputs.unjudged_run = if unjudged { outputs.unjudged_run + 1 } else { 0 };
                if outputs.unjudged_run >= UNJUDGED_LIMIT {
                    outputs.freeze(name);
                }
                None
            }
            (Part::Bmm350(magnetometer), Run::Magnetometer(outputs)) => {
                outputs.tick(name, magnetometer, port, now_us);
                None
            }
            (Part::Sdp810(pitot), Run::Pitot(outputs)) => {
                outputs.tick(name, pitot, port, delay, now_us);
                None
            }
            (Part::Ina226(monitor), Run::Power(outputs)) => {
                outputs.tick(name, monitor, port, now_us);
                None
            }
            (Part::Vl53(ranger), Run::Ranger(outputs)) => {
                outputs.tick(name, ranger, port, now_us);
                None
            }
            _ => None,
        }
    }

    /// Whether the part is an IMU, whose run goes first in each owner wake (D13 of the M9 plan): the lowest latency
    /// for the backup IMU.
    pub fn first(&self) -> bool {
        matches!(self, Part::Bmi323(_))
    }

    /// The calibration the part waits for, in plan B's words (`calibration()`); empty when none. The ICP-10111 asks
    /// for its ground until it holds one (setup captures it, so an up ICP never asks); the BMM350's is worded on the
    /// executor from its run's atomics ([`magnetometer_instruction`]); the others' come with their milestones (M10).
    pub fn calibration(&self) -> &'static str {
        match self {
            Part::Icp10111(barometer) => barometer.instruction(),
            Part::Sdp810(pitot) if !pitot.tared() => PITOT_INSTRUCTION,
            _ => "",
        }
    }

    /// Plan B's `calibrate()`: None on success, else why. The ICP-10111 captures its ground again from fresh
    /// conversions (plan B icp10111.py:337-344); a part with nothing to calibrate succeeds when nothing is pending
    /// (plan B's default `calibration() or None`).
    ///
    /// The BMP581 captures its ground the same way (M9). The BMM350's calibration is computed from the evidence its
    /// run gathered and handed back NOT in force ([`Calibrated::Persist`]): the executor persists it, then applies it
    /// ([`Part::apply_calibration`]); a calibration it refuses answers its instruction.
    pub fn calibrate(&mut self, name: &str, port: &mut I2cPort, delay: &mut Delay) -> Calibrated {
        Calibrated::Done(match self {
            Part::Icp10111(barometer) => match barometer.capture_ground(port, delay) {
                Ok(ground) => {
                    log::info!("{name} :: calibrated: ground {ground:.1} m AMSL");
                    None
                }
                Err(fault) => Some(format!("ground zero: {fault}")),
            },
            Part::Bmp581(barometer) => match barometer.capture_ground(port, delay) {
                Ok(ground) => {
                    log::info!("{name} :: calibrated: ground {ground:.1} m AMSL");
                    None
                }
                Err(fault) => Some(format!("ground zero: {fault}")),
            },
            Part::Sdp810(pitot) => match pitot.calibrate() {
                Ok(tare) => {
                    log::info!("{name} :: calibrated: zero_offset {:.2} Pa", tare as f32 / 100.0);
                    note_pitot(pitot);
                    owe_pitot_tare(tare);
                    None
                }
                Err(why) => Some(why.to_string()),
            },
            Part::Bmm350(magnetometer) => match magnetometer.calibrate() {
                Ok(calibration) => {
                    return Calibrated::Persist {
                        key: drivers::bmm350::NVS_KEY,
                        blob: calibration.to_bytes(),
                    };
                }
                Err(why) => Some(why),
            },
            other => Some(other.calibration().to_string()).filter(|pending| !pending.is_empty()),
        })
    }

    /// Puts the calibration `blob` in force once the executor persisted it (D9 of the M9 plan): the BMM350's, logged
    /// in plan B's words. None on success, else why.
    pub fn apply_calibration(&mut self, name: &str, blob: &[u8]) -> Option<String> {
        let Part::Bmm350(magnetometer) = self else {
            return Some(format!("{name} takes no calibration blob"));
        };
        let Some(calibration) = magnetometer.restore(blob) else {
            return Some("the calibration blob is no calibration".to_string());
        };
        note_magnetometer(magnetometer);
        log::info!(
            "{name} :: calibrated: centre ({}, {}) radii ({}, {})",
            calibration.centre_x,
            calibration.centre_y,
            calibration.radius_x,
            calibration.radius_y
        );
        None
    }

    /// Plan B's `update()`: the changed properties. The ICP-10111 takes `ground`, and `rezero` from its own altitude
    /// channel when fresher than three periods; a part with nothing writable changes nothing.
    ///
    /// Errors: the part refused the change (CC answers `err refused <text>`).
    pub fn update(&mut self, run: Option<&Run>, properties: &Value, now_us: u64) -> Result<Vec<&'static str>, String> {
        match self {
            Part::Icp10111(barometer) => {
                let window_us = barometer.period_ms().saturating_mul(3).saturating_mul(1000);
                let fresh = match run {
                    Some(Run::Barometer(outputs)) => outputs
                        .altitude
                        .as_ref()
                        .and_then(|writer| writer.recent(window_us, now_us)),
                    Some(Run::Imu(_) | Run::Magnetometer(_) | Run::Pitot(_) | Run::Power(_) | Run::Ranger(_))
                    | None => None,
                };
                barometer
                    .update(properties, fresh)
                    .map_err(|refusal| refusal.to_string())
            }
            Part::Bmp581(barometer) => {
                let fresh = match run {
                    Some(Run::Barometer(outputs)) => outputs
                        .altitude
                        .as_ref()
                        .and_then(|writer| writer.recent(drivers::bmp581::REZERO_WINDOW_US as u32, now_us)),
                    _ => None,
                };
                barometer
                    .update(properties, fresh)
                    .map_err(|refusal| refusal.to_string())
            }
            Part::Sdp810(pitot) => {
                let update = pitot.update(properties).map_err(|refusal| refusal.to_string())?;
                note_pitot(pitot);
                if let Some(tare) = update.persist {
                    owe_pitot_tare(tare);
                }
                Ok(update.changed)
            }
            _ => Ok(Vec::new()),
        }
    }

    /// The warm start's pitot tare (the crumb's, Pa): put in force without persisting (a flash write mid-air is not
    /// wanted). Whether the part is a pitot that took it.
    pub fn restore_pitot_zero(&mut self, pascal: f32) -> bool {
        let Part::Sdp810(pitot) = self else {
            return false;
        };
        pitot.set_zero_offset(pascal);
        note_pitot(pitot);
        true
    }

    /// Whether the part re-zeroes a barometer's ground, as [`rezeroes`] says of its implementation: its `update` and
    /// `calibrate` also run at DONE.
    pub fn rezeroes(&self) -> bool {
        matches!(self, Part::Icp10111(_) | Part::Bmp581(_))
    }

    /// Whether the part sent a general call since the last ask: only the ICP-10111 sends one, to free itself.
    pub fn take_general_call(&mut self) -> bool {
        match self {
            Part::Icp10111(driver) => driver.take_general_call(),
            _ => false,
        }
    }
}

/// The length of a vector of three.
fn magnitude([x, y, z]: [f32; 3]) -> f32 {
    (x * x + y * y + z * z).sqrt()
}
